#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <deque>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <filesystem>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <pcl/common/common.h>
#include <pcl/features/normal_3d.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "msg_types/msg/barrel_detect.hpp"

using namespace std::chrono_literals;

namespace
{
struct HsvRange
{
  cv::Scalar low;
  cv::Scalar high;
};

struct Candidate
{
  std::string color;
  float confidence{0.0F};
  cv::Rect bbox;
  cv::Point center_px;
  std::vector<cv::Point> contour;
  Eigen::Vector3f centroid_camera{Eigen::Vector3f::Zero()};
  Eigen::Vector3f centroid_map{Eigen::Vector3f::Zero()};
  Eigen::Vector3f axis{Eigen::Vector3f::UnitZ()};
  Eigen::Vector3f axis_map{Eigen::Vector3f::UnitZ()};
  float normal_x{0.0F};
  float normal_y{0.0F};
  float largest_extent_m{0.0F};
  float middle_extent_m{0.0F};
  float thickness_m{0.0F};
  bool horizontal{false};
  int inliers{0};
  float residual{0.0F};
  std::vector<cv::Point> ransac_inlier_pixels;
};

struct PixelCluster
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud{new pcl::PointCloud<pcl::PointXYZRGB>()};
  std::vector<cv::Point> pixels;
};

struct LeakDebugCandidate
{
  std::string color;
  cv::Rect bbox;
  std::vector<cv::Point> contour;
  int point_count{0};
  double area_px{0.0};
  double fill_ratio{0.0};
  double circularity{0.0};
  double axis_ratio{0.0};
  double source_z_inlier_ratio{0.0};
  float centroid_z{0.0F};
  float source_min_z{0.0F};
  float source_max_z{0.0F};
  float thickness_m{0.0F};
  float distance_m{0.0F};
  bool accepted{false};
  std::string reason;
};

struct DebugRegion
{
  std::string color;
  std::string reason;
  cv::Rect bbox;
  std::vector<cv::Point> contour;
  int sample_count{0};
  float metric{std::numeric_limits<float>::quiet_NaN()};
};

struct DebugAlignment
{
  std::string color;
  cv::Rect bbox;
  std::vector<cv::Point> contour;
  int best_dx{0};
  int best_dy{0};
  int best_count{0};
};

struct BarrelTrack
{
  int id{0};
  float x{0.0F};
  float y{0.0F};
  float z{0.0F};
  float normal_x{0.0F};
  float normal_y{0.0F};
  std::unordered_map<std::string, int> color_votes;
  std::deque<bool> horizontal_votes;
  int seen_count{0};
  int missed_frames{0};
  bool accepted{false};
  bool published{false};
  bool leaking{false};
  bool leak_confirmed_once{false};
  int leak_positive_count{0};
  int leak_negative_count{0};
  float last_published_x{0.0F};
  float last_published_y{0.0F};
  float last_published_normal_x{0.0F};
  float last_published_normal_y{0.0F};
  std::string last_published_color;
  bool last_published_leaking{false};
  cv::Rect last_bbox;

  std::string color() const
  {
    if (color_votes.empty()) {
      return "unknown";
    }
    return std::max_element(
      color_votes.begin(), color_votes.end(),
      [](const auto & a, const auto & b) { return a.second < b.second; })->first;
  }

  bool horizontal() const
  {
    if (horizontal_votes.empty()) {
      return false;
    }
    const int yes = std::count(horizontal_votes.begin(), horizontal_votes.end(), true);
    return yes >= static_cast<int>(std::ceil(horizontal_votes.size() / 2.0));
  }
};

int odd_kernel(int value)
{
  value = std::max(1, value);
  return value % 2 == 1 ? value : value + 1;
}

std_msgs::msg::ColorRGBA marker_color(const std::string & color, float alpha)
{
  std_msgs::msg::ColorRGBA rgba;
  rgba.a = alpha;
  if (color == "red") {
    rgba.r = 1.0F; rgba.g = 0.08F; rgba.b = 0.05F;
  } else if (color == "green") {
    rgba.r = 0.05F; rgba.g = 0.9F; rgba.b = 0.1F;
  } else if (color == "blue") {
    rgba.r = 0.05F; rgba.g = 0.25F; rgba.b = 1.0F;
  } else if (color == "yellow") {
    rgba.r = 1.0F; rgba.g = 0.85F; rgba.b = 0.05F;
  } else {
    rgba.r = 0.7F; rgba.g = 0.7F; rgba.b = 0.7F;
  }
  return rgba;
}

cv::Scalar draw_color(const std::string & color)
{
  if (color == "red") {
    return {0, 0, 255};
  }
  if (color == "green") {
    return {0, 220, 0};
  }
  if (color == "blue") {
    return {255, 80, 0};
  }
  if (color == "yellow") {
    return {0, 220, 255};
  }
  return {220, 220, 220};
}
}  // namespace

class BarrelDetectorCpp : public rclcpp::Node
{
public:
  explicit BarrelDetectorCpp(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("detect_barrel_cpp", options),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    declare_parameters();
    load_parameters();

    barrel_pub_ = create_publisher<msg_types::msg::BarrelDetect>(barrel_topic_, 10);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic_, 10);
    debug_pub_ = create_publisher<sensor_msgs::msg::Image>(debug_overlay_topic_, 10);
    debug_mask_pub_ = create_publisher<sensor_msgs::msg::Image>(debug_mask_topic_, 10);
    debug_rejection_pub_ = create_publisher<sensor_msgs::msg::Image>(debug_rejection_topic_, 10);
    debug_depth_alignment_pub_ =
      create_publisher<sensor_msgs::msg::Image>(debug_depth_alignment_topic_, 10);
    debug_depth_validity_pub_ =
      create_publisher<sensor_msgs::msg::Image>(debug_depth_validity_topic_, 10);
    debug_leak_pub_ =
      create_publisher<sensor_msgs::msg::Image>(debug_leak_overlay_topic_, 10);

    image_sub_.subscribe(this, image_topic_, rmw_qos_profile_sensor_data);
    cloud_sub_.subscribe(this, point_cloud_topic_, rmw_qos_profile_sensor_data);
    sync_ = std::make_shared<Synchronizer>(SyncPolicy(sync_queue_size_), image_sub_, cloud_sub_);
    sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(sync_slop_s_));
    sync_->registerCallback(
      std::bind(&BarrelDetectorCpp::synced_callback, this, std::placeholders::_1, std::placeholders::_2));

    publish_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / std::max(0.1, publish_hz_)),
      std::bind(&BarrelDetectorCpp::publish_tracks, this));

    RCLCPP_INFO(
      get_logger(), "C++ barrel detector ready: image=%s point_cloud=%s",
      image_topic_.c_str(), point_cloud_topic_.c_str());
  }

private:
  using Image = sensor_msgs::msg::Image;
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<Image, PointCloud2>;
  using Synchronizer = message_filters::Synchronizer<SyncPolicy>;

  void declare_parameters()
  {
    declare_parameter("image_topic", "/oakd/rgb/preview/image_raw");
    declare_parameter("point_cloud_topic", "/oakd/rgb/preview/depth/points");
    declare_parameter("target_frame", "map");
    declare_parameter("barrel_topic", "/barrel_detect");
    declare_parameter("marker_topic", "/barrel_markers");
    declare_parameter("debug_overlay_topic", "/barrel/debug_overlay");
    declare_parameter("debug_mask_topic", "/barrel/debug_mask");
    declare_parameter("debug_rejection_topic", "/barrel/debug_rejections");
    declare_parameter("debug_depth_alignment_topic", "/barrel/debug_depth_alignment");
    declare_parameter("debug_depth_validity_topic", "/barrel/debug_depth_validity");
    declare_parameter("debug_leak_overlay_topic", "/barrel/debug_leak_overlay");
    declare_parameter("publish_hz", 2.0);
    declare_parameter("sync_queue_size", 5);
    declare_parameter("sync_slop_s", 0.08);
    declare_parameter("first_track_id", 100);
    declare_parameter("accept_threshold", 4);
    declare_parameter("dedup_distance_m", 2.0);
    declare_parameter("republish_move_threshold_m", 0.05);
    declare_parameter("republish_rotation_threshold_rad", 0.1);
    declare_parameter("track_timeout_frames", 150);
    declare_parameter("max_barrel_height_m", 0.70);
    declare_parameter("depth_min_m", 0.15);
    declare_parameter("depth_max_m", 6.0);
    declare_parameter("cluster_tolerance_m", 0.07);
    declare_parameter("cluster_min_points", 40);
    declare_parameter("cluster_max_points", 20000);
    declare_parameter("candidate_min_3d_largest_extent_m", 0.0);
    declare_parameter("candidate_max_3d_largest_extent_m", 0.0);
    declare_parameter("candidate_min_3d_middle_extent_m", 0.0);
    declare_parameter("candidate_min_3d_depth_extent_m", 0.0);
    declare_parameter("candidate_min_3d_depth_to_middle_ratio", 0.0);
    declare_parameter("candidate_min_3d_middle_largest_ratio", 0.0);
    declare_parameter("candidate_max_3d_middle_largest_ratio", 0.0);
    declare_parameter("candidate_max_distance_m", 2.0);
    declare_parameter("normal_search_radius_m", 0.05);
    declare_parameter("ransac_max_iterations", 250);
    declare_parameter("ransac_distance_threshold_m", 0.035);
    declare_parameter("ransac_normal_distance_weight", 0.1);
    declare_parameter("cylinder_radius_min_m", 0.08);
    declare_parameter("cylinder_radius_max_m", 0.45);
    declare_parameter("cylinder_inlier_min", 35);
    declare_parameter("cylinder_fit_residual_max_m", 0.05);
    declare_parameter("vertical_dot_threshold", 0.75);
    declare_parameter("horizontal_dot_threshold", 0.35);
    declare_parameter("candidate_min_area_px", 180.0);
    declare_parameter("candidate_max_area_px", 90000.0);
    declare_parameter("candidate_min_width_px", 12);
    declare_parameter("candidate_min_height_px", 12);
    declare_parameter("color_confidence_min", 0.45);
    declare_parameter("mask_morph_open_kernel", 3);
    declare_parameter("mask_morph_close_kernel", 9);
    declare_parameter("stability_window_m", 8);
    declare_parameter("marker_scale_x", 0.28);
    declare_parameter("marker_scale_y", 0.28);
    declare_parameter("marker_scale_z", 0.45);
    declare_parameter("marker_alpha", 0.9);
    declare_parameter("marker_lifetime_s", 1.5);
    declare_parameter("marker_namespace", "barrels");
    declare_parameter("enable_debug_overlay", true);
    declare_parameter("enable_debug_mask", true);
    declare_parameter("enable_debug_rejections", true);
    declare_parameter("enable_debug_rejection_log", true);
    declare_parameter("enable_debug_depth_alignment", true);
    declare_parameter("enable_debug_depth_validity", true);
    declare_parameter("enable_debug_leak_overlay", true);
    declare_parameter("leak_search_padding_px", 60);
    declare_parameter("leak_mask_morph_open_kernel", 3);
    declare_parameter("leak_mask_morph_close_kernel", 7);
    declare_parameter("leak_min_area_px", 80.0);
    declare_parameter("leak_max_area_px", 20000.0);
    declare_parameter("leak_min_fill_ratio", 0.45);
    declare_parameter("leak_min_circularity", 0.30);
    declare_parameter("leak_min_axis_ratio", 0.25);
    declare_parameter("leak_min_points", 15);
    declare_parameter("leak_min_height_m", 0.005);
    declare_parameter("leak_max_height_m", 0.08);
    declare_parameter("leak_source_z_min_m", -0.24);
    declare_parameter("leak_source_z_max_m", -0.21);
    declare_parameter("leak_source_z_inlier_ratio_min", 0.85);
    declare_parameter("leak_max_thickness_m", 0.035);
    declare_parameter("leak_max_distance_from_barrel_m", 0.85);
    declare_parameter("leak_horizontal_barrels_only", true);
    declare_parameter("leak_confirm_threshold", 3);
    declare_parameter("leak_clear_threshold", 3);
    declare_parameter("debug_depth_alignment_search_px", 50);
    declare_parameter("debug_depth_alignment_step_px", 5);
    declare_parameter("debug_window_name", "barrel");
    declare_parameter("debug_mask_window_name", "barrel_mask");
    declare_parameter("debug_rejection_window_name", "barrel_rejections");
    declare_parameter("debug_depth_alignment_window_name", "barrel_depth_alignment");
    declare_parameter("debug_depth_validity_window_name", "barrel_depth_validity");
    declare_parameter("debug_leak_window_name", "barrel_leak");
    declare_parameter("show_debug_window", false);
    declare_parameter("draw_barrel_outline", true);
    declare_parameter("draw_blob_metrics", true);
    declare_parameter("draw_track_ids", true);
    declare_parameter("draw_normal_arrow", true);
    declare_parameter("hsv_ranges.red", std::vector<int64_t>{0, 90, 40, 10, 255, 255, 160, 90, 40, 179, 255, 255});
    declare_parameter("hsv_ranges.green", std::vector<int64_t>{35, 70, 35, 90, 255, 255});
    declare_parameter("hsv_ranges.blue", std::vector<int64_t>{90, 60, 35, 140, 255, 255});
    declare_parameter("hsv_ranges.yellow", std::vector<int64_t>{16, 80, 50, 35, 255, 255});
    declare_parameter("hsv_ranges.purple", std::vector<int64_t>{130, 45, 35, 165, 255, 255});
    declare_parameter("hsv_ranges.orange", std::vector<int64_t>{5, 80, 45, 20, 255, 255});
    declare_parameter("hsv_ranges.brown", std::vector<int64_t>{5, 35, 20, 25, 190, 170});
    declare_parameter("hsv_ranges.black", std::vector<int64_t>{0, 0, 0, 179, 255, 55});
  }

  void load_parameters()
  {
    image_topic_ = get_parameter("image_topic").as_string();
    point_cloud_topic_ = get_parameter("point_cloud_topic").as_string();
    target_frame_ = get_parameter("target_frame").as_string();
    barrel_topic_ = get_parameter("barrel_topic").as_string();
    marker_topic_ = get_parameter("marker_topic").as_string();
    debug_overlay_topic_ = get_parameter("debug_overlay_topic").as_string();
    debug_mask_topic_ = get_parameter("debug_mask_topic").as_string();
    debug_rejection_topic_ = get_parameter("debug_rejection_topic").as_string();
    debug_depth_alignment_topic_ = get_parameter("debug_depth_alignment_topic").as_string();
    debug_depth_validity_topic_ = get_parameter("debug_depth_validity_topic").as_string();
    debug_leak_overlay_topic_ = get_parameter("debug_leak_overlay_topic").as_string();
    publish_hz_ = get_parameter("publish_hz").as_double();
    sync_queue_size_ = static_cast<uint32_t>(get_parameter("sync_queue_size").as_int());
    sync_slop_s_ = get_parameter("sync_slop_s").as_double();
    next_track_id_ = static_cast<int>(get_parameter("first_track_id").as_int());
    accept_threshold_ = static_cast<int>(get_parameter("accept_threshold").as_int());
    dedup_distance_m_ = get_parameter("dedup_distance_m").as_double();
    republish_move_threshold_m_ = get_parameter("republish_move_threshold_m").as_double();
    republish_rotation_threshold_rad_ = get_parameter("republish_rotation_threshold_rad").as_double();
    track_timeout_frames_ = static_cast<int>(get_parameter("track_timeout_frames").as_int());
    max_barrel_height_m_ = get_parameter("max_barrel_height_m").as_double();
    depth_min_m_ = get_parameter("depth_min_m").as_double();
    depth_max_m_ = get_parameter("depth_max_m").as_double();
    cluster_tolerance_m_ = get_parameter("cluster_tolerance_m").as_double();
    cluster_min_points_ = static_cast<int>(get_parameter("cluster_min_points").as_int());
    cluster_max_points_ = static_cast<int>(get_parameter("cluster_max_points").as_int());
    candidate_min_3d_largest_extent_m_ = get_parameter("candidate_min_3d_largest_extent_m").as_double();
    candidate_max_3d_largest_extent_m_ = get_parameter("candidate_max_3d_largest_extent_m").as_double();
    candidate_min_3d_middle_extent_m_ = get_parameter("candidate_min_3d_middle_extent_m").as_double();
    candidate_min_3d_depth_extent_m_ = get_parameter("candidate_min_3d_depth_extent_m").as_double();
    candidate_min_3d_depth_to_middle_ratio_ =
      get_parameter("candidate_min_3d_depth_to_middle_ratio").as_double();
    candidate_min_3d_middle_largest_ratio_ =
      get_parameter("candidate_min_3d_middle_largest_ratio").as_double();
    candidate_max_3d_middle_largest_ratio_ =
      get_parameter("candidate_max_3d_middle_largest_ratio").as_double();
    candidate_max_distance_m_ = get_parameter("candidate_max_distance_m").as_double();
    normal_search_radius_m_ = get_parameter("normal_search_radius_m").as_double();
    ransac_max_iterations_ = static_cast<int>(get_parameter("ransac_max_iterations").as_int());
    ransac_distance_threshold_m_ = get_parameter("ransac_distance_threshold_m").as_double();
    ransac_normal_distance_weight_ = get_parameter("ransac_normal_distance_weight").as_double();
    cylinder_radius_min_m_ = get_parameter("cylinder_radius_min_m").as_double();
    cylinder_radius_max_m_ = get_parameter("cylinder_radius_max_m").as_double();
    cylinder_inlier_min_ = static_cast<int>(get_parameter("cylinder_inlier_min").as_int());
    cylinder_fit_residual_max_m_ = get_parameter("cylinder_fit_residual_max_m").as_double();
    vertical_dot_threshold_ = get_parameter("vertical_dot_threshold").as_double();
    horizontal_dot_threshold_ = get_parameter("horizontal_dot_threshold").as_double();
    candidate_min_area_px_ = get_parameter("candidate_min_area_px").as_double();
    candidate_max_area_px_ = get_parameter("candidate_max_area_px").as_double();
    candidate_min_width_px_ = static_cast<int>(get_parameter("candidate_min_width_px").as_int());
    candidate_min_height_px_ = static_cast<int>(get_parameter("candidate_min_height_px").as_int());
    color_confidence_min_ = get_parameter("color_confidence_min").as_double();
    mask_morph_open_kernel_ = static_cast<int>(get_parameter("mask_morph_open_kernel").as_int());
    mask_morph_close_kernel_ = static_cast<int>(get_parameter("mask_morph_close_kernel").as_int());
    stability_window_m_ = static_cast<size_t>(get_parameter("stability_window_m").as_int());
    marker_scale_x_ = get_parameter("marker_scale_x").as_double();
    marker_scale_y_ = get_parameter("marker_scale_y").as_double();
    marker_scale_z_ = get_parameter("marker_scale_z").as_double();
    marker_alpha_ = get_parameter("marker_alpha").as_double();
    marker_lifetime_s_ = get_parameter("marker_lifetime_s").as_double();
    marker_namespace_ = get_parameter("marker_namespace").as_string();
    enable_debug_overlay_ = get_parameter("enable_debug_overlay").as_bool();
    enable_debug_mask_ = get_parameter("enable_debug_mask").as_bool();
    enable_debug_rejections_ = get_parameter("enable_debug_rejections").as_bool();
    enable_debug_rejection_log_ = get_parameter("enable_debug_rejection_log").as_bool();
    enable_debug_depth_alignment_ = get_parameter("enable_debug_depth_alignment").as_bool();
    enable_debug_depth_validity_ = get_parameter("enable_debug_depth_validity").as_bool();
    enable_debug_leak_overlay_ = get_parameter("enable_debug_leak_overlay").as_bool();
    leak_search_padding_px_ = static_cast<int>(get_parameter("leak_search_padding_px").as_int());
    leak_mask_morph_open_kernel_ =
      static_cast<int>(get_parameter("leak_mask_morph_open_kernel").as_int());
    leak_mask_morph_close_kernel_ =
      static_cast<int>(get_parameter("leak_mask_morph_close_kernel").as_int());
    leak_min_area_px_ = get_parameter("leak_min_area_px").as_double();
    leak_max_area_px_ = get_parameter("leak_max_area_px").as_double();
    leak_min_fill_ratio_ = get_parameter("leak_min_fill_ratio").as_double();
    leak_min_circularity_ = get_parameter("leak_min_circularity").as_double();
    leak_min_axis_ratio_ = get_parameter("leak_min_axis_ratio").as_double();
    leak_min_points_ = static_cast<int>(get_parameter("leak_min_points").as_int());
    leak_min_height_m_ = get_parameter("leak_min_height_m").as_double();
    leak_max_height_m_ = get_parameter("leak_max_height_m").as_double();
    leak_source_z_min_m_ = get_parameter("leak_source_z_min_m").as_double();
    leak_source_z_max_m_ = get_parameter("leak_source_z_max_m").as_double();
    leak_source_z_inlier_ratio_min_ =
      get_parameter("leak_source_z_inlier_ratio_min").as_double();
    leak_max_thickness_m_ = get_parameter("leak_max_thickness_m").as_double();
    leak_max_distance_from_barrel_m_ = get_parameter("leak_max_distance_from_barrel_m").as_double();
    leak_horizontal_barrels_only_ = get_parameter("leak_horizontal_barrels_only").as_bool();
    leak_confirm_threshold_ =
      std::max(1, static_cast<int>(get_parameter("leak_confirm_threshold").as_int()));
    leak_clear_threshold_ =
      std::max(1, static_cast<int>(get_parameter("leak_clear_threshold").as_int()));
    debug_depth_alignment_search_px_ =
      static_cast<int>(get_parameter("debug_depth_alignment_search_px").as_int());
    debug_depth_alignment_step_px_ =
      std::max(1, static_cast<int>(get_parameter("debug_depth_alignment_step_px").as_int()));
    debug_window_name_ = get_parameter("debug_window_name").as_string();
    debug_mask_window_name_ = get_parameter("debug_mask_window_name").as_string();
    debug_rejection_window_name_ = get_parameter("debug_rejection_window_name").as_string();
    debug_depth_alignment_window_name_ =
      get_parameter("debug_depth_alignment_window_name").as_string();
    debug_depth_validity_window_name_ =
      get_parameter("debug_depth_validity_window_name").as_string();
    debug_leak_window_name_ = get_parameter("debug_leak_window_name").as_string();
    show_debug_window_ = get_parameter("show_debug_window").as_bool();
    draw_barrel_outline_ = get_parameter("draw_barrel_outline").as_bool();
    draw_blob_metrics_ = get_parameter("draw_blob_metrics").as_bool();
    draw_track_ids_ = get_parameter("draw_track_ids").as_bool();
    draw_normal_arrow_ = get_parameter("draw_normal_arrow").as_bool();

    for (const auto & color : {"red", "green", "blue", "yellow", "purple", "orange", "brown", "black"}) {
      hsv_ranges_[color] = parse_hsv_ranges(std::string("hsv_ranges.") + color);
    }
  }

  std::vector<HsvRange> parse_hsv_ranges(const std::string & parameter_name)
  {
    const auto raw = get_parameter(parameter_name).as_integer_array();
    if (raw.size() % 6 != 0) {
      throw std::runtime_error(parameter_name + " must contain groups of 6 HSV values");
    }
    std::vector<HsvRange> ranges;
    for (size_t i = 0; i < raw.size(); i += 6) {
      ranges.push_back({
        cv::Scalar(raw[i], raw[i + 1], raw[i + 2]),
        cv::Scalar(raw[i + 3], raw[i + 4], raw[i + 5])});
    }
    return ranges;
  }

  void synced_callback(const Image::ConstSharedPtr & image_msg, const PointCloud2::ConstSharedPtr & cloud_msg)
  {
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvShare(image_msg, "bgr8");
    } catch (const cv_bridge::Exception & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "cv_bridge failed: %s", ex.what());
      return;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr organized(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::fromROSMsg(*cloud_msg, *organized);
    if (organized->width != static_cast<uint32_t>(cv_ptr->image.cols) ||
      organized->height != static_cast<uint32_t>(cv_ptr->image.rows))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Point cloud is not organized like image (%ux%u vs %dx%d); skipping frame",
        organized->width, organized->height, cv_ptr->image.cols, cv_ptr->image.rows);
      return;
    }

    cv::Mat hsv;
    cv::cvtColor(cv_ptr->image, hsv, cv::COLOR_BGR2HSV);
    cv::Mat debug_mask;
    std::vector<DebugRegion> debug_regions;
    std::vector<DebugAlignment> debug_alignments;
    auto candidates = detect_candidates(
      hsv, organized, cloud_msg->header, &debug_mask, &debug_regions, &debug_alignments);
    update_tracks(candidates);

    if (enable_debug_mask_) {
      publish_mask(debug_mask, image_msg->header);
    }
    if (enable_debug_depth_validity_) {
      publish_depth_validity(cv_ptr->image, debug_mask, organized, image_msg->header);
    }
    if (enable_debug_overlay_) {
      publish_overlay(cv_ptr->image, candidates, image_msg->header);
    }
    if (enable_debug_rejections_) {
      publish_rejections(cv_ptr->image, debug_regions, image_msg->header);
    }
    if (enable_debug_depth_alignment_) {
      publish_depth_alignment(cv_ptr->image, debug_alignments, image_msg->header);
    }
    if (enable_debug_leak_overlay_) {
      publish_leak_overlay(cv_ptr->image, hsv, organized, candidates, image_msg->header);
    }
    if (enable_debug_rejection_log_) {
      log_rejection_summary(candidates, debug_regions);
    }
  }

  std::vector<Candidate> detect_candidates(
    const cv::Mat & hsv,
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & organized,
    const std_msgs::msg::Header & header,
    cv::Mat * debug_mask,
    std::vector<DebugRegion> * debug_regions,
    std::vector<DebugAlignment> * debug_alignments)
  {
    std::vector<Candidate> candidates;
    if (debug_mask != nullptr) {
      *debug_mask = cv::Mat::zeros(hsv.size(), CV_8UC1);
    }
    for (const auto & [color, ranges] : hsv_ranges_) {
      cv::Mat mask = build_mask(hsv, ranges);
      if (debug_mask != nullptr) {
        cv::bitwise_or(*debug_mask, mask, *debug_mask);
      }
      std::vector<std::vector<cv::Point>> contours;
      cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
      for (const auto & contour : contours) {
        const double area = cv::contourArea(contour);
        const cv::Rect bbox = cv::boundingRect(contour);
        if (area < candidate_min_area_px_ || area > candidate_max_area_px_) {
          add_debug_region(color, "area", contour, bbox, debug_regions, 0, static_cast<float>(area));
          continue;
        }
        if (bbox.width < candidate_min_width_px_ || bbox.height < candidate_min_height_px_) {
          add_debug_region(color, "bbox", contour, bbox, debug_regions);
          continue;
        }
        const float confidence = static_cast<float>(area / std::max(1, bbox.area()));
        if (confidence < color_confidence_min_) {
          add_debug_region(color, "fill", contour, bbox, debug_regions, 0, confidence);
          continue;
        }

        int finite_depth_points = 0;
        auto clusters = clusters_from_contour(contour, bbox, organized, &finite_depth_points);
        if (clusters.empty()) {
          const std::string reason = finite_depth_points < cluster_min_points_ ? "depth" : "cluster";
          add_debug_region(
            color, reason, contour, bbox, debug_regions, finite_depth_points,
            static_cast<float>(finite_depth_points));
          if (finite_depth_points == 0 && debug_alignments != nullptr) {
            debug_alignments->push_back(find_best_depth_alignment(color, contour, bbox, organized));
          }
        }
        for (const auto & cluster : clusters) {
          Candidate candidate;
          candidate.color = color;
          candidate.confidence = confidence;
          candidate.bbox = bbox;
          candidate.center_px = {(bbox.x + bbox.width / 2), (bbox.y + bbox.height / 2)};
          candidate.contour = contour;
          std::string extent_reason;
          float extent_metric = std::numeric_limits<float>::quiet_NaN();
          if (!passes_candidate_extent_gate(cluster.cloud, &candidate, &extent_reason, &extent_metric)) {
            add_debug_region(color, extent_reason, contour, bbox, debug_regions, 0, extent_metric);
            continue;
          }
          std::string fit_reason;
          float fit_metric = std::numeric_limits<float>::quiet_NaN();
          if (!fit_cylinder(cluster, candidate, &fit_reason, &fit_metric)) {
            add_debug_region(color, fit_reason, contour, bbox, debug_regions, 0, fit_metric);
            continue;
          }
          const float candidate_distance = candidate.centroid_camera.norm();
          if (candidate_max_distance_m_ > 0.0 && candidate_distance > candidate_max_distance_m_) {
            add_debug_region(color, "distance", contour, bbox, debug_regions, 0, candidate_distance);
            continue;
          }
          std::string transform_reason;
          float transform_metric = std::numeric_limits<float>::quiet_NaN();
          if (!transform_candidate(header, candidate, &transform_reason, &transform_metric)) {
            add_debug_region(color, transform_reason, contour, bbox, debug_regions, 0, transform_metric);
            continue;
          }
          if (!passes_height_gate(candidate)) {
            add_debug_region(color, "height", contour, bbox, debug_regions, 0, candidate.centroid_map.z());
            continue;
          }
          candidates.push_back(candidate);
        }
      }
    }
    return candidates;
  }

  static void add_debug_region(
    const std::string & color,
    const std::string & reason,
    const std::vector<cv::Point> & contour,
    const cv::Rect & bbox,
    std::vector<DebugRegion> * debug_regions,
    int sample_count = 0,
    float metric = std::numeric_limits<float>::quiet_NaN())
  {
    if (debug_regions == nullptr) {
      return;
    }
    DebugRegion region;
    region.color = color;
    region.reason = reason.empty() ? "reject" : reason;
    region.bbox = bbox;
    region.contour = contour;
    region.sample_count = sample_count;
    region.metric = metric;
    debug_regions->push_back(region);
  }

  DebugAlignment find_best_depth_alignment(
    const std::string & color,
    const std::vector<cv::Point> & contour,
    const cv::Rect & bbox,
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & organized) const
  {
    DebugAlignment alignment;
    alignment.color = color;
    alignment.bbox = bbox;
    alignment.contour = contour;

    const int search_px = std::max(0, debug_depth_alignment_search_px_);
    const int step_px = std::max(1, debug_depth_alignment_step_px_);
    for (int dy = -search_px; dy <= search_px; dy += step_px) {
      for (int dx = -search_px; dx <= search_px; dx += step_px) {
        const int count = count_finite_depth_points(contour, bbox, organized, dx, dy);
        if (count > alignment.best_count) {
          alignment.best_count = count;
          alignment.best_dx = dx;
          alignment.best_dy = dy;
        }
      }
    }
    return alignment;
  }

  int count_finite_depth_points(
    const std::vector<cv::Point> & contour,
    const cv::Rect & bbox,
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & organized,
    int dx,
    int dy) const
  {
    cv::Mat contour_mask = cv::Mat::zeros(
      static_cast<int>(organized->height), static_cast<int>(organized->width), CV_8UC1);
    std::vector<std::vector<cv::Point>> contours{contour};
    cv::drawContours(contour_mask, contours, 0, cv::Scalar(255), cv::FILLED);

    int count = 0;
    const int x_end = std::min(bbox.x + bbox.width, static_cast<int>(organized->width));
    const int y_end = std::min(bbox.y + bbox.height, static_cast<int>(organized->height));
    for (int y = std::max(0, bbox.y); y < y_end; ++y) {
      for (int x = std::max(0, bbox.x); x < x_end; ++x) {
        if (contour_mask.at<uint8_t>(y, x) == 0) {
          continue;
        }
        const int sample_x = x + dx;
        const int sample_y = y + dy;
        if (sample_x < 0 || sample_y < 0 ||
          sample_x >= static_cast<int>(organized->width) ||
          sample_y >= static_cast<int>(organized->height))
        {
          continue;
        }
        const auto & point = organized->at(sample_x, sample_y);
        if (pcl::isFinite(point) && point.z >= depth_min_m_ && point.z <= depth_max_m_) {
          ++count;
        }
      }
    }
    return count;
  }

  bool passes_candidate_extent_gate(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
    Candidate * candidate,
    std::string * reject_reason,
    float * reject_metric) const
  {
    if (cloud->empty()) {
      set_reject(reject_reason, reject_metric, "extent", 0.0F);
      return false;
    }

    pcl::PointXYZRGB min_pt;
    pcl::PointXYZRGB max_pt;
    pcl::getMinMax3D(*cloud, min_pt, max_pt);
    std::array<float, 3> extents{
      std::abs(max_pt.x - min_pt.x),
      std::abs(max_pt.y - min_pt.y),
      std::abs(max_pt.z - min_pt.z)};
    std::sort(extents.begin(), extents.end());

    const float thickness = extents[0];
    const float middle = extents[1];
    const float largest = extents[2];
    if (candidate != nullptr) {
      candidate->largest_extent_m = largest;
      candidate->middle_extent_m = middle;
      candidate->thickness_m = thickness;
    }

    if (candidate_min_3d_largest_extent_m_ > 0.0 && largest < candidate_min_3d_largest_extent_m_) {
      set_reject(reject_reason, reject_metric, "extent", largest);
      return false;
    }
    if (candidate_max_3d_largest_extent_m_ > 0.0 && largest > candidate_max_3d_largest_extent_m_) {
      set_reject(reject_reason, reject_metric, "oversize", largest);
      return false;
    }
    if (candidate_min_3d_middle_extent_m_ > 0.0 && middle < candidate_min_3d_middle_extent_m_) {
      set_reject(reject_reason, reject_metric, "width", middle);
      return false;
    }
    if (candidate_min_3d_depth_extent_m_ > 0.0 && thickness < candidate_min_3d_depth_extent_m_) {
      set_reject(reject_reason, reject_metric, "thickness", thickness);
      return false;
    }

    if (candidate_min_3d_depth_to_middle_ratio_ > 0.0 && middle > 1e-4F) {
      const float ratio = thickness / middle;
      if (ratio < candidate_min_3d_depth_to_middle_ratio_) {
        set_reject(reject_reason, reject_metric, "flatness", ratio);
        return false;
      }
    }
    if (middle > 1e-4F && largest > 1e-4F) {
      const float ratio = middle / largest;
      if (candidate_min_3d_middle_largest_ratio_ > 0.0 &&
        ratio < candidate_min_3d_middle_largest_ratio_)
      {
        set_reject(reject_reason, reject_metric, "aspect", ratio);
        return false;
      }
      if (candidate_max_3d_middle_largest_ratio_ > 0.0 &&
        ratio > candidate_max_3d_middle_largest_ratio_)
      {
        set_reject(reject_reason, reject_metric, "aspect", ratio);
        return false;
      }
    }

    return true;
  }

  static void set_reject(
    std::string * reject_reason,
    float * reject_metric,
    const std::string & reason,
    float metric)
  {
    if (reject_reason != nullptr) {
      *reject_reason = reason;
    }
    if (reject_metric != nullptr) {
      *reject_metric = metric;
    }
  }

  cv::Mat build_mask(
    const cv::Mat & hsv,
    const std::vector<HsvRange> & ranges,
    int open_kernel,
    int close_kernel,
    bool open_first)
  {
    cv::Mat mask = cv::Mat::zeros(hsv.size(), CV_8UC1);
    for (const auto & range : ranges) {
      cv::Mat part;
      cv::inRange(hsv, range.low, range.high, part);
      cv::bitwise_or(mask, part, mask);
    }
    const int open_k = odd_kernel(open_kernel);
    const int close_k = odd_kernel(close_kernel);

    auto apply_open = [&]() {
      if (open_k <= 1) {
        return;
      }
      const auto kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, {open_k, open_k});
      cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
    };
    auto apply_close = [&]() {
      if (close_k <= 1) {
        return;
      }
      const auto kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, {close_k, close_k});
      cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
    };

    if (open_first) {
      apply_open();
      apply_close();
    } else {
      apply_close();
      apply_open();
    }
    return mask;
  }

  cv::Mat build_mask(const cv::Mat & hsv, const std::vector<HsvRange> & ranges)
  {
    return build_mask(hsv, ranges, mask_morph_open_kernel_, mask_morph_close_kernel_, false);
  }

  std::vector<PixelCluster> clusters_from_contour(
    const std::vector<cv::Point> & contour,
    const cv::Rect & bbox,
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & organized,
    int * finite_depth_points = nullptr)
  {
    cv::Mat contour_mask = cv::Mat::zeros(static_cast<int>(organized->height), static_cast<int>(organized->width), CV_8UC1);
    std::vector<std::vector<cv::Point>> contours{contour};
    cv::drawContours(contour_mask, contours, 0, cv::Scalar(255), cv::FILLED);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    std::vector<cv::Point> pixels;
    const int x_end = std::min(bbox.x + bbox.width, static_cast<int>(organized->width));
    const int y_end = std::min(bbox.y + bbox.height, static_cast<int>(organized->height));
    for (int y = std::max(0, bbox.y); y < y_end; ++y) {
      for (int x = std::max(0, bbox.x); x < x_end; ++x) {
        if (contour_mask.at<uint8_t>(y, x) == 0) {
          continue;
        }
        const auto & point = organized->at(x, y);
        if (!pcl::isFinite(point) || point.z > depth_max_m_) {
          continue;
        }
        cloud->push_back(point);
        pixels.emplace_back(x, y);
      }
    }
    if (finite_depth_points != nullptr) {
      *finite_depth_points = static_cast<int>(cloud->size());
    }
    if (static_cast<int>(cloud->size()) < cluster_min_points_) {
      return {};
    }

    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
    tree->setInputCloud(cloud);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance(cluster_tolerance_m_);
    ec.setMinClusterSize(cluster_min_points_);
    ec.setMaxClusterSize(cluster_max_points_);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    std::vector<PixelCluster> clusters;
    for (const auto & indices : cluster_indices) {
      PixelCluster cluster;
      cluster.cloud->reserve(indices.indices.size());
      cluster.pixels.reserve(indices.indices.size());
      for (const int idx : indices.indices) {
        cluster.cloud->push_back((*cloud)[idx]);
        if (idx >= 0 && idx < static_cast<int>(pixels.size())) {
          cluster.pixels.push_back(pixels[idx]);
        }
      }
      clusters.push_back(cluster);
    }
    return clusters;
  }

  bool fit_cylinder(
    const PixelCluster & cluster,
    Candidate & candidate,
    std::string * reject_reason = nullptr,
    float * reject_metric = nullptr)
  {
    const auto & cloud = cluster.cloud;
    if (static_cast<int>(cloud->size()) < cylinder_inlier_min_) {
      if (reject_reason != nullptr) {
        *reject_reason = "points";
      }
      if (reject_metric != nullptr) {
        *reject_metric = static_cast<float>(cloud->size());
      }
      return false;
    }

    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
    ne.setSearchMethod(tree);
    ne.setInputCloud(cloud);
    ne.setRadiusSearch(normal_search_radius_m_);
    ne.compute(*normals);

    pcl::SACSegmentationFromNormals<pcl::PointXYZRGB, pcl::Normal> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients());
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_CYLINDER);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight(ransac_normal_distance_weight_);
    seg.setMaxIterations(ransac_max_iterations_);
    seg.setDistanceThreshold(ransac_distance_threshold_m_);
    seg.setRadiusLimits(cylinder_radius_min_m_, cylinder_radius_max_m_);
    seg.setInputCloud(cloud);
    seg.setInputNormals(normals);
    seg.segment(*inliers, *coeff);

    if (static_cast<int>(inliers->indices.size()) < cylinder_inlier_min_ || coeff->values.size() < 7) {
      if (reject_reason != nullptr) {
        *reject_reason = "ransac";
      }
      if (reject_metric != nullptr) {
        *reject_metric = static_cast<float>(inliers->indices.size());
      }
      return false;
    }

    const Eigen::Vector3f axis_point(coeff->values[0], coeff->values[1], coeff->values[2]);
    Eigen::Vector3f axis(coeff->values[3], coeff->values[4], coeff->values[5]);
    if (axis.norm() < 1e-4F) {
      if (reject_reason != nullptr) {
        *reject_reason = "axis";
      }
      return false;
    }
    axis.normalize();
    const float radius = coeff->values[6];

    float residual_sum = 0.0F;
    Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
    for (const int idx : inliers->indices) {
      const auto & p = (*cloud)[idx];
      const Eigen::Vector3f point(p.x, p.y, p.z);
      centroid += point;
      const Eigen::Vector3f delta = point - axis_point;
      const float radial_distance = (delta - delta.dot(axis) * axis).norm();
      residual_sum += std::abs(radial_distance - radius);
    }
    const float residual = residual_sum / static_cast<float>(inliers->indices.size());
    if (residual > cylinder_fit_residual_max_m_) {
      if (reject_reason != nullptr) {
        *reject_reason = "residual";
      }
      if (reject_metric != nullptr) {
        *reject_metric = residual;
      }
      return false;
    }

    centroid /= static_cast<float>(inliers->indices.size());
    candidate.centroid_camera = centroid;
    candidate.axis = axis;
    candidate.inliers = static_cast<int>(inliers->indices.size());
    candidate.residual = residual;
    candidate.ransac_inlier_pixels.clear();
    candidate.ransac_inlier_pixels.reserve(inliers->indices.size());
    for (const int idx : inliers->indices) {
      if (idx >= 0 && idx < static_cast<int>(cluster.pixels.size())) {
        candidate.ransac_inlier_pixels.push_back(cluster.pixels[idx]);
      }
    }
    return true;
  }

  bool transform_candidate(
    const std_msgs::msg::Header & source_header,
    Candidate & candidate,
    std::string * reject_reason = nullptr,
    float * reject_metric = nullptr)
  {
    if (target_frame_ == source_header.frame_id || target_frame_.empty()) {
      candidate.centroid_map = candidate.centroid_camera;
      candidate.axis_map = candidate.axis;
      return apply_orientation_gate(candidate, reject_reason, reject_metric);
    }

    try {
      const auto transform = tf_buffer_.lookupTransform(
        target_frame_, source_header.frame_id, source_header.stamp, 100ms);

      geometry_msgs::msg::PointStamped source;
      source.header = source_header;
      source.point.x = candidate.centroid_camera.x();
      source.point.y = candidate.centroid_camera.y();
      source.point.z = candidate.centroid_camera.z();

      geometry_msgs::msg::PointStamped target;
      tf2::doTransform(source, target, transform);
      candidate.centroid_map = Eigen::Vector3f(target.point.x, target.point.y, target.point.z);

      geometry_msgs::msg::PointStamped axis_end;
      axis_end.header = source_header;
      const Eigen::Vector3f camera_axis_end = candidate.centroid_camera + candidate.axis;
      axis_end.point.x = camera_axis_end.x();
      axis_end.point.y = camera_axis_end.y();
      axis_end.point.z = camera_axis_end.z();

      geometry_msgs::msg::PointStamped target_axis_end;
      tf2::doTransform(axis_end, target_axis_end, transform);
      candidate.axis_map = Eigen::Vector3f(
        target_axis_end.point.x - target.point.x,
        target_axis_end.point.y - target.point.y,
        target_axis_end.point.z - target.point.z);
      if (candidate.axis_map.norm() > 1e-4F) {
        candidate.axis_map.normalize();
      } else {
        candidate.axis_map = candidate.axis;
      }
      return apply_orientation_gate(candidate, reject_reason, reject_metric);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "TF lookup failed: %s", ex.what());
      if (reject_reason != nullptr) {
        *reject_reason = "tf";
      }
      return false;
    }
  }

  bool apply_orientation_gate(
    Candidate & candidate,
    std::string * reject_reason = nullptr,
    float * reject_metric = nullptr) const
  {
    if (!std::isfinite(candidate.axis_map.x()) ||
      !std::isfinite(candidate.axis_map.y()) ||
      !std::isfinite(candidate.axis_map.z()))
    {
      if (reject_reason != nullptr) {
        *reject_reason = "orientation";
      }
      return false;
    }

    if (candidate.axis_map.norm() < 1e-4F) {
      if (reject_reason != nullptr) {
        *reject_reason = "orientation";
      }
      return false;
    }
    candidate.axis_map.normalize();

    const float vertical_dot = std::abs(candidate.axis_map.z());
    if (vertical_dot <= horizontal_dot_threshold_) {
      candidate.horizontal = true;
      set_candidate_normal(candidate);
      return true;
    }
    if (vertical_dot >= vertical_dot_threshold_) {
      candidate.horizontal = false;
      set_candidate_normal(candidate);
      return true;
    }

    RCLCPP_DEBUG(
      get_logger(),
      "Rejected %s barrel candidate by orientation: |axis.z|=%.3f, horizontal<=%.3f, vertical>=%.3f",
      candidate.color.c_str(),
      vertical_dot,
      horizontal_dot_threshold_,
      vertical_dot_threshold_);
    if (reject_reason != nullptr) {
      *reject_reason = "orientation";
    }
    if (reject_metric != nullptr) {
      *reject_metric = vertical_dot;
    }
    return false;
  }

  static void set_candidate_normal(Candidate & candidate)
  {
    if (!candidate.horizontal) {
      candidate.normal_x = 0.0F;
      candidate.normal_y = 0.0F;
      return;
    }

    Eigen::Vector2f normal(candidate.axis_map.x(), candidate.axis_map.y());
    if (normal.norm() < 1e-4F) {
      candidate.normal_x = 0.0F;
      candidate.normal_y = 0.0F;
      return;
    }
    normal.normalize();
    if (normal.x() < 0.0F || (std::abs(normal.x()) < 1e-4F && normal.y() < 0.0F)) {
      normal *= -1.0F;
    }
    candidate.normal_x = normal.x();
    candidate.normal_y = normal.y();
  }

  bool passes_height_gate(const Candidate & candidate) const
  {
    if (!std::isfinite(candidate.centroid_map.z())) {
      return false;
    }
    if (candidate.centroid_map.z() > max_barrel_height_m_) {
      RCLCPP_DEBUG(
        get_logger(),
        "Rejected %s barrel candidate by height: z=%.3f > max=%.3f at map=(%.2f, %.2f)",
        candidate.color.c_str(),
        candidate.centroid_map.z(),
        max_barrel_height_m_,
        candidate.centroid_map.x(),
        candidate.centroid_map.y());
      return false;
    }
    return true;
  }

  void update_tracks(const std::vector<Candidate> & candidates)
  {
    std::vector<bool> matched(tracks_.size(), false);
    for (const auto & candidate : candidates) {
      int best_idx = -1;
      double best_dist = std::numeric_limits<double>::max();
      for (size_t i = 0; i < tracks_.size(); ++i) {
        const double dx = tracks_[i].x - candidate.centroid_map.x();
        const double dy = tracks_[i].y - candidate.centroid_map.y();
        const double dist = std::hypot(dx, dy);
        if (track_matches_candidate_color(tracks_[i], candidate) &&
          dist < best_dist && dist <= dedup_distance_m_)
        {
          best_dist = dist;
          best_idx = static_cast<int>(i);
        }
      }
      if (best_idx < 0) {
        BarrelTrack track;
        track.id = next_track_id_++;
        track.horizontal_votes = std::deque<bool>();
        tracks_.push_back(track);
        matched.push_back(false);
        best_idx = static_cast<int>(tracks_.size() - 1);
      }

      auto & track = tracks_[best_idx];
      matched[best_idx] = true;
      const float alpha = track.seen_count > 0 ? 0.35F : 1.0F;
      track.x = alpha * candidate.centroid_map.x() + (1.0F - alpha) * track.x;
      track.y = alpha * candidate.centroid_map.y() + (1.0F - alpha) * track.y;
      track.z = alpha * candidate.centroid_map.z() + (1.0F - alpha) * track.z;
      update_track_normal(track, candidate);
      track.color_votes[candidate.color] += 1;
      push_limited(track.horizontal_votes, candidate.horizontal, stability_window_m_);
      track.last_bbox = candidate.bbox;
      track.seen_count += 1;
      track.missed_frames = 0;
      track.accepted = track.seen_count >= accept_threshold_;
    }

    for (size_t i = 0; i < tracks_.size(); ++i) {
      if (!matched[i]) {
        tracks_[i].missed_frames += 1;
      }
    }

    tracks_.erase(
      std::remove_if(
        tracks_.begin(), tracks_.end(),
        [this](const BarrelTrack & track) {
          return !track.published && track.missed_frames > track_timeout_frames_;
        }),
      tracks_.end());
  }

  static void update_track_normal(BarrelTrack & track, const Candidate & candidate)
  {
    Eigen::Vector2f incoming(candidate.normal_x, candidate.normal_y);
    if (incoming.norm() < 1e-4F) {
      return;
    }
    incoming.normalize();

    Eigen::Vector2f current(track.normal_x, track.normal_y);
    if (current.norm() < 1e-4F) {
      track.normal_x = incoming.x();
      track.normal_y = incoming.y();
      return;
    }
    current.normalize();
    if (current.dot(incoming) < 0.0F) {
      incoming *= -1.0F;
    }
    Eigen::Vector2f updated = 0.35F * incoming + 0.65F * current;
    if (updated.norm() >= 1e-4F) {
      updated.normalize();
      track.normal_x = updated.x();
      track.normal_y = updated.y();
    }
  }

  static void push_limited(std::deque<bool> & values, bool value, size_t limit)
  {
    values.push_back(value);
    while (values.size() > limit) {
      values.pop_front();
    }
  }

  bool should_publish(const BarrelTrack & track) const
  {
    if (!track.accepted) {
      return false;
    }
    if (!track.published) {
      return true;
    }
    const double dx = track.x - track.last_published_x;
    const double dy = track.y - track.last_published_y;
    if (std::hypot(dx, dy) > republish_move_threshold_m_) {
      return true;
    }
    if (track.color() != track.last_published_color) {
      return true;
    }
    if (track.leaking != track.last_published_leaking) {
      return true;
    }
    const Eigen::Vector2f current_normal = published_normal(track);
    const Eigen::Vector2f last_normal(track.last_published_normal_x, track.last_published_normal_y);
    return normal_angle_delta(current_normal, last_normal) > republish_rotation_threshold_rad_;
  }

  static Eigen::Vector2f published_normal(const BarrelTrack & track)
  {
    if (!track.horizontal()) {
      return Eigen::Vector2f::Zero();
    }
    return {track.normal_x, track.normal_y};
  }

  static double normal_angle_delta(Eigen::Vector2f a, Eigen::Vector2f b)
  {
    const float a_norm = a.norm();
    const float b_norm = b.norm();
    if (a_norm < 1e-4F && b_norm < 1e-4F) {
      return 0.0;
    }
    if (a_norm < 1e-4F || b_norm < 1e-4F) {
      return std::numeric_limits<double>::infinity();
    }
    a /= a_norm;
    b /= b_norm;
    const float dot = std::clamp(a.dot(b), -1.0F, 1.0F);
    return std::acos(dot);
  }

  int matching_track_index_for_candidate(const Candidate & candidate) const
  {
    int best_idx = -1;
    double best_dist = std::numeric_limits<double>::max();
    for (size_t i = 0; i < tracks_.size(); ++i) {
      const double dx = tracks_[i].x - candidate.centroid_map.x();
      const double dy = tracks_[i].y - candidate.centroid_map.y();
      const double dist = std::hypot(dx, dy);
      if (track_matches_candidate_color(tracks_[i], candidate) &&
        dist < best_dist && dist <= dedup_distance_m_)
      {
        best_dist = dist;
        best_idx = static_cast<int>(i);
      }
    }
    return best_idx;
  }

  static bool track_matches_candidate_color(const BarrelTrack & track, const Candidate & candidate)
  {
    const std::string track_color = track.color();
    return track_color == "unknown" || track_color == candidate.color;
  }

  void update_track_leak_state(BarrelTrack & track, bool leak_detected)
  {
    if (track.leak_confirmed_once) {
      track.leaking = true;
      return;
    }

    if (leak_detected) {
      track.leak_negative_count = 0;
      track.leak_positive_count += 1;
      if (track.leak_confirmed_once || track.leak_positive_count >= leak_confirm_threshold_) {
        track.leaking = true;
        track.leak_confirmed_once = true;
      }
      return;
    }

    track.leak_positive_count = 0;
    track.leak_negative_count = 0;
  }

  void publish_tracks()
  {
    visualization_msgs::msg::MarkerArray markers;
    const auto now = get_clock()->now();
    for (auto & track : tracks_) {
      if (!track.accepted) {
        continue;
      }
      if (should_publish(track)) {
        msg_types::msg::BarrelDetect msg;
        msg.x = track.x;
        msg.y = track.y;
        msg.normal_x = track.horizontal() ? track.normal_x : 0.0F;
        msg.normal_y = track.horizontal() ? track.normal_y : 0.0F;
        msg.id = static_cast<int8_t>(std::clamp(track.id, 0, 127));
        msg.color = track.color();
        msg.horizontal = track.horizontal();
        msg.leaking = track.leaking;
        barrel_pub_->publish(msg);
        track.published = true;
        track.last_published_x = track.x;
        track.last_published_y = track.y;
        const Eigen::Vector2f normal = published_normal(track);
        track.last_published_normal_x = normal.x();
        track.last_published_normal_y = normal.y();
        track.last_published_color = msg.color;
        track.last_published_leaking = msg.leaking;
      }

      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = target_frame_;
      marker.header.stamp = now;
      marker.ns = marker_namespace_;
      marker.id = track.id;
      marker.type = visualization_msgs::msg::Marker::CYLINDER;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position.x = track.x;
      marker.pose.position.y = track.y;
      marker.pose.position.z = track.z;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = marker_scale_x_;
      marker.scale.y = marker_scale_y_;
      marker.scale.z = marker_scale_z_;
      marker.color = marker_color(track.color(), static_cast<float>(marker_alpha_));
      marker.lifetime = rclcpp::Duration::from_seconds(marker_lifetime_s_);
      markers.markers.push_back(marker);
    }
    marker_pub_->publish(markers);
  }

  void publish_mask(const cv::Mat & mask, const std_msgs::msg::Header & header)
  {
    if (mask.empty()) {
      return;
    }

    auto out = cv_bridge::CvImage(header, "mono8", mask).toImageMsg();
    debug_mask_pub_->publish(*out);

    if (show_debug_window_) {
      cv::imshow(debug_mask_window_name_, mask);
      cv::waitKey(1);
    }
  }

  void log_rejection_summary(
    const std::vector<Candidate> & candidates,
    const std::vector<DebugRegion> & debug_regions)
  {
    std::unordered_map<std::string, int> counts;
    for (const auto & region : debug_regions) {
      counts[region.reason] += 1;
    }

    std::string summary = "barrel debug: accepted_candidates=" + std::to_string(candidates.size());
    for (const auto & [reason, count] : counts) {
      summary += " " + reason + "=" + std::to_string(count);
    }
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "%s", summary.c_str());
  }

  void publish_rejections(
    const cv::Mat & image,
    const std::vector<DebugRegion> & debug_regions,
    const std_msgs::msg::Header & header)
  {
    cv::Mat overlay = image.clone();
    for (const auto & region : debug_regions) {
      const auto color = draw_color(region.color);
      if (draw_barrel_outline_) {
        std::vector<std::vector<cv::Point>> contours{region.contour};
        cv::drawContours(overlay, contours, 0, color, 1);
        cv::rectangle(overlay, region.bbox, color, 1);
      }

      std::string label = region.color + " reject:" + region.reason;
      if ((region.reason == "depth" || region.reason == "cluster" ||
        region.reason == "points" || region.reason == "ransac") && std::isfinite(region.metric))
      {
        label += " n=" + std::to_string(static_cast<int>(region.metric));
      } else if (
        (region.reason == "area" || region.reason == "fill" || region.reason == "height" ||
        region.reason == "orientation" || region.reason == "residual" ||
        region.reason == "extent" || region.reason == "oversize" ||
        region.reason == "width" || region.reason == "thickness" ||
        region.reason == "flatness" || region.reason == "aspect") && std::isfinite(region.metric))
      {
        label += " " + std::to_string(region.metric).substr(0, 5);
      }

      cv::putText(
        overlay, label, {region.bbox.x, std::max(15, region.bbox.y - 6)},
        cv::FONT_HERSHEY_SIMPLEX, 0.42, color, 1, cv::LINE_AA);
    }

    auto out = cv_bridge::CvImage(header, "bgr8", overlay).toImageMsg();
    debug_rejection_pub_->publish(*out);

    if (show_debug_window_) {
      cv::imshow(debug_rejection_window_name_, overlay);
      cv::waitKey(1);
    }
  }

  void publish_depth_alignment(
    const cv::Mat & image,
    const std::vector<DebugAlignment> & debug_alignments,
    const std_msgs::msg::Header & header)
  {
    cv::Mat overlay = image.clone();
    for (const auto & alignment : debug_alignments) {
      const auto color = draw_color(alignment.color);
      std::vector<std::vector<cv::Point>> contours{alignment.contour};
      cv::drawContours(overlay, contours, 0, color, 1);
      cv::rectangle(overlay, alignment.bbox, color, 1);

      cv::Rect shifted = alignment.bbox;
      shifted.x += alignment.best_dx;
      shifted.y += alignment.best_dy;
      shifted &= cv::Rect(0, 0, image.cols, image.rows);
      if (shifted.area() > 0) {
        cv::rectangle(overlay, shifted, {255, 255, 0}, 2);
        cv::arrowedLine(
          overlay,
          {alignment.bbox.x + alignment.bbox.width / 2, alignment.bbox.y + alignment.bbox.height / 2},
          {shifted.x + shifted.width / 2, shifted.y + shifted.height / 2},
          {255, 255, 0}, 2, cv::LINE_AA, 0, 0.25);
      }

      const std::string label =
        alignment.color + " depth align dx=" + std::to_string(alignment.best_dx) +
        " dy=" + std::to_string(alignment.best_dy) +
        " n=" + std::to_string(alignment.best_count);
      cv::putText(
        overlay, label, {alignment.bbox.x, std::max(15, alignment.bbox.y - 22)},
        cv::FONT_HERSHEY_SIMPLEX, 0.42, {255, 255, 0}, 1, cv::LINE_AA);
    }

    auto out = cv_bridge::CvImage(header, "bgr8", overlay).toImageMsg();
    debug_depth_alignment_pub_->publish(*out);

    if (show_debug_window_) {
      cv::imshow(debug_depth_alignment_window_name_, overlay);
      cv::waitKey(1);
    }
  }

  void publish_depth_validity(
    const cv::Mat & image,
    const cv::Mat & mask,
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & organized,
    const std_msgs::msg::Header & header)
  {
    cv::Mat overlay = image.clone();
    for (int y = 0; y < image.rows; ++y) {
      for (int x = 0; x < image.cols; ++x) {
        const auto & point = organized->at(x, y);
        const bool valid_depth =
          pcl::isFinite(point) && point.z >= depth_min_m_ && point.z <= depth_max_m_;
        const bool masked = !mask.empty() && mask.at<uint8_t>(y, x) != 0;

        if (masked && valid_depth) {
          overlay.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 255, 255);
        } else if (masked) {
          overlay.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 255);
        } else if (valid_depth) {
          const cv::Vec3b original = overlay.at<cv::Vec3b>(y, x);
          overlay.at<cv::Vec3b>(y, x) = cv::Vec3b(
            static_cast<uint8_t>(original[0] * 0.35),
            static_cast<uint8_t>(std::min(255.0, original[1] * 0.35 + 160.0)),
            static_cast<uint8_t>(original[2] * 0.35));
        } else {
          const cv::Vec3b original = overlay.at<cv::Vec3b>(y, x);
          overlay.at<cv::Vec3b>(y, x) = cv::Vec3b(
            static_cast<uint8_t>(original[0] * 0.25),
            static_cast<uint8_t>(original[1] * 0.25),
            static_cast<uint8_t>(original[2] * 0.25));
        }
      }
    }

    cv::putText(
      overlay,
      "green=valid depth  yellow=mask+depth  red=mask no depth",
      {8, 18}, cv::FONT_HERSHEY_SIMPLEX, 0.45, {255, 255, 255}, 1, cv::LINE_AA);

    auto out = cv_bridge::CvImage(header, "bgr8", overlay).toImageMsg();
    debug_depth_validity_pub_->publish(*out);

    if (show_debug_window_) {
      cv::imshow(debug_depth_validity_window_name_, overlay);
      cv::waitKey(1);
    }
  }

  void publish_leak_overlay(
    const cv::Mat & image,
    const cv::Mat & hsv,
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & organized,
    const std::vector<Candidate> & candidates,
    const std_msgs::msg::Header & header)
  {
    cv::Mat overlay = image.clone();
    std::vector<LeakDebugCandidate> leak_candidates;
    std::vector<bool> leak_observed_by_track(tracks_.size(), false);
    std::vector<bool> leak_positive_by_track(tracks_.size(), false);
    const cv::Mat leak_source_z_mask = build_leak_source_z_mask(hsv.size(), organized);

    for (const auto & barrel : candidates) {
      if (leak_horizontal_barrels_only_ && !barrel.horizontal) {
        continue;
      }
      const int track_idx = matching_track_index_for_candidate(barrel);
      if (track_idx >= 0) {
        leak_observed_by_track[static_cast<size_t>(track_idx)] = true;
      }

      const cv::Rect image_rect(0, 0, image.cols, image.rows);
      cv::Rect search = barrel.bbox;
      search.x -= leak_search_padding_px_;
      search.y -= leak_search_padding_px_;
      search.width += leak_search_padding_px_ * 2;
      search.height += leak_search_padding_px_ * 2;
      search &= image_rect;
      if (search.area() <= 0) {
        continue;
      }

      cv::rectangle(overlay, search, {255, 255, 0}, 1);

      for (const auto & [leak_color_name, ranges] : hsv_ranges_) {
        cv::Mat leak_mask = build_mask(
          hsv, ranges, leak_mask_morph_open_kernel_, leak_mask_morph_close_kernel_, true);
        cv::bitwise_and(leak_mask, leak_source_z_mask, leak_mask);
        cv::Mat roi_mask = leak_mask(search).clone();

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(roi_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        for (auto contour : contours) {
          for (auto & point : contour) {
            point.x += search.x;
            point.y += search.y;
          }

          LeakDebugCandidate leak;
          leak.color = leak_color_name;
          leak.contour = contour;
          leak.bbox = cv::boundingRect(contour);
          leak.area_px = cv::contourArea(contour);
          evaluate_leak_candidate(barrel, leak, organized, header);
          if (leak.accepted && track_idx >= 0) {
            leak_positive_by_track[static_cast<size_t>(track_idx)] = true;
          }
          leak_candidates.push_back(leak);
        }
      }
    }

    const size_t track_count = std::min(tracks_.size(), leak_observed_by_track.size());
    for (size_t i = 0; i < track_count; ++i) {
      if (leak_observed_by_track[i]) {
        update_track_leak_state(tracks_[i], leak_positive_by_track[i]);
      }
    }

    for (const auto & barrel : candidates) {
      const auto color = draw_color(barrel.color);
      cv::rectangle(overlay, barrel.bbox, color, 1);
    }

    for (const auto & leak : leak_candidates) {
      const cv::Scalar color = leak.accepted ? cv::Scalar(255, 0, 255) : cv::Scalar(80, 80, 80);
      const int thickness = leak.accepted ? 3 : 1;
      std::vector<std::vector<cv::Point>> contours{leak.contour};
      cv::drawContours(overlay, contours, 0, color, thickness);
      draw_fitted_leak_shape(overlay, leak.contour, color, thickness);

      std::string label;
      if (leak.accepted) {
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(2)
           << "LEAK " << leak.color
           << " area " << static_cast<int>(leak.area_px)
           << " fill " << leak.fill_ratio
           << " circ " << leak.circularity
           << " z " << leak.source_z_inlier_ratio;
        label = ss.str();
      } else {
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(2)
           << leak.reason
           << " a " << static_cast<int>(leak.area_px)
           << " f " << leak.fill_ratio
           << " c " << leak.circularity
           << " ax " << leak.axis_ratio
           << " z " << leak.source_z_inlier_ratio;
        label = ss.str();
      }
      cv::putText(
        overlay, label, {leak.bbox.x, std::max(15, leak.bbox.y - 6)},
        cv::FONT_HERSHEY_SIMPLEX, 0.42, color, 1, cv::LINE_AA);
    }

    auto out = cv_bridge::CvImage(header, "bgr8", overlay).toImageMsg();
    debug_leak_pub_->publish(*out);

    cv::imshow(debug_leak_window_name_, overlay);
    cv::waitKey(1);
  }

  cv::Mat build_leak_source_z_mask(
    const cv::Size & image_size,
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & organized) const
  {
    cv::Mat mask = cv::Mat::zeros(image_size, CV_8UC1);
    const double source_z_min = std::min(leak_source_z_min_m_, leak_source_z_max_m_);
    const double source_z_max = std::max(leak_source_z_min_m_, leak_source_z_max_m_);
    const int width = std::min(image_size.width, static_cast<int>(organized->width));
    const int height = std::min(image_size.height, static_cast<int>(organized->height));

    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        const auto & point = organized->at(x, y);
        if (pcl::isFinite(point) && point.z >= source_z_min && point.z <= source_z_max) {
          mask.at<uint8_t>(y, x) = 255;
        }
      }
    }
    return mask;
  }

  static void draw_fitted_leak_shape(
    cv::Mat & overlay,
    const std::vector<cv::Point> & contour,
    const cv::Scalar & color,
    int thickness)
  {
    const int safe_thickness = std::max(1, thickness);
    if (contour.size() >= 5) {
      const cv::RotatedRect fitted = cv::fitEllipse(contour);
      const float width = fitted.size.width;
      const float height = fitted.size.height;
      if (
        std::isfinite(width) && std::isfinite(height) &&
        width > 0.0F && height > 0.0F)
      {
        cv::ellipse(overlay, fitted, color, safe_thickness, cv::LINE_AA);
        return;
      }
    }

    cv::Point2f center;
    float radius = 0.0F;
    cv::minEnclosingCircle(contour, center, radius);
    if (radius > 0.0F) {
      cv::circle(
        overlay, center, static_cast<int>(std::round(radius)), color, safe_thickness, cv::LINE_AA);
    }
  }

  void evaluate_leak_candidate(
    const Candidate & barrel,
    LeakDebugCandidate & leak,
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & organized,
    const std_msgs::msg::Header & header)
  {
    if (leak.area_px < leak_min_area_px_ || leak.area_px > leak_max_area_px_) {
      leak.reason = "area";
      return;
    }
    leak.fill_ratio = leak.area_px / std::max(1, leak.bbox.area());
    if (leak.fill_ratio < leak_min_fill_ratio_) {
      leak.reason = "fill";
      return;
    }

    const double perimeter = cv::arcLength(leak.contour, true);
    if (perimeter <= 1e-6) {
      leak.reason = "perimeter";
      return;
    }
    constexpr double kPi = 3.14159265358979323846;
    leak.circularity = 4.0 * kPi * leak.area_px / (perimeter * perimeter);
    if (leak.circularity < leak_min_circularity_) {
      leak.reason = "circle";
      return;
    }

    leak.axis_ratio = static_cast<double>(std::min(leak.bbox.width, leak.bbox.height)) /
      static_cast<double>(std::max(1, std::max(leak.bbox.width, leak.bbox.height)));
    if (leak.contour.size() >= 5) {
      const cv::RotatedRect ellipse = cv::fitEllipse(leak.contour);
      const double major = std::max(ellipse.size.width, ellipse.size.height);
      const double minor = std::min(ellipse.size.width, ellipse.size.height);
      if (major > 1e-6) {
        leak.axis_ratio = minor / major;
      }
    }
    if (leak.axis_ratio < leak_min_axis_ratio_) {
      leak.reason = "axis";
      return;
    }

    cv::Mat contour_mask = cv::Mat::zeros(
      static_cast<int>(organized->height), static_cast<int>(organized->width), CV_8UC1);
    std::vector<std::vector<cv::Point>> contours{leak.contour};
    cv::drawContours(contour_mask, contours, 0, cv::Scalar(255), cv::FILLED);

    std::vector<Eigen::Vector3f> map_points;
    Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
    float source_min_z = std::numeric_limits<float>::max();
    float source_max_z = -std::numeric_limits<float>::max();
    int source_z_inliers = 0;
    const double source_z_min = std::min(leak_source_z_min_m_, leak_source_z_max_m_);
    const double source_z_max = std::max(leak_source_z_min_m_, leak_source_z_max_m_);
    const int x_end = std::min(leak.bbox.x + leak.bbox.width, static_cast<int>(organized->width));
    const int y_end = std::min(leak.bbox.y + leak.bbox.height, static_cast<int>(organized->height));
    for (int y = std::max(0, leak.bbox.y); y < y_end; ++y) {
      for (int x = std::max(0, leak.bbox.x); x < x_end; ++x) {
        if (contour_mask.at<uint8_t>(y, x) == 0) {
          continue;
        }
        const auto & point = organized->at(x, y);
        if (!pcl::isFinite(point)) {
          continue;
        }
        source_min_z = std::min(source_min_z, point.z);
        source_max_z = std::max(source_max_z, point.z);
        if (point.z >= source_z_min && point.z <= source_z_max) {
          ++source_z_inliers;
        }

        Eigen::Vector3f map_point;
        if (!transform_point_to_target(header, Eigen::Vector3f(point.x, point.y, point.z), &map_point)) {
          leak.reason = "tf";
          return;
        }
        map_points.push_back(map_point);
        centroid += map_point;
      }
    }

    leak.point_count = static_cast<int>(map_points.size());
    if (leak.point_count < leak_min_points_) {
      leak.reason = "points";
      return;
    }
    leak.source_min_z = source_min_z;
    leak.source_max_z = source_max_z;
    leak.source_z_inlier_ratio =
      static_cast<double>(source_z_inliers) / static_cast<double>(leak.point_count);
    if (leak.source_z_inlier_ratio < leak_source_z_inlier_ratio_min_) {
      leak.reason = "zband";
      return;
    }

    centroid /= static_cast<float>(map_points.size());
    leak.centroid_z = centroid.z();
    leak.distance_m = std::hypot(
      centroid.x() - barrel.centroid_map.x(), centroid.y() - barrel.centroid_map.y());
    if (leak.distance_m > leak_max_distance_from_barrel_m_) {
      leak.reason = "distance";
      return;
    }
    if (leak.centroid_z < leak_min_height_m_ || leak.centroid_z > leak_max_height_m_) {
      leak.reason = "height";
      return;
    }

    Eigen::Vector3f min_pt = map_points.front();
    Eigen::Vector3f max_pt = map_points.front();
    for (const auto & point : map_points) {
      min_pt = min_pt.cwiseMin(point);
      max_pt = max_pt.cwiseMax(point);
    }
    std::array<float, 3> extents{
      std::abs(max_pt.x() - min_pt.x()),
      std::abs(max_pt.y() - min_pt.y()),
      std::abs(max_pt.z() - min_pt.z())};
    std::sort(extents.begin(), extents.end());
    leak.thickness_m = extents[0];
    if (leak.thickness_m > leak_max_thickness_m_) {
      leak.reason = "thick";
      return;
    }

    leak.accepted = true;
    leak.reason = "accepted";
  }

  bool transform_point_to_target(
    const std_msgs::msg::Header & source_header,
    const Eigen::Vector3f & source_point,
    Eigen::Vector3f * target_point)
  {
    if (target_point == nullptr) {
      return false;
    }
    if (target_frame_ == source_header.frame_id || target_frame_.empty()) {
      *target_point = source_point;
      return true;
    }

    try {
      const auto transform = tf_buffer_.lookupTransform(
        target_frame_, source_header.frame_id, source_header.stamp, 100ms);
      geometry_msgs::msg::PointStamped source;
      source.header = source_header;
      source.point.x = source_point.x();
      source.point.y = source_point.y();
      source.point.z = source_point.z();

      geometry_msgs::msg::PointStamped target;
      tf2::doTransform(source, target, transform);
      *target_point = Eigen::Vector3f(target.point.x, target.point.y, target.point.z);
      return true;
    } catch (const tf2::TransformException &) {
      return false;
    }
  }

  void publish_overlay(
    const cv::Mat & image,
    const std::vector<Candidate> & candidates,
    const std_msgs::msg::Header & header)
  {
    cv::Mat overlay = image.clone();
    for (const auto & candidate : candidates) {
      const auto color = draw_color(candidate.color);
      const int track_idx = matching_track_index_for_candidate(candidate);
      const BarrelTrack * track = track_idx >= 0 ? &tracks_[static_cast<size_t>(track_idx)] : nullptr;
      const bool accepted = track != nullptr && track->accepted;
      const int thickness = accepted ? 3 : 1;
      if (draw_barrel_outline_) {
        cv::ellipse(
          overlay,
          candidate.center_px,
          {std::max(4, candidate.bbox.width / 2), std::max(4, candidate.bbox.height / 2)},
          0.0, 0.0, 360.0, color, thickness);
        cv::rectangle(overlay, candidate.bbox, color, thickness);
      }
      if (draw_normal_arrow_ && candidate.horizontal) {
        cv::Point arrow_end(
          candidate.center_px.x + static_cast<int>(candidate.normal_x * 32.0F),
          candidate.center_px.y - static_cast<int>(candidate.normal_y * 32.0F));
        cv::arrowedLine(overlay, candidate.center_px, arrow_end, color, 2, cv::LINE_AA, 0, 0.25);
      }
      if (draw_track_ids_) {
        std::string label = candidate.color + (candidate.horizontal ? " horizontal" : " vertical");
        if (track != nullptr) {
          label =
            "id=" + std::to_string(track->id) + " " +
            (accepted ? "ACCEPTED" : "seen " + std::to_string(track->seen_count) + "/" + std::to_string(accept_threshold_)) +
            (track->published ? " published " : " ") +
            candidate.color + (candidate.horizontal ? " horizontal" : " vertical");
        }
        cv::putText(
          overlay, label, {candidate.bbox.x, std::max(15, candidate.bbox.y - 6)},
          cv::FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv::LINE_AA);
      }
      if (draw_blob_metrics_) {
        const std::string label =
          "inliers " + std::to_string(candidate.inliers) +
          " ext " + std::to_string(candidate.largest_extent_m).substr(0, 4) +
          " thick " + std::to_string(candidate.thickness_m).substr(0, 4) +
          " residual " + std::to_string(candidate.residual).substr(0, 4) +
          " z " + std::to_string(candidate.centroid_map.z()).substr(0, 4) +
          " normal (" + std::to_string(candidate.normal_x).substr(0, 4) +
          "," + std::to_string(candidate.normal_y).substr(0, 4) + ")";
        cv::putText(
          overlay, label, {candidate.bbox.x, candidate.bbox.y + candidate.bbox.height + 16},
          cv::FONT_HERSHEY_SIMPLEX, 0.4, color, 1, cv::LINE_AA);
      }
    }
    auto out = cv_bridge::CvImage(header, "bgr8", overlay).toImageMsg();
    debug_pub_->publish(*out);

    cv::imshow(debug_window_name_, overlay);
    cv::waitKey(1);
  }

  std::string image_topic_;
  std::string point_cloud_topic_;
  std::string target_frame_;
  std::string barrel_topic_;
  std::string marker_topic_;
  std::string debug_overlay_topic_;
  std::string debug_mask_topic_;
  std::string debug_rejection_topic_;
  std::string debug_depth_alignment_topic_;
  std::string debug_depth_validity_topic_;
  std::string debug_leak_overlay_topic_;
  double publish_hz_{2.0};
  uint32_t sync_queue_size_{5};
  double sync_slop_s_{0.08};
  int next_track_id_{100};
  int accept_threshold_{4};
  double dedup_distance_m_{2.0};
  double republish_move_threshold_m_{0.05};
  double republish_rotation_threshold_rad_{0.1};
  int track_timeout_frames_{150};
  double max_barrel_height_m_{0.70};
  double depth_min_m_{0.15};
  double depth_max_m_{6.0};
  double cluster_tolerance_m_{0.07};
  int cluster_min_points_{40};
  int cluster_max_points_{20000};
  double candidate_min_3d_largest_extent_m_{0.0};
  double candidate_max_3d_largest_extent_m_{0.0};
  double candidate_min_3d_middle_extent_m_{0.0};
  double candidate_min_3d_depth_extent_m_{0.0};
  double candidate_min_3d_depth_to_middle_ratio_{0.0};
  double candidate_min_3d_middle_largest_ratio_{0.0};
  double candidate_max_3d_middle_largest_ratio_{0.0};
  double candidate_max_distance_m_{2.0};
  double normal_search_radius_m_{0.05};
  int ransac_max_iterations_{250};
  double ransac_distance_threshold_m_{0.035};
  double ransac_normal_distance_weight_{0.1};
  double cylinder_radius_min_m_{0.08};
  double cylinder_radius_max_m_{0.45};
  int cylinder_inlier_min_{35};
  double cylinder_fit_residual_max_m_{0.05};
  double vertical_dot_threshold_{0.75};
  double horizontal_dot_threshold_{0.35};
  double candidate_min_area_px_{180.0};
  double candidate_max_area_px_{90000.0};
  int candidate_min_width_px_{12};
  int candidate_min_height_px_{12};
  double color_confidence_min_{0.45};
  int mask_morph_open_kernel_{3};
  int mask_morph_close_kernel_{9};
  size_t stability_window_m_{8};
  double marker_scale_x_{0.28};
  double marker_scale_y_{0.28};
  double marker_scale_z_{0.45};
  double marker_alpha_{0.9};
  double marker_lifetime_s_{1.5};
  std::string marker_namespace_{"barrels"};
  bool enable_debug_overlay_{true};
  bool enable_debug_mask_{true};
  bool enable_debug_rejections_{true};
  bool enable_debug_rejection_log_{true};
  bool enable_debug_depth_alignment_{true};
  bool enable_debug_depth_validity_{true};
  bool enable_debug_leak_overlay_{true};
  int leak_search_padding_px_{60};
  int leak_mask_morph_open_kernel_{3};
  int leak_mask_morph_close_kernel_{7};
  double leak_min_area_px_{80.0};
  double leak_max_area_px_{20000.0};
  double leak_min_fill_ratio_{0.45};
  double leak_min_circularity_{0.30};
  double leak_min_axis_ratio_{0.25};
  int leak_min_points_{15};
  double leak_min_height_m_{0.005};
  double leak_max_height_m_{0.08};
  double leak_source_z_min_m_{-0.24};
  double leak_source_z_max_m_{-0.21};
  double leak_source_z_inlier_ratio_min_{0.85};
  double leak_max_thickness_m_{0.035};
  double leak_max_distance_from_barrel_m_{0.85};
  bool leak_horizontal_barrels_only_{true};
  int leak_confirm_threshold_{3};
  int leak_clear_threshold_{3};
  int debug_depth_alignment_search_px_{50};
  int debug_depth_alignment_step_px_{5};
  std::string debug_window_name_{"barrel"};
  std::string debug_mask_window_name_{"barrel_mask"};
  std::string debug_rejection_window_name_{"barrel_rejections"};
  std::string debug_depth_alignment_window_name_{"barrel_depth_alignment"};
  std::string debug_depth_validity_window_name_{"barrel_depth_validity"};
  std::string debug_leak_window_name_{"barrel_leak"};
  bool show_debug_window_{false};
  bool draw_barrel_outline_{true};
  bool draw_blob_metrics_{true};
  bool draw_track_ids_{true};
  bool draw_normal_arrow_{true};

  std::unordered_map<std::string, std::vector<HsvRange>> hsv_ranges_;
  std::vector<BarrelTrack> tracks_;

  message_filters::Subscriber<Image> image_sub_;
  message_filters::Subscriber<PointCloud2> cloud_sub_;
  std::shared_ptr<Synchronizer> sync_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<msg_types::msg::BarrelDetect>::SharedPtr barrel_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_mask_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_rejection_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_depth_alignment_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_depth_validity_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_leak_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

int main(int argc, char ** argv)
{
  std::vector<std::string> args;
  args.reserve(static_cast<size_t>(argc));
  for (int i = 0; i < argc; ++i) {
    args.emplace_back(argv[i]);
  }

  bool has_params_file = false;
  for (size_t i = 1; i < args.size(); ++i) {
    if (args[i] == "--params-file") {
      has_params_file = true;
      break;
    }
  }

  rclcpp::NodeOptions options;
  if (!has_params_file) {
    const std::string default_params = std::filesystem::path(
      ament_index_cpp::get_package_share_directory("barrel_leak_cpp")) /
      "config/barrel_leak_cpp.yaml";
    options.arguments({"--ros-args", "--params-file", default_params});
  }

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BarrelDetectorCpp>(options));
  rclcpp::shutdown();
  return 0;
}
