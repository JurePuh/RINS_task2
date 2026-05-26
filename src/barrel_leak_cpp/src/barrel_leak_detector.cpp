#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <limits>
#include <memory>
#include <numeric>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <pcl/common/common.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
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
  bool horizontal{false};
  int inliers{0};
  float residual{0.0F};
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
  float last_published_x{0.0F};
  float last_published_y{0.0F};
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
  BarrelDetectorCpp()
  : Node("detect_barrel_cpp"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    declare_parameters();
    load_parameters();

    barrel_pub_ = create_publisher<msg_types::msg::BarrelDetect>(barrel_topic_, 10);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic_, 10);
    debug_pub_ = create_publisher<sensor_msgs::msg::Image>(debug_overlay_topic_, 10);

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
    declare_parameter("publish_hz", 2.0);
    declare_parameter("sync_queue_size", 5);
    declare_parameter("sync_slop_s", 0.08);
    declare_parameter("first_track_id", 100);
    declare_parameter("accept_threshold", 4);
    declare_parameter("dedup_distance_m", 0.45);
    declare_parameter("republish_move_threshold_m", 0.05);
    declare_parameter("track_timeout_frames", 18);
    declare_parameter("max_barrel_height_m", 0.70);
    declare_parameter("depth_min_m", 0.15);
    declare_parameter("depth_max_m", 6.0);
    declare_parameter("cluster_tolerance_m", 0.07);
    declare_parameter("cluster_min_points", 40);
    declare_parameter("cluster_max_points", 20000);
    declare_parameter("normal_search_radius_m", 0.05);
    declare_parameter("ransac_max_iterations", 250);
    declare_parameter("ransac_distance_threshold_m", 0.035);
    declare_parameter("ransac_normal_distance_weight", 0.1);
    declare_parameter("cylinder_radius_min_m", 0.08);
    declare_parameter("cylinder_radius_max_m", 0.45);
    declare_parameter("cylinder_inlier_min", 35);
    declare_parameter("cylinder_fit_residual_max_m", 0.05);
    declare_parameter("horizontal_dot_threshold", 0.55);
    declare_parameter("use_color_prefilter", true);
    declare_parameter("candidate_min_area_px", 180.0);
    declare_parameter("candidate_max_area_px", 90000.0);
    declare_parameter("candidate_min_width_px", 12);
    declare_parameter("candidate_min_height_px", 12);
    declare_parameter("color_confidence_min", 0.45);
    declare_parameter("mask_morph_open_kernel", 3);
    declare_parameter("mask_morph_close_kernel", 9);
    declare_parameter("allow_gray_fallback_label", false);
    declare_parameter("gray_saturation_max", 35);
    declare_parameter("stability_window_m", 8);
    declare_parameter("stability_required_n", 5);
    declare_parameter("marker_scale_x", 0.28);
    declare_parameter("marker_scale_y", 0.28);
    declare_parameter("marker_scale_z", 0.45);
    declare_parameter("marker_alpha", 0.9);
    declare_parameter("marker_lifetime_s", 1.5);
    declare_parameter("marker_namespace", "barrels");
    declare_parameter("enable_debug_overlay", true);
    declare_parameter("debug_window_name", "barrel");
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
    publish_hz_ = get_parameter("publish_hz").as_double();
    sync_queue_size_ = static_cast<uint32_t>(get_parameter("sync_queue_size").as_int());
    sync_slop_s_ = get_parameter("sync_slop_s").as_double();
    next_track_id_ = static_cast<int>(get_parameter("first_track_id").as_int());
    accept_threshold_ = static_cast<int>(get_parameter("accept_threshold").as_int());
    dedup_distance_m_ = get_parameter("dedup_distance_m").as_double();
    republish_move_threshold_m_ = get_parameter("republish_move_threshold_m").as_double();
    track_timeout_frames_ = static_cast<int>(get_parameter("track_timeout_frames").as_int());
    max_barrel_height_m_ = get_parameter("max_barrel_height_m").as_double();
    depth_min_m_ = get_parameter("depth_min_m").as_double();
    depth_max_m_ = get_parameter("depth_max_m").as_double();
    cluster_tolerance_m_ = get_parameter("cluster_tolerance_m").as_double();
    cluster_min_points_ = static_cast<int>(get_parameter("cluster_min_points").as_int());
    cluster_max_points_ = static_cast<int>(get_parameter("cluster_max_points").as_int());
    normal_search_radius_m_ = get_parameter("normal_search_radius_m").as_double();
    ransac_max_iterations_ = static_cast<int>(get_parameter("ransac_max_iterations").as_int());
    ransac_distance_threshold_m_ = get_parameter("ransac_distance_threshold_m").as_double();
    ransac_normal_distance_weight_ = get_parameter("ransac_normal_distance_weight").as_double();
    cylinder_radius_min_m_ = get_parameter("cylinder_radius_min_m").as_double();
    cylinder_radius_max_m_ = get_parameter("cylinder_radius_max_m").as_double();
    cylinder_inlier_min_ = static_cast<int>(get_parameter("cylinder_inlier_min").as_int());
    cylinder_fit_residual_max_m_ = get_parameter("cylinder_fit_residual_max_m").as_double();
    horizontal_dot_threshold_ = get_parameter("horizontal_dot_threshold").as_double();
    candidate_min_area_px_ = get_parameter("candidate_min_area_px").as_double();
    candidate_max_area_px_ = get_parameter("candidate_max_area_px").as_double();
    candidate_min_width_px_ = static_cast<int>(get_parameter("candidate_min_width_px").as_int());
    candidate_min_height_px_ = static_cast<int>(get_parameter("candidate_min_height_px").as_int());
    color_confidence_min_ = get_parameter("color_confidence_min").as_double();
    mask_morph_open_kernel_ = static_cast<int>(get_parameter("mask_morph_open_kernel").as_int());
    mask_morph_close_kernel_ = static_cast<int>(get_parameter("mask_morph_close_kernel").as_int());
    stability_window_m_ = static_cast<size_t>(get_parameter("stability_window_m").as_int());
    stability_required_n_ = static_cast<int>(get_parameter("stability_required_n").as_int());
    marker_scale_x_ = get_parameter("marker_scale_x").as_double();
    marker_scale_y_ = get_parameter("marker_scale_y").as_double();
    marker_scale_z_ = get_parameter("marker_scale_z").as_double();
    marker_alpha_ = get_parameter("marker_alpha").as_double();
    marker_lifetime_s_ = get_parameter("marker_lifetime_s").as_double();
    marker_namespace_ = get_parameter("marker_namespace").as_string();
    enable_debug_overlay_ = get_parameter("enable_debug_overlay").as_bool();
    debug_window_name_ = get_parameter("debug_window_name").as_string();
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
    auto candidates = detect_candidates(hsv, organized, cloud_msg->header);
    update_tracks(candidates);

    if (enable_debug_overlay_) {
      publish_overlay(cv_ptr->image, candidates, image_msg->header);
    }
  }

  std::vector<Candidate> detect_candidates(
    const cv::Mat & hsv,
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & organized,
    const std_msgs::msg::Header & header)
  {
    std::vector<Candidate> candidates;
    for (const auto & [color, ranges] : hsv_ranges_) {
      cv::Mat mask = build_mask(hsv, ranges);
      std::vector<std::vector<cv::Point>> contours;
      cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
      for (const auto & contour : contours) {
        const double area = cv::contourArea(contour);
        if (area < candidate_min_area_px_ || area > candidate_max_area_px_) {
          continue;
        }
        const cv::Rect bbox = cv::boundingRect(contour);
        if (bbox.width < candidate_min_width_px_ || bbox.height < candidate_min_height_px_) {
          continue;
        }
        const float confidence = static_cast<float>(area / std::max(1, bbox.area()));
        if (confidence < color_confidence_min_) {
          continue;
        }

        auto cluster_clouds = clusters_from_contour(contour, bbox, organized);
        for (const auto & cluster : cluster_clouds) {
          Candidate candidate;
          candidate.color = color;
          candidate.confidence = confidence;
          candidate.bbox = bbox;
          candidate.center_px = {(bbox.x + bbox.width / 2), (bbox.y + bbox.height / 2)};
          candidate.contour = contour;
          if (!fit_cylinder(cluster, candidate)) {
            continue;
          }
          if (!transform_candidate(header, candidate)) {
            continue;
          }
          if (!passes_height_gate(candidate)) {
            continue;
          }
          candidates.push_back(candidate);
        }
      }
    }
    return candidates;
  }

  cv::Mat build_mask(const cv::Mat & hsv, const std::vector<HsvRange> & ranges)
  {
    cv::Mat mask = cv::Mat::zeros(hsv.size(), CV_8UC1);
    for (const auto & range : ranges) {
      cv::Mat part;
      cv::inRange(hsv, range.low, range.high, part);
      cv::bitwise_or(mask, part, mask);
    }
    const int close_k = odd_kernel(mask_morph_close_kernel_);
    const int open_k = odd_kernel(mask_morph_open_kernel_);
    if (close_k > 1) {
      const auto kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, {close_k, close_k});
      cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
    }
    if (open_k > 1) {
      const auto kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, {open_k, open_k});
      cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
    }
    return mask;
  }

  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clusters_from_contour(
    const std::vector<cv::Point> & contour,
    const cv::Rect & bbox,
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & organized)
  {
    cv::Mat contour_mask = cv::Mat::zeros(static_cast<int>(organized->height), static_cast<int>(organized->width), CV_8UC1);
    std::vector<std::vector<cv::Point>> contours{contour};
    cv::drawContours(contour_mask, contours, 0, cv::Scalar(255), cv::FILLED);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    const int x_end = std::min(bbox.x + bbox.width, static_cast<int>(organized->width));
    const int y_end = std::min(bbox.y + bbox.height, static_cast<int>(organized->height));
    for (int y = std::max(0, bbox.y); y < y_end; ++y) {
      for (int x = std::max(0, bbox.x); x < x_end; ++x) {
        if (contour_mask.at<uint8_t>(y, x) == 0) {
          continue;
        }
        const auto & point = organized->at(x, y);
        if (!pcl::isFinite(point) || point.z < depth_min_m_ || point.z > depth_max_m_) {
          continue;
        }
        cloud->push_back(point);
      }
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

    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clusters;
    for (const auto & indices : cluster_indices) {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZRGB>());
      cluster->reserve(indices.indices.size());
      for (const int idx : indices.indices) {
        cluster->push_back((*cloud)[idx]);
      }
      clusters.push_back(cluster);
    }
    return clusters;
  }

  bool fit_cylinder(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud, Candidate & candidate)
  {
    if (static_cast<int>(cloud->size()) < cylinder_inlier_min_) {
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
      return false;
    }

    const Eigen::Vector3f axis_point(coeff->values[0], coeff->values[1], coeff->values[2]);
    Eigen::Vector3f axis(coeff->values[3], coeff->values[4], coeff->values[5]);
    if (axis.norm() < 1e-4F) {
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
      return false;
    }

    centroid /= static_cast<float>(inliers->indices.size());
    candidate.centroid_camera = centroid;
    candidate.axis = axis;
    candidate.horizontal = std::abs(axis.z()) < horizontal_dot_threshold_;
    candidate.inliers = static_cast<int>(inliers->indices.size());
    candidate.residual = residual;
    return true;
  }

  bool transform_candidate(
    const std_msgs::msg::Header & source_header,
    Candidate & candidate)
  {
    if (target_frame_ == source_header.frame_id || target_frame_.empty()) {
      candidate.centroid_map = candidate.centroid_camera;
      candidate.axis_map = candidate.axis;
      candidate.horizontal = std::abs(candidate.axis_map.z()) < horizontal_dot_threshold_;
      set_candidate_normal(candidate);
      return true;
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
      candidate.horizontal = std::abs(candidate.axis_map.z()) < horizontal_dot_threshold_;
      set_candidate_normal(candidate);
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "TF lookup failed: %s", ex.what());
      return false;
    }
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
        if (dist < best_dist && dist <= dedup_distance_m_) {
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
        [this](const BarrelTrack & track) { return track.missed_frames > track_timeout_frames_; }),
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
    return std::hypot(dx, dy) > republish_move_threshold_m_;
  }

  const BarrelTrack * matching_track_for_candidate(const Candidate & candidate) const
  {
    const BarrelTrack * best_track = nullptr;
    double best_dist = std::numeric_limits<double>::max();
    for (const auto & track : tracks_) {
      const double dx = track.x - candidate.centroid_map.x();
      const double dy = track.y - candidate.centroid_map.y();
      const double dist = std::hypot(dx, dy);
      if (dist < best_dist && dist <= dedup_distance_m_) {
        best_dist = dist;
        best_track = &track;
      }
    }
    return best_track;
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
        barrel_pub_->publish(msg);
        track.published = true;
        track.last_published_x = track.x;
        track.last_published_y = track.y;
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

  void publish_overlay(
    const cv::Mat & image,
    const std::vector<Candidate> & candidates,
    const std_msgs::msg::Header & header)
  {
    cv::Mat overlay = image.clone();
    for (const auto & candidate : candidates) {
      const auto color = draw_color(candidate.color);
      const BarrelTrack * track = matching_track_for_candidate(candidate);
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

    if (show_debug_window_) {
      cv::imshow(debug_window_name_, overlay);
      cv::waitKey(1);
    }
  }

  std::string image_topic_;
  std::string point_cloud_topic_;
  std::string target_frame_;
  std::string barrel_topic_;
  std::string marker_topic_;
  std::string debug_overlay_topic_;
  double publish_hz_{2.0};
  uint32_t sync_queue_size_{5};
  double sync_slop_s_{0.08};
  int next_track_id_{100};
  int accept_threshold_{4};
  double dedup_distance_m_{0.45};
  double republish_move_threshold_m_{0.05};
  int track_timeout_frames_{18};
  double max_barrel_height_m_{0.70};
  double depth_min_m_{0.15};
  double depth_max_m_{6.0};
  double cluster_tolerance_m_{0.07};
  int cluster_min_points_{40};
  int cluster_max_points_{20000};
  double normal_search_radius_m_{0.05};
  int ransac_max_iterations_{250};
  double ransac_distance_threshold_m_{0.035};
  double ransac_normal_distance_weight_{0.1};
  double cylinder_radius_min_m_{0.08};
  double cylinder_radius_max_m_{0.45};
  int cylinder_inlier_min_{35};
  double cylinder_fit_residual_max_m_{0.05};
  double horizontal_dot_threshold_{0.55};
  double candidate_min_area_px_{180.0};
  double candidate_max_area_px_{90000.0};
  int candidate_min_width_px_{12};
  int candidate_min_height_px_{12};
  double color_confidence_min_{0.45};
  int mask_morph_open_kernel_{3};
  int mask_morph_close_kernel_{9};
  size_t stability_window_m_{8};
  int stability_required_n_{5};
  double marker_scale_x_{0.28};
  double marker_scale_y_{0.28};
  double marker_scale_z_{0.45};
  double marker_alpha_{0.9};
  double marker_lifetime_s_{1.5};
  std::string marker_namespace_{"barrels"};
  bool enable_debug_overlay_{true};
  std::string debug_window_name_{"barrel"};
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
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BarrelDetectorCpp>());
  rclcpp::shutdown();
  return 0;
}
