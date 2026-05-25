import rclpy
import rclpy.duration
import rclpy.time
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from ultralytics import YOLO

from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2

import tf2_ros
from geometry_msgs.msg import PoseStamped
import tf2_geometry_msgs

from cv_bridge import CvBridge, CvBridgeError

import cv2
import math
import numpy as np

import message_filters
from message_filters import ApproximateTimeSynchronizer

from msg_types.msg import FaceDetect


# Accumulator for sightings of the same face until it crosses the accept threshold.
class Face:
    def __init__(
        self,
        x: float,
        y: float,
        id: int | None = None,
        seen_counter: int = 0,
        sum_x: float = 0.0,
        sum_y: float = 0.0,
    ) -> None:
        self.x: float = x
        self.y: float = y
        self.id: int | None = id
        self.seen_counter: int = seen_counter
        self.sum_x: float = sum_x
        self.sum_y: float = sum_y
        self.last_pub_x: float = 0.0
        self.last_pub_y: float = 0.0


class detect_faces(Node):
    def __init__(self) -> None:
        super().__init__('detect_faces')

        # models
        self.model = YOLO("yolov8n.pt")

        # publishers
        # fires once per accepted face, ever
        self.face_pub = self.create_publisher(FaceDetect, "/face_detect", 10)

        # tf
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # synchronized rgb + depth-points
        self.rgb_sub = message_filters.Subscriber(
            self, Image, "/oakd/rgb/preview/image_raw", qos_profile=qos_profile_sensor_data
        )
        self.pc_sub = message_filters.Subscriber(
            self, PointCloud2, "/oakd/rgb/preview/depth/points", qos_profile=qos_profile_sensor_data
        )
        self.ts = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.pc_sub], queue_size=5, slop=0.05
        )
        self.ts.registerCallback(self.synced_callback)

        self.bridge = CvBridge()
        self.detection_color = (0, 0, 255)

        self.declare_parameters(namespace='', parameters=[('device', '')]) # type: ignore
        self.device: str = self.get_parameter('device').get_parameter_value().string_value

        # face logic
        # potential_faces: detections still accumulating sightings toward the
        #     accept threshold.
        # accepted_faces: faces already published; further sightings update
        #     their running mean and re-publish on /face_detect when the mean
        #     drifts past `republish_threshold`.
        self.potential_faces: list[Face] = []
        self.accepted_faces: list[Face] = []
        self.id_counter: int = 0
        self.accept_threshold: int = 20
        self.republish_threshold: float = 0.05

        self._logger.info(
            f"detect_faces started (device='{self.device}', accept_threshold={self.accept_threshold})"
        )

    def seen(self, x: float, y: float) -> Face | None:
        """Return the matching potential face within 0.5 m, or None."""
        for face in self.potential_faces:
            if abs(face.x - x) < 0.5 and abs(face.y - y) < 0.5:
                return face
        return None

    def _find_accepted(self, x: float, y: float) -> Face | None:
        """Return the matching already-accepted face within 0.5 m, or None."""
        for face in self.accepted_faces:
            if abs(face.x - x) < 0.5 and abs(face.y - y) < 0.5:
                return face
        return None

    # Publishes FaceDetect, marks key seen, removes from potential list.
    def _accept_face(self, face: Face, pc_msg: PointCloud2, map_pose: PoseStamped) -> None:
        face.id = self.id_counter
        self.id_counter += 1

        msg = FaceDetect()
        msg.id = face.id
        msg.x = face.x
        msg.y = face.y
        self.face_pub.publish(msg)

        face.last_pub_x = face.x
        face.last_pub_y = face.y
        self.accepted_faces.append(face)
        self.potential_faces.remove(face)

        self._logger.info(
            f"ACCEPTED face id={face.id} at x={face.x:.2f}, y={face.y:.2f} "
            f"(total accepted={len(self.accepted_faces)})"
        )

    def _republish_face(self, face: Face, pc_msg: PointCloud2, map_pose: PoseStamped) -> None:
        msg = FaceDetect()
        msg.id = face.id  # type: ignore[assignment]
        msg.x = face.x
        msg.y = face.y
        self.face_pub.publish(msg)

        self._logger.info(
            f"REPUBLISHED face id={face.id} at x={face.x:.2f}, y={face.y:.2f} "
            f"(shift from last={math.hypot(face.x - face.last_pub_x, face.y - face.last_pub_y):.3f} m)"
        )

        face.last_pub_x = face.x
        face.last_pub_y = face.y

    def synced_callback(self, rgb_msg: Image, pc_msg: PointCloud2) -> None:
        try:
            cv_image = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
        except CvBridgeError as e:
            self._logger.warn(f"CvBridge conversion failed: {e}")
            return

        res = self.model.predict(
            cv_image, imgsz=(256, 320), show=False, verbose=False,
            classes=[0], device=self.device,
        )

        # reshape point cloud to (H, W, 3) so we can index by pixel
        height = pc_msg.height
        width = pc_msg.width
        pc_xyz = pc2.read_points_numpy(pc_msg, field_names=("x", "y", "z")) # type: ignore
        pc_xyz = pc_xyz.reshape((height, width, 3))

        boxes = []
        for x in res:
            if x.boxes.xyxy.nelement() == 0: # type: ignore
                continue
            for b in x.boxes.xyxy: # type: ignore
                boxes.append(b)

        for bbox in boxes:
            # bbox -> pixel rect (clamped to image)
            h, w = cv_image.shape[:2]
            x1 = max(0, int(float(bbox[0])))
            y1 = max(0, int(float(bbox[1])))
            x2 = min(w, max(x1 + 1, int(float(bbox[2]))))
            y2 = min(h, max(y1 + 1, int(float(bbox[3]))))

            cv_image = cv2.rectangle(cv_image, (x1, y1), (x2, y2), self.detection_color, 3)
            cx = int((float(bbox[0]) + float(bbox[2])) / 2)
            cy = int((float(bbox[1]) + float(bbox[3])) / 2)
            cv_image = cv2.circle(cv_image, (cx, cy), 5, self.detection_color, -1)

            d = pc_xyz[cy, cx, :]
            if np.isnan(d[0]):
                self._logger.warn("Depth is NaN at face center, skipping")
                continue

            # build a pose in the camera frame, then transform to map
            pose = PoseStamped()
            pose.header = pc_msg.header
            pose.pose.position.x = float(d[0])
            pose.pose.position.y = float(d[1])
            pose.pose.position.z = float(d[2])
            pose.pose.orientation.w = 1.0

            try:
                trans = self.tf_buffer.lookup_transform(
                    "map", pose.header.frame_id, rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.5),
                )
                map_pose = tf2_geometry_msgs.do_transform_pose_stamped(pose, trans)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:  # type: ignore
                self._logger.warn(f"TF transform failed: {e}")
                continue

            face_x = map_pose.pose.position.x
            face_y = map_pose.pose.position.y

            # If this matches an already-accepted face, update its running
            # mean and republish on /face_detect when the mean drifts past
            # `republish_threshold`. The movement node uses the id to update
            # the pending person's location in place.
            accepted = self._find_accepted(face_x, face_y)
            if accepted is not None:
                accepted.sum_x += face_x
                accepted.sum_y += face_y
                accepted.seen_counter += 1
                accepted.x = accepted.sum_x / accepted.seen_counter
                accepted.y = accepted.sum_y / accepted.seen_counter
                shift = math.hypot(
                    accepted.x - accepted.last_pub_x,
                    accepted.y - accepted.last_pub_y,
                )
                if shift > self.republish_threshold:
                    self._republish_face(accepted, pc_msg, map_pose)
                continue

            face = self.seen(face_x, face_y)
            if face is None:
                face = Face(x=face_x, y=face_y, seen_counter=1, sum_x=face_x, sum_y=face_y)
                self.potential_faces.append(face)
                self._logger.info(
                    f"NEW potential face at x={face_x:.2f}, y={face_y:.2f} "
                    f"(potential_count={len(self.potential_faces)})"
                )
                continue

            # update running mean
            face.sum_x += face_x
            face.sum_y += face_y
            face.seen_counter += 1
            face.x = face.sum_x / face.seen_counter
            face.y = face.sum_y / face.seen_counter
            self._logger.debug(
                f"updated potential face x={face.x:.2f}, y={face.y:.2f}, "
                f"seen={face.seen_counter}/{self.accept_threshold}"
            )

            if face.seen_counter > self.accept_threshold:
                self._logger.debug(
                    f"crossed threshold at x={face.x:.2f}, y={face.y:.2f}"
                )
                self._accept_face(face, pc_msg, map_pose)

        cv2.imshow("image", cv_image)
        key_pressed = cv2.waitKey(1)
        if key_pressed == 27:
            self._logger.info("ESC pressed, exiting")
            exit()


def main() -> None:
    rclpy.init(args=None)
    node = detect_faces()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
