import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSReliabilityPolicy

from ultralytics import YOLO

from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2

import tf2_ros
from geometry_msgs.msg import PoseStamped

import tf2_geometry_msgs

from visualization_msgs.msg import Marker
from cv_bridge import CvBridge, CvBridgeError

import cv2
import numpy as np
import torch
from facenet_pytorch import InceptionResnetV1

import message_filters
from message_filters import ApproximateTimeSynchronizer

from msg_types.msg import FaceDetect

class Face():
    def __init__(self, x, y, id=None, seen_counter=0, sum_x=0, sum_y=0):
        self.x = x
        self.y = y
        self.id = id
        self.seen_counter = seen_counter
        self.sum_x = sum_x
        self.sum_y = sum_y


class detect_faces(Node):
    def __init__(self):
        super().__init__('detect_faces')
        
        self.get_logger().info(f"VERSION: FACES")

        # model definition
        self.model = YOLO("yolov8n.pt")
        self.embedding_model = InceptionResnetV1(pretrained='vggface2').eval()

        # marker publisher
        self.marker_pub = self.create_publisher(Marker, "/people_marker2", 10)

        # face found publisher — fires once per accepted face, ever.
        self.face_pub = self.create_publisher(FaceDetect, "/face_detect", 10)

        # tf
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Create message_filters subscribers (not regular subscriptions)
        self.rgb_sub = message_filters.Subscriber(self, Image, "/oakd/rgb/preview/image_raw", qos_profile=qos_profile_sensor_data)
        self.pc_sub = message_filters.Subscriber(self, PointCloud2, "/oakd/rgb/preview/depth/points", qos_profile=qos_profile_sensor_data)

        # synchronize topics with 50ms error allowed
        self.ts = ApproximateTimeSynchronizer([self.rgb_sub, self.pc_sub],queue_size=5,slop=0.05)
        self.ts.registerCallback(self.synced_callback)
        
        
        self.bridge = CvBridge()
        self.detection_color = (0,0,255)

        self.declare_parameters(
			namespace='',
			parameters=[
				('device', ''),
		])
        self.device = self.get_parameter('device').get_parameter_value().string_value


        # face logic
        # potential_faces: detections still accumulating sightings toward the
        #     accept threshold.
        # accepted_face_keys: quantised (x, y) cells of faces already published.
        #     Used to silently drop further sightings of the same face so they
        #     don't keep spawning new entries in potential_faces.
        self.potential_faces = []
        self.accepted_face_keys: set[tuple[int, int]] = set()
        self.id_counter = 0
        self.accept_threshold = 10

    @staticmethod
    def _accept_key(x: float, y: float) -> tuple[int, int]:
        # 0.5 m bins — matches the 0.5 m tolerance used by `seen()`.
        return (round(x * 2), round(y * 2))


    def build_marker(self, pc_msg, map_pose, face_id):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = pc_msg.header.stamp
        marker.ns = "faces"
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.id = face_id
        marker.pose.position.x = map_pose.pose.position.x
        marker.pose.position.y = map_pose.pose.position.y
        marker.pose.position.z = map_pose.pose.position.z
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2

        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0

        return marker


    def seen(self, new_face):
        """ checking if the face was already seen

        arg: new_face is tuple (x, y)
        """

        x, y = new_face[0], new_face[1]
        for face in self.potential_faces:
            if abs(face.x - x) < 0.5 and abs(face.y - y) < 0.5:
                return True, face

        return False, None


    def embed_face(self, cropped_face):
        face_resized = cv2.resize(cropped_face, (160, 160))
        face_rgb = cv2.cvtColor(face_resized, cv2.COLOR_BGR2RGB)
        face_normalized = (face_rgb / 255.0 - 0.5) / 0.5
        tensor = torch.tensor(face_normalized.transpose(2, 0, 1), dtype=torch.float32).unsqueeze(0)

        with torch.no_grad():
            embedding = self.embedding_model(tensor)
    
        return embedding.squeeze().numpy()  # 128-d vector

    def synced_callback(self, rgb_msg, pc_msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
        except CvBridgeError as e:
            print(e)
            return

        res = self.model.predict(cv_image, imgsz=(256, 320), show=False, verbose=False, classes=[0], device=self.device)

        # parsing the point cloud
        height = pc_msg.height
        width = pc_msg.width
        a = pc2.read_points_numpy(pc_msg, field_names=("x", "y", "z"))
        a = a.reshape((height, width, 3))

        for x in res:
            bbox = x.boxes.xyxy
            if bbox.nelement() == 0:
                continue

            bbox = bbox[0]

            # crop face
            h, w = cv_image.shape[:2]
            x1 = max(0, int(float(bbox[0])))
            y1 = max(0, int(float(bbox[1])))
            x2 = min(w, max(x1 + 1, int(float(bbox[2]))))
            y2 = min(h, max(y1 + 1, int(float(bbox[3]))))
            cropped_face = cv_image[y1:y2, x1:x2]

            cv_image = cv2.rectangle(cv_image, (x1, y1), (x2, y2), self.detection_color, 3)
            cx = int((bbox[0] + bbox[2]) / 2)
            cy = int((bbox[1] + bbox[3]) / 2)
            cv_image = cv2.circle(cv_image, (cx, cy), 5, self.detection_color, -1)

            # depth extraction
            d = a[cy, cx, :]

            if np.isnan(d[0]):
                self._logger.warn("Depth is NaN at face center, skipping")
                continue


            # face classifier
            # face_embedding = self.embed_face(cropped_face)
            # self._logger.info(f"face_embedding")



            # Build a pose
            pose = PoseStamped()
            pose.header = pc_msg.header 
            pose.pose.position.x = float(d[0])
            pose.pose.position.y = float(d[1])
            pose.pose.position.z = float(d[2])
            pose.pose.orientation.w = 1.0

            try:
                trans = self.tf_buffer.lookup_transform("map", pose.header.frame_id, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.5))
                map_pose = tf2_geometry_msgs.do_transform_pose_stamped(pose, trans)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                self._logger.warn(f"TF transform failed: {e}")
                continue


            face_x = map_pose.pose.position.x
            face_y = map_pose.pose.position.y
            face_z = map_pose.pose.position.z
            new_face = (face_x, face_y)

        

            # Drop further sightings of an already-accepted face. Mission
            # logic (visited/handled) lives in the movement node now; the
            # detector just fires once when a face crosses the threshold.
            if self._accept_key(face_x, face_y) in self.accepted_face_keys:
                continue

            was_seen, face = self.seen(new_face)
            if not was_seen:
                face = Face(x=face_x, y=face_y, seen_counter=1, sum_x=face_x, sum_y=face_y)
                self.potential_faces.append(face)
                self._logger.info(f"NEW face x={face_x}, y={face_y}")
            else:
                face.sum_x += face_x
                face.sum_y += face_y
                face.seen_counter += 1
                face.x = face.sum_x / face.seen_counter
                face.y = face.sum_y / face.seen_counter

                if face.seen_counter > self.accept_threshold:
                    face.id = self.id_counter
                    self.id_counter += 1

                    marker = self.build_marker(pc_msg, map_pose, face.id)
                    self.marker_pub.publish(marker)

                    msg = FaceDetect()
                    msg.id = face.id
                    msg.x = face.x
                    msg.y = face.y
                    self.face_pub.publish(msg)
                    self._logger.info(
                        f"Accepted face id={face.id} at x={face.x:.2f}, y={face.y:.2f}"
                    )

                    self.accepted_face_keys.add(self._accept_key(face.x, face.y))
                    self.potential_faces.remove(face)
                        

    
            #self._logger.info(f"person detected at x={map_pose.pose.position.x}, y={map_pose.pose.position.y}")

        cv2.imshow("image", cv_image)
        key = cv2.waitKey(1)
        if key == 27:
            print("exiting")
            exit()


def main():
	print('Face detection node starting.')

	rclpy.init(args=None)
	node = detect_faces()
	rclpy.spin(node)

if __name__ == '__main__':
	main()
