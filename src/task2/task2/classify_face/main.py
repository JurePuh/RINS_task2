"""
g
ros2 run task2 classify_face
ros2 service call /classify_face msg_types/srv/ClassifyFace {}

"""
from __future__ import annotations

import threading
import time
from collections import Counter

import cv2
import numpy as np
import rclpy
import torch
from facenet_pytorch import InceptionResnetV1
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image
from ultralytics import YOLO

from cv_bridge import CvBridge, CvBridgeError

from task2.classify_face.personnel_embeddings_dict import personnel_data

from msg_types.srv import ClassifyFace


def nearest_personnel_match(embedding: np.ndarray):
    if not personnel_data:
        return None, None, None
    q = np.asarray(embedding, dtype=np.float64).ravel()
    best_key, best_role, best_gender = None, None, None
    best_d = np.inf
    for person_key, record in personnel_data.items():
        ref = np.asarray(record["embedding"], dtype=np.float64).ravel()
        if ref.shape != q.shape:
            continue
        d = float(np.linalg.norm(q - ref))
        if d < best_d:
            best_d = d
            best_key = person_key
            best_role = record["role"]
            best_gender = record["gender"]
    return best_key, best_role, best_gender


class ClassifyFaceNode(Node):
    NUM_SAMPLES = 10

    def __init__(self):
        super().__init__("classify_face")

        self.declare_parameter("image_topic", "/oakd/rgb/preview/image_raw")
        self.declare_parameter("device", "")
        self.declare_parameter("frame_timeout_sec", 2.0)
        self.declare_parameter("num_samples", self.NUM_SAMPLES)

        self._image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        self._device = self.get_parameter("device").get_parameter_value().string_value
        self._frame_timeout = self.get_parameter("frame_timeout_sec").get_parameter_value().double_value
        self._num_samples = self.get_parameter("num_samples").get_parameter_value().integer_value
        if self._num_samples < 1:
            self._num_samples = self.NUM_SAMPLES

        self._bridge = CvBridge()
        self._frame_lock = threading.Lock()
        self._last_msg: Image | None = None
        self._last_stamp_ns: int | None = None

        # Service must not call spin_once; allow image cb to run in parallel while waiting.
        self._cb_group = ReentrantCallbackGroup()

        self._model = YOLO("yolov8n.pt")
        emb_dev = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self._embed_device = emb_dev
        self._embedding_model = InceptionResnetV1(pretrained="vggface2").eval().to(emb_dev)

        self._sub = self.create_subscription(
            Image, self._image_topic, self._image_cb, 1, callback_group=self._cb_group
        )
        self._srv = self.create_service(
            ClassifyFace, "classify_face", self._handle_classify, callback_group=self._cb_group
        )

        self.get_logger().info(
            f"classify_face ready: topic={self._image_topic} samples={self._num_samples}"
        )

    def _image_cb(self, msg: Image) -> None:
        with self._frame_lock:
            self._last_msg = msg
            self._last_stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec

    def _wait_fresh_frame(self, previous_stamp_ns: int | None) -> Image | None:
        """Wait for a frame with a new stamp. Never call rclpy.spin_once (executor already spinning)."""
        t0 = time.monotonic()
        while (time.monotonic() - t0) < self._frame_timeout:
            with self._frame_lock:
                last_msg = self._last_msg
                last_stamp = self._last_stamp_ns
            if last_msg is None:
                time.sleep(0.01)
                continue
            if previous_stamp_ns is None:
                return last_msg
            if last_stamp != previous_stamp_ns:
                return last_msg
            time.sleep(0.01)
        return None

    def _embed_face(self, cropped_bgr: np.ndarray) -> np.ndarray:
        face_resized = cv2.resize(cropped_bgr, (160, 160))
        face_rgb = cv2.cvtColor(face_resized, cv2.COLOR_BGR2RGB)
        face_normalized = (face_rgb / 255.0 - 0.5) / 0.5
        t = torch.tensor(face_normalized.transpose(2, 0, 1), dtype=torch.float32).unsqueeze(0).to(
            self._embed_device
        )
        with torch.no_grad():
            embedding = self._embedding_model(t)
        return embedding.squeeze().detach().cpu().numpy()

    def _detect_and_crop(self, cv_image: np.ndarray) -> np.ndarray | None:
        res = self._model.predict(
            cv_image, imgsz=(256, 320), show=False, verbose=False, classes=[0], device=self._device
        )
        for r in res:
            bbox = r.boxes.xyxy
            if bbox is None or bbox.nelement() == 0:
                continue
            box = bbox[0]
            h, w = cv_image.shape[:2]
            x1 = max(0, int(float(box[0])))
            y1 = max(0, int(float(box[1])))
            x2 = min(w, max(x1 + 1, int(float(box[2]))))
            y2 = min(h, max(y1 + 1, int(float(box[3]))))
            return cv_image[y1:y2, x1:x2]
        return None

    def _handle_classify(self, request, response):
        del request  # empty
        if not personnel_data:
            response.success = False
            response.message = "personnel_embeddings_dict is empty"
            response.name = ""
            response.role = ""
            response.gender = ""
            return response

        votes: list[str | None] = []
        prev_stamp: int | None = None

        for i in range(self._num_samples):
            msg = self._wait_fresh_frame(prev_stamp)
            if msg is None:
                response.success = False
                response.message = (
                    f"No image or timeout waiting for frame {i + 1}/{self._num_samples} "
                    f"(check {self._image_topic})"
                )
                response.name = ""
                response.role = ""
                response.gender = ""
                return response

            try:
                cv_image = self._bridge.imgmsg_to_cv2(msg, "bgr8")
            except CvBridgeError as e:
                response.success = False
                response.message = str(e)
                response.name = ""
                response.role = ""
                response.gender = ""
                return response

            with self._frame_lock:
                prev_stamp = self._last_stamp_ns
            cropped = self._detect_and_crop(cv_image)
            if cropped is None or cropped.size == 0:
                if i == 0:
                    response.success = False
                    response.message = (
                        "YOLO did not detect a person (class 0) in the first frame — "
                        "move closer or ensure the person is in view"
                    )
                    response.name = ""
                    response.role = ""
                    response.gender = ""
                    return response
                votes.append(None)
                continue

            emb = self._embed_face(cropped)
            key, _, _ = nearest_personnel_match(emb)
            votes.append(key)

        counted = [v for v in votes if v is not None]
        if not counted:
            response.success = False
            response.message = "No person detected or no embedding match in any frame"
            response.name = ""
            response.role = ""
            response.gender = ""
            return response

        winner = Counter(counted).most_common(1)[0][0]
        rec = personnel_data[winner]
        response.success = True
        response.name = winner
        response.role = rec["role"]
        response.gender = rec["gender"]
        vote_count = sum(1 for v in votes if v == winner)
        response.message = f"{vote_count}/{self._num_samples} votes"
        self.get_logger().info(f"classify_face: name={winner} {response.message}")
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ClassifyFaceNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
