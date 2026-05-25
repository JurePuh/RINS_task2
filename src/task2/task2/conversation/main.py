from __future__ import annotations

import threading
import time
from collections import deque
from dataclasses import dataclass, field

import rclpy
from msg_types.srv import ConversePerson
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from task2.conversation.dialogue_logic import DialogueSession
from task2.conversation.soniox_client import SonioxMicClient, SonioxUnavailableError


@dataclass
class PendingConversation:
    gender: str
    done: threading.Event = field(default_factory=threading.Event)
    task: str = ""
    error: str = ""


class ConversationNode(Node):
    def __init__(self) -> None:
        super().__init__("conversation")

        self.declare_parameter("speak_topic", "/speak")
        self.declare_parameter("service_name", "converse_person")
        self.declare_parameter("tts_words_per_minute", 100.0)
        self.declare_parameter("tts_guard_sec", 0.5)

        self._speak_topic = self.get_parameter("speak_topic").get_parameter_value().string_value
        self._service_name = self.get_parameter("service_name").get_parameter_value().string_value
        self._tts_wpm = self.get_parameter("tts_words_per_minute").get_parameter_value().double_value
        self._tts_guard_sec = self.get_parameter("tts_guard_sec").get_parameter_value().double_value

        self._cb_group = ReentrantCallbackGroup()
        self._speak_pub = self.create_publisher(String, self._speak_topic, 10)
        self._srv = self.create_service(
            ConversePerson,
            self._service_name,
            self._handle_conversation,
            callback_group=self._cb_group,
        )

        self._queue: deque[PendingConversation] = deque()
        self._queue_condition = threading.Condition()
        self._shutdown = False
        self._stt = SonioxMicClient(
            log=lambda msg: self.get_logger().info(msg),
            warn=lambda msg: self.get_logger().warn(msg),
        )
        self._worker = threading.Thread(target=self._worker_loop, daemon=True)
        self._worker.start()

        self.get_logger().info(
            f"conversation ready: service=/{self._service_name} speak_topic={self._speak_topic}"
        )

    def destroy_node(self) -> bool:
        self._shutdown = True
        with self._queue_condition:
            self._queue_condition.notify_all()
        self._stt.stop()
        if self._worker.is_alive():
            self._worker.join(timeout=2.0)
        return super().destroy_node()

    def _handle_conversation(self, request, response):
        gender = self._normalize_gender(request.gender)
        if gender is None:
            self.get_logger().error(f"invalid gender for conversation: {request.gender!r}")
            response.task = ""
            return response

        pending = PendingConversation(gender=gender)
        with self._queue_condition:
            self._queue.append(pending)
            position = len(self._queue)
            self._queue_condition.notify()

        self.get_logger().info(f"queued conversation gender={gender} position={position}")
        pending.done.wait()
        if pending.error:
            self.get_logger().error(pending.error)
            response.task = ""
        else:
            response.task = pending.task
        return response

    def _worker_loop(self) -> None:
        while not self._shutdown:
            pending = self._next_pending()
            if pending is None:
                continue
            try:
                self._stt.start()
                self._stt.wait_until_connected()
                self.get_logger().info(f"starting conversation gender={pending.gender}")
                session = DialogueSession(
                    pending.gender,
                    speak=self._speak,
                    listen=self._listen,
                    log=lambda msg: self.get_logger().info(msg),
                )
                pending.task = session.run()
                self.get_logger().info(f"conversation finished task={pending.task}")
            except SonioxUnavailableError as exc:
                pending.error = f"conversation unavailable: {exc}"
            except Exception as exc:  # noqa: BLE001 - service response needs a logged failure.
                pending.error = f"conversation failed: {exc}"
            finally:
                self._stt.set_listen_enabled(False)
                pending.done.set()

    def _next_pending(self) -> PendingConversation | None:
        with self._queue_condition:
            while not self._queue and not self._shutdown:
                self._queue_condition.wait(timeout=0.2)
            if self._shutdown:
                return None
            return self._queue.popleft()

    def _speak(self, text: str) -> None:
        self._stt.set_listen_enabled(False)
        msg = String()
        msg.data = text
        self._speak_pub.publish(msg)
        self._sleep_for_tts(text)
        self._stt.clear_utterances()
        self._stt.set_listen_enabled(True)

    def _listen(self) -> str:
        self._stt.set_listen_enabled(True)
        utterance = self._stt.get_utterance()
        self.get_logger().info(f"heard: {utterance}")
        return utterance

    def _sleep_for_tts(self, text: str) -> None:
        word_count = max(1, len(text.split()))
        seconds_per_word = 60.0 / max(1.0, self._tts_wpm)
        duration = word_count * seconds_per_word + self._tts_guard_sec
        end_time = time.monotonic() + duration
        while time.monotonic() < end_time and rclpy.ok():
            time.sleep(min(0.1, max(0.0, end_time - time.monotonic())))

    @staticmethod
    def _normalize_gender(value: str) -> str | None:
        gender = (value or "").strip().lower()
        if gender in ("male", "female"):
            return gender
        return None


def main() -> None:
    rclpy.init(args=None)
    node = ConversationNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
