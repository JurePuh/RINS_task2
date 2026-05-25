from __future__ import annotations

import os
import queue
import threading
import time
from collections.abc import Callable


class SonioxUnavailableError(RuntimeError):
    pass


class SonioxMicClient:
    def __init__(
        self,
        log: Callable[[str], None] | None = None,
        warn: Callable[[str], None] | None = None,
        sample_rate: int = 16000,
        block_size: int = 1600,
        reconnect_delay_sec: float = 2.0,
    ) -> None:
        self._log = log or (lambda _msg: None)
        self._warn = warn or (lambda _msg: None)
        self._sample_rate = sample_rate
        self._block_size = block_size
        self._reconnect_delay_sec = reconnect_delay_sec
        self._audio_queue: queue.Queue[bytes] = queue.Queue(maxsize=100)
        self._utterance_queue: queue.Queue[str] = queue.Queue()
        self._stop_event = threading.Event()
        self._listen_enabled = threading.Event()
        self._connected = threading.Event()
        self._thread: threading.Thread | None = None

    @property
    def connected(self) -> bool:
        return self._connected.is_set()

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        if not os.getenv("SONIOX_API_KEY"):
            raise SonioxUnavailableError("SONIOX_API_KEY is not set")
        self._ensure_runtime_dependencies()
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run_forever, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)

    def set_listen_enabled(self, enabled: bool) -> None:
        if enabled:
            self._listen_enabled.set()
        else:
            self._listen_enabled.clear()
            self.clear_utterances()

    def clear_utterances(self) -> None:
        while True:
            try:
                self._utterance_queue.get_nowait()
            except queue.Empty:
                return

    def wait_until_connected(self) -> None:
        while not self._stop_event.is_set():
            if self._connected.wait(timeout=0.25):
                return
            self._warn("waiting for Soniox/microphone connection")

    def get_utterance(self) -> str:
        while not self._stop_event.is_set():
            self.wait_until_connected()
            try:
                utterance = self._utterance_queue.get(timeout=0.25)
            except queue.Empty:
                continue
            if utterance:
                return utterance
        raise SonioxUnavailableError("Soniox microphone client was stopped")

    def _run_forever(self) -> None:
        while not self._stop_event.is_set():
            try:
                self._run_session()
            except Exception as exc:  # noqa: BLE001 - runtime client must reconnect on any drop.
                self._connected.clear()
                self._warn(f"Soniox/microphone disconnected: {exc}")
                time.sleep(self._reconnect_delay_sec)

    def _run_session(self) -> None:
        import sounddevice as sd
        from soniox import SonioxClient

        client = SonioxClient()
        config = self._make_config()

        with client.realtime.stt.connect(config=config) as session:
            self._connected.set()
            self._log("Soniox realtime STT connected")
            with sd.RawInputStream(
                samplerate=self._sample_rate,
                blocksize=self._block_size,
                dtype="int16",
                channels=1,
                callback=self._audio_callback,
            ):
                audio_thread = threading.Thread(
                    target=self._send_audio_loop,
                    args=(session,),
                    daemon=True,
                )
                audio_thread.start()
                self._receive_events(session)

    def _audio_callback(self, indata, frames, time_info, status) -> None:
        del frames, time_info
        if status:
            self._warn(f"sounddevice status: {status}")
        if not self._listen_enabled.is_set():
            return
        try:
            self._audio_queue.put_nowait(bytes(indata))
        except queue.Full:
            self._warn("audio queue full, dropping microphone chunk")

    def _send_audio_loop(self, session) -> None:
        while not self._stop_event.is_set():
            try:
                chunk = self._audio_queue.get(timeout=0.2)
            except queue.Empty:
                continue
            if not self._listen_enabled.is_set():
                continue
            session.send_byte_chunk(chunk)

    def _receive_events(self, session) -> None:
        final_tokens: list[str] = []
        last_final_at = 0.0
        for event in session.receive_events():
            if self._stop_event.is_set():
                return
            tokens = getattr(event, "tokens", None) or []
            for token in tokens:
                text = getattr(token, "text", "")
                if not text or text.startswith("<"):
                    continue
                if getattr(token, "is_final", False):
                    final_tokens.append(text)
                    last_final_at = time.monotonic()

            is_endpoint = bool(
                getattr(event, "is_endpoint", False)
                or getattr(event, "endpoint", False)
                or getattr(event, "speech_final", False)
            )
            if final_tokens and (is_endpoint or time.monotonic() - last_final_at > 0.9):
                utterance = "".join(final_tokens).strip()
                final_tokens.clear()
                if self._listen_enabled.is_set() and utterance:
                    self._utterance_queue.put(utterance)

    def _make_config(self):
        from soniox.types import RealtimeSTTConfig

        configs = (
            {
                "model": "stt-rt-v4",
                "audio_format": "pcm_s16le",
                "sample_rate_hertz": self._sample_rate,
                "num_channels": 1,
                "enable_endpoint_detection": True,
                "language_hints": ["en"],
            },
            {
                "model": "stt-rt-v4",
                "audio_format": "pcm_s16le",
                "sample_rate": self._sample_rate,
                "num_channels": 1,
                "enable_endpoint_detection": True,
                "language_hints": ["en"],
            },
            {
                "model": "stt-rt-v4",
                "audio_format": "pcm_s16le",
                "enable_endpoint_detection": True,
                "language_hints": ["en"],
            },
        )
        last_error: Exception | None = None
        for kwargs in configs:
            try:
                return RealtimeSTTConfig(**kwargs)
            except Exception as exc:  # noqa: BLE001 - SDK versions validate config differently.
                last_error = exc
        raise SonioxUnavailableError(f"could not build Soniox config: {last_error}")

    @staticmethod
    def _ensure_runtime_dependencies() -> None:
        missing = []
        try:
            import soniox  # noqa: F401
        except ImportError:
            missing.append("soniox")
        try:
            import sounddevice  # noqa: F401
        except ImportError:
            missing.append("sounddevice")
        if missing:
            packages = " ".join(missing)
            raise SonioxUnavailableError(f"missing runtime package(s): {packages}")
