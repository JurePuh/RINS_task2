from __future__ import annotations

import re
import string
from dataclasses import dataclass


TASK_ANOMALY_RED = "anomaly_red"
TASK_ANOMALY_GREEN = "anomaly_green"
TASK_INSPECT_BARRELS = "inspect_barrels"
TASK_COUNT_RINGS = "count_rings"

TASK_PRIORITY = (
    TASK_ANOMALY_RED,
    TASK_ANOMALY_GREEN,
    TASK_INSPECT_BARRELS,
    TASK_COUNT_RINGS,
)

TASK_SPEECH = {
    TASK_ANOMALY_RED: "detect anomalies in the red cell",
    TASK_ANOMALY_GREEN: "detect anomalies in the green cell",
    TASK_INSPECT_BARRELS: "inspect the barrels",
    TASK_COUNT_RINGS: "count the rings",
}

TASK_CONFIRMATION_SPEECH = {
    TASK_ANOMALY_RED: "red cell",
    TASK_ANOMALY_GREEN: "green cell",
    TASK_INSPECT_BARRELS: "barrels",
    TASK_COUNT_RINGS: "rings",
}


@dataclass(frozen=True)
class TaskParseResult:
    task: str | None
    ambiguous: bool = False


_PUNCT_TRANSLATION = str.maketrans({c: " " for c in string.punctuation})
_RED_CELL_PATTERN = re.compile(
    r"\b(anomaly|anomalies)\b.*\bred\b.*\bcell\b"
    r"|\bred\b.*\bcell\b.*\b(anomaly|anomalies)\b"
)
_GREEN_CELL_PATTERN = re.compile(
    r"\b(anomaly|anomalies)\b.*\bgreen\b.*\bcell\b"
    r"|\bgreen\b.*\bcell\b.*\b(anomaly|anomalies)\b"
)
_BARREL_PATTERN = re.compile(r"\b(barrel|barrels)\b")
_RING_PATTERN = re.compile(r"\b(ring|rings)\b")


def normalize_transcript(text: str) -> str:
    text = text.lower().translate(_PUNCT_TRANSLATION)
    text = " ".join(text.split())
    text = _repair_common_stt_compactions(text)
    return " ".join(text.split())


def _repair_common_stt_compactions(text: str) -> str:
    replacements = {
        "counterings": "count the rings",
        "countering": "count the rings",
        "conjurings": "count the rings",
        "conjuring": "count the rings",
        "counttherings": "count the rings",
        "countrings": "count rings",
        "count rings": "count rings",
        "inspectbarrels": "inspect barrels",
        "inspectthebarrels": "inspect the barrels",
        "green cell": "green cell",
        "greencell": "green cell",
        "red cell": "red cell",
        "redcell": "red cell",
    }
    for src, dst in replacements.items():
        text = re.sub(rf"\b{re.escape(src)}\b", dst, text)
    return text


def parse_task(text: str) -> TaskParseResult:
    normalized = normalize_transcript(text)
    if not normalized:
        return TaskParseResult(task=None)

    matched: set[str] = set()
    if _RED_CELL_PATTERN.search(normalized):
        matched.add(TASK_ANOMALY_RED)
    if _GREEN_CELL_PATTERN.search(normalized):
        matched.add(TASK_ANOMALY_GREEN)
    if _BARREL_PATTERN.search(normalized):
        matched.add(TASK_INSPECT_BARRELS)
    if _RING_PATTERN.search(normalized):
        matched.add(TASK_COUNT_RINGS)

    if TASK_ANOMALY_RED in matched and TASK_ANOMALY_GREEN in matched:
        return TaskParseResult(task=None, ambiguous=True)

    for task in TASK_PRIORITY:
        if task in matched:
            return TaskParseResult(task=task)
    return TaskParseResult(task=None)


def is_affirmative(text: str) -> bool:
    normalized = normalize_transcript(text)
    first_token = normalized.split(maxsplit=1)[0] if normalized else ""
    return first_token == "yes"
