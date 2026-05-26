"""Homophone repairs for speech-to-text output.

The STT engine occasionally mis-hears domain words (e.g. "rings" -> "wings",
"red" -> "read"). This module maps such variants back to the canonical word
used by the task parser.

To add a new repair: append a variant to the appropriate set in
`HOMOPHONES`, or add a new entry keyed by the canonical word.
Matching is whole-word and case-insensitive; input is expected to already
be lowercased and punctuation-stripped.
"""
from __future__ import annotations

import re


HOMOPHONES: dict[str, set[str]] = {
    "red": {"read", "raid"},
    "green": {"grin", "grain", "screen", "queen"},
    "rings": {"wings", "things", "kings", "brings", "rinks"},
    "count": {"counts", "kant", "cant", "mount", "account"},
    "barrels": {"barrel", "barrows", "parallels", "parcels", "barrows"},
    "inspect": {"expect", "suspect", "inspector", "inspected"},
    "detect": {"deject", "the tech", "detects", "detected"},
    "anomalies": {"anomaly", "anomalous", "animals", "enemies", "anomalie"},
    "cell": {"sell", "shell", "tell", "sale", "cells"},
}


def _build_pattern(variants: set[str]) -> re.Pattern[str]:
    escaped = sorted((re.escape(v) for v in variants), key=len, reverse=True)
    return re.compile(rf"\b(?:{'|'.join(escaped)})\b")


_COMPILED: list[tuple[str, re.Pattern[str]]] = [
    (canonical, _build_pattern(variants))
    for canonical, variants in HOMOPHONES.items()
    if variants
]


def repair_homophones(text: str) -> str:
    """Replace known mis-heard variants with their canonical word."""
    for canonical, pattern in _COMPILED:
        text = pattern.sub(canonical, text)
    return text
