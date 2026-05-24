"""Throttled-logging helper for high-frequency tick paths.

Per-callsite rate limit: each `key` gets its own last-emit timestamp, so many
distinct debug call sites can fire together but no single one exceeds ~1 Hz.
"""

from rclpy.impl.rcutils_logger import RcutilsLogger
from rclpy.node import Node


_last_emit: dict[str, float] = {}


def log_throttled(
    logger: RcutilsLogger,
    node: Node,
    key: str,
    level: str,
    msg: str,
    period_s: float = 1.0,
) -> None:
    """Emit `msg` at `level` only if `period_s` has passed since the last emit
    for this `key`. `key` should be unique per call site (e.g. "FollowBlueLine.driving").
    `level` is one of "debug", "info", "warn"/"warning", "error".
    """
    now = node.get_clock().now().nanoseconds * 1e-9
    last = _last_emit.get(key)
    if last is not None and (now - last) < period_s:
        return
    _last_emit[key] = now
    if level == "debug":
        logger.debug(msg)
    elif level == "info":
        logger.info(msg)
    elif level in ("warn", "warning"):
        logger.warning(msg)
    elif level == "error":
        logger.error(msg)
    else:
        logger.info(msg)
