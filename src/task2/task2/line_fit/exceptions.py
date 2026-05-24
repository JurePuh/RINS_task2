class LineFitError(Exception):
    """Expected failure: the query cannot be answered with the current data.
    Caught by the service handler → returns success=False to the caller."""


class LineFitInternalError(RuntimeError):
    """Programming error / violated invariant inside line_fit.
    NOT caught by the service handler — propagates and crashes loudly
    so bugs are visible rather than silently swallowed."""
