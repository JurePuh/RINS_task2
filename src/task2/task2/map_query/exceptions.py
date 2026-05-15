class WallQueryError(Exception):
    """Expected failure: the query cannot be answered given current map state.
    Caught by the service handler → returns success=False to the caller."""


class WallQueryInternalError(RuntimeError):
    """Programming error / violated invariant inside map_query.
    NOT caught by the service handler — propagates and crashes loudly
    so bugs are visible rather than silently swallowed."""
