"""Blackboard key names.

The blackboard is py_trees' shared key/value store. Using constants here
instead of bare strings means a typo becomes an import error, not a silent
"why isn't my behaviour seeing the data" bug.

py_trees prefixes keys with '/' to namespace them; we follow that convention.
"""

# Queue of FaceDetect messages waiting to be visited.
# Written by: the rclpy /face_detect subscription callback (see tree.py).
# Read by:    HasUnhandledFace, ComputeFaceDestination.
# Popped by:  MarkFaceHandled (after a successful GoToFace run).
PENDING_FACES = "/pending_faces"

# IDs of faces we have already finished visiting. Used by the ingest callback
# to drop duplicates before they reach the queue.
HANDLED_FACES = "/handled_faces"

# A NavigateToPose.Goal computed by ComputeFaceDestination, consumed by the
# FromBlackboard nav2 action client child of GoToFace.
FACE_DESTINATION = "/face_destination"
