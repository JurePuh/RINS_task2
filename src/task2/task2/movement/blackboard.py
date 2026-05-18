"""Blackboard key names.
py_trees prefixes keys with '/' to namespace them; we follow that convention.
"""

# Queue of FaceDetect messages waiting to be visited.
PENDING_FACES = "/pending_faces"

# IDs of faces we have already finished visiting. Used by the ingest callback
# to drop duplicates before they reach the queue.
HANDLED_FACES = "/handled_faces"

# A NavigateToPose.Goal computed by ComputeFaceDestination, consumed by the
# nav2 action client child of GoToFace.
FACE_DESTINATION = "/face_destination"

# Set by the /face_detect subscription when it overwrites an in-flight face's
# coordinates. NavigateToFaceDestination watches this and bails (FAILURE) so
# the GoToFace sequence restarts and ComputeFaceDestination re-runs with the
# fresh coordinates.
RECOMPUTE_DESTINATION = "/recompute_destination"
