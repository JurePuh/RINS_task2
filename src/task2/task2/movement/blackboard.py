"""Blackboard key names.
py_trees prefixes keys with '/' to namespace them; we follow that convention.
"""

# --- Faces --------------------------------------------------------
PENDING_FACES = "/pending_faces" # deque[Face] of newly-seen faces that still need talking to.
HANDLED_FACES = "/handled_faces" # set[Face] of faces we've already talked to.
FACE_DESTINATION = "/face_destination" # Point we need to go to in order to talk to the next face.
RECOMPUTE_DESTINATION = "/recompute_destination" # Flag that updated cordinates of current destination arrived.

ACTIVE_PERSON = "/active_person" # Person currently being talked to
CONVERSATION_RESULT = "/conversation_result" # Task the person requested. Values: "count_rings" | "inspect_barrels" | "anomaly_red" | "anomaly_green" | ""

# --- Task singletons ---------------------------------------------------------
# Objects where task data is stored. One per task type.
TASK_COUNT_RINGS = "/task_count_rings"
TASK_INSPECT_BARRELS = "/task_inspect_barrels"
TASK_ANOMALY_RED = "/task_anomaly_red"
TASK_ANOMALY_GREEN = "/task_anomaly_green"

# --- Barrels -----------------------------------------------------------------
PENDING_BARRELS = "/pending_barrels" # deque[Barrel] of newly-seen horizontal barrels that still need visiting.
ACTIVE_BARREL = "/active_barrel" # Barrel currently being visited
BARREL_DESTINATION = "/barrel_destination" # Point we need to go to in order to visit the next barrel
RECOMPUTE_BARREL_DESTINATION = "/recompute_barrel_destination" # Flag that updated cordinates of current barrel destination arrived.

# --- Anomaly -----------------------------------------------------------------
ACTIVE_ANOMALY_RED = "/active_anomaly_red"     # True while the red anomaly sequence is running
ACTIVE_ANOMALY_GREEN = "/active_anomaly_green" # True while the green anomaly sequence is running
ANOMALY_DESTINATION = "/anomaly_destination" # Point we need to go to in order to inspect the next anomaly

# --- Mission control ---------------------------------------------------------
EXPLORATION_DONE = "/exploration_done" # Flag to note when we are done with the first room
