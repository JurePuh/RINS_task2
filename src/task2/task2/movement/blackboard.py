"""Blackboard key names.
py_trees prefixes keys with '/' to namespace them; we follow that convention.
"""

# --- Faces --------------------------------------------------------
PENDING_PEOPLE = "/pending_people" # deque[Person] of newly-seen people that still need talking to.
HANDLED_PEOPLE = "/handled_people" # set[Person] of people we've already talked to.
FACE_DESTINATION = "/face_destination" # Goal we need to go to in order to talk to the next face
RECOMPUTE_FACE_DESTINATION = "/recompute_face_destination" # Flag that updated cordinates of current destination arrived.

CONVERSATION_RESULT = "/conversation_result" # Task the person requested. Values: "count_rings" | "inspect_barrels" | "anomaly_red" | "anomaly_green" | ""

# --- Task singletons ---------------------------------------------------------
# Objects where task data is stored. One per task type.
TASK_COUNT_RINGS = "/task_count_rings"
TASK_INSPECT_BARRELS = "/task_inspect_barrels"
TASK_ANOMALY_RED = "/task_anomaly_red"
TASK_ANOMALY_GREEN = "/task_anomaly_green"

# --- Rings -----------------------------------------------------------------
RING_ACTIVE = "/ring_active" # True when ring count should be in report

# --- Barrels -----------------------------------------------------------------
BARREL_ACTIVE = "/barrel_active" # True while we should visit horizontal barrels.
PENDING_BARRELS = "/pending_barrels" # deque[Barrel] of newly-seen horizontal barrels that still need visiting.
HANDLED_BARRELS = "/handled_barrels" # set[Barrel] of horizontal barrels we've already visited.
BARREL_DESTINATION = "/barrel_destination" # Goal we need to go to in order to visit the next barrel
RECOMPUTE_BARREL_DESTINATION = "/recompute_barrel_destination" # Flag that updated cordinates of current barrel destination arrived.

# --- Anomaly -----------------------------------------------------------------
ANOMALY_RED_ACTIVE = "/anomaly_red_active"     # True while the red anomaly sequence is running
ANOMALY_GREEN_ACTIVE = "/anomaly_green_active" # True while the green anomaly sequence is running
ANOMALY_DESTINATION = "/anomaly_destination" # Goal we need to go to in order to inspect the next anomaly
