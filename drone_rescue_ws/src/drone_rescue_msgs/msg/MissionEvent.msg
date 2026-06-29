# Operator-facing mission event. Published on /mission/events
# (RELIABLE, depth=50). The dashboard subscribes and renders a scrolling,
# color-coded log. Severity drives the color (0=info, 1=warn, 2=error).
#
# event_type values (loose enum, matched as strings for forward-compat):
#   SCANNING_STARTED            INFO
#   CANDIDATE_DETECTED          INFO
#   INVESTIGATE_DISPATCHED      INFO
#   CONFIRM_DISPATCHED          INFO
#   VICTIM_CONFIRMED            INFO
#   CANDIDATE_REJECTED          INFO
#   BATTERY_RTH                 WARN
#   DRONE_DOWN                  ERROR  (drone unrecoverable, sector reassigned)
#   DRONE_DAMAGE_REPORT         ERROR  (executor self-reported)
#   SECTOR_REASSIGNED           WARN
#   TASK_TIMEOUT                WARN
#   MISSION_COMPLETE            INFO
#   MISSION_TIMEOUT             WARN

uint8 SEVERITY_INFO = 0
uint8 SEVERITY_WARN = 1
uint8 SEVERITY_ERROR = 2

std_msgs/Header header
string event_type
string drone_name           # empty for global events
string detail               # human-readable explanation
uint32 victim_id            # 0 if N/A
geometry_msgs/Point position
uint8  severity
# Structured numeric carrier for the candidate
# confidence on CANDIDATE_DETECTED events. Avoids re-parsing it
# out of `detail` in the analytics layer (ROC sweep). 0.0 for events
# where confidence is N/A.
float32 confidence
