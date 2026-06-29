# Per-drone peer-broadcast message. Every executor publishes this on
# /<drone>/peer_state at low rate so siblings + supervisors can build an
# awareness map without going through the central mission_manager.
#
# Used for collision avoidance, opportunistic handoff, and dashboard state.

std_msgs/Header header
string drone_name
geometry_msgs/Pose pose
float32 battery            # 0.0..1.0
uint8   task_type          # mirrors TaskAssignment.task_type enum
uint32  current_task_id
uint32  busy_with_victim   # 0 if not assigned to a victim
bool    is_down            # true when executor in DOWN state (damaged)
# Local progress through the dispatched waypoint list, supplies the
# dashboard's State table with "SCAN (12/85)" so operators can see
# coverage progression at a glance.
uint32  wp_index
uint32  wp_total
