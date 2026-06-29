# Task issued by mission_manager to a single drone executor.
# The drone's executor enters/preempts a behavior-tree branch based on task_type.

uint8 SCAN_WAYPOINTS = 0
uint8 INVESTIGATE    = 1
uint8 CONFIRM        = 2
uint8 RTH            = 3
uint8 LAND           = 4
uint8 IDLE           = 5

std_msgs/Header header
string drone_name
uint32 task_id
uint8  task_type

# For SCAN_WAYPOINTS: ordered list of (x, y, z) waypoints
geometry_msgs/Point[] waypoints

# For INVESTIGATE / CONFIRM: a single point to fly to / orbit
geometry_msgs/Point target_point

# Cross-link to the candidate that triggered this task; 0 if N/A
uint32 victim_id

# 0 = lowest, 3 = highest. Used by the executor to decide preemption.
uint8 priority

# For INVESTIGATE / CONFIRM: how long to stay at target_point.
float32 hover_seconds

# For CONFIRM: radius of the orbit around target_point. 0.0 means the
# executor should pick its own default (legacy behaviour). Plumbed from
# mission_manager's confirm_orbit_radius parameter so a scenario YAML
# can vary it per run.
float32 confirm_orbit_radius

# For INVESTIGATE: the multi-view orbit the deliberative layer (L3) commits
# for this candidate. The executor visits each angle in turn, dwelling
# dwell_s at each viewpoint at investigate_radius around target_point.
# Empty / 0.0 means the executor falls back to its own investigate_radius_m
# / investigate_dwell_s params + the 4 cardinal angles (legacy default;
# keeps old bags replay-safe). (F2-C, 2026-05-22)
float32 investigate_radius
float32 dwell_s
float32[] investigate_angles
