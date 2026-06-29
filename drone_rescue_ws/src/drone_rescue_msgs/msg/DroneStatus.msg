# Status message for individual drone

std_msgs/Header header

# Drone identifier
string drone_id

# Position and orientation
geometry_msgs/Pose pose

# Velocity
geometry_msgs/Twist velocity

# Battery state
float32 battery_level      # 0.0 to 1.0 (percentage)
float32 battery_drain_rate # Units per second (affected by wind)

# Operational state. Values mirror the controller-side DroneState
# enum (drone_rescue_coordination/lib/domain/drone_state.py), which
# drone_controller publishes as `status.state = self.state.value`.
# The declared constants below had drifted (EMERGENCY was 5, but the
# publisher emits HOVER=5 / NAVIGATING=6 / EMERGENCY=7); realigned
# 2026-06-10. Operator-facing labels: see
# drone_rescue_ui_common.constants.CONTROLLER_STATE_LABEL.
uint8 STATE_IDLE = 0
uint8 STATE_TAKEOFF = 1
uint8 STATE_SURVEYING = 2
uint8 STATE_RETURNING = 3
uint8 STATE_LANDING = 4
uint8 STATE_HOVER = 5
uint8 STATE_NAVIGATING = 6
uint8 STATE_EMERGENCY = 7
uint8 state

# Sensor status
bool camera_active
bool lidar_active

# Current target position (if surveying)
geometry_msgs/Point target_position

# Coverage statistics
float32 area_scanned       # Square meters scanned by this drone
uint32 cells_visited       # Number of pheromone cells visited
