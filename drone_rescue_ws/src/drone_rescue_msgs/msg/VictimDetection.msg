# Victim detection message
# Published when a drone detects a potential victim

std_msgs/Header header

# Detection metadata
string drone_name                    # Drone that detected the victim
uint32 victim_id                     # Unique victim identifier

# Location
geometry_msgs/Point position         # World coordinates of victim
float32 confidence                   # Detection confidence (0.0 - 1.0)

# Detection type constants
uint8 DETECTION_VISUAL=0
uint8 DETECTION_THERMAL=1
uint8 DETECTION_ARUCO=2

# Detection information
uint8 detection_type                 # Type of detection method used
int32 aruco_id                       # ArUco marker ID (-1 if not ArUco detection)

# Status
bool confirmed                       # Whether detection has been confirmed by multiple drones
uint8 priority                       # Priority level (0=low, 1=medium, 2=high, 3=critical)
