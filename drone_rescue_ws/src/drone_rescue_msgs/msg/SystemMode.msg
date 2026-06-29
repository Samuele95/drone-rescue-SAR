std_msgs/Header header

# System operational mode
uint8 MODE_NORMAL = 0
uint8 MODE_DEGRADED = 1
uint8 MODE_SAFE = 2
uint8 mode

# Reason for last mode transition (for logging/debugging)
string transition_reason

# Time spent in current mode (seconds)
float32 time_in_mode
