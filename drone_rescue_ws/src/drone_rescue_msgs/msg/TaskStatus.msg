# Status feedback from a drone_executor back to mission_manager.

uint8 ACCEPTED    = 0
uint8 IN_PROGRESS = 1
uint8 COMPLETED   = 2
uint8 FAILED      = 3
uint8 PREEMPTED   = 4

std_msgs/Header header
string drone_name
uint32 task_id
uint8  status
string detail
