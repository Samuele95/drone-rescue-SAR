# Global mission state published by mission_manager at 1 Hz.

uint8 INIT          = 0
uint8 ARMING        = 1
uint8 DEPLOYING     = 2
uint8 SCANNING      = 3
uint8 INVESTIGATING = 4
uint8 COMPLETE      = 5
uint8 ABORTED       = 6

std_msgs/Header header
uint8 status

# Coverage progress, in sectors.
uint32 sectors_total
uint32 sectors_completed

# Detection / confirmation tally.
uint32 victims_found
uint32 victims_confirmed

# Human-readable per-drone summary, e.g. "drone1: SCAN(7/24)" / "drone2: CONFIRM(v3)".
string[] active_tasks_summary
