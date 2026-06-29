# Coverage metrics for multi-drone survey mission

std_msgs/Header header

# Coverage progress
float32 percentage_covered      # 0-100%
uint32 cells_visited            # Number of cells with pheromone > threshold
uint32 total_cells              # Total cells in survey area

# Timing
float32 elapsed_time_seconds    # Time since survey started
float32 estimated_time_remaining # Estimated seconds to completion

# Victim detection
uint32 victims_found            # Number of victims detected
geometry_msgs/Point[] victim_locations  # Detected victim positions

# Fleet status
string[] active_drones          # List of currently surveying drones
uint8 drones_surveying          # Count of drones in SURVEYING state
uint8 drones_returning          # Count of drones returning to base
uint8 drones_landed             # Count of drones that have landed
