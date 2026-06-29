# Pheromone map for stigmergic coordination
# Grid-based representation of pheromone intensities

std_msgs/Header header

# Grid dimensions
uint32 width          # Number of cells in x direction
uint32 height         # Number of cells in y direction
float32 resolution    # Cell size in meters (e.g., 1.0 = 1m x 1m cells)

# Origin of the grid (world coordinates of cell [0,0])
geometry_msgs/Point origin

# Pheromone values as flat array (row-major order)
# Values range from 0.0 (no pheromone) to 1.0 (max pheromone)
float32[] data
