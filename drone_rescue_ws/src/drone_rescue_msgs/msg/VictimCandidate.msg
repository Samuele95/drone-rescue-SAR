# A multi-view-confirmed victim candidate emitted by detection_filter.
# Mission_manager subscribes and converts each into INVESTIGATE / CONFIRM tasks.

std_msgs/Header header
uint32 candidate_id
geometry_msgs/Point position

# Bayesian-fused confidence over all clustered sightings:
#   P = 1 - prod(1 - p_i)
float32 confidence

# Number of distinct sightings in the cluster (NOT distinct drones).
uint32 observation_count

# Which drones contributed at least one sighting to this cluster.
string[] reporting_drones

# True when:
#   - reporting_drones contains >= 2 unique names AND
#   - confidence >= confirmation_threshold (default 0.7)
bool confirmed
