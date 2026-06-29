# Per-drone health snapshot, published by drone_health_monitor at 5 Hz.
# mission_manager subscribes to all drones' health and reassigns sectors when
# a drone is unrecoverable. The drone_executor also subscribes to its OWN
# health stream; when unrecoverable=True it enters a hold pose and emits a
# DRONE_DAMAGE_REPORT mission event.

std_msgs/Header header
string drone_name

float32 alive_for_s          # seconds since this monitor first saw odom

# Continuous anomaly score in [0, 1]; 1 = severe.
float32 anomaly_score
# Comma-separated reason tokens for the latest score, e.g. "imu_spike,odom_freeze"
string  reason

# Hard fail flag: ≥2 anomalies AND drone z<5m. mission_manager treats this
# drone as DOWN, partitions its remaining waypoints across surviving drones.
bool    unrecoverable

# Per-channel diagnostics (also drive the dashboard state table).
float32 battery_remaining_s   # linear curve fit, NaN if unknown
float32 imu_accel_norm        # m/s²
float32 odom_age_s            # seconds since last odom
float32 lidar_min_range_m     # nearest LiDAR return; -1 if no scan yet
float32 vel_command_divergence_m_s   # |cmd_vel - actual_vel|
