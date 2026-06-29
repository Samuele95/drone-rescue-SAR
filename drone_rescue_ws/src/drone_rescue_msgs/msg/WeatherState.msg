# Weather conditions affecting drone operations

std_msgs/Header header

# Weather condition enum
uint8 WEATHER_CLEAR = 0
uint8 WEATHER_WINDY = 1
uint8 WEATHER_STORM = 2
uint8 condition

# Wind parameters
geometry_msgs/Vector3 wind_velocity    # Wind vector in m/s (world frame)
float32 wind_speed                      # Magnitude of wind (m/s)
float32 wind_direction                  # Direction in radians (0 = North, increases clockwise)

# Visibility (affects sensor range)
float32 visibility         # 0.0 to 1.0 (1.0 = full visibility)

# Effect multipliers for drones
float32 lidar_range_multiplier    # Applied to LiDAR max range
float32 battery_drain_multiplier  # Applied to battery consumption

# Forecast (optional)
float32 time_to_next_change       # Seconds until weather may change
uint8 predicted_next_condition    # Predicted next weather state
