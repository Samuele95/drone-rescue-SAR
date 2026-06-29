cmake_minimum_required(VERSION 3.8)
project(drone_rescue_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PheromoneMap.msg"
  "msg/DroneStatus.msg"
  "msg/WeatherState.msg"
  "msg/CoverageMetrics.msg"
  "msg/VictimDetection.msg"
  "msg/DroneReadiness.msg"
  "msg/FleetReadiness.msg"
  "msg/SystemMode.msg"
  "msg/TaskAssignment.msg"
  "msg/TaskStatus.msg"
  "msg/VictimCandidate.msg"
  "msg/MissionState.msg"
  "msg/DronePeerState.msg"
  "msg/DroneHealth.msg"
  "msg/MissionEvent.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
