cmake_minimum_required(VERSION 3.8)
project(atlasmesh_vehicle_agent)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate custom interfaces
set(msg_files
  "msg/VehicleState.msg"
  "msg/VehicleCommand.msg"
  "msg/VehicleProfile.msg"
  "msg/SafetyStatus.msg"
  "msg/HealthStatus.msg"
  "msg/PolicyDecision.msg"
  "msg/TelemetryData.msg"
  "msg/OTAUpdate.msg"
)

set(srv_files
  "srv/UpdateVehicleProfile.srv"
  "srv/ExecuteSafeStop.srv"
  "srv/ValidateODD.srv"
  "srv/ApplyOTAUpdate.srv"
)

set(action_files
  "action/ExecuteTrip.action"
  "action/NavigateToGoal.action"
  "action/PerformManeuver.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES
    std_msgs
    geometry_msgs
    sensor_msgs
    nav_msgs
    diagnostic_msgs
    control_msgs
    trajectory_msgs
)

# Core vehicle agent executable
add_executable(vehicle_agent_node
  src/vehicle_agent_node.cpp
  src/vehicle_controller.cpp
  src/safety_monitor.cpp
  src/health_monitor.cpp
  src/policy_client.cpp
  src/cloud_bridge_client.cpp
  src/sensor_manager.cpp
  src/navigation_manager.cpp
  src/ota_manager.cpp
)

ament_target_dependencies(vehicle_agent_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  diagnostic_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav2_msgs
  control_msgs
  trajectory_msgs
  pcl_ros
  image_transport
  cv_bridge
)

rosidl_target_interfaces(vehicle_agent_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Safety monitor node
add_executable(safety_monitor_node
  src/safety_monitor_node.cpp
  src/safety_monitor.cpp
  src/emergency_stop.cpp
  src/watchdog.cpp
)

ament_target_dependencies(safety_monitor_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  diagnostic_msgs
)

rosidl_target_interfaces(safety_monitor_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Health monitor node
add_executable(health_monitor_node
  src/health_monitor_node.cpp
  src/health_monitor.cpp
  src/diagnostics_collector.cpp
  src/performance_monitor.cpp
)

ament_target_dependencies(health_monitor_node
  rclcpp
  std_msgs
  diagnostic_msgs
  sensor_msgs
)

rosidl_target_interfaces(health_monitor_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Cloud bridge node
add_executable(cloud_bridge_node
  src/cloud_bridge_node.cpp
  src/cloud_bridge_client.cpp
  src/message_queue.cpp
  src/telemetry_publisher.cpp
)

ament_target_dependencies(cloud_bridge_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  diagnostic_msgs
)

rosidl_target_interfaces(cloud_bridge_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# OTA update manager
add_executable(ota_manager_node
  src/ota_manager_node.cpp
  src/ota_manager.cpp
  src/update_verifier.cpp
  src/rollback_manager.cpp
)

ament_target_dependencies(ota_manager_node
  rclcpp
  std_msgs
)

rosidl_target_interfaces(ota_manager_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Install executables
install(TARGETS
  vehicle_agent_node
  safety_monitor_node
  health_monitor_node
  cloud_bridge_node
  ota_manager_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python scripts
install(PROGRAMS
  scripts/vehicle_diagnostics.py
  scripts/sensor_calibration.py
  scripts/performance_analyzer.py
  DESTINATION lib/${PROJECT_NAME}
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  
  # Linting
  ament_lint_auto_find_test_dependencies()
  
  # Unit tests
  ament_add_gtest(test_vehicle_controller test/test_vehicle_controller.cpp)
  ament_target_dependencies(test_vehicle_controller rclcpp)
  rosidl_target_interfaces(test_vehicle_controller ${PROJECT_NAME} "rosidl_typesupport_cpp")
  
  ament_add_gtest(test_safety_monitor test/test_safety_monitor.cpp)
  ament_target_dependencies(test_safety_monitor rclcpp)
  rosidl_target_interfaces(test_safety_monitor ${PROJECT_NAME} "rosidl_typesupport_cpp")
  
  # Integration tests
  add_launch_test(test/integration/test_vehicle_agent_integration.py)
  add_launch_test(test/integration/test_safety_systems.py)
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
