# Vehicle Agent Service

> **TL;DR:** ROS2-based on-vehicle software brain for the AtlasMesh Fleet Management System that connects autonomous driving sensors to the fleet management cloud, handling real-time perception, control, and communication.

## Overview

The Vehicle Agent is the **on-vehicle software brain** that runs on the vehicle's compute unit (CPU/GPU) and handles real-time perception, control, and communication. It connects autonomous driving sensors to the fleet management cloud using ROS2 DDS middleware.

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    VEHICLE AGENT                            │
│                  (ROS2 Humble/Iron)                         │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  ┌──────────────┐  ┌──────────────┐  ┌─────────────┐     │
│  │ Perception   │  │ Localization │  │   Control   │     │
│  │   Module     │  │    Module    │  │   Module    │     │
│  └──────────────┘  └──────────────┘  └─────────────┘     │
│         ▲                 ▲                  │             │
│         │                 │                  ▼             │
│  ┌──────────────────────────────────────────────────┐     │
│  │         ROS2 DDS Middleware (Cyclone/FastDDS)    │     │
│  └──────────────────────────────────────────────────┘     │
│         ▲                 ▲                  │             │
│  ┌──────┴─────┐    ┌──────┴──────┐   ┌──────┴──────┐    │
│  │  Sensor    │    │   Vehicle   │   │   Cloud     │    │
│  │  Drivers   │    │   CAN Bus   │   │   Bridge    │    │
│  └────────────┘    └─────────────┘   └─────────────┘    │
│         ▲                 ▲                  │             │
└─────────┼─────────────────┼──────────────────┼─────────────┘
          │                 │                  │
    ┌─────┴────┐      ┌─────┴─────┐     ┌─────┴─────┐
    │ LiDAR    │      │  Vehicle  │     │  4G/5G    │
    │ Radar    │      │  ECUs     │     │  Modem    │
    │ Cameras  │      │  (Steering,│     │ (Cloud    │
    │ GPS/IMU  │      │   Brake,  │     │  Conn.)   │
    └──────────┘      │   Throttle)│     └───────────┘
                      └───────────┘
```

## Features

- **ROS2 DDS Middleware**: Real-time communication between modules
- **Perception Module**: LiDAR, camera, radar processing and fusion
- **Localization Module**: GPS/IMU fusion, SLAM, HD map matching
- **Control Module**: Path planning, motion control, emergency systems
- **Sensor Drivers**: Hardware abstraction for all sensors
- **Vehicle CAN Bus**: Communication with vehicle ECUs
- **Cloud Bridge**: Secure communication with fleet management
- **Safety Monitoring**: Real-time safety event detection
- **ODD Compliance**: Operational Design Domain enforcement

## ROS2 Nodes

### Perception Nodes
- `/vehicle_agent/perception/lidar_processor` - LiDAR point cloud processing
- `/vehicle_agent/perception/camera_processor` - Camera image processing
- `/vehicle_agent/perception/radar_processor` - Radar target processing
- `/vehicle_agent/perception/fusion_engine` - Multi-sensor fusion
- `/vehicle_agent/perception/object_tracker` - Object tracking and prediction

### Localization Nodes
- `/vehicle_agent/localization/gps_processor` - GPS/IMU processing
- `/vehicle_agent/localization/slam_engine` - Simultaneous Localization and Mapping
- `/vehicle_agent/localization/map_matcher` - HD map matching
- `/vehicle_agent/localization/pose_estimator` - Pose estimation

### Control Nodes
- `/vehicle_agent/control/path_planner` - Global and local path planning
- `/vehicle_agent/control/motion_controller` - Motion control execution
- `/vehicle_agent/control/emergency_handler` - Emergency stop and safety
- `/vehicle_agent/control/vehicle_interface` - CAN bus communication

### System Nodes
- `/vehicle_agent/system/health_monitor` - System health monitoring
- `/vehicle_agent/system/cloud_bridge` - Cloud communication
- `/vehicle_agent/system/safety_monitor` - Safety event detection
- `/vehicle_agent/system/odd_enforcer` - ODD compliance checking

## API Endpoints

### Vehicle Control
- `POST /api/v1/vehicle-agent/emergency-stop` - Emergency stop
- `POST /api/v1/vehicle-agent/resume` - Resume operations
- `POST /api/v1/vehicle-agent/set-speed-limit` - Set speed limit
- `POST /api/v1/vehicle-agent/update-route` - Update planned route

### Status & Health
- `GET /api/v1/vehicle-agent/status` - Vehicle status
- `GET /api/v1/vehicle-agent/health` - System health
- `GET /api/v1/vehicle-agent/sensors` - Sensor status
- `GET /api/v1/vehicle-agent/position` - Current position

### Configuration
- `POST /api/v1/vehicle-agent/configure` - Update configuration
- `GET /api/v1/vehicle-agent/config` - Get configuration
- `POST /api/v1/vehicle-agent/calibrate` - Sensor calibration

## Database Schema

### Vehicle State
- `vehicle_state` - Current vehicle state and status
- `sensor_health` - Sensor health and calibration status
- `localization_data` - Position and orientation data
- `perception_data` - Object detection and tracking data
- `control_commands` - Control command history
- `safety_events` - Safety event log

### Configuration
- `vehicle_config` - Vehicle configuration parameters
- `sensor_config` - Sensor configuration and calibration
- `safety_config` - Safety parameters and thresholds
- `odd_config` - Operational Design Domain configuration

## Dependencies

- **ROS2 Humble/Iron**: Core ROS2 framework
- **rclcpp**: C++ ROS2 client library
- **sensor_msgs**: Sensor message types
- **geometry_msgs**: Geometry message types
- **nav_msgs**: Navigation message types
- **autoware_msgs**: Autoware message types
- **PCL**: Point Cloud Library for LiDAR processing
- **OpenCV**: Computer vision for camera processing
- **Eigen**: Linear algebra library
- **GTSAM**: Factor graph optimization for SLAM

## Deployment

### Docker
```bash
docker build -t vehicle-agent .
docker run --rm -it --network host vehicle-agent
```

### Kubernetes
```bash
kubectl apply -f k8s/
```

### Local Development
```bash
# Install ROS2 dependencies
sudo apt install ros-humble-desktop

# Build the workspace
colcon build

# Source the workspace
source install/setup.bash

# Run the vehicle agent
ros2 launch vehicle_agent vehicle_agent.launch.py
```

## Safety Features

- **Emergency Stop**: Immediate stop capability
- **Safety Monitoring**: Real-time safety event detection
- **ODD Enforcement**: Operational Design Domain compliance
- **Sensor Redundancy**: Multiple sensor validation
- **Fail-Safe Modes**: Graceful degradation on failures
- **Audit Logging**: Complete operation audit trail

## Performance

- **Latency**: < 100ms end-to-end perception to control
- **Throughput**: 30Hz perception, 100Hz control
- **Reliability**: 99.9% uptime target
- **Safety**: ASIL-D compliance for safety-critical functions

## Monitoring

- **Health Metrics**: CPU, memory, sensor status
- **Performance Metrics**: Latency, throughput, accuracy
- **Safety Metrics**: Emergency stops, ODD violations
- **Operational Metrics**: Trip completion, efficiency

## Troubleshooting

### Common Issues
1. **Sensor Failures**: Check sensor connections and calibration
2. **Localization Loss**: Verify GPS signal and HD map availability
3. **Control Issues**: Check CAN bus communication and vehicle interface
4. **Cloud Connectivity**: Verify network connection and authentication

### Logs
- **ROS2 Logs**: `/opt/atlasmesh/logs/ros2/`
- **Application Logs**: `/opt/atlasmesh/logs/vehicle-agent/`
- **System Logs**: `/var/log/atlasmesh/`

## Development

### Adding New Sensors
1. Create sensor driver node
2. Add message types
3. Update fusion engine
4. Add configuration parameters
5. Update health monitoring

### Adding New Control Modes
1. Define control interface
2. Implement control logic
3. Add safety checks
4. Update emergency handling
5. Add configuration options
