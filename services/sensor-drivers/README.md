# Sensor Drivers Service

> **TL;DR:** Hardware abstraction layer providing unified sensor drivers for LiDAR, Camera, Radar, GPS/IMU, and other vehicle sensors with real-time data processing and calibration.

## Overview

The Sensor Drivers Service provides a unified hardware abstraction layer for all vehicle sensors, including LiDAR, cameras, radar, GPS/IMU, and wheel encoders. It handles sensor initialization, data acquisition, calibration, and real-time processing.

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                SENSOR DRIVERS SERVICE                      │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  ┌──────────────┐  ┌──────────────┐  ┌─────────────┐     │
│  │   Sensor     │  │   Data       │  │   Calibration│     │
│  │  Manager     │  │  Processor   │  │   Manager    │     │
│  └──────────────┘  └──────────────┘  └─────────────┘     │
│         ▲                 ▲                  │             │
│         │                 │                  ▼             │
│  ┌──────────────────────────────────────────────────┐     │
│  │         Hardware Abstraction Layer (HAL)        │     │
│  └──────────────────────────────────────────────────┘     │
│         ▲                 ▲                  │             │
│  ┌──────┴─────┐    ┌──────┴──────┐   ┌──────┴──────┐    │
│  │   LiDAR    │    │   Camera    │   │    Radar    │    │
│  │  Drivers   │    │  Drivers    │   │  Drivers    │    │
│  └────────────┘    └─────────────┘   └─────────────┘    │
│         ▲                 ▲                  │             │
└─────────┼─────────────────┼──────────────────┼─────────────┘
          │                 │                  │
    ┌─────┴────┐      ┌─────┴─────┐     ┌─────┴─────┐
    │  GPS/IMU │      │   Wheel   │     │   Other   │
    │ Drivers  │      │ Encoders  │     │ Sensors   │
    └──────────┘      └───────────┘     └───────────┘
```

## Features

- **Multi-Sensor Support**: LiDAR, Camera, Radar, GPS/IMU, Wheel Encoders
- **Hardware Abstraction**: Unified interface for different sensor types
- **Real-time Processing**: High-frequency data acquisition and processing
- **Sensor Calibration**: Automatic and manual calibration capabilities
- **Data Fusion**: Multi-sensor data fusion and synchronization
- **Error Handling**: Robust error detection and recovery
- **Performance Monitoring**: Sensor health and performance metrics
- **Configuration Management**: Dynamic sensor configuration
- **ROS2 Integration**: Native ROS2 message publishing

## Supported Sensors

### LiDAR Sensors
- **Velodyne**: VLP-16, VLP-32, HDL-32E, HDL-64E
- **Ouster**: OS1, OS2 series
- **Robosense**: RS-LiDAR series
- **Livox**: Avia, Horizon, Tele-15
- **Hesai**: Pandar series

### Camera Sensors
- **Monocular Cameras**: USB, GigE, MIPI
- **Stereo Cameras**: ZED, RealSense, Intel D435i
- **Multi-Camera Systems**: 360° camera arrays
- **Thermal Cameras**: FLIR, Seek Thermal

### Radar Sensors
- **Millimeter Wave**: Continental ARS408, Delphi ESR
- **Ultrasonic**: Parking sensors, proximity detection
- **Laser Radar**: SICK LMS, Hokuyo URG

### GPS/IMU Sensors
- **RTK GPS**: NovAtel, Trimble, u-blox
- **IMU**: Xsens, Bosch, InvenSense
- **Compass**: Honeywell, PNI
- **Wheel Encoders**: Optical, magnetic encoders

## API Endpoints

### Sensor Management
- `GET /api/v1/sensors` - List all sensors
- `GET /api/v1/sensors/{id}` - Get sensor details
- `POST /api/v1/sensors/{id}/start` - Start sensor
- `POST /api/v1/sensors/{id}/stop` - Stop sensor
- `POST /api/v1/sensors/{id}/reset` - Reset sensor

### Data Acquisition
- `GET /api/v1/sensors/{id}/data` - Get latest sensor data
- `GET /api/v1/sensors/{id}/stream` - Stream sensor data
- `POST /api/v1/sensors/{id}/capture` - Capture single frame

### Calibration
- `POST /api/v1/sensors/{id}/calibrate` - Start calibration
- `GET /api/v1/sensors/{id}/calibration` - Get calibration data
- `PUT /api/v1/sensors/{id}/calibration` - Update calibration

### Configuration
- `GET /api/v1/sensors/{id}/config` - Get sensor configuration
- `PUT /api/v1/sensors/{id}/config` - Update sensor configuration
- `POST /api/v1/sensors/{id}/test` - Test sensor configuration

## Data Formats

### LiDAR Data
```json
{
  "timestamp": "2024-01-01T12:00:00Z",
  "sensor_id": "lidar_001",
  "points": [
    {
      "x": 1.0,
      "y": 2.0,
      "z": 3.0,
      "intensity": 255,
      "ring": 0
    }
  ],
  "header": {
    "frame_id": "lidar_link",
    "seq": 12345
  }
}
```

### Camera Data
```json
{
  "timestamp": "2024-01-01T12:00:00Z",
  "sensor_id": "camera_001",
  "image": {
    "width": 1920,
    "height": 1080,
    "encoding": "bgr8",
    "data": "base64_encoded_image"
  },
  "camera_info": {
    "fx": 1000.0,
    "fy": 1000.0,
    "cx": 960.0,
    "cy": 540.0,
    "distortion": [0.0, 0.0, 0.0, 0.0, 0.0]
  }
}
```

### GPS/IMU Data
```json
{
  "timestamp": "2024-01-01T12:00:00Z",
  "sensor_id": "gps_imu_001",
  "gps": {
    "latitude": 25.2048,
    "longitude": 55.2744,
    "altitude": 10.0,
    "accuracy": 1.0,
    "satellites": 12
  },
  "imu": {
    "linear_acceleration": {"x": 0.0, "y": 0.0, "z": 9.81},
    "angular_velocity": {"x": 0.0, "y": 0.0, "z": 0.0},
    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
  }
}
```

## Configuration

### Sensor Configuration
```yaml
sensors:
  lidar:
    - id: "lidar_001"
      type: "velodyne_vlp16"
      port: "/dev/ttyUSB0"
      baudrate: 115200
      frame_id: "lidar_link"
      frequency: 10.0
      
  camera:
    - id: "camera_001"
      type: "usb_camera"
      device: "/dev/video0"
      width: 1920
      height: 1080
      fps: 30
      frame_id: "camera_link"
      
  radar:
    - id: "radar_001"
      type: "continental_ars408"
      can_interface: "can0"
      can_id: 0x200
      frame_id: "radar_link"
      
  gps_imu:
    - id: "gps_imu_001"
      type: "novatel_span"
      port: "/dev/ttyUSB1"
      baudrate: 115200
      frame_id: "gps_imu_link"
```

## Performance

- **LiDAR**: Up to 100Hz data rate
- **Camera**: Up to 60fps at 4K resolution
- **Radar**: Up to 50Hz data rate
- **GPS/IMU**: Up to 200Hz data rate
- **Latency**: < 10ms end-to-end
- **Throughput**: 1GB/s+ data processing

## Deployment

### Docker
```bash
docker build -t sensor-drivers .
docker run --rm -it --device=/dev/ttyUSB0 --device=/dev/video0 sensor-drivers
```

### Kubernetes
```bash
kubectl apply -f k8s/
```

### Local Development
```bash
# Install dependencies
go mod download

# Run the service
go run ./cmd/main.go
```

## Troubleshooting

### Common Issues
1. **Sensor Not Detected**: Check device permissions and connections
2. **Data Quality Issues**: Verify calibration and configuration
3. **Performance Issues**: Check system resources and sensor settings
4. **Connection Errors**: Verify hardware connections and drivers

### Logs
- **Application Logs**: `/opt/atlasmesh/logs/sensor-drivers/`
- **Sensor Logs**: `/opt/atlasmesh/logs/sensors/`
- **System Logs**: `/var/log/atlasmesh/`

## Development

### Adding New Sensors
1. Implement sensor driver interface
2. Add configuration schema
3. Update data formats
4. Add calibration procedures
5. Update documentation

### Driver Interface
```go
type SensorDriver interface {
    Initialize(config SensorConfig) error
    Start() error
    Stop() error
    GetData() (SensorData, error)
    Calibrate() error
    GetStatus() SensorStatus
    Close() error
}
```
