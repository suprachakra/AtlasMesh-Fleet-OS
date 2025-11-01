# Vehicle Interface Service

> **TL;DR:** CAN Bus communication and ECU integration service providing real-time vehicle control, diagnostics, and hardware abstraction for autonomous vehicle operations.

## Overview

The Vehicle Interface Service provides low-level communication with vehicle Electronic Control Units (ECUs) through CAN Bus, enabling real-time vehicle control, diagnostics, and sensor data acquisition. It serves as the bridge between the autonomous software and vehicle hardware.

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                VEHICLE INTERFACE SERVICE                    │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  ┌──────────────┐  ┌──────────────┐  ┌─────────────┐     │
│  │   CAN Bus    │  │   ECU        │  │   Vehicle   │     │
│  │   Manager    │  │   Interface  │  │   Control   │     │
│  └──────────────┘  └──────────────┘  └─────────────┘     │
│         ▲                 ▲                  │             │
│         │                 │                  ▼             │
│  ┌──────────────────────────────────────────────────┐     │
│  │         Hardware Abstraction Layer (HAL)        │     │
│  └──────────────────────────────────────────────────┘     │
│         ▲                 ▲                  │             │
│  ┌──────┴─────┐    ┌──────┴──────┐   ┌──────┴──────┐    │
│  │   Engine   │    │   Brake     │   │  Steering   │    │
│  │    ECU     │    │    ECU      │   │    ECU      │    │
│  └────────────┘    └─────────────┘   └─────────────┘    │
│         ▲                 ▲                  │             │
└─────────┼─────────────────┼──────────────────┼─────────────┘
          │                 │                  │
    ┌─────┴────┐      ┌─────┴─────┐     ┌─────┴─────┐
    │  Throttle│      │   Brake   │     │  Steering │
    │  Control │      │  Control  │     │  Control  │
    └──────────┘      └───────────┘     └───────────┘
```

## Features

- **CAN Bus Communication**: Real-time CAN message handling
- **ECU Integration**: Direct communication with vehicle ECUs
- **Vehicle Control**: Throttle, brake, steering control
- **Diagnostics**: Real-time vehicle diagnostics and health monitoring
- **Safety Systems**: Emergency stop and fail-safe mechanisms
- **Data Logging**: Comprehensive vehicle data logging
- **Protocol Support**: J1939, ISO 14229, OBD-II protocols
- **Hardware Abstraction**: Unified interface for different vehicle types
- **ROS2 Integration**: Native ROS2 message publishing

## Supported ECUs

### Powertrain
- **Engine Control Unit (ECU)**: Engine management and control
- **Transmission Control Unit (TCU)**: Gear shifting and transmission
- **Throttle Control**: Electronic throttle control
- **Fuel System**: Fuel injection and management

### Chassis
- **Brake Control Unit (BCU)**: ABS, ESP, brake force distribution
- **Steering Control Unit (SCU)**: Power steering and stability
- **Suspension Control**: Active suspension systems
- **Tire Pressure Monitoring**: TPMS integration

### Body
- **Body Control Module (BCM)**: Lighting, doors, windows
- **Climate Control**: HVAC system control
- **Infotainment**: Audio, navigation, connectivity
- **Security**: Immobilizer, alarm systems

### Safety
- **Airbag Control Unit**: SRS and airbag systems
- **Stability Control**: ESC, traction control
- **Collision Avoidance**: Forward collision warning
- **Lane Departure**: Lane keeping assist

## API Endpoints

### Vehicle Control
- `POST /api/v1/vehicle/control/throttle` - Set throttle position
- `POST /api/v1/vehicle/control/brake` - Apply brake force
- `POST /api/v1/vehicle/control/steering` - Set steering angle
- `POST /api/v1/vehicle/control/gear` - Set gear position
- `POST /api/v1/vehicle/control/emergency-stop` - Emergency stop

### Diagnostics
- `GET /api/v1/vehicle/diagnostics` - Get vehicle diagnostics
- `GET /api/v1/vehicle/diagnostics/codes` - Get DTC codes
- `POST /api/v1/vehicle/diagnostics/clear-codes` - Clear DTC codes
- `GET /api/v1/vehicle/diagnostics/live-data` - Get live diagnostic data

### ECU Communication
- `GET /api/v1/vehicle/ecus` - List all ECUs
- `GET /api/v1/vehicle/ecus/{id}` - Get ECU information
- `POST /api/v1/vehicle/ecus/{id}/read` - Read ECU data
- `POST /api/v1/vehicle/ecus/{id}/write` - Write ECU data

### CAN Bus
- `GET /api/v1/vehicle/can/messages` - Get CAN messages
- `POST /api/v1/vehicle/can/send` - Send CAN message
- `GET /api/v1/vehicle/can/statistics` - Get CAN statistics
- `POST /api/v1/vehicle/can/filter` - Set CAN filters

## Data Formats

### Vehicle Control Command
```json
{
  "timestamp": "2024-01-01T12:00:00Z",
  "command_type": "throttle",
  "value": 0.5,
  "unit": "percent",
  "safety_checks": {
    "emergency_stop": false,
    "brake_override": false,
    "steering_override": false
  }
}
```

### ECU Data
```json
{
  "timestamp": "2024-01-01T12:00:00Z",
  "ecu_id": "engine_ecu",
  "data": {
    "rpm": 2500,
    "throttle_position": 45.2,
    "engine_temperature": 85.5,
    "fuel_level": 75.0
  },
  "status": "normal",
  "dtc_codes": []
}
```

### CAN Message
```json
{
  "timestamp": "2024-01-01T12:00:00Z",
  "id": "0x123",
  "data": "FF00FF00FF00FF00",
  "dlc": 8,
  "interface": "can0",
  "direction": "rx"
}
```

## Configuration

### CAN Bus Configuration
```yaml
can_bus:
  interfaces:
    - name: "can0"
      type: "socketcan"
      bitrate: 500000
      channel: "can0"
    - name: "can1"
      type: "socketcan"
      bitrate: 250000
      channel: "can1"
      
  filters:
    - id: "0x100-0x1FF"
      mask: "0x700"
      interface: "can0"
```

### ECU Configuration
```yaml
ecus:
  engine_ecu:
    id: "0x7E0"
    name: "Engine Control Unit"
    protocol: "ISO_14229"
    services:
      - "ReadDataByIdentifier"
      - "WriteDataByIdentifier"
      - "RoutineControl"
      
  brake_ecu:
    id: "0x7E1"
    name: "Brake Control Unit"
    protocol: "J1939"
    services:
      - "RequestPGN"
      - "CommandPGN"
```

## Performance

- **CAN Message Rate**: Up to 1MHz
- **Response Time**: < 10ms for critical commands
- **Throughput**: 1MB/s+ data processing
- **Latency**: < 5ms end-to-end
- **Reliability**: 99.99% message delivery

## Safety Features

- **Emergency Stop**: Immediate vehicle stop capability
- **Fail-Safe Modes**: Graceful degradation on failures
- **Safety Checks**: Pre-command safety validation
- **Override Protection**: Manual override detection
- **Diagnostic Monitoring**: Continuous health monitoring

## Deployment

### Docker
```bash
docker build -t vehicle-interface .
docker run --rm -it --privileged --network host vehicle-interface
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
1. **CAN Interface Not Found**: Check CAN interface configuration
2. **ECU Communication Failed**: Verify ECU addresses and protocols
3. **Control Commands Rejected**: Check safety systems and overrides
4. **High Message Loss**: Check CAN bus termination and wiring

### Logs
- **Application Logs**: `/opt/atlasmesh/logs/vehicle-interface/`
- **CAN Logs**: `/opt/atlasmesh/logs/can/`
- **ECU Logs**: `/opt/atlasmesh/logs/ecu/`

## Development

### Adding New ECUs
1. Define ECU configuration
2. Implement communication protocol
3. Add data parsing logic
4. Update safety checks
5. Add diagnostics support

### Driver Interface
```go
type VehicleDriver interface {
    Initialize(config VehicleConfig) error
    Start() error
    Stop() error
    SendCommand(cmd VehicleCommand) error
    ReadData(ecuID string) (ECUData, error)
    GetDiagnostics() (Diagnostics, error)
    Close() error
}
```