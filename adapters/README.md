# Vehicle Integration Adapters

Vehicle-agnostic adapter SDK for integrating any vehicle with drive-by-wire capabilities into the AtlasMesh Fleet OS.

## Design Principles

- **Hardware Abstraction**: Common interface for all vehicle types
- **Hot Pluggable**: Add new vehicle types without core system changes
- **Type Safety**: Strongly typed interfaces with runtime validation
- **Fault Tolerance**: Graceful degradation on adapter failures
- **Certification**: Each adapter must pass conformance testing

## Architecture

```
Vehicle Hardware
     ↓
Drive-by-Wire Interface
     ↓
Vehicle Adapter (implements standard interface)
     ↓
Fleet Management System
```

## Directory Structure

```
adapters/
├── sdk/                    # Core adapter SDK
│   ├── interfaces/        # TypeScript interface definitions
│   ├── base-adapter/      # Base adapter class
│   ├── validators/        # Input/output validation
│   └── testing/           # Conformance test framework
├── vehicles/              # Vehicle-specific adapters
│   ├── ugv-themis/        # UGV Themis adapter
│   ├── haul-truck-cat/    # CAT mining truck adapter
│   ├── sedan-generic/     # Generic sedan adapter
│   └── template/          # Template for new adapters
├── sensors/               # Sensor hardware adapters
│   ├── lidar/
│   ├── camera/
│   └── radar/
├── dbw/                   # Drive-by-wire adapters
│   ├── dataspeed/
│   ├── autonomoustuff/
│   └── generic-can/
├── erp/                   # ERP/WMS/TOS system adapters
│   ├── sap/
│   ├── oracle-wms/
│   └── generic-rest/
└── testing/               # Integration tests
    ├── conformance/       # Adapter certification tests
    └── compatibility/     # Cross-adapter compatibility
```

## Core Interfaces

### Vehicle Adapter Interface

```typescript
// adapters/sdk/interfaces/IVehicleAdapter.ts
export interface IVehicleAdapter {
  // Identification
  readonly vehicleId: string;
  readonly vehicleType: VehicleType;
  readonly capabilityManifest: CapabilityManifest;

  // Lifecycle
  initialize(): Promise<void>;
  connect(): Promise<void>;
  disconnect(): Promise<void>;
  healthCheck(): Promise<HealthStatus>;

  // Control
  sendMotionCommand(command: MotionCommand): Promise<CommandResult>;
  sendSteeringCommand(command: SteeringCommand): Promise<CommandResult>;
  enableAutonomousMode(): Promise<void>;
  disableAutonomousMode(): Promise<void>;
  emergencyStop(): Promise<void>;

  // State
  getVehicleState(): Promise<VehicleState>;
  getTelemetry(): Promise<VehicleTelemetry>;
  getBatteryStatus(): Promise<BatteryStatus>;

  // Events
  onStateChange: EventEmitter<VehicleStateChangeEvent>;
  onTelemetry: EventEmitter<VehicleTelemetryEvent>;
  onFault: EventEmitter<VehicleFaultEvent>;
}
```

### Drive-by-Wire Interface

```typescript
// adapters/sdk/interfaces/IDbWAdapter.ts
export interface IDbWAdapter {
  // Actuation
  setThrottle(percentage: number): Promise<void>;
  setBrake(percentage: number): Promise<void>;
  setSteering(angle: number): Promise<void>;
  setGear(gear: Gear): Promise<void>;

  // Feedback
  getActuatorStatus(): Promise<ActuatorStatus>;
  getFailsafeStatus(): Promise<FailsafeStatus>;
  
  // Safety
  enableFailsafe(): Promise<void>;
  disableFailsafe(): Promise<void>;
  testActuators(): Promise<TestResults>;
}
```

### Sensor Adapter Interface

```typescript
// adapters/sdk/interfaces/ISensorAdapter.ts
export interface ISensorAdapter {
  readonly sensorType: SensorType;
  readonly sensorModel: string;
  readonly capabilities: SensorCapabilities;

  // Data streams
  getPointCloud(): AsyncIterable<PointCloud>;
  getCameraFrames(): AsyncIterable<CameraFrame>;
  getRadarTracks(): AsyncIterable<RadarTrack>;
  
  // Configuration
  configure(settings: SensorSettings): Promise<void>;
  calibrate(): Promise<CalibrationResult>;
  
  // Health
  getSensorHealth(): Promise<SensorHealth>;
  runDiagnostics(): Promise<DiagnosticResults>;
}
```

## Capability Manifest System

Each vehicle adapter declares its capabilities in a standard manifest:

```typescript
// Example capability manifest for UGV Themis
export const UGV_THEMIS_MANIFEST: CapabilityManifest = {
  vehicle_id: "ugv_themis",
  vehicle_class: "ugv",
  
  // Physical capabilities
  dimensions: { length: 2.5, width: 1.8, height: 1.5 }, // meters
  weight: { empty: 1200, max_payload: 1000 }, // kg
  max_speed: 25, // km/h
  max_grade: 30, // degrees
  
  // Drive capabilities
  drive_by_wire: {
    steering: { min_angle: -30, max_angle: 30, resolution: 0.1 },
    throttle: { min: 0, max: 100, resolution: 0.1 },
    brake: { min: 0, max: 100, resolution: 0.1 },
    gear: ["park", "neutral", "drive", "reverse"]
  },
  
  // Sensor suite
  sensors: [
    { type: "lidar", model: "velodyne_vlp16", position: [0, 0, 1.5] },
    { type: "camera", model: "flir_blackfly", count: 6 },
    { type: "imu", model: "vectornav_vn100" }
  ],
  
  // Compute platform
  compute: {
    platform: "nvidia_jetson_agx_orin",
    cpu_cores: 8,
    gpu_memory: "32GB",
    storage: "1TB_nvme"
  },
  
  // Power system
  power: {
    type: "battery",
    capacity: "50kWh",
    voltage: "48V",
    max_power: "2kW",
    charging: ["ccs2", "type2"]
  },
  
  // Environmental limits
  operating_conditions: {
    temperature: { min: -20, max: 55 }, // celsius
    humidity: { min: 0, max: 95 }, // percentage
    ingress_protection: "IP67",
    altitude: { min: -500, max: 3000 } // meters
  },
  
  // Operational domains
  supported_odds: ["mining_site", "port_yard", "military_base"],
  
  // Regulatory
  certifications: ["ce_marking", "fcc_part_15"],
  safety_integrity_level: "sil2"
};
```

## Adapter Development

### Creating a New Adapter

1. Use the template:
```bash
cp -r adapters/vehicles/template adapters/vehicles/my-vehicle
cd adapters/vehicles/my-vehicle
npm install
```

2. Implement the required interfaces:
```typescript
// adapters/vehicles/my-vehicle/src/MyVehicleAdapter.ts
import { BaseVehicleAdapter } from '@atlasmesh/adapter-sdk';

export class MyVehicleAdapter extends BaseVehicleAdapter {
  async initialize(): Promise<void> {
    // Initialize vehicle connection
  }
  
  async sendMotionCommand(command: MotionCommand): Promise<CommandResult> {
    // Translate to vehicle-specific protocol
  }
  
  // ... implement all required methods
}
```

3. Define capability manifest:
```typescript
// adapters/vehicles/my-vehicle/src/manifest.ts
export const MY_VEHICLE_MANIFEST: CapabilityManifest = {
  // Define vehicle capabilities
};
```

### Conformance Testing

All adapters must pass certification tests:

```bash
# Run conformance tests
npm run test:conformance -- --adapter=ugv-themis

# Generate certification report
npm run certify -- --adapter=ugv-themis --output=cert_report.pdf
```

### Integration Testing

Test adapter with real hardware:

```bash
# Hardware-in-the-loop testing
npm run test:hil -- --adapter=ugv-themis --vehicle-id=test_001

# Simulation testing
npm run test:sim -- --adapter=ugv-themis --scenario=basic_navigation
```

## ERP/WMS Integration Adapters

Connect to enterprise systems for logistics coordination:

```typescript
// adapters/erp/sap/src/SAPAdapter.ts
export class SAPAdapter implements IERPAdapter {
  async createWorkOrder(order: WorkOrder): Promise<string> {
    // Create SAP work order
  }
  
  async updateInventory(update: InventoryUpdate): Promise<void> {
    // Update SAP inventory
  }
  
  async getSchedule(): Promise<Schedule> {
    // Fetch SAP schedule
  }
}
```

## Fault Tolerance

Adapters implement graceful degradation:

```typescript
// Example fault tolerance
class ResilientVehicleAdapter extends BaseVehicleAdapter {
  async sendMotionCommand(command: MotionCommand): Promise<CommandResult> {
    try {
      return await this.primaryChannel.send(command);
    } catch (primaryError) {
      this.logger.warn("Primary channel failed, trying backup", primaryError);
      return await this.backupChannel.send(command);
    }
  }
}
```

## Performance Requirements

- **Latency**: Control commands ≤ 10ms p95
- **Throughput**: Handle ≥ 100 commands/second
- **Reliability**: 99.9% uptime during operations
- **Recovery**: Automatic reconnection within 5 seconds

## Security

All adapters implement:
- mTLS for all communications
- Message signing for safety-critical commands
- Input validation and sanitization
- Secure credential management
