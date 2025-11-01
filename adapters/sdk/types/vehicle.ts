/**
 * vehicle.ts
 * Type definitions for the AtlasMesh Vehicle Adapter SDK
 */

/**
 * Represents the status of a vehicle adapter
 */
export enum AdapterStatus {
  DISCONNECTED = 'disconnected',
  CONNECTING = 'connecting',
  CONNECTED = 'connected',
  ERROR = 'error',
  INITIALIZING = 'initializing',
  READY = 'ready'
}

/**
 * Represents a command that can be sent to a vehicle
 */
export interface VehicleCommand {
  /** Unique identifier for the command */
  id: string;
  
  /** Type of command */
  type: VehicleCommandType;
  
  /** Command-specific parameters */
  parameters: Record<string, any>;
  
  /** Timeout in milliseconds */
  timeoutMs?: number;
  
  /** Priority level of the command */
  priority?: CommandPriority;
}

/**
 * Types of commands that can be sent to a vehicle
 */
export enum VehicleCommandType {
  // Drive-by-wire commands
  STEERING = 'steering',
  THROTTLE = 'throttle',
  BRAKE = 'brake',
  GEAR = 'gear',
  PARKING_BRAKE = 'parking_brake',
  TURN_SIGNAL = 'turn_signal',
  
  // System commands
  POWER = 'power',
  RESET = 'reset',
  CALIBRATE = 'calibrate',
  
  // Safety commands
  EMERGENCY_STOP = 'emergency_stop',
  SAFE_STOP = 'safe_stop',
  
  // Diagnostic commands
  RUN_DIAGNOSTICS = 'run_diagnostics',
  CLEAR_FAULTS = 'clear_faults',
  
  // Auxiliary commands
  LIGHTS = 'lights',
  HORN = 'horn',
  HVAC = 'hvac',
  WIPER = 'wiper',
  
  // Sensor commands
  SENSOR_CONFIG = 'sensor_config',
  
  // Custom commands
  CUSTOM = 'custom'
}

/**
 * Command priority levels
 */
export enum CommandPriority {
  LOW = 'low',
  NORMAL = 'normal',
  HIGH = 'high',
  CRITICAL = 'critical'
}

/**
 * Result of a command execution
 */
export interface CommandResult {
  /** Command ID this result is for */
  commandId: string;
  
  /** Success or failure */
  success: boolean;
  
  /** Detailed status code */
  statusCode: number;
  
  /** Human-readable message */
  message: string;
  
  /** Command-specific result data */
  data?: Record<string, any>;
  
  /** Timestamp when the command was completed */
  timestamp: number;
}

/**
 * Vehicle error information
 */
export interface VehicleError {
  /** Error code */
  code: string;
  
  /** Human-readable error message */
  message: string;
  
  /** Error severity */
  severity: ErrorSeverity;
  
  /** Subsystem that generated the error */
  subsystem: string;
  
  /** Additional error details */
  details?: Record<string, any>;
  
  /** Timestamp when the error occurred */
  timestamp: number;
}

/**
 * Error severity levels
 */
export enum ErrorSeverity {
  INFO = 'info',
  WARNING = 'warning',
  ERROR = 'error',
  CRITICAL = 'critical'
}

/**
 * Vehicle state information
 */
export interface VehicleState {
  /** Unique identifier for this state snapshot */
  id: string;
  
  /** Vehicle operational mode */
  mode: VehicleMode;
  
  /** Current motion state */
  motion: MotionState;
  
  /** Current gear */
  gear: GearPosition;
  
  /** Current steering angle in degrees (negative = left, positive = right) */
  steeringAngle: number;
  
  /** Current throttle position (0-100%) */
  throttlePosition: number;
  
  /** Current brake position (0-100%) */
  brakePosition: number;
  
  /** Parking brake status */
  parkingBrake: boolean;
  
  /** Turn signal status */
  turnSignal: TurnSignalState;
  
  /** Current speed in km/h */
  speed: number;
  
  /** Current acceleration in m/s² */
  acceleration: number;
  
  /** Current heading in degrees (0-359, 0 = North) */
  heading: number;
  
  /** Current location */
  location: GeoLocation;
  
  /** Timestamp when this state was captured */
  timestamp: number;
  
  /** Additional state information */
  additionalState?: Record<string, any>;
}

/**
 * Vehicle operational modes
 */
export enum VehicleMode {
  MANUAL = 'manual',
  AUTONOMOUS = 'autonomous',
  REMOTE = 'remote',
  CALIBRATION = 'calibration',
  MAINTENANCE = 'maintenance',
  EMERGENCY = 'emergency'
}

/**
 * Vehicle motion states
 */
export enum MotionState {
  STOPPED = 'stopped',
  MOVING = 'moving',
  ACCELERATING = 'accelerating',
  DECELERATING = 'decelerating',
  TURNING = 'turning'
}

/**
 * Gear positions
 */
export enum GearPosition {
  PARK = 'park',
  REVERSE = 'reverse',
  NEUTRAL = 'neutral',
  DRIVE = 'drive',
  LOW = 'low',
  MANUAL_1 = 'manual_1',
  MANUAL_2 = 'manual_2',
  MANUAL_3 = 'manual_3',
  MANUAL_4 = 'manual_4',
  MANUAL_5 = 'manual_5',
  MANUAL_6 = 'manual_6'
}

/**
 * Turn signal states
 */
export enum TurnSignalState {
  OFF = 'off',
  LEFT = 'left',
  RIGHT = 'right',
  HAZARD = 'hazard'
}

/**
 * Geographic location
 */
export interface GeoLocation {
  /** Latitude in decimal degrees */
  latitude: number;
  
  /** Longitude in decimal degrees */
  longitude: number;
  
  /** Altitude in meters above sea level */
  altitude?: number;
  
  /** Accuracy of the location in meters */
  accuracy?: number;
  
  /** Timestamp when the location was captured */
  timestamp: number;
}

/**
 * Vehicle health information
 */
export interface VehicleHealth {
  /** Overall health status */
  overall: HealthStatus;
  
  /** Battery/fuel level (0-100%) */
  energyLevel: number;
  
  /** Estimated range in kilometers */
  estimatedRange: number;
  
  /** Health status of vehicle subsystems */
  subsystems: {
    /** Drive-by-wire system health */
    driveByWire: HealthStatus;
    
    /** Sensor system health */
    sensors: HealthStatus;
    
    /** Compute system health */
    compute: HealthStatus;
    
    /** Communications system health */
    communications: HealthStatus;
    
    /** Power system health */
    power: HealthStatus;
  };
  
  /** Active fault codes */
  faultCodes: FaultCode[];
  
  /** Timestamp when this health data was captured */
  timestamp: number;
}

/**
 * Health status levels
 */
export enum HealthStatus {
  GOOD = 'good',
  WARNING = 'warning',
  CRITICAL = 'critical',
  UNKNOWN = 'unknown'
}

/**
 * Fault code information
 */
export interface FaultCode {
  /** Fault code identifier */
  code: string;
  
  /** Human-readable description */
  description: string;
  
  /** Subsystem that generated the fault */
  subsystem: string;
  
  /** Fault severity */
  severity: ErrorSeverity;
  
  /** Timestamp when the fault was first detected */
  timestamp: number;
  
  /** Whether the fault is currently active */
  active: boolean;
}

/**
 * Vehicle telemetry data
 */
export interface VehicleTelemetry {
  /** Unique identifier for this telemetry snapshot */
  id: string;
  
  /** Vehicle state */
  state: VehicleState;
  
  /** Vehicle health */
  health: VehicleHealth;
  
  /** Energy metrics */
  energy: {
    /** Voltage in volts */
    voltage: number;
    
    /** Current in amperes */
    current?: number;
    
    /** Power consumption in watts */
    power?: number;
    
    /** Energy consumption rate in kWh/100km or L/100km */
    consumptionRate: number;
  };
  
  /** Environmental conditions */
  environment: {
    /** External temperature in Celsius */
    externalTemperature: number;
    
    /** Internal temperature in Celsius */
    internalTemperature: number;
    
    /** Humidity percentage */
    humidity?: number;
    
    /** Light level in lux */
    lightLevel?: number;
  };
  
  /** Odometry data */
  odometry: {
    /** Total distance traveled in kilometers */
    totalDistance: number;
    
    /** Trip distance in kilometers */
    tripDistance: number;
    
    /** Operating hours */
    operatingHours: number;
  };
  
  /** Timestamp when this telemetry was captured */
  timestamp: number;
  
  /** Additional telemetry data */
  additionalTelemetry?: Record<string, any>;
}

/**
 * Vehicle capabilities information
 */
export interface VehicleCapabilities {
  /** Vehicle model identifier */
  modelId: string;
  
  /** Vehicle class */
  vehicleClass: string;
  
  /** Physical dimensions */
  dimensions: {
    /** Length in meters */
    length: number;
    
    /** Width in meters */
    width: number;
    
    /** Height in meters */
    height: number;
    
    /** Wheelbase in meters */
    wheelbase: number;
  };
  
  /** Weight specifications */
  weight: {
    /** Empty weight in kilograms */
    empty: number;
    
    /** Maximum gross weight in kilograms */
    maxGross: number;
    
    /** Maximum payload in kilograms */
    maxPayload: number;
  };
  
  /** Performance specifications */
  performance: {
    /** Maximum speed in km/h */
    maxSpeed: number;
    
    /** Maximum grade in degrees */
    maxGrade: number;
    
    /** Acceleration 0-100 km/h in seconds */
    acceleration?: number;
    
    /** Braking distance from 100-0 km/h in meters */
    brakingDistance?: number;
  };
  
  /** Drive-by-wire capabilities */
  driveByWire: {
    /** Steering capabilities */
    steering: {
      /** Whether steering control is available */
      available: boolean;
      
      /** Minimum steering angle in degrees */
      minAngle: number;
      
      /** Maximum steering angle in degrees */
      maxAngle: number;
      
      /** Steering angle resolution in degrees */
      resolution: number;
    };
    
    /** Throttle capabilities */
    throttle: {
      /** Whether throttle control is available */
      available: boolean;
      
      /** Minimum throttle position (0-100%) */
      minPosition: number;
      
      /** Maximum throttle position (0-100%) */
      maxPosition: number;
      
      /** Throttle position resolution (%) */
      resolution: number;
    };
    
    /** Brake capabilities */
    brake: {
      /** Whether brake control is available */
      available: boolean;
      
      /** Minimum brake position (0-100%) */
      minPosition: number;
      
      /** Maximum brake position (0-100%) */
      maxPosition: number;
      
      /** Whether ABS is available */
      absAvailable: boolean;
      
      /** Whether emergency brake is available */
      emergencyBrakeAvailable: boolean;
    };
    
    /** Gear capabilities */
    gear: {
      /** Whether gear control is available */
      available: boolean;
      
      /** Available gear positions */
      availablePositions: GearPosition[];
      
      /** Whether automatic transmission is available */
      automatic: boolean;
    };
    
    /** Whether parking brake control is available */
    parkingBrake: boolean;
    
    /** Whether turn signal control is available */
    turnSignal: boolean;
  };
  
  /** Sensor capabilities */
  sensors: SensorCapability[];
  
  /** Compute capabilities */
  compute: {
    /** Compute platform identifier */
    platform: string;
    
    /** Number of CPU cores */
    cpuCores: number;
    
    /** CPU architecture */
    cpuArch: string;
    
    /** Memory in gigabytes */
    memoryGb: number;
    
    /** GPU memory in gigabytes */
    gpuMemoryGb?: number;
    
    /** Storage in gigabytes */
    storageGb: number;
    
    /** Storage type */
    storageType: string;
  };
  
  /** Power system capabilities */
  power: {
    /** Power system type */
    type: PowerSystemType;
    
    /** Energy capacity (kWh for electric, L for fuel) */
    capacity: string;
    
    /** System voltage */
    voltage: string;
    
    /** Maximum power output */
    maxPower?: string;
    
    /** Available charging/refueling types */
    chargingTypes?: string[];
    
    /** Whether backup power is available */
    backupPower: boolean;
  };
  
  /** Operating condition limits */
  operatingConditions: {
    /** Temperature range in Celsius */
    temperature: {
      min: number;
      max: number;
    };
    
    /** Humidity range in percentage */
    humidity?: {
      min: number;
      max: number;
    };
    
    /** Altitude range in meters */
    altitude?: {
      min: number;
      max: number;
    };
    
    /** Ingress protection rating (e.g., "IP67") */
    ingressProtection?: string;
    
    /** Whether dust resistance is available */
    dustResistance: boolean;
    
    /** Whether water resistance is available */
    waterResistance: boolean;
  };
  
  /** Supported operational design domains */
  supportedOdds: string[];
  
  /** Certifications held by the vehicle */
  certifications: string[];
  
  /** Safety integrity level */
  safetyIntegrityLevel?: string;
  
  /** Timestamp when these capabilities were last updated */
  timestamp: number;
}

/**
 * Power system types
 */
export enum PowerSystemType {
  BATTERY = 'battery',
  FUEL = 'fuel',
  HYBRID = 'hybrid',
  DIESEL = 'diesel',
  GASOLINE = 'gasoline'
}

/**
 * Sensor capability information
 */
export interface SensorCapability {
  /** Sensor type */
  type: SensorType;
  
  /** Sensor model */
  model: string;
  
  /** Sensor manufacturer */
  manufacturer?: string;
  
  /** Number of sensors of this type */
  count: number;
  
  /** Sensor position on the vehicle */
  position?: {
    /** X position in meters from vehicle center */
    x: number;
    
    /** Y position in meters from vehicle center */
    y: number;
    
    /** Z position in meters from ground */
    z: number;
    
    /** Roll angle in degrees */
    roll: number;
    
    /** Pitch angle in degrees */
    pitch: number;
    
    /** Yaw angle in degrees */
    yaw: number;
  };
  
  /** Sensor field of view */
  fieldOfView?: {
    /** Horizontal field of view in degrees */
    horizontal: number;
    
    /** Vertical field of view in degrees */
    vertical: number;
  };
  
  /** Sensor range */
  range?: {
    /** Minimum range in meters */
    min: number;
    
    /** Maximum range in meters */
    max: number;
  };
  
  /** Sensor resolution */
  resolution?: string;
  
  /** Sensor frame rate in Hz */
  frameRate?: number;
}

/**
 * Sensor types
 */
export enum SensorType {
  LIDAR = 'lidar',
  CAMERA = 'camera',
  RADAR = 'radar',
  ULTRASONIC = 'ultrasonic',
  IMU = 'imu',
  GPS = 'gps',
  ODOMETRY = 'odometry',
  THERMAL = 'thermal'
}

/**
 * Raw sensor data
 */
export interface SensorData {
  /** Sensor identifier */
  sensorId: string;
  
  /** Sensor type */
  type: SensorType;
  
  /** Data format */
  format: string;
  
  /** Raw data as base64 string or binary buffer */
  data: string | Buffer;
  
  /** Timestamp when the data was captured */
  timestamp: number;
  
  /** Additional metadata */
  metadata?: Record<string, any>;
}

/**
 * Diagnostic data
 */
export interface DiagnosticData {
  /** Diagnostic run identifier */
  id: string;
  
  /** Overall diagnostic result */
  result: DiagnosticResult;
  
  /** Subsystem-specific results */
  subsystems: Record<string, DiagnosticSubsystemResult>;
  
  /** Timestamp when diagnostics were run */
  timestamp: number;
}

/**
 * Diagnostic result types
 */
export enum DiagnosticResult {
  PASS = 'pass',
  FAIL = 'fail',
  WARNING = 'warning',
  INCONCLUSIVE = 'inconclusive'
}

/**
 * Diagnostic subsystem result
 */
export interface DiagnosticSubsystemResult {
  /** Result for this subsystem */
  result: DiagnosticResult;
  
  /** Human-readable message */
  message: string;
  
  /** Detailed test results */
  tests: DiagnosticTest[];
}

/**
 * Individual diagnostic test
 */
export interface DiagnosticTest {
  /** Test identifier */
  id: string;
  
  /** Test name */
  name: string;
  
  /** Test result */
  result: DiagnosticResult;
  
  /** Human-readable message */
  message: string;
  
  /** Measured value (if applicable) */
  value?: any;
  
  /** Expected value range (if applicable) */
  expectedRange?: {
    min: any;
    max: any;
  };
}
