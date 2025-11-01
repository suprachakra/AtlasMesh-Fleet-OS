/**
 * IVehicleAdapter.ts
 * Core interface for vehicle adapters in the AtlasMesh Fleet OS
 * 
 * This interface defines the contract that all vehicle adapters must implement
 * to provide a consistent abstraction layer between the Fleet OS and different
 * vehicle platforms, enabling the vehicle-agnostic promise of AtlasMesh.
 */

import { 
  VehicleCommand, 
  VehicleState, 
  VehicleCapabilities, 
  VehicleHealth,
  VehicleTelemetry,
  AdapterStatus,
  CommandResult,
  VehicleError,
  SensorData,
  DiagnosticData
} from '../types/vehicle';

/**
 * Core interface that all vehicle adapters must implement
 */
export interface IVehicleAdapter {
  /**
   * Initialize the adapter with vehicle-specific configuration
   * @param config Vehicle-specific configuration parameters
   * @returns Promise resolving to adapter status
   */
  initialize(config: Record<string, any>): Promise<AdapterStatus>;
  
  /**
   * Connect to the vehicle's drive-by-wire system
   * @returns Promise resolving to adapter status
   */
  connect(): Promise<AdapterStatus>;
  
  /**
   * Disconnect from the vehicle's drive-by-wire system
   * @returns Promise resolving to adapter status
   */
  disconnect(): Promise<AdapterStatus>;
  
  /**
   * Get the current connection status of the adapter
   * @returns Current adapter status
   */
  getStatus(): AdapterStatus;
  
  /**
   * Get the capabilities of the connected vehicle
   * @returns Promise resolving to vehicle capabilities
   */
  getCapabilities(): Promise<VehicleCapabilities>;
  
  /**
   * Get the current state of the vehicle
   * @returns Promise resolving to vehicle state
   */
  getState(): Promise<VehicleState>;
  
  /**
   * Get the current health status of the vehicle
   * @returns Promise resolving to vehicle health
   */
  getHealth(): Promise<VehicleHealth>;
  
  /**
   * Get the latest telemetry data from the vehicle
   * @returns Promise resolving to vehicle telemetry
   */
  getTelemetry(): Promise<VehicleTelemetry>;
  
  /**
   * Send a command to the vehicle
   * @param command The command to send
   * @returns Promise resolving to command result
   */
  sendCommand(command: VehicleCommand): Promise<CommandResult>;
  
  /**
   * Register a callback for state changes
   * @param callback Function to call when vehicle state changes
   * @returns Subscription ID used to unregister
   */
  onStateChange(callback: (state: VehicleState) => void): string;
  
  /**
   * Register a callback for health changes
   * @param callback Function to call when vehicle health changes
   * @returns Subscription ID used to unregister
   */
  onHealthChange(callback: (health: VehicleHealth) => void): string;
  
  /**
   * Register a callback for telemetry updates
   * @param callback Function to call when new telemetry is available
   * @returns Subscription ID used to unregister
   */
  onTelemetry(callback: (telemetry: VehicleTelemetry) => void): string;
  
  /**
   * Register a callback for errors
   * @param callback Function to call when an error occurs
   * @returns Subscription ID used to unregister
   */
  onError(callback: (error: VehicleError) => void): string;
  
  /**
   * Unregister a previously registered callback
   * @param subscriptionId The subscription ID returned from on* methods
   * @returns True if successfully unregistered
   */
  unregister(subscriptionId: string): boolean;
  
  /**
   * Get raw sensor data from the vehicle
   * @param sensorId Optional specific sensor ID
   * @returns Promise resolving to sensor data
   */
  getSensorData(sensorId?: string): Promise<SensorData>;
  
  /**
   * Run diagnostics on the vehicle or specific subsystem
   * @param subsystem Optional subsystem to diagnose
   * @returns Promise resolving to diagnostic data
   */
  runDiagnostics(subsystem?: string): Promise<DiagnosticData>;
  
  /**
   * Update adapter firmware/configuration
   * @param updatePackage Update package data or URL
   * @returns Promise resolving to update result
   */
  updateAdapter(updatePackage: string | Buffer): Promise<CommandResult>;
  
  /**
   * Get the adapter version information
   * @returns Version string
   */
  getVersion(): string;
}

/**
 * Factory interface for creating vehicle adapters
 */
export interface IVehicleAdapterFactory {
  /**
   * Create a new vehicle adapter instance
   * @param vehicleType The type of vehicle to create an adapter for
   * @param options Additional options for adapter creation
   * @returns Promise resolving to a vehicle adapter instance
   */
  createAdapter(vehicleType: string, options?: Record<string, any>): Promise<IVehicleAdapter>;
  
  /**
   * Get the list of supported vehicle types
   * @returns Array of supported vehicle type strings
   */
  getSupportedVehicleTypes(): string[];
}
