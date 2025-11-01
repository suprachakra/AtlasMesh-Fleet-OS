import { useState, useEffect, useCallback } from 'react';
import { useWebSocket } from './useWebSocket';
import { VehicleTelemetry, FleetEvent } from '../services/websocketService';

export interface Vehicle {
  id: string;
  licensePlate: string;
  model: string;
  status: 'active' | 'maintenance' | 'inactive' | 'emergency';
  location: {
    latitude: number;
    longitude: number;
    altitude?: number;
    heading?: number;
    speed?: number;
  };
  batteryLevel: number;
  fuelLevel: number;
  healthScore: number;
  lastSeen: string;
  driver?: string;
  alerts: Array<{
    type: string;
    severity: 'low' | 'medium' | 'high' | 'critical';
    message: string;
    timestamp: number;
  }>;
}

export interface FleetStatus {
  totalVehicles: number;
  activeVehicles: number;
  maintenanceVehicles: number;
  emergencyVehicles: number;
  averageHealthScore: number;
  lastUpdated: number;
}

export interface UseRealTimeVehiclesOptions {
  autoRefresh?: boolean;
  refreshInterval?: number;
  filterByStatus?: string[];
  filterByFleet?: string[];
}

export interface UseRealTimeVehiclesReturn {
  vehicles: Vehicle[];
  fleetStatus: FleetStatus;
  isLoading: boolean;
  error: string | null;
  refreshVehicles: () => void;
  getVehicleById: (id: string) => Vehicle | undefined;
  getVehiclesByStatus: (status: string) => Vehicle[];
  getVehiclesByFleet: (fleetId: string) => Vehicle[];
  getEmergencyVehicles: () => Vehicle[];
  getMaintenanceVehicles: () => Vehicle[];
  getActiveVehicles: () => Vehicle[];
  subscribeToVehicle: (vehicleId: string, callback: (vehicle: Vehicle) => void) => void;
  unsubscribeFromVehicle: (vehicleId: string, callback: (vehicle: Vehicle) => void) => void;
}

/**
 * React hook for real-time vehicle monitoring
 * Provides live updates of vehicle status, location, and telemetry
 */
export const useRealTimeVehicles = (options: UseRealTimeVehiclesOptions = {}): UseRealTimeVehiclesReturn => {
  const {
    autoRefresh = true,
    refreshInterval = 5000,
    filterByStatus = [],
    filterByFleet = [],
  } = options;

  const [vehicles, setVehicles] = useState<Vehicle[]>([]);
  const [fleetStatus, setFleetStatus] = useState<FleetStatus>({
    totalVehicles: 0,
    activeVehicles: 0,
    maintenanceVehicles: 0,
    emergencyVehicles: 0,
    averageHealthScore: 0,
    lastUpdated: 0,
  });
  const [isLoading, setIsLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);

  const { isConnected, subscribe, unsubscribe, requestFleetStatus } = useWebSocket();

  // Load initial vehicle data
  useEffect(() => {
    loadInitialData();
  }, []);

  // Set up real-time subscriptions
  useEffect(() => {
    if (isConnected) {
      setupSubscriptions();
      requestFleetStatus();
    }
  }, [isConnected]);

  // Auto-refresh setup
  useEffect(() => {
    if (!autoRefresh) return;

    const interval = setInterval(() => {
      if (isConnected) {
        requestFleetStatus();
      }
    }, refreshInterval);

    return () => clearInterval(interval);
  }, [autoRefresh, refreshInterval, isConnected]);

  // Load initial data
  const loadInitialData = async () => {
    try {
      setIsLoading(true);
      setError(null);
      
      // Fetch vehicles from Fleet Manager API
      const response = await fetch('/api/v1/fleet/vehicles', {
        method: 'GET',
        headers: {
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${localStorage.getItem('authToken')}`,
        },
      });

      if (!response.ok) {
        throw new Error(`Failed to fetch vehicles: ${response.statusText}`);
      }

      const data = await response.json();
      const apiVehicles: Vehicle[] = data.vehicles.map((v: any) => ({
        id: v.vehicle_id,
        licensePlate: v.license_plate || v.asset_tag,
        model: `${v.manufacturer} ${v.model}`,
        status: mapVehicleStatus(v.operational_status),
        location: {
          latitude: v.current_location?.latitude || 0,
          longitude: v.current_location?.longitude || 0,
          altitude: v.current_location?.altitude,
          heading: v.current_location?.heading,
          speed: v.current_location?.speed,
        },
        batteryLevel: v.telemetry?.battery_level || 0,
        fuelLevel: v.telemetry?.fuel_level || 0,
        healthScore: v.health_score || 0,
        lastSeen: v.last_seen || new Date().toISOString(),
        driver: v.current_driver,
        alerts: v.alerts || [],
      }));

      setVehicles(apiVehicles);
      updateFleetStatus(apiVehicles);
    } catch (err) {
      console.error('Failed to load vehicles from API, falling back to mock data:', err);
      
      // Fallback to mock data if API fails
      const mockVehicles: Vehicle[] = [
        {
          id: '1',
          licensePlate: 'AD-001',
          model: 'Tesla Model 3',
          status: 'active',
          location: { latitude: 24.4539, longitude: 54.3773, speed: 45 },
          batteryLevel: 85,
          fuelLevel: 0,
          healthScore: 95,
          lastSeen: '2 minutes ago',
          driver: 'Ahmed Al Mansouri',
          alerts: [],
        },
        {
          id: '2',
          licensePlate: 'AD-002',
          model: 'BMW iX',
          status: 'maintenance',
          location: { latitude: 24.4539, longitude: 54.3773 },
          batteryLevel: 45,
          fuelLevel: 0,
          healthScore: 78,
          lastSeen: '1 hour ago',
          alerts: [
            {
              type: 'maintenance',
              severity: 'medium',
              message: 'Battery health declining',
              timestamp: Date.now() - 3600000,
            },
          ],
        },
        {
          id: '3',
          licensePlate: 'AD-003',
          model: 'Mercedes EQS',
          status: 'active',
          location: { latitude: 24.4539, longitude: 54.3773, speed: 30 },
          batteryLevel: 92,
          fuelLevel: 0,
          healthScore: 98,
          lastSeen: '5 minutes ago',
          driver: 'Fatima Al Zahra',
          alerts: [],
        },
      ];

      setVehicles(mockVehicles);
      updateFleetStatus(mockVehicles);
    } finally {
      setIsLoading(false);
    }
  };

  // Map API status to UI status
  const mapVehicleStatus = (apiStatus: string): Vehicle['status'] => {
    switch (apiStatus?.toLowerCase()) {
      case 'online':
      case 'active':
        return 'active';
      case 'maintenance':
      case 'servicing':
        return 'maintenance';
      case 'emergency':
      case 'critical':
        return 'emergency';
      case 'offline':
      case 'inactive':
      default:
        return 'inactive';
    }
  };

  // Set up WebSocket subscriptions
  const setupSubscriptions = () => {
    // Subscribe to vehicle telemetry updates
    subscribe('vehicle_telemetry', (message) => {
      const telemetry = message.data as VehicleTelemetry;
      updateVehicleFromTelemetry(telemetry);
    });

    // Subscribe to fleet events
    subscribe('fleet_event', (message) => {
      const event = message.data as FleetEvent;
      handleFleetEvent(event);
    });

    // Subscribe to emergency alerts
    subscribe('emergency_alert', (message) => {
      const alert = message.data;
      handleEmergencyAlert(alert);
    });

    // Subscribe to vehicle status changes
    subscribe('vehicle_status_change', (message) => {
      const { vehicleId, status, data } = message.data;
      updateVehicleStatus(vehicleId, status, data);
    });
  };

  // Update vehicle from telemetry data
  const updateVehicleFromTelemetry = (telemetry: VehicleTelemetry) => {
    setVehicles(prevVehicles => {
      const updatedVehicles = prevVehicles.map(vehicle => {
        if (vehicle.id === telemetry.vehicleId) {
          return {
            ...vehicle,
            location: {
              latitude: telemetry.location.latitude,
              longitude: telemetry.location.longitude,
              altitude: telemetry.location.altitude,
              heading: telemetry.location.heading,
              speed: telemetry.location.speed,
            },
            batteryLevel: telemetry.status.batteryLevel,
            fuelLevel: telemetry.status.fuelLevel,
            healthScore: telemetry.status.healthScore,
            lastSeen: new Date(telemetry.timestamp).toLocaleString(),
            alerts: telemetry.alerts,
          };
        }
        return vehicle;
      });

      updateFleetStatus(updatedVehicles);
      return updatedVehicles;
    });
  };

  // Handle fleet events
  const handleFleetEvent = (event: FleetEvent) => {
    console.log('Fleet event received:', event);
    
    // Update vehicle status based on event
    if (event.type === 'vehicle_status_change') {
      updateVehicleStatus(event.vehicleId, event.data.status, event.data);
    }
  };

  // Handle emergency alerts
  const handleEmergencyAlert = (alert: any) => {
    console.log('Emergency alert received:', alert);
    
    setVehicles(prevVehicles => {
      const updatedVehicles = prevVehicles.map(vehicle => {
        if (vehicle.id === alert.vehicleId) {
          return {
            ...vehicle,
            status: 'emergency',
            alerts: [
              ...vehicle.alerts,
              {
                type: 'emergency',
                severity: 'critical',
                message: alert.message,
                timestamp: Date.now(),
              },
            ],
          };
        }
        return vehicle;
      });

      updateFleetStatus(updatedVehicles);
      return updatedVehicles;
    });
  };

  // Update vehicle status
  const updateVehicleStatus = (vehicleId: string, status: string, data: any) => {
    setVehicles(prevVehicles => {
      const updatedVehicles = prevVehicles.map(vehicle => {
        if (vehicle.id === vehicleId) {
          return {
            ...vehicle,
            status: status as Vehicle['status'],
            ...data,
          };
        }
        return vehicle;
      });

      updateFleetStatus(updatedVehicles);
      return updatedVehicles;
    });
  };

  // Update fleet status
  const updateFleetStatus = (vehicles: Vehicle[]) => {
    const totalVehicles = vehicles.length;
    const activeVehicles = vehicles.filter(v => v.status === 'active').length;
    const maintenanceVehicles = vehicles.filter(v => v.status === 'maintenance').length;
    const emergencyVehicles = vehicles.filter(v => v.status === 'emergency').length;
    const averageHealthScore = vehicles.length > 0 
      ? vehicles.reduce((sum, v) => sum + v.healthScore, 0) / vehicles.length 
      : 0;

    setFleetStatus({
      totalVehicles,
      activeVehicles,
      maintenanceVehicles,
      emergencyVehicles,
      averageHealthScore: Math.round(averageHealthScore),
      lastUpdated: Date.now(),
    });
  };

  // Refresh vehicles manually
  const refreshVehicles = useCallback(() => {
    loadInitialData();
  }, []);

  // Get vehicle by ID
  const getVehicleById = useCallback((id: string): Vehicle | undefined => {
    return vehicles.find(vehicle => vehicle.id === id);
  }, [vehicles]);

  // Get vehicles by status
  const getVehiclesByStatus = useCallback((status: string): Vehicle[] => {
    return vehicles.filter(vehicle => vehicle.status === status);
  }, [vehicles]);

  // Get vehicles by fleet (placeholder - would need fleet ID in vehicle data)
  const getVehiclesByFleet = useCallback((fleetId: string): Vehicle[] => {
    // In a real implementation, vehicles would have a fleetId property
    return vehicles;
  }, [vehicles]);

  // Get emergency vehicles
  const getEmergencyVehicles = useCallback((): Vehicle[] => {
    return vehicles.filter(vehicle => vehicle.status === 'emergency');
  }, [vehicles]);

  // Get maintenance vehicles
  const getMaintenanceVehicles = useCallback((): Vehicle[] => {
    return vehicles.filter(vehicle => vehicle.status === 'maintenance');
  }, [vehicles]);

  // Get active vehicles
  const getActiveVehicles = useCallback((): Vehicle[] => {
    return vehicles.filter(vehicle => vehicle.status === 'active');
  }, [vehicles]);

  // Subscribe to specific vehicle updates
  const subscribeToVehicle = useCallback((vehicleId: string, callback: (vehicle: Vehicle) => void): void => {
    subscribe(`vehicle_${vehicleId}`, (message) => {
      const vehicle = message.data as Vehicle;
      callback(vehicle);
    });
  }, [subscribe]);

  // Unsubscribe from vehicle updates
  const unsubscribeFromVehicle = useCallback((vehicleId: string, callback: (vehicle: Vehicle) => void): void => {
    unsubscribe(`vehicle_${vehicleId}`, callback);
  }, [unsubscribe]);

  return {
    vehicles,
    fleetStatus,
    isLoading,
    error,
    refreshVehicles,
    getVehicleById,
    getVehiclesByStatus,
    getVehiclesByFleet,
    getEmergencyVehicles,
    getMaintenanceVehicles,
    getActiveVehicles,
    subscribeToVehicle,
    unsubscribeFromVehicle,
  };
};

export default useRealTimeVehicles;