import * as Location from 'expo-location';
import { Alert } from 'react-native';

export interface LocationData {
  latitude: number;
  longitude: number;
  accuracy: number;
  altitude: number | null;
  speed: number | null;
  heading: number | null;
  timestamp: number;
}

export interface LocationPermissionStatus {
  granted: boolean;
  canAskAgain: boolean;
  status: Location.LocationPermissionResponse['status'];
}

class LocationService {
  private static instance: LocationService;
  private isTracking: boolean = false;
  private watchId: Location.LocationSubscription | null = null;
  private lastLocation: LocationData | null = null;
  private locationCallbacks: ((location: LocationData) => void)[] = [];

  private constructor() {}

  public static getInstance(): LocationService {
    if (!LocationService.instance) {
      LocationService.instance = new LocationService();
    }
    return LocationService.instance;
  }

  /**
   * Initialize location service and request permissions
   */
  public static async initialize(): Promise<boolean> {
    try {
      const service = LocationService.getInstance();
      const permissionStatus = await service.requestPermissions();
      
      if (!permissionStatus.granted) {
        Alert.alert(
          'Location Permission Required',
          'This app needs location access to track fleet vehicles and provide navigation services.',
          [
            { text: 'Cancel', style: 'cancel' },
            { text: 'Settings', onPress: () => Location.openSettings() }
          ]
        );
        return false;
      }

      return true;
    } catch (error) {
      console.error('Failed to initialize location service:', error);
      return false;
    }
  }

  /**
   * Request location permissions
   */
  public async requestPermissions(): Promise<LocationPermissionStatus> {
    try {
      // Request foreground location permission
      const foregroundStatus = await Location.requestForegroundPermissionsAsync();
      
      if (foregroundStatus.status !== 'granted') {
        return {
          granted: false,
          canAskAgain: foregroundStatus.canAskAgain,
          status: foregroundStatus.status,
        };
      }

      // Request background location permission for drivers
      const backgroundStatus = await Location.requestBackgroundPermissionsAsync();
      
      return {
        granted: true,
        canAskAgain: backgroundStatus.canAskAgain,
        status: backgroundStatus.status,
      };
    } catch (error) {
      console.error('Error requesting location permissions:', error);
      return {
        granted: false,
        canAskAgain: false,
        status: 'denied',
      };
    }
  }

  /**
   * Get current location
   */
  public async getCurrentLocation(): Promise<LocationData | null> {
    try {
      const location = await Location.getCurrentPositionAsync({
        accuracy: Location.Accuracy.High,
        timeInterval: 1000,
        distanceInterval: 1,
      });

      const locationData: LocationData = {
        latitude: location.coords.latitude,
        longitude: location.coords.longitude,
        accuracy: location.coords.accuracy || 0,
        altitude: location.coords.altitude,
        speed: location.coords.speed,
        heading: location.coords.heading,
        timestamp: location.timestamp,
      };

      this.lastLocation = locationData;
      return locationData;
    } catch (error) {
      console.error('Error getting current location:', error);
      return null;
    }
  }

  /**
   * Start location tracking
   */
  public async startTracking(): Promise<boolean> {
    try {
      if (this.isTracking) {
        console.log('Location tracking already active');
        return true;
      }

      const permissionStatus = await this.requestPermissions();
      if (!permissionStatus.granted) {
        console.error('Location permission not granted');
        return false;
      }

      this.watchId = await Location.watchPositionAsync(
        {
          accuracy: Location.Accuracy.High,
          timeInterval: 5000, // Update every 5 seconds
          distanceInterval: 10, // Update every 10 meters
        },
        (location) => {
          const locationData: LocationData = {
            latitude: location.coords.latitude,
            longitude: location.coords.longitude,
            accuracy: location.coords.accuracy || 0,
            altitude: location.coords.altitude,
            speed: location.coords.speed,
            heading: location.coords.heading,
            timestamp: location.timestamp,
          };

          this.lastLocation = locationData;
          this.notifyLocationCallbacks(locationData);
        }
      );

      this.isTracking = true;
      console.log('Location tracking started');
      return true;
    } catch (error) {
      console.error('Error starting location tracking:', error);
      return false;
    }
  }

  /**
   * Stop location tracking
   */
  public stopTracking(): void {
    try {
      if (this.watchId) {
        this.watchId.remove();
        this.watchId = null;
      }
      this.isTracking = false;
      console.log('Location tracking stopped');
    } catch (error) {
      console.error('Error stopping location tracking:', error);
    }
  }

  /**
   * Check if location tracking is active
   */
  public isLocationTrackingActive(): boolean {
    return this.isTracking;
  }

  /**
   * Get last known location
   */
  public getLastLocation(): LocationData | null {
    return this.lastLocation;
  }

  /**
   * Add location update callback
   */
  public addLocationCallback(callback: (location: LocationData) => void): void {
    this.locationCallbacks.push(callback);
  }

  /**
   * Remove location update callback
   */
  public removeLocationCallback(callback: (location: LocationData) => void): void {
    const index = this.locationCallbacks.indexOf(callback);
    if (index > -1) {
      this.locationCallbacks.splice(index, 1);
    }
  }

  /**
   * Notify all location callbacks
   */
  private notifyLocationCallbacks(location: LocationData): void {
    this.locationCallbacks.forEach(callback => {
      try {
        callback(location);
      } catch (error) {
        console.error('Error in location callback:', error);
      }
    });
  }

  /**
   * Calculate distance between two points
   */
  public calculateDistance(
    lat1: number,
    lon1: number,
    lat2: number,
    lon2: number
  ): number {
    const R = 6371; // Earth's radius in kilometers
    const dLat = this.deg2rad(lat2 - lat1);
    const dLon = this.deg2rad(lon2 - lon1);
    const a =
      Math.sin(dLat / 2) * Math.sin(dLat / 2) +
      Math.cos(this.deg2rad(lat1)) *
        Math.cos(this.deg2rad(lat2)) *
        Math.sin(dLon / 2) *
        Math.sin(dLon / 2);
    const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
    const distance = R * c; // Distance in kilometers
    return distance;
  }

  /**
   * Convert degrees to radians
   */
  private deg2rad(deg: number): number {
    return deg * (Math.PI / 180);
  }

  /**
   * Get location permission status
   */
  public async getPermissionStatus(): Promise<LocationPermissionStatus> {
    try {
      const foregroundStatus = await Location.getForegroundPermissionsAsync();
      const backgroundStatus = await Location.getBackgroundPermissionsAsync();
      
      return {
        granted: foregroundStatus.status === 'granted',
        canAskAgain: foregroundStatus.canAskAgain,
        status: foregroundStatus.status,
      };
    } catch (error) {
      console.error('Error getting permission status:', error);
      return {
        granted: false,
        canAskAgain: false,
        status: 'denied',
      };
    }
  }

  /**
   * Check if location services are enabled
   */
  public async isLocationEnabled(): Promise<boolean> {
    try {
      const enabled = await Location.hasServicesEnabledAsync();
      return enabled;
    } catch (error) {
      console.error('Error checking location services:', error);
      return false;
    }
  }

  /**
   * Get location accuracy level
   */
  public getLocationAccuracy(): Location.Accuracy {
    return Location.Accuracy.High;
  }

  /**
   * Format location data for display
   */
  public formatLocation(location: LocationData): string {
    return `${location.latitude.toFixed(6)}, ${location.longitude.toFixed(6)}`;
  }

  /**
   * Get location address (reverse geocoding)
   */
  public async getAddressFromLocation(latitude: number, longitude: number): Promise<string | null> {
    try {
      const addresses = await Location.reverseGeocodeAsync({
        latitude,
        longitude,
      });

      if (addresses.length > 0) {
        const address = addresses[0];
        return `${address.street || ''} ${address.city || ''} ${address.region || ''} ${address.country || ''}`.trim();
      }

      return null;
    } catch (error) {
      console.error('Error getting address from location:', error);
      return null;
    }
  }
}

export default LocationService;
