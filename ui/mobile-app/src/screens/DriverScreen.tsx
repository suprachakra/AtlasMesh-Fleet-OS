import React, { useState, useEffect } from 'react';
import {
  View,
  StyleSheet,
  ScrollView,
  Alert,
  RefreshControl,
} from 'react-native';
import {
  Card,
  Title,
  Paragraph,
  Button,
  List,
  Chip,
  Surface,
  Text,
  IconButton,
  ActivityIndicator,
  Switch,
  ProgressBar,
} from 'react-native-paper';
import { Ionicons } from '@expo/vector-icons';
import MapView, { Marker, Polyline } from 'react-native-maps';
import { theme } from '../theme/theme';
import { AuthService } from '../services/AuthService';

interface Trip {
  id: string;
  status: 'assigned' | 'started' | 'in_progress' | 'completed' | 'cancelled';
  pickupLocation: {
    latitude: number;
    longitude: number;
    address: string;
  };
  dropoffLocation: {
    latitude: number;
    longitude: number;
    address: string;
  };
  passenger?: {
    name: string;
    phone: string;
    rating: number;
  };
  scheduledTime: Date;
  estimatedDuration: number; // minutes
  distance: number; // km
  fare?: number;
  specialInstructions?: string;
  route?: {
    latitude: number;
    longitude: number;
  }[];
}

interface DriverStatus {
  isOnline: boolean;
  isAvailable: boolean;
  currentTrip?: Trip;
  todayStats: {
    tripsCompleted: number;
    hoursWorked: number;
    earnings: number;
    rating: number;
  };
  vehicleStatus: {
    fuelLevel: number;
    batteryLevel?: number;
    mileage: number;
    lastMaintenance: Date;
  };
}

const DriverScreen: React.FC = () => {
  const [driverStatus, setDriverStatus] = useState<DriverStatus | null>(null);
  const [upcomingTrips, setUpcomingTrips] = useState<Trip[]>([]);
  const [isLoading, setIsLoading] = useState(true);
  const [isRefreshing, setIsRefreshing] = useState(false);
  const [mapRegion, setMapRegion] = useState({
    latitude: 25.2048,
    longitude: 55.2708,
    latitudeDelta: 0.0922,
    longitudeDelta: 0.0421,
  });

  useEffect(() => {
    loadDriverData();
  }, []);

  const loadDriverData = async () => {
    try {
      setIsLoading(true);
      
      // Load driver status
      const statusResponse = await AuthService.makeAuthenticatedRequest(
        'https://api.atlasmesh.ae/api/v1/driver/status'
      );
      const status = await statusResponse.json();
      setDriverStatus(status);

      // Load upcoming trips
      const tripsResponse = await AuthService.makeAuthenticatedRequest(
        'https://api.atlasmesh.ae/api/v1/driver/trips/upcoming'
      );
      const trips = await tripsResponse.json();
      setUpcomingTrips(trips);
      
    } catch (error) {
      console.error('Failed to load driver data:', error);
      // Load mock data as fallback
      loadMockData();
    } finally {
      setIsLoading(false);
      setIsRefreshing(false);
    }
  };

  const loadMockData = () => {
    setDriverStatus({
      isOnline: true,
      isAvailable: true,
      todayStats: {
        tripsCompleted: 8,
        hoursWorked: 6.5,
        earnings: 450,
        rating: 4.8,
      },
      vehicleStatus: {
        fuelLevel: 75,
        batteryLevel: 85,
        mileage: 45230,
        lastMaintenance: new Date(Date.now() - 15 * 24 * 60 * 60 * 1000),
      },
    });

    setUpcomingTrips([
      {
        id: '1',
        status: 'assigned',
        pickupLocation: {
          latitude: 25.2048,
          longitude: 55.2708,
          address: 'Dubai Mall, Downtown Dubai',
        },
        dropoffLocation: {
          latitude: 25.1972,
          longitude: 55.2744,
          address: 'Burj Khalifa, Downtown Dubai',
        },
        passenger: {
          name: 'Sarah Ahmed',
          phone: '+971501234567',
          rating: 4.9,
        },
        scheduledTime: new Date(Date.now() + 15 * 60 * 1000),
        estimatedDuration: 12,
        distance: 2.5,
        fare: 25,
        specialInstructions: 'Please wait at main entrance',
      },
      {
        id: '2',
        status: 'assigned',
        pickupLocation: {
          latitude: 25.2285,
          longitude: 55.3273,
          address: 'Dubai International Airport',
        },
        dropoffLocation: {
          latitude: 25.2048,
          longitude: 55.2708,
          address: 'Business Bay Metro Station',
        },
        passenger: {
          name: 'Mohammed Ali',
          phone: '+971507654321',
          rating: 4.7,
        },
        scheduledTime: new Date(Date.now() + 45 * 60 * 1000),
        estimatedDuration: 25,
        distance: 15.2,
        fare: 65,
      },
    ]);
  };

  const onRefresh = () => {
    setIsRefreshing(true);
    loadDriverData();
  };

  const toggleOnlineStatus = async () => {
    if (!driverStatus) return;

    try {
      const newStatus = !driverStatus.isOnline;
      
      const response = await AuthService.makeAuthenticatedRequest(
        'https://api.atlasmesh.ae/api/v1/driver/status',
        {
          method: 'PUT',
          body: JSON.stringify({ isOnline: newStatus }),
        }
      );

      if (response.ok) {
        setDriverStatus({
          ...driverStatus,
          isOnline: newStatus,
          isAvailable: newStatus,
        });
      }
    } catch (error) {
      console.error('Failed to update status:', error);
      Alert.alert('Error', 'Failed to update online status');
    }
  };

  const acceptTrip = async (tripId: string) => {
    try {
      const response = await AuthService.makeAuthenticatedRequest(
        `https://api.atlasmesh.ae/api/v1/driver/trips/${tripId}/accept`,
        { method: 'POST' }
      );

      if (response.ok) {
        Alert.alert('Trip Accepted', 'You have accepted the trip. Navigate to pickup location.');
        loadDriverData();
      }
    } catch (error) {
      console.error('Failed to accept trip:', error);
      Alert.alert('Error', 'Failed to accept trip');
    }
  };

  const startTrip = async (tripId: string) => {
    try {
      const response = await AuthService.makeAuthenticatedRequest(
        `https://api.atlasmesh.ae/api/v1/driver/trips/${tripId}/start`,
        { method: 'POST' }
      );

      if (response.ok) {
        Alert.alert('Trip Started', 'Trip has been started. Drive safely!');
        loadDriverData();
      }
    } catch (error) {
      console.error('Failed to start trip:', error);
      Alert.alert('Error', 'Failed to start trip');
    }
  };

  const completeTrip = async (tripId: string) => {
    Alert.alert(
      'Complete Trip',
      'Are you sure you want to complete this trip?',
      [
        { text: 'Cancel', style: 'cancel' },
        {
          text: 'Complete',
          onPress: async () => {
            try {
              const response = await AuthService.makeAuthenticatedRequest(
                `https://api.atlasmesh.ae/api/v1/driver/trips/${tripId}/complete`,
                { method: 'POST' }
              );

              if (response.ok) {
                Alert.alert('Trip Completed', 'Trip has been completed successfully!');
                loadDriverData();
              }
            } catch (error) {
              console.error('Failed to complete trip:', error);
              Alert.alert('Error', 'Failed to complete trip');
            }
          }
        }
      ]
    );
  };

  const getStatusColor = (status: string) => {
    switch (status) {
      case 'assigned': return theme.colors.warning;
      case 'started': return theme.colors.info;
      case 'in_progress': return theme.colors.primary;
      case 'completed': return theme.colors.success;
      case 'cancelled': return theme.colors.error;
      default: return theme.colors.text;
    }
  };

  const getTripActions = (trip: Trip) => {
    switch (trip.status) {
      case 'assigned':
        return (
          <Button
            mode="contained"
            onPress={() => acceptTrip(trip.id)}
            style={styles.actionButton}
          >
            Accept Trip
          </Button>
        );
      case 'started':
        return (
          <Button
            mode="contained"
            onPress={() => startTrip(trip.id)}
            style={styles.actionButton}
          >
            Start Trip
          </Button>
        );
      case 'in_progress':
        return (
          <Button
            mode="contained"
            onPress={() => completeTrip(trip.id)}
            style={styles.actionButton}
          >
            Complete Trip
          </Button>
        );
      default:
        return null;
    }
  };

  if (isLoading) {
    return (
      <View style={styles.loadingContainer}>
        <ActivityIndicator size="large" color={theme.colors.primary} />
        <Text style={styles.loadingText}>Loading driver dashboard...</Text>
      </View>
    );
  }

  if (!driverStatus) {
    return (
      <View style={styles.errorContainer}>
        <Text style={styles.errorText}>Failed to load driver data</Text>
        <Button onPress={loadDriverData}>Retry</Button>
      </View>
    );
  }

  return (
    <View style={styles.container}>
      <ScrollView
        style={styles.scrollView}
        showsVerticalScrollIndicator={false}
        refreshControl={
          <RefreshControl refreshing={isRefreshing} onRefresh={onRefresh} />
        }
      >
        {/* Status Card */}
        <Card style={styles.card}>
          <Card.Content>
            <View style={styles.statusHeader}>
              <Title>Driver Status</Title>
              <View style={styles.onlineToggle}>
                <Text style={styles.onlineLabel}>
                  {driverStatus.isOnline ? 'Online' : 'Offline'}
                </Text>
                <Switch
                  value={driverStatus.isOnline}
                  onValueChange={toggleOnlineStatus}
                  color={theme.colors.success}
                />
              </View>
            </View>
            
            <View style={styles.statusIndicators}>
              <Chip
                style={[
                  styles.statusChip,
                  { backgroundColor: driverStatus.isOnline ? theme.colors.success : theme.colors.error }
                ]}
                textStyle={styles.statusChipText}
              >
                {driverStatus.isOnline ? 'ONLINE' : 'OFFLINE'}
              </Chip>
              <Chip
                style={[
                  styles.statusChip,
                  { backgroundColor: driverStatus.isAvailable ? theme.colors.primary : theme.colors.warning }
                ]}
                textStyle={styles.statusChipText}
              >
                {driverStatus.isAvailable ? 'AVAILABLE' : 'BUSY'}
              </Chip>
            </View>
          </Card.Content>
        </Card>

        {/* Today's Stats */}
        <Card style={styles.card}>
          <Card.Content>
            <Title>Today's Performance</Title>
            <View style={styles.statsGrid}>
              <View style={styles.statItem}>
                <Text style={styles.statValue}>{driverStatus.todayStats.tripsCompleted}</Text>
                <Text style={styles.statLabel}>Trips</Text>
              </View>
              <View style={styles.statItem}>
                <Text style={styles.statValue}>{driverStatus.todayStats.hoursWorked}h</Text>
                <Text style={styles.statLabel}>Hours</Text>
              </View>
              <View style={styles.statItem}>
                <Text style={styles.statValue}>AED {driverStatus.todayStats.earnings}</Text>
                <Text style={styles.statLabel}>Earnings</Text>
              </View>
              <View style={styles.statItem}>
                <Text style={styles.statValue}>⭐ {driverStatus.todayStats.rating}</Text>
                <Text style={styles.statLabel}>Rating</Text>
              </View>
            </View>
          </Card.Content>
        </Card>

        {/* Vehicle Status */}
        <Card style={styles.card}>
          <Card.Content>
            <Title>Vehicle Status</Title>
            <View style={styles.vehicleStatus}>
              <View style={styles.statusRow}>
                <Text style={styles.statusLabel}>Fuel Level</Text>
                <View style={styles.progressContainer}>
                  <ProgressBar
                    progress={driverStatus.vehicleStatus.fuelLevel / 100}
                    color={driverStatus.vehicleStatus.fuelLevel > 25 ? theme.colors.success : theme.colors.error}
                    style={styles.progressBar}
                  />
                  <Text style={styles.progressText}>{driverStatus.vehicleStatus.fuelLevel}%</Text>
                </View>
              </View>
              
              {driverStatus.vehicleStatus.batteryLevel && (
                <View style={styles.statusRow}>
                  <Text style={styles.statusLabel}>Battery Level</Text>
                  <View style={styles.progressContainer}>
                    <ProgressBar
                      progress={driverStatus.vehicleStatus.batteryLevel / 100}
                      color={driverStatus.vehicleStatus.batteryLevel > 20 ? theme.colors.primary : theme.colors.warning}
                      style={styles.progressBar}
                    />
                    <Text style={styles.progressText}>{driverStatus.vehicleStatus.batteryLevel}%</Text>
                  </View>
                </View>
              )}
              
              <View style={styles.statusRow}>
                <Text style={styles.statusLabel}>Mileage</Text>
                <Text style={styles.statusValue}>{driverStatus.vehicleStatus.mileage.toLocaleString()} km</Text>
              </View>
            </View>
          </Card.Content>
        </Card>

        {/* Current/Upcoming Trips */}
        <Card style={styles.card}>
          <Card.Content>
            <Title>Trips</Title>
            {upcomingTrips.length === 0 ? (
              <Paragraph style={styles.noTripsText}>
                No upcoming trips. Stay online to receive trip requests.
              </Paragraph>
            ) : (
              upcomingTrips.map((trip) => (
                <Surface key={trip.id} style={styles.tripCard}>
                  <View style={styles.tripHeader}>
                    <View style={styles.tripInfo}>
                      <Text style={styles.tripId}>Trip #{trip.id}</Text>
                      <Chip
                        style={[styles.tripStatusChip, { backgroundColor: getStatusColor(trip.status) }]}
                        textStyle={styles.tripStatusText}
                      >
                        {trip.status.toUpperCase()}
                      </Chip>
                    </View>
                    <Text style={styles.tripTime}>
                      {trip.scheduledTime.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })}
                    </Text>
                  </View>

                  <View style={styles.tripDetails}>
                    <View style={styles.locationRow}>
                      <Ionicons name="radio-button-on" size={16} color={theme.colors.success} />
                      <Text style={styles.locationText}>{trip.pickupLocation.address}</Text>
                    </View>
                    <View style={styles.locationRow}>
                      <Ionicons name="location" size={16} color={theme.colors.error} />
                      <Text style={styles.locationText}>{trip.dropoffLocation.address}</Text>
                    </View>
                  </View>

                  {trip.passenger && (
                    <View style={styles.passengerInfo}>
                      <Ionicons name="person" size={16} color={theme.colors.primary} />
                      <Text style={styles.passengerName}>{trip.passenger.name}</Text>
                      <Text style={styles.passengerRating}>⭐ {trip.passenger.rating}</Text>
                    </View>
                  )}

                  <View style={styles.tripMeta}>
                    <Text style={styles.metaText}>{trip.distance} km • {trip.estimatedDuration} min</Text>
                    {trip.fare && <Text style={styles.fareText}>AED {trip.fare}</Text>}
                  </View>

                  {trip.specialInstructions && (
                    <Text style={styles.specialInstructions}>
                      📝 {trip.specialInstructions}
                    </Text>
                  )}

                  <View style={styles.tripActions}>
                    {getTripActions(trip)}
                    <Button
                      mode="outlined"
                      onPress={() => {
                        // Navigate to trip details or map
                        Alert.alert('Navigation', 'Opening navigation to pickup location...');
                      }}
                      style={styles.navigationButton}
                    >
                      Navigate
                    </Button>
                  </View>
                </Surface>
              ))
            )}
          </Card.Content>
        </Card>

        {/* Map View */}
        {upcomingTrips.length > 0 && (
          <Card style={styles.card}>
            <Card.Content>
              <Title>Trip Map</Title>
              <View style={styles.mapContainer}>
                <MapView
                  style={styles.map}
                  region={mapRegion}
                  onRegionChangeComplete={setMapRegion}
                >
                  {upcomingTrips.map((trip) => (
                    <React.Fragment key={trip.id}>
                      <Marker
                        coordinate={trip.pickupLocation}
                        title="Pickup"
                        description={trip.pickupLocation.address}
                        pinColor={theme.colors.success}
                      />
                      <Marker
                        coordinate={trip.dropoffLocation}
                        title="Dropoff"
                        description={trip.dropoffLocation.address}
                        pinColor={theme.colors.error}
                      />
                      {trip.route && (
                        <Polyline
                          coordinates={trip.route}
                          strokeColor={theme.colors.primary}
                          strokeWidth={3}
                        />
                      )}
                    </React.Fragment>
                  ))}
                </MapView>
              </View>
            </Card.Content>
          </Card>
        )}
      </ScrollView>
    </View>
  );
};

const styles = StyleSheet.create({
  container: {
    flex: 1,
    backgroundColor: theme.colors.background,
  },
  loadingContainer: {
    flex: 1,
    justifyContent: 'center',
    alignItems: 'center',
    backgroundColor: theme.colors.background,
  },
  loadingText: {
    marginTop: theme.spacing.md,
    color: theme.colors.text,
  },
  errorContainer: {
    flex: 1,
    justifyContent: 'center',
    alignItems: 'center',
    backgroundColor: theme.colors.background,
    padding: theme.spacing.md,
  },
  errorText: {
    marginBottom: theme.spacing.md,
    color: theme.colors.error,
    textAlign: 'center',
  },
  scrollView: {
    flex: 1,
    padding: theme.spacing.md,
  },
  card: {
    marginBottom: theme.spacing.md,
    elevation: 2,
  },
  statusHeader: {
    flexDirection: 'row',
    justifyContent: 'space-between',
    alignItems: 'center',
    marginBottom: theme.spacing.md,
  },
  onlineToggle: {
    flexDirection: 'row',
    alignItems: 'center',
  },
  onlineLabel: {
    marginRight: theme.spacing.sm,
    fontWeight: 'bold',
    color: theme.colors.text,
  },
  statusIndicators: {
    flexDirection: 'row',
    gap: theme.spacing.sm,
  },
  statusChip: {
    height: 28,
  },
  statusChipText: {
    fontSize: 12,
    color: 'white',
    fontWeight: 'bold',
  },
  statsGrid: {
    flexDirection: 'row',
    justifyContent: 'space-between',
    marginTop: theme.spacing.md,
  },
  statItem: {
    alignItems: 'center',
    flex: 1,
  },
  statValue: {
    fontSize: 20,
    fontWeight: 'bold',
    color: theme.colors.primary,
  },
  statLabel: {
    fontSize: 12,
    color: theme.colors.disabled,
    marginTop: theme.spacing.xs,
  },
  vehicleStatus: {
    marginTop: theme.spacing.md,
  },
  statusRow: {
    flexDirection: 'row',
    justifyContent: 'space-between',
    alignItems: 'center',
    marginBottom: theme.spacing.md,
  },
  statusLabel: {
    fontSize: 14,
    color: theme.colors.text,
    flex: 1,
  },
  statusValue: {
    fontSize: 14,
    fontWeight: 'bold',
    color: theme.colors.text,
  },
  progressContainer: {
    flexDirection: 'row',
    alignItems: 'center',
    flex: 2,
  },
  progressBar: {
    flex: 1,
    height: 8,
    marginRight: theme.spacing.sm,
  },
  progressText: {
    fontSize: 12,
    color: theme.colors.text,
    minWidth: 35,
    textAlign: 'right',
  },
  noTripsText: {
    textAlign: 'center',
    color: theme.colors.disabled,
    fontStyle: 'italic',
    marginTop: theme.spacing.md,
  },
  tripCard: {
    marginTop: theme.spacing.md,
    padding: theme.spacing.md,
    borderRadius: theme.roundness,
    elevation: 1,
  },
  tripHeader: {
    flexDirection: 'row',
    justifyContent: 'space-between',
    alignItems: 'center',
    marginBottom: theme.spacing.sm,
  },
  tripInfo: {
    flexDirection: 'row',
    alignItems: 'center',
    flex: 1,
  },
  tripId: {
    fontSize: 16,
    fontWeight: 'bold',
    color: theme.colors.text,
    marginRight: theme.spacing.sm,
  },
  tripStatusChip: {
    height: 24,
  },
  tripStatusText: {
    fontSize: 10,
    color: 'white',
    fontWeight: 'bold',
  },
  tripTime: {
    fontSize: 14,
    fontWeight: 'bold',
    color: theme.colors.primary,
  },
  tripDetails: {
    marginBottom: theme.spacing.sm,
  },
  locationRow: {
    flexDirection: 'row',
    alignItems: 'center',
    marginBottom: theme.spacing.xs,
  },
  locationText: {
    marginLeft: theme.spacing.sm,
    fontSize: 14,
    color: theme.colors.text,
    flex: 1,
  },
  passengerInfo: {
    flexDirection: 'row',
    alignItems: 'center',
    marginBottom: theme.spacing.sm,
    paddingVertical: theme.spacing.xs,
    paddingHorizontal: theme.spacing.sm,
    backgroundColor: theme.colors.primary + '10',
    borderRadius: theme.roundness,
  },
  passengerName: {
    marginLeft: theme.spacing.sm,
    fontSize: 14,
    fontWeight: 'bold',
    color: theme.colors.text,
    flex: 1,
  },
  passengerRating: {
    fontSize: 12,
    color: theme.colors.warning,
  },
  tripMeta: {
    flexDirection: 'row',
    justifyContent: 'space-between',
    alignItems: 'center',
    marginBottom: theme.spacing.sm,
  },
  metaText: {
    fontSize: 12,
    color: theme.colors.disabled,
  },
  fareText: {
    fontSize: 16,
    fontWeight: 'bold',
    color: theme.colors.success,
  },
  specialInstructions: {
    fontSize: 12,
    color: theme.colors.warning,
    fontStyle: 'italic',
    marginBottom: theme.spacing.sm,
    padding: theme.spacing.sm,
    backgroundColor: theme.colors.warning + '10',
    borderRadius: theme.roundness,
  },
  tripActions: {
    flexDirection: 'row',
    justifyContent: 'space-between',
    gap: theme.spacing.sm,
  },
  actionButton: {
    flex: 1,
  },
  navigationButton: {
    flex: 1,
  },
  mapContainer: {
    height: 200,
    marginTop: theme.spacing.md,
    borderRadius: theme.roundness,
    overflow: 'hidden',
  },
  map: {
    flex: 1,
  },
});

export default DriverScreen;
