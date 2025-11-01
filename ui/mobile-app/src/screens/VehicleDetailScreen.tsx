import React, { useState, useEffect } from 'react';
import {
  View,
  Text,
  ScrollView,
  StyleSheet,
  TouchableOpacity,
  Alert,
  Dimensions,
} from 'react-native';
import { Ionicons } from '@expo/vector-icons';
import { theme } from '../theme/theme';

interface VehicleDetailScreenProps {
  route: {
    params: {
      vehicle: {
        id: string;
        licensePlate: string;
        model: string;
        status: string;
        location: { lat: number; lng: number };
        batteryLevel: number;
        fuelLevel: number;
        lastSeen: string;
        driver?: string;
        healthScore: number;
      };
    };
  };
  navigation: any;
}

interface VehicleMetrics {
  speed: number;
  temperature: number;
  odometer: number;
  engineHours: number;
  tirePressure: number[];
  lastMaintenance: string;
  nextMaintenance: string;
}

export default function VehicleDetailScreen({ route, navigation }: VehicleDetailScreenProps) {
  const { vehicle } = route.params;
  const [metrics, setMetrics] = useState<VehicleMetrics>({
    speed: 0,
    temperature: 0,
    odometer: 0,
    engineHours: 0,
    tirePressure: [0, 0, 0, 0],
    lastMaintenance: '',
    nextMaintenance: '',
  });
  const [isLoading, setIsLoading] = useState(true);

  useEffect(() => {
    loadVehicleMetrics();
  }, []);

  const loadVehicleMetrics = async () => {
    // Simulate API call
    setTimeout(() => {
      setMetrics({
        speed: 45,
        temperature: 28,
        odometer: 125430,
        engineHours: 2847,
        tirePressure: [32, 31, 33, 32],
        lastMaintenance: '2024-01-15',
        nextMaintenance: '2024-04-15',
      });
      setIsLoading(false);
    }, 1000);
  };

  const getStatusColor = (status: string) => {
    switch (status) {
      case 'active': return '#10b981';
      case 'maintenance': return '#f59e0b';
      case 'inactive': return '#6b7280';
      case 'emergency': return '#ef4444';
      default: return '#6b7280';
    }
  };

  const getStatusIcon = (status: string) => {
    switch (status) {
      case 'active': return 'checkmark-circle';
      case 'maintenance': return 'construct';
      case 'inactive': return 'pause-circle';
      case 'emergency': return 'warning';
      default: return 'help-circle';
    }
  };

  const getHealthColor = (score: number) => {
    if (score >= 90) return '#10b981';
    if (score >= 70) return '#f59e0b';
    return '#ef4444';
  };

  const handleEmergencyStop = () => {
    Alert.alert(
      'Emergency Stop',
      'Are you sure you want to perform an emergency stop on this vehicle?',
      [
        { text: 'Cancel', style: 'cancel' },
        { text: 'Confirm', style: 'destructive', onPress: () => {
          // Handle emergency stop
          console.log('Emergency stop triggered for', vehicle.licensePlate);
        }},
      ]
    );
  };

  const handleMaintenanceRequest = () => {
    Alert.alert(
      'Maintenance Request',
      'Request maintenance for this vehicle?',
      [
        { text: 'Cancel', style: 'cancel' },
        { text: 'Request', onPress: () => {
          // Handle maintenance request
          console.log('Maintenance requested for', vehicle.licensePlate);
        }},
      ]
    );
  };

  const handleLocationView = () => {
    // Navigate to map view
    console.log('View location on map');
  };

  return (
    <ScrollView style={styles.container}>
      {/* Header */}
      <View style={[styles.header, { backgroundColor: getStatusColor(vehicle.status) }]}>
        <View style={styles.headerContent}>
          <TouchableOpacity
            style={styles.backButton}
            onPress={() => navigation.goBack()}
          >
            <Ionicons name="arrow-back" size={24} color="#fff" />
          </TouchableOpacity>
          <View style={styles.headerInfo}>
            <Text style={styles.licensePlate}>{vehicle.licensePlate}</Text>
            <Text style={styles.model}>{vehicle.model}</Text>
            <View style={styles.statusContainer}>
              <Ionicons
                name={getStatusIcon(vehicle.status) as any}
                size={16}
                color="#fff"
              />
              <Text style={styles.status}>{vehicle.status.toUpperCase()}</Text>
            </View>
          </View>
          <TouchableOpacity style={styles.menuButton}>
            <Ionicons name="ellipsis-vertical" size={24} color="#fff" />
          </TouchableOpacity>
        </View>
      </View>

      {/* Quick Actions */}
      <View style={styles.actionsContainer}>
        <TouchableOpacity style={styles.actionButton}>
          <Ionicons name="location-outline" size={24} color={theme.colors.primary} />
          <Text style={styles.actionText}>Location</Text>
        </TouchableOpacity>
        
        <TouchableOpacity style={styles.actionButton}>
          <Ionicons name="analytics-outline" size={24} color={theme.colors.primary} />
          <Text style={styles.actionText}>Analytics</Text>
        </TouchableOpacity>
        
        <TouchableOpacity style={styles.actionButton}>
          <Ionicons name="construct-outline" size={24} color={theme.colors.primary} />
          <Text style={styles.actionText}>Maintenance</Text>
        </TouchableOpacity>
        
        <TouchableOpacity style={styles.actionButton}>
          <Ionicons name="settings-outline" size={24} color={theme.colors.primary} />
          <Text style={styles.actionText}>Settings</Text>
        </TouchableOpacity>
      </View>

      {/* Vehicle Status */}
      <View style={styles.section}>
        <Text style={styles.sectionTitle}>Vehicle Status</Text>
        <View style={styles.statusGrid}>
          <View style={styles.statusCard}>
            <Ionicons name="battery-half-outline" size={24} color="#10b981" />
            <Text style={styles.statusValue}>{vehicle.batteryLevel}%</Text>
            <Text style={styles.statusLabel}>Battery</Text>
          </View>
          
          <View style={styles.statusCard}>
            <Ionicons name="heart-outline" size={24} color={getHealthColor(vehicle.healthScore)} />
            <Text style={[styles.statusValue, { color: getHealthColor(vehicle.healthScore) }]}>
              {vehicle.healthScore}%
            </Text>
            <Text style={styles.statusLabel}>Health</Text>
          </View>
          
          <View style={styles.statusCard}>
            <Ionicons name="time-outline" size={24} color="#3b82f6" />
            <Text style={styles.statusValue}>{vehicle.lastSeen}</Text>
            <Text style={styles.statusLabel}>Last Seen</Text>
          </View>
        </View>
      </View>

      {/* Real-time Metrics */}
      <View style={styles.section}>
        <Text style={styles.sectionTitle}>Real-time Metrics</Text>
        <View style={styles.metricsGrid}>
          <View style={styles.metricCard}>
            <Ionicons name="speedometer-outline" size={20} color="#666" />
            <Text style={styles.metricValue}>{metrics.speed} km/h</Text>
            <Text style={styles.metricLabel}>Speed</Text>
          </View>
          
          <View style={styles.metricCard}>
            <Ionicons name="thermometer-outline" size={20} color="#666" />
            <Text style={styles.metricValue}>{metrics.temperature}°C</Text>
            <Text style={styles.metricLabel}>Temperature</Text>
          </View>
          
          <View style={styles.metricCard}>
            <Ionicons name="car-outline" size={20} color="#666" />
            <Text style={styles.metricValue}>{metrics.odometer.toLocaleString()} km</Text>
            <Text style={styles.metricLabel}>Odometer</Text>
          </View>
          
          <View style={styles.metricCard}>
            <Ionicons name="time-outline" size={20} color="#666" />
            <Text style={styles.metricValue}>{metrics.engineHours} hrs</Text>
            <Text style={styles.metricLabel}>Engine Hours</Text>
          </View>
        </View>
      </View>

      {/* Tire Pressure */}
      <View style={styles.section}>
        <Text style={styles.sectionTitle}>Tire Pressure</Text>
        <View style={styles.tireContainer}>
          <View style={styles.tireRow}>
            <View style={styles.tireItem}>
              <Text style={styles.tireLabel}>Front Left</Text>
              <Text style={styles.tireValue}>{metrics.tirePressure[0]} PSI</Text>
            </View>
            <View style={styles.tireItem}>
              <Text style={styles.tireLabel}>Front Right</Text>
              <Text style={styles.tireValue}>{metrics.tirePressure[1]} PSI</Text>
            </View>
          </View>
          <View style={styles.tireRow}>
            <View style={styles.tireItem}>
              <Text style={styles.tireLabel}>Rear Left</Text>
              <Text style={styles.tireValue}>{metrics.tirePressure[2]} PSI</Text>
            </View>
            <View style={styles.tireItem}>
              <Text style={styles.tireLabel}>Rear Right</Text>
              <Text style={styles.tireValue}>{metrics.tirePressure[3]} PSI</Text>
            </View>
          </View>
        </View>
      </View>

      {/* Maintenance Info */}
      <View style={styles.section}>
        <Text style={styles.sectionTitle}>Maintenance</Text>
        <View style={styles.maintenanceCard}>
          <View style={styles.maintenanceItem}>
            <Ionicons name="calendar-outline" size={20} color="#666" />
            <View style={styles.maintenanceInfo}>
              <Text style={styles.maintenanceLabel}>Last Maintenance</Text>
              <Text style={styles.maintenanceValue}>{metrics.lastMaintenance}</Text>
            </View>
          </View>
          
          <View style={styles.maintenanceItem}>
            <Ionicons name="calendar-outline" size={20} color="#666" />
            <View style={styles.maintenanceInfo}>
              <Text style={styles.maintenanceLabel}>Next Maintenance</Text>
              <Text style={styles.maintenanceValue}>{metrics.nextMaintenance}</Text>
            </View>
          </View>
        </View>
      </View>

      {/* Emergency Actions */}
      {vehicle.status === 'emergency' && (
        <View style={styles.section}>
          <Text style={styles.sectionTitle}>Emergency Actions</Text>
          <TouchableOpacity style={styles.emergencyButton} onPress={handleEmergencyStop}>
            <Ionicons name="warning" size={20} color="#fff" />
            <Text style={styles.emergencyButtonText}>Emergency Stop</Text>
          </TouchableOpacity>
        </View>
      )}

      {/* Action Buttons */}
      <View style={styles.actionButtonsContainer}>
        <TouchableOpacity style={styles.secondaryButton} onPress={handleMaintenanceRequest}>
          <Ionicons name="construct-outline" size={20} color={theme.colors.primary} />
          <Text style={styles.secondaryButtonText}>Request Maintenance</Text>
        </TouchableOpacity>
        
        <TouchableOpacity style={styles.primaryButton} onPress={handleLocationView}>
          <Ionicons name="location-outline" size={20} color="#fff" />
          <Text style={styles.primaryButtonText}>View on Map</Text>
        </TouchableOpacity>
      </View>
    </ScrollView>
  );
}

const styles = StyleSheet.create({
  container: {
    flex: 1,
    backgroundColor: '#f8fafc',
  },
  header: {
    paddingTop: 20,
    paddingBottom: 30,
    paddingHorizontal: 20,
  },
  headerContent: {
    flexDirection: 'row',
    alignItems: 'center',
  },
  backButton: {
    padding: 8,
    marginRight: 16,
  },
  headerInfo: {
    flex: 1,
  },
  licensePlate: {
    fontSize: 24,
    fontWeight: 'bold',
    color: '#fff',
    marginBottom: 4,
  },
  model: {
    fontSize: 16,
    color: '#e3f2fd',
    marginBottom: 8,
  },
  statusContainer: {
    flexDirection: 'row',
    alignItems: 'center',
  },
  status: {
    fontSize: 12,
    color: '#fff',
    marginLeft: 4,
    fontWeight: 'bold',
  },
  menuButton: {
    padding: 8,
  },
  actionsContainer: {
    flexDirection: 'row',
    padding: 20,
    backgroundColor: '#fff',
    borderBottomWidth: 1,
    borderBottomColor: '#e2e8f0',
  },
  actionButton: {
    flex: 1,
    alignItems: 'center',
    padding: 12,
  },
  actionText: {
    fontSize: 12,
    color: '#666',
    marginTop: 4,
  },
  section: {
    margin: 20,
  },
  sectionTitle: {
    fontSize: 18,
    fontWeight: 'bold',
    color: '#333',
    marginBottom: 16,
  },
  statusGrid: {
    flexDirection: 'row',
    justifyContent: 'space-between',
  },
  statusCard: {
    flex: 1,
    backgroundColor: '#fff',
    padding: 16,
    borderRadius: 12,
    alignItems: 'center',
    marginHorizontal: 4,
    shadowColor: '#000',
    shadowOffset: { width: 0, height: 2 },
    shadowOpacity: 0.1,
    shadowRadius: 4,
    elevation: 2,
  },
  statusValue: {
    fontSize: 20,
    fontWeight: 'bold',
    color: '#333',
    marginTop: 8,
    marginBottom: 4,
  },
  statusLabel: {
    fontSize: 12,
    color: '#666',
  },
  metricsGrid: {
    flexDirection: 'row',
    flexWrap: 'wrap',
    justifyContent: 'space-between',
  },
  metricCard: {
    width: '48%',
    backgroundColor: '#fff',
    padding: 16,
    borderRadius: 12,
    alignItems: 'center',
    marginBottom: 12,
    shadowColor: '#000',
    shadowOffset: { width: 0, height: 2 },
    shadowOpacity: 0.1,
    shadowRadius: 4,
    elevation: 2,
  },
  metricValue: {
    fontSize: 16,
    fontWeight: 'bold',
    color: '#333',
    marginTop: 8,
    marginBottom: 4,
  },
  metricLabel: {
    fontSize: 12,
    color: '#666',
  },
  tireContainer: {
    backgroundColor: '#fff',
    padding: 16,
    borderRadius: 12,
    shadowColor: '#000',
    shadowOffset: { width: 0, height: 2 },
    shadowOpacity: 0.1,
    shadowRadius: 4,
    elevation: 2,
  },
  tireRow: {
    flexDirection: 'row',
    justifyContent: 'space-between',
    marginBottom: 16,
  },
  tireItem: {
    flex: 1,
    alignItems: 'center',
  },
  tireLabel: {
    fontSize: 12,
    color: '#666',
    marginBottom: 4,
  },
  tireValue: {
    fontSize: 16,
    fontWeight: 'bold',
    color: '#333',
  },
  maintenanceCard: {
    backgroundColor: '#fff',
    padding: 16,
    borderRadius: 12,
    shadowColor: '#000',
    shadowOffset: { width: 0, height: 2 },
    shadowOpacity: 0.1,
    shadowRadius: 4,
    elevation: 2,
  },
  maintenanceItem: {
    flexDirection: 'row',
    alignItems: 'center',
    marginBottom: 12,
  },
  maintenanceInfo: {
    marginLeft: 12,
  },
  maintenanceLabel: {
    fontSize: 14,
    color: '#666',
    marginBottom: 2,
  },
  maintenanceValue: {
    fontSize: 16,
    fontWeight: 'bold',
    color: '#333',
  },
  emergencyButton: {
    flexDirection: 'row',
    alignItems: 'center',
    justifyContent: 'center',
    backgroundColor: '#ef4444',
    padding: 16,
    borderRadius: 12,
  },
  emergencyButtonText: {
    color: '#fff',
    fontSize: 16,
    fontWeight: 'bold',
    marginLeft: 8,
  },
  actionButtonsContainer: {
    flexDirection: 'row',
    padding: 20,
    gap: 12,
  },
  secondaryButton: {
    flex: 1,
    flexDirection: 'row',
    alignItems: 'center',
    justifyContent: 'center',
    backgroundColor: '#fff',
    padding: 16,
    borderRadius: 12,
    borderWidth: 1,
    borderColor: theme.colors.primary,
  },
  secondaryButtonText: {
    color: theme.colors.primary,
    fontSize: 16,
    fontWeight: 'bold',
    marginLeft: 8,
  },
  primaryButton: {
    flex: 1,
    flexDirection: 'row',
    alignItems: 'center',
    justifyContent: 'center',
    backgroundColor: theme.colors.primary,
    padding: 16,
    borderRadius: 12,
  },
  primaryButtonText: {
    color: '#fff',
    fontSize: 16,
    fontWeight: 'bold',
    marginLeft: 8,
  },
});
