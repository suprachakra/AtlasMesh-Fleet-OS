import React, { useState, useEffect } from 'react';
import {
  View,
  Text,
  FlatList,
  TouchableOpacity,
  StyleSheet,
  TextInput,
  RefreshControl,
  Alert,
} from 'react-native';
import { Ionicons } from '@expo/vector-icons';
import { theme } from '../theme/theme';

interface Vehicle {
  id: string;
  licensePlate: string;
  model: string;
  status: 'active' | 'maintenance' | 'inactive' | 'emergency';
  location: {
    lat: number;
    lng: number;
  };
  batteryLevel: number;
  fuelLevel: number;
  lastSeen: string;
  driver?: string;
  healthScore: number;
}

interface VehicleListScreenProps {
  navigation: any;
}

export default function VehicleListScreen({ navigation }: VehicleListScreenProps) {
  const [vehicles, setVehicles] = useState<Vehicle[]>([]);
  const [filteredVehicles, setFilteredVehicles] = useState<Vehicle[]>([]);
  const [searchQuery, setSearchQuery] = useState('');
  const [selectedStatus, setSelectedStatus] = useState<string>('all');
  const [isRefreshing, setIsRefreshing] = useState(false);

  useEffect(() => {
    loadVehicles();
  }, []);

  useEffect(() => {
    filterVehicles();
  }, [vehicles, searchQuery, selectedStatus]);

  const loadVehicles = async () => {
    // Simulate API call
    setTimeout(() => {
      const mockVehicles: Vehicle[] = [
        {
          id: '1',
          licensePlate: 'AD-001',
          model: 'Tesla Model 3',
          status: 'active',
          location: { lat: 24.4539, lng: 54.3773 },
          batteryLevel: 85,
          fuelLevel: 0,
          lastSeen: '2 minutes ago',
          driver: 'Ahmed Al Mansouri',
          healthScore: 95,
        },
        {
          id: '2',
          licensePlate: 'AD-002',
          model: 'BMW iX',
          status: 'maintenance',
          location: { lat: 24.4539, lng: 54.3773 },
          batteryLevel: 45,
          fuelLevel: 0,
          lastSeen: '1 hour ago',
          healthScore: 78,
        },
        {
          id: '3',
          licensePlate: 'AD-003',
          model: 'Mercedes EQS',
          status: 'active',
          location: { lat: 24.4539, lng: 54.3773 },
          batteryLevel: 92,
          fuelLevel: 0,
          lastSeen: '5 minutes ago',
          driver: 'Fatima Al Zahra',
          healthScore: 98,
        },
        {
          id: '4',
          licensePlate: 'AD-004',
          model: 'Audi e-tron',
          status: 'inactive',
          location: { lat: 24.4539, lng: 54.3773 },
          batteryLevel: 15,
          fuelLevel: 0,
          lastSeen: '3 hours ago',
          healthScore: 65,
        },
        {
          id: '5',
          licensePlate: 'AD-005',
          model: 'Volkswagen ID.4',
          status: 'emergency',
          location: { lat: 24.4539, lng: 54.3773 },
          batteryLevel: 8,
          fuelLevel: 0,
          lastSeen: '10 minutes ago',
          healthScore: 45,
        },
      ];
      setVehicles(mockVehicles);
    }, 1000);
  };

  const filterVehicles = () => {
    let filtered = vehicles;

    // Filter by search query
    if (searchQuery) {
      filtered = filtered.filter(vehicle =>
        vehicle.licensePlate.toLowerCase().includes(searchQuery.toLowerCase()) ||
        vehicle.model.toLowerCase().includes(searchQuery.toLowerCase()) ||
        (vehicle.driver && vehicle.driver.toLowerCase().includes(searchQuery.toLowerCase()))
      );
    }

    // Filter by status
    if (selectedStatus !== 'all') {
      filtered = filtered.filter(vehicle => vehicle.status === selectedStatus);
    }

    setFilteredVehicles(filtered);
  };

  const handleRefresh = async () => {
    setIsRefreshing(true);
    await loadVehicles();
    setIsRefreshing(false);
  };

  const handleVehiclePress = (vehicle: Vehicle) => {
    navigation.navigate('VehicleDetail', { vehicle });
  };

  const handleEmergencyAction = (vehicle: Vehicle) => {
    Alert.alert(
      'Emergency Action',
      `Are you sure you want to perform emergency action on ${vehicle.licensePlate}?`,
      [
        { text: 'Cancel', style: 'cancel' },
        { text: 'Confirm', style: 'destructive', onPress: () => {
          // Handle emergency action
          console.log('Emergency action triggered for', vehicle.licensePlate);
        }},
      ]
    );
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

  const renderVehicle = ({ item }: { item: Vehicle }) => (
    <TouchableOpacity
      style={styles.vehicleCard}
      onPress={() => handleVehiclePress(item)}
    >
      <View style={styles.vehicleHeader}>
        <View style={styles.vehicleInfo}>
          <Text style={styles.licensePlate}>{item.licensePlate}</Text>
          <Text style={styles.model}>{item.model}</Text>
          {item.driver && <Text style={styles.driver}>Driver: {item.driver}</Text>}
        </View>
        <View style={styles.statusContainer}>
          <Ionicons
            name={getStatusIcon(item.status) as any}
            size={20}
            color={getStatusColor(item.status)}
          />
          <Text style={[styles.status, { color: getStatusColor(item.status) }]}>
            {item.status.toUpperCase()}
          </Text>
        </View>
      </View>

      <View style={styles.vehicleDetails}>
        <View style={styles.detailRow}>
          <View style={styles.detailItem}>
            <Ionicons name="battery-half-outline" size={16} color="#666" />
            <Text style={styles.detailText}>{item.batteryLevel}%</Text>
          </View>
          <View style={styles.detailItem}>
            <Ionicons name="time-outline" size={16} color="#666" />
            <Text style={styles.detailText}>{item.lastSeen}</Text>
          </View>
          <View style={styles.detailItem}>
            <Ionicons name="heart-outline" size={16} color={getHealthColor(item.healthScore)} />
            <Text style={[styles.detailText, { color: getHealthColor(item.healthScore) }]}>
              {item.healthScore}%
            </Text>
          </View>
        </View>
      </View>

      {item.status === 'emergency' && (
        <TouchableOpacity
          style={styles.emergencyButton}
          onPress={() => handleEmergencyAction(item)}
        >
          <Ionicons name="warning" size={16} color="#fff" />
          <Text style={styles.emergencyButtonText}>Emergency Action</Text>
        </TouchableOpacity>
      )}
    </TouchableOpacity>
  );

  const statusFilters = [
    { key: 'all', label: 'All', count: vehicles.length },
    { key: 'active', label: 'Active', count: vehicles.filter(v => v.status === 'active').length },
    { key: 'maintenance', label: 'Maintenance', count: vehicles.filter(v => v.status === 'maintenance').length },
    { key: 'inactive', label: 'Inactive', count: vehicles.filter(v => v.status === 'inactive').length },
    { key: 'emergency', label: 'Emergency', count: vehicles.filter(v => v.status === 'emergency').length },
  ];

  return (
    <View style={styles.container}>
      {/* Header */}
      <View style={styles.header}>
        <Text style={styles.headerTitle}>Fleet Vehicles</Text>
        <TouchableOpacity style={styles.headerButton}>
          <Ionicons name="filter" size={24} color={theme.colors.primary} />
        </TouchableOpacity>
      </View>

      {/* Search Bar */}
      <View style={styles.searchContainer}>
        <Ionicons name="search" size={20} color="#666" style={styles.searchIcon} />
        <TextInput
          style={styles.searchInput}
          placeholder="Search vehicles..."
          value={searchQuery}
          onChangeText={setSearchQuery}
          placeholderTextColor="#999"
        />
      </View>

      {/* Status Filters */}
      <ScrollView horizontal showsHorizontalScrollIndicator={false} style={styles.filtersContainer}>
        {statusFilters.map((filter) => (
          <TouchableOpacity
            key={filter.key}
            style={[
              styles.filterButton,
              selectedStatus === filter.key && styles.filterButtonActive
            ]}
            onPress={() => setSelectedStatus(filter.key)}
          >
            <Text style={[
              styles.filterButtonText,
              selectedStatus === filter.key && styles.filterButtonTextActive
            ]}>
              {filter.label} ({filter.count})
            </Text>
          </TouchableOpacity>
        ))}
      </ScrollView>

      {/* Vehicle List */}
      <FlatList
        data={filteredVehicles}
        renderItem={renderVehicle}
        keyExtractor={(item) => item.id}
        contentContainerStyle={styles.listContainer}
        refreshControl={
          <RefreshControl refreshing={isRefreshing} onRefresh={handleRefresh} />
        }
        showsVerticalScrollIndicator={false}
      />
    </View>
  );
}

const styles = StyleSheet.create({
  container: {
    flex: 1,
    backgroundColor: '#f8fafc',
  },
  header: {
    flexDirection: 'row',
    justifyContent: 'space-between',
    alignItems: 'center',
    padding: 20,
    backgroundColor: '#fff',
    borderBottomWidth: 1,
    borderBottomColor: '#e2e8f0',
  },
  headerTitle: {
    fontSize: 24,
    fontWeight: 'bold',
    color: '#333',
  },
  headerButton: {
    padding: 8,
  },
  searchContainer: {
    flexDirection: 'row',
    alignItems: 'center',
    backgroundColor: '#fff',
    margin: 20,
    paddingHorizontal: 16,
    borderRadius: 8,
    borderWidth: 1,
    borderColor: '#e2e8f0',
  },
  searchIcon: {
    marginRight: 12,
  },
  searchInput: {
    flex: 1,
    height: 48,
    fontSize: 16,
    color: '#333',
  },
  filtersContainer: {
    paddingHorizontal: 20,
    marginBottom: 16,
  },
  filterButton: {
    paddingHorizontal: 16,
    paddingVertical: 8,
    borderRadius: 20,
    backgroundColor: '#fff',
    marginRight: 8,
    borderWidth: 1,
    borderColor: '#e2e8f0',
  },
  filterButtonActive: {
    backgroundColor: theme.colors.primary,
    borderColor: theme.colors.primary,
  },
  filterButtonText: {
    fontSize: 14,
    color: '#666',
    fontWeight: '500',
  },
  filterButtonTextActive: {
    color: '#fff',
  },
  listContainer: {
    padding: 20,
  },
  vehicleCard: {
    backgroundColor: '#fff',
    borderRadius: 12,
    padding: 16,
    marginBottom: 12,
    shadowColor: '#000',
    shadowOffset: { width: 0, height: 2 },
    shadowOpacity: 0.1,
    shadowRadius: 4,
    elevation: 2,
  },
  vehicleHeader: {
    flexDirection: 'row',
    justifyContent: 'space-between',
    alignItems: 'flex-start',
    marginBottom: 12,
  },
  vehicleInfo: {
    flex: 1,
  },
  licensePlate: {
    fontSize: 18,
    fontWeight: 'bold',
    color: '#333',
    marginBottom: 4,
  },
  model: {
    fontSize: 14,
    color: '#666',
    marginBottom: 4,
  },
  driver: {
    fontSize: 12,
    color: '#999',
  },
  statusContainer: {
    flexDirection: 'row',
    alignItems: 'center',
  },
  status: {
    fontSize: 12,
    fontWeight: 'bold',
    marginLeft: 4,
  },
  vehicleDetails: {
    marginBottom: 12,
  },
  detailRow: {
    flexDirection: 'row',
    justifyContent: 'space-between',
  },
  detailItem: {
    flexDirection: 'row',
    alignItems: 'center',
  },
  detailText: {
    fontSize: 12,
    color: '#666',
    marginLeft: 4,
  },
  emergencyButton: {
    flexDirection: 'row',
    alignItems: 'center',
    justifyContent: 'center',
    backgroundColor: '#ef4444',
    paddingVertical: 8,
    paddingHorizontal: 16,
    borderRadius: 6,
  },
  emergencyButtonText: {
    color: '#fff',
    fontSize: 14,
    fontWeight: 'bold',
    marginLeft: 4,
  },
});
