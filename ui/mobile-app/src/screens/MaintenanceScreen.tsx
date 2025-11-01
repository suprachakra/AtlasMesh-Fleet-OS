import React, { useState, useEffect } from 'react';
import {
  View,
  Text,
  ScrollView,
  StyleSheet,
  TouchableOpacity,
  FlatList,
  Alert,
  RefreshControl,
} from 'react-native';
import { Ionicons } from '@expo/vector-icons';
import { theme } from '../theme/theme';

interface MaintenanceItem {
  id: string;
  vehicleId: string;
  licensePlate: string;
  type: 'scheduled' | 'emergency' | 'preventive' | 'repair';
  status: 'pending' | 'in_progress' | 'completed' | 'cancelled';
  priority: 'low' | 'medium' | 'high' | 'critical';
  description: string;
  scheduledDate: string;
  estimatedDuration: number;
  assignedTechnician?: string;
  cost?: number;
  parts?: string[];
}

interface MaintenanceScreenProps {
  navigation: any;
}

export default function MaintenanceScreen({ navigation }: MaintenanceScreenProps) {
  const [maintenanceItems, setMaintenanceItems] = useState<MaintenanceItem[]>([]);
  const [selectedFilter, setSelectedFilter] = useState<string>('all');
  const [isRefreshing, setIsRefreshing] = useState(false);

  useEffect(() => {
    loadMaintenanceItems();
  }, []);

  const loadMaintenanceItems = async () => {
    // Simulate API call
    setTimeout(() => {
      const mockItems: MaintenanceItem[] = [
        {
          id: '1',
          vehicleId: 'V-001',
          licensePlate: 'AD-001',
          type: 'scheduled',
          status: 'pending',
          priority: 'high',
          description: 'Regular oil change and filter replacement',
          scheduledDate: '2024-12-20',
          estimatedDuration: 120,
          assignedTechnician: 'Ahmed Al Mansouri',
          cost: 250,
          parts: ['Oil Filter', 'Engine Oil 5W-30'],
        },
        {
          id: '2',
          vehicleId: 'V-002',
          licensePlate: 'AD-002',
          type: 'emergency',
          status: 'in_progress',
          priority: 'critical',
          description: 'Brake system inspection and repair',
          scheduledDate: '2024-12-19',
          estimatedDuration: 180,
          assignedTechnician: 'Fatima Al Zahra',
          cost: 450,
          parts: ['Brake Pads', 'Brake Fluid'],
        },
        {
          id: '3',
          vehicleId: 'V-003',
          licensePlate: 'AD-003',
          type: 'preventive',
          status: 'completed',
          priority: 'medium',
          description: 'Battery health check and cleaning',
          scheduledDate: '2024-12-18',
          estimatedDuration: 60,
          assignedTechnician: 'Mohammed Al Rashid',
          cost: 150,
          parts: ['Battery Cleaner', 'Terminal Grease'],
        },
        {
          id: '4',
          vehicleId: 'V-004',
          licensePlate: 'AD-004',
          type: 'repair',
          status: 'pending',
          priority: 'high',
          description: 'Air conditioning system repair',
          scheduledDate: '2024-12-21',
          estimatedDuration: 240,
          assignedTechnician: 'Aisha Al Maktoum',
          cost: 800,
          parts: ['AC Compressor', 'Refrigerant', 'AC Filter'],
        },
      ];
      setMaintenanceItems(mockItems);
    }, 1000);
  };

  const handleRefresh = async () => {
    setIsRefreshing(true);
    await loadMaintenanceItems();
    setIsRefreshing(false);
  };

  const getPriorityColor = (priority: string) => {
    switch (priority) {
      case 'critical': return '#ef4444';
      case 'high': return '#f59e0b';
      case 'medium': return '#3b82f6';
      case 'low': return '#10b981';
      default: return '#6b7280';
    }
  };

  const getStatusColor = (status: string) => {
    switch (status) {
      case 'completed': return '#10b981';
      case 'in_progress': return '#3b82f6';
      case 'pending': return '#f59e0b';
      case 'cancelled': return '#ef4444';
      default: return '#6b7280';
    }
  };

  const getTypeIcon = (type: string) => {
    switch (type) {
      case 'scheduled': return 'calendar-outline';
      case 'emergency': return 'warning-outline';
      case 'preventive': return 'shield-checkmark-outline';
      case 'repair': return 'construct-outline';
      default: return 'help-circle-outline';
    }
  };

  const handleMaintenanceAction = (item: MaintenanceItem, action: string) => {
    Alert.alert(
      `${action} Maintenance`,
      `Are you sure you want to ${action.toLowerCase()} this maintenance item?`,
      [
        { text: 'Cancel', style: 'cancel' },
        { text: 'Confirm', onPress: () => {
          // Handle maintenance action
          console.log(`${action} maintenance item:`, item.id);
        }},
      ]
    );
  };

  const renderMaintenanceItem = ({ item }: { item: MaintenanceItem }) => (
    <View style={styles.maintenanceCard}>
      <View style={styles.cardHeader}>
        <View style={styles.vehicleInfo}>
          <Text style={styles.licensePlate}>{item.licensePlate}</Text>
          <Text style={styles.description}>{item.description}</Text>
        </View>
        <View style={styles.statusContainer}>
          <View style={[styles.priorityBadge, { backgroundColor: getPriorityColor(item.priority) }]}>
            <Text style={styles.priorityText}>{item.priority.toUpperCase()}</Text>
          </View>
          <View style={[styles.statusBadge, { backgroundColor: getStatusColor(item.status) }]}>
            <Text style={styles.statusText}>{item.status.toUpperCase()}</Text>
          </View>
        </View>
      </View>

      <View style={styles.cardDetails}>
        <View style={styles.detailRow}>
          <View style={styles.detailItem}>
            <Ionicons name="calendar-outline" size={16} color="#666" />
            <Text style={styles.detailText}>{item.scheduledDate}</Text>
          </View>
          <View style={styles.detailItem}>
            <Ionicons name="time-outline" size={16} color="#666" />
            <Text style={styles.detailText}>{item.estimatedDuration} min</Text>
          </View>
        </View>

        {item.assignedTechnician && (
          <View style={styles.detailRow}>
            <View style={styles.detailItem}>
              <Ionicons name="person-outline" size={16} color="#666" />
              <Text style={styles.detailText}>{item.assignedTechnician}</Text>
            </View>
            {item.cost && (
              <View style={styles.detailItem}>
                <Ionicons name="cash-outline" size={16} color="#666" />
                <Text style={styles.detailText}>AED {item.cost}</Text>
              </View>
            )}
          </View>
        )}

        {item.parts && item.parts.length > 0 && (
          <View style={styles.partsContainer}>
            <Text style={styles.partsLabel}>Required Parts:</Text>
            <Text style={styles.partsText}>{item.parts.join(', ')}</Text>
          </View>
        )}
      </View>

      <View style={styles.cardActions}>
        {item.status === 'pending' && (
          <TouchableOpacity
            style={[styles.actionButton, styles.startButton]}
            onPress={() => handleMaintenanceAction(item, 'Start')}
          >
            <Ionicons name="play-outline" size={16} color="#fff" />
            <Text style={styles.actionButtonText}>Start</Text>
          </TouchableOpacity>
        )}

        {item.status === 'in_progress' && (
          <TouchableOpacity
            style={[styles.actionButton, styles.completeButton]}
            onPress={() => handleMaintenanceAction(item, 'Complete')}
          >
            <Ionicons name="checkmark-outline" size={16} color="#fff" />
            <Text style={styles.actionButtonText}>Complete</Text>
          </TouchableOpacity>
        )}

        <TouchableOpacity
          style={[styles.actionButton, styles.detailsButton]}
          onPress={() => navigation.navigate('MaintenanceDetail', { item })}
        >
          <Ionicons name="eye-outline" size={16} color={theme.colors.primary} />
          <Text style={[styles.actionButtonText, { color: theme.colors.primary }]}>Details</Text>
        </TouchableOpacity>
      </View>
    </View>
  );

  const filters = [
    { key: 'all', label: 'All', count: maintenanceItems.length },
    { key: 'pending', label: 'Pending', count: maintenanceItems.filter(item => item.status === 'pending').length },
    { key: 'in_progress', label: 'In Progress', count: maintenanceItems.filter(item => item.status === 'in_progress').length },
    { key: 'completed', label: 'Completed', count: maintenanceItems.filter(item => item.status === 'completed').length },
  ];

  const filteredItems = selectedFilter === 'all' 
    ? maintenanceItems 
    : maintenanceItems.filter(item => item.status === selectedFilter);

  return (
    <View style={styles.container}>
      {/* Header */}
      <View style={styles.header}>
        <Text style={styles.headerTitle}>Maintenance</Text>
        <TouchableOpacity style={styles.headerButton}>
          <Ionicons name="add" size={24} color={theme.colors.primary} />
        </TouchableOpacity>
      </View>

      {/* Filters */}
      <ScrollView horizontal showsHorizontalScrollIndicator={false} style={styles.filtersContainer}>
        {filters.map((filter) => (
          <TouchableOpacity
            key={filter.key}
            style={[
              styles.filterButton,
              selectedFilter === filter.key && styles.filterButtonActive
            ]}
            onPress={() => setSelectedFilter(filter.key)}
          >
            <Text style={[
              styles.filterButtonText,
              selectedFilter === filter.key && styles.filterButtonTextActive
            ]}>
              {filter.label} ({filter.count})
            </Text>
          </TouchableOpacity>
        ))}
      </ScrollView>

      {/* Maintenance List */}
      <FlatList
        data={filteredItems}
        renderItem={renderMaintenanceItem}
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
  maintenanceCard: {
    backgroundColor: '#fff',
    borderRadius: 12,
    padding: 16,
    marginBottom: 16,
    shadowColor: '#000',
    shadowOffset: { width: 0, height: 2 },
    shadowOpacity: 0.1,
    shadowRadius: 4,
    elevation: 2,
  },
  cardHeader: {
    flexDirection: 'row',
    justifyContent: 'space-between',
    alignItems: 'flex-start',
    marginBottom: 12,
  },
  vehicleInfo: {
    flex: 1,
  },
  licensePlate: {
    fontSize: 16,
    fontWeight: 'bold',
    color: '#333',
    marginBottom: 4,
  },
  description: {
    fontSize: 14,
    color: '#666',
    lineHeight: 20,
  },
  statusContainer: {
    alignItems: 'flex-end',
  },
  priorityBadge: {
    paddingHorizontal: 8,
    paddingVertical: 4,
    borderRadius: 12,
    marginBottom: 4,
  },
  priorityText: {
    fontSize: 10,
    color: '#fff',
    fontWeight: 'bold',
  },
  statusBadge: {
    paddingHorizontal: 8,
    paddingVertical: 4,
    borderRadius: 12,
  },
  statusText: {
    fontSize: 10,
    color: '#fff',
    fontWeight: 'bold',
  },
  cardDetails: {
    marginBottom: 16,
  },
  detailRow: {
    flexDirection: 'row',
    justifyContent: 'space-between',
    marginBottom: 8,
  },
  detailItem: {
    flexDirection: 'row',
    alignItems: 'center',
    flex: 1,
  },
  detailText: {
    fontSize: 12,
    color: '#666',
    marginLeft: 4,
  },
  partsContainer: {
    marginTop: 8,
    padding: 8,
    backgroundColor: '#f8fafc',
    borderRadius: 6,
  },
  partsLabel: {
    fontSize: 12,
    color: '#666',
    marginBottom: 4,
  },
  partsText: {
    fontSize: 12,
    color: '#333',
  },
  cardActions: {
    flexDirection: 'row',
    justifyContent: 'space-between',
  },
  actionButton: {
    flexDirection: 'row',
    alignItems: 'center',
    paddingHorizontal: 16,
    paddingVertical: 8,
    borderRadius: 6,
    flex: 1,
    marginHorizontal: 4,
    justifyContent: 'center',
  },
  startButton: {
    backgroundColor: '#10b981',
  },
  completeButton: {
    backgroundColor: '#3b82f6',
  },
  detailsButton: {
    backgroundColor: '#fff',
    borderWidth: 1,
    borderColor: theme.colors.primary,
  },
  actionButtonText: {
    fontSize: 14,
    fontWeight: 'bold',
    marginLeft: 4,
    color: '#fff',
  },
});
