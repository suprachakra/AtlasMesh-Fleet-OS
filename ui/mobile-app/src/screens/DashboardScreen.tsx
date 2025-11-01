import React, { useState, useEffect } from 'react';
import {
  View,
  Text,
  ScrollView,
  StyleSheet,
  TouchableOpacity,
  RefreshControl,
  Dimensions,
} from 'react-native';
import { Ionicons } from '@expo/vector-icons';
import { theme } from '../theme/theme';
import { User } from '../types/User';

interface DashboardScreenProps {
  user: User;
}

interface DashboardStats {
  totalVehicles: number;
  activeVehicles: number;
  maintenanceVehicles: number;
  totalTrips: number;
  completedTrips: number;
  safetyScore: number;
}

export default function DashboardScreen({ user }: DashboardScreenProps) {
  const [stats, setStats] = useState<DashboardStats>({
    totalVehicles: 0,
    activeVehicles: 0,
    maintenanceVehicles: 0,
    totalTrips: 0,
    completedTrips: 0,
    safetyScore: 0,
  });
  const [isRefreshing, setIsRefreshing] = useState(false);
  const [recentAlerts, setRecentAlerts] = useState([
    { id: '1', type: 'warning', message: 'Vehicle V-001 requires maintenance', time: '2 min ago' },
    { id: '2', type: 'info', message: 'Trip T-123 completed successfully', time: '5 min ago' },
    { id: '3', type: 'success', message: 'Fleet health score improved to 95%', time: '10 min ago' },
  ]);

  useEffect(() => {
    loadDashboardData();
  }, []);

  const loadDashboardData = async () => {
    // Simulate API call
    setTimeout(() => {
      setStats({
        totalVehicles: 45,
        activeVehicles: 38,
        maintenanceVehicles: 7,
        totalTrips: 156,
        completedTrips: 142,
        safetyScore: 95,
      });
    }, 1000);
  };

  const handleRefresh = async () => {
    setIsRefreshing(true);
    await loadDashboardData();
    setIsRefreshing(false);
  };

  const getGreeting = () => {
    const hour = new Date().getHours();
    if (hour < 12) return 'Good Morning';
    if (hour < 18) return 'Good Afternoon';
    return 'Good Evening';
  };

  const getAlertIcon = (type: string) => {
    switch (type) {
      case 'warning': return 'warning-outline';
      case 'info': return 'information-circle-outline';
      case 'success': return 'checkmark-circle-outline';
      default: return 'alert-circle-outline';
    }
  };

  const getAlertColor = (type: string) => {
    switch (type) {
      case 'warning': return '#f59e0b';
      case 'info': return '#3b82f6';
      case 'success': return '#10b981';
      default: return '#6b7280';
    }
  };

  return (
    <ScrollView 
      style={styles.container}
      refreshControl={
        <RefreshControl refreshing={isRefreshing} onRefresh={handleRefresh} />
      }
    >
      {/* Header */}
      <View style={styles.header}>
        <View style={styles.headerContent}>
          <View>
            <Text style={styles.greeting}>{getGreeting()}</Text>
            <Text style={styles.userName}>{user.firstName} {user.lastName}</Text>
            <Text style={styles.userRole}>{user.role}</Text>
          </View>
          <View style={styles.headerIcon}>
            <Ionicons name="person-circle" size={60} color="#fff" />
          </View>
        </View>
      </View>

      {/* Stats Cards */}
      <View style={styles.statsContainer}>
        <View style={styles.statsRow}>
          <View style={[styles.statCard, styles.primaryCard]}>
            <Ionicons name="car-outline" size={24} color="#fff" />
            <Text style={styles.statNumber}>{stats.totalVehicles}</Text>
            <Text style={styles.statLabel}>Total Vehicles</Text>
          </View>
          
          <View style={[styles.statCard, styles.successCard]}>
            <Ionicons name="checkmark-circle-outline" size={24} color="#fff" />
            <Text style={styles.statNumber}>{stats.activeVehicles}</Text>
            <Text style={styles.statLabel}>Active</Text>
          </View>
        </View>

        <View style={styles.statsRow}>
          <View style={[styles.statCard, styles.warningCard]}>
            <Ionicons name="construct-outline" size={24} color="#fff" />
            <Text style={styles.statNumber}>{stats.maintenanceVehicles}</Text>
            <Text style={styles.statLabel}>Maintenance</Text>
          </View>
          
          <View style={[styles.statCard, styles.infoCard]}>
            <Ionicons name="shield-checkmark-outline" size={24} color="#fff" />
            <Text style={styles.statNumber}>{stats.safetyScore}%</Text>
            <Text style={styles.statLabel}>Safety Score</Text>
          </View>
        </View>
      </View>

      {/* Quick Actions */}
      <View style={styles.section}>
        <Text style={styles.sectionTitle}>Quick Actions</Text>
        <View style={styles.actionsGrid}>
          <TouchableOpacity style={styles.actionButton}>
            <Ionicons name="car-outline" size={24} color={theme.colors.primary} />
            <Text style={styles.actionText}>View Vehicles</Text>
          </TouchableOpacity>
          
          <TouchableOpacity style={styles.actionButton}>
            <Ionicons name="construct-outline" size={24} color={theme.colors.primary} />
            <Text style={styles.actionText}>Maintenance</Text>
          </TouchableOpacity>
          
          <TouchableOpacity style={styles.actionButton}>
            <Ionicons name="warning-outline" size={24} color={theme.colors.primary} />
            <Text style={styles.actionText}>Emergency</Text>
          </TouchableOpacity>
          
          <TouchableOpacity style={styles.actionButton}>
            <Ionicons name="analytics-outline" size={24} color={theme.colors.primary} />
            <Text style={styles.actionText}>Reports</Text>
          </TouchableOpacity>
        </View>
      </View>

      {/* Recent Alerts */}
      <View style={styles.section}>
        <Text style={styles.sectionTitle}>Recent Alerts</Text>
        {recentAlerts.map((alert) => (
          <View key={alert.id} style={styles.alertItem}>
            <Ionicons 
              name={getAlertIcon(alert.type) as any} 
              size={20} 
              color={getAlertColor(alert.type)} 
            />
            <View style={styles.alertContent}>
              <Text style={styles.alertMessage}>{alert.message}</Text>
              <Text style={styles.alertTime}>{alert.time}</Text>
            </View>
          </View>
        ))}
      </View>

      {/* Trip Stats */}
      <View style={styles.section}>
        <Text style={styles.sectionTitle}>Today's Performance</Text>
        <View style={styles.performanceCard}>
          <View style={styles.performanceItem}>
            <Text style={styles.performanceNumber}>{stats.totalTrips}</Text>
            <Text style={styles.performanceLabel}>Total Trips</Text>
          </View>
          <View style={styles.performanceItem}>
            <Text style={styles.performanceNumber}>{stats.completedTrips}</Text>
            <Text style={styles.performanceLabel}>Completed</Text>
          </View>
          <View style={styles.performanceItem}>
            <Text style={styles.performanceNumber}>
              {Math.round((stats.completedTrips / stats.totalTrips) * 100)}%
            </Text>
            <Text style={styles.performanceLabel}>Success Rate</Text>
          </View>
        </View>
      </View>
    </ScrollView>
  );
}

const { width } = Dimensions.get('window');

const styles = StyleSheet.create({
  container: {
    flex: 1,
    backgroundColor: '#f8fafc',
  },
  header: {
    backgroundColor: theme.colors.primary,
    paddingTop: 20,
    paddingBottom: 30,
    paddingHorizontal: 20,
  },
  headerContent: {
    flexDirection: 'row',
    justifyContent: 'space-between',
    alignItems: 'center',
  },
  greeting: {
    fontSize: 16,
    color: '#e3f2fd',
    marginBottom: 4,
  },
  userName: {
    fontSize: 24,
    fontWeight: 'bold',
    color: '#fff',
    marginBottom: 4,
  },
  userRole: {
    fontSize: 14,
    color: '#e3f2fd',
    textTransform: 'uppercase',
    letterSpacing: 0.5,
  },
  headerIcon: {
    opacity: 0.8,
  },
  statsContainer: {
    padding: 20,
  },
  statsRow: {
    flexDirection: 'row',
    marginBottom: 12,
  },
  statCard: {
    flex: 1,
    padding: 16,
    borderRadius: 12,
    marginHorizontal: 6,
    alignItems: 'center',
  },
  primaryCard: {
    backgroundColor: theme.colors.primary,
  },
  successCard: {
    backgroundColor: '#10b981',
  },
  warningCard: {
    backgroundColor: '#f59e0b',
  },
  infoCard: {
    backgroundColor: '#3b82f6',
  },
  statNumber: {
    fontSize: 24,
    fontWeight: 'bold',
    color: '#fff',
    marginTop: 8,
    marginBottom: 4,
  },
  statLabel: {
    fontSize: 12,
    color: '#fff',
    opacity: 0.9,
    textAlign: 'center',
  },
  section: {
    marginHorizontal: 20,
    marginBottom: 24,
  },
  sectionTitle: {
    fontSize: 18,
    fontWeight: 'bold',
    color: '#333',
    marginBottom: 16,
  },
  actionsGrid: {
    flexDirection: 'row',
    flexWrap: 'wrap',
    justifyContent: 'space-between',
  },
  actionButton: {
    width: (width - 60) / 2,
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
  actionText: {
    fontSize: 14,
    color: '#333',
    marginTop: 8,
    textAlign: 'center',
  },
  alertItem: {
    flexDirection: 'row',
    backgroundColor: '#fff',
    padding: 16,
    borderRadius: 8,
    marginBottom: 8,
    alignItems: 'center',
    shadowColor: '#000',
    shadowOffset: { width: 0, height: 1 },
    shadowOpacity: 0.05,
    shadowRadius: 2,
    elevation: 1,
  },
  alertContent: {
    flex: 1,
    marginLeft: 12,
  },
  alertMessage: {
    fontSize: 14,
    color: '#333',
    marginBottom: 4,
  },
  alertTime: {
    fontSize: 12,
    color: '#666',
  },
  performanceCard: {
    backgroundColor: '#fff',
    padding: 20,
    borderRadius: 12,
    flexDirection: 'row',
    justifyContent: 'space-around',
    shadowColor: '#000',
    shadowOffset: { width: 0, height: 2 },
    shadowOpacity: 0.1,
    shadowRadius: 4,
    elevation: 2,
  },
  performanceItem: {
    alignItems: 'center',
  },
  performanceNumber: {
    fontSize: 20,
    fontWeight: 'bold',
    color: theme.colors.primary,
    marginBottom: 4,
  },
  performanceLabel: {
    fontSize: 12,
    color: '#666',
    textAlign: 'center',
  },
});
