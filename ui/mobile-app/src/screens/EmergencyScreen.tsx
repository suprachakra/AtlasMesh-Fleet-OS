import React, { useState, useEffect } from 'react';
import {
  View,
  StyleSheet,
  ScrollView,
  Alert,
  Linking,
  Platform,
} from 'react-native';
import {
  Card,
  Title,
  Paragraph,
  Button,
  FAB,
  List,
  Chip,
  Surface,
  Text,
  IconButton,
  ActivityIndicator,
} from 'react-native-paper';
import { Ionicons } from '@expo/vector-icons';
import * as Location from 'expo-location';
import { theme } from '../theme/theme';
import { AuthService } from '../services/AuthService';

interface EmergencyContact {
  id: string;
  name: string;
  phone: string;
  type: 'police' | 'fire' | 'medical' | 'roadside' | 'fleet' | 'supervisor';
  priority: number;
  available24h: boolean;
  description?: string;
}

interface EmergencyIncident {
  id: string;
  type: 'accident' | 'breakdown' | 'medical' | 'security' | 'weather' | 'other';
  severity: 'low' | 'medium' | 'high' | 'critical';
  status: 'reported' | 'acknowledged' | 'responding' | 'resolved';
  location: {
    latitude: number;
    longitude: number;
    address?: string;
  };
  description: string;
  reportedAt: Date;
  reportedBy: string;
  assignedTo?: string;
  estimatedResponse?: string;
}

const EmergencyScreen: React.FC = () => {
  const [incidents, setIncidents] = useState<EmergencyIncident[]>([]);
  const [emergencyContacts, setEmergencyContacts] = useState<EmergencyContact[]>([]);
  const [currentLocation, setCurrentLocation] = useState<Location.LocationObject | null>(null);
  const [isLoading, setIsLoading] = useState(false);
  const [isReportingEmergency, setIsReportingEmergency] = useState(false);

  useEffect(() => {
    loadEmergencyData();
    getCurrentLocation();
  }, []);

  const loadEmergencyData = async () => {
    try {
      setIsLoading(true);
      
      // Load emergency contacts
      const contactsResponse = await AuthService.makeAuthenticatedRequest(
        'https://api.atlasmesh.ae/api/v1/emergency/contacts'
      );
      const contacts = await contactsResponse.json();
      setEmergencyContacts(contacts);

      // Load recent incidents
      const incidentsResponse = await AuthService.makeAuthenticatedRequest(
        'https://api.atlasmesh.ae/api/v1/emergency/incidents'
      );
      const incidentsData = await incidentsResponse.json();
      setIncidents(incidentsData);
      
    } catch (error) {
      console.error('Failed to load emergency data:', error);
      // Load mock data as fallback
      loadMockData();
    } finally {
      setIsLoading(false);
    }
  };

  const loadMockData = () => {
    setEmergencyContacts([
      {
        id: '1',
        name: 'UAE Emergency Services',
        phone: '999',
        type: 'police',
        priority: 1,
        available24h: true,
        description: 'Police, Fire, Ambulance'
      },
      {
        id: '2',
        name: 'Dubai Police',
        phone: '+971-4-609-6999',
        type: 'police',
        priority: 2,
        available24h: true,
        description: 'Dubai Police Emergency'
      },
      {
        id: '3',
        name: 'Fleet Control Center',
        phone: '+971-50-123-4567',
        type: 'fleet',
        priority: 3,
        available24h: true,
        description: 'AtlasMesh Fleet Emergency'
      },
      {
        id: '4',
        name: 'Roadside Assistance',
        phone: '+971-800-4357',
        type: 'roadside',
        priority: 4,
        available24h: true,
        description: '24/7 Vehicle Recovery'
      },
    ]);

    setIncidents([
      {
        id: '1',
        type: 'breakdown',
        severity: 'medium',
        status: 'responding',
        location: {
          latitude: 25.2048,
          longitude: 55.2708,
          address: 'Sheikh Zayed Road, Dubai'
        },
        description: 'Vehicle engine overheating',
        reportedAt: new Date(Date.now() - 30 * 60 * 1000),
        reportedBy: 'Driver Ahmed',
        assignedTo: 'Technician Team A',
        estimatedResponse: '15 minutes'
      }
    ]);
  };

  const getCurrentLocation = async () => {
    try {
      const { status } = await Location.requestForegroundPermissionsAsync();
      if (status !== 'granted') {
        Alert.alert('Permission Denied', 'Location permission is required for emergency services');
        return;
      }

      const location = await Location.getCurrentPositionAsync({});
      setCurrentLocation(location);
    } catch (error) {
      console.error('Failed to get location:', error);
    }
  };

  const makeEmergencyCall = (contact: EmergencyContact) => {
    Alert.alert(
      'Emergency Call',
      `Call ${contact.name} at ${contact.phone}?`,
      [
        { text: 'Cancel', style: 'cancel' },
        {
          text: 'Call',
          style: 'destructive',
          onPress: () => {
            const phoneUrl = Platform.OS === 'ios' ? `tel:${contact.phone}` : `tel:${contact.phone}`;
            Linking.openURL(phoneUrl);
          }
        }
      ]
    );
  };

  const reportEmergency = async (type: string, severity: string, description: string) => {
    try {
      setIsReportingEmergency(true);
      
      const location = currentLocation || await Location.getCurrentPositionAsync({});
      
      const incident = {
        type,
        severity,
        description,
        location: {
          latitude: location.coords.latitude,
          longitude: location.coords.longitude,
        },
        timestamp: new Date().toISOString(),
      };

      const response = await AuthService.makeAuthenticatedRequest(
        'https://api.atlasmesh.ae/api/v1/emergency/incidents',
        {
          method: 'POST',
          body: JSON.stringify(incident),
        }
      );

      if (response.ok) {
        Alert.alert('Emergency Reported', 'Your emergency has been reported and help is on the way.');
        loadEmergencyData(); // Refresh data
      } else {
        throw new Error('Failed to report emergency');
      }
    } catch (error) {
      console.error('Failed to report emergency:', error);
      Alert.alert('Error', 'Failed to report emergency. Please try calling directly.');
    } finally {
      setIsReportingEmergency(false);
    }
  };

  const showEmergencyReportDialog = () => {
    Alert.alert(
      'Report Emergency',
      'What type of emergency are you reporting?',
      [
        { text: 'Cancel', style: 'cancel' },
        {
          text: 'Medical Emergency',
          onPress: () => reportEmergency('medical', 'critical', 'Medical emergency reported via mobile app')
        },
        {
          text: 'Vehicle Accident',
          onPress: () => reportEmergency('accident', 'high', 'Vehicle accident reported via mobile app')
        },
        {
          text: 'Vehicle Breakdown',
          onPress: () => reportEmergency('breakdown', 'medium', 'Vehicle breakdown reported via mobile app')
        },
        {
          text: 'Security Issue',
          onPress: () => reportEmergency('security', 'high', 'Security issue reported via mobile app')
        },
      ]
    );
  };

  const getSeverityColor = (severity: string) => {
    switch (severity) {
      case 'critical': return theme.colors.error;
      case 'high': return theme.colors.warning;
      case 'medium': return theme.colors.info;
      case 'low': return theme.colors.success;
      default: return theme.colors.text;
    }
  };

  const getStatusColor = (status: string) => {
    switch (status) {
      case 'resolved': return theme.colors.success;
      case 'responding': return theme.colors.info;
      case 'acknowledged': return theme.colors.warning;
      case 'reported': return theme.colors.error;
      default: return theme.colors.text;
    }
  };

  const getContactIcon = (type: string) => {
    switch (type) {
      case 'police': return 'shield-outline';
      case 'fire': return 'flame-outline';
      case 'medical': return 'medical-outline';
      case 'roadside': return 'car-outline';
      case 'fleet': return 'business-outline';
      case 'supervisor': return 'person-outline';
      default: return 'call-outline';
    }
  };

  if (isLoading) {
    return (
      <View style={styles.loadingContainer}>
        <ActivityIndicator size="large" color={theme.colors.primary} />
        <Text style={styles.loadingText}>Loading emergency information...</Text>
      </View>
    );
  }

  return (
    <View style={styles.container}>
      <ScrollView style={styles.scrollView} showsVerticalScrollIndicator={false}>
        {/* Emergency Alert Banner */}
        <Surface style={styles.alertBanner}>
          <View style={styles.alertContent}>
            <Ionicons name="warning" size={24} color={theme.colors.error} />
            <View style={styles.alertText}>
              <Title style={styles.alertTitle}>Emergency Services</Title>
              <Paragraph style={styles.alertDescription}>
                For immediate life-threatening emergencies, call 999
              </Paragraph>
            </View>
          </View>
        </Surface>

        {/* Quick Actions */}
        <Card style={styles.card}>
          <Card.Content>
            <Title>Quick Actions</Title>
            <View style={styles.quickActions}>
              <Button
                mode="contained"
                icon="call"
                onPress={() => makeEmergencyCall(emergencyContacts[0])}
                style={[styles.quickActionButton, { backgroundColor: theme.colors.error }]}
                labelStyle={styles.quickActionLabel}
              >
                Call 999
              </Button>
              <Button
                mode="contained"
                icon="car"
                onPress={() => makeEmergencyCall(emergencyContacts.find(c => c.type === 'roadside') || emergencyContacts[0])}
                style={[styles.quickActionButton, { backgroundColor: theme.colors.warning }]}
                labelStyle={styles.quickActionLabel}
              >
                Roadside
              </Button>
              <Button
                mode="contained"
                icon="business"
                onPress={() => makeEmergencyCall(emergencyContacts.find(c => c.type === 'fleet') || emergencyContacts[0])}
                style={[styles.quickActionButton, { backgroundColor: theme.colors.primary }]}
                labelStyle={styles.quickActionLabel}
              >
                Fleet Control
              </Button>
            </View>
          </Card.Content>
        </Card>

        {/* Active Incidents */}
        {incidents.length > 0 && (
          <Card style={styles.card}>
            <Card.Content>
              <Title>Active Incidents</Title>
              {incidents.map((incident) => (
                <Surface key={incident.id} style={styles.incidentCard}>
                  <View style={styles.incidentHeader}>
                    <View style={styles.incidentInfo}>
                      <Text style={styles.incidentType}>
                        {incident.type.charAt(0).toUpperCase() + incident.type.slice(1)}
                      </Text>
                      <View style={styles.incidentChips}>
                        <Chip
                          style={[styles.severityChip, { backgroundColor: getSeverityColor(incident.severity) }]}
                          textStyle={styles.chipText}
                        >
                          {incident.severity.toUpperCase()}
                        </Chip>
                        <Chip
                          style={[styles.statusChip, { backgroundColor: getStatusColor(incident.status) }]}
                          textStyle={styles.chipText}
                        >
                          {incident.status.toUpperCase()}
                        </Chip>
                      </View>
                    </View>
                    <IconButton
                      icon="information-outline"
                      size={20}
                      onPress={() => {
                        Alert.alert(
                          'Incident Details',
                          `Location: ${incident.location.address || 'Unknown'}\n` +
                          `Reported: ${incident.reportedAt.toLocaleString()}\n` +
                          `By: ${incident.reportedBy}\n` +
                          `Assigned: ${incident.assignedTo || 'Unassigned'}\n` +
                          `ETA: ${incident.estimatedResponse || 'Unknown'}\n\n` +
                          `Description: ${incident.description}`
                        );
                      }}
                    />
                  </View>
                  <Paragraph style={styles.incidentDescription}>
                    {incident.description}
                  </Paragraph>
                  {incident.estimatedResponse && (
                    <Text style={styles.estimatedResponse}>
                      ETA: {incident.estimatedResponse}
                    </Text>
                  )}
                </Surface>
              ))}
            </Card.Content>
          </Card>
        )}

        {/* Emergency Contacts */}
        <Card style={styles.card}>
          <Card.Content>
            <Title>Emergency Contacts</Title>
            {emergencyContacts.map((contact) => (
              <List.Item
                key={contact.id}
                title={contact.name}
                description={`${contact.phone} • ${contact.description}`}
                left={(props) => (
                  <List.Icon
                    {...props}
                    icon={getContactIcon(contact.type)}
                    color={theme.colors.primary}
                  />
                )}
                right={(props) => (
                  <View style={styles.contactActions}>
                    {contact.available24h && (
                      <Chip style={styles.availabilityChip} textStyle={styles.availabilityText}>
                        24/7
                      </Chip>
                    )}
                    <IconButton
                      {...props}
                      icon="call"
                      size={20}
                      onPress={() => makeEmergencyCall(contact)}
                    />
                  </View>
                )}
                onPress={() => makeEmergencyCall(contact)}
                style={styles.contactItem}
              />
            ))}
          </Card.Content>
        </Card>

        {/* Location Info */}
        {currentLocation && (
          <Card style={styles.card}>
            <Card.Content>
              <Title>Current Location</Title>
              <Paragraph>
                Latitude: {currentLocation.coords.latitude.toFixed(6)}
              </Paragraph>
              <Paragraph>
                Longitude: {currentLocation.coords.longitude.toFixed(6)}
              </Paragraph>
              <Paragraph style={styles.locationNote}>
                This location will be shared when reporting emergencies
              </Paragraph>
            </Card.Content>
          </Card>
        )}
      </ScrollView>

      {/* Floating Action Button */}
      <FAB
        style={[styles.fab, { backgroundColor: theme.colors.error }]}
        icon="alert"
        label="Report Emergency"
        onPress={showEmergencyReportDialog}
        loading={isReportingEmergency}
        disabled={isReportingEmergency}
      />
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
  scrollView: {
    flex: 1,
    padding: theme.spacing.md,
  },
  alertBanner: {
    marginBottom: theme.spacing.md,
    padding: theme.spacing.md,
    backgroundColor: theme.colors.error + '20',
    borderRadius: theme.roundness,
    borderLeftWidth: 4,
    borderLeftColor: theme.colors.error,
  },
  alertContent: {
    flexDirection: 'row',
    alignItems: 'center',
  },
  alertText: {
    marginLeft: theme.spacing.md,
    flex: 1,
  },
  alertTitle: {
    color: theme.colors.error,
    fontSize: 18,
    fontWeight: 'bold',
  },
  alertDescription: {
    color: theme.colors.text,
    fontSize: 14,
  },
  card: {
    marginBottom: theme.spacing.md,
    elevation: 2,
  },
  quickActions: {
    flexDirection: 'row',
    justifyContent: 'space-between',
    marginTop: theme.spacing.md,
  },
  quickActionButton: {
    flex: 1,
    marginHorizontal: theme.spacing.xs,
  },
  quickActionLabel: {
    fontSize: 12,
    fontWeight: 'bold',
  },
  incidentCard: {
    marginTop: theme.spacing.md,
    padding: theme.spacing.md,
    borderRadius: theme.roundness,
    elevation: 1,
  },
  incidentHeader: {
    flexDirection: 'row',
    justifyContent: 'space-between',
    alignItems: 'flex-start',
    marginBottom: theme.spacing.sm,
  },
  incidentInfo: {
    flex: 1,
  },
  incidentType: {
    fontSize: 16,
    fontWeight: 'bold',
    color: theme.colors.text,
    marginBottom: theme.spacing.xs,
  },
  incidentChips: {
    flexDirection: 'row',
    gap: theme.spacing.xs,
  },
  severityChip: {
    height: 24,
  },
  statusChip: {
    height: 24,
  },
  chipText: {
    fontSize: 10,
    color: 'white',
    fontWeight: 'bold',
  },
  incidentDescription: {
    fontSize: 14,
    color: theme.colors.text,
    marginBottom: theme.spacing.xs,
  },
  estimatedResponse: {
    fontSize: 12,
    color: theme.colors.primary,
    fontWeight: 'bold',
  },
  contactItem: {
    borderBottomWidth: 1,
    borderBottomColor: theme.colors.disabled + '30',
  },
  contactActions: {
    flexDirection: 'row',
    alignItems: 'center',
  },
  availabilityChip: {
    height: 20,
    backgroundColor: theme.colors.success,
    marginRight: theme.spacing.xs,
  },
  availabilityText: {
    fontSize: 10,
    color: 'white',
    fontWeight: 'bold',
  },
  locationNote: {
    fontSize: 12,
    color: theme.colors.disabled,
    fontStyle: 'italic',
    marginTop: theme.spacing.xs,
  },
  fab: {
    position: 'absolute',
    margin: theme.spacing.md,
    right: 0,
    bottom: 0,
  },
});

export default EmergencyScreen;
