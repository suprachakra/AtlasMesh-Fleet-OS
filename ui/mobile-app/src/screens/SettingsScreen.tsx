import React, { useState, useEffect } from 'react';
import {
  View,
  Text,
  ScrollView,
  StyleSheet,
  TouchableOpacity,
  Switch,
  Alert,
  TextInput,
} from 'react-native';
import { Ionicons } from '@expo/vector-icons';
import { theme } from '../theme/theme';
import { User } from '../types/User';

interface SettingsScreenProps {
  navigation: any;
  user: User;
  onLogout: () => void;
}

export default function SettingsScreen({ navigation, user, onLogout }: SettingsScreenProps) {
  const [settings, setSettings] = useState({
    notifications: true,
    locationTracking: true,
    darkMode: false,
    autoSync: true,
    emergencyAlerts: true,
    maintenanceReminders: true,
    tripUpdates: true,
  });

  const [profile, setProfile] = useState({
    firstName: user.firstName,
    lastName: user.lastName,
    email: user.email,
    phone: user.phone || '',
    department: user.department || '',
  });

  const [isEditing, setIsEditing] = useState(false);

  const handleSettingChange = (key: string, value: boolean) => {
    setSettings(prev => ({ ...prev, [key]: value }));
  };

  const handleProfileChange = (key: string, value: string) => {
    setProfile(prev => ({ ...prev, [key]: value }));
  };

  const handleSaveProfile = () => {
    // Simulate API call
    Alert.alert('Success', 'Profile updated successfully');
    setIsEditing(false);
  };

  const handleLogout = () => {
    Alert.alert(
      'Logout',
      'Are you sure you want to logout?',
      [
        { text: 'Cancel', style: 'cancel' },
        { text: 'Logout', style: 'destructive', onPress: onLogout },
      ]
    );
  };

  const handleChangePassword = () => {
    Alert.alert('Change Password', 'Password change functionality will be implemented');
  };

  const handleDeleteAccount = () => {
    Alert.alert(
      'Delete Account',
      'This action cannot be undone. Are you sure you want to delete your account?',
      [
        { text: 'Cancel', style: 'cancel' },
        { text: 'Delete', style: 'destructive', onPress: () => {
          Alert.alert('Account Deleted', 'Your account has been deleted');
        }},
      ]
    );
  };

  const renderSettingItem = (
    icon: string,
    title: string,
    subtitle: string,
    value: boolean,
    onValueChange: (value: boolean) => void,
    color?: string
  ) => (
    <View style={styles.settingItem}>
      <View style={styles.settingLeft}>
        <Ionicons name={icon as any} size={24} color={color || theme.colors.primary} />
        <View style={styles.settingText}>
          <Text style={styles.settingTitle}>{title}</Text>
          <Text style={styles.settingSubtitle}>{subtitle}</Text>
        </View>
      </View>
      <Switch
        value={value}
        onValueChange={onValueChange}
        trackColor={{ false: '#e2e8f0', true: theme.colors.primary }}
        thumbColor={value ? '#fff' : '#f4f3f4'}
      />
    </View>
  );

  const renderProfileField = (label: string, value: string, key: string, placeholder?: string) => (
    <View style={styles.profileField}>
      <Text style={styles.profileLabel}>{label}</Text>
      {isEditing ? (
        <TextInput
          style={styles.profileInput}
          value={value}
          onChangeText={(text) => handleProfileChange(key, text)}
          placeholder={placeholder}
          placeholderTextColor="#999"
        />
      ) : (
        <Text style={styles.profileValue}>{value || 'Not set'}</Text>
      )}
    </View>
  );

  return (
    <ScrollView style={styles.container}>
      {/* Header */}
      <View style={styles.header}>
        <Text style={styles.headerTitle}>Settings</Text>
        <TouchableOpacity style={styles.headerButton}>
          <Ionicons name="help-circle-outline" size={24} color={theme.colors.primary} />
        </TouchableOpacity>
      </View>

      {/* Profile Section */}
      <View style={styles.section}>
        <Text style={styles.sectionTitle}>Profile</Text>
        <View style={styles.profileCard}>
          <View style={styles.profileHeader}>
            <View style={styles.profileAvatar}>
              <Ionicons name="person" size={32} color="#fff" />
            </View>
            <View style={styles.profileInfo}>
              <Text style={styles.profileName}>{user.firstName} {user.lastName}</Text>
              <Text style={styles.profileRole}>{user.role}</Text>
            </View>
            <TouchableOpacity
              style={styles.editButton}
              onPress={() => setIsEditing(!isEditing)}
            >
              <Ionicons name={isEditing ? "close" : "pencil"} size={20} color={theme.colors.primary} />
            </TouchableOpacity>
          </View>

          {renderProfileField('First Name', profile.firstName, 'firstName', 'Enter first name')}
          {renderProfileField('Last Name', profile.lastName, 'lastName', 'Enter last name')}
          {renderProfileField('Email', profile.email, 'email', 'Enter email')}
          {renderProfileField('Phone', profile.phone, 'phone', 'Enter phone number')}
          {renderProfileField('Department', profile.department, 'department', 'Enter department')}

          {isEditing && (
            <View style={styles.profileActions}>
              <TouchableOpacity
                style={styles.cancelButton}
                onPress={() => setIsEditing(false)}
              >
                <Text style={styles.cancelButtonText}>Cancel</Text>
              </TouchableOpacity>
              <TouchableOpacity
                style={styles.saveButton}
                onPress={handleSaveProfile}
              >
                <Text style={styles.saveButtonText}>Save</Text>
              </TouchableOpacity>
            </View>
          )}
        </View>
      </View>

      {/* Notifications Section */}
      <View style={styles.section}>
        <Text style={styles.sectionTitle}>Notifications</Text>
        <View style={styles.settingsCard}>
          {renderSettingItem(
            'notifications-outline',
            'Push Notifications',
            'Receive push notifications',
            settings.notifications,
            (value) => handleSettingChange('notifications', value)
          )}
          {renderSettingItem(
            'warning-outline',
            'Emergency Alerts',
            'Critical safety alerts',
            settings.emergencyAlerts,
            (value) => handleSettingChange('emergencyAlerts', value),
            '#ef4444'
          )}
          {renderSettingItem(
            'construct-outline',
            'Maintenance Reminders',
            'Vehicle maintenance alerts',
            settings.maintenanceReminders,
            (value) => handleSettingChange('maintenanceReminders', value),
            '#f59e0b'
          )}
          {renderSettingItem(
            'car-outline',
            'Trip Updates',
            'Real-time trip notifications',
            settings.tripUpdates,
            (value) => handleSettingChange('tripUpdates', value),
            '#10b981'
          )}
        </View>
      </View>

      {/* Privacy & Security Section */}
      <View style={styles.section}>
        <Text style={styles.sectionTitle}>Privacy & Security</Text>
        <View style={styles.settingsCard}>
          {renderSettingItem(
            'location-outline',
            'Location Tracking',
            'Allow location tracking for fleet operations',
            settings.locationTracking,
            (value) => handleSettingChange('locationTracking', value)
          )}
          {renderSettingItem(
            'sync-outline',
            'Auto Sync',
            'Automatically sync data',
            settings.autoSync,
            (value) => handleSettingChange('autoSync', value)
          )}
        </View>
      </View>

      {/* Appearance Section */}
      <View style={styles.section}>
        <Text style={styles.sectionTitle}>Appearance</Text>
        <View style={styles.settingsCard}>
          {renderSettingItem(
            'moon-outline',
            'Dark Mode',
            'Use dark theme',
            settings.darkMode,
            (value) => handleSettingChange('darkMode', value)
          )}
        </View>
      </View>

      {/* Account Actions */}
      <View style={styles.section}>
        <Text style={styles.sectionTitle}>Account</Text>
        <View style={styles.settingsCard}>
          <TouchableOpacity style={styles.actionItem} onPress={handleChangePassword}>
            <View style={styles.actionLeft}>
              <Ionicons name="lock-closed-outline" size={24} color={theme.colors.primary} />
              <Text style={styles.actionTitle}>Change Password</Text>
            </View>
            <Ionicons name="chevron-forward" size={20} color="#666" />
          </TouchableOpacity>

          <TouchableOpacity style={styles.actionItem} onPress={() => navigation.navigate('About')}>
            <View style={styles.actionLeft}>
              <Ionicons name="information-circle-outline" size={24} color={theme.colors.primary} />
              <Text style={styles.actionTitle}>About</Text>
            </View>
            <Ionicons name="chevron-forward" size={20} color="#666" />
          </TouchableOpacity>

          <TouchableOpacity style={styles.actionItem} onPress={() => navigation.navigate('Help')}>
            <View style={styles.actionLeft}>
              <Ionicons name="help-circle-outline" size={24} color={theme.colors.primary} />
              <Text style={styles.actionTitle}>Help & Support</Text>
            </View>
            <Ionicons name="chevron-forward" size={20} color="#666" />
          </TouchableOpacity>
        </View>
      </View>

      {/* Danger Zone */}
      <View style={styles.section}>
        <Text style={styles.sectionTitle}>Danger Zone</Text>
        <View style={styles.settingsCard}>
          <TouchableOpacity style={styles.dangerItem} onPress={handleLogout}>
            <View style={styles.actionLeft}>
              <Ionicons name="log-out-outline" size={24} color="#ef4444" />
              <Text style={[styles.actionTitle, { color: '#ef4444' }]}>Logout</Text>
            </View>
            <Ionicons name="chevron-forward" size={20} color="#ef4444" />
          </TouchableOpacity>

          <TouchableOpacity style={styles.dangerItem} onPress={handleDeleteAccount}>
            <View style={styles.actionLeft}>
              <Ionicons name="trash-outline" size={24} color="#ef4444" />
              <Text style={[styles.actionTitle, { color: '#ef4444' }]}>Delete Account</Text>
            </View>
            <Ionicons name="chevron-forward" size={20} color="#ef4444" />
          </TouchableOpacity>
        </View>
      </View>

      {/* App Version */}
      <View style={styles.footer}>
        <Text style={styles.footerText}>AtlasMesh Fleet OS Mobile</Text>
        <Text style={styles.footerText}>Version 2.0.0</Text>
        <Text style={styles.footerText}>© 2024 AtlasMesh Technologies</Text>
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
  section: {
    margin: 20,
  },
  sectionTitle: {
    fontSize: 18,
    fontWeight: 'bold',
    color: '#333',
    marginBottom: 16,
  },
  profileCard: {
    backgroundColor: '#fff',
    borderRadius: 12,
    padding: 20,
    shadowColor: '#000',
    shadowOffset: { width: 0, height: 2 },
    shadowOpacity: 0.1,
    shadowRadius: 4,
    elevation: 2,
  },
  profileHeader: {
    flexDirection: 'row',
    alignItems: 'center',
    marginBottom: 20,
  },
  profileAvatar: {
    width: 60,
    height: 60,
    borderRadius: 30,
    backgroundColor: theme.colors.primary,
    justifyContent: 'center',
    alignItems: 'center',
    marginRight: 16,
  },
  profileInfo: {
    flex: 1,
  },
  profileName: {
    fontSize: 18,
    fontWeight: 'bold',
    color: '#333',
    marginBottom: 4,
  },
  profileRole: {
    fontSize: 14,
    color: '#666',
  },
  editButton: {
    padding: 8,
  },
  profileField: {
    marginBottom: 16,
  },
  profileLabel: {
    fontSize: 14,
    color: '#666',
    marginBottom: 8,
  },
  profileInput: {
    borderWidth: 1,
    borderColor: '#e2e8f0',
    borderRadius: 8,
    padding: 12,
    fontSize: 16,
    color: '#333',
    backgroundColor: '#f8fafc',
  },
  profileValue: {
    fontSize: 16,
    color: '#333',
    paddingVertical: 12,
  },
  profileActions: {
    flexDirection: 'row',
    justifyContent: 'space-between',
    marginTop: 20,
  },
  cancelButton: {
    flex: 1,
    padding: 12,
    borderRadius: 8,
    borderWidth: 1,
    borderColor: '#e2e8f0',
    alignItems: 'center',
    marginRight: 8,
  },
  cancelButtonText: {
    fontSize: 16,
    color: '#666',
    fontWeight: 'bold',
  },
  saveButton: {
    flex: 1,
    padding: 12,
    borderRadius: 8,
    backgroundColor: theme.colors.primary,
    alignItems: 'center',
    marginLeft: 8,
  },
  saveButtonText: {
    fontSize: 16,
    color: '#fff',
    fontWeight: 'bold',
  },
  settingsCard: {
    backgroundColor: '#fff',
    borderRadius: 12,
    shadowColor: '#000',
    shadowOffset: { width: 0, height: 2 },
    shadowOpacity: 0.1,
    shadowRadius: 4,
    elevation: 2,
  },
  settingItem: {
    flexDirection: 'row',
    alignItems: 'center',
    justifyContent: 'space-between',
    padding: 16,
    borderBottomWidth: 1,
    borderBottomColor: '#f1f5f9',
  },
  settingLeft: {
    flexDirection: 'row',
    alignItems: 'center',
    flex: 1,
  },
  settingText: {
    marginLeft: 16,
    flex: 1,
  },
  settingTitle: {
    fontSize: 16,
    fontWeight: '500',
    color: '#333',
    marginBottom: 2,
  },
  settingSubtitle: {
    fontSize: 14,
    color: '#666',
  },
  actionItem: {
    flexDirection: 'row',
    alignItems: 'center',
    justifyContent: 'space-between',
    padding: 16,
    borderBottomWidth: 1,
    borderBottomColor: '#f1f5f9',
  },
  actionLeft: {
    flexDirection: 'row',
    alignItems: 'center',
    flex: 1,
  },
  actionTitle: {
    fontSize: 16,
    fontWeight: '500',
    color: '#333',
    marginLeft: 16,
  },
  dangerItem: {
    flexDirection: 'row',
    alignItems: 'center',
    justifyContent: 'space-between',
    padding: 16,
    borderBottomWidth: 1,
    borderBottomColor: '#f1f5f9',
  },
  footer: {
    alignItems: 'center',
    padding: 20,
    marginBottom: 20,
  },
  footerText: {
    fontSize: 12,
    color: '#999',
    marginBottom: 4,
  },
});
