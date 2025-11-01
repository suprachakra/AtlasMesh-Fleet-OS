import * as Notifications from 'expo-notifications';
import { Platform } from 'react-native';
import { Alert } from 'react-native';

export interface NotificationData {
  title: string;
  body: string;
  data?: any;
  sound?: boolean;
  priority?: 'min' | 'low' | 'default' | 'high' | 'max';
  badge?: number;
}

export interface NotificationPermissionStatus {
  granted: boolean;
  canAskAgain: boolean;
  status: Notifications.PermissionStatus;
}

class NotificationService {
  private static instance: NotificationService;
  private notificationCallbacks: ((notification: Notifications.Notification) => void)[] = [];
  private responseCallbacks: ((response: Notifications.NotificationResponse) => void)[] = [];

  private constructor() {
    this.setupNotificationHandlers();
  }

  public static getInstance(): NotificationService {
    if (!NotificationService.instance) {
      NotificationService.instance = new NotificationService();
    }
    return NotificationService.instance;
  }

  /**
   * Initialize notification service
   */
  public static async initialize(): Promise<boolean> {
    try {
      const service = NotificationService.getInstance();
      const permissionStatus = await service.requestPermissions();
      
      if (!permissionStatus.granted) {
        Alert.alert(
          'Notification Permission Required',
          'This app needs notification access to send you important fleet updates and emergency alerts.',
          [
            { text: 'Cancel', style: 'cancel' },
            { text: 'Settings', onPress: () => Notifications.openSettings() }
          ]
        );
        return false;
      }

      return true;
    } catch (error) {
      console.error('Failed to initialize notification service:', error);
      return false;
    }
  }

  /**
   * Setup notification handlers
   */
  private setupNotificationHandlers(): void {
    // Configure notification behavior
    Notifications.setNotificationHandler({
      handleNotification: async () => ({
        shouldShowAlert: true,
        shouldPlaySound: true,
        shouldSetBadge: true,
      }),
    });

    // Handle notifications received while app is in foreground
    Notifications.addNotificationReceivedListener((notification) => {
      console.log('Notification received:', notification);
      this.notifyNotificationCallbacks(notification);
    });

    // Handle notification responses (when user taps notification)
    Notifications.addNotificationResponseReceivedListener((response) => {
      console.log('Notification response:', response);
      this.notifyResponseCallbacks(response);
    });
  }

  /**
   * Request notification permissions
   */
  public async requestPermissions(): Promise<NotificationPermissionStatus> {
    try {
      const { status: existingStatus } = await Notifications.getPermissionsAsync();
      let finalStatus = existingStatus;

      if (existingStatus !== 'granted') {
        const { status } = await Notifications.requestPermissionsAsync();
        finalStatus = status;
      }

      if (finalStatus !== 'granted') {
        return {
          granted: false,
          canAskAgain: finalStatus === 'undetermined',
          status: finalStatus,
        };
      }

      // Request additional permissions for Android
      if (Platform.OS === 'android') {
        await Notifications.setNotificationChannelAsync('default', {
          name: 'default',
          importance: Notifications.AndroidImportance.MAX,
          vibrationPattern: [0, 250, 250, 250],
          lightColor: '#FF231F7C',
        });
      }

      return {
        granted: true,
        canAskAgain: false,
        status: finalStatus,
      };
    } catch (error) {
      console.error('Error requesting notification permissions:', error);
      return {
        granted: false,
        canAskAgain: false,
        status: 'denied',
      };
    }
  }

  /**
   * Send local notification
   */
  public async sendLocalNotification(notificationData: NotificationData): Promise<string | null> {
    try {
      const notificationId = await Notifications.scheduleNotificationAsync({
        content: {
          title: notificationData.title,
          body: notificationData.body,
          data: notificationData.data || {},
          sound: notificationData.sound !== false,
          badge: notificationData.badge,
        },
        trigger: null, // Send immediately
      });

      console.log('Local notification sent:', notificationId);
      return notificationId;
    } catch (error) {
      console.error('Error sending local notification:', error);
      return null;
    }
  }

  /**
   * Send scheduled notification
   */
  public async sendScheduledNotification(
    notificationData: NotificationData,
    trigger: Notifications.NotificationTriggerInput
  ): Promise<string | null> {
    try {
      const notificationId = await Notifications.scheduleNotificationAsync({
        content: {
          title: notificationData.title,
          body: notificationData.body,
          data: notificationData.data || {},
          sound: notificationData.sound !== false,
          badge: notificationData.badge,
        },
        trigger,
      });

      console.log('Scheduled notification sent:', notificationId);
      return notificationId;
    } catch (error) {
      console.error('Error sending scheduled notification:', error);
      return null;
    }
  }

  /**
   * Cancel notification
   */
  public async cancelNotification(notificationId: string): Promise<void> {
    try {
      await Notifications.cancelScheduledNotificationAsync(notificationId);
      console.log('Notification cancelled:', notificationId);
    } catch (error) {
      console.error('Error cancelling notification:', error);
    }
  }

  /**
   * Cancel all notifications
   */
  public async cancelAllNotifications(): Promise<void> {
    try {
      await Notifications.cancelAllScheduledNotificationsAsync();
      console.log('All notifications cancelled');
    } catch (error) {
      console.error('Error cancelling all notifications:', error);
    }
  }

  /**
   * Get notification permission status
   */
  public async getPermissionStatus(): Promise<NotificationPermissionStatus> {
    try {
      const { status, canAskAgain } = await Notifications.getPermissionsAsync();
      return {
        granted: status === 'granted',
        canAskAgain,
        status,
      };
    } catch (error) {
      console.error('Error getting notification permission status:', error);
      return {
        granted: false,
        canAskAgain: false,
        status: 'denied',
      };
    }
  }

  /**
   * Add notification received callback
   */
  public addNotificationCallback(callback: (notification: Notifications.Notification) => void): void {
    this.notificationCallbacks.push(callback);
  }

  /**
   * Remove notification received callback
   */
  public removeNotificationCallback(callback: (notification: Notifications.Notification) => void): void {
    const index = this.notificationCallbacks.indexOf(callback);
    if (index > -1) {
      this.notificationCallbacks.splice(index, 1);
    }
  }

  /**
   * Add notification response callback
   */
  public addResponseCallback(callback: (response: Notifications.NotificationResponse) => void): void {
    this.responseCallbacks.push(callback);
  }

  /**
   * Remove notification response callback
   */
  public removeResponseCallback(callback: (response: Notifications.NotificationResponse) => void): void {
    const index = this.responseCallbacks.indexOf(callback);
    if (index > -1) {
      this.responseCallbacks.splice(index, 1);
    }
  }

  /**
   * Notify all notification callbacks
   */
  private notifyNotificationCallbacks(notification: Notifications.Notification): void {
    this.notificationCallbacks.forEach(callback => {
      try {
        callback(notification);
      } catch (error) {
        console.error('Error in notification callback:', error);
      }
    });
  }

  /**
   * Notify all response callbacks
   */
  private notifyResponseCallbacks(response: Notifications.NotificationResponse): void {
    this.responseCallbacks.forEach(callback => {
      try {
        callback(response);
      } catch (error) {
        console.error('Error in response callback:', error);
      }
    });
  }

  /**
   * Send emergency notification
   */
  public async sendEmergencyNotification(
    title: string,
    body: string,
    data?: any
  ): Promise<string | null> {
    return this.sendLocalNotification({
      title,
      body,
      data,
      sound: true,
      priority: 'max',
      badge: 1,
    });
  }

  /**
   * Send maintenance reminder
   */
  public async sendMaintenanceReminder(
    vehicleId: string,
    vehicleName: string,
    maintenanceType: string
  ): Promise<string | null> {
    return this.sendLocalNotification({
      title: 'Maintenance Required',
      body: `${vehicleName} requires ${maintenanceType}`,
      data: {
        type: 'maintenance',
        vehicleId,
        vehicleName,
        maintenanceType,
      },
      sound: true,
      priority: 'high',
    });
  }

  /**
   * Send trip update
   */
  public async sendTripUpdate(
    tripId: string,
    status: string,
    message: string
  ): Promise<string | null> {
    return this.sendLocalNotification({
      title: 'Trip Update',
      body: message,
      data: {
        type: 'trip',
        tripId,
        status,
      },
      sound: false,
      priority: 'default',
    });
  }

  /**
   * Send safety alert
   */
  public async sendSafetyAlert(
    vehicleId: string,
    alertType: string,
    message: string
  ): Promise<string | null> {
    return this.sendLocalNotification({
      title: 'Safety Alert',
      body: message,
      data: {
        type: 'safety',
        vehicleId,
        alertType,
      },
      sound: true,
      priority: 'max',
      badge: 1,
    });
  }

  /**
   * Get notification history
   */
  public async getNotificationHistory(): Promise<Notifications.Notification[]> {
    try {
      const notifications = await Notifications.getPresentedNotificationsAsync();
      return notifications;
    } catch (error) {
      console.error('Error getting notification history:', error);
      return [];
    }
  }

  /**
   * Clear notification badge
   */
  public async clearBadge(): Promise<void> {
    try {
      await Notifications.setBadgeCountAsync(0);
    } catch (error) {
      console.error('Error clearing badge:', error);
    }
  }

  /**
   * Set notification badge count
   */
  public async setBadgeCount(count: number): Promise<void> {
    try {
      await Notifications.setBadgeCountAsync(count);
    } catch (error) {
      console.error('Error setting badge count:', error);
    }
  }
}

export default NotificationService;
