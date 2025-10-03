import React, { useEffect, useState } from 'react';
import { StatusBar } from 'expo-status-bar';
import { NavigationContainer } from '@react-navigation/native';
import { createBottomTabNavigator } from '@react-navigation/bottom-tabs';
import { createStackNavigator } from '@react-navigation/stack';
import { Provider as PaperProvider } from 'react-native-paper';
import { Ionicons } from '@expo/vector-icons';
import * as Location from 'expo-location';
import * as Notifications from 'expo-notifications';
import AsyncStorage from '@react-native-async-storage/async-storage';

// Screens
import LoginScreen from './src/screens/LoginScreen';
import DashboardScreen from './src/screens/DashboardScreen';
import VehicleListScreen from './src/screens/VehicleListScreen';
import VehicleDetailScreen from './src/screens/VehicleDetailScreen';
import MaintenanceScreen from './src/screens/MaintenanceScreen';
import EmergencyScreen from './src/screens/EmergencyScreen';
import DriverScreen from './src/screens/DriverScreen';
import TechnicianScreen from './src/screens/TechnicianScreen';
import SettingsScreen from './src/screens/SettingsScreen';

// Services
import { AuthService } from './src/services/AuthService';
import { LocationService } from './src/services/LocationService';
import { NotificationService } from './src/services/NotificationService';

// Types
import { User, UserRole } from './src/types/User';

// Theme
import { theme } from './src/theme/theme';

const Tab = createBottomTabNavigator();
const Stack = createStackNavigator();

// Configure notifications
Notifications.setNotificationHandler({
  handleNotification: async () => ({
    shouldShowAlert: true,
    shouldPlaySound: true,
    shouldSetBadge: false,
  }),
});

// Main Tab Navigator
function MainTabs({ user }: { user: User }) {
  const getTabScreens = () => {
    const commonScreens = [
      {
        name: 'Dashboard',
        component: DashboardScreen,
        icon: 'speedometer-outline',
        label: 'Dashboard'
      },
      {
        name: 'Vehicles',
        component: VehicleListScreen,
        icon: 'car-outline',
        label: 'Vehicles'
      },
      {
        name: 'Emergency',
        component: EmergencyScreen,
        icon: 'warning-outline',
        label: 'Emergency'
      },
      {
        name: 'Settings',
        component: SettingsScreen,
        icon: 'settings-outline',
        label: 'Settings'
      }
    ];

    // Role-specific screens
    if (user.role === UserRole.DRIVER) {
      return [
        {
          name: 'Driver',
          component: DriverScreen,
          icon: 'person-outline',
          label: 'Driver'
        },
        ...commonScreens
      ];
    }

    if (user.role === UserRole.TECHNICIAN) {
      return [
        {
          name: 'Technician',
          component: TechnicianScreen,
          icon: 'construct-outline',
          label: 'Technician'
        },
        {
          name: 'Maintenance',
          component: MaintenanceScreen,
          icon: 'build-outline',
          label: 'Maintenance'
        },
        ...commonScreens
      ];
    }

    // Default screens for other roles
    return commonScreens;
  };

  const screens = getTabScreens();

  return (
    <Tab.Navigator
      screenOptions={({ route }) => ({
        tabBarIcon: ({ focused, color, size }) => {
          const screen = screens.find(s => s.name === route.name);
          const iconName = screen?.icon || 'help-outline';
          
          return <Ionicons name={iconName as any} size={size} color={color} />;
        },
        tabBarActiveTintColor: theme.colors.primary,
        tabBarInactiveTintColor: 'gray',
        tabBarStyle: {
          paddingBottom: 5,
          paddingTop: 5,
          height: 60,
        },
        headerStyle: {
          backgroundColor: theme.colors.primary,
        },
        headerTintColor: '#fff',
        headerTitleStyle: {
          fontWeight: 'bold',
        },
      })}
    >
      {screens.map((screen) => (
        <Tab.Screen
          key={screen.name}
          name={screen.name}
          component={screen.component}
          options={{
            tabBarLabel: screen.label,
            title: screen.label,
          }}
        />
      ))}
    </Tab.Navigator>
  );
}

// Main Stack Navigator
function MainStack() {
  return (
    <Stack.Navigator
      screenOptions={{
        headerStyle: {
          backgroundColor: theme.colors.primary,
        },
        headerTintColor: '#fff',
        headerTitleStyle: {
          fontWeight: 'bold',
        },
      }}
    >
      <Stack.Screen
        name="MainTabs"
        component={MainTabs}
        options={{ headerShown: false }}
      />
      <Stack.Screen
        name="VehicleDetail"
        component={VehicleDetailScreen}
        options={{ title: 'Vehicle Details' }}
      />
    </Stack.Navigator>
  );
}

// Auth Stack Navigator
function AuthStack() {
  return (
    <Stack.Navigator screenOptions={{ headerShown: false }}>
      <Stack.Screen name="Login" component={LoginScreen} />
    </Stack.Navigator>
  );
}

export default function App() {
  const [isLoading, setIsLoading] = useState(true);
  const [user, setUser] = useState<User | null>(null);

  useEffect(() => {
    initializeApp();
  }, []);

  const initializeApp = async () => {
    try {
      // Request permissions
      await requestPermissions();
      
      // Initialize services
      await LocationService.initialize();
      await NotificationService.initialize();
      
      // Check for existing authentication
      const savedUser = await AuthService.getCurrentUser();
      if (savedUser) {
        setUser(savedUser);
      }
      
      // Start location tracking if user is authenticated
      if (savedUser) {
        LocationService.startTracking();
      }
      
    } catch (error) {
      console.error('Failed to initialize app:', error);
    } finally {
      setIsLoading(false);
    }
  };

  const requestPermissions = async () => {
    // Location permission
    const { status: locationStatus } = await Location.requestForegroundPermissionsAsync();
    if (locationStatus !== 'granted') {
      console.warn('Location permission not granted');
    }

    // Background location permission (for drivers)
    const { status: backgroundLocationStatus } = await Location.requestBackgroundPermissionsAsync();
    if (backgroundLocationStatus !== 'granted') {
      console.warn('Background location permission not granted');
    }

    // Notification permission
    const { status: notificationStatus } = await Notifications.requestPermissionsAsync();
    if (notificationStatus !== 'granted') {
      console.warn('Notification permission not granted');
    }
  };

  const handleLogin = (authenticatedUser: User) => {
    setUser(authenticatedUser);
    LocationService.startTracking();
  };

  const handleLogout = async () => {
    await AuthService.logout();
    LocationService.stopTracking();
    setUser(null);
  };

  if (isLoading) {
    // You can replace this with a proper loading screen
    return null;
  }

  return (
    <PaperProvider theme={theme}>
      <NavigationContainer>
        <StatusBar style="light" backgroundColor={theme.colors.primary} />
        {user ? (
          <MainStack />
        ) : (
          <AuthStack />
        )}
      </NavigationContainer>
    </PaperProvider>
  );
}

// Export for use in other components
export { handleLogout };
