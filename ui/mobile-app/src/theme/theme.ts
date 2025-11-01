import { DefaultTheme } from 'react-native-paper';

export const theme = {
  ...DefaultTheme,
  colors: {
    ...DefaultTheme.colors,
    primary: '#2196F3',
    accent: '#FF9800',
    background: '#F5F5F5',
    surface: '#FFFFFF',
    text: '#212121',
    disabled: '#BDBDBD',
    placeholder: '#757575',
    backdrop: 'rgba(0, 0, 0, 0.5)',
    error: '#F44336',
    success: '#4CAF50',
    warning: '#FF9800',
    info: '#2196F3',
    
    // Custom colors for AtlasMesh
    primaryDark: '#1976D2',
    primaryLight: '#BBDEFB',
    secondary: '#4CAF50',
    secondaryDark: '#388E3C',
    secondaryLight: '#C8E6C9',
    
    // Status colors
    online: '#4CAF50',
    offline: '#F44336',
    idle: '#FF9800',
    maintenance: '#9C27B0',
    emergency: '#F44336',
    
    // Vehicle status colors
    available: '#4CAF50',
    busy: '#FF9800',
    outOfService: '#F44336',
    charging: '#2196F3',
    
    // Map colors
    route: '#2196F3',
    geofence: '#9C27B0',
    poi: '#FF5722',
    
    // Chart colors
    chart1: '#2196F3',
    chart2: '#4CAF50',
    chart3: '#FF9800',
    chart4: '#9C27B0',
    chart5: '#F44336',
  },
  
  // Typography
  fonts: {
    ...DefaultTheme.fonts,
    regular: {
      fontFamily: 'System',
      fontWeight: '400' as const,
    },
    medium: {
      fontFamily: 'System',
      fontWeight: '500' as const,
    },
    light: {
      fontFamily: 'System',
      fontWeight: '300' as const,
    },
    thin: {
      fontFamily: 'System',
      fontWeight: '100' as const,
    },
  },
  
  // Spacing
  spacing: {
    xs: 4,
    sm: 8,
    md: 16,
    lg: 24,
    xl: 32,
    xxl: 48,
  },
  
  // Border radius
  roundness: 8,
  
  // Shadows
  shadows: {
    small: {
      shadowColor: '#000',
      shadowOffset: {
        width: 0,
        height: 1,
      },
      shadowOpacity: 0.22,
      shadowRadius: 2.22,
      elevation: 3,
    },
    medium: {
      shadowColor: '#000',
      shadowOffset: {
        width: 0,
        height: 2,
      },
      shadowOpacity: 0.25,
      shadowRadius: 3.84,
      elevation: 5,
    },
    large: {
      shadowColor: '#000',
      shadowOffset: {
        width: 0,
        height: 4,
      },
      shadowOpacity: 0.30,
      shadowRadius: 4.65,
      elevation: 8,
    },
  },
  
  // Animation
  animation: {
    scale: 1.0,
  },
};

export type Theme = typeof theme;
