// AtlasMesh Fleet OS - Environment Configuration
// Centralized configuration for map providers, API endpoints, and feature flags

export interface EnvironmentConfig {
  // API Configuration
  apiBaseUrl: string
  wsBaseUrl: string
  
  // Map Provider Configuration
  mapProviders: {
    google: {
      apiKey: string
      enabled: boolean
    }
    mapbox: {
      apiKey: string
      enabled: boolean
    }
    openstreetmap: {
      enabled: boolean
      tileServer: string
    }
  }
  
  // Default Map Settings (Abu Dhabi)
  defaultMap: {
    provider: 'openstreetmap' | 'google' | 'satellite'
    center: {
      lat: number
      lng: number
    }
    zoom: number
  }
  
  // Feature Flags
  features: {
    enableClustering: boolean
    enableRealTimeUpdates: boolean
    enableVehicleTrails: boolean
    enableGeofenceMonitoring: boolean
    enableHeatmaps: boolean
    enableOfflineMode: boolean
  }
  
  // Performance Settings
  performance: {
    mapUpdateInterval: number // ms
    telemetryBatchSize: number
    maxVehiclesOnMap: number
    maxRoutePoints: number
    clusterRadius: number
    maxClusterZoom: number
  }
  
  // Development Settings
  development: {
    debugMode: boolean
    mockData: boolean
    logLevel: 'debug' | 'info' | 'warn' | 'error'
    enablePerformanceMonitoring: boolean
  }
  
  // Security Settings
  security: {
    enableHttps: boolean
    csrfTokenHeader: string
    apiTimeout: number // ms
  }
}

// Default configuration for Abu Dhabi deployment
export const defaultConfig: EnvironmentConfig = {
  apiBaseUrl: process.env.REACT_APP_API_BASE_URL || 'http://localhost:8080',
  wsBaseUrl: process.env.REACT_APP_WS_BASE_URL || 'ws://localhost:8080',
  
  mapProviders: {
    google: {
      apiKey: process.env.REACT_APP_GOOGLE_MAPS_API_KEY || '',
      enabled: true
    },
    mapbox: {
      apiKey: process.env.REACT_APP_MAPBOX_API_KEY || '',
      enabled: false // Disabled in favor of OSM + Google
    },
    openstreetmap: {
      enabled: true,
      tileServer: 'https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png'
    }
  },
  
  defaultMap: {
    provider: (process.env.REACT_APP_DEFAULT_MAP_PROVIDER as any) || 'openstreetmap',
    center: {
      lat: parseFloat(process.env.REACT_APP_DEFAULT_MAP_CENTER_LAT || '24.4539'), // Abu Dhabi
      lng: parseFloat(process.env.REACT_APP_DEFAULT_MAP_CENTER_LNG || '54.3773')  // Abu Dhabi
    },
    zoom: parseInt(process.env.REACT_APP_DEFAULT_MAP_ZOOM || '12')
  },
  
  features: {
    enableClustering: process.env.REACT_APP_ENABLE_CLUSTERING !== 'false',
    enableRealTimeUpdates: process.env.REACT_APP_ENABLE_REAL_TIME_UPDATES !== 'false',
    enableVehicleTrails: process.env.REACT_APP_ENABLE_VEHICLE_TRAILS !== 'false',
    enableGeofenceMonitoring: process.env.REACT_APP_ENABLE_GEOFENCE_MONITORING !== 'false',
    enableHeatmaps: process.env.REACT_APP_ENABLE_HEATMAPS === 'true',
    enableOfflineMode: process.env.REACT_APP_ENABLE_OFFLINE_MODE === 'true'
  },
  
  performance: {
    mapUpdateInterval: parseInt(process.env.REACT_APP_MAP_UPDATE_INTERVAL || '5000'),
    telemetryBatchSize: parseInt(process.env.REACT_APP_TELEMETRY_BATCH_SIZE || '100'),
    maxVehiclesOnMap: parseInt(process.env.REACT_APP_MAX_VEHICLES_ON_MAP || '200'),
    maxRoutePoints: parseInt(process.env.REACT_APP_MAX_ROUTE_POINTS || '1000'),
    clusterRadius: parseInt(process.env.REACT_APP_CLUSTER_RADIUS || '50'),
    maxClusterZoom: parseInt(process.env.REACT_APP_MAX_CLUSTER_ZOOM || '15')
  },
  
  development: {
    debugMode: process.env.REACT_APP_DEBUG_MODE === 'true',
    mockData: process.env.REACT_APP_MOCK_DATA !== 'false',
    logLevel: (process.env.REACT_APP_LOG_LEVEL as any) || 'info',
    enablePerformanceMonitoring: process.env.REACT_APP_ENABLE_PERFORMANCE_MONITORING === 'true'
  },
  
  security: {
    enableHttps: process.env.REACT_APP_ENABLE_HTTPS === 'true',
    csrfTokenHeader: process.env.REACT_APP_CSRF_TOKEN_HEADER || 'X-CSRF-Token',
    apiTimeout: parseInt(process.env.REACT_APP_API_TIMEOUT || '30000')
  }
}

// Environment-specific configurations
export const environments = {
  development: {
    ...defaultConfig,
    apiBaseUrl: 'http://localhost:8080',
    wsBaseUrl: 'ws://localhost:8080',
    development: {
      ...defaultConfig.development,
      debugMode: true,
      mockData: true,
      logLevel: 'debug' as const
    }
  },
  
  staging: {
    ...defaultConfig,
    apiBaseUrl: 'https://api-staging.atlasmesh.com',
    wsBaseUrl: 'wss://api-staging.atlasmesh.com',
    development: {
      ...defaultConfig.development,
      debugMode: false,
      mockData: false,
      logLevel: 'info' as const
    }
  },
  
  production: {
    ...defaultConfig,
    apiBaseUrl: 'https://api.atlasmesh.com',
    wsBaseUrl: 'wss://api.atlasmesh.com',
    security: {
      ...defaultConfig.security,
      enableHttps: true
    },
    development: {
      ...defaultConfig.development,
      debugMode: false,
      mockData: false,
      logLevel: 'warn' as const,
      enablePerformanceMonitoring: true
    }
  }
}

// Get current environment configuration
export const getEnvironmentConfig = (): EnvironmentConfig => {
  const env = process.env.NODE_ENV || 'development'
  
  switch (env) {
    case 'production':
      return environments.production
    case 'staging':
      return environments.staging
    case 'development':
    default:
      return environments.development
  }
}

// Export current config
export const config = getEnvironmentConfig()

// Utility functions
export const isProduction = () => process.env.NODE_ENV === 'production'
export const isStaging = () => process.env.NODE_ENV === 'staging'
export const isDevelopment = () => process.env.NODE_ENV === 'development'

// Map provider availability checks
export const getAvailableMapProviders = (): string[] => {
  const providers: string[] = []
  
  if (config.mapProviders.openstreetmap.enabled) {
    providers.push('openstreetmap')
  }
  
  if (config.mapProviders.google.enabled && config.mapProviders.google.apiKey) {
    providers.push('google', 'satellite')
  }
  
  if (config.mapProviders.mapbox.enabled && config.mapProviders.mapbox.apiKey) {
    providers.push('mapbox')
  }
  
  return providers
}

// Validation functions
export const validateConfig = (): { valid: boolean; errors: string[] } => {
  const errors: string[] = []
  
  // Check required API URLs
  if (!config.apiBaseUrl) {
    errors.push('API base URL is required')
  }
  
  // Check map provider configuration
  const availableProviders = getAvailableMapProviders()
  if (availableProviders.length === 0) {
    errors.push('At least one map provider must be enabled and configured')
  }
  
  if (!availableProviders.includes(config.defaultMap.provider)) {
    errors.push(`Default map provider '${config.defaultMap.provider}' is not available`)
  }
  
  // Check Abu Dhabi coordinates
  if (config.defaultMap.center.lat < 24.0 || config.defaultMap.center.lat > 25.0) {
    errors.push('Default latitude should be within Abu Dhabi region (24.0-25.0)')
  }
  
  if (config.defaultMap.center.lng < 54.0 || config.defaultMap.center.lng > 55.0) {
    errors.push('Default longitude should be within Abu Dhabi region (54.0-55.0)')
  }
  
  return {
    valid: errors.length === 0,
    errors
  }
}

// Log configuration on startup
if (config.development.debugMode) {
  console.info('🗺️ AtlasMesh Fleet OS Map Configuration:', {
    environment: process.env.NODE_ENV,
    defaultProvider: config.defaultMap.provider,
    availableProviders: getAvailableMapProviders(),
    center: config.defaultMap.center,
    features: config.features,
    validation: validateConfig()
  })
}
