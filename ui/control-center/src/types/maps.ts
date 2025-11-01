// Map provider types for dual OpenStreetMap + Google Maps support
// Abu Dhabi centered mapping solution for AtlasMesh Fleet OS

export type MapProvider = 'openstreetmap' | 'google' | 'satellite'

export interface MapCoordinate {
  lat: number
  lng: number
  altitude?: number
}

export interface MapBounds {
  north: number
  south: number
  east: number
  west: number
}

export interface MapViewport {
  center: MapCoordinate
  zoom: number
  bounds?: MapBounds
}

export interface MapMarker {
  id: string
  position: MapCoordinate
  title?: string
  description?: string
  icon?: string
  iconUrl?: string
  iconSize?: [number, number]
  popup?: string
  clickable?: boolean
  draggable?: boolean
  rotation?: number
  zIndex?: number
}

export interface MapRoute {
  id: string
  name: string
  coordinates: MapCoordinate[]
  color?: string
  weight?: number
  opacity?: number
  dashArray?: number[]
  interactive?: boolean
}

export interface MapGeofence {
  id: string
  name: string
  type: 'circle' | 'polygon' | 'rectangle'
  geometry: {
    type: 'circle' | 'polygon' | 'rectangle'
    center?: MapCoordinate
    radius?: number // meters
    coordinates?: MapCoordinate[]
    bounds?: MapBounds
  }
  strokeColor?: string
  strokeWeight?: number
  strokeOpacity?: number
  fillColor?: string
  fillOpacity?: number
  interactive?: boolean
}

export interface MapLayer {
  id: string
  name: string
  type: 'markers' | 'routes' | 'geofences' | 'heatmap' | 'tiles'
  visible: boolean
  opacity: number
  zIndex?: number
  data?: any[]
}

export interface MapStyle {
  id: string
  name: string
  provider: MapProvider
  styleUrl?: string
  config?: Record<string, any>
}

export interface MapEvent {
  type: 'click' | 'dblclick' | 'contextmenu' | 'zoom' | 'move' | 'load'
  position?: MapCoordinate
  target?: any
  originalEvent?: Event
}

// Vehicle-specific map types
export interface VehicleMapMarker extends MapMarker {
  vehicleId: string
  status: 'operational' | 'warning' | 'critical' | 'offline' | 'charging'
  autonomyLevel: 'L0' | 'L1' | 'L2' | 'L3' | 'L4' | 'L5'
  heading: number
  speed: number
  batteryLevel?: number
  lastUpdate: Date
  trail?: MapCoordinate[]
}

export interface RouteMapOverlay extends MapRoute {
  routeId: string
  vehicleId?: string
  status: 'active' | 'planned' | 'completed' | 'emergency'
  waypoints?: Array<{
    id: string
    position: MapCoordinate
    type: 'start' | 'end' | 'checkpoint' | 'pickup' | 'dropoff'
    dwellTime?: number
  }>
}

export interface GeofenceMapOverlay extends MapGeofence {
  geofenceId: string
  status: 'active' | 'inactive' | 'violated'
  rules: Array<{
    condition: 'entry' | 'exit' | 'dwell' | 'speed'
    action: 'alert' | 'stop' | 'reroute' | 'log'
    severity: 'info' | 'warning' | 'critical'
  }>
  violationCount?: number
  lastViolation?: Date
}

// Map configuration
export interface MapConfig {
  defaultProvider: MapProvider
  defaultCenter: MapCoordinate
  defaultZoom: number
  minZoom: number
  maxZoom: number
  enableRotation: boolean
  enableTilt: boolean
  enableClustering: boolean
  clusterRadius: number
  maxClusterZoom: number
  animationDuration: number
  apiKeys: {
    google?: string
    mapbox?: string
  }
  styles: MapStyle[]
  layers: MapLayer[]
}

// Abu Dhabi specific constants
export const ABU_DHABI_CONFIG: MapConfig = {
  defaultProvider: 'openstreetmap',
  defaultCenter: {
    lat: 24.4539,
    lng: 54.3773
  },
  defaultZoom: 12,
  minZoom: 8,
  maxZoom: 20,
  enableRotation: true,
  enableTilt: false,
  enableClustering: true,
  clusterRadius: 50,
  maxClusterZoom: 15,
  animationDuration: 300,
  apiKeys: {
    google: process.env.REACT_APP_GOOGLE_MAPS_API_KEY
  },
  styles: [
    {
      id: 'osm-standard',
      name: 'OpenStreetMap Standard',
      provider: 'openstreetmap',
      styleUrl: 'https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png'
    },
    {
      id: 'google-roadmap',
      name: 'Google Roadmap',
      provider: 'google',
      config: { mapTypeId: 'roadmap' }
    },
    {
      id: 'google-satellite',
      name: 'Google Satellite',
      provider: 'satellite',
      config: { mapTypeId: 'hybrid' }
    }
  ],
  layers: [
    {
      id: 'vehicles',
      name: 'Vehicles',
      type: 'markers',
      visible: true,
      opacity: 1.0,
      zIndex: 100
    },
    {
      id: 'routes',
      name: 'Routes',
      type: 'routes',
      visible: true,
      opacity: 0.8,
      zIndex: 50
    },
    {
      id: 'geofences',
      name: 'Geofences',
      type: 'geofences',
      visible: true,
      opacity: 0.6,
      zIndex: 25
    }
  ]
}

// Map utility types
export interface MapUtils {
  calculateDistance: (from: MapCoordinate, to: MapCoordinate) => number
  calculateBounds: (coordinates: MapCoordinate[]) => MapBounds
  isPointInPolygon: (point: MapCoordinate, polygon: MapCoordinate[]) => boolean
  isPointInCircle: (point: MapCoordinate, center: MapCoordinate, radius: number) => boolean
  formatCoordinate: (coord: number, type: 'lat' | 'lng') => string
  convertToMapProvider: (coordinates: MapCoordinate[], provider: MapProvider) => any
}
