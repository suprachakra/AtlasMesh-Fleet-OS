// AtlasMesh Fleet OS - Map Service
// Centralized service for map operations, vehicle tracking, and real-time updates
// Supports OpenStreetMap and Google Maps with Abu Dhabi focus

import { config } from '../config/environment'
import type { 
  MapProvider, 
  MapCoordinate, 
  VehicleMapMarker, 
  RouteMapOverlay, 
  GeofenceMapOverlay 
} from '../types/maps'

export interface VehiclePosition {
  vehicleId: string
  position: MapCoordinate
  heading: number
  speed: number
  timestamp: Date
  accuracy: number
  status: 'operational' | 'warning' | 'critical' | 'offline' | 'charging'
  autonomyLevel: 'L0' | 'L1' | 'L2' | 'L3' | 'L4' | 'L5'
  batteryLevel?: number
  currentTripId?: string
}

export interface RouteData {
  routeId: string
  name: string
  coordinates: MapCoordinate[]
  status: 'active' | 'planned' | 'completed' | 'emergency'
  vehicleId?: string
  estimatedDuration?: number
  distance?: number
  waypoints?: Array<{
    id: string
    position: MapCoordinate
    type: 'start' | 'end' | 'checkpoint' | 'pickup' | 'dropoff'
    dwellTime?: number
    completed?: boolean
  }>
}

export interface GeofenceData {
  geofenceId: string
  name: string
  type: 'circle' | 'polygon'
  geometry: {
    type: 'circle' | 'polygon'
    center?: MapCoordinate
    radius?: number
    coordinates?: MapCoordinate[]
  }
  rules: Array<{
    condition: 'entry' | 'exit' | 'dwell' | 'speed'
    action: 'alert' | 'stop' | 'reroute' | 'log'
    severity: 'info' | 'warning' | 'critical'
    threshold?: number
  }>
  status: 'active' | 'inactive'
  violationCount: number
  lastViolation?: Date
}

export interface MapServiceConfig {
  updateInterval: number
  maxVehicles: number
  enableClustering: boolean
  enableRealTimeUpdates: boolean
  enableVehicleTrails: boolean
  trailLength: number
  defaultProvider: MapProvider
  abuDhabiCenter: MapCoordinate
}

class MapService {
  private config: MapServiceConfig
  private vehiclePositions: Map<string, VehiclePosition> = new Map()
  private vehicleTrails: Map<string, MapCoordinate[]> = new Map()
  private routes: Map<string, RouteData> = new Map()
  private geofences: Map<string, GeofenceData> = new Map()
  private updateInterval?: NodeJS.Timeout
  private subscribers: Map<string, (data: any) => void> = new Map()
  private websocket?: WebSocket

  constructor(config?: Partial<MapServiceConfig>) {
    this.config = {
      updateInterval: config?.updateInterval || 5000,
      maxVehicles: config?.maxVehicles || 200,
      enableClustering: config?.enableClustering ?? true,
      enableRealTimeUpdates: config?.enableRealTimeUpdates ?? true,
      enableVehicleTrails: config?.enableVehicleTrails ?? true,
      trailLength: config?.trailLength || 50,
      defaultProvider: config?.defaultProvider || 'openstreetmap',
      abuDhabiCenter: config?.abuDhabiCenter || { lat: 24.4539, lng: 54.3773 }
    }
  }

  // Initialize the map service
  async initialize(): Promise<void> {
    console.info('🗺️ Initializing AtlasMesh Map Service for Abu Dhabi', {
      config: this.config,
      timestamp: new Date().toISOString()
    })

    if (this.config.enableRealTimeUpdates) {
      await this.connectWebSocket()
      this.startPeriodicUpdates()
    }

    // Load initial data
    await this.loadInitialData()
  }

  // Connect to WebSocket for real-time updates
  private async connectWebSocket(): Promise<void> {
    const wsUrl = config.wsBaseUrl + '/ws/fleet-updates'
    
    try {
      this.websocket = new WebSocket(wsUrl)
      
      this.websocket.onopen = () => {
        console.info('🔗 WebSocket connected for real-time fleet updates')
      }
      
      this.websocket.onmessage = (event) => {
        try {
          const data = JSON.parse(event.data)
          this.handleRealtimeUpdate(data)
        } catch (error) {
          console.error('❌ Failed to parse WebSocket message:', error)
        }
      }
      
      this.websocket.onclose = () => {
        console.warn('🔌 WebSocket disconnected, attempting to reconnect...')
        setTimeout(() => this.connectWebSocket(), 5000)
      }
      
      this.websocket.onerror = (error) => {
        console.error('🚫 WebSocket error:', error)
      }
    } catch (error) {
      console.error('❌ Failed to connect WebSocket:', error)
    }
  }

  // Handle real-time updates from WebSocket
  private handleRealtimeUpdate(data: any): void {
    switch (data.type) {
      case 'vehicle_position':
        this.updateVehiclePosition(data.payload)
        break
      case 'route_update':
        this.updateRoute(data.payload)
        break
      case 'geofence_violation':
        this.handleGeofenceViolation(data.payload)
        break
      case 'emergency_alert':
        this.handleEmergencyAlert(data.payload)
        break
      default:
        console.warn('🤷 Unknown real-time update type:', data.type)
    }
  }

  // Start periodic updates for non-WebSocket data
  private startPeriodicUpdates(): void {
    this.updateInterval = setInterval(async () => {
      try {
        await this.fetchVehiclePositions()
        await this.fetchRouteUpdates()
        await this.checkGeofenceViolations()
      } catch (error) {
        console.error('❌ Periodic update failed:', error)
      }
    }, this.config.updateInterval)
  }

  // Load initial data
  private async loadInitialData(): Promise<void> {
    try {
      const [vehicles, routes, geofences] = await Promise.all([
        this.fetchVehiclePositions(),
        this.fetchRoutes(),
        this.fetchGeofences()
      ])

      console.info('📊 Initial data loaded:', {
        vehicles: vehicles.length,
        routes: routes.length,
        geofences: geofences.length
      })
    } catch (error) {
      console.error('❌ Failed to load initial data:', error)
    }
  }

  // Fetch vehicle positions from API
  private async fetchVehiclePositions(): Promise<VehiclePosition[]> {
    try {
      const response = await fetch(`${config.apiBaseUrl}/api/v1/vehicles/positions`, {
        headers: {
          'Content-Type': 'application/json',
          'Accept': 'application/json'
        }
      })

      if (!response.ok) {
        throw new Error(`HTTP ${response.status}: ${response.statusText}`)
      }

      const data = await response.json()
      const positions: VehiclePosition[] = data.vehicles || []

      // Update internal state
      positions.forEach(position => {
        this.updateVehiclePosition(position)
      })

      // Notify subscribers
      this.notifySubscribers('vehicle_positions', positions)

      return positions
    } catch (error) {
      console.error('❌ Failed to fetch vehicle positions:', error)
      
      // Return mock data in development
      if (config.development.mockData) {
        return this.generateMockVehiclePositions()
      }
      
      return []
    }
  }

  // Update single vehicle position
  private updateVehiclePosition(position: VehiclePosition): void {
    const previousPosition = this.vehiclePositions.get(position.vehicleId)
    this.vehiclePositions.set(position.vehicleId, position)

    // Update vehicle trail
    if (this.config.enableVehicleTrails && previousPosition) {
      let trail = this.vehicleTrails.get(position.vehicleId) || []
      trail.push(position.position)
      
      // Limit trail length
      if (trail.length > this.config.trailLength) {
        trail = trail.slice(-this.config.trailLength)
      }
      
      this.vehicleTrails.set(position.vehicleId, trail)
    }

    // Check for geofence violations
    this.checkVehicleGeofences(position)

    // Notify subscribers
    this.notifySubscribers('vehicle_position_updated', position)
  }

  // Fetch routes from API
  private async fetchRoutes(): Promise<RouteData[]> {
    try {
      const response = await fetch(`${config.apiBaseUrl}/api/v1/routes/active`, {
        headers: {
          'Content-Type': 'application/json',
          'Accept': 'application/json'
        }
      })

      if (!response.ok) {
        throw new Error(`HTTP ${response.status}: ${response.statusText}`)
      }

      const data = await response.json()
      const routes: RouteData[] = data.routes || []

      // Update internal state
      routes.forEach(route => {
        this.routes.set(route.routeId, route)
      })

      // Notify subscribers
      this.notifySubscribers('routes', routes)

      return routes
    } catch (error) {
      console.error('❌ Failed to fetch routes:', error)
      
      // Return mock data in development
      if (config.development.mockData) {
        return this.generateMockRoutes()
      }
      
      return []
    }
  }

  // Update route data
  private updateRoute(route: RouteData): void {
    this.routes.set(route.routeId, route)
    this.notifySubscribers('route_updated', route)
  }

  // Fetch geofences from API
  private async fetchGeofences(): Promise<GeofenceData[]> {
    try {
      const response = await fetch(`${config.apiBaseUrl}/api/v1/geofences/active`, {
        headers: {
          'Content-Type': 'application/json',
          'Accept': 'application/json'
        }
      })

      if (!response.ok) {
        throw new Error(`HTTP ${response.status}: ${response.statusText}`)
      }

      const data = await response.json()
      const geofences: GeofenceData[] = data.geofences || []

      // Update internal state
      geofences.forEach(geofence => {
        this.geofences.set(geofence.geofenceId, geofence)
      })

      // Notify subscribers
      this.notifySubscribers('geofences', geofences)

      return geofences
    } catch (error) {
      console.error('❌ Failed to fetch geofences:', error)
      
      // Return mock data in development
      if (config.development.mockData) {
        return this.generateMockGeofences()
      }
      
      return []
    }
  }

  // Check vehicle against geofences
  private checkVehicleGeofences(vehicle: VehiclePosition): void {
    this.geofences.forEach(geofence => {
      if (geofence.status !== 'active') return

      const isInside = this.isPointInGeofence(vehicle.position, geofence)
      
      geofence.rules.forEach(rule => {
        let violation = false

        switch (rule.condition) {
          case 'entry':
            violation = isInside
            break
          case 'exit':
            violation = !isInside
            break
          case 'speed':
            violation = isInside && vehicle.speed > (rule.threshold || 50)
            break
          case 'dwell':
            // Would need to track dwell time
            break
        }

        if (violation) {
          this.handleGeofenceViolation({
            vehicleId: vehicle.vehicleId,
            geofenceId: geofence.geofenceId,
            rule,
            position: vehicle.position,
            timestamp: new Date()
          })
        }
      })
    })
  }

  // Check if point is inside geofence
  private isPointInGeofence(point: MapCoordinate, geofence: GeofenceData): boolean {
    switch (geofence.geometry.type) {
      case 'circle':
        if (!geofence.geometry.center || !geofence.geometry.radius) return false
        const distance = this.calculateDistance(point, geofence.geometry.center)
        return distance <= geofence.geometry.radius
      
      case 'polygon':
        if (!geofence.geometry.coordinates) return false
        return this.isPointInPolygon(point, geofence.geometry.coordinates)
      
      default:
        return false
    }
  }

  // Calculate distance between two points (Haversine formula)
  private calculateDistance(point1: MapCoordinate, point2: MapCoordinate): number {
    const R = 6371000 // Earth's radius in meters
    const φ1 = point1.lat * Math.PI / 180
    const φ2 = point2.lat * Math.PI / 180
    const Δφ = (point2.lat - point1.lat) * Math.PI / 180
    const Δλ = (point2.lng - point1.lng) * Math.PI / 180

    const a = Math.sin(Δφ/2) * Math.sin(Δφ/2) +
              Math.cos(φ1) * Math.cos(φ2) *
              Math.sin(Δλ/2) * Math.sin(Δλ/2)
    const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1-a))

    return R * c
  }

  // Point-in-polygon test (ray casting algorithm)
  private isPointInPolygon(point: MapCoordinate, polygon: MapCoordinate[]): boolean {
    let inside = false
    const x = point.lng
    const y = point.lat

    for (let i = 0, j = polygon.length - 1; i < polygon.length; j = i++) {
      const xi = polygon[i].lng
      const yi = polygon[i].lat
      const xj = polygon[j].lng
      const yj = polygon[j].lat

      if (((yi > y) !== (yj > y)) && (x < (xj - xi) * (y - yi) / (yj - yi) + xi)) {
        inside = !inside
      }
    }

    return inside
  }

  // Handle geofence violation
  private handleGeofenceViolation(violation: any): void {
    console.warn('🚨 Geofence violation detected:', violation)
    
    // Update geofence violation count
    const geofence = this.geofences.get(violation.geofenceId)
    if (geofence) {
      geofence.violationCount++
      geofence.lastViolation = violation.timestamp
      this.geofences.set(violation.geofenceId, geofence)
    }

    // Notify subscribers
    this.notifySubscribers('geofence_violation', violation)
  }

  // Handle emergency alert
  private handleEmergencyAlert(alert: any): void {
    console.error('🚨 Emergency alert:', alert)
    this.notifySubscribers('emergency_alert', alert)
  }

  // Check geofence violations periodically
  private async checkGeofenceViolations(): Promise<void> {
    this.vehiclePositions.forEach(vehicle => {
      this.checkVehicleGeofences(vehicle)
    })
  }

  // Fetch route updates
  private async fetchRouteUpdates(): Promise<void> {
    // This would fetch any route changes, ETA updates, etc.
    // For now, just trigger a routes refresh
    await this.fetchRoutes()
  }

  // Subscribe to updates
  subscribe(eventType: string, callback: (data: any) => void): string {
    const subscriptionId = `${eventType}_${Date.now()}_${Math.random()}`
    this.subscribers.set(subscriptionId, callback)
    return subscriptionId
  }

  // Unsubscribe from updates
  unsubscribe(subscriptionId: string): void {
    this.subscribers.delete(subscriptionId)
  }

  // Notify all subscribers
  private notifySubscribers(eventType: string, data: any): void {
    this.subscribers.forEach((callback, id) => {
      if (id.startsWith(eventType)) {
        try {
          callback(data)
        } catch (error) {
          console.error(`❌ Subscriber callback error for ${eventType}:`, error)
        }
      }
    })
  }

  // Get current vehicle positions
  getVehiclePositions(): VehiclePosition[] {
    return Array.from(this.vehiclePositions.values())
  }

  // Get vehicle trail
  getVehicleTrail(vehicleId: string): MapCoordinate[] {
    return this.vehicleTrails.get(vehicleId) || []
  }

  // Get current routes
  getRoutes(): RouteData[] {
    return Array.from(this.routes.values())
  }

  // Get current geofences
  getGeofences(): GeofenceData[] {
    return Array.from(this.geofences.values())
  }

  // Generate mock vehicle positions for development
  private generateMockVehiclePositions(): VehiclePosition[] {
    const mockVehicles: VehiclePosition[] = []
    const abuDhabiCenter = this.config.abuDhabiCenter
    
    for (let i = 1; i <= 10; i++) {
      // Generate positions around Abu Dhabi
      const lat = abuDhabiCenter.lat + (Math.random() - 0.5) * 0.1
      const lng = abuDhabiCenter.lng + (Math.random() - 0.5) * 0.1
      
      mockVehicles.push({
        vehicleId: `AV-${i.toString().padStart(3, '0')}`,
        position: { lat, lng },
        heading: Math.random() * 360,
        speed: Math.random() * 60, // km/h
        timestamp: new Date(),
        accuracy: 3 + Math.random() * 2,
        status: ['operational', 'warning', 'critical', 'offline', 'charging'][Math.floor(Math.random() * 5)] as any,
        autonomyLevel: ['L3', 'L4', 'L5'][Math.floor(Math.random() * 3)] as any,
        batteryLevel: 20 + Math.random() * 80,
        currentTripId: Math.random() > 0.5 ? `trip-${i}` : undefined
      })
    }
    
    return mockVehicles
  }

  // Generate mock routes for development
  private generateMockRoutes(): RouteData[] {
    const abuDhabiCenter = this.config.abuDhabiCenter
    
    return [
      {
        routeId: 'route-001',
        name: 'Abu Dhabi Airport → Downtown',
        coordinates: [
          { lat: 24.4330, lng: 54.6511 }, // Airport
          { lat: 24.4539, lng: 54.3773 }  // Downtown
        ],
        status: 'active',
        vehicleId: 'AV-001',
        estimatedDuration: 1800, // 30 minutes
        distance: 35000 // 35 km
      },
      {
        routeId: 'route-002',
        name: 'Corniche → Marina Mall',
        coordinates: [
          { lat: 24.4797, lng: 54.3178 }, // Corniche
          { lat: 24.4923, lng: 54.3203 }  // Marina Mall
        ],
        status: 'active',
        vehicleId: 'AV-002',
        estimatedDuration: 900, // 15 minutes
        distance: 8000 // 8 km
      }
    ]
  }

  // Generate mock geofences for development
  private generateMockGeofences(): GeofenceData[] {
    return [
      {
        geofenceId: 'geofence-001',
        name: 'Abu Dhabi Airport Restricted Zone',
        type: 'circle',
        geometry: {
          type: 'circle',
          center: { lat: 24.4330, lng: 54.6511 },
          radius: 2000 // 2km radius
        },
        rules: [
          {
            condition: 'entry',
            action: 'alert',
            severity: 'warning'
          }
        ],
        status: 'active',
        violationCount: 0
      },
      {
        geofenceId: 'geofence-002',
        name: 'Downtown Speed Zone',
        type: 'polygon',
        geometry: {
          type: 'polygon',
          coordinates: [
            { lat: 24.4500, lng: 54.3700 },
            { lat: 24.4600, lng: 54.3700 },
            { lat: 24.4600, lng: 54.3850 },
            { lat: 24.4500, lng: 54.3850 }
          ]
        },
        rules: [
          {
            condition: 'speed',
            action: 'alert',
            severity: 'critical',
            threshold: 30 // 30 km/h
          }
        ],
        status: 'active',
        violationCount: 2,
        lastViolation: new Date(Date.now() - 3600000) // 1 hour ago
      }
    ]
  }

  // Cleanup resources
  dispose(): void {
    if (this.updateInterval) {
      clearInterval(this.updateInterval)
    }
    
    if (this.websocket) {
      this.websocket.close()
    }
    
    this.subscribers.clear()
    this.vehiclePositions.clear()
    this.vehicleTrails.clear()
    this.routes.clear()
    this.geofences.clear()
    
    console.info('🧹 Map service disposed')
  }
}

// Export singleton instance
export const mapService = new MapService({
  updateInterval: config.performance.mapUpdateInterval,
  maxVehicles: config.performance.maxVehiclesOnMap,
  enableClustering: config.features.enableClustering,
  enableRealTimeUpdates: config.features.enableRealTimeUpdates,
  enableVehicleTrails: config.features.enableVehicleTrails,
  trailLength: 50,
  defaultProvider: config.defaultMap.provider,
  abuDhabiCenter: config.defaultMap.center
})

export default MapService
