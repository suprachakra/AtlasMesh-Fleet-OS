import React, { useState, useEffect, useRef, useCallback, useMemo } from 'react'
import { 
  Map, Layers, RotateCcw, Satellite, Navigation, MapPin, 
  Zap, Shield, AlertTriangle, Eye, EyeOff, Settings 
} from 'lucide-react'
import { Button } from '../ui/Button'
import { Badge } from '../ui/Badge'
import { Select } from '../ui/Select'
import { Switch } from '../ui/Switch'
import { Card } from '../ui/Card'
import { useMapProvider, MapProvider, ABU_DHABI_CENTER } from '../../hooks/useMapProvider'

// Vehicle and route types
export interface Vehicle {
  vehicleId: string
  position: {
    lat: number
    lng: number
    heading: number
    speed: number
    accuracy: number
  }
  status: 'operational' | 'warning' | 'critical' | 'offline' | 'charging'
  autonomyLevel: 'L0' | 'L1' | 'L2' | 'L3' | 'L4' | 'L5'
  batteryLevel?: number
  currentTrip?: string
  lastUpdate: Date
}

export interface Route {
  routeId: string
  name: string
  coordinates: Array<{ lat: number; lng: number }>
  type: 'active' | 'planned' | 'completed' | 'emergency'
  vehicleId?: string
  color?: string
}

export interface Geofence {
  geofenceId: string
  name: string
  type: 'circle' | 'polygon'
  geometry: {
    type: 'circle' | 'polygon'
    center?: { lat: number; lng: number }
    radius?: number
    coordinates?: Array<{ lat: number; lng: number }>
  }
  rules: Array<{
    condition: 'entry' | 'exit' | 'dwell' | 'speed'
    action: 'alert' | 'stop' | 'reroute' | 'log'
    severity: 'info' | 'warning' | 'critical'
  }>
  status: 'active' | 'inactive'
  color?: string
}

export interface MapLayer {
  id: string
  name: string
  type: 'vehicles' | 'routes' | 'geofences' | 'traffic' | 'weather' | 'incidents'
  visible: boolean
  opacity: number
  color?: string
}

interface DualMapProviderProps {
  vehicles?: Vehicle[]
  routes?: Route[]
  geofences?: Geofence[]
  selectedVehicleId?: string
  selectedRouteId?: string
  onVehicleSelect?: (vehicleId: string) => void
  onRouteSelect?: (routeId: string) => void
  onMapClick?: (lat: number, lng: number) => void
  className?: string
  height?: string
  showControls?: boolean
  showLegend?: boolean
  initialProvider?: MapProvider
  enableClustering?: boolean
  enableRealTimeUpdates?: boolean
}

const DualMapProvider: React.FC<DualMapProviderProps> = ({
  vehicles = [],
  routes = [],
  geofences = [],
  selectedVehicleId,
  selectedRouteId,
  onVehicleSelect,
  onRouteSelect,
  onMapClick,
  className = '',
  height = '600px',
  showControls = true,
  showLegend = true,
  initialProvider = 'openstreetmap',
  enableClustering = true,
  enableRealTimeUpdates = true
}) => {
  const mapContainerRef = useRef<HTMLDivElement>(null)
  const [activeLayers, setActiveLayers] = useState<Set<string>>(new Set([
    'vehicles', 'routes', 'geofences'
  ]))
  const [showTrails, setShowTrails] = useState(false)
  const [showHeatmap, setShowHeatmap] = useState(false)
  const [mapStyle, setMapStyle] = useState('default')
  const [isFollowingVehicle, setIsFollowingVehicle] = useState<string | null>(null)

  // Initialize map provider
  const mapInstance = useMapProvider({
    container: mapContainerRef.current,
    initialProvider,
    center: ABU_DHABI_CENTER,
    zoom: 12,
    onProviderChange: (provider) => {
      console.info(`Map provider switched to: ${provider}`)
    },
    onMapReady: (map) => {
      console.info('Map ready:', map)
      // Add click handler
      if (onMapClick) {
        addMapClickHandler(map, mapInstance.provider)
      }
    },
    onError: (error) => {
      console.error('Map error:', error)
    }
  })

  // Add map click handler based on provider
  const addMapClickHandler = useCallback((map: any, provider: MapProvider) => {
    if (!onMapClick) return

    switch (provider) {
      case 'openstreetmap':
        map.on('click', (e: any) => {
          onMapClick(e.latlng.lat, e.latlng.lng)
        })
        break
      case 'google':
      case 'satellite':
        map.addListener('click', (e: any) => {
          onMapClick(e.latLng.lat(), e.latLng.lng())
        })
        break
    }
  }, [onMapClick])

  // Vehicle status colors and icons
  const vehicleStatusConfig = useMemo(() => ({
    operational: { color: '#10B981', icon: '🚗', label: 'Operational' },
    warning: { color: '#F59E0B', icon: '⚠️', label: 'Warning' },
    critical: { color: '#EF4444', icon: '🚨', label: 'Critical' },
    offline: { color: '#6B7280', icon: '📵', label: 'Offline' },
    charging: { color: '#8B5CF6', icon: '🔋', label: 'Charging' }
  }), [])

  // Route type colors
  const routeTypeConfig = useMemo(() => ({
    active: { color: '#3B82F6', weight: 4, opacity: 0.8 },
    planned: { color: '#6B7280', weight: 3, opacity: 0.6 },
    completed: { color: '#10B981', weight: 2, opacity: 0.4 },
    emergency: { color: '#EF4444', weight: 5, opacity: 1.0 }
  }), [])

  // Update vehicle markers
  useEffect(() => {
    if (!mapInstance.isLoaded || !activeLayers.has('vehicles')) return

    // Clear existing vehicle markers
    vehicles.forEach(vehicle => {
      mapInstance.removeMarker(`vehicle-${vehicle.vehicleId}`)
    })

    // Add vehicle markers
    vehicles.forEach(vehicle => {
      const config = vehicleStatusConfig[vehicle.status]
      const isSelected = selectedVehicleId === vehicle.vehicleId

      // Create popup content
      const popupContent = `
        <div class="p-2 min-w-[200px]">
          <h3 class="font-semibold text-gray-900">${vehicle.vehicleId}</h3>
          <div class="mt-2 space-y-1 text-sm">
            <div class="flex justify-between">
              <span class="text-gray-600">Status:</span>
              <span class="font-medium" style="color: ${config.color}">${config.label}</span>
            </div>
            <div class="flex justify-between">
              <span class="text-gray-600">Speed:</span>
              <span class="font-medium">${(vehicle.position.speed * 3.6).toFixed(1)} km/h</span>
            </div>
            <div class="flex justify-between">
              <span class="text-gray-600">Heading:</span>
              <span class="font-medium">${vehicle.position.heading}°</span>
            </div>
            ${vehicle.batteryLevel ? `
              <div class="flex justify-between">
                <span class="text-gray-600">Battery:</span>
                <span class="font-medium">${vehicle.batteryLevel}%</span>
              </div>
            ` : ''}
            <div class="flex justify-between">
              <span class="text-gray-600">Last Update:</span>
              <span class="font-medium">${vehicle.lastUpdate.toLocaleTimeString()}</span>
            </div>
          </div>
          ${vehicle.currentTrip ? `
            <div class="mt-2 p-2 bg-blue-50 rounded">
              <span class="text-xs text-blue-600">Current Trip: ${vehicle.currentTrip}</span>
            </div>
          ` : ''}
        </div>
      `

      mapInstance.addMarker(
        `vehicle-${vehicle.vehicleId}`,
        vehicle.position.lat,
        vehicle.position.lng,
        {
          icon: mapInstance.provider === 'openstreetmap' ? createLeafletVehicleIcon(vehicle, config, isSelected) : undefined,
          iconUrl: mapInstance.provider !== 'openstreetmap' ? createGoogleVehicleIcon(vehicle, config, isSelected) : undefined,
          title: `${vehicle.vehicleId} - ${config.label}`,
          popup: popupContent
        }
      )
    })

    // Follow selected vehicle
    if (isFollowingVehicle && selectedVehicleId) {
      const vehicle = vehicles.find(v => v.vehicleId === selectedVehicleId)
      if (vehicle) {
        mapInstance.setCenter(vehicle.position.lat, vehicle.position.lng)
      }
    }
  }, [mapInstance, vehicles, activeLayers, selectedVehicleId, vehicleStatusConfig, isFollowingVehicle])

  // Update route overlays
  useEffect(() => {
    if (!mapInstance.isLoaded || !activeLayers.has('routes')) return

    // Clear existing routes
    routes.forEach(route => {
      mapInstance.removeRoute(`route-${route.routeId}`)
    })

    // Add routes
    routes.forEach(route => {
      const config = routeTypeConfig[route.type]
      const isSelected = selectedRouteId === route.routeId

      mapInstance.addRoute(
        `route-${route.routeId}`,
        route.coordinates,
        {
          color: route.color || config.color,
          weight: isSelected ? config.weight + 1 : config.weight,
          opacity: isSelected ? 1.0 : config.opacity
        }
      )
    })
  }, [mapInstance, routes, activeLayers, selectedRouteId, routeTypeConfig])

  // Update geofence overlays
  useEffect(() => {
    if (!mapInstance.isLoaded || !activeLayers.has('geofences')) return

    // Clear existing geofences
    geofences.forEach(geofence => {
      mapInstance.removeGeofence(`geofence-${geofence.geofenceId}`)
    })

    // Add geofences
    geofences.forEach(geofence => {
      if (geofence.status !== 'active') return

      mapInstance.addGeofence(
        `geofence-${geofence.geofenceId}`,
        geofence.geometry,
        {
          strokeColor: geofence.color || '#EF4444',
          fillColor: geofence.color || '#EF4444',
          fillOpacity: 0.2,
          strokeWeight: 2,
          strokeOpacity: 0.8
        }
      )
    })
  }, [mapInstance, geofences, activeLayers])

  // Create Leaflet vehicle icon
  const createLeafletVehicleIcon = useCallback((vehicle: Vehicle, config: any, isSelected: boolean) => {
    const size = isSelected ? 36 : 28
    return {
      iconUrl: `data:image/svg+xml;base64,${btoa(`
        <svg width="${size}" height="${size}" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
          <circle cx="12" cy="12" r="10" fill="${config.color}" stroke="white" stroke-width="2"/>
          <text x="12" y="16" font-family="Arial" font-size="12" fill="white" text-anchor="middle">${config.icon}</text>
          ${isSelected ? '<circle cx="12" cy="12" r="12" fill="none" stroke="white" stroke-width="3" opacity="0.8"/>' : ''}
        </svg>
      `)}`,
      iconSize: [size, size],
      iconAnchor: [size / 2, size / 2],
      popupAnchor: [0, -size / 2]
    }
  }, [])

  // Create Google Maps vehicle icon URL
  const createGoogleVehicleIcon = useCallback((vehicle: Vehicle, config: any, isSelected: boolean) => {
    const size = isSelected ? 36 : 28
    return `data:image/svg+xml;base64,${btoa(`
      <svg width="${size}" height="${size}" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
        <circle cx="12" cy="12" r="10" fill="${config.color}" stroke="white" stroke-width="2"/>
        <text x="12" y="16" font-family="Arial" font-size="12" fill="white" text-anchor="middle">${config.icon}</text>
        ${isSelected ? '<circle cx="12" cy="12" r="12" fill="none" stroke="white" stroke-width="3" opacity="0.8"/>' : ''}
      </svg>
    `)}`
  }, [])

  // Layer toggle handler
  const toggleLayer = useCallback((layerId: string) => {
    setActiveLayers(prev => {
      const newLayers = new Set(prev)
      if (newLayers.has(layerId)) {
        newLayers.delete(layerId)
      } else {
        newLayers.add(layerId)
      }
      return newLayers
    })
  }, [])

  // Fit bounds to show all vehicles
  const fitToVehicles = useCallback(() => {
    if (vehicles.length === 0) return

    const bounds = vehicles.reduce((acc, vehicle) => {
      return {
        north: Math.max(acc.north, vehicle.position.lat),
        south: Math.min(acc.south, vehicle.position.lat),
        east: Math.max(acc.east, vehicle.position.lng),
        west: Math.min(acc.west, vehicle.position.lng)
      }
    }, {
      north: vehicles[0].position.lat,
      south: vehicles[0].position.lat,
      east: vehicles[0].position.lng,
      west: vehicles[0].position.lng
    })

    mapInstance.fitBounds(bounds)
  }, [vehicles, mapInstance])

  // Reset to Abu Dhabi center
  const resetToCenter = useCallback(() => {
    mapInstance.setCenter(ABU_DHABI_CENTER.lat, ABU_DHABI_CENTER.lng, ABU_DHABI_CENTER.zoom)
    setIsFollowingVehicle(null)
  }, [mapInstance])

  return (
    <div className={`relative ${className}`} style={{ height }}>
      {/* Map Container */}
      <div 
        ref={mapContainerRef} 
        className="w-full h-full rounded-lg overflow-hidden"
        style={{ minHeight: '400px' }}
      />

      {/* Map Controls */}
      {showControls && (
        <div className="absolute top-4 right-4 flex flex-col space-y-2">
          <Card className="p-2">
            <div className="flex flex-col space-y-2">
              {/* Map Provider Selector */}
              <div className="flex items-center space-x-2">
                <Map className="w-4 h-4 text-gray-600" />
                <Select
                  value={mapInstance.provider}
                  onValueChange={(value: MapProvider) => mapInstance.switchProvider(value)}
                  className="text-xs"
                >
                  <option value="openstreetmap">OpenStreetMap</option>
                  <option value="google">Google Maps</option>
                  <option value="satellite">Satellite</option>
                </Select>
              </div>

              {/* Layer Controls */}
              <div className="flex items-center space-x-1">
                <Layers className="w-4 h-4 text-gray-600" />
                <div className="flex space-x-1">
                  {[
                    { id: 'vehicles', icon: '🚗', label: 'Vehicles' },
                    { id: 'routes', icon: '🛣️', label: 'Routes' },
                    { id: 'geofences', icon: '🚧', label: 'Geofences' }
                  ].map(layer => (
                    <Button
                      key={layer.id}
                      variant={activeLayers.has(layer.id) ? 'default' : 'outline'}
                      size="sm"
                      onClick={() => toggleLayer(layer.id)}
                      className="text-xs px-2 py-1"
                      title={layer.label}
                    >
                      {layer.icon}
                    </Button>
                  ))}
                </div>
              </div>

              {/* Map Actions */}
              <div className="flex space-x-1">
                <Button
                  variant="outline"
                  size="sm"
                  onClick={fitToVehicles}
                  className="text-xs px-2 py-1"
                  title="Fit to Vehicles"
                  disabled={vehicles.length === 0}
                >
                  <Navigation className="w-3 h-3" />
                </Button>
                <Button
                  variant="outline"
                  size="sm"
                  onClick={resetToCenter}
                  className="text-xs px-2 py-1"
                  title="Reset to Abu Dhabi"
                >
                  <RotateCcw className="w-3 h-3" />
                </Button>
              </div>

              {/* Additional Controls */}
              <div className="flex flex-col space-y-1">
                <div className="flex items-center justify-between">
                  <span className="text-xs text-gray-600">Trails</span>
                  <Switch
                    checked={showTrails}
                    onCheckedChange={setShowTrails}
                    size="sm"
                  />
                </div>
                <div className="flex items-center justify-between">
                  <span className="text-xs text-gray-600">Heatmap</span>
                  <Switch
                    checked={showHeatmap}
                    onCheckedChange={setShowHeatmap}
                    size="sm"
                  />
                </div>
              </div>
            </div>
          </Card>
        </div>
      )}

      {/* Map Status */}
      <div className="absolute bottom-4 left-4 flex items-center space-x-2">
        <Badge 
          className={`${mapInstance.isLoaded ? 'bg-green-100 text-green-800' : 'bg-yellow-100 text-yellow-800'}`}
        >
          {mapInstance.provider.toUpperCase()} {mapInstance.isLoaded ? 'Ready' : 'Loading...'}
        </Badge>
        
        {mapInstance.error && (
          <Badge className="bg-red-100 text-red-800">
            Error: {mapInstance.error}
          </Badge>
        )}

        {enableRealTimeUpdates && (
          <Badge className="bg-blue-100 text-blue-800">
            Live Updates
          </Badge>
        )}
      </div>

      {/* Legend */}
      {showLegend && (
        <div className="absolute bottom-4 right-4">
          <Card className="p-3">
            <h4 className="text-sm font-medium text-gray-900 mb-2">Legend</h4>
            <div className="space-y-1">
              {Object.entries(vehicleStatusConfig).map(([status, config]) => (
                <div key={status} className="flex items-center space-x-2 text-xs">
                  <div 
                    className="w-3 h-3 rounded-full border border-white"
                    style={{ backgroundColor: config.color }}
                  />
                  <span className="text-gray-600">{config.label}</span>
                </div>
              ))}
            </div>
          </Card>
        </div>
      )}

      {/* Loading Overlay */}
      {!mapInstance.isLoaded && (
        <div className="absolute inset-0 bg-gray-50 bg-opacity-75 flex items-center justify-center">
          <div className="text-center">
            <div className="animate-spin rounded-full h-8 w-8 border-b-2 border-blue-600 mx-auto mb-2"></div>
            <p className="text-sm text-gray-600">Loading {mapInstance.provider} map...</p>
          </div>
        </div>
      )}
    </div>
  )
}

export default DualMapProvider
