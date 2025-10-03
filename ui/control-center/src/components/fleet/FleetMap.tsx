import React, { useMemo } from 'react'
import { useTranslation } from 'react-i18next'
import DualMapProvider from './DualMapProvider'

// Hooks
import { useRealTimeVehicles } from '../../hooks/useRealTimeVehicles'
import { useGeofences } from '@hooks/useGeofences'

// Types
import type { Vehicle, Trip, Geofence, MapLayer } from '@types/fleet'

interface FleetMapProps {
  vehicles?: Vehicle[]
  trips?: Trip[]
  selectedVehicleId?: string
  selectedTripId?: string
  onVehicleSelect?: (vehicleId: string) => void
  onTripSelect?: (tripId: string) => void
  onBulkSelect?: (vehicleIds: string[]) => void
  className?: string
  height?: string
  showControls?: boolean
  showLegend?: boolean
  enableSelection?: boolean
  enableBulkSelection?: boolean
  timeRange?: 'live' | '15m' | '1h' | '24h'
}

export function FleetMap({
  vehicles = [],
  trips = [],
  selectedVehicleId,
  selectedTripId,
  onVehicleSelect,
  onTripSelect,
  onBulkSelect,
  className = '',
  height = '600px',
  showControls = true,
  showLegend = true,
  enableSelection = true,
  enableBulkSelection = false,
  timeRange = 'live',
}: FleetMapProps) {
  const { t } = useTranslation()

  // Use real-time vehicle tracking
  const {
    vehicles: realTimeVehicles,
    vehicleTrails,
    selectedVehicle,
    isLoading,
    error,
    lastUpdate,
    connectionStatus,
    violationAlerts,
    selectVehicle,
    statistics
  } = useRealTimeVehicles({
    enableRealTime: timeRange === 'live',
    enableTrails: true,
    trailLength: 50,
    updateInterval: 5000,
    maxVehicles: 200
  })

  // Use real-time vehicles if available, fallback to prop vehicles
  const activeVehicles = realTimeVehicles.length > 0 ? realTimeVehicles : vehicles

  // Convert vehicles to DualMapProvider format
  const mappedVehicles = useMemo(() => {
    return activeVehicles.map(vehicle => ({
      vehicleId: vehicle.vehicleId,
      position: {
        lat: 'position' in vehicle ? vehicle.position.lat : (vehicle.location?.latitude || 24.4539),
        lng: 'position' in vehicle ? vehicle.position.lng : (vehicle.location?.longitude || 54.3773),
        heading: 'position' in vehicle ? vehicle.heading : (vehicle.location?.heading || 0),
        speed: 'position' in vehicle ? vehicle.speed : (vehicle.location?.speed || 0),
        accuracy: 'position' in vehicle ? vehicle.accuracy : (vehicle.location?.accuracy || 5)
      },
      status: vehicle.status as 'operational' | 'warning' | 'critical' | 'offline' | 'charging',
      autonomyLevel: vehicle.autonomyLevel || 'L4',
      batteryLevel: vehicle.batteryLevel,
      currentTrip: 'currentTripId' in vehicle ? vehicle.currentTripId : vehicle.currentTrip,
      lastUpdate: 'timestamp' in vehicle ? vehicle.timestamp : new Date(vehicle.lastUpdate || Date.now())
    }))
  }, [activeVehicles])

  // Convert routes to DualMapProvider format
  const mappedRoutes = useMemo(() => {
    return trips.map(trip => ({
      routeId: trip.tripId,
      name: trip.origin + ' → ' + trip.destination,
      coordinates: trip.route?.waypoints?.map(wp => ({
        lat: wp.latitude,
        lng: wp.longitude
      })) || [],
      type: trip.status === 'active' ? 'active' as const : 
            trip.status === 'completed' ? 'completed' as const : 
            'planned' as const,
      vehicleId: trip.assignedVehicleId,
      color: trip.status === 'active' ? '#3B82F6' : 
             trip.status === 'completed' ? '#10B981' : '#6B7280'
    }))
  }, [trips])

  // Get geofences data
  const { geofences: rawGeofences } = useGeofences()

  // Convert geofences to DualMapProvider format
  const mappedGeofences = useMemo(() => {
    return rawGeofences.map(geofence => ({
      geofenceId: geofence.id,
      name: geofence.name,
      type: geofence.shape === 'circle' ? 'circle' as const : 'polygon' as const,
      geometry: {
        type: geofence.shape === 'circle' ? 'circle' as const : 'polygon' as const,
        center: geofence.shape === 'circle' ? {
          lat: geofence.center?.latitude || 0,
          lng: geofence.center?.longitude || 0
        } : undefined,
        radius: geofence.radius,
        coordinates: geofence.shape === 'polygon' ? 
          geofence.coordinates?.map(coord => ({
            lat: coord.latitude,
            lng: coord.longitude
          })) : undefined
      },
      rules: geofence.rules?.map(rule => ({
        condition: rule.type as 'entry' | 'exit' | 'dwell' | 'speed',
        action: rule.action as 'alert' | 'stop' | 'reroute' | 'log',
        severity: rule.severity as 'info' | 'warning' | 'critical'
      })) || [],
      status: geofence.active ? 'active' as const : 'inactive' as const,
      color: geofence.color || '#EF4444'
    }))
  }, [rawGeofences])

  // Handle bulk selection
  const handleVehicleSelect = (vehicleId: string) => {
    // Update real-time vehicle selection
    selectVehicle(vehicleId)
    
    if (enableBulkSelection) {
      // Handle bulk selection logic here
      onBulkSelect?.([vehicleId])
    } else {
      onVehicleSelect?.(vehicleId)
    }
  }

  // Show loading or error states
  if (isLoading && activeVehicles.length === 0) {
    return (
      <div className={`relative ${className} flex items-center justify-center`} style={{ height }}>
        <div className="text-center">
          <div className="animate-spin rounded-full h-8 w-8 border-b-2 border-blue-600 mx-auto mb-2"></div>
          <p className="text-sm text-gray-600">Loading Abu Dhabi fleet data...</p>
          <p className="text-xs text-gray-500">Connection: {connectionStatus}</p>
        </div>
      </div>
    )
  }

  if (error) {
    return (
      <div className={`relative ${className} flex items-center justify-center`} style={{ height }}>
        <div className="text-center">
          <div className="text-red-500 mb-2">⚠️</div>
          <p className="text-sm text-red-600">Failed to load fleet data</p>
          <p className="text-xs text-gray-500">{error}</p>
          <button 
            onClick={() => window.location.reload()} 
            className="mt-2 px-3 py-1 text-xs bg-blue-500 text-white rounded hover:bg-blue-600"
          >
            Retry
          </button>
        </div>
      </div>
    )
  }

  return (
    <div className={`relative ${className}`} style={{ height }}>
      {/* Real-time status indicator */}
      {timeRange === 'live' && (
        <div className="absolute top-2 left-2 z-10">
          <div className={`px-2 py-1 text-xs rounded-full ${
            connectionStatus === 'connected' 
              ? 'bg-green-100 text-green-800' 
              : connectionStatus === 'connecting'
              ? 'bg-yellow-100 text-yellow-800'
              : 'bg-red-100 text-red-800'
          }`}>
            <div className="flex items-center space-x-1">
              <div className={`w-2 h-2 rounded-full ${
                connectionStatus === 'connected' 
                  ? 'bg-green-500 animate-pulse' 
                  : connectionStatus === 'connecting'
                  ? 'bg-yellow-500 animate-pulse'
                  : 'bg-red-500'
              }`}></div>
              <span>
                {connectionStatus === 'connected' ? 'Live' : 
                 connectionStatus === 'connecting' ? 'Connecting' : 'Offline'}
              </span>
            </div>
          </div>
        </div>
      )}

      {/* Violation alerts */}
      {violationAlerts.length > 0 && (
        <div className="absolute top-2 right-2 z-10">
          <div className="bg-red-100 border border-red-300 rounded-lg p-2 max-w-sm">
            <div className="flex items-center space-x-2">
              <div className="text-red-500">🚨</div>
              <div className="text-xs text-red-800">
                {violationAlerts.length} active alert{violationAlerts.length > 1 ? 's' : ''}
              </div>
            </div>
            <div className="mt-1 text-xs text-red-700">
              Latest: {violationAlerts[0]?.message}
            </div>
          </div>
        </div>
      )}

      {/* Fleet statistics */}
      {statistics && timeRange === 'live' && (
        <div className="absolute bottom-2 left-2 z-10">
          <div className="bg-white bg-opacity-90 rounded-lg p-2 text-xs">
            <div className="font-medium text-gray-900">Fleet Status</div>
            <div className="mt-1 space-y-1">
              <div className="flex justify-between">
                <span className="text-green-600">Operational:</span>
                <span className="font-medium">{statistics.operational}</span>
              </div>
              <div className="flex justify-between">
                <span className="text-yellow-600">Warning:</span>
                <span className="font-medium">{statistics.warning}</span>
              </div>
              <div className="flex justify-between">
                <span className="text-red-600">Critical:</span>
                <span className="font-medium">{statistics.critical}</span>
              </div>
              <div className="flex justify-between">
                <span className="text-gray-600">Avg Speed:</span>
                <span className="font-medium">{statistics.averageSpeed.toFixed(1)} km/h</span>
              </div>
            </div>
            {lastUpdate && (
              <div className="mt-1 text-gray-500">
                Updated: {lastUpdate.toLocaleTimeString()}
              </div>
            )}
          </div>
        </div>
      )}

      <DualMapProvider
        vehicles={mappedVehicles}
        routes={mappedRoutes}
        geofences={mappedGeofences}
        selectedVehicleId={selectedVehicleId || selectedVehicle?.vehicleId}
        selectedRouteId={selectedTripId}
        onVehicleSelect={handleVehicleSelect}
        onRouteSelect={onTripSelect}
        onMapClick={(lat, lng) => {
          console.info(`Abu Dhabi map clicked at: ${lat.toFixed(6)}, ${lng.toFixed(6)}`)
          // Handle map click for route planning, geofence creation, etc.
        }}
        className="w-full h-full"
        height="100%"
        showControls={showControls}
        showLegend={showLegend}
        initialProvider="openstreetmap"
        enableClustering={true}
        enableRealTimeUpdates={timeRange === 'live'}
      />
    </div>
  )
}