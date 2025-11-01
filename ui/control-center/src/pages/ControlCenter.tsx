import React, { useState, useCallback, useMemo, useEffect, useRef } from 'react'
import { 
  MapPin, Zap, AlertTriangle, Clock, Battery, Wifi, Phone, Camera, Navigation, 
  Pause, Play, Square, RotateCcw, Filter, Search, RefreshCw, Layers, Settings, 
  Users, Activity, Shield, Radio, Eye, EyeOff, Maximize2, Minimize2, Volume2, 
  VolumeX, Target, Route, Compass, Thermometer, Gauge, Signal, WifiOff,
  CheckCircle, XCircle, AlertCircle, Info
} from 'lucide-react'
import { Button } from '../components/ui/Button'
import { Badge } from '../components/ui/Badge'
import { Card } from '../components/ui/Card'
import { Input } from '../components/ui/Input'
import { Select } from '../components/ui/Select'
import { Tabs, TabsContent, TabsList, TabsTrigger } from '../components/ui/Tabs'
import { Dialog, DialogContent, DialogHeader, DialogTitle } from '../components/ui/Dialog'
import { Alert, AlertDescription } from '../components/ui/Alert'
import { Tooltip, TooltipContent, TooltipProvider, TooltipTrigger } from '../components/ui/Tooltip'
import { Slider } from '../components/ui/Slider'
import { Switch } from '../components/ui/Switch'
import { Progress } from '../components/ui/Progress'
import { ScrollArea } from '../components/ui/ScrollArea'

// Types
interface Vehicle {
  id: string
  name: string
  plate: string
  model: string
  status: 'online' | 'offline' | 'in_trip' | 'maintenance' | 'emergency'
  operationalStatus: 'available' | 'assigned' | 'en_route' | 'loading' | 'unloading' | 'paused'
  autonomyLevel: 'L0' | 'L1' | 'L2' | 'L3' | 'L4' | 'L5'
  position: {
    lat: number
    lng: number
    heading: number
    speed: number
    altitude?: number
  }
  telemetry: {
    battery: number
    connectivity: 'excellent' | 'good' | 'fair' | 'poor' | 'disconnected'
    temperature: number
    lastUpdate: Date
    networkLatency: number
    signalStrength: number
  }
  trip?: {
    id: string
    status: 'planned' | 'active' | 'completed' | 'cancelled'
    progress: number
    eta: Date
    destination: string
  }
  alerts: VehicleAlert[]
  driver?: {
    id: string
    name: string
    status: 'active' | 'break' | 'offline'
  }
  capabilities: {
    teleoperation: boolean
    remoteAssist: boolean
    emergencyStop: boolean
    diagnostics: boolean
  }
}

interface VehicleAlert {
  id: string
  severity: 'info' | 'warning' | 'critical' | 'emergency'
  type: string
  message: string
  timestamp: Date
  acknowledged: boolean
}

interface MapLayer {
  id: string
  name: string
  visible: boolean
  opacity: number
  type: 'vehicles' | 'trips' | 'geofences' | 'weather' | 'traffic' | 'hazards' | 'hd_map'
}

interface ConnectionStatus {
  websocket: 'connected' | 'disconnected' | 'reconnecting'
  api: 'healthy' | 'degraded' | 'offline'
  lastUpdate: Date
  latency: number
}

// Mock data
const mockVehicles: Vehicle[] = [
  {
    id: 'v1',
    name: 'Atlas-001',
    plate: 'AV-001',
    model: 'Atlas Mesh L4',
    status: 'online',
    operationalStatus: 'en_route',
    autonomyLevel: 'L4',
    position: { lat: 25.2048, lng: 55.2708, heading: 45, speed: 35 },
    telemetry: {
      battery: 85,
      connectivity: 'excellent',
      temperature: 42,
      lastUpdate: new Date(),
      networkLatency: 45,
      signalStrength: -65
    },
    trip: {
      id: 't1',
      status: 'active',
      progress: 65,
      eta: new Date(Date.now() + 15 * 60 * 1000),
      destination: 'Dubai Mall'
    },
    alerts: [
      {
        id: 'a1',
        severity: 'warning',
        type: 'temperature',
        message: 'Engine temperature elevated',
        timestamp: new Date(Date.now() - 5 * 60 * 1000),
        acknowledged: false
      }
    ],
    driver: {
      id: 'd1',
      name: 'Ahmed Al-Mansouri',
      status: 'active'
    },
    capabilities: {
      teleoperation: true,
      remoteAssist: true,
      emergencyStop: true,
      diagnostics: true
    }
  },
  {
    id: 'v2',
    name: 'Atlas-002',
    plate: 'AV-002',
    model: 'Atlas Mesh L5',
    status: 'online',
    operationalStatus: 'available',
    autonomyLevel: 'L5',
    position: { lat: 25.1972, lng: 55.2744, heading: 180, speed: 0 },
    telemetry: {
      battery: 92,
      connectivity: 'good',
      temperature: 38,
      lastUpdate: new Date(Date.now() - 2 * 1000),
      networkLatency: 78,
      signalStrength: -72
    },
    alerts: [],
    capabilities: {
      teleoperation: true,
      remoteAssist: true,
      emergencyStop: true,
      diagnostics: true
    }
  },
  {
    id: 'v3',
    name: 'Atlas-003',
    plate: 'AV-003',
    model: 'Atlas Mesh L4',
    status: 'offline',
    operationalStatus: 'maintenance',
    autonomyLevel: 'L4',
    position: { lat: 25.2103, lng: 55.2640, heading: 270, speed: 0 },
    telemetry: {
      battery: 15,
      connectivity: 'disconnected',
      temperature: 35,
      lastUpdate: new Date(Date.now() - 10 * 60 * 1000),
      networkLatency: 0,
      signalStrength: 0
    },
    alerts: [
      {
        id: 'a2',
        severity: 'critical',
        type: 'battery',
        message: 'Battery level critically low',
        timestamp: new Date(Date.now() - 8 * 60 * 1000),
        acknowledged: false
      }
    ],
    capabilities: {
      teleoperation: false,
      remoteAssist: false,
      emergencyStop: false,
      diagnostics: false
    }
  }
]

const ControlCenter: React.FC = () => {
  // State
  const [vehicles, setVehicles] = useState<Vehicle[]>(mockVehicles)
  const [selectedVehicles, setSelectedVehicles] = useState<string[]>([])
  const [selectedVehicle, setSelectedVehicle] = useState<Vehicle | null>(null)
  const [showVehicleDetail, setShowVehicleDetail] = useState(false)
  const [mapLayers, setMapLayers] = useState<MapLayer[]>([
    { id: 'vehicles', name: 'Vehicles', visible: true, opacity: 100, type: 'vehicles' },
    { id: 'trips', name: 'Active Trips', visible: true, opacity: 90, type: 'trips' },
    { id: 'geofences', name: 'Geofences', visible: true, opacity: 70, type: 'geofences' },
    { id: 'weather', name: 'Weather', visible: false, opacity: 60, type: 'weather' },
    { id: 'traffic', name: 'Traffic', visible: false, opacity: 80, type: 'traffic' },
    { id: 'hazards', name: 'Hazards', visible: true, opacity: 100, type: 'hazards' },
    { id: 'hd_map', name: 'HD Map', visible: false, opacity: 50, type: 'hd_map' }
  ])
  const [connectionStatus, setConnectionStatus] = useState<ConnectionStatus>({
    websocket: 'connected',
    api: 'healthy',
    lastUpdate: new Date(),
    latency: 45
  })
  const [filters, setFilters] = useState({
    status: '',
    operationalStatus: '',
    autonomyLevel: '',
    search: ''
  })
  const [viewMode, setViewMode] = useState<'split' | 'map_only' | 'list_only'>('split')
  const [autoRefresh, setAutoRefresh] = useState(true)
  const [refreshInterval, setRefreshInterval] = useState(5) // seconds
  const [showTeleopDialog, setShowTeleopDialog] = useState(false)
  const [showEmergencyDialog, setShowEmergencyDialog] = useState(false)
  const [showLayersPanel, setShowLayersPanel] = useState(false)
  const [soundEnabled, setSoundEnabled] = useState(true)
  const [showStaleDataWarning, setShowStaleDataWarning] = useState(false)

  // Refs
  const mapRef = useRef<any>(null)
  const refreshTimerRef = useRef<NodeJS.Timeout | null>(null)

  // Real-time updates simulation
  useEffect(() => {
    if (!autoRefresh) return

    refreshTimerRef.current = setInterval(() => {
      setVehicles(prevVehicles => 
        prevVehicles.map(vehicle => {
          // Simulate position updates for online vehicles
          if (vehicle.status === 'online' && vehicle.operationalStatus === 'en_route') {
            const newLat = vehicle.position.lat + (Math.random() - 0.5) * 0.001
            const newLng = vehicle.position.lng + (Math.random() - 0.5) * 0.001
            const newSpeed = Math.max(0, vehicle.position.speed + (Math.random() - 0.5) * 10)
            
            return {
              ...vehicle,
              position: {
                ...vehicle.position,
                lat: newLat,
                lng: newLng,
                speed: newSpeed
              },
              telemetry: {
                ...vehicle.telemetry,
                lastUpdate: new Date(),
                networkLatency: Math.max(20, vehicle.telemetry.networkLatency + (Math.random() - 0.5) * 20)
              }
            }
          }
          return vehicle
        })
      )

      setConnectionStatus(prev => ({
        ...prev,
        lastUpdate: new Date(),
        latency: Math.max(20, prev.latency + (Math.random() - 0.5) * 20)
      }))
    }, refreshInterval * 1000)

    return () => {
      if (refreshTimerRef.current) {
        clearInterval(refreshTimerRef.current)
      }
    }
  }, [autoRefresh, refreshInterval])

  // Check for stale data
  useEffect(() => {
    const checkStaleData = () => {
      const now = new Date()
      const hasStaleData = vehicles.some(vehicle => 
        now.getTime() - vehicle.telemetry.lastUpdate.getTime() > 10000 // 10 seconds
      )
      setShowStaleDataWarning(hasStaleData)
    }

    const staleCheckInterval = setInterval(checkStaleData, 2000)
    return () => clearInterval(staleCheckInterval)
  }, [vehicles])

  // Filtered vehicles
  const filteredVehicles = useMemo(() => {
    return vehicles.filter(vehicle => {
      if (filters.status && vehicle.status !== filters.status) return false
      if (filters.operationalStatus && vehicle.operationalStatus !== filters.operationalStatus) return false
      if (filters.autonomyLevel && vehicle.autonomyLevel !== filters.autonomyLevel) return false
      if (filters.search) {
        const searchTerm = filters.search.toLowerCase()
        return (
          vehicle.name.toLowerCase().includes(searchTerm) ||
          vehicle.plate.toLowerCase().includes(searchTerm) ||
          vehicle.id.toLowerCase().includes(searchTerm)
        )
      }
      return true
    })
  }, [vehicles, filters])

  // Handle vehicle selection
  const handleVehicleClick = useCallback((vehicleId: string) => {
    const vehicle = vehicles.find(v => v.id === vehicleId)
    if (vehicle) {
      setSelectedVehicle(vehicle)
      setShowVehicleDetail(true)
    }
  }, [vehicles])

  // Handle bulk actions
  const handleBulkAction = useCallback((action: string) => {
    console.info(`Control Center: ${action} executed on ${selectedVehicles.length} vehicles`, {
      action,
      vehicleIds: selectedVehicles,
      timestamp: new Date().toISOString(),
      operator: 'current_user'
    })

    // Implement bulk actions based on the action type
    switch (action) {
      case 'emergency_stop':
        setShowEmergencyDialog(true)
        break
      case 'pause':
        // Implement pause logic
        break
      case 'resume':
        // Implement resume logic
        break
      case 'assign_trip':
        // Open trip assignment dialog
        break
      case 'remote_assist':
        setShowTeleopDialog(true)
        break
    }
  }, [selectedVehicles])

  // Handle layer toggle
  const toggleLayer = useCallback((layerId: string) => {
    setMapLayers(prev => 
      prev.map(layer => 
        layer.id === layerId 
          ? { ...layer, visible: !layer.visible }
          : layer
      )
    )
  }, [])

  // Handle layer opacity change
  const updateLayerOpacity = useCallback((layerId: string, opacity: number) => {
    setMapLayers(prev => 
      prev.map(layer => 
        layer.id === layerId 
          ? { ...layer, opacity }
          : layer
      )
    )
  }, [])

  // Get status color
  const getStatusColor = (status: string) => {
    switch (status) {
      case 'online': return 'text-green-600 bg-green-100'
      case 'offline': return 'text-gray-600 bg-gray-100'
      case 'maintenance': return 'text-yellow-600 bg-yellow-100'
      case 'emergency': return 'text-red-600 bg-red-100'
      case 'available': return 'text-blue-600 bg-blue-100'
      case 'en_route': return 'text-purple-600 bg-purple-100'
      case 'assigned': return 'text-orange-600 bg-orange-100'
      default: return 'text-gray-600 bg-gray-100'
    }
  }

  // Get connectivity icon
  const getConnectivityIcon = (connectivity: string) => {
    switch (connectivity) {
      case 'excellent': return <Wifi className="w-4 h-4 text-green-600" />
      case 'good': return <Wifi className="w-4 h-4 text-blue-600" />
      case 'fair': return <Wifi className="w-4 h-4 text-yellow-600" />
      case 'poor': return <Wifi className="w-4 h-4 text-orange-600" />
      case 'disconnected': return <WifiOff className="w-4 h-4 text-red-600" />
      default: return <WifiOff className="w-4 h-4 text-gray-600" />
    }
  }

  return (
    <div className="flex flex-col h-screen bg-gray-50">
      {/* Header */}
      <div className="bg-white border-b border-gray-200 px-6 py-4">
        <div className="flex items-center justify-between">
          <div className="flex items-center space-x-4">
            <Activity className="w-6 h-6 text-blue-600" />
            <h1 className="text-2xl font-semibold text-gray-900">Control Center</h1>
            
            {/* Connection Status */}
            <div className="flex items-center space-x-2">
              <div className={`w-2 h-2 rounded-full ${
                connectionStatus.websocket === 'connected' ? 'bg-green-500' : 
                connectionStatus.websocket === 'reconnecting' ? 'bg-yellow-500 animate-pulse' : 'bg-red-500'
              }`} />
              <span className="text-sm text-gray-600">
                WS: {connectionStatus.websocket} | API: {connectionStatus.api} | {connectionStatus.latency}ms
              </span>
              
              {showStaleDataWarning && (
                <Badge variant="warning" className="ml-2">
                  <AlertTriangle className="w-3 h-3 mr-1" />
                  Stale Data
                </Badge>
              )}
            </div>
          </div>

          <div className="flex items-center space-x-3">
            {/* Auto Refresh Controls */}
            <div className="flex items-center space-x-2">
              <Switch
                checked={autoRefresh}
                onCheckedChange={setAutoRefresh}
              />
              <span className="text-sm text-gray-600">Auto Refresh</span>
              <Select
                value={refreshInterval.toString()}
                onValueChange={(value) => setRefreshInterval(parseInt(value))}
                disabled={!autoRefresh}
              >
                <option value="1">1s</option>
                <option value="5">5s</option>
                <option value="10">10s</option>
                <option value="30">30s</option>
              </Select>
            </div>

            {/* Sound Toggle */}
            <Button
              variant="outline"
              size="sm"
              onClick={() => setSoundEnabled(!soundEnabled)}
            >
              {soundEnabled ? <Volume2 className="w-4 h-4" /> : <VolumeX className="w-4 h-4" />}
            </Button>

            {/* View Mode */}
            <div className="flex border border-gray-300 rounded-md">
              <Button
                variant={viewMode === 'map_only' ? 'default' : 'ghost'}
                size="sm"
                onClick={() => setViewMode('map_only')}
                className="rounded-r-none"
              >
                <MapPin className="w-4 h-4" />
              </Button>
              <Button
                variant={viewMode === 'split' ? 'default' : 'ghost'}
                size="sm"
                onClick={() => setViewMode('split')}
                className="rounded-none border-x"
              >
                <Eye className="w-4 h-4" />
              </Button>
              <Button
                variant={viewMode === 'list_only' ? 'default' : 'ghost'}
                size="sm"
                onClick={() => setViewMode('list_only')}
                className="rounded-l-none"
              >
                <Users className="w-4 h-4" />
              </Button>
            </div>

            {/* Layers */}
            <Button
              variant="outline"
              onClick={() => setShowLayersPanel(!showLayersPanel)}
            >
              <Layers className="w-4 h-4 mr-2" />
              Layers
            </Button>
          </div>
        </div>

        {/* Filters */}
        <div className="flex items-center space-x-4 mt-4">
          <Filter className="w-4 h-4 text-gray-500" />
          
          <div className="flex-1 max-w-md">
            <div className="relative">
              <Search className="w-4 h-4 absolute left-3 top-1/2 transform -translate-y-1/2 text-gray-400" />
              <Input
                placeholder="Search vehicles..."
                value={filters.search}
                onChange={(e) => setFilters({ ...filters, search: e.target.value })}
                className="pl-10"
              />
            </div>
          </div>

          <Select
            value={filters.status}
            onValueChange={(value) => setFilters({ ...filters, status: value })}
          >
            <option value="">All Status</option>
            <option value="online">Online</option>
            <option value="offline">Offline</option>
            <option value="maintenance">Maintenance</option>
            <option value="emergency">Emergency</option>
          </Select>

          <Select
            value={filters.operationalStatus}
            onValueChange={(value) => setFilters({ ...filters, operationalStatus: value })}
          >
            <option value="">All Operations</option>
            <option value="available">Available</option>
            <option value="assigned">Assigned</option>
            <option value="en_route">En Route</option>
            <option value="loading">Loading</option>
            <option value="paused">Paused</option>
          </Select>

          <Select
            value={filters.autonomyLevel}
            onValueChange={(value) => setFilters({ ...filters, autonomyLevel: value })}
          >
            <option value="">All Levels</option>
            <option value="L0">L0 - Manual</option>
            <option value="L1">L1 - Assisted</option>
            <option value="L2">L2 - Partial</option>
            <option value="L3">L3 - Conditional</option>
            <option value="L4">L4 - High</option>
            <option value="L5">L5 - Full</option>
          </Select>

          {selectedVehicles.length > 0 && (
            <div className="flex items-center space-x-2 ml-4 pl-4 border-l border-gray-300">
              <span className="text-sm text-gray-600">{selectedVehicles.length} selected</span>
              <Button size="sm" onClick={() => handleBulkAction('emergency_stop')} variant="destructive">
                <Square className="w-4 h-4 mr-1" />
                Emergency Stop
              </Button>
              <Button size="sm" onClick={() => handleBulkAction('pause')} variant="outline">
                <Pause className="w-4 h-4 mr-1" />
                Pause
              </Button>
              <Button size="sm" onClick={() => handleBulkAction('resume')} variant="outline">
                <Play className="w-4 h-4 mr-1" />
                Resume
              </Button>
              <Button size="sm" onClick={() => handleBulkAction('remote_assist')} variant="outline">
                <Radio className="w-4 h-4 mr-1" />
                Assist
              </Button>
            </div>
          )}
        </div>
      </div>

      {/* Main Content */}
      <div className="flex flex-1 overflow-hidden">
        {/* Map Section */}
        {(viewMode === 'map_only' || viewMode === 'split') && (
          <div className={`${viewMode === 'split' ? 'flex-1' : 'w-full'} relative`}>
            {/* Map Component */}
            <div className="h-full bg-gray-100 flex items-center justify-center">
              <div className="text-center">
                <MapPin className="w-16 h-16 text-gray-400 mx-auto mb-4" />
                <h3 className="text-lg font-medium text-gray-900 mb-2">Live Fleet Map</h3>
                <p className="text-gray-600">Real-time vehicle tracking and monitoring</p>
                <div className="mt-4 space-y-2">
                  <div className="text-sm text-gray-500">
                    Active Vehicles: {vehicles.filter(v => v.status === 'online').length}
                  </div>
                  <div className="text-sm text-gray-500">
                    In Transit: {vehicles.filter(v => v.operationalStatus === 'en_route').length}
                  </div>
                </div>
              </div>
            </div>

            {/* Layers Panel */}
            {showLayersPanel && (
              <div className="absolute top-4 left-4 w-80 bg-white rounded-lg shadow-lg border border-gray-200 p-4 z-10">
                <div className="flex items-center justify-between mb-4">
                  <h3 className="text-lg font-medium text-gray-900">Map Layers</h3>
                  <Button
                    variant="ghost"
                    size="sm"
                    onClick={() => setShowLayersPanel(false)}
                  >
                    <EyeOff className="w-4 h-4" />
                  </Button>
                </div>

                <div className="space-y-4">
                  {mapLayers.map(layer => (
                    <div key={layer.id} className="space-y-2">
                      <div className="flex items-center justify-between">
                        <label className="flex items-center space-x-2">
                          <Switch
                            checked={layer.visible}
                            onCheckedChange={() => toggleLayer(layer.id)}
                          />
                          <span className="text-sm font-medium text-gray-700">{layer.name}</span>
                        </label>
                        <span className="text-xs text-gray-500">{layer.opacity}%</span>
                      </div>
                      {layer.visible && (
                        <Slider
                          value={[layer.opacity]}
                          onValueChange={([value]) => updateLayerOpacity(layer.id, value)}
                          max={100}
                          step={10}
                          className="w-full"
                        />
                      )}
                    </div>
                  ))}
                </div>
              </div>
            )}

            {/* Map Controls */}
            <div className="absolute bottom-4 right-4 flex flex-col space-y-2">
              <Button variant="outline" className="bg-white">
                <Target className="w-4 h-4" />
              </Button>
              <Button variant="outline" className="bg-white">
                <Compass className="w-4 h-4" />
              </Button>
              <Button variant="outline" className="bg-white">
                <Route className="w-4 h-4" />
              </Button>
            </div>
          </div>
        )}

        {/* Vehicle List Section */}
        {(viewMode === 'list_only' || viewMode === 'split') && (
          <div className={`${viewMode === 'split' ? 'w-1/3' : 'w-full'} border-l border-gray-200 bg-white flex flex-col`}>
            <div className="p-4 border-b border-gray-200">
              <div className="flex items-center justify-between">
                <h2 className="text-lg font-medium text-gray-900">Live Vehicles</h2>
                <Badge variant="secondary">
                  {filteredVehicles.length} of {vehicles.length}
                </Badge>
              </div>
            </div>

            <ScrollArea className="flex-1">
              <div className="p-4 space-y-3">
                {filteredVehicles.map(vehicle => {
                  const isStale = new Date().getTime() - vehicle.telemetry.lastUpdate.getTime() > 10000
                  const criticalAlerts = vehicle.alerts.filter(a => a.severity === 'critical' || a.severity === 'emergency')

                  return (
                    <Card
                      key={vehicle.id}
                      className={`cursor-pointer transition-all hover:shadow-md ${
                        selectedVehicles.includes(vehicle.id) ? 'ring-2 ring-blue-500' : ''
                      } ${isStale ? 'border-yellow-300 bg-yellow-50' : ''}`}
                      onClick={() => {
                        if (selectedVehicles.includes(vehicle.id)) {
                          setSelectedVehicles(prev => prev.filter(id => id !== vehicle.id))
                        } else {
                          setSelectedVehicles(prev => [...prev, vehicle.id])
                        }
                      }}
                    >
                      <div className="p-4">
                        <div className="flex items-center justify-between mb-2">
                          <div className="flex items-center space-x-3">
                            <div>
                              <div className="font-medium text-gray-900">{vehicle.name}</div>
                              <div className="text-sm text-gray-500">{vehicle.plate} • {vehicle.model}</div>
                            </div>
                          </div>

                          <div className="flex items-center space-x-2">
                            <Badge className={getStatusColor(vehicle.status)}>
                              {vehicle.status}
                            </Badge>
                            <Button
                              variant="ghost"
                              size="sm"
                              onClick={(e) => {
                                e.stopPropagation()
                                handleVehicleClick(vehicle.id)
                              }}
                            >
                              <Eye className="w-4 h-4" />
                            </Button>
                          </div>
                        </div>

                        <div className="grid grid-cols-2 gap-4 text-sm">
                          <div className="space-y-1">
                            <div className="flex items-center space-x-2">
                              <Badge className={getStatusColor(vehicle.operationalStatus)} size="sm">
                                {vehicle.operationalStatus}
                              </Badge>
                              <Badge variant="outline" size="sm">
                                {vehicle.autonomyLevel}
                              </Badge>
                            </div>
                            
                            <div className="flex items-center space-x-2 text-gray-600">
                              <Battery className="w-4 h-4" />
                              <span>{vehicle.telemetry.battery}%</span>
                              <Progress value={vehicle.telemetry.battery} className="w-16 h-2" />
                            </div>

                            <div className="flex items-center space-x-2 text-gray-600">
                              {getConnectivityIcon(vehicle.telemetry.connectivity)}
                              <span className="capitalize">{vehicle.telemetry.connectivity}</span>
                              {vehicle.telemetry.networkLatency > 0 && (
                                <span className="text-xs">({vehicle.telemetry.networkLatency}ms)</span>
                              )}
                            </div>
                          </div>

                          <div className="space-y-1">
                            <div className="flex items-center space-x-2 text-gray-600">
                              <Gauge className="w-4 h-4" />
                              <span>{vehicle.position.speed} km/h</span>
                            </div>

                            <div className="flex items-center space-x-2 text-gray-600">
                              <Thermometer className="w-4 h-4" />
                              <span>{vehicle.telemetry.temperature}°C</span>
                            </div>

                            <div className="flex items-center space-x-2 text-gray-600">
                              <Clock className="w-4 h-4" />
                              <span className={isStale ? 'text-yellow-600 font-medium' : ''}>
                                {Math.round((new Date().getTime() - vehicle.telemetry.lastUpdate.getTime()) / 1000)}s ago
                              </span>
                            </div>
                          </div>
                        </div>

                        {/* Trip Info */}
                        {vehicle.trip && (
                          <div className="mt-3 p-2 bg-blue-50 rounded border border-blue-200">
                            <div className="text-sm">
                              <div className="font-medium text-blue-900">Trip {vehicle.trip.id}</div>
                              <div className="text-blue-700">→ {vehicle.trip.destination}</div>
                              <div className="flex items-center justify-between mt-1">
                                <Progress value={vehicle.trip.progress} className="w-20 h-2" />
                                <span className="text-xs text-blue-600">
                                  ETA: {vehicle.trip.eta.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })}
                                </span>
                              </div>
                            </div>
                          </div>
                        )}

                        {/* Alerts */}
                        {vehicle.alerts.length > 0 && (
                          <div className="mt-3 space-y-1">
                            {vehicle.alerts.slice(0, 2).map(alert => (
                              <div
                                key={alert.id}
                                className={`flex items-center space-x-2 text-xs p-2 rounded ${
                                  alert.severity === 'critical' || alert.severity === 'emergency'
                                    ? 'bg-red-50 text-red-700 border border-red-200'
                                    : 'bg-yellow-50 text-yellow-700 border border-yellow-200'
                                }`}
                              >
                                <AlertTriangle className="w-3 h-3 flex-shrink-0" />
                                <span className="flex-1">{alert.message}</span>
                                {!alert.acknowledged && (
                                  <div className="w-2 h-2 bg-red-500 rounded-full animate-pulse" />
                                )}
                              </div>
                            ))}
                            {vehicle.alerts.length > 2 && (
                              <div className="text-xs text-gray-500 text-center">
                                +{vehicle.alerts.length - 2} more alerts
                              </div>
                            )}
                          </div>
                        )}

                        {/* Driver Info */}
                        {vehicle.driver && (
                          <div className="mt-3 flex items-center space-x-2 text-sm text-gray-600">
                            <Users className="w-4 h-4" />
                            <span>{vehicle.driver.name}</span>
                            <Badge 
                              size="sm" 
                              className={vehicle.driver.status === 'active' ? 'bg-green-100 text-green-700' : 'bg-gray-100 text-gray-700'}
                            >
                              {vehicle.driver.status}
                            </Badge>
                          </div>
                        )}
                      </div>
                    </Card>
                  )
                })}

                {filteredVehicles.length === 0 && (
                  <div className="text-center py-12">
                    <Users className="w-16 h-16 text-gray-400 mx-auto mb-4" />
                    <h3 className="text-lg font-medium text-gray-900 mb-2">No Vehicles Found</h3>
                    <p className="text-gray-600">Try adjusting your filters</p>
                  </div>
                )}
              </div>
            </ScrollArea>
          </div>
        )}
      </div>

      {/* Vehicle Detail Dialog */}
      <Dialog open={showVehicleDetail} onOpenChange={setShowVehicleDetail}>
        <DialogContent className="max-w-4xl max-h-[90vh] overflow-y-auto">
          <DialogHeader>
            <DialogTitle>
              Vehicle Detail: {selectedVehicle?.name}
            </DialogTitle>
          </DialogHeader>

          {selectedVehicle && (
            <div className="space-y-6">
              {/* Overview */}
              <div className="grid grid-cols-3 gap-6">
                <Card className="p-4">
                  <div className="text-center">
                    <div className="text-2xl font-bold text-gray-900">{selectedVehicle.telemetry.battery}%</div>
                    <div className="text-sm text-gray-600">Battery</div>
                    <Progress value={selectedVehicle.telemetry.battery} className="mt-2" />
                  </div>
                </Card>

                <Card className="p-4">
                  <div className="text-center">
                    <div className="text-2xl font-bold text-gray-900">{selectedVehicle.position.speed}</div>
                    <div className="text-sm text-gray-600">km/h</div>
                    <div className="text-xs text-gray-500 mt-1">Current Speed</div>
                  </div>
                </Card>

                <Card className="p-4">
                  <div className="text-center">
                    <div className="text-2xl font-bold text-gray-900">{selectedVehicle.telemetry.temperature}°C</div>
                    <div className="text-sm text-gray-600">Temperature</div>
                    <div className="text-xs text-gray-500 mt-1">Engine Temp</div>
                  </div>
                </Card>
              </div>

              {/* Detailed Info Tabs */}
              <Tabs defaultValue="overview">
                <TabsList>
                  <TabsTrigger value="overview">Overview</TabsTrigger>
                  <TabsTrigger value="telemetry">Telemetry</TabsTrigger>
                  <TabsTrigger value="alerts">Alerts</TabsTrigger>
                  <TabsTrigger value="controls">Controls</TabsTrigger>
                </TabsList>

                <TabsContent value="overview" className="space-y-4">
                  <div className="grid grid-cols-2 gap-6">
                    <div>
                      <h3 className="text-lg font-medium text-gray-900 mb-3">Vehicle Information</h3>
                      <div className="space-y-2 text-sm">
                        <div className="flex justify-between">
                          <span className="text-gray-600">VIN:</span>
                          <span className="font-medium">{selectedVehicle.id}</span>
                        </div>
                        <div className="flex justify-between">
                          <span className="text-gray-600">Plate:</span>
                          <span className="font-medium">{selectedVehicle.plate}</span>
                        </div>
                        <div className="flex justify-between">
                          <span className="text-gray-600">Model:</span>
                          <span className="font-medium">{selectedVehicle.model}</span>
                        </div>
                        <div className="flex justify-between">
                          <span className="text-gray-600">Autonomy Level:</span>
                          <Badge>{selectedVehicle.autonomyLevel}</Badge>
                        </div>
                        <div className="flex justify-between">
                          <span className="text-gray-600">Status:</span>
                          <Badge className={getStatusColor(selectedVehicle.status)}>
                            {selectedVehicle.status}
                          </Badge>
                        </div>
                      </div>
                    </div>

                    <div>
                      <h3 className="text-lg font-medium text-gray-900 mb-3">Current Position</h3>
                      <div className="space-y-2 text-sm">
                        <div className="flex justify-between">
                          <span className="text-gray-600">Latitude:</span>
                          <span className="font-medium">{selectedVehicle.position.lat.toFixed(6)}</span>
                        </div>
                        <div className="flex justify-between">
                          <span className="text-gray-600">Longitude:</span>
                          <span className="font-medium">{selectedVehicle.position.lng.toFixed(6)}</span>
                        </div>
                        <div className="flex justify-between">
                          <span className="text-gray-600">Heading:</span>
                          <span className="font-medium">{selectedVehicle.position.heading}°</span>
                        </div>
                        <div className="flex justify-between">
                          <span className="text-gray-600">Speed:</span>
                          <span className="font-medium">{selectedVehicle.position.speed} km/h</span>
                        </div>
                      </div>
                    </div>
                  </div>
                </TabsContent>

                <TabsContent value="telemetry" className="space-y-4">
                  <div className="grid grid-cols-2 gap-6">
                    <Card className="p-4">
                      <h3 className="text-lg font-medium text-gray-900 mb-3">Connectivity</h3>
                      <div className="space-y-3">
                        <div className="flex items-center justify-between">
                          <span className="text-gray-600">Status:</span>
                          <div className="flex items-center space-x-2">
                            {getConnectivityIcon(selectedVehicle.telemetry.connectivity)}
                            <span className="capitalize">{selectedVehicle.telemetry.connectivity}</span>
                          </div>
                        </div>
                        <div className="flex items-center justify-between">
                          <span className="text-gray-600">Latency:</span>
                          <span>{selectedVehicle.telemetry.networkLatency}ms</span>
                        </div>
                        <div className="flex items-center justify-between">
                          <span className="text-gray-600">Signal:</span>
                          <span>{selectedVehicle.telemetry.signalStrength}dBm</span>
                        </div>
                        <div className="flex items-center justify-between">
                          <span className="text-gray-600">Last Update:</span>
                          <span>{selectedVehicle.telemetry.lastUpdate.toLocaleTimeString()}</span>
                        </div>
                      </div>
                    </Card>

                    <Card className="p-4">
                      <h3 className="text-lg font-medium text-gray-900 mb-3">System Health</h3>
                      <div className="space-y-3">
                        <div className="flex items-center justify-between">
                          <span className="text-gray-600">Battery:</span>
                          <div className="flex items-center space-x-2">
                            <Progress value={selectedVehicle.telemetry.battery} className="w-20 h-2" />
                            <span>{selectedVehicle.telemetry.battery}%</span>
                          </div>
                        </div>
                        <div className="flex items-center justify-between">
                          <span className="text-gray-600">Temperature:</span>
                          <span className={selectedVehicle.telemetry.temperature > 45 ? 'text-red-600 font-medium' : ''}>
                            {selectedVehicle.telemetry.temperature}°C
                          </span>
                        </div>
                      </div>
                    </Card>
                  </div>
                </TabsContent>

                <TabsContent value="alerts" className="space-y-4">
                  {selectedVehicle.alerts.length > 0 ? (
                    <div className="space-y-3">
                      {selectedVehicle.alerts.map(alert => (
                        <Card key={alert.id} className="p-4">
                          <div className="flex items-start justify-between">
                            <div className="flex items-start space-x-3">
                              <div className={`mt-1 ${
                                alert.severity === 'critical' || alert.severity === 'emergency' 
                                  ? 'text-red-500' 
                                  : alert.severity === 'warning' 
                                    ? 'text-yellow-500' 
                                    : 'text-blue-500'
                              }`}>
                                <AlertTriangle className="w-5 h-5" />
                              </div>
                              <div>
                                <div className="font-medium text-gray-900">{alert.message}</div>
                                <div className="text-sm text-gray-600">
                                  {alert.type} • {alert.timestamp.toLocaleString()}
                                </div>
                              </div>
                            </div>
                            <div className="flex items-center space-x-2">
                              <Badge 
                                variant={
                                  alert.severity === 'critical' || alert.severity === 'emergency' 
                                    ? 'destructive' 
                                    : alert.severity === 'warning' 
                                      ? 'warning' 
                                      : 'secondary'
                                }
                              >
                                {alert.severity}
                              </Badge>
                              {!alert.acknowledged && (
                                <Button size="sm" variant="outline">
                                  Acknowledge
                                </Button>
                              )}
                            </div>
                          </div>
                        </Card>
                      ))}
                    </div>
                  ) : (
                    <div className="text-center py-12">
                      <CheckCircle className="w-16 h-16 text-green-500 mx-auto mb-4" />
                      <h3 className="text-lg font-medium text-gray-900 mb-2">No Active Alerts</h3>
                      <p className="text-gray-600">Vehicle is operating normally</p>
                    </div>
                  )}
                </TabsContent>

                <TabsContent value="controls" className="space-y-4">
                  <div className="grid grid-cols-2 gap-6">
                    <Card className="p-4">
                      <h3 className="text-lg font-medium text-gray-900 mb-3">Vehicle Controls</h3>
                      <div className="space-y-3">
                        <Button 
                          className="w-full justify-start" 
                          disabled={!selectedVehicle.capabilities.emergencyStop}
                          variant="destructive"
                        >
                          <Square className="w-4 h-4 mr-2" />
                          Emergency Stop
                        </Button>
                        <Button 
                          className="w-full justify-start" 
                          variant="outline"
                        >
                          <Pause className="w-4 h-4 mr-2" />
                          Pause Vehicle
                        </Button>
                        <Button 
                          className="w-full justify-start" 
                          variant="outline"
                        >
                          <Play className="w-4 h-4 mr-2" />
                          Resume Vehicle
                        </Button>
                        <Button 
                          className="w-full justify-start" 
                          variant="outline"
                          disabled={!selectedVehicle.capabilities.remoteAssist}
                        >
                          <Radio className="w-4 h-4 mr-2" />
                          Remote Assist
                        </Button>
                      </div>
                    </Card>

                    <Card className="p-4">
                      <h3 className="text-lg font-medium text-gray-900 mb-3">Diagnostics</h3>
                      <div className="space-y-3">
                        <Button 
                          className="w-full justify-start" 
                          variant="outline"
                          disabled={!selectedVehicle.capabilities.diagnostics}
                        >
                          <Activity className="w-4 h-4 mr-2" />
                          Run Diagnostics
                        </Button>
                        <Button 
                          className="w-full justify-start" 
                          variant="outline"
                        >
                          <Camera className="w-4 h-4 mr-2" />
                          View Cameras
                        </Button>
                        <Button 
                          className="w-full justify-start" 
                          variant="outline"
                          disabled={!selectedVehicle.capabilities.teleoperation}
                        >
                          <Navigation className="w-4 h-4 mr-2" />
                          Teleoperation
                        </Button>
                        <Button 
                          className="w-full justify-start" 
                          variant="outline"
                        >
                          <RefreshCw className="w-4 h-4 mr-2" />
                          Restart Systems
                        </Button>
                      </div>
                    </Card>
                  </div>
                </TabsContent>
              </Tabs>
            </div>
          )}
        </DialogContent>
      </Dialog>

      {/* Emergency Dialog */}
      <Dialog open={showEmergencyDialog} onOpenChange={setShowEmergencyDialog}>
        <DialogContent>
          <DialogHeader>
            <DialogTitle className="text-red-600">Emergency Stop Confirmation</DialogTitle>
          </DialogHeader>
          <div className="space-y-4">
            <Alert>
              <AlertTriangle className="w-4 h-4" />
              <AlertDescription>
                You are about to execute an emergency stop on {selectedVehicles.length} vehicle(s). 
                This action cannot be undone and will immediately halt all vehicle operations.
              </AlertDescription>
            </Alert>
            <div className="flex justify-end space-x-3">
              <Button variant="outline" onClick={() => setShowEmergencyDialog(false)}>
                Cancel
              </Button>
              <Button variant="destructive" onClick={() => {
                // Implement emergency stop
                setShowEmergencyDialog(false)
                setSelectedVehicles([])
              }}>
                Confirm Emergency Stop
              </Button>
            </div>
          </div>
        </DialogContent>
      </Dialog>

      {/* Teleoperation Dialog */}
      <Dialog open={showTeleopDialog} onOpenChange={setShowTeleopDialog}>
        <DialogContent className="max-w-6xl">
          <DialogHeader>
            <DialogTitle>Remote Teleoperation</DialogTitle>
          </DialogHeader>
          <div className="space-y-4">
            <div className="text-center py-12">
              <Radio className="w-16 h-16 text-blue-500 mx-auto mb-4" />
              <h3 className="text-lg font-medium text-gray-900 mb-2">Teleoperation Console</h3>
              <p className="text-gray-600">Remote vehicle control interface</p>
            </div>
          </div>
        </DialogContent>
      </Dialog>
    </div>
  )
}

export default ControlCenter
