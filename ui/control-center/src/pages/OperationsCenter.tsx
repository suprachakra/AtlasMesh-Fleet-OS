import React, { useState, useCallback, useMemo, useEffect } from 'react'
import { 
  Activity, MapPin, AlertTriangle, Users, Clock, Battery, Wifi, Signal,
  Play, Pause, Square, Radio, Eye, Filter, Search, RefreshCw, Layers,
  TrendingUp, TrendingDown, Zap, Shield, Car, Route, Gauge, Volume2,
  VolumeX, Maximize2, Target, Bell, BellOff, Settings
} from 'lucide-react'
import { Button } from '../components/ui/Button'
import { Badge } from '../components/ui/Badge'
import { Card } from '../components/ui/Card'
import { Input } from '../components/ui/Input'
import { Select } from '../components/ui/Select'
import { Tabs, TabsContent, TabsList, TabsTrigger } from '../components/ui/Tabs'
import { Dialog, DialogContent, DialogHeader, DialogTitle } from '../components/ui/Dialog'
import { Alert, AlertDescription } from '../components/ui/Alert'
import { Progress } from '../components/ui/Progress'
import { ScrollArea } from '../components/ui/ScrollArea'
import { Switch } from '../components/ui/Switch'

// Types
interface Vehicle {
  id: string
  name: string
  plate: string
  status: 'online' | 'offline' | 'maintenance' | 'emergency'
  operationalStatus: 'available' | 'assigned' | 'en_route' | 'loading' | 'paused'
  position: { lat: number; lng: number; heading: number; speed: number }
  health: { overall: number; battery: number; connectivity: string }
  lastUpdate: Date
  alerts: VehicleAlert[]
  currentTrip?: { id: string; destination: string; progress: number; eta: Date }
  driver?: { name: string; status: string }
}

interface VehicleAlert {
  id: string
  severity: 'info' | 'warning' | 'critical' | 'emergency'
  type: string
  message: string
  timestamp: Date
  acknowledged: boolean
  vehicleId: string
  vehicleName: string
}

interface FleetKPI {
  id: string
  name: string
  value: number | string
  unit?: string
  trend: 'up' | 'down' | 'stable'
  trendValue: number
  status: 'good' | 'warning' | 'critical'
  description: string
}

// Mock data
const mockVehicles: Vehicle[] = [
  {
    id: 'v1',
    name: 'Atlas-001',
    plate: 'AV-001',
    status: 'online',
    operationalStatus: 'en_route',
    position: { lat: 25.2048, lng: 55.2708, heading: 45, speed: 35 },
    health: { overall: 95, battery: 85, connectivity: 'excellent' },
    lastUpdate: new Date(),
    alerts: [
      {
        id: 'a1',
        severity: 'warning',
        type: 'temperature',
        message: 'Engine temperature elevated',
        timestamp: new Date(Date.now() - 5 * 60 * 1000),
        acknowledged: false,
        vehicleId: 'v1',
        vehicleName: 'Atlas-001'
      }
    ],
    currentTrip: {
      id: 't1',
      destination: 'Dubai Mall',
      progress: 65,
      eta: new Date(Date.now() + 15 * 60 * 1000)
    },
    driver: { name: 'Ahmed Al-Mansouri', status: 'active' }
  },
  {
    id: 'v2',
    name: 'Atlas-002',
    plate: 'AV-002',
    status: 'online',
    operationalStatus: 'available',
    position: { lat: 25.1972, lng: 55.2744, heading: 180, speed: 0 },
    health: { overall: 92, battery: 88, connectivity: 'good' },
    lastUpdate: new Date(Date.now() - 30 * 1000),
    alerts: [],
    driver: undefined
  },
  {
    id: 'v3',
    name: 'Atlas-003',
    plate: 'AV-003',
    status: 'offline',
    operationalStatus: 'maintenance',
    position: { lat: 25.2103, lng: 55.2640, heading: 270, speed: 0 },
    health: { overall: 65, battery: 15, connectivity: 'disconnected' },
    lastUpdate: new Date(Date.now() - 10 * 60 * 1000),
    alerts: [
      {
        id: 'a2',
        severity: 'critical',
        type: 'battery',
        message: 'Battery level critically low',
        timestamp: new Date(Date.now() - 8 * 60 * 1000),
        acknowledged: false,
        vehicleId: 'v3',
        vehicleName: 'Atlas-003'
      }
    ]
  }
]

const mockKPIs: FleetKPI[] = [
  {
    id: 'active_vehicles',
    name: 'Active Vehicles',
    value: 24,
    unit: 'vehicles',
    trend: 'up',
    trendValue: 8.5,
    status: 'good',
    description: 'Vehicles currently online and operational'
  },
  {
    id: 'utilization',
    name: 'Fleet Utilization',
    value: 78,
    unit: '%',
    trend: 'up',
    trendValue: 5.2,
    status: 'good',
    description: 'Percentage of fleet actively serving trips'
  },
  {
    id: 'avg_trip_time',
    name: 'Avg Trip Time',
    value: '24.5',
    unit: 'min',
    trend: 'down',
    trendValue: 3.1,
    status: 'good',
    description: 'Average time to complete trips'
  },
  {
    id: 'sla_compliance',
    name: 'SLA Compliance',
    value: 94.2,
    unit: '%',
    trend: 'stable',
    trendValue: 0.1,
    status: 'warning',
    description: 'Percentage of trips meeting SLA requirements'
  },
  {
    id: 'safety_score',
    name: 'Safety Score',
    value: 98.7,
    unit: '%',
    trend: 'up',
    trendValue: 1.2,
    status: 'good',
    description: 'Overall fleet safety performance score'
  },
  {
    id: 'maintenance_due',
    name: 'Maintenance Due',
    value: 3,
    unit: 'vehicles',
    trend: 'up',
    trendValue: 2,
    status: 'warning',
    description: 'Vehicles requiring scheduled maintenance'
  }
]

const OperationsCenter: React.FC = () => {
  // State
  const [vehicles, setVehicles] = useState<Vehicle[]>(mockVehicles)
  const [kpis, setKPIs] = useState<FleetKPI[]>(mockKPIs)
  const [selectedVehicles, setSelectedVehicles] = useState<string[]>([])
  const [activeTab, setActiveTab] = useState('vehicles')
  const [mapLayers, setMapLayers] = useState({
    vehicles: true,
    trips: true,
    geofences: true,
    weather: false,
    traffic: false
  })
  const [filters, setFilters] = useState({
    status: '',
    operationalStatus: '',
    search: ''
  })
  const [autoRefresh, setAutoRefresh] = useState(true)
  const [soundEnabled, setSoundEnabled] = useState(true)
  const [showEmergencyDialog, setShowEmergencyDialog] = useState(false)
  const [connectionStatus, setConnectionStatus] = useState({
    websocket: 'connected' as 'connected' | 'disconnected' | 'reconnecting',
    lastUpdate: new Date(),
    latency: 45
  })

  // Real-time simulation
  useEffect(() => {
    if (!autoRefresh) return

    const interval = setInterval(() => {
      setVehicles(prev => prev.map(vehicle => {
        if (vehicle.status === 'online' && vehicle.operationalStatus === 'en_route') {
          return {
            ...vehicle,
            position: {
              ...vehicle.position,
              lat: vehicle.position.lat + (Math.random() - 0.5) * 0.001,
              lng: vehicle.position.lng + (Math.random() - 0.5) * 0.001,
              speed: Math.max(0, vehicle.position.speed + (Math.random() - 0.5) * 10)
            },
            lastUpdate: new Date()
          }
        }
        return vehicle
      }))

      setConnectionStatus(prev => ({
        ...prev,
        lastUpdate: new Date(),
        latency: Math.max(20, prev.latency + (Math.random() - 0.5) * 20)
      }))
    }, 5000)

    return () => clearInterval(interval)
  }, [autoRefresh])

  // Computed values
  const filteredVehicles = useMemo(() => {
    return vehicles.filter(vehicle => {
      if (filters.status && vehicle.status !== filters.status) return false
      if (filters.operationalStatus && vehicle.operationalStatus !== filters.operationalStatus) return false
      if (filters.search) {
        const searchTerm = filters.search.toLowerCase()
        return (
          vehicle.name.toLowerCase().includes(searchTerm) ||
          vehicle.plate.toLowerCase().includes(searchTerm)
        )
      }
      return true
    })
  }, [vehicles, filters])

  const allAlerts = useMemo(() => {
    return vehicles.flatMap(vehicle => vehicle.alerts)
      .sort((a, b) => b.timestamp.getTime() - a.timestamp.getTime())
  }, [vehicles])

  const criticalAlerts = useMemo(() => {
    return allAlerts.filter(alert => 
      alert.severity === 'critical' || alert.severity === 'emergency'
    )
  }, [allAlerts])

  // Handlers
  const handleBulkAction = useCallback((action: string) => {
    console.info(`Operations Center: ${action} executed on ${selectedVehicles.length} vehicles`)
    
    switch (action) {
      case 'emergency_stop':
        setShowEmergencyDialog(true)
        break
      case 'pause':
      case 'resume':
      case 'remote_assist':
        // Implement actions
        break
    }
  }, [selectedVehicles])

  const handleAcknowledgeAlert = useCallback((alertId: string) => {
    setVehicles(prev => prev.map(vehicle => ({
      ...vehicle,
      alerts: vehicle.alerts.map(alert => 
        alert.id === alertId ? { ...alert, acknowledged: true } : alert
      )
    })))
  }, [])

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

  const getKPIStatusColor = (status: string) => {
    switch (status) {
      case 'good': return 'text-green-600'
      case 'warning': return 'text-yellow-600'
      case 'critical': return 'text-red-600'
      default: return 'text-gray-600'
    }
  }

  const getTrendIcon = (trend: string) => {
    switch (trend) {
      case 'up': return <TrendingUp className="w-4 h-4 text-green-600" />
      case 'down': return <TrendingDown className="w-4 h-4 text-red-600" />
      default: return <div className="w-4 h-4" />
    }
  }

  return (
    <div className="flex flex-col h-screen bg-gray-50">
      {/* Header */}
      <div className="bg-white border-b border-gray-200 px-6 py-4">
        <div className="flex items-center justify-between">
          <div className="flex items-center space-x-4">
            <Activity className="w-6 h-6 text-blue-600" />
            <h1 className="text-2xl font-semibold text-gray-900">Operations Command Center</h1>
            
            {/* Connection Status */}
            <div className="flex items-center space-x-2">
              <div className={`w-2 h-2 rounded-full ${
                connectionStatus.websocket === 'connected' ? 'bg-green-500' : 
                connectionStatus.websocket === 'reconnecting' ? 'bg-yellow-500 animate-pulse' : 'bg-red-500'
              }`} />
              <span className="text-sm text-gray-600">
                {connectionStatus.websocket} | {connectionStatus.latency}ms
              </span>
            </div>
          </div>

          <div className="flex items-center space-x-3">
            {/* Auto Refresh */}
            <div className="flex items-center space-x-2">
              <Switch checked={autoRefresh} onCheckedChange={setAutoRefresh} />
              <span className="text-sm text-gray-600">Auto Refresh</span>
            </div>

            {/* Sound Toggle */}
            <Button
              variant="outline"
              size="sm"
              onClick={() => setSoundEnabled(!soundEnabled)}
            >
              {soundEnabled ? <Volume2 className="w-4 h-4" /> : <VolumeX className="w-4 h-4" />}
            </Button>

            {/* Layers */}
            <Button variant="outline" size="sm">
              <Layers className="w-4 h-4 mr-2" />
              Layers
            </Button>

            {/* Full Screen */}
            <Button variant="outline" size="sm">
              <Maximize2 className="w-4 h-4" />
            </Button>
          </div>
        </div>

        {/* Quick Stats */}
        <div className="grid grid-cols-6 gap-4 mt-4">
          <Card className="p-3">
            <div className="text-center">
              <div className="text-2xl font-bold text-green-600">
                {vehicles.filter(v => v.status === 'online').length}
              </div>
              <div className="text-sm text-gray-600">Online</div>
            </div>
          </Card>
          <Card className="p-3">
            <div className="text-center">
              <div className="text-2xl font-bold text-purple-600">
                {vehicles.filter(v => v.operationalStatus === 'en_route').length}
              </div>
              <div className="text-sm text-gray-600">En Route</div>
            </div>
          </Card>
          <Card className="p-3">
            <div className="text-center">
              <div className="text-2xl font-bold text-blue-600">
                {vehicles.filter(v => v.operationalStatus === 'available').length}
              </div>
              <div className="text-sm text-gray-600">Available</div>
            </div>
          </Card>
          <Card className="p-3">
            <div className="text-center">
              <div className={`text-2xl font-bold ${criticalAlerts.length > 0 ? 'text-red-600' : 'text-green-600'}`}>
                {criticalAlerts.length}
              </div>
              <div className="text-sm text-gray-600">Critical Alerts</div>
            </div>
          </Card>
          <Card className="p-3">
            <div className="text-center">
              <div className="text-2xl font-bold text-gray-900">94.2%</div>
              <div className="text-sm text-gray-600">SLA Compliance</div>
            </div>
          </Card>
          <Card className="p-3">
            <div className="text-center">
              <div className="text-2xl font-bold text-gray-900">98.7%</div>
              <div className="text-sm text-gray-600">Safety Score</div>
            </div>
          </Card>
        </div>
      </div>

      {/* Main Content */}
      <div className="flex flex-1 overflow-hidden">
        {/* Left Panel - Live Map */}
        <div className="flex-1 relative">
          <div className="h-full bg-gray-100 flex items-center justify-center">
            <div className="text-center">
              <MapPin className="w-16 h-16 text-gray-400 mx-auto mb-4" />
              <h3 className="text-lg font-medium text-gray-900 mb-2">Live Fleet Map</h3>
              <p className="text-gray-600">Real-time vehicle tracking and monitoring</p>
              
              {/* Map Stats */}
              <div className="mt-6 grid grid-cols-2 gap-4 max-w-md mx-auto">
                <div className="bg-white rounded-lg p-4 shadow">
                  <div className="text-2xl font-bold text-blue-600">{vehicles.length}</div>
                  <div className="text-sm text-gray-600">Total Vehicles</div>
                </div>
                <div className="bg-white rounded-lg p-4 shadow">
                  <div className="text-2xl font-bold text-green-600">
                    {vehicles.filter(v => v.status === 'online').length}
                  </div>
                  <div className="text-sm text-gray-600">Active Now</div>
                </div>
              </div>
            </div>
          </div>

          {/* Map Controls */}
          <div className="absolute bottom-4 right-4 flex flex-col space-y-2">
            <Button variant="outline" className="bg-white">
              <Target className="w-4 h-4" />
            </Button>
            <Button variant="outline" className="bg-white">
              <RefreshCw className="w-4 h-4" />
            </Button>
          </div>
        </div>

        {/* Right Panel - Tabbed Content */}
        <div className="w-1/3 border-l border-gray-200 bg-white flex flex-col">
          <div className="p-4 border-b border-gray-200">
            <Tabs value={activeTab} onValueChange={setActiveTab}>
              <TabsList className="grid grid-cols-3 w-full">
                <TabsTrigger value="vehicles" className="flex items-center space-x-2">
                  <Car className="w-4 h-4" />
                  <span>Vehicles</span>
                  <Badge variant="secondary" className="ml-1">
                    {filteredVehicles.length}
                  </Badge>
                </TabsTrigger>
                <TabsTrigger value="alerts" className="flex items-center space-x-2">
                  <AlertTriangle className="w-4 h-4" />
                  <span>Alerts</span>
                  <Badge variant={criticalAlerts.length > 0 ? 'destructive' : 'secondary'} className="ml-1">
                    {allAlerts.filter(a => !a.acknowledged).length}
                  </Badge>
                </TabsTrigger>
                <TabsTrigger value="kpis" className="flex items-center space-x-2">
                  <TrendingUp className="w-4 h-4" />
                  <span>KPIs</span>
                </TabsTrigger>
              </TabsList>
            </Tabs>
          </div>

          <div className="flex-1 overflow-hidden">
            <Tabs value={activeTab} onValueChange={setActiveTab}>
              {/* Live Vehicles Tab */}
              <TabsContent value="vehicles" className="h-full flex flex-col">
                <div className="p-4 border-b border-gray-200">
                  <div className="flex items-center space-x-2 mb-3">
                    <div className="relative flex-1">
                      <Search className="w-4 h-4 absolute left-3 top-1/2 transform -translate-y-1/2 text-gray-400" />
                      <Input
                        placeholder="Search vehicles..."
                        value={filters.search}
                        onChange={(e) => setFilters({ ...filters, search: e.target.value })}
                        className="pl-10"
                      />
                    </div>
                    <Button variant="outline" size="sm">
                      <Filter className="w-4 h-4" />
                    </Button>
                  </div>

                  <div className="flex space-x-2">
                    <Select
                      value={filters.status}
                      onValueChange={(value) => setFilters({ ...filters, status: value })}
                    >
                      <option value="">All Status</option>
                      <option value="online">Online</option>
                      <option value="offline">Offline</option>
                      <option value="maintenance">Maintenance</option>
                    </Select>

                    <Select
                      value={filters.operationalStatus}
                      onValueChange={(value) => setFilters({ ...filters, operationalStatus: value })}
                    >
                      <option value="">All Operations</option>
                      <option value="available">Available</option>
                      <option value="en_route">En Route</option>
                      <option value="assigned">Assigned</option>
                    </Select>
                  </div>

                  {selectedVehicles.length > 0 && (
                    <div className="mt-3 p-3 bg-blue-50 rounded-lg">
                      <div className="flex items-center justify-between">
                        <span className="text-sm font-medium text-blue-900">
                          {selectedVehicles.length} vehicles selected
                        </span>
                        <div className="flex space-x-1">
                          <Button size="sm" onClick={() => handleBulkAction('emergency_stop')} variant="destructive">
                            <Square className="w-3 h-3 mr-1" />
                            Stop
                          </Button>
                          <Button size="sm" onClick={() => handleBulkAction('pause')} variant="outline">
                            <Pause className="w-3 h-3 mr-1" />
                            Pause
                          </Button>
                          <Button size="sm" onClick={() => handleBulkAction('remote_assist')} variant="outline">
                            <Radio className="w-3 h-3 mr-1" />
                            Assist
                          </Button>
                        </div>
                      </div>
                    </div>
                  )}
                </div>

                <ScrollArea className="flex-1">
                  <div className="p-4 space-y-3">
                    {filteredVehicles.map(vehicle => {
                      const isSelected = selectedVehicles.includes(vehicle.id)
                      const isStale = new Date().getTime() - vehicle.lastUpdate.getTime() > 60000

                      return (
                        <Card
                          key={vehicle.id}
                          className={`cursor-pointer transition-all hover:shadow-md ${
                            isSelected ? 'ring-2 ring-blue-500' : ''
                          } ${isStale ? 'border-yellow-300 bg-yellow-50' : ''}`}
                          onClick={() => {
                            if (isSelected) {
                              setSelectedVehicles(prev => prev.filter(id => id !== vehicle.id))
                            } else {
                              setSelectedVehicles(prev => [...prev, vehicle.id])
                            }
                          }}
                        >
                          <div className="p-4">
                            <div className="flex items-center justify-between mb-2">
                              <div>
                                <div className="font-medium text-gray-900">{vehicle.name}</div>
                                <div className="text-sm text-gray-500">{vehicle.plate}</div>
                              </div>
                              <div className="flex items-center space-x-2">
                                <Badge className={getStatusColor(vehicle.status)}>
                                  {vehicle.status}
                                </Badge>
                                <Button variant="ghost" size="sm">
                                  <Eye className="w-4 h-4" />
                                </Button>
                              </div>
                            </div>

                            <div className="space-y-2">
                              <div className="flex items-center justify-between text-sm">
                                <Badge className={getStatusColor(vehicle.operationalStatus)} size="sm">
                                  {vehicle.operationalStatus}
                                </Badge>
                                <span className="text-gray-600">{vehicle.position.speed} km/h</span>
                              </div>

                              <div className="flex items-center justify-between text-sm">
                                <div className="flex items-center space-x-2">
                                  <Battery className="w-4 h-4 text-gray-500" />
                                  <span>{vehicle.health.battery}%</span>
                                  <Progress value={vehicle.health.battery} className="w-16 h-2" />
                                </div>
                                <div className="flex items-center space-x-1">
                                  <div className={`w-2 h-2 rounded-full ${
                                    vehicle.health.connectivity === 'excellent' ? 'bg-green-500' :
                                    vehicle.health.connectivity === 'good' ? 'bg-blue-500' :
                                    vehicle.health.connectivity === 'fair' ? 'bg-yellow-500' : 'bg-red-500'
                                  }`} />
                                  <span className="text-xs capitalize">{vehicle.health.connectivity}</span>
                                </div>
                              </div>

                              {vehicle.currentTrip && (
                                <div className="mt-2 p-2 bg-blue-50 rounded border border-blue-200">
                                  <div className="text-sm">
                                    <div className="font-medium text-blue-900">→ {vehicle.currentTrip.destination}</div>
                                    <div className="flex items-center justify-between mt-1">
                                      <Progress value={vehicle.currentTrip.progress} className="w-20 h-2" />
                                      <span className="text-xs text-blue-600">
                                        ETA: {vehicle.currentTrip.eta.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })}
                                      </span>
                                    </div>
                                  </div>
                                </div>
                              )}

                              {vehicle.alerts.length > 0 && (
                                <div className="mt-2">
                                  {vehicle.alerts.slice(0, 1).map(alert => (
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
                                </div>
                              )}

                              {vehicle.driver && (
                                <div className="flex items-center space-x-2 text-sm text-gray-600 mt-2">
                                  <Users className="w-4 h-4" />
                                  <span>{vehicle.driver.name}</span>
                                  <Badge size="sm" className="bg-green-100 text-green-700">
                                    {vehicle.driver.status}
                                  </Badge>
                                </div>
                              )}
                            </div>
                          </div>
                        </Card>
                      )
                    })}

                    {filteredVehicles.length === 0 && (
                      <div className="text-center py-12">
                        <Car className="w-16 h-16 text-gray-400 mx-auto mb-4" />
                        <h3 className="text-lg font-medium text-gray-900 mb-2">No Vehicles Found</h3>
                        <p className="text-gray-600">Try adjusting your filters</p>
                      </div>
                    )}
                  </div>
                </ScrollArea>
              </TabsContent>

              {/* Active Alerts Tab */}
              <TabsContent value="alerts" className="h-full flex flex-col">
                <div className="p-4 border-b border-gray-200">
                  <div className="flex items-center justify-between">
                    <h3 className="font-medium text-gray-900">Active Alerts</h3>
                    <div className="flex space-x-2">
                      <Button size="sm" variant="outline">
                        <Bell className="w-4 h-4 mr-1" />
                        Acknowledge All
                      </Button>
                      <Button size="sm" variant="outline">
                        <Settings className="w-4 h-4" />
                      </Button>
                    </div>
                  </div>
                </div>

                <ScrollArea className="flex-1">
                  <div className="p-4 space-y-3">
                    {allAlerts.map(alert => (
                      <Card
                        key={alert.id}
                        className={`${
                          alert.severity === 'critical' || alert.severity === 'emergency'
                            ? 'border-red-200 bg-red-50'
                            : alert.severity === 'warning'
                              ? 'border-yellow-200 bg-yellow-50'
                              : 'border-blue-200 bg-blue-50'
                        } ${alert.acknowledged ? 'opacity-60' : ''}`}
                      >
                        <div className="p-4">
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
                              <div className="flex-1">
                                <div className="font-medium text-gray-900">{alert.message}</div>
                                <div className="text-sm text-gray-600 mt-1">
                                  {alert.vehicleName} • {alert.type} • {alert.timestamp.toLocaleString()}
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
                                <Button 
                                  size="sm" 
                                  variant="outline"
                                  onClick={() => handleAcknowledgeAlert(alert.id)}
                                >
                                  Ack
                                </Button>
                              )}
                            </div>
                          </div>
                        </div>
                      </Card>
                    ))}

                    {allAlerts.length === 0 && (
                      <div className="text-center py-12">
                        <Shield className="w-16 h-16 text-green-500 mx-auto mb-4" />
                        <h3 className="text-lg font-medium text-gray-900 mb-2">All Clear</h3>
                        <p className="text-gray-600">No active alerts at this time</p>
                      </div>
                    )}
                  </div>
                </ScrollArea>
              </TabsContent>

              {/* Fleet KPIs Tab */}
              <TabsContent value="kpis" className="h-full flex flex-col">
                <div className="p-4 border-b border-gray-200">
                  <div className="flex items-center justify-between">
                    <h3 className="font-medium text-gray-900">Fleet KPIs</h3>
                    <Button size="sm" variant="outline">
                      <RefreshCw className="w-4 h-4 mr-1" />
                      Refresh
                    </Button>
                  </div>
                </div>

                <ScrollArea className="flex-1">
                  <div className="p-4 space-y-4">
                    {kpis.map(kpi => (
                      <Card key={kpi.id} className="p-4">
                        <div className="flex items-center justify-between mb-2">
                          <div className="font-medium text-gray-900">{kpi.name}</div>
                          <div className="flex items-center space-x-1">
                            {getTrendIcon(kpi.trend)}
                            <span className={`text-sm ${
                              kpi.trend === 'up' ? 'text-green-600' : 
                              kpi.trend === 'down' ? 'text-red-600' : 'text-gray-600'
                            }`}>
                              {kpi.trendValue > 0 ? '+' : ''}{kpi.trendValue}%
                            </span>
                          </div>
                        </div>
                        
                        <div className="flex items-end justify-between">
                          <div>
                            <div className={`text-2xl font-bold ${getKPIStatusColor(kpi.status)}`}>
                              {kpi.value}
                              {kpi.unit && <span className="text-lg text-gray-600 ml-1">{kpi.unit}</span>}
                            </div>
                            <div className="text-sm text-gray-600 mt-1">{kpi.description}</div>
                          </div>
                          
                          <Badge 
                            variant={
                              kpi.status === 'good' ? 'success' :
                              kpi.status === 'warning' ? 'warning' : 'destructive'
                            }
                          >
                            {kpi.status}
                          </Badge>
                        </div>
                      </Card>
                    ))}
                  </div>
                </ScrollArea>
              </TabsContent>
            </Tabs>
          </div>
        </div>
      </div>

      {/* Emergency Stop Dialog */}
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
                setShowEmergencyDialog(false)
                setSelectedVehicles([])
              }}>
                Confirm Emergency Stop
              </Button>
            </div>
          </div>
        </DialogContent>
      </Dialog>
    </div>
  )
}

export default OperationsCenter
