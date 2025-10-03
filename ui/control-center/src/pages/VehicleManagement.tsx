import React, { useState, useMemo, useCallback } from 'react'
import { 
  Car, Search, Filter, Plus, Settings, Download, Upload, Eye, Edit, Trash2, 
  AlertTriangle, CheckCircle, Clock, Battery, Wifi, MapPin, User, Wrench, 
  FileText, Calendar, Activity, Shield, Zap, RefreshCw, RotateCcw, Pin, 
  Smartphone, Signal, HardDrive, Cpu, Thermometer, Gauge, Users, Tags,
  ExternalLink, Copy, Archive, Star, StarOff
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
import { Progress } from '../components/ui/Progress'
import { ScrollArea } from '../components/ui/ScrollArea'
import { DataTable } from '../components/ui/DataTable'
import { Switch } from '../components/ui/Switch'

// Types
interface Vehicle {
  id: string
  vin: string
  name: string
  plate: string
  make: string
  model: string
  year: number
  depot: string
  status: 'unknown' | 'normal' | 'down' | 'deprecated'
  operationalStatus: 'available' | 'in_service' | 'maintenance' | 'offline'
  autonomyLevel: 'L0' | 'L1' | 'L2' | 'L3' | 'L4' | 'L5'
  health: {
    overall: number
    battery: number
    engine: number
    sensors: number
    communication: number
  }
  connectivity: {
    status: 'excellent' | 'good' | 'fair' | 'poor' | 'disconnected'
    lastSeen: Date
    networkLatency: number
    signalStrength: number
  }
  location: {
    current: string
    latitude?: number
    longitude?: number
    lastUpdate: Date
  }
  versions: {
    binary: string
    roadGraph: string
    map: string
    lsm: string
    policy: string
  }
  blacklist: string[]
  odometer: number
  lastService: Date
  nextService: Date
  permits: {
    registration: {
      number: string
      expires: Date
      status: 'valid' | 'expiring' | 'expired'
    }
    insurance: {
      number: string
      expires: Date
      status: 'valid' | 'expiring' | 'expired'
    }
    inspection: {
      number: string
      expires: Date
      status: 'valid' | 'expiring' | 'expired'
    }
  }
  sim: {
    iccid: string
    status: 'active' | 'inactive' | 'suspended'
    plan: string
    dataUsage: number
    lastHeartbeat: Date
  }
  assignments: {
    currentTrip?: string
    driver?: {
      id: string
      name: string
    }
    schedule: string[]
  }
  groups: string[]
  tags: string[]
  starred: boolean
}

interface WorkLog {
  id: string
  vehicleId: string
  type: 'maintenance' | 'repair' | 'inspection' | 'upgrade' | 'incident'
  title: string
  description: string
  technician: string
  startTime: Date
  endTime?: Date
  status: 'scheduled' | 'in_progress' | 'completed' | 'cancelled'
  priority: 'low' | 'medium' | 'high' | 'critical'
  parts: Array<{
    partNumber: string
    description: string
    quantity: number
    cost: number
  }>
  attachments: Array<{
    name: string
    url: string
    type: string
  }>
  cost: number
}

interface VehicleGroup {
  id: string
  name: string
  description: string
  vehicleIds: string[]
  color: string
  created: Date
}

// Mock data
const mockVehicles: Vehicle[] = [
  {
    id: 'v1',
    vin: 'WVW1K7AJ5DW123456',
    name: 'Atlas-001',
    plate: 'AV-001',
    make: 'AtlasMesh',
    model: 'L4 Sedan',
    year: 2024,
    depot: 'Dubai Main',
    status: 'normal',
    operationalStatus: 'available',
    autonomyLevel: 'L4',
    health: {
      overall: 95,
      battery: 92,
      engine: 98,
      sensors: 94,
      communication: 96
    },
    connectivity: {
      status: 'excellent',
      lastSeen: new Date(),
      networkLatency: 45,
      signalStrength: -65
    },
    location: {
      current: 'Dubai Main Depot',
      latitude: 25.2048,
      longitude: 55.2708,
      lastUpdate: new Date()
    },
    versions: {
      binary: 'v2.1.0',
      roadGraph: 'v2.0.5',
      map: 'v1.9.8',
      lsm: 'v1.5.2',
      policy: 'v1.2.1'
    },
    blacklist: [],
    odometer: 45678,
    lastService: new Date(Date.now() - 30 * 24 * 60 * 60 * 1000),
    nextService: new Date(Date.now() + 60 * 24 * 60 * 60 * 1000),
    permits: {
      registration: {
        number: 'REG-001-2024',
        expires: new Date(Date.now() + 365 * 24 * 60 * 60 * 1000),
        status: 'valid'
      },
      insurance: {
        number: 'INS-001-2024',
        expires: new Date(Date.now() + 180 * 24 * 60 * 60 * 1000),
        status: 'valid'
      },
      inspection: {
        number: 'INSP-001-2024',
        expires: new Date(Date.now() + 90 * 24 * 60 * 60 * 1000),
        status: 'valid'
      }
    },
    sim: {
      iccid: '8971234567890123456',
      status: 'active',
      plan: 'Unlimited 5G',
      dataUsage: 2.5,
      lastHeartbeat: new Date()
    },
    assignments: {
      currentTrip: undefined,
      driver: undefined,
      schedule: []
    },
    groups: ['main_fleet', 'l4_vehicles'],
    tags: ['premium', 'dubai'],
    starred: true
  },
  {
    id: 'v2',
    vin: 'WVW1K7AJ5DW789012',
    name: 'Atlas-002',
    plate: 'AV-002',
    make: 'AtlasMesh',
    model: 'L5 SUV',
    year: 2024,
    depot: 'Abu Dhabi',
    status: 'normal',
    operationalStatus: 'in_service',
    autonomyLevel: 'L5',
    health: {
      overall: 88,
      battery: 85,
      engine: 92,
      sensors: 87,
      communication: 89
    },
    connectivity: {
      status: 'good',
      lastSeen: new Date(Date.now() - 30 * 1000),
      networkLatency: 78,
      signalStrength: -72
    },
    location: {
      current: 'En Route - Sheikh Zayed Road',
      latitude: 25.1972,
      longitude: 55.2744,
      lastUpdate: new Date(Date.now() - 30 * 1000)
    },
    versions: {
      binary: 'v2.0.8',
      roadGraph: 'v2.0.5',
      map: 'v1.9.8',
      lsm: 'v1.5.2',
      policy: 'v1.2.1'
    },
    blacklist: ['sensor_lidar_front'],
    odometer: 32145,
    lastService: new Date(Date.now() - 15 * 24 * 60 * 60 * 1000),
    nextService: new Date(Date.now() + 75 * 24 * 60 * 60 * 1000),
    permits: {
      registration: {
        number: 'REG-002-2024',
        expires: new Date(Date.now() + 350 * 24 * 60 * 60 * 1000),
        status: 'valid'
      },
      insurance: {
        number: 'INS-002-2024',
        expires: new Date(Date.now() + 15 * 24 * 60 * 60 * 1000),
        status: 'expiring'
      },
      inspection: {
        number: 'INSP-002-2024',
        expires: new Date(Date.now() + 5 * 24 * 60 * 60 * 1000),
        status: 'expiring'
      }
    },
    sim: {
      iccid: '8971234567890789012',
      status: 'active',
      plan: '50GB 4G',
      dataUsage: 12.8,
      lastHeartbeat: new Date(Date.now() - 30 * 1000)
    },
    assignments: {
      currentTrip: 'trip_001',
      driver: {
        id: 'd1',
        name: 'Ahmed Al-Mansouri'
      },
      schedule: ['trip_002', 'trip_003']
    },
    groups: ['main_fleet', 'l5_vehicles'],
    tags: ['cargo', 'abu_dhabi'],
    starred: false
  },
  {
    id: 'v3',
    vin: 'WVW1K7AJ5DW345678',
    name: 'Atlas-003',
    plate: 'AV-003',
    make: 'AtlasMesh',
    model: 'L4 Van',
    year: 2023,
    depot: 'Sharjah',
    status: 'down',
    operationalStatus: 'maintenance',
    autonomyLevel: 'L4',
    health: {
      overall: 65,
      battery: 45,
      engine: 72,
      sensors: 68,
      communication: 75
    },
    connectivity: {
      status: 'disconnected',
      lastSeen: new Date(Date.now() - 10 * 60 * 1000),
      networkLatency: 0,
      signalStrength: 0
    },
    location: {
      current: 'Sharjah Service Center',
      latitude: 25.3463,
      longitude: 55.4209,
      lastUpdate: new Date(Date.now() - 10 * 60 * 1000)
    },
    versions: {
      binary: 'v2.0.5',
      roadGraph: 'v1.9.8',
      map: 'v1.9.5',
      lsm: 'v1.4.8',
      policy: 'v1.1.9'
    },
    blacklist: ['camera_rear', 'sensor_radar_left'],
    odometer: 67890,
    lastService: new Date(Date.now() - 5 * 24 * 60 * 60 * 1000),
    nextService: new Date(Date.now() + 30 * 24 * 60 * 60 * 1000),
    permits: {
      registration: {
        number: 'REG-003-2023',
        expires: new Date(Date.now() + 180 * 24 * 60 * 60 * 1000),
        status: 'valid'
      },
      insurance: {
        number: 'INS-003-2023',
        expires: new Date(Date.now() - 5 * 24 * 60 * 60 * 1000),
        status: 'expired'
      },
      inspection: {
        number: 'INSP-003-2023',
        expires: new Date(Date.now() - 10 * 24 * 60 * 60 * 1000),
        status: 'expired'
      }
    },
    sim: {
      iccid: '8971234567890345678',
      status: 'inactive',
      plan: '20GB 4G',
      dataUsage: 0.0,
      lastHeartbeat: new Date(Date.now() - 10 * 60 * 1000)
    },
    assignments: {
      currentTrip: undefined,
      driver: undefined,
      schedule: []
    },
    groups: ['maintenance_queue'],
    tags: ['cargo', 'sharjah', 'maintenance_required'],
    starred: false
  }
]

const mockWorkLogs: WorkLog[] = [
  {
    id: 'wl1',
    vehicleId: 'v3',
    type: 'maintenance',
    title: 'Battery Replacement',
    description: 'Replace main battery pack due to degraded performance',
    technician: 'Omar Hassan',
    startTime: new Date(),
    status: 'in_progress',
    priority: 'high',
    parts: [
      {
        partNumber: 'BAT-001',
        description: 'Main Battery Pack 100kWh',
        quantity: 1,
        cost: 15000
      }
    ],
    attachments: [],
    cost: 15000
  }
]

const VehicleManagement: React.FC = () => {
  // State
  const [vehicles, setVehicles] = useState<Vehicle[]>(mockVehicles)
  const [workLogs, setWorkLogs] = useState<WorkLog[]>(mockWorkLogs)
  const [selectedVehicles, setSelectedVehicles] = useState<string[]>([])
  const [selectedVehicle, setSelectedVehicle] = useState<Vehicle | null>(null)
  const [showVehicleDetail, setShowVehicleDetail] = useState(false)
  const [showAddVehicle, setShowAddVehicle] = useState(false)
  const [showCreateGroup, setShowCreateGroup] = useState(false)
  const [showOTADialog, setShowOTADialog] = useState(false)
  const [viewMode, setViewMode] = useState<'table' | 'cards' | 'map'>('table')
  const [filters, setFilters] = useState({
    status: '',
    operationalStatus: '',
    autonomyLevel: '',
    depot: '',
    model: '',
    permitExpiry: '',
    search: ''
  })
  const [sortBy, setSortBy] = useState('name')
  const [sortOrder, setSortOrder] = useState<'asc' | 'desc'>('asc')

  // Computed values
  const filteredVehicles = useMemo(() => {
    let filtered = vehicles.filter(vehicle => {
      if (filters.status && vehicle.status !== filters.status) return false
      if (filters.operationalStatus && vehicle.operationalStatus !== filters.operationalStatus) return false
      if (filters.autonomyLevel && vehicle.autonomyLevel !== filters.autonomyLevel) return false
      if (filters.depot && vehicle.depot !== filters.depot) return false
      if (filters.model && vehicle.model !== filters.model) return false
      if (filters.permitExpiry) {
        const now = new Date()
        const twentyDaysFromNow = new Date(now.getTime() + 20 * 24 * 60 * 60 * 1000)
        
        const hasExpiringPermit = 
          vehicle.permits.registration.expires <= twentyDaysFromNow ||
          vehicle.permits.insurance.expires <= twentyDaysFromNow ||
          vehicle.permits.inspection.expires <= twentyDaysFromNow

        if (filters.permitExpiry === 'expiring' && !hasExpiringPermit) return false
        if (filters.permitExpiry === 'valid' && hasExpiringPermit) return false
      }
      if (filters.search) {
        const searchTerm = filters.search.toLowerCase()
        return (
          vehicle.name.toLowerCase().includes(searchTerm) ||
          vehicle.plate.toLowerCase().includes(searchTerm) ||
          vehicle.vin.toLowerCase().includes(searchTerm) ||
          vehicle.id.toLowerCase().includes(searchTerm)
        )
      }
      return true
    })

    // Sort
    filtered.sort((a, b) => {
      let aValue: any = a[sortBy as keyof Vehicle]
      let bValue: any = b[sortBy as keyof Vehicle]

      if (typeof aValue === 'string') {
        aValue = aValue.toLowerCase()
        bValue = bValue.toLowerCase()
      }

      if (sortOrder === 'asc') {
        return aValue < bValue ? -1 : aValue > bValue ? 1 : 0
      } else {
        return aValue > bValue ? -1 : aValue < bValue ? 1 : 0
      }
    })

    return filtered
  }, [vehicles, filters, sortBy, sortOrder])

  // Statistics
  const stats = useMemo(() => {
    const total = vehicles.length
    const online = vehicles.filter(v => v.operationalStatus !== 'offline').length
    const maintenance = vehicles.filter(v => v.operationalStatus === 'maintenance').length
    const available = vehicles.filter(v => v.operationalStatus === 'available').length
    const avgHealth = Math.round(vehicles.reduce((sum, v) => sum + v.health.overall, 0) / total)
    const expiringPermits = vehicles.filter(v => {
      const now = new Date()
      const twentyDaysFromNow = new Date(now.getTime() + 20 * 24 * 60 * 60 * 1000)
      return (
        v.permits.registration.expires <= twentyDaysFromNow ||
        v.permits.insurance.expires <= twentyDaysFromNow ||
        v.permits.inspection.expires <= twentyDaysFromNow
      )
    }).length

    return { total, online, maintenance, available, avgHealth, expiringPermits }
  }, [vehicles])

  // Handlers
  const handleVehicleClick = useCallback((vehicleId: string) => {
    const vehicle = vehicles.find(v => v.id === vehicleId)
    if (vehicle) {
      setSelectedVehicle(vehicle)
      setShowVehicleDetail(true)
    }
  }, [vehicles])

  const handleToggleStar = useCallback((vehicleId: string) => {
    setVehicles(prev => prev.map(v => 
      v.id === vehicleId ? { ...v, starred: !v.starred } : v
    ))
  }, [])

  const handleBulkAction = useCallback((action: string) => {
    console.info(`Vehicle Management: ${action} executed on ${selectedVehicles.length} vehicles`, {
      action,
      vehicleIds: selectedVehicles,
      timestamp: new Date().toISOString()
    })

    switch (action) {
      case 'ota_update':
        setShowOTADialog(true)
        break
      case 'maintenance_schedule':
        // Open maintenance scheduling dialog
        break
      case 'group_assign':
        // Open group assignment dialog
        break
      case 'export':
        // Export selected vehicles data
        break
    }
  }, [selectedVehicles])

  // Get status color
  const getStatusColor = (status: string) => {
    switch (status) {
      case 'normal': return 'text-green-600 bg-green-100'
      case 'unknown': return 'text-gray-600 bg-gray-100'
      case 'down': return 'text-red-600 bg-red-100'
      case 'deprecated': return 'text-orange-600 bg-orange-100'
      case 'available': return 'text-blue-600 bg-blue-100'
      case 'in_service': return 'text-purple-600 bg-purple-100'
      case 'maintenance': return 'text-yellow-600 bg-yellow-100'
      case 'offline': return 'text-gray-600 bg-gray-100'
      default: return 'text-gray-600 bg-gray-100'
    }
  }

  const getPermitStatusColor = (status: string) => {
    switch (status) {
      case 'valid': return 'text-green-600 bg-green-100'
      case 'expiring': return 'text-yellow-600 bg-yellow-100'
      case 'expired': return 'text-red-600 bg-red-100'
      default: return 'text-gray-600 bg-gray-100'
    }
  }

  // Table columns
  const tableColumns = [
    {
      header: 'Vehicle',
      accessorKey: 'name',
      cell: ({ row }: any) => (
        <div className="flex items-center space-x-3">
          <Button
            variant="ghost"
            size="sm"
            onClick={() => handleToggleStar(row.original.id)}
          >
            {row.original.starred ? (
              <Star className="w-4 h-4 text-yellow-500 fill-current" />
            ) : (
              <StarOff className="w-4 h-4 text-gray-400" />
            )}
          </Button>
          <div>
            <div className="font-medium text-gray-900">{row.original.name}</div>
            <div className="text-sm text-gray-500">{row.original.plate} • {row.original.vin}</div>
          </div>
        </div>
      )
    },
    {
      header: 'Status',
      accessorKey: 'status',
      cell: ({ row }: any) => (
        <div className="space-y-1">
          <Badge className={getStatusColor(row.original.status)}>
            {row.original.status}
          </Badge>
          <Badge className={getStatusColor(row.original.operationalStatus)} size="sm">
            {row.original.operationalStatus}
          </Badge>
        </div>
      )
    },
    {
      header: 'Health',
      accessorKey: 'health',
      cell: ({ row }: any) => {
        const health = row.original.health.overall
        return (
          <div className="flex items-center space-x-2">
            <Progress value={health} className="w-16 h-2" />
            <span className={`text-sm font-medium ${
              health >= 90 ? 'text-green-600' : 
              health >= 70 ? 'text-yellow-600' : 'text-red-600'
            }`}>
              {health}%
            </span>
          </div>
        )
      }
    },
    {
      header: 'Location',
      accessorKey: 'location',
      cell: ({ row }: any) => (
        <div className="text-sm">
          <div className="font-medium text-gray-900">{row.original.location.current}</div>
          <div className="text-gray-500">{row.original.depot}</div>
        </div>
      )
    },
    {
      header: 'Connectivity',
      accessorKey: 'connectivity',
      cell: ({ row }: any) => {
        const { connectivity } = row.original
        const isStale = new Date().getTime() - connectivity.lastSeen.getTime() > 60000 // 1 minute
        
        return (
          <div className="flex items-center space-x-2">
            <div className={`w-2 h-2 rounded-full ${
              connectivity.status === 'excellent' ? 'bg-green-500' :
              connectivity.status === 'good' ? 'bg-blue-500' :
              connectivity.status === 'fair' ? 'bg-yellow-500' :
              connectivity.status === 'poor' ? 'bg-orange-500' : 'bg-red-500'
            }`} />
            <span className={`text-sm capitalize ${isStale ? 'text-red-600' : 'text-gray-700'}`}>
              {connectivity.status}
            </span>
            {isStale && <AlertTriangle className="w-4 h-4 text-red-500" />}
          </div>
        )
      }
    },
    {
      header: 'Permits',
      accessorKey: 'permits',
      cell: ({ row }: any) => {
        const { permits } = row.original
        const hasExpiring = [permits.registration, permits.insurance, permits.inspection]
          .some(p => p.status === 'expiring' || p.status === 'expired')
        
        return (
          <div className="flex items-center space-x-1">
            <TooltipProvider>
              <Tooltip>
                <TooltipTrigger>
                  <Badge className={getPermitStatusColor(permits.registration.status)} size="sm">
                    R
                  </Badge>
                </TooltipTrigger>
                <TooltipContent>
                  Registration: {permits.registration.status}
                </TooltipContent>
              </Tooltip>
            </TooltipProvider>
            
            <TooltipProvider>
              <Tooltip>
                <TooltipTrigger>
                  <Badge className={getPermitStatusColor(permits.insurance.status)} size="sm">
                    I
                  </Badge>
                </TooltipTrigger>
                <TooltipContent>
                  Insurance: {permits.insurance.status}
                </TooltipContent>
              </Tooltip>
            </TooltipProvider>
            
            <TooltipProvider>
              <Tooltip>
                <TooltipTrigger>
                  <Badge className={getPermitStatusColor(permits.inspection.status)} size="sm">
                    T
                  </Badge>
                </TooltipTrigger>
                <TooltipContent>
                  Inspection: {permits.inspection.status}
                </TooltipContent>
              </Tooltip>
            </TooltipProvider>
            
            {hasExpiring && <AlertTriangle className="w-4 h-4 text-yellow-500 ml-1" />}
          </div>
        )
      }
    },
    {
      header: 'Versions',
      accessorKey: 'versions',
      cell: ({ row }: any) => (
        <div className="text-xs space-y-1">
          <div>Binary: {row.original.versions.binary}</div>
          <div>Map: {row.original.versions.map}</div>
        </div>
      )
    },
    {
      header: 'Actions',
      cell: ({ row }: any) => (
        <div className="flex items-center space-x-1">
          <TooltipProvider>
            <Tooltip>
              <TooltipTrigger asChild>
                <Button
                  variant="ghost"
                  size="sm"
                  onClick={() => handleVehicleClick(row.original.id)}
                >
                  <Eye className="w-4 h-4" />
                </Button>
              </TooltipTrigger>
              <TooltipContent>View Details</TooltipContent>
            </Tooltip>
          </TooltipProvider>
          
          <TooltipProvider>
            <Tooltip>
              <TooltipTrigger asChild>
                <Button variant="ghost" size="sm">
                  <Edit className="w-4 h-4" />
                </Button>
              </TooltipTrigger>
              <TooltipContent>Edit Vehicle</TooltipContent>
            </Tooltip>
          </TooltipProvider>
          
          <TooltipProvider>
            <Tooltip>
              <TooltipTrigger asChild>
                <Button variant="ghost" size="sm">
                  <Settings className="w-4 h-4" />
                </Button>
              </TooltipTrigger>
              <TooltipContent>Settings</TooltipContent>
            </Tooltip>
          </TooltipProvider>
        </div>
      )
    }
  ]

  return (
    <div className="flex flex-col h-screen bg-gray-50">
      {/* Header */}
      <div className="bg-white border-b border-gray-200 px-6 py-4">
        <div className="flex items-center justify-between">
          <div className="flex items-center space-x-4">
            <Car className="w-6 h-6 text-blue-600" />
            <h1 className="text-2xl font-semibold text-gray-900">Vehicle Management</h1>
          </div>

          <div className="flex items-center space-x-3">
            {/* View Mode Toggle */}
            <div className="flex border border-gray-300 rounded-md">
              <Button
                variant={viewMode === 'table' ? 'default' : 'ghost'}
                size="sm"
                onClick={() => setViewMode('table')}
                className="rounded-r-none"
              >
                <FileText className="w-4 h-4" />
              </Button>
              <Button
                variant={viewMode === 'cards' ? 'default' : 'ghost'}
                size="sm"
                onClick={() => setViewMode('cards')}
                className="rounded-none border-x"
              >
                <Activity className="w-4 h-4" />
              </Button>
              <Button
                variant={viewMode === 'map' ? 'default' : 'ghost'}
                size="sm"
                onClick={() => setViewMode('map')}
                className="rounded-l-none"
              >
                <MapPin className="w-4 h-4" />
              </Button>
            </div>

            {/* Actions */}
            <Button
              variant="outline"
              onClick={() => setShowCreateGroup(true)}
            >
              <Users className="w-4 h-4 mr-2" />
              New Group
            </Button>

            <Button
              onClick={() => setShowAddVehicle(true)}
            >
              <Plus className="w-4 h-4 mr-2" />
              Add Vehicle
            </Button>

            <Button variant="outline">
              <Download className="w-4 h-4 mr-2" />
              Export
            </Button>
          </div>
        </div>

        {/* Statistics */}
        <div className="grid grid-cols-6 gap-4 mt-4">
          <Card className="p-3">
            <div className="text-center">
              <div className="text-2xl font-bold text-gray-900">{stats.total}</div>
              <div className="text-sm text-gray-600">Total Vehicles</div>
            </div>
          </Card>
          <Card className="p-3">
            <div className="text-center">
              <div className="text-2xl font-bold text-green-600">{stats.online}</div>
              <div className="text-sm text-gray-600">Online</div>
            </div>
          </Card>
          <Card className="p-3">
            <div className="text-center">
              <div className="text-2xl font-bold text-blue-600">{stats.available}</div>
              <div className="text-sm text-gray-600">Available</div>
            </div>
          </Card>
          <Card className="p-3">
            <div className="text-center">
              <div className="text-2xl font-bold text-yellow-600">{stats.maintenance}</div>
              <div className="text-sm text-gray-600">Maintenance</div>
            </div>
          </Card>
          <Card className="p-3">
            <div className="text-center">
              <div className="text-2xl font-bold text-gray-900">{stats.avgHealth}%</div>
              <div className="text-sm text-gray-600">Avg Health</div>
            </div>
          </Card>
          <Card className="p-3">
            <div className="text-center">
              <div className={`text-2xl font-bold ${stats.expiringPermits > 0 ? 'text-red-600' : 'text-green-600'}`}>
                {stats.expiringPermits}
              </div>
              <div className="text-sm text-gray-600">Expiring Permits</div>
            </div>
          </Card>
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
            <option value="normal">Normal</option>
            <option value="unknown">Unknown</option>
            <option value="down">Down</option>
            <option value="deprecated">Deprecated</option>
          </Select>

          <Select
            value={filters.operationalStatus}
            onValueChange={(value) => setFilters({ ...filters, operationalStatus: value })}
          >
            <option value="">All Operations</option>
            <option value="available">Available</option>
            <option value="in_service">In Service</option>
            <option value="maintenance">Maintenance</option>
            <option value="offline">Offline</option>
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

          <Select
            value={filters.permitExpiry}
            onValueChange={(value) => setFilters({ ...filters, permitExpiry: value })}
          >
            <option value="">All Permits</option>
            <option value="valid">Valid (>20 days)</option>
            <option value="expiring">Expiring (≤20 days)</option>
          </Select>

          {selectedVehicles.length > 0 && (
            <div className="flex items-center space-x-2 ml-4 pl-4 border-l border-gray-300">
              <span className="text-sm text-gray-600">{selectedVehicles.length} selected</span>
              <Button size="sm" onClick={() => handleBulkAction('ota_update')} variant="outline">
                <RefreshCw className="w-4 h-4 mr-1" />
                OTA Update
              </Button>
              <Button size="sm" onClick={() => handleBulkAction('maintenance_schedule')} variant="outline">
                <Wrench className="w-4 h-4 mr-1" />
                Schedule Service
              </Button>
              <Button size="sm" onClick={() => handleBulkAction('group_assign')} variant="outline">
                <Users className="w-4 h-4 mr-1" />
                Assign Group
              </Button>
            </div>
          )}
        </div>
      </div>

      {/* Main Content */}
      <div className="flex-1 overflow-hidden">
        {viewMode === 'table' && (
          <div className="p-6">
            <DataTable
              columns={tableColumns}
              data={filteredVehicles}
              onSelectionChange={setSelectedVehicles}
            />
          </div>
        )}

        {viewMode === 'cards' && (
          <ScrollArea className="h-full">
            <div className="p-6">
              <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 xl:grid-cols-4 gap-6">
                {filteredVehicles.map(vehicle => (
                  <Card
                    key={vehicle.id}
                    className={`cursor-pointer transition-all hover:shadow-md ${
                      selectedVehicles.includes(vehicle.id) ? 'ring-2 ring-blue-500' : ''
                    }`}
                    onClick={() => {
                      if (selectedVehicles.includes(vehicle.id)) {
                        setSelectedVehicles(prev => prev.filter(id => id !== vehicle.id))
                      } else {
                        setSelectedVehicles(prev => [...prev, vehicle.id])
                      }
                    }}
                  >
                    <div className="p-4">
                      <div className="flex items-center justify-between mb-3">
                        <div className="flex items-center space-x-2">
                          <Button
                            variant="ghost"
                            size="sm"
                            onClick={(e) => {
                              e.stopPropagation()
                              handleToggleStar(vehicle.id)
                            }}
                          >
                            {vehicle.starred ? (
                              <Star className="w-4 h-4 text-yellow-500 fill-current" />
                            ) : (
                              <StarOff className="w-4 h-4 text-gray-400" />
                            )}
                          </Button>
                          <div>
                            <div className="font-medium text-gray-900">{vehicle.name}</div>
                            <div className="text-sm text-gray-500">{vehicle.plate}</div>
                          </div>
                        </div>
                        
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

                      <div className="space-y-3">
                        <div className="flex items-center justify-between">
                          <Badge className={getStatusColor(vehicle.status)}>
                            {vehicle.status}
                          </Badge>
                          <Badge className={getStatusColor(vehicle.operationalStatus)} size="sm">
                            {vehicle.operationalStatus}
                          </Badge>
                        </div>

                        <div className="space-y-2">
                          <div className="flex items-center justify-between text-sm">
                            <span className="text-gray-600">Health:</span>
                            <div className="flex items-center space-x-2">
                              <Progress value={vehicle.health.overall} className="w-16 h-2" />
                              <span className={`font-medium ${
                                vehicle.health.overall >= 90 ? 'text-green-600' : 
                                vehicle.health.overall >= 70 ? 'text-yellow-600' : 'text-red-600'
                              }`}>
                                {vehicle.health.overall}%
                              </span>
                            </div>
                          </div>

                          <div className="flex items-center justify-between text-sm">
                            <span className="text-gray-600">Battery:</span>
                            <div className="flex items-center space-x-2">
                              <Battery className="w-4 h-4 text-gray-500" />
                              <span>{vehicle.health.battery}%</span>
                            </div>
                          </div>

                          <div className="flex items-center justify-between text-sm">
                            <span className="text-gray-600">Connectivity:</span>
                            <div className="flex items-center space-x-2">
                              <div className={`w-2 h-2 rounded-full ${
                                vehicle.connectivity.status === 'excellent' ? 'bg-green-500' :
                                vehicle.connectivity.status === 'good' ? 'bg-blue-500' :
                                vehicle.connectivity.status === 'fair' ? 'bg-yellow-500' :
                                vehicle.connectivity.status === 'poor' ? 'bg-orange-500' : 'bg-red-500'
                              }`} />
                              <span className="capitalize">{vehicle.connectivity.status}</span>
                            </div>
                          </div>
                        </div>

                        <div className="pt-2 border-t border-gray-200">
                          <div className="text-sm text-gray-600">
                            <div className="flex items-center space-x-1 mb-1">
                              <MapPin className="w-3 h-3" />
                              <span className="truncate">{vehicle.location.current}</span>
                            </div>
                            <div className="flex items-center space-x-1">
                              <Gauge className="w-3 h-3" />
                              <span>{vehicle.odometer.toLocaleString()} km</span>
                            </div>
                          </div>
                        </div>

                        {vehicle.blacklist.length > 0 && (
                          <div className="pt-2 border-t border-gray-200">
                            <div className="flex items-center space-x-1 text-sm text-red-600">
                              <AlertTriangle className="w-4 h-4" />
                              <span>{vehicle.blacklist.length} blacklisted components</span>
                            </div>
                          </div>
                        )}
                      </div>
                    </div>
                  </Card>
                ))}
              </div>

              {filteredVehicles.length === 0 && (
                <div className="text-center py-12">
                  <Car className="w-16 h-16 text-gray-400 mx-auto mb-4" />
                  <h3 className="text-lg font-medium text-gray-900 mb-2">No Vehicles Found</h3>
                  <p className="text-gray-600">Try adjusting your filters or add a new vehicle</p>
                </div>
              )}
            </div>
          </ScrollArea>
        )}

        {viewMode === 'map' && (
          <div className="h-full bg-gray-100 flex items-center justify-center">
            <div className="text-center">
              <MapPin className="w-16 h-16 text-gray-400 mx-auto mb-4" />
              <h3 className="text-lg font-medium text-gray-900 mb-2">Fleet Map View</h3>
              <p className="text-gray-600">Vehicle locations and status on map</p>
            </div>
          </div>
        )}
      </div>

      {/* Vehicle Detail Dialog */}
      <Dialog open={showVehicleDetail} onOpenChange={setShowVehicleDetail}>
        <DialogContent className="max-w-6xl max-h-[90vh] overflow-y-auto">
          <DialogHeader>
            <DialogTitle>
              Vehicle Details: {selectedVehicle?.name}
            </DialogTitle>
          </DialogHeader>

          {selectedVehicle && (
            <div className="space-y-6">
              <Tabs defaultValue="overview">
                <TabsList>
                  <TabsTrigger value="overview">Overview</TabsTrigger>
                  <TabsTrigger value="health">Health & Diagnostics</TabsTrigger>
                  <TabsTrigger value="versions">Software Versions</TabsTrigger>
                  <TabsTrigger value="permits">Permits & Compliance</TabsTrigger>
                  <TabsTrigger value="sim">SIM & Connectivity</TabsTrigger>
                  <TabsTrigger value="schedule">Schedules & Assignments</TabsTrigger>
                  <TabsTrigger value="worklogs">Work Logs</TabsTrigger>
                </TabsList>

                <TabsContent value="overview" className="space-y-4">
                  <div className="grid grid-cols-3 gap-6">
                    <Card className="p-4">
                      <div className="text-center">
                        <div className="text-2xl font-bold text-gray-900">{selectedVehicle.health.overall}%</div>
                        <div className="text-sm text-gray-600">Overall Health</div>
                        <Progress value={selectedVehicle.health.overall} className="mt-2" />
                      </div>
                    </Card>

                    <Card className="p-4">
                      <div className="text-center">
                        <div className="text-2xl font-bold text-gray-900">{selectedVehicle.odometer.toLocaleString()}</div>
                        <div className="text-sm text-gray-600">Kilometers</div>
                      </div>
                    </Card>

                    <Card className="p-4">
                      <div className="text-center">
                        <div className="text-2xl font-bold text-gray-900">{selectedVehicle.autonomyLevel}</div>
                        <div className="text-sm text-gray-600">Autonomy Level</div>
                      </div>
                    </Card>
                  </div>

                  <div className="grid grid-cols-2 gap-6">
                    <div>
                      <h3 className="text-lg font-medium text-gray-900 mb-3">Vehicle Information</h3>
                      <div className="space-y-2 text-sm">
                        <div className="flex justify-between">
                          <span className="text-gray-600">VIN:</span>
                          <span className="font-medium">{selectedVehicle.vin}</span>
                        </div>
                        <div className="flex justify-between">
                          <span className="text-gray-600">Make/Model:</span>
                          <span className="font-medium">{selectedVehicle.make} {selectedVehicle.model}</span>
                        </div>
                        <div className="flex justify-between">
                          <span className="text-gray-600">Year:</span>
                          <span className="font-medium">{selectedVehicle.year}</span>
                        </div>
                        <div className="flex justify-between">
                          <span className="text-gray-600">Depot:</span>
                          <span className="font-medium">{selectedVehicle.depot}</span>
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
                      <h3 className="text-lg font-medium text-gray-900 mb-3">Current Location</h3>
                      <div className="space-y-2 text-sm">
                        <div className="flex justify-between">
                          <span className="text-gray-600">Location:</span>
                          <span className="font-medium">{selectedVehicle.location.current}</span>
                        </div>
                        {selectedVehicle.location.latitude && (
                          <>
                            <div className="flex justify-between">
                              <span className="text-gray-600">Latitude:</span>
                              <span className="font-medium">{selectedVehicle.location.latitude.toFixed(6)}</span>
                            </div>
                            <div className="flex justify-between">
                              <span className="text-gray-600">Longitude:</span>
                              <span className="font-medium">{selectedVehicle.location.longitude.toFixed(6)}</span>
                            </div>
                          </>
                        )}
                        <div className="flex justify-between">
                          <span className="text-gray-600">Last Update:</span>
                          <span className="font-medium">{selectedVehicle.location.lastUpdate.toLocaleString()}</span>
                        </div>
                      </div>
                    </div>
                  </div>
                </TabsContent>

                <TabsContent value="health" className="space-y-4">
                  <div className="grid grid-cols-2 gap-6">
                    <Card className="p-4">
                      <h3 className="text-lg font-medium text-gray-900 mb-3">System Health</h3>
                      <div className="space-y-3">
                        {Object.entries(selectedVehicle.health).map(([key, value]) => (
                          <div key={key} className="flex items-center justify-between">
                            <span className="text-gray-600 capitalize">{key.replace('_', ' ')}:</span>
                            <div className="flex items-center space-x-2">
                              <Progress value={value} className="w-20 h-2" />
                              <span className={`font-medium ${
                                value >= 90 ? 'text-green-600' : 
                                value >= 70 ? 'text-yellow-600' : 'text-red-600'
                              }`}>
                                {value}%
                              </span>
                            </div>
                          </div>
                        ))}
                      </div>
                    </Card>

                    <Card className="p-4">
                      <h3 className="text-lg font-medium text-gray-900 mb-3">Blacklisted Components</h3>
                      {selectedVehicle.blacklist.length > 0 ? (
                        <div className="space-y-2">
                          {selectedVehicle.blacklist.map((component, index) => (
                            <div key={index} className="flex items-center space-x-2 text-sm">
                              <AlertTriangle className="w-4 h-4 text-red-500" />
                              <span className="text-red-700">{component}</span>
                            </div>
                          ))}
                        </div>
                      ) : (
                        <div className="text-center py-4">
                          <CheckCircle className="w-8 h-8 text-green-500 mx-auto mb-2" />
                          <p className="text-sm text-gray-600">No blacklisted components</p>
                        </div>
                      )}
                    </Card>
                  </div>
                </TabsContent>

                <TabsContent value="versions" className="space-y-4">
                  <Card className="p-4">
                    <div className="flex items-center justify-between mb-4">
                      <h3 className="text-lg font-medium text-gray-900">Software Versions</h3>
                      <div className="flex space-x-2">
                        <Button size="sm" variant="outline">
                          <Pin className="w-4 h-4 mr-1" />
                          Pin Version
                        </Button>
                        <Button size="sm" variant="outline">
                          <RotateCcw className="w-4 h-4 mr-1" />
                          Rollback
                        </Button>
                      </div>
                    </div>
                    
                    <div className="grid grid-cols-2 gap-6">
                      <div className="space-y-3">
                        {Object.entries(selectedVehicle.versions).map(([key, value]) => (
                          <div key={key} className="flex items-center justify-between p-2 bg-gray-50 rounded">
                            <span className="font-medium capitalize">{key.replace('_', ' ')}:</span>
                            <div className="flex items-center space-x-2">
                              <Badge variant="outline">{value}</Badge>
                              <Button variant="ghost" size="sm">
                                <ExternalLink className="w-3 h-3" />
                              </Button>
                            </div>
                          </div>
                        ))}
                      </div>
                      
                      <div>
                        <h4 className="font-medium text-gray-900 mb-2">Version History</h4>
                        <div className="space-y-2 text-sm">
                          <div className="p-2 bg-blue-50 rounded border border-blue-200">
                            <div className="font-medium text-blue-900">v2.1.0 (Current)</div>
                            <div className="text-blue-700">Deployed: 2 days ago</div>
                          </div>
                          <div className="p-2 bg-gray-50 rounded">
                            <div className="font-medium text-gray-900">v2.0.8</div>
                            <div className="text-gray-600">Deployed: 1 week ago</div>
                          </div>
                          <div className="p-2 bg-gray-50 rounded">
                            <div className="font-medium text-gray-900">v2.0.5</div>
                            <div className="text-gray-600">Deployed: 2 weeks ago</div>
                          </div>
                        </div>
                      </div>
                    </div>
                  </Card>
                </TabsContent>

                <TabsContent value="permits" className="space-y-4">
                  <div className="grid grid-cols-3 gap-6">
                    {Object.entries(selectedVehicle.permits).map(([key, permit]) => (
                      <Card key={key} className="p-4">
                        <h3 className="text-lg font-medium text-gray-900 mb-3 capitalize">{key}</h3>
                        <div className="space-y-2 text-sm">
                          <div className="flex justify-between">
                            <span className="text-gray-600">Number:</span>
                            <span className="font-medium">{permit.number}</span>
                          </div>
                          <div className="flex justify-between">
                            <span className="text-gray-600">Expires:</span>
                            <span className="font-medium">{permit.expires.toLocaleDateString()}</span>
                          </div>
                          <div className="flex justify-between">
                            <span className="text-gray-600">Status:</span>
                            <Badge className={getPermitStatusColor(permit.status)}>
                              {permit.status}
                            </Badge>
                          </div>
                          {permit.status === 'expiring' && (
                            <Alert className="mt-2">
                              <AlertTriangle className="w-4 h-4" />
                              <AlertDescription className="text-xs">
                                Expires in {Math.ceil((permit.expires.getTime() - new Date().getTime()) / (1000 * 60 * 60 * 24))} days
                              </AlertDescription>
                            </Alert>
                          )}
                        </div>
                      </Card>
                    ))}
                  </div>
                </TabsContent>

                <TabsContent value="sim" className="space-y-4">
                  <Card className="p-4">
                    <h3 className="text-lg font-medium text-gray-900 mb-3">SIM Card Information</h3>
                    <div className="grid grid-cols-2 gap-6">
                      <div className="space-y-2 text-sm">
                        <div className="flex justify-between">
                          <span className="text-gray-600">ICCID:</span>
                          <span className="font-medium font-mono">{selectedVehicle.sim.iccid}</span>
                        </div>
                        <div className="flex justify-between">
                          <span className="text-gray-600">Status:</span>
                          <Badge className={selectedVehicle.sim.status === 'active' ? 'bg-green-100 text-green-700' : 'bg-red-100 text-red-700'}>
                            {selectedVehicle.sim.status}
                          </Badge>
                        </div>
                        <div className="flex justify-between">
                          <span className="text-gray-600">Plan:</span>
                          <span className="font-medium">{selectedVehicle.sim.plan}</span>
                        </div>
                      </div>
                      <div className="space-y-2 text-sm">
                        <div className="flex justify-between">
                          <span className="text-gray-600">Data Usage:</span>
                          <span className="font-medium">{selectedVehicle.sim.dataUsage} GB</span>
                        </div>
                        <div className="flex justify-between">
                          <span className="text-gray-600">Last Heartbeat:</span>
                          <span className="font-medium">{selectedVehicle.sim.lastHeartbeat.toLocaleString()}</span>
                        </div>
                      </div>
                    </div>
                  </Card>

                  <Card className="p-4">
                    <h3 className="text-lg font-medium text-gray-900 mb-3">Connectivity Status</h3>
                    <div className="grid grid-cols-2 gap-6">
                      <div className="space-y-2 text-sm">
                        <div className="flex justify-between">
                          <span className="text-gray-600">Status:</span>
                          <div className="flex items-center space-x-2">
                            <div className={`w-2 h-2 rounded-full ${
                              selectedVehicle.connectivity.status === 'excellent' ? 'bg-green-500' :
                              selectedVehicle.connectivity.status === 'good' ? 'bg-blue-500' :
                              selectedVehicle.connectivity.status === 'fair' ? 'bg-yellow-500' :
                              selectedVehicle.connectivity.status === 'poor' ? 'bg-orange-500' : 'bg-red-500'
                            }`} />
                            <span className="capitalize">{selectedVehicle.connectivity.status}</span>
                          </div>
                        </div>
                        <div className="flex justify-between">
                          <span className="text-gray-600">Network Latency:</span>
                          <span className="font-medium">{selectedVehicle.connectivity.networkLatency}ms</span>
                        </div>
                      </div>
                      <div className="space-y-2 text-sm">
                        <div className="flex justify-between">
                          <span className="text-gray-600">Signal Strength:</span>
                          <span className="font-medium">{selectedVehicle.connectivity.signalStrength}dBm</span>
                        </div>
                        <div className="flex justify-between">
                          <span className="text-gray-600">Last Seen:</span>
                          <span className="font-medium">{selectedVehicle.connectivity.lastSeen.toLocaleString()}</span>
                        </div>
                      </div>
                    </div>
                  </Card>
                </TabsContent>

                <TabsContent value="schedule" className="space-y-4">
                  <div className="grid grid-cols-2 gap-6">
                    <Card className="p-4">
                      <h3 className="text-lg font-medium text-gray-900 mb-3">Current Assignment</h3>
                      {selectedVehicle.assignments.currentTrip ? (
                        <div className="space-y-2 text-sm">
                          <div className="flex justify-between">
                            <span className="text-gray-600">Trip ID:</span>
                            <span className="font-medium">{selectedVehicle.assignments.currentTrip}</span>
                          </div>
                          {selectedVehicle.assignments.driver && (
                            <div className="flex justify-between">
                              <span className="text-gray-600">Driver:</span>
                              <span className="font-medium">{selectedVehicle.assignments.driver.name}</span>
                            </div>
                          )}
                        </div>
                      ) : (
                        <div className="text-center py-4">
                          <p className="text-sm text-gray-600">No current assignment</p>
                        </div>
                      )}
                    </Card>

                    <Card className="p-4">
                      <h3 className="text-lg font-medium text-gray-900 mb-3">Upcoming Schedule</h3>
                      {selectedVehicle.assignments.schedule.length > 0 ? (
                        <div className="space-y-2">
                          {selectedVehicle.assignments.schedule.map((tripId, index) => (
                            <div key={index} className="flex items-center space-x-2 text-sm">
                              <Clock className="w-4 h-4 text-gray-500" />
                              <span>{tripId}</span>
                            </div>
                          ))}
                        </div>
                      ) : (
                        <div className="text-center py-4">
                          <p className="text-sm text-gray-600">No scheduled trips</p>
                        </div>
                      )}
                    </Card>
                  </div>

                  <Card className="p-4">
                    <h3 className="text-lg font-medium text-gray-900 mb-3">Groups & Tags</h3>
                    <div className="space-y-3">
                      <div>
                        <span className="text-sm text-gray-600 block mb-2">Groups:</span>
                        <div className="flex flex-wrap gap-2">
                          {selectedVehicle.groups.map((group, index) => (
                            <Badge key={index} variant="secondary">
                              <Users className="w-3 h-3 mr-1" />
                              {group}
                            </Badge>
                          ))}
                        </div>
                      </div>
                      <div>
                        <span className="text-sm text-gray-600 block mb-2">Tags:</span>
                        <div className="flex flex-wrap gap-2">
                          {selectedVehicle.tags.map((tag, index) => (
                            <Badge key={index} variant="outline">
                              <Tags className="w-3 h-3 mr-1" />
                              {tag}
                            </Badge>
                          ))}
                        </div>
                      </div>
                    </div>
                  </Card>
                </TabsContent>

                <TabsContent value="worklogs" className="space-y-4">
                  <div className="flex items-center justify-between">
                    <h3 className="text-lg font-medium text-gray-900">Work Logs</h3>
                    <Button size="sm">
                      <Plus className="w-4 h-4 mr-1" />
                      Add Work Log
                    </Button>
                  </div>

                  {workLogs.filter(log => log.vehicleId === selectedVehicle.id).length > 0 ? (
                    <div className="space-y-3">
                      {workLogs
                        .filter(log => log.vehicleId === selectedVehicle.id)
                        .map(log => (
                          <Card key={log.id} className="p-4">
                            <div className="flex items-start justify-between">
                              <div>
                                <div className="font-medium text-gray-900">{log.title}</div>
                                <div className="text-sm text-gray-600 mt-1">{log.description}</div>
                                <div className="flex items-center space-x-4 mt-2 text-sm text-gray-500">
                                  <span>Type: {log.type}</span>
                                  <span>Technician: {log.technician}</span>
                                  <span>Started: {log.startTime.toLocaleDateString()}</span>
                                </div>
                              </div>
                              <div className="flex items-center space-x-2">
                                <Badge 
                                  variant={
                                    log.status === 'completed' ? 'success' :
                                    log.status === 'in_progress' ? 'warning' :
                                    log.status === 'cancelled' ? 'destructive' : 'secondary'
                                  }
                                >
                                  {log.status}
                                </Badge>
                                <Badge variant="outline">
                                  {log.priority}
                                </Badge>
                              </div>
                            </div>
                            
                            {log.parts.length > 0 && (
                              <div className="mt-3 pt-3 border-t border-gray-200">
                                <div className="text-sm font-medium text-gray-900 mb-2">Parts Used:</div>
                                <div className="space-y-1">
                                  {log.parts.map((part, index) => (
                                    <div key={index} className="text-sm text-gray-600">
                                      {part.description} (Qty: {part.quantity}) - ${part.cost}
                                    </div>
                                  ))}
                                </div>
                                <div className="text-sm font-medium text-gray-900 mt-2">
                                  Total Cost: ${log.cost}
                                </div>
                              </div>
                            )}
                          </Card>
                        ))}
                    </div>
                  ) : (
                    <div className="text-center py-12">
                      <FileText className="w-16 h-16 text-gray-400 mx-auto mb-4" />
                      <h3 className="text-lg font-medium text-gray-900 mb-2">No Work Logs</h3>
                      <p className="text-gray-600">No maintenance or service records found</p>
                    </div>
                  )}
                </TabsContent>
              </Tabs>
            </div>
          )}
        </DialogContent>
      </Dialog>

      {/* Add Vehicle Dialog */}
      <Dialog open={showAddVehicle} onOpenChange={setShowAddVehicle}>
        <DialogContent className="max-w-2xl">
          <DialogHeader>
            <DialogTitle>Add New Vehicle</DialogTitle>
          </DialogHeader>
          <div className="space-y-4">
            <div className="text-center py-8">
              <Plus className="w-16 h-16 text-gray-400 mx-auto mb-4" />
              <h3 className="text-lg font-medium text-gray-900 mb-2">Add Vehicle Form</h3>
              <p className="text-gray-600">Vehicle registration and onboarding form</p>
            </div>
          </div>
        </DialogContent>
      </Dialog>

      {/* OTA Update Dialog */}
      <Dialog open={showOTADialog} onOpenChange={setShowOTADialog}>
        <DialogContent>
          <DialogHeader>
            <DialogTitle>OTA Update</DialogTitle>
          </DialogHeader>
          <div className="space-y-4">
            <Alert>
              <RefreshCw className="w-4 h-4" />
              <AlertDescription>
                You are about to initiate an OTA update on {selectedVehicles.length} vehicle(s). 
                This will require dual authentication and may take several minutes to complete.
              </AlertDescription>
            </Alert>
            <div className="flex justify-end space-x-3">
              <Button variant="outline" onClick={() => setShowOTADialog(false)}>
                Cancel
              </Button>
              <Button onClick={() => {
                // Implement OTA update
                setShowOTADialog(false)
                setSelectedVehicles([])
              }}>
                Proceed with Update
              </Button>
            </div>
          </div>
        </DialogContent>
      </Dialog>
    </div>
  )
}

export default VehicleManagement
