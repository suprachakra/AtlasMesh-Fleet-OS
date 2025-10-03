import React, { useState, useEffect, useCallback, useMemo } from 'react'
import { 
  Activity, Wifi, WifiOff, Battery, Thermometer, Gauge, Navigation,
  AlertTriangle, CheckCircle, XCircle, Clock, MapPin, Zap, Shield,
  Camera, Radar, Radio, Cpu, HardDrive, Network, Signal, Eye,
  TrendingUp, TrendingDown, Minus, RefreshCw, Filter, Search,
  Play, Pause, SkipForward, Rewind, Maximize2, Minimize2
} from 'lucide-react'
import { Button } from '../ui/Button'
import { Badge } from '../ui/Badge'
import { Card } from '../ui/Card'
import { Progress } from '../ui/Progress'
import { Select } from '../ui/Select'
import { Input } from '../ui/Input'
import { Switch } from '../ui/Switch'
import { Tabs, TabsContent, TabsList, TabsTrigger } from '../ui/Tabs'
import { Alert, AlertDescription } from '../ui/Alert'

// Types
interface VehicleTelemetry {
  vehicleId: string
  timestamp: Date
  location: {
    latitude: number
    longitude: number
    altitude?: number
    heading: number
    speed: number // m/s
    accuracy: number // meters
  }
  powertrain: {
    batteryLevel: number // 0-100%
    batteryHealth: number // 0-100%
    batteryTemperature: number // Celsius
    batteryVoltage: number // Volts
    batteryCurrent: number // Amperes
    chargingStatus: 'charging' | 'discharging' | 'idle' | 'full'
    estimatedRange: number // kilometers
    energyConsumption: number // kWh/100km
    regenerativeEfficiency: number // 0-100%
  }
  autonomy: {
    mode: 'manual' | 'assisted' | 'autonomous' | 'remote' | 'emergency'
    confidence: number // 0-100%
    decisionLatency: number // milliseconds
    pathPlanningStatus: 'active' | 'replanning' | 'failed' | 'idle'
    obstacleCount: number
    trajectoryDeviation: number // meters
    safetyScore: number // 0-100%
    interventionCount: number
    lastIntervention?: Date
  }
  sensors: {
    lidar: SensorStatus[]
    cameras: SensorStatus[]
    radar: SensorStatus[]
    ultrasonic: SensorStatus[]
    imu: SensorStatus
    gnss: SensorStatus
    odometry: SensorStatus
  }
  compute: {
    cpuUsage: number // 0-100%
    memoryUsage: number // 0-100%
    diskUsage: number // 0-100%
    gpuUsage: number // 0-100%
    temperature: number // Celsius
    networkLatency: number // milliseconds
    networkBandwidth: number // Mbps
    processCount: number
    uptime: number // seconds
  }
  connectivity: {
    cellular: {
      signalStrength: number // -120 to -50 dBm
      networkType: '4G' | '5G' | '3G' | 'Unknown'
      dataUsage: number // MB
      latency: number // milliseconds
      connected: boolean
    }
    wifi: {
      signalStrength: number // -100 to -30 dBm
      ssid?: string
      connected: boolean
      dataUsage: number // MB
    }
    v2x: {
      connected: boolean
      messageCount: number
      lastMessage?: Date
      neighbors: number
    }
  }
  safety: {
    airbagStatus: 'ready' | 'deployed' | 'fault'
    seatbeltStatus: boolean[]
    doorStatus: boolean[] // true = open, false = closed
    emergencyBrakeStatus: 'ready' | 'active' | 'fault'
    collisionRisk: number // 0-100%
    safetySystemsActive: boolean
    lastSafetyEvent?: Date
    geofenceStatus: 'inside' | 'outside' | 'approaching' | 'unknown'
  }
  diagnostics: {
    faultCodes: FaultCode[]
    warningLights: WarningLight[]
    maintenanceAlerts: MaintenanceAlert[]
    systemHealth: number // 0-100%
    lastDiagnostic: Date
    nextScheduledMaintenance: Date
  }
}

interface SensorStatus {
  id: string
  name: string
  type: string
  status: 'online' | 'offline' | 'degraded' | 'calibrating' | 'error'
  health: number // 0-100%
  temperature?: number
  dataRate?: number // Hz
  lastUpdate: Date
  errorCount: number
  calibrationStatus?: 'calibrated' | 'needs_calibration' | 'calibrating'
}

interface FaultCode {
  code: string
  description: string
  severity: 'info' | 'warning' | 'critical' | 'error'
  system: string
  timestamp: Date
  cleared: boolean
  count: number
}

interface WarningLight {
  id: string
  name: string
  status: 'off' | 'on' | 'blinking'
  color: 'red' | 'yellow' | 'green' | 'blue'
  description: string
  priority: 'low' | 'medium' | 'high' | 'critical'
}

interface MaintenanceAlert {
  id: string
  type: 'scheduled' | 'predictive' | 'immediate'
  component: string
  description: string
  dueDate: Date
  priority: 'low' | 'medium' | 'high' | 'critical'
  estimatedCost: number
}

interface VehicleStatus {
  vehicleId: string
  operationalStatus: 'operational' | 'warning' | 'critical' | 'offline' | 'maintenance'
  lastSeen: Date
  connectionQuality: 'excellent' | 'good' | 'fair' | 'poor' | 'disconnected'
  alertCount: number
  healthScore: number // 0-100%
  location: string
  currentTrip?: string
  nextMaintenance: Date
}

interface MonitoringAlert {
  id: string
  vehicleId: string
  type: 'telemetry' | 'connectivity' | 'safety' | 'maintenance' | 'security'
  severity: 'info' | 'warning' | 'critical' | 'error'
  title: string
  description: string
  timestamp: Date
  acknowledged: boolean
  resolved: boolean
  resolvedAt?: Date
  assignedTo?: string
  relatedAlerts: string[]
}

interface RealTimeVehicleMonitoringProps {
  vehicleIds?: string[]
  fleetId?: string
  autoRefresh?: boolean
  refreshInterval?: number // seconds
  onAlert?: (alert: MonitoringAlert) => void
  onVehicleSelected?: (vehicleId: string) => void
  className?: string
}

const RealTimeVehicleMonitoring: React.FC<RealTimeVehicleMonitoringProps> = ({
  vehicleIds,
  fleetId,
  autoRefresh = true,
  refreshInterval = 5,
  onAlert,
  onVehicleSelected,
  className = ''
}) => {
  // State
  const [vehicles, setVehicles] = useState<VehicleStatus[]>([])
  const [selectedVehicle, setSelectedVehicle] = useState<string | null>(null)
  const [telemetryData, setTelemetryData] = useState<Record<string, VehicleTelemetry>>({})
  const [alerts, setAlerts] = useState<MonitoringAlert[]>([])
  const [activeTab, setActiveTab] = useState('overview')
  const [filters, setFilters] = useState({
    status: '',
    search: '',
    alertLevel: ''
  })
  const [isPlaying, setIsPlaying] = useState(autoRefresh)
  const [isFullscreen, setIsFullscreen] = useState(false)
  const [showGrid, setShowGrid] = useState(false)
  const [selectedMetrics, setSelectedMetrics] = useState([
    'battery', 'location', 'autonomy', 'connectivity', 'safety'
  ])

  // Mock data initialization
  const initializeMockData = useCallback(() => {
    const mockVehicles: VehicleStatus[] = [
      {
        vehicleId: 'atlas-001',
        operationalStatus: 'operational',
        lastSeen: new Date(),
        connectionQuality: 'excellent',
        alertCount: 0,
        healthScore: 94,
        location: 'Dubai Mall',
        currentTrip: 'trip-20241126-001',
        nextMaintenance: new Date('2024-12-15T09:00:00Z')
      },
      {
        vehicleId: 'atlas-002',
        operationalStatus: 'warning',
        lastSeen: new Date(Date.now() - 30000),
        connectionQuality: 'good',
        alertCount: 2,
        healthScore: 78,
        location: 'Business Bay',
        nextMaintenance: new Date('2024-12-08T14:00:00Z')
      },
      {
        vehicleId: 'atlas-003',
        operationalStatus: 'critical',
        lastSeen: new Date(Date.now() - 120000),
        connectionQuality: 'poor',
        alertCount: 5,
        healthScore: 42,
        location: 'DIFC',
        nextMaintenance: new Date('2024-11-28T10:00:00Z')
      },
      {
        vehicleId: 'atlas-004',
        operationalStatus: 'offline',
        lastSeen: new Date(Date.now() - 600000),
        connectionQuality: 'disconnected',
        alertCount: 1,
        healthScore: 0,
        location: 'Unknown',
        nextMaintenance: new Date('2024-12-20T11:00:00Z')
      }
    ]

    const mockTelemetry: Record<string, VehicleTelemetry> = {
      'atlas-001': {
        vehicleId: 'atlas-001',
        timestamp: new Date(),
        location: {
          latitude: 25.1972,
          longitude: 55.2796,
          altitude: 12,
          heading: 45,
          speed: 8.33, // 30 km/h
          accuracy: 2.1
        },
        powertrain: {
          batteryLevel: 78,
          batteryHealth: 94,
          batteryTemperature: 32,
          batteryVoltage: 400.2,
          batteryCurrent: -45.8,
          chargingStatus: 'discharging',
          estimatedRange: 234,
          energyConsumption: 18.5,
          regenerativeEfficiency: 87
        },
        autonomy: {
          mode: 'autonomous',
          confidence: 92,
          decisionLatency: 45,
          pathPlanningStatus: 'active',
          obstacleCount: 3,
          trajectoryDeviation: 0.8,
          safetyScore: 95,
          interventionCount: 0
        },
        sensors: {
          lidar: [
            {
              id: 'lidar-front',
              name: 'Front LiDAR',
              type: 'Velodyne VLS-128',
              status: 'online',
              health: 96,
              temperature: 35,
              dataRate: 10,
              lastUpdate: new Date(),
              errorCount: 0,
              calibrationStatus: 'calibrated'
            }
          ],
          cameras: [
            {
              id: 'cam-front',
              name: 'Front Camera',
              type: 'RGB Camera',
              status: 'online',
              health: 98,
              dataRate: 30,
              lastUpdate: new Date(),
              errorCount: 0,
              calibrationStatus: 'calibrated'
            },
            {
              id: 'cam-rear',
              name: 'Rear Camera',
              type: 'RGB Camera',
              status: 'online',
              health: 95,
              dataRate: 30,
              lastUpdate: new Date(),
              errorCount: 1,
              calibrationStatus: 'calibrated'
            }
          ],
          radar: [
            {
              id: 'radar-front',
              name: 'Front Radar',
              type: 'Continental ARS540',
              status: 'online',
              health: 93,
              temperature: 42,
              dataRate: 20,
              lastUpdate: new Date(),
              errorCount: 0,
              calibrationStatus: 'calibrated'
            }
          ],
          ultrasonic: [
            {
              id: 'us-fl',
              name: 'Front Left Ultrasonic',
              type: 'Ultrasonic Sensor',
              status: 'online',
              health: 89,
              dataRate: 10,
              lastUpdate: new Date(),
              errorCount: 2
            }
          ],
          imu: {
            id: 'imu-main',
            name: 'Main IMU',
            type: 'Bosch BMI088',
            status: 'online',
            health: 97,
            temperature: 38,
            dataRate: 100,
            lastUpdate: new Date(),
            errorCount: 0,
            calibrationStatus: 'calibrated'
          },
          gnss: {
            id: 'gnss-main',
            name: 'GNSS Receiver',
            type: 'u-blox F9P',
            status: 'online',
            health: 91,
            dataRate: 10,
            lastUpdate: new Date(),
            errorCount: 0,
            calibrationStatus: 'calibrated'
          },
          odometry: {
            id: 'odom-main',
            name: 'Wheel Odometry',
            type: 'Encoder-based',
            status: 'online',
            health: 94,
            dataRate: 50,
            lastUpdate: new Date(),
            errorCount: 1
          }
        },
        compute: {
          cpuUsage: 68,
          memoryUsage: 72,
          diskUsage: 45,
          gpuUsage: 84,
          temperature: 65,
          networkLatency: 23,
          networkBandwidth: 45.2,
          processCount: 127,
          uptime: 86400 * 5 // 5 days
        },
        connectivity: {
          cellular: {
            signalStrength: -75,
            networkType: '5G',
            dataUsage: 1250,
            latency: 23,
            connected: true
          },
          wifi: {
            signalStrength: -45,
            ssid: 'AtlasMesh-Depot',
            connected: false,
            dataUsage: 0
          },
          v2x: {
            connected: true,
            messageCount: 1547,
            lastMessage: new Date(),
            neighbors: 4
          }
        },
        safety: {
          airbagStatus: 'ready',
          seatbeltStatus: [true, true, false, false],
          doorStatus: [false, false, false, false, false], // 4 doors + trunk
          emergencyBrakeStatus: 'ready',
          collisionRisk: 15,
          safetySystemsActive: true,
          geofenceStatus: 'inside'
        },
        diagnostics: {
          faultCodes: [],
          warningLights: [],
          maintenanceAlerts: [
            {
              id: 'maint-001',
              type: 'scheduled',
              component: 'Battery Coolant',
              description: 'Battery coolant system service due',
              dueDate: new Date('2024-12-15T09:00:00Z'),
              priority: 'medium',
              estimatedCost: 450
            }
          ],
          systemHealth: 94,
          lastDiagnostic: new Date(),
          nextScheduledMaintenance: new Date('2024-12-15T09:00:00Z')
        }
      },
      'atlas-003': {
        vehicleId: 'atlas-003',
        timestamp: new Date(Date.now() - 120000),
        location: {
          latitude: 25.2142,
          longitude: 55.2711,
          altitude: 8,
          heading: 180,
          speed: 0,
          accuracy: 5.2
        },
        powertrain: {
          batteryLevel: 23,
          batteryHealth: 67,
          batteryTemperature: 45,
          batteryVoltage: 385.1,
          batteryCurrent: 0,
          chargingStatus: 'idle',
          estimatedRange: 45,
          energyConsumption: 28.3,
          regenerativeEfficiency: 65
        },
        autonomy: {
          mode: 'manual',
          confidence: 0,
          decisionLatency: 0,
          pathPlanningStatus: 'idle',
          obstacleCount: 0,
          trajectoryDeviation: 0,
          safetyScore: 42,
          interventionCount: 3,
          lastIntervention: new Date(Date.now() - 300000)
        },
        sensors: {
          lidar: [
            {
              id: 'lidar-front',
              name: 'Front LiDAR',
              type: 'Velodyne VLS-128',
              status: 'degraded',
              health: 45,
              temperature: 52,
              dataRate: 5,
              lastUpdate: new Date(Date.now() - 60000),
              errorCount: 15,
              calibrationStatus: 'needs_calibration'
            }
          ],
          cameras: [
            {
              id: 'cam-front',
              name: 'Front Camera',
              type: 'RGB Camera',
              status: 'error',
              health: 12,
              dataRate: 0,
              lastUpdate: new Date(Date.now() - 180000),
              errorCount: 25
            }
          ],
          radar: [
            {
              id: 'radar-front',
              name: 'Front Radar',
              type: 'Continental ARS540',
              status: 'offline',
              health: 0,
              dataRate: 0,
              lastUpdate: new Date(Date.now() - 300000),
              errorCount: 45
            }
          ],
          ultrasonic: [],
          imu: {
            id: 'imu-main',
            name: 'Main IMU',
            type: 'Bosch BMI088',
            status: 'online',
            health: 78,
            temperature: 41,
            dataRate: 100,
            lastUpdate: new Date(),
            errorCount: 3
          },
          gnss: {
            id: 'gnss-main',
            name: 'GNSS Receiver',
            type: 'u-blox F9P',
            status: 'degraded',
            health: 56,
            dataRate: 5,
            lastUpdate: new Date(Date.now() - 30000),
            errorCount: 8
          },
          odometry: {
            id: 'odom-main',
            name: 'Wheel Odometry',
            type: 'Encoder-based',
            status: 'online',
            health: 89,
            dataRate: 50,
            lastUpdate: new Date(),
            errorCount: 2
          }
        },
        compute: {
          cpuUsage: 95,
          memoryUsage: 89,
          diskUsage: 78,
          gpuUsage: 12,
          temperature: 85,
          networkLatency: 156,
          networkBandwidth: 12.5,
          processCount: 89,
          uptime: 86400 * 2
        },
        connectivity: {
          cellular: {
            signalStrength: -105,
            networkType: '4G',
            dataUsage: 2450,
            latency: 156,
            connected: true
          },
          wifi: {
            signalStrength: -85,
            connected: false,
            dataUsage: 0
          },
          v2x: {
            connected: false,
            messageCount: 0,
            neighbors: 0
          }
        },
        safety: {
          airbagStatus: 'fault',
          seatbeltStatus: [false, false, false, false],
          doorStatus: [true, false, false, false, false],
          emergencyBrakeStatus: 'fault',
          collisionRisk: 85,
          safetySystemsActive: false,
          lastSafetyEvent: new Date(Date.now() - 300000),
          geofenceStatus: 'outside'
        },
        diagnostics: {
          faultCodes: [
            {
              code: 'P0420',
              description: 'Catalytic Converter Efficiency Below Threshold',
              severity: 'warning',
              system: 'Powertrain',
              timestamp: new Date(Date.now() - 3600000),
              cleared: false,
              count: 3
            },
            {
              code: 'B1015',
              description: 'LiDAR Sensor Malfunction',
              severity: 'critical',
              system: 'Sensors',
              timestamp: new Date(Date.now() - 180000),
              cleared: false,
              count: 1
            }
          ],
          warningLights: [
            {
              id: 'check-engine',
              name: 'Check Engine',
              status: 'on',
              color: 'yellow',
              description: 'Engine system fault detected',
              priority: 'high'
            },
            {
              id: 'brake-system',
              name: 'Brake System',
              status: 'blinking',
              color: 'red',
              description: 'Brake system malfunction',
              priority: 'critical'
            }
          ],
          maintenanceAlerts: [
            {
              id: 'maint-002',
              type: 'immediate',
              component: 'LiDAR Sensor',
              description: 'URGENT: LiDAR sensor requires immediate replacement',
              dueDate: new Date(),
              priority: 'critical',
              estimatedCost: 8500
            },
            {
              id: 'maint-003',
              type: 'immediate',
              component: 'Brake System',
              description: 'URGENT: Brake system inspection required',
              dueDate: new Date(),
              priority: 'critical',
              estimatedCost: 1200
            }
          ],
          systemHealth: 42,
          lastDiagnostic: new Date(Date.now() - 120000),
          nextScheduledMaintenance: new Date('2024-11-28T10:00:00Z')
        }
      }
    }

    const mockAlerts: MonitoringAlert[] = [
      {
        id: 'alert-001',
        vehicleId: 'atlas-003',
        type: 'safety',
        severity: 'critical',
        title: 'Brake System Malfunction',
        description: 'Emergency brake system has detected a critical fault. Vehicle should not operate.',
        timestamp: new Date(Date.now() - 180000),
        acknowledged: false,
        resolved: false,
        relatedAlerts: ['alert-002']
      },
      {
        id: 'alert-002',
        vehicleId: 'atlas-003',
        type: 'telemetry',
        severity: 'critical',
        title: 'LiDAR Sensor Failure',
        description: 'Front LiDAR sensor has stopped responding. Autonomous operation disabled.',
        timestamp: new Date(Date.now() - 120000),
        acknowledged: false,
        resolved: false,
        relatedAlerts: ['alert-001']
      },
      {
        id: 'alert-003',
        vehicleId: 'atlas-002',
        type: 'connectivity',
        severity: 'warning',
        title: 'Intermittent Connection',
        description: 'Vehicle experiencing intermittent connectivity issues with central systems.',
        timestamp: new Date(Date.now() - 300000),
        acknowledged: true,
        resolved: false,
        assignedTo: 'tech_team'
      }
    ]

    setVehicles(mockVehicles)
    setTelemetryData(mockTelemetry)
    setAlerts(mockAlerts)

    // Set initial selection
    if (!selectedVehicle && mockVehicles.length > 0) {
      setSelectedVehicle(mockVehicles[0].vehicleId)
    }
  }, [selectedVehicle])

  useEffect(() => {
    initializeMockData()
  }, [initializeMockData])

  // Auto-refresh effect
  useEffect(() => {
    if (!isPlaying) return

    const interval = setInterval(() => {
      // Simulate real-time telemetry updates
      setTelemetryData(prev => {
        const updated = { ...prev }
        Object.keys(updated).forEach(vehicleId => {
          const telemetry = updated[vehicleId]
          if (telemetry) {
            // Update timestamp
            telemetry.timestamp = new Date()
            
            // Simulate battery drain
            if (telemetry.powertrain.chargingStatus === 'discharging') {
              telemetry.powertrain.batteryLevel = Math.max(0, 
                telemetry.powertrain.batteryLevel - (Math.random() * 0.5)
              )
            }
            
            // Simulate speed changes
            telemetry.location.speed += (Math.random() - 0.5) * 2
            telemetry.location.speed = Math.max(0, Math.min(25, telemetry.location.speed))
            
            // Simulate sensor health fluctuations
            if (telemetry.sensors.lidar[0]) {
              telemetry.sensors.lidar[0].health += (Math.random() - 0.5) * 2
              telemetry.sensors.lidar[0].health = Math.max(0, Math.min(100, telemetry.sensors.lidar[0].health))
            }
            
            // Update vehicle status
            setVehicles(prevVehicles => prevVehicles.map(v => 
              v.vehicleId === vehicleId 
                ? { 
                    ...v, 
                    lastSeen: new Date(),
                    healthScore: Math.round((telemetry.diagnostics.systemHealth + telemetry.powertrain.batteryHealth) / 2)
                  }
                : v
            ))
          }
        })
        return updated
      })
    }, refreshInterval * 1000)

    return () => clearInterval(interval)
  }, [isPlaying, refreshInterval])

  // Filtered data
  const filteredVehicles = useMemo(() => {
    return vehicles
      .filter(v => !filters.status || v.operationalStatus === filters.status)
      .filter(v => !filters.search || 
        v.vehicleId.toLowerCase().includes(filters.search.toLowerCase()) ||
        v.location.toLowerCase().includes(filters.search.toLowerCase())
      )
      .sort((a, b) => {
        // Sort by status priority: critical > warning > operational > offline
        const statusPriority = { critical: 4, warning: 3, operational: 2, offline: 1, maintenance: 1 }
        const aPriority = statusPriority[a.operationalStatus] || 0
        const bPriority = statusPriority[b.operationalStatus] || 0
        
        if (aPriority !== bPriority) {
          return bPriority - aPriority
        }
        
        return b.lastSeen.getTime() - a.lastSeen.getTime()
      })
  }, [vehicles, filters])

  const filteredAlerts = useMemo(() => {
    return alerts
      .filter(a => !filters.alertLevel || a.severity === filters.alertLevel)
      .filter(a => !selectedVehicle || a.vehicleId === selectedVehicle)
      .sort((a, b) => {
        if (!a.acknowledged && b.acknowledged) return -1
        if (a.acknowledged && !b.acknowledged) return 1
        return b.timestamp.getTime() - a.timestamp.getTime()
      })
  }, [alerts, filters.alertLevel, selectedVehicle])

  // Handlers
  const handleVehicleSelect = useCallback((vehicleId: string) => {
    setSelectedVehicle(vehicleId)
    onVehicleSelected?.(vehicleId)
  }, [onVehicleSelected])

  const handlePlayPause = useCallback(() => {
    setIsPlaying(!isPlaying)
  }, [isPlaying])

  const handleAlertAcknowledge = useCallback((alertId: string) => {
    setAlerts(prev => prev.map(alert => 
      alert.id === alertId 
        ? { ...alert, acknowledged: true }
        : alert
    ))
  }, [])

  const handleAlertResolve = useCallback((alertId: string) => {
    setAlerts(prev => prev.map(alert => 
      alert.id === alertId 
        ? { ...alert, resolved: true, resolvedAt: new Date() }
        : alert
    ))
  }, [])

  // Helper functions
  const getStatusColor = (status: string) => {
    switch (status) {
      case 'operational': return 'bg-green-100 text-green-800'
      case 'warning': return 'bg-yellow-100 text-yellow-800'
      case 'critical': return 'bg-red-100 text-red-800'
      case 'offline': return 'bg-gray-100 text-gray-800'
      case 'maintenance': return 'bg-blue-100 text-blue-800'
      default: return 'bg-gray-100 text-gray-800'
    }
  }

  const getStatusIcon = (status: string) => {
    switch (status) {
      case 'operational': return <CheckCircle className="w-4 h-4 text-green-500" />
      case 'warning': return <AlertTriangle className="w-4 h-4 text-yellow-500" />
      case 'critical': return <XCircle className="w-4 h-4 text-red-500" />
      case 'offline': return <WifiOff className="w-4 h-4 text-gray-500" />
      case 'maintenance': return <Clock className="w-4 h-4 text-blue-500" />
      default: return <Activity className="w-4 h-4 text-gray-500" />
    }
  }

  const getConnectionIcon = (quality: string) => {
    switch (quality) {
      case 'excellent': return <Signal className="w-4 h-4 text-green-500" />
      case 'good': return <Signal className="w-4 h-4 text-blue-500" />
      case 'fair': return <Signal className="w-4 h-4 text-yellow-500" />
      case 'poor': return <Signal className="w-4 h-4 text-red-500" />
      case 'disconnected': return <WifiOff className="w-4 h-4 text-gray-500" />
      default: return <Signal className="w-4 h-4 text-gray-500" />
    }
  }

  const getSeverityColor = (severity: string) => {
    switch (severity) {
      case 'critical': return 'bg-red-100 text-red-800 border-red-200'
      case 'error': return 'bg-red-100 text-red-800 border-red-200'
      case 'warning': return 'bg-yellow-100 text-yellow-800 border-yellow-200'
      case 'info': return 'bg-blue-100 text-blue-800 border-blue-200'
      default: return 'bg-gray-100 text-gray-800 border-gray-200'
    }
  }

  const formatUptime = (seconds: number) => {
    const days = Math.floor(seconds / 86400)
    const hours = Math.floor((seconds % 86400) / 3600)
    const mins = Math.floor((seconds % 3600) / 60)
    
    if (days > 0) return `${days}d ${hours}h`
    if (hours > 0) return `${hours}h ${mins}m`
    return `${mins}m`
  }

  const selectedTelemetry = selectedVehicle ? telemetryData[selectedVehicle] : null

  return (
    <div className={`space-y-6 ${className} ${isFullscreen ? 'fixed inset-0 z-50 bg-white p-6' : ''}`}>
      {/* Header */}
      <div className="flex items-center justify-between">
        <div className="flex items-center space-x-3">
          <Activity className="w-6 h-6 text-blue-600" />
          <h2 className="text-2xl font-semibold text-gray-900">Real-time Vehicle Monitoring</h2>
          {fleetId && (
            <Badge variant="outline">Fleet: {fleetId}</Badge>
          )}
          <div className="flex items-center space-x-2">
            <div className={`w-2 h-2 rounded-full ${isPlaying ? 'bg-green-500' : 'bg-red-500'}`} />
            <span className="text-sm text-gray-600">{isPlaying ? 'Live' : 'Paused'}</span>
          </div>
        </div>
        <div className="flex items-center space-x-2">
          <Button variant="outline" size="sm" onClick={handlePlayPause}>
            {isPlaying ? <Pause className="w-4 h-4" /> : <Play className="w-4 h-4" />}
          </Button>
          <Button variant="outline" size="sm" onClick={() => setShowGrid(!showGrid)}>
            {showGrid ? 'List' : 'Grid'}
          </Button>
          <Button variant="outline" size="sm" onClick={() => setIsFullscreen(!isFullscreen)}>
            {isFullscreen ? <Minimize2 className="w-4 h-4" /> : <Maximize2 className="w-4 h-4" />}
          </Button>
          <Button variant="outline" onClick={initializeMockData}>
            <RefreshCw className="w-4 h-4 mr-2" />
            Refresh
          </Button>
        </div>
      </div>

      {/* Fleet Status Overview */}
      <div className="grid grid-cols-2 md:grid-cols-6 gap-4">
        <Card className="p-4">
          <div className="text-center">
            <div className="text-2xl font-bold text-gray-900">{vehicles.length}</div>
            <div className="text-sm text-gray-600">Total Vehicles</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className="text-2xl font-bold text-green-600">{vehicles.filter(v => v.operationalStatus === 'operational').length}</div>
            <div className="text-sm text-gray-600">Operational</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className="text-2xl font-bold text-yellow-600">{vehicles.filter(v => v.operationalStatus === 'warning').length}</div>
            <div className="text-sm text-gray-600">Warning</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className="text-2xl font-bold text-red-600">{vehicles.filter(v => v.operationalStatus === 'critical').length}</div>
            <div className="text-sm text-gray-600">Critical</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className="text-2xl font-bold text-gray-600">{vehicles.filter(v => v.operationalStatus === 'offline').length}</div>
            <div className="text-sm text-gray-600">Offline</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className="text-2xl font-bold text-orange-600">{alerts.filter(a => !a.acknowledged).length}</div>
            <div className="text-sm text-gray-600">Active Alerts</div>
          </div>
        </Card>
      </div>

      <div className="grid grid-cols-1 lg:grid-cols-4 gap-6">
        {/* Left Panel - Vehicle List & Filters */}
        <div className="lg:col-span-1 space-y-4">
          {/* Filters */}
          <Card className="p-4">
            <h3 className="font-medium text-gray-900 mb-3">Filters</h3>
            <div className="space-y-3">
              <div className="flex items-center space-x-2">
                <Search className="w-4 h-4 text-gray-500" />
                <Input
                  placeholder="Search vehicles..."
                  value={filters.search}
                  onChange={(e) => setFilters(prev => ({ ...prev, search: e.target.value }))}
                  className="flex-1"
                />
              </div>
              
              <Select
                value={filters.status}
                onValueChange={(value) => setFilters(prev => ({ ...prev, status: value }))}
              >
                <option value="">All Status</option>
                <option value="operational">Operational</option>
                <option value="warning">Warning</option>
                <option value="critical">Critical</option>
                <option value="offline">Offline</option>
                <option value="maintenance">Maintenance</option>
              </Select>

              <Select
                value={filters.alertLevel}
                onValueChange={(value) => setFilters(prev => ({ ...prev, alertLevel: value }))}
              >
                <option value="">All Alerts</option>
                <option value="critical">Critical</option>
                <option value="error">Error</option>
                <option value="warning">Warning</option>
                <option value="info">Info</option>
              </Select>
            </div>
          </Card>

          {/* Vehicle List */}
          <Card className="p-4">
            <h3 className="font-medium text-gray-900 mb-3">Vehicles ({filteredVehicles.length})</h3>
            <div className="space-y-2 max-h-96 overflow-y-auto">
              {filteredVehicles.map(vehicle => (
                <div
                  key={vehicle.vehicleId}
                  className={`p-3 border rounded cursor-pointer transition-colors ${
                    selectedVehicle === vehicle.vehicleId 
                      ? 'border-blue-500 bg-blue-50' 
                      : 'border-gray-200 hover:border-gray-300'
                  }`}
                  onClick={() => handleVehicleSelect(vehicle.vehicleId)}
                >
                  <div className="flex items-center justify-between mb-2">
                    <span className="font-medium text-gray-900">{vehicle.vehicleId}</span>
                    <div className="flex items-center space-x-1">
                      {getStatusIcon(vehicle.operationalStatus)}
                      {getConnectionIcon(vehicle.connectionQuality)}
                    </div>
                  </div>
                  <div className="flex items-center justify-between text-sm">
                    <span className="text-gray-600">{vehicle.location}</span>
                    <Badge className={getStatusColor(vehicle.operationalStatus)} size="sm">
                      {vehicle.operationalStatus}
                    </Badge>
                  </div>
                  <div className="flex items-center justify-between text-xs mt-1">
                    <span className="text-gray-500">Health: {vehicle.healthScore}%</span>
                    {vehicle.alertCount > 0 && (
                      <span className="text-red-600">{vehicle.alertCount} alerts</span>
                    )}
                  </div>
                </div>
              ))}
            </div>
          </Card>
        </div>

        {/* Center Panel - Telemetry Details */}
        <div className="lg:col-span-2">
          {selectedTelemetry ? (
            <Card className="p-4">
              <div className="flex items-center justify-between mb-4">
                <h3 className="font-medium text-gray-900">Vehicle Telemetry: {selectedTelemetry.vehicleId}</h3>
                <div className="flex items-center space-x-2 text-sm text-gray-600">
                  <Clock className="w-4 h-4" />
                  <span>Last Update: {selectedTelemetry.timestamp.toLocaleTimeString()}</span>
                </div>
              </div>

              <Tabs value={activeTab} onValueChange={setActiveTab}>
                <TabsList className="grid w-full grid-cols-5">
                  <TabsTrigger value="overview">Overview</TabsTrigger>
                  <TabsTrigger value="powertrain">Powertrain</TabsTrigger>
                  <TabsTrigger value="sensors">Sensors</TabsTrigger>
                  <TabsTrigger value="compute">Compute</TabsTrigger>
                  <TabsTrigger value="safety">Safety</TabsTrigger>
                </TabsList>

                <TabsContent value="overview" className="space-y-4">
                  <div className="grid grid-cols-2 md:grid-cols-4 gap-4">
                    <div className="text-center p-3 bg-gray-50 rounded">
                      <div className="flex items-center justify-center mb-2">
                        <MapPin className="w-5 h-5 text-blue-600" />
                      </div>
                      <div className="text-sm text-gray-600">Location</div>
                      <div className="font-medium">{selectedTelemetry.location.latitude.toFixed(4)}, {selectedTelemetry.location.longitude.toFixed(4)}</div>
                    </div>
                    <div className="text-center p-3 bg-gray-50 rounded">
                      <div className="flex items-center justify-center mb-2">
                        <Gauge className="w-5 h-5 text-green-600" />
                      </div>
                      <div className="text-sm text-gray-600">Speed</div>
                      <div className="font-medium">{(selectedTelemetry.location.speed * 3.6).toFixed(1)} km/h</div>
                    </div>
                    <div className="text-center p-3 bg-gray-50 rounded">
                      <div className="flex items-center justify-center mb-2">
                        <Battery className="w-5 h-5 text-yellow-600" />
                      </div>
                      <div className="text-sm text-gray-600">Battery</div>
                      <div className="font-medium">{selectedTelemetry.powertrain.batteryLevel}%</div>
                    </div>
                    <div className="text-center p-3 bg-gray-50 rounded">
                      <div className="flex items-center justify-center mb-2">
                        <Shield className="w-5 h-5 text-purple-600" />
                      </div>
                      <div className="text-sm text-gray-600">Safety Score</div>
                      <div className="font-medium">{selectedTelemetry.autonomy.safetyScore}%</div>
                    </div>
                  </div>

                  <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
                    <div>
                      <h4 className="font-medium text-gray-900 mb-2">Autonomy Status</h4>
                      <div className="space-y-2">
                        <div className="flex justify-between">
                          <span className="text-sm text-gray-600">Mode:</span>
                          <Badge className={
                            selectedTelemetry.autonomy.mode === 'autonomous' ? 'bg-green-100 text-green-800' :
                            selectedTelemetry.autonomy.mode === 'manual' ? 'bg-red-100 text-red-800' :
                            'bg-blue-100 text-blue-800'
                          } size="sm">
                            {selectedTelemetry.autonomy.mode}
                          </Badge>
                        </div>
                        <div className="flex justify-between">
                          <span className="text-sm text-gray-600">Confidence:</span>
                          <span className="text-sm font-medium">{selectedTelemetry.autonomy.confidence}%</span>
                        </div>
                        <div className="flex justify-between">
                          <span className="text-sm text-gray-600">Decision Latency:</span>
                          <span className="text-sm font-medium">{selectedTelemetry.autonomy.decisionLatency}ms</span>
                        </div>
                        <div className="flex justify-between">
                          <span className="text-sm text-gray-600">Obstacles:</span>
                          <span className="text-sm font-medium">{selectedTelemetry.autonomy.obstacleCount}</span>
                        </div>
                      </div>
                    </div>

                    <div>
                      <h4 className="font-medium text-gray-900 mb-2">Connectivity</h4>
                      <div className="space-y-2">
                        <div className="flex justify-between">
                          <span className="text-sm text-gray-600">Cellular:</span>
                          <div className="flex items-center space-x-1">
                            <Badge className={selectedTelemetry.connectivity.cellular.connected ? 'bg-green-100 text-green-800' : 'bg-red-100 text-red-800'} size="sm">
                              {selectedTelemetry.connectivity.cellular.networkType}
                            </Badge>
                            <span className="text-xs text-gray-500">{selectedTelemetry.connectivity.cellular.signalStrength} dBm</span>
                          </div>
                        </div>
                        <div className="flex justify-between">
                          <span className="text-sm text-gray-600">Latency:</span>
                          <span className="text-sm font-medium">{selectedTelemetry.connectivity.cellular.latency}ms</span>
                        </div>
                        <div className="flex justify-between">
                          <span className="text-sm text-gray-600">V2X:</span>
                          <div className="flex items-center space-x-1">
                            <Badge className={selectedTelemetry.connectivity.v2x.connected ? 'bg-green-100 text-green-800' : 'bg-red-100 text-red-800'} size="sm">
                              {selectedTelemetry.connectivity.v2x.connected ? 'Connected' : 'Disconnected'}
                            </Badge>
                            <span className="text-xs text-gray-500">{selectedTelemetry.connectivity.v2x.neighbors} neighbors</span>
                          </div>
                        </div>
                      </div>
                    </div>
                  </div>
                </TabsContent>

                <TabsContent value="powertrain" className="space-y-4">
                  <div className="grid grid-cols-2 md:grid-cols-3 gap-4">
                    <div>
                      <h4 className="font-medium text-gray-900 mb-2">Battery Status</h4>
                      <div className="space-y-2">
                        <div className="flex justify-between items-center">
                          <span className="text-sm text-gray-600">Level:</span>
                          <div className="flex items-center space-x-2">
                            <Progress value={selectedTelemetry.powertrain.batteryLevel} className="w-16 h-2" />
                            <span className="text-sm font-medium">{selectedTelemetry.powertrain.batteryLevel}%</span>
                          </div>
                        </div>
                        <div className="flex justify-between">
                          <span className="text-sm text-gray-600">Health:</span>
                          <span className="text-sm font-medium">{selectedTelemetry.powertrain.batteryHealth}%</span>
                        </div>
                        <div className="flex justify-between">
                          <span className="text-sm text-gray-600">Temperature:</span>
                          <span className="text-sm font-medium">{selectedTelemetry.powertrain.batteryTemperature}°C</span>
                        </div>
                        <div className="flex justify-between">
                          <span className="text-sm text-gray-600">Voltage:</span>
                          <span className="text-sm font-medium">{selectedTelemetry.powertrain.batteryVoltage}V</span>
                        </div>
                      </div>
                    </div>

                    <div>
                      <h4 className="font-medium text-gray-900 mb-2">Energy</h4>
                      <div className="space-y-2">
                        <div className="flex justify-between">
                          <span className="text-sm text-gray-600">Est. Range:</span>
                          <span className="text-sm font-medium">{selectedTelemetry.powertrain.estimatedRange} km</span>
                        </div>
                        <div className="flex justify-between">
                          <span className="text-sm text-gray-600">Consumption:</span>
                          <span className="text-sm font-medium">{selectedTelemetry.powertrain.energyConsumption} kWh/100km</span>
                        </div>
                        <div className="flex justify-between">
                          <span className="text-sm text-gray-600">Regen Efficiency:</span>
                          <span className="text-sm font-medium">{selectedTelemetry.powertrain.regenerativeEfficiency}%</span>
                        </div>
                        <div className="flex justify-between">
                          <span className="text-sm text-gray-600">Status:</span>
                          <Badge className={
                            selectedTelemetry.powertrain.chargingStatus === 'charging' ? 'bg-green-100 text-green-800' :
                            selectedTelemetry.powertrain.chargingStatus === 'discharging' ? 'bg-yellow-100 text-yellow-800' :
                            'bg-gray-100 text-gray-800'
                          } size="sm">
                            {selectedTelemetry.powertrain.chargingStatus}
                          </Badge>
                        </div>
                      </div>
                    </div>
                  </div>
                </TabsContent>

                <TabsContent value="sensors" className="space-y-4">
                  <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
                    <div>
                      <h4 className="font-medium text-gray-900 mb-2">LiDAR Sensors</h4>
                      <div className="space-y-2">
                        {selectedTelemetry.sensors.lidar.map(sensor => (
                          <div key={sensor.id} className="p-2 bg-gray-50 rounded">
                            <div className="flex items-center justify-between">
                              <span className="text-sm font-medium">{sensor.name}</span>
                              <Badge className={
                                sensor.status === 'online' ? 'bg-green-100 text-green-800' :
                                sensor.status === 'degraded' ? 'bg-yellow-100 text-yellow-800' :
                                'bg-red-100 text-red-800'
                              } size="sm">
                                {sensor.status}
                              </Badge>
                            </div>
                            <div className="grid grid-cols-2 gap-2 text-xs mt-1">
                              <div>Health: {sensor.health}%</div>
                              <div>Temp: {sensor.temperature}°C</div>
                              <div>Rate: {sensor.dataRate} Hz</div>
                              <div>Errors: {sensor.errorCount}</div>
                            </div>
                          </div>
                        ))}
                      </div>
                    </div>

                    <div>
                      <h4 className="font-medium text-gray-900 mb-2">Cameras</h4>
                      <div className="space-y-2">
                        {selectedTelemetry.sensors.cameras.map(sensor => (
                          <div key={sensor.id} className="p-2 bg-gray-50 rounded">
                            <div className="flex items-center justify-between">
                              <span className="text-sm font-medium">{sensor.name}</span>
                              <Badge className={
                                sensor.status === 'online' ? 'bg-green-100 text-green-800' :
                                sensor.status === 'error' ? 'bg-red-100 text-red-800' :
                                'bg-yellow-100 text-yellow-800'
                              } size="sm">
                                {sensor.status}
                              </Badge>
                            </div>
                            <div className="grid grid-cols-2 gap-2 text-xs mt-1">
                              <div>Health: {sensor.health}%</div>
                              <div>Rate: {sensor.dataRate} FPS</div>
                              <div>Errors: {sensor.errorCount}</div>
                              <div>Cal: {sensor.calibrationStatus || 'N/A'}</div>
                            </div>
                          </div>
                        ))}
                      </div>
                    </div>

                    <div>
                      <h4 className="font-medium text-gray-900 mb-2">Navigation</h4>
                      <div className="space-y-2">
                        <div className="p-2 bg-gray-50 rounded">
                          <div className="flex items-center justify-between">
                            <span className="text-sm font-medium">{selectedTelemetry.sensors.gnss.name}</span>
                            <Badge className={
                              selectedTelemetry.sensors.gnss.status === 'online' ? 'bg-green-100 text-green-800' :
                              'bg-yellow-100 text-yellow-800'
                            } size="sm">
                              {selectedTelemetry.sensors.gnss.status}
                            </Badge>
                          </div>
                          <div className="grid grid-cols-2 gap-2 text-xs mt-1">
                            <div>Health: {selectedTelemetry.sensors.gnss.health}%</div>
                            <div>Rate: {selectedTelemetry.sensors.gnss.dataRate} Hz</div>
                            <div>Accuracy: {selectedTelemetry.location.accuracy}m</div>
                            <div>Satellites: {8 + Math.floor(Math.random() * 8)}</div>
                          </div>
                        </div>

                        <div className="p-2 bg-gray-50 rounded">
                          <div className="flex items-center justify-between">
                            <span className="text-sm font-medium">{selectedTelemetry.sensors.imu.name}</span>
                            <Badge className="bg-green-100 text-green-800" size="sm">
                              {selectedTelemetry.sensors.imu.status}
                            </Badge>
                          </div>
                          <div className="grid grid-cols-2 gap-2 text-xs mt-1">
                            <div>Health: {selectedTelemetry.sensors.imu.health}%</div>
                            <div>Temp: {selectedTelemetry.sensors.imu.temperature}°C</div>
                            <div>Rate: {selectedTelemetry.sensors.imu.dataRate} Hz</div>
                            <div>Errors: {selectedTelemetry.sensors.imu.errorCount}</div>
                          </div>
                        </div>
                      </div>
                    </div>
                  </div>
                </TabsContent>

                <TabsContent value="compute" className="space-y-4">
                  <div className="grid grid-cols-2 md:grid-cols-3 gap-4">
                    <div>
                      <h4 className="font-medium text-gray-900 mb-2">CPU & Memory</h4>
                      <div className="space-y-2">
                        <div className="flex justify-between items-center">
                          <span className="text-sm text-gray-600">CPU Usage:</span>
                          <div className="flex items-center space-x-2">
                            <Progress value={selectedTelemetry.compute.cpuUsage} className="w-16 h-2" />
                            <span className="text-sm font-medium">{selectedTelemetry.compute.cpuUsage}%</span>
                          </div>
                        </div>
                        <div className="flex justify-between items-center">
                          <span className="text-sm text-gray-600">Memory:</span>
                          <div className="flex items-center space-x-2">
                            <Progress value={selectedTelemetry.compute.memoryUsage} className="w-16 h-2" />
                            <span className="text-sm font-medium">{selectedTelemetry.compute.memoryUsage}%</span>
                          </div>
                        </div>
                        <div className="flex justify-between items-center">
                          <span className="text-sm text-gray-600">GPU:</span>
                          <div className="flex items-center space-x-2">
                            <Progress value={selectedTelemetry.compute.gpuUsage} className="w-16 h-2" />
                            <span className="text-sm font-medium">{selectedTelemetry.compute.gpuUsage}%</span>
                          </div>
                        </div>
                        <div className="flex justify-between">
                          <span className="text-sm text-gray-600">Temperature:</span>
                          <span className="text-sm font-medium">{selectedTelemetry.compute.temperature}°C</span>
                        </div>
                      </div>
                    </div>

                    <div>
                      <h4 className="font-medium text-gray-900 mb-2">Network</h4>
                      <div className="space-y-2">
                        <div className="flex justify-between">
                          <span className="text-sm text-gray-600">Latency:</span>
                          <span className="text-sm font-medium">{selectedTelemetry.compute.networkLatency}ms</span>
                        </div>
                        <div className="flex justify-between">
                          <span className="text-sm text-gray-600">Bandwidth:</span>
                          <span className="text-sm font-medium">{selectedTelemetry.compute.networkBandwidth} Mbps</span>
                        </div>
                        <div className="flex justify-between">
                          <span className="text-sm text-gray-600">Processes:</span>
                          <span className="text-sm font-medium">{selectedTelemetry.compute.processCount}</span>
                        </div>
                        <div className="flex justify-between">
                          <span className="text-sm text-gray-600">Uptime:</span>
                          <span className="text-sm font-medium">{formatUptime(selectedTelemetry.compute.uptime)}</span>
                        </div>
                      </div>
                    </div>

                    <div>
                      <h4 className="font-medium text-gray-900 mb-2">Storage</h4>
                      <div className="space-y-2">
                        <div className="flex justify-between items-center">
                          <span className="text-sm text-gray-600">Disk Usage:</span>
                          <div className="flex items-center space-x-2">
                            <Progress value={selectedTelemetry.compute.diskUsage} className="w-16 h-2" />
                            <span className="text-sm font-medium">{selectedTelemetry.compute.diskUsage}%</span>
                          </div>
                        </div>
                      </div>
                    </div>
                  </div>
                </TabsContent>

                <TabsContent value="safety" className="space-y-4">
                  <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
                    <div>
                      <h4 className="font-medium text-gray-900 mb-2">Safety Systems</h4>
                      <div className="space-y-2">
                        <div className="flex justify-between">
                          <span className="text-sm text-gray-600">Airbag Status:</span>
                          <Badge className={
                            selectedTelemetry.safety.airbagStatus === 'ready' ? 'bg-green-100 text-green-800' :
                            'bg-red-100 text-red-800'
                          } size="sm">
                            {selectedTelemetry.safety.airbagStatus}
                          </Badge>
                        </div>
                        <div className="flex justify-between">
                          <span className="text-sm text-gray-600">Emergency Brake:</span>
                          <Badge className={
                            selectedTelemetry.safety.emergencyBrakeStatus === 'ready' ? 'bg-green-100 text-green-800' :
                            'bg-red-100 text-red-800'
                          } size="sm">
                            {selectedTelemetry.safety.emergencyBrakeStatus}
                          </Badge>
                        </div>
                        <div className="flex justify-between">
                          <span className="text-sm text-gray-600">Safety Score:</span>
                          <span className="text-sm font-medium">{selectedTelemetry.autonomy.safetyScore}%</span>
                        </div>
                        <div className="flex justify-between">
                          <span className="text-sm text-gray-600">Collision Risk:</span>
                          <span className={`text-sm font-medium ${
                            selectedTelemetry.safety.collisionRisk > 70 ? 'text-red-600' :
                            selectedTelemetry.safety.collisionRisk > 30 ? 'text-yellow-600' :
                            'text-green-600'
                          }`}>
                            {selectedTelemetry.safety.collisionRisk}%
                          </span>
                        </div>
                      </div>
                    </div>

                    <div>
                      <h4 className="font-medium text-gray-900 mb-2">Vehicle Status</h4>
                      <div className="space-y-2">
                        <div className="flex justify-between">
                          <span className="text-sm text-gray-600">Doors:</span>
                          <span className="text-sm font-medium">
                            {selectedTelemetry.safety.doorStatus.filter(d => d).length} open
                          </span>
                        </div>
                        <div className="flex justify-between">
                          <span className="text-sm text-gray-600">Seatbelts:</span>
                          <span className="text-sm font-medium">
                            {selectedTelemetry.safety.seatbeltStatus.filter(s => s).length} fastened
                          </span>
                        </div>
                        <div className="flex justify-between">
                          <span className="text-sm text-gray-600">Geofence:</span>
                          <Badge className={
                            selectedTelemetry.safety.geofenceStatus === 'inside' ? 'bg-green-100 text-green-800' :
                            selectedTelemetry.safety.geofenceStatus === 'outside' ? 'bg-red-100 text-red-800' :
                            'bg-yellow-100 text-yellow-800'
                          } size="sm">
                            {selectedTelemetry.safety.geofenceStatus}
                          </Badge>
                        </div>
                      </div>
                    </div>
                  </div>

                  {/* Fault Codes */}
                  {selectedTelemetry.diagnostics.faultCodes.length > 0 && (
                    <div>
                      <h4 className="font-medium text-gray-900 mb-2">Active Fault Codes</h4>
                      <div className="space-y-2">
                        {selectedTelemetry.diagnostics.faultCodes.map(fault => (
                          <div key={fault.code} className={`p-2 border rounded ${getSeverityColor(fault.severity)}`}>
                            <div className="flex items-center justify-between">
                              <span className="text-sm font-medium">{fault.code}</span>
                              <Badge className={getSeverityColor(fault.severity)} size="sm">
                                {fault.severity}
                              </Badge>
                            </div>
                            <p className="text-sm mt-1">{fault.description}</p>
                            <div className="flex justify-between text-xs text-gray-500 mt-1">
                              <span>{fault.system}</span>
                              <span>Count: {fault.count}</span>
                            </div>
                          </div>
                        ))}
                      </div>
                    </div>
                  )}

                  {/* Warning Lights */}
                  {selectedTelemetry.diagnostics.warningLights.length > 0 && (
                    <div>
                      <h4 className="font-medium text-gray-900 mb-2">Warning Lights</h4>
                      <div className="grid grid-cols-2 gap-2">
                        {selectedTelemetry.diagnostics.warningLights.map(light => (
                          <div key={light.id} className="p-2 bg-gray-50 rounded">
                            <div className="flex items-center justify-between">
                              <span className="text-sm font-medium">{light.name}</span>
                              <div className={`w-3 h-3 rounded-full ${
                                light.color === 'red' ? 'bg-red-500' :
                                light.color === 'yellow' ? 'bg-yellow-500' :
                                light.color === 'green' ? 'bg-green-500' :
                                'bg-blue-500'
                              } ${light.status === 'blinking' ? 'animate-pulse' : ''}`} />
                            </div>
                            <p className="text-xs text-gray-600 mt-1">{light.description}</p>
                          </div>
                        ))}
                      </div>
                    </div>
                  )}
                </TabsContent>
              </Tabs>
            </Card>
          ) : (
            <Card className="p-4 h-96 flex items-center justify-center">
              <div className="text-center">
                <Activity className="w-16 h-16 text-gray-400 mx-auto mb-4" />
                <h3 className="text-lg font-medium text-gray-900 mb-2">No Vehicle Selected</h3>
                <p className="text-gray-600">
                  Select a vehicle from the list to view real-time telemetry data
                </p>
              </div>
            </Card>
          )}
        </div>

        {/* Right Panel - Alerts */}
        <div className="lg:col-span-1">
          <Card className="p-4">
            <div className="flex items-center justify-between mb-4">
              <h3 className="font-medium text-gray-900">Active Alerts</h3>
              <Badge variant="outline">{filteredAlerts.length}</Badge>
            </div>
            
            <div className="space-y-3 max-h-96 overflow-y-auto">
              {filteredAlerts.length === 0 ? (
                <div className="text-center text-gray-500 py-8">
                  <CheckCircle className="w-12 h-12 text-green-500 mx-auto mb-2" />
                  <p>No active alerts</p>
                </div>
              ) : (
                filteredAlerts.map(alert => (
                  <div
                    key={alert.id}
                    className={`p-3 border rounded ${getSeverityColor(alert.severity)} ${
                      alert.acknowledged ? 'opacity-60' : ''
                    }`}
                  >
                    <div className="flex items-start justify-between mb-2">
                      <div>
                        <h4 className="font-medium text-sm">{alert.title}</h4>
                        <p className="text-xs text-gray-600">{alert.vehicleId}</p>
                      </div>
                      <Badge className={getSeverityColor(alert.severity)} size="sm">
                        {alert.severity}
                      </Badge>
                    </div>
                    
                    <p className="text-sm mb-2">{alert.description}</p>
                    
                    <div className="flex items-center justify-between text-xs text-gray-500">
                      <span>{alert.timestamp.toLocaleTimeString()}</span>
                      <div className="flex items-center space-x-1">
                        {!alert.acknowledged && (
                          <Button
                            variant="outline"
                            size="sm"
                            onClick={() => handleAlertAcknowledge(alert.id)}
                            className="text-xs px-2 py-1 h-6"
                          >
                            Ack
                          </Button>
                        )}
                        {alert.acknowledged && !alert.resolved && (
                          <Button
                            variant="outline"
                            size="sm"
                            onClick={() => handleAlertResolve(alert.id)}
                            className="text-xs px-2 py-1 h-6"
                          >
                            Resolve
                          </Button>
                        )}
                      </div>
                    </div>
                    
                    {alert.relatedAlerts.length > 0 && (
                      <div className="mt-2 text-xs text-gray-600">
                        Related: {alert.relatedAlerts.length} alert(s)
                      </div>
                    )}
                  </div>
                ))
              )}
            </div>
          </Card>
        </div>
      </div>
    </div>
  )
}

export default RealTimeVehicleMonitoring
