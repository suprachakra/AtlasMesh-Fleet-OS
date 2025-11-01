import React, { useState, useCallback, useEffect, useMemo } from 'react'
import { 
  AlertTriangle, Clock, User, Car, Building, CloudRain, Wifi, Shield,
  CheckCircle, XCircle, AlertCircle, Info, MapPin, Calendar, Gauge,
  RefreshCw, Eye, Settings, Filter, Zap, Thermometer, Battery
} from 'lucide-react'
import { Button } from '../ui/Button'
import { Badge } from '../ui/Badge'
import { Card } from '../ui/Card'
import { Dialog, DialogContent, DialogHeader, DialogTitle } from '../ui/Dialog'
import { Alert, AlertDescription } from '../ui/Alert'
import { Progress } from '../ui/Progress'
import { Tabs, TabsContent, TabsList, TabsTrigger } from '../ui/Tabs'

// Types
interface ConflictCheck {
  id: string
  category: 'driver' | 'vehicle' | 'depot' | 'weather' | 'odd' | 'version' | 'regulatory'
  type: string
  severity: 'info' | 'warning' | 'error' | 'critical'
  status: 'pass' | 'fail' | 'warning' | 'checking'
  title: string
  description: string
  details: string
  resolution?: string
  affectedTrips: string[]
  checkTime: Date
  estimatedImpact: {
    delayMinutes?: number
    costIncrease?: number
    riskLevel: 'low' | 'medium' | 'high' | 'critical'
  }
}

interface TripSchedule {
  id: string
  vehicleId: string
  vehicleName: string
  driverId?: string
  driverName?: string
  startTime: Date
  endTime: Date
  estimatedDuration: number
  route: {
    origin: string
    destination: string
    distance: number
    estimatedTime: number
  }
  requirements: {
    weatherConditions: string[]
    oddZones: string[]
    minimumBatteryLevel: number
    requiredVersions: Record<string, string>
  }
  priority: 'low' | 'medium' | 'high' | 'critical'
  status: 'scheduled' | 'confirmed' | 'in_progress' | 'completed' | 'cancelled'
}

interface ConflictCheckerProps {
  tripSchedules: TripSchedule[]
  onConflictResolved?: (conflictId: string, resolution: string) => void
  onTripRescheduled?: (tripId: string, newSchedule: Partial<TripSchedule>) => void
  autoRefresh?: boolean
  className?: string
}

const ConflictChecker: React.FC<ConflictCheckerProps> = ({
  tripSchedules,
  onConflictResolved,
  onTripRescheduled,
  autoRefresh = true,
  className = ''
}) => {
  // State
  const [conflicts, setConflicts] = useState<ConflictCheck[]>([])
  const [isChecking, setIsChecking] = useState(false)
  const [showDetailDialog, setShowDetailDialog] = useState(false)
  const [selectedConflict, setSelectedConflict] = useState<ConflictCheck | null>(null)
  const [activeTab, setActiveTab] = useState('overview')
  const [filters, setFilters] = useState({
    category: '',
    severity: '',
    status: ''
  })

  // Mock external data sources
  const mockDriverSchedules = [
    { driverId: 'driver-001', name: 'Ahmed Al-Mansouri', hoursThisWeek: 35, maxHoursPerWeek: 40, shiftEnd: new Date(Date.now() + 4 * 60 * 60 * 1000) },
    { driverId: 'driver-002', name: 'Sarah Johnson', hoursThisWeek: 38, maxHoursPerWeek: 40, shiftEnd: new Date(Date.now() + 6 * 60 * 60 * 1000) },
    { driverId: 'driver-003', name: 'Mohamed Hassan', hoursThisWeek: 42, maxHoursPerWeek: 40, shiftEnd: new Date(Date.now() + 2 * 60 * 60 * 1000) }
  ]

  const mockVehicleStatus = [
    { vehicleId: 'v1', batteryLevel: 85, maintenanceStatus: 'good', lastServiceDate: new Date(Date.now() - 7 * 24 * 60 * 60 * 1000), nextServiceDue: new Date(Date.now() + 23 * 24 * 60 * 60 * 1000) },
    { vehicleId: 'v2', batteryLevel: 45, maintenanceStatus: 'warning', lastServiceDate: new Date(Date.now() - 14 * 24 * 60 * 60 * 1000), nextServiceDue: new Date(Date.now() + 16 * 24 * 60 * 60 * 1000) },
    { vehicleId: 'v3', batteryLevel: 92, maintenanceStatus: 'good', lastServiceDate: new Date(Date.now() - 3 * 24 * 60 * 60 * 1000), nextServiceDue: new Date(Date.now() + 27 * 24 * 60 * 60 * 1000) }
  ]

  const mockDepotCapacity = {
    maxConcurrentVehicles: 50,
    currentOccupancy: 35,
    chargingStations: { total: 20, available: 8 },
    maintenanceBays: { total: 5, available: 2 }
  }

  const mockWeatherConditions = {
    current: { condition: 'clear', temperature: 28, windSpeed: 15, visibility: 10 },
    forecast: [
      { time: new Date(Date.now() + 2 * 60 * 60 * 1000), condition: 'rain', temperature: 25, windSpeed: 25, visibility: 5 },
      { time: new Date(Date.now() + 4 * 60 * 60 * 1000), condition: 'heavy_rain', temperature: 23, windSpeed: 35, visibility: 2 },
      { time: new Date(Date.now() + 6 * 60 * 60 * 1000), condition: 'clear', temperature: 26, windSpeed: 20, visibility: 10 }
    ]
  }

  const mockODDZones = [
    { zone: 'downtown_dubai', status: 'restricted', reason: 'construction', validUntil: new Date(Date.now() + 48 * 60 * 60 * 1000) },
    { zone: 'dubai_mall', status: 'limited', reason: 'high_traffic', validUntil: new Date(Date.now() + 8 * 60 * 60 * 1000) },
    { zone: 'airport_road', status: 'available', reason: null, validUntil: null }
  ]

  const mockVersions = {
    currentVersions: {
      binary: 'v2.1.0',
      roadGraph: 'v1.8.5',
      map: 'v3.2.1',
      lsm: 'v1.4.2',
      blacklist: 'v2.0.8'
    },
    requiredVersions: {
      binary: 'v2.1.0',
      roadGraph: 'v1.8.4',
      map: 'v3.2.0',
      lsm: 'v1.4.0',
      blacklist: 'v2.0.6'
    }
  }

  // Run conflict checks
  const runConflictChecks = useCallback(async () => {
    setIsChecking(true)
    const newConflicts: ConflictCheck[] = []

    // Driver hours checks
    for (const trip of tripSchedules) {
      if (trip.driverId) {
        const driver = mockDriverSchedules.find(d => d.driverId === trip.driverId)
        if (driver) {
          const tripDuration = (trip.endTime.getTime() - trip.startTime.getTime()) / (60 * 60 * 1000)
          const totalHours = driver.hoursThisWeek + tripDuration

          if (totalHours > driver.maxHoursPerWeek) {
            newConflicts.push({
              id: `driver-hours-${trip.id}`,
              category: 'driver',
              type: 'working_hours_exceeded',
              severity: 'error',
              status: 'fail',
              title: 'Driver Working Hours Exceeded',
              description: `Driver ${driver.name} would exceed maximum weekly working hours`,
              details: `Current hours: ${driver.hoursThisWeek}h, Trip duration: ${tripDuration.toFixed(1)}h, Total: ${totalHours.toFixed(1)}h, Max: ${driver.maxHoursPerWeek}h`,
              resolution: 'Assign different driver or reschedule trip',
              affectedTrips: [trip.id],
              checkTime: new Date(),
              estimatedImpact: {
                delayMinutes: 30,
                costIncrease: 150,
                riskLevel: 'high'
              }
            })
          } else if (totalHours > driver.maxHoursPerWeek * 0.9) {
            newConflicts.push({
              id: `driver-hours-warning-${trip.id}`,
              category: 'driver',
              type: 'working_hours_approaching',
              severity: 'warning',
              status: 'warning',
              title: 'Driver Approaching Hour Limit',
              description: `Driver ${driver.name} is approaching maximum weekly working hours`,
              details: `Current hours: ${driver.hoursThisWeek}h, After trip: ${totalHours.toFixed(1)}h, Max: ${driver.maxHoursPerWeek}h`,
              resolution: 'Monitor closely or consider backup driver',
              affectedTrips: [trip.id],
              checkTime: new Date(),
              estimatedImpact: {
                riskLevel: 'medium'
              }
            })
          }

          // Check if driver shift ends before trip completion
          if (driver.shiftEnd < trip.endTime) {
            newConflicts.push({
              id: `driver-shift-${trip.id}`,
              category: 'driver',
              type: 'shift_conflict',
              severity: 'error',
              status: 'fail',
              title: 'Driver Shift Ends During Trip',
              description: `Driver ${driver.name}'s shift ends before trip completion`,
              details: `Shift ends: ${driver.shiftEnd.toLocaleTimeString()}, Trip ends: ${trip.endTime.toLocaleTimeString()}`,
              resolution: 'Extend shift or assign different driver',
              affectedTrips: [trip.id],
              checkTime: new Date(),
              estimatedImpact: {
                delayMinutes: 45,
                costIncrease: 200,
                riskLevel: 'high'
              }
            })
          }
        }
      }
    }

    // Vehicle availability checks
    for (const trip of tripSchedules) {
      const vehicle = mockVehicleStatus.find(v => v.vehicleId === trip.vehicleId)
      if (vehicle) {
        // Battery level check
        if (vehicle.batteryLevel < trip.requirements.minimumBatteryLevel) {
          newConflicts.push({
            id: `vehicle-battery-${trip.id}`,
            category: 'vehicle',
            type: 'insufficient_battery',
            severity: 'error',
            status: 'fail',
            title: 'Insufficient Battery Level',
            description: `Vehicle ${trip.vehicleName} battery level below required minimum`,
            details: `Current: ${vehicle.batteryLevel}%, Required: ${trip.requirements.minimumBatteryLevel}%`,
            resolution: 'Charge vehicle or use different vehicle',
            affectedTrips: [trip.id],
            checkTime: new Date(),
            estimatedImpact: {
              delayMinutes: 60,
              costIncrease: 100,
              riskLevel: 'high'
            }
          })
        } else if (vehicle.batteryLevel < trip.requirements.minimumBatteryLevel + 20) {
          newConflicts.push({
            id: `vehicle-battery-warning-${trip.id}`,
            category: 'vehicle',
            type: 'low_battery_warning',
            severity: 'warning',
            status: 'warning',
            title: 'Low Battery Warning',
            description: `Vehicle ${trip.vehicleName} battery level is low`,
            details: `Current: ${vehicle.batteryLevel}%, Recommended: ${trip.requirements.minimumBatteryLevel + 20}%`,
            resolution: 'Consider charging before trip',
            affectedTrips: [trip.id],
            checkTime: new Date(),
            estimatedImpact: {
              riskLevel: 'medium'
            }
          })
        }

        // Maintenance check
        if (vehicle.maintenanceStatus === 'warning') {
          newConflicts.push({
            id: `vehicle-maintenance-${trip.id}`,
            category: 'vehicle',
            type: 'maintenance_due',
            severity: 'warning',
            status: 'warning',
            title: 'Vehicle Maintenance Due',
            description: `Vehicle ${trip.vehicleName} has pending maintenance`,
            details: `Last service: ${vehicle.lastServiceDate.toLocaleDateString()}, Next due: ${vehicle.nextServiceDue.toLocaleDateString()}`,
            resolution: 'Schedule maintenance or use different vehicle',
            affectedTrips: [trip.id],
            checkTime: new Date(),
            estimatedImpact: {
              riskLevel: 'medium'
            }
          })
        }
      }

      // Check for overlapping trips on same vehicle
      const overlappingTrips = tripSchedules.filter(otherTrip => 
        otherTrip.id !== trip.id &&
        otherTrip.vehicleId === trip.vehicleId &&
        otherTrip.status !== 'cancelled' &&
        ((trip.startTime >= otherTrip.startTime && trip.startTime < otherTrip.endTime) ||
         (trip.endTime > otherTrip.startTime && trip.endTime <= otherTrip.endTime) ||
         (trip.startTime <= otherTrip.startTime && trip.endTime >= otherTrip.endTime))
      )

      if (overlappingTrips.length > 0) {
        newConflicts.push({
          id: `vehicle-overlap-${trip.id}`,
          category: 'vehicle',
          type: 'schedule_overlap',
          severity: 'critical',
          status: 'fail',
          title: 'Vehicle Double Booking',
          description: `Vehicle ${trip.vehicleName} is scheduled for multiple trips`,
          details: `Overlapping with trips: ${overlappingTrips.map(t => t.id).join(', ')}`,
          resolution: 'Reschedule one of the conflicting trips',
          affectedTrips: [trip.id, ...overlappingTrips.map(t => t.id)],
          checkTime: new Date(),
          estimatedImpact: {
            delayMinutes: 120,
            costIncrease: 500,
            riskLevel: 'critical'
          }
        })
      }
    }

    // Depot capacity checks
    const concurrentTrips = tripSchedules.filter(trip => {
      const now = new Date()
      return trip.startTime <= now && trip.endTime >= now && trip.status !== 'cancelled'
    })

    if (concurrentTrips.length > mockDepotCapacity.maxConcurrentVehicles) {
      newConflicts.push({
        id: 'depot-capacity-exceeded',
        category: 'depot',
        type: 'capacity_exceeded',
        severity: 'error',
        status: 'fail',
        title: 'Depot Capacity Exceeded',
        description: 'Number of concurrent trips exceeds depot capacity',
        details: `Concurrent trips: ${concurrentTrips.length}, Max capacity: ${mockDepotCapacity.maxConcurrentVehicles}`,
        resolution: 'Reschedule some trips or increase capacity',
        affectedTrips: concurrentTrips.map(t => t.id),
        checkTime: new Date(),
        estimatedImpact: {
          delayMinutes: 90,
          costIncrease: 300,
          riskLevel: 'high'
        }
      })
    }

    // Weather condition checks
    for (const trip of tripSchedules) {
      const tripWeather = mockWeatherConditions.forecast.find(w => 
        Math.abs(w.time.getTime() - trip.startTime.getTime()) < 60 * 60 * 1000
      )

      if (tripWeather) {
        const isWeatherSuitable = trip.requirements.weatherConditions.includes(tripWeather.condition)
        
        if (!isWeatherSuitable) {
          const severity = tripWeather.condition.includes('heavy') ? 'error' : 'warning'
          newConflicts.push({
            id: `weather-${trip.id}`,
            category: 'weather',
            type: 'unsuitable_conditions',
            severity,
            status: severity === 'error' ? 'fail' : 'warning',
            title: 'Unsuitable Weather Conditions',
            description: `Weather conditions not suitable for scheduled trip`,
            details: `Expected: ${tripWeather.condition}, Visibility: ${tripWeather.visibility}km, Wind: ${tripWeather.windSpeed}km/h`,
            resolution: severity === 'error' ? 'Postpone trip until conditions improve' : 'Monitor conditions closely',
            affectedTrips: [trip.id],
            checkTime: new Date(),
            estimatedImpact: {
              delayMinutes: severity === 'error' ? 180 : 30,
              costIncrease: severity === 'error' ? 400 : 100,
              riskLevel: severity === 'error' ? 'critical' : 'medium'
            }
          })
        }
      }
    }

    // ODD zone checks
    for (const trip of tripSchedules) {
      for (const requiredZone of trip.requirements.oddZones) {
        const zoneStatus = mockODDZones.find(z => z.zone === requiredZone)
        
        if (zoneStatus && zoneStatus.status === 'restricted') {
          newConflicts.push({
            id: `odd-${trip.id}-${requiredZone}`,
            category: 'odd',
            type: 'zone_restricted',
            severity: 'error',
            status: 'fail',
            title: 'ODD Zone Restricted',
            description: `Required ODD zone is currently restricted`,
            details: `Zone: ${requiredZone}, Reason: ${zoneStatus.reason}, Until: ${zoneStatus.validUntil?.toLocaleString()}`,
            resolution: 'Use alternative route or wait for restriction to lift',
            affectedTrips: [trip.id],
            checkTime: new Date(),
            estimatedImpact: {
              delayMinutes: 60,
              costIncrease: 200,
              riskLevel: 'high'
            }
          })
        } else if (zoneStatus && zoneStatus.status === 'limited') {
          newConflicts.push({
            id: `odd-warning-${trip.id}-${requiredZone}`,
            category: 'odd',
            type: 'zone_limited',
            severity: 'warning',
            status: 'warning',
            title: 'ODD Zone Limited',
            description: `ODD zone has limited availability`,
            details: `Zone: ${requiredZone}, Reason: ${zoneStatus.reason}`,
            resolution: 'Monitor zone status and consider alternatives',
            affectedTrips: [trip.id],
            checkTime: new Date(),
            estimatedImpact: {
              delayMinutes: 15,
              riskLevel: 'medium'
            }
          })
        }
      }
    }

    // Version mismatch checks
    for (const trip of tripSchedules) {
      for (const [component, requiredVersion] of Object.entries(trip.requirements.requiredVersions)) {
        const currentVersion = mockVersions.currentVersions[component as keyof typeof mockVersions.currentVersions]
        
        if (currentVersion !== requiredVersion) {
          const isNewer = currentVersion > requiredVersion
          newConflicts.push({
            id: `version-${trip.id}-${component}`,
            category: 'version',
            type: 'version_mismatch',
            severity: isNewer ? 'warning' : 'error',
            status: isNewer ? 'warning' : 'fail',
            title: `${component.toUpperCase()} Version Mismatch`,
            description: `Version mismatch detected for ${component}`,
            details: `Current: ${currentVersion}, Required: ${requiredVersion}`,
            resolution: isNewer ? 'Update requirements or downgrade' : 'Update component version',
            affectedTrips: [trip.id],
            checkTime: new Date(),
            estimatedImpact: {
              delayMinutes: isNewer ? 15 : 45,
              costIncrease: isNewer ? 50 : 150,
              riskLevel: isNewer ? 'low' : 'medium'
            }
          })
        }
      }
    }

    // Simulate check delay
    await new Promise(resolve => setTimeout(resolve, 2000))
    
    setConflicts(newConflicts)
    setIsChecking(false)
  }, [tripSchedules])

  // Auto-refresh effect
  useEffect(() => {
    if (autoRefresh) {
      runConflictChecks()
      const interval = setInterval(runConflictChecks, 30000) // Check every 30 seconds
      return () => clearInterval(interval)
    }
  }, [autoRefresh, runConflictChecks])

  // Filtered conflicts
  const filteredConflicts = useMemo(() => {
    return conflicts
      .filter(conflict => !filters.category || conflict.category === filters.category)
      .filter(conflict => !filters.severity || conflict.severity === filters.severity)
      .filter(conflict => !filters.status || conflict.status === filters.status)
      .sort((a, b) => {
        // Sort by severity and then by time
        const severityOrder = { critical: 4, error: 3, warning: 2, info: 1 }
        const aSeverity = severityOrder[a.severity]
        const bSeverity = severityOrder[b.severity]
        
        if (aSeverity !== bSeverity) {
          return bSeverity - aSeverity
        }
        
        return b.checkTime.getTime() - a.checkTime.getTime()
      })
  }, [conflicts, filters])

  // Statistics
  const stats = useMemo(() => {
    const total = conflicts.length
    const critical = conflicts.filter(c => c.severity === 'critical').length
    const errors = conflicts.filter(c => c.severity === 'error').length
    const warnings = conflicts.filter(c => c.severity === 'warning').length
    const passed = conflicts.filter(c => c.status === 'pass').length
    const failed = conflicts.filter(c => c.status === 'fail').length
    
    const totalDelayMinutes = conflicts.reduce((sum, c) => sum + (c.estimatedImpact.delayMinutes || 0), 0)
    const totalCostIncrease = conflicts.reduce((sum, c) => sum + (c.estimatedImpact.costIncrease || 0), 0)
    
    return {
      total,
      critical,
      errors,
      warnings,
      passed,
      failed,
      totalDelayMinutes,
      totalCostIncrease
    }
  }, [conflicts])

  // Handlers
  const handleViewDetails = useCallback((conflict: ConflictCheck) => {
    setSelectedConflict(conflict)
    setShowDetailDialog(true)
  }, [])

  const handleResolveConflict = useCallback((conflictId: string, resolution: string) => {
    setConflicts(prev => prev.filter(c => c.id !== conflictId))
    onConflictResolved?.(conflictId, resolution)
  }, [onConflictResolved])

  const getStatusIcon = (status: ConflictCheck['status']) => {
    switch (status) {
      case 'pass': return <CheckCircle className="w-5 h-5 text-green-600" />
      case 'fail': return <XCircle className="w-5 h-5 text-red-600" />
      case 'warning': return <AlertCircle className="w-5 h-5 text-yellow-600" />
      case 'checking': return <RefreshCw className="w-5 h-5 text-blue-600 animate-spin" />
      default: return <Info className="w-5 h-5 text-gray-600" />
    }
  }

  const getSeverityColor = (severity: ConflictCheck['severity']) => {
    switch (severity) {
      case 'critical': return 'bg-red-100 text-red-800 border-red-200'
      case 'error': return 'bg-red-100 text-red-800 border-red-200'
      case 'warning': return 'bg-yellow-100 text-yellow-800 border-yellow-200'
      case 'info': return 'bg-blue-100 text-blue-800 border-blue-200'
      default: return 'bg-gray-100 text-gray-800 border-gray-200'
    }
  }

  const getCategoryIcon = (category: ConflictCheck['category']) => {
    switch (category) {
      case 'driver': return <User className="w-5 h-5" />
      case 'vehicle': return <Car className="w-5 h-5" />
      case 'depot': return <Building className="w-5 h-5" />
      case 'weather': return <CloudRain className="w-5 h-5" />
      case 'odd': return <MapPin className="w-5 h-5" />
      case 'version': return <Settings className="w-5 h-5" />
      case 'regulatory': return <Shield className="w-5 h-5" />
      default: return <AlertTriangle className="w-5 h-5" />
    }
  }

  return (
    <div className={`space-y-6 ${className}`}>
      {/* Header */}
      <div className="flex items-center justify-between">
        <div className="flex items-center space-x-3">
          <AlertTriangle className="w-6 h-6 text-orange-600" />
          <h2 className="text-2xl font-semibold text-gray-900">Scheduling Conflict Checker</h2>
          {isChecking && (
            <Badge className="bg-blue-100 text-blue-800">
              <RefreshCw className="w-3 h-3 mr-1 animate-spin" />
              Checking...
            </Badge>
          )}
        </div>
        <Button onClick={runConflictChecks} disabled={isChecking}>
          <RefreshCw className="w-4 h-4 mr-2" />
          Run Checks
        </Button>
      </div>

      {/* Statistics */}
      <div className="grid grid-cols-2 md:grid-cols-4 lg:grid-cols-8 gap-4">
        <Card className="p-4">
          <div className="text-center">
            <div className="text-2xl font-bold text-gray-900">{stats.total}</div>
            <div className="text-sm text-gray-600">Total Checks</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className={`text-2xl font-bold ${stats.critical > 0 ? 'text-red-600' : 'text-green-600'}`}>
              {stats.critical}
            </div>
            <div className="text-sm text-gray-600">Critical</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className={`text-2xl font-bold ${stats.errors > 0 ? 'text-red-600' : 'text-green-600'}`}>
              {stats.errors}
            </div>
            <div className="text-sm text-gray-600">Errors</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className={`text-2xl font-bold ${stats.warnings > 0 ? 'text-yellow-600' : 'text-green-600'}`}>
              {stats.warnings}
            </div>
            <div className="text-sm text-gray-600">Warnings</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className="text-2xl font-bold text-green-600">{stats.passed}</div>
            <div className="text-sm text-gray-600">Passed</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className={`text-2xl font-bold ${stats.failed > 0 ? 'text-red-600' : 'text-green-600'}`}>
              {stats.failed}
            </div>
            <div className="text-sm text-gray-600">Failed</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className={`text-2xl font-bold ${stats.totalDelayMinutes > 0 ? 'text-red-600' : 'text-green-600'}`}>
              {Math.round(stats.totalDelayMinutes / 60)}h
            </div>
            <div className="text-sm text-gray-600">Est. Delay</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className={`text-2xl font-bold ${stats.totalCostIncrease > 0 ? 'text-red-600' : 'text-green-600'}`}>
              ${stats.totalCostIncrease}
            </div>
            <div className="text-sm text-gray-600">Est. Cost</div>
          </div>
        </Card>
      </div>

      {/* Summary Alert */}
      {(stats.critical > 0 || stats.errors > 0) && (
        <Alert className="border-red-200 bg-red-50">
          <AlertTriangle className="w-4 h-4 text-red-600" />
          <AlertDescription className="text-red-800">
            <strong>Action Required:</strong> {stats.critical + stats.errors} critical conflicts detected that require immediate attention.
            {stats.totalDelayMinutes > 0 && (
              <span> Estimated impact: {Math.round(stats.totalDelayMinutes / 60)} hours delay, ${stats.totalCostIncrease} additional cost.</span>
            )}
          </AlertDescription>
        </Alert>
      )}

      {/* Filters */}
      <Card className="p-4">
        <div className="flex flex-wrap items-center gap-4">
          <div className="flex items-center space-x-2">
            <Filter className="w-4 h-4 text-gray-500" />
            <span className="text-sm font-medium text-gray-700">Filters:</span>
          </div>
          
          <select
            value={filters.category}
            onChange={(e) => setFilters(prev => ({ ...prev, category: e.target.value }))}
            className="px-3 py-1 border border-gray-300 rounded text-sm"
          >
            <option value="">All Categories</option>
            <option value="driver">Driver</option>
            <option value="vehicle">Vehicle</option>
            <option value="depot">Depot</option>
            <option value="weather">Weather</option>
            <option value="odd">ODD</option>
            <option value="version">Version</option>
            <option value="regulatory">Regulatory</option>
          </select>

          <select
            value={filters.severity}
            onChange={(e) => setFilters(prev => ({ ...prev, severity: e.target.value }))}
            className="px-3 py-1 border border-gray-300 rounded text-sm"
          >
            <option value="">All Severities</option>
            <option value="critical">Critical</option>
            <option value="error">Error</option>
            <option value="warning">Warning</option>
            <option value="info">Info</option>
          </select>

          <select
            value={filters.status}
            onChange={(e) => setFilters(prev => ({ ...prev, status: e.target.value }))}
            className="px-3 py-1 border border-gray-300 rounded text-sm"
          >
            <option value="">All Status</option>
            <option value="pass">Pass</option>
            <option value="fail">Fail</option>
            <option value="warning">Warning</option>
            <option value="checking">Checking</option>
          </select>
        </div>
      </Card>

      {/* Conflicts List */}
      <div className="space-y-4">
        {filteredConflicts.map(conflict => (
          <Card key={conflict.id} className={`p-4 border-l-4 ${getSeverityColor(conflict.severity)}`}>
            <div className="flex items-start justify-between">
              <div className="flex items-start space-x-3">
                {getStatusIcon(conflict.status)}
                <div className="flex items-center space-x-2">
                  {getCategoryIcon(conflict.category)}
                </div>
                <div className="flex-1">
                  <div className="flex items-center space-x-3 mb-2">
                    <h3 className="text-lg font-medium text-gray-900">{conflict.title}</h3>
                    <Badge className={getSeverityColor(conflict.severity)}>
                      {conflict.severity}
                    </Badge>
                    <Badge variant="outline" className="capitalize">
                      {conflict.category}
                    </Badge>
                  </div>
                  
                  <p className="text-gray-600 mb-2">{conflict.description}</p>
                  <p className="text-sm text-gray-500 mb-3">{conflict.details}</p>
                  
                  {conflict.resolution && (
                    <div className="p-2 bg-blue-50 rounded border border-blue-200 mb-3">
                      <div className="text-sm font-medium text-blue-900">Recommended Resolution:</div>
                      <div className="text-sm text-blue-700">{conflict.resolution}</div>
                    </div>
                  )}
                  
                  <div className="flex items-center space-x-4 text-sm text-gray-600">
                    <div className="flex items-center space-x-1">
                      <Calendar className="w-4 h-4" />
                      <span>Checked: {conflict.checkTime.toLocaleTimeString()}</span>
                    </div>
                    
                    <div className="flex items-center space-x-1">
                      <AlertTriangle className="w-4 h-4" />
                      <span>Affects {conflict.affectedTrips.length} trip(s)</span>
                    </div>
                    
                    {conflict.estimatedImpact.delayMinutes && (
                      <div className="flex items-center space-x-1">
                        <Clock className="w-4 h-4" />
                        <span>+{conflict.estimatedImpact.delayMinutes}min delay</span>
                      </div>
                    )}
                    
                    {conflict.estimatedImpact.costIncrease && (
                      <div className="flex items-center space-x-1">
                        <span>+${conflict.estimatedImpact.costIncrease} cost</span>
                      </div>
                    )}
                  </div>
                </div>
              </div>
              
              <div className="flex items-center space-x-2">
                <Button
                  variant="outline"
                  size="sm"
                  onClick={() => handleViewDetails(conflict)}
                >
                  <Eye className="w-4 h-4" />
                </Button>
                
                {conflict.resolution && (
                  <Button
                    size="sm"
                    onClick={() => handleResolveConflict(conflict.id, conflict.resolution!)}
                  >
                    Resolve
                  </Button>
                )}
              </div>
            </div>
          </Card>
        ))}
        
        {filteredConflicts.length === 0 && !isChecking && (
          <div className="text-center py-12">
            <CheckCircle className="w-16 h-16 text-green-500 mx-auto mb-4" />
            <h3 className="text-lg font-medium text-gray-900 mb-2">No Conflicts Detected</h3>
            <p className="text-gray-600">
              {conflicts.length === 0 
                ? 'All scheduling checks passed successfully'
                : 'No conflicts match your current filters'
              }
            </p>
          </div>
        )}
      </div>

      {/* Conflict Detail Dialog */}
      <Dialog open={showDetailDialog} onOpenChange={setShowDetailDialog}>
        <DialogContent className="max-w-4xl max-h-[90vh] overflow-y-auto">
          <DialogHeader>
            <DialogTitle>Conflict Details</DialogTitle>
          </DialogHeader>

          {selectedConflict && (
            <div className="space-y-6">
              {/* Basic Info */}
              <div className="grid grid-cols-2 gap-6">
                <Card className="p-4">
                  <h3 className="text-lg font-medium text-gray-900 mb-3">Conflict Information</h3>
                  <div className="space-y-2 text-sm">
                    <div className="flex justify-between">
                      <span className="text-gray-600">Title:</span>
                      <span className="font-medium">{selectedConflict.title}</span>
                    </div>
                    <div className="flex justify-between">
                      <span className="text-gray-600">Category:</span>
                      <Badge variant="outline" className="capitalize">
                        {selectedConflict.category}
                      </Badge>
                    </div>
                    <div className="flex justify-between">
                      <span className="text-gray-600">Type:</span>
                      <span className="font-medium">{selectedConflict.type.replace('_', ' ')}</span>
                    </div>
                    <div className="flex justify-between">
                      <span className="text-gray-600">Severity:</span>
                      <Badge className={getSeverityColor(selectedConflict.severity)}>
                        {selectedConflict.severity}
                      </Badge>
                    </div>
                    <div className="flex justify-between">
                      <span className="text-gray-600">Status:</span>
                      <div className="flex items-center space-x-1">
                        {getStatusIcon(selectedConflict.status)}
                        <span className="capitalize">{selectedConflict.status}</span>
                      </div>
                    </div>
                    <div className="flex justify-between">
                      <span className="text-gray-600">Checked:</span>
                      <span className="font-medium">{selectedConflict.checkTime.toLocaleString()}</span>
                    </div>
                  </div>
                </Card>

                <Card className="p-4">
                  <h3 className="text-lg font-medium text-gray-900 mb-3">Impact Assessment</h3>
                  <div className="space-y-3">
                    <div className="flex items-center justify-between">
                      <span className="text-gray-600">Risk Level:</span>
                      <Badge className={
                        selectedConflict.estimatedImpact.riskLevel === 'critical' ? 'bg-red-100 text-red-800' :
                        selectedConflict.estimatedImpact.riskLevel === 'high' ? 'bg-orange-100 text-orange-800' :
                        selectedConflict.estimatedImpact.riskLevel === 'medium' ? 'bg-yellow-100 text-yellow-800' :
                        'bg-green-100 text-green-800'
                      }>
                        {selectedConflict.estimatedImpact.riskLevel}
                      </Badge>
                    </div>
                    
                    {selectedConflict.estimatedImpact.delayMinutes && (
                      <div className="flex items-center justify-between">
                        <span className="text-gray-600">Estimated Delay:</span>
                        <span className="font-medium text-red-600">
                          {selectedConflict.estimatedImpact.delayMinutes} minutes
                        </span>
                      </div>
                    )}
                    
                    {selectedConflict.estimatedImpact.costIncrease && (
                      <div className="flex items-center justify-between">
                        <span className="text-gray-600">Cost Increase:</span>
                        <span className="font-medium text-red-600">
                          ${selectedConflict.estimatedImpact.costIncrease}
                        </span>
                      </div>
                    )}
                    
                    <div className="flex items-center justify-between">
                      <span className="text-gray-600">Affected Trips:</span>
                      <span className="font-medium">{selectedConflict.affectedTrips.length}</span>
                    </div>
                  </div>
                </Card>
              </div>

              {/* Description and Details */}
              <Card className="p-4">
                <h3 className="text-lg font-medium text-gray-900 mb-3">Description</h3>
                <p className="text-gray-600 mb-4">{selectedConflict.description}</p>
                
                <h4 className="font-medium text-gray-900 mb-2">Details</h4>
                <p className="text-sm text-gray-600">{selectedConflict.details}</p>
              </Card>

              {/* Resolution */}
              {selectedConflict.resolution && (
                <Card className="p-4 border-blue-200 bg-blue-50">
                  <h3 className="text-lg font-medium text-blue-900 mb-3">Recommended Resolution</h3>
                  <p className="text-blue-800">{selectedConflict.resolution}</p>
                </Card>
              )}

              {/* Affected Trips */}
              <Card className="p-4">
                <h3 className="text-lg font-medium text-gray-900 mb-3">Affected Trips</h3>
                <div className="space-y-2">
                  {selectedConflict.affectedTrips.map(tripId => {
                    const trip = tripSchedules.find(t => t.id === tripId)
                    return (
                      <div key={tripId} className="flex items-center justify-between p-2 bg-gray-50 rounded">
                        <div>
                          <div className="font-medium">Trip {tripId}</div>
                          {trip && (
                            <div className="text-sm text-gray-600">
                              {trip.vehicleName} • {trip.route.origin} → {trip.route.destination}
                            </div>
                          )}
                        </div>
                        {trip && (
                          <Badge className={
                            trip.priority === 'critical' ? 'bg-red-100 text-red-800' :
                            trip.priority === 'high' ? 'bg-orange-100 text-orange-800' :
                            trip.priority === 'medium' ? 'bg-yellow-100 text-yellow-800' :
                            'bg-blue-100 text-blue-800'
                          }>
                            {trip.priority} priority
                          </Badge>
                        )}
                      </div>
                    )
                  })}
                </div>
              </Card>

              {/* Actions */}
              <div className="flex justify-end space-x-3 pt-4 border-t">
                <Button variant="outline" onClick={() => setShowDetailDialog(false)}>
                  Close
                </Button>
                {selectedConflict.resolution && (
                  <Button onClick={() => {
                    handleResolveConflict(selectedConflict.id, selectedConflict.resolution!)
                    setShowDetailDialog(false)
                  }}>
                    Apply Resolution
                  </Button>
                )}
              </div>
            </div>
          )}
        </DialogContent>
      </Dialog>
    </div>
  )
}

export default ConflictChecker
