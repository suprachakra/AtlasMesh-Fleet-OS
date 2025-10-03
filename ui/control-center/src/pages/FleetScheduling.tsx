import React, { useState, useCallback, useEffect, useMemo } from 'react'
import { Calendar, Clock, Plus, Copy, Eye, Filter, Save, AlertTriangle, MapPin, Users, Settings } from 'lucide-react'
import { Button } from '../components/ui/Button'
import { Badge } from '../components/ui/Badge'
import { Card } from '../components/ui/Card'
import { Input } from '../components/ui/Input'
import { Select } from '../components/ui/Select'
import { Tabs, TabsContent, TabsList, TabsTrigger } from '../components/ui/Tabs'
import { Dialog, DialogContent, DialogHeader, DialogTitle } from '../components/ui/Dialog'
import { Alert, AlertDescription } from '../components/ui/Alert'
import { Tooltip, TooltipContent, TooltipProvider, TooltipTrigger } from '../components/ui/Tooltip'

// Types
interface Trip {
  id: string
  type: 'operation' | 'release'
  vehicleId: string
  vehicleName: string
  startTime: Date
  endTime: Date
  duration: number // minutes
  targetHours: number
  businessType: string
  driveType: 'autonomous' | 'manual' | 'hybrid'
  coDriver: boolean
  driverId?: string
  coDriverId?: string
  status: 'planned' | 'confirmed' | 'in_progress' | 'completed' | 'cancelled' | 'blocked'
  conflicts: ConflictType[]
  mapVersion: string
  staticMapVersion: string
  onboardFlags: string[]
  prepTime: number
  vpnRequired: boolean
  operationalNotes: string
  safetyOperatorRequired: boolean
  route?: {
    startLocation: string
    endLocation: string
    waypoints: string[]
  }
  sla: {
    pickupETA: Date
    dropETA: Date
    breached: boolean
    reason?: string
  }
}

interface ConflictType {
  type: 'driver_hours' | 'vehicle_availability' | 'depot_capacity' | 'odd_weather' | 'version_mismatch'
  severity: 'warning' | 'error' | 'critical'
  message: string
  affectedField: string
}

interface Vehicle {
  id: string
  name: string
  plate: string
  model: string
  status: 'available' | 'in_trip' | 'maintenance' | 'offline'
  autonomyLevel: string
  currentLocation: string
  depot: string
}

interface Driver {
  id: string
  name: string
  license: string
  hoursWorked: number
  maxHours: number
  status: 'available' | 'on_duty' | 'off_duty' | 'break'
}

// Mock data
const mockVehicles: Vehicle[] = [
  { id: 'v1', name: 'Atlas-001', plate: 'AV-001', model: 'Atlas Mesh L4', status: 'available', autonomyLevel: 'L4', currentLocation: 'Depot A', depot: 'A' },
  { id: 'v2', name: 'Atlas-002', plate: 'AV-002', model: 'Atlas Mesh L4', status: 'in_trip', autonomyLevel: 'L4', currentLocation: 'Route 1', depot: 'A' },
  { id: 'v3', name: 'Atlas-003', plate: 'AV-003', model: 'Atlas Mesh L5', status: 'available', autonomyLevel: 'L5', currentLocation: 'Depot B', depot: 'B' },
]

const mockDrivers: Driver[] = [
  { id: 'd1', name: 'John Smith', license: 'CDL-001', hoursWorked: 6, maxHours: 10, status: 'available' },
  { id: 'd2', name: 'Sarah Johnson', license: 'CDL-002', hoursWorked: 8, maxHours: 10, status: 'on_duty' },
  { id: 'd3', name: 'Mike Wilson', license: 'CDL-003', hoursWorked: 2, maxHours: 10, status: 'available' },
]

const FleetScheduling: React.FC = () => {
  const [currentView, setCurrentView] = useState<'daily' | 'weekly'>('daily')
  const [selectedDate, setSelectedDate] = useState(new Date())
  const [trips, setTrips] = useState<Trip[]>([])
  const [selectedTrips, setSelectedTrips] = useState<string[]>([])
  const [showAddTripDialog, setShowAddTripDialog] = useState(false)
  const [showPreviewDialog, setShowPreviewDialog] = useState(false)
  const [filters, setFilters] = useState({
    vehicle: '',
    driver: '',
    depot: '',
    businessType: '',
    driveType: '',
    status: '',
    mapVersion: ''
  })

  // Add Trip Form State
  const [tripForm, setTripForm] = useState({
    type: 'operation' as 'operation' | 'release',
    vehicleId: '',
    startTime: '',
    duration: 120,
    targetHours: 8,
    businessType: 'passenger',
    driveType: 'autonomous' as 'autonomous' | 'manual' | 'hybrid',
    coDriver: false,
    driverId: '',
    coDriverId: '',
    mapVersion: 'v2.1.0',
    staticMapVersion: 'v2.0.5',
    onboardFlags: [] as string[],
    prepTime: 15,
    vpnRequired: false,
    operationalNotes: '',
    safetyOperatorRequired: true,
    route: {
      startLocation: '',
      endLocation: '',
      waypoints: [] as string[]
    }
  })

  const [formStep, setFormStep] = useState<'basic' | 'advanced' | 'assignment'>('basic')
  const [conflicts, setConflicts] = useState<ConflictType[]>([])
  const [draggedTrip, setDraggedTrip] = useState<string | null>(null)

  // Time slots for daily view (6 AM to 10 PM in 30-minute intervals)
  const timeSlots = useMemo(() => {
    const slots = []
    for (let hour = 6; hour <= 22; hour++) {
      for (let minute = 0; minute < 60; minute += 30) {
        slots.push({
          time: `${hour.toString().padStart(2, '0')}:${minute.toString().padStart(2, '0')}`,
          timestamp: hour * 60 + minute
        })
      }
    }
    return slots
  }, [])

  // Conflict detection
  const detectConflicts = useCallback((trip: Partial<Trip>) => {
    const newConflicts: ConflictType[] = []

    // Driver hours check
    if (trip.driverId) {
      const driver = mockDrivers.find(d => d.id === trip.driverId)
      if (driver && driver.hoursWorked + (trip.duration || 0) / 60 > driver.maxHours) {
        newConflicts.push({
          type: 'driver_hours',
          severity: 'error',
          message: `Driver ${driver.name} would exceed maximum hours (${driver.maxHours}h)`,
          affectedField: 'driverId'
        })
      }
    }

    // Vehicle availability check
    if (trip.vehicleId && trip.startTime) {
      const conflictingTrips = trips.filter(t => 
        t.vehicleId === trip.vehicleId &&
        t.id !== trip.id &&
        t.status !== 'cancelled'
      )
      if (conflictingTrips.length > 0) {
        newConflicts.push({
          type: 'vehicle_availability',
          severity: 'critical',
          message: `Vehicle ${trip.vehicleId} has conflicting trips`,
          affectedField: 'vehicleId'
        })
      }
    }

    // Map version compatibility
    if (trip.mapVersion && trip.staticMapVersion) {
      const mapMajor = parseInt(trip.mapVersion.split('.')[0].replace('v', ''))
      const staticMajor = parseInt(trip.staticMapVersion.split('.')[0].replace('v', ''))
      if (Math.abs(mapMajor - staticMajor) > 0) {
        newConflicts.push({
          type: 'version_mismatch',
          severity: 'warning',
          message: 'Map and static map versions have major version mismatch',
          affectedField: 'mapVersion'
        })
      }
    }

    return newConflicts
  }, [trips])

  // Handle form changes and conflict detection
  const handleFormChange = (field: string, value: unknown) => {
    const updatedForm = { ...tripForm, [field]: value }
    setTripForm(updatedForm)
    
    // Detect conflicts in real-time
    const newConflicts = detectConflicts(updatedForm)
    setConflicts(newConflicts)
  }

  // Handle trip creation
  const handleCreateTrip = () => {
    const startDateTime = new Date(`${selectedDate.toDateString()} ${tripForm.startTime}`)
    const endDateTime = new Date(startDateTime.getTime() + tripForm.duration * 60000)

    const newTrip: Trip = {
      id: `trip_${Date.now()}`,
      type: tripForm.type,
      vehicleId: tripForm.vehicleId,
      vehicleName: mockVehicles.find(v => v.id === tripForm.vehicleId)?.name || '',
      startTime: startDateTime,
      endTime: endDateTime,
      duration: tripForm.duration,
      targetHours: tripForm.targetHours,
      businessType: tripForm.businessType,
      driveType: tripForm.driveType,
      coDriver: tripForm.coDriver,
      driverId: tripForm.driverId,
      coDriverId: tripForm.coDriverId,
      status: 'planned',
      conflicts: detectConflicts(tripForm),
      mapVersion: tripForm.mapVersion,
      staticMapVersion: tripForm.staticMapVersion,
      onboardFlags: tripForm.onboardFlags,
      prepTime: tripForm.prepTime,
      vpnRequired: tripForm.vpnRequired,
      operationalNotes: tripForm.operationalNotes,
      safetyOperatorRequired: tripForm.safetyOperatorRequired,
      route: tripForm.route,
      sla: {
        pickupETA: new Date(startDateTime.getTime() + tripForm.prepTime * 60000),
        dropETA: endDateTime,
        breached: false
      }
    }

    setTrips([...trips, newTrip])
    setShowAddTripDialog(false)
    resetForm()
  }

  const resetForm = () => {
    setTripForm({
      type: 'operation',
      vehicleId: '',
      startTime: '',
      duration: 120,
      targetHours: 8,
      businessType: 'passenger',
      driveType: 'autonomous',
      coDriver: false,
      driverId: '',
      coDriverId: '',
      mapVersion: 'v2.1.0',
      staticMapVersion: 'v2.0.5',
      onboardFlags: [],
      prepTime: 15,
      vpnRequired: false,
      operationalNotes: '',
      safetyOperatorRequired: true,
      route: {
        startLocation: '',
        endLocation: '',
        waypoints: []
      }
    })
    setFormStep('basic')
    setConflicts([])
  }

  // Handle batch operations
  const handleBatchDuplicate = () => {
    const selectedTripObjects = trips.filter(t => selectedTrips.includes(t.id))
    const duplicatedTrips = selectedTripObjects.map(trip => ({
      ...trip,
      id: `trip_${Date.now()}_${Math.random()}`,
      startTime: new Date(trip.startTime.getTime() + 24 * 60 * 60 * 1000), // Next day
      endTime: new Date(trip.endTime.getTime() + 24 * 60 * 60 * 1000),
      status: 'planned' as const,
      conflicts: []
    }))
    
    setTrips([...trips, ...duplicatedTrips])
    setSelectedTrips([])
  }

  const handlePreview = () => {
    setShowPreviewDialog(true)
  }

  // Drag and drop handlers
  const handleDragStart = (tripId: string) => {
    setDraggedTrip(tripId)
  }

  const handleDragEnd = () => {
    setDraggedTrip(null)
  }

  const handleDrop = (timeSlot: number, vehicleId: string) => {
    if (!draggedTrip) return

    const trip = trips.find(t => t.id === draggedTrip)
    if (!trip) return

    const newStartTime = new Date(selectedDate)
    newStartTime.setHours(Math.floor(timeSlot / 60), timeSlot % 60, 0, 0)
    const newEndTime = new Date(newStartTime.getTime() + trip.duration * 60000)

    const updatedTrips = trips.map(t => 
      t.id === draggedTrip 
        ? { ...t, vehicleId, startTime: newStartTime, endTime: newEndTime }
        : t
    )

    setTrips(updatedTrips)
  }

  // Get trips for a specific time slot and vehicle
  const getTripsForSlot = (timeSlot: number, vehicleId: string) => {
    return trips.filter(trip => {
      const tripStart = trip.startTime.getHours() * 60 + trip.startTime.getMinutes()
      const tripEnd = trip.endTime.getHours() * 60 + trip.endTime.getMinutes()
      return trip.vehicleId === vehicleId && 
             timeSlot >= tripStart && 
             timeSlot < tripEnd &&
             trip.startTime.toDateString() === selectedDate.toDateString()
    })
  }

  // Filter trips based on current filters
  const filteredTrips = useMemo(() => {
    return trips.filter(trip => {
      if (filters.vehicle && trip.vehicleId !== filters.vehicle) return false
      if (filters.driver && trip.driverId !== filters.driver) return false
      if (filters.businessType && trip.businessType !== filters.businessType) return false
      if (filters.driveType && trip.driveType !== filters.driveType) return false
      if (filters.status && trip.status !== filters.status) return false
      if (filters.mapVersion && trip.mapVersion !== filters.mapVersion) return false
      return true
    })
  }, [trips, filters])

  return (
    <div className="flex flex-col h-screen bg-gray-50">
      {/* Header */}
      <div className="bg-white border-b border-gray-200 px-6 py-4">
        <div className="flex items-center justify-between">
          <div className="flex items-center space-x-4">
            <Calendar className="w-6 h-6 text-blue-600" />
            <h1 className="text-2xl font-semibold text-gray-900">Fleet Scheduling</h1>
            <Badge variant="outline" className="ml-2">
              {currentView === 'daily' ? 'Daily View' : 'Weekly View'}
            </Badge>
          </div>

          <div className="flex items-center space-x-3">
            {/* View Toggle */}
            <Tabs value={currentView} onValueChange={(value) => setCurrentView(value as 'daily' | 'weekly')}>
              <TabsList>
                <TabsTrigger value="daily">Daily</TabsTrigger>
                <TabsTrigger value="weekly">Weekly</TabsTrigger>
              </TabsList>
            </Tabs>

            {/* Date Picker */}
            <Input
              type="date"
              value={selectedDate.toISOString().split('T')[0]}
              onChange={(e) => setSelectedDate(new Date(e.target.value))}
              className="w-40"
            />

            {/* Actions */}
            <Button
              variant="outline"
              onClick={handleBatchDuplicate}
              disabled={selectedTrips.length === 0}
            >
              <Copy className="w-4 h-4 mr-2" />
              Batch Duplicate
            </Button>

            <Button
              variant="outline"
              onClick={handlePreview}
            >
              <Eye className="w-4 h-4 mr-2" />
              Preview
            </Button>

            <Button
              onClick={() => setShowAddTripDialog(true)}
            >
              <Plus className="w-4 h-4 mr-2" />
              Add Trip
            </Button>
          </div>
        </div>

        {/* Filters */}
        <div className="flex items-center space-x-4 mt-4">
          <Filter className="w-4 h-4 text-gray-500" />
          <Select
            value={filters.vehicle}
            onValueChange={(value) => setFilters({ ...filters, vehicle: value })}
          >
            <option value="">All Vehicles</option>
            {mockVehicles.map(vehicle => (
              <option key={vehicle.id} value={vehicle.id}>{vehicle.name}</option>
            ))}
          </Select>

          <Select
            value={filters.driver}
            onValueChange={(value) => setFilters({ ...filters, driver: value })}
          >
            <option value="">All Drivers</option>
            {mockDrivers.map(driver => (
              <option key={driver.id} value={driver.id}>{driver.name}</option>
            ))}
          </Select>

          <Select
            value={filters.businessType}
            onValueChange={(value) => setFilters({ ...filters, businessType: value })}
          >
            <option value="">All Business Types</option>
            <option value="passenger">Passenger</option>
            <option value="cargo">Cargo</option>
            <option value="service">Service</option>
          </Select>

          <Select
            value={filters.status}
            onValueChange={(value) => setFilters({ ...filters, status: value })}
          >
            <option value="">All Status</option>
            <option value="planned">Planned</option>
            <option value="confirmed">Confirmed</option>
            <option value="in_progress">In Progress</option>
            <option value="completed">Completed</option>
            <option value="blocked">Blocked</option>
          </Select>
        </div>
      </div>

      {/* Left Rail - Trip Facets */}
      <div className="flex flex-1 overflow-hidden">
        <div className="w-64 bg-white border-r border-gray-200 overflow-y-auto">
          <div className="p-4">
            <h3 className="text-sm font-medium text-gray-900 mb-3">Trip Type</h3>
            <div className="space-y-2">
              <label className="flex items-center">
                <input type="checkbox" className="rounded border-gray-300" />
                <span className="ml-2 text-sm text-gray-700">Operation</span>
                <Badge variant="secondary" className="ml-auto">
                  {filteredTrips.filter(t => t.type === 'operation').length}
                </Badge>
              </label>
              <label className="flex items-center">
                <input type="checkbox" className="rounded border-gray-300" />
                <span className="ml-2 text-sm text-gray-700">Release</span>
                <Badge variant="secondary" className="ml-auto">
                  {filteredTrips.filter(t => t.type === 'release').length}
                </Badge>
              </label>
            </div>

            <h3 className="text-sm font-medium text-gray-900 mb-3 mt-6">Trip Status</h3>
            <div className="space-y-2">
              {['planned', 'confirmed', 'in_progress', 'completed', 'blocked'].map(status => (
                <label key={status} className="flex items-center">
                  <input type="checkbox" className="rounded border-gray-300" />
                  <span className="ml-2 text-sm text-gray-700 capitalize">{status.replace('_', ' ')}</span>
                  <Badge variant="secondary" className="ml-auto">
                    {filteredTrips.filter(t => t.status === status).length}
                  </Badge>
                </label>
              ))}
            </div>

            <h3 className="text-sm font-medium text-gray-900 mb-3 mt-6">Quick Links</h3>
            <div className="space-y-2">
              <Button variant="ghost" className="w-full justify-start text-sm">
                <MapPin className="w-4 h-4 mr-2" />
                On-board Site URL
              </Button>
              <Button variant="ghost" className="w-full justify-start text-sm">
                <MapPin className="w-4 h-4 mr-2" />
                Map Version
              </Button>
              <Button variant="ghost" className="w-full justify-start text-sm">
                <Settings className="w-4 h-4 mr-2" />
                Policy Settings
              </Button>
              <Button variant="ghost" className="w-full justify-start text-sm">
                <Users className="w-4 h-4 mr-2" />
                Operator Assignment
              </Button>
            </div>
          </div>
        </div>

        {/* Main Scheduling Canvas */}
        <div className="flex-1 overflow-auto">
          {currentView === 'daily' ? (
            <div className="p-6">
              <div className="bg-white rounded-lg border border-gray-200">
                {/* Time header */}
                <div className="grid grid-cols-[200px_1fr] border-b border-gray-200">
                  <div className="p-4 bg-gray-50 border-r border-gray-200">
                    <span className="text-sm font-medium text-gray-900">Vehicle</span>
                  </div>
                  <div className="grid grid-cols-17 gap-0">
                    {timeSlots.filter((_, index) => index % 2 === 0).map((slot, index) => (
                      <div key={slot.time} className="p-2 text-center border-r border-gray-100 last:border-r-0">
                        <span className="text-xs text-gray-600">{slot.time}</span>
                      </div>
                    ))}
                  </div>
                </div>

                {/* Vehicle rows */}
                {mockVehicles.map(vehicle => (
                  <div key={vehicle.id} className="grid grid-cols-[200px_1fr] border-b border-gray-100 last:border-b-0">
                    <div className="p-4 bg-gray-50 border-r border-gray-200">
                      <div className="flex items-center space-x-3">
                        <div className="flex-1">
                          <div className="text-sm font-medium text-gray-900">{vehicle.name}</div>
                          <div className="text-xs text-gray-500">{vehicle.plate} • {vehicle.autonomyLevel}</div>
                        </div>
                        <Badge 
                          variant={vehicle.status === 'available' ? 'success' : 
                                  vehicle.status === 'in_trip' ? 'warning' : 'secondary'}
                        >
                          {vehicle.status}
                        </Badge>
                      </div>
                    </div>
                    
                    <div className="relative grid grid-cols-34 gap-0 min-h-[80px]">
                      {timeSlots.map(slot => {
                        const tripsInSlot = getTripsForSlot(slot.timestamp, vehicle.id)
                        return (
                          <div
                            key={`${vehicle.id}-${slot.timestamp}`}
                            className="border-r border-gray-100 last:border-r-0 relative"
                            onDragOver={(e) => e.preventDefault()}
                            onDrop={() => handleDrop(slot.timestamp, vehicle.id)}
                          >
                            {tripsInSlot.map(trip => {
                              const duration = (trip.endTime.getTime() - trip.startTime.getTime()) / (1000 * 60)
                              const widthCols = Math.max(1, Math.ceil(duration / 30))
                              
                              return (
                                <TooltipProvider key={trip.id}>
                                  <Tooltip>
                                    <TooltipTrigger asChild>
                                      <div
                                        className={`absolute top-1 left-1 right-1 bottom-1 rounded px-2 py-1 cursor-move text-xs
                                          ${trip.type === 'operation' 
                                            ? 'bg-blue-100 border border-blue-300 text-blue-800' 
                                            : 'bg-green-100 border border-green-300 text-green-800'
                                          }
                                          ${trip.conflicts.length > 0 ? 'border-red-400 bg-red-50' : ''}
                                        `}
                                        style={{ 
                                          gridColumn: `span ${widthCols}`,
                                          zIndex: draggedTrip === trip.id ? 10 : 1
                                        }}
                                        draggable
                                        onDragStart={() => handleDragStart(trip.id)}
                                        onDragEnd={handleDragEnd}
                                      >
                                        <div className="font-medium truncate">
                                          {trip.businessType}
                                        </div>
                                        <div className="text-xs opacity-75">
                                          {trip.startTime.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })}
                                        </div>
                                        {trip.conflicts.length > 0 && (
                                          <AlertTriangle className="w-3 h-3 text-red-500 absolute top-1 right-1" />
                                        )}
                                      </div>
                                    </TooltipTrigger>
                                    <TooltipContent>
                                      <div className="space-y-1">
                                        <div className="font-medium">{trip.businessType}</div>
                                        <div className="text-xs">
                                          {trip.startTime.toLocaleString()} - {trip.endTime.toLocaleString()}
                                        </div>
                                        <div className="text-xs">Duration: {trip.duration}min</div>
                                        <div className="text-xs">Driver: {trip.safetyOperatorRequired ? 'Required' : 'Driverless'}</div>
                                        {trip.conflicts.length > 0 && (
                                          <div className="text-xs text-red-600">
                                            {trip.conflicts.length} conflict(s)
                                          </div>
                                        )}
                                      </div>
                                    </TooltipContent>
                                  </Tooltip>
                                </TooltipProvider>
                              )
                            })}
                          </div>
                        )
                      })}
                    </div>
                  </div>
                ))}
              </div>

              {/* Legend */}
              <div className="mt-6 flex items-center justify-center space-x-8">
                <div className="flex items-center space-x-2">
                  <div className="w-4 h-4 bg-blue-100 border border-blue-300 rounded"></div>
                  <span className="text-sm text-gray-700">Operation</span>
                </div>
                <div className="flex items-center space-x-2">
                  <div className="w-4 h-4 bg-green-100 border border-green-300 rounded"></div>
                  <span className="text-sm text-gray-700">Release</span>
                </div>
                <div className="flex items-center space-x-2">
                  <div className="w-4 h-4 bg-red-50 border border-red-400 rounded"></div>
                  <span className="text-sm text-gray-700">Conflict</span>
                </div>
              </div>
            </div>
          ) : (
            <div className="p-6">
              <div className="text-center py-12">
                <Calendar className="w-16 h-16 text-gray-400 mx-auto mb-4" />
                <h3 className="text-lg font-medium text-gray-900 mb-2">Weekly View</h3>
                <p className="text-gray-600">Weekly scheduling view coming soon</p>
              </div>
            </div>
          )}
        </div>
      </div>

      {/* Add Trip Dialog */}
      <Dialog open={showAddTripDialog} onOpenChange={setShowAddTripDialog}>
        <DialogContent className="max-w-4xl max-h-[90vh] overflow-y-auto">
          <DialogHeader>
            <DialogTitle>Add New Trip</DialogTitle>
          </DialogHeader>

          <div className="space-y-6">
            {/* Form Steps */}
            <Tabs value={formStep} onValueChange={(value) => setFormStep(value as any)}>
              <TabsList>
                <TabsTrigger value="basic">Basic</TabsTrigger>
                <TabsTrigger value="advanced">Advanced</TabsTrigger>
                <TabsTrigger value="assignment">Assignment</TabsTrigger>
              </TabsList>

              {/* Conflicts Alert */}
              {conflicts.length > 0 && (
                <Alert className="mt-4">
                  <AlertTriangle className="w-4 h-4" />
                  <AlertDescription>
                    <div className="font-medium mb-2">Conflicts detected:</div>
                    <ul className="space-y-1">
                      {conflicts.map((conflict, index) => (
                        <li key={index} className="text-sm">
                          <Badge variant={conflict.severity === 'critical' ? 'destructive' : 'warning'} className="mr-2">
                            {conflict.severity}
                          </Badge>
                          {conflict.message}
                        </li>
                      ))}
                    </ul>
                  </AlertDescription>
                </Alert>
              )}

              <TabsContent value="basic" className="space-y-4">
                <div className="grid grid-cols-2 gap-4">
                  <div>
                    <label className="block text-sm font-medium text-gray-700 mb-1">Trip Type</label>
                    <Select
                      value={tripForm.type}
                      onValueChange={(value) => handleFormChange('type', value)}
                    >
                      <option value="operation">Operation</option>
                      <option value="release">Release</option>
                    </Select>
                  </div>

                  <div>
                    <label className="block text-sm font-medium text-gray-700 mb-1">Vehicle</label>
                    <Select
                      value={tripForm.vehicleId}
                      onValueChange={(value) => handleFormChange('vehicleId', value)}
                    >
                      <option value="">Select Vehicle</option>
                      {mockVehicles.map(vehicle => (
                        <option key={vehicle.id} value={vehicle.id}>{vehicle.name} ({vehicle.plate})</option>
                      ))}
                    </Select>
                  </div>

                  <div>
                    <label className="block text-sm font-medium text-gray-700 mb-1">Start Time</label>
                    <Input
                      type="time"
                      value={tripForm.startTime}
                      onChange={(e) => handleFormChange('startTime', e.target.value)}
                    />
                  </div>

                  <div>
                    <label className="block text-sm font-medium text-gray-700 mb-1">Duration (minutes)</label>
                    <Input
                      type="number"
                      value={tripForm.duration}
                      onChange={(e) => handleFormChange('duration', parseInt(e.target.value))}
                      min="30"
                      step="30"
                    />
                  </div>

                  <div>
                    <label className="block text-sm font-medium text-gray-700 mb-1">Target Hours</label>
                    <Input
                      type="number"
                      value={tripForm.targetHours}
                      onChange={(e) => handleFormChange('targetHours', parseInt(e.target.value))}
                      min="1"
                      max="12"
                    />
                  </div>

                  <div>
                    <label className="block text-sm font-medium text-gray-700 mb-1">Business Type</label>
                    <Select
                      value={tripForm.businessType}
                      onValueChange={(value) => handleFormChange('businessType', value)}
                    >
                      <option value="passenger">Passenger</option>
                      <option value="cargo">Cargo</option>
                      <option value="service">Service</option>
                      <option value="testing">Testing</option>
                    </Select>
                  </div>

                  <div>
                    <label className="block text-sm font-medium text-gray-700 mb-1">Drive Type</label>
                    <Select
                      value={tripForm.driveType}
                      onValueChange={(value) => handleFormChange('driveType', value)}
                    >
                      <option value="autonomous">Autonomous</option>
                      <option value="manual">Manual</option>
                      <option value="hybrid">Hybrid</option>
                    </Select>
                  </div>

                  <div>
                    <label className="flex items-center space-x-2">
                      <input
                        type="checkbox"
                        checked={tripForm.coDriver}
                        onChange={(e) => handleFormChange('coDriver', e.target.checked)}
                        className="rounded border-gray-300"
                      />
                      <span className="text-sm font-medium text-gray-700">Co-Driver Required</span>
                    </label>
                  </div>
                </div>
              </TabsContent>

              <TabsContent value="advanced" className="space-y-4">
                <div className="grid grid-cols-2 gap-4">
                  <div>
                    <label className="block text-sm font-medium text-gray-700 mb-1">Map Version</label>
                    <Select
                      value={tripForm.mapVersion}
                      onValueChange={(value) => handleFormChange('mapVersion', value)}
                    >
                      <option value="v2.1.0">v2.1.0 (Latest)</option>
                      <option value="v2.0.5">v2.0.5</option>
                      <option value="v1.9.8">v1.9.8</option>
                    </Select>
                  </div>

                  <div>
                    <label className="block text-sm font-medium text-gray-700 mb-1">Static Map Version</label>
                    <Select
                      value={tripForm.staticMapVersion}
                      onValueChange={(value) => handleFormChange('staticMapVersion', value)}
                    >
                      <option value="v2.0.5">v2.0.5 (Latest)</option>
                      <option value="v2.0.0">v2.0.0</option>
                      <option value="v1.9.5">v1.9.5</option>
                    </Select>
                  </div>

                  <div>
                    <label className="block text-sm font-medium text-gray-700 mb-1">Prep Time (minutes)</label>
                    <Input
                      type="number"
                      value={tripForm.prepTime}
                      onChange={(e) => handleFormChange('prepTime', parseInt(e.target.value))}
                      min="5"
                      max="60"
                    />
                  </div>

                  <div>
                    <label className="flex items-center space-x-2">
                      <input
                        type="checkbox"
                        checked={tripForm.vpnRequired}
                        onChange={(e) => handleFormChange('vpnRequired', e.target.checked)}
                        className="rounded border-gray-300"
                      />
                      <span className="text-sm font-medium text-gray-700">VPN Required</span>
                    </label>
                  </div>

                  <div className="col-span-2">
                    <label className="block text-sm font-medium text-gray-700 mb-1">Onboard Flags</label>
                    <div className="grid grid-cols-3 gap-2">
                      {['debug_mode', 'verbose_logging', 'test_mode', 'safe_mode', 'performance_monitoring', 'data_collection'].map(flag => (
                        <label key={flag} className="flex items-center space-x-2">
                          <input
                            type="checkbox"
                            checked={tripForm.onboardFlags.includes(flag)}
                            onChange={(e) => {
                              const flags = e.target.checked 
                                ? [...tripForm.onboardFlags, flag]
                                : tripForm.onboardFlags.filter(f => f !== flag)
                              handleFormChange('onboardFlags', flags)
                            }}
                            className="rounded border-gray-300"
                          />
                          <span className="text-sm text-gray-700">{flag.replace('_', ' ')}</span>
                        </label>
                      ))}
                    </div>
                  </div>

                  <div className="col-span-2">
                    <label className="block text-sm font-medium text-gray-700 mb-1">Operational Notes</label>
                    <textarea
                      value={tripForm.operationalNotes}
                      onChange={(e) => handleFormChange('operationalNotes', e.target.value)}
                      className="w-full px-3 py-2 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-blue-500"
                      rows={3}
                      placeholder="Any special instructions or notes for this trip..."
                    />
                  </div>
                </div>
              </TabsContent>

              <TabsContent value="assignment" className="space-y-4">
                <div className="grid grid-cols-2 gap-4">
                  <div>
                    <label className="flex items-center space-x-2 mb-4">
                      <input
                        type="checkbox"
                        checked={tripForm.safetyOperatorRequired}
                        onChange={(e) => handleFormChange('safetyOperatorRequired', e.target.checked)}
                        className="rounded border-gray-300"
                      />
                      <span className="text-sm font-medium text-gray-700">Safety Operator Required (L4)</span>
                    </label>
                    
                    {!tripForm.safetyOperatorRequired && (
                      <Alert>
                        <AlertTriangle className="w-4 h-4" />
                        <AlertDescription>
                          <div className="font-medium">Driverless Mode</div>
                          <div className="text-sm mt-1">
                            This trip will operate in fully autonomous mode. Ensure all safety protocols are met:
                            <ul className="list-disc list-inside mt-2 space-y-1">
                              <li>Vehicle certified for L5 operation</li>
                              <li>Route approved for driverless operation</li>
                              <li>Emergency response protocols in place</li>
                              <li>Remote monitoring capabilities active</li>
                            </ul>
                          </div>
                        </AlertDescription>
                      </Alert>
                    )}
                  </div>

                  {tripForm.safetyOperatorRequired && (
                    <>
                      <div>
                        <label className="block text-sm font-medium text-gray-700 mb-1">Primary Driver</label>
                        <Select
                          value={tripForm.driverId}
                          onValueChange={(value) => handleFormChange('driverId', value)}
                        >
                          <option value="">Select Driver</option>
                          {mockDrivers.filter(d => d.status === 'available').map(driver => (
                            <option key={driver.id} value={driver.id}>
                              {driver.name} ({driver.hoursWorked}/{driver.maxHours}h)
                            </option>
                          ))}
                        </Select>
                      </div>

                      {tripForm.coDriver && (
                        <div>
                          <label className="block text-sm font-medium text-gray-700 mb-1">Co-Driver</label>
                          <Select
                            value={tripForm.coDriverId}
                            onValueChange={(value) => handleFormChange('coDriverId', value)}
                          >
                            <option value="">Select Co-Driver</option>
                            {mockDrivers.filter(d => d.status === 'available' && d.id !== tripForm.driverId).map(driver => (
                              <option key={driver.id} value={driver.id}>
                                {driver.name} ({driver.hoursWorked}/{driver.maxHours}h)
                              </option>
                            ))}
                          </Select>
                        </div>
                      )}
                    </>
                  )}

                  <div className="col-span-2">
                    <label className="block text-sm font-medium text-gray-700 mb-1">Route</label>
                    <div className="grid grid-cols-2 gap-2">
                      <Input
                        placeholder="Start Location"
                        value={tripForm.route.startLocation}
                        onChange={(e) => handleFormChange('route', { ...tripForm.route, startLocation: e.target.value })}
                      />
                      <Input
                        placeholder="End Location"
                        value={tripForm.route.endLocation}
                        onChange={(e) => handleFormChange('route', { ...tripForm.route, endLocation: e.target.value })}
                      />
                    </div>
                  </div>
                </div>
              </TabsContent>
            </Tabs>

            <div className="flex justify-between pt-4 border-t border-gray-200">
              <Button
                variant="outline"
                onClick={() => setShowAddTripDialog(false)}
              >
                Cancel
              </Button>

              <div className="flex space-x-3">
                {formStep !== 'basic' && (
                  <Button
                    variant="outline"
                    onClick={() => {
                      const steps = ['basic', 'advanced', 'assignment']
                      const currentIndex = steps.indexOf(formStep)
                      setFormStep(steps[currentIndex - 1] as any)
                    }}
                  >
                    Previous
                  </Button>
                )}

                {formStep !== 'assignment' ? (
                  <Button
                    onClick={() => {
                      const steps = ['basic', 'advanced', 'assignment']
                      const currentIndex = steps.indexOf(formStep)
                      setFormStep(steps[currentIndex + 1] as any)
                    }}
                    disabled={formStep === 'basic' && (!tripForm.vehicleId || !tripForm.startTime)}
                  >
                    Next
                  </Button>
                ) : (
                  <Button
                    onClick={handleCreateTrip}
                    disabled={conflicts.some(c => c.severity === 'critical') || !tripForm.vehicleId || !tripForm.startTime}
                  >
                    <Save className="w-4 h-4 mr-2" />
                    Create Trip
                  </Button>
                )}
              </div>
            </div>
          </div>
        </DialogContent>
      </Dialog>

      {/* Preview Dialog */}
      <Dialog open={showPreviewDialog} onOpenChange={setShowPreviewDialog}>
        <DialogContent className="max-w-2xl">
          <DialogHeader>
            <DialogTitle>Schedule Preview</DialogTitle>
          </DialogHeader>
          <div className="space-y-4">
            <div className="text-center py-8">
              <Eye className="w-16 h-16 text-gray-400 mx-auto mb-4" />
              <h3 className="text-lg font-medium text-gray-900 mb-2">Schedule Impact Preview</h3>
              <p className="text-gray-600">ETA/coverage/cost impact analysis coming soon</p>
            </div>
          </div>
        </DialogContent>
      </Dialog>
    </div>
  )
}

export default FleetScheduling
