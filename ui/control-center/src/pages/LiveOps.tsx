import React, { useState, useEffect } from 'react'
import { useTranslation } from 'react-i18next'
import { Helmet } from 'react-helmet-async'
import { motion } from 'framer-motion'
import {
  MapIcon,
  TruckIcon,
  ClockIcon,
  ExclamationTriangleIcon,
  AdjustmentsHorizontalIcon,
  ViewColumnsIcon,
  Squares2X2Icon,
  ListBulletIcon,
  PlayIcon,
  PauseIcon,
  StopIcon,
  PhoneIcon,
} from '@heroicons/react/24/outline'

// Components
import { FleetMap } from '@components/fleet/FleetMap'
import { AutonomyIndicator } from '@components/autonomy/AutonomyIndicator'
import { EmergencyControls } from '@components/safety/EmergencyControls'
import { Button } from '@components/ui/Button'
import { Badge } from '@components/ui/Badge'
import { Card, CardContent, CardHeader, CardTitle } from '@components/ui/Card'
import { Tabs, TabsContent, TabsList, TabsTrigger } from '@components/ui/Tabs'
import { Input } from '@components/ui/Input'
import { Select } from '@components/ui/Select'
import { Switch } from '@components/ui/Switch'
import { DataTable } from '@components/ui/DataTable'

// Hooks
import { useFleetVehicles } from '@hooks/useFleetVehicles'
import { useActiveTrips } from '@hooks/useActiveTrips'
import { useRealtimeUpdates } from '@hooks/useRealtimeUpdates'
import { useAuth } from '@hooks/useAuth'
import { useSector } from '@hooks/useSector'

// Types
import type { Vehicle, Trip, Alert } from '@types/fleet'

interface LiveOpsProps {}

export default function LiveOps({}: LiveOpsProps) {
  const { t } = useTranslation()
  const { user, permissions } = useAuth()
  const { currentSector, sectorConfig } = useSector()

  // State
  const [selectedVehicleId, setSelectedVehicleId] = useState<string | null>(null)
  const [selectedTripId, setSelectedTripId] = useState<string | null>(null)
  const [selectedVehicles, setSelectedVehicles] = useState<string[]>([])
  const [viewMode, setViewMode] = useState<'map' | 'split' | 'table'>('map')
  const [activeTab, setActiveTab] = useState<'vehicles' | 'trips'>('vehicles')
  const [searchTerm, setSearchTerm] = useState('')
  const [statusFilter, setStatusFilter] = useState('all')
  const [autonomyFilter, setAutonomyFilter] = useState('all')
  const [showAlerts, setShowAlerts] = useState(true)
  const [showTrails, setShowTrails] = useState(false)
  const [showSensorCoverage, setShowSensorCoverage] = useState(false)
  const [timeRange, setTimeRange] = useState<'live' | '15m' | '1h' | '24h'>('live')

  // Data fetching
  const {
    data: vehicles,
    isLoading: vehiclesLoading,
    error: vehiclesError,
    refetch: refetchVehicles,
  } = useFleetVehicles({
    includeInactive: false,
    includeOffline: true,
  })

  const {
    data: trips,
    isLoading: tripsLoading,
    error: tripsError,
    refetch: refetchTrips,
  } = useActiveTrips()

  const {
    vehicles: realtimeVehicles,
    trips: realtimeTrips,
    alerts: realtimeAlerts,
    isConnected,
    lastUpdate,
  } = useRealtimeUpdates()

  // Merge real-time data with base data
  const mergedVehicles = vehicles?.map(vehicle => {
    const realtimeData = realtimeVehicles?.find(rv => rv.vehicleId === vehicle.vehicleId)
    return realtimeData ? { ...vehicle, ...realtimeData } : vehicle
  }) || []

  const mergedTrips = trips?.map(trip => {
    const realtimeData = realtimeTrips?.find(rt => rt.tripId === trip.tripId)
    return realtimeData ? { ...trip, ...realtimeData } : trip
  }) || []

  // Filter data
  const filteredVehicles = mergedVehicles.filter(vehicle => {
    const matchesSearch = vehicle.assetTag.toLowerCase().includes(searchTerm.toLowerCase()) ||
                         vehicle.model.toLowerCase().includes(searchTerm.toLowerCase())
    const matchesStatus = statusFilter === 'all' || vehicle.operationalStatus === statusFilter
    const matchesAutonomy = autonomyFilter === 'all' || vehicle.autonomyLevel === autonomyFilter
    return matchesSearch && matchesStatus && matchesAutonomy
  })

  const filteredTrips = mergedTrips.filter(trip => {
    const matchesSearch = trip.tripId.toLowerCase().includes(searchTerm.toLowerCase()) ||
                         trip.missionType.toLowerCase().includes(searchTerm.toLowerCase())
    return matchesSearch
  })

  // Get selected vehicle/trip data
  const selectedVehicle = selectedVehicleId ? 
    filteredVehicles.find(v => v.vehicleId === selectedVehicleId) : null
  const selectedTrip = selectedTripId ? 
    filteredTrips.find(t => t.tripId === selectedTripId) : null

  // Stats calculation
  const stats = {
    totalVehicles: mergedVehicles.length,
    activeVehicles: mergedVehicles.filter(v => v.operationalStatus === 'driving_av').length,
    assistanceRequired: mergedVehicles.filter(v => v.operationalStatus === 'remote_assist').length,
    safedVehicles: mergedVehicles.filter(v => v.operationalStatus === 'safed').length,
    activeTrips: mergedTrips.filter(t => t.status === 'active').length,
    criticalAlerts: realtimeAlerts?.filter(a => a.severity === 'critical' && !a.acknowledged).length || 0,
  }

  // Vehicle table columns
  const vehicleColumns = [
    {
      accessorKey: 'assetTag',
      header: t('vehicle.assetTag'),
      cell: ({ row }: any) => (
        <div className="flex items-center space-x-2">
          <button
            onClick={() => setSelectedVehicleId(row.original.vehicleId)}
            className="text-blue-600 hover:text-blue-500 font-medium"
          >
            {row.original.assetTag}
          </button>
        </div>
      ),
    },
    {
      accessorKey: 'autonomyLevel',
      header: t('vehicle.autonomy'),
      cell: ({ row }: any) => (
        <AutonomyIndicator
          level={row.original.autonomyLevel || 'L0'}
          status={row.original.autonomyStatus || 'offline'}
          confidence={row.original.autonomyConfidence}
          oddCompliance={row.original.oddCompliance}
          size="sm"
        />
      ),
    },
    {
      accessorKey: 'operationalStatus',
      header: t('vehicle.status'),
      cell: ({ row }: any) => {
        const statusColors = {
          driving_av: 'success',
          waiting: 'secondary',
          remote_assist: 'warning',
          fallback: 'warning',
          safed: 'danger',
          offline: 'secondary',
          maintenance: 'warning',
          charging: 'info',
        }
        return (
          <Badge variant={statusColors[row.original.operationalStatus as keyof typeof statusColors] as any}>
            {t(`vehicle.status.${row.original.operationalStatus}`)}
          </Badge>
        )
      },
    },
    {
      accessorKey: 'currentTrip',
      header: t('vehicle.currentTrip'),
      cell: ({ row }: any) => (
        row.original.currentTripId ? (
          <button
            onClick={() => setSelectedTripId(row.original.currentTripId)}
            className="text-blue-600 hover:text-blue-500 text-sm"
          >
            {row.original.currentTripId.slice(0, 8)}...
          </button>
        ) : (
          <span className="text-gray-400 text-sm">{t('vehicle.noTrip')}</span>
        )
      ),
    },
    {
      accessorKey: 'batteryLevel',
      header: t('vehicle.battery'),
      cell: ({ row }: any) => (
        <div className="flex items-center space-x-2">
          <div className="w-16 bg-gray-200 rounded-full h-2">
            <div
              className={`h-2 rounded-full ${
                row.original.batteryLevel > 50 ? 'bg-green-500' :
                row.original.batteryLevel > 20 ? 'bg-yellow-500' : 'bg-red-500'
              }`}
              style={{ width: `${row.original.batteryLevel || 0}%` }}
            />
          </div>
          <span className="text-sm font-mono">{row.original.batteryLevel || 0}%</span>
        </div>
      ),
    },
    {
      accessorKey: 'alerts',
      header: t('vehicle.alerts'),
      cell: ({ row }: any) => {
        const alertCount = row.original.alerts?.length || 0
        const criticalCount = row.original.alerts?.filter((a: any) => a.severity === 'critical').length || 0
        
        if (alertCount === 0) return <span className="text-gray-400">-</span>
        
        return (
          <div className="flex items-center space-x-1">
            {criticalCount > 0 && (
              <Badge variant="danger" size="sm">
                {criticalCount}
              </Badge>
            )}
            <Badge variant="secondary" size="sm">
              {alertCount}
            </Badge>
          </div>
        )
      },
    },
    {
      id: 'actions',
      header: t('actions.title'),
      cell: ({ row }: any) => (
        <div className="flex items-center space-x-1">
          {row.original.operationalStatus === 'remote_assist' && (
            <Button size="xs" variant="warning" icon={PhoneIcon}>
              {t('actions.assist')}
            </Button>
          )}
          {row.original.operationalStatus === 'driving_av' && (
            <Button size="xs" variant="secondary" icon={PauseIcon}>
              {t('actions.pause')}
            </Button>
          )}
          {row.original.operationalStatus === 'safed' && (
            <Button size="xs" variant="success" icon={PlayIcon}>
              {t('actions.resume')}
            </Button>
          )}
        </div>
      ),
    },
  ]

  // Trip table columns
  const tripColumns = [
    {
      accessorKey: 'tripId',
      header: t('trip.id'),
      cell: ({ row }: any) => (
        <button
          onClick={() => setSelectedTripId(row.original.tripId)}
          className="text-blue-600 hover:text-blue-500 font-medium"
        >
          {row.original.tripId.slice(0, 8)}...
        </button>
      ),
    },
    {
      accessorKey: 'vehicleId',
      header: t('trip.vehicle'),
      cell: ({ row }: any) => (
        <button
          onClick={() => setSelectedVehicleId(row.original.vehicleId)}
          className="text-blue-600 hover:text-blue-500"
        >
          {row.original.vehicleAssetTag || row.original.vehicleId.slice(0, 8)}
        </button>
      ),
    },
    {
      accessorKey: 'status',
      header: t('trip.status'),
      cell: ({ row }: any) => {
        const statusColors = {
          scheduled: 'secondary',
          active: 'success',
          paused: 'warning',
          completed: 'success',
          cancelled: 'secondary',
          failed: 'danger',
        }
        return (
          <Badge variant={statusColors[row.original.status as keyof typeof statusColors] as any}>
            {t(`trip.status.${row.original.status}`)}
          </Badge>
        )
      },
    },
    {
      accessorKey: 'progress',
      header: t('trip.progress'),
      cell: ({ row }: any) => {
        const progress = row.original.progress || 0
        return (
          <div className="flex items-center space-x-2">
            <div className="w-16 bg-gray-200 rounded-full h-2">
              <div
                className="bg-blue-600 h-2 rounded-full"
                style={{ width: `${progress}%` }}
              />
            </div>
            <span className="text-sm font-mono">{progress}%</span>
          </div>
        )
      },
    },
    {
      accessorKey: 'eta',
      header: t('trip.eta'),
      cell: ({ row }: any) => (
        <span className="text-sm font-mono">
          {row.original.eta ? new Date(row.original.eta).toLocaleTimeString() : '-'}
        </span>
      ),
    },
  ]

  // Handle bulk actions
  const handleBulkAction = (action: string) => {
    // Log bulk action for audit trail
    console.info(`Fleet operation: ${action} executed on ${selectedVehicles.length} vehicles`, {
      action,
      vehicleCount: selectedVehicles.length,
      timestamp: new Date().toISOString()
    })
    // Implement bulk actions
  }

  return (
    <>
      <Helmet>
        <title>{t('pages.liveOps')} - AtlasMesh Fleet OS</title>
      </Helmet>

      <div className="h-screen flex flex-col">
        {/* Header */}
        <div className="flex-shrink-0 bg-white dark:bg-gray-800 border-b border-gray-200 dark:border-gray-700 px-6 py-4">
          <div className="flex items-center justify-between">
            <div>
              <h1 className="text-2xl font-bold text-gray-900 dark:text-white flex items-center space-x-2">
                <MapIcon className="h-6 w-6" />
                <span>{t('pages.liveOps')}</span>
              </h1>
              <div className="flex items-center space-x-4 mt-1 text-sm text-gray-500">
                <span>{t('liveOps.sector', { sector: currentSector })}</span>
                <Badge variant={isConnected ? 'success' : 'danger'}>
                  {isConnected ? t('common.connected') : t('common.disconnected')}
                </Badge>
                {lastUpdate && (
                  <span>{t('liveOps.lastUpdate', { time: new Date(lastUpdate).toLocaleTimeString() })}</span>
                )}
              </div>
            </div>

            {/* View Controls */}
            <div className="flex items-center space-x-4">
              {/* Time Range */}
              <Select
                value={timeRange}
                onChange={(value) => setTimeRange(value as any)}
                options={[
                  { value: 'live', label: t('liveOps.timeRange.live') },
                  { value: '15m', label: t('liveOps.timeRange.15m') },
                  { value: '1h', label: t('liveOps.timeRange.1h') },
                  { value: '24h', label: t('liveOps.timeRange.24h') },
                ]}
              />

              {/* View Mode Toggle */}
              <div className="flex rounded-md shadow-sm">
                <Button
                  variant={viewMode === 'map' ? 'primary' : 'secondary'}
                  size="sm"
                  icon={MapIcon}
                  onClick={() => setViewMode('map')}
                  className="rounded-r-none"
                >
                  {t('liveOps.viewMode.map')}
                </Button>
                <Button
                  variant={viewMode === 'split' ? 'primary' : 'secondary'}
                  size="sm"
                  icon={ViewColumnsIcon}
                  onClick={() => setViewMode('split')}
                  className="rounded-none border-l-0"
                >
                  {t('liveOps.viewMode.split')}
                </Button>
                <Button
                  variant={viewMode === 'table' ? 'primary' : 'secondary'}
                  size="sm"
                  icon={ListBulletIcon}
                  onClick={() => setViewMode('table')}
                  className="rounded-l-none border-l-0"
                >
                  {t('liveOps.viewMode.table')}
                </Button>
              </div>
            </div>
          </div>

          {/* Stats Bar */}
          <div className="grid grid-cols-6 gap-4 mt-4">
            <div className="bg-gray-50 dark:bg-gray-700 rounded-lg p-3">
              <div className="text-2xl font-bold text-gray-900 dark:text-white">{stats.totalVehicles}</div>
              <div className="text-sm text-gray-500">{t('stats.totalVehicles')}</div>
            </div>
            <div className="bg-green-50 dark:bg-green-900/20 rounded-lg p-3">
              <div className="text-2xl font-bold text-green-600">{stats.activeVehicles}</div>
              <div className="text-sm text-green-600">{t('stats.activeVehicles')}</div>
            </div>
            <div className="bg-yellow-50 dark:bg-yellow-900/20 rounded-lg p-3">
              <div className="text-2xl font-bold text-yellow-600">{stats.assistanceRequired}</div>
              <div className="text-sm text-yellow-600">{t('stats.assistanceRequired')}</div>
            </div>
            <div className="bg-red-50 dark:bg-red-900/20 rounded-lg p-3">
              <div className="text-2xl font-bold text-red-600">{stats.safedVehicles}</div>
              <div className="text-sm text-red-600">{t('stats.safedVehicles')}</div>
            </div>
            <div className="bg-blue-50 dark:bg-blue-900/20 rounded-lg p-3">
              <div className="text-2xl font-bold text-blue-600">{stats.activeTrips}</div>
              <div className="text-sm text-blue-600">{t('stats.activeTrips')}</div>
            </div>
            <div className="bg-red-50 dark:bg-red-900/20 rounded-lg p-3">
              <div className="text-2xl font-bold text-red-600">{stats.criticalAlerts}</div>
              <div className="text-sm text-red-600">{t('stats.criticalAlerts')}</div>
            </div>
          </div>
        </div>

        {/* Main Content */}
        <div className="flex-1 flex overflow-hidden">
          {/* Map View */}
          {(viewMode === 'map' || viewMode === 'split') && (
            <div className={`${viewMode === 'split' ? 'w-2/3' : 'flex-1'} relative`}>
              <FleetMap
                vehicles={filteredVehicles}
                trips={filteredTrips}
                selectedVehicleId={selectedVehicleId}
                selectedTripId={selectedTripId}
                onVehicleSelect={setSelectedVehicleId}
                onTripSelect={setSelectedTripId}
                onBulkSelect={setSelectedVehicles}
                enableBulkSelection={permissions.includes('fleet:bulk_actions')}
                timeRange={timeRange}
                className="h-full"
                height="100%"
              />
            </div>
          )}

          {/* Side Panel or Table View */}
          <div className={`${
            viewMode === 'map' ? 'w-1/3' : 
            viewMode === 'split' ? 'w-1/3' : 
            'flex-1'
          } bg-white dark:bg-gray-800 border-l border-gray-200 dark:border-gray-700 flex flex-col`}>
            
            {/* Filters */}
            <div className="flex-shrink-0 p-4 border-b border-gray-200 dark:border-gray-700">
              <div className="space-y-3">
                <Input
                  placeholder={t('liveOps.search.placeholder')}
                  value={searchTerm}
                  onChange={(e) => setSearchTerm(e.target.value)}
                />
                
                <div className="flex space-x-2">
                  <Select
                    value={statusFilter}
                    onChange={setStatusFilter}
                    options={[
                      { value: 'all', label: t('filters.allStatuses') },
                      { value: 'driving_av', label: t('vehicle.status.driving_av') },
                      { value: 'remote_assist', label: t('vehicle.status.remote_assist') },
                      { value: 'safed', label: t('vehicle.status.safed') },
                      { value: 'offline', label: t('vehicle.status.offline') },
                    ]}
                  />
                  
                  <Select
                    value={autonomyFilter}
                    onChange={setAutonomyFilter}
                    options={[
                      { value: 'all', label: t('filters.allAutonomy') },
                      { value: 'L0', label: 'L0' },
                      { value: 'L1', label: 'L1' },
                      { value: 'L2', label: 'L2' },
                      { value: 'L3', label: 'L3' },
                      { value: 'L4', label: 'L4' },
                      { value: 'L5', label: 'L5' },
                    ]}
                  />
                </div>
              </div>
            </div>

            {/* Entity Lists */}
            <div className="flex-1 overflow-hidden">
              <Tabs value={activeTab} onValueChange={(value) => setActiveTab(value as any)}>
                <TabsList className="w-full">
                  <TabsTrigger value="vehicles" className="flex-1">
                    <TruckIcon className="h-4 w-4 mr-2" />
                    {t('liveOps.tabs.vehicles')} ({filteredVehicles.length})
                  </TabsTrigger>
                  <TabsTrigger value="trips" className="flex-1">
                    <MapIcon className="h-4 w-4 mr-2" />
                    {t('liveOps.tabs.trips')} ({filteredTrips.length})
                  </TabsTrigger>
                </TabsList>

                <TabsContent value="vehicles" className="h-full overflow-auto">
                  {viewMode === 'table' ? (
                    <DataTable
                      columns={vehicleColumns}
                      data={filteredVehicles}
                      onRowSelectionChange={setSelectedVehicles}
                    />
                  ) : (
                    <div className="space-y-2 p-4">
                      {filteredVehicles.map((vehicle) => (
                        <Card
                          key={vehicle.vehicleId}
                          className={`cursor-pointer transition-colors ${
                            selectedVehicleId === vehicle.vehicleId
                              ? 'ring-2 ring-blue-500 bg-blue-50 dark:bg-blue-900/20'
                              : 'hover:bg-gray-50 dark:hover:bg-gray-700'
                          }`}
                          onClick={() => setSelectedVehicleId(vehicle.vehicleId)}
                        >
                          <CardContent className="p-4">
                            <div className="flex items-center justify-between mb-2">
                              <h3 className="font-medium">{vehicle.assetTag}</h3>
                              <div className="flex items-center space-x-2">
                                {vehicle.alerts?.filter(a => a.severity === 'critical').length > 0 && (
                                  <ExclamationTriangleIcon className="h-4 w-4 text-red-500" />
                                )}
                              </div>
                            </div>
                            
                            <AutonomyIndicator
                              level={vehicle.autonomyLevel || 'L0'}
                              status={vehicle.autonomyStatus || 'offline'}
                              confidence={vehicle.autonomyConfidence}
                              oddCompliance={vehicle.oddCompliance}
                              size="sm"
                            />
                            
                            <div className="mt-2 text-sm text-gray-500">
                              {vehicle.currentTripId && (
                                <div>Trip: {vehicle.currentTripId.slice(0, 8)}...</div>
                              )}
                              <div>Battery: {vehicle.batteryLevel || 0}%</div>
                            </div>
                          </CardContent>
                        </Card>
                      ))}
                    </div>
                  )}
                </TabsContent>

                <TabsContent value="trips" className="h-full overflow-auto">
                  {viewMode === 'table' ? (
                    <DataTable
                      columns={tripColumns}
                      data={filteredTrips}
                    />
                  ) : (
                    <div className="space-y-2 p-4">
                      {filteredTrips.map((trip) => (
                        <Card
                          key={trip.tripId}
                          className={`cursor-pointer transition-colors ${
                            selectedTripId === trip.tripId
                              ? 'ring-2 ring-blue-500 bg-blue-50 dark:bg-blue-900/20'
                              : 'hover:bg-gray-50 dark:hover:bg-gray-700'
                          }`}
                          onClick={() => setSelectedTripId(trip.tripId)}
                        >
                          <CardContent className="p-4">
                            <div className="flex items-center justify-between mb-2">
                              <h3 className="font-medium">{trip.tripId.slice(0, 8)}...</h3>
                              <Badge variant={
                                trip.status === 'active' ? 'success' :
                                trip.status === 'paused' ? 'warning' : 'secondary'
                              }>
                                {t(`trip.status.${trip.status}`)}
                              </Badge>
                            </div>
                            
                            <div className="text-sm text-gray-500 space-y-1">
                              <div>Vehicle: {trip.vehicleAssetTag || trip.vehicleId.slice(0, 8)}</div>
                              <div>Progress: {trip.progress || 0}%</div>
                              {trip.eta && (
                                <div>ETA: {new Date(trip.eta).toLocaleTimeString()}</div>
                              )}
                            </div>
                          </CardContent>
                        </Card>
                      ))}
                    </div>
                  )}
                </TabsContent>
              </Tabs>
            </div>

            {/* Selected Entity Details */}
            {(selectedVehicle || selectedTrip) && (
              <div className="flex-shrink-0 border-t border-gray-200 dark:border-gray-700 p-4">
                {selectedVehicle && (
                  <div className="space-y-4">
                    <div>
                      <h3 className="font-medium text-lg">{selectedVehicle.assetTag}</h3>
                      <p className="text-sm text-gray-500">
                        {selectedVehicle.manufacturer} {selectedVehicle.model}
                      </p>
                    </div>
                    
                    <AutonomyIndicator
                      level={selectedVehicle.autonomyLevel || 'L0'}
                      status={selectedVehicle.autonomyStatus || 'offline'}
                      confidence={selectedVehicle.autonomyConfidence}
                      oddCompliance={selectedVehicle.oddCompliance}
                      interventionRequired={selectedVehicle.operationalStatus === 'remote_assist'}
                      showDetails
                    />
                    
                    {permissions.includes('emergency:execute') && (
                      <EmergencyControls
                        vehicleId={selectedVehicle.vehicleId}
                        tripId={selectedVehicle.currentTripId}
                        layout="vertical"
                      />
                    )}
                  </div>
                )}
                
                {selectedTrip && (
                  <div className="space-y-4">
                    <div>
                      <h3 className="font-medium text-lg">{selectedTrip.tripId.slice(0, 8)}...</h3>
                      <p className="text-sm text-gray-500">
                        {t(`trip.missions.${selectedTrip.missionType}`)}
                      </p>
                    </div>
                    
                    <div className="space-y-2 text-sm">
                      <div className="flex justify-between">
                        <span>{t('trip.progress')}</span>
                        <span>{selectedTrip.progress || 0}%</span>
                      </div>
                      <div className="w-full bg-gray-200 rounded-full h-2">
                        <div
                          className="bg-blue-600 h-2 rounded-full"
                          style={{ width: `${selectedTrip.progress || 0}%` }}
                        />
                      </div>
                    </div>
                  </div>
                )}
              </div>
            )}

            {/* Bulk Actions */}
            {selectedVehicles.length > 0 && (
              <div className="flex-shrink-0 border-t border-gray-200 dark:border-gray-700 p-4 bg-blue-50 dark:bg-blue-900/20">
                <div className="flex items-center justify-between">
                  <span className="text-sm text-blue-700 dark:text-blue-300">
                    {t('liveOps.selected', { count: selectedVehicles.length })}
                  </span>
                  <div className="flex space-x-2">
                    <Button size="sm" variant="secondary" onClick={() => handleBulkAction('pause')}>
                      {t('actions.bulkPause')}
                    </Button>
                    <Button size="sm" variant="secondary" onClick={() => handleBulkAction('maintenance')}>
                      {t('actions.bulkMaintenance')}
                    </Button>
                  </div>
                </div>
              </div>
            )}
          </div>
        </div>
      </div>
    </>
  )
}
