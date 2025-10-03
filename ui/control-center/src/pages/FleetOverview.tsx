import React, { useState, useEffect } from 'react'
import { useTranslation } from 'react-i18next'
import { Helmet } from 'react-helmet-async'
import { motion } from 'framer-motion'
import { Link } from 'react-router-dom'
import {
  TruckIcon,
  MapPinIcon,
  BoltIcon,
  ExclamationTriangleIcon,
  CheckCircleIcon,
  ClockIcon,
  WrenchScrewdriverIcon,
  FunnelIcon,
  MagnifyingGlassIcon,
  PlusIcon,
} from '@heroicons/react/24/outline'

// Components
import { LoadingSpinner } from '@components/ui/LoadingSpinner'
import { ErrorMessage } from '@components/ui/ErrorMessage'
import { Button } from '@components/ui/Button'
import { Input } from '@components/ui/Input'
import { Select } from '@components/ui/Select'
import { Badge } from '@components/ui/Badge'
import { FleetMap } from '@components/fleet/FleetMap'

// Hooks
import { useFleetVehicles } from '@hooks/useFleetVehicles'
import { useAuth } from '@hooks/useAuth'
import { useSector } from '@hooks/useSector'

// Types
import type { Vehicle, VehicleStatus, VehicleFilter } from '@types/fleet'

const statusColors = {
  active: 'bg-green-100 text-green-800 dark:bg-green-900 dark:text-green-300',
  idle: 'bg-yellow-100 text-yellow-800 dark:bg-yellow-900 dark:text-yellow-300',
  maintenance: 'bg-red-100 text-red-800 dark:bg-red-900 dark:text-red-300',
  offline: 'bg-gray-100 text-gray-800 dark:bg-gray-900 dark:text-gray-300',
}

const statusIcons = {
  active: CheckCircleIcon,
  idle: ClockIcon,
  maintenance: WrenchScrewdriverIcon,
  offline: ExclamationTriangleIcon,
}

export default function FleetOverview() {
  const { t } = useTranslation()
  const { user, permissions } = useAuth()
  const { currentSector, sectorConfig } = useSector()

  // State
  const [searchTerm, setSearchTerm] = useState('')
  const [statusFilter, setStatusFilter] = useState<VehicleStatus | 'all'>('all')
  const [viewMode, setViewMode] = useState<'grid' | 'map'>('grid')
  const [selectedVehicles, setSelectedVehicles] = useState<string[]>([])

  // Data fetching
  const {
    data: vehicles,
    isLoading,
    error,
    refetch,
  } = useFleetVehicles({
    search: searchTerm,
    status: statusFilter === 'all' ? undefined : statusFilter,
  })

  // Filter and search logic
  const filteredVehicles = vehicles?.filter((vehicle) => {
    const matchesSearch = vehicle.assetTag.toLowerCase().includes(searchTerm.toLowerCase()) ||
                         vehicle.model.toLowerCase().includes(searchTerm.toLowerCase())
    const matchesStatus = statusFilter === 'all' || vehicle.status === statusFilter
    return matchesSearch && matchesStatus
  }) || []

  // Stats calculation
  const stats = {
    total: vehicles?.length || 0,
    active: vehicles?.filter(v => v.status === 'active').length || 0,
    idle: vehicles?.filter(v => v.status === 'idle').length || 0,
    maintenance: vehicles?.filter(v => v.status === 'maintenance').length || 0,
    offline: vehicles?.filter(v => v.status === 'offline').length || 0,
  }

  const handleVehicleSelect = (vehicleId: string) => {
    setSelectedVehicles(prev => 
      prev.includes(vehicleId) 
        ? prev.filter(id => id !== vehicleId)
        : [...prev, vehicleId]
    )
  }

  const handleBulkAction = (action: string) => {
    // Implement bulk actions (dispatch, maintenance mode, etc.)
    console.log(`Bulk action: ${action} on vehicles:`, selectedVehicles)
  }

  if (isLoading) {
    return <LoadingSpinner />
  }

  if (error) {
    return <ErrorMessage error={error} onRetry={refetch} />
  }

  return (
    <>
      <Helmet>
        <title>{t('pages.fleet')} - AtlasMesh Fleet OS</title>
      </Helmet>

      <div className="space-y-6">
        {/* Header */}
        <div className="flex flex-col sm:flex-row sm:items-center sm:justify-between">
          <div>
            <h1 className="text-2xl font-bold text-gray-900 dark:text-white">
              {t('pages.fleet')}
            </h1>
            <p className="mt-1 text-sm text-gray-500 dark:text-gray-400">
              {t('fleet.overview.description', { sector: currentSector })}
            </p>
          </div>
          
          {permissions.includes('fleet:create') && (
            <div className="mt-4 sm:mt-0">
              <Button
                variant="primary"
                size="sm"
                icon={PlusIcon}
                onClick={() => {/* Navigate to add vehicle */}}
              >
                {t('fleet.actions.addVehicle')}
              </Button>
            </div>
          )}
        </div>

        {/* Stats Cards */}
        <div className="grid grid-cols-1 gap-5 sm:grid-cols-2 lg:grid-cols-5">
          <div className="bg-white dark:bg-gray-800 overflow-hidden shadow rounded-lg">
            <div className="p-5">
              <div className="flex items-center">
                <div className="flex-shrink-0">
                  <TruckIcon className="h-6 w-6 text-gray-400" />
                </div>
                <div className="ml-5 w-0 flex-1">
                  <dl>
                    <dt className="text-sm font-medium text-gray-500 dark:text-gray-400 truncate">
                      {t('stats.totalVehicles')}
                    </dt>
                    <dd className="text-lg font-medium text-gray-900 dark:text-white">
                      {stats.total}
                    </dd>
                  </dl>
                </div>
              </div>
            </div>
          </div>

          <div className="bg-white dark:bg-gray-800 overflow-hidden shadow rounded-lg">
            <div className="p-5">
              <div className="flex items-center">
                <div className="flex-shrink-0">
                  <CheckCircleIcon className="h-6 w-6 text-green-400" />
                </div>
                <div className="ml-5 w-0 flex-1">
                  <dl>
                    <dt className="text-sm font-medium text-gray-500 dark:text-gray-400 truncate">
                      {t('stats.activeVehicles')}
                    </dt>
                    <dd className="text-lg font-medium text-gray-900 dark:text-white">
                      {stats.active}
                    </dd>
                  </dl>
                </div>
              </div>
            </div>
          </div>

          <div className="bg-white dark:bg-gray-800 overflow-hidden shadow rounded-lg">
            <div className="p-5">
              <div className="flex items-center">
                <div className="flex-shrink-0">
                  <ClockIcon className="h-6 w-6 text-yellow-400" />
                </div>
                <div className="ml-5 w-0 flex-1">
                  <dl>
                    <dt className="text-sm font-medium text-gray-500 dark:text-gray-400 truncate">
                      {t('stats.idleVehicles')}
                    </dt>
                    <dd className="text-lg font-medium text-gray-900 dark:text-white">
                      {stats.idle}
                    </dd>
                  </dl>
                </div>
              </div>
            </div>
          </div>

          <div className="bg-white dark:bg-gray-800 overflow-hidden shadow rounded-lg">
            <div className="p-5">
              <div className="flex items-center">
                <div className="flex-shrink-0">
                  <WrenchScrewdriverIcon className="h-6 w-6 text-red-400" />
                </div>
                <div className="ml-5 w-0 flex-1">
                  <dl>
                    <dt className="text-sm font-medium text-gray-500 dark:text-gray-400 truncate">
                      {t('stats.maintenanceVehicles')}
                    </dt>
                    <dd className="text-lg font-medium text-gray-900 dark:text-white">
                      {stats.maintenance}
                    </dd>
                  </dl>
                </div>
              </div>
            </div>
          </div>

          <div className="bg-white dark:bg-gray-800 overflow-hidden shadow rounded-lg">
            <div className="p-5">
              <div className="flex items-center">
                <div className="flex-shrink-0">
                  <ExclamationTriangleIcon className="h-6 w-6 text-gray-400" />
                </div>
                <div className="ml-5 w-0 flex-1">
                  <dl>
                    <dt className="text-sm font-medium text-gray-500 dark:text-gray-400 truncate">
                      {t('stats.offlineVehicles')}
                    </dt>
                    <dd className="text-lg font-medium text-gray-900 dark:text-white">
                      {stats.offline}
                    </dd>
                  </dl>
                </div>
              </div>
            </div>
          </div>
        </div>

        {/* Filters and Search */}
        <div className="bg-white dark:bg-gray-800 shadow rounded-lg">
          <div className="px-4 py-5 sm:p-6">
            <div className="flex flex-col sm:flex-row sm:items-center sm:justify-between space-y-4 sm:space-y-0 sm:space-x-4">
              <div className="flex-1 min-w-0">
                <div className="relative">
                  <div className="absolute inset-y-0 left-0 pl-3 flex items-center pointer-events-none">
                    <MagnifyingGlassIcon className="h-5 w-5 text-gray-400" />
                  </div>
                  <Input
                    type="text"
                    placeholder={t('fleet.search.placeholder')}
                    value={searchTerm}
                    onChange={(e) => setSearchTerm(e.target.value)}
                    className="pl-10"
                  />
                </div>
              </div>
              
              <div className="flex items-center space-x-4">
                <Select
                  value={statusFilter}
                  onChange={(value) => setStatusFilter(value as VehicleStatus | 'all')}
                  options={[
                    { value: 'all', label: t('fleet.filters.allStatuses') },
                    { value: 'active', label: t('vehicle.status.active') },
                    { value: 'idle', label: t('vehicle.status.idle') },
                    { value: 'maintenance', label: t('vehicle.status.maintenance') },
                    { value: 'offline', label: t('vehicle.status.offline') },
                  ]}
                />
                
                <div className="flex rounded-md shadow-sm">
                  <Button
                    variant={viewMode === 'grid' ? 'primary' : 'secondary'}
                    size="sm"
                    onClick={() => setViewMode('grid')}
                    className="rounded-r-none"
                  >
                    Grid
                  </Button>
                  <Button
                    variant={viewMode === 'map' ? 'primary' : 'secondary'}
                    size="sm"
                    onClick={() => setViewMode('map')}
                    className="rounded-l-none"
                  >
                    Map
                  </Button>
                </div>
              </div>
            </div>

            {selectedVehicles.length > 0 && (
              <div className="mt-4 flex items-center justify-between p-4 bg-blue-50 dark:bg-blue-900/20 rounded-lg">
                <span className="text-sm text-blue-700 dark:text-blue-300">
                  {t('fleet.selected', { count: selectedVehicles.length })}
                </span>
                <div className="flex space-x-2">
                  <Button
                    variant="secondary"
                    size="sm"
                    onClick={() => handleBulkAction('dispatch')}
                  >
                    {t('fleet.actions.bulkDispatch')}
                  </Button>
                  <Button
                    variant="secondary"
                    size="sm"
                    onClick={() => handleBulkAction('maintenance')}
                  >
                    {t('fleet.actions.bulkMaintenance')}
                  </Button>
                </div>
              </div>
            )}
          </div>
        </div>

        {/* Content */}
        {viewMode === 'grid' ? (
          <div className="grid grid-cols-1 gap-6 sm:grid-cols-2 lg:grid-cols-3 xl:grid-cols-4">
            {filteredVehicles.map((vehicle) => {
              const StatusIcon = statusIcons[vehicle.status]
              return (
                <motion.div
                  key={vehicle.vehicleId}
                  initial={{ opacity: 0, y: 20 }}
                  animate={{ opacity: 1, y: 0 }}
                  className="bg-white dark:bg-gray-800 shadow rounded-lg overflow-hidden hover:shadow-lg transition-shadow"
                >
                  <div className="p-6">
                    <div className="flex items-center justify-between">
                      <div className="flex items-center">
                        <input
                          type="checkbox"
                          checked={selectedVehicles.includes(vehicle.vehicleId)}
                          onChange={() => handleVehicleSelect(vehicle.vehicleId)}
                          className="h-4 w-4 text-blue-600 focus:ring-blue-500 border-gray-300 rounded"
                        />
                        <div className="ml-3">
                          <h3 className="text-lg font-medium text-gray-900 dark:text-white">
                            {vehicle.assetTag}
                          </h3>
                          <p className="text-sm text-gray-500 dark:text-gray-400">
                            {vehicle.manufacturer} {vehicle.model}
                          </p>
                        </div>
                      </div>
                      <Badge className={statusColors[vehicle.status]}>
                        <StatusIcon className="h-4 w-4 mr-1" />
                        {t(`vehicle.status.${vehicle.status}`)}
                      </Badge>
                    </div>

                    <div className="mt-4 space-y-2">
                      <div className="flex items-center text-sm text-gray-500 dark:text-gray-400">
                        <MapPinIcon className="h-4 w-4 mr-2" />
                        {vehicle.currentLocation ? 
                          `${vehicle.currentLocation.lat.toFixed(4)}, ${vehicle.currentLocation.lng.toFixed(4)}` :
                          t('vehicle.location.unknown')
                        }
                      </div>
                      <div className="flex items-center text-sm text-gray-500 dark:text-gray-400">
                        <ClockIcon className="h-4 w-4 mr-2" />
                        {t('vehicle.lastSeen')}: {new Date(vehicle.lastSeen).toLocaleString()}
                      </div>
                    </div>

                    <div className="mt-6 flex justify-between">
                      <Link
                        to={`/fleet/vehicle/${vehicle.vehicleId}`}
                        className="text-blue-600 hover:text-blue-500 text-sm font-medium"
                      >
                        {t('actions.viewDetails')}
                      </Link>
                      {permissions.includes('fleet:dispatch') && vehicle.status === 'idle' && (
                        <Button
                          variant="primary"
                          size="xs"
                          onClick={() => {/* Dispatch vehicle */}}
                        >
                          {t('actions.dispatch')}
                        </Button>
                      )}
                    </div>
                  </div>
                </motion.div>
              )
            })}
          </div>
        ) : (
          <div className="bg-white dark:bg-gray-800 shadow rounded-lg overflow-hidden">
            <div className="h-96">
              <FleetMap vehicles={filteredVehicles} />
            </div>
          </div>
        )}

        {filteredVehicles.length === 0 && (
          <div className="text-center py-12">
            <TruckIcon className="mx-auto h-12 w-12 text-gray-400" />
            <h3 className="mt-2 text-sm font-medium text-gray-900 dark:text-white">
              {t('fleet.empty.title')}
            </h3>
            <p className="mt-1 text-sm text-gray-500 dark:text-gray-400">
              {t('fleet.empty.description')}
            </p>
          </div>
        )}
      </div>
    </>
  )
}
