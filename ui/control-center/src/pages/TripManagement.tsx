import React, { useState, useEffect } from 'react'
import { useTranslation } from 'react-i18next'
import { Helmet } from 'react-helmet-async'
import { motion } from 'framer-motion'
import { Link } from 'react-router-dom'
import {
  PlusIcon,
  MagnifyingGlassIcon,
  FunnelIcon,
  MapIcon,
  ClockIcon,
  CheckCircleIcon,
  ExclamationTriangleIcon,
  XCircleIcon,
  PlayIcon,
  PauseIcon,
  StopIcon,
} from '@heroicons/react/24/outline'

// Components
import { LoadingSpinner } from '@components/ui/LoadingSpinner'
import { ErrorMessage } from '@components/ui/ErrorMessage'
import { Button } from '@components/ui/Button'
import { Input } from '@components/ui/Input'
import { Select } from '@components/ui/Select'
import { Badge } from '@components/ui/Badge'
import { Card, CardContent, CardHeader, CardTitle } from '@components/ui/Card'
import { DataTable } from '@components/ui/DataTable'

// Hooks
import { useTrips } from '@hooks/useTrips'
import { useAuth } from '@hooks/useAuth'
import { useSector } from '@hooks/useSector'

// Types
import type { Trip, TripStatus, TripFilter } from '@types/fleet'

const statusColors = {
  scheduled: 'bg-blue-100 text-blue-800 dark:bg-blue-900 dark:text-blue-300',
  active: 'bg-green-100 text-green-800 dark:bg-green-900 dark:text-green-300',
  paused: 'bg-yellow-100 text-yellow-800 dark:bg-yellow-900 dark:text-yellow-300',
  completed: 'bg-gray-100 text-gray-800 dark:bg-gray-900 dark:text-gray-300',
  cancelled: 'bg-red-100 text-red-800 dark:bg-red-900 dark:text-red-300',
  failed: 'bg-red-100 text-red-800 dark:bg-red-900 dark:text-red-300',
}

const statusIcons = {
  scheduled: ClockIcon,
  active: PlayIcon,
  paused: PauseIcon,
  completed: CheckCircleIcon,
  cancelled: XCircleIcon,
  failed: ExclamationTriangleIcon,
}

export default function TripManagement() {
  const { t } = useTranslation()
  const { user, permissions } = useAuth()
  const { currentSector, sectorConfig } = useSector()

  // State
  const [searchTerm, setSearchTerm] = useState('')
  const [statusFilter, setStatusFilter] = useState<TripStatus | 'all'>('all')
  const [dateRange, setDateRange] = useState('today')
  const [selectedTrips, setSelectedTrips] = useState<string[]>([])

  // Data fetching
  const {
    data: trips,
    isLoading,
    error,
    refetch,
  } = useTrips({
    search: searchTerm,
    status: statusFilter === 'all' ? undefined : statusFilter,
    dateRange,
  })

  // Filter logic
  const filteredTrips = trips?.filter((trip) => {
    const matchesSearch = trip.missionType.toLowerCase().includes(searchTerm.toLowerCase()) ||
                         trip.vehicleId.toLowerCase().includes(searchTerm.toLowerCase())
    const matchesStatus = statusFilter === 'all' || trip.status === statusFilter
    return matchesSearch && matchesStatus
  }) || []

  // Stats calculation
  const stats = {
    total: trips?.length || 0,
    scheduled: trips?.filter(t => t.status === 'scheduled').length || 0,
    active: trips?.filter(t => t.status === 'active').length || 0,
    completed: trips?.filter(t => t.status === 'completed').length || 0,
    cancelled: trips?.filter(t => t.status === 'cancelled').length || 0,
  }

  // Trip actions
  const handleTripAction = async (tripId: string, action: string) => {
    if (!permissions.includes('trips:manage')) {
      return
    }

    try {
      console.log(`Executing action: ${action} on trip: ${tripId}`)
      // Implement trip action API calls
      await new Promise(resolve => setTimeout(resolve, 1000))
      refetch()
    } catch (error) {
      console.error('Trip action failed:', error)
    }
  }

  const handleBulkAction = async (action: string) => {
    if (!permissions.includes('trips:manage')) {
      return
    }

    try {
      console.log(`Bulk action: ${action} on trips:`, selectedTrips)
      // Implement bulk action API calls
      await new Promise(resolve => setTimeout(resolve, 2000))
      refetch()
      setSelectedTrips([])
    } catch (error) {
      console.error('Bulk action failed:', error)
    }
  }

  // Table columns
  const columns = [
    {
      id: 'select',
      header: ({ table }: any) => (
        <input
          type="checkbox"
          checked={table.getIsAllPageRowsSelected()}
          onChange={(e) => table.toggleAllPageRowsSelected(!!e.target.checked)}
          className="h-4 w-4 text-blue-600 focus:ring-blue-500 border-gray-300 rounded"
        />
      ),
      cell: ({ row }: any) => (
        <input
          type="checkbox"
          checked={row.getIsSelected()}
          onChange={(e) => row.toggleSelected(!!e.target.checked)}
          className="h-4 w-4 text-blue-600 focus:ring-blue-500 border-gray-300 rounded"
        />
      ),
    },
    {
      accessorKey: 'tripId',
      header: t('trip.id'),
      cell: ({ row }: any) => (
        <Link
          to={`/trips/${row.original.tripId}`}
          className="text-blue-600 hover:text-blue-500 font-medium"
        >
          {row.original.tripId.slice(0, 8)}...
        </Link>
      ),
    },
    {
      accessorKey: 'missionType',
      header: t('trip.missionType'),
      cell: ({ row }: any) => (
        <Badge variant="secondary">
          {t(`trip.missions.${row.original.missionType}`)}
        </Badge>
      ),
    },
    {
      accessorKey: 'vehicleId',
      header: t('trip.vehicle'),
      cell: ({ row }: any) => (
        <Link
          to={`/fleet/vehicle/${row.original.vehicleId}`}
          className="text-blue-600 hover:text-blue-500"
        >
          {row.original.vehicleAssetTag || row.original.vehicleId.slice(0, 8)}
        </Link>
      ),
    },
    {
      accessorKey: 'status',
      header: t('trip.status'),
      cell: ({ row }: any) => {
        const StatusIcon = statusIcons[row.original.status as TripStatus]
        return (
          <Badge className={statusColors[row.original.status as TripStatus]}>
            <StatusIcon className="h-4 w-4 mr-1" />
            {t(`trip.status.${row.original.status}`)}
          </Badge>
        )
      },
    },
    {
      accessorKey: 'origin',
      header: t('trip.origin'),
      cell: ({ row }: any) => (
        <div className="text-sm">
          {row.original.origin ? 
            `${row.original.origin.lat.toFixed(4)}, ${row.original.origin.lng.toFixed(4)}` :
            t('trip.location.unknown')
          }
        </div>
      ),
    },
    {
      accessorKey: 'destination',
      header: t('trip.destination'),
      cell: ({ row }: any) => (
        <div className="text-sm">
          {row.original.destination ? 
            `${row.original.destination.lat.toFixed(4)}, ${row.original.destination.lng.toFixed(4)}` :
            t('trip.location.unknown')
          }
        </div>
      ),
    },
    {
      accessorKey: 'scheduledStart',
      header: t('trip.scheduledStart'),
      cell: ({ row }: any) => (
        <div className="text-sm">
          {new Date(row.original.scheduledStart).toLocaleString()}
        </div>
      ),
    },
    {
      accessorKey: 'progress',
      header: t('trip.progress'),
      cell: ({ row }: any) => {
        const progress = row.original.progress || 0
        return (
          <div className="w-full bg-gray-200 rounded-full h-2">
            <div
              className="bg-blue-600 h-2 rounded-full"
              style={{ width: `${progress}%` }}
            />
          </div>
        )
      },
    },
    {
      id: 'actions',
      header: t('actions.title'),
      cell: ({ row }: any) => {
        const trip = row.original
        return (
          <div className="flex items-center space-x-2">
            {trip.status === 'scheduled' && permissions.includes('trips:manage') && (
              <Button
                variant="ghost"
                size="xs"
                icon={PlayIcon}
                onClick={() => handleTripAction(trip.tripId, 'start')}
              >
                {t('actions.start')}
              </Button>
            )}
            {trip.status === 'active' && permissions.includes('trips:manage') && (
              <>
                <Button
                  variant="ghost"
                  size="xs"
                  icon={PauseIcon}
                  onClick={() => handleTripAction(trip.tripId, 'pause')}
                >
                  {t('actions.pause')}
                </Button>
                <Button
                  variant="ghost"
                  size="xs"
                  icon={StopIcon}
                  onClick={() => handleTripAction(trip.tripId, 'stop')}
                >
                  {t('actions.stop')}
                </Button>
              </>
            )}
            {trip.status === 'paused' && permissions.includes('trips:manage') && (
              <Button
                variant="ghost"
                size="xs"
                icon={PlayIcon}
                onClick={() => handleTripAction(trip.tripId, 'resume')}
              >
                {t('actions.resume')}
              </Button>
            )}
          </div>
        )
      },
    },
  ]

  if (isLoading) {
    return <LoadingSpinner />
  }

  if (error) {
    return <ErrorMessage error={error} onRetry={refetch} />
  }

  return (
    <>
      <Helmet>
        <title>{t('pages.trips')} - AtlasMesh Fleet OS</title>
      </Helmet>

      <div className="space-y-6">
        {/* Header */}
        <div className="flex flex-col sm:flex-row sm:items-center sm:justify-between">
          <div>
            <h1 className="text-2xl font-bold text-gray-900 dark:text-white">
              {t('pages.trips')}
            </h1>
            <p className="mt-1 text-sm text-gray-500 dark:text-gray-400">
              {t('trips.overview.description', { sector: currentSector })}
            </p>
          </div>
          
          {permissions.includes('trips:create') && (
            <div className="mt-4 sm:mt-0">
              <Button
                variant="primary"
                size="sm"
                icon={PlusIcon}
                onClick={() => {/* Navigate to create trip */}}
              >
                {t('trips.actions.createTrip')}
              </Button>
            </div>
          )}
        </div>

        {/* Stats Cards */}
        <div className="grid grid-cols-1 gap-5 sm:grid-cols-2 lg:grid-cols-5">
          <Card>
            <CardContent className="p-6">
              <div className="flex items-center">
                <div className="flex-shrink-0">
                  <MapIcon className="h-6 w-6 text-gray-400" />
                </div>
                <div className="ml-5 w-0 flex-1">
                  <dl>
                    <dt className="text-sm font-medium text-gray-500 dark:text-gray-400 truncate">
                      {t('stats.totalTrips')}
                    </dt>
                    <dd className="text-lg font-medium text-gray-900 dark:text-white">
                      {stats.total}
                    </dd>
                  </dl>
                </div>
              </div>
            </CardContent>
          </Card>

          <Card>
            <CardContent className="p-6">
              <div className="flex items-center">
                <div className="flex-shrink-0">
                  <ClockIcon className="h-6 w-6 text-blue-400" />
                </div>
                <div className="ml-5 w-0 flex-1">
                  <dl>
                    <dt className="text-sm font-medium text-gray-500 dark:text-gray-400 truncate">
                      {t('stats.scheduledTrips')}
                    </dt>
                    <dd className="text-lg font-medium text-gray-900 dark:text-white">
                      {stats.scheduled}
                    </dd>
                  </dl>
                </div>
              </div>
            </CardContent>
          </Card>

          <Card>
            <CardContent className="p-6">
              <div className="flex items-center">
                <div className="flex-shrink-0">
                  <PlayIcon className="h-6 w-6 text-green-400" />
                </div>
                <div className="ml-5 w-0 flex-1">
                  <dl>
                    <dt className="text-sm font-medium text-gray-500 dark:text-gray-400 truncate">
                      {t('stats.activeTrips')}
                    </dt>
                    <dd className="text-lg font-medium text-gray-900 dark:text-white">
                      {stats.active}
                    </dd>
                  </dl>
                </div>
              </div>
            </CardContent>
          </Card>

          <Card>
            <CardContent className="p-6">
              <div className="flex items-center">
                <div className="flex-shrink-0">
                  <CheckCircleIcon className="h-6 w-6 text-gray-400" />
                </div>
                <div className="ml-5 w-0 flex-1">
                  <dl>
                    <dt className="text-sm font-medium text-gray-500 dark:text-gray-400 truncate">
                      {t('stats.completedTrips')}
                    </dt>
                    <dd className="text-lg font-medium text-gray-900 dark:text-white">
                      {stats.completed}
                    </dd>
                  </dl>
                </div>
              </div>
            </CardContent>
          </Card>

          <Card>
            <CardContent className="p-6">
              <div className="flex items-center">
                <div className="flex-shrink-0">
                  <XCircleIcon className="h-6 w-6 text-red-400" />
                </div>
                <div className="ml-5 w-0 flex-1">
                  <dl>
                    <dt className="text-sm font-medium text-gray-500 dark:text-gray-400 truncate">
                      {t('stats.cancelledTrips')}
                    </dt>
                    <dd className="text-lg font-medium text-gray-900 dark:text-white">
                      {stats.cancelled}
                    </dd>
                  </dl>
                </div>
              </div>
            </CardContent>
          </Card>
        </div>

        {/* Filters */}
        <Card>
          <CardContent className="p-6">
            <div className="flex flex-col sm:flex-row sm:items-center sm:justify-between space-y-4 sm:space-y-0 sm:space-x-4">
              <div className="flex-1 min-w-0">
                <div className="relative">
                  <div className="absolute inset-y-0 left-0 pl-3 flex items-center pointer-events-none">
                    <MagnifyingGlassIcon className="h-5 w-5 text-gray-400" />
                  </div>
                  <Input
                    type="text"
                    placeholder={t('trips.search.placeholder')}
                    value={searchTerm}
                    onChange={(e) => setSearchTerm(e.target.value)}
                    className="pl-10"
                  />
                </div>
              </div>
              
              <div className="flex items-center space-x-4">
                <Select
                  value={statusFilter}
                  onChange={(value) => setStatusFilter(value as TripStatus | 'all')}
                  options={[
                    { value: 'all', label: t('trips.filters.allStatuses') },
                    { value: 'scheduled', label: t('trip.status.scheduled') },
                    { value: 'active', label: t('trip.status.active') },
                    { value: 'paused', label: t('trip.status.paused') },
                    { value: 'completed', label: t('trip.status.completed') },
                    { value: 'cancelled', label: t('trip.status.cancelled') },
                  ]}
                />
                
                <Select
                  value={dateRange}
                  onChange={(value) => setDateRange(value)}
                  options={[
                    { value: 'today', label: t('trips.filters.today') },
                    { value: 'week', label: t('trips.filters.thisWeek') },
                    { value: 'month', label: t('trips.filters.thisMonth') },
                    { value: 'all', label: t('trips.filters.allTime') },
                  ]}
                />
              </div>
            </div>

            {selectedTrips.length > 0 && (
              <div className="mt-4 flex items-center justify-between p-4 bg-blue-50 dark:bg-blue-900/20 rounded-lg">
                <span className="text-sm text-blue-700 dark:text-blue-300">
                  {t('trips.selected', { count: selectedTrips.length })}
                </span>
                <div className="flex space-x-2">
                  <Button
                    variant="secondary"
                    size="sm"
                    onClick={() => handleBulkAction('pause')}
                  >
                    {t('trips.actions.bulkPause')}
                  </Button>
                  <Button
                    variant="secondary"
                    size="sm"
                    onClick={() => handleBulkAction('cancel')}
                  >
                    {t('trips.actions.bulkCancel')}
                  </Button>
                </div>
              </div>
            )}
          </CardContent>
        </Card>

        {/* Trips Table */}
        <Card>
          <CardHeader>
            <CardTitle>{t('trips.table.title')}</CardTitle>
          </CardHeader>
          <CardContent>
            <DataTable
              columns={columns}
              data={filteredTrips}
              onRowSelectionChange={setSelectedTrips}
            />
          </CardContent>
        </Card>

        {filteredTrips.length === 0 && (
          <div className="text-center py-12">
            <MapIcon className="mx-auto h-12 w-12 text-gray-400" />
            <h3 className="mt-2 text-sm font-medium text-gray-900 dark:text-white">
              {t('trips.empty.title')}
            </h3>
            <p className="mt-1 text-sm text-gray-500 dark:text-gray-400">
              {t('trips.empty.description')}
            </p>
            {permissions.includes('trips:create') && (
              <div className="mt-6">
                <Button
                  variant="primary"
                  icon={PlusIcon}
                  onClick={() => {/* Navigate to create trip */}}
                >
                  {t('trips.actions.createFirstTrip')}
                </Button>
              </div>
            )}
          </div>
        )}
      </div>
    </>
  )
}
