import React, { useState, useEffect } from 'react'
import { useTranslation } from 'react-i18next'
import { Helmet } from 'react-helmet-async'
import { motion } from 'framer-motion'
import {
  ExclamationTriangleIcon,
  ExclamationCircleIcon,
  InformationCircleIcon,
  CheckCircleIcon,
  BellIcon,
  BellSlashIcon,
  MagnifyingGlassIcon,
  FunnelIcon,
  EyeIcon,
  EyeSlashIcon,
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
import { useAlerts } from '@hooks/useAlerts'
import { useAuth } from '@hooks/useAuth'
import { useSector } from '@hooks/useSector'

// Types
import type { Alert, AlertSeverity, AlertFilter } from '@types/alerts'

const severityColors = {
  critical: 'bg-red-100 text-red-800 dark:bg-red-900 dark:text-red-300',
  high: 'bg-orange-100 text-orange-800 dark:bg-orange-900 dark:text-orange-300',
  medium: 'bg-yellow-100 text-yellow-800 dark:bg-yellow-900 dark:text-yellow-300',
  low: 'bg-blue-100 text-blue-800 dark:bg-blue-900 dark:text-blue-300',
  info: 'bg-gray-100 text-gray-800 dark:bg-gray-900 dark:text-gray-300',
}

const severityIcons = {
  critical: ExclamationTriangleIcon,
  high: ExclamationCircleIcon,
  medium: ExclamationCircleIcon,
  low: InformationCircleIcon,
  info: InformationCircleIcon,
}

export default function Alerts() {
  const { t } = useTranslation()
  const { user, permissions } = useAuth()
  const { currentSector, sectorConfig } = useSector()

  // State
  const [searchTerm, setSearchTerm] = useState('')
  const [severityFilter, setSeverityFilter] = useState<AlertSeverity | 'all'>('all')
  const [statusFilter, setStatusFilter] = useState<'all' | 'active' | 'acknowledged' | 'resolved'>('all')
  const [categoryFilter, setCategoryFilter] = useState<string>('all')
  const [selectedAlerts, setSelectedAlerts] = useState<string[]>([])

  // Data fetching
  const {
    data: alerts,
    isLoading,
    error,
    refetch,
  } = useAlerts({
    search: searchTerm,
    severity: severityFilter === 'all' ? undefined : severityFilter,
    status: statusFilter === 'all' ? undefined : statusFilter,
    category: categoryFilter === 'all' ? undefined : categoryFilter,
  })

  // Filter logic
  const filteredAlerts = alerts?.filter((alert) => {
    const matchesSearch = alert.message.toLowerCase().includes(searchTerm.toLowerCase()) ||
                         alert.vehicleId?.toLowerCase().includes(searchTerm.toLowerCase()) ||
                         alert.category.toLowerCase().includes(searchTerm.toLowerCase())
    const matchesSeverity = severityFilter === 'all' || alert.severity === severityFilter
    const matchesStatus = statusFilter === 'all' || 
                         (statusFilter === 'active' && !alert.acknowledged && !alert.resolved) ||
                         (statusFilter === 'acknowledged' && alert.acknowledged && !alert.resolved) ||
                         (statusFilter === 'resolved' && alert.resolved)
    const matchesCategory = categoryFilter === 'all' || alert.category === categoryFilter
    
    return matchesSearch && matchesSeverity && matchesStatus && matchesCategory
  }) || []

  // Stats calculation
  const stats = {
    total: alerts?.length || 0,
    critical: alerts?.filter(a => a.severity === 'critical' && !a.resolved).length || 0,
    high: alerts?.filter(a => a.severity === 'high' && !a.resolved).length || 0,
    medium: alerts?.filter(a => a.severity === 'medium' && !a.resolved).length || 0,
    active: alerts?.filter(a => !a.acknowledged && !a.resolved).length || 0,
    acknowledged: alerts?.filter(a => a.acknowledged && !a.resolved).length || 0,
  }

  // Get unique categories for filter
  const categories = [...new Set(alerts?.map(a => a.category) || [])]

  // Alert actions
  const handleAlertAction = async (alertId: string, action: string) => {
    if (!permissions.includes('alerts:manage')) {
      return
    }

    try {
      console.log(`Executing action: ${action} on alert: ${alertId}`)
      // Implement alert action API calls
      await new Promise(resolve => setTimeout(resolve, 1000))
      refetch()
    } catch (error) {
      console.error('Alert action failed:', error)
    }
  }

  const handleBulkAction = async (action: string) => {
    if (!permissions.includes('alerts:manage')) {
      return
    }

    try {
      console.log(`Bulk action: ${action} on alerts:`, selectedAlerts)
      // Implement bulk action API calls
      await new Promise(resolve => setTimeout(resolve, 2000))
      refetch()
      setSelectedAlerts([])
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
      accessorKey: 'severity',
      header: t('alert.severity'),
      cell: ({ row }: any) => {
        const SeverityIcon = severityIcons[row.original.severity as AlertSeverity]
        return (
          <Badge className={severityColors[row.original.severity as AlertSeverity]}>
            <SeverityIcon className="h-4 w-4 mr-1" />
            {t(`alert.severity.${row.original.severity}`)}
          </Badge>
        )
      },
    },
    {
      accessorKey: 'category',
      header: t('alert.category'),
      cell: ({ row }: any) => (
        <Badge variant="secondary">
          {t(`alert.categories.${row.original.category}`)}
        </Badge>
      ),
    },
    {
      accessorKey: 'message',
      header: t('alert.message'),
      cell: ({ row }: any) => (
        <div className="max-w-xs truncate" title={row.original.message}>
          {row.original.message}
        </div>
      ),
    },
    {
      accessorKey: 'vehicleId',
      header: t('alert.vehicle'),
      cell: ({ row }: any) => (
        row.original.vehicleId ? (
          <a
            href={`/fleet/vehicle/${row.original.vehicleId}`}
            className="text-blue-600 hover:text-blue-500"
          >
            {row.original.vehicleAssetTag || row.original.vehicleId.slice(0, 8)}
          </a>
        ) : (
          <span className="text-gray-400">{t('alert.system')}</span>
        )
      ),
    },
    {
      accessorKey: 'timestamp',
      header: t('alert.timestamp'),
      cell: ({ row }: any) => (
        <div className="text-sm">
          {new Date(row.original.timestamp).toLocaleString()}
        </div>
      ),
    },
    {
      accessorKey: 'status',
      header: t('alert.status'),
      cell: ({ row }: any) => {
        const alert = row.original
        if (alert.resolved) {
          return (
            <Badge className="bg-green-100 text-green-800 dark:bg-green-900 dark:text-green-300">
              <CheckCircleIcon className="h-4 w-4 mr-1" />
              {t('alert.status.resolved')}
            </Badge>
          )
        } else if (alert.acknowledged) {
          return (
            <Badge className="bg-yellow-100 text-yellow-800 dark:bg-yellow-900 dark:text-yellow-300">
              <EyeIcon className="h-4 w-4 mr-1" />
              {t('alert.status.acknowledged')}
            </Badge>
          )
        } else {
          return (
            <Badge className="bg-red-100 text-red-800 dark:bg-red-900 dark:text-red-300">
              <BellIcon className="h-4 w-4 mr-1" />
              {t('alert.status.active')}
            </Badge>
          )
        }
      },
    },
    {
      id: 'actions',
      header: t('actions.title'),
      cell: ({ row }: any) => {
        const alert = row.original
        return (
          <div className="flex items-center space-x-2">
            {!alert.acknowledged && permissions.includes('alerts:manage') && (
              <Button
                variant="ghost"
                size="xs"
                icon={EyeIcon}
                onClick={() => handleAlertAction(alert.alertId, 'acknowledge')}
              >
                {t('actions.acknowledge')}
              </Button>
            )}
            {alert.acknowledged && !alert.resolved && permissions.includes('alerts:manage') && (
              <Button
                variant="ghost"
                size="xs"
                icon={CheckCircleIcon}
                onClick={() => handleAlertAction(alert.alertId, 'resolve')}
              >
                {t('actions.resolve')}
              </Button>
            )}
            {permissions.includes('alerts:manage') && (
              <Button
                variant="ghost"
                size="xs"
                icon={BellSlashIcon}
                onClick={() => handleAlertAction(alert.alertId, 'mute')}
              >
                {t('actions.mute')}
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
        <title>{t('pages.alerts')} - AtlasMesh Fleet OS</title>
      </Helmet>

      <div className="space-y-6">
        {/* Header */}
        <div className="flex flex-col sm:flex-row sm:items-center sm:justify-between">
          <div>
            <h1 className="text-2xl font-bold text-gray-900 dark:text-white">
              {t('pages.alerts')}
            </h1>
            <p className="mt-1 text-sm text-gray-500 dark:text-gray-400">
              {t('alerts.overview.description')}
            </p>
          </div>
          
          <div className="mt-4 sm:mt-0 flex items-center space-x-3">
            <Button
              variant="secondary"
              size="sm"
              icon={BellIcon}
              onClick={refetch}
            >
              {t('actions.refresh')}
            </Button>
          </div>
        </div>

        {/* Stats Cards */}
        <div className="grid grid-cols-1 gap-5 sm:grid-cols-2 lg:grid-cols-6">
          <Card>
            <CardContent className="p-6">
              <div className="flex items-center">
                <div className="flex-shrink-0">
                  <BellIcon className="h-6 w-6 text-gray-400" />
                </div>
                <div className="ml-5 w-0 flex-1">
                  <dl>
                    <dt className="text-sm font-medium text-gray-500 dark:text-gray-400 truncate">
                      {t('stats.totalAlerts')}
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
                  <ExclamationTriangleIcon className="h-6 w-6 text-red-400" />
                </div>
                <div className="ml-5 w-0 flex-1">
                  <dl>
                    <dt className="text-sm font-medium text-gray-500 dark:text-gray-400 truncate">
                      {t('stats.criticalAlerts')}
                    </dt>
                    <dd className="text-lg font-medium text-gray-900 dark:text-white">
                      {stats.critical}
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
                  <ExclamationCircleIcon className="h-6 w-6 text-orange-400" />
                </div>
                <div className="ml-5 w-0 flex-1">
                  <dl>
                    <dt className="text-sm font-medium text-gray-500 dark:text-gray-400 truncate">
                      {t('stats.highAlerts')}
                    </dt>
                    <dd className="text-lg font-medium text-gray-900 dark:text-white">
                      {stats.high}
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
                  <ExclamationCircleIcon className="h-6 w-6 text-yellow-400" />
                </div>
                <div className="ml-5 w-0 flex-1">
                  <dl>
                    <dt className="text-sm font-medium text-gray-500 dark:text-gray-400 truncate">
                      {t('stats.mediumAlerts')}
                    </dt>
                    <dd className="text-lg font-medium text-gray-900 dark:text-white">
                      {stats.medium}
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
                  <BellIcon className="h-6 w-6 text-red-400" />
                </div>
                <div className="ml-5 w-0 flex-1">
                  <dl>
                    <dt className="text-sm font-medium text-gray-500 dark:text-gray-400 truncate">
                      {t('stats.activeAlerts')}
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
                  <EyeIcon className="h-6 w-6 text-yellow-400" />
                </div>
                <div className="ml-5 w-0 flex-1">
                  <dl>
                    <dt className="text-sm font-medium text-gray-500 dark:text-gray-400 truncate">
                      {t('stats.acknowledgedAlerts')}
                    </dt>
                    <dd className="text-lg font-medium text-gray-900 dark:text-white">
                      {stats.acknowledged}
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
            <div className="flex flex-col space-y-4">
              <div className="flex flex-col sm:flex-row sm:items-center sm:justify-between space-y-4 sm:space-y-0 sm:space-x-4">
                <div className="flex-1 min-w-0">
                  <div className="relative">
                    <div className="absolute inset-y-0 left-0 pl-3 flex items-center pointer-events-none">
                      <MagnifyingGlassIcon className="h-5 w-5 text-gray-400" />
                    </div>
                    <Input
                      type="text"
                      placeholder={t('alerts.search.placeholder')}
                      value={searchTerm}
                      onChange={(e) => setSearchTerm(e.target.value)}
                      className="pl-10"
                    />
                  </div>
                </div>
                
                <div className="flex items-center space-x-4">
                  <Select
                    value={severityFilter}
                    onChange={(value) => setSeverityFilter(value as AlertSeverity | 'all')}
                    options={[
                      { value: 'all', label: t('alerts.filters.allSeverities') },
                      { value: 'critical', label: t('alert.severity.critical') },
                      { value: 'high', label: t('alert.severity.high') },
                      { value: 'medium', label: t('alert.severity.medium') },
                      { value: 'low', label: t('alert.severity.low') },
                      { value: 'info', label: t('alert.severity.info') },
                    ]}
                  />
                  
                  <Select
                    value={statusFilter}
                    onChange={(value) => setStatusFilter(value as 'all' | 'active' | 'acknowledged' | 'resolved')}
                    options={[
                      { value: 'all', label: t('alerts.filters.allStatuses') },
                      { value: 'active', label: t('alert.status.active') },
                      { value: 'acknowledged', label: t('alert.status.acknowledged') },
                      { value: 'resolved', label: t('alert.status.resolved') },
                    ]}
                  />

                  <Select
                    value={categoryFilter}
                    onChange={(value) => setCategoryFilter(value)}
                    options={[
                      { value: 'all', label: t('alerts.filters.allCategories') },
                      ...categories.map(cat => ({
                        value: cat,
                        label: t(`alert.categories.${cat}`)
                      }))
                    ]}
                  />
                </div>
              </div>

              {selectedAlerts.length > 0 && (
                <div className="flex items-center justify-between p-4 bg-blue-50 dark:bg-blue-900/20 rounded-lg">
                  <span className="text-sm text-blue-700 dark:text-blue-300">
                    {t('alerts.selected', { count: selectedAlerts.length })}
                  </span>
                  <div className="flex space-x-2">
                    <Button
                      variant="secondary"
                      size="sm"
                      onClick={() => handleBulkAction('acknowledge')}
                    >
                      {t('alerts.actions.bulkAcknowledge')}
                    </Button>
                    <Button
                      variant="secondary"
                      size="sm"
                      onClick={() => handleBulkAction('resolve')}
                    >
                      {t('alerts.actions.bulkResolve')}
                    </Button>
                    <Button
                      variant="secondary"
                      size="sm"
                      onClick={() => handleBulkAction('mute')}
                    >
                      {t('alerts.actions.bulkMute')}
                    </Button>
                  </div>
                </div>
              )}
            </div>
          </CardContent>
        </Card>

        {/* Alerts Table */}
        <Card>
          <CardHeader>
            <CardTitle>{t('alerts.table.title')}</CardTitle>
          </CardHeader>
          <CardContent>
            <DataTable
              columns={columns}
              data={filteredAlerts}
              onRowSelectionChange={setSelectedAlerts}
            />
          </CardContent>
        </Card>

        {filteredAlerts.length === 0 && (
          <div className="text-center py-12">
            <CheckCircleIcon className="mx-auto h-12 w-12 text-green-400" />
            <h3 className="mt-2 text-sm font-medium text-gray-900 dark:text-white">
              {t('alerts.empty.title')}
            </h3>
            <p className="mt-1 text-sm text-gray-500 dark:text-gray-400">
              {t('alerts.empty.description')}
            </p>
          </div>
        )}
      </div>
    </>
  )
}
