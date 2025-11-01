import React, { useState, useEffect } from 'react'
import { useParams, useNavigate } from 'react-router-dom'
import { useTranslation } from 'react-i18next'
import { Helmet } from 'react-helmet-async'
import { motion } from 'framer-motion'
import {
  ArrowLeftIcon,
  MapPinIcon,
  BoltIcon,
  ExclamationTriangleIcon,
  CheckCircleIcon,
  ClockIcon,
  WrenchScrewdriverIcon,
  CogIcon,
  ChartBarIcon,
  DocumentTextIcon,
  PlayIcon,
  StopIcon,
  PauseIcon,
} from '@heroicons/react/24/outline'

// Components
import { LoadingSpinner } from '@components/ui/LoadingSpinner'
import { ErrorMessage } from '@components/ui/ErrorMessage'
import { Button } from '@components/ui/Button'
import { Badge } from '@components/ui/Badge'
import { Tabs, TabsContent, TabsList, TabsTrigger } from '@components/ui/Tabs'
import { Card, CardContent, CardHeader, CardTitle } from '@components/ui/Card'
import { VehicleMap } from '@components/fleet/VehicleMap'
import { TelemetryChart } from '@components/telemetry/TelemetryChart'
import { AlertsList } from '@components/alerts/AlertsList'

// Hooks
import { useVehicleDetails } from '@hooks/useVehicleDetails'
import { useVehicleTelemetry } from '@hooks/useVehicleTelemetry'
import { useVehicleAlerts } from '@hooks/useVehicleAlerts'
import { useAuth } from '@hooks/useAuth'

// Types
import type { Vehicle, VehicleCommand } from '@types/fleet'

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

export default function VehicleDetails() {
  const { vehicleId } = useParams<{ vehicleId: string }>()
  const navigate = useNavigate()
  const { t } = useTranslation()
  const { user, permissions } = useAuth()

  // State
  const [activeTab, setActiveTab] = useState('overview')
  const [commandInProgress, setCommandInProgress] = useState<string | null>(null)

  // Data fetching
  const {
    data: vehicle,
    isLoading: vehicleLoading,
    error: vehicleError,
    refetch: refetchVehicle,
  } = useVehicleDetails(vehicleId!)

  const {
    data: telemetry,
    isLoading: telemetryLoading,
    error: telemetryError,
  } = useVehicleTelemetry(vehicleId!, {
    timeRange: '24h',
    metrics: ['speed', 'battery', 'location', 'system_health'],
  })

  const {
    data: alerts,
    isLoading: alertsLoading,
    error: alertsError,
  } = useVehicleAlerts(vehicleId!, {
    limit: 50,
    severity: ['critical', 'high', 'medium'],
  })

  // Command handlers
  const handleCommand = async (command: VehicleCommand) => {
    if (!permissions.includes('fleet:command')) {
      return
    }

    setCommandInProgress(command.type)
    try {
      // Implement command execution
      // Log vehicle command for audit trail
      console.info(`Vehicle command executed: ${command.type}`, {
        vehicleId,
        command: command.type,
        timestamp: new Date().toISOString(),
        operator: 'current_user' // TODO: Get from auth context
      })
      await new Promise(resolve => setTimeout(resolve, 2000)) // Simulate API call
      refetchVehicle()
    } catch (error) {
      console.error('Command failed:', error)
    } finally {
      setCommandInProgress(null)
    }
  }

  const handleEmergencyStop = () => {
    handleCommand({ type: 'emergency_stop', payload: {} })
  }

  const handleMaintenanceMode = () => {
    handleCommand({ 
      type: 'maintenance_mode', 
      payload: { enabled: vehicle?.status !== 'maintenance' } 
    })
  }

  const handleDispatch = () => {
    // Navigate to trip creation with pre-selected vehicle
    navigate(`/trips/create?vehicleId=${vehicleId}`)
  }

  if (vehicleLoading) {
    return <LoadingSpinner />
  }

  if (vehicleError || !vehicle) {
    return <ErrorMessage error={vehicleError} onRetry={refetchVehicle} />
  }

  const StatusIcon = statusIcons[vehicle.status]

  return (
    <>
      <Helmet>
        <title>{vehicle.assetTag} - {t('pages.fleet')} - AtlasMesh Fleet OS</title>
      </Helmet>

      <div className="space-y-6">
        {/* Header */}
        <div className="flex items-center justify-between">
          <div className="flex items-center space-x-4">
            <Button
              variant="ghost"
              size="sm"
              icon={ArrowLeftIcon}
              onClick={() => navigate('/fleet')}
            >
              {t('actions.back')}
            </Button>
            <div>
              <div className="flex items-center space-x-3">
                <h1 className="text-2xl font-bold text-gray-900 dark:text-white">
                  {vehicle.assetTag}
                </h1>
                <Badge className={statusColors[vehicle.status]}>
                  <StatusIcon className="h-4 w-4 mr-1" />
                  {t(`vehicle.status.${vehicle.status}`)}
                </Badge>
              </div>
              <p className="mt-1 text-sm text-gray-500 dark:text-gray-400">
                {vehicle.manufacturer} {vehicle.model} • {t('vehicle.serialNumber')}: {vehicle.serialNumber}
              </p>
            </div>
          </div>

          {/* Action Buttons */}
          <div className="flex items-center space-x-3">
            {permissions.includes('fleet:command') && (
              <>
                {vehicle.status === 'idle' && (
                  <Button
                    variant="primary"
                    size="sm"
                    icon={PlayIcon}
                    onClick={handleDispatch}
                    disabled={commandInProgress !== null}
                  >
                    {t('actions.dispatch')}
                  </Button>
                )}
                
                <Button
                  variant={vehicle.status === 'maintenance' ? 'primary' : 'secondary'}
                  size="sm"
                  icon={WrenchScrewdriverIcon}
                  onClick={handleMaintenanceMode}
                  disabled={commandInProgress !== null}
                  loading={commandInProgress === 'maintenance_mode'}
                >
                  {vehicle.status === 'maintenance' 
                    ? t('actions.exitMaintenance') 
                    : t('actions.maintenanceMode')
                  }
                </Button>

                <Button
                  variant="danger"
                  size="sm"
                  icon={StopIcon}
                  onClick={handleEmergencyStop}
                  disabled={commandInProgress !== null}
                  loading={commandInProgress === 'emergency_stop'}
                >
                  {t('actions.emergencyStop')}
                </Button>
              </>
            )}
          </div>
        </div>

        {/* Quick Stats */}
        <div className="grid grid-cols-1 gap-5 sm:grid-cols-2 lg:grid-cols-4">
          <Card>
            <CardContent className="p-6">
              <div className="flex items-center">
                <div className="flex-shrink-0">
                  <MapPinIcon className="h-6 w-6 text-blue-400" />
                </div>
                <div className="ml-5 w-0 flex-1">
                  <dl>
                    <dt className="text-sm font-medium text-gray-500 dark:text-gray-400 truncate">
                      {t('vehicle.location.current')}
                    </dt>
                    <dd className="text-lg font-medium text-gray-900 dark:text-white">
                      {vehicle.currentLocation ? 
                        `${vehicle.currentLocation.lat.toFixed(4)}, ${vehicle.currentLocation.lng.toFixed(4)}` :
                        t('vehicle.location.unknown')
                      }
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
                  <BoltIcon className="h-6 w-6 text-green-400" />
                </div>
                <div className="ml-5 w-0 flex-1">
                  <dl>
                    <dt className="text-sm font-medium text-gray-500 dark:text-gray-400 truncate">
                      {t('vehicle.battery.level')}
                    </dt>
                    <dd className="text-lg font-medium text-gray-900 dark:text-white">
                      {telemetry?.latest?.battery?.soc ? `${telemetry.latest.battery.soc}%` : 'N/A'}
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
                  <ClockIcon className="h-6 w-6 text-yellow-400" />
                </div>
                <div className="ml-5 w-0 flex-1">
                  <dl>
                    <dt className="text-sm font-medium text-gray-500 dark:text-gray-400 truncate">
                      {t('vehicle.lastSeen')}
                    </dt>
                    <dd className="text-lg font-medium text-gray-900 dark:text-white">
                      {new Date(vehicle.lastSeen).toLocaleString()}
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
                      {t('vehicle.alerts.active')}
                    </dt>
                    <dd className="text-lg font-medium text-gray-900 dark:text-white">
                      {alerts?.filter(a => !a.acknowledged).length || 0}
                    </dd>
                  </dl>
                </div>
              </div>
            </CardContent>
          </Card>
        </div>

        {/* Tabs */}
        <Tabs value={activeTab} onValueChange={setActiveTab}>
          <TabsList className="grid w-full grid-cols-5">
            <TabsTrigger value="overview">{t('vehicle.tabs.overview')}</TabsTrigger>
            <TabsTrigger value="telemetry">{t('vehicle.tabs.telemetry')}</TabsTrigger>
            <TabsTrigger value="alerts">{t('vehicle.tabs.alerts')}</TabsTrigger>
            <TabsTrigger value="maintenance">{t('vehicle.tabs.maintenance')}</TabsTrigger>
            <TabsTrigger value="configuration">{t('vehicle.tabs.configuration')}</TabsTrigger>
          </TabsList>

          <TabsContent value="overview" className="space-y-6">
            <div className="grid grid-cols-1 lg:grid-cols-2 gap-6">
              {/* Vehicle Map */}
              <Card>
                <CardHeader>
                  <CardTitle>{t('vehicle.map.title')}</CardTitle>
                </CardHeader>
                <CardContent>
                  <div className="h-64">
                    <VehicleMap vehicle={vehicle} />
                  </div>
                </CardContent>
              </Card>

              {/* Vehicle Information */}
              <Card>
                <CardHeader>
                  <CardTitle>{t('vehicle.information.title')}</CardTitle>
                </CardHeader>
                <CardContent className="space-y-4">
                  <div className="grid grid-cols-2 gap-4">
                    <div>
                      <dt className="text-sm font-medium text-gray-500 dark:text-gray-400">
                        {t('vehicle.manufacturer')}
                      </dt>
                      <dd className="mt-1 text-sm text-gray-900 dark:text-white">
                        {vehicle.manufacturer}
                      </dd>
                    </div>
                    <div>
                      <dt className="text-sm font-medium text-gray-500 dark:text-gray-400">
                        {t('vehicle.model')}
                      </dt>
                      <dd className="mt-1 text-sm text-gray-900 dark:text-white">
                        {vehicle.model}
                      </dd>
                    </div>
                    <div>
                      <dt className="text-sm font-medium text-gray-500 dark:text-gray-400">
                        {t('vehicle.serialNumber')}
                      </dt>
                      <dd className="mt-1 text-sm text-gray-900 dark:text-white">
                        {vehicle.serialNumber}
                      </dd>
                    </div>
                    <div>
                      <dt className="text-sm font-medium text-gray-500 dark:text-gray-400">
                        {t('vehicle.fleetId')}
                      </dt>
                      <dd className="mt-1 text-sm text-gray-900 dark:text-white">
                        {vehicle.fleetId}
                      </dd>
                    </div>
                  </div>

                  {vehicle.capabilities && (
                    <div>
                      <dt className="text-sm font-medium text-gray-500 dark:text-gray-400 mb-2">
                        {t('vehicle.capabilities')}
                      </dt>
                      <div className="flex flex-wrap gap-2">
                        {Object.entries(vehicle.capabilities).map(([key, value]) => (
                          <Badge key={key} variant="secondary">
                            {key}: {String(value)}
                          </Badge>
                        ))}
                      </div>
                    </div>
                  )}
                </CardContent>
              </Card>
            </div>
          </TabsContent>

          <TabsContent value="telemetry" className="space-y-6">
            {telemetryLoading ? (
              <LoadingSpinner />
            ) : telemetryError ? (
              <ErrorMessage error={telemetryError} />
            ) : (
              <div className="grid grid-cols-1 lg:grid-cols-2 gap-6">
                <Card>
                  <CardHeader>
                    <CardTitle>{t('telemetry.speed.title')}</CardTitle>
                  </CardHeader>
                  <CardContent>
                    <TelemetryChart
                      data={telemetry?.speed || []}
                      metric="speed"
                      unit="km/h"
                    />
                  </CardContent>
                </Card>

                <Card>
                  <CardHeader>
                    <CardTitle>{t('telemetry.battery.title')}</CardTitle>
                  </CardHeader>
                  <CardContent>
                    <TelemetryChart
                      data={telemetry?.battery || []}
                      metric="soc"
                      unit="%"
                    />
                  </CardContent>
                </Card>

                <Card>
                  <CardHeader>
                    <CardTitle>{t('telemetry.systemHealth.title')}</CardTitle>
                  </CardHeader>
                  <CardContent>
                    <TelemetryChart
                      data={telemetry?.systemHealth || []}
                      metric="overall_score"
                      unit="score"
                    />
                  </CardContent>
                </Card>

                <Card>
                  <CardHeader>
                    <CardTitle>{t('telemetry.location.title')}</CardTitle>
                  </CardHeader>
                  <CardContent>
                    <div className="h-64">
                      <VehicleMap 
                        vehicle={vehicle} 
                        showPath={true}
                        pathData={telemetry?.location || []}
                      />
                    </div>
                  </CardContent>
                </Card>
              </div>
            )}
          </TabsContent>

          <TabsContent value="alerts" className="space-y-6">
            {alertsLoading ? (
              <LoadingSpinner />
            ) : alertsError ? (
              <ErrorMessage error={alertsError} />
            ) : (
              <Card>
                <CardHeader>
                  <CardTitle>{t('vehicle.alerts.title')}</CardTitle>
                </CardHeader>
                <CardContent>
                  <AlertsList alerts={alerts || []} vehicleId={vehicleId} />
                </CardContent>
              </Card>
            )}
          </TabsContent>

          <TabsContent value="maintenance" className="space-y-6">
            <Card>
              <CardHeader>
                <CardTitle>{t('vehicle.maintenance.title')}</CardTitle>
              </CardHeader>
              <CardContent>
                <div className="text-center py-8">
                  <WrenchScrewdriverIcon className="mx-auto h-12 w-12 text-gray-400" />
                  <h3 className="mt-2 text-sm font-medium text-gray-900 dark:text-white">
                    {t('vehicle.maintenance.comingSoon')}
                  </h3>
                  <p className="mt-1 text-sm text-gray-500 dark:text-gray-400">
                    {t('vehicle.maintenance.description')}
                  </p>
                </div>
              </CardContent>
            </Card>
          </TabsContent>

          <TabsContent value="configuration" className="space-y-6">
            <Card>
              <CardHeader>
                <CardTitle>{t('vehicle.configuration.title')}</CardTitle>
              </CardHeader>
              <CardContent>
                <div className="text-center py-8">
                  <CogIcon className="mx-auto h-12 w-12 text-gray-400" />
                  <h3 className="mt-2 text-sm font-medium text-gray-900 dark:text-white">
                    {t('vehicle.configuration.comingSoon')}
                  </h3>
                  <p className="mt-1 text-sm text-gray-500 dark:text-gray-400">
                    {t('vehicle.configuration.description')}
                  </p>
                </div>
              </CardContent>
            </Card>
          </TabsContent>
        </Tabs>
      </div>
    </>
  )
}
