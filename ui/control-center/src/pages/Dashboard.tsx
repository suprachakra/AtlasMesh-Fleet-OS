import React, { useState, useEffect } from 'react'
import { useTranslation } from 'react-i18next'
import { Helmet } from 'react-helmet-async'
import { motion } from 'framer-motion'
import {
  TruckIcon,
  MapIcon,
  ExclamationTriangleIcon,
  CheckCircleIcon,
  ClockIcon,
  BoltIcon,
} from '@heroicons/react/24/outline'

// Components
import { StatsCard } from '@components/ui/StatsCard'
import { FleetMap } from '@components/fleet/FleetMap'
import { RecentAlerts } from '@components/alerts/RecentAlerts'
import { ActiveTrips } from '@components/trips/ActiveTrips'
import { SystemHealth } from '@components/system/SystemHealth'
import { QuickActions } from '@components/ui/QuickActions'
import { LoadingSpinner } from '@components/ui/LoadingSpinner'
import { ErrorMessage } from '@components/ui/ErrorMessage'

// Hooks
import { useFleetStats } from '@hooks/useFleetStats'
import { useRealtimeUpdates } from '@hooks/useRealtimeUpdates'
import { useSector } from '@hooks/useSector'
import { useAuth } from '@hooks/useAuth'

// Types
import type { FleetStats, Vehicle, Alert, Trip } from '@types/fleet'

const containerVariants = {
  hidden: { opacity: 0 },
  visible: {
    opacity: 1,
    transition: {
      staggerChildren: 0.1,
    },
  },
}

const itemVariants = {
  hidden: { opacity: 0, y: 20 },
  visible: {
    opacity: 1,
    y: 0,
    transition: {
      duration: 0.5,
      ease: 'easeOut',
    },
  },
}

export default function Dashboard() {
  const { t } = useTranslation()
  const { user, permissions } = useAuth()
  const { currentSector, sectorConfig } = useSector()
  
  // Data fetching
  const {
    data: fleetStats,
    isLoading: statsLoading,
    error: statsError,
    refetch: refetchStats,
  } = useFleetStats()
  
  // Real-time updates
  const { 
    vehicles, 
    alerts, 
    trips, 
    isConnected,
    lastUpdate 
  } = useRealtimeUpdates()
  
  // Local state
  const [selectedTimeRange, setSelectedTimeRange] = useState('24h')
  const [refreshing, setRefreshing] = useState(false)
  
  // Auto-refresh data every 30 seconds
  useEffect(() => {
    const interval = setInterval(() => {
      if (isConnected) {
        refetchStats()
      }
    }, 30000)
    
    return () => clearInterval(interval)
  }, [isConnected, refetchStats])
  
  // Manual refresh handler
  const handleRefresh = async () => {
    setRefreshing(true)
    try {
      await refetchStats()
    } finally {
      setRefreshing(false)
    }
  }
  
  // Calculate derived stats
  const activeVehicles = vehicles?.filter(v => v.status === 'active').length || 0
  const idleVehicles = vehicles?.filter(v => v.status === 'idle').length || 0
  const maintenanceVehicles = vehicles?.filter(v => v.status === 'maintenance').length || 0
  const criticalAlerts = alerts?.filter(a => a.severity === 'critical').length || 0
  const activeTripsCount = trips?.filter(t => t.status === 'active').length || 0
  
  // Quick actions based on sector and permissions
  const quickActions = [
    {
      name: t('actions.createTrip'),
      icon: MapIcon,
      href: '/trips/new',
      color: 'primary',
      disabled: !permissions.includes('trips:create'),
    },
    {
      name: t('actions.dispatchVehicle'),
      icon: TruckIcon,
      href: '/fleet/dispatch',
      color: 'success',
      disabled: !permissions.includes('dispatch:create'),
      sectors: ['logistics', 'ridehail'],
    },
    {
      name: t('actions.viewAlerts'),
      icon: ExclamationTriangleIcon,
      href: '/alerts',
      color: 'warning',
      badge: criticalAlerts,
    },
    {
      name: t('actions.systemHealth'),
      icon: BoltIcon,
      href: '/system/health',
      color: 'info',
      disabled: !permissions.includes('system:read'),
    },
  ].filter(action => 
    !action.sectors || action.sectors.includes(currentSector)
  )
  
  if (statsLoading && !fleetStats) {
    return (
      <div className="flex items-center justify-center h-96">
        <LoadingSpinner size="lg" />
      </div>
    )
  }
  
  if (statsError) {
    return (
      <ErrorMessage
        title={t('errors.failedToLoadDashboard')}
        message={statsError.message}
        onRetry={handleRefresh}
      />
    )
  }
  
  return (
    <>
      <Helmet>
        <title>{t('pages.dashboard')} - AtlasMesh Control Center</title>
        <meta name="description" content={t('meta.dashboard.description')} />
      </Helmet>
      
      <motion.div
        variants={containerVariants}
        initial="hidden"
        animate="visible"
        className="space-y-6"
      >
        {/* Page Header */}
        <motion.div variants={itemVariants} className="md:flex md:items-center md:justify-between">
          <div className="min-w-0 flex-1">
            <h1 className="text-2xl font-bold leading-7 text-gray-900 dark:text-gray-100 sm:truncate sm:text-3xl sm:tracking-tight">
              {t('dashboard.welcome', { name: user?.name?.split(' ')[0] || t('user.operator') })}
            </h1>
            <div className="mt-1 flex flex-col sm:mt-0 sm:flex-row sm:flex-wrap sm:space-x-6">
              <div className="mt-2 flex items-center text-sm text-gray-500 dark:text-gray-400">
                <span className="capitalize">{sectorConfig?.displayName || currentSector}</span>
                <span className="mx-2">•</span>
                <span>{t('dashboard.lastUpdated')}: {lastUpdate ? new Date(lastUpdate).toLocaleTimeString() : t('common.never')}</span>
              </div>
            </div>
          </div>
          
          <div className="mt-4 flex md:ml-4 md:mt-0 space-x-3">
            <button
              type="button"
              onClick={handleRefresh}
              disabled={refreshing}
              className="inline-flex items-center rounded-md bg-white dark:bg-gray-800 px-3 py-2 text-sm font-semibold text-gray-900 dark:text-gray-100 shadow-sm ring-1 ring-inset ring-gray-300 dark:ring-gray-600 hover:bg-gray-50 dark:hover:bg-gray-700 focus-visible-ring disabled:opacity-50"
              aria-label={t('actions.refresh')}
            >
              <BoltIcon className={`-ml-0.5 mr-1.5 h-5 w-5 ${refreshing ? 'animate-spin' : ''}`} />
              {refreshing ? t('common.refreshing') : t('actions.refresh')}
            </button>
          </div>
        </motion.div>
        
        {/* Connection Status */}
        {!isConnected && (
          <motion.div variants={itemVariants}>
            <div className="rounded-md bg-warning-50 dark:bg-warning-900/20 p-4 border border-warning-200 dark:border-warning-800">
              <div className="flex">
                <ExclamationTriangleIcon className="h-5 w-5 text-warning-400" aria-hidden="true" />
                <div className="ml-3">
                  <h3 className="text-sm font-medium text-warning-800 dark:text-warning-200">
                    {t('connection.disconnected')}
                  </h3>
                  <p className="mt-1 text-sm text-warning-700 dark:text-warning-300">
                    {t('connection.disconnectedMessage')}
                  </p>
                </div>
              </div>
            </div>
          </motion.div>
        )}
        
        {/* Stats Cards */}
        <motion.div variants={itemVariants}>
          <div className="grid grid-cols-1 gap-5 sm:grid-cols-2 lg:grid-cols-4">
            <StatsCard
              title={t('stats.activeVehicles')}
              value={activeVehicles}
              icon={TruckIcon}
              color="success"
              change={fleetStats?.vehicleStats?.activeChange}
              changeType={fleetStats?.vehicleStats?.activeChange >= 0 ? 'increase' : 'decrease'}
            />
            <StatsCard
              title={t('stats.activeTrips')}
              value={activeTripsCount}
              icon={MapIcon}
              color="primary"
              change={fleetStats?.tripStats?.activeChange}
              changeType={fleetStats?.tripStats?.activeChange >= 0 ? 'increase' : 'decrease'}
            />
            <StatsCard
              title={t('stats.criticalAlerts')}
              value={criticalAlerts}
              icon={ExclamationTriangleIcon}
              color="error"
              change={fleetStats?.alertStats?.criticalChange}
              changeType={fleetStats?.alertStats?.criticalChange >= 0 ? 'increase' : 'decrease'}
            />
            <StatsCard
              title={t('stats.systemHealth')}
              value={`${fleetStats?.systemHealth?.overall || 0}%`}
              icon={CheckCircleIcon}
              color="info"
              change={fleetStats?.systemHealth?.change}
              changeType={fleetStats?.systemHealth?.change >= 0 ? 'increase' : 'decrease'}
            />
          </div>
        </motion.div>
        
        {/* Quick Actions */}
        <motion.div variants={itemVariants}>
          <QuickActions actions={quickActions} />
        </motion.div>
        
        {/* Main Content Grid */}
        <div className="grid grid-cols-1 gap-6 lg:grid-cols-3">
          {/* Fleet Map */}
          <motion.div variants={itemVariants} className="lg:col-span-2">
            <div className="bg-white dark:bg-gray-800 overflow-hidden shadow rounded-lg">
              <div className="px-4 py-5 sm:p-6">
                <h3 className="text-lg leading-6 font-medium text-gray-900 dark:text-gray-100 mb-4">
                  {t('dashboard.fleetMap')}
                </h3>
                <FleetMap
                  vehicles={vehicles || []}
                  height={400}
                  showControls={true}
                  showTraffic={sectorConfig?.features?.traffic}
                  showWeather={sectorConfig?.features?.weather}
                />
              </div>
            </div>
          </motion.div>
          
          {/* Recent Alerts */}
          <motion.div variants={itemVariants}>
            <div className="bg-white dark:bg-gray-800 overflow-hidden shadow rounded-lg">
              <div className="px-4 py-5 sm:p-6">
                <h3 className="text-lg leading-6 font-medium text-gray-900 dark:text-gray-100 mb-4">
                  {t('dashboard.recentAlerts')}
                </h3>
                <RecentAlerts alerts={alerts || []} limit={5} />
              </div>
            </div>
          </motion.div>
        </div>
        
        {/* Secondary Content Grid */}
        <div className="grid grid-cols-1 gap-6 lg:grid-cols-2">
          {/* Active Trips */}
          <motion.div variants={itemVariants}>
            <div className="bg-white dark:bg-gray-800 overflow-hidden shadow rounded-lg">
              <div className="px-4 py-5 sm:p-6">
                <h3 className="text-lg leading-6 font-medium text-gray-900 dark:text-gray-100 mb-4">
                  {t('dashboard.activeTrips')}
                </h3>
                <ActiveTrips trips={trips || []} limit={5} />
              </div>
            </div>
          </motion.div>
          
          {/* System Health */}
          <motion.div variants={itemVariants}>
            <div className="bg-white dark:bg-gray-800 overflow-hidden shadow rounded-lg">
              <div className="px-4 py-5 sm:p-6">
                <h3 className="text-lg leading-6 font-medium text-gray-900 dark:text-gray-100 mb-4">
                  {t('dashboard.systemHealth')}
                </h3>
                <SystemHealth 
                  data={fleetStats?.systemHealth}
                  showDetails={permissions.includes('system:read')}
                />
              </div>
            </div>
          </motion.div>
        </div>
      </motion.div>
    </>
  )
}
