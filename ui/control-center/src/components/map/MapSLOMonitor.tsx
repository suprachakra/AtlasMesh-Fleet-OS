import React, { useState, useCallback, useEffect, useMemo } from 'react'
import { 
  Map, Clock, Zap, AlertTriangle, CheckCircle, TrendingUp, TrendingDown,
  Gauge, Activity, Wifi, WifiOff, RefreshCw, Settings, Eye, BarChart3,
  Signal, Database, Server, Globe, MapPin, Target, Timer, Layers
} from 'lucide-react'
import { Button } from '../ui/Button'
import { Badge } from '../ui/Badge'
import { Card } from '../ui/Card'
import { Dialog, DialogContent, DialogHeader, DialogTitle } from '../ui/Dialog'
import { Alert, AlertDescription } from '../ui/Alert'
import { Progress } from '../ui/Progress'
import { Tabs, TabsContent, TabsList, TabsTrigger } from '../ui/Tabs'
import { Switch } from '../ui/Switch'

// Types
interface MapSLO {
  id: string
  name: string
  description: string
  category: 'performance' | 'data_quality' | 'availability' | 'user_experience'
  metric: string
  target: number
  current: number
  unit: string
  status: 'healthy' | 'warning' | 'critical' | 'unknown'
  trend: 'improving' | 'stable' | 'degrading'
  lastUpdated: Date
  measurements: SLOMeasurement[]
  thresholds: {
    warning: number
    critical: number
  }
  budget: {
    total: number
    consumed: number
    remaining: number
  }
}

interface SLOMeasurement {
  timestamp: Date
  value: number
  status: 'pass' | 'fail'
}

interface MapDataSource {
  id: string
  name: string
  type: 'tiles' | 'vector' | 'raster' | 'realtime' | 'static'
  status: 'online' | 'offline' | 'degraded' | 'stale'
  lastUpdate: Date
  updateFrequency: number // minutes
  stalenessThreshold: number // minutes
  dataFreshness: number // minutes since last update
  quality: {
    accuracy: number
    completeness: number
    consistency: number
  }
  performance: {
    loadTime: number
    throughput: number
    errorRate: number
  }
  coverage: {
    geographic: number
    temporal: number
  }
}

interface MapPerformanceMetrics {
  fps: number
  frameTime: number
  renderTime: number
  memoryUsage: number
  markerCount: number
  markerCap: number
  layerCount: number
  tileLoadTime: number
  networkLatency: number
  cacheHitRate: number
}

interface StaleDataIndicator {
  id: string
  sourceId: string
  sourceName: string
  dataType: string
  lastUpdate: Date
  staleness: number // minutes
  severity: 'info' | 'warning' | 'critical'
  affectedAreas: string[]
  impact: 'low' | 'medium' | 'high' | 'critical'
  autoRefresh: boolean
  refreshStatus: 'idle' | 'refreshing' | 'failed'
}

interface MapSLOMonitorProps {
  mapInstance?: any
  onSLOViolation?: (slo: MapSLO) => void
  onDataRefresh?: (sourceId: string) => void
  className?: string
}

const MapSLOMonitor: React.FC<MapSLOMonitorProps> = ({
  mapInstance,
  onSLOViolation,
  onDataRefresh,
  className = ''
}) => {
  // State
  const [slos, setSLOs] = useState<MapSLO[]>([])
  const [dataSources, setDataSources] = useState<MapDataSource[]>([])
  const [performanceMetrics, setPerformanceMetrics] = useState<MapPerformanceMetrics | null>(null)
  const [staleIndicators, setStaleIndicators] = useState<StaleDataIndicator[]>([])
  const [showDetailDialog, setShowDetailDialog] = useState(false)
  const [selectedSLO, setSelectedSLO] = useState<MapSLO | null>(null)
  const [activeTab, setActiveTab] = useState('overview')
  const [monitoringEnabled, setMonitoringEnabled] = useState(true)
  const [autoRefreshStale, setAutoRefreshStale] = useState(true)

  // Mock data initialization
  const initializeMockData = useCallback(() => {
    // Mock SLOs
    const mockSLOs: MapSLO[] = [
      {
        id: 'slo-map-load-time',
        name: 'Map Load Time',
        description: 'Time to load initial map view',
        category: 'performance',
        metric: 'load_time',
        target: 2000,
        current: 1850,
        unit: 'ms',
        status: 'healthy',
        trend: 'stable',
        lastUpdated: new Date(),
        measurements: Array.from({ length: 24 }, (_, i) => ({
          timestamp: new Date(Date.now() - (23 - i) * 60 * 60 * 1000),
          value: 1800 + Math.random() * 400,
          status: Math.random() > 0.1 ? 'pass' as const : 'fail' as const
        })),
        thresholds: { warning: 2500, critical: 3000 },
        budget: { total: 100, consumed: 12, remaining: 88 }
      },
      {
        id: 'slo-tile-load-time',
        name: 'Tile Load Time',
        description: 'Average time to load map tiles',
        category: 'performance',
        metric: 'tile_load_time',
        target: 500,
        current: 680,
        unit: 'ms',
        status: 'warning',
        trend: 'degrading',
        lastUpdated: new Date(),
        measurements: Array.from({ length: 24 }, (_, i) => ({
          timestamp: new Date(Date.now() - (23 - i) * 60 * 60 * 1000),
          value: 500 + Math.random() * 300,
          status: Math.random() > 0.15 ? 'pass' as const : 'fail' as const
        })),
        thresholds: { warning: 600, critical: 800 },
        budget: { total: 100, consumed: 28, remaining: 72 }
      },
      {
        id: 'slo-fps',
        name: 'Frame Rate',
        description: 'Map rendering frame rate',
        category: 'performance',
        metric: 'fps',
        target: 30,
        current: 28,
        unit: 'fps',
        status: 'healthy',
        trend: 'stable',
        lastUpdated: new Date(),
        measurements: Array.from({ length: 24 }, (_, i) => ({
          timestamp: new Date(Date.now() - (23 - i) * 60 * 60 * 1000),
          value: 25 + Math.random() * 10,
          status: Math.random() > 0.05 ? 'pass' as const : 'fail' as const
        })),
        thresholds: { warning: 25, critical: 20 },
        budget: { total: 100, consumed: 8, remaining: 92 }
      },
      {
        id: 'slo-marker-cap',
        name: 'Marker Capacity',
        description: 'Number of concurrent markers on map',
        category: 'performance',
        metric: 'marker_count',
        target: 1000,
        current: 850,
        unit: 'markers',
        status: 'healthy',
        trend: 'stable',
        lastUpdated: new Date(),
        measurements: Array.from({ length: 24 }, (_, i) => ({
          timestamp: new Date(Date.now() - (23 - i) * 60 * 60 * 1000),
          value: 700 + Math.random() * 400,
          status: Math.random() > 0.02 ? 'pass' as const : 'fail' as const
        })),
        thresholds: { warning: 900, critical: 1000 },
        budget: { total: 100, consumed: 5, remaining: 95 }
      },
      {
        id: 'slo-data-freshness',
        name: 'Data Freshness',
        description: 'Maximum age of map data',
        category: 'data_quality',
        metric: 'data_age',
        target: 5,
        current: 8,
        unit: 'minutes',
        status: 'warning',
        trend: 'degrading',
        lastUpdated: new Date(),
        measurements: Array.from({ length: 24 }, (_, i) => ({
          timestamp: new Date(Date.now() - (23 - i) * 60 * 60 * 1000),
          value: 3 + Math.random() * 8,
          status: Math.random() > 0.2 ? 'pass' as const : 'fail' as const
        })),
        thresholds: { warning: 7, critical: 10 },
        budget: { total: 100, consumed: 35, remaining: 65 }
      },
      {
        id: 'slo-availability',
        name: 'Map Service Availability',
        description: 'Uptime of map services',
        category: 'availability',
        metric: 'uptime',
        target: 99.9,
        current: 99.7,
        unit: '%',
        status: 'warning',
        trend: 'stable',
        lastUpdated: new Date(),
        measurements: Array.from({ length: 24 }, (_, i) => ({
          timestamp: new Date(Date.now() - (23 - i) * 60 * 60 * 1000),
          value: 99.5 + Math.random() * 0.5,
          status: Math.random() > 0.1 ? 'pass' as const : 'fail' as const
        })),
        thresholds: { warning: 99.5, critical: 99.0 },
        budget: { total: 100, consumed: 22, remaining: 78 }
      }
    ]

    // Mock data sources
    const mockDataSources: MapDataSource[] = [
      {
        id: 'source-base-tiles',
        name: 'Base Map Tiles',
        type: 'tiles',
        status: 'online',
        lastUpdate: new Date(Date.now() - 2 * 60 * 1000),
        updateFrequency: 60,
        stalenessThreshold: 120,
        dataFreshness: 2,
        quality: { accuracy: 98.5, completeness: 99.2, consistency: 97.8 },
        performance: { loadTime: 450, throughput: 2.5, errorRate: 0.02 },
        coverage: { geographic: 100, temporal: 95 }
      },
      {
        id: 'source-traffic',
        name: 'Real-time Traffic',
        type: 'realtime',
        status: 'online',
        lastUpdate: new Date(Date.now() - 30 * 1000),
        updateFrequency: 1,
        stalenessThreshold: 5,
        dataFreshness: 0.5,
        quality: { accuracy: 92.1, completeness: 88.7, consistency: 91.3 },
        performance: { loadTime: 850, throughput: 1.8, errorRate: 0.05 },
        coverage: { geographic: 85, temporal: 98 }
      },
      {
        id: 'source-vehicle-positions',
        name: 'Vehicle Positions',
        type: 'realtime',
        status: 'online',
        lastUpdate: new Date(Date.now() - 15 * 1000),
        updateFrequency: 0.25,
        stalenessThreshold: 2,
        dataFreshness: 0.25,
        quality: { accuracy: 99.8, completeness: 97.5, consistency: 99.1 },
        performance: { loadTime: 120, throughput: 5.2, errorRate: 0.01 },
        coverage: { geographic: 100, temporal: 100 }
      },
      {
        id: 'source-weather',
        name: 'Weather Overlay',
        type: 'raster',
        status: 'stale',
        lastUpdate: new Date(Date.now() - 12 * 60 * 1000),
        updateFrequency: 10,
        stalenessThreshold: 15,
        dataFreshness: 12,
        quality: { accuracy: 85.2, completeness: 78.9, consistency: 82.1 },
        performance: { loadTime: 1200, throughput: 0.8, errorRate: 0.08 },
        coverage: { geographic: 90, temporal: 75 }
      },
      {
        id: 'source-poi',
        name: 'Points of Interest',
        type: 'static',
        status: 'online',
        lastUpdate: new Date(Date.now() - 4 * 60 * 60 * 1000),
        updateFrequency: 1440,
        stalenessThreshold: 2880,
        dataFreshness: 240,
        quality: { accuracy: 96.8, completeness: 94.2, consistency: 95.7 },
        performance: { loadTime: 680, throughput: 1.2, errorRate: 0.03 },
        coverage: { geographic: 95, temporal: 85 }
      }
    ]

    // Mock performance metrics
    const mockPerformanceMetrics: MapPerformanceMetrics = {
      fps: 28,
      frameTime: 35.7,
      renderTime: 22.1,
      memoryUsage: 245,
      markerCount: 850,
      markerCap: 1000,
      layerCount: 8,
      tileLoadTime: 680,
      networkLatency: 45,
      cacheHitRate: 87.5
    }

    // Mock stale data indicators
    const mockStaleIndicators: StaleDataIndicator[] = [
      {
        id: 'stale-weather',
        sourceId: 'source-weather',
        sourceName: 'Weather Overlay',
        dataType: 'Weather Data',
        lastUpdate: new Date(Date.now() - 12 * 60 * 1000),
        staleness: 12,
        severity: 'warning',
        affectedAreas: ['Downtown Dubai', 'Dubai Marina', 'DIFC'],
        impact: 'medium',
        autoRefresh: true,
        refreshStatus: 'idle'
      }
    ]

    setSLOs(mockSLOs)
    setDataSources(mockDataSources)
    setPerformanceMetrics(mockPerformanceMetrics)
    setStaleIndicators(mockStaleIndicators)
  }, [])

  // Initialize data on mount
  useEffect(() => {
    initializeMockData()
  }, [initializeMockData])

  // Real-time monitoring simulation
  useEffect(() => {
    if (!monitoringEnabled) return

    const interval = setInterval(() => {
      // Update SLOs with new measurements
      setSLOs(prev => prev.map(slo => {
        const newValue = slo.current + (Math.random() - 0.5) * (slo.target * 0.1)
        const measurement: SLOMeasurement = {
          timestamp: new Date(),
          value: newValue,
          status: (slo.metric === 'data_age' || slo.metric === 'tile_load_time') ? 
            (newValue <= slo.target ? 'pass' : 'fail') :
            (newValue >= slo.target ? 'pass' : 'fail')
        }

        let status: MapSLO['status'] = 'healthy'
        if (slo.metric === 'data_age' || slo.metric === 'tile_load_time') {
          if (newValue >= slo.thresholds.critical) status = 'critical'
          else if (newValue >= slo.thresholds.warning) status = 'warning'
        } else {
          if (newValue <= slo.thresholds.critical) status = 'critical'
          else if (newValue <= slo.thresholds.warning) status = 'warning'
        }

        return {
          ...slo,
          current: newValue,
          status,
          lastUpdated: new Date(),
          measurements: [...slo.measurements.slice(-23), measurement]
        }
      }))

      // Update performance metrics
      setPerformanceMetrics(prev => prev ? {
        ...prev,
        fps: Math.max(15, Math.min(60, prev.fps + (Math.random() - 0.5) * 5)),
        frameTime: Math.max(16, prev.frameTime + (Math.random() - 0.5) * 10),
        renderTime: Math.max(10, prev.renderTime + (Math.random() - 0.5) * 5),
        memoryUsage: Math.max(100, Math.min(500, prev.memoryUsage + (Math.random() - 0.5) * 20)),
        markerCount: Math.max(0, Math.min(1000, prev.markerCount + Math.floor((Math.random() - 0.5) * 50))),
        networkLatency: Math.max(10, prev.networkLatency + (Math.random() - 0.5) * 10),
        cacheHitRate: Math.max(70, Math.min(95, prev.cacheHitRate + (Math.random() - 0.5) * 5))
      } : null)

      // Update data source freshness
      setDataSources(prev => prev.map(source => {
        const timeSinceUpdate = (new Date().getTime() - source.lastUpdate.getTime()) / (60 * 1000)
        const freshness = timeSinceUpdate
        let status: MapDataSource['status'] = 'online'
        
        if (freshness > source.stalenessThreshold * 2) status = 'offline'
        else if (freshness > source.stalenessThreshold) status = 'stale'
        else if (freshness > source.stalenessThreshold * 0.8) status = 'degraded'

        return { ...source, dataFreshness: freshness, status }
      }))

      // Update stale indicators
      setStaleIndicators(prev => prev.map(indicator => {
        const staleness = (new Date().getTime() - indicator.lastUpdate.getTime()) / (60 * 1000)
        let severity: StaleDataIndicator['severity'] = 'info'
        let impact: StaleDataIndicator['impact'] = 'low'

        if (staleness > 15) {
          severity = 'critical'
          impact = 'critical'
        } else if (staleness > 10) {
          severity = 'warning'
          impact = 'high'
        } else if (staleness > 5) {
          severity = 'warning'
          impact = 'medium'
        }

        return { ...indicator, staleness, severity, impact }
      }))
    }, 5000)

    return () => clearInterval(interval)
  }, [monitoringEnabled])

  // Auto-refresh stale data
  useEffect(() => {
    if (!autoRefreshStale) return

    const interval = setInterval(() => {
      staleIndicators.forEach(indicator => {
        if (indicator.autoRefresh && indicator.severity === 'critical' && indicator.refreshStatus === 'idle') {
          handleRefreshData(indicator.sourceId)
        }
      })
    }, 30000)

    return () => clearInterval(interval)
  }, [autoRefreshStale, staleIndicators])

  // Handlers
  const handleViewSLODetails = useCallback((slo: MapSLO) => {
    setSelectedSLO(slo)
    setShowDetailDialog(true)
  }, [])

  const handleRefreshData = useCallback((sourceId: string) => {
    setStaleIndicators(prev => prev.map(indicator => 
      indicator.sourceId === sourceId 
        ? { ...indicator, refreshStatus: 'refreshing' as const }
        : indicator
    ))

    // Simulate refresh
    setTimeout(() => {
      setDataSources(prev => prev.map(source => 
        source.id === sourceId 
          ? { ...source, lastUpdate: new Date(), dataFreshness: 0 }
          : source
      ))

      setStaleIndicators(prev => prev.map(indicator => 
        indicator.sourceId === sourceId 
          ? { 
              ...indicator, 
              lastUpdate: new Date(), 
              staleness: 0, 
              severity: 'info' as const,
              impact: 'low' as const,
              refreshStatus: 'idle' as const 
            }
          : indicator
      ))

      onDataRefresh?.(sourceId)
    }, 2000)
  }, [onDataRefresh])

  // Computed values
  const sloStats = useMemo(() => {
    const total = slos.length
    const healthy = slos.filter(s => s.status === 'healthy').length
    const warning = slos.filter(s => s.status === 'warning').length
    const critical = slos.filter(s => s.status === 'critical').length
    const avgBudgetConsumed = slos.reduce((sum, s) => sum + s.budget.consumed, 0) / total

    return { total, healthy, warning, critical, avgBudgetConsumed }
  }, [slos])

  const dataSourceStats = useMemo(() => {
    const total = dataSources.length
    const online = dataSources.filter(s => s.status === 'online').length
    const stale = dataSources.filter(s => s.status === 'stale').length
    const offline = dataSources.filter(s => s.status === 'offline').length
    const avgFreshness = dataSources.reduce((sum, s) => sum + s.dataFreshness, 0) / total

    return { total, online, stale, offline, avgFreshness }
  }, [dataSources])

  const getStatusColor = (status: string) => {
    switch (status) {
      case 'healthy':
      case 'online': return 'text-green-600 bg-green-100'
      case 'warning':
      case 'degraded': return 'text-yellow-600 bg-yellow-100'
      case 'critical':
      case 'stale': return 'text-orange-600 bg-orange-100'
      case 'offline': return 'text-red-600 bg-red-100'
      default: return 'text-gray-600 bg-gray-100'
    }
  }

  const getTrendIcon = (trend: string) => {
    switch (trend) {
      case 'improving': return <TrendingUp className="w-4 h-4 text-green-600" />
      case 'degrading': return <TrendingDown className="w-4 h-4 text-red-600" />
      default: return <Activity className="w-4 h-4 text-gray-600" />
    }
  }

  return (
    <div className={`space-y-6 ${className}`}>
      {/* Header */}
      <div className="flex items-center justify-between">
        <div className="flex items-center space-x-3">
          <Map className="w-6 h-6 text-blue-600" />
          <h2 className="text-2xl font-semibold text-gray-900">Map SLO Monitor</h2>
          <div className="flex items-center space-x-2">
            <Switch checked={monitoringEnabled} onCheckedChange={setMonitoringEnabled} />
            <span className="text-sm text-gray-600">Real-time Monitoring</span>
          </div>
        </div>
        <div className="flex items-center space-x-3">
          <div className="flex items-center space-x-2">
            <Switch checked={autoRefreshStale} onCheckedChange={setAutoRefreshStale} />
            <span className="text-sm text-gray-600">Auto-refresh Stale</span>
          </div>
          <Button variant="outline" onClick={initializeMockData}>
            <RefreshCw className="w-4 h-4 mr-2" />
            Refresh
          </Button>
        </div>
      </div>

      {/* Overview Stats */}
      <div className="grid grid-cols-2 md:grid-cols-4 lg:grid-cols-8 gap-4">
        <Card className="p-4">
          <div className="text-center">
            <div className="text-2xl font-bold text-gray-900">{sloStats.total}</div>
            <div className="text-sm text-gray-600">Total SLOs</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className="text-2xl font-bold text-green-600">{sloStats.healthy}</div>
            <div className="text-sm text-gray-600">Healthy</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className="text-2xl font-bold text-yellow-600">{sloStats.warning}</div>
            <div className="text-sm text-gray-600">Warning</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className={`text-2xl font-bold ${sloStats.critical > 0 ? 'text-red-600' : 'text-green-600'}`}>
              {sloStats.critical}
            </div>
            <div className="text-sm text-gray-600">Critical</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className="text-2xl font-bold text-blue-600">{Math.round(sloStats.avgBudgetConsumed)}%</div>
            <div className="text-sm text-gray-600">Avg Budget</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className="text-2xl font-bold text-green-600">{dataSourceStats.online}</div>
            <div className="text-sm text-gray-600">Sources Online</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className={`text-2xl font-bold ${dataSourceStats.stale > 0 ? 'text-orange-600' : 'text-green-600'}`}>
              {dataSourceStats.stale}
            </div>
            <div className="text-sm text-gray-600">Stale Sources</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className="text-2xl font-bold text-purple-600">
              {performanceMetrics ? Math.round(performanceMetrics.fps) : 0}
            </div>
            <div className="text-sm text-gray-600">FPS</div>
          </div>
        </Card>
      </div>

      {/* Stale Data Alerts */}
      {staleIndicators.length > 0 && (
        <div className="space-y-3">
          <h3 className="text-lg font-medium text-gray-900 flex items-center">
            <WifiOff className="w-5 h-5 mr-2 text-orange-600" />
            Stale Data Indicators
          </h3>
          {staleIndicators.map(indicator => (
            <Alert key={indicator.id} className={`border-l-4 ${
              indicator.severity === 'critical' ? 'border-red-400 bg-red-50' :
              indicator.severity === 'warning' ? 'border-yellow-400 bg-yellow-50' :
              'border-blue-400 bg-blue-50'
            }`}>
              <div className="flex items-center justify-between">
                <div className="flex items-start space-x-3">
                  <AlertTriangle className={`w-5 h-5 mt-0.5 ${
                    indicator.severity === 'critical' ? 'text-red-600' :
                    indicator.severity === 'warning' ? 'text-yellow-600' :
                    'text-blue-600'
                  }`} />
                  <div>
                    <div className="font-medium text-gray-900">
                      {indicator.sourceName} - Data is {Math.round(indicator.staleness)} minutes old
                    </div>
                    <div className="text-sm text-gray-600 mt-1">
                      Last updated: {indicator.lastUpdate.toLocaleTimeString()} • 
                      Impact: {indicator.impact} • 
                      Affected areas: {indicator.affectedAreas.join(', ')}
                    </div>
                  </div>
                </div>
                <div className="flex items-center space-x-2">
                  <Badge className={
                    indicator.severity === 'critical' ? 'bg-red-100 text-red-800' :
                    indicator.severity === 'warning' ? 'bg-yellow-100 text-yellow-800' :
                    'bg-blue-100 text-blue-800'
                  }>
                    {indicator.severity}
                  </Badge>
                  <Button
                    size="sm"
                    variant="outline"
                    onClick={() => handleRefreshData(indicator.sourceId)}
                    disabled={indicator.refreshStatus === 'refreshing'}
                  >
                    {indicator.refreshStatus === 'refreshing' ? (
                      <RefreshCw className="w-4 h-4 animate-spin" />
                    ) : (
                      <RefreshCw className="w-4 h-4" />
                    )}
                  </Button>
                </div>
              </div>
            </Alert>
          ))}
        </div>
      )}

      {/* Main Content Tabs */}
      <Tabs value={activeTab} onValueChange={setActiveTab}>
        <TabsList>
          <TabsTrigger value="overview">SLO Overview</TabsTrigger>
          <TabsTrigger value="performance">Performance</TabsTrigger>
          <TabsTrigger value="data_sources">Data Sources</TabsTrigger>
          <TabsTrigger value="budgets">Error Budgets</TabsTrigger>
        </TabsList>

        <TabsContent value="overview" className="space-y-4">
          <div className="grid grid-cols-1 lg:grid-cols-2 gap-6">
            {slos.map(slo => (
              <Card key={slo.id} className="p-6 hover:shadow-md transition-shadow cursor-pointer"
                    onClick={() => handleViewSLODetails(slo)}>
                <div className="flex items-start justify-between mb-4">
                  <div>
                    <h3 className="text-lg font-medium text-gray-900">{slo.name}</h3>
                    <p className="text-sm text-gray-600">{slo.description}</p>
                  </div>
                  <div className="flex items-center space-x-2">
                    <Badge className={getStatusColor(slo.status)}>
                      {slo.status}
                    </Badge>
                    {getTrendIcon(slo.trend)}
                  </div>
                </div>

                <div className="space-y-3">
                  <div className="flex items-center justify-between">
                    <span className="text-sm text-gray-600">Current Value</span>
                    <span className="text-lg font-bold text-gray-900">
                      {slo.current.toFixed(1)}{slo.unit}
                    </span>
                  </div>

                  <div className="flex items-center justify-between">
                    <span className="text-sm text-gray-600">Target</span>
                    <span className="text-sm font-medium text-gray-700">
                      {slo.target.toFixed(1)}{slo.unit}
                    </span>
                  </div>

                  <div>
                    <div className="flex items-center justify-between text-sm mb-1">
                      <span className="text-gray-600">Error Budget</span>
                      <span className="text-gray-700">{slo.budget.remaining}% remaining</span>
                    </div>
                    <Progress 
                      value={100 - slo.budget.consumed} 
                      className={`h-2 ${
                        slo.budget.consumed > 80 ? 'bg-red-200' :
                        slo.budget.consumed > 60 ? 'bg-yellow-200' : 'bg-green-200'
                      }`}
                    />
                  </div>

                  <div className="text-xs text-gray-500">
                    Last updated: {slo.lastUpdated.toLocaleTimeString()}
                  </div>
                </div>
              </Card>
            ))}
          </div>
        </TabsContent>

        <TabsContent value="performance" className="space-y-6">
          {performanceMetrics && (
            <>
              <div className="grid grid-cols-2 md:grid-cols-4 gap-6">
                <Card className="p-4 text-center">
                  <Gauge className="w-8 h-8 text-blue-600 mx-auto mb-2" />
                  <div className="text-2xl font-bold text-gray-900">
                    {Math.round(performanceMetrics.fps)}
                  </div>
                  <div className="text-sm text-gray-600">FPS</div>
                  <Progress value={(performanceMetrics.fps / 60) * 100} className="mt-2 h-2" />
                </Card>

                <Card className="p-4 text-center">
                  <Timer className="w-8 h-8 text-green-600 mx-auto mb-2" />
                  <div className="text-2xl font-bold text-gray-900">
                    {Math.round(performanceMetrics.frameTime)}
                  </div>
                  <div className="text-sm text-gray-600">Frame Time (ms)</div>
                  <Progress value={Math.min(100, (performanceMetrics.frameTime / 50) * 100)} className="mt-2 h-2" />
                </Card>

                <Card className="p-4 text-center">
                  <Database className="w-8 h-8 text-purple-600 mx-auto mb-2" />
                  <div className="text-2xl font-bold text-gray-900">
                    {Math.round(performanceMetrics.memoryUsage)}
                  </div>
                  <div className="text-sm text-gray-600">Memory (MB)</div>
                  <Progress value={(performanceMetrics.memoryUsage / 500) * 100} className="mt-2 h-2" />
                </Card>

                <Card className="p-4 text-center">
                  <Target className="w-8 h-8 text-orange-600 mx-auto mb-2" />
                  <div className="text-2xl font-bold text-gray-900">
                    {performanceMetrics.markerCount}
                  </div>
                  <div className="text-sm text-gray-600">Markers</div>
                  <Progress value={(performanceMetrics.markerCount / performanceMetrics.markerCap) * 100} className="mt-2 h-2" />
                </Card>
              </div>

              <div className="grid grid-cols-1 md:grid-cols-2 gap-6">
                <Card className="p-4">
                  <h3 className="text-lg font-medium text-gray-900 mb-4">Network Performance</h3>
                  <div className="space-y-3">
                    <div className="flex items-center justify-between">
                      <span className="text-sm text-gray-600">Tile Load Time</span>
                      <span className="font-medium">{Math.round(performanceMetrics.tileLoadTime)}ms</span>
                    </div>
                    <div className="flex items-center justify-between">
                      <span className="text-sm text-gray-600">Network Latency</span>
                      <span className="font-medium">{Math.round(performanceMetrics.networkLatency)}ms</span>
                    </div>
                    <div className="flex items-center justify-between">
                      <span className="text-sm text-gray-600">Cache Hit Rate</span>
                      <span className="font-medium">{performanceMetrics.cacheHitRate.toFixed(1)}%</span>
                    </div>
                  </div>
                </Card>

                <Card className="p-4">
                  <h3 className="text-lg font-medium text-gray-900 mb-4">Rendering Performance</h3>
                  <div className="space-y-3">
                    <div className="flex items-center justify-between">
                      <span className="text-sm text-gray-600">Render Time</span>
                      <span className="font-medium">{performanceMetrics.renderTime.toFixed(1)}ms</span>
                    </div>
                    <div className="flex items-center justify-between">
                      <span className="text-sm text-gray-600">Active Layers</span>
                      <span className="font-medium">{performanceMetrics.layerCount}</span>
                    </div>
                    <div className="flex items-center justify-between">
                      <span className="text-sm text-gray-600">Marker Utilization</span>
                      <span className="font-medium">
                        {Math.round((performanceMetrics.markerCount / performanceMetrics.markerCap) * 100)}%
                      </span>
                    </div>
                  </div>
                </Card>
              </div>
            </>
          )}
        </TabsContent>

        <TabsContent value="data_sources" className="space-y-4">
          <div className="grid grid-cols-1 lg:grid-cols-2 gap-6">
            {dataSources.map(source => (
              <Card key={source.id} className="p-6">
                <div className="flex items-start justify-between mb-4">
                  <div>
                    <h3 className="text-lg font-medium text-gray-900">{source.name}</h3>
                    <p className="text-sm text-gray-600 capitalize">{source.type} data source</p>
                  </div>
                  <div className="flex items-center space-x-2">
                    <Badge className={getStatusColor(source.status)}>
                      {source.status}
                    </Badge>
                    <Button
                      size="sm"
                      variant="outline"
                      onClick={() => handleRefreshData(source.id)}
                    >
                      <RefreshCw className="w-4 h-4" />
                    </Button>
                  </div>
                </div>

                <div className="space-y-3 text-sm">
                  <div className="flex items-center justify-between">
                    <span className="text-gray-600">Data Freshness</span>
                    <span className={`font-medium ${
                      source.dataFreshness > source.stalenessThreshold ? 'text-red-600' :
                      source.dataFreshness > source.stalenessThreshold * 0.8 ? 'text-yellow-600' :
                      'text-green-600'
                    }`}>
                      {Math.round(source.dataFreshness)}min ago
                    </span>
                  </div>

                  <div className="flex items-center justify-between">
                    <span className="text-gray-600">Update Frequency</span>
                    <span className="font-medium">
                      {source.updateFrequency < 1 ? 
                        `${Math.round(source.updateFrequency * 60)}s` : 
                        `${source.updateFrequency}min`
                      }
                    </span>
                  </div>

                  <div className="grid grid-cols-3 gap-4 pt-2">
                    <div className="text-center">
                      <div className="text-lg font-bold text-blue-600">
                        {source.quality.accuracy.toFixed(1)}%
                      </div>
                      <div className="text-xs text-gray-600">Accuracy</div>
                    </div>
                    <div className="text-center">
                      <div className="text-lg font-bold text-green-600">
                        {source.quality.completeness.toFixed(1)}%
                      </div>
                      <div className="text-xs text-gray-600">Complete</div>
                    </div>
                    <div className="text-center">
                      <div className="text-lg font-bold text-purple-600">
                        {source.quality.consistency.toFixed(1)}%
                      </div>
                      <div className="text-xs text-gray-600">Consistent</div>
                    </div>
                  </div>

                  <div className="pt-2 border-t">
                    <div className="flex items-center justify-between">
                      <span className="text-gray-600">Load Time</span>
                      <span className="font-medium">{source.performance.loadTime}ms</span>
                    </div>
                    <div className="flex items-center justify-between">
                      <span className="text-gray-600">Error Rate</span>
                      <span className="font-medium">{(source.performance.errorRate * 100).toFixed(2)}%</span>
                    </div>
                  </div>
                </div>
              </Card>
            ))}
          </div>
        </TabsContent>

        <TabsContent value="budgets" className="space-y-4">
          <div className="grid grid-cols-1 lg:grid-cols-2 gap-6">
            {slos.map(slo => (
              <Card key={slo.id} className="p-6">
                <div className="flex items-center justify-between mb-4">
                  <h3 className="text-lg font-medium text-gray-900">{slo.name}</h3>
                  <Badge className={
                    slo.budget.consumed > 80 ? 'bg-red-100 text-red-800' :
                    slo.budget.consumed > 60 ? 'bg-yellow-100 text-yellow-800' :
                    'bg-green-100 text-green-800'
                  }>
                    {slo.budget.remaining}% remaining
                  </Badge>
                </div>

                <div className="space-y-4">
                  <div>
                    <div className="flex items-center justify-between text-sm mb-2">
                      <span className="text-gray-600">Error Budget Consumption</span>
                      <span className="font-medium">{slo.budget.consumed}%</span>
                    </div>
                    <Progress 
                      value={slo.budget.consumed} 
                      className={`h-3 ${
                        slo.budget.consumed > 80 ? 'bg-red-200' :
                        slo.budget.consumed > 60 ? 'bg-yellow-200' : 'bg-green-200'
                      }`}
                    />
                  </div>

                  <div className="grid grid-cols-3 gap-4 text-center text-sm">
                    <div>
                      <div className="font-bold text-gray-900">{slo.budget.total}%</div>
                      <div className="text-gray-600">Total Budget</div>
                    </div>
                    <div>
                      <div className="font-bold text-red-600">{slo.budget.consumed}%</div>
                      <div className="text-gray-600">Consumed</div>
                    </div>
                    <div>
                      <div className="font-bold text-green-600">{slo.budget.remaining}%</div>
                      <div className="text-gray-600">Remaining</div>
                    </div>
                  </div>

                  <div className="pt-3 border-t">
                    <div className="text-sm text-gray-600">
                      <div>Target: {slo.target}{slo.unit}</div>
                      <div>Current: {slo.current.toFixed(1)}{slo.unit}</div>
                      <div>Status: <span className={
                        slo.status === 'healthy' ? 'text-green-600' :
                        slo.status === 'warning' ? 'text-yellow-600' :
                        'text-red-600'
                      }>{slo.status}</span></div>
                    </div>
                  </div>
                </div>
              </Card>
            ))}
          </div>
        </TabsContent>
      </Tabs>

      {/* SLO Detail Dialog */}
      <Dialog open={showDetailDialog} onOpenChange={setShowDetailDialog}>
        <DialogContent className="max-w-4xl max-h-[90vh] overflow-y-auto">
          <DialogHeader>
            <DialogTitle>SLO Details: {selectedSLO?.name}</DialogTitle>
          </DialogHeader>

          {selectedSLO && (
            <div className="space-y-6">
              <div className="grid grid-cols-2 gap-6">
                <Card className="p-4">
                  <h3 className="text-lg font-medium text-gray-900 mb-3">Current Status</h3>
                  <div className="space-y-2 text-sm">
                    <div className="flex justify-between">
                      <span className="text-gray-600">Current Value:</span>
                      <span className="font-medium">{selectedSLO.current.toFixed(1)}{selectedSLO.unit}</span>
                    </div>
                    <div className="flex justify-between">
                      <span className="text-gray-600">Target:</span>
                      <span className="font-medium">{selectedSLO.target}{selectedSLO.unit}</span>
                    </div>
                    <div className="flex justify-between">
                      <span className="text-gray-600">Status:</span>
                      <Badge className={getStatusColor(selectedSLO.status)}>
                        {selectedSLO.status}
                      </Badge>
                    </div>
                    <div className="flex justify-between">
                      <span className="text-gray-600">Trend:</span>
                      <div className="flex items-center space-x-1">
                        {getTrendIcon(selectedSLO.trend)}
                        <span className="capitalize">{selectedSLO.trend}</span>
                      </div>
                    </div>
                  </div>
                </Card>

                <Card className="p-4">
                  <h3 className="text-lg font-medium text-gray-900 mb-3">Error Budget</h3>
                  <div className="space-y-3">
                    <div>
                      <div className="flex items-center justify-between text-sm mb-1">
                        <span className="text-gray-600">Budget Consumption</span>
                        <span className="font-medium">{selectedSLO.budget.consumed}%</span>
                      </div>
                      <Progress 
                        value={selectedSLO.budget.consumed} 
                        className={`h-3 ${
                          selectedSLO.budget.consumed > 80 ? 'bg-red-200' :
                          selectedSLO.budget.consumed > 60 ? 'bg-yellow-200' : 'bg-green-200'
                        }`}
                      />
                    </div>
                    <div className="grid grid-cols-2 gap-4 text-sm">
                      <div>
                        <span className="text-gray-600">Remaining:</span>
                        <div className="font-medium text-green-600">{selectedSLO.budget.remaining}%</div>
                      </div>
                      <div>
                        <span className="text-gray-600">Total:</span>
                        <div className="font-medium">{selectedSLO.budget.total}%</div>
                      </div>
                    </div>
                  </div>
                </Card>
              </div>

              <Card className="p-4">
                <h3 className="text-lg font-medium text-gray-900 mb-3">Description</h3>
                <p className="text-gray-600">{selectedSLO.description}</p>
                <div className="mt-3 grid grid-cols-2 gap-4 text-sm">
                  <div>
                    <span className="text-gray-600">Category:</span>
                    <Badge variant="outline" className="ml-2 capitalize">
                      {selectedSLO.category.replace('_', ' ')}
                    </Badge>
                  </div>
                  <div>
                    <span className="text-gray-600">Metric:</span>
                    <span className="ml-2 font-medium">{selectedSLO.metric}</span>
                  </div>
                </div>
              </Card>

              <Card className="p-4">
                <h3 className="text-lg font-medium text-gray-900 mb-3">Thresholds</h3>
                <div className="grid grid-cols-2 gap-4 text-sm">
                  <div className="p-3 bg-yellow-50 rounded border border-yellow-200">
                    <div className="font-medium text-yellow-900">Warning Threshold</div>
                    <div className="text-yellow-700">{selectedSLO.thresholds.warning}{selectedSLO.unit}</div>
                  </div>
                  <div className="p-3 bg-red-50 rounded border border-red-200">
                    <div className="font-medium text-red-900">Critical Threshold</div>
                    <div className="text-red-700">{selectedSLO.thresholds.critical}{selectedSLO.unit}</div>
                  </div>
                </div>
              </Card>

              <Card className="p-4">
                <h3 className="text-lg font-medium text-gray-900 mb-3">Recent Measurements (24h)</h3>
                <div className="text-sm text-gray-600 mb-2">
                  Pass Rate: {Math.round((selectedSLO.measurements.filter(m => m.status === 'pass').length / selectedSLO.measurements.length) * 100)}%
                </div>
                <div className="h-32 bg-gray-50 rounded flex items-center justify-center">
                  <BarChart3 className="w-12 h-12 text-gray-400" />
                  <span className="ml-2 text-gray-500">Measurement chart would be rendered here</span>
                </div>
              </Card>
            </div>
          )}
        </DialogContent>
      </Dialog>
    </div>
  )
}

export default MapSLOMonitor
