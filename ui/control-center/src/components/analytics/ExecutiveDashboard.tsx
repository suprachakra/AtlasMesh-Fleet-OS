import React, { useState, useEffect } from 'react'
import { useTranslation } from 'react-i18next'
import { motion } from 'framer-motion'
import {
  ChartBarIcon,
  TruckIcon,
  CurrencyDollarIcon,
  ShieldCheckIcon,
  ExclamationTriangleIcon,
  ArrowUpIcon,
  ArrowDownIcon,
  ClockIcon,
  MapPinIcon,
} from '@heroicons/react/24/outline'

// Components
import { Card, CardContent, CardHeader, CardTitle } from '@components/ui/Card'
import { Badge } from '@components/ui/Badge'
import { Button } from '@components/ui/Button'
import { LoadingSpinner } from '@components/ui/LoadingSpinner'

// Charts
import {
  LineChart,
  Line,
  AreaChart,
  Area,
  BarChart,
  Bar,
  PieChart,
  Pie,
  Cell,
  XAxis,
  YAxis,
  CartesianGrid,
  Tooltip,
  Legend,
  ResponsiveContainer,
} from 'recharts'

// Types
interface ExecutiveDashboardData {
  timestamp: string
  fleet_overview: FleetOverviewMetrics
  operational_metrics: OperationalMetrics
  financial_metrics: FinancialMetrics
  safety_metrics: SafetyMetrics
  compliance_metrics: ComplianceMetrics
  environmental_metrics: EnvironmentalMetrics
  trends: Record<string, DataPoint[]>
  alerts: Alert[]
  recommendations: string[]
}

interface FleetOverviewMetrics {
  total_vehicles: number
  active_vehicles: number
  utilization_rate: number
  availability_rate: number
  maintenance_vehicles: number
  offline_vehicles: number
  avg_health_score: number
}

interface OperationalMetrics {
  total_trips: number
  completed_trips: number
  cancelled_trips: number
  avg_trip_duration_minutes: number
  total_distance_km: number
  avg_speed_kmh: number
  on_time_performance: number
  customer_satisfaction: number
}

interface FinancialMetrics {
  revenue: number
  operating_costs: number
  fuel_costs: number
  maintenance_costs: number
  cost_per_km: number
  cost_per_trip: number
  profitability: number
  roi: number
}

interface SafetyMetrics {
  safety_score: number
  incidents: number
  critical_incidents: number
  near_misses: number
  human_interventions: number
  mtbf_hours: number
  safety_training: number
}

interface ComplianceMetrics {
  compliance_score: number
  active_violations: number
  resolved_violations: number
  audit_readiness: number
  certification_status: string
  regulatory_updates: number
}

interface EnvironmentalMetrics {
  co2_emissions_kg: number
  fuel_consumption_liters: number
  energy_efficiency: number
  carbon_footprint: number
  sustainability_score: number
  weather_impact: string
}

interface DataPoint {
  timestamp: string
  value: number
}

interface Alert {
  id: string
  type: string
  severity: 'low' | 'medium' | 'high' | 'critical'
  title: string
  description: string
  vehicle_id?: string
  timestamp: string
  status: string
}

const ExecutiveDashboard: React.FC = () => {
  const { t } = useTranslation()
  const [dashboardData, setDashboardData] = useState<ExecutiveDashboardData | null>(null)
  const [loading, setLoading] = useState(true)
  const [error, setError] = useState<string | null>(null)
  const [refreshInterval, setRefreshInterval] = useState<NodeJS.Timeout | null>(null)

  // Fetch dashboard data
  const fetchDashboardData = async () => {
    try {
      const response = await fetch('/api/v1/dashboards/executive')
      if (!response.ok) {
        throw new Error('Failed to fetch dashboard data')
      }
      const data = await response.json()
      setDashboardData(data)
      setError(null)
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Unknown error')
    } finally {
      setLoading(false)
    }
  }

  useEffect(() => {
    fetchDashboardData()

    // Set up auto-refresh every 5 minutes
    const interval = setInterval(fetchDashboardData, 5 * 60 * 1000)
    setRefreshInterval(interval)

    return () => {
      if (refreshInterval) {
        clearInterval(refreshInterval)
      }
    }
  }, [])

  if (loading) {
    return (
      <div className="flex items-center justify-center h-64">
        <LoadingSpinner />
      </div>
    )
  }

  if (error || !dashboardData) {
    return (
      <div className="flex items-center justify-center h-64">
        <div className="text-center">
          <ExclamationTriangleIcon className="h-12 w-12 text-red-500 mx-auto mb-4" />
          <p className="text-gray-600">{error || 'No data available'}</p>
          <Button onClick={fetchDashboardData} className="mt-4">
            Retry
          </Button>
        </div>
      </div>
    )
  }

  const {
    fleet_overview,
    operational_metrics,
    financial_metrics,
    safety_metrics,
    compliance_metrics,
    environmental_metrics,
    trends,
    alerts,
    recommendations,
  } = dashboardData

  // Chart colors
  const colors = {
    primary: '#3B82F6',
    secondary: '#10B981',
    warning: '#F59E0B',
    danger: '#EF4444',
    success: '#22C55E',
  }

  // Prepare chart data
  const utilizationTrendData = trends.utilization?.map(point => ({
    time: new Date(point.timestamp).toLocaleTimeString('en-US', { hour: '2-digit', minute: '2-digit' }),
    vehicles: point.value,
  })) || []

  const fleetStatusData = [
    { name: 'Active', value: fleet_overview.active_vehicles, color: colors.success },
    { name: 'Maintenance', value: fleet_overview.maintenance_vehicles, color: colors.warning },
    { name: 'Offline', value: fleet_overview.offline_vehicles, color: colors.danger },
  ]

  const performanceData = [
    { metric: 'Utilization', value: fleet_overview.utilization_rate, target: 85 },
    { metric: 'Availability', value: fleet_overview.availability_rate, target: 95 },
    { metric: 'On-Time', value: operational_metrics.on_time_performance, target: 90 },
    { metric: 'Safety Score', value: safety_metrics.safety_score, target: 95 },
  ]

  return (
    <div className="space-y-6 p-6">
      {/* Header */}
      <div className="flex items-center justify-between">
        <div>
          <h1 className="text-3xl font-bold text-gray-900">Executive Dashboard</h1>
          <p className="text-gray-600">
            Abu Dhabi Autonomous Fleet Operations - {new Date().toLocaleDateString()}
          </p>
        </div>
        <div className="flex items-center space-x-4">
          <Badge variant="success">Live Data</Badge>
          <Button onClick={fetchDashboardData} variant="outline">
            Refresh
          </Button>
        </div>
      </div>

      {/* Key Metrics Cards */}
      <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 gap-6">
        {/* Fleet Overview */}
        <Card>
          <CardHeader className="flex flex-row items-center justify-between space-y-0 pb-2">
            <CardTitle className="text-sm font-medium">Fleet Utilization</CardTitle>
            <TruckIcon className="h-4 w-4 text-muted-foreground" />
          </CardHeader>
          <CardContent>
            <div className="text-2xl font-bold">
              {fleet_overview.utilization_rate.toFixed(1)}%
            </div>
            <p className="text-xs text-muted-foreground">
              {fleet_overview.active_vehicles} of {fleet_overview.total_vehicles} vehicles active
            </p>
            <div className="flex items-center mt-2">
              {fleet_overview.utilization_rate > 80 ? (
                <ArrowUpIcon className="h-4 w-4 text-green-500" />
              ) : (
                <ArrowDownIcon className="h-4 w-4 text-red-500" />
              )}
              <span className={`text-xs ml-1 ${
                fleet_overview.utilization_rate > 80 ? 'text-green-500' : 'text-red-500'
              }`}>
                Target: 85%
              </span>
            </div>
          </CardContent>
        </Card>

        {/* Financial Performance */}
        <Card>
          <CardHeader className="flex flex-row items-center justify-between space-y-0 pb-2">
            <CardTitle className="text-sm font-medium">Revenue (24h)</CardTitle>
            <CurrencyDollarIcon className="h-4 w-4 text-muted-foreground" />
          </CardHeader>
          <CardContent>
            <div className="text-2xl font-bold">
              ${financial_metrics.revenue.toLocaleString()}
            </div>
            <p className="text-xs text-muted-foreground">
              ROI: {financial_metrics.roi.toFixed(1)}%
            </p>
            <div className="flex items-center mt-2">
              {financial_metrics.roi > 15 ? (
                <ArrowUpIcon className="h-4 w-4 text-green-500" />
              ) : (
                <ArrowDownIcon className="h-4 w-4 text-red-500" />
              )}
              <span className={`text-xs ml-1 ${
                financial_metrics.roi > 15 ? 'text-green-500' : 'text-red-500'
              }`}>
                Target: 15%
              </span>
            </div>
          </CardContent>
        </Card>

        {/* Safety Score */}
        <Card>
          <CardHeader className="flex flex-row items-center justify-between space-y-0 pb-2">
            <CardTitle className="text-sm font-medium">Safety Score</CardTitle>
            <ShieldCheckIcon className="h-4 w-4 text-muted-foreground" />
          </CardHeader>
          <CardContent>
            <div className="text-2xl font-bold">
              {safety_metrics.safety_score.toFixed(1)}
            </div>
            <p className="text-xs text-muted-foreground">
              {safety_metrics.incidents} incidents (24h)
            </p>
            <div className="flex items-center mt-2">
              {safety_metrics.safety_score > 90 ? (
                <ArrowUpIcon className="h-4 w-4 text-green-500" />
              ) : (
                <ArrowDownIcon className="h-4 w-4 text-red-500" />
              )}
              <span className={`text-xs ml-1 ${
                safety_metrics.safety_score > 90 ? 'text-green-500' : 'text-red-500'
              }`}>
                Target: 95
              </span>
            </div>
          </CardContent>
        </Card>

        {/* Compliance */}
        <Card>
          <CardHeader className="flex flex-row items-center justify-between space-y-0 pb-2">
            <CardTitle className="text-sm font-medium">Compliance</CardTitle>
            <ChartBarIcon className="h-4 w-4 text-muted-foreground" />
          </CardHeader>
          <CardContent>
            <div className="text-2xl font-bold">
              {compliance_metrics.compliance_score.toFixed(1)}%
            </div>
            <p className="text-xs text-muted-foreground">
              {compliance_metrics.active_violations} active violations
            </p>
            <Badge 
              variant={compliance_metrics.certification_status === 'compliant' ? 'success' : 'warning'}
              className="mt-2"
            >
              {compliance_metrics.certification_status}
            </Badge>
          </CardContent>
        </Card>
      </div>

      {/* Charts Row */}
      <div className="grid grid-cols-1 lg:grid-cols-2 gap-6">
        {/* Fleet Utilization Trend */}
        <Card>
          <CardHeader>
            <CardTitle>Fleet Utilization Trend (24h)</CardTitle>
          </CardHeader>
          <CardContent>
            <ResponsiveContainer width="100%" height={300}>
              <AreaChart data={utilizationTrendData}>
                <CartesianGrid strokeDasharray="3 3" />
                <XAxis dataKey="time" />
                <YAxis />
                <Tooltip />
                <Area
                  type="monotone"
                  dataKey="vehicles"
                  stroke={colors.primary}
                  fill={colors.primary}
                  fillOpacity={0.3}
                />
              </AreaChart>
            </ResponsiveContainer>
          </CardContent>
        </Card>

        {/* Fleet Status Distribution */}
        <Card>
          <CardHeader>
            <CardTitle>Fleet Status Distribution</CardTitle>
          </CardHeader>
          <CardContent>
            <ResponsiveContainer width="100%" height={300}>
              <PieChart>
                <Pie
                  data={fleetStatusData}
                  cx="50%"
                  cy="50%"
                  labelLine={false}
                  label={({ name, value }) => `${name}: ${value}`}
                  outerRadius={80}
                  fill="#8884d8"
                  dataKey="value"
                >
                  {fleetStatusData.map((entry, index) => (
                    <Cell key={`cell-${index}`} fill={entry.color} />
                  ))}
                </Pie>
                <Tooltip />
              </PieChart>
            </ResponsiveContainer>
          </CardContent>
        </Card>
      </div>

      {/* Performance Metrics */}
      <Card>
        <CardHeader>
          <CardTitle>Key Performance Indicators</CardTitle>
        </CardHeader>
        <CardContent>
          <ResponsiveContainer width="100%" height={300}>
            <BarChart data={performanceData}>
              <CartesianGrid strokeDasharray="3 3" />
              <XAxis dataKey="metric" />
              <YAxis domain={[0, 100]} />
              <Tooltip />
              <Legend />
              <Bar dataKey="value" fill={colors.primary} name="Current" />
              <Bar dataKey="target" fill={colors.secondary} name="Target" />
            </BarChart>
          </ResponsiveContainer>
        </CardContent>
      </Card>

      {/* Alerts and Recommendations */}
      <div className="grid grid-cols-1 lg:grid-cols-2 gap-6">
        {/* Active Alerts */}
        <Card>
          <CardHeader>
            <CardTitle className="flex items-center">
              <ExclamationTriangleIcon className="h-5 w-5 mr-2 text-yellow-500" />
              Active Alerts ({alerts.length})
            </CardTitle>
          </CardHeader>
          <CardContent>
            <div className="space-y-3 max-h-64 overflow-y-auto">
              {alerts.length === 0 ? (
                <p className="text-gray-500 text-center py-4">No active alerts</p>
              ) : (
                alerts.map((alert) => (
                  <motion.div
                    key={alert.id}
                    initial={{ opacity: 0, y: 10 }}
                    animate={{ opacity: 1, y: 0 }}
                    className="flex items-start space-x-3 p-3 bg-gray-50 rounded-lg"
                  >
                    <div className={`w-2 h-2 rounded-full mt-2 ${
                      alert.severity === 'critical' ? 'bg-red-500' :
                      alert.severity === 'high' ? 'bg-orange-500' :
                      alert.severity === 'medium' ? 'bg-yellow-500' :
                      'bg-blue-500'
                    }`} />
                    <div className="flex-1">
                      <div className="flex items-center justify-between">
                        <h4 className="font-medium text-sm">{alert.title}</h4>
                        <Badge variant={
                          alert.severity === 'critical' ? 'destructive' :
                          alert.severity === 'high' ? 'warning' :
                          'default'
                        }>
                          {alert.severity}
                        </Badge>
                      </div>
                      <p className="text-xs text-gray-600 mt-1">{alert.description}</p>
                      {alert.vehicle_id && (
                        <p className="text-xs text-blue-600 mt-1">Vehicle: {alert.vehicle_id}</p>
                      )}
                      <div className="flex items-center text-xs text-gray-500 mt-2">
                        <ClockIcon className="h-3 w-3 mr-1" />
                        {new Date(alert.timestamp).toLocaleString()}
                      </div>
                    </div>
                  </motion.div>
                ))
              )}
            </div>
          </CardContent>
        </Card>

        {/* Recommendations */}
        <Card>
          <CardHeader>
            <CardTitle>AI Recommendations</CardTitle>
          </CardHeader>
          <CardContent>
            <div className="space-y-3 max-h-64 overflow-y-auto">
              {recommendations.length === 0 ? (
                <p className="text-gray-500 text-center py-4">No recommendations available</p>
              ) : (
                recommendations.map((recommendation, index) => (
                  <motion.div
                    key={index}
                    initial={{ opacity: 0, x: -10 }}
                    animate={{ opacity: 1, x: 0 }}
                    transition={{ delay: index * 0.1 }}
                    className="flex items-start space-x-3 p-3 bg-blue-50 rounded-lg"
                  >
                    <div className="w-2 h-2 bg-blue-500 rounded-full mt-2" />
                    <p className="text-sm text-gray-700">{recommendation}</p>
                  </motion.div>
                ))
              )}
            </div>
          </CardContent>
        </Card>
      </div>

      {/* Environmental Impact (Abu Dhabi Specific) */}
      <Card>
        <CardHeader>
          <CardTitle>Environmental Impact - UAE Sustainability Goals</CardTitle>
        </CardHeader>
        <CardContent>
          <div className="grid grid-cols-1 md:grid-cols-4 gap-4">
            <div className="text-center">
              <div className="text-2xl font-bold text-green-600">
                {environmental_metrics.co2_emissions_kg.toFixed(1)} kg
              </div>
              <p className="text-sm text-gray-600">CO₂ Emissions (24h)</p>
            </div>
            <div className="text-center">
              <div className="text-2xl font-bold text-blue-600">
                {environmental_metrics.fuel_consumption_liters.toFixed(1)} L
              </div>
              <p className="text-sm text-gray-600">Fuel Consumption</p>
            </div>
            <div className="text-center">
              <div className="text-2xl font-bold text-purple-600">
                {environmental_metrics.energy_efficiency.toFixed(1)} km/L
              </div>
              <p className="text-sm text-gray-600">Energy Efficiency</p>
            </div>
            <div className="text-center">
              <div className="text-2xl font-bold text-orange-600">
                {environmental_metrics.sustainability_score.toFixed(1)}%
              </div>
              <p className="text-sm text-gray-600">Sustainability Score</p>
            </div>
          </div>
          <div className="mt-4 p-3 bg-green-50 rounded-lg">
            <p className="text-sm text-green-800">
              🇦🇪 Contributing to UAE Vision 2071: Net Zero by 2050 initiative
            </p>
          </div>
        </CardContent>
      </Card>
    </div>
  )
}

export default ExecutiveDashboard
