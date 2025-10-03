import React, { useState, useEffect } from 'react'
import { useTranslation } from 'react-i18next'
import { motion } from 'framer-motion'
import {
  WrenchScrewdriverIcon,
  ExclamationTriangleIcon,
  ClockIcon,
  CurrencyDollarIcon,
  ChartBarIcon,
  CalendarIcon,
  BoltIcon,
  Cog6ToothIcon,
} from '@heroicons/react/24/outline'

// Components
import { Card, CardContent, CardHeader, CardTitle } from '@components/ui/Card'
import { Badge } from '@components/ui/Badge'
import { Button } from '@components/ui/Button'
import { LoadingSpinner } from '@components/ui/LoadingSpinner'
import { DataTable } from '@components/shared/DataTable'

// Charts
import {
  LineChart,
  Line,
  BarChart,
  Bar,
  ScatterChart,
  Scatter,
  XAxis,
  YAxis,
  CartesianGrid,
  Tooltip,
  Legend,
  ResponsiveContainer,
  ReferenceLine,
} from 'recharts'

// Types
interface MaintenancePrediction {
  vehicle_id: string
  asset_tag: string
  maintenance_type: string
  predicted_date: string
  confidence_score: number
  urgency_level: 'low' | 'medium' | 'high' | 'critical'
  estimated_cost: number
  recommended_actions: string[]
  environmental_factors: {
    heat_impact?: string
    dust_exposure?: string
    recommended_season?: string
    cooling_priority?: boolean
  }
}

interface VehicleHealthData {
  vehicle_id: string
  asset_tag: string
  health_score: number
  battery_health: number
  mechanical_health: number
  sensor_health: number
  environmental_impact: number
  maintenance_urgency: string
  recommendations: string[]
  last_updated: string
}

interface MaintenanceSchedule {
  schedule_id: string
  vehicle_id: string
  asset_tag: string
  maintenance_type: string
  scheduled_date: string
  estimated_duration: number
  assigned_technician?: string
  status: 'scheduled' | 'in_progress' | 'completed' | 'overdue'
  priority: 'low' | 'medium' | 'high' | 'critical'
}

interface CostAnalysis {
  total_predicted_costs: number
  cost_by_type: Record<string, number>
  cost_savings_potential: number
  preventive_vs_reactive: {
    preventive: number
    reactive: number
  }
  monthly_trend: Array<{
    month: string
    predicted: number
    actual: number
  }>
}

const PredictiveMaintenanceDashboard: React.FC = () => {
  const { t } = useTranslation()
  const [predictions, setPredictions] = useState<MaintenancePrediction[]>([])
  const [healthData, setHealthData] = useState<VehicleHealthData[]>([])
  const [schedule, setSchedule] = useState<MaintenanceSchedule[]>([])
  const [costAnalysis, setCostAnalysis] = useState<CostAnalysis | null>(null)
  const [loading, setLoading] = useState(true)
  const [error, setError] = useState<string | null>(null)
  const [selectedTimeframe, setSelectedTimeframe] = useState<'7d' | '30d' | '90d'>('30d')

  // Fetch maintenance data
  const fetchMaintenanceData = async () => {
    try {
      setLoading(true)
      
      // Fetch predictions
      const predictionsResponse = await fetch('/api/v1/maintenance/predictions')
      const predictionsData = await predictionsResponse.json()
      setPredictions(predictionsData)

      // Fetch vehicle health data
      const healthResponse = await fetch('/api/v1/analytics/vehicle-health')
      const healthData = await healthResponse.json()
      setHealthData(healthData)

      // Fetch maintenance schedule
      const scheduleResponse = await fetch('/api/v1/maintenance/schedule')
      const scheduleData = await scheduleResponse.json()
      setSchedule(scheduleData)

      // Fetch cost analysis
      const costResponse = await fetch(`/api/v1/analytics/maintenance-costs?timeframe=${selectedTimeframe}`)
      const costData = await costResponse.json()
      setCostAnalysis(costData)

      setError(null)
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Failed to fetch maintenance data')
    } finally {
      setLoading(false)
    }
  }

  useEffect(() => {
    fetchMaintenanceData()
  }, [selectedTimeframe])

  if (loading) {
    return (
      <div className="flex items-center justify-center h-64">
        <LoadingSpinner />
      </div>
    )
  }

  if (error) {
    return (
      <div className="flex items-center justify-center h-64">
        <div className="text-center">
          <ExclamationTriangleIcon className="h-12 w-12 text-red-500 mx-auto mb-4" />
          <p className="text-gray-600">{error}</p>
          <Button onClick={fetchMaintenanceData} className="mt-4">
            Retry
          </Button>
        </div>
      </div>
    )
  }

  // Calculate summary metrics
  const criticalPredictions = predictions.filter(p => p.urgency_level === 'critical').length
  const highPredictions = predictions.filter(p => p.urgency_level === 'high').length
  const avgHealthScore = healthData.length > 0 
    ? healthData.reduce((sum, v) => sum + v.health_score, 0) / healthData.length 
    : 0
  const overdueMaintenance = schedule.filter(s => s.status === 'overdue').length

  // Prepare chart data
  const healthScoreDistribution = healthData.map(vehicle => ({
    vehicle: vehicle.asset_tag,
    health: vehicle.health_score,
    battery: vehicle.battery_health,
    mechanical: vehicle.mechanical_health,
    sensor: vehicle.sensor_health,
    environmental: vehicle.environmental_impact,
  }))

  const maintenanceCostTrend = costAnalysis?.monthly_trend || []

  const urgencyDistribution = [
    { name: 'Critical', value: predictions.filter(p => p.urgency_level === 'critical').length, color: '#EF4444' },
    { name: 'High', value: predictions.filter(p => p.urgency_level === 'high').length, color: '#F59E0B' },
    { name: 'Medium', value: predictions.filter(p => p.urgency_level === 'medium').length, color: '#3B82F6' },
    { name: 'Low', value: predictions.filter(p => p.urgency_level === 'low').length, color: '#10B981' },
  ]

  // Table columns for predictions
  const predictionColumns = [
    {
      header: 'Vehicle',
      accessorKey: 'asset_tag',
      cell: ({ row }: any) => (
        <div className="font-medium">{row.original.asset_tag}</div>
      ),
    },
    {
      header: 'Maintenance Type',
      accessorKey: 'maintenance_type',
      cell: ({ row }: any) => (
        <Badge variant="outline">{row.original.maintenance_type}</Badge>
      ),
    },
    {
      header: 'Predicted Date',
      accessorKey: 'predicted_date',
      cell: ({ row }: any) => (
        <div className="flex items-center">
          <CalendarIcon className="h-4 w-4 mr-2 text-gray-500" />
          {new Date(row.original.predicted_date).toLocaleDateString()}
        </div>
      ),
    },
    {
      header: 'Urgency',
      accessorKey: 'urgency_level',
      cell: ({ row }: any) => (
        <Badge variant={
          row.original.urgency_level === 'critical' ? 'destructive' :
          row.original.urgency_level === 'high' ? 'warning' :
          row.original.urgency_level === 'medium' ? 'default' :
          'success'
        }>
          {row.original.urgency_level}
        </Badge>
      ),
    },
    {
      header: 'Confidence',
      accessorKey: 'confidence_score',
      cell: ({ row }: any) => (
        <div className="flex items-center">
          <div className="w-16 bg-gray-200 rounded-full h-2 mr-2">
            <div 
              className="bg-blue-600 h-2 rounded-full" 
              style={{ width: `${row.original.confidence_score * 100}%` }}
            />
          </div>
          <span className="text-sm">{(row.original.confidence_score * 100).toFixed(0)}%</span>
        </div>
      ),
    },
    {
      header: 'Est. Cost',
      accessorKey: 'estimated_cost',
      cell: ({ row }: any) => (
        <div className="font-medium">${row.original.estimated_cost.toLocaleString()}</div>
      ),
    },
  ]

  return (
    <div className="space-y-6 p-6">
      {/* Header */}
      <div className="flex items-center justify-between">
        <div>
          <h1 className="text-3xl font-bold text-gray-900">Predictive Maintenance</h1>
          <p className="text-gray-600">
            AI-powered maintenance predictions for Abu Dhabi fleet operations
          </p>
        </div>
        <div className="flex items-center space-x-4">
          <select
            value={selectedTimeframe}
            onChange={(e) => setSelectedTimeframe(e.target.value as '7d' | '30d' | '90d')}
            className="border border-gray-300 rounded-md px-3 py-2"
          >
            <option value="7d">Last 7 days</option>
            <option value="30d">Last 30 days</option>
            <option value="90d">Last 90 days</option>
          </select>
          <Button onClick={fetchMaintenanceData} variant="outline">
            Refresh
          </Button>
        </div>
      </div>

      {/* Summary Cards */}
      <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 gap-6">
        <Card>
          <CardHeader className="flex flex-row items-center justify-between space-y-0 pb-2">
            <CardTitle className="text-sm font-medium">Critical Predictions</CardTitle>
            <ExclamationTriangleIcon className="h-4 w-4 text-red-500" />
          </CardHeader>
          <CardContent>
            <div className="text-2xl font-bold text-red-600">{criticalPredictions}</div>
            <p className="text-xs text-muted-foreground">
              Require immediate attention
            </p>
          </CardContent>
        </Card>

        <Card>
          <CardHeader className="flex flex-row items-center justify-between space-y-0 pb-2">
            <CardTitle className="text-sm font-medium">High Priority</CardTitle>
            <WrenchScrewdriverIcon className="h-4 w-4 text-orange-500" />
          </CardHeader>
          <CardContent>
            <div className="text-2xl font-bold text-orange-600">{highPredictions}</div>
            <p className="text-xs text-muted-foreground">
              Schedule within 7 days
            </p>
          </CardContent>
        </Card>

        <Card>
          <CardHeader className="flex flex-row items-center justify-between space-y-0 pb-2">
            <CardTitle className="text-sm font-medium">Avg Health Score</CardTitle>
            <ChartBarIcon className="h-4 w-4 text-blue-500" />
          </CardHeader>
          <CardContent>
            <div className="text-2xl font-bold text-blue-600">
              {avgHealthScore.toFixed(1)}
            </div>
            <p className="text-xs text-muted-foreground">
              Fleet average (0-1 scale)
            </p>
          </CardContent>
        </Card>

        <Card>
          <CardHeader className="flex flex-row items-center justify-between space-y-0 pb-2">
            <CardTitle className="text-sm font-medium">Overdue Items</CardTitle>
            <ClockIcon className="h-4 w-4 text-red-500" />
          </CardHeader>
          <CardContent>
            <div className="text-2xl font-bold text-red-600">{overdueMaintenance}</div>
            <p className="text-xs text-muted-foreground">
              Past due maintenance
            </p>
          </CardContent>
        </Card>
      </div>

      {/* Charts Row */}
      <div className="grid grid-cols-1 lg:grid-cols-2 gap-6">
        {/* Vehicle Health Scores */}
        <Card>
          <CardHeader>
            <CardTitle>Vehicle Health Distribution</CardTitle>
          </CardHeader>
          <CardContent>
            <ResponsiveContainer width="100%" height={300}>
              <ScatterChart data={healthScoreDistribution}>
                <CartesianGrid strokeDasharray="3 3" />
                <XAxis 
                  dataKey="vehicle" 
                  angle={-45}
                  textAnchor="end"
                  height={80}
                />
                <YAxis domain={[0, 1]} />
                <Tooltip 
                  formatter={(value: number, name: string) => [
                    (value * 100).toFixed(1) + '%', 
                    name.charAt(0).toUpperCase() + name.slice(1)
                  ]}
                />
                <Scatter dataKey="health" fill="#3B82F6" name="Overall Health" />
                <ReferenceLine y={0.8} stroke="#10B981" strokeDasharray="5 5" label="Target" />
                <ReferenceLine y={0.6} stroke="#F59E0B" strokeDasharray="5 5" label="Warning" />
                <ReferenceLine y={0.4} stroke="#EF4444" strokeDasharray="5 5" label="Critical" />
              </ScatterChart>
            </ResponsiveContainer>
          </CardContent>
        </Card>

        {/* Maintenance Cost Trend */}
        <Card>
          <CardHeader>
            <CardTitle>Maintenance Cost Trend</CardTitle>
          </CardHeader>
          <CardContent>
            <ResponsiveContainer width="100%" height={300}>
              <LineChart data={maintenanceCostTrend}>
                <CartesianGrid strokeDasharray="3 3" />
                <XAxis dataKey="month" />
                <YAxis />
                <Tooltip formatter={(value: number) => [`$${value.toLocaleString()}`, '']} />
                <Legend />
                <Line 
                  type="monotone" 
                  dataKey="predicted" 
                  stroke="#3B82F6" 
                  strokeWidth={2}
                  name="Predicted Costs"
                />
                <Line 
                  type="monotone" 
                  dataKey="actual" 
                  stroke="#10B981" 
                  strokeWidth={2}
                  name="Actual Costs"
                />
              </LineChart>
            </ResponsiveContainer>
          </CardContent>
        </Card>
      </div>

      {/* Abu Dhabi Environmental Factors */}
      <Card>
        <CardHeader>
          <CardTitle className="flex items-center">
            <BoltIcon className="h-5 w-5 mr-2 text-yellow-500" />
            Abu Dhabi Environmental Impact Analysis
          </CardTitle>
        </CardHeader>
        <CardContent>
          <div className="grid grid-cols-1 md:grid-cols-3 gap-6">
            <div className="bg-orange-50 p-4 rounded-lg">
              <h3 className="font-semibold text-orange-800 mb-2">Heat Impact</h3>
              <p className="text-sm text-orange-700">
                Extreme temperatures (45°C+) increase battery degradation by 15-20%.
                Recommend increased cooling system maintenance frequency.
              </p>
              <div className="mt-3">
                <Badge variant="warning">High Risk: Summer Months</Badge>
              </div>
            </div>
            
            <div className="bg-yellow-50 p-4 rounded-lg">
              <h3 className="font-semibold text-yellow-800 mb-2">Dust & Sand Exposure</h3>
              <p className="text-sm text-yellow-700">
                Desert conditions require 2x filter replacement frequency.
                Monitor air intake systems and sensor cleaning schedules.
              </p>
              <div className="mt-3">
                <Badge variant="warning">Sandstorm Season: Mar-May</Badge>
              </div>
            </div>
            
            <div className="bg-blue-50 p-4 rounded-lg">
              <h3 className="font-semibold text-blue-800 mb-2">Optimal Maintenance Windows</h3>
              <p className="text-sm text-blue-700">
                Best maintenance periods: Oct-Mar (cooler weather).
                Schedule major overhauls during winter months for efficiency.
              </p>
              <div className="mt-3">
                <Badge variant="default">Recommended: Winter Schedule</Badge>
              </div>
            </div>
          </div>
        </CardContent>
      </Card>

      {/* Maintenance Predictions Table */}
      <Card>
        <CardHeader>
          <CardTitle>Upcoming Maintenance Predictions</CardTitle>
        </CardHeader>
        <CardContent>
          <DataTable
            columns={predictionColumns}
            data={predictions}
            searchPlaceholder="Search vehicles..."
            pageSize={10}
          />
        </CardContent>
      </Card>

      {/* Cost Analysis */}
      {costAnalysis && (
        <Card>
          <CardHeader>
            <CardTitle className="flex items-center">
              <CurrencyDollarIcon className="h-5 w-5 mr-2 text-green-500" />
              Cost Analysis & Savings Potential
            </CardTitle>
          </CardHeader>
          <CardContent>
            <div className="grid grid-cols-1 md:grid-cols-4 gap-6">
              <div className="text-center">
                <div className="text-2xl font-bold text-gray-900">
                  ${costAnalysis.total_predicted_costs.toLocaleString()}
                </div>
                <p className="text-sm text-gray-600">Total Predicted Costs</p>
              </div>
              
              <div className="text-center">
                <div className="text-2xl font-bold text-green-600">
                  ${costAnalysis.cost_savings_potential.toLocaleString()}
                </div>
                <p className="text-sm text-gray-600">Potential Savings</p>
              </div>
              
              <div className="text-center">
                <div className="text-2xl font-bold text-blue-600">
                  ${costAnalysis.preventive_vs_reactive.preventive.toLocaleString()}
                </div>
                <p className="text-sm text-gray-600">Preventive Costs</p>
              </div>
              
              <div className="text-center">
                <div className="text-2xl font-bold text-red-600">
                  ${costAnalysis.preventive_vs_reactive.reactive.toLocaleString()}
                </div>
                <p className="text-sm text-gray-600">Reactive Costs</p>
              </div>
            </div>
            
            <div className="mt-6 p-4 bg-green-50 rounded-lg">
              <h4 className="font-semibold text-green-800 mb-2">Cost Optimization Recommendations</h4>
              <ul className="text-sm text-green-700 space-y-1">
                <li>• Shift to 80% preventive maintenance to reduce reactive costs by 40%</li>
                <li>• Implement predictive analytics to optimize part inventory and reduce downtime</li>
                <li>• Schedule major maintenance during Abu Dhabi's cooler months (Oct-Mar)</li>
                <li>• Consider bulk purchasing agreements for high-frequency replacement parts</li>
              </ul>
            </div>
          </CardContent>
        </Card>
      )}

      {/* Action Items */}
      <Card>
        <CardHeader>
          <CardTitle className="flex items-center">
            <Cog6ToothIcon className="h-5 w-5 mr-2 text-purple-500" />
            Recommended Actions
          </CardTitle>
        </CardHeader>
        <CardContent>
          <div className="space-y-4">
            {criticalPredictions > 0 && (
              <motion.div
                initial={{ opacity: 0, x: -10 }}
                animate={{ opacity: 1, x: 0 }}
                className="flex items-center justify-between p-4 bg-red-50 rounded-lg border border-red-200"
              >
                <div>
                  <h4 className="font-semibold text-red-800">Immediate Action Required</h4>
                  <p className="text-sm text-red-700">
                    {criticalPredictions} vehicles require critical maintenance within 24-48 hours
                  </p>
                </div>
                <Button variant="destructive" size="sm">
                  Schedule Now
                </Button>
              </motion.div>
            )}
            
            {highPredictions > 0 && (
              <motion.div
                initial={{ opacity: 0, x: -10 }}
                animate={{ opacity: 1, x: 0 }}
                transition={{ delay: 0.1 }}
                className="flex items-center justify-between p-4 bg-orange-50 rounded-lg border border-orange-200"
              >
                <div>
                  <h4 className="font-semibold text-orange-800">Schedule This Week</h4>
                  <p className="text-sm text-orange-700">
                    {highPredictions} vehicles need maintenance scheduled within 7 days
                  </p>
                </div>
                <Button variant="warning" size="sm">
                  Plan Schedule
                </Button>
              </motion.div>
            )}
            
            <motion.div
              initial={{ opacity: 0, x: -10 }}
              animate={{ opacity: 1, x: 0 }}
              transition={{ delay: 0.2 }}
              className="flex items-center justify-between p-4 bg-blue-50 rounded-lg border border-blue-200"
            >
              <div>
                <h4 className="font-semibold text-blue-800">Optimize Inventory</h4>
                <p className="text-sm text-blue-700">
                  Review parts inventory based on predictive maintenance schedule
                </p>
              </div>
              <Button variant="outline" size="sm">
                Review Inventory
              </Button>
            </motion.div>
          </div>
        </CardContent>
      </Card>
    </div>
  )
}

export default PredictiveMaintenanceDashboard
