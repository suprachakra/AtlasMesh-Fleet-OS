import React, { useState, useEffect, useCallback, useMemo } from 'react'
import { 
  Activity, AlertTriangle, TrendingUp, TrendingDown, Wrench, Calendar,
  Battery, Thermometer, Gauge, Settings, RefreshCw, Download, Filter,
  CheckCircle, XCircle, Clock, BarChart3, LineChart, PieChart, Target,
  Zap, Shield, Tool, Cpu, HardDrive, Wifi, Camera, Navigation
} from 'lucide-react'
import { Button } from '../ui/Button'
import { Badge } from '../ui/Badge'
import { Card } from '../ui/Card'
import { Select } from '../ui/Select'
import { Input } from '../ui/Input'
import { Progress } from '../ui/Progress'
import { Tabs, TabsContent, TabsList, TabsTrigger } from '../ui/Tabs'
import { Alert, AlertDescription } from '../ui/Alert'

// Types
interface ComponentHealth {
  id: string
  name: string
  category: 'powertrain' | 'sensors' | 'compute' | 'safety' | 'comfort' | 'connectivity'
  type: string
  vehicleId: string
  currentStatus: 'healthy' | 'warning' | 'critical' | 'failed' | 'maintenance_due'
  healthScore: number // 0-100
  remainingUsefulLife: RemainingUsefulLife
  predictiveModel: PredictiveModel
  maintenanceSchedule: MaintenanceSchedule
  historicalTrends: HealthTrend[]
  anomalies: Anomaly[]
  recommendations: MaintenanceRecommendation[]
  lastUpdated: Date
  nextPrediction: Date
}

interface RemainingUsefulLife {
  value: number
  unit: 'hours' | 'days' | 'cycles' | 'kilometers' | 'months'
  confidence: number // 0-100
  degradationRate: number // per unit time
  criticalThreshold: number
  warningThreshold: number
  estimatedFailureDate: Date
  uncertainty: number // ± days/hours
}

interface PredictiveModel {
  id: string
  name: string
  version: string
  type: 'regression' | 'classification' | 'time_series' | 'anomaly_detection' | 'ensemble'
  algorithm: string
  accuracy: number // 0-100
  precision: number // 0-100
  recall: number // 0-100
  f1Score: number // 0-100
  trainedOn: Date
  lastRetrained: Date
  nextRetraining: Date
  features: ModelFeature[]
  hyperparameters: Record<string, any>
  performance: ModelPerformance
}

interface ModelFeature {
  name: string
  importance: number // 0-1
  type: 'numerical' | 'categorical' | 'temporal' | 'derived'
  description: string
  dataSource: string
}

interface ModelPerformance {
  validationScore: number
  crossValidationScore: number
  testScore: number
  confusionMatrix?: number[][]
  rocAuc?: number
  prCurve?: { precision: number[], recall: number[] }
  featureImportance: { feature: string, importance: number }[]
}

interface MaintenanceSchedule {
  nextScheduled: Date
  type: 'preventive' | 'predictive' | 'corrective' | 'emergency'
  priority: 'low' | 'medium' | 'high' | 'critical'
  estimatedDuration: number // hours
  estimatedCost: number
  requiredParts: RequiredPart[]
  requiredSkills: string[]
  downtime: number // hours
  workOrderId?: string
  assignedTechnician?: string
}

interface RequiredPart {
  partId: string
  partName: string
  quantity: number
  availability: 'in_stock' | 'order_required' | 'backordered'
  leadTime: number // days
  cost: number
}

interface HealthTrend {
  timestamp: Date
  healthScore: number
  rulValue: number
  confidence: number
  anomalyScore?: number
  contextualFactors: Record<string, any>
}

interface Anomaly {
  id: string
  timestamp: Date
  type: 'deviation' | 'pattern_change' | 'threshold_breach' | 'correlation_break'
  severity: 'low' | 'medium' | 'high' | 'critical'
  description: string
  affectedMetrics: string[]
  confidence: number
  rootCause?: string
  impact: string
  resolved: boolean
  resolvedAt?: Date
  resolution?: string
}

interface MaintenanceRecommendation {
  id: string
  type: 'immediate' | 'scheduled' | 'monitor' | 'replace' | 'inspect'
  priority: 'low' | 'medium' | 'high' | 'critical'
  description: string
  rationale: string
  estimatedCost: number
  estimatedBenefit: number
  roi: number
  riskReduction: number
  timeframe: string
  prerequisites: string[]
  alternatives: Alternative[]
  confidence: number
}

interface Alternative {
  description: string
  cost: number
  benefit: number
  risk: string
  timeframe: string
}

interface FleetHealth {
  totalVehicles: number
  healthyVehicles: number
  warningVehicles: number
  criticalVehicles: number
  failedComponents: number
  maintenanceDue: number
  averageHealthScore: number
  predictedFailures: PredictedFailure[]
  maintenanceCostProjection: CostProjection
  fleetAvailability: number
}

interface PredictedFailure {
  componentId: string
  vehicleId: string
  componentName: string
  estimatedFailureDate: Date
  confidence: number
  impact: 'low' | 'medium' | 'high' | 'critical'
  preventiveCost: number
  correctiveCost: number
  downtime: number
}

interface CostProjection {
  period: 'week' | 'month' | 'quarter' | 'year'
  predictedCost: number
  preventiveCost: number
  correctiveCost: number
  emergencyCost: number
  costSavings: number
  breakdown: CostBreakdown[]
}

interface CostBreakdown {
  category: string
  amount: number
  percentage: number
  trend: 'increasing' | 'decreasing' | 'stable'
}

interface PredictiveMaintenanceDashboardProps {
  vehicleId?: string
  fleetId?: string
  componentType?: string
  timeRange?: 'day' | 'week' | 'month' | 'quarter' | 'year'
  onMaintenanceScheduled?: (schedule: MaintenanceSchedule) => void
  onAnomalyDetected?: (anomaly: Anomaly) => void
  className?: string
}

const PredictiveMaintenanceDashboard: React.FC<PredictiveMaintenanceDashboardProps> = ({
  vehicleId,
  fleetId,
  componentType,
  timeRange = 'month',
  onMaintenanceScheduled,
  onAnomalyDetected,
  className = ''
}) => {
  // State
  const [components, setComponents] = useState<ComponentHealth[]>([])
  const [fleetHealth, setFleetHealth] = useState<FleetHealth | null>(null)
  const [selectedComponent, setSelectedComponent] = useState<ComponentHealth | null>(null)
  const [activeTab, setActiveTab] = useState('overview')
  const [filters, setFilters] = useState({
    status: '',
    category: '',
    vehicle: '',
    priority: ''
  })
  const [loading, setLoading] = useState(false)
  const [autoRefresh, setAutoRefresh] = useState(true)

  // Mock data initialization
  const initializeMockData = useCallback(() => {
    const mockComponents: ComponentHealth[] = [
      {
        id: 'comp-battery-001',
        name: 'Main Battery Pack',
        category: 'powertrain',
        type: 'lithium_ion_battery',
        vehicleId: vehicleId || 'atlas-001',
        currentStatus: 'warning',
        healthScore: 78,
        remainingUsefulLife: {
          value: 145,
          unit: 'days',
          confidence: 89,
          degradationRate: 0.3, // % per month
          criticalThreshold: 60,
          warningThreshold: 75,
          estimatedFailureDate: new Date('2025-05-20T00:00:00Z'),
          uncertainty: 12
        },
        predictiveModel: {
          id: 'model-battery-rul',
          name: 'Battery RUL Predictor',
          version: '2.1.0',
          type: 'regression',
          algorithm: 'Random Forest',
          accuracy: 91.2,
          precision: 89.5,
          recall: 92.1,
          f1Score: 90.8,
          trainedOn: new Date('2024-09-15T10:00:00Z'),
          lastRetrained: new Date('2024-11-01T08:00:00Z'),
          nextRetraining: new Date('2024-12-01T08:00:00Z'),
          features: [
            { name: 'charge_cycles', importance: 0.34, type: 'numerical', description: 'Total charge/discharge cycles', dataSource: 'battery_management_system' },
            { name: 'temperature_avg', importance: 0.28, type: 'numerical', description: 'Average operating temperature', dataSource: 'thermal_sensors' },
            { name: 'depth_of_discharge', importance: 0.22, type: 'numerical', description: 'Average depth of discharge', dataSource: 'battery_management_system' },
            { name: 'fast_charge_frequency', importance: 0.16, type: 'numerical', description: 'Frequency of fast charging', dataSource: 'charging_logs' }
          ],
          hyperparameters: {
            n_estimators: 100,
            max_depth: 10,
            min_samples_split: 5,
            random_state: 42
          },
          performance: {
            validationScore: 0.912,
            crossValidationScore: 0.898,
            testScore: 0.905,
            featureImportance: [
              { feature: 'charge_cycles', importance: 0.34 },
              { feature: 'temperature_avg', importance: 0.28 },
              { feature: 'depth_of_discharge', importance: 0.22 },
              { feature: 'fast_charge_frequency', importance: 0.16 }
            ]
          }
        },
        maintenanceSchedule: {
          nextScheduled: new Date('2024-12-15T09:00:00Z'),
          type: 'predictive',
          priority: 'medium',
          estimatedDuration: 4,
          estimatedCost: 2500,
          requiredParts: [
            {
              partId: 'battery-coolant-001',
              partName: 'Battery Coolant System',
              quantity: 1,
              availability: 'in_stock',
              leadTime: 0,
              cost: 450
            }
          ],
          requiredSkills: ['battery_specialist', 'electrical_technician'],
          downtime: 6
        },
        historicalTrends: [
          { timestamp: new Date('2024-10-26T00:00:00Z'), healthScore: 82, rulValue: 165, confidence: 91 },
          { timestamp: new Date('2024-11-02T00:00:00Z'), healthScore: 80, rulValue: 158, confidence: 90 },
          { timestamp: new Date('2024-11-09T00:00:00Z'), healthScore: 79, rulValue: 152, confidence: 89 },
          { timestamp: new Date('2024-11-16T00:00:00Z'), healthScore: 78, rulValue: 148, confidence: 89 },
          { timestamp: new Date('2024-11-23T00:00:00Z'), healthScore: 78, rulValue: 145, confidence: 89 }
        ],
        anomalies: [
          {
            id: 'anom-battery-001',
            timestamp: new Date('2024-11-20T14:30:00Z'),
            type: 'threshold_breach',
            severity: 'medium',
            description: 'Battery temperature exceeded normal operating range during fast charging',
            affectedMetrics: ['temperature', 'charging_efficiency'],
            confidence: 87,
            impact: 'Accelerated degradation if pattern continues',
            resolved: false
          }
        ],
        recommendations: [
          {
            id: 'rec-battery-001',
            type: 'scheduled',
            priority: 'medium',
            description: 'Replace battery coolant system and perform thermal calibration',
            rationale: 'Temperature anomalies detected, coolant system showing signs of degradation',
            estimatedCost: 2500,
            estimatedBenefit: 8500,
            roi: 240,
            riskReduction: 65,
            timeframe: '2-3 weeks',
            prerequisites: ['vehicle_availability', 'parts_procurement'],
            alternatives: [
              {
                description: 'Monitor closely and replace at next scheduled maintenance',
                cost: 0,
                benefit: 3000,
                risk: 'Higher chance of unexpected failure',
                timeframe: '3 months'
              }
            ],
            confidence: 89
          }
        ],
        lastUpdated: new Date('2024-11-26T10:30:00Z'),
        nextPrediction: new Date('2024-11-27T10:30:00Z')
      },
      {
        id: 'comp-lidar-001',
        name: 'Front LiDAR Sensor',
        category: 'sensors',
        type: 'lidar_sensor',
        vehicleId: vehicleId || 'atlas-001',
        currentStatus: 'healthy',
        healthScore: 94,
        remainingUsefulLife: {
          value: 2.8,
          unit: 'months',
          confidence: 76,
          degradationRate: 2.1, // % per month
          criticalThreshold: 70,
          warningThreshold: 85,
          estimatedFailureDate: new Date('2025-02-15T00:00:00Z'),
          uncertainty: 18
        },
        predictiveModel: {
          id: 'model-lidar-health',
          name: 'LiDAR Health Monitor',
          version: '1.5.2',
          type: 'anomaly_detection',
          algorithm: 'Isolation Forest',
          accuracy: 85.3,
          precision: 82.1,
          recall: 88.9,
          f1Score: 85.4,
          trainedOn: new Date('2024-08-20T14:00:00Z'),
          lastRetrained: new Date('2024-10-20T14:00:00Z'),
          nextRetraining: new Date('2024-12-20T14:00:00Z'),
          features: [
            { name: 'point_cloud_density', importance: 0.31, type: 'numerical', description: 'Density of point cloud data', dataSource: 'lidar_processor' },
            { name: 'range_accuracy', importance: 0.26, type: 'numerical', description: 'Range measurement accuracy', dataSource: 'calibration_system' },
            { name: 'rotation_speed_variance', importance: 0.24, type: 'numerical', description: 'Variance in rotation speed', dataSource: 'motor_controller' },
            { name: 'laser_power_stability', importance: 0.19, type: 'numerical', description: 'Laser power stability', dataSource: 'laser_diagnostics' }
          ],
          hyperparameters: {
            contamination: 0.1,
            n_estimators: 100,
            max_samples: 256,
            random_state: 42
          },
          performance: {
            validationScore: 0.853,
            crossValidationScore: 0.841,
            testScore: 0.847,
            featureImportance: [
              { feature: 'point_cloud_density', importance: 0.31 },
              { feature: 'range_accuracy', importance: 0.26 },
              { feature: 'rotation_speed_variance', importance: 0.24 },
              { feature: 'laser_power_stability', importance: 0.19 }
            ]
          }
        },
        maintenanceSchedule: {
          nextScheduled: new Date('2025-01-15T10:00:00Z'),
          type: 'preventive',
          priority: 'low',
          estimatedDuration: 2,
          estimatedCost: 850,
          requiredParts: [
            {
              partId: 'lidar-lens-001',
              partName: 'LiDAR Protective Lens',
              quantity: 1,
              availability: 'in_stock',
              leadTime: 0,
              cost: 125
            }
          ],
          requiredSkills: ['sensor_technician'],
          downtime: 2
        },
        historicalTrends: [
          { timestamp: new Date('2024-10-26T00:00:00Z'), healthScore: 96, rulValue: 3.2, confidence: 78 },
          { timestamp: new Date('2024-11-02T00:00:00Z'), healthScore: 95, rulValue: 3.1, confidence: 77 },
          { timestamp: new Date('2024-11-09T00:00:00Z'), healthScore: 95, rulValue: 3.0, confidence: 76 },
          { timestamp: new Date('2024-11-16T00:00:00Z'), healthScore: 94, rulValue: 2.9, confidence: 76 },
          { timestamp: new Date('2024-11-23T00:00:00Z'), healthScore: 94, rulValue: 2.8, confidence: 76 }
        ],
        anomalies: [],
        recommendations: [
          {
            id: 'rec-lidar-001',
            type: 'monitor',
            priority: 'low',
            description: 'Continue monitoring sensor performance, schedule lens cleaning',
            rationale: 'Sensor operating within normal parameters, preventive cleaning recommended',
            estimatedCost: 150,
            estimatedBenefit: 500,
            roi: 233,
            riskReduction: 15,
            timeframe: '1 month',
            prerequisites: [],
            alternatives: [],
            confidence: 76
          }
        ],
        lastUpdated: new Date('2024-11-26T10:30:00Z'),
        nextPrediction: new Date('2024-11-27T10:30:00Z')
      },
      {
        id: 'comp-brake-001',
        name: 'Front Brake System',
        category: 'safety',
        type: 'disc_brake',
        vehicleId: vehicleId || 'atlas-001',
        currentStatus: 'critical',
        healthScore: 42,
        remainingUsefulLife: {
          value: 18,
          unit: 'days',
          confidence: 94,
          degradationRate: 8.2, // % per month
          criticalThreshold: 40,
          warningThreshold: 60,
          estimatedFailureDate: new Date('2024-12-14T00:00:00Z'),
          uncertainty: 3
        },
        predictiveModel: {
          id: 'model-brake-wear',
          name: 'Brake Wear Predictor',
          version: '3.0.1',
          type: 'time_series',
          algorithm: 'LSTM Neural Network',
          accuracy: 96.7,
          precision: 95.8,
          recall: 97.3,
          f1Score: 96.5,
          trainedOn: new Date('2024-07-10T12:00:00Z'),
          lastRetrained: new Date('2024-10-10T12:00:00Z'),
          nextRetraining: new Date('2024-01-10T12:00:00Z'),
          features: [
            { name: 'pad_thickness', importance: 0.42, type: 'numerical', description: 'Brake pad thickness', dataSource: 'wear_sensors' },
            { name: 'brake_temperature', importance: 0.31, type: 'numerical', description: 'Brake disc temperature', dataSource: 'thermal_sensors' },
            { name: 'braking_frequency', importance: 0.18, type: 'numerical', description: 'Frequency of brake applications', dataSource: 'brake_controller' },
            { name: 'vehicle_weight', importance: 0.09, type: 'numerical', description: 'Vehicle weight during braking', dataSource: 'load_sensors' }
          ],
          hyperparameters: {
            lstm_units: 50,
            dropout: 0.2,
            learning_rate: 0.001,
            batch_size: 32,
            epochs: 100
          },
          performance: {
            validationScore: 0.967,
            crossValidationScore: 0.962,
            testScore: 0.965,
            featureImportance: [
              { feature: 'pad_thickness', importance: 0.42 },
              { feature: 'brake_temperature', importance: 0.31 },
              { feature: 'braking_frequency', importance: 0.18 },
              { feature: 'vehicle_weight', importance: 0.09 }
            ]
          }
        },
        maintenanceSchedule: {
          nextScheduled: new Date('2024-12-02T08:00:00Z'),
          type: 'emergency',
          priority: 'critical',
          estimatedDuration: 6,
          estimatedCost: 1200,
          requiredParts: [
            {
              partId: 'brake-pad-front-001',
              partName: 'Front Brake Pads (Set)',
              quantity: 1,
              availability: 'in_stock',
              leadTime: 0,
              cost: 280
            },
            {
              partId: 'brake-disc-front-001',
              partName: 'Front Brake Disc',
              quantity: 2,
              availability: 'order_required',
              leadTime: 2,
              cost: 450
            }
          ],
          requiredSkills: ['brake_specialist', 'safety_inspector'],
          downtime: 8,
          workOrderId: 'WO-BRAKE-20241202-001'
        },
        historicalTrends: [
          { timestamp: new Date('2024-10-26T00:00:00Z'), healthScore: 68, rulValue: 45, confidence: 92 },
          { timestamp: new Date('2024-11-02T00:00:00Z'), healthScore: 62, rulValue: 38, confidence: 93 },
          { timestamp: new Date('2024-11-09T00:00:00Z'), healthScore: 55, rulValue: 32, confidence: 94 },
          { timestamp: new Date('2024-11-16T00:00:00Z'), healthScore: 48, rulValue: 25, confidence: 94 },
          { timestamp: new Date('2024-11-23T00:00:00Z'), healthScore: 42, rulValue: 18, confidence: 94 }
        ],
        anomalies: [
          {
            id: 'anom-brake-001',
            timestamp: new Date('2024-11-24T16:20:00Z'),
            type: 'threshold_breach',
            severity: 'critical',
            description: 'Brake pad thickness below safety threshold',
            affectedMetrics: ['pad_thickness', 'braking_distance'],
            confidence: 98,
            rootCause: 'Accelerated wear due to heavy usage pattern',
            impact: 'Safety risk - immediate maintenance required',
            resolved: false
          }
        ],
        recommendations: [
          {
            id: 'rec-brake-001',
            type: 'immediate',
            priority: 'critical',
            description: 'URGENT: Replace brake pads and discs immediately',
            rationale: 'Brake pad thickness below safety threshold, vehicle should not operate',
            estimatedCost: 1200,
            estimatedBenefit: 15000,
            roi: 1150,
            riskReduction: 95,
            timeframe: 'Immediate',
            prerequisites: ['vehicle_out_of_service', 'emergency_parts_order'],
            alternatives: [],
            confidence: 98
          }
        ],
        lastUpdated: new Date('2024-11-26T10:30:00Z'),
        nextPrediction: new Date('2024-11-26T22:30:00Z')
      }
    ]

    const mockFleetHealth: FleetHealth = {
      totalVehicles: 45,
      healthyVehicles: 38,
      warningVehicles: 5,
      criticalVehicles: 2,
      failedComponents: 1,
      maintenanceDue: 12,
      averageHealthScore: 81.3,
      predictedFailures: [
        {
          componentId: 'comp-brake-001',
          vehicleId: 'atlas-001',
          componentName: 'Front Brake System',
          estimatedFailureDate: new Date('2024-12-14T00:00:00Z'),
          confidence: 94,
          impact: 'critical',
          preventiveCost: 1200,
          correctiveCost: 3500,
          downtime: 8
        },
        {
          componentId: 'comp-battery-002',
          vehicleId: 'atlas-003',
          componentName: 'Main Battery Pack',
          estimatedFailureDate: new Date('2025-01-08T00:00:00Z'),
          confidence: 87,
          impact: 'high',
          preventiveCost: 2500,
          correctiveCost: 8500,
          downtime: 12
        }
      ],
      maintenanceCostProjection: {
        period: 'month',
        predictedCost: 45000,
        preventiveCost: 28000,
        correctiveCost: 12000,
        emergencyCost: 5000,
        costSavings: 18000,
        breakdown: [
          { category: 'Brake Systems', amount: 15000, percentage: 33.3, trend: 'increasing' },
          { category: 'Battery Packs', amount: 12000, percentage: 26.7, trend: 'stable' },
          { category: 'Sensors', amount: 8000, percentage: 17.8, trend: 'decreasing' },
          { category: 'Powertrain', amount: 6000, percentage: 13.3, trend: 'stable' },
          { category: 'Other', amount: 4000, percentage: 8.9, trend: 'stable' }
        ]
      },
      fleetAvailability: 94.2
    }

    setComponents(mockComponents)
    setFleetHealth(mockFleetHealth)
  }, [vehicleId])

  useEffect(() => {
    initializeMockData()
  }, [initializeMockData])

  // Auto-refresh effect
  useEffect(() => {
    if (!autoRefresh) return

    const interval = setInterval(() => {
      // Simulate real-time updates
      setComponents(prev => prev.map(comp => ({
        ...comp,
        lastUpdated: new Date(),
        // Simulate slight health score changes
        healthScore: Math.max(0, Math.min(100, comp.healthScore + (Math.random() - 0.5) * 2))
      })))
    }, 30000) // Update every 30 seconds

    return () => clearInterval(interval)
  }, [autoRefresh])

  // Filtered data
  const filteredComponents = useMemo(() => {
    return components
      .filter(comp => !filters.status || comp.currentStatus === filters.status)
      .filter(comp => !filters.category || comp.category === filters.category)
      .filter(comp => !filters.vehicle || comp.vehicleId === filters.vehicle)
      .filter(comp => !filters.priority || 
        (comp.currentStatus === 'critical' && filters.priority === 'critical') ||
        (comp.currentStatus === 'warning' && filters.priority === 'high') ||
        (comp.currentStatus === 'healthy' && filters.priority === 'low')
      )
      .sort((a, b) => {
        // Sort by priority: critical > warning > healthy
        const statusPriority = { critical: 3, failed: 3, warning: 2, maintenance_due: 2, healthy: 1 }
        const aPriority = statusPriority[a.currentStatus] || 0
        const bPriority = statusPriority[b.currentStatus] || 0
        
        if (aPriority !== bPriority) {
          return bPriority - aPriority
        }
        
        return a.remainingUsefulLife.value - b.remainingUsefulLife.value
      })
  }, [components, filters])

  // Handlers
  const handleComponentSelect = useCallback((component: ComponentHealth) => {
    setSelectedComponent(component)
  }, [])

  const handleScheduleMaintenance = useCallback((componentId: string) => {
    const component = components.find(c => c.id === componentId)
    if (component) {
      onMaintenanceScheduled?.(component.maintenanceSchedule)
      // Update component status
      setComponents(prev => prev.map(c => 
        c.id === componentId 
          ? { ...c, currentStatus: 'maintenance_due' as const }
          : c
      ))
    }
  }, [components, onMaintenanceScheduled])

  const handleRefreshData = useCallback(() => {
    setLoading(true)
    setTimeout(() => {
      initializeMockData()
      setLoading(false)
    }, 1000)
  }, [initializeMockData])

  // Helper functions
  const getStatusColor = (status: string) => {
    switch (status) {
      case 'healthy': return 'bg-green-100 text-green-800'
      case 'warning': return 'bg-yellow-100 text-yellow-800'
      case 'critical': return 'bg-red-100 text-red-800'
      case 'failed': return 'bg-red-100 text-red-800'
      case 'maintenance_due': return 'bg-blue-100 text-blue-800'
      default: return 'bg-gray-100 text-gray-800'
    }
  }

  const getStatusIcon = (status: string) => {
    switch (status) {
      case 'healthy': return <CheckCircle className="w-4 h-4 text-green-500" />
      case 'warning': return <AlertTriangle className="w-4 h-4 text-yellow-500" />
      case 'critical': return <XCircle className="w-4 h-4 text-red-500" />
      case 'failed': return <XCircle className="w-4 h-4 text-red-500" />
      case 'maintenance_due': return <Clock className="w-4 h-4 text-blue-500" />
      default: return <Activity className="w-4 h-4 text-gray-500" />
    }
  }

  const getCategoryIcon = (category: string) => {
    switch (category) {
      case 'powertrain': return <Zap className="w-5 h-5" />
      case 'sensors': return <Camera className="w-5 h-5" />
      case 'compute': return <Cpu className="w-5 h-5" />
      case 'safety': return <Shield className="w-5 h-5" />
      case 'comfort': return <Settings className="w-5 h-5" />
      case 'connectivity': return <Wifi className="w-5 h-5" />
      default: return <Tool className="w-5 h-5" />
    }
  }

  const formatRUL = (rul: RemainingUsefulLife) => {
    const { value, unit } = rul
    if (value < 1 && unit === 'days') {
      return `${Math.round(value * 24)} hours`
    }
    return `${Math.round(value)} ${unit}`
  }

  const getRULColor = (rul: RemainingUsefulLife) => {
    const percentage = (rul.value / (rul.criticalThreshold + rul.warningThreshold)) * 100
    if (percentage < 25) return 'text-red-600'
    if (percentage < 50) return 'text-yellow-600'
    return 'text-green-600'
  }

  return (
    <div className={`space-y-6 ${className}`}>
      {/* Header */}
      <div className="flex items-center justify-between">
        <div className="flex items-center space-x-3">
          <Activity className="w-6 h-6 text-blue-600" />
          <h2 className="text-2xl font-semibold text-gray-900">Predictive Maintenance Dashboard</h2>
          {vehicleId && (
            <Badge variant="outline">Vehicle: {vehicleId}</Badge>
          )}
          {fleetId && (
            <Badge variant="outline">Fleet: {fleetId}</Badge>
          )}
        </div>
        <div className="flex items-center space-x-2">
          <div className="flex items-center space-x-2">
            <span className="text-sm text-gray-600">Auto-refresh:</span>
            <Button
              variant="outline"
              size="sm"
              onClick={() => setAutoRefresh(!autoRefresh)}
              className={autoRefresh ? 'bg-green-50 text-green-700' : ''}
            >
              {autoRefresh ? 'On' : 'Off'}
            </Button>
          </div>
          <Button variant="outline" onClick={handleRefreshData} disabled={loading}>
            <RefreshCw className={`w-4 h-4 mr-2 ${loading ? 'animate-spin' : ''}`} />
            Refresh
          </Button>
          <Button variant="outline">
            <Download className="w-4 h-4 mr-2" />
            Export
          </Button>
        </div>
      </div>

      {/* Fleet Health Overview */}
      {fleetHealth && (
        <div className="grid grid-cols-2 md:grid-cols-4 lg:grid-cols-8 gap-4">
          <Card className="p-4">
            <div className="text-center">
              <div className="text-2xl font-bold text-gray-900">{fleetHealth.totalVehicles}</div>
              <div className="text-sm text-gray-600">Total Vehicles</div>
            </div>
          </Card>
          <Card className="p-4">
            <div className="text-center">
              <div className="text-2xl font-bold text-green-600">{fleetHealth.healthyVehicles}</div>
              <div className="text-sm text-gray-600">Healthy</div>
            </div>
          </Card>
          <Card className="p-4">
            <div className="text-center">
              <div className="text-2xl font-bold text-yellow-600">{fleetHealth.warningVehicles}</div>
              <div className="text-sm text-gray-600">Warning</div>
            </div>
          </Card>
          <Card className="p-4">
            <div className="text-center">
              <div className="text-2xl font-bold text-red-600">{fleetHealth.criticalVehicles}</div>
              <div className="text-sm text-gray-600">Critical</div>
            </div>
          </Card>
          <Card className="p-4">
            <div className="text-center">
              <div className="text-2xl font-bold text-blue-600">{fleetHealth.maintenanceDue}</div>
              <div className="text-sm text-gray-600">Due Soon</div>
            </div>
          </Card>
          <Card className="p-4">
            <div className="text-center">
              <div className="text-2xl font-bold text-purple-600">{fleetHealth.averageHealthScore.toFixed(1)}</div>
              <div className="text-sm text-gray-600">Avg Health</div>
            </div>
          </Card>
          <Card className="p-4">
            <div className="text-center">
              <div className="text-2xl font-bold text-indigo-600">{fleetHealth.fleetAvailability.toFixed(1)}%</div>
              <div className="text-sm text-gray-600">Availability</div>
            </div>
          </Card>
          <Card className="p-4">
            <div className="text-center">
              <div className="text-2xl font-bold text-orange-600">${(fleetHealth.maintenanceCostProjection.predictedCost / 1000).toFixed(0)}K</div>
              <div className="text-sm text-gray-600">Monthly Cost</div>
            </div>
          </Card>
        </div>
      )}

      <div className="grid grid-cols-1 lg:grid-cols-4 gap-6">
        {/* Left Panel - Filters & Controls */}
        <div className="lg:col-span-1 space-y-4">
          {/* Tab Navigation */}
          <Card className="p-4">
            <Tabs value={activeTab} onValueChange={setActiveTab}>
              <TabsList className="grid w-full grid-cols-1 gap-1">
                <TabsTrigger value="overview">
                  <BarChart3 className="w-4 h-4 mr-2" />
                  Overview
                </TabsTrigger>
                <TabsTrigger value="components">
                  <Tool className="w-4 h-4 mr-2" />
                  Components
                </TabsTrigger>
                <TabsTrigger value="predictions">
                  <TrendingUp className="w-4 h-4 mr-2" />
                  Predictions
                </TabsTrigger>
                <TabsTrigger value="costs">
                  <Target className="w-4 h-4 mr-2" />
                  Costs
                </TabsTrigger>
              </TabsList>
            </Tabs>
          </Card>

          {/* Filters */}
          <Card className="p-4">
            <h3 className="font-medium text-gray-900 mb-3">Filters</h3>
            <div className="space-y-3">
              <Select
                value={filters.status}
                onValueChange={(value) => setFilters(prev => ({ ...prev, status: value }))}
              >
                <option value="">All Status</option>
                <option value="healthy">Healthy</option>
                <option value="warning">Warning</option>
                <option value="critical">Critical</option>
                <option value="failed">Failed</option>
                <option value="maintenance_due">Maintenance Due</option>
              </Select>

              <Select
                value={filters.category}
                onValueChange={(value) => setFilters(prev => ({ ...prev, category: value }))}
              >
                <option value="">All Categories</option>
                <option value="powertrain">Powertrain</option>
                <option value="sensors">Sensors</option>
                <option value="compute">Compute</option>
                <option value="safety">Safety</option>
                <option value="comfort">Comfort</option>
                <option value="connectivity">Connectivity</option>
              </Select>

              <Select
                value={filters.priority}
                onValueChange={(value) => setFilters(prev => ({ ...prev, priority: value }))}
              >
                <option value="">All Priorities</option>
                <option value="critical">Critical</option>
                <option value="high">High</option>
                <option value="medium">Medium</option>
                <option value="low">Low</option>
              </Select>

              {!vehicleId && (
                <Input
                  placeholder="Filter by vehicle..."
                  value={filters.vehicle}
                  onChange={(e) => setFilters(prev => ({ ...prev, vehicle: e.target.value }))}
                />
              )}
            </div>
          </Card>

          {/* Quick Actions */}
          <Card className="p-4">
            <h3 className="font-medium text-gray-900 mb-3">Quick Actions</h3>
            <div className="space-y-2">
              <Button variant="outline" size="sm" className="w-full justify-start">
                <Calendar className="w-4 h-4 mr-2" />
                Schedule Maintenance
              </Button>
              <Button variant="outline" size="sm" className="w-full justify-start">
                <AlertTriangle className="w-4 h-4 mr-2" />
                View Alerts
              </Button>
              <Button variant="outline" size="sm" className="w-full justify-start">
                <BarChart3 className="w-4 h-4 mr-2" />
                Generate Report
              </Button>
            </div>
          </Card>
        </div>

        {/* Center Panel - Main Content */}
        <div className="lg:col-span-2">
          <Card className="p-4">
            <Tabs value={activeTab} onValueChange={setActiveTab}>
              <TabsContent value="overview" className="space-y-4">
                <h3 className="font-medium text-gray-900">System Health Overview</h3>
                
                {/* Critical Alerts */}
                {filteredComponents.some(c => c.currentStatus === 'critical') && (
                  <Alert className="border-red-200 bg-red-50">
                    <AlertTriangle className="h-4 w-4 text-red-600" />
                    <AlertDescription className="text-red-800">
                      <strong>Critical Components Detected:</strong> {filteredComponents.filter(c => c.currentStatus === 'critical').length} component(s) require immediate attention.
                    </AlertDescription>
                  </Alert>
                )}

                {/* Health Distribution */}
                <div className="grid grid-cols-2 md:grid-cols-4 gap-4">
                  {Object.entries({
                    healthy: { count: filteredComponents.filter(c => c.currentStatus === 'healthy').length, color: 'text-green-600' },
                    warning: { count: filteredComponents.filter(c => c.currentStatus === 'warning').length, color: 'text-yellow-600' },
                    critical: { count: filteredComponents.filter(c => c.currentStatus === 'critical').length, color: 'text-red-600' },
                    maintenance_due: { count: filteredComponents.filter(c => c.currentStatus === 'maintenance_due').length, color: 'text-blue-600' }
                  }).map(([status, { count, color }]) => (
                    <div key={status} className="text-center p-3 bg-gray-50 rounded">
                      <div className={`text-xl font-bold ${color}`}>{count}</div>
                      <div className="text-sm text-gray-600 capitalize">{status.replace('_', ' ')}</div>
                    </div>
                  ))}
                </div>

                {/* Top Issues */}
                <div>
                  <h4 className="font-medium text-gray-900 mb-3">Components Requiring Attention</h4>
                  <div className="space-y-2">
                    {filteredComponents
                      .filter(c => c.currentStatus !== 'healthy')
                      .slice(0, 5)
                      .map(component => (
                        <div
                          key={component.id}
                          className="flex items-center justify-between p-3 bg-gray-50 rounded cursor-pointer hover:bg-gray-100"
                          onClick={() => handleComponentSelect(component)}
                        >
                          <div className="flex items-center space-x-3">
                            {getCategoryIcon(component.category)}
                            <div>
                              <div className="font-medium">{component.name}</div>
                              <div className="text-sm text-gray-600">{component.vehicleId}</div>
                            </div>
                          </div>
                          <div className="flex items-center space-x-2">
                            <Badge className={getStatusColor(component.currentStatus)} size="sm">
                              {component.currentStatus}
                            </Badge>
                            <span className={`text-sm font-medium ${getRULColor(component.remainingUsefulLife)}`}>
                              {formatRUL(component.remainingUsefulLife)}
                            </span>
                          </div>
                        </div>
                      ))}
                  </div>
                </div>
              </TabsContent>

              <TabsContent value="components" className="space-y-4">
                <div className="flex items-center justify-between">
                  <h3 className="font-medium text-gray-900">Components ({filteredComponents.length})</h3>
                </div>
                <div className="space-y-3 max-h-96 overflow-y-auto">
                  {filteredComponents.map(component => (
                    <div
                      key={component.id}
                      className={`p-4 border rounded cursor-pointer transition-colors ${
                        selectedComponent?.id === component.id 
                          ? 'border-blue-500 bg-blue-50' 
                          : 'border-gray-200 hover:border-gray-300'
                      }`}
                      onClick={() => handleComponentSelect(component)}
                    >
                      <div className="flex items-start justify-between mb-2">
                        <div className="flex items-center space-x-3">
                          {getCategoryIcon(component.category)}
                          <div>
                            <h4 className="font-medium text-gray-900">{component.name}</h4>
                            <p className="text-sm text-gray-600">{component.vehicleId} • {component.type}</p>
                          </div>
                        </div>
                        <div className="flex items-center space-x-2">
                          {getStatusIcon(component.currentStatus)}
                          <Badge className={getStatusColor(component.currentStatus)} size="sm">
                            {component.currentStatus}
                          </Badge>
                        </div>
                      </div>
                      
                      <div className="grid grid-cols-3 gap-4 text-sm">
                        <div>
                          <span className="text-gray-600">Health Score:</span>
                          <div className="flex items-center space-x-2">
                            <Progress value={component.healthScore} className="w-16 h-2" />
                            <span className="font-medium">{component.healthScore}%</span>
                          </div>
                        </div>
                        <div>
                          <span className="text-gray-600">RUL:</span>
                          <div className={`font-medium ${getRULColor(component.remainingUsefulLife)}`}>
                            {formatRUL(component.remainingUsefulLife)}
                          </div>
                        </div>
                        <div>
                          <span className="text-gray-600">Confidence:</span>
                          <div className="font-medium">{component.remainingUsefulLife.confidence}%</div>
                        </div>
                      </div>

                      {component.anomalies.length > 0 && (
                        <div className="mt-2 flex items-center space-x-1">
                          <AlertTriangle className="w-3 h-3 text-yellow-500" />
                          <span className="text-xs text-yellow-600">{component.anomalies.length} anomalies detected</span>
                        </div>
                      )}
                    </div>
                  ))}
                </div>
              </TabsContent>

              <TabsContent value="predictions" className="space-y-4">
                <h3 className="font-medium text-gray-900">Failure Predictions</h3>
                {fleetHealth?.predictedFailures.map(prediction => (
                  <div key={prediction.componentId} className="p-4 border rounded">
                    <div className="flex items-start justify-between mb-2">
                      <div>
                        <h4 className="font-medium text-gray-900">{prediction.componentName}</h4>
                        <p className="text-sm text-gray-600">{prediction.vehicleId}</p>
                      </div>
                      <Badge className={
                        prediction.impact === 'critical' ? 'bg-red-100 text-red-800' :
                        prediction.impact === 'high' ? 'bg-orange-100 text-orange-800' :
                        'bg-yellow-100 text-yellow-800'
                      } size="sm">
                        {prediction.impact}
                      </Badge>
                    </div>
                    
                    <div className="grid grid-cols-2 gap-4 text-sm">
                      <div>
                        <span className="text-gray-600">Predicted Failure:</span>
                        <div className="font-medium">{prediction.estimatedFailureDate.toLocaleDateString()}</div>
                      </div>
                      <div>
                        <span className="text-gray-600">Confidence:</span>
                        <div className="font-medium">{prediction.confidence}%</div>
                      </div>
                      <div>
                        <span className="text-gray-600">Preventive Cost:</span>
                        <div className="font-medium text-green-600">${prediction.preventiveCost}</div>
                      </div>
                      <div>
                        <span className="text-gray-600">Corrective Cost:</span>
                        <div className="font-medium text-red-600">${prediction.correctiveCost}</div>
                      </div>
                    </div>
                    
                    <div className="mt-3 flex items-center justify-between">
                      <span className="text-sm text-gray-600">
                        Savings: <span className="font-medium text-green-600">${prediction.correctiveCost - prediction.preventiveCost}</span>
                      </span>
                      <Button size="sm" onClick={() => handleScheduleMaintenance(prediction.componentId)}>
                        Schedule Maintenance
                      </Button>
                    </div>
                  </div>
                ))}
              </TabsContent>

              <TabsContent value="costs" className="space-y-4">
                <h3 className="font-medium text-gray-900">Cost Analysis</h3>
                {fleetHealth?.maintenanceCostProjection && (
                  <>
                    <div className="grid grid-cols-2 md:grid-cols-4 gap-4">
                      <div className="text-center p-3 bg-gray-50 rounded">
                        <div className="text-xl font-bold text-gray-900">
                          ${(fleetHealth.maintenanceCostProjection.predictedCost / 1000).toFixed(0)}K
                        </div>
                        <div className="text-sm text-gray-600">Total Predicted</div>
                      </div>
                      <div className="text-center p-3 bg-green-50 rounded">
                        <div className="text-xl font-bold text-green-600">
                          ${(fleetHealth.maintenanceCostProjection.preventiveCost / 1000).toFixed(0)}K
                        </div>
                        <div className="text-sm text-gray-600">Preventive</div>
                      </div>
                      <div className="text-center p-3 bg-yellow-50 rounded">
                        <div className="text-xl font-bold text-yellow-600">
                          ${(fleetHealth.maintenanceCostProjection.correctiveCost / 1000).toFixed(0)}K
                        </div>
                        <div className="text-sm text-gray-600">Corrective</div>
                      </div>
                      <div className="text-center p-3 bg-blue-50 rounded">
                        <div className="text-xl font-bold text-blue-600">
                          ${(fleetHealth.maintenanceCostProjection.costSavings / 1000).toFixed(0)}K
                        </div>
                        <div className="text-sm text-gray-600">Savings</div>
                      </div>
                    </div>
                    
                    <div>
                      <h4 className="font-medium text-gray-900 mb-3">Cost Breakdown</h4>
                      <div className="space-y-2">
                        {fleetHealth.maintenanceCostProjection.breakdown.map(item => (
                          <div key={item.category} className="flex items-center justify-between p-2 bg-gray-50 rounded">
                            <span className="font-medium">{item.category}</span>
                            <div className="flex items-center space-x-2">
                              <span className="text-sm text-gray-600">{item.percentage.toFixed(1)}%</span>
                              <span className="font-medium">${(item.amount / 1000).toFixed(0)}K</span>
                              {item.trend === 'increasing' && <TrendingUp className="w-3 h-3 text-red-500" />}
                              {item.trend === 'decreasing' && <TrendingDown className="w-3 h-3 text-green-500" />}
                            </div>
                          </div>
                        ))}
                      </div>
                    </div>
                  </>
                )}
              </TabsContent>
            </Tabs>
          </Card>
        </div>

        {/* Right Panel - Details */}
        <div className="lg:col-span-1">
          {selectedComponent ? (
            <Card className="p-4">
              <div className="flex items-center justify-between mb-4">
                <h3 className="font-medium text-gray-900">Component Details</h3>
                <Button variant="outline" size="sm" onClick={() => handleScheduleMaintenance(selectedComponent.id)}>
                  <Calendar className="w-4 h-4 mr-2" />
                  Schedule
                </Button>
              </div>

              <div className="space-y-4">
                <div>
                  <div className="flex items-center space-x-2 mb-2">
                    {getCategoryIcon(selectedComponent.category)}
                    <h4 className="font-medium text-gray-900">{selectedComponent.name}</h4>
                  </div>
                  <p className="text-sm text-gray-600 mb-2">{selectedComponent.vehicleId} • {selectedComponent.type}</p>
                  
                  <div className="flex items-center space-x-2 mb-3">
                    {getStatusIcon(selectedComponent.currentStatus)}
                    <Badge className={getStatusColor(selectedComponent.currentStatus)}>
                      {selectedComponent.currentStatus}
                    </Badge>
                  </div>
                </div>

                <div>
                  <h5 className="font-medium text-gray-900 mb-2">Health Metrics</h5>
                  <div className="space-y-2">
                    <div className="flex justify-between items-center">
                      <span className="text-sm text-gray-600">Health Score:</span>
                      <div className="flex items-center space-x-2">
                        <Progress value={selectedComponent.healthScore} className="w-16 h-2" />
                        <span className="text-sm font-medium">{selectedComponent.healthScore}%</span>
                      </div>
                    </div>
                    <div className="flex justify-between">
                      <span className="text-sm text-gray-600">RUL:</span>
                      <span className={`text-sm font-medium ${getRULColor(selectedComponent.remainingUsefulLife)}`}>
                        {formatRUL(selectedComponent.remainingUsefulLife)}
                      </span>
                    </div>
                    <div className="flex justify-between">
                      <span className="text-sm text-gray-600">Confidence:</span>
                      <span className="text-sm font-medium">{selectedComponent.remainingUsefulLife.confidence}%</span>
                    </div>
                  </div>
                </div>

                <div>
                  <h5 className="font-medium text-gray-900 mb-2">Predictive Model</h5>
                  <div className="text-sm space-y-1">
                    <div className="flex justify-between">
                      <span className="text-gray-600">Algorithm:</span>
                      <span className="font-medium">{selectedComponent.predictiveModel.algorithm}</span>
                    </div>
                    <div className="flex justify-between">
                      <span className="text-gray-600">Accuracy:</span>
                      <span className="font-medium">{selectedComponent.predictiveModel.accuracy.toFixed(1)}%</span>
                    </div>
                    <div className="flex justify-between">
                      <span className="text-gray-600">Last Trained:</span>
                      <span className="font-medium">{selectedComponent.predictiveModel.lastRetrained.toLocaleDateString()}</span>
                    </div>
                  </div>
                </div>

                <div>
                  <h5 className="font-medium text-gray-900 mb-2">Maintenance Schedule</h5>
                  <div className="text-sm space-y-1">
                    <div className="flex justify-between">
                      <span className="text-gray-600">Next Scheduled:</span>
                      <span className="font-medium">{selectedComponent.maintenanceSchedule.nextScheduled.toLocaleDateString()}</span>
                    </div>
                    <div className="flex justify-between">
                      <span className="text-gray-600">Type:</span>
                      <Badge className={
                        selectedComponent.maintenanceSchedule.type === 'emergency' ? 'bg-red-100 text-red-800' :
                        selectedComponent.maintenanceSchedule.type === 'predictive' ? 'bg-blue-100 text-blue-800' :
                        'bg-gray-100 text-gray-800'
                      } size="sm">
                        {selectedComponent.maintenanceSchedule.type}
                      </Badge>
                    </div>
                    <div className="flex justify-between">
                      <span className="text-gray-600">Est. Cost:</span>
                      <span className="font-medium">${selectedComponent.maintenanceSchedule.estimatedCost}</span>
                    </div>
                    <div className="flex justify-between">
                      <span className="text-gray-600">Downtime:</span>
                      <span className="font-medium">{selectedComponent.maintenanceSchedule.downtime}h</span>
                    </div>
                  </div>
                </div>

                {selectedComponent.anomalies.length > 0 && (
                  <div>
                    <h5 className="font-medium text-gray-900 mb-2">Recent Anomalies</h5>
                    <div className="space-y-2">
                      {selectedComponent.anomalies.slice(0, 2).map(anomaly => (
                        <div key={anomaly.id} className="p-2 bg-yellow-50 border border-yellow-200 rounded text-sm">
                          <div className="flex items-center justify-between">
                            <span className="font-medium">{anomaly.type}</span>
                            <Badge className={
                              anomaly.severity === 'critical' ? 'bg-red-100 text-red-800' :
                              anomaly.severity === 'high' ? 'bg-orange-100 text-orange-800' :
                              'bg-yellow-100 text-yellow-800'
                            } size="sm">
                              {anomaly.severity}
                            </Badge>
                          </div>
                          <p className="text-yellow-700 mt-1">{anomaly.description}</p>
                          <p className="text-xs text-yellow-600 mt-1">{anomaly.timestamp.toLocaleString()}</p>
                        </div>
                      ))}
                    </div>
                  </div>
                )}

                {selectedComponent.recommendations.length > 0 && (
                  <div>
                    <h5 className="font-medium text-gray-900 mb-2">Recommendations</h5>
                    <div className="space-y-2">
                      {selectedComponent.recommendations.slice(0, 2).map(rec => (
                        <div key={rec.id} className="p-2 bg-blue-50 border border-blue-200 rounded text-sm">
                          <div className="flex items-center justify-between">
                            <span className="font-medium">{rec.type}</span>
                            <Badge className={
                              rec.priority === 'critical' ? 'bg-red-100 text-red-800' :
                              rec.priority === 'high' ? 'bg-orange-100 text-orange-800' :
                              'bg-blue-100 text-blue-800'
                            } size="sm">
                              {rec.priority}
                            </Badge>
                          </div>
                          <p className="text-blue-700 mt-1">{rec.description}</p>
                          <div className="flex justify-between mt-2 text-xs">
                            <span className="text-blue-600">ROI: {rec.roi}%</span>
                            <span className="text-blue-600">Risk Reduction: {rec.riskReduction}%</span>
                          </div>
                        </div>
                      ))}
                    </div>
                  </div>
                )}
              </div>
            </Card>
          ) : (
            <Card className="p-4 h-96 flex items-center justify-center">
              <div className="text-center">
                <Activity className="w-16 h-16 text-gray-400 mx-auto mb-4" />
                <h3 className="text-lg font-medium text-gray-900 mb-2">No Selection</h3>
                <p className="text-gray-600">
                  Select a component to view detailed health metrics and predictions
                </p>
              </div>
            </Card>
          )}
        </div>
      </div>
    </div>
  )
}

export default PredictiveMaintenanceDashboard
