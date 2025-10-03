import React, { useState, useEffect, useMemo } from 'react';
import { motion, AnimatePresence } from 'framer-motion';
import {
  WrenchScrewdriverIcon,
  ExclamationTriangleIcon,
  ClockIcon,
  CurrencyDollarIcon,
  ChartBarIcon,
  BoltIcon,
  Cog6ToothIcon,
  CalendarDaysIcon,
  TruckIcon,
  CheckCircleIcon,
  XCircleIcon,
  InformationCircleIcon,
  ArrowTrendingUpIcon,
  ArrowTrendingDownIcon,
} from '@heroicons/react/24/outline';
import { useQuery } from '@tanstack/react-query';
import { Line, Bar, Scatter, Doughnut } from 'react-chartjs-2';
import {
  Chart as ChartJS,
  CategoryScale,
  LinearScale,
  PointElement,
  LineElement,
  BarElement,
  ArcElement,
  Title,
  Tooltip,
  Legend,
  Filler,
} from 'chart.js';

ChartJS.register(
  CategoryScale,
  LinearScale,
  PointElement,
  LineElement,
  BarElement,
  ArcElement,
  Title,
  Tooltip,
  Legend,
  Filler
);

interface MaintenanceData {
  predictions: MaintenancePrediction[];
  healthOverview: HealthOverview;
  costAnalysis: MaintenanceCostAnalysis;
  scheduleOptimization: ScheduleOptimization;
  riskAssessment: MaintenanceRiskAssessment;
  abuDhabiFactors: AbuDhabiMaintenanceFactors;
  historicalTrends: HistoricalMaintenanceTrends;
}

interface MaintenancePrediction {
  vehicleId: string;
  assetTag: string;
  predictedDate: string;
  maintenanceType: string;
  priority: 'critical' | 'high' | 'medium' | 'low';
  confidence: number;
  estimatedCostAED: number;
  estimatedDurationHours: number;
  riskScore: number;
  recommendedAction: string;
  components: ComponentPrediction[];
  abuDhabiFactors?: AbuDhabiMaintenanceFactor[];
}

interface ComponentPrediction {
  componentName: string;
  currentCondition: number; // 0-100%
  predictedFailureDate: string;
  replacementCostAED: number;
  criticalityLevel: string;
}

interface AbuDhabiMaintenanceFactor {
  factor: string;
  impact: string;
  recommendation: string;
}

interface HealthOverview {
  totalVehicles: number;
  healthyVehicles: number;
  atRiskVehicles: number;
  criticalVehicles: number;
  avgHealthScore: number;
  maintenanceDueCount: number;
  overdueMaintenanceCount: number;
}

interface MaintenanceCostAnalysis {
  totalProjectedCostAED: number;
  preventiveCostAED: number;
  correctiveCostAED: number;
  emergencyCostAED: number;
  costSavingsFromPredictiveAED: number;
  avgCostPerVehicleAED: number;
  monthlyProjections: MonthlyProjection[];
}

interface MonthlyProjection {
  month: string;
  projectedCostAED: number;
  seasonalFactor: number;
  confidence: number;
  abuDhabiPremium: number;
}

interface ScheduleOptimization {
  optimalMaintenanceWindows: MaintenanceWindow[];
  resourceUtilization: ResourceUtilization;
  workloadDistribution: WorkloadDistribution[];
  recommendations: string[];
}

interface MaintenanceWindow {
  startDate: string;
  endDate: string;
  vehicleCount: number;
  estimatedCostAED: number;
  seasonalSuitability: string;
  abuDhabiConsiderations: string[];
}

interface ResourceUtilization {
  technicians: {
    available: number;
    utilized: number;
    efficiency: number;
  };
  bays: {
    total: number;
    occupied: number;
    utilization: number;
  };
  equipment: {
    available: number;
    inUse: number;
    maintenanceRequired: number;
  };
}

interface WorkloadDistribution {
  week: string;
  scheduledMaintenance: number;
  predictedBreakdowns: number;
  emergencyRepairs: number;
  capacity: number;
}

interface MaintenanceRiskAssessment {
  overallRiskScore: number;
  riskFactors: RiskFactor[];
  mitigationStrategies: MitigationStrategy[];
  businessImpactAnalysis: BusinessImpactAnalysis;
}

interface RiskFactor {
  factor: string;
  probability: number;
  impact: string;
  affectedVehicles: number;
  estimatedCostAED: number;
}

interface MitigationStrategy {
  strategy: string;
  effectiveness: number;
  implementationCostAED: number;
  timeframe: string;
}

interface BusinessImpactAnalysis {
  potentialDowntimeHours: number;
  revenueAtRiskAED: number;
  customerImpactScore: number;
  reputationRiskLevel: string;
}

interface AbuDhabiMaintenanceFactors {
  extremeHeatImpact: number;
  dustStormAdjustments: number;
  seasonalMaintenanceLoad: SeasonalLoad[];
  culturalConsiderations: CulturalConsideration[];
  regulatoryCompliance: ComplianceMetric[];
}

interface SeasonalLoad {
  season: string;
  loadMultiplier: number;
  primaryConcerns: string[];
  recommendedActions: string[];
}

interface CulturalConsideration {
  factor: string;
  impact: string;
  adjustment: string;
}

interface ComplianceMetric {
  regulation: string;
  complianceLevel: number;
  requirements: string[];
}

interface HistoricalMaintenanceTrends {
  monthlyBreakdowns: MonthlyBreakdown[];
  componentFailureRates: ComponentFailureRate[];
  costTrends: CostTrend[];
  seasonalPatterns: SeasonalPattern[];
}

interface MonthlyBreakdown {
  month: string;
  breakdowns: number;
  preventiveActions: number;
  totalCostAED: number;
}

interface ComponentFailureRate {
  component: string;
  failureRate: number;
  avgLifespanMonths: number;
  replacementCostAED: number;
}

interface CostTrend {
  period: string;
  costAED: number;
  trend: 'increasing' | 'decreasing' | 'stable';
}

interface SeasonalPattern {
  season: string;
  maintenanceIncrease: number;
  primaryIssues: string[];
}

const PredictiveMaintenanceDashboard: React.FC = () => {
  const [selectedTimeHorizon, setSelectedTimeHorizon] = useState('30d');
  const [selectedVehicleFilter, setSelectedVehicleFilter] = useState('all');
  const [selectedPriorityFilter, setSelectedPriorityFilter] = useState('all');
  const [refreshInterval, setRefreshInterval] = useState(60000); // 1 minute

  // Fetch maintenance data
  const { data: maintenanceData, isLoading, error, refetch } = useQuery<MaintenanceData>({
    queryKey: ['predictive-maintenance', selectedTimeHorizon, selectedVehicleFilter, selectedPriorityFilter],
    queryFn: async () => {
      const params = new URLSearchParams({
        horizon: selectedTimeHorizon,
        vehicle_filter: selectedVehicleFilter,
        priority_filter: selectedPriorityFilter,
      });
      
      const response = await fetch(`/api/v1/maintenance/predictive/dashboard?${params}`);
      if (!response.ok) {
        throw new Error('Failed to fetch maintenance data');
      }
      return response.json();
    },
    refetchInterval: refreshInterval,
    staleTime: 30000, // 30 seconds
  });

  // Auto-refresh control
  useEffect(() => {
    const interval = setInterval(() => {
      refetch();
    }, refreshInterval);

    return () => clearInterval(interval);
  }, [refetch, refreshInterval]);

  // Chart configurations
  const chartOptions = useMemo(() => ({
    responsive: true,
    maintainAspectRatio: false,
    plugins: {
      legend: {
        position: 'top' as const,
        labels: {
          usePointStyle: true,
          padding: 15,
          font: { size: 11 },
        },
      },
      tooltip: {
        backgroundColor: 'rgba(0, 0, 0, 0.8)',
        titleColor: 'white',
        bodyColor: 'white',
        borderColor: 'rgba(255, 255, 255, 0.1)',
        borderWidth: 1,
      },
    },
    scales: {
      x: {
        grid: { color: 'rgba(0, 0, 0, 0.1)' },
        ticks: { font: { size: 10 } },
      },
      y: {
        grid: { color: 'rgba(0, 0, 0, 0.1)' },
        ticks: { font: { size: 10 } },
      },
    },
  }), []);

  // Generate maintenance prediction timeline chart
  const generateMaintenanceTimelineChart = () => {
    if (!maintenanceData?.predictions) return null;

    const next30Days = Array.from({ length: 30 }, (_, i) => {
      const date = new Date();
      date.setDate(date.getDate() + i);
      return date.toISOString().split('T')[0];
    });

    const maintenanceByDay = next30Days.map(date => {
      const dayMaintenance = maintenanceData.predictions.filter(pred => 
        pred.predictedDate.split('T')[0] === date
      );
      return {
        date: date,
        count: dayMaintenance.length,
        criticalCount: dayMaintenance.filter(m => m.priority === 'critical').length,
        highCount: dayMaintenance.filter(m => m.priority === 'high').length,
        cost: dayMaintenance.reduce((sum, m) => sum + m.estimatedCostAED, 0),
      };
    });

    return {
      labels: next30Days.map(date => new Date(date).toLocaleDateString('en-US', { month: 'short', day: 'numeric' })),
      datasets: [
        {
          label: 'Critical Priority',
          data: maintenanceByDay.map(d => d.criticalCount),
          backgroundColor: '#EF4444',
          borderColor: '#DC2626',
          borderWidth: 1,
        },
        {
          label: 'High Priority',
          data: maintenanceByDay.map(d => d.highCount),
          backgroundColor: '#F59E0B',
          borderColor: '#D97706',
          borderWidth: 1,
        },
        {
          label: 'Other',
          data: maintenanceByDay.map(d => d.count - d.criticalCount - d.highCount),
          backgroundColor: '#3B82F6',
          borderColor: '#2563EB',
          borderWidth: 1,
        },
      ],
    };
  };

  // Generate health score distribution chart
  const generateHealthDistributionChart = () => {
    if (!maintenanceData?.healthOverview) return null;

    return {
      labels: ['Healthy (80-100%)', 'Good (60-79%)', 'At Risk (40-59%)', 'Critical (<40%)'],
      datasets: [
        {
          data: [
            maintenanceData.healthOverview.healthyVehicles,
            Math.max(0, maintenanceData.healthOverview.totalVehicles - maintenanceData.healthOverview.healthyVehicles - maintenanceData.healthOverview.atRiskVehicles - maintenanceData.healthOverview.criticalVehicles),
            maintenanceData.healthOverview.atRiskVehicles,
            maintenanceData.healthOverview.criticalVehicles,
          ],
          backgroundColor: ['#10B981', '#3B82F6', '#F59E0B', '#EF4444'],
          borderWidth: 0,
        },
      ],
    };
  };

  // Generate cost projection chart
  const generateCostProjectionChart = () => {
    if (!maintenanceData?.costAnalysis?.monthlyProjections) return null;

    return {
      labels: maintenanceData.costAnalysis.monthlyProjections.map(p => p.month),
      datasets: [
        {
          label: 'Projected Cost (AED)',
          data: maintenanceData.costAnalysis.monthlyProjections.map(p => p.projectedCostAED),
          borderColor: '#3B82F6',
          backgroundColor: 'rgba(59, 130, 246, 0.1)',
          fill: true,
          tension: 0.4,
        },
        {
          label: 'Abu Dhabi Premium (AED)',
          data: maintenanceData.costAnalysis.monthlyProjections.map(p => p.abuDhabiPremium || 0),
          borderColor: '#F59E0B',
          backgroundColor: 'rgba(245, 158, 11, 0.1)',
          fill: true,
          tension: 0.4,
        },
      ],
    };
  };

  const getPriorityColor = (priority: string) => {
    switch (priority) {
      case 'critical': return 'text-red-600 bg-red-100';
      case 'high': return 'text-orange-600 bg-orange-100';
      case 'medium': return 'text-yellow-600 bg-yellow-100';
      case 'low': return 'text-green-600 bg-green-100';
      default: return 'text-gray-600 bg-gray-100';
    }
  };

  const getPriorityIcon = (priority: string) => {
    switch (priority) {
      case 'critical': return <ExclamationTriangleIcon className="h-4 w-4" />;
      case 'high': return <ClockIcon className="h-4 w-4" />;
      case 'medium': return <Cog6ToothIcon className="h-4 w-4" />;
      case 'low': return <CheckCircleIcon className="h-4 w-4" />;
      default: return <InformationCircleIcon className="h-4 w-4" />;
    }
  };

  if (isLoading) {
    return (
      <div className="flex items-center justify-center h-64">
        <div className="animate-spin rounded-full h-8 w-8 border-b-2 border-blue-600"></div>
        <span className="ml-2 text-gray-600">Loading maintenance data...</span>
      </div>
    );
  }

  if (error) {
    return (
      <div className="bg-red-50 border border-red-200 rounded-md p-4">
        <div className="flex">
          <ExclamationTriangleIcon className="h-5 w-5 text-red-400" />
          <div className="ml-3">
            <h3 className="text-sm font-medium text-red-800">Error loading maintenance data</h3>
            <p className="mt-1 text-sm text-red-700">
              Failed to load predictive maintenance data. Please try refreshing the page.
            </p>
          </div>
        </div>
      </div>
    );
  }

  return (
    <div className="space-y-6">
      {/* Header with controls */}
      <div className="flex flex-col sm:flex-row sm:items-center sm:justify-between">
        <div>
          <h1 className="text-2xl font-bold text-gray-900">Predictive Maintenance Dashboard</h1>
          <p className="mt-1 text-sm text-gray-500">
            AI-powered maintenance predictions and optimization for Abu Dhabi operations
          </p>
        </div>
        
        <div className="mt-4 sm:mt-0 flex flex-wrap gap-3">
          <select
            value={selectedTimeHorizon}
            onChange={(e) => setSelectedTimeHorizon(e.target.value)}
            className="rounded-md border-gray-300 shadow-sm focus:border-blue-500 focus:ring-blue-500 sm:text-sm"
          >
            <option value="7d">Next 7 Days</option>
            <option value="30d">Next 30 Days</option>
            <option value="90d">Next 90 Days</option>
          </select>
          
          <select
            value={selectedPriorityFilter}
            onChange={(e) => setSelectedPriorityFilter(e.target.value)}
            className="rounded-md border-gray-300 shadow-sm focus:border-blue-500 focus:ring-blue-500 sm:text-sm"
          >
            <option value="all">All Priorities</option>
            <option value="critical">Critical Only</option>
            <option value="high">High Priority</option>
            <option value="medium">Medium Priority</option>
          </select>
          
          <button
            onClick={() => refetch()}
            className="inline-flex items-center px-3 py-2 border border-gray-300 shadow-sm text-sm leading-4 font-medium rounded-md text-gray-700 bg-white hover:bg-gray-50 focus:outline-none focus:ring-2 focus:ring-offset-2 focus:ring-blue-500"
          >
            <ArrowTrendingUpIcon className="h-4 w-4 mr-1" />
            Refresh
          </button>
        </div>
      </div>

      {/* Key Metrics */}
      <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 gap-6">
        <motion.div
          initial={{ opacity: 0, y: 20 }}
          animate={{ opacity: 1, y: 0 }}
          transition={{ duration: 0.5 }}
          className="bg-white overflow-hidden shadow rounded-lg"
        >
          <div className="p-5">
            <div className="flex items-center">
              <div className="flex-shrink-0">
                <WrenchScrewdriverIcon className="h-6 w-6 text-blue-600" />
              </div>
              <div className="ml-5 w-0 flex-1">
                <dl>
                  <dt className="text-sm font-medium text-gray-500 truncate">Maintenance Due</dt>
                  <dd className="text-lg font-medium text-gray-900">
                    {maintenanceData?.healthOverview.maintenanceDueCount || 0}
                  </dd>
                </dl>
              </div>
              <div className="flex-shrink-0">
                {(maintenanceData?.healthOverview.maintenanceDueCount || 0) > 0 ? 
                  <ArrowTrendingUpIcon className="h-5 w-5 text-orange-500" /> :
                  <CheckCircleIcon className="h-5 w-5 text-green-500" />
                }
              </div>
            </div>
          </div>
          <div className="bg-gray-50 px-5 py-3">
            <div className="text-sm">
              <span className="font-medium text-orange-600">
                {maintenanceData?.healthOverview.overdueMaintenanceCount || 0}
              </span>
              <span className="text-gray-500 ml-1">overdue</span>
            </div>
          </div>
        </motion.div>

        <motion.div
          initial={{ opacity: 0, y: 20 }}
          animate={{ opacity: 1, y: 0 }}
          transition={{ duration: 0.5, delay: 0.1 }}
          className="bg-white overflow-hidden shadow rounded-lg"
        >
          <div className="p-5">
            <div className="flex items-center">
              <div className="flex-shrink-0">
                <BoltIcon className="h-6 w-6 text-green-600" />
              </div>
              <div className="ml-5 w-0 flex-1">
                <dl>
                  <dt className="text-sm font-medium text-gray-500 truncate">Avg Health Score</dt>
                  <dd className="text-lg font-medium text-gray-900">
                    {(maintenanceData?.healthOverview.avgHealthScore || 0).toFixed(1)}%
                  </dd>
                </dl>
              </div>
              <div className="flex-shrink-0">
                <ArrowTrendingUpIcon className="h-5 w-5 text-green-500" />
              </div>
            </div>
          </div>
          <div className="bg-gray-50 px-5 py-3">
            <div className="text-sm">
              <span className="font-medium text-green-600">+2.3%</span>
              <span className="text-gray-500 ml-1">from last month</span>
            </div>
          </div>
        </motion.div>

        <motion.div
          initial={{ opacity: 0, y: 20 }}
          animate={{ opacity: 1, y: 0 }}
          transition={{ duration: 0.5, delay: 0.2 }}
          className="bg-white overflow-hidden shadow rounded-lg"
        >
          <div className="p-5">
            <div className="flex items-center">
              <div className="flex-shrink-0">
                <CurrencyDollarIcon className="h-6 w-6 text-yellow-600" />
              </div>
              <div className="ml-5 w-0 flex-1">
                <dl>
                  <dt className="text-sm font-medium text-gray-500 truncate">Projected Cost</dt>
                  <dd className="text-lg font-medium text-gray-900">
                    {(maintenanceData?.costAnalysis.totalProjectedCostAED || 0).toLocaleString()} AED
                  </dd>
                </dl>
              </div>
              <div className="flex-shrink-0">
                <ArrowTrendingDownIcon className="h-5 w-5 text-green-500" />
              </div>
            </div>
          </div>
          <div className="bg-gray-50 px-5 py-3">
            <div className="text-sm">
              <span className="font-medium text-green-600">
                -{((maintenanceData?.costAnalysis.costSavingsFromPredictiveAED || 0) / 1000).toFixed(0)}k AED
              </span>
              <span className="text-gray-500 ml-1">savings</span>
            </div>
          </div>
        </motion.div>

        <motion.div
          initial={{ opacity: 0, y: 20 }}
          animate={{ opacity: 1, y: 0 }}
          transition={{ duration: 0.5, delay: 0.3 }}
          className="bg-white overflow-hidden shadow rounded-lg"
        >
          <div className="p-5">
            <div className="flex items-center">
              <div className="flex-shrink-0">
                <ExclamationTriangleIcon className="h-6 w-6 text-red-600" />
              </div>
              <div className="ml-5 w-0 flex-1">
                <dl>
                  <dt className="text-sm font-medium text-gray-500 truncate">Critical Vehicles</dt>
                  <dd className="text-lg font-medium text-gray-900">
                    {maintenanceData?.healthOverview.criticalVehicles || 0}
                  </dd>
                </dl>
              </div>
              <div className="flex-shrink-0">
                <ArrowTrendingDownIcon className="h-5 w-5 text-green-500" />
              </div>
            </div>
          </div>
          <div className="bg-gray-50 px-5 py-3">
            <div className="text-sm">
              <span className="font-medium text-green-600">-2</span>
              <span className="text-gray-500 ml-1">from last week</span>
            </div>
          </div>
        </motion.div>
      </div>

      {/* Charts Grid */}
      <div className="grid grid-cols-1 lg:grid-cols-2 gap-6">
        {/* Maintenance Timeline */}
        <motion.div
          initial={{ opacity: 0, scale: 0.95 }}
          animate={{ opacity: 1, scale: 1 }}
          transition={{ duration: 0.5, delay: 0.4 }}
          className="bg-white shadow rounded-lg p-6"
        >
          <div className="flex items-center justify-between mb-4">
            <h3 className="text-lg font-medium text-gray-900">Maintenance Timeline (30 Days)</h3>
            <CalendarDaysIcon className="h-5 w-5 text-gray-400" />
          </div>
          <div className="h-64">
            <Bar data={generateMaintenanceTimelineChart()} options={chartOptions} />
          </div>
        </motion.div>

        {/* Health Distribution */}
        <motion.div
          initial={{ opacity: 0, scale: 0.95 }}
          animate={{ opacity: 1, scale: 1 }}
          transition={{ duration: 0.5, delay: 0.5 }}
          className="bg-white shadow rounded-lg p-6"
        >
          <div className="flex items-center justify-between mb-4">
            <h3 className="text-lg font-medium text-gray-900">Fleet Health Distribution</h3>
            <ChartBarIcon className="h-5 w-5 text-gray-400" />
          </div>
          <div className="h-64">
            <Doughnut data={generateHealthDistributionChart()} options={chartOptions} />
          </div>
        </motion.div>

        {/* Cost Projections */}
        <motion.div
          initial={{ opacity: 0, scale: 0.95 }}
          animate={{ opacity: 1, scale: 1 }}
          transition={{ duration: 0.5, delay: 0.6 }}
          className="bg-white shadow rounded-lg p-6 lg:col-span-2"
        >
          <div className="flex items-center justify-between mb-4">
            <h3 className="text-lg font-medium text-gray-900">Cost Projections & Abu Dhabi Factors</h3>
            <div className="flex items-center space-x-2">
              <span className="text-xs text-gray-500">🇦🇪 UAE Specific</span>
              <CurrencyDollarIcon className="h-5 w-5 text-gray-400" />
            </div>
          </div>
          <div className="h-64">
            <Line data={generateCostProjectionChart()} options={chartOptions} />
          </div>
        </motion.div>
      </div>

      {/* Maintenance Predictions List */}
      <motion.div
        initial={{ opacity: 0, y: 20 }}
        animate={{ opacity: 1, y: 0 }}
        transition={{ duration: 0.5, delay: 0.7 }}
        className="bg-white shadow rounded-lg"
      >
        <div className="px-6 py-4 border-b border-gray-200">
          <h3 className="text-lg font-medium text-gray-900">Upcoming Maintenance Predictions</h3>
          <p className="mt-1 text-sm text-gray-500">
            AI-powered predictions with confidence scores and cost estimates
          </p>
        </div>
        <div className="overflow-hidden">
          <div className="overflow-x-auto">
            <table className="min-w-full divide-y divide-gray-200">
              <thead className="bg-gray-50">
                <tr>
                  <th className="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider">
                    Vehicle
                  </th>
                  <th className="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider">
                    Maintenance Type
                  </th>
                  <th className="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider">
                    Priority
                  </th>
                  <th className="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider">
                    Predicted Date
                  </th>
                  <th className="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider">
                    Confidence
                  </th>
                  <th className="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider">
                    Est. Cost (AED)
                  </th>
                  <th className="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider">
                    Action
                  </th>
                </tr>
              </thead>
              <tbody className="bg-white divide-y divide-gray-200">
                {maintenanceData?.predictions?.slice(0, 10).map((prediction, index) => (
                  <motion.tr
                    key={prediction.vehicleId}
                    initial={{ opacity: 0, x: -20 }}
                    animate={{ opacity: 1, x: 0 }}
                    transition={{ duration: 0.3, delay: index * 0.05 }}
                    className="hover:bg-gray-50"
                  >
                    <td className="px-6 py-4 whitespace-nowrap">
                      <div className="flex items-center">
                        <TruckIcon className="h-5 w-5 text-gray-400 mr-2" />
                        <div>
                          <div className="text-sm font-medium text-gray-900">
                            {prediction.assetTag}
                          </div>
                          <div className="text-sm text-gray-500">
                            {prediction.vehicleId.substring(0, 8)}...
                          </div>
                        </div>
                      </div>
                    </td>
                    <td className="px-6 py-4 whitespace-nowrap">
                      <div className="text-sm text-gray-900">{prediction.maintenanceType}</div>
                      <div className="text-sm text-gray-500">
                        {prediction.estimatedDurationHours}h estimated
                      </div>
                    </td>
                    <td className="px-6 py-4 whitespace-nowrap">
                      <span className={`inline-flex items-center px-2.5 py-0.5 rounded-full text-xs font-medium ${getPriorityColor(prediction.priority)}`}>
                        {getPriorityIcon(prediction.priority)}
                        <span className="ml-1 capitalize">{prediction.priority}</span>
                      </span>
                    </td>
                    <td className="px-6 py-4 whitespace-nowrap text-sm text-gray-900">
                      {new Date(prediction.predictedDate).toLocaleDateString()}
                    </td>
                    <td className="px-6 py-4 whitespace-nowrap">
                      <div className="flex items-center">
                        <div className="w-16 bg-gray-200 rounded-full h-2 mr-2">
                          <div
                            className="bg-blue-600 h-2 rounded-full"
                            style={{ width: `${prediction.confidence * 100}%` }}
                          ></div>
                        </div>
                        <span className="text-sm text-gray-900">
                          {(prediction.confidence * 100).toFixed(0)}%
                        </span>
                      </div>
                    </td>
                    <td className="px-6 py-4 whitespace-nowrap text-sm text-gray-900">
                      {prediction.estimatedCostAED.toLocaleString()}
                    </td>
                    <td className="px-6 py-4 whitespace-nowrap text-right text-sm font-medium">
                      <button className="text-blue-600 hover:text-blue-900">
                        Schedule
                      </button>
                    </td>
                  </motion.tr>
                ))}
              </tbody>
            </table>
          </div>
        </div>
      </motion.div>

      {/* Abu Dhabi Specific Factors */}
      <motion.div
        initial={{ opacity: 0, y: 20 }}
        animate={{ opacity: 1, y: 0 }}
        transition={{ duration: 0.5, delay: 0.8 }}
        className="bg-white shadow rounded-lg p-6"
      >
        <div className="flex items-center justify-between mb-6">
          <h3 className="text-lg font-medium text-gray-900">Abu Dhabi Environmental Factors</h3>
          <div className="flex items-center space-x-2">
            <span className="text-xs text-gray-500">🇦🇪 UAE Specific</span>
          </div>
        </div>
        
        <div className="grid grid-cols-1 md:grid-cols-3 gap-6">
          {/* Extreme Heat Impact */}
          <div className="bg-orange-50 rounded-lg p-4">
            <div className="flex items-center mb-3">
              <div className="w-3 h-3 bg-orange-500 rounded-full mr-2"></div>
              <h4 className="text-sm font-medium text-gray-900">Extreme Heat Impact</h4>
            </div>
            <div className="text-2xl font-bold text-orange-600 mb-1">
              +{maintenanceData?.abuDhabiFactors?.extremeHeatImpact || 25}%
            </div>
            <div className="text-xs text-gray-600">
              Increased maintenance frequency during summer months (June-September)
            </div>
          </div>

          {/* Dust Storm Adjustments */}
          <div className="bg-yellow-50 rounded-lg p-4">
            <div className="flex items-center mb-3">
              <div className="w-3 h-3 bg-yellow-500 rounded-full mr-2"></div>
              <h4 className="text-sm font-medium text-gray-900">Dust Protection</h4>
            </div>
            <div className="text-2xl font-bold text-yellow-600 mb-1">
              +{maintenanceData?.abuDhabiFactors?.dustStormAdjustments || 15}%
            </div>
            <div className="text-xs text-gray-600">
              Enhanced filtration and cleaning requirements for desert conditions
            </div>
          </div>

          {/* Seasonal Maintenance Load */}
          <div className="bg-blue-50 rounded-lg p-4">
            <div className="flex items-center mb-3">
              <div className="w-3 h-3 bg-blue-500 rounded-full mr-2"></div>
              <h4 className="text-sm font-medium text-gray-900">Seasonal Optimization</h4>
            </div>
            <div className="text-2xl font-bold text-blue-600 mb-1">
              Winter
            </div>
            <div className="text-xs text-gray-600">
              Optimal maintenance window: December - February (40% cost reduction)
            </div>
          </div>
        </div>

        {/* Cultural Considerations */}
        <div className="mt-6 pt-6 border-t border-gray-200">
          <h4 className="text-sm font-medium text-gray-900 mb-3">Cultural & Regulatory Considerations</h4>
          <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
            {maintenanceData?.abuDhabiFactors?.culturalConsiderations?.map((consideration, index) => (
              <div key={index} className="flex items-start space-x-3 p-3 bg-gray-50 rounded-md">
                <InformationCircleIcon className="h-5 w-5 text-blue-500 mt-0.5" />
                <div>
                  <div className="text-sm font-medium text-gray-900">{consideration.factor}</div>
                  <div className="text-xs text-gray-600 mt-1">{consideration.adjustment}</div>
                </div>
              </div>
            ))}
          </div>
        </div>
      </motion.div>
    </div>
  );
};

export default PredictiveMaintenanceDashboard;
