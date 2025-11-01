import React, { useState, useEffect, useMemo } from 'react';
import { motion, AnimatePresence } from 'framer-motion';
import {
  ChartBarIcon,
  TruckIcon,
  BoltIcon,
  ExclamationTriangleIcon,
  CurrencyDollarIcon,
  ClockIcon,
  MapPinIcon,
  Cog6ToothIcon,
  ArrowTrendingUpIcon,
  ArrowTrendingDownIcon,
  InformationCircleIcon,
} from '@heroicons/react/24/outline';
import { useQuery } from '@tanstack/react-query';
import { Line, Bar, Doughnut, Area } from 'react-chartjs-2';
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
import { apiService } from '../../services/apiService';

// Register Chart.js components
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

interface AnalyticsData {
  fleetOverview: FleetOverview;
  performanceMetrics: PerformanceMetrics;
  operationalMetrics: OperationalMetrics;
  costAnalysis: CostAnalysis;
  healthMetrics: HealthMetrics;
  predictiveInsights: PredictiveInsights;
  abuDhabiMetrics: AbuDhabiMetrics;
}

interface FleetOverview {
  totalVehicles: number;
  activeVehicles: number;
  dispatchedVehicles: number;
  maintenanceVehicles: number;
  offlineVehicles: number;
  utilizationRate: number;
  availabilityRate: number;
  avgHealthScore: number;
}

interface PerformanceMetrics {
  totalDistanceKm: number;
  avgSpeedKmh: number;
  fuelEfficiencyL100km: number;
  energyEfficiencyKwhKm: number;
  operationalHours: number;
  tripCompletionRate: number;
  onTimePerformance: number;
}

interface OperationalMetrics {
  tripsCompleted: number;
  avgTripDuration: number;
  totalPassengers: number;
  totalCargo: number;
  emergencyStops: number;
  maintenanceEvents: number;
  alertsGenerated: number;
}

interface CostAnalysis {
  totalCostsAED: number;
  fuelCostsAED: number;
  maintenanceCostsAED: number;
  operationalCostsAED: number;
  costPerKmAED: number;
  costPerVehiclePerDayAED: number;
  projectedMonthlyCostAED: number;
}

interface HealthMetrics {
  avgBatteryLevel: number;
  avgFuelLevel: number;
  criticalAlerts: number;
  maintenanceDue: number;
  systemFaults: number;
  connectivityIssues: number;
}

interface PredictiveInsights {
  maintenancePredictions: MaintenancePrediction[];
  demandForecast: DemandForecast;
  costProjections: CostProjection[];
  riskAssessment: RiskAssessment;
}

interface MaintenancePrediction {
  vehicleId: string;
  assetTag: string;
  predictedDate: string;
  maintenanceType: string;
  confidence: number;
  estimatedCostAED: number;
}

interface DemandForecast {
  nextWeekDemand: number;
  peakHours: string[];
  seasonalTrends: SeasonalTrend[];
}

interface SeasonalTrend {
  period: string;
  demandMultiplier: number;
  factors: string[];
}

interface CostProjection {
  month: string;
  projectedCostAED: number;
  seasonalFactor: number;
  confidence: number;
}

interface RiskAssessment {
  overallRiskScore: number;
  riskFactors: RiskFactor[];
  recommendations: string[];
}

interface RiskFactor {
  factor: string;
  impact: string;
  probability: number;
  mitigation: string;
}

interface AbuDhabiMetrics {
  withinEmiratePercentage: number;
  extremeHeatExposure: number;
  dustStormIncidents: number;
  prayerTimeCompliance: number;
  culturalAdaptationScore: number;
  regulatoryCompliance: number;
}

const AnalyticsDashboard: React.FC = () => {
  const [selectedTimeRange, setSelectedTimeRange] = useState('24h');
  const [selectedMetric, setSelectedMetric] = useState('overview');
  const [refreshInterval, setRefreshInterval] = useState(30000); // 30 seconds

  // Fetch analytics data using API service
  const { data: analyticsData, isLoading, error, refetch } = useQuery<AnalyticsData>({
    queryKey: ['analytics', selectedTimeRange],
    queryFn: async () => {
      try {
        // Fetch multiple analytics endpoints in parallel
        const [fleetKPIs, operationalMetrics, financialMetrics, safetyMetrics] = await Promise.all([
          apiService.getFleetKPIs(selectedTimeRange),
          apiService.getAnalyticsOperationalMetrics(),
          apiService.getFinancialMetrics(),
          apiService.getSafetyMetrics(),
        ]);

        // Transform API responses into dashboard data format
        const transformedData: AnalyticsData = {
          fleetOverview: {
            totalVehicles: fleetKPIs.data?.fleet_availability || 0,
            activeVehicles: fleetKPIs.data?.utilization_rate || 0,
            maintenanceVehicles: 0, // Calculate from operational metrics
            emergencyVehicles: safetyMetrics.data?.incident_rate || 0,
            averageHealthScore: fleetKPIs.data?.fleet_availability || 0,
            totalDistance: 0, // Calculate from trip metrics
            totalTrips: 0, // Calculate from trip metrics
            onTimePerformance: fleetKPIs.data?.on_time_performance || 0,
          },
          performanceMetrics: {
            utilizationRate: fleetKPIs.data?.utilization_rate || 0,
            averageTripTime: fleetKPIs.data?.average_trip_time || 0,
            fuelEfficiency: fleetKPIs.data?.fuel_efficiency || 0,
            customerSatisfaction: fleetKPIs.data?.customer_satisfaction || 0,
            serviceLevelAgreement: operationalMetrics.data?.service_level_agreement || 0,
            resourceUtilization: operationalMetrics.data?.resource_utilization || 0,
            capacityUtilization: operationalMetrics.data?.capacity_utilization || 0,
            uptime: operationalMetrics.data?.uptime || 0,
          },
          operationalMetrics: {
            incidentRate: safetyMetrics.data?.incident_rate || 0,
            responseTime: operationalMetrics.data?.response_time || 0,
            complianceScore: safetyMetrics.data?.compliance_score || 0,
            safetyTraining: safetyMetrics.data?.safety_training || 0,
            emergencyResponse: safetyMetrics.data?.emergency_response || 0,
            riskScore: safetyMetrics.data?.risk_score || 0,
            auditScore: safetyMetrics.data?.audit_score || 0,
            maintenanceEfficiency: 0, // Calculate from maintenance data
          },
          costAnalysis: {
            operatingCosts: financialMetrics.data?.operating_costs || 0,
            revenuePerVehicle: financialMetrics.data?.revenue_per_vehicle || 0,
            roi: financialMetrics.data?.roi || 0,
            budgetVariance: financialMetrics.data?.budget_variance || 0,
            costEfficiency: financialMetrics.data?.cost_efficiency || 0,
            profitMargin: financialMetrics.data?.profit_margin || 0,
            totalRevenue: fleetKPIs.data?.total_revenue || 0,
            costPerKilometer: fleetKPIs.data?.cost_per_kilometer || 0,
          },
          trends: {
            utilizationTrend: [], // Generate from historical data
            performanceTrend: [], // Generate from historical data
            costTrend: [], // Generate from historical data
            safetyTrend: [], // Generate from historical data
          },
          alerts: [], // Generate from real-time data
          recommendations: [], // Generate from analytics
        };

        return transformedData;
      } catch (error) {
        console.error('Failed to fetch analytics data:', error);
        throw error;
      }
    },
    refetchInterval: refreshInterval,
    staleTime: 10000, // 10 seconds
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
          padding: 20,
          font: {
            size: 12,
          },
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
        grid: {
          color: 'rgba(0, 0, 0, 0.1)',
        },
        ticks: {
          font: {
            size: 11,
          },
        },
      },
      y: {
        grid: {
          color: 'rgba(0, 0, 0, 0.1)',
        },
        ticks: {
          font: {
            size: 11,
          },
        },
      },
    },
  }), []);

  // Generate chart data
  const generateFleetUtilizationChart = () => {
    if (!analyticsData) return null;

    return {
      labels: ['Active', 'Dispatched', 'Maintenance', 'Offline'],
      datasets: [
        {
          data: [
            analyticsData.fleetOverview.activeVehicles,
            analyticsData.fleetOverview.dispatchedVehicles,
            analyticsData.fleetOverview.maintenanceVehicles,
            analyticsData.fleetOverview.offlineVehicles,
          ],
          backgroundColor: [
            '#10B981', // Green for active
            '#3B82F6', // Blue for dispatched
            '#F59E0B', // Amber for maintenance
            '#EF4444', // Red for offline
          ],
          borderWidth: 0,
        },
      ],
    };
  };

  const generatePerformanceTrendChart = () => {
    if (!analyticsData) return null;

    // Mock time series data - in production, this would come from the API
    const hours = Array.from({ length: 24 }, (_, i) => `${i}:00`);
    const utilizationData = hours.map(() => Math.random() * 40 + 60); // 60-100% utilization
    const healthData = hours.map(() => Math.random() * 20 + 80); // 80-100% health

    return {
      labels: hours,
      datasets: [
        {
          label: 'Fleet Utilization %',
          data: utilizationData,
          borderColor: '#3B82F6',
          backgroundColor: 'rgba(59, 130, 246, 0.1)',
          fill: true,
          tension: 0.4,
        },
        {
          label: 'Avg Health Score %',
          data: healthData,
          borderColor: '#10B981',
          backgroundColor: 'rgba(16, 185, 129, 0.1)',
          fill: true,
          tension: 0.4,
        },
      ],
    };
  };

  const generateCostBreakdownChart = () => {
    if (!analyticsData) return null;

    return {
      labels: ['Fuel', 'Maintenance', 'Operations', 'Insurance', 'Depreciation'],
      datasets: [
        {
          label: 'Cost (AED)',
          data: [
            analyticsData.costAnalysis.fuelCostsAED,
            analyticsData.costAnalysis.maintenanceCostsAED,
            analyticsData.costAnalysis.operationalCostsAED,
            analyticsData.costAnalysis.operationalCostsAED * 0.3, // Insurance estimate
            analyticsData.costAnalysis.operationalCostsAED * 0.4, // Depreciation estimate
          ],
          backgroundColor: [
            '#EF4444', // Red for fuel
            '#F59E0B', // Amber for maintenance
            '#3B82F6', // Blue for operations
            '#8B5CF6', // Purple for insurance
            '#6B7280', // Gray for depreciation
          ],
          borderWidth: 1,
          borderColor: '#ffffff',
        },
      ],
    };
  };

  if (isLoading) {
    return (
      <div className="flex items-center justify-center h-64">
        <div className="animate-spin rounded-full h-8 w-8 border-b-2 border-blue-600"></div>
        <span className="ml-2 text-gray-600">Loading analytics...</span>
      </div>
    );
  }

  if (error) {
    return (
      <div className="bg-red-50 border border-red-200 rounded-md p-4">
        <div className="flex">
          <ExclamationTriangleIcon className="h-5 w-5 text-red-400" />
          <div className="ml-3">
            <h3 className="text-sm font-medium text-red-800">Error loading analytics</h3>
            <p className="mt-1 text-sm text-red-700">
              Failed to load analytics data. Please try refreshing the page.
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
          <h1 className="text-2xl font-bold text-gray-900">Fleet Analytics Dashboard</h1>
          <p className="mt-1 text-sm text-gray-500">
            Comprehensive insights into fleet performance and operations
          </p>
        </div>
        
        <div className="mt-4 sm:mt-0 flex space-x-3">
          <select
            value={selectedTimeRange}
            onChange={(e) => setSelectedTimeRange(e.target.value)}
            className="rounded-md border-gray-300 shadow-sm focus:border-blue-500 focus:ring-blue-500 sm:text-sm"
          >
            <option value="1h">Last Hour</option>
            <option value="24h">Last 24 Hours</option>
            <option value="7d">Last 7 Days</option>
            <option value="30d">Last 30 Days</option>
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

      {/* Key Performance Indicators */}
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
                <TruckIcon className="h-6 w-6 text-blue-600" />
              </div>
              <div className="ml-5 w-0 flex-1">
                <dl>
                  <dt className="text-sm font-medium text-gray-500 truncate">Fleet Utilization</dt>
                  <dd className="text-lg font-medium text-gray-900">
                    {analyticsData?.fleetOverview.utilizationRate.toFixed(1)}%
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
              <span className="font-medium text-green-600">+2.5%</span>
              <span className="text-gray-500 ml-1">from last period</span>
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
                    {analyticsData?.fleetOverview.avgHealthScore.toFixed(1)}%
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
              <span className="font-medium text-green-600">+1.2%</span>
              <span className="text-gray-500 ml-1">from last period</span>
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
                  <dt className="text-sm font-medium text-gray-500 truncate">Cost per KM</dt>
                  <dd className="text-lg font-medium text-gray-900">
                    {analyticsData?.costAnalysis.costPerKmAED.toFixed(2)} AED
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
              <span className="font-medium text-green-600">-3.1%</span>
              <span className="text-gray-500 ml-1">cost reduction</span>
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
                  <dt className="text-sm font-medium text-gray-500 truncate">Critical Alerts</dt>
                  <dd className="text-lg font-medium text-gray-900">
                    {analyticsData?.healthMetrics.criticalAlerts || 0}
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
              <span className="font-medium text-green-600">-15%</span>
              <span className="text-gray-500 ml-1">from last period</span>
            </div>
          </div>
        </motion.div>
      </div>

      {/* Main Charts Grid */}
      <div className="grid grid-cols-1 lg:grid-cols-2 gap-6">
        {/* Fleet Utilization Chart */}
        <motion.div
          initial={{ opacity: 0, scale: 0.95 }}
          animate={{ opacity: 1, scale: 1 }}
          transition={{ duration: 0.5, delay: 0.4 }}
          className="bg-white shadow rounded-lg p-6"
        >
          <div className="flex items-center justify-between mb-4">
            <h3 className="text-lg font-medium text-gray-900">Fleet Status Distribution</h3>
            <InformationCircleIcon className="h-5 w-5 text-gray-400" />
          </div>
          <div className="h-64">
            <Doughnut data={generateFleetUtilizationChart()} options={chartOptions} />
          </div>
        </motion.div>

        {/* Performance Trend Chart */}
        <motion.div
          initial={{ opacity: 0, scale: 0.95 }}
          animate={{ opacity: 1, scale: 1 }}
          transition={{ duration: 0.5, delay: 0.5 }}
          className="bg-white shadow rounded-lg p-6"
        >
          <div className="flex items-center justify-between mb-4">
            <h3 className="text-lg font-medium text-gray-900">Performance Trends (24h)</h3>
            <InformationCircleIcon className="h-5 w-5 text-gray-400" />
          </div>
          <div className="h-64">
            <Line data={generatePerformanceTrendChart()} options={chartOptions} />
          </div>
        </motion.div>

        {/* Cost Breakdown Chart */}
        <motion.div
          initial={{ opacity: 0, scale: 0.95 }}
          animate={{ opacity: 1, scale: 1 }}
          transition={{ duration: 0.5, delay: 0.6 }}
          className="bg-white shadow rounded-lg p-6"
        >
          <div className="flex items-center justify-between mb-4">
            <h3 className="text-lg font-medium text-gray-900">Cost Breakdown</h3>
            <InformationCircleIcon className="h-5 w-5 text-gray-400" />
          </div>
          <div className="h-64">
            <Bar data={generateCostBreakdownChart()} options={chartOptions} />
          </div>
        </motion.div>

        {/* Abu Dhabi Specific Metrics */}
        <motion.div
          initial={{ opacity: 0, scale: 0.95 }}
          animate={{ opacity: 1, scale: 1 }}
          transition={{ duration: 0.5, delay: 0.7 }}
          className="bg-white shadow rounded-lg p-6"
        >
          <div className="flex items-center justify-between mb-4">
            <h3 className="text-lg font-medium text-gray-900">Abu Dhabi Adaptations</h3>
            <div className="flex items-center space-x-1">
              <span className="text-xs text-gray-500">🇦🇪 UAE Specific</span>
            </div>
          </div>
          <div className="space-y-4">
            <div className="flex justify-between items-center">
              <span className="text-sm text-gray-600">Within Emirate Operations</span>
              <span className="text-sm font-medium text-gray-900">
                {analyticsData?.abuDhabiMetrics?.withinEmiratePercentage || 95}%
              </span>
            </div>
            <div className="w-full bg-gray-200 rounded-full h-2">
              <div 
                className="bg-green-600 h-2 rounded-full" 
                style={{ width: `${analyticsData?.abuDhabiMetrics?.withinEmiratePercentage || 95}%` }}
              ></div>
            </div>
            
            <div className="flex justify-between items-center">
              <span className="text-sm text-gray-600">Prayer Time Compliance</span>
              <span className="text-sm font-medium text-gray-900">
                {analyticsData?.abuDhabiMetrics?.prayerTimeCompliance || 92}%
              </span>
            </div>
            <div className="w-full bg-gray-200 rounded-full h-2">
              <div 
                className="bg-blue-600 h-2 rounded-full" 
                style={{ width: `${analyticsData?.abuDhabiMetrics?.prayerTimeCompliance || 92}%` }}
              ></div>
            </div>
            
            <div className="flex justify-between items-center">
              <span className="text-sm text-gray-600">Regulatory Compliance</span>
              <span className="text-sm font-medium text-gray-900">
                {analyticsData?.abuDhabiMetrics?.regulatoryCompliance || 98}%
              </span>
            </div>
            <div className="w-full bg-gray-200 rounded-full h-2">
              <div 
                className="bg-purple-600 h-2 rounded-full" 
                style={{ width: `${analyticsData?.abuDhabiMetrics?.regulatoryCompliance || 98}%` }}
              ></div>
            </div>
          </div>
        </motion.div>
      </div>

      {/* Predictive Insights Section */}
      <motion.div
        initial={{ opacity: 0, y: 20 }}
        animate={{ opacity: 1, y: 0 }}
        transition={{ duration: 0.5, delay: 0.8 }}
        className="bg-white shadow rounded-lg p-6"
      >
        <div className="flex items-center justify-between mb-6">
          <h3 className="text-lg font-medium text-gray-900">Predictive Insights</h3>
          <div className="flex items-center space-x-2">
            <Cog6ToothIcon className="h-5 w-5 text-gray-400" />
            <span className="text-xs text-gray-500">AI-Powered</span>
          </div>
        </div>
        
        <div className="grid grid-cols-1 md:grid-cols-3 gap-6">
          {/* Maintenance Predictions */}
          <div>
            <h4 className="text-sm font-medium text-gray-900 mb-3">Upcoming Maintenance</h4>
            <div className="space-y-3">
              {analyticsData?.predictiveInsights?.maintenancePredictions?.slice(0, 3).map((prediction, index) => (
                <div key={index} className="flex items-center space-x-3 p-3 bg-yellow-50 rounded-md">
                  <Cog6ToothIcon className="h-5 w-5 text-yellow-600" />
                  <div className="flex-1 min-w-0">
                    <p className="text-sm font-medium text-gray-900 truncate">
                      {prediction.assetTag}
                    </p>
                    <p className="text-xs text-gray-500">
                      {prediction.maintenanceType} - {prediction.predictedDate}
                    </p>
                  </div>
                  <div className="text-xs text-gray-900 font-medium">
                    {prediction.estimatedCostAED} AED
                  </div>
                </div>
              ))}
            </div>
          </div>

          {/* Demand Forecast */}
          <div>
            <h4 className="text-sm font-medium text-gray-900 mb-3">Demand Forecast</h4>
            <div className="space-y-3">
              <div className="p-3 bg-blue-50 rounded-md">
                <div className="flex items-center justify-between">
                  <span className="text-sm text-gray-600">Next Week Demand</span>
                  <span className="text-sm font-medium text-gray-900">
                    +{analyticsData?.predictiveInsights?.demandForecast?.nextWeekDemand || 15}%
                  </span>
                </div>
              </div>
              <div className="p-3 bg-green-50 rounded-md">
                <div className="text-sm text-gray-600 mb-1">Peak Hours</div>
                <div className="text-xs text-gray-500">
                  {analyticsData?.predictiveInsights?.demandForecast?.peakHours?.join(', ') || '7-9 AM, 5-7 PM'}
                </div>
              </div>
            </div>
          </div>

          {/* Risk Assessment */}
          <div>
            <h4 className="text-sm font-medium text-gray-900 mb-3">Risk Assessment</h4>
            <div className="space-y-3">
              <div className="p-3 bg-red-50 rounded-md">
                <div className="flex items-center justify-between mb-2">
                  <span className="text-sm text-gray-600">Overall Risk</span>
                  <span className="text-sm font-medium text-red-900">
                    {analyticsData?.predictiveInsights?.riskAssessment?.overallRiskScore || 'Low'}
                  </span>
                </div>
                <div className="text-xs text-gray-500">
                  {analyticsData?.predictiveInsights?.riskAssessment?.riskFactors?.length || 2} factors identified
                </div>
              </div>
              <div className="space-y-1">
                {analyticsData?.predictiveInsights?.riskAssessment?.recommendations?.slice(0, 2).map((rec, index) => (
                  <div key={index} className="text-xs text-gray-600 p-2 bg-gray-50 rounded">
                    {rec}
                  </div>
                ))}
              </div>
            </div>
          </div>
        </div>
      </motion.div>

      {/* Operational Summary */}
      <motion.div
        initial={{ opacity: 0, y: 20 }}
        animate={{ opacity: 1, y: 0 }}
        transition={{ duration: 0.5, delay: 0.9 }}
        className="bg-white shadow rounded-lg p-6"
      >
        <h3 className="text-lg font-medium text-gray-900 mb-4">Operational Summary</h3>
        <div className="grid grid-cols-2 md:grid-cols-4 gap-4">
          <div className="text-center">
            <div className="text-2xl font-bold text-blue-600">
              {analyticsData?.operationalMetrics.tripsCompleted || 0}
            </div>
            <div className="text-sm text-gray-500">Trips Completed</div>
          </div>
          <div className="text-center">
            <div className="text-2xl font-bold text-green-600">
              {(analyticsData?.performanceMetrics.totalDistanceKm || 0).toLocaleString()} km
            </div>
            <div className="text-sm text-gray-500">Total Distance</div>
          </div>
          <div className="text-center">
            <div className="text-2xl font-bold text-yellow-600">
              {analyticsData?.operationalMetrics.avgTripDuration || 0} min
            </div>
            <div className="text-sm text-gray-500">Avg Trip Duration</div>
          </div>
          <div className="text-center">
            <div className="text-2xl font-bold text-purple-600">
              {(analyticsData?.performanceMetrics.tripCompletionRate || 0).toFixed(1)}%
            </div>
            <div className="text-sm text-gray-500">Completion Rate</div>
          </div>
        </div>
      </motion.div>
    </div>
  );
};

export default AnalyticsDashboard;
