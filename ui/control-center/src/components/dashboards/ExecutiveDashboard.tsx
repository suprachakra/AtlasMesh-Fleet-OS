import React, { useState, useEffect, useMemo } from 'react';
import { motion, AnimatePresence } from 'framer-motion';
import {
  ChartBarIcon,
  CurrencyDollarIcon,
  TruckIcon,
  UsersIcon,
  ExclamationTriangleIcon,
  CheckCircleIcon,
  ClockIcon,
  ArrowUpIcon,
  ArrowDownIcon,
  GlobeAltIcon,
  ShieldCheckIcon,
  BoltIcon,
  CogIcon,
  ArrowTrendingUpIcon,
  ArrowTrendingDownIcon,
  CalendarDaysIcon,
  MapPinIcon,
  SignalIcon,
  EyeIcon,
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

interface ExecutiveDashboardData {
  kpis: ExecutiveKPIs;
  financials: FinancialMetrics;
  operations: OperationalMetrics;
  safety: SafetyMetrics;
  compliance: ComplianceOverview;
  fleet: FleetOverview;
  performance: PerformanceMetrics;
  market: MarketInsights;
  risks: RiskAssessment;
  sustainability: SustainabilityMetrics;
  growth: GrowthMetrics;
  regional: RegionalPerformance;
}

interface ExecutiveKPIs {
  totalRevenue: number;
  revenueGrowth: number;
  totalTrips: number;
  tripGrowth: number;
  activeVehicles: number;
  fleetUtilization: number;
  customerSatisfaction: number;
  operationalEfficiency: number;
  safetyScore: number;
  complianceScore: number;
  profitMargin: number;
  marketShare: number;
}

interface FinancialMetrics {
  revenue: RevenueBreakdown;
  costs: CostBreakdown;
  profitability: ProfitabilityMetrics;
  cashFlow: CashFlowMetrics;
  investments: InvestmentMetrics;
  roi: ROIMetrics;
  forecast: FinancialForecast;
}

interface RevenueBreakdown {
  total: number;
  byService: ServiceRevenue[];
  byRegion: RegionRevenue[];
  byCustomerType: CustomerTypeRevenue[];
  growth: GrowthTrend[];
}

interface ServiceRevenue {
  service: string;
  revenue: number;
  percentage: number;
  growth: number;
}

interface RegionRevenue {
  region: string;
  revenue: number;
  percentage: number;
  growth: number;
}

interface CustomerTypeRevenue {
  type: string;
  revenue: number;
  percentage: number;
  growth: number;
}

interface GrowthTrend {
  period: string;
  value: number;
  growth: number;
}

interface CostBreakdown {
  total: number;
  operational: number;
  maintenance: number;
  fuel: number;
  personnel: number;
  technology: number;
  compliance: number;
  other: number;
  trends: CostTrend[];
}

interface CostTrend {
  category: string;
  current: number;
  previous: number;
  change: number;
}

interface ProfitabilityMetrics {
  grossMargin: number;
  operatingMargin: number;
  netMargin: number;
  ebitda: number;
  trends: ProfitabilityTrend[];
}

interface ProfitabilityTrend {
  period: string;
  grossMargin: number;
  operatingMargin: number;
  netMargin: number;
}

interface CashFlowMetrics {
  operating: number;
  investing: number;
  financing: number;
  free: number;
  burnRate: number;
  runway: number;
}

interface InvestmentMetrics {
  totalInvested: number;
  rdSpend: number;
  capex: number;
  technology: number;
  fleet: number;
  infrastructure: number;
}

interface ROIMetrics {
  overall: number;
  byInvestment: InvestmentROI[];
  paybackPeriod: number;
}

interface InvestmentROI {
  investment: string;
  roi: number;
  paybackMonths: number;
}

interface FinancialForecast {
  revenue: ForecastData[];
  costs: ForecastData[];
  profit: ForecastData[];
  confidence: number;
}

interface ForecastData {
  period: string;
  value: number;
  confidence: number;
}

interface OperationalMetrics {
  totalTrips: number;
  completedTrips: number;
  cancelledTrips: number;
  averageTripTime: number;
  averageWaitTime: number;
  onTimePerformance: number;
  routeEfficiency: number;
  fuelEfficiency: number;
  energyConsumption: number;
  maintenanceEvents: number;
  uptime: number;
  customerRating: number;
  trends: OperationalTrend[];
}

interface OperationalTrend {
  metric: string;
  current: number;
  target: number;
  trend: 'up' | 'down' | 'stable';
  change: number;
}

interface SafetyMetrics {
  accidentsPerMile: number;
  safetyScore: number;
  incidentReports: number;
  nearMisses: number;
  safetyTraining: number;
  complianceViolations: number;
  emergencyResponses: number;
  safetyInvestments: number;
  trends: SafetyTrend[];
}

interface SafetyTrend {
  period: string;
  accidents: number;
  incidents: number;
  score: number;
}

interface ComplianceOverview {
  overallScore: number;
  regulations: RegulationStatus[];
  audits: AuditStatus[];
  certifications: CertificationStatus[];
  violations: ViolationSummary[];
  costs: ComplianceCosts;
}

interface RegulationStatus {
  regulation: string;
  status: 'compliant' | 'non-compliant' | 'pending';
  score: number;
  lastReview: string;
}

interface AuditStatus {
  audit: string;
  status: 'passed' | 'failed' | 'pending';
  score: number;
  date: string;
}

interface CertificationStatus {
  certification: string;
  status: 'valid' | 'expired' | 'pending';
  expiryDate: string;
}

interface ViolationSummary {
  type: string;
  count: number;
  severity: 'low' | 'medium' | 'high' | 'critical';
  cost: number;
}

interface ComplianceCosts {
  total: number;
  audits: number;
  certifications: number;
  violations: number;
  training: number;
}

interface FleetOverview {
  totalVehicles: number;
  activeVehicles: number;
  inMaintenance: number;
  utilization: number;
  averageAge: number;
  fuelEfficiency: number;
  maintenanceCosts: number;
  vehicleTypes: VehicleTypeBreakdown[];
  healthScores: VehicleHealthScore[];
}

interface VehicleTypeBreakdown {
  type: string;
  count: number;
  utilization: number;
  efficiency: number;
}

interface VehicleHealthScore {
  vehicleId: string;
  score: number;
  status: 'excellent' | 'good' | 'fair' | 'poor';
  lastMaintenance: string;
}

interface PerformanceMetrics {
  systemUptime: number;
  responseTime: number;
  throughput: number;
  errorRate: number;
  availability: number;
  scalability: number;
  reliability: number;
  efficiency: number;
  trends: PerformanceTrend[];
}

interface PerformanceTrend {
  metric: string;
  values: TimeSeriesData[];
  target: number;
  status: 'above' | 'below' | 'on-target';
}

interface TimeSeriesData {
  timestamp: string;
  value: number;
}

interface MarketInsights {
  marketSize: number;
  marketGrowth: number;
  marketShare: number;
  competitors: CompetitorAnalysis[];
  opportunities: MarketOpportunity[];
  threats: MarketThreat[];
  customerSegments: CustomerSegment[];
}

interface CompetitorAnalysis {
  competitor: string;
  marketShare: number;
  strengths: string[];
  weaknesses: string[];
  revenue: number;
}

interface MarketOpportunity {
  opportunity: string;
  potential: number;
  timeline: string;
  investment: number;
  probability: number;
}

interface MarketThreat {
  threat: string;
  impact: number;
  probability: number;
  mitigation: string;
}

interface CustomerSegment {
  segment: string;
  size: number;
  growth: number;
  revenue: number;
  satisfaction: number;
}

interface RiskAssessment {
  overallRisk: number;
  risks: Risk[];
  mitigations: RiskMitigation[];
  exposures: RiskExposure[];
  insurance: InsuranceCoverage[];
}

interface Risk {
  risk: string;
  category: string;
  probability: number;
  impact: number;
  score: number;
  status: 'open' | 'mitigated' | 'accepted';
}

interface RiskMitigation {
  risk: string;
  mitigation: string;
  effectiveness: number;
  cost: number;
  timeline: string;
}

interface RiskExposure {
  category: string;
  exposure: number;
  limit: number;
  coverage: number;
}

interface InsuranceCoverage {
  type: string;
  coverage: number;
  premium: number;
  deductible: number;
  expires: string;
}

interface SustainabilityMetrics {
  carbonFootprint: number;
  energyEfficiency: number;
  wasteReduction: number;
  sustainabilityScore: number;
  greenInitiatives: GreenInitiative[];
  certifications: SustainabilityCertification[];
  goals: SustainabilityGoal[];
}

interface GreenInitiative {
  initiative: string;
  impact: number;
  investment: number;
  timeline: string;
  status: 'planned' | 'in-progress' | 'completed';
}

interface SustainabilityCertification {
  certification: string;
  level: string;
  issueDate: string;
  expiryDate: string;
}

interface SustainabilityGoal {
  goal: string;
  target: number;
  current: number;
  deadline: string;
  progress: number;
}

interface GrowthMetrics {
  revenueGrowth: number;
  customerGrowth: number;
  marketExpansion: number;
  newServices: number;
  partnerships: number;
  investments: number;
  projections: GrowthProjection[];
}

interface GrowthProjection {
  metric: string;
  current: number;
  projected: number;
  timeline: string;
  confidence: number;
}

interface RegionalPerformance {
  regions: RegionPerformance[];
  expansion: ExpansionPlan[];
  localization: LocalizationStatus[];
}

interface RegionPerformance {
  region: string;
  revenue: number;
  growth: number;
  marketShare: number;
  vehicles: number;
  customers: number;
  satisfaction: number;
}

interface ExpansionPlan {
  region: string;
  timeline: string;
  investment: number;
  expectedRevenue: number;
  riskLevel: 'low' | 'medium' | 'high';
}

interface LocalizationStatus {
  region: string;
  progress: number;
  challenges: string[];
  opportunities: string[];
}

const ExecutiveDashboard: React.FC = () => {
  const [selectedTimeRange, setSelectedTimeRange] = useState('30d');
  const [selectedView, setSelectedView] = useState('overview');
  const [refreshInterval, setRefreshInterval] = useState(300000); // 5 minutes

  // Fetch executive dashboard data
  const { data: dashboardData, isLoading, error, refetch } = useQuery<ExecutiveDashboardData>({
    queryKey: ['executive-dashboard', selectedTimeRange, selectedView],
    queryFn: async () => {
      const params = new URLSearchParams({
        range: selectedTimeRange,
        view: selectedView,
      });
      
      const response = await fetch(`/api/v1/executive/dashboard?${params}`);
      if (!response.ok) {
        throw new Error('Failed to fetch executive dashboard data');
      }
      return response.json();
    },
    refetchInterval: refreshInterval,
    staleTime: 60000, // 1 minute
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

  // Generate revenue trend chart
  const generateRevenueTrendChart = () => {
    if (!dashboardData?.financials.revenue.growth) return null;

    const labels = dashboardData.financials.revenue.growth.map(g => g.period);
    const revenueData = dashboardData.financials.revenue.growth.map(g => g.value);
    const growthData = dashboardData.financials.revenue.growth.map(g => g.growth);

    return {
      labels,
      datasets: [
        {
          label: 'Revenue (AED)',
          data: revenueData,
          borderColor: '#10B981',
          backgroundColor: 'rgba(16, 185, 129, 0.1)',
          fill: true,
          tension: 0.4,
          yAxisID: 'y',
        },
        {
          label: 'Growth Rate (%)',
          data: growthData,
          borderColor: '#3B82F6',
          backgroundColor: 'rgba(59, 130, 246, 0.1)',
          fill: false,
          tension: 0.4,
          yAxisID: 'y1',
        },
      ],
    };
  };

  // Generate operational metrics chart
  const generateOperationalMetricsChart = () => {
    if (!dashboardData?.operations.trends) return null;

    const labels = dashboardData.operations.trends.map(t => t.metric);
    const currentData = dashboardData.operations.trends.map(t => t.current);
    const targetData = dashboardData.operations.trends.map(t => t.target);

    return {
      labels,
      datasets: [
        {
          label: 'Current Performance',
          data: currentData,
          backgroundColor: 'rgba(59, 130, 246, 0.8)',
          borderColor: '#3B82F6',
          borderWidth: 1,
        },
        {
          label: 'Target',
          data: targetData,
          backgroundColor: 'rgba(16, 185, 129, 0.8)',
          borderColor: '#10B981',
          borderWidth: 1,
        },
      ],
    };
  };

  // Generate fleet utilization chart
  const generateFleetUtilizationChart = () => {
    if (!dashboardData?.fleet.vehicleTypes) return null;

    return {
      labels: dashboardData.fleet.vehicleTypes.map(v => v.type),
      datasets: [
        {
          data: dashboardData.fleet.vehicleTypes.map(v => v.utilization),
          backgroundColor: [
            '#3B82F6',
            '#10B981',
            '#F59E0B',
            '#EF4444',
            '#8B5CF6',
          ],
          borderWidth: 0,
        },
      ],
    };
  };

  // Generate regional performance chart
  const generateRegionalPerformanceChart = () => {
    if (!dashboardData?.regional.regions) return null;

    const labels = dashboardData.regional.regions.map(r => r.region);
    const revenueData = dashboardData.regional.regions.map(r => r.revenue);
    const growthData = dashboardData.regional.regions.map(r => r.growth);

    return {
      labels,
      datasets: [
        {
          label: 'Revenue (AED)',
          data: revenueData,
          backgroundColor: 'rgba(59, 130, 246, 0.8)',
          borderColor: '#3B82F6',
          borderWidth: 1,
          yAxisID: 'y',
        },
        {
          label: 'Growth Rate (%)',
          data: growthData,
          backgroundColor: 'rgba(16, 185, 129, 0.8)',
          borderColor: '#10B981',
          borderWidth: 1,
          yAxisID: 'y1',
        },
      ],
    };
  };

  const formatCurrency = (value: number) => {
    return new Intl.NumberFormat('en-AE', {
      style: 'currency',
      currency: 'AED',
      minimumFractionDigits: 0,
      maximumFractionDigits: 0,
    }).format(value);
  };

  const formatNumber = (value: number) => {
    return new Intl.NumberFormat('en-AE').format(value);
  };

  const formatPercentage = (value: number) => {
    return `${value.toFixed(1)}%`;
  };

  const getTrendIcon = (trend: 'up' | 'down' | 'stable', value: number) => {
    if (trend === 'up') {
      return <ArrowTrendingUpIcon className={`h-4 w-4 ${value > 0 ? 'text-green-500' : 'text-red-500'}`} />;
    } else if (trend === 'down') {
      return <ArrowTrendingDownIcon className={`h-4 w-4 ${value > 0 ? 'text-red-500' : 'text-green-500'}`} />;
    } else {
      return <div className="h-4 w-4 bg-gray-400 rounded-full"></div>;
    }
  };

  if (isLoading) {
    return (
      <div className="flex items-center justify-center h-64">
        <div className="animate-spin rounded-full h-8 w-8 border-b-2 border-blue-600"></div>
        <span className="ml-2 text-gray-600">Loading executive dashboard...</span>
      </div>
    );
  }

  if (error) {
    return (
      <div className="bg-red-50 border border-red-200 rounded-md p-4">
        <div className="flex">
          <ExclamationTriangleIcon className="h-5 w-5 text-red-400" />
          <div className="ml-3">
            <h3 className="text-sm font-medium text-red-800">Error loading dashboard</h3>
            <p className="mt-1 text-sm text-red-700">
              Failed to load executive dashboard. Please try refreshing the page.
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
          <h1 className="text-2xl font-bold text-gray-900">Executive Dashboard</h1>
          <p className="mt-1 text-sm text-gray-500">
            Strategic overview of AtlasMesh Fleet OS performance and key business metrics
          </p>
        </div>
        
        <div className="mt-4 sm:mt-0 flex flex-wrap gap-3">
          <select
            value={selectedTimeRange}
            onChange={(e) => setSelectedTimeRange(e.target.value)}
            className="rounded-md border-gray-300 shadow-sm focus:border-blue-500 focus:ring-blue-500 sm:text-sm"
          >
            <option value="7d">Last 7 Days</option>
            <option value="30d">Last 30 Days</option>
            <option value="90d">Last Quarter</option>
            <option value="1y">Last Year</option>
          </select>
          
          <select
            value={selectedView}
            onChange={(e) => setSelectedView(e.target.value)}
            className="rounded-md border-gray-300 shadow-sm focus:border-blue-500 focus:ring-blue-500 sm:text-sm"
          >
            <option value="overview">Overview</option>
            <option value="financial">Financial</option>
            <option value="operational">Operational</option>
            <option value="strategic">Strategic</option>
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
                <CurrencyDollarIcon className="h-6 w-6 text-green-600" />
              </div>
              <div className="ml-5 w-0 flex-1">
                <dl>
                  <dt className="text-sm font-medium text-gray-500 truncate">Total Revenue</dt>
                  <dd className="text-lg font-medium text-gray-900">
                    {formatCurrency(dashboardData?.kpis.totalRevenue || 0)}
                  </dd>
                </dl>
              </div>
              <div className="flex-shrink-0">
                <div className="flex items-center text-sm">
                  {dashboardData?.kpis.revenueGrowth && dashboardData.kpis.revenueGrowth > 0 ? (
                    <ArrowUpIcon className="h-4 w-4 text-green-500" />
                  ) : (
                    <ArrowDownIcon className="h-4 w-4 text-red-500" />
                  )}
                  <span className={dashboardData?.kpis.revenueGrowth && dashboardData.kpis.revenueGrowth > 0 ? 'text-green-600' : 'text-red-600'}>
                    {formatPercentage(dashboardData?.kpis.revenueGrowth || 0)}
                  </span>
                </div>
              </div>
            </div>
          </div>
          <div className="bg-gray-50 px-5 py-3">
            <div className="text-sm">
              <span className="font-medium text-green-600">
                {formatNumber(dashboardData?.kpis.totalTrips || 0)}
              </span>
              <span className="text-gray-500 ml-1">total trips completed</span>
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
                <TruckIcon className="h-6 w-6 text-blue-600" />
              </div>
              <div className="ml-5 w-0 flex-1">
                <dl>
                  <dt className="text-sm font-medium text-gray-500 truncate">Fleet Utilization</dt>
                  <dd className="text-lg font-medium text-gray-900">
                    {formatPercentage(dashboardData?.kpis.fleetUtilization || 0)}
                  </dd>
                </dl>
              </div>
              <div className="flex-shrink-0">
                <CheckCircleIcon className="h-5 w-5 text-green-500" />
              </div>
            </div>
          </div>
          <div className="bg-gray-50 px-5 py-3">
            <div className="text-sm">
              <span className="font-medium text-blue-600">
                {dashboardData?.kpis.activeVehicles || 0}
              </span>
              <span className="text-gray-500 ml-1">active vehicles</span>
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
                <ShieldCheckIcon className="h-6 w-6 text-purple-600" />
              </div>
              <div className="ml-5 w-0 flex-1">
                <dl>
                  <dt className="text-sm font-medium text-gray-500 truncate">Safety Score</dt>
                  <dd className="text-lg font-medium text-gray-900">
                    {formatPercentage(dashboardData?.kpis.safetyScore || 0)}
                  </dd>
                </dl>
              </div>
              <div className="flex-shrink-0">
                <ShieldCheckIcon className="h-5 w-5 text-purple-500" />
              </div>
            </div>
          </div>
          <div className="bg-gray-50 px-5 py-3">
            <div className="text-sm">
              <span className="font-medium text-purple-600">
                {formatPercentage(dashboardData?.kpis.complianceScore || 0)}
              </span>
              <span className="text-gray-500 ml-1">compliance score</span>
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
                <UsersIcon className="h-6 w-6 text-orange-600" />
              </div>
              <div className="ml-5 w-0 flex-1">
                <dl>
                  <dt className="text-sm font-medium text-gray-500 truncate">Customer Satisfaction</dt>
                  <dd className="text-lg font-medium text-gray-900">
                    {formatPercentage(dashboardData?.kpis.customerSatisfaction || 0)}
                  </dd>
                </dl>
              </div>
              <div className="flex-shrink-0">
                <CheckCircleIcon className="h-5 w-5 text-orange-500" />
              </div>
            </div>
          </div>
          <div className="bg-gray-50 px-5 py-3">
            <div className="text-sm">
              <span className="font-medium text-orange-600">
                {formatPercentage(dashboardData?.kpis.marketShare || 0)}
              </span>
              <span className="text-gray-500 ml-1">market share</span>
            </div>
          </div>
        </motion.div>
      </div>

      {/* Charts Grid */}
      <div className="grid grid-cols-1 lg:grid-cols-2 gap-6">
        {/* Revenue Trends */}
        <motion.div
          initial={{ opacity: 0, scale: 0.95 }}
          animate={{ opacity: 1, scale: 1 }}
          transition={{ duration: 0.5, delay: 0.4 }}
          className="bg-white shadow rounded-lg p-6"
        >
          <div className="flex items-center justify-between mb-4">
            <h3 className="text-lg font-medium text-gray-900">Revenue & Growth Trends</h3>
            <CurrencyDollarIcon className="h-5 w-5 text-gray-400" />
          </div>
          <div className="h-64">
            <Line data={generateRevenueTrendChart()} options={chartOptions} />
          </div>
        </motion.div>

        {/* Operational Performance */}
        <motion.div
          initial={{ opacity: 0, scale: 0.95 }}
          animate={{ opacity: 1, scale: 1 }}
          transition={{ duration: 0.5, delay: 0.5 }}
          className="bg-white shadow rounded-lg p-6"
        >
          <div className="flex items-center justify-between mb-4">
            <h3 className="text-lg font-medium text-gray-900">Operational Performance</h3>
            <BoltIcon className="h-5 w-5 text-gray-400" />
          </div>
          <div className="h-64">
            <Bar data={generateOperationalMetricsChart()} options={chartOptions} />
          </div>
        </motion.div>

        {/* Fleet Utilization */}
        <motion.div
          initial={{ opacity: 0, scale: 0.95 }}
          animate={{ opacity: 1, scale: 1 }}
          transition={{ duration: 0.5, delay: 0.6 }}
          className="bg-white shadow rounded-lg p-6"
        >
          <div className="flex items-center justify-between mb-4">
            <h3 className="text-lg font-medium text-gray-900">Fleet Utilization by Type</h3>
            <TruckIcon className="h-5 w-5 text-gray-400" />
          </div>
          <div className="h-64">
            <Doughnut data={generateFleetUtilizationChart()} options={chartOptions} />
          </div>
        </motion.div>

        {/* Regional Performance */}
        <motion.div
          initial={{ opacity: 0, scale: 0.95 }}
          animate={{ opacity: 1, scale: 1 }}
          transition={{ duration: 0.5, delay: 0.7 }}
          className="bg-white shadow rounded-lg p-6"
        >
          <div className="flex items-center justify-between mb-4">
            <h3 className="text-lg font-medium text-gray-900">Regional Performance</h3>
            <GlobeAltIcon className="h-5 w-5 text-gray-400" />
          </div>
          <div className="h-64">
            <Bar data={generateRegionalPerformanceChart()} options={chartOptions} />
          </div>
        </motion.div>
      </div>

      {/* Strategic Insights */}
      <div className="grid grid-cols-1 lg:grid-cols-3 gap-6">
        {/* Financial Health */}
        <motion.div
          initial={{ opacity: 0, y: 20 }}
          animate={{ opacity: 1, y: 0 }}
          transition={{ duration: 0.5, delay: 0.8 }}
          className="bg-white shadow rounded-lg p-6"
        >
          <div className="flex items-center justify-between mb-4">
            <h3 className="text-lg font-medium text-gray-900">Financial Health</h3>
            <ChartBarIcon className="h-5 w-5 text-gray-400" />
          </div>
          
          <div className="space-y-4">
            <div className="flex justify-between items-center">
              <span className="text-sm text-gray-600">Profit Margin</span>
              <div className="flex items-center">
                <span className="text-sm font-medium text-gray-900">
                  {formatPercentage(dashboardData?.kpis.profitMargin || 0)}
                </span>
                {dashboardData?.kpis.profitMargin && dashboardData.kpis.profitMargin > 15 ? (
                  <CheckCircleIcon className="h-4 w-4 text-green-500 ml-1" />
                ) : (
                  <ExclamationTriangleIcon className="h-4 w-4 text-yellow-500 ml-1" />
                )}
              </div>
            </div>
            
            <div className="flex justify-between items-center">
              <span className="text-sm text-gray-600">Cash Flow</span>
              <div className="flex items-center">
                <span className="text-sm font-medium text-gray-900">
                  {formatCurrency(dashboardData?.financials.cashFlow.free || 0)}
                </span>
                {(dashboardData?.financials.cashFlow.free || 0) > 0 ? (
                  <ArrowUpIcon className="h-4 w-4 text-green-500 ml-1" />
                ) : (
                  <ArrowDownIcon className="h-4 w-4 text-red-500 ml-1" />
                )}
              </div>
            </div>
            
            <div className="flex justify-between items-center">
              <span className="text-sm text-gray-600">ROI</span>
              <div className="flex items-center">
                <span className="text-sm font-medium text-gray-900">
                  {formatPercentage(dashboardData?.financials.roi.overall || 0)}
                </span>
                {(dashboardData?.financials.roi.overall || 0) > 20 ? (
                  <CheckCircleIcon className="h-4 w-4 text-green-500 ml-1" />
                ) : (
                  <ClockIcon className="h-4 w-4 text-yellow-500 ml-1" />
                )}
              </div>
            </div>
            
            <div className="flex justify-between items-center">
              <span className="text-sm text-gray-600">Burn Rate</span>
              <span className="text-sm font-medium text-gray-900">
                {formatCurrency(dashboardData?.financials.cashFlow.burnRate || 0)}/month
              </span>
            </div>
          </div>
        </motion.div>

        {/* Risk Assessment */}
        <motion.div
          initial={{ opacity: 0, y: 20 }}
          animate={{ opacity: 1, y: 0 }}
          transition={{ duration: 0.5, delay: 0.9 }}
          className="bg-white shadow rounded-lg p-6"
        >
          <div className="flex items-center justify-between mb-4">
            <h3 className="text-lg font-medium text-gray-900">Risk Assessment</h3>
            <ExclamationTriangleIcon className="h-5 w-5 text-gray-400" />
          </div>
          
          <div className="space-y-4">
            <div className="flex justify-between items-center">
              <span className="text-sm text-gray-600">Overall Risk Score</span>
              <div className="flex items-center">
                <span className={`text-sm font-medium ${
                  (dashboardData?.risks.overallRisk || 0) < 30 ? 'text-green-600' :
                  (dashboardData?.risks.overallRisk || 0) < 60 ? 'text-yellow-600' : 'text-red-600'
                }`}>
                  {dashboardData?.risks.overallRisk || 0}/100
                </span>
              </div>
            </div>
            
            {dashboardData?.risks.risks?.slice(0, 3).map((risk, index) => (
              <div key={index} className="flex justify-between items-center">
                <span className="text-sm text-gray-600 truncate">{risk.risk}</span>
                <span className={`text-xs px-2 py-1 rounded-full ${
                  risk.score < 30 ? 'bg-green-100 text-green-800' :
                  risk.score < 60 ? 'bg-yellow-100 text-yellow-800' : 'bg-red-100 text-red-800'
                }`}>
                  {risk.score}
                </span>
              </div>
            ))}
          </div>
        </motion.div>

        {/* Sustainability Metrics */}
        <motion.div
          initial={{ opacity: 0, y: 20 }}
          animate={{ opacity: 1, y: 0 }}
          transition={{ duration: 0.5, delay: 1.0 }}
          className="bg-white shadow rounded-lg p-6"
        >
          <div className="flex items-center justify-between mb-4">
            <h3 className="text-lg font-medium text-gray-900">Sustainability</h3>
            <div className="flex items-center space-x-1">
              <span className="text-xs text-green-600">🌱 Green</span>
              <div className="w-2 h-2 bg-green-500 rounded-full"></div>
            </div>
          </div>
          
          <div className="space-y-4">
            <div className="flex justify-between items-center">
              <span className="text-sm text-gray-600">Sustainability Score</span>
              <div className="flex items-center">
                <span className="text-sm font-medium text-green-600">
                  {formatPercentage(dashboardData?.sustainability.sustainabilityScore || 0)}
                </span>
                <CheckCircleIcon className="h-4 w-4 text-green-500 ml-1" />
              </div>
            </div>
            
            <div className="flex justify-between items-center">
              <span className="text-sm text-gray-600">Carbon Footprint</span>
              <span className="text-sm font-medium text-gray-900">
                {formatNumber(dashboardData?.sustainability.carbonFootprint || 0)} tons CO₂
              </span>
            </div>
            
            <div className="flex justify-between items-center">
              <span className="text-sm text-gray-600">Energy Efficiency</span>
              <span className="text-sm font-medium text-green-600">
                {formatPercentage(dashboardData?.sustainability.energyEfficiency || 0)}
              </span>
            </div>
            
            <div className="flex justify-between items-center">
              <span className="text-sm text-gray-600">Green Initiatives</span>
              <span className="text-sm font-medium text-gray-900">
                {dashboardData?.sustainability.greenInitiatives?.length || 0} active
              </span>
            </div>
          </div>
        </motion.div>
      </div>

      {/* Action Items & Alerts */}
      <motion.div
        initial={{ opacity: 0, y: 20 }}
        animate={{ opacity: 1, y: 0 }}
        transition={{ duration: 0.5, delay: 1.1 }}
        className="bg-white shadow rounded-lg p-6"
      >
        <div className="flex items-center justify-between mb-4">
          <h3 className="text-lg font-medium text-gray-900">Executive Action Items</h3>
          <CalendarDaysIcon className="h-5 w-5 text-gray-400" />
        </div>
        
        <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-4">
          <div className="bg-red-50 border border-red-200 rounded-md p-4">
            <div className="flex items-start">
              <ExclamationTriangleIcon className="h-5 w-5 text-red-400 mt-0.5" />
              <div className="ml-3">
                <h4 className="text-sm font-medium text-red-800">Critical Risks</h4>
                <p className="mt-1 text-sm text-red-700">
                  {dashboardData?.risks.risks?.filter(r => r.score > 70).length || 0} high-risk items require immediate attention
                </p>
                <button className="mt-2 text-xs text-red-600 hover:text-red-800 underline">
                  Review Risk Assessment
                </button>
              </div>
            </div>
          </div>
          
          <div className="bg-yellow-50 border border-yellow-200 rounded-md p-4">
            <div className="flex items-start">
              <ClockIcon className="h-5 w-5 text-yellow-400 mt-0.5" />
              <div className="ml-3">
                <h4 className="text-sm font-medium text-yellow-800">Compliance Reviews</h4>
                <p className="mt-1 text-sm text-yellow-700">
                  {dashboardData?.compliance.regulations?.filter(r => r.status === 'pending').length || 0} regulations pending review
                </p>
                <button className="mt-2 text-xs text-yellow-600 hover:text-yellow-800 underline">
                  View Compliance Dashboard
                </button>
              </div>
            </div>
          </div>
          
          <div className="bg-blue-50 border border-blue-200 rounded-md p-4">
            <div className="flex items-start">
              <ArrowTrendingUpIcon className="h-5 w-5 text-blue-400 mt-0.5" />
              <div className="ml-3">
                <h4 className="text-sm font-medium text-blue-800">Growth Opportunities</h4>
                <p className="mt-1 text-sm text-blue-700">
                  {dashboardData?.market.opportunities?.length || 0} market opportunities identified
                </p>
                <button className="mt-2 text-xs text-blue-600 hover:text-blue-800 underline">
                  Explore Opportunities
                </button>
              </div>
            </div>
          </div>
        </div>
      </motion.div>
    </div>
  );
};

export default ExecutiveDashboard;
