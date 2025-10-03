import React, { useState, useEffect, useMemo } from 'react';
import { motion, AnimatePresence } from 'framer-motion';
import {
  ShieldCheckIcon,
  DocumentTextIcon,
  ExclamationTriangleIcon,
  CheckCircleIcon,
  ClockIcon,
  XCircleIcon,
  InformationCircleIcon,
  ArrowDownTrayIcon,
  EyeIcon,
  Cog6ToothIcon,
  CalendarDaysIcon,
  GlobeAltIcon,
  BuildingOfficeIcon,
  ScaleIcon,
} from '@heroicons/react/24/outline';
import { useQuery } from '@tanstack/react-query';
import { Bar, Line, Doughnut, Radar } from 'react-chartjs-2';
import {
  Chart as ChartJS,
  CategoryScale,
  LinearScale,
  PointElement,
  LineElement,
  BarElement,
  ArcElement,
  RadialLinearScale,
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
  RadialLinearScale,
  Title,
  Tooltip,
  Legend,
  Filler
);

interface ComplianceData {
  overallCompliance: OverallCompliance;
  regulations: RegulationCompliance[];
  auditTrails: AuditTrail[];
  evidencePackages: EvidencePackage[];
  riskAssessment: ComplianceRiskAssessment;
  certifications: Certification[];
  reports: ComplianceReport[];
  uaeSpecificCompliance: UAECompliance;
  internationalStandards: InternationalStandard[];
}

interface OverallCompliance {
  complianceScore: number;
  totalRegulations: number;
  compliantRegulations: number;
  nonCompliantRegulations: number;
  pendingRegulations: number;
  lastAuditDate: string;
  nextAuditDate: string;
  criticalIssues: number;
  resolvedIssues: number;
}

interface RegulationCompliance {
  id: string;
  name: string;
  category: string;
  jurisdiction: string;
  status: 'compliant' | 'non-compliant' | 'pending' | 'partial';
  compliancePercentage: number;
  lastAssessment: string;
  nextReview: string;
  requirements: Requirement[];
  evidence: EvidenceItem[];
  riskLevel: 'low' | 'medium' | 'high' | 'critical';
  responsibleTeam: string;
  estimatedEffort: string;
}

interface Requirement {
  id: string;
  description: string;
  status: 'met' | 'not-met' | 'partial' | 'not-applicable';
  evidence: string[];
  lastVerified: string;
  dueDate?: string;
}

interface EvidenceItem {
  id: string;
  type: string;
  description: string;
  documentUrl: string;
  uploadDate: string;
  expiryDate?: string;
  status: 'valid' | 'expired' | 'pending-renewal';
}

interface AuditTrail {
  id: string;
  timestamp: string;
  action: string;
  actor: string;
  resource: string;
  outcome: string;
  details: Record<string, any>;
  complianceImpact: string;
  riskLevel: string;
}

interface EvidencePackage {
  id: string;
  name: string;
  description: string;
  createdDate: string;
  lastUpdated: string;
  status: 'complete' | 'incomplete' | 'under-review' | 'approved';
  regulations: string[];
  documents: DocumentReference[];
  automatedEvidence: AutomatedEvidence[];
  reviewers: string[];
  approvalDate?: string;
}

interface DocumentReference {
  id: string;
  name: string;
  type: string;
  size: string;
  uploadDate: string;
  hash: string;
  digitalSignature?: string;
}

interface AutomatedEvidence {
  type: string;
  source: string;
  timestamp: string;
  data: Record<string, any>;
  verificationStatus: string;
}

interface ComplianceRiskAssessment {
  overallRiskScore: number;
  riskFactors: RiskFactor[];
  mitigationStrategies: MitigationStrategy[];
  complianceGaps: ComplianceGap[];
  recommendations: string[];
}

interface RiskFactor {
  factor: string;
  probability: number;
  impact: string;
  riskScore: number;
  affectedRegulations: string[];
  mitigationActions: string[];
}

interface MitigationStrategy {
  strategy: string;
  effectiveness: number;
  implementationCost: number;
  timeframe: string;
  priority: string;
}

interface ComplianceGap {
  regulation: string;
  requirement: string;
  currentStatus: string;
  targetStatus: string;
  gapDescription: string;
  remediation: string;
  timeline: string;
  cost: number;
}

interface Certification {
  id: string;
  name: string;
  issuingBody: string;
  issueDate: string;
  expiryDate: string;
  status: 'valid' | 'expired' | 'pending-renewal' | 'suspended';
  scope: string;
  certificateUrl: string;
  renewalProcess: string;
  cost: number;
}

interface ComplianceReport {
  id: string;
  name: string;
  type: string;
  generatedDate: string;
  period: string;
  status: 'draft' | 'final' | 'submitted' | 'approved';
  submissionDeadline?: string;
  recipient: string;
  downloadUrl: string;
  size: string;
}

interface UAECompliance {
  adtaCompliance: ADTACompliance;
  emiratesCompliance: EmiratesCompliance;
  federalCompliance: FederalCompliance;
  islamicConsiderations: IslamicConsideration[];
  culturalCompliance: CulturalCompliance;
}

interface ADTACompliance {
  licenseStatus: string;
  operationalPermits: OperationalPermit[];
  safetyStandards: SafetyStandard[];
  reportingRequirements: ReportingRequirement[];
  inspectionSchedule: InspectionSchedule[];
}

interface OperationalPermit {
  permitType: string;
  permitNumber: string;
  issueDate: string;
  expiryDate: string;
  status: string;
  conditions: string[];
}

interface SafetyStandard {
  standard: string;
  complianceLevel: number;
  lastAssessment: string;
  nextAssessment: string;
  requirements: string[];
}

interface ReportingRequirement {
  reportType: string;
  frequency: string;
  nextDue: string;
  status: string;
  recipient: string;
}

interface InspectionSchedule {
  inspectionType: string;
  scheduledDate: string;
  inspector: string;
  status: string;
  checklist: string[];
}

interface EmiratesCompliance {
  emirate: string;
  localRegulations: LocalRegulation[];
  municipalRequirements: MunicipalRequirement[];
  environmentalCompliance: EnvironmentalCompliance;
}

interface LocalRegulation {
  regulation: string;
  complianceStatus: string;
  requirements: string[];
  lastUpdate: string;
}

interface MunicipalRequirement {
  requirement: string;
  status: string;
  dueDate: string;
  cost: number;
}

interface EnvironmentalCompliance {
  emissionStandards: EmissionStandard[];
  wasteManagement: WasteManagementCompliance;
  energyEfficiency: EnergyEfficiencyCompliance;
}

interface EmissionStandard {
  standard: string;
  currentLevel: number;
  requiredLevel: number;
  complianceStatus: string;
}

interface WasteManagementCompliance {
  wasteTypes: string[];
  disposalMethods: string[];
  certifications: string[];
  complianceLevel: number;
}

interface EnergyEfficiencyCompliance {
  standards: string[];
  currentEfficiency: number;
  targetEfficiency: number;
  improvementPlan: string[];
}

interface FederalCompliance {
  federalLaws: FederalLaw[];
  nationalStandards: NationalStandard[];
  crossBorderRequirements: CrossBorderRequirement[];
}

interface FederalLaw {
  law: string;
  complianceStatus: string;
  requirements: string[];
  penalties: string[];
}

interface NationalStandard {
  standard: string;
  complianceLevel: number;
  certification: string;
  validUntil: string;
}

interface CrossBorderRequirement {
  requirement: string;
  applicableRoutes: string[];
  documentation: string[];
  complianceStatus: string;
}

interface IslamicConsideration {
  aspect: string;
  requirement: string;
  implementation: string;
  complianceLevel: number;
}

interface CulturalCompliance {
  prayerTimeConsiderations: PrayerTimeConsideration;
  ramadanAdjustments: RamadanAdjustment;
  culturalSensitivity: CulturalSensitivity;
  languageRequirements: LanguageRequirement[];
}

interface PrayerTimeConsideration {
  implementationStatus: string;
  adjustmentsMade: string[];
  complianceLevel: number;
}

interface RamadanAdjustment {
  operationalAdjustments: string[];
  scheduleModifications: string[];
  complianceLevel: number;
}

interface CulturalSensitivity {
  trainingPrograms: string[];
  policies: string[];
  complianceLevel: number;
}

interface LanguageRequirement {
  language: string;
  requirement: string;
  implementationStatus: string;
  complianceLevel: number;
}

interface InternationalStandard {
  standard: string;
  organization: string;
  complianceLevel: number;
  certification: string;
  validUntil: string;
  requirements: string[];
}

const ComplianceDashboard: React.FC = () => {
  const [selectedTimeRange, setSelectedTimeRange] = useState('30d');
  const [selectedCategory, setSelectedCategory] = useState('all');
  const [selectedJurisdiction, setSelectedJurisdiction] = useState('all');
  const [refreshInterval, setRefreshInterval] = useState(300000); // 5 minutes

  // Fetch compliance data
  const { data: complianceData, isLoading, error, refetch } = useQuery<ComplianceData>({
    queryKey: ['compliance', selectedTimeRange, selectedCategory, selectedJurisdiction],
    queryFn: async () => {
      const params = new URLSearchParams({
        range: selectedTimeRange,
        category: selectedCategory,
        jurisdiction: selectedJurisdiction,
      });
      
      const response = await fetch(`/api/v1/compliance/dashboard?${params}`);
      if (!response.ok) {
        throw new Error('Failed to fetch compliance data');
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

  // Generate compliance status chart
  const generateComplianceStatusChart = () => {
    if (!complianceData?.overallCompliance) return null;

    return {
      labels: ['Compliant', 'Non-Compliant', 'Pending Review', 'Partial Compliance'],
      datasets: [
        {
          data: [
            complianceData.overallCompliance.compliantRegulations,
            complianceData.overallCompliance.nonCompliantRegulations,
            complianceData.overallCompliance.pendingRegulations,
            Math.max(0, complianceData.overallCompliance.totalRegulations - 
              complianceData.overallCompliance.compliantRegulations - 
              complianceData.overallCompliance.nonCompliantRegulations - 
              complianceData.overallCompliance.pendingRegulations),
          ],
          backgroundColor: ['#10B981', '#EF4444', '#F59E0B', '#6B7280'],
          borderWidth: 0,
        },
      ],
    };
  };

  // Generate compliance trends chart
  const generateComplianceTrendsChart = () => {
    // Mock trend data - in production, this would come from the API
    const months = ['Jan', 'Feb', 'Mar', 'Apr', 'May', 'Jun'];
    const complianceScores = [82, 85, 88, 87, 90, 92];
    const riskScores = [28, 25, 22, 23, 20, 18];

    return {
      labels: months,
      datasets: [
        {
          label: 'Compliance Score %',
          data: complianceScores,
          borderColor: '#10B981',
          backgroundColor: 'rgba(16, 185, 129, 0.1)',
          fill: true,
          tension: 0.4,
        },
        {
          label: 'Risk Score %',
          data: riskScores,
          borderColor: '#EF4444',
          backgroundColor: 'rgba(239, 68, 68, 0.1)',
          fill: true,
          tension: 0.4,
        },
      ],
    };
  };

  // Generate UAE compliance radar chart
  const generateUAEComplianceRadarChart = () => {
    if (!complianceData?.uaeSpecificCompliance) return null;

    return {
      labels: [
        'ADTA Compliance',
        'Federal Laws',
        'Environmental',
        'Cultural Sensitivity',
        'Islamic Considerations',
        'Municipal Requirements',
      ],
      datasets: [
        {
          label: 'Compliance Level %',
          data: [95, 88, 92, 96, 98, 85], // Mock data - would come from API
          backgroundColor: 'rgba(59, 130, 246, 0.2)',
          borderColor: '#3B82F6',
          pointBackgroundColor: '#3B82F6',
          pointBorderColor: '#fff',
          pointHoverBackgroundColor: '#fff',
          pointHoverBorderColor: '#3B82F6',
        },
      ],
    };
  };

  const getStatusColor = (status: string) => {
    switch (status) {
      case 'compliant':
      case 'valid':
      case 'approved':
      case 'complete':
        return 'text-green-600 bg-green-100';
      case 'non-compliant':
      case 'expired':
      case 'suspended':
        return 'text-red-600 bg-red-100';
      case 'pending':
      case 'under-review':
      case 'pending-renewal':
        return 'text-yellow-600 bg-yellow-100';
      case 'partial':
      case 'incomplete':
        return 'text-orange-600 bg-orange-100';
      default:
        return 'text-gray-600 bg-gray-100';
    }
  };

  const getStatusIcon = (status: string) => {
    switch (status) {
      case 'compliant':
      case 'valid':
      case 'approved':
      case 'complete':
        return <CheckCircleIcon className="h-4 w-4" />;
      case 'non-compliant':
      case 'expired':
      case 'suspended':
        return <XCircleIcon className="h-4 w-4" />;
      case 'pending':
      case 'under-review':
      case 'pending-renewal':
        return <ClockIcon className="h-4 w-4" />;
      case 'partial':
      case 'incomplete':
        return <ExclamationTriangleIcon className="h-4 w-4" />;
      default:
        return <InformationCircleIcon className="h-4 w-4" />;
    }
  };

  if (isLoading) {
    return (
      <div className="flex items-center justify-center h-64">
        <div className="animate-spin rounded-full h-8 w-8 border-b-2 border-blue-600"></div>
        <span className="ml-2 text-gray-600">Loading compliance data...</span>
      </div>
    );
  }

  if (error) {
    return (
      <div className="bg-red-50 border border-red-200 rounded-md p-4">
        <div className="flex">
          <ExclamationTriangleIcon className="h-5 w-5 text-red-400" />
          <div className="ml-3">
            <h3 className="text-sm font-medium text-red-800">Error loading compliance data</h3>
            <p className="mt-1 text-sm text-red-700">
              Failed to load compliance dashboard. Please try refreshing the page.
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
          <h1 className="text-2xl font-bold text-gray-900">Compliance Dashboard</h1>
          <p className="mt-1 text-sm text-gray-500">
            Comprehensive regulatory compliance monitoring for UAE autonomous vehicle operations
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
            <option value="90d">Last 90 Days</option>
            <option value="1y">Last Year</option>
          </select>
          
          <select
            value={selectedCategory}
            onChange={(e) => setSelectedCategory(e.target.value)}
            className="rounded-md border-gray-300 shadow-sm focus:border-blue-500 focus:ring-blue-500 sm:text-sm"
          >
            <option value="all">All Categories</option>
            <option value="safety">Safety</option>
            <option value="environmental">Environmental</option>
            <option value="operational">Operational</option>
            <option value="data-privacy">Data Privacy</option>
          </select>
          
          <select
            value={selectedJurisdiction}
            onChange={(e) => setSelectedJurisdiction(e.target.value)}
            className="rounded-md border-gray-300 shadow-sm focus:border-blue-500 focus:ring-blue-500 sm:text-sm"
          >
            <option value="all">All Jurisdictions</option>
            <option value="uae">UAE Federal</option>
            <option value="abu-dhabi">Abu Dhabi</option>
            <option value="adta">ADTA</option>
            <option value="international">International</option>
          </select>
          
          <button
            onClick={() => refetch()}
            className="inline-flex items-center px-3 py-2 border border-gray-300 shadow-sm text-sm leading-4 font-medium rounded-md text-gray-700 bg-white hover:bg-gray-50 focus:outline-none focus:ring-2 focus:ring-offset-2 focus:ring-blue-500"
          >
            <ArrowDownTrayIcon className="h-4 w-4 mr-1" />
            Refresh
          </button>
        </div>
      </div>

      {/* Key Compliance Metrics */}
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
                <ShieldCheckIcon className="h-6 w-6 text-green-600" />
              </div>
              <div className="ml-5 w-0 flex-1">
                <dl>
                  <dt className="text-sm font-medium text-gray-500 truncate">Overall Compliance</dt>
                  <dd className="text-lg font-medium text-gray-900">
                    {complianceData?.overallCompliance.complianceScore.toFixed(1)}%
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
              <span className="font-medium text-green-600">
                {complianceData?.overallCompliance.compliantRegulations}
              </span>
              <span className="text-gray-500 ml-1">
                of {complianceData?.overallCompliance.totalRegulations} regulations
              </span>
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
                <ExclamationTriangleIcon className="h-6 w-6 text-red-600" />
              </div>
              <div className="ml-5 w-0 flex-1">
                <dl>
                  <dt className="text-sm font-medium text-gray-500 truncate">Critical Issues</dt>
                  <dd className="text-lg font-medium text-gray-900">
                    {complianceData?.overallCompliance.criticalIssues || 0}
                  </dd>
                </dl>
              </div>
              <div className="flex-shrink-0">
                {(complianceData?.overallCompliance.criticalIssues || 0) > 0 ? 
                  <ExclamationTriangleIcon className="h-5 w-5 text-red-500" /> :
                  <CheckCircleIcon className="h-5 w-5 text-green-500" />
                }
              </div>
            </div>
          </div>
          <div className="bg-gray-50 px-5 py-3">
            <div className="text-sm">
              <span className="font-medium text-green-600">
                {complianceData?.overallCompliance.resolvedIssues || 0}
              </span>
              <span className="text-gray-500 ml-1">resolved this month</span>
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
                <CalendarDaysIcon className="h-6 w-6 text-blue-600" />
              </div>
              <div className="ml-5 w-0 flex-1">
                <dl>
                  <dt className="text-sm font-medium text-gray-500 truncate">Next Audit</dt>
                  <dd className="text-lg font-medium text-gray-900">
                    {complianceData?.overallCompliance.nextAuditDate ? 
                      new Date(complianceData.overallCompliance.nextAuditDate).toLocaleDateString() : 
                      'TBD'
                    }
                  </dd>
                </dl>
              </div>
              <div className="flex-shrink-0">
                <ClockIcon className="h-5 w-5 text-blue-500" />
              </div>
            </div>
          </div>
          <div className="bg-gray-50 px-5 py-3">
            <div className="text-sm">
              <span className="text-gray-500">Last audit: </span>
              <span className="font-medium text-gray-900">
                {complianceData?.overallCompliance.lastAuditDate ? 
                  new Date(complianceData.overallCompliance.lastAuditDate).toLocaleDateString() : 
                  'Never'
                }
              </span>
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
                <DocumentTextIcon className="h-6 w-6 text-purple-600" />
              </div>
              <div className="ml-5 w-0 flex-1">
                <dl>
                  <dt className="text-sm font-medium text-gray-500 truncate">Evidence Packages</dt>
                  <dd className="text-lg font-medium text-gray-900">
                    {complianceData?.evidencePackages?.length || 0}
                  </dd>
                </dl>
              </div>
              <div className="flex-shrink-0">
                <DocumentTextIcon className="h-5 w-5 text-purple-500" />
              </div>
            </div>
          </div>
          <div className="bg-gray-50 px-5 py-3">
            <div className="text-sm">
              <span className="font-medium text-green-600">
                {complianceData?.evidencePackages?.filter(p => p.status === 'approved').length || 0}
              </span>
              <span className="text-gray-500 ml-1">approved packages</span>
            </div>
          </div>
        </motion.div>
      </div>

      {/* Charts Grid */}
      <div className="grid grid-cols-1 lg:grid-cols-2 gap-6">
        {/* Compliance Status Distribution */}
        <motion.div
          initial={{ opacity: 0, scale: 0.95 }}
          animate={{ opacity: 1, scale: 1 }}
          transition={{ duration: 0.5, delay: 0.4 }}
          className="bg-white shadow rounded-lg p-6"
        >
          <div className="flex items-center justify-between mb-4">
            <h3 className="text-lg font-medium text-gray-900">Compliance Status Distribution</h3>
            <ShieldCheckIcon className="h-5 w-5 text-gray-400" />
          </div>
          <div className="h-64">
            <Doughnut data={generateComplianceStatusChart()} options={chartOptions} />
          </div>
        </motion.div>

        {/* Compliance Trends */}
        <motion.div
          initial={{ opacity: 0, scale: 0.95 }}
          animate={{ opacity: 1, scale: 1 }}
          transition={{ duration: 0.5, delay: 0.5 }}
          className="bg-white shadow rounded-lg p-6"
        >
          <div className="flex items-center justify-between mb-4">
            <h3 className="text-lg font-medium text-gray-900">Compliance Trends (6 Months)</h3>
            <ScaleIcon className="h-5 w-5 text-gray-400" />
          </div>
          <div className="h-64">
            <Line data={generateComplianceTrendsChart()} options={chartOptions} />
          </div>
        </motion.div>

        {/* UAE Compliance Radar */}
        <motion.div
          initial={{ opacity: 0, scale: 0.95 }}
          animate={{ opacity: 1, scale: 1 }}
          transition={{ duration: 0.5, delay: 0.6 }}
          className="bg-white shadow rounded-lg p-6 lg:col-span-2"
        >
          <div className="flex items-center justify-between mb-4">
            <h3 className="text-lg font-medium text-gray-900">UAE Specific Compliance Assessment</h3>
            <div className="flex items-center space-x-2">
              <span className="text-xs text-gray-500">🇦🇪 UAE Specific</span>
              <GlobeAltIcon className="h-5 w-5 text-gray-400" />
            </div>
          </div>
          <div className="h-64">
            <Radar data={generateUAEComplianceRadarChart()} options={chartOptions} />
          </div>
        </motion.div>
      </div>

      {/* Regulations Compliance Table */}
      <motion.div
        initial={{ opacity: 0, y: 20 }}
        animate={{ opacity: 1, y: 0 }}
        transition={{ duration: 0.5, delay: 0.7 }}
        className="bg-white shadow rounded-lg"
      >
        <div className="px-6 py-4 border-b border-gray-200">
          <h3 className="text-lg font-medium text-gray-900">Regulatory Compliance Status</h3>
          <p className="mt-1 text-sm text-gray-500">
            Detailed view of compliance status for each regulation
          </p>
        </div>
        <div className="overflow-hidden">
          <div className="overflow-x-auto">
            <table className="min-w-full divide-y divide-gray-200">
              <thead className="bg-gray-50">
                <tr>
                  <th className="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider">
                    Regulation
                  </th>
                  <th className="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider">
                    Category
                  </th>
                  <th className="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider">
                    Jurisdiction
                  </th>
                  <th className="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider">
                    Status
                  </th>
                  <th className="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider">
                    Compliance %
                  </th>
                  <th className="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider">
                    Risk Level
                  </th>
                  <th className="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider">
                    Next Review
                  </th>
                  <th className="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider">
                    Actions
                  </th>
                </tr>
              </thead>
              <tbody className="bg-white divide-y divide-gray-200">
                {complianceData?.regulations?.slice(0, 10).map((regulation, index) => (
                  <motion.tr
                    key={regulation.id}
                    initial={{ opacity: 0, x: -20 }}
                    animate={{ opacity: 1, x: 0 }}
                    transition={{ duration: 0.3, delay: index * 0.05 }}
                    className="hover:bg-gray-50"
                  >
                    <td className="px-6 py-4 whitespace-nowrap">
                      <div className="flex items-center">
                        <ScaleIcon className="h-5 w-5 text-gray-400 mr-2" />
                        <div>
                          <div className="text-sm font-medium text-gray-900">
                            {regulation.name}
                          </div>
                          <div className="text-sm text-gray-500">
                            {regulation.id}
                          </div>
                        </div>
                      </div>
                    </td>
                    <td className="px-6 py-4 whitespace-nowrap">
                      <span className="inline-flex items-center px-2.5 py-0.5 rounded-full text-xs font-medium bg-blue-100 text-blue-800">
                        {regulation.category}
                      </span>
                    </td>
                    <td className="px-6 py-4 whitespace-nowrap text-sm text-gray-900">
                      {regulation.jurisdiction}
                    </td>
                    <td className="px-6 py-4 whitespace-nowrap">
                      <span className={`inline-flex items-center px-2.5 py-0.5 rounded-full text-xs font-medium ${getStatusColor(regulation.status)}`}>
                        {getStatusIcon(regulation.status)}
                        <span className="ml-1 capitalize">{regulation.status.replace('-', ' ')}</span>
                      </span>
                    </td>
                    <td className="px-6 py-4 whitespace-nowrap">
                      <div className="flex items-center">
                        <div className="w-16 bg-gray-200 rounded-full h-2 mr-2">
                          <div
                            className={`h-2 rounded-full ${
                              regulation.compliancePercentage >= 80 ? 'bg-green-600' :
                              regulation.compliancePercentage >= 60 ? 'bg-yellow-600' : 'bg-red-600'
                            }`}
                            style={{ width: `${regulation.compliancePercentage}%` }}
                          ></div>
                        </div>
                        <span className="text-sm text-gray-900">
                          {regulation.compliancePercentage.toFixed(0)}%
                        </span>
                      </div>
                    </td>
                    <td className="px-6 py-4 whitespace-nowrap">
                      <span className={`inline-flex items-center px-2.5 py-0.5 rounded-full text-xs font-medium ${
                        regulation.riskLevel === 'critical' ? 'bg-red-100 text-red-800' :
                        regulation.riskLevel === 'high' ? 'bg-orange-100 text-orange-800' :
                        regulation.riskLevel === 'medium' ? 'bg-yellow-100 text-yellow-800' :
                        'bg-green-100 text-green-800'
                      }`}>
                        {regulation.riskLevel}
                      </span>
                    </td>
                    <td className="px-6 py-4 whitespace-nowrap text-sm text-gray-900">
                      {new Date(regulation.nextReview).toLocaleDateString()}
                    </td>
                    <td className="px-6 py-4 whitespace-nowrap text-right text-sm font-medium space-x-2">
                      <button className="text-blue-600 hover:text-blue-900">
                        <EyeIcon className="h-4 w-4" />
                      </button>
                      <button className="text-green-600 hover:text-green-900">
                        <ArrowDownTrayIcon className="h-4 w-4" />
                      </button>
                    </td>
                  </motion.tr>
                ))}
              </tbody>
            </table>
          </div>
        </div>
      </motion.div>

      {/* UAE Specific Compliance Section */}
      <motion.div
        initial={{ opacity: 0, y: 20 }}
        animate={{ opacity: 1, y: 0 }}
        transition={{ duration: 0.5, delay: 0.8 }}
        className="bg-white shadow rounded-lg p-6"
      >
        <div className="flex items-center justify-between mb-6">
          <h3 className="text-lg font-medium text-gray-900">UAE Specific Compliance</h3>
          <div className="flex items-center space-x-2">
            <span className="text-xs text-gray-500">🇦🇪 UAE Specific</span>
            <BuildingOfficeIcon className="h-5 w-5 text-gray-400" />
          </div>
        </div>
        
        <div className="grid grid-cols-1 md:grid-cols-3 gap-6">
          {/* ADTA Compliance */}
          <div className="bg-blue-50 rounded-lg p-4">
            <div className="flex items-center mb-3">
              <div className="w-3 h-3 bg-blue-500 rounded-full mr-2"></div>
              <h4 className="text-sm font-medium text-gray-900">ADTA Compliance</h4>
            </div>
            <div className="space-y-2">
              <div className="flex justify-between items-center">
                <span className="text-xs text-gray-600">License Status</span>
                <span className="text-xs font-medium text-green-600">Valid</span>
              </div>
              <div className="flex justify-between items-center">
                <span className="text-xs text-gray-600">Safety Standards</span>
                <span className="text-xs font-medium text-blue-600">95%</span>
              </div>
              <div className="flex justify-between items-center">
                <span className="text-xs text-gray-600">Reporting</span>
                <span className="text-xs font-medium text-green-600">Current</span>
              </div>
            </div>
          </div>

          {/* Cultural Compliance */}
          <div className="bg-purple-50 rounded-lg p-4">
            <div className="flex items-center mb-3">
              <div className="w-3 h-3 bg-purple-500 rounded-full mr-2"></div>
              <h4 className="text-sm font-medium text-gray-900">Cultural Compliance</h4>
            </div>
            <div className="space-y-2">
              <div className="flex justify-between items-center">
                <span className="text-xs text-gray-600">Prayer Time Adaptation</span>
                <span className="text-xs font-medium text-green-600">98%</span>
              </div>
              <div className="flex justify-between items-center">
                <span className="text-xs text-gray-600">Ramadan Adjustments</span>
                <span className="text-xs font-medium text-green-600">100%</span>
              </div>
              <div className="flex justify-between items-center">
                <span className="text-xs text-gray-600">Language Requirements</span>
                <span className="text-xs font-medium text-blue-600">92%</span>
              </div>
            </div>
          </div>

          {/* Environmental Compliance */}
          <div className="bg-green-50 rounded-lg p-4">
            <div className="flex items-center mb-3">
              <div className="w-3 h-3 bg-green-500 rounded-full mr-2"></div>
              <h4 className="text-sm font-medium text-gray-900">Environmental</h4>
            </div>
            <div className="space-y-2">
              <div className="flex justify-between items-center">
                <span className="text-xs text-gray-600">Emission Standards</span>
                <span className="text-xs font-medium text-green-600">Compliant</span>
              </div>
              <div className="flex justify-between items-center">
                <span className="text-xs text-gray-600">Waste Management</span>
                <span className="text-xs font-medium text-green-600">Certified</span>
              </div>
              <div className="flex justify-between items-center">
                <span className="text-xs text-gray-600">Energy Efficiency</span>
                <span className="text-xs font-medium text-blue-600">88%</span>
              </div>
            </div>
          </div>
        </div>
      </motion.div>

      {/* Recent Audit Trail */}
      <motion.div
        initial={{ opacity: 0, y: 20 }}
        animate={{ opacity: 1, y: 0 }}
        transition={{ duration: 0.5, delay: 0.9 }}
        className="bg-white shadow rounded-lg p-6"
      >
        <div className="flex items-center justify-between mb-4">
          <h3 className="text-lg font-medium text-gray-900">Recent Audit Trail</h3>
          <Cog6ToothIcon className="h-5 w-5 text-gray-400" />
        </div>
        
        <div className="space-y-3">
          {complianceData?.auditTrails?.slice(0, 5).map((trail, index) => (
            <div key={trail.id} className="flex items-start space-x-3 p-3 bg-gray-50 rounded-md">
              <div className="flex-shrink-0">
                <div className={`w-2 h-2 rounded-full mt-2 ${
                  trail.outcome === 'success' ? 'bg-green-500' :
                  trail.outcome === 'warning' ? 'bg-yellow-500' : 'bg-red-500'
                }`}></div>
              </div>
              <div className="flex-1 min-w-0">
                <div className="text-sm font-medium text-gray-900">
                  {trail.action}
                </div>
                <div className="text-sm text-gray-500">
                  {trail.actor} • {trail.resource}
                </div>
                <div className="text-xs text-gray-400 mt-1">
                  {new Date(trail.timestamp).toLocaleString()}
                </div>
              </div>
              <div className="flex-shrink-0">
                <span className={`inline-flex items-center px-2 py-1 rounded-full text-xs font-medium ${
                  trail.riskLevel === 'high' ? 'bg-red-100 text-red-800' :
                  trail.riskLevel === 'medium' ? 'bg-yellow-100 text-yellow-800' :
                  'bg-green-100 text-green-800'
                }`}>
                  {trail.riskLevel}
                </span>
              </div>
            </div>
          ))}
        </div>
      </motion.div>
    </div>
  );
};

export default ComplianceDashboard;
