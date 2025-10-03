import React, { useState, useEffect, useCallback, useMemo } from 'react'
import { 
  Database, Shield, Clock, Globe, MapPin, FileX, AlertTriangle,
  CheckCircle, XCircle, Settings, RefreshCw, Download, Upload,
  Search, Filter, Eye, EyeOff, Lock, Unlock, Calendar, Users,
  Activity, BarChart3, Trash2, Archive, Copy, ExternalLink
} from 'lucide-react'
import { Button } from '../ui/Button'
import { Badge } from '../ui/Badge'
import { Card } from '../ui/Card'
import { Dialog, DialogContent, DialogHeader, DialogTitle, DialogFooter } from '../ui/Dialog'
import { Input } from '../ui/Input'
import { Textarea } from '../ui/Textarea'
import { Select } from '../ui/Select'
import { Checkbox } from '../ui/Checkbox'
import { Alert, AlertDescription } from '../ui/Alert'
import { Tabs, TabsContent, TabsList, TabsTrigger } from '../ui/Tabs'
import { Table, TableBody, TableCell, TableHead, TableHeader, TableRow } from '../ui/Table'
import { Switch } from '../ui/Switch'
import { Label } from '../ui/Label'
import { Progress } from '../ui/Progress'
import { DatePicker } from '../ui/DatePicker'

// Types
interface DataCategory {
  id: string
  name: string
  description: string
  classification: 'public' | 'internal' | 'confidential' | 'restricted'
  piiLevel: 'none' | 'low' | 'medium' | 'high'
  regulatoryFrameworks: string[]
  dataTypes: string[]
  sources: string[]
  purposes: string[]
  retentionPolicy: RetentionPolicy
  residencyPolicy: ResidencyPolicy
  accessControls: AccessControl[]
  encryptionRequirements: EncryptionRequirement
  auditRequirements: AuditRequirement
  status: 'active' | 'deprecated' | 'pending_review'
  createdAt: Date
  updatedAt: Date
  createdBy: string
  lastReviewedAt?: Date
  nextReviewDue: Date
}

interface RetentionPolicy {
  id: string
  name: string
  defaultRetentionPeriod: number // days
  maxRetentionPeriod: number // days
  minRetentionPeriod: number // days
  autoDeleteEnabled: boolean
  archiveBeforeDelete: boolean
  archiveAfterDays: number
  legalHoldSupport: boolean
  customRules: RetentionRule[]
  exceptions: RetentionException[]
  deletionMethod: 'soft' | 'hard' | 'cryptographic'
  verificationRequired: boolean
  approvalWorkflow: string[]
}

interface ResidencyPolicy {
  id: string
  name: string
  allowedRegions: string[]
  restrictedRegions: string[]
  crossBorderTransferRules: CrossBorderRule[]
  localProcessingRequired: boolean
  dataLocalizationLevel: 'strict' | 'moderate' | 'flexible'
  sovereigntyRequirements: string[]
  transferMechanisms: TransferMechanism[]
  complianceFrameworks: string[]
  exceptions: ResidencyException[]
  monitoring: ResidencyMonitoring
}

interface RetentionRule {
  id: string
  condition: string
  operator: 'equals' | 'not_equals' | 'greater_than' | 'less_than' | 'contains'
  value: string | number
  retentionPeriod: number // days
  action: 'delete' | 'archive' | 'anonymize' | 'pseudonymize'
  priority: number
  enabled: boolean
  description: string
}

interface RetentionException {
  id: string
  reason: 'legal_hold' | 'litigation' | 'investigation' | 'compliance' | 'business_critical'
  description: string
  dataFilter: string
  startDate: Date
  endDate?: Date
  approvedBy: string
  status: 'active' | 'expired' | 'cancelled'
  autoExpiry: boolean
}

interface CrossBorderRule {
  id: string
  fromRegion: string
  toRegion: string
  allowed: boolean
  conditions: string[]
  transferMechanism: string
  approvalRequired: boolean
  monitoringLevel: 'basic' | 'enhanced' | 'strict'
  dataTypes: string[]
  purposes: string[]
}

interface TransferMechanism {
  id: string
  name: string
  type: 'adequacy_decision' | 'scc' | 'bcr' | 'certification' | 'cod'
  description: string
  applicableRegions: string[]
  validUntil?: Date
  requirements: string[]
  status: 'active' | 'expired' | 'pending'
}

interface ResidencyException {
  id: string
  reason: string
  description: string
  dataTypes: string[]
  fromRegion: string
  toRegion: string
  approvedBy: string
  validFrom: Date
  validUntil: Date
  conditions: string[]
  status: 'active' | 'expired' | 'pending'
}

interface ResidencyMonitoring {
  enabled: boolean
  alertOnViolation: boolean
  blockOnViolation: boolean
  auditLevel: 'basic' | 'detailed' | 'comprehensive'
  reportingFrequency: 'daily' | 'weekly' | 'monthly'
  dashboardEnabled: boolean
}

interface AccessControl {
  role: string
  permissions: string[]
  conditions: string[]
  timeRestrictions?: TimeRestriction[]
  ipRestrictions?: string[]
  purposeRestrictions?: string[]
}

interface TimeRestriction {
  days: string[]
  startTime: string
  endTime: string
  timezone: string
}

interface EncryptionRequirement {
  atRest: boolean
  inTransit: boolean
  inProcessing: boolean
  keyManagement: 'customer' | 'service' | 'hybrid'
  algorithm: string
  keyRotationDays: number
  hsm: boolean
}

interface AuditRequirement {
  level: 'basic' | 'detailed' | 'comprehensive'
  retention: number // days
  realTimeMonitoring: boolean
  alerting: boolean
  reporting: string[]
  thirdPartyAccess: boolean
}

interface DataFlow {
  id: string
  name: string
  source: DataSource
  destination: DataDestination
  dataCategories: string[]
  volume: DataVolume
  frequency: 'real-time' | 'batch' | 'on-demand'
  encryption: boolean
  monitoring: boolean
  compliance: ComplianceStatus
  lastTransfer: Date
  status: 'active' | 'paused' | 'error'
}

interface DataSource {
  id: string
  name: string
  type: 'database' | 'api' | 'file' | 'stream'
  region: string
  classification: string
}

interface DataDestination {
  id: string
  name: string
  type: 'database' | 'api' | 'file' | 'stream' | 'analytics' | 'archive'
  region: string
  classification: string
}

interface DataVolume {
  recordsPerDay: number
  sizeGB: number
  peakThroughput: number
}

interface ComplianceStatus {
  overall: 'compliant' | 'non-compliant' | 'warning' | 'unknown'
  frameworks: FrameworkCompliance[]
  lastAssessed: Date
  nextAssessment: Date
  issues: ComplianceIssue[]
}

interface FrameworkCompliance {
  framework: string
  status: 'compliant' | 'non-compliant' | 'partial'
  score: number // 0-100
  requirements: RequirementCompliance[]
}

interface RequirementCompliance {
  requirement: string
  status: 'met' | 'not_met' | 'partial'
  evidence: string[]
  gaps: string[]
}

interface ComplianceIssue {
  id: string
  severity: 'low' | 'medium' | 'high' | 'critical'
  category: 'retention' | 'residency' | 'access' | 'encryption' | 'audit'
  description: string
  recommendation: string
  dueDate?: Date
  assignedTo?: string
  status: 'open' | 'in_progress' | 'resolved'
}

interface DataSubject {
  id: string
  type: 'individual' | 'employee' | 'customer' | 'partner'
  region: string
  preferences: DataPreference[]
  consents: DataConsent[]
  requests: DataSubjectRequest[]
  dataCategories: string[]
  lastActivity: Date
}

interface DataPreference {
  category: string
  purpose: string
  allowed: boolean
  restrictionLevel: 'none' | 'limited' | 'strict'
  expiryDate?: Date
}

interface DataConsent {
  id: string
  purpose: string
  dataCategories: string[]
  grantedAt: Date
  expiresAt?: Date
  withdrawnAt?: Date
  method: 'explicit' | 'implicit' | 'legitimate_interest'
  evidence: string
  status: 'active' | 'expired' | 'withdrawn'
}

interface DataSubjectRequest {
  id: string
  type: 'access' | 'rectification' | 'erasure' | 'portability' | 'restriction' | 'objection'
  description: string
  requestedAt: Date
  dueDate: Date
  status: 'pending' | 'in_progress' | 'completed' | 'rejected'
  assignedTo?: string
  response?: string
  completedAt?: Date
}

interface DataResidencyRetentionControlsProps {
  entityId?: string
  entityType?: 'vehicle' | 'fleet' | 'system' | 'user'
  readOnly?: boolean
  onPolicyUpdated?: (policy: any) => void
  onComplianceIssue?: (issue: ComplianceIssue) => void
  className?: string
}

const DataResidencyRetentionControls: React.FC<DataResidencyRetentionControlsProps> = ({
  entityId,
  entityType = 'system',
  readOnly = false,
  onPolicyUpdated,
  onComplianceIssue,
  className = ''
}) => {
  // State
  const [dataCategories, setDataCategories] = useState<DataCategory[]>([])
  const [dataFlows, setDataFlows] = useState<DataFlow[]>([])
  const [dataSubjects, setDataSubjects] = useState<DataSubject[]>([])
  const [selectedCategory, setSelectedCategory] = useState<DataCategory | null>(null)
  const [selectedFlow, setSelectedFlow] = useState<DataFlow | null>(null)
  const [activeTab, setActiveTab] = useState('categories')
  const [showPolicyDialog, setShowPolicyDialog] = useState(false)
  const [showFlowDialog, setShowFlowDialog] = useState(false)
  const [showSubjectDialog, setShowSubjectDialog] = useState(false)
  const [filters, setFilters] = useState({
    search: '',
    classification: '',
    region: '',
    framework: '',
    status: ''
  })
  const [complianceOverview, setComplianceOverview] = useState<any>(null)
  const [loading, setLoading] = useState(false)

  // Mock data initialization
  const initializeMockData = useCallback(() => {
    const mockCategories: DataCategory[] = [
      {
        id: 'cat-vehicle-telemetry',
        name: 'Vehicle Telemetry Data',
        description: 'Real-time and historical vehicle sensor data, location, speed, diagnostics',
        classification: 'confidential',
        piiLevel: 'medium',
        regulatoryFrameworks: ['GDPR', 'UAE_DPA', 'ISO_27001'],
        dataTypes: ['location', 'sensor_data', 'diagnostic_codes', 'performance_metrics'],
        sources: ['vehicle_edge', 'telemetry_gateway', 'diagnostic_api'],
        purposes: ['fleet_optimization', 'predictive_maintenance', 'safety_monitoring'],
        retentionPolicy: {
          id: 'ret-telemetry',
          name: 'Telemetry Retention Policy',
          defaultRetentionPeriod: 2555, // 7 years
          maxRetentionPeriod: 3650, // 10 years
          minRetentionPeriod: 365, // 1 year
          autoDeleteEnabled: true,
          archiveBeforeDelete: true,
          archiveAfterDays: 1095, // 3 years
          legalHoldSupport: true,
          customRules: [
            {
              id: 'rule-accident-data',
              condition: 'incident_type',
              operator: 'equals',
              value: 'accident',
              retentionPeriod: 3650, // 10 years for accident data
              action: 'archive',
              priority: 1,
              enabled: true,
              description: 'Extended retention for accident-related telemetry'
            }
          ],
          exceptions: [],
          deletionMethod: 'cryptographic',
          verificationRequired: true,
          approvalWorkflow: ['data_protection_officer', 'legal_team']
        },
        residencyPolicy: {
          id: 'res-telemetry',
          name: 'Telemetry Residency Policy',
          allowedRegions: ['UAE', 'EU', 'US'],
          restrictedRegions: ['CN', 'RU'],
          crossBorderTransferRules: [
            {
              id: 'rule-uae-eu',
              fromRegion: 'UAE',
              toRegion: 'EU',
              allowed: true,
              conditions: ['adequacy_decision', 'scc'],
              transferMechanism: 'adequacy_decision',
              approvalRequired: false,
              monitoringLevel: 'basic',
              dataTypes: ['aggregated_telemetry'],
              purposes: ['analytics', 'reporting']
            }
          ],
          localProcessingRequired: true,
          dataLocalizationLevel: 'moderate',
          sovereigntyRequirements: ['UAE_TDRA', 'EU_GDPR'],
          transferMechanisms: [
            {
              id: 'mech-adequacy',
              name: 'EU Adequacy Decision',
              type: 'adequacy_decision',
              description: 'EU adequacy decision for UAE',
              applicableRegions: ['UAE', 'EU'],
              requirements: ['data_protection_certification'],
              status: 'active'
            }
          ],
          exceptions: [],
          monitoring: {
            enabled: true,
            alertOnViolation: true,
            blockOnViolation: true,
            auditLevel: 'detailed',
            reportingFrequency: 'daily',
            dashboardEnabled: true
          }
        },
        accessControls: [
          {
            role: 'fleet_operator',
            permissions: ['read', 'aggregate'],
            conditions: ['business_hours', 'operational_purpose'],
            timeRestrictions: [
              {
                days: ['monday', 'tuesday', 'wednesday', 'thursday', 'friday'],
                startTime: '06:00',
                endTime: '22:00',
                timezone: 'Asia/Dubai'
              }
            ],
            purposeRestrictions: ['fleet_optimization', 'safety_monitoring']
          },
          {
            role: 'data_analyst',
            permissions: ['read', 'aggregate', 'anonymize'],
            conditions: ['approved_research', 'anonymization_required'],
            purposeRestrictions: ['analytics', 'reporting', 'research']
          }
        ],
        encryptionRequirements: {
          atRest: true,
          inTransit: true,
          inProcessing: true,
          keyManagement: 'customer',
          algorithm: 'AES-256-GCM',
          keyRotationDays: 90,
          hsm: true
        },
        auditRequirements: {
          level: 'comprehensive',
          retention: 2555, // 7 years
          realTimeMonitoring: true,
          alerting: true,
          reporting: ['monthly_compliance', 'quarterly_review', 'annual_audit'],
          thirdPartyAccess: false
        },
        status: 'active',
        createdAt: new Date('2024-01-15T10:00:00Z'),
        updatedAt: new Date('2024-11-20T14:30:00Z'),
        createdBy: 'data_governance_team',
        lastReviewedAt: new Date('2024-11-01T09:00:00Z'),
        nextReviewDue: new Date('2025-05-01T09:00:00Z')
      },
      {
        id: 'cat-passenger-data',
        name: 'Passenger Personal Data',
        description: 'Passenger identification, contact information, payment data, trip history',
        classification: 'restricted',
        piiLevel: 'high',
        regulatoryFrameworks: ['GDPR', 'UAE_DPA', 'PCI_DSS', 'ISO_27001'],
        dataTypes: ['personal_identifiers', 'contact_info', 'payment_data', 'biometric_data'],
        sources: ['mobile_app', 'booking_system', 'payment_gateway'],
        purposes: ['service_delivery', 'billing', 'customer_support', 'fraud_prevention'],
        retentionPolicy: {
          id: 'ret-passenger',
          name: 'Passenger Data Retention Policy',
          defaultRetentionPeriod: 1095, // 3 years
          maxRetentionPeriod: 2555, // 7 years
          minRetentionPeriod: 30, // 30 days
          autoDeleteEnabled: true,
          archiveBeforeDelete: false,
          archiveAfterDays: 0,
          legalHoldSupport: true,
          customRules: [
            {
              id: 'rule-inactive-users',
              condition: 'last_activity',
              operator: 'greater_than',
              value: 730, // 2 years
              retentionPeriod: 30,
              action: 'anonymize',
              priority: 1,
              enabled: true,
              description: 'Anonymize data for inactive users after 2 years'
            }
          ],
          exceptions: [],
          deletionMethod: 'hard',
          verificationRequired: true,
          approvalWorkflow: ['data_protection_officer', 'privacy_team', 'legal_team']
        },
        residencyPolicy: {
          id: 'res-passenger',
          name: 'Passenger Data Residency Policy',
          allowedRegions: ['UAE'],
          restrictedRegions: ['CN', 'RU', 'IR'],
          crossBorderTransferRules: [],
          localProcessingRequired: true,
          dataLocalizationLevel: 'strict',
          sovereigntyRequirements: ['UAE_TDRA', 'UAE_DPA_LOCALIZATION'],
          transferMechanisms: [],
          exceptions: [],
          monitoring: {
            enabled: true,
            alertOnViolation: true,
            blockOnViolation: true,
            auditLevel: 'comprehensive',
            reportingFrequency: 'daily',
            dashboardEnabled: true
          }
        },
        accessControls: [
          {
            role: 'customer_service',
            permissions: ['read'],
            conditions: ['customer_consent', 'business_purpose'],
            timeRestrictions: [
              {
                days: ['monday', 'tuesday', 'wednesday', 'thursday', 'friday', 'saturday', 'sunday'],
                startTime: '00:00',
                endTime: '23:59',
                timezone: 'Asia/Dubai'
              }
            ],
            purposeRestrictions: ['customer_support', 'service_delivery']
          }
        ],
        encryptionRequirements: {
          atRest: true,
          inTransit: true,
          inProcessing: true,
          keyManagement: 'customer',
          algorithm: 'AES-256-GCM',
          keyRotationDays: 30,
          hsm: true
        },
        auditRequirements: {
          level: 'comprehensive',
          retention: 2555, // 7 years
          realTimeMonitoring: true,
          alerting: true,
          reporting: ['daily_access', 'weekly_compliance', 'monthly_review'],
          thirdPartyAccess: false
        },
        status: 'active',
        createdAt: new Date('2024-01-10T08:00:00Z'),
        updatedAt: new Date('2024-11-25T16:00:00Z'),
        createdBy: 'privacy_team',
        lastReviewedAt: new Date('2024-11-15T10:00:00Z'),
        nextReviewDue: new Date('2025-02-15T10:00:00Z')
      }
    ]

    const mockFlows: DataFlow[] = [
      {
        id: 'flow-telemetry-analytics',
        name: 'Telemetry to Analytics Pipeline',
        source: {
          id: 'src-vehicle-edge',
          name: 'Vehicle Edge Devices',
          type: 'stream',
          region: 'UAE',
          classification: 'confidential'
        },
        destination: {
          id: 'dest-analytics-platform',
          name: 'Analytics Platform',
          type: 'analytics',
          region: 'UAE',
          classification: 'confidential'
        },
        dataCategories: ['cat-vehicle-telemetry'],
        volume: {
          recordsPerDay: 8640000, // 100 records/sec * 86400 sec
          sizeGB: 432, // ~500 bytes per record
          peakThroughput: 1000 // records/sec
        },
        frequency: 'real-time',
        encryption: true,
        monitoring: true,
        compliance: {
          overall: 'compliant',
          frameworks: [
            {
              framework: 'GDPR',
              status: 'compliant',
              score: 95,
              requirements: [
                {
                  requirement: 'Data minimization',
                  status: 'met',
                  evidence: ['data_filtering_config', 'purpose_limitation_policy'],
                  gaps: []
                },
                {
                  requirement: 'Storage limitation',
                  status: 'met',
                  evidence: ['retention_policy', 'automated_deletion'],
                  gaps: []
                }
              ]
            }
          ],
          lastAssessed: new Date('2024-11-20T10:00:00Z'),
          nextAssessment: new Date('2024-12-20T10:00:00Z'),
          issues: []
        },
        lastTransfer: new Date('2024-11-26T10:30:00Z'),
        status: 'active'
      },
      {
        id: 'flow-passenger-backup',
        name: 'Passenger Data Backup',
        source: {
          id: 'src-booking-db',
          name: 'Booking Database',
          type: 'database',
          region: 'UAE',
          classification: 'restricted'
        },
        destination: {
          id: 'dest-backup-storage',
          name: 'Encrypted Backup Storage',
          type: 'archive',
          region: 'UAE',
          classification: 'restricted'
        },
        dataCategories: ['cat-passenger-data'],
        volume: {
          recordsPerDay: 50000,
          sizeGB: 25,
          peakThroughput: 100
        },
        frequency: 'batch',
        encryption: true,
        monitoring: true,
        compliance: {
          overall: 'warning',
          frameworks: [
            {
              framework: 'UAE_DPA',
              status: 'partial',
              score: 78,
              requirements: [
                {
                  requirement: 'Local processing',
                  status: 'met',
                  evidence: ['infrastructure_audit', 'data_center_certification'],
                  gaps: []
                },
                {
                  requirement: 'Access logging',
                  status: 'partial',
                  evidence: ['audit_logs'],
                  gaps: ['real_time_alerting', 'comprehensive_monitoring']
                }
              ]
            }
          ],
          lastAssessed: new Date('2024-11-18T14:00:00Z'),
          nextAssessment: new Date('2024-12-18T14:00:00Z'),
          issues: [
            {
              id: 'issue-backup-monitoring',
              severity: 'medium',
              category: 'audit',
              description: 'Backup access monitoring needs enhancement',
              recommendation: 'Implement real-time alerting for backup access',
              dueDate: new Date('2024-12-15T00:00:00Z'),
              assignedTo: 'security_team',
              status: 'open'
            }
          ]
        },
        lastTransfer: new Date('2024-11-26T02:00:00Z'),
        status: 'active'
      }
    ]

    const mockSubjects: DataSubject[] = [
      {
        id: 'subj-passenger-001',
        type: 'customer',
        region: 'UAE',
        preferences: [
          {
            category: 'location_tracking',
            purpose: 'service_delivery',
            allowed: true,
            restrictionLevel: 'limited'
          },
          {
            category: 'marketing',
            purpose: 'promotional_offers',
            allowed: false,
            restrictionLevel: 'strict'
          }
        ],
        consents: [
          {
            id: 'consent-service-001',
            purpose: 'service_delivery',
            dataCategories: ['location', 'contact_info'],
            grantedAt: new Date('2024-10-15T12:00:00Z'),
            method: 'explicit',
            evidence: 'mobile_app_consent_screen_v2.1',
            status: 'active'
          }
        ],
        requests: [
          {
            id: 'req-access-001',
            type: 'access',
            description: 'Request for copy of all personal data',
            requestedAt: new Date('2024-11-20T14:30:00Z'),
            dueDate: new Date('2024-12-20T14:30:00Z'),
            status: 'in_progress',
            assignedTo: 'privacy_team'
          }
        ],
        dataCategories: ['cat-passenger-data'],
        lastActivity: new Date('2024-11-25T18:45:00Z')
      }
    ]

    const mockComplianceOverview = {
      overallScore: 87,
      totalCategories: mockCategories.length,
      compliantCategories: 1,
      nonCompliantCategories: 0,
      warningCategories: 1,
      totalFlows: mockFlows.length,
      activeFlows: 2,
      totalSubjects: mockSubjects.length,
      activeRequests: 1,
      upcomingReviews: 2,
      criticalIssues: 0,
      frameworks: [
        { name: 'GDPR', score: 92, status: 'compliant' },
        { name: 'UAE_DPA', score: 85, status: 'compliant' },
        { name: 'ISO_27001', score: 89, status: 'compliant' }
      ]
    }

    setDataCategories(mockCategories)
    setDataFlows(mockFlows)
    setDataSubjects(mockSubjects)
    setComplianceOverview(mockComplianceOverview)
  }, [])

  useEffect(() => {
    initializeMockData()
  }, [initializeMockData])

  // Filtered data
  const filteredCategories = useMemo(() => {
    return dataCategories
      .filter(cat => !filters.search || 
        cat.name.toLowerCase().includes(filters.search.toLowerCase()) ||
        cat.description.toLowerCase().includes(filters.search.toLowerCase())
      )
      .filter(cat => !filters.classification || cat.classification === filters.classification)
      .filter(cat => !filters.status || cat.status === filters.status)
      .filter(cat => !filters.framework || cat.regulatoryFrameworks.includes(filters.framework))
      .sort((a, b) => b.updatedAt.getTime() - a.updatedAt.getTime())
  }, [dataCategories, filters])

  const filteredFlows = useMemo(() => {
    return dataFlows
      .filter(flow => !filters.search || 
        flow.name.toLowerCase().includes(filters.search.toLowerCase())
      )
      .filter(flow => !filters.region || 
        flow.source.region === filters.region || 
        flow.destination.region === filters.region
      )
      .filter(flow => !filters.status || flow.status === filters.status)
      .sort((a, b) => b.lastTransfer.getTime() - a.lastTransfer.getTime())
  }, [dataFlows, filters])

  // Handlers
  const handleCategorySelect = useCallback((category: DataCategory) => {
    setSelectedCategory(category)
  }, [])

  const handleFlowSelect = useCallback((flow: DataFlow) => {
    setSelectedFlow(flow)
  }, [])

  const handlePolicyUpdate = useCallback((categoryId: string, updates: Partial<DataCategory>) => {
    setDataCategories(prev => prev.map(cat => 
      cat.id === categoryId 
        ? { ...cat, ...updates, updatedAt: new Date() }
        : cat
    ))
    onPolicyUpdated?.(updates)
  }, [onPolicyUpdated])

  const handleComplianceCheck = useCallback(async (categoryId: string) => {
    setLoading(true)
    
    // Simulate compliance assessment
    setTimeout(() => {
      setDataCategories(prev => prev.map(cat => 
        cat.id === categoryId
          ? {
              ...cat,
              lastReviewedAt: new Date(),
              nextReviewDue: new Date(Date.now() + 90 * 24 * 60 * 60 * 1000), // 90 days from now
              updatedAt: new Date()
            }
          : cat
      ))
      setLoading(false)
    }, 2000)
  }, [])

  // Helper functions
  const getClassificationColor = (classification: string) => {
    switch (classification) {
      case 'public': return 'bg-green-100 text-green-800'
      case 'internal': return 'bg-blue-100 text-blue-800'
      case 'confidential': return 'bg-orange-100 text-orange-800'
      case 'restricted': return 'bg-red-100 text-red-800'
      default: return 'bg-gray-100 text-gray-800'
    }
  }

  const getPiiLevelColor = (level: string) => {
    switch (level) {
      case 'none': return 'bg-gray-100 text-gray-800'
      case 'low': return 'bg-green-100 text-green-800'
      case 'medium': return 'bg-yellow-100 text-yellow-800'
      case 'high': return 'bg-red-100 text-red-800'
      default: return 'bg-gray-100 text-gray-800'
    }
  }

  const getComplianceColor = (status: string) => {
    switch (status) {
      case 'compliant': return 'bg-green-100 text-green-800'
      case 'non-compliant': return 'bg-red-100 text-red-800'
      case 'warning': return 'bg-yellow-100 text-yellow-800'
      case 'partial': return 'bg-orange-100 text-orange-800'
      default: return 'bg-gray-100 text-gray-800'
    }
  }

  const formatRetentionPeriod = (days: number) => {
    if (days >= 365) {
      const years = Math.floor(days / 365)
      const remainingDays = days % 365
      return remainingDays > 0 ? `${years}y ${remainingDays}d` : `${years} year${years > 1 ? 's' : ''}`
    }
    return `${days} day${days > 1 ? 's' : ''}`
  }

  return (
    <div className={`space-y-6 ${className}`}>
      {/* Header */}
      <div className="flex items-center justify-between">
        <div className="flex items-center space-x-3">
          <Shield className="w-6 h-6 text-blue-600" />
          <h2 className="text-2xl font-semibold text-gray-900">Data Residency & Retention Controls</h2>
          {readOnly && (
            <Badge variant="outline">Read Only</Badge>
          )}
          {entityId && (
            <Badge variant="outline">{entityType}: {entityId}</Badge>
          )}
        </div>
        <div className="flex items-center space-x-2">
          {!readOnly && (
            <>
              <Button variant="outline" onClick={() => setShowPolicyDialog(true)}>
                <Settings className="w-4 h-4 mr-2" />
                New Policy
              </Button>
              <Button variant="outline">
                <Upload className="w-4 h-4 mr-2" />
                Import
              </Button>
            </>
          )}
          <Button variant="outline" onClick={initializeMockData}>
            <RefreshCw className="w-4 h-4 mr-2" />
            Refresh
          </Button>
          <Button variant="outline">
            <Download className="w-4 h-4 mr-2" />
            Export Report
          </Button>
        </div>
      </div>

      {/* Compliance Overview */}
      {complianceOverview && (
        <div className="grid grid-cols-2 md:grid-cols-4 lg:grid-cols-8 gap-4">
          <Card className="p-4">
            <div className="text-center">
              <div className="text-2xl font-bold text-blue-600">{complianceOverview.overallScore}</div>
              <div className="text-sm text-gray-600">Overall Score</div>
            </div>
          </Card>
          <Card className="p-4">
            <div className="text-center">
              <div className="text-2xl font-bold text-gray-900">{complianceOverview.totalCategories}</div>
              <div className="text-sm text-gray-600">Data Categories</div>
            </div>
          </Card>
          <Card className="p-4">
            <div className="text-center">
              <div className="text-2xl font-bold text-green-600">{complianceOverview.compliantCategories}</div>
              <div className="text-sm text-gray-600">Compliant</div>
            </div>
          </Card>
          <Card className="p-4">
            <div className="text-center">
              <div className="text-2xl font-bold text-yellow-600">{complianceOverview.warningCategories}</div>
              <div className="text-sm text-gray-600">Warnings</div>
            </div>
          </Card>
          <Card className="p-4">
            <div className="text-center">
              <div className="text-2xl font-bold text-purple-600">{complianceOverview.totalFlows}</div>
              <div className="text-sm text-gray-600">Data Flows</div>
            </div>
          </Card>
          <Card className="p-4">
            <div className="text-center">
              <div className="text-2xl font-bold text-indigo-600">{complianceOverview.totalSubjects}</div>
              <div className="text-sm text-gray-600">Data Subjects</div>
            </div>
          </Card>
          <Card className="p-4">
            <div className="text-center">
              <div className="text-2xl font-bold text-orange-600">{complianceOverview.activeRequests}</div>
              <div className="text-sm text-gray-600">Active Requests</div>
            </div>
          </Card>
          <Card className="p-4">
            <div className="text-center">
              <div className="text-2xl font-bold text-red-600">{complianceOverview.criticalIssues}</div>
              <div className="text-sm text-gray-600">Critical Issues</div>
            </div>
          </Card>
        </div>
      )}

      <div className="grid grid-cols-1 lg:grid-cols-4 gap-6">
        {/* Left Panel - Navigation & Filters */}
        <div className="lg:col-span-1 space-y-4">
          {/* Tab Navigation */}
          <Card className="p-4">
            <Tabs value={activeTab} onValueChange={setActiveTab}>
              <TabsList className="grid w-full grid-cols-1 gap-2">
                <TabsTrigger value="categories">
                  <Database className="w-4 h-4 mr-2" />
                  Data Categories
                </TabsTrigger>
                <TabsTrigger value="flows">
                  <Activity className="w-4 h-4 mr-2" />
                  Data Flows
                </TabsTrigger>
                <TabsTrigger value="subjects">
                  <Users className="w-4 h-4 mr-2" />
                  Data Subjects
                </TabsTrigger>
                <TabsTrigger value="compliance">
                  <CheckCircle className="w-4 h-4 mr-2" />
                  Compliance
                </TabsTrigger>
              </TabsList>
            </Tabs>
          </Card>

          {/* Filters */}
          <Card className="p-4">
            <div className="space-y-3">
              <div className="flex items-center space-x-2">
                <Search className="w-4 h-4 text-gray-500" />
                <Input
                  placeholder="Search..."
                  value={filters.search}
                  onChange={(e) => setFilters(prev => ({ ...prev, search: e.target.value }))}
                  className="flex-1"
                />
              </div>
              
              {activeTab === 'categories' && (
                <>
                  <Select
                    value={filters.classification}
                    onValueChange={(value) => setFilters(prev => ({ ...prev, classification: value }))}
                  >
                    <option value="">All Classifications</option>
                    <option value="public">Public</option>
                    <option value="internal">Internal</option>
                    <option value="confidential">Confidential</option>
                    <option value="restricted">Restricted</option>
                  </Select>

                  <Select
                    value={filters.framework}
                    onValueChange={(value) => setFilters(prev => ({ ...prev, framework: value }))}
                  >
                    <option value="">All Frameworks</option>
                    <option value="GDPR">GDPR</option>
                    <option value="UAE_DPA">UAE DPA</option>
                    <option value="PCI_DSS">PCI DSS</option>
                    <option value="ISO_27001">ISO 27001</option>
                  </Select>
                </>
              )}

              {activeTab === 'flows' && (
                <Select
                  value={filters.region}
                  onValueChange={(value) => setFilters(prev => ({ ...prev, region: value }))}
                >
                  <option value="">All Regions</option>
                  <option value="UAE">UAE</option>
                  <option value="EU">EU</option>
                  <option value="US">US</option>
                </Select>
              )}

              <Select
                value={filters.status}
                onValueChange={(value) => setFilters(prev => ({ ...prev, status: value }))}
              >
                <option value="">All Status</option>
                <option value="active">Active</option>
                <option value="inactive">Inactive</option>
                <option value="pending_review">Pending Review</option>
                <option value="deprecated">Deprecated</option>
              </Select>
            </div>
          </Card>

          {/* Framework Compliance */}
          {complianceOverview && (
            <Card className="p-4">
              <h3 className="font-medium text-gray-900 mb-3">Framework Compliance</h3>
              <div className="space-y-3">
                {complianceOverview.frameworks.map((framework: any) => (
                  <div key={framework.name} className="flex items-center justify-between">
                    <span className="text-sm font-medium">{framework.name}</span>
                    <div className="flex items-center space-x-2">
                      <Progress value={framework.score} className="w-16 h-2" />
                      <Badge className={getComplianceColor(framework.status)} size="sm">
                        {framework.score}%
                      </Badge>
                    </div>
                  </div>
                ))}
              </div>
            </Card>
          )}
        </div>

        {/* Center Panel - List */}
        <div className="lg:col-span-2">
          <Card className="p-4">
            <Tabs value={activeTab} onValueChange={setActiveTab}>
              <TabsContent value="categories" className="space-y-4">
                <div className="flex items-center justify-between">
                  <h3 className="font-medium text-gray-900">Data Categories ({filteredCategories.length})</h3>
                  {!readOnly && (
                    <Button variant="outline" size="sm" onClick={() => setShowPolicyDialog(true)}>
                      <Settings className="w-4 h-4 mr-2" />
                      New Category
                    </Button>
                  )}
                </div>
                <div className="space-y-3 max-h-96 overflow-y-auto">
                  {filteredCategories.map(category => (
                    <div
                      key={category.id}
                      className={`p-4 border rounded cursor-pointer transition-colors ${
                        selectedCategory?.id === category.id 
                          ? 'border-blue-500 bg-blue-50' 
                          : 'border-gray-200 hover:border-gray-300'
                      }`}
                      onClick={() => handleCategorySelect(category)}
                    >
                      <div className="flex items-start justify-between mb-2">
                        <h4 className="font-medium text-gray-900">{category.name}</h4>
                        <div className="flex items-center space-x-1">
                          <Badge className={getClassificationColor(category.classification)} size="sm">
                            {category.classification}
                          </Badge>
                          <Badge className={getPiiLevelColor(category.piiLevel)} size="sm">
                            PII: {category.piiLevel}
                          </Badge>
                        </div>
                      </div>
                      <p className="text-sm text-gray-600 mb-3">{category.description}</p>
                      <div className="grid grid-cols-2 gap-4 text-xs">
                        <div>
                          <span className="text-gray-500">Retention:</span>
                          <span className="font-medium ml-1">
                            {formatRetentionPeriod(category.retentionPolicy.defaultRetentionPeriod)}
                          </span>
                        </div>
                        <div>
                          <span className="text-gray-500">Regions:</span>
                          <span className="font-medium ml-1">
                            {category.residencyPolicy.allowedRegions.join(', ')}
                          </span>
                        </div>
                        <div>
                          <span className="text-gray-500">Frameworks:</span>
                          <span className="font-medium ml-1">{category.regulatoryFrameworks.length}</span>
                        </div>
                        <div>
                          <span className="text-gray-500">Next Review:</span>
                          <span className="font-medium ml-1">
                            {category.nextReviewDue.toLocaleDateString()}
                          </span>
                        </div>
                      </div>
                    </div>
                  ))}
                </div>
              </TabsContent>

              <TabsContent value="flows" className="space-y-4">
                <div className="flex items-center justify-between">
                  <h3 className="font-medium text-gray-900">Data Flows ({filteredFlows.length})</h3>
                  {!readOnly && (
                    <Button variant="outline" size="sm" onClick={() => setShowFlowDialog(true)}>
                      <Activity className="w-4 h-4 mr-2" />
                      New Flow
                    </Button>
                  )}
                </div>
                <div className="space-y-3 max-h-96 overflow-y-auto">
                  {filteredFlows.map(flow => (
                    <div
                      key={flow.id}
                      className={`p-4 border rounded cursor-pointer transition-colors ${
                        selectedFlow?.id === flow.id 
                          ? 'border-blue-500 bg-blue-50' 
                          : 'border-gray-200 hover:border-gray-300'
                      }`}
                      onClick={() => handleFlowSelect(flow)}
                    >
                      <div className="flex items-start justify-between mb-2">
                        <h4 className="font-medium text-gray-900">{flow.name}</h4>
                        <div className="flex items-center space-x-1">
                          <Badge className={getComplianceColor(flow.compliance.overall)} size="sm">
                            {flow.compliance.overall}
                          </Badge>
                          <Badge className={flow.status === 'active' ? 'bg-green-100 text-green-800' : 'bg-gray-100 text-gray-800'} size="sm">
                            {flow.status}
                          </Badge>
                        </div>
                      </div>
                      <div className="grid grid-cols-2 gap-4 text-xs mb-2">
                        <div>
                          <span className="text-gray-500">Source:</span>
                          <span className="font-medium ml-1">{flow.source.name} ({flow.source.region})</span>
                        </div>
                        <div>
                          <span className="text-gray-500">Destination:</span>
                          <span className="font-medium ml-1">{flow.destination.name} ({flow.destination.region})</span>
                        </div>
                        <div>
                          <span className="text-gray-500">Volume:</span>
                          <span className="font-medium ml-1">{flow.volume.sizeGB} GB/day</span>
                        </div>
                        <div>
                          <span className="text-gray-500">Last Transfer:</span>
                          <span className="font-medium ml-1">{flow.lastTransfer.toLocaleTimeString()}</span>
                        </div>
                      </div>
                      {flow.compliance.issues.length > 0 && (
                        <div className="flex items-center space-x-1">
                          <AlertTriangle className="w-3 h-3 text-yellow-500" />
                          <span className="text-xs text-yellow-600">{flow.compliance.issues.length} compliance issues</span>
                        </div>
                      )}
                    </div>
                  ))}
                </div>
              </TabsContent>

              <TabsContent value="subjects" className="space-y-4">
                <div className="flex items-center justify-between">
                  <h3 className="font-medium text-gray-900">Data Subjects ({dataSubjects.length})</h3>
                  <Button variant="outline" size="sm" onClick={() => setShowSubjectDialog(true)}>
                    <Users className="w-4 h-4 mr-2" />
                    View Requests
                  </Button>
                </div>
                <div className="space-y-3 max-h-96 overflow-y-auto">
                  {dataSubjects.map(subject => (
                    <div key={subject.id} className="p-4 border rounded">
                      <div className="flex items-start justify-between mb-2">
                        <h4 className="font-medium text-gray-900">{subject.id}</h4>
                        <div className="flex items-center space-x-1">
                          <Badge className="bg-blue-100 text-blue-800" size="sm">
                            {subject.type}
                          </Badge>
                          <Badge className="bg-gray-100 text-gray-800" size="sm">
                            {subject.region}
                          </Badge>
                        </div>
                      </div>
                      <div className="grid grid-cols-2 gap-4 text-xs">
                        <div>
                          <span className="text-gray-500">Consents:</span>
                          <span className="font-medium ml-1">{subject.consents.length}</span>
                        </div>
                        <div>
                          <span className="text-gray-500">Requests:</span>
                          <span className="font-medium ml-1">{subject.requests.length}</span>
                        </div>
                        <div>
                          <span className="text-gray-500">Categories:</span>
                          <span className="font-medium ml-1">{subject.dataCategories.length}</span>
                        </div>
                        <div>
                          <span className="text-gray-500">Last Activity:</span>
                          <span className="font-medium ml-1">{subject.lastActivity.toLocaleDateString()}</span>
                        </div>
                      </div>
                    </div>
                  ))}
                </div>
              </TabsContent>

              <TabsContent value="compliance" className="space-y-4">
                <h3 className="font-medium text-gray-900">Compliance Dashboard</h3>
                <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
                  <Card className="p-4">
                    <h4 className="font-medium text-gray-900 mb-3">Framework Scores</h4>
                    {complianceOverview?.frameworks.map((framework: any) => (
                      <div key={framework.name} className="flex items-center justify-between mb-2">
                        <span className="text-sm">{framework.name}</span>
                        <div className="flex items-center space-x-2">
                          <Progress value={framework.score} className="w-20 h-2" />
                          <span className="text-sm font-medium">{framework.score}%</span>
                        </div>
                      </div>
                    ))}
                  </Card>
                  <Card className="p-4">
                    <h4 className="font-medium text-gray-900 mb-3">Recent Issues</h4>
                    {dataFlows.flatMap(flow => flow.compliance.issues).slice(0, 3).map(issue => (
                      <div key={issue.id} className="mb-3 p-2 bg-yellow-50 border border-yellow-200 rounded">
                        <div className="flex items-center justify-between">
                          <span className="text-sm font-medium">{issue.category}</span>
                          <Badge className={
                            issue.severity === 'critical' ? 'bg-red-100 text-red-800' :
                            issue.severity === 'high' ? 'bg-orange-100 text-orange-800' :
                            'bg-yellow-100 text-yellow-800'
                          } size="sm">
                            {issue.severity}
                          </Badge>
                        </div>
                        <p className="text-xs text-gray-600 mt-1">{issue.description}</p>
                      </div>
                    ))}
                  </Card>
                </div>
              </TabsContent>
            </Tabs>
          </Card>
        </div>

        {/* Right Panel - Details */}
        <div className="lg:col-span-1">
          {selectedCategory ? (
            <Card className="p-4">
              <div className="flex items-center justify-between mb-4">
                <h3 className="font-medium text-gray-900">Category Details</h3>
                {!readOnly && (
                  <div className="flex items-center space-x-2">
                    <Button 
                      variant="outline" 
                      size="sm"
                      onClick={() => handleComplianceCheck(selectedCategory.id)}
                      disabled={loading}
                    >
                      {loading ? (
                        <Activity className="w-4 h-4 animate-spin" />
                      ) : (
                        <CheckCircle className="w-4 h-4" />
                      )}
                    </Button>
                  </div>
                )}
              </div>

              <div className="space-y-4">
                <div>
                  <h4 className="font-medium text-gray-900 mb-2">{selectedCategory.name}</h4>
                  <p className="text-sm text-gray-600 mb-3">{selectedCategory.description}</p>
                  
                  <div className="flex flex-wrap gap-2 mb-3">
                    <Badge className={getClassificationColor(selectedCategory.classification)}>
                      {selectedCategory.classification}
                    </Badge>
                    <Badge className={getPiiLevelColor(selectedCategory.piiLevel)}>
                      PII: {selectedCategory.piiLevel}
                    </Badge>
                  </div>
                </div>

                <div>
                  <h5 className="font-medium text-gray-900 mb-2">Retention Policy</h5>
                  <div className="text-sm space-y-1">
                    <div className="flex justify-between">
                      <span className="text-gray-600">Default Period:</span>
                      <span className="font-medium">{formatRetentionPeriod(selectedCategory.retentionPolicy.defaultRetentionPeriod)}</span>
                    </div>
                    <div className="flex justify-between">
                      <span className="text-gray-600">Auto Delete:</span>
                      <span className="font-medium">{selectedCategory.retentionPolicy.autoDeleteEnabled ? 'Yes' : 'No'}</span>
                    </div>
                    <div className="flex justify-between">
                      <span className="text-gray-600">Deletion Method:</span>
                      <span className="font-medium">{selectedCategory.retentionPolicy.deletionMethod}</span>
                    </div>
                  </div>
                </div>

                <div>
                  <h5 className="font-medium text-gray-900 mb-2">Residency Policy</h5>
                  <div className="text-sm space-y-1">
                    <div>
                      <span className="text-gray-600">Allowed Regions:</span>
                      <div className="flex flex-wrap gap-1 mt-1">
                        {selectedCategory.residencyPolicy.allowedRegions.map(region => (
                          <Badge key={region} variant="outline" size="sm">{region}</Badge>
                        ))}
                      </div>
                    </div>
                    <div className="flex justify-between">
                      <span className="text-gray-600">Localization:</span>
                      <span className="font-medium">{selectedCategory.residencyPolicy.dataLocalizationLevel}</span>
                    </div>
                  </div>
                </div>

                <div>
                  <h5 className="font-medium text-gray-900 mb-2">Regulatory Frameworks</h5>
                  <div className="flex flex-wrap gap-1">
                    {selectedCategory.regulatoryFrameworks.map(framework => (
                      <Badge key={framework} className="bg-purple-100 text-purple-800" size="sm">
                        {framework}
                      </Badge>
                    ))}
                  </div>
                </div>

                <div>
                  <h5 className="font-medium text-gray-900 mb-2">Encryption</h5>
                  <div className="text-sm space-y-1">
                    <div className="flex justify-between">
                      <span className="text-gray-600">At Rest:</span>
                      <span className="font-medium">{selectedCategory.encryptionRequirements.atRest ? 'Required' : 'Not Required'}</span>
                    </div>
                    <div className="flex justify-between">
                      <span className="text-gray-600">In Transit:</span>
                      <span className="font-medium">{selectedCategory.encryptionRequirements.inTransit ? 'Required' : 'Not Required'}</span>
                    </div>
                    <div className="flex justify-between">
                      <span className="text-gray-600">Algorithm:</span>
                      <span className="font-medium">{selectedCategory.encryptionRequirements.algorithm}</span>
                    </div>
                  </div>
                </div>

                <div>
                  <h5 className="font-medium text-gray-900 mb-2">Review Schedule</h5>
                  <div className="text-sm space-y-1">
                    <div className="flex justify-between">
                      <span className="text-gray-600">Last Reviewed:</span>
                      <span className="font-medium">
                        {selectedCategory.lastReviewedAt?.toLocaleDateString() || 'Never'}
                      </span>
                    </div>
                    <div className="flex justify-between">
                      <span className="text-gray-600">Next Review:</span>
                      <span className="font-medium">{selectedCategory.nextReviewDue.toLocaleDateString()}</span>
                    </div>
                  </div>
                </div>
              </div>
            </Card>
          ) : selectedFlow ? (
            <Card className="p-4">
              <div className="flex items-center justify-between mb-4">
                <h3 className="font-medium text-gray-900">Flow Details</h3>
              </div>

              <div className="space-y-4">
                <div>
                  <h4 className="font-medium text-gray-900 mb-2">{selectedFlow.name}</h4>
                  <div className="flex items-center space-x-2 mb-3">
                    <Badge className={getComplianceColor(selectedFlow.compliance.overall)}>
                      {selectedFlow.compliance.overall}
                    </Badge>
                    <Badge className={selectedFlow.status === 'active' ? 'bg-green-100 text-green-800' : 'bg-gray-100 text-gray-800'}>
                      {selectedFlow.status}
                    </Badge>
                  </div>
                </div>

                <div>
                  <h5 className="font-medium text-gray-900 mb-2">Data Path</h5>
                  <div className="space-y-2 text-sm">
                    <div className="p-2 bg-blue-50 rounded">
                      <div className="font-medium">{selectedFlow.source.name}</div>
                      <div className="text-gray-600">{selectedFlow.source.region} • {selectedFlow.source.type}</div>
                    </div>
                    <div className="text-center text-gray-400">↓</div>
                    <div className="p-2 bg-green-50 rounded">
                      <div className="font-medium">{selectedFlow.destination.name}</div>
                      <div className="text-gray-600">{selectedFlow.destination.region} • {selectedFlow.destination.type}</div>
                    </div>
                  </div>
                </div>

                <div>
                  <h5 className="font-medium text-gray-900 mb-2">Volume & Performance</h5>
                  <div className="text-sm space-y-1">
                    <div className="flex justify-between">
                      <span className="text-gray-600">Records/Day:</span>
                      <span className="font-medium">{selectedFlow.volume.recordsPerDay.toLocaleString()}</span>
                    </div>
                    <div className="flex justify-between">
                      <span className="text-gray-600">Size/Day:</span>
                      <span className="font-medium">{selectedFlow.volume.sizeGB} GB</span>
                    </div>
                    <div className="flex justify-between">
                      <span className="text-gray-600">Peak Throughput:</span>
                      <span className="font-medium">{selectedFlow.volume.peakThroughput} rec/sec</span>
                    </div>
                  </div>
                </div>

                <div>
                  <h5 className="font-medium text-gray-900 mb-2">Security</h5>
                  <div className="text-sm space-y-1">
                    <div className="flex justify-between">
                      <span className="text-gray-600">Encryption:</span>
                      <span className="font-medium">{selectedFlow.encryption ? 'Enabled' : 'Disabled'}</span>
                    </div>
                    <div className="flex justify-between">
                      <span className="text-gray-600">Monitoring:</span>
                      <span className="font-medium">{selectedFlow.monitoring ? 'Enabled' : 'Disabled'}</span>
                    </div>
                  </div>
                </div>

                {selectedFlow.compliance.issues.length > 0 && (
                  <div>
                    <h5 className="font-medium text-gray-900 mb-2">Compliance Issues</h5>
                    <div className="space-y-2">
                      {selectedFlow.compliance.issues.map(issue => (
                        <div key={issue.id} className="p-2 bg-yellow-50 border border-yellow-200 rounded text-sm">
                          <div className="flex items-center justify-between">
                            <span className="font-medium">{issue.category}</span>
                            <Badge className={
                              issue.severity === 'critical' ? 'bg-red-100 text-red-800' :
                              issue.severity === 'high' ? 'bg-orange-100 text-orange-800' :
                              'bg-yellow-100 text-yellow-800'
                            } size="sm">
                              {issue.severity}
                            </Badge>
                          </div>
                          <p className="text-yellow-700 mt-1">{issue.description}</p>
                          <p className="text-yellow-600 mt-1 text-xs">Recommendation: {issue.recommendation}</p>
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
                <Database className="w-16 h-16 text-gray-400 mx-auto mb-4" />
                <h3 className="text-lg font-medium text-gray-900 mb-2">No Selection</h3>
                <p className="text-gray-600">
                  Select a data category or flow to view details
                </p>
              </div>
            </Card>
          )}
        </div>
      </div>
    </div>
  )
}

export default DataResidencyRetentionControls
