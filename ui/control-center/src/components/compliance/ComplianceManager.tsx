import React, { useState, useCallback, useEffect, useMemo } from 'react'
import { 
  Shield, FileText, Calendar, AlertTriangle, CheckCircle, Clock, Download,
  ExternalLink, Eye, Edit, Trash2, Plus, Search, Filter, RefreshCw,
  Award, Building, User, MapPin, Phone, Mail, Globe, Gavel, Lock
} from 'lucide-react'
import { Button } from '../ui/Button'
import { Badge } from '../ui/Badge'
import { Card } from '../ui/Card'
import { Dialog, DialogContent, DialogHeader, DialogTitle } from '../ui/Dialog'
import { Input } from '../ui/Input'
import { Textarea } from '../ui/Textarea'
import { Select } from '../ui/Select'
import { Alert, AlertDescription } from '../ui/Alert'
import { Progress } from '../ui/Progress'
import { Tabs, TabsContent, TabsList, TabsTrigger } from '../ui/Tabs'

// Types
interface ComplianceDocument {
  id: string
  type: 'permit' | 'license' | 'insurance' | 'inspection' | 'certification' | 'policy' | 'audit'
  title: string
  description: string
  documentNumber: string
  issuedBy: string
  issuedDate: Date
  expiryDate: Date
  status: 'valid' | 'expiring' | 'expired' | 'suspended' | 'pending'
  vehicleId?: string
  vehicleName?: string
  jurisdiction: string
  category: string
  attachments: ComplianceAttachment[]
  requirements: ComplianceRequirement[]
  renewalProcess?: RenewalProcess
  lastAuditDate?: Date
  nextAuditDate?: Date
  complianceScore?: number
  riskLevel: 'low' | 'medium' | 'high' | 'critical'
  tags: string[]
  notes: string[]
  policyVersion?: string
  evidencePackage?: EvidencePackage
}

interface ComplianceAttachment {
  id: string
  filename: string
  type: 'certificate' | 'report' | 'photo' | 'document' | 'evidence'
  url: string
  uploadedBy: string
  uploadedAt: Date
  hash: string
  signed: boolean
  description?: string
}

interface ComplianceRequirement {
  id: string
  requirement: string
  status: 'met' | 'partial' | 'not_met' | 'not_applicable'
  evidence?: string[]
  lastChecked: Date
  nextCheck: Date
  responsible: string
}

interface RenewalProcess {
  id: string
  startDate: Date
  submissionDeadline: Date
  renewalFee: number
  requiredDocuments: string[]
  status: 'not_started' | 'in_progress' | 'submitted' | 'approved' | 'rejected'
  submittedBy?: string
  submittedDate?: Date
  approvedBy?: string
  approvedDate?: Date
  rejectionReason?: string
}

interface EvidencePackage {
  id: string
  createdAt: Date
  createdBy: string
  version: string
  digitalSignature: string
  contents: {
    documents: string[]
    telemetryData: string[]
    incidentReports: string[]
    maintenanceRecords: string[]
  }
  exportFormat: 'pdf' | 'json' | 'xml'
  encryptionLevel: 'none' | 'standard' | 'high'
}

interface ComplianceManagerProps {
  vehicleId?: string
  vehicleName?: string
  showPolicyVersions?: boolean
  onComplianceUpdated?: (document: ComplianceDocument) => void
  className?: string
}

const ComplianceManager: React.FC<ComplianceManagerProps> = ({
  vehicleId,
  vehicleName,
  showPolicyVersions = true,
  onComplianceUpdated,
  className = ''
}) => {
  // State
  const [documents, setDocuments] = useState<ComplianceDocument[]>([])
  const [showAddDialog, setShowAddDialog] = useState(false)
  const [showDetailDialog, setShowDetailDialog] = useState(false)
  const [selectedDocument, setSelectedDocument] = useState<ComplianceDocument | null>(null)
  const [activeTab, setActiveTab] = useState('overview')
  const [filters, setFilters] = useState({
    type: '',
    status: '',
    riskLevel: '',
    search: ''
  })

  // Mock data
  const mockDocuments: ComplianceDocument[] = [
    {
      id: 'comp-001',
      type: 'permit',
      title: 'Autonomous Vehicle Operating Permit',
      description: 'Permit for autonomous vehicle operation in Dubai Municipality',
      documentNumber: 'AV-DXB-2024-001',
      issuedBy: 'Dubai Municipality',
      issuedDate: new Date('2024-01-15'),
      expiryDate: new Date('2024-12-31'),
      status: 'valid',
      vehicleId: 'v1',
      vehicleName: 'Atlas-001',
      jurisdiction: 'Dubai, UAE',
      category: 'Operating Permit',
      attachments: [
        {
          id: 'att-001',
          filename: 'av_permit_atlas_001.pdf',
          type: 'certificate',
          url: '/compliance/av_permit_atlas_001.pdf',
          uploadedBy: 'System',
          uploadedAt: new Date('2024-01-15'),
          hash: 'sha256:abc123def456...',
          signed: true,
          description: 'Official AV operating permit'
        }
      ],
      requirements: [
        {
          id: 'req-001',
          requirement: 'Valid insurance coverage minimum $5M',
          status: 'met',
          evidence: ['insurance_policy_2024.pdf'],
          lastChecked: new Date(),
          nextCheck: new Date(Date.now() + 30 * 24 * 60 * 60 * 1000),
          responsible: 'Legal Team'
        },
        {
          id: 'req-002',
          requirement: 'Monthly safety inspection reports',
          status: 'met',
          evidence: ['safety_inspection_11_2024.pdf'],
          lastChecked: new Date(),
          nextCheck: new Date(Date.now() + 7 * 24 * 60 * 60 * 1000),
          responsible: 'Safety Team'
        },
        {
          id: 'req-003',
          requirement: 'Operator certification Level 4',
          status: 'met',
          evidence: ['operator_cert_ahmed_2024.pdf'],
          lastChecked: new Date(),
          nextCheck: new Date(Date.now() + 90 * 24 * 60 * 60 * 1000),
          responsible: 'HR Team'
        }
      ],
      renewalProcess: {
        id: 'ren-001',
        startDate: new Date('2024-10-01'),
        submissionDeadline: new Date('2024-11-30'),
        renewalFee: 5000,
        requiredDocuments: ['insurance_policy', 'safety_reports', 'operator_certs'],
        status: 'in_progress',
        submittedBy: 'Legal Team',
        submittedDate: new Date('2024-11-15')
      },
      lastAuditDate: new Date('2024-10-15'),
      nextAuditDate: new Date('2025-01-15'),
      complianceScore: 94,
      riskLevel: 'low',
      tags: ['operating-permit', 'dubai-municipality', 'av-specific'],
      notes: [
        'Renewal application submitted on time',
        'All requirements currently met',
        'Next audit scheduled for Q1 2025'
      ],
      policyVersion: 'v2.1.0',
      evidencePackage: {
        id: 'evp-001',
        createdAt: new Date(),
        createdBy: 'Compliance System',
        version: '1.0',
        digitalSignature: 'sig_abc123def456...',
        contents: {
          documents: ['permit', 'insurance', 'inspections'],
          telemetryData: ['safety_events', 'operational_logs'],
          incidentReports: [],
          maintenanceRecords: ['monthly_inspections']
        },
        exportFormat: 'pdf',
        encryptionLevel: 'high'
      }
    },
    {
      id: 'comp-002',
      type: 'insurance',
      title: 'Commercial Vehicle Insurance',
      description: 'Comprehensive commercial vehicle insurance coverage',
      documentNumber: 'INS-CV-2024-789',
      issuedBy: 'AXA Insurance UAE',
      issuedDate: new Date('2024-02-01'),
      expiryDate: new Date('2025-01-31'),
      status: 'valid',
      vehicleId: 'v1',
      vehicleName: 'Atlas-001',
      jurisdiction: 'UAE',
      category: 'Insurance Coverage',
      attachments: [
        {
          id: 'att-002',
          filename: 'insurance_policy_2024.pdf',
          type: 'certificate',
          url: '/compliance/insurance_policy_2024.pdf',
          uploadedBy: 'Insurance Team',
          uploadedAt: new Date('2024-02-01'),
          hash: 'sha256:def456ghi789...',
          signed: true,
          description: 'Commercial vehicle insurance policy'
        }
      ],
      requirements: [
        {
          id: 'req-004',
          requirement: 'Minimum coverage $5M liability',
          status: 'met',
          evidence: ['policy_schedule.pdf'],
          lastChecked: new Date(),
          nextCheck: new Date(Date.now() + 30 * 24 * 60 * 60 * 1000),
          responsible: 'Finance Team'
        },
        {
          id: 'req-005',
          requirement: 'AV-specific coverage endorsement',
          status: 'met',
          evidence: ['av_endorsement.pdf'],
          lastChecked: new Date(),
          nextCheck: new Date(Date.now() + 90 * 24 * 60 * 60 * 1000),
          responsible: 'Legal Team'
        }
      ],
      complianceScore: 98,
      riskLevel: 'low',
      tags: ['insurance', 'liability', 'av-coverage'],
      notes: [
        'Policy includes AV-specific coverage',
        'Premium paid annually',
        'Claims history: 0 incidents'
      ],
      policyVersion: 'v2.1.0'
    },
    {
      id: 'comp-003',
      type: 'inspection',
      title: 'Safety System Inspection',
      description: 'Quarterly safety system inspection certificate',
      documentNumber: 'SI-Q4-2024-001',
      issuedBy: 'Authorized Safety Inspector',
      issuedDate: new Date('2024-11-01'),
      expiryDate: new Date('2025-01-31'),
      status: 'expiring',
      vehicleId: 'v1',
      vehicleName: 'Atlas-001',
      jurisdiction: 'UAE',
      category: 'Safety Inspection',
      attachments: [
        {
          id: 'att-003',
          filename: 'safety_inspection_q4_2024.pdf',
          type: 'report',
          url: '/compliance/safety_inspection_q4_2024.pdf',
          uploadedBy: 'Safety Inspector',
          uploadedAt: new Date('2024-11-01'),
          hash: 'sha256:ghi789jkl012...',
          signed: true,
          description: 'Q4 2024 safety inspection report'
        }
      ],
      requirements: [
        {
          id: 'req-006',
          requirement: 'All safety systems functional',
          status: 'met',
          evidence: ['system_test_results.pdf'],
          lastChecked: new Date('2024-11-01'),
          nextCheck: new Date('2025-02-01'),
          responsible: 'Safety Team'
        },
        {
          id: 'req-007',
          requirement: 'Emergency stop system tested',
          status: 'met',
          evidence: ['emergency_stop_test.pdf'],
          lastChecked: new Date('2024-11-01'),
          nextCheck: new Date('2025-02-01'),
          responsible: 'Safety Team'
        }
      ],
      complianceScore: 96,
      riskLevel: 'medium',
      tags: ['safety-inspection', 'quarterly', 'systems-check'],
      notes: [
        'All systems passed inspection',
        'Next inspection due February 2025',
        'Minor recommendations for sensor calibration'
      ],
      policyVersion: 'v2.1.0'
    }
  ]

  // Initialize data
  useEffect(() => {
    setDocuments(mockDocuments)
  }, [])

  // Filtered documents
  const filteredDocuments = useMemo(() => {
    return documents
      .filter(doc => !vehicleId || doc.vehicleId === vehicleId)
      .filter(doc => !filters.type || doc.type === filters.type)
      .filter(doc => !filters.status || doc.status === filters.status)
      .filter(doc => !filters.riskLevel || doc.riskLevel === filters.riskLevel)
      .filter(doc => {
        if (!filters.search) return true
        const searchTerm = filters.search.toLowerCase()
        return (
          doc.title.toLowerCase().includes(searchTerm) ||
          doc.description.toLowerCase().includes(searchTerm) ||
          doc.documentNumber.toLowerCase().includes(searchTerm) ||
          doc.issuedBy.toLowerCase().includes(searchTerm)
        )
      })
      .sort((a, b) => {
        // Sort by status priority and expiry date
        const statusPriority = { expired: 4, expiring: 3, suspended: 2, valid: 1, pending: 0 }
        const aStatus = statusPriority[a.status]
        const bStatus = statusPriority[b.status]
        
        if (aStatus !== bStatus) {
          return bStatus - aStatus
        }
        
        return a.expiryDate.getTime() - b.expiryDate.getTime()
      })
  }, [documents, vehicleId, filters])

  // Statistics
  const stats = useMemo(() => {
    const docs = vehicleId ? 
      documents.filter(d => d.vehicleId === vehicleId) : 
      documents

    const totalDocs = docs.length
    const validDocs = docs.filter(d => d.status === 'valid').length
    const expiringDocs = docs.filter(d => d.status === 'expiring').length
    const expiredDocs = docs.filter(d => d.status === 'expired').length
    const highRiskDocs = docs.filter(d => d.riskLevel === 'high' || d.riskLevel === 'critical').length
    
    const avgComplianceScore = docs.length > 0 ? 
      docs.reduce((sum, d) => sum + (d.complianceScore || 0), 0) / docs.length : 0

    const upcomingRenewals = docs.filter(d => {
      const daysUntilExpiry = (d.expiryDate.getTime() - new Date().getTime()) / (24 * 60 * 60 * 1000)
      return daysUntilExpiry <= 30 && daysUntilExpiry > 0
    }).length

    return {
      totalDocs,
      validDocs,
      expiringDocs,
      expiredDocs,
      highRiskDocs,
      avgComplianceScore,
      upcomingRenewals
    }
  }, [documents, vehicleId])

  // Handlers
  const handleViewDetails = useCallback((document: ComplianceDocument) => {
    setSelectedDocument(document)
    setShowDetailDialog(true)
  }, [])

  const handleExportEvidence = useCallback((document: ComplianceDocument) => {
    if (!document.evidencePackage) return

    // Create evidence export package
    const exportData = {
      document: {
        id: document.id,
        type: document.type,
        title: document.title,
        documentNumber: document.documentNumber,
        issuedBy: document.issuedBy,
        issuedDate: document.issuedDate,
        expiryDate: document.expiryDate,
        status: document.status,
        policyVersion: document.policyVersion
      },
      evidencePackage: document.evidencePackage,
      requirements: document.requirements.map(req => ({
        requirement: req.requirement,
        status: req.status,
        evidence: req.evidence,
        lastChecked: req.lastChecked
      })),
      attachments: document.attachments.map(att => ({
        filename: att.filename,
        type: att.type,
        hash: att.hash,
        signed: att.signed,
        uploadedAt: att.uploadedAt
      })),
      exportedAt: new Date(),
      exportedBy: 'Current User',
      digitalSignature: document.evidencePackage.digitalSignature
    }

    // In real implementation, this would generate a cryptographically signed export
    const blob = new Blob([JSON.stringify(exportData, null, 2)], { type: 'application/json' })
    const url = URL.createObjectURL(blob)
    const a = document.createElement('a')
    a.href = url
    a.download = `compliance_evidence_${document.id}_${new Date().toISOString().split('T')[0]}.json`
    document.body.appendChild(a)
    a.click()
    document.body.removeChild(a)
    URL.revokeObjectURL(url)

    console.info('Compliance evidence exported', {
      documentId: document.id,
      documentType: document.type,
      policyVersion: document.policyVersion,
      timestamp: new Date().toISOString()
    })
  }, [])

  const getStatusColor = (status: string) => {
    switch (status) {
      case 'valid': return 'bg-green-100 text-green-800'
      case 'expiring': return 'bg-yellow-100 text-yellow-800'
      case 'expired': return 'bg-red-100 text-red-800'
      case 'suspended': return 'bg-red-100 text-red-800'
      case 'pending': return 'bg-blue-100 text-blue-800'
      default: return 'bg-gray-100 text-gray-800'
    }
  }

  const getRiskColor = (riskLevel: string) => {
    switch (riskLevel) {
      case 'critical': return 'bg-red-100 text-red-800'
      case 'high': return 'bg-orange-100 text-orange-800'
      case 'medium': return 'bg-yellow-100 text-yellow-800'
      case 'low': return 'bg-green-100 text-green-800'
      default: return 'bg-gray-100 text-gray-800'
    }
  }

  const getRequirementStatusIcon = (status: string) => {
    switch (status) {
      case 'met': return <CheckCircle className="w-4 h-4 text-green-600" />
      case 'partial': return <Clock className="w-4 h-4 text-yellow-600" />
      case 'not_met': return <AlertTriangle className="w-4 h-4 text-red-600" />
      case 'not_applicable': return <div className="w-4 h-4 bg-gray-300 rounded-full" />
      default: return <Clock className="w-4 h-4 text-gray-600" />
    }
  }

  const getDaysUntilExpiry = (expiryDate: Date) => {
    return Math.ceil((expiryDate.getTime() - new Date().getTime()) / (24 * 60 * 60 * 1000))
  }

  return (
    <div className={`space-y-6 ${className}`}>
      {/* Header */}
      <div className="flex items-center justify-between">
        <div className="flex items-center space-x-3">
          <Shield className="w-6 h-6 text-blue-600" />
          <h2 className="text-2xl font-semibold text-gray-900">
            {vehicleId ? `Compliance - ${vehicleName}` : 'Fleet Compliance'}
          </h2>
          {showPolicyVersions && (
            <Badge variant="outline" className="ml-2">
              Policy v2.1.0
            </Badge>
          )}
        </div>
        <Button onClick={() => setShowAddDialog(true)}>
          <Plus className="w-4 h-4 mr-2" />
          Add Document
        </Button>
      </div>

      {/* Statistics */}
      <div className="grid grid-cols-2 md:grid-cols-4 lg:grid-cols-7 gap-4">
        <Card className="p-4">
          <div className="text-center">
            <div className="text-2xl font-bold text-gray-900">{stats.totalDocs}</div>
            <div className="text-sm text-gray-600">Total Documents</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className="text-2xl font-bold text-green-600">{stats.validDocs}</div>
            <div className="text-sm text-gray-600">Valid</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className="text-2xl font-bold text-yellow-600">{stats.expiringDocs}</div>
            <div className="text-sm text-gray-600">Expiring</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className={`text-2xl font-bold ${stats.expiredDocs > 0 ? 'text-red-600' : 'text-green-600'}`}>
              {stats.expiredDocs}
            </div>
            <div className="text-sm text-gray-600">Expired</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className={`text-2xl font-bold ${stats.highRiskDocs > 0 ? 'text-red-600' : 'text-green-600'}`}>
              {stats.highRiskDocs}
            </div>
            <div className="text-sm text-gray-600">High Risk</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className="text-2xl font-bold text-blue-600">{Math.round(stats.avgComplianceScore)}%</div>
            <div className="text-sm text-gray-600">Avg Score</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className={`text-2xl font-bold ${stats.upcomingRenewals > 0 ? 'text-yellow-600' : 'text-green-600'}`}>
              {stats.upcomingRenewals}
            </div>
            <div className="text-sm text-gray-600">Due Soon</div>
          </div>
        </Card>
      </div>

      {/* Filters */}
      <Card className="p-4">
        <div className="flex flex-wrap items-center gap-4">
          <div className="flex items-center space-x-2">
            <Search className="w-4 h-4 text-gray-500" />
            <Input
              placeholder="Search compliance documents..."
              value={filters.search}
              onChange={(e) => setFilters(prev => ({ ...prev, search: e.target.value }))}
              className="w-64"
            />
          </div>
          
          <Select
            value={filters.type}
            onValueChange={(value) => setFilters(prev => ({ ...prev, type: value }))}
          >
            <option value="">All Types</option>
            <option value="permit">Permit</option>
            <option value="license">License</option>
            <option value="insurance">Insurance</option>
            <option value="inspection">Inspection</option>
            <option value="certification">Certification</option>
            <option value="policy">Policy</option>
            <option value="audit">Audit</option>
          </Select>

          <Select
            value={filters.status}
            onValueChange={(value) => setFilters(prev => ({ ...prev, status: value }))}
          >
            <option value="">All Status</option>
            <option value="valid">Valid</option>
            <option value="expiring">Expiring</option>
            <option value="expired">Expired</option>
            <option value="suspended">Suspended</option>
            <option value="pending">Pending</option>
          </Select>

          <Select
            value={filters.riskLevel}
            onValueChange={(value) => setFilters(prev => ({ ...prev, riskLevel: value }))}
          >
            <option value="">All Risk Levels</option>
            <option value="low">Low</option>
            <option value="medium">Medium</option>
            <option value="high">High</option>
            <option value="critical">Critical</option>
          </Select>

          <Button variant="outline" size="sm">
            <RefreshCw className="w-4 h-4 mr-2" />
            Refresh
          </Button>
        </div>
      </Card>

      {/* Documents List */}
      <div className="space-y-4">
        {filteredDocuments.map(doc => {
          const daysUntilExpiry = getDaysUntilExpiry(doc.expiryDate)
          
          return (
            <Card key={doc.id} className="p-6 hover:shadow-md transition-shadow">
              <div className="flex items-start justify-between">
                <div className="flex-1">
                  <div className="flex items-center space-x-3 mb-3">
                    <h3 className="text-lg font-medium text-gray-900">{doc.title}</h3>
                    <Badge className={getStatusColor(doc.status)}>
                      {doc.status}
                    </Badge>
                    <Badge className={getRiskColor(doc.riskLevel)}>
                      {doc.riskLevel} risk
                    </Badge>
                    <Badge variant="outline">
                      {doc.type}
                    </Badge>
                    {showPolicyVersions && doc.policyVersion && (
                      <Badge variant="outline" className="bg-blue-50 text-blue-700">
                        {doc.policyVersion}
                      </Badge>
                    )}
                  </div>

                  <p className="text-gray-600 mb-4">{doc.description}</p>

                  <div className="grid grid-cols-2 md:grid-cols-4 gap-4 text-sm mb-4">
                    <div>
                      <div className="flex items-center space-x-1 text-gray-500 mb-1">
                        <FileText className="w-4 h-4" />
                        <span>Document Number</span>
                      </div>
                      <div className="font-medium">{doc.documentNumber}</div>
                    </div>

                    <div>
                      <div className="flex items-center space-x-1 text-gray-500 mb-1">
                        <Building className="w-4 h-4" />
                        <span>Issued By</span>
                      </div>
                      <div className="font-medium">{doc.issuedBy}</div>
                    </div>

                    <div>
                      <div className="flex items-center space-x-1 text-gray-500 mb-1">
                        <Calendar className="w-4 h-4" />
                        <span>Expires</span>
                      </div>
                      <div className={`font-medium ${
                        daysUntilExpiry <= 0 ? 'text-red-600' :
                        daysUntilExpiry <= 30 ? 'text-yellow-600' : 'text-green-600'
                      }`}>
                        {doc.expiryDate.toLocaleDateString()}
                        {daysUntilExpiry > 0 && (
                          <span className="text-xs ml-1">({daysUntilExpiry}d)</span>
                        )}
                      </div>
                    </div>

                    <div>
                      <div className="flex items-center space-x-1 text-gray-500 mb-1">
                        <MapPin className="w-4 h-4" />
                        <span>Jurisdiction</span>
                      </div>
                      <div className="font-medium">{doc.jurisdiction}</div>
                    </div>
                  </div>

                  {/* Compliance Score */}
                  {doc.complianceScore && (
                    <div className="mb-4">
                      <div className="flex items-center justify-between text-sm mb-1">
                        <span className="text-gray-600">Compliance Score</span>
                        <span className="font-medium">{doc.complianceScore}%</span>
                      </div>
                      <Progress value={doc.complianceScore} className="h-2" />
                    </div>
                  )}

                  {/* Requirements Summary */}
                  <div className="mb-4">
                    <div className="text-sm font-medium text-gray-900 mb-2">Requirements Status</div>
                    <div className="flex flex-wrap gap-2">
                      {doc.requirements.slice(0, 3).map(req => (
                        <div key={req.id} className="flex items-center space-x-1 text-xs">
                          {getRequirementStatusIcon(req.status)}
                          <span className="text-gray-600">{req.requirement}</span>
                        </div>
                      ))}
                      {doc.requirements.length > 3 && (
                        <span className="text-xs text-gray-500">
                          +{doc.requirements.length - 3} more
                        </span>
                      )}
                    </div>
                  </div>

                  {/* Renewal Process */}
                  {doc.renewalProcess && (
                    <div className="mb-4 p-3 bg-blue-50 rounded border border-blue-200">
                      <div className="flex items-center justify-between">
                        <div className="text-sm">
                          <div className="font-medium text-blue-900">Renewal Process</div>
                          <div className="text-blue-700">
                            Status: {doc.renewalProcess.status.replace('_', ' ')} • 
                            Deadline: {doc.renewalProcess.submissionDeadline.toLocaleDateString()}
                          </div>
                        </div>
                        <Badge className="bg-blue-100 text-blue-800">
                          {doc.renewalProcess.status.replace('_', ' ')}
                        </Badge>
                      </div>
                    </div>
                  )}

                  {/* Tags */}
                  {doc.tags.length > 0 && (
                    <div className="flex items-center space-x-2">
                      <span className="text-sm text-gray-500">Tags:</span>
                      <div className="flex flex-wrap gap-1">
                        {doc.tags.map(tag => (
                          <Badge key={tag} variant="outline" size="sm">
                            {tag}
                          </Badge>
                        ))}
                      </div>
                    </div>
                  )}
                </div>

                <div className="flex items-center space-x-2">
                  <Button
                    variant="outline"
                    size="sm"
                    onClick={() => handleViewDetails(doc)}
                  >
                    <Eye className="w-4 h-4" />
                  </Button>
                  
                  <Button variant="outline" size="sm">
                    <Edit className="w-4 h-4" />
                  </Button>

                  {doc.evidencePackage && (
                    <Button
                      variant="outline"
                      size="sm"
                      onClick={() => handleExportEvidence(doc)}
                    >
                      <Download className="w-4 h-4" />
                    </Button>
                  )}
                </div>
              </div>
            </Card>
          )
        })}

        {filteredDocuments.length === 0 && (
          <div className="text-center py-12">
            <Shield className="w-16 h-16 text-gray-400 mx-auto mb-4" />
            <h3 className="text-lg font-medium text-gray-900 mb-2">No Compliance Documents</h3>
            <p className="text-gray-600">
              {filters.search || filters.type || filters.status || filters.riskLevel
                ? 'No documents match your current filters'
                : 'No compliance documents found'
              }
            </p>
          </div>
        )}
      </div>

      {/* Document Detail Dialog */}
      <Dialog open={showDetailDialog} onOpenChange={setShowDetailDialog}>
        <DialogContent className="max-w-6xl max-h-[90vh] overflow-y-auto">
          <DialogHeader>
            <DialogTitle>Compliance Document Details</DialogTitle>
          </DialogHeader>

          {selectedDocument && (
            <Tabs value={activeTab} onValueChange={setActiveTab}>
              <TabsList>
                <TabsTrigger value="overview">Overview</TabsTrigger>
                <TabsTrigger value="requirements">Requirements</TabsTrigger>
                <TabsTrigger value="attachments">Attachments</TabsTrigger>
                <TabsTrigger value="renewal">Renewal</TabsTrigger>
                <TabsTrigger value="evidence">Evidence</TabsTrigger>
              </TabsList>

              <TabsContent value="overview" className="space-y-4">
                <div className="grid grid-cols-2 gap-6">
                  <Card className="p-4">
                    <h3 className="text-lg font-medium text-gray-900 mb-3">Document Information</h3>
                    <div className="space-y-2 text-sm">
                      <div className="flex justify-between">
                        <span className="text-gray-600">Title:</span>
                        <span className="font-medium">{selectedDocument.title}</span>
                      </div>
                      <div className="flex justify-between">
                        <span className="text-gray-600">Document Number:</span>
                        <span className="font-medium">{selectedDocument.documentNumber}</span>
                      </div>
                      <div className="flex justify-between">
                        <span className="text-gray-600">Type:</span>
                        <Badge variant="outline">{selectedDocument.type}</Badge>
                      </div>
                      <div className="flex justify-between">
                        <span className="text-gray-600">Category:</span>
                        <span className="font-medium">{selectedDocument.category}</span>
                      </div>
                      <div className="flex justify-between">
                        <span className="text-gray-600">Status:</span>
                        <Badge className={getStatusColor(selectedDocument.status)}>
                          {selectedDocument.status}
                        </Badge>
                      </div>
                      <div className="flex justify-between">
                        <span className="text-gray-600">Risk Level:</span>
                        <Badge className={getRiskColor(selectedDocument.riskLevel)}>
                          {selectedDocument.riskLevel}
                        </Badge>
                      </div>
                      {showPolicyVersions && selectedDocument.policyVersion && (
                        <div className="flex justify-between">
                          <span className="text-gray-600">Policy Version:</span>
                          <Badge variant="outline" className="bg-blue-50 text-blue-700">
                            {selectedDocument.policyVersion}
                          </Badge>
                        </div>
                      )}
                    </div>
                  </Card>

                  <Card className="p-4">
                    <h3 className="text-lg font-medium text-gray-900 mb-3">Issuer & Dates</h3>
                    <div className="space-y-2 text-sm">
                      <div className="flex justify-between">
                        <span className="text-gray-600">Issued By:</span>
                        <span className="font-medium">{selectedDocument.issuedBy}</span>
                      </div>
                      <div className="flex justify-between">
                        <span className="text-gray-600">Jurisdiction:</span>
                        <span className="font-medium">{selectedDocument.jurisdiction}</span>
                      </div>
                      <div className="flex justify-between">
                        <span className="text-gray-600">Issued Date:</span>
                        <span className="font-medium">{selectedDocument.issuedDate.toLocaleDateString()}</span>
                      </div>
                      <div className="flex justify-between">
                        <span className="text-gray-600">Expiry Date:</span>
                        <span className={`font-medium ${
                          getDaysUntilExpiry(selectedDocument.expiryDate) <= 30 ? 'text-red-600' : 'text-gray-900'
                        }`}>
                          {selectedDocument.expiryDate.toLocaleDateString()}
                        </span>
                      </div>
                      {selectedDocument.lastAuditDate && (
                        <div className="flex justify-between">
                          <span className="text-gray-600">Last Audit:</span>
                          <span className="font-medium">{selectedDocument.lastAuditDate.toLocaleDateString()}</span>
                        </div>
                      )}
                      {selectedDocument.nextAuditDate && (
                        <div className="flex justify-between">
                          <span className="text-gray-600">Next Audit:</span>
                          <span className="font-medium">{selectedDocument.nextAuditDate.toLocaleDateString()}</span>
                        </div>
                      )}
                    </div>
                  </Card>
                </div>

                <Card className="p-4">
                  <h3 className="text-lg font-medium text-gray-900 mb-3">Description</h3>
                  <p className="text-gray-600">{selectedDocument.description}</p>
                </Card>

                {selectedDocument.complianceScore && (
                  <Card className="p-4">
                    <h3 className="text-lg font-medium text-gray-900 mb-3">Compliance Score</h3>
                    <div className="flex items-center space-x-4">
                      <div className="flex-1">
                        <Progress value={selectedDocument.complianceScore} className="h-3" />
                      </div>
                      <div className="text-2xl font-bold text-blue-600">
                        {selectedDocument.complianceScore}%
                      </div>
                    </div>
                  </Card>
                )}

                {selectedDocument.notes.length > 0 && (
                  <Card className="p-4">
                    <h3 className="text-lg font-medium text-gray-900 mb-3">Notes</h3>
                    <div className="space-y-2">
                      {selectedDocument.notes.map((note, index) => (
                        <div key={index} className="p-2 bg-gray-50 rounded text-sm">
                          {note}
                        </div>
                      ))}
                    </div>
                  </Card>
                )}
              </TabsContent>

              <TabsContent value="requirements" className="space-y-4">
                <div className="space-y-3">
                  {selectedDocument.requirements.map(req => (
                    <Card key={req.id} className="p-4">
                      <div className="flex items-start justify-between mb-3">
                        <div className="flex items-start space-x-3">
                          {getRequirementStatusIcon(req.status)}
                          <div className="flex-1">
                            <h4 className="font-medium text-gray-900">{req.requirement}</h4>
                            <div className="text-sm text-gray-600 mt-1">
                              Responsible: {req.responsible}
                            </div>
                          </div>
                        </div>
                        <Badge className={
                          req.status === 'met' ? 'bg-green-100 text-green-800' :
                          req.status === 'partial' ? 'bg-yellow-100 text-yellow-800' :
                          req.status === 'not_met' ? 'bg-red-100 text-red-800' :
                          'bg-gray-100 text-gray-800'
                        }>
                          {req.status.replace('_', ' ')}
                        </Badge>
                      </div>

                      <div className="grid grid-cols-2 gap-4 text-sm">
                        <div>
                          <span className="text-gray-500">Last Checked:</span>
                          <div className="font-medium">{req.lastChecked.toLocaleDateString()}</div>
                        </div>
                        <div>
                          <span className="text-gray-500">Next Check:</span>
                          <div className="font-medium">{req.nextCheck.toLocaleDateString()}</div>
                        </div>
                      </div>

                      {req.evidence && req.evidence.length > 0 && (
                        <div className="mt-3">
                          <div className="text-sm font-medium text-gray-900 mb-1">Evidence:</div>
                          <div className="flex flex-wrap gap-1">
                            {req.evidence.map((evidence, index) => (
                              <Badge key={index} variant="outline" size="sm">
                                {evidence}
                              </Badge>
                            ))}
                          </div>
                        </div>
                      )}
                    </Card>
                  ))}

                  {selectedDocument.requirements.length === 0 && (
                    <div className="text-center py-8">
                      <CheckCircle className="w-12 h-12 text-gray-400 mx-auto mb-2" />
                      <p className="text-gray-600">No requirements defined</p>
                    </div>
                  )}
                </div>
              </TabsContent>

              <TabsContent value="attachments" className="space-y-4">
                <div className="space-y-3">
                  {selectedDocument.attachments.map(attachment => (
                    <Card key={attachment.id} className="p-4">
                      <div className="flex items-center justify-between">
                        <div className="flex items-center space-x-3">
                          <div className="p-2 bg-blue-100 rounded">
                            <FileText className="w-5 h-5 text-blue-600" />
                          </div>
                          <div>
                            <div className="font-medium text-gray-900 flex items-center space-x-2">
                              <span>{attachment.filename}</span>
                              {attachment.signed && (
                                <Badge className="bg-green-100 text-green-800" size="sm">
                                  <Lock className="w-3 h-3 mr-1" />
                                  Signed
                                </Badge>
                              )}
                            </div>
                            <div className="text-sm text-gray-600">{attachment.description}</div>
                            <div className="text-xs text-gray-500">
                              Uploaded by {attachment.uploadedBy} on {attachment.uploadedAt.toLocaleString()}
                            </div>
                            <div className="text-xs text-gray-500 font-mono">
                              Hash: {attachment.hash.substring(0, 16)}...
                            </div>
                          </div>
                        </div>
                        <div className="flex items-center space-x-2">
                          <Button variant="outline" size="sm">
                            <Eye className="w-4 h-4" />
                          </Button>
                          <Button variant="outline" size="sm">
                            <Download className="w-4 h-4" />
                          </Button>
                        </div>
                      </div>
                    </Card>
                  ))}

                  {selectedDocument.attachments.length === 0 && (
                    <div className="text-center py-8">
                      <FileText className="w-12 h-12 text-gray-400 mx-auto mb-2" />
                      <p className="text-gray-600">No attachments</p>
                    </div>
                  )}
                </div>
              </TabsContent>

              <TabsContent value="renewal" className="space-y-4">
                {selectedDocument.renewalProcess ? (
                  <Card className="p-4">
                    <h3 className="text-lg font-medium text-gray-900 mb-3">Renewal Process</h3>
                    
                    <div className="grid grid-cols-2 gap-6 mb-4">
                      <div className="space-y-2 text-sm">
                        <div className="flex justify-between">
                          <span className="text-gray-600">Status:</span>
                          <Badge className={getStatusColor(selectedDocument.renewalProcess.status)}>
                            {selectedDocument.renewalProcess.status.replace('_', ' ')}
                          </Badge>
                        </div>
                        <div className="flex justify-between">
                          <span className="text-gray-600">Start Date:</span>
                          <span className="font-medium">
                            {selectedDocument.renewalProcess.startDate.toLocaleDateString()}
                          </span>
                        </div>
                        <div className="flex justify-between">
                          <span className="text-gray-600">Deadline:</span>
                          <span className="font-medium text-red-600">
                            {selectedDocument.renewalProcess.submissionDeadline.toLocaleDateString()}
                          </span>
                        </div>
                        <div className="flex justify-between">
                          <span className="text-gray-600">Renewal Fee:</span>
                          <span className="font-medium">
                            ${selectedDocument.renewalProcess.renewalFee.toLocaleString()}
                          </span>
                        </div>
                      </div>

                      <div className="space-y-2 text-sm">
                        {selectedDocument.renewalProcess.submittedBy && (
                          <div className="flex justify-between">
                            <span className="text-gray-600">Submitted By:</span>
                            <span className="font-medium">{selectedDocument.renewalProcess.submittedBy}</span>
                          </div>
                        )}
                        {selectedDocument.renewalProcess.submittedDate && (
                          <div className="flex justify-between">
                            <span className="text-gray-600">Submitted:</span>
                            <span className="font-medium">
                              {selectedDocument.renewalProcess.submittedDate.toLocaleDateString()}
                            </span>
                          </div>
                        )}
                        {selectedDocument.renewalProcess.approvedBy && (
                          <div className="flex justify-between">
                            <span className="text-gray-600">Approved By:</span>
                            <span className="font-medium">{selectedDocument.renewalProcess.approvedBy}</span>
                          </div>
                        )}
                        {selectedDocument.renewalProcess.rejectionReason && (
                          <div className="flex justify-between">
                            <span className="text-gray-600">Rejection Reason:</span>
                            <span className="font-medium text-red-600">
                              {selectedDocument.renewalProcess.rejectionReason}
                            </span>
                          </div>
                        )}
                      </div>
                    </div>

                    <div>
                      <h4 className="font-medium text-gray-900 mb-2">Required Documents</h4>
                      <div className="flex flex-wrap gap-2">
                        {selectedDocument.renewalProcess.requiredDocuments.map((doc, index) => (
                          <Badge key={index} variant="outline">
                            {doc}
                          </Badge>
                        ))}
                      </div>
                    </div>
                  </Card>
                ) : (
                  <div className="text-center py-8">
                    <Calendar className="w-12 h-12 text-gray-400 mx-auto mb-2" />
                    <p className="text-gray-600">No renewal process defined</p>
                  </div>
                )}
              </TabsContent>

              <TabsContent value="evidence" className="space-y-4">
                {selectedDocument.evidencePackage ? (
                  <Card className="p-4">
                    <div className="flex items-center justify-between mb-4">
                      <h3 className="text-lg font-medium text-gray-900">Evidence Package</h3>
                      <Button
                        variant="outline"
                        onClick={() => handleExportEvidence(selectedDocument)}
                      >
                        <Download className="w-4 h-4 mr-2" />
                        Export Evidence
                      </Button>
                    </div>
                    
                    <div className="grid grid-cols-2 gap-6 mb-4">
                      <div className="space-y-2 text-sm">
                        <div className="flex justify-between">
                          <span className="text-gray-600">Package ID:</span>
                          <span className="font-medium font-mono">{selectedDocument.evidencePackage.id}</span>
                        </div>
                        <div className="flex justify-between">
                          <span className="text-gray-600">Version:</span>
                          <span className="font-medium">{selectedDocument.evidencePackage.version}</span>
                        </div>
                        <div className="flex justify-between">
                          <span className="text-gray-600">Created By:</span>
                          <span className="font-medium">{selectedDocument.evidencePackage.createdBy}</span>
                        </div>
                        <div className="flex justify-between">
                          <span className="text-gray-600">Created:</span>
                          <span className="font-medium">
                            {selectedDocument.evidencePackage.createdAt.toLocaleString()}
                          </span>
                        </div>
                      </div>

                      <div className="space-y-2 text-sm">
                        <div className="flex justify-between">
                          <span className="text-gray-600">Format:</span>
                          <Badge variant="outline">{selectedDocument.evidencePackage.exportFormat.toUpperCase()}</Badge>
                        </div>
                        <div className="flex justify-between">
                          <span className="text-gray-600">Encryption:</span>
                          <Badge variant="outline">{selectedDocument.evidencePackage.encryptionLevel}</Badge>
                        </div>
                        <div className="flex justify-between">
                          <span className="text-gray-600">Digital Signature:</span>
                          <Badge className="bg-green-100 text-green-800">
                            <Lock className="w-3 h-3 mr-1" />
                            Verified
                          </Badge>
                        </div>
                      </div>
                    </div>

                    <div>
                      <h4 className="font-medium text-gray-900 mb-2">Package Contents</h4>
                      <div className="grid grid-cols-2 gap-4">
                        <div>
                          <div className="text-sm text-gray-600 mb-1">Documents</div>
                          <div className="space-y-1">
                            {selectedDocument.evidencePackage.contents.documents.map((doc, index) => (
                              <Badge key={index} variant="outline" size="sm">
                                {doc}
                              </Badge>
                            ))}
                          </div>
                        </div>
                        <div>
                          <div className="text-sm text-gray-600 mb-1">Telemetry Data</div>
                          <div className="space-y-1">
                            {selectedDocument.evidencePackage.contents.telemetryData.map((data, index) => (
                              <Badge key={index} variant="outline" size="sm">
                                {data}
                              </Badge>
                            ))}
                          </div>
                        </div>
                        <div>
                          <div className="text-sm text-gray-600 mb-1">Incident Reports</div>
                          <div className="space-y-1">
                            {selectedDocument.evidencePackage.contents.incidentReports.length > 0 ? 
                              selectedDocument.evidencePackage.contents.incidentReports.map((report, index) => (
                                <Badge key={index} variant="outline" size="sm">
                                  {report}
                                </Badge>
                              )) : 
                              <span className="text-xs text-gray-500">None</span>
                            }
                          </div>
                        </div>
                        <div>
                          <div className="text-sm text-gray-600 mb-1">Maintenance Records</div>
                          <div className="space-y-1">
                            {selectedDocument.evidencePackage.contents.maintenanceRecords.map((record, index) => (
                              <Badge key={index} variant="outline" size="sm">
                                {record}
                              </Badge>
                            ))}
                          </div>
                        </div>
                      </div>
                    </div>

                    <div className="mt-4 p-3 bg-green-50 rounded border border-green-200">
                      <div className="flex items-center space-x-2">
                        <Lock className="w-4 h-4 text-green-600" />
                        <span className="text-sm font-medium text-green-900">
                          Evidence package is cryptographically signed and tamper-evident
                        </span>
                      </div>
                      <div className="text-xs text-green-700 mt-1 font-mono">
                        Signature: {selectedDocument.evidencePackage.digitalSignature.substring(0, 32)}...
                      </div>
                    </div>
                  </Card>
                ) : (
                  <div className="text-center py-8">
                    <Shield className="w-12 h-12 text-gray-400 mx-auto mb-2" />
                    <p className="text-gray-600">No evidence package available</p>
                    <Button className="mt-4" variant="outline">
                      <Plus className="w-4 h-4 mr-2" />
                      Generate Evidence Package
                    </Button>
                  </div>
                )}
              </TabsContent>
            </Tabs>
          )}
        </DialogContent>
      </Dialog>
    </div>
  )
}

export default ComplianceManager
