import React, { useState, useEffect } from 'react'
import { useTranslation } from 'react-i18next'
import { motion } from 'framer-motion'
import {
  ShieldCheckIcon,
  DocumentTextIcon,
  ExclamationTriangleIcon,
  CheckCircleIcon,
  ClockIcon,
  DownloadIcon,
  EyeIcon,
  FlagIcon,
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
interface ComplianceReport {
  report_id: string
  organization_id: string
  report_date: string
  compliance_score: number
  total_vehicles: number
  compliant_vehicles: number
  violations_count: number
  critical_violations: number
  regulatory_requirements: {
    uae_av_regulations: {
      autonomy_levels_approved: string[]
      safety_driver_required: boolean
      insurance_mandatory: boolean
      regular_inspections: boolean
    }
    adta_guidelines: {
      data_residency: string
      privacy_compliance: string
      cybersecurity_standards: string
    }
    environmental_standards: {
      emissions_limits: string
      noise_limits: string
      waste_management: string
    }
  }
  compliance_status: string
  recommendations: string[]
  next_audit_date: string
}

interface AuditTrail {
  audit_id: string
  entity_type: string
  entity_id: string
  operation: string
  old_values?: any
  new_values?: any
  changed_fields: string[]
  changed_by: string
  change_reason?: string
  changed_at: string
  correlation_id: string
  signature?: string
}

interface EvidencePackage {
  package_id: string
  title: string
  description: string
  compliance_area: string
  created_date: string
  status: 'draft' | 'review' | 'approved' | 'submitted'
  documents: EvidenceDocument[]
  reviewer?: string
  submission_date?: string
}

interface EvidenceDocument {
  document_id: string
  filename: string
  document_type: string
  size: number
  uploaded_date: string
  checksum: string
}

interface ComplianceViolation {
  violation_id: string
  vehicle_id: string
  asset_tag: string
  violation_type: string
  severity: 'low' | 'medium' | 'high' | 'critical'
  description: string
  regulation_reference: string
  detected_date: string
  status: 'active' | 'acknowledged' | 'resolved' | 'dismissed'
  assigned_to?: string
  resolution_date?: string
  resolution_notes?: string
}

const ComplianceAuditDashboard: React.FC = () => {
  const { t } = useTranslation()
  const [complianceReport, setComplianceReport] = useState<ComplianceReport | null>(null)
  const [auditTrail, setAuditTrail] = useState<AuditTrail[]>([])
  const [evidencePackages, setEvidencePackages] = useState<EvidencePackage[]>([])
  const [violations, setViolations] = useState<ComplianceViolation[]>([])
  const [loading, setLoading] = useState(true)
  const [error, setError] = useState<string | null>(null)
  const [selectedTab, setSelectedTab] = useState<'overview' | 'violations' | 'audit' | 'evidence'>('overview')

  // Fetch compliance data
  const fetchComplianceData = async () => {
    try {
      setLoading(true)

      // Fetch compliance report
      const reportResponse = await fetch('/api/v1/compliance/report')
      const reportData = await reportResponse.json()
      setComplianceReport(reportData)

      // Fetch audit trail
      const auditResponse = await fetch('/api/v1/audit/trail?limit=100')
      const auditData = await auditResponse.json()
      setAuditTrail(auditData)

      // Fetch evidence packages
      const evidenceResponse = await fetch('/api/v1/compliance/evidence-packages')
      const evidenceData = await evidenceResponse.json()
      setEvidencePackages(evidenceData)

      // Fetch violations
      const violationsResponse = await fetch('/api/v1/compliance/violations')
      const violationsData = await violationsResponse.json()
      setViolations(violationsData)

      setError(null)
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Failed to fetch compliance data')
    } finally {
      setLoading(false)
    }
  }

  useEffect(() => {
    fetchComplianceData()
  }, [])

  // Generate compliance report
  const generateReport = async (reportType: string) => {
    try {
      const response = await fetch(`/api/v1/compliance/generate-report`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ report_type: reportType }),
      })
      
      if (response.ok) {
        const blob = await response.blob()
        const url = window.URL.createObjectURL(blob)
        const a = document.createElement('a')
        a.href = url
        a.download = `${reportType}_report_${new Date().toISOString().split('T')[0]}.pdf`
        document.body.appendChild(a)
        a.click()
        window.URL.revokeObjectURL(url)
        document.body.removeChild(a)
      }
    } catch (err) {
      console.error('Failed to generate report:', err)
    }
  }

  if (loading) {
    return (
      <div className="flex items-center justify-center h-64">
        <LoadingSpinner />
      </div>
    )
  }

  if (error || !complianceReport) {
    return (
      <div className="flex items-center justify-center h-64">
        <div className="text-center">
          <ExclamationTriangleIcon className="h-12 w-12 text-red-500 mx-auto mb-4" />
          <p className="text-gray-600">{error || 'No compliance data available'}</p>
          <Button onClick={fetchComplianceData} className="mt-4">
            Retry
          </Button>
        </div>
      </div>
    )
  }

  // Prepare chart data
  const complianceScoreHistory = [
    { month: 'Jan', score: 92 },
    { month: 'Feb', score: 94 },
    { month: 'Mar', score: 91 },
    { month: 'Apr', score: 96 },
    { month: 'May', score: complianceReport.compliance_score },
  ]

  const violationsByType = violations.reduce((acc, violation) => {
    acc[violation.violation_type] = (acc[violation.violation_type] || 0) + 1
    return acc
  }, {} as Record<string, number>)

  const violationTypeData = Object.entries(violationsByType).map(([type, count]) => ({
    name: type,
    value: count,
  }))

  const complianceStatusData = [
    { name: 'Compliant', value: complianceReport.compliant_vehicles, color: '#10B981' },
    { name: 'Non-Compliant', value: complianceReport.total_vehicles - complianceReport.compliant_vehicles, color: '#EF4444' },
  ]

  // Table columns
  const violationColumns = [
    {
      header: 'Vehicle',
      accessorKey: 'asset_tag',
      cell: ({ row }: any) => (
        <div className="font-medium">{row.original.asset_tag}</div>
      ),
    },
    {
      header: 'Violation Type',
      accessorKey: 'violation_type',
      cell: ({ row }: any) => (
        <Badge variant="outline">{row.original.violation_type}</Badge>
      ),
    },
    {
      header: 'Severity',
      accessorKey: 'severity',
      cell: ({ row }: any) => (
        <Badge variant={
          row.original.severity === 'critical' ? 'destructive' :
          row.original.severity === 'high' ? 'warning' :
          'default'
        }>
          {row.original.severity}
        </Badge>
      ),
    },
    {
      header: 'Status',
      accessorKey: 'status',
      cell: ({ row }: any) => (
        <Badge variant={
          row.original.status === 'resolved' ? 'success' :
          row.original.status === 'active' ? 'destructive' :
          'default'
        }>
          {row.original.status}
        </Badge>
      ),
    },
    {
      header: 'Detected Date',
      accessorKey: 'detected_date',
      cell: ({ row }: any) => (
        <div>{new Date(row.original.detected_date).toLocaleDateString()}</div>
      ),
    },
    {
      header: 'Actions',
      id: 'actions',
      cell: ({ row }: any) => (
        <div className="flex space-x-2">
          <Button size="sm" variant="outline">
            <EyeIcon className="h-4 w-4" />
          </Button>
          {row.original.status === 'active' && (
            <Button size="sm" variant="default">
              Resolve
            </Button>
          )}
        </div>
      ),
    },
  ]

  const auditColumns = [
    {
      header: 'Entity',
      accessorKey: 'entity_type',
      cell: ({ row }: any) => (
        <div>
          <div className="font-medium">{row.original.entity_type}</div>
          <div className="text-xs text-gray-500">{row.original.entity_id}</div>
        </div>
      ),
    },
    {
      header: 'Operation',
      accessorKey: 'operation',
      cell: ({ row }: any) => (
        <Badge variant={
          row.original.operation === 'CREATE' ? 'success' :
          row.original.operation === 'DELETE' ? 'destructive' :
          'default'
        }>
          {row.original.operation}
        </Badge>
      ),
    },
    {
      header: 'Changed By',
      accessorKey: 'changed_by',
    },
    {
      header: 'Date',
      accessorKey: 'changed_at',
      cell: ({ row }: any) => (
        <div>{new Date(row.original.changed_at).toLocaleString()}</div>
      ),
    },
    {
      header: 'Actions',
      id: 'actions',
      cell: ({ row }: any) => (
        <Button size="sm" variant="outline">
          <EyeIcon className="h-4 w-4" />
        </Button>
      ),
    },
  ]

  return (
    <div className="space-y-6 p-6">
      {/* Header */}
      <div className="flex items-center justify-between">
        <div>
          <h1 className="text-3xl font-bold text-gray-900">Compliance & Audit</h1>
          <p className="text-gray-600">
            UAE Regulatory Compliance and Audit Trail Management
          </p>
        </div>
        <div className="flex items-center space-x-4">
          <Button onClick={() => generateReport('compliance')} variant="outline">
            <DownloadIcon className="h-4 w-4 mr-2" />
            Export Report
          </Button>
          <Button onClick={fetchComplianceData}>
            Refresh
          </Button>
        </div>
      </div>

      {/* Tab Navigation */}
      <div className="border-b border-gray-200">
        <nav className="-mb-px flex space-x-8">
          {[
            { id: 'overview', name: 'Overview', icon: ShieldCheckIcon },
            { id: 'violations', name: 'Violations', icon: ExclamationTriangleIcon },
            { id: 'audit', name: 'Audit Trail', icon: DocumentTextIcon },
            { id: 'evidence', name: 'Evidence Packages', icon: FlagIcon },
          ].map((tab) => (
            <button
              key={tab.id}
              onClick={() => setSelectedTab(tab.id as any)}
              className={`flex items-center py-2 px-1 border-b-2 font-medium text-sm ${
                selectedTab === tab.id
                  ? 'border-blue-500 text-blue-600'
                  : 'border-transparent text-gray-500 hover:text-gray-700 hover:border-gray-300'
              }`}
            >
              <tab.icon className="h-5 w-5 mr-2" />
              {tab.name}
            </button>
          ))}
        </nav>
      </div>

      {/* Overview Tab */}
      {selectedTab === 'overview' && (
        <div className="space-y-6">
          {/* Summary Cards */}
          <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 gap-6">
            <Card>
              <CardHeader className="flex flex-row items-center justify-between space-y-0 pb-2">
                <CardTitle className="text-sm font-medium">Compliance Score</CardTitle>
                <ShieldCheckIcon className="h-4 w-4 text-green-500" />
              </CardHeader>
              <CardContent>
                <div className="text-2xl font-bold text-green-600">
                  {complianceReport.compliance_score.toFixed(1)}%
                </div>
                <p className="text-xs text-muted-foreground">
                  Target: 95%
                </p>
              </CardContent>
            </Card>

            <Card>
              <CardHeader className="flex flex-row items-center justify-between space-y-0 pb-2">
                <CardTitle className="text-sm font-medium">Active Violations</CardTitle>
                <ExclamationTriangleIcon className="h-4 w-4 text-red-500" />
              </CardHeader>
              <CardContent>
                <div className="text-2xl font-bold text-red-600">
                  {complianceReport.violations_count}
                </div>
                <p className="text-xs text-muted-foreground">
                  {complianceReport.critical_violations} critical
                </p>
              </CardContent>
            </Card>

            <Card>
              <CardHeader className="flex flex-row items-center justify-between space-y-0 pb-2">
                <CardTitle className="text-sm font-medium">Compliant Vehicles</CardTitle>
                <CheckCircleIcon className="h-4 w-4 text-blue-500" />
              </CardHeader>
              <CardContent>
                <div className="text-2xl font-bold text-blue-600">
                  {complianceReport.compliant_vehicles}
                </div>
                <p className="text-xs text-muted-foreground">
                  of {complianceReport.total_vehicles} total
                </p>
              </CardContent>
            </Card>

            <Card>
              <CardHeader className="flex flex-row items-center justify-between space-y-0 pb-2">
                <CardTitle className="text-sm font-medium">Next Audit</CardTitle>
                <ClockIcon className="h-4 w-4 text-orange-500" />
              </CardHeader>
              <CardContent>
                <div className="text-lg font-bold text-orange-600">
                  {new Date(complianceReport.next_audit_date).toLocaleDateString()}
                </div>
                <p className="text-xs text-muted-foreground">
                  Scheduled audit date
                </p>
              </CardContent>
            </Card>
          </div>

          {/* Charts */}
          <div className="grid grid-cols-1 lg:grid-cols-2 gap-6">
            {/* Compliance Score Trend */}
            <Card>
              <CardHeader>
                <CardTitle>Compliance Score Trend</CardTitle>
              </CardHeader>
              <CardContent>
                <ResponsiveContainer width="100%" height={300}>
                  <LineChart data={complianceScoreHistory}>
                    <CartesianGrid strokeDasharray="3 3" />
                    <XAxis dataKey="month" />
                    <YAxis domain={[80, 100]} />
                    <Tooltip />
                    <Line 
                      type="monotone" 
                      dataKey="score" 
                      stroke="#3B82F6" 
                      strokeWidth={2}
                    />
                  </LineChart>
                </ResponsiveContainer>
              </CardContent>
            </Card>

            {/* Compliance Status Distribution */}
            <Card>
              <CardHeader>
                <CardTitle>Fleet Compliance Status</CardTitle>
              </CardHeader>
              <CardContent>
                <ResponsiveContainer width="100%" height={300}>
                  <PieChart>
                    <Pie
                      data={complianceStatusData}
                      cx="50%"
                      cy="50%"
                      labelLine={false}
                      label={({ name, value }) => `${name}: ${value}`}
                      outerRadius={80}
                      fill="#8884d8"
                      dataKey="value"
                    >
                      {complianceStatusData.map((entry, index) => (
                        <Cell key={`cell-${index}`} fill={entry.color} />
                      ))}
                    </Pie>
                    <Tooltip />
                  </PieChart>
                </ResponsiveContainer>
              </CardContent>
            </Card>
          </div>

          {/* UAE Regulatory Requirements */}
          <Card>
            <CardHeader>
              <CardTitle className="flex items-center">
                <FlagIcon className="h-5 w-5 mr-2 text-red-500" />
                UAE Regulatory Requirements Compliance
              </CardTitle>
            </CardHeader>
            <CardContent>
              <div className="grid grid-cols-1 md:grid-cols-3 gap-6">
                <div className="bg-red-50 p-4 rounded-lg">
                  <h3 className="font-semibold text-red-800 mb-3">UAE AV Regulations</h3>
                  <div className="space-y-2">
                    <div className="flex items-center justify-between">
                      <span className="text-sm">Approved Autonomy Levels</span>
                      <Badge variant="success">L0-L4</Badge>
                    </div>
                    <div className="flex items-center justify-between">
                      <span className="text-sm">Safety Driver Required</span>
                      <CheckCircleIcon className="h-4 w-4 text-green-500" />
                    </div>
                    <div className="flex items-center justify-between">
                      <span className="text-sm">Insurance Mandatory</span>
                      <CheckCircleIcon className="h-4 w-4 text-green-500" />
                    </div>
                    <div className="flex items-center justify-between">
                      <span className="text-sm">Regular Inspections</span>
                      <CheckCircleIcon className="h-4 w-4 text-green-500" />
                    </div>
                  </div>
                </div>

                <div className="bg-blue-50 p-4 rounded-lg">
                  <h3 className="font-semibold text-blue-800 mb-3">ADTA Guidelines</h3>
                  <div className="space-y-2">
                    <div className="flex items-center justify-between">
                      <span className="text-sm">Data Residency</span>
                      <Badge variant="success">UAE Only</Badge>
                    </div>
                    <div className="flex items-center justify-between">
                      <span className="text-sm">Privacy Compliance</span>
                      <Badge variant="success">GDPR Equivalent</Badge>
                    </div>
                    <div className="flex items-center justify-between">
                      <span className="text-sm">Cybersecurity</span>
                      <Badge variant="success">ISO 27001</Badge>
                    </div>
                  </div>
                </div>

                <div className="bg-green-50 p-4 rounded-lg">
                  <h3 className="font-semibold text-green-800 mb-3">Environmental Standards</h3>
                  <div className="space-y-2">
                    <div className="flex items-center justify-between">
                      <span className="text-sm">Emissions Limits</span>
                      <Badge variant="success">Euro 6</Badge>
                    </div>
                    <div className="flex items-center justify-between">
                      <span className="text-sm">Noise Limits</span>
                      <Badge variant="success">70 dB</Badge>
                    </div>
                    <div className="flex items-center justify-between">
                      <span className="text-sm">Waste Management</span>
                      <CheckCircleIcon className="h-4 w-4 text-green-500" />
                    </div>
                  </div>
                </div>
              </div>
            </CardContent>
          </Card>

          {/* Recommendations */}
          <Card>
            <CardHeader>
              <CardTitle>Compliance Recommendations</CardTitle>
            </CardHeader>
            <CardContent>
              <div className="space-y-3">
                {complianceReport.recommendations.map((recommendation, index) => (
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
                ))}
              </div>
            </CardContent>
          </Card>
        </div>
      )}

      {/* Violations Tab */}
      {selectedTab === 'violations' && (
        <div className="space-y-6">
          <Card>
            <CardHeader>
              <CardTitle>Compliance Violations</CardTitle>
            </CardHeader>
            <CardContent>
              <DataTable
                columns={violationColumns}
                data={violations}
                searchPlaceholder="Search violations..."
                pageSize={10}
              />
            </CardContent>
          </Card>

          {/* Violations by Type Chart */}
          <Card>
            <CardHeader>
              <CardTitle>Violations by Type</CardTitle>
            </CardHeader>
            <CardContent>
              <ResponsiveContainer width="100%" height={300}>
                <BarChart data={violationTypeData}>
                  <CartesianGrid strokeDasharray="3 3" />
                  <XAxis dataKey="name" />
                  <YAxis />
                  <Tooltip />
                  <Bar dataKey="value" fill="#EF4444" />
                </BarChart>
              </ResponsiveContainer>
            </CardContent>
          </Card>
        </div>
      )}

      {/* Audit Trail Tab */}
      {selectedTab === 'audit' && (
        <Card>
          <CardHeader>
            <CardTitle>Audit Trail</CardTitle>
          </CardHeader>
          <CardContent>
            <DataTable
              columns={auditColumns}
              data={auditTrail}
              searchPlaceholder="Search audit trail..."
              pageSize={15}
            />
          </CardContent>
        </Card>
      )}

      {/* Evidence Packages Tab */}
      {selectedTab === 'evidence' && (
        <div className="space-y-6">
          <div className="flex justify-between items-center">
            <h2 className="text-xl font-semibold">Evidence Packages</h2>
            <Button>
              Create New Package
            </Button>
          </div>

          <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-6">
            {evidencePackages.map((pkg) => (
              <Card key={pkg.package_id}>
                <CardHeader>
                  <CardTitle className="text-lg">{pkg.title}</CardTitle>
                  <Badge variant={
                    pkg.status === 'approved' ? 'success' :
                    pkg.status === 'review' ? 'warning' :
                    pkg.status === 'submitted' ? 'default' :
                    'outline'
                  }>
                    {pkg.status}
                  </Badge>
                </CardHeader>
                <CardContent>
                  <p className="text-sm text-gray-600 mb-4">{pkg.description}</p>
                  <div className="space-y-2">
                    <div className="flex justify-between text-sm">
                      <span>Compliance Area:</span>
                      <span className="font-medium">{pkg.compliance_area}</span>
                    </div>
                    <div className="flex justify-between text-sm">
                      <span>Documents:</span>
                      <span className="font-medium">{pkg.documents.length}</span>
                    </div>
                    <div className="flex justify-between text-sm">
                      <span>Created:</span>
                      <span>{new Date(pkg.created_date).toLocaleDateString()}</span>
                    </div>
                  </div>
                  <div className="flex space-x-2 mt-4">
                    <Button size="sm" variant="outline" className="flex-1">
                      <EyeIcon className="h-4 w-4 mr-2" />
                      View
                    </Button>
                    <Button size="sm" variant="outline">
                      <DownloadIcon className="h-4 w-4" />
                    </Button>
                  </div>
                </CardContent>
              </Card>
            ))}
          </div>
        </div>
      )}
    </div>
  )
}

export default ComplianceAuditDashboard
