import React, { useState, useEffect } from 'react'
import { useTranslation } from 'react-i18next'
import { Helmet } from 'react-helmet-async'
import { motion } from 'framer-motion'
import {
  ShieldCheckIcon,
  DocumentTextIcon,
  ClockIcon,
  ExclamationTriangleIcon,
  CheckCircleIcon,
  XCircleIcon,
  ArrowDownTrayIcon,
  EyeIcon,
  CalendarIcon,
  UserIcon,
  TagIcon,
  ChartBarIcon,
} from '@heroicons/react/24/outline'

// Components
import { Button } from '@components/ui/Button'
import { Badge } from '@components/ui/Badge'
import { Card, CardContent, CardHeader, CardTitle } from '@components/ui/Card'
import { Tabs, TabsContent, TabsList, TabsTrigger } from '@components/ui/Tabs'
import { Input } from '@components/ui/Input'
import { Select } from '@components/ui/Select'
import { DataTable } from '@components/ui/DataTable'
import { Modal } from '@components/ui/Modal'
import { DatePicker } from '@components/ui/DatePicker'
import { Progress } from '@components/ui/Progress'

// Hooks
import { useCompliance } from '@hooks/useCompliance'
import { useAuth } from '@hooks/useAuth'

// Types
interface ComplianceFramework {
  id: string
  name: string
  version: string
  description: string
  status: 'compliant' | 'non_compliant' | 'partial' | 'pending'
  lastAssessment: string
  nextAssessment: string
  requirements: ComplianceRequirement[]
  certifications: Certification[]
}

interface ComplianceRequirement {
  id: string
  frameworkId: string
  code: string
  title: string
  description: string
  category: string
  priority: 'low' | 'medium' | 'high' | 'critical'
  status: 'compliant' | 'non_compliant' | 'partial' | 'not_assessed'
  evidence: Evidence[]
  lastReview: string
  nextReview: string
  assignedTo: string
  automatedCheck: boolean
}

interface Evidence {
  id: string
  type: 'document' | 'screenshot' | 'log' | 'certificate' | 'report'
  name: string
  description: string
  url: string
  uploadedBy: string
  uploadedAt: string
  expiresAt?: string
  verified: boolean
  verifiedBy?: string
  verifiedAt?: string
}

interface Certification {
  id: string
  name: string
  issuedBy: string
  issuedAt: string
  expiresAt: string
  status: 'active' | 'expired' | 'pending' | 'revoked'
  certificateUrl: string
}

interface AuditLog {
  id: string
  timestamp: string
  action: string
  entity: string
  entityId: string
  actor: string
  changes: any
  ipAddress: string
  userAgent: string
  signature: string
}

export default function Compliance() {
  const { t } = useTranslation()
  const { user, permissions } = useAuth()

  // State
  const [activeTab, setActiveTab] = useState<'frameworks' | 'requirements' | 'evidence' | 'audits'>('frameworks')
  const [selectedFramework, setSelectedFramework] = useState<ComplianceFramework | null>(null)
  const [selectedRequirement, setSelectedRequirement] = useState<ComplianceRequirement | null>(null)
  const [evidenceModalOpen, setEvidenceModalOpen] = useState(false)
  const [auditModalOpen, setAuditModalOpen] = useState(false)
  const [reportModalOpen, setReportModalOpen] = useState(false)
  
  // Filters
  const [frameworkFilter, setFrameworkFilter] = useState('all')
  const [statusFilter, setStatusFilter] = useState('all')
  const [priorityFilter, setPriorityFilter] = useState('all')
  const [dateRange, setDateRange] = useState({ start: '', end: '' })
  const [searchTerm, setSearchTerm] = useState('')

  // Data fetching
  const {
    frameworks,
    requirements,
    auditLogs,
    generateComplianceReport,
    uploadEvidence,
    verifyEvidence,
    isLoading,
  } = useCompliance({
    framework: frameworkFilter === 'all' ? undefined : frameworkFilter,
    status: statusFilter === 'all' ? undefined : statusFilter,
    priority: priorityFilter === 'all' ? undefined : priorityFilter,
    dateRange: dateRange.start && dateRange.end ? dateRange : undefined,
    search: searchTerm,
  })

  // Calculate compliance metrics
  const complianceMetrics = React.useMemo(() => {
    if (!frameworks) return null

    const totalRequirements = frameworks.reduce((sum, f) => sum + f.requirements.length, 0)
    const compliantRequirements = frameworks.reduce(
      (sum, f) => sum + f.requirements.filter(r => r.status === 'compliant').length, 0
    )
    const nonCompliantRequirements = frameworks.reduce(
      (sum, f) => sum + f.requirements.filter(r => r.status === 'non_compliant').length, 0
    )
    const partialRequirements = frameworks.reduce(
      (sum, f) => sum + f.requirements.filter(r => r.status === 'partial').length, 0
    )

    const overallCompliance = totalRequirements > 0 ? (compliantRequirements / totalRequirements) * 100 : 0

    return {
      totalRequirements,
      compliantRequirements,
      nonCompliantRequirements,
      partialRequirements,
      overallCompliance,
      frameworksCount: frameworks.length,
      compliantFrameworks: frameworks.filter(f => f.status === 'compliant').length,
    }
  }, [frameworks])

  // Framework table columns
  const frameworkColumns = [
    {
      accessorKey: 'name',
      header: t('compliance.framework'),
      cell: ({ row }: any) => (
        <div className="flex items-center space-x-2">
          <ShieldCheckIcon className="h-4 w-4 text-blue-500" />
          <div>
            <div className="font-medium">{row.original.name}</div>
            <div className="text-sm text-gray-500">{row.original.version}</div>
          </div>
        </div>
      ),
    },
    {
      accessorKey: 'status',
      header: t('compliance.status'),
      cell: ({ row }: any) => {
        const statusColors = {
          compliant: 'success',
          non_compliant: 'danger',
          partial: 'warning',
          pending: 'secondary',
        }
        return (
          <Badge variant={statusColors[row.original.status as keyof typeof statusColors] as any}>
            {t(`compliance.status.${row.original.status}`)}
          </Badge>
        )
      },
    },
    {
      accessorKey: 'requirements',
      header: t('compliance.requirements'),
      cell: ({ row }: any) => {
        const total = row.original.requirements.length
        const compliant = row.original.requirements.filter((r: any) => r.status === 'compliant').length
        const percentage = total > 0 ? Math.round((compliant / total) * 100) : 0
        
        return (
          <div className="flex items-center space-x-2">
            <div className="w-16 bg-gray-200 rounded-full h-2">
              <div
                className={`h-2 rounded-full ${
                  percentage >= 90 ? 'bg-green-500' :
                  percentage >= 70 ? 'bg-yellow-500' : 'bg-red-500'
                }`}
                style={{ width: `${percentage}%` }}
              />
            </div>
            <span className="text-sm font-mono">{compliant}/{total}</span>
          </div>
        )
      },
    },
    {
      accessorKey: 'lastAssessment',
      header: t('compliance.lastAssessment'),
      cell: ({ row }: any) => (
        <span className="text-sm">
          {new Date(row.original.lastAssessment).toLocaleDateString()}
        </span>
      ),
    },
    {
      accessorKey: 'nextAssessment',
      header: t('compliance.nextAssessment'),
      cell: ({ row }: any) => {
        const nextDate = new Date(row.original.nextAssessment)
        const isOverdue = nextDate < new Date()
        
        return (
          <div className={`text-sm ${isOverdue ? 'text-red-600 font-medium' : ''}`}>
            {nextDate.toLocaleDateString()}
            {isOverdue && (
              <ExclamationTriangleIcon className="inline h-4 w-4 ml-1" />
            )}
          </div>
        )
      },
    },
    {
      id: 'actions',
      header: t('actions.title'),
      cell: ({ row }: any) => (
        <div className="flex items-center space-x-2">
          <Button
            size="xs"
            variant="ghost"
            icon={EyeIcon}
            onClick={() => setSelectedFramework(row.original)}
          >
            {t('actions.view')}
          </Button>
          <Button
            size="xs"
            variant="ghost"
            icon={ArrowDownTrayIcon}
            onClick={() => generateComplianceReport(row.original.id, 'pdf')}
          >
            {t('actions.report')}
          </Button>
        </div>
      ),
    },
  ]

  // Requirements table columns
  const requirementColumns = [
    {
      accessorKey: 'code',
      header: t('compliance.code'),
      cell: ({ row }: any) => (
        <span className="font-mono text-sm">{row.original.code}</span>
      ),
    },
    {
      accessorKey: 'title',
      header: t('compliance.requirement'),
      cell: ({ row }: any) => (
        <div className="max-w-xs">
          <div className="font-medium truncate">{row.original.title}</div>
          <div className="text-sm text-gray-500 truncate">{row.original.category}</div>
        </div>
      ),
    },
    {
      accessorKey: 'priority',
      header: t('compliance.priority'),
      cell: ({ row }: any) => {
        const priorityColors = {
          low: 'secondary',
          medium: 'warning',
          high: 'danger',
          critical: 'danger',
        }
        return (
          <Badge variant={priorityColors[row.original.priority as keyof typeof priorityColors] as any}>
            {t(`compliance.priority.${row.original.priority}`)}
          </Badge>
        )
      },
    },
    {
      accessorKey: 'status',
      header: t('compliance.status'),
      cell: ({ row }: any) => {
        const statusIcons = {
          compliant: CheckCircleIcon,
          non_compliant: XCircleIcon,
          partial: ExclamationTriangleIcon,
          not_assessed: ClockIcon,
        }
        const StatusIcon = statusIcons[row.original.status as keyof typeof statusIcons]
        const statusColors = {
          compliant: 'text-green-600',
          non_compliant: 'text-red-600',
          partial: 'text-yellow-600',
          not_assessed: 'text-gray-400',
        }
        
        return (
          <div className="flex items-center space-x-2">
            <StatusIcon className={`h-4 w-4 ${statusColors[row.original.status as keyof typeof statusColors]}`} />
            <span className="text-sm">{t(`compliance.status.${row.original.status}`)}</span>
          </div>
        )
      },
    },
    {
      accessorKey: 'evidence',
      header: t('compliance.evidence'),
      cell: ({ row }: any) => {
        const evidenceCount = row.original.evidence.length
        const verifiedCount = row.original.evidence.filter((e: any) => e.verified).length
        
        return (
          <div className="flex items-center space-x-2">
            <DocumentTextIcon className="h-4 w-4 text-gray-400" />
            <span className="text-sm">{verifiedCount}/{evidenceCount}</span>
          </div>
        )
      },
    },
    {
      accessorKey: 'assignedTo',
      header: t('compliance.assignedTo'),
      cell: ({ row }: any) => (
        <div className="flex items-center space-x-2">
          <UserIcon className="h-4 w-4 text-gray-400" />
          <span className="text-sm">{row.original.assignedTo}</span>
        </div>
      ),
    },
    {
      id: 'actions',
      header: t('actions.title'),
      cell: ({ row }: any) => (
        <div className="flex items-center space-x-2">
          <Button
            size="xs"
            variant="ghost"
            icon={EyeIcon}
            onClick={() => setSelectedRequirement(row.original)}
          >
            {t('actions.view')}
          </Button>
          {permissions.includes('compliance:manage') && (
            <Button
              size="xs"
              variant="ghost"
              icon={DocumentTextIcon}
              onClick={() => {
                setSelectedRequirement(row.original)
                setEvidenceModalOpen(true)
              }}
            >
              {t('actions.evidence')}
            </Button>
          )}
        </div>
      ),
    },
  ]

  // Audit log columns
  const auditColumns = [
    {
      accessorKey: 'timestamp',
      header: t('audit.timestamp'),
      cell: ({ row }: any) => (
        <span className="text-sm font-mono">
          {new Date(row.original.timestamp).toLocaleString()}
        </span>
      ),
    },
    {
      accessorKey: 'action',
      header: t('audit.action'),
      cell: ({ row }: any) => (
        <Badge variant="secondary">{row.original.action}</Badge>
      ),
    },
    {
      accessorKey: 'entity',
      header: t('audit.entity'),
      cell: ({ row }: any) => (
        <div>
          <div className="font-medium">{row.original.entity}</div>
          <div className="text-sm text-gray-500 font-mono">{row.original.entityId.slice(0, 8)}...</div>
        </div>
      ),
    },
    {
      accessorKey: 'actor',
      header: t('audit.actor'),
      cell: ({ row }: any) => (
        <div className="flex items-center space-x-2">
          <UserIcon className="h-4 w-4 text-gray-400" />
          <span className="text-sm">{row.original.actor}</span>
        </div>
      ),
    },
    {
      accessorKey: 'signature',
      header: t('audit.signature'),
      cell: ({ row }: any) => (
        <div className="flex items-center space-x-2">
          <ShieldCheckIcon className="h-4 w-4 text-green-500" />
          <span className="text-xs font-mono">{row.original.signature.slice(0, 16)}...</span>
        </div>
      ),
    },
    {
      id: 'actions',
      header: t('actions.title'),
      cell: ({ row }: any) => (
        <Button
          size="xs"
          variant="ghost"
          icon={EyeIcon}
          onClick={() => {
            // Show audit details modal
            setAuditModalOpen(true)
          }}
        >
          {t('actions.details')}
        </Button>
      ),
    },
  ]

  return (
    <>
      <Helmet>
        <title>{t('pages.compliance')} - AtlasMesh Fleet OS</title>
      </Helmet>

      <div className="space-y-6">
        {/* Header */}
        <div className="flex items-center justify-between">
          <div>
            <h1 className="text-3xl font-bold text-gray-900 dark:text-white">
              {t('pages.compliance')}
            </h1>
            <p className="text-sm text-gray-500 dark:text-gray-400">
              {t('compliance.description')}
            </p>
          </div>

          <div className="flex items-center space-x-3">
            <Button
              variant="secondary"
              icon={ArrowDownTrayIcon}
              onClick={() => setReportModalOpen(true)}
            >
              {t('compliance.generateReport')}
            </Button>
            {permissions.includes('compliance:manage') && (
              <Button
                variant="primary"
                icon={DocumentTextIcon}
                onClick={() => setEvidenceModalOpen(true)}
              >
                {t('compliance.uploadEvidence')}
              </Button>
            )}
          </div>
        </div>

        {/* Compliance Metrics */}
        {complianceMetrics && (
          <div className="grid grid-cols-1 md:grid-cols-4 gap-6">
            <Card>
              <CardContent className="p-6">
                <div className="flex items-center justify-between">
                  <div>
                    <p className="text-sm font-medium text-gray-600 dark:text-gray-400">
                      {t('compliance.metrics.overallCompliance')}
                    </p>
                    <p className="text-3xl font-bold text-gray-900 dark:text-white">
                      {Math.round(complianceMetrics.overallCompliance)}%
                    </p>
                  </div>
                  <ChartBarIcon className="h-8 w-8 text-blue-500" />
                </div>
                <Progress 
                  value={complianceMetrics.overallCompliance} 
                  className="mt-3"
                />
              </CardContent>
            </Card>

            <Card>
              <CardContent className="p-6">
                <div className="flex items-center justify-between">
                  <div>
                    <p className="text-sm font-medium text-gray-600 dark:text-gray-400">
                      {t('compliance.metrics.compliantRequirements')}
                    </p>
                    <p className="text-3xl font-bold text-green-600">
                      {complianceMetrics.compliantRequirements}
                    </p>
                  </div>
                  <CheckCircleIcon className="h-8 w-8 text-green-500" />
                </div>
                <p className="text-sm text-gray-500 mt-1">
                  {t('compliance.metrics.outOf', { total: complianceMetrics.totalRequirements })}
                </p>
              </CardContent>
            </Card>

            <Card>
              <CardContent className="p-6">
                <div className="flex items-center justify-between">
                  <div>
                    <p className="text-sm font-medium text-gray-600 dark:text-gray-400">
                      {t('compliance.metrics.nonCompliantRequirements')}
                    </p>
                    <p className="text-3xl font-bold text-red-600">
                      {complianceMetrics.nonCompliantRequirements}
                    </p>
                  </div>
                  <XCircleIcon className="h-8 w-8 text-red-500" />
                </div>
                <p className="text-sm text-gray-500 mt-1">
                  {t('compliance.metrics.requireAttention')}
                </p>
              </CardContent>
            </Card>

            <Card>
              <CardContent className="p-6">
                <div className="flex items-center justify-between">
                  <div>
                    <p className="text-sm font-medium text-gray-600 dark:text-gray-400">
                      {t('compliance.metrics.frameworks')}
                    </p>
                    <p className="text-3xl font-bold text-gray-900 dark:text-white">
                      {complianceMetrics.compliantFrameworks}/{complianceMetrics.frameworksCount}
                    </p>
                  </div>
                  <ShieldCheckIcon className="h-8 w-8 text-blue-500" />
                </div>
                <p className="text-sm text-gray-500 mt-1">
                  {t('compliance.metrics.compliantFrameworks')}
                </p>
              </CardContent>
            </Card>
          </div>
        )}

        {/* Filters */}
        <Card>
          <CardContent className="p-4">
            <div className="grid grid-cols-1 md:grid-cols-5 gap-4">
              <Input
                placeholder={t('compliance.search.placeholder')}
                value={searchTerm}
                onChange={(e) => setSearchTerm(e.target.value)}
              />
              
              <Select
                value={frameworkFilter}
                onChange={setFrameworkFilter}
                options={[
                  { value: 'all', label: t('filters.allFrameworks') },
                  { value: 'gdpr', label: 'GDPR' },
                  { value: 'ccpa', label: 'CCPA' },
                  { value: 'iso27001', label: 'ISO 27001' },
                  { value: 'soc2', label: 'SOC 2' },
                  { value: 'nist', label: 'NIST' },
                ]}
              />
              
              <Select
                value={statusFilter}
                onChange={setStatusFilter}
                options={[
                  { value: 'all', label: t('filters.allStatuses') },
                  { value: 'compliant', label: t('compliance.status.compliant') },
                  { value: 'non_compliant', label: t('compliance.status.non_compliant') },
                  { value: 'partial', label: t('compliance.status.partial') },
                  { value: 'pending', label: t('compliance.status.pending') },
                ]}
              />
              
              <Select
                value={priorityFilter}
                onChange={setPriorityFilter}
                options={[
                  { value: 'all', label: t('filters.allPriorities') },
                  { value: 'critical', label: t('compliance.priority.critical') },
                  { value: 'high', label: t('compliance.priority.high') },
                  { value: 'medium', label: t('compliance.priority.medium') },
                  { value: 'low', label: t('compliance.priority.low') },
                ]}
              />
              
              <DatePicker
                placeholder={t('compliance.dateRange')}
                value={dateRange}
                onChange={setDateRange}
              />
            </div>
          </CardContent>
        </Card>

        {/* Main Content Tabs */}
        <Tabs value={activeTab} onValueChange={(value) => setActiveTab(value as any)}>
          <TabsList>
            <TabsTrigger value="frameworks">
              <ShieldCheckIcon className="h-4 w-4 mr-2" />
              {t('compliance.tabs.frameworks')}
            </TabsTrigger>
            <TabsTrigger value="requirements">
              <DocumentTextIcon className="h-4 w-4 mr-2" />
              {t('compliance.tabs.requirements')}
            </TabsTrigger>
            <TabsTrigger value="evidence">
              <DocumentTextIcon className="h-4 w-4 mr-2" />
              {t('compliance.tabs.evidence')}
            </TabsTrigger>
            <TabsTrigger value="audits">
              <ClockIcon className="h-4 w-4 mr-2" />
              {t('compliance.tabs.audits')}
            </TabsTrigger>
          </TabsList>

          <TabsContent value="frameworks">
            <Card>
              <CardHeader>
                <CardTitle>{t('compliance.frameworks.title')}</CardTitle>
              </CardHeader>
              <CardContent>
                <DataTable
                  columns={frameworkColumns}
                  data={frameworks || []}
                  loading={isLoading}
                />
              </CardContent>
            </Card>
          </TabsContent>

          <TabsContent value="requirements">
            <Card>
              <CardHeader>
                <CardTitle>{t('compliance.requirements.title')}</CardTitle>
              </CardHeader>
              <CardContent>
                <DataTable
                  columns={requirementColumns}
                  data={requirements || []}
                  loading={isLoading}
                />
              </CardContent>
            </Card>
          </TabsContent>

          <TabsContent value="evidence">
            <Card>
              <CardHeader>
                <CardTitle>{t('compliance.evidence.title')}</CardTitle>
              </CardHeader>
              <CardContent>
                <div className="text-center py-8">
                  <DocumentTextIcon className="mx-auto h-12 w-12 text-gray-400" />
                  <h3 className="mt-2 text-sm font-medium text-gray-900 dark:text-white">
                    {t('compliance.evidence.empty')}
                  </h3>
                  <p className="mt-1 text-sm text-gray-500">
                    {t('compliance.evidence.emptyDescription')}
                  </p>
                  <div className="mt-6">
                    <Button
                      variant="primary"
                      icon={DocumentTextIcon}
                      onClick={() => setEvidenceModalOpen(true)}
                    >
                      {t('compliance.uploadEvidence')}
                    </Button>
                  </div>
                </div>
              </CardContent>
            </Card>
          </TabsContent>

          <TabsContent value="audits">
            <Card>
              <CardHeader>
                <CardTitle>{t('compliance.audits.title')}</CardTitle>
              </CardHeader>
              <CardContent>
                <DataTable
                  columns={auditColumns}
                  data={auditLogs || []}
                  loading={isLoading}
                />
              </CardContent>
            </Card>
          </TabsContent>
        </Tabs>
      </div>

      {/* Evidence Upload Modal */}
      <Modal
        open={evidenceModalOpen}
        onClose={() => setEvidenceModalOpen(false)}
        title={t('compliance.uploadEvidence')}
        size="lg"
      >
        <div className="space-y-4">
          <p className="text-sm text-gray-600 dark:text-gray-400">
            {t('compliance.evidence.uploadDescription')}
          </p>
          
          <div className="border-2 border-dashed border-gray-300 dark:border-gray-600 rounded-lg p-6">
            <div className="text-center">
              <DocumentTextIcon className="mx-auto h-12 w-12 text-gray-400" />
              <div className="mt-4">
                <label htmlFor="file-upload" className="cursor-pointer">
                  <span className="mt-2 block text-sm font-medium text-gray-900 dark:text-white">
                    {t('compliance.evidence.dropFiles')}
                  </span>
                  <input id="file-upload" name="file-upload" type="file" className="sr-only" multiple />
                </label>
                <p className="mt-1 text-xs text-gray-500">
                  {t('compliance.evidence.supportedFormats')}
                </p>
              </div>
            </div>
          </div>

          <div className="flex justify-end space-x-3 pt-4 border-t">
            <Button
              variant="secondary"
              onClick={() => setEvidenceModalOpen(false)}
            >
              {t('actions.cancel')}
            </Button>
            <Button variant="primary">
              {t('actions.upload')}
            </Button>
          </div>
        </div>
      </Modal>

      {/* Report Generation Modal */}
      <Modal
        open={reportModalOpen}
        onClose={() => setReportModalOpen(false)}
        title={t('compliance.generateReport')}
      >
        <div className="space-y-4">
          <p className="text-sm text-gray-600 dark:text-gray-400">
            {t('compliance.report.description')}
          </p>

          <div className="space-y-3">
            <Button
              variant="secondary"
              className="w-full justify-start"
              onClick={() => generateComplianceReport('all', 'pdf')}
            >
              <DocumentTextIcon className="h-4 w-4 mr-2" />
              {t('compliance.report.comprehensive')}
            </Button>
            
            <Button
              variant="secondary"
              className="w-full justify-start"
              onClick={() => generateComplianceReport('summary', 'pdf')}
            >
              <ChartBarIcon className="h-4 w-4 mr-2" />
              {t('compliance.report.summary')}
            </Button>
            
            <Button
              variant="secondary"
              className="w-full justify-start"
              onClick={() => generateComplianceReport('audit', 'pdf')}
            >
              <ClockIcon className="h-4 w-4 mr-2" />
              {t('compliance.report.auditTrail')}
            </Button>
          </div>
        </div>
      </Modal>
    </>
  )
}
