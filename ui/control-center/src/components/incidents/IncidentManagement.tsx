import React, { useState, useEffect } from 'react'
import { useTranslation } from 'react-i18next'
import {
  ExclamationTriangleIcon,
  DocumentTextIcon,
  VideoCameraIcon,
  MapPinIcon,
  ClockIcon,
  UserIcon,
  TagIcon,
  ArrowDownTrayIcon,
  EyeIcon,
  PencilIcon,
  CheckCircleIcon,
  XCircleIcon,
} from '@heroicons/react/24/outline'

// Components
import { Button } from '@components/ui/Button'
import { Badge } from '@components/ui/Badge'
import { Card, CardContent, CardHeader, CardTitle } from '@components/ui/Card'
import { Tabs, TabsContent, TabsList, TabsTrigger } from '@components/ui/Tabs'
import { Input } from '@components/ui/Input'
import { Textarea } from '@components/ui/Textarea'
import { Select } from '@components/ui/Select'
import { DataTable } from '@components/ui/DataTable'
import { Modal } from '@components/ui/Modal'
import { Timeline } from '@components/ui/Timeline'

// Hooks
import { useIncidents } from '@hooks/useIncidents'
import { useAuth } from '@hooks/useAuth'

// Types
interface IncidentManagementProps {
  className?: string
}

interface Incident {
  incidentId: string
  title: string
  description: string
  severity: 'low' | 'medium' | 'high' | 'critical'
  category: string
  status: 'new' | 'investigating' | 'resolved' | 'closed'
  vehicleId?: string
  tripId?: string
  location?: { lat: number; lng: number; address?: string }
  reportedBy: string
  reportedAt: string
  assignedTo?: string
  resolvedAt?: string
  tags: string[]
  evidence: EvidenceItem[]
  timeline: TimelineEvent[]
  rootCause?: string
  correctiveActions: CorrectiveAction[]
  preventiveActions: PreventiveAction[]
}

interface EvidenceItem {
  id: string
  type: 'video' | 'image' | 'telemetry' | 'log' | 'document'
  name: string
  url: string
  timestamp: string
  duration?: number // for videos
  size: number
  hash: string
}

interface TimelineEvent {
  id: string
  timestamp: string
  event: string
  description: string
  actor: string
  data?: any
}

interface CorrectiveAction {
  id: string
  description: string
  assignedTo: string
  dueDate: string
  status: 'pending' | 'in_progress' | 'completed'
  completedAt?: string
}

interface PreventiveAction {
  id: string
  description: string
  assignedTo: string
  dueDate: string
  status: 'pending' | 'in_progress' | 'completed'
  completedAt?: string
}

export function IncidentManagement({ className = '' }: IncidentManagementProps) {
  const { t } = useTranslation()
  const { user, permissions } = useAuth()

  // State
  const [selectedIncident, setSelectedIncident] = useState<Incident | null>(null)
  const [incidentModalOpen, setIncidentModalOpen] = useState(false)
  const [evidenceModalOpen, setEvidenceModalOpen] = useState(false)
  const [exportModalOpen, setExportModalOpen] = useState(false)
  const [searchTerm, setSearchTerm] = useState('')
  const [severityFilter, setSeverityFilter] = useState('all')
  const [statusFilter, setStatusFilter] = useState('all')
  const [categoryFilter, setCategoryFilter] = useState('all')
  const [assigneeFilter, setAssigneeFilter] = useState('all')

  // Form state for new/edit incident
  const [incidentForm, setIncidentForm] = useState({
    title: '',
    description: '',
    severity: 'medium' as const,
    category: '',
    vehicleId: '',
    tripId: '',
    location: '',
    tags: [] as string[],
    rootCause: '',
  })

  // Data fetching
  const {
    data: incidents,
    isLoading,
    error,
    createIncident,
    updateIncident,
    deleteIncident,
    exportIncident,
  } = useIncidents({
    search: searchTerm,
    severity: severityFilter === 'all' ? undefined : severityFilter,
    status: statusFilter === 'all' ? undefined : statusFilter,
    category: categoryFilter === 'all' ? undefined : categoryFilter,
    assignee: assigneeFilter === 'all' ? undefined : assigneeFilter,
  })

  // Filter options
  const severityOptions = [
    { value: 'all', label: t('filters.allSeverities') },
    { value: 'low', label: t('incident.severity.low') },
    { value: 'medium', label: t('incident.severity.medium') },
    { value: 'high', label: t('incident.severity.high') },
    { value: 'critical', label: t('incident.severity.critical') },
  ]

  const statusOptions = [
    { value: 'all', label: t('filters.allStatuses') },
    { value: 'new', label: t('incident.status.new') },
    { value: 'investigating', label: t('incident.status.investigating') },
    { value: 'resolved', label: t('incident.status.resolved') },
    { value: 'closed', label: t('incident.status.closed') },
  ]

  const categoryOptions = [
    { value: 'all', label: t('filters.allCategories') },
    { value: 'safety', label: t('incident.category.safety') },
    { value: 'technical', label: t('incident.category.technical') },
    { value: 'operational', label: t('incident.category.operational') },
    { value: 'compliance', label: t('incident.category.compliance') },
    { value: 'security', label: t('incident.category.security') },
  ]

  // Severity colors
  const severityColors = {
    low: 'bg-blue-100 text-blue-800 dark:bg-blue-900 dark:text-blue-300',
    medium: 'bg-yellow-100 text-yellow-800 dark:bg-yellow-900 dark:text-yellow-300',
    high: 'bg-orange-100 text-orange-800 dark:bg-orange-900 dark:text-orange-300',
    critical: 'bg-red-100 text-red-800 dark:bg-red-900 dark:text-red-300',
  }

  const statusColors = {
    new: 'bg-gray-100 text-gray-800 dark:bg-gray-900 dark:text-gray-300',
    investigating: 'bg-blue-100 text-blue-800 dark:bg-blue-900 dark:text-blue-300',
    resolved: 'bg-green-100 text-green-800 dark:bg-green-900 dark:text-green-300',
    closed: 'bg-gray-100 text-gray-800 dark:bg-gray-900 dark:text-gray-300',
  }

  // Table columns
  const columns = [
    {
      accessorKey: 'incidentId',
      header: t('incident.id'),
      cell: ({ row }: any) => (
        <button
          onClick={() => setSelectedIncident(row.original)}
          className="text-blue-600 hover:text-blue-500 font-mono text-sm"
        >
          {row.original.incidentId.slice(0, 8)}...
        </button>
      ),
    },
    {
      accessorKey: 'title',
      header: t('incident.title'),
      cell: ({ row }: any) => (
        <div className="max-w-xs truncate" title={row.original.title}>
          {row.original.title}
        </div>
      ),
    },
    {
      accessorKey: 'severity',
      header: t('incident.severity'),
      cell: ({ row }: any) => (
        <Badge className={severityColors[row.original.severity as keyof typeof severityColors]}>
          {t(`incident.severity.${row.original.severity}`)}
        </Badge>
      ),
    },
    {
      accessorKey: 'status',
      header: t('incident.status'),
      cell: ({ row }: any) => (
        <Badge className={statusColors[row.original.status as keyof typeof statusColors]}>
          {t(`incident.status.${row.original.status}`)}
        </Badge>
      ),
    },
    {
      accessorKey: 'category',
      header: t('incident.category'),
      cell: ({ row }: any) => (
        <Badge variant="secondary">
          {t(`incident.category.${row.original.category}`)}
        </Badge>
      ),
    },
    {
      accessorKey: 'vehicleId',
      header: t('incident.vehicle'),
      cell: ({ row }: any) => (
        row.original.vehicleId ? (
          <span className="font-mono text-sm">{row.original.vehicleId.slice(0, 8)}</span>
        ) : (
          <span className="text-gray-400">-</span>
        )
      ),
    },
    {
      accessorKey: 'reportedAt',
      header: t('incident.reportedAt'),
      cell: ({ row }: any) => (
        <span className="text-sm">
          {new Date(row.original.reportedAt).toLocaleString()}
        </span>
      ),
    },
    {
      accessorKey: 'assignedTo',
      header: t('incident.assignedTo'),
      cell: ({ row }: any) => (
        row.original.assignedTo ? (
          <span className="text-sm">{row.original.assignedTo}</span>
        ) : (
          <span className="text-gray-400">{t('incident.unassigned')}</span>
        )
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
            onClick={() => setSelectedIncident(row.original)}
          >
            {t('actions.view')}
          </Button>
          {permissions.includes('incidents:edit') && (
            <Button
              size="xs"
              variant="ghost"
              icon={PencilIcon}
              onClick={() => {
                setIncidentForm({
                  title: row.original.title,
                  description: row.original.description,
                  severity: row.original.severity,
                  category: row.original.category,
                  vehicleId: row.original.vehicleId || '',
                  tripId: row.original.tripId || '',
                  location: row.original.location?.address || '',
                  tags: row.original.tags,
                  rootCause: row.original.rootCause || '',
                })
                setSelectedIncident(row.original)
                setIncidentModalOpen(true)
              }}
            >
              {t('actions.edit')}
            </Button>
          )}
        </div>
      ),
    },
  ]

  // Handle form submission
  const handleSubmit = async () => {
    try {
      if (selectedIncident) {
        await updateIncident(selectedIncident.incidentId, incidentForm)
      } else {
        await createIncident({
          ...incidentForm,
          reportedBy: user?.id || 'unknown',
          reportedAt: new Date().toISOString(),
          status: 'new',
          timeline: [],
          evidence: [],
          correctiveActions: [],
          preventiveActions: [],
        })
      }
      setIncidentModalOpen(false)
      setSelectedIncident(null)
      resetForm()
    } catch (error) {
      console.error('Failed to save incident:', error)
    }
  }

  const resetForm = () => {
    setIncidentForm({
      title: '',
      description: '',
      severity: 'medium',
      category: '',
      vehicleId: '',
      tripId: '',
      location: '',
      tags: [],
      rootCause: '',
    })
  }

  // Handle evidence export
  const handleExportEvidence = async (format: 'pdf' | 'json' | 'zip') => {
    if (!selectedIncident) return

    try {
      await exportIncident(selectedIncident.incidentId, format)
      setExportModalOpen(false)
    } catch (error) {
      console.error('Failed to export evidence:', error)
    }
  }

  return (
    <div className={`space-y-6 ${className}`}>
      {/* Header */}
      <div className="flex items-center justify-between">
        <div>
          <h2 className="text-2xl font-bold text-gray-900 dark:text-white">
            {t('incidents.title')}
          </h2>
          <p className="text-sm text-gray-500 dark:text-gray-400">
            {t('incidents.description')}
          </p>
        </div>

        {permissions.includes('incidents:create') && (
          <Button
            variant="primary"
            icon={DocumentTextIcon}
            onClick={() => {
              resetForm()
              setSelectedIncident(null)
              setIncidentModalOpen(true)
            }}
          >
            {t('incidents.create')}
          </Button>
        )}
      </div>

      {/* Filters */}
      <Card>
        <CardContent className="p-4">
          <div className="grid grid-cols-1 md:grid-cols-5 gap-4">
            <Input
              placeholder={t('incidents.search.placeholder')}
              value={searchTerm}
              onChange={(e) => setSearchTerm(e.target.value)}
            />
            
            <Select
              value={severityFilter}
              onChange={setSeverityFilter}
              options={severityOptions}
            />
            
            <Select
              value={statusFilter}
              onChange={setStatusFilter}
              options={statusOptions}
            />
            
            <Select
              value={categoryFilter}
              onChange={setCategoryFilter}
              options={categoryOptions}
            />
            
            <Select
              value={assigneeFilter}
              onChange={setAssigneeFilter}
              options={[
                { value: 'all', label: t('filters.allAssignees') },
                { value: 'unassigned', label: t('incident.unassigned') },
                { value: 'me', label: t('incident.assignedToMe') },
              ]}
            />
          </div>
        </CardContent>
      </Card>

      {/* Incidents Table */}
      <Card>
        <CardHeader>
          <CardTitle>{t('incidents.list')}</CardTitle>
        </CardHeader>
        <CardContent>
          <DataTable
            columns={columns}
            data={incidents || []}
            loading={isLoading}
          />
        </CardContent>
      </Card>

      {/* Incident Detail Modal */}
      <Modal
        open={!!selectedIncident && !incidentModalOpen}
        onClose={() => setSelectedIncident(null)}
        title={selectedIncident?.title || ''}
        size="xl"
      >
        {selectedIncident && (
          <div className="space-y-6">
            {/* Header */}
            <div className="flex items-start justify-between">
              <div className="space-y-2">
                <div className="flex items-center space-x-2">
                  <Badge className={severityColors[selectedIncident.severity]}>
                    {t(`incident.severity.${selectedIncident.severity}`)}
                  </Badge>
                  <Badge className={statusColors[selectedIncident.status]}>
                    {t(`incident.status.${selectedIncident.status}`)}
                  </Badge>
                  <Badge variant="secondary">
                    {t(`incident.category.${selectedIncident.category}`)}
                  </Badge>
                </div>
                
                <div className="text-sm text-gray-500 space-y-1">
                  <div className="flex items-center space-x-2">
                    <UserIcon className="h-4 w-4" />
                    <span>{t('incident.reportedBy')}: {selectedIncident.reportedBy}</span>
                  </div>
                  <div className="flex items-center space-x-2">
                    <ClockIcon className="h-4 w-4" />
                    <span>{t('incident.reportedAt')}: {new Date(selectedIncident.reportedAt).toLocaleString()}</span>
                  </div>
                  {selectedIncident.location && (
                    <div className="flex items-center space-x-2">
                      <MapPinIcon className="h-4 w-4" />
                      <span>{selectedIncident.location.address || `${selectedIncident.location.lat}, ${selectedIncident.location.lng}`}</span>
                    </div>
                  )}
                </div>
              </div>

              <div className="flex space-x-2">
                {permissions.includes('incidents:export') && (
                  <Button
                    variant="secondary"
                    size="sm"
                    icon={ArrowDownTrayIcon}
                    onClick={() => setExportModalOpen(true)}
                  >
                    {t('actions.export')}
                  </Button>
                )}
                {permissions.includes('incidents:edit') && (
                  <Button
                    variant="primary"
                    size="sm"
                    icon={PencilIcon}
                    onClick={() => {
                      setIncidentForm({
                        title: selectedIncident.title,
                        description: selectedIncident.description,
                        severity: selectedIncident.severity,
                        category: selectedIncident.category,
                        vehicleId: selectedIncident.vehicleId || '',
                        tripId: selectedIncident.tripId || '',
                        location: selectedIncident.location?.address || '',
                        tags: selectedIncident.tags,
                        rootCause: selectedIncident.rootCause || '',
                      })
                      setIncidentModalOpen(true)
                    }}
                  >
                    {t('actions.edit')}
                  </Button>
                )}
              </div>
            </div>

            {/* Tabs */}
            <Tabs defaultValue="overview">
              <TabsList>
                <TabsTrigger value="overview">{t('incident.tabs.overview')}</TabsTrigger>
                <TabsTrigger value="timeline">{t('incident.tabs.timeline')}</TabsTrigger>
                <TabsTrigger value="evidence">{t('incident.tabs.evidence')}</TabsTrigger>
                <TabsTrigger value="actions">{t('incident.tabs.actions')}</TabsTrigger>
              </TabsList>

              <TabsContent value="overview" className="space-y-4">
                <div>
                  <h4 className="font-medium mb-2">{t('incident.description')}</h4>
                  <p className="text-sm text-gray-600 dark:text-gray-400">
                    {selectedIncident.description}
                  </p>
                </div>

                {selectedIncident.rootCause && (
                  <div>
                    <h4 className="font-medium mb-2">{t('incident.rootCause')}</h4>
                    <p className="text-sm text-gray-600 dark:text-gray-400">
                      {selectedIncident.rootCause}
                    </p>
                  </div>
                )}

                {selectedIncident.tags.length > 0 && (
                  <div>
                    <h4 className="font-medium mb-2">{t('incident.tags')}</h4>
                    <div className="flex flex-wrap gap-2">
                      {selectedIncident.tags.map((tag, index) => (
                        <Badge key={index} variant="secondary" size="sm">
                          <TagIcon className="h-3 w-3 mr-1" />
                          {tag}
                        </Badge>
                      ))}
                    </div>
                  </div>
                )}
              </TabsContent>

              <TabsContent value="timeline">
                <Timeline
                  events={selectedIncident.timeline.map(event => ({
                    id: event.id,
                    timestamp: event.timestamp,
                    title: event.event,
                    description: event.description,
                    actor: event.actor,
                  }))}
                />
              </TabsContent>

              <TabsContent value="evidence" className="space-y-4">
                <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-4">
                  {selectedIncident.evidence.map((item) => (
                    <Card key={item.id} className="cursor-pointer hover:shadow-md transition-shadow">
                      <CardContent className="p-4">
                        <div className="flex items-center space-x-3 mb-2">
                          {item.type === 'video' && <VideoCameraIcon className="h-5 w-5 text-blue-500" />}
                          {item.type === 'image' && <DocumentTextIcon className="h-5 w-5 text-green-500" />}
                          {item.type === 'telemetry' && <DocumentTextIcon className="h-5 w-5 text-purple-500" />}
                          {item.type === 'log' && <DocumentTextIcon className="h-5 w-5 text-gray-500" />}
                          {item.type === 'document' && <DocumentTextIcon className="h-5 w-5 text-orange-500" />}
                          
                          <div className="flex-1 min-w-0">
                            <h4 className="font-medium truncate">{item.name}</h4>
                            <p className="text-xs text-gray-500">
                              {new Date(item.timestamp).toLocaleString()}
                            </p>
                          </div>
                        </div>
                        
                        <div className="text-xs text-gray-500 space-y-1">
                          <div>Size: {(item.size / 1024 / 1024).toFixed(2)} MB</div>
                          {item.duration && <div>Duration: {item.duration}s</div>}
                          <div className="font-mono">Hash: {item.hash.slice(0, 16)}...</div>
                        </div>
                      </CardContent>
                    </Card>
                  ))}
                </div>
              </TabsContent>

              <TabsContent value="actions" className="space-y-6">
                {/* Corrective Actions */}
                <div>
                  <h4 className="font-medium mb-4">{t('incident.correctiveActions')}</h4>
                  <div className="space-y-3">
                    {selectedIncident.correctiveActions.map((action) => (
                      <div key={action.id} className="flex items-center justify-between p-3 border rounded-lg">
                        <div className="flex-1">
                          <p className="text-sm">{action.description}</p>
                          <div className="text-xs text-gray-500 mt-1">
                            Assigned to: {action.assignedTo} | Due: {new Date(action.dueDate).toLocaleDateString()}
                          </div>
                        </div>
                        <Badge variant={
                          action.status === 'completed' ? 'success' :
                          action.status === 'in_progress' ? 'warning' : 'secondary'
                        }>
                          {t(`incident.actionStatus.${action.status}`)}
                        </Badge>
                      </div>
                    ))}
                  </div>
                </div>

                {/* Preventive Actions */}
                <div>
                  <h4 className="font-medium mb-4">{t('incident.preventiveActions')}</h4>
                  <div className="space-y-3">
                    {selectedIncident.preventiveActions.map((action) => (
                      <div key={action.id} className="flex items-center justify-between p-3 border rounded-lg">
                        <div className="flex-1">
                          <p className="text-sm">{action.description}</p>
                          <div className="text-xs text-gray-500 mt-1">
                            Assigned to: {action.assignedTo} | Due: {new Date(action.dueDate).toLocaleDateString()}
                          </div>
                        </div>
                        <Badge variant={
                          action.status === 'completed' ? 'success' :
                          action.status === 'in_progress' ? 'warning' : 'secondary'
                        }>
                          {t(`incident.actionStatus.${action.status}`)}
                        </Badge>
                      </div>
                    ))}
                  </div>
                </div>
              </TabsContent>
            </Tabs>
          </div>
        )}
      </Modal>

      {/* Create/Edit Incident Modal */}
      <Modal
        open={incidentModalOpen}
        onClose={() => {
          setIncidentModalOpen(false)
          setSelectedIncident(null)
          resetForm()
        }}
        title={selectedIncident ? t('incidents.edit') : t('incidents.create')}
        size="lg"
      >
        <div className="space-y-4">
          <div>
            <label className="block text-sm font-medium mb-1">{t('incident.title')} *</label>
            <Input
              value={incidentForm.title}
              onChange={(e) => setIncidentForm({ ...incidentForm, title: e.target.value })}
              placeholder={t('incident.titlePlaceholder')}
            />
          </div>

          <div>
            <label className="block text-sm font-medium mb-1">{t('incident.description')} *</label>
            <Textarea
              value={incidentForm.description}
              onChange={(e) => setIncidentForm({ ...incidentForm, description: e.target.value })}
              placeholder={t('incident.descriptionPlaceholder')}
              rows={4}
            />
          </div>

          <div className="grid grid-cols-2 gap-4">
            <div>
              <label className="block text-sm font-medium mb-1">{t('incident.severity')} *</label>
              <Select
                value={incidentForm.severity}
                onChange={(value) => setIncidentForm({ ...incidentForm, severity: value as any })}
                options={severityOptions.slice(1)} // Remove 'all' option
              />
            </div>

            <div>
              <label className="block text-sm font-medium mb-1">{t('incident.category')} *</label>
              <Select
                value={incidentForm.category}
                onChange={(value) => setIncidentForm({ ...incidentForm, category: value })}
                options={categoryOptions.slice(1)} // Remove 'all' option
              />
            </div>
          </div>

          <div className="grid grid-cols-2 gap-4">
            <div>
              <label className="block text-sm font-medium mb-1">{t('incident.vehicleId')}</label>
              <Input
                value={incidentForm.vehicleId}
                onChange={(e) => setIncidentForm({ ...incidentForm, vehicleId: e.target.value })}
                placeholder={t('incident.vehicleIdPlaceholder')}
              />
            </div>

            <div>
              <label className="block text-sm font-medium mb-1">{t('incident.tripId')}</label>
              <Input
                value={incidentForm.tripId}
                onChange={(e) => setIncidentForm({ ...incidentForm, tripId: e.target.value })}
                placeholder={t('incident.tripIdPlaceholder')}
              />
            </div>
          </div>

          <div>
            <label className="block text-sm font-medium mb-1">{t('incident.location')}</label>
            <Input
              value={incidentForm.location}
              onChange={(e) => setIncidentForm({ ...incidentForm, location: e.target.value })}
              placeholder={t('incident.locationPlaceholder')}
            />
          </div>

          {selectedIncident && (
            <div>
              <label className="block text-sm font-medium mb-1">{t('incident.rootCause')}</label>
              <Textarea
                value={incidentForm.rootCause}
                onChange={(e) => setIncidentForm({ ...incidentForm, rootCause: e.target.value })}
                placeholder={t('incident.rootCausePlaceholder')}
                rows={3}
              />
            </div>
          )}

          <div className="flex justify-end space-x-3 pt-4 border-t">
            <Button
              variant="secondary"
              onClick={() => {
                setIncidentModalOpen(false)
                setSelectedIncident(null)
                resetForm()
              }}
            >
              {t('actions.cancel')}
            </Button>
            
            <Button
              variant="primary"
              onClick={handleSubmit}
              disabled={!incidentForm.title || !incidentForm.description || !incidentForm.category}
            >
              {selectedIncident ? t('actions.update') : t('actions.create')}
            </Button>
          </div>
        </div>
      </Modal>

      {/* Export Modal */}
      <Modal
        open={exportModalOpen}
        onClose={() => setExportModalOpen(false)}
        title={t('incidents.export.title')}
      >
        <div className="space-y-4">
          <p className="text-sm text-gray-600 dark:text-gray-400">
            {t('incidents.export.description')}
          </p>

          <div className="space-y-3">
            <Button
              variant="secondary"
              className="w-full justify-start"
              onClick={() => handleExportEvidence('pdf')}
            >
              <DocumentTextIcon className="h-4 w-4 mr-2" />
              {t('incidents.export.pdf')}
            </Button>
            
            <Button
              variant="secondary"
              className="w-full justify-start"
              onClick={() => handleExportEvidence('json')}
            >
              <DocumentTextIcon className="h-4 w-4 mr-2" />
              {t('incidents.export.json')}
            </Button>
            
            <Button
              variant="secondary"
              className="w-full justify-start"
              onClick={() => handleExportEvidence('zip')}
            >
              <ArrowDownTrayIcon className="h-4 w-4 mr-2" />
              {t('incidents.export.evidenceBundle')}
            </Button>
          </div>
        </div>
      </Modal>
    </div>
  )
}
