import React, { useState, useCallback, useEffect, useMemo } from 'react'
import { 
  Wrench, AlertTriangle, Calendar, Clock, DollarSign, User, CheckCircle,
  TrendingUp, TrendingDown, Battery, Thermometer, Gauge, Settings,
  FileText, Package, Truck, MapPin, Phone, Mail, ExternalLink, Plus,
  Edit, Trash2, Eye, Download, Filter, Search, RefreshCw
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
interface MaintenanceRecord {
  id: string
  vehicleId: string
  vehicleName: string
  type: 'scheduled' | 'unscheduled' | 'predictive' | 'emergency'
  category: 'preventive' | 'corrective' | 'predictive' | 'emergency'
  priority: 'low' | 'medium' | 'high' | 'critical'
  status: 'scheduled' | 'in_progress' | 'completed' | 'cancelled' | 'delayed'
  title: string
  description: string
  scheduledDate: Date
  startedDate?: Date
  completedDate?: Date
  estimatedDuration: number // minutes
  actualDuration?: number // minutes
  estimatedCost: number
  actualCost?: number
  assignedTechnician?: string
  facility: string
  workOrders: WorkOrder[]
  parts: PartUsage[]
  faults: VehicleFault[]
  predictiveIndicators?: PredictiveIndicator[]
  notes: string[]
  attachments: MaintenanceAttachment[]
  nextMaintenanceDate?: Date
  mileage?: number
}

interface WorkOrder {
  id: string
  title: string
  description: string
  status: 'pending' | 'in_progress' | 'completed' | 'cancelled'
  assignedTo?: string
  estimatedHours: number
  actualHours?: number
  priority: 'low' | 'medium' | 'high' | 'critical'
  category: string
  createdAt: Date
  completedAt?: Date
}

interface PartUsage {
  id: string
  partNumber: string
  partName: string
  quantity: number
  unitCost: number
  totalCost: number
  supplier: string
  warrantyMonths: number
  installedDate: Date
  serialNumber?: string
  lotNumber?: string
}

interface VehicleFault {
  id: string
  code: string
  description: string
  severity: 'info' | 'warning' | 'error' | 'critical'
  system: string
  detectedAt: Date
  resolvedAt?: Date
  resolution?: string
  recurrent: boolean
  relatedFaults: string[]
}

interface PredictiveIndicator {
  id: string
  component: string
  metric: string
  currentValue: number
  normalRange: { min: number; max: number }
  warningThreshold: number
  criticalThreshold: number
  trend: 'improving' | 'stable' | 'degrading'
  remainingUsefulLife?: number // days
  confidence: number // 0-100%
  lastUpdated: Date
}

interface MaintenanceAttachment {
  id: string
  filename: string
  type: 'photo' | 'document' | 'video' | 'report'
  url: string
  uploadedBy: string
  uploadedAt: Date
  description?: string
}

interface MaintenanceManagerProps {
  vehicleId?: string
  vehicleName?: string
  onMaintenanceScheduled?: (maintenance: MaintenanceRecord) => void
  className?: string
}

const MaintenanceManager: React.FC<MaintenanceManagerProps> = ({
  vehicleId,
  vehicleName,
  onMaintenanceScheduled,
  className = ''
}) => {
  // State
  const [maintenanceRecords, setMaintenanceRecords] = useState<MaintenanceRecord[]>([])
  const [showScheduleDialog, setShowScheduleDialog] = useState(false)
  const [showDetailDialog, setShowDetailDialog] = useState(false)
  const [selectedRecord, setSelectedRecord] = useState<MaintenanceRecord | null>(null)
  const [activeTab, setActiveTab] = useState('overview')
  const [filters, setFilters] = useState({
    status: '',
    type: '',
    priority: '',
    search: ''
  })
  const [newMaintenance, setNewMaintenance] = useState({
    type: 'scheduled' as const,
    category: 'preventive' as const,
    priority: 'medium' as const,
    title: '',
    description: '',
    scheduledDate: '',
    estimatedDuration: 120,
    estimatedCost: 0,
    facility: 'main_depot'
  })

  // Mock data
  const mockRecords: MaintenanceRecord[] = [
    {
      id: 'maint-001',
      vehicleId: 'v1',
      vehicleName: 'Atlas-001',
      type: 'predictive',
      category: 'predictive',
      priority: 'high',
      status: 'scheduled',
      title: 'Brake Pad Replacement (Predictive)',
      description: 'Predictive maintenance indicates brake pads will reach minimum thickness within 500km',
      scheduledDate: new Date(Date.now() + 3 * 24 * 60 * 60 * 1000),
      estimatedDuration: 180,
      estimatedCost: 450,
      assignedTechnician: 'Ahmed Al-Rashid',
      facility: 'Main Depot',
      workOrders: [
        {
          id: 'wo-001',
          title: 'Replace Front Brake Pads',
          description: 'Replace worn brake pads on front axle',
          status: 'pending',
          estimatedHours: 2,
          priority: 'high',
          category: 'Brakes',
          createdAt: new Date()
        },
        {
          id: 'wo-002',
          title: 'Brake System Inspection',
          description: 'Inspect brake rotors and fluid levels',
          status: 'pending',
          estimatedHours: 1,
          priority: 'medium',
          category: 'Brakes',
          createdAt: new Date()
        }
      ],
      parts: [
        {
          id: 'part-001',
          partNumber: 'BP-F-2024-001',
          partName: 'Front Brake Pads (Set)',
          quantity: 1,
          unitCost: 180,
          totalCost: 180,
          supplier: 'Brembo Automotive',
          warrantyMonths: 12,
          installedDate: new Date()
        },
        {
          id: 'part-002',
          partNumber: 'BF-DOT4-500ML',
          partName: 'Brake Fluid DOT4 (500ml)',
          quantity: 1,
          unitCost: 25,
          totalCost: 25,
          supplier: 'Motul',
          warrantyMonths: 24,
          installedDate: new Date()
        }
      ],
      faults: [
        {
          id: 'fault-001',
          code: 'BRK-001',
          description: 'Brake pad thickness below warning threshold',
          severity: 'warning',
          system: 'Braking System',
          detectedAt: new Date(Date.now() - 2 * 24 * 60 * 60 * 1000),
          recurrent: false,
          relatedFaults: []
        }
      ],
      predictiveIndicators: [
        {
          id: 'pred-001',
          component: 'Front Brake Pads',
          metric: 'Thickness (mm)',
          currentValue: 3.2,
          normalRange: { min: 8, max: 12 },
          warningThreshold: 4,
          criticalThreshold: 2,
          trend: 'degrading',
          remainingUsefulLife: 15,
          confidence: 92,
          lastUpdated: new Date()
        },
        {
          id: 'pred-002',
          component: 'Brake Rotors',
          metric: 'Surface Roughness (μm)',
          currentValue: 2.1,
          normalRange: { min: 0.5, max: 1.5 },
          warningThreshold: 2.0,
          criticalThreshold: 3.0,
          trend: 'stable',
          confidence: 78,
          lastUpdated: new Date()
        }
      ],
      notes: [
        'Predictive model indicates accelerated wear due to urban driving conditions',
        'Schedule during low-demand period to minimize service disruption'
      ],
      attachments: [],
      nextMaintenanceDate: new Date(Date.now() + 90 * 24 * 60 * 60 * 1000),
      mileage: 45230
    },
    {
      id: 'maint-002',
      vehicleId: 'v1',
      vehicleName: 'Atlas-001',
      type: 'scheduled',
      category: 'preventive',
      priority: 'medium',
      status: 'completed',
      title: 'Monthly Safety Inspection',
      description: 'Routine monthly safety and systems check',
      scheduledDate: new Date(Date.now() - 7 * 24 * 60 * 60 * 1000),
      startedDate: new Date(Date.now() - 7 * 24 * 60 * 60 * 1000),
      completedDate: new Date(Date.now() - 7 * 24 * 60 * 60 * 1000 + 2 * 60 * 60 * 1000),
      estimatedDuration: 90,
      actualDuration: 120,
      estimatedCost: 150,
      actualCost: 175,
      assignedTechnician: 'Sarah Johnson',
      facility: 'Main Depot',
      workOrders: [
        {
          id: 'wo-003',
          title: 'Safety Systems Check',
          description: 'Test all safety-critical systems',
          status: 'completed',
          assignedTo: 'Sarah Johnson',
          estimatedHours: 1.5,
          actualHours: 2,
          priority: 'high',
          category: 'Safety',
          createdAt: new Date(Date.now() - 8 * 24 * 60 * 60 * 1000),
          completedAt: new Date(Date.now() - 7 * 24 * 60 * 60 * 1000)
        }
      ],
      parts: [],
      faults: [],
      notes: [
        'All safety systems passed inspection',
        'Minor software update applied to collision avoidance system'
      ],
      attachments: [
        {
          id: 'att-001',
          filename: 'safety_inspection_report_001.pdf',
          type: 'report',
          url: '/attachments/safety_inspection_report_001.pdf',
          uploadedBy: 'Sarah Johnson',
          uploadedAt: new Date(Date.now() - 7 * 24 * 60 * 60 * 1000),
          description: 'Complete safety inspection report'
        }
      ],
      nextMaintenanceDate: new Date(Date.now() + 23 * 24 * 60 * 60 * 1000),
      mileage: 44890
    }
  ]

  // Initialize data
  useEffect(() => {
    setMaintenanceRecords(mockRecords)
  }, [])

  // Filtered records
  const filteredRecords = useMemo(() => {
    return maintenanceRecords
      .filter(record => !vehicleId || record.vehicleId === vehicleId)
      .filter(record => !filters.status || record.status === filters.status)
      .filter(record => !filters.type || record.type === filters.type)
      .filter(record => !filters.priority || record.priority === filters.priority)
      .filter(record => {
        if (!filters.search) return true
        const searchTerm = filters.search.toLowerCase()
        return (
          record.title.toLowerCase().includes(searchTerm) ||
          record.description.toLowerCase().includes(searchTerm) ||
          record.vehicleName.toLowerCase().includes(searchTerm)
        )
      })
      .sort((a, b) => {
        // Sort by priority and date
        const priorityOrder = { critical: 4, high: 3, medium: 2, low: 1 }
        const aPriority = priorityOrder[a.priority]
        const bPriority = priorityOrder[b.priority]
        
        if (aPriority !== bPriority) {
          return bPriority - aPriority
        }
        
        return a.scheduledDate.getTime() - b.scheduledDate.getTime()
      })
  }, [maintenanceRecords, vehicleId, filters])

  // Statistics
  const stats = useMemo(() => {
    const records = vehicleId ? 
      maintenanceRecords.filter(r => r.vehicleId === vehicleId) : 
      maintenanceRecords

    const totalRecords = records.length
    const completedRecords = records.filter(r => r.status === 'completed')
    const scheduledRecords = records.filter(r => r.status === 'scheduled')
    const overdueRecords = records.filter(r => 
      r.status === 'scheduled' && r.scheduledDate < new Date()
    )

    const totalCost = completedRecords.reduce((sum, r) => sum + (r.actualCost || r.estimatedCost), 0)
    const avgCost = completedRecords.length > 0 ? totalCost / completedRecords.length : 0

    const totalDuration = completedRecords.reduce((sum, r) => sum + (r.actualDuration || r.estimatedDuration), 0)
    const avgDuration = completedRecords.length > 0 ? totalDuration / completedRecords.length : 0

    const criticalFaults = records.flatMap(r => r.faults).filter(f => f.severity === 'critical').length

    return {
      totalRecords,
      completedRecords: completedRecords.length,
      scheduledRecords: scheduledRecords.length,
      overdueRecords: overdueRecords.length,
      totalCost,
      avgCost,
      avgDuration,
      criticalFaults
    }
  }, [maintenanceRecords, vehicleId])

  // Handlers
  const handleScheduleMaintenance = useCallback(async () => {
    const maintenance: MaintenanceRecord = {
      id: `maint-${Date.now()}`,
      vehicleId: vehicleId || 'unknown',
      vehicleName: vehicleName || 'Unknown Vehicle',
      type: newMaintenance.type,
      category: newMaintenance.category,
      priority: newMaintenance.priority,
      status: 'scheduled',
      title: newMaintenance.title,
      description: newMaintenance.description,
      scheduledDate: new Date(newMaintenance.scheduledDate),
      estimatedDuration: newMaintenance.estimatedDuration,
      estimatedCost: newMaintenance.estimatedCost,
      facility: newMaintenance.facility,
      workOrders: [],
      parts: [],
      faults: [],
      notes: [],
      attachments: []
    }

    setMaintenanceRecords(prev => [maintenance, ...prev])
    onMaintenanceScheduled?.(maintenance)
    
    // Reset form
    setNewMaintenance({
      type: 'scheduled',
      category: 'preventive',
      priority: 'medium',
      title: '',
      description: '',
      scheduledDate: '',
      estimatedDuration: 120,
      estimatedCost: 0,
      facility: 'main_depot'
    })
    
    setShowScheduleDialog(false)

    console.info('Maintenance scheduled', {
      maintenanceId: maintenance.id,
      vehicleId,
      type: maintenance.type,
      scheduledDate: maintenance.scheduledDate.toISOString(),
      timestamp: new Date().toISOString()
    })
  }, [newMaintenance, vehicleId, vehicleName, onMaintenanceScheduled])

  const handleViewDetails = useCallback((record: MaintenanceRecord) => {
    setSelectedRecord(record)
    setShowDetailDialog(true)
  }, [])

  const getStatusColor = (status: string) => {
    switch (status) {
      case 'completed': return 'bg-green-100 text-green-800'
      case 'in_progress': return 'bg-blue-100 text-blue-800'
      case 'scheduled': return 'bg-yellow-100 text-yellow-800'
      case 'cancelled': return 'bg-gray-100 text-gray-800'
      case 'delayed': return 'bg-red-100 text-red-800'
      default: return 'bg-gray-100 text-gray-800'
    }
  }

  const getPriorityColor = (priority: string) => {
    switch (priority) {
      case 'critical': return 'bg-red-100 text-red-800'
      case 'high': return 'bg-orange-100 text-orange-800'
      case 'medium': return 'bg-yellow-100 text-yellow-800'
      case 'low': return 'bg-blue-100 text-blue-800'
      default: return 'bg-gray-100 text-gray-800'
    }
  }

  const getTrendIcon = (trend: string) => {
    switch (trend) {
      case 'improving': return <TrendingUp className="w-4 h-4 text-green-600" />
      case 'degrading': return <TrendingDown className="w-4 h-4 text-red-600" />
      default: return <div className="w-4 h-4" />
    }
  }

  return (
    <div className={`space-y-6 ${className}`}>
      {/* Header */}
      <div className="flex items-center justify-between">
        <div className="flex items-center space-x-3">
          <Wrench className="w-6 h-6 text-blue-600" />
          <h2 className="text-2xl font-semibold text-gray-900">
            {vehicleId ? `Maintenance - ${vehicleName}` : 'Fleet Maintenance'}
          </h2>
        </div>
        <Button onClick={() => setShowScheduleDialog(true)}>
          <Plus className="w-4 h-4 mr-2" />
          Schedule Maintenance
        </Button>
      </div>

      {/* Statistics */}
      <div className="grid grid-cols-2 md:grid-cols-4 lg:grid-cols-8 gap-4">
        <Card className="p-4">
          <div className="text-center">
            <div className="text-2xl font-bold text-gray-900">{stats.totalRecords}</div>
            <div className="text-sm text-gray-600">Total Records</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className="text-2xl font-bold text-green-600">{stats.completedRecords}</div>
            <div className="text-sm text-gray-600">Completed</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className="text-2xl font-bold text-yellow-600">{stats.scheduledRecords}</div>
            <div className="text-sm text-gray-600">Scheduled</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className={`text-2xl font-bold ${stats.overdueRecords > 0 ? 'text-red-600' : 'text-green-600'}`}>
              {stats.overdueRecords}
            </div>
            <div className="text-sm text-gray-600">Overdue</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className="text-2xl font-bold text-blue-600">${stats.totalCost.toLocaleString()}</div>
            <div className="text-sm text-gray-600">Total Cost</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className="text-2xl font-bold text-purple-600">${Math.round(stats.avgCost).toLocaleString()}</div>
            <div className="text-sm text-gray-600">Avg Cost</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className="text-2xl font-bold text-indigo-600">{Math.round(stats.avgDuration / 60)}h</div>
            <div className="text-sm text-gray-600">Avg Duration</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className={`text-2xl font-bold ${stats.criticalFaults > 0 ? 'text-red-600' : 'text-green-600'}`}>
              {stats.criticalFaults}
            </div>
            <div className="text-sm text-gray-600">Critical Faults</div>
          </div>
        </Card>
      </div>

      {/* Filters */}
      <Card className="p-4">
        <div className="flex flex-wrap items-center gap-4">
          <div className="flex items-center space-x-2">
            <Search className="w-4 h-4 text-gray-500" />
            <Input
              placeholder="Search maintenance records..."
              value={filters.search}
              onChange={(e) => setFilters(prev => ({ ...prev, search: e.target.value }))}
              className="w-64"
            />
          </div>
          
          <Select
            value={filters.status}
            onValueChange={(value) => setFilters(prev => ({ ...prev, status: value }))}
          >
            <option value="">All Status</option>
            <option value="scheduled">Scheduled</option>
            <option value="in_progress">In Progress</option>
            <option value="completed">Completed</option>
            <option value="cancelled">Cancelled</option>
            <option value="delayed">Delayed</option>
          </Select>

          <Select
            value={filters.type}
            onValueChange={(value) => setFilters(prev => ({ ...prev, type: value }))}
          >
            <option value="">All Types</option>
            <option value="scheduled">Scheduled</option>
            <option value="unscheduled">Unscheduled</option>
            <option value="predictive">Predictive</option>
            <option value="emergency">Emergency</option>
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

          <Button variant="outline" size="sm">
            <RefreshCw className="w-4 h-4 mr-2" />
            Refresh
          </Button>
        </div>
      </Card>

      {/* Maintenance Records */}
      <div className="space-y-4">
        {filteredRecords.map(record => (
          <Card key={record.id} className="p-6 hover:shadow-md transition-shadow">
            <div className="flex items-start justify-between">
              <div className="flex-1">
                <div className="flex items-center space-x-3 mb-3">
                  <h3 className="text-lg font-medium text-gray-900">{record.title}</h3>
                  <Badge className={getStatusColor(record.status)}>
                    {record.status.replace('_', ' ')}
                  </Badge>
                  <Badge className={getPriorityColor(record.priority)}>
                    {record.priority}
                  </Badge>
                  <Badge variant="outline">
                    {record.type}
                  </Badge>
                </div>

                <p className="text-gray-600 mb-4">{record.description}</p>

                <div className="grid grid-cols-2 md:grid-cols-4 gap-4 text-sm">
                  <div>
                    <div className="flex items-center space-x-1 text-gray-500 mb-1">
                      <Calendar className="w-4 h-4" />
                      <span>Scheduled</span>
                    </div>
                    <div className="font-medium">
                      {record.scheduledDate.toLocaleDateString()}
                    </div>
                  </div>

                  <div>
                    <div className="flex items-center space-x-1 text-gray-500 mb-1">
                      <Clock className="w-4 h-4" />
                      <span>Duration</span>
                    </div>
                    <div className="font-medium">
                      {Math.round((record.actualDuration || record.estimatedDuration) / 60)}h
                    </div>
                  </div>

                  <div>
                    <div className="flex items-center space-x-1 text-gray-500 mb-1">
                      <DollarSign className="w-4 h-4" />
                      <span>Cost</span>
                    </div>
                    <div className="font-medium">
                      ${(record.actualCost || record.estimatedCost).toLocaleString()}
                    </div>
                  </div>

                  <div>
                    <div className="flex items-center space-x-1 text-gray-500 mb-1">
                      <User className="w-4 h-4" />
                      <span>Technician</span>
                    </div>
                    <div className="font-medium">
                      {record.assignedTechnician || 'Unassigned'}
                    </div>
                  </div>
                </div>

                {/* Predictive Indicators */}
                {record.predictiveIndicators && record.predictiveIndicators.length > 0 && (
                  <div className="mt-4">
                    <h4 className="text-sm font-medium text-gray-900 mb-2">Predictive Health</h4>
                    <div className="grid grid-cols-1 md:grid-cols-2 gap-3">
                      {record.predictiveIndicators.slice(0, 2).map(indicator => (
                        <div key={indicator.id} className="p-3 bg-gray-50 rounded border">
                          <div className="flex items-center justify-between mb-2">
                            <div className="font-medium text-sm">{indicator.component}</div>
                            <div className="flex items-center space-x-1">
                              {getTrendIcon(indicator.trend)}
                              <span className="text-xs text-gray-500">{indicator.confidence}%</span>
                            </div>
                          </div>
                          <div className="text-sm text-gray-600 mb-1">
                            {indicator.metric}: {indicator.currentValue}
                          </div>
                          {indicator.remainingUsefulLife && (
                            <div className="text-xs text-orange-600">
                              RUL: {indicator.remainingUsefulLife} days
                            </div>
                          )}
                        </div>
                      ))}
                    </div>
                  </div>
                )}

                {/* Work Orders Summary */}
                {record.workOrders.length > 0 && (
                  <div className="mt-4">
                    <div className="flex items-center space-x-2 text-sm text-gray-600">
                      <FileText className="w-4 h-4" />
                      <span>{record.workOrders.length} work orders</span>
                      {record.parts.length > 0 && (
                        <>
                          <span>•</span>
                          <Package className="w-4 h-4" />
                          <span>{record.parts.length} parts</span>
                        </>
                      )}
                      {record.faults.length > 0 && (
                        <>
                          <span>•</span>
                          <AlertTriangle className="w-4 h-4" />
                          <span>{record.faults.length} faults</span>
                        </>
                      )}
                    </div>
                  </div>
                )}
              </div>

              <div className="flex items-center space-x-2">
                <Button
                  variant="outline"
                  size="sm"
                  onClick={() => handleViewDetails(record)}
                >
                  <Eye className="w-4 h-4" />
                </Button>
                
                <Button variant="outline" size="sm">
                  <Edit className="w-4 h-4" />
                </Button>

                {record.attachments.length > 0 && (
                  <Button variant="outline" size="sm">
                    <Download className="w-4 h-4" />
                  </Button>
                )}
              </div>
            </div>
          </Card>
        ))}

        {filteredRecords.length === 0 && (
          <div className="text-center py-12">
            <Wrench className="w-16 h-16 text-gray-400 mx-auto mb-4" />
            <h3 className="text-lg font-medium text-gray-900 mb-2">No Maintenance Records</h3>
            <p className="text-gray-600">
              {filters.search || filters.status || filters.type || filters.priority
                ? 'No records match your current filters'
                : 'No maintenance records found'
              }
            </p>
          </div>
        )}
      </div>

      {/* Schedule Maintenance Dialog */}
      <Dialog open={showScheduleDialog} onOpenChange={setShowScheduleDialog}>
        <DialogContent className="max-w-2xl">
          <DialogHeader>
            <DialogTitle>Schedule Maintenance</DialogTitle>
          </DialogHeader>

          <div className="space-y-4">
            {vehicleId && (
              <Alert>
                <Wrench className="w-4 h-4" />
                <AlertDescription>
                  Scheduling maintenance for: <strong>{vehicleName}</strong>
                </AlertDescription>
              </Alert>
            )}

            <div className="grid grid-cols-2 gap-4">
              <div>
                <label className="block text-sm font-medium text-gray-700 mb-2">
                  Type
                </label>
                <Select
                  value={newMaintenance.type}
                  onValueChange={(value) => setNewMaintenance(prev => ({ 
                    ...prev, 
                    type: value as 'scheduled' | 'unscheduled' | 'predictive' | 'emergency'
                  }))}
                >
                  <option value="scheduled">Scheduled</option>
                  <option value="unscheduled">Unscheduled</option>
                  <option value="predictive">Predictive</option>
                  <option value="emergency">Emergency</option>
                </Select>
              </div>

              <div>
                <label className="block text-sm font-medium text-gray-700 mb-2">
                  Category
                </label>
                <Select
                  value={newMaintenance.category}
                  onValueChange={(value) => setNewMaintenance(prev => ({ 
                    ...prev, 
                    category: value as 'preventive' | 'corrective' | 'predictive' | 'emergency'
                  }))}
                >
                  <option value="preventive">Preventive</option>
                  <option value="corrective">Corrective</option>
                  <option value="predictive">Predictive</option>
                  <option value="emergency">Emergency</option>
                </Select>
              </div>
            </div>

            <div>
              <label className="block text-sm font-medium text-gray-700 mb-2">
                Title *
              </label>
              <Input
                value={newMaintenance.title}
                onChange={(e) => setNewMaintenance(prev => ({ ...prev, title: e.target.value }))}
                placeholder="Brief description of maintenance work"
                required
              />
            </div>

            <div>
              <label className="block text-sm font-medium text-gray-700 mb-2">
                Description *
              </label>
              <Textarea
                value={newMaintenance.description}
                onChange={(e) => setNewMaintenance(prev => ({ ...prev, description: e.target.value }))}
                placeholder="Detailed description of work to be performed"
                rows={3}
                required
              />
            </div>

            <div className="grid grid-cols-2 gap-4">
              <div>
                <label className="block text-sm font-medium text-gray-700 mb-2">
                  Priority
                </label>
                <Select
                  value={newMaintenance.priority}
                  onValueChange={(value) => setNewMaintenance(prev => ({ 
                    ...prev, 
                    priority: value as 'low' | 'medium' | 'high' | 'critical'
                  }))}
                >
                  <option value="low">Low</option>
                  <option value="medium">Medium</option>
                  <option value="high">High</option>
                  <option value="critical">Critical</option>
                </Select>
              </div>

              <div>
                <label className="block text-sm font-medium text-gray-700 mb-2">
                  Scheduled Date *
                </label>
                <Input
                  type="datetime-local"
                  value={newMaintenance.scheduledDate}
                  onChange={(e) => setNewMaintenance(prev => ({ ...prev, scheduledDate: e.target.value }))}
                  required
                />
              </div>
            </div>

            <div className="grid grid-cols-2 gap-4">
              <div>
                <label className="block text-sm font-medium text-gray-700 mb-2">
                  Estimated Duration (minutes)
                </label>
                <Input
                  type="number"
                  value={newMaintenance.estimatedDuration}
                  onChange={(e) => setNewMaintenance(prev => ({ 
                    ...prev, 
                    estimatedDuration: parseInt(e.target.value) || 0
                  }))}
                  min="0"
                />
              </div>

              <div>
                <label className="block text-sm font-medium text-gray-700 mb-2">
                  Estimated Cost ($)
                </label>
                <Input
                  type="number"
                  value={newMaintenance.estimatedCost}
                  onChange={(e) => setNewMaintenance(prev => ({ 
                    ...prev, 
                    estimatedCost: parseFloat(e.target.value) || 0
                  }))}
                  min="0"
                  step="0.01"
                />
              </div>
            </div>

            <div>
              <label className="block text-sm font-medium text-gray-700 mb-2">
                Facility
              </label>
              <Select
                value={newMaintenance.facility}
                onValueChange={(value) => setNewMaintenance(prev => ({ ...prev, facility: value }))}
              >
                <option value="main_depot">Main Depot</option>
                <option value="service_center_1">Service Center 1</option>
                <option value="service_center_2">Service Center 2</option>
                <option value="mobile_unit">Mobile Service Unit</option>
              </Select>
            </div>

            <div className="flex justify-end space-x-3 pt-4">
              <Button
                variant="outline"
                onClick={() => setShowScheduleDialog(false)}
              >
                Cancel
              </Button>
              <Button
                onClick={handleScheduleMaintenance}
                disabled={!newMaintenance.title || !newMaintenance.description || !newMaintenance.scheduledDate}
              >
                Schedule Maintenance
              </Button>
            </div>
          </div>
        </DialogContent>
      </Dialog>

      {/* Maintenance Detail Dialog */}
      <Dialog open={showDetailDialog} onOpenChange={setShowDetailDialog}>
        <DialogContent className="max-w-6xl max-h-[90vh] overflow-y-auto">
          <DialogHeader>
            <DialogTitle>Maintenance Details</DialogTitle>
          </DialogHeader>

          {selectedRecord && (
            <Tabs value={activeTab} onValueChange={setActiveTab}>
              <TabsList>
                <TabsTrigger value="overview">Overview</TabsTrigger>
                <TabsTrigger value="work_orders">Work Orders</TabsTrigger>
                <TabsTrigger value="parts">Parts</TabsTrigger>
                <TabsTrigger value="faults">Faults</TabsTrigger>
                <TabsTrigger value="predictive">Predictive</TabsTrigger>
                <TabsTrigger value="attachments">Attachments</TabsTrigger>
              </TabsList>

              <TabsContent value="overview" className="space-y-4">
                <div className="grid grid-cols-2 gap-6">
                  <Card className="p-4">
                    <h3 className="text-lg font-medium text-gray-900 mb-3">Basic Information</h3>
                    <div className="space-y-2 text-sm">
                      <div className="flex justify-between">
                        <span className="text-gray-600">Title:</span>
                        <span className="font-medium">{selectedRecord.title}</span>
                      </div>
                      <div className="flex justify-between">
                        <span className="text-gray-600">Vehicle:</span>
                        <span className="font-medium">{selectedRecord.vehicleName}</span>
                      </div>
                      <div className="flex justify-between">
                        <span className="text-gray-600">Type:</span>
                        <Badge variant="outline">{selectedRecord.type}</Badge>
                      </div>
                      <div className="flex justify-between">
                        <span className="text-gray-600">Category:</span>
                        <span className="font-medium">{selectedRecord.category}</span>
                      </div>
                      <div className="flex justify-between">
                        <span className="text-gray-600">Priority:</span>
                        <Badge className={getPriorityColor(selectedRecord.priority)}>
                          {selectedRecord.priority}
                        </Badge>
                      </div>
                      <div className="flex justify-between">
                        <span className="text-gray-600">Status:</span>
                        <Badge className={getStatusColor(selectedRecord.status)}>
                          {selectedRecord.status.replace('_', ' ')}
                        </Badge>
                      </div>
                    </div>
                  </Card>

                  <Card className="p-4">
                    <h3 className="text-lg font-medium text-gray-900 mb-3">Schedule & Cost</h3>
                    <div className="space-y-2 text-sm">
                      <div className="flex justify-between">
                        <span className="text-gray-600">Scheduled:</span>
                        <span className="font-medium">{selectedRecord.scheduledDate.toLocaleString()}</span>
                      </div>
                      {selectedRecord.startedDate && (
                        <div className="flex justify-between">
                          <span className="text-gray-600">Started:</span>
                          <span className="font-medium">{selectedRecord.startedDate.toLocaleString()}</span>
                        </div>
                      )}
                      {selectedRecord.completedDate && (
                        <div className="flex justify-between">
                          <span className="text-gray-600">Completed:</span>
                          <span className="font-medium">{selectedRecord.completedDate.toLocaleString()}</span>
                        </div>
                      )}
                      <div className="flex justify-between">
                        <span className="text-gray-600">Duration:</span>
                        <span className="font-medium">
                          {Math.round((selectedRecord.actualDuration || selectedRecord.estimatedDuration) / 60)}h
                        </span>
                      </div>
                      <div className="flex justify-between">
                        <span className="text-gray-600">Cost:</span>
                        <span className="font-medium">
                          ${(selectedRecord.actualCost || selectedRecord.estimatedCost).toLocaleString()}
                        </span>
                      </div>
                      <div className="flex justify-between">
                        <span className="text-gray-600">Technician:</span>
                        <span className="font-medium">{selectedRecord.assignedTechnician || 'Unassigned'}</span>
                      </div>
                      <div className="flex justify-between">
                        <span className="text-gray-600">Facility:</span>
                        <span className="font-medium">{selectedRecord.facility}</span>
                      </div>
                    </div>
                  </Card>
                </div>

                <Card className="p-4">
                  <h3 className="text-lg font-medium text-gray-900 mb-3">Description</h3>
                  <p className="text-gray-600">{selectedRecord.description}</p>
                </Card>

                {selectedRecord.notes.length > 0 && (
                  <Card className="p-4">
                    <h3 className="text-lg font-medium text-gray-900 mb-3">Notes</h3>
                    <div className="space-y-2">
                      {selectedRecord.notes.map((note, index) => (
                        <div key={index} className="p-2 bg-gray-50 rounded text-sm">
                          {note}
                        </div>
                      ))}
                    </div>
                  </Card>
                )}
              </TabsContent>

              <TabsContent value="work_orders" className="space-y-4">
                <div className="space-y-3">
                  {selectedRecord.workOrders.map(workOrder => (
                    <Card key={workOrder.id} className="p-4">
                      <div className="flex items-center justify-between mb-3">
                        <h4 className="font-medium text-gray-900">{workOrder.title}</h4>
                        <div className="flex items-center space-x-2">
                          <Badge className={getPriorityColor(workOrder.priority)}>
                            {workOrder.priority}
                          </Badge>
                          <Badge className={getStatusColor(workOrder.status)}>
                            {workOrder.status.replace('_', ' ')}
                          </Badge>
                        </div>
                      </div>
                      <p className="text-gray-600 text-sm mb-3">{workOrder.description}</p>
                      <div className="grid grid-cols-4 gap-4 text-sm">
                        <div>
                          <span className="text-gray-500">Category:</span>
                          <div className="font-medium">{workOrder.category}</div>
                        </div>
                        <div>
                          <span className="text-gray-500">Assigned To:</span>
                          <div className="font-medium">{workOrder.assignedTo || 'Unassigned'}</div>
                        </div>
                        <div>
                          <span className="text-gray-500">Estimated Hours:</span>
                          <div className="font-medium">{workOrder.estimatedHours}h</div>
                        </div>
                        <div>
                          <span className="text-gray-500">Actual Hours:</span>
                          <div className="font-medium">{workOrder.actualHours || '-'}h</div>
                        </div>
                      </div>
                    </Card>
                  ))}
                  
                  {selectedRecord.workOrders.length === 0 && (
                    <div className="text-center py-8">
                      <FileText className="w-12 h-12 text-gray-400 mx-auto mb-2" />
                      <p className="text-gray-600">No work orders defined</p>
                    </div>
                  )}
                </div>
              </TabsContent>

              <TabsContent value="parts" className="space-y-4">
                <div className="space-y-3">
                  {selectedRecord.parts.map(part => (
                    <Card key={part.id} className="p-4">
                      <div className="flex items-center justify-between mb-3">
                        <h4 className="font-medium text-gray-900">{part.partName}</h4>
                        <div className="font-medium text-blue-600">
                          ${part.totalCost.toLocaleString()}
                        </div>
                      </div>
                      
                      <div className="grid grid-cols-4 gap-4 text-sm">
                        <div>
                          <span className="text-gray-500">Part Number:</span>
                          <div className="font-medium">{part.partNumber}</div>
                        </div>
                        <div>
                          <span className="text-gray-500">Quantity:</span>
                          <div className="font-medium">{part.quantity}</div>
                        </div>
                        <div>
                          <span className="text-gray-500">Unit Cost:</span>
                          <div className="font-medium">${part.unitCost}</div>
                        </div>
                        <div>
                          <span className="text-gray-500">Supplier:</span>
                          <div className="font-medium">{part.supplier}</div>
                        </div>
                      </div>
                      
                      <div className="grid grid-cols-3 gap-4 text-sm mt-3">
                        <div>
                          <span className="text-gray-500">Warranty:</span>
                          <div className="font-medium">{part.warrantyMonths} months</div>
                        </div>
                        <div>
                          <span className="text-gray-500">Installed:</span>
                          <div className="font-medium">{part.installedDate.toLocaleDateString()}</div>
                        </div>
                        <div>
                          <span className="text-gray-500">Serial Number:</span>
                          <div className="font-medium">{part.serialNumber || 'N/A'}</div>
                        </div>
                      </div>
                    </Card>
                  ))}
                  
                  {selectedRecord.parts.length === 0 && (
                    <div className="text-center py-8">
                      <Package className="w-12 h-12 text-gray-400 mx-auto mb-2" />
                      <p className="text-gray-600">No parts used</p>
                    </div>
                  )}
                </div>
              </TabsContent>

              <TabsContent value="faults" className="space-y-4">
                <div className="space-y-3">
                  {selectedRecord.faults.map(fault => (
                    <Card key={fault.id} className="p-4">
                      <div className="flex items-center justify-between mb-3">
                        <div className="flex items-center space-x-3">
                          <h4 className="font-medium text-gray-900">{fault.code}</h4>
                          <Badge className={
                            fault.severity === 'critical' ? 'bg-red-100 text-red-800' :
                            fault.severity === 'error' ? 'bg-orange-100 text-orange-800' :
                            fault.severity === 'warning' ? 'bg-yellow-100 text-yellow-800' :
                            'bg-blue-100 text-blue-800'
                          }>
                            {fault.severity}
                          </Badge>
                        </div>
                        <div className="text-sm text-gray-600">
                          {fault.resolvedAt ? 'Resolved' : 'Active'}
                        </div>
                      </div>
                      
                      <p className="text-gray-600 text-sm mb-3">{fault.description}</p>
                      
                      <div className="grid grid-cols-3 gap-4 text-sm">
                        <div>
                          <span className="text-gray-500">System:</span>
                          <div className="font-medium">{fault.system}</div>
                        </div>
                        <div>
                          <span className="text-gray-500">Detected:</span>
                          <div className="font-medium">{fault.detectedAt.toLocaleString()}</div>
                        </div>
                        <div>
                          <span className="text-gray-500">Recurrent:</span>
                          <div className={`font-medium ${fault.recurrent ? 'text-red-600' : 'text-green-600'}`}>
                            {fault.recurrent ? 'Yes' : 'No'}
                          </div>
                        </div>
                      </div>
                      
                      {fault.resolution && (
                        <div className="mt-3 p-2 bg-green-50 rounded">
                          <div className="text-sm font-medium text-green-900">Resolution:</div>
                          <div className="text-sm text-green-700">{fault.resolution}</div>
                        </div>
                      )}
                    </Card>
                  ))}
                  
                  {selectedRecord.faults.length === 0 && (
                    <div className="text-center py-8">
                      <CheckCircle className="w-12 h-12 text-green-400 mx-auto mb-2" />
                      <p className="text-gray-600">No faults detected</p>
                    </div>
                  )}
                </div>
              </TabsContent>

              <TabsContent value="predictive" className="space-y-4">
                <div className="space-y-4">
                  {selectedRecord.predictiveIndicators?.map(indicator => (
                    <Card key={indicator.id} className="p-4">
                      <div className="flex items-center justify-between mb-3">
                        <h4 className="font-medium text-gray-900">{indicator.component}</h4>
                        <div className="flex items-center space-x-2">
                          {getTrendIcon(indicator.trend)}
                          <span className="text-sm text-gray-600">{indicator.confidence}% confidence</span>
                        </div>
                      </div>
                      
                      <div className="grid grid-cols-4 gap-4 text-sm mb-4">
                        <div>
                          <span className="text-gray-500">Metric:</span>
                          <div className="font-medium">{indicator.metric}</div>
                        </div>
                        <div>
                          <span className="text-gray-500">Current Value:</span>
                          <div className="font-medium">{indicator.currentValue}</div>
                        </div>
                        <div>
                          <span className="text-gray-500">Normal Range:</span>
                          <div className="font-medium">
                            {indicator.normalRange.min} - {indicator.normalRange.max}
                          </div>
                        </div>
                        <div>
                          <span className="text-gray-500">Trend:</span>
                          <div className={`font-medium ${
                            indicator.trend === 'improving' ? 'text-green-600' :
                            indicator.trend === 'degrading' ? 'text-red-600' : 'text-gray-600'
                          }`}>
                            {indicator.trend}
                          </div>
                        </div>
                      </div>
                      
                      {indicator.remainingUsefulLife && (
                        <div className="mb-4">
                          <div className="flex items-center justify-between text-sm mb-1">
                            <span className="text-gray-600">Remaining Useful Life</span>
                            <span className="font-medium">{indicator.remainingUsefulLife} days</span>
                          </div>
                          <Progress 
                            value={Math.max(0, Math.min(100, (indicator.remainingUsefulLife / 365) * 100))} 
                            className="h-2"
                          />
                        </div>
                      )}
                      
                      <div className="text-xs text-gray-500">
                        Last updated: {indicator.lastUpdated.toLocaleString()}
                      </div>
                    </Card>
                  ))}
                  
                  {(!selectedRecord.predictiveIndicators || selectedRecord.predictiveIndicators.length === 0) && (
                    <div className="text-center py-8">
                      <TrendingUp className="w-12 h-12 text-gray-400 mx-auto mb-2" />
                      <p className="text-gray-600">No predictive indicators available</p>
                    </div>
                  )}
                </div>
              </TabsContent>

              <TabsContent value="attachments" className="space-y-4">
                <div className="space-y-3">
                  {selectedRecord.attachments.map(attachment => (
                    <Card key={attachment.id} className="p-4">
                      <div className="flex items-center justify-between">
                        <div className="flex items-center space-x-3">
                          <div className="p-2 bg-blue-100 rounded">
                            {attachment.type === 'photo' && <Camera className="w-5 h-5 text-blue-600" />}
                            {attachment.type === 'document' && <FileText className="w-5 h-5 text-blue-600" />}
                            {attachment.type === 'video' && <Camera className="w-5 h-5 text-blue-600" />}
                            {attachment.type === 'report' && <FileText className="w-5 h-5 text-blue-600" />}
                          </div>
                          <div>
                            <div className="font-medium text-gray-900">{attachment.filename}</div>
                            <div className="text-sm text-gray-600">{attachment.description}</div>
                            <div className="text-xs text-gray-500">
                              Uploaded by {attachment.uploadedBy} on {attachment.uploadedAt.toLocaleString()}
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
                  
                  {selectedRecord.attachments.length === 0 && (
                    <div className="text-center py-8">
                      <FileText className="w-12 h-12 text-gray-400 mx-auto mb-2" />
                      <p className="text-gray-600">No attachments</p>
                    </div>
                  )}
                </div>
              </TabsContent>
            </Tabs>
          )}
        </DialogContent>
      </Dialog>
    </div>
  )
}

export default MaintenanceManager
