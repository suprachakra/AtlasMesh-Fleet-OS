import React, { useState, useCallback, useEffect, useMemo } from 'react'
import { 
  GitBranch, Download, Upload, Lock, Unlock, AlertTriangle, CheckCircle,
  Clock, User, Shield, RefreshCw, Eye, Settings, History, Tag, Zap,
  Database, Map as MapIcon, Code, FileText, X, Check, RotateCcw, Play
} from 'lucide-react'
import { Button } from '../ui/Button'
import { Badge } from '../ui/Badge'
import { Card } from '../ui/Card'
import { Dialog, DialogContent, DialogHeader, DialogTitle } from '../ui/Dialog'
import { Input } from '../ui/Input'
import { Select } from '../ui/Select'
import { Alert, AlertDescription } from '../ui/Alert'
import { Progress } from '../ui/Progress'
import { Tabs, TabsContent, TabsList, TabsTrigger } from '../ui/Tabs'
import { Switch } from '../ui/Switch'

// Types
interface VersionInfo {
  id: string
  component: 'binary' | 'roadgraph' | 'map' | 'lsm' | 'blacklist' | 'policy'
  name: string
  currentVersion: string
  availableVersions: VersionEntry[]
  pinnedVersion?: string
  rollbackVersion?: string
  status: 'up_to_date' | 'update_available' | 'pinned' | 'rollback_pending' | 'updating'
  lastUpdated: Date
  updateChannel: 'stable' | 'beta' | 'alpha'
  autoUpdate: boolean
  rbacRequired: boolean
  vehicles: VehicleVersionStatus[]
}

interface VersionEntry {
  version: string
  releaseDate: Date
  description: string
  changelog: string[]
  size: number
  checksum: string
  signed: boolean
  stability: 'stable' | 'beta' | 'alpha' | 'experimental'
  compatibility: string[]
  dependencies: Record<string, string>
  rollbackSupported: boolean
  criticalUpdate: boolean
}

interface VehicleVersionStatus {
  vehicleId: string
  vehicleName: string
  currentVersion: string
  targetVersion?: string
  updateStatus: 'synced' | 'updating' | 'pending' | 'failed' | 'rollback'
  lastSync: Date
  updateProgress?: number
}

interface VersionOperation {
  id: string
  type: 'update' | 'rollback' | 'pin' | 'unpin'
  component: string
  fromVersion: string
  toVersion: string
  status: 'pending' | 'in_progress' | 'completed' | 'failed' | 'cancelled'
  startTime: Date
  endTime?: Date
  progress: number
  affectedVehicles: string[]
  operator: string
  approver?: string
  reason: string
  rbacApproved: boolean
}

interface RBACPermission {
  userId: string
  userName: string
  role: string
  permissions: {
    canUpdate: boolean
    canRollback: boolean
    canPin: boolean
    canApprove: boolean
    components: string[]
  }
  lastActivity: Date
}

interface VersionManagerProps {
  vehicleId?: string
  vehicleName?: string
  onVersionChanged?: (component: string, version: string) => void
  className?: string
}

const VersionManager: React.FC<VersionManagerProps> = ({
  vehicleId,
  vehicleName,
  onVersionChanged,
  className = ''
}) => {
  // State
  const [versions, setVersions] = useState<VersionInfo[]>([])
  const [operations, setOperations] = useState<VersionOperation[]>([])
  const [showUpdateDialog, setShowUpdateDialog] = useState(false)
  const [showRollbackDialog, setShowRollbackDialog] = useState(false)
  const [showPinDialog, setShowPinDialog] = useState(false)
  const [selectedComponent, setSelectedComponent] = useState<VersionInfo | null>(null)
  const [selectedVersion, setSelectedVersion] = useState<VersionEntry | null>(null)
  const [activeTab, setActiveTab] = useState('overview')
  const [operationReason, setOperationReason] = useState('')
  const [requiresApproval, setRequiresApproval] = useState(false)
  const [currentUser] = useState({
    id: 'user-001',
    name: 'Current User',
    role: 'fleet_manager',
    permissions: {
      canUpdate: true,
      canRollback: true,
      canPin: true,
      canApprove: false,
      components: ['binary', 'roadgraph', 'map', 'lsm', 'blacklist']
    }
  })

  // Mock data initialization
  const initializeMockData = useCallback(() => {
    const mockVersions: VersionInfo[] = [
      {
        id: 'version-binary',
        component: 'binary',
        name: 'AtlasMesh Core Binary',
        currentVersion: 'v2.1.0',
        availableVersions: [
          {
            version: 'v2.1.1',
            releaseDate: new Date('2024-11-20'),
            description: 'Performance improvements and bug fixes',
            changelog: [
              'Improved path planning algorithm efficiency by 15%',
              'Fixed memory leak in sensor fusion module',
              'Enhanced emergency stop response time',
              'Updated ML model for better object detection'
            ],
            size: 245,
            checksum: 'sha256:abc123def456...',
            signed: true,
            stability: 'stable',
            compatibility: ['v2.0.x', 'v2.1.x'],
            dependencies: { 'roadgraph': '>=v1.8.4', 'map': '>=v3.2.0' },
            rollbackSupported: true,
            criticalUpdate: false
          },
          {
            version: 'v2.2.0-beta',
            releaseDate: new Date('2024-11-25'),
            description: 'Next generation planning system (Beta)',
            changelog: [
              'New multi-modal path planning system',
              'Enhanced weather adaptation algorithms',
              'Improved vehicle-to-infrastructure communication',
              'Beta support for dynamic route optimization'
            ],
            size: 278,
            checksum: 'sha256:def456ghi789...',
            signed: true,
            stability: 'beta',
            compatibility: ['v2.1.x'],
            dependencies: { 'roadgraph': '>=v1.9.0', 'map': '>=v3.3.0' },
            rollbackSupported: true,
            criticalUpdate: false
          }
        ],
        pinnedVersion: undefined,
        rollbackVersion: 'v2.0.8',
        status: 'update_available',
        lastUpdated: new Date('2024-11-15'),
        updateChannel: 'stable',
        autoUpdate: false,
        rbacRequired: true,
        vehicles: [
          {
            vehicleId: 'v1',
            vehicleName: 'Atlas-001',
            currentVersion: 'v2.1.0',
            updateStatus: 'synced',
            lastSync: new Date()
          },
          {
            vehicleId: 'v2',
            vehicleName: 'Atlas-002',
            currentVersion: 'v2.0.8',
            targetVersion: 'v2.1.0',
            updateStatus: 'pending',
            lastSync: new Date(Date.now() - 30 * 60 * 1000),
            updateProgress: 0
          }
        ]
      },
      {
        id: 'version-roadgraph',
        component: 'roadgraph',
        name: 'Road Graph Data',
        currentVersion: 'v1.8.5',
        availableVersions: [
          {
            version: 'v1.8.6',
            releaseDate: new Date('2024-11-18'),
            description: 'Updated road network for Dubai Metro expansion',
            changelog: [
              'Added new metro line connections',
              'Updated traffic light timing data',
              'Fixed intersection geometry issues',
              'Added temporary construction zone data'
            ],
            size: 156,
            checksum: 'sha256:ghi789jkl012...',
            signed: true,
            stability: 'stable',
            compatibility: ['v2.1.x'],
            dependencies: {},
            rollbackSupported: true,
            criticalUpdate: true
          }
        ],
        pinnedVersion: undefined,
        rollbackVersion: 'v1.8.4',
        status: 'update_available',
        lastUpdated: new Date('2024-11-10'),
        updateChannel: 'stable',
        autoUpdate: true,
        rbacRequired: false,
        vehicles: [
          {
            vehicleId: 'v1',
            vehicleName: 'Atlas-001',
            currentVersion: 'v1.8.5',
            updateStatus: 'synced',
            lastSync: new Date()
          }
        ]
      },
      {
        id: 'version-map',
        component: 'map',
        name: 'High-Definition Maps',
        currentVersion: 'v3.2.1',
        availableVersions: [
          {
            version: 'v3.2.2',
            releaseDate: new Date('2024-11-22'),
            description: 'Enhanced lane marking accuracy and new POIs',
            changelog: [
              'Improved lane marking detection accuracy by 8%',
              'Added 1,200 new points of interest',
              'Updated parking lot layouts for 15 locations',
              'Enhanced tunnel and overpass mapping'
            ],
            size: 892,
            checksum: 'sha256:jkl012mno345...',
            signed: true,
            stability: 'stable',
            compatibility: ['v2.1.x'],
            dependencies: {},
            rollbackSupported: true,
            criticalUpdate: false
          }
        ],
        pinnedVersion: 'v3.2.1',
        rollbackVersion: 'v3.2.0',
        status: 'pinned',
        lastUpdated: new Date('2024-11-05'),
        updateChannel: 'stable',
        autoUpdate: false,
        rbacRequired: true,
        vehicles: [
          {
            vehicleId: 'v1',
            vehicleName: 'Atlas-001',
            currentVersion: 'v3.2.1',
            updateStatus: 'synced',
            lastSync: new Date()
          }
        ]
      },
      {
        id: 'version-lsm',
        component: 'lsm',
        name: 'Localization & Sensor Maps',
        currentVersion: 'v1.4.2',
        availableVersions: [],
        pinnedVersion: undefined,
        rollbackVersion: 'v1.4.1',
        status: 'up_to_date',
        lastUpdated: new Date('2024-11-12'),
        updateChannel: 'stable',
        autoUpdate: true,
        rbacRequired: false,
        vehicles: [
          {
            vehicleId: 'v1',
            vehicleName: 'Atlas-001',
            currentVersion: 'v1.4.2',
            updateStatus: 'synced',
            lastSync: new Date()
          }
        ]
      },
      {
        id: 'version-blacklist',
        component: 'blacklist',
        name: 'Security Blacklist',
        currentVersion: 'v2.0.8',
        availableVersions: [
          {
            version: 'v2.0.9',
            releaseDate: new Date('2024-11-26'),
            description: 'Critical security update - immediate deployment recommended',
            changelog: [
              'Added 47 new malicious IP addresses',
              'Updated threat signature database',
              'Enhanced DDoS protection patterns',
              'Fixed false positive in certificate validation'
            ],
            size: 12,
            checksum: 'sha256:mno345pqr678...',
            signed: true,
            stability: 'stable',
            compatibility: ['v2.0.x', 'v2.1.x'],
            dependencies: {},
            rollbackSupported: true,
            criticalUpdate: true
          }
        ],
        pinnedVersion: undefined,
        rollbackVersion: 'v2.0.7',
        status: 'update_available',
        lastUpdated: new Date('2024-11-08'),
        updateChannel: 'stable',
        autoUpdate: true,
        rbacRequired: true,
        vehicles: [
          {
            vehicleId: 'v1',
            vehicleName: 'Atlas-001',
            currentVersion: 'v2.0.8',
            updateStatus: 'synced',
            lastSync: new Date()
          }
        ]
      }
    ]

    const mockOperations: VersionOperation[] = [
      {
        id: 'op-001',
        type: 'update',
        component: 'blacklist',
        fromVersion: 'v2.0.7',
        toVersion: 'v2.0.8',
        status: 'completed',
        startTime: new Date(Date.now() - 2 * 60 * 60 * 1000),
        endTime: new Date(Date.now() - 90 * 60 * 1000),
        progress: 100,
        affectedVehicles: ['v1', 'v2', 'v3'],
        operator: 'System Auto-Update',
        reason: 'Critical security update',
        rbacApproved: true
      },
      {
        id: 'op-002',
        type: 'pin',
        component: 'map',
        fromVersion: 'v3.2.1',
        toVersion: 'v3.2.1',
        status: 'completed',
        startTime: new Date(Date.now() - 24 * 60 * 60 * 1000),
        endTime: new Date(Date.now() - 24 * 60 * 60 * 1000 + 5 * 60 * 1000),
        progress: 100,
        affectedVehicles: ['v1'],
        operator: 'Ahmed Al-Mansouri',
        approver: 'Fleet Manager',
        reason: 'Stability during high-traffic period',
        rbacApproved: true
      }
    ]

    setVersions(mockVersions)
    setOperations(mockOperations)
  }, [])

  useEffect(() => {
    initializeMockData()
  }, [initializeMockData])

  // Handlers
  const handleUpdate = useCallback((component: VersionInfo, version: VersionEntry) => {
    setSelectedComponent(component)
    setSelectedVersion(version)
    setRequiresApproval(component.rbacRequired && version.criticalUpdate)
    setShowUpdateDialog(true)
  }, [])

  const handleRollback = useCallback((component: VersionInfo) => {
    setSelectedComponent(component)
    setRequiresApproval(component.rbacRequired)
    setShowRollbackDialog(true)
  }, [])

  const handlePin = useCallback((component: VersionInfo) => {
    setSelectedComponent(component)
    setRequiresApproval(component.rbacRequired)
    setShowPinDialog(true)
  }, [])

  const executeVersionOperation = useCallback(async (
    type: VersionOperation['type'],
    component: VersionInfo,
    targetVersion?: string
  ) => {
    const operation: VersionOperation = {
      id: `op-${Date.now()}`,
      type,
      component: component.component,
      fromVersion: component.currentVersion,
      toVersion: targetVersion || component.currentVersion,
      status: 'pending',
      startTime: new Date(),
      progress: 0,
      affectedVehicles: vehicleId ? [vehicleId] : component.vehicles.map(v => v.vehicleId),
      operator: currentUser.name,
      reason: operationReason,
      rbacApproved: !requiresApproval || currentUser.permissions.canApprove
    }

    setOperations(prev => [operation, ...prev])

    // Simulate operation execution
    setTimeout(() => {
      setOperations(prev => prev.map(op => 
        op.id === operation.id 
          ? { ...op, status: 'in_progress' as const }
          : op
      ))

      // Simulate progress
      const progressInterval = setInterval(() => {
        setOperations(prev => prev.map(op => {
          if (op.id === operation.id && op.progress < 100) {
            const newProgress = Math.min(100, op.progress + Math.random() * 20)
            return { ...op, progress: newProgress }
          }
          return op
        }))
      }, 1000)

      // Complete operation
      setTimeout(() => {
        clearInterval(progressInterval)
        setOperations(prev => prev.map(op => 
          op.id === operation.id 
            ? { 
                ...op, 
                status: 'completed' as const, 
                progress: 100,
                endTime: new Date()
              }
            : op
        ))

        // Update version status
        setVersions(prev => prev.map(v => {
          if (v.id === component.id) {
            let newStatus: VersionInfo['status'] = v.status
            let newCurrentVersion = v.currentVersion
            let newPinnedVersion = v.pinnedVersion

            switch (type) {
              case 'update':
                newCurrentVersion = targetVersion || v.currentVersion
                newStatus = 'up_to_date'
                break
              case 'rollback':
                newCurrentVersion = v.rollbackVersion || v.currentVersion
                newStatus = 'up_to_date'
                break
              case 'pin':
                newPinnedVersion = v.currentVersion
                newStatus = 'pinned'
                break
              case 'unpin':
                newPinnedVersion = undefined
                newStatus = v.availableVersions.length > 0 ? 'update_available' : 'up_to_date'
                break
            }

            return {
              ...v,
              currentVersion: newCurrentVersion,
              pinnedVersion: newPinnedVersion,
              status: newStatus,
              lastUpdated: new Date()
            }
          }
          return v
        }))

        onVersionChanged?.(component.component, targetVersion || component.currentVersion)
      }, 5000)
    }, 1000)

    // Reset form
    setOperationReason('')
    setShowUpdateDialog(false)
    setShowRollbackDialog(false)
    setShowPinDialog(false)
  }, [vehicleId, currentUser, operationReason, requiresApproval, onVersionChanged])

  // Computed values
  const stats = useMemo(() => {
    const filteredVersions = vehicleId ? 
      versions.map(v => ({
        ...v,
        vehicles: v.vehicles.filter(vehicle => vehicle.vehicleId === vehicleId)
      })) : 
      versions

    const total = filteredVersions.length
    const upToDate = filteredVersions.filter(v => v.status === 'up_to_date').length
    const updatesAvailable = filteredVersions.filter(v => v.status === 'update_available').length
    const pinned = filteredVersions.filter(v => v.status === 'pinned').length
    const criticalUpdates = filteredVersions.filter(v => 
      v.availableVersions.some(av => av.criticalUpdate)
    ).length

    const activeOperations = operations.filter(op => 
      op.status === 'pending' || op.status === 'in_progress'
    ).length

    return {
      total,
      upToDate,
      updatesAvailable,
      pinned,
      criticalUpdates,
      activeOperations
    }
  }, [versions, operations, vehicleId])

  const getStatusColor = (status: VersionInfo['status']) => {
    switch (status) {
      case 'up_to_date': return 'bg-green-100 text-green-800'
      case 'update_available': return 'bg-blue-100 text-blue-800'
      case 'pinned': return 'bg-purple-100 text-purple-800'
      case 'rollback_pending': return 'bg-yellow-100 text-yellow-800'
      case 'updating': return 'bg-blue-100 text-blue-800'
      default: return 'bg-gray-100 text-gray-800'
    }
  }

  const getComponentIcon = (component: string) => {
    switch (component) {
      case 'binary': return <Code className="w-5 h-5" />
      case 'roadgraph': return <GitBranch className="w-5 h-5" />
      case 'map': return <MapIcon className="w-5 h-5" />
      case 'lsm': return <Database className="w-5 h-5" />
      case 'blacklist': return <Shield className="w-5 h-5" />
      case 'policy': return <FileText className="w-5 h-5" />
      default: return <Settings className="w-5 h-5" />
    }
  }

  const getOperationStatusColor = (status: VersionOperation['status']) => {
    switch (status) {
      case 'completed': return 'bg-green-100 text-green-800'
      case 'in_progress': return 'bg-blue-100 text-blue-800'
      case 'pending': return 'bg-yellow-100 text-yellow-800'
      case 'failed': return 'bg-red-100 text-red-800'
      case 'cancelled': return 'bg-gray-100 text-gray-800'
      default: return 'bg-gray-100 text-gray-800'
    }
  }

  return (
    <div className={`space-y-6 ${className}`}>
      {/* Header */}
      <div className="flex items-center justify-between">
        <div className="flex items-center space-x-3">
          <GitBranch className="w-6 h-6 text-blue-600" />
          <h2 className="text-2xl font-semibold text-gray-900">
            Version Management
            {vehicleId && <span className="text-lg font-normal text-gray-600 ml-2">- {vehicleName}</span>}
          </h2>
        </div>
        <Button variant="outline" onClick={initializeMockData}>
          <RefreshCw className="w-4 h-4 mr-2" />
          Refresh
        </Button>
      </div>

      {/* Statistics */}
      <div className="grid grid-cols-2 md:grid-cols-6 gap-4">
        <Card className="p-4">
          <div className="text-center">
            <div className="text-2xl font-bold text-gray-900">{stats.total}</div>
            <div className="text-sm text-gray-600">Components</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className="text-2xl font-bold text-green-600">{stats.upToDate}</div>
            <div className="text-sm text-gray-600">Up to Date</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className="text-2xl font-bold text-blue-600">{stats.updatesAvailable}</div>
            <div className="text-sm text-gray-600">Updates Available</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className="text-2xl font-bold text-purple-600">{stats.pinned}</div>
            <div className="text-sm text-gray-600">Pinned</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className={`text-2xl font-bold ${stats.criticalUpdates > 0 ? 'text-red-600' : 'text-green-600'}`}>
              {stats.criticalUpdates}
            </div>
            <div className="text-sm text-gray-600">Critical Updates</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className={`text-2xl font-bold ${stats.activeOperations > 0 ? 'text-blue-600' : 'text-gray-600'}`}>
              {stats.activeOperations}
            </div>
            <div className="text-sm text-gray-600">Active Operations</div>
          </div>
        </Card>
      </div>

      {/* Critical Updates Alert */}
      {stats.criticalUpdates > 0 && (
        <Alert className="border-red-200 bg-red-50">
          <AlertTriangle className="w-4 h-4 text-red-600" />
          <AlertDescription className="text-red-800">
            <strong>Critical Updates Available:</strong> {stats.criticalUpdates} component(s) have critical security or safety updates that should be applied immediately.
          </AlertDescription>
        </Alert>
      )}

      {/* Main Content */}
      <Tabs value={activeTab} onValueChange={setActiveTab}>
        <TabsList>
          <TabsTrigger value="overview">Version Overview</TabsTrigger>
          <TabsTrigger value="operations">Operations History</TabsTrigger>
          <TabsTrigger value="rbac">Access Control</TabsTrigger>
        </TabsList>

        <TabsContent value="overview" className="space-y-4">
          <div className="space-y-6">
            {versions
              .filter(version => !vehicleId || version.vehicles.some(v => v.vehicleId === vehicleId))
              .map(version => (
              <Card key={version.id} className="p-6">
                <div className="flex items-start justify-between mb-4">
                  <div className="flex items-center space-x-3">
                    <div className="p-2 bg-blue-100 rounded">
                      {getComponentIcon(version.component)}
                    </div>
                    <div>
                      <h3 className="text-lg font-medium text-gray-900">{version.name}</h3>
                      <p className="text-sm text-gray-600 capitalize">{version.component} component</p>
                    </div>
                  </div>
                  <div className="flex items-center space-x-2">
                    <Badge className={getStatusColor(version.status)}>
                      {version.status.replace('_', ' ')}
                    </Badge>
                    <Badge variant="outline" className="capitalize">
                      {version.updateChannel}
                    </Badge>
                    {version.autoUpdate && (
                      <Badge variant="outline" className="bg-green-50 text-green-700">
                        Auto-update
                      </Badge>
                    )}
                    {version.rbacRequired && (
                      <Badge variant="outline" className="bg-orange-50 text-orange-700">
                        <Lock className="w-3 h-3 mr-1" />
                        RBAC
                      </Badge>
                    )}
                  </div>
                </div>

                <div className="grid grid-cols-2 md:grid-cols-4 gap-4 mb-4">
                  <div>
                    <div className="text-sm text-gray-600">Current Version</div>
                    <div className="font-medium text-gray-900">{version.currentVersion}</div>
                  </div>
                  <div>
                    <div className="text-sm text-gray-600">Last Updated</div>
                    <div className="font-medium text-gray-900">{version.lastUpdated.toLocaleDateString()}</div>
                  </div>
                  <div>
                    <div className="text-sm text-gray-600">Pinned Version</div>
                    <div className="font-medium text-gray-900">
                      {version.pinnedVersion || 'None'}
                    </div>
                  </div>
                  <div>
                    <div className="text-sm text-gray-600">Rollback Available</div>
                    <div className="font-medium text-gray-900">
                      {version.rollbackVersion || 'None'}
                    </div>
                  </div>
                </div>

                {/* Available Updates */}
                {version.availableVersions.length > 0 && (
                  <div className="mb-4">
                    <h4 className="text-sm font-medium text-gray-900 mb-2">Available Updates</h4>
                    <div className="space-y-2">
                      {version.availableVersions.map(availableVersion => (
                        <div key={availableVersion.version} className="p-3 border border-gray-200 rounded-lg">
                          <div className="flex items-center justify-between mb-2">
                            <div className="flex items-center space-x-2">
                              <span className="font-medium text-gray-900">{availableVersion.version}</span>
                              <Badge variant="outline" className={
                                availableVersion.stability === 'stable' ? 'bg-green-50 text-green-700' :
                                availableVersion.stability === 'beta' ? 'bg-blue-50 text-blue-700' :
                                'bg-yellow-50 text-yellow-700'
                              }>
                                {availableVersion.stability}
                              </Badge>
                              {availableVersion.criticalUpdate && (
                                <Badge className="bg-red-100 text-red-800">
                                  Critical
                                </Badge>
                              )}
                              {availableVersion.signed && (
                                <Badge variant="outline" className="bg-green-50 text-green-700">
                                  <Shield className="w-3 h-3 mr-1" />
                                  Signed
                                </Badge>
                              )}
                            </div>
                            <div className="text-sm text-gray-600">
                              {availableVersion.releaseDate.toLocaleDateString()} • {availableVersion.size}MB
                            </div>
                          </div>
                          <p className="text-sm text-gray-600 mb-2">{availableVersion.description}</p>
                          {availableVersion.changelog.length > 0 && (
                            <div className="text-xs text-gray-500">
                              <div className="font-medium mb-1">Changelog:</div>
                              <ul className="list-disc list-inside space-y-0.5">
                                {availableVersion.changelog.slice(0, 2).map((change, index) => (
                                  <li key={index}>{change}</li>
                                ))}
                                {availableVersion.changelog.length > 2 && (
                                  <li>...and {availableVersion.changelog.length - 2} more changes</li>
                                )}
                              </ul>
                            </div>
                          )}
                        </div>
                      ))}
                    </div>
                  </div>
                )}

                {/* Vehicle Status */}
                {version.vehicles.length > 0 && (
                  <div className="mb-4">
                    <h4 className="text-sm font-medium text-gray-900 mb-2">Vehicle Status</h4>
                    <div className="grid grid-cols-1 md:grid-cols-2 gap-2">
                      {version.vehicles
                        .filter(vehicle => !vehicleId || vehicle.vehicleId === vehicleId)
                        .map(vehicle => (
                        <div key={vehicle.vehicleId} className="flex items-center justify-between p-2 bg-gray-50 rounded">
                          <div>
                            <div className="font-medium text-sm">{vehicle.vehicleName}</div>
                            <div className="text-xs text-gray-600">
                              {vehicle.currentVersion}
                              {vehicle.targetVersion && vehicle.targetVersion !== vehicle.currentVersion && (
                                <span> → {vehicle.targetVersion}</span>
                              )}
                            </div>
                          </div>
                          <div className="flex items-center space-x-2">
                            <Badge size="sm" className={
                              vehicle.updateStatus === 'synced' ? 'bg-green-100 text-green-800' :
                              vehicle.updateStatus === 'updating' ? 'bg-blue-100 text-blue-800' :
                              vehicle.updateStatus === 'pending' ? 'bg-yellow-100 text-yellow-800' :
                              vehicle.updateStatus === 'failed' ? 'bg-red-100 text-red-800' :
                              'bg-gray-100 text-gray-800'
                            }>
                              {vehicle.updateStatus}
                            </Badge>
                            {vehicle.updateProgress !== undefined && vehicle.updateProgress < 100 && (
                              <div className="w-16">
                                <Progress value={vehicle.updateProgress} className="h-1" />
                              </div>
                            )}
                          </div>
                        </div>
                      ))}
                    </div>
                  </div>
                )}

                {/* Actions */}
                <div className="flex items-center justify-between pt-4 border-t">
                  <div className="text-sm text-gray-600">
                    {version.availableVersions.length > 0 && (
                      <span>{version.availableVersions.length} update(s) available</span>
                    )}
                  </div>
                  <div className="flex items-center space-x-2">
                    {version.availableVersions.length > 0 && (
                      <Button
                        size="sm"
                        onClick={() => handleUpdate(version, version.availableVersions[0])}
                        disabled={!currentUser.permissions.canUpdate || !currentUser.permissions.components.includes(version.component)}
                      >
                        <Download className="w-4 h-4 mr-1" />
                        Update
                      </Button>
                    )}
                    
                    {version.rollbackVersion && (
                      <Button
                        size="sm"
                        variant="outline"
                        onClick={() => handleRollback(version)}
                        disabled={!currentUser.permissions.canRollback || !currentUser.permissions.components.includes(version.component)}
                      >
                        <RotateCcw className="w-4 h-4 mr-1" />
                        Rollback
                      </Button>
                    )}
                    
                    {version.status === 'pinned' ? (
                      <Button
                        size="sm"
                        variant="outline"
                        onClick={() => executeVersionOperation('unpin', version)}
                        disabled={!currentUser.permissions.canPin || !currentUser.permissions.components.includes(version.component)}
                      >
                        <Unlock className="w-4 h-4 mr-1" />
                        Unpin
                      </Button>
                    ) : (
                      <Button
                        size="sm"
                        variant="outline"
                        onClick={() => handlePin(version)}
                        disabled={!currentUser.permissions.canPin || !currentUser.permissions.components.includes(version.component)}
                      >
                        <Lock className="w-4 h-4 mr-1" />
                        Pin
                      </Button>
                    )}
                  </div>
                </div>
              </Card>
            ))}
          </div>
        </TabsContent>

        <TabsContent value="operations" className="space-y-4">
          <div className="space-y-4">
            {operations.map(operation => (
              <Card key={operation.id} className="p-4">
                <div className="flex items-start justify-between">
                  <div className="flex items-start space-x-3">
                    <div className="p-2 bg-gray-100 rounded">
                      {operation.type === 'update' && <Download className="w-4 h-4" />}
                      {operation.type === 'rollback' && <RotateCcw className="w-4 h-4" />}
                      {operation.type === 'pin' && <Lock className="w-4 h-4" />}
                      {operation.type === 'unpin' && <Unlock className="w-4 h-4" />}
                    </div>
                    <div className="flex-1">
                      <div className="flex items-center space-x-3 mb-2">
                        <h4 className="font-medium text-gray-900 capitalize">
                          {operation.type} {operation.component}
                        </h4>
                        <Badge className={getOperationStatusColor(operation.status)}>
                          {operation.status.replace('_', ' ')}
                        </Badge>
                      </div>
                      
                      <div className="text-sm text-gray-600 mb-2">
                        {operation.fromVersion} → {operation.toVersion}
                      </div>
                      
                      <div className="text-sm text-gray-600 mb-2">
                        Reason: {operation.reason}
                      </div>
                      
                      <div className="grid grid-cols-2 md:grid-cols-4 gap-4 text-sm">
                        <div>
                          <span className="text-gray-500">Operator:</span>
                          <div className="font-medium">{operation.operator}</div>
                        </div>
                        <div>
                          <span className="text-gray-500">Started:</span>
                          <div className="font-medium">{operation.startTime.toLocaleString()}</div>
                        </div>
                        <div>
                          <span className="text-gray-500">Duration:</span>
                          <div className="font-medium">
                            {operation.endTime ? 
                              Math.round((operation.endTime.getTime() - operation.startTime.getTime()) / 60000) + 'm' :
                              Math.round((new Date().getTime() - operation.startTime.getTime()) / 60000) + 'm'
                            }
                          </div>
                        </div>
                        <div>
                          <span className="text-gray-500">Vehicles:</span>
                          <div className="font-medium">{operation.affectedVehicles.length}</div>
                        </div>
                      </div>
                      
                      {operation.approver && (
                        <div className="text-sm text-green-600 mt-2">
                          Approved by: {operation.approver}
                        </div>
                      )}
                    </div>
                  </div>
                  
                  <div className="text-right">
                    {operation.status === 'in_progress' && (
                      <div className="w-24">
                        <div className="text-sm text-gray-600 mb-1">{Math.round(operation.progress)}%</div>
                        <Progress value={operation.progress} className="h-2" />
                      </div>
                    )}
                    {operation.status === 'completed' && (
                      <CheckCircle className="w-6 h-6 text-green-600" />
                    )}
                    {operation.status === 'failed' && (
                      <X className="w-6 h-6 text-red-600" />
                    )}
                  </div>
                </div>
              </Card>
            ))}
            
            {operations.length === 0 && (
              <div className="text-center py-12">
                <History className="w-16 h-16 text-gray-400 mx-auto mb-4" />
                <h3 className="text-lg font-medium text-gray-900 mb-2">No Operations History</h3>
                <p className="text-gray-600">Version operations will appear here</p>
              </div>
            )}
          </div>
        </TabsContent>

        <TabsContent value="rbac" className="space-y-4">
          <Card className="p-6">
            <h3 className="text-lg font-medium text-gray-900 mb-4">Current User Permissions</h3>
            <div className="grid grid-cols-2 gap-6">
              <div>
                <div className="text-sm text-gray-600 mb-2">User Information</div>
                <div className="space-y-1 text-sm">
                  <div><span className="font-medium">Name:</span> {currentUser.name}</div>
                  <div><span className="font-medium">Role:</span> {currentUser.role}</div>
                </div>
              </div>
              <div>
                <div className="text-sm text-gray-600 mb-2">Permissions</div>
                <div className="space-y-1 text-sm">
                  <div className="flex items-center space-x-2">
                    {currentUser.permissions.canUpdate ? <Check className="w-4 h-4 text-green-600" /> : <X className="w-4 h-4 text-red-600" />}
                    <span>Can Update Components</span>
                  </div>
                  <div className="flex items-center space-x-2">
                    {currentUser.permissions.canRollback ? <Check className="w-4 h-4 text-green-600" /> : <X className="w-4 h-4 text-red-600" />}
                    <span>Can Rollback Components</span>
                  </div>
                  <div className="flex items-center space-x-2">
                    {currentUser.permissions.canPin ? <Check className="w-4 h-4 text-green-600" /> : <X className="w-4 h-4 text-red-600" />}
                    <span>Can Pin/Unpin Versions</span>
                  </div>
                  <div className="flex items-center space-x-2">
                    {currentUser.permissions.canApprove ? <Check className="w-4 h-4 text-green-600" /> : <X className="w-4 h-4 text-red-600" />}
                    <span>Can Approve Operations</span>
                  </div>
                </div>
              </div>
            </div>
            <div className="mt-4">
              <div className="text-sm text-gray-600 mb-2">Accessible Components</div>
              <div className="flex flex-wrap gap-2">
                {currentUser.permissions.components.map(component => (
                  <Badge key={component} variant="outline" className="capitalize">
                    {component}
                  </Badge>
                ))}
              </div>
            </div>
          </Card>
        </TabsContent>
      </Tabs>

      {/* Update Dialog */}
      <Dialog open={showUpdateDialog} onOpenChange={setShowUpdateDialog}>
        <DialogContent className="max-w-2xl">
          <DialogHeader>
            <DialogTitle>Update Component</DialogTitle>
          </DialogHeader>
          
          {selectedComponent && selectedVersion && (
            <div className="space-y-4">
              <div className="p-4 bg-blue-50 rounded-lg border border-blue-200">
                <div className="flex items-center space-x-3 mb-2">
                  {getComponentIcon(selectedComponent.component)}
                  <div>
                    <div className="font-medium text-blue-900">{selectedComponent.name}</div>
                    <div className="text-sm text-blue-700">
                      {selectedComponent.currentVersion} → {selectedVersion.version}
                    </div>
                  </div>
                </div>
                
                {selectedVersion.criticalUpdate && (
                  <Alert className="mt-3">
                    <AlertTriangle className="w-4 h-4" />
                    <AlertDescription>
                      This is a critical update that should be applied immediately for security or safety reasons.
                    </AlertDescription>
                  </Alert>
                )}
              </div>

              <div>
                <h4 className="font-medium text-gray-900 mb-2">Update Details</h4>
                <div className="text-sm text-gray-600 space-y-1">
                  <div>Version: {selectedVersion.version}</div>
                  <div>Size: {selectedVersion.size}MB</div>
                  <div>Release Date: {selectedVersion.releaseDate.toLocaleDateString()}</div>
                  <div>Stability: {selectedVersion.stability}</div>
                </div>
              </div>

              <div>
                <h4 className="font-medium text-gray-900 mb-2">Description</h4>
                <p className="text-sm text-gray-600">{selectedVersion.description}</p>
              </div>

              {selectedVersion.changelog.length > 0 && (
                <div>
                  <h4 className="font-medium text-gray-900 mb-2">Changelog</h4>
                  <ul className="text-sm text-gray-600 list-disc list-inside space-y-0.5">
                    {selectedVersion.changelog.map((change, index) => (
                      <li key={index}>{change}</li>
                    ))}
                  </ul>
                </div>
              )}

              <div>
                <label className="block text-sm font-medium text-gray-700 mb-2">
                  Reason for Update *
                </label>
                <textarea
                  value={operationReason}
                  onChange={(e) => setOperationReason(e.target.value)}
                  placeholder="Describe why this update is being applied..."
                  className="w-full p-3 border border-gray-300 rounded-md"
                  rows={3}
                  required
                />
              </div>

              {requiresApproval && (
                <Alert>
                  <Shield className="w-4 h-4" />
                  <AlertDescription>
                    This operation requires RBAC approval due to the critical nature of the update.
                  </AlertDescription>
                </Alert>
              )}

              <div className="flex justify-end space-x-3 pt-4">
                <Button
                  variant="outline"
                  onClick={() => setShowUpdateDialog(false)}
                >
                  Cancel
                </Button>
                <Button
                  onClick={() => executeVersionOperation('update', selectedComponent, selectedVersion.version)}
                  disabled={!operationReason.trim()}
                >
                  <Play className="w-4 h-4 mr-2" />
                  Apply Update
                </Button>
              </div>
            </div>
          )}
        </DialogContent>
      </Dialog>

      {/* Rollback Dialog */}
      <Dialog open={showRollbackDialog} onOpenChange={setShowRollbackDialog}>
        <DialogContent className="max-w-2xl">
          <DialogHeader>
            <DialogTitle>Rollback Component</DialogTitle>
          </DialogHeader>
          
          {selectedComponent && (
            <div className="space-y-4">
              <Alert className="border-yellow-200 bg-yellow-50">
                <AlertTriangle className="w-4 h-4 text-yellow-600" />
                <AlertDescription className="text-yellow-800">
                  Rolling back will revert {selectedComponent.name} from {selectedComponent.currentVersion} to {selectedComponent.rollbackVersion}.
                  This action cannot be undone automatically.
                </AlertDescription>
              </Alert>

              <div>
                <label className="block text-sm font-medium text-gray-700 mb-2">
                  Reason for Rollback *
                </label>
                <textarea
                  value={operationReason}
                  onChange={(e) => setOperationReason(e.target.value)}
                  placeholder="Describe why this rollback is necessary..."
                  className="w-full p-3 border border-gray-300 rounded-md"
                  rows={3}
                  required
                />
              </div>

              {requiresApproval && (
                <Alert>
                  <Shield className="w-4 h-4" />
                  <AlertDescription>
                    This rollback operation requires RBAC approval.
                  </AlertDescription>
                </Alert>
              )}

              <div className="flex justify-end space-x-3 pt-4">
                <Button
                  variant="outline"
                  onClick={() => setShowRollbackDialog(false)}
                >
                  Cancel
                </Button>
                <Button
                  onClick={() => executeVersionOperation('rollback', selectedComponent)}
                  disabled={!operationReason.trim()}
                  className="bg-yellow-600 hover:bg-yellow-700"
                >
                  <RotateCcw className="w-4 h-4 mr-2" />
                  Confirm Rollback
                </Button>
              </div>
            </div>
          )}
        </DialogContent>
      </Dialog>

      {/* Pin Dialog */}
      <Dialog open={showPinDialog} onOpenChange={setShowPinDialog}>
        <DialogContent className="max-w-2xl">
          <DialogHeader>
            <DialogTitle>Pin Component Version</DialogTitle>
          </DialogHeader>
          
          {selectedComponent && (
            <div className="space-y-4">
              <Alert className="border-purple-200 bg-purple-50">
                <Lock className="w-4 h-4 text-purple-600" />
                <AlertDescription className="text-purple-800">
                  Pinning will prevent {selectedComponent.name} from receiving automatic updates.
                  The component will remain at version {selectedComponent.currentVersion} until manually unpinned.
                </AlertDescription>
              </Alert>

              <div>
                <label className="block text-sm font-medium text-gray-700 mb-2">
                  Reason for Pinning *
                </label>
                <textarea
                  value={operationReason}
                  onChange={(e) => setOperationReason(e.target.value)}
                  placeholder="Describe why this version should be pinned..."
                  className="w-full p-3 border border-gray-300 rounded-md"
                  rows={3}
                  required
                />
              </div>

              {requiresApproval && (
                <Alert>
                  <Shield className="w-4 h-4" />
                  <AlertDescription>
                    This pin operation requires RBAC approval.
                  </AlertDescription>
                </Alert>
              )}

              <div className="flex justify-end space-x-3 pt-4">
                <Button
                  variant="outline"
                  onClick={() => setShowPinDialog(false)}
                >
                  Cancel
                </Button>
                <Button
                  onClick={() => executeVersionOperation('pin', selectedComponent)}
                  disabled={!operationReason.trim()}
                  className="bg-purple-600 hover:bg-purple-700"
                >
                  <Lock className="w-4 h-4 mr-2" />
                  Pin Version
                </Button>
              </div>
            </div>
          )}
        </DialogContent>
      </Dialog>
    </div>
  )
}

export default VersionManager
