import React, { useState, useCallback, useMemo, useEffect } from 'react'
import { 
  HardDrive, Server, Cpu, Thermometer, Zap, Wifi, AlertTriangle, CheckCircle,
  RefreshCw, Upload, Download, Shield, Lock, Eye, Settings, Activity, Gauge,
  MemoryStick, Database, Network, Clock, TrendingUp, TrendingDown, Play, Pause,
  Power, PowerOff, RotateCcw, Archive, Trash2, FileText, AlertCircle, Info
} from 'lucide-react'
import { Button } from '../components/ui/Button'
import { Badge } from '../components/ui/Badge'
import { Card } from '../components/ui/Card'
import { Input } from '../components/ui/Input'
import { Select } from '../components/ui/Select'
import { Tabs, TabsContent, TabsList, TabsTrigger } from '../components/ui/Tabs'
import { Dialog, DialogContent, DialogHeader, DialogTitle } from '../components/ui/Dialog'
import { Alert, AlertDescription } from '../components/ui/Alert'
import { Progress } from '../components/ui/Progress'
import { ScrollArea } from '../components/ui/ScrollArea'
import { Switch } from '../components/ui/Switch'
import { Tooltip, TooltipContent, TooltipProvider, TooltipTrigger } from '../components/ui/Tooltip'

// Types
interface GarageNode {
  id: string
  name: string
  location: string
  status: 'online' | 'offline' | 'maintenance' | 'error'
  type: 'primary' | 'backup' | 'edge'
  specs: {
    cpu: string
    memory: string
    storage: string
    network: string
  }
  health: {
    cpu: number
    memory: number
    temperature: number
    network: number
    overall: number
  }
  storage: {
    total: number
    used: number
    available: number
    utilization: number
  }
  network: {
    bandwidth: number
    latency: number
    packetLoss: number
    connections: number
  }
  services: GarageService[]
  alerts: GarageAlert[]
  lastUpdate: Date
}

interface GarageService {
  id: string
  name: string
  status: 'running' | 'stopped' | 'error' | 'starting'
  port: number
  cpu: number
  memory: number
  uptime: string
  version: string
}

interface GarageAlert {
  id: string
  severity: 'info' | 'warning' | 'critical' | 'emergency'
  type: 'hardware' | 'software' | 'network' | 'security'
  message: string
  timestamp: Date
  acknowledged: boolean
}

interface ImageDeployment {
  id: string
  name: string
  version: string
  status: 'pending' | 'uploading' | 'verifying' | 'deploying' | 'completed' | 'failed'
  progress: number
  targetNodes: string[]
  checksum: string
  size: number
  startTime: Date
  endTime?: Date
}

// Mock data
const mockNodes: GarageNode[] = [
  {
    id: 'node1',
    name: 'Dubai-Main-01',
    location: 'Dubai Main Depot',
    status: 'online',
    type: 'primary',
    specs: {
      cpu: 'Intel Xeon E5-2680 v4',
      memory: '128GB DDR4',
      storage: '10TB NVMe RAID',
      network: '10Gb Ethernet'
    },
    health: {
      cpu: 45,
      memory: 68,
      temperature: 42,
      network: 95,
      overall: 88
    },
    storage: {
      total: 10240,
      used: 6144,
      available: 4096,
      utilization: 60
    },
    network: {
      bandwidth: 8.5,
      latency: 2,
      packetLoss: 0.01,
      connections: 156
    },
    services: [
      {
        id: 's1',
        name: 'Image Registry',
        status: 'running',
        port: 5000,
        cpu: 12,
        memory: 2048,
        uptime: '15d 4h 23m',
        version: 'v2.8.1'
      },
      {
        id: 's2',
        name: 'OTA Manager',
        status: 'running',
        port: 8080,
        cpu: 8,
        memory: 1536,
        uptime: '15d 4h 23m',
        version: 'v1.4.2'
      }
    ],
    alerts: [
      {
        id: 'a1',
        severity: 'warning',
        type: 'hardware',
        message: 'Disk temperature elevated on /dev/sda3',
        timestamp: new Date(Date.now() - 30 * 60 * 1000),
        acknowledged: false
      }
    ],
    lastUpdate: new Date()
  },
  {
    id: 'node2',
    name: 'Abu-Dhabi-01',
    location: 'Abu Dhabi Service Center',
    status: 'online',
    type: 'backup',
    specs: {
      cpu: 'Intel Xeon Silver 4214',
      memory: '64GB DDR4',
      storage: '5TB NVMe RAID',
      network: '1Gb Ethernet'
    },
    health: {
      cpu: 32,
      memory: 54,
      temperature: 38,
      network: 92,
      overall: 92
    },
    storage: {
      total: 5120,
      used: 2048,
      available: 3072,
      utilization: 40
    },
    network: {
      bandwidth: 0.8,
      latency: 5,
      packetLoss: 0.02,
      connections: 89
    },
    services: [
      {
        id: 's3',
        name: 'Backup Service',
        status: 'running',
        port: 9000,
        cpu: 5,
        memory: 1024,
        uptime: '8d 12h 15m',
        version: 'v1.2.0'
      }
    ],
    alerts: [],
    lastUpdate: new Date(Date.now() - 45 * 1000)
  },
  {
    id: 'node3',
    name: 'Sharjah-Edge-01',
    location: 'Sharjah Maintenance Hub',
    status: 'error',
    type: 'edge',
    specs: {
      cpu: 'Intel Core i7-10700K',
      memory: '32GB DDR4',
      storage: '2TB NVMe',
      network: '1Gb Ethernet'
    },
    health: {
      cpu: 85,
      memory: 92,
      temperature: 78,
      network: 45,
      overall: 35
    },
    storage: {
      total: 2048,
      used: 1843,
      available: 205,
      utilization: 90
    },
    network: {
      bandwidth: 0.2,
      latency: 45,
      packetLoss: 2.5,
      connections: 12
    },
    services: [
      {
        id: 's4',
        name: 'Edge Sync',
        status: 'error',
        port: 7000,
        cpu: 45,
        memory: 512,
        uptime: '0m',
        version: 'v0.9.1'
      }
    ],
    alerts: [
      {
        id: 'a2',
        severity: 'critical',
        type: 'hardware',
        message: 'Critical temperature threshold exceeded',
        timestamp: new Date(Date.now() - 10 * 60 * 1000),
        acknowledged: false
      },
      {
        id: 'a3',
        severity: 'critical',
        type: 'hardware',
        message: 'Storage utilization above 90%',
        timestamp: new Date(Date.now() - 15 * 60 * 1000),
        acknowledged: false
      }
    ],
    lastUpdate: new Date(Date.now() - 5 * 60 * 1000)
  }
]

const mockDeployments: ImageDeployment[] = [
  {
    id: 'd1',
    name: 'atlas-os-v2.1.0',
    version: 'v2.1.0',
    status: 'deploying',
    progress: 75,
    targetNodes: ['node1', 'node2'],
    checksum: 'sha256:a1b2c3d4...',
    size: 2048,
    startTime: new Date(Date.now() - 30 * 60 * 1000)
  },
  {
    id: 'd2',
    name: 'security-patch-202412',
    version: 'patch-202412',
    status: 'completed',
    progress: 100,
    targetNodes: ['node1'],
    checksum: 'sha256:e5f6g7h8...',
    size: 156,
    startTime: new Date(Date.now() - 2 * 60 * 60 * 1000),
    endTime: new Date(Date.now() - 90 * 60 * 1000)
  }
]

const GarageManagement: React.FC = () => {
  // State
  const [nodes, setNodes] = useState<GarageNode[]>(mockNodes)
  const [deployments, setDeployments] = useState<ImageDeployment[]>(mockDeployments)
  const [selectedNode, setSelectedNode] = useState<GarageNode | null>(null)
  const [showNodeDetail, setShowNodeDetail] = useState(false)
  const [showImageUpload, setShowImageUpload] = useState(false)
  const [showDeployDialog, setShowDeployDialog] = useState(false)
  const [activeTab, setActiveTab] = useState('overview')
  const [autoRefresh, setAutoRefresh] = useState(true)
  const [refreshInterval, setRefreshInterval] = useState(10) // seconds

  // Real-time updates simulation
  useEffect(() => {
    if (!autoRefresh) return

    const interval = setInterval(() => {
      setNodes(prev => prev.map(node => ({
        ...node,
        health: {
          ...node.health,
          cpu: Math.max(10, Math.min(95, node.health.cpu + (Math.random() - 0.5) * 10)),
          memory: Math.max(20, Math.min(95, node.health.memory + (Math.random() - 0.5) * 5)),
          temperature: Math.max(25, Math.min(85, node.health.temperature + (Math.random() - 0.5) * 3))
        },
        network: {
          ...node.network,
          latency: Math.max(1, node.network.latency + (Math.random() - 0.5) * 2),
          bandwidth: Math.max(0.1, node.network.bandwidth + (Math.random() - 0.5) * 0.5)
        },
        lastUpdate: new Date()
      })))

      // Update deployment progress
      setDeployments(prev => prev.map(deployment => {
        if (deployment.status === 'deploying' && deployment.progress < 100) {
          const newProgress = Math.min(100, deployment.progress + Math.random() * 5)
          return {
            ...deployment,
            progress: newProgress,
            status: newProgress === 100 ? 'completed' : deployment.status,
            endTime: newProgress === 100 ? new Date() : deployment.endTime
          }
        }
        return deployment
      }))
    }, refreshInterval * 1000)

    return () => clearInterval(interval)
  }, [autoRefresh, refreshInterval])

  // Computed values
  const stats = useMemo(() => {
    const totalNodes = nodes.length
    const onlineNodes = nodes.filter(n => n.status === 'online').length
    const errorNodes = nodes.filter(n => n.status === 'error').length
    const avgHealth = Math.round(nodes.reduce((sum, n) => sum + n.health.overall, 0) / totalNodes)
    const totalStorage = nodes.reduce((sum, n) => sum + n.storage.total, 0)
    const usedStorage = nodes.reduce((sum, n) => sum + n.storage.used, 0)
    const storageUtilization = Math.round((usedStorage / totalStorage) * 100)
    const activeAlerts = nodes.flatMap(n => n.alerts).filter(a => !a.acknowledged).length
    const runningServices = nodes.flatMap(n => n.services).filter(s => s.status === 'running').length

    return {
      totalNodes,
      onlineNodes,
      errorNodes,
      avgHealth,
      storageUtilization,
      activeAlerts,
      runningServices
    }
  }, [nodes])

  // Handlers
  const handleNodeClick = useCallback((nodeId: string) => {
    const node = nodes.find(n => n.id === nodeId)
    if (node) {
      setSelectedNode(node)
      setShowNodeDetail(true)
    }
  }, [nodes])

  const handleServiceAction = useCallback((nodeId: string, serviceId: string, action: string) => {
    setNodes(prev => prev.map(node => {
      if (node.id === nodeId) {
        return {
          ...node,
          services: node.services.map(service => {
            if (service.id === serviceId) {
              switch (action) {
                case 'start':
                  return { ...service, status: 'starting' as const }
                case 'stop':
                  return { ...service, status: 'stopped' as const }
                case 'restart':
                  return { ...service, status: 'starting' as const }
                default:
                  return service
              }
            }
            return service
          })
        }
      }
      return node
    }))
  }, [])

  const handleAcknowledgeAlert = useCallback((nodeId: string, alertId: string) => {
    setNodes(prev => prev.map(node => {
      if (node.id === nodeId) {
        return {
          ...node,
          alerts: node.alerts.map(alert => 
            alert.id === alertId ? { ...alert, acknowledged: true } : alert
          )
        }
      }
      return node
    }))
  }, [])

  const getStatusColor = (status: string) => {
    switch (status) {
      case 'online': return 'text-green-600 bg-green-100'
      case 'offline': return 'text-gray-600 bg-gray-100'
      case 'maintenance': return 'text-yellow-600 bg-yellow-100'
      case 'error': return 'text-red-600 bg-red-100'
      case 'running': return 'text-green-600 bg-green-100'
      case 'stopped': return 'text-red-600 bg-red-100'
      case 'starting': return 'text-blue-600 bg-blue-100'
      default: return 'text-gray-600 bg-gray-100'
    }
  }

  const getHealthColor = (value: number) => {
    if (value >= 80) return 'text-green-600'
    if (value >= 60) return 'text-yellow-600'
    return 'text-red-600'
  }

  const getDeploymentStatusColor = (status: string) => {
    switch (status) {
      case 'completed': return 'text-green-600 bg-green-100'
      case 'deploying': return 'text-blue-600 bg-blue-100'
      case 'failed': return 'text-red-600 bg-red-100'
      case 'pending': return 'text-gray-600 bg-gray-100'
      default: return 'text-gray-600 bg-gray-100'
    }
  }

  return (
    <div className="flex flex-col h-screen bg-gray-50">
      {/* Header */}
      <div className="bg-white border-b border-gray-200 px-6 py-4">
        <div className="flex items-center justify-between">
          <div className="flex items-center space-x-4">
            <Server className="w-6 h-6 text-blue-600" />
            <h1 className="text-2xl font-semibold text-gray-900">Garage PC Management</h1>
          </div>

          <div className="flex items-center space-x-3">
            {/* Auto Refresh Controls */}
            <div className="flex items-center space-x-2">
              <Switch checked={autoRefresh} onCheckedChange={setAutoRefresh} />
              <span className="text-sm text-gray-600">Auto Refresh</span>
              <Select
                value={refreshInterval.toString()}
                onValueChange={(value) => setRefreshInterval(parseInt(value))}
                disabled={!autoRefresh}
              >
                <option value="5">5s</option>
                <option value="10">10s</option>
                <option value="30">30s</option>
                <option value="60">1m</option>
              </Select>
            </div>

            {/* Actions */}
            <Button
              variant="outline"
              onClick={() => setShowImageUpload(true)}
            >
              <Upload className="w-4 h-4 mr-2" />
              Upload Image
            </Button>

            <Button
              onClick={() => setShowDeployDialog(true)}
            >
              <RefreshCw className="w-4 h-4 mr-2" />
              Deploy Update
            </Button>
          </div>
        </div>

        {/* Statistics */}
        <div className="grid grid-cols-7 gap-4 mt-4">
          <Card className="p-3">
            <div className="text-center">
              <div className="text-2xl font-bold text-gray-900">{stats.totalNodes}</div>
              <div className="text-sm text-gray-600">Total Nodes</div>
            </div>
          </Card>
          <Card className="p-3">
            <div className="text-center">
              <div className="text-2xl font-bold text-green-600">{stats.onlineNodes}</div>
              <div className="text-sm text-gray-600">Online</div>
            </div>
          </Card>
          <Card className="p-3">
            <div className="text-center">
              <div className={`text-2xl font-bold ${stats.errorNodes > 0 ? 'text-red-600' : 'text-green-600'}`}>
                {stats.errorNodes}
              </div>
              <div className="text-sm text-gray-600">Errors</div>
            </div>
          </Card>
          <Card className="p-3">
            <div className="text-center">
              <div className={`text-2xl font-bold ${getHealthColor(stats.avgHealth)}`}>
                {stats.avgHealth}%
              </div>
              <div className="text-sm text-gray-600">Avg Health</div>
            </div>
          </Card>
          <Card className="p-3">
            <div className="text-center">
              <div className={`text-2xl font-bold ${
                stats.storageUtilization > 80 ? 'text-red-600' : 
                stats.storageUtilization > 60 ? 'text-yellow-600' : 'text-green-600'
              }`}>
                {stats.storageUtilization}%
              </div>
              <div className="text-sm text-gray-600">Storage Used</div>
            </div>
          </Card>
          <Card className="p-3">
            <div className="text-center">
              <div className={`text-2xl font-bold ${stats.activeAlerts > 0 ? 'text-red-600' : 'text-green-600'}`}>
                {stats.activeAlerts}
              </div>
              <div className="text-sm text-gray-600">Active Alerts</div>
            </div>
          </Card>
          <Card className="p-3">
            <div className="text-center">
              <div className="text-2xl font-bold text-blue-600">{stats.runningServices}</div>
              <div className="text-sm text-gray-600">Services</div>
            </div>
          </Card>
        </div>
      </div>

      {/* Main Content */}
      <div className="flex-1 overflow-hidden">
        <Tabs value={activeTab} onValueChange={setActiveTab} className="h-full flex flex-col">
          <div className="px-6 pt-4">
            <TabsList>
              <TabsTrigger value="overview">Node Overview</TabsTrigger>
              <TabsTrigger value="deployments">Image Deployments</TabsTrigger>
              <TabsTrigger value="monitoring">System Monitoring</TabsTrigger>
              <TabsTrigger value="security">Security & Access</TabsTrigger>
            </TabsList>
          </div>

          <div className="flex-1 overflow-hidden">
            <TabsContent value="overview" className="h-full">
              <ScrollArea className="h-full">
                <div className="p-6">
                  <div className="grid grid-cols-1 lg:grid-cols-2 xl:grid-cols-3 gap-6">
                    {nodes.map(node => {
                      const isStale = new Date().getTime() - node.lastUpdate.getTime() > 60000
                      const criticalAlerts = node.alerts.filter(a => a.severity === 'critical' || a.severity === 'emergency')

                      return (
                        <Card
                          key={node.id}
                          className={`cursor-pointer transition-all hover:shadow-md ${
                            isStale ? 'border-yellow-300 bg-yellow-50' : ''
                          }`}
                          onClick={() => handleNodeClick(node.id)}
                        >
                          <div className="p-6">
                            <div className="flex items-center justify-between mb-4">
                              <div>
                                <div className="font-medium text-gray-900">{node.name}</div>
                                <div className="text-sm text-gray-500">{node.location}</div>
                              </div>
                              <div className="flex items-center space-x-2">
                                <Badge className={getStatusColor(node.status)}>
                                  {node.status}
                                </Badge>
                                <Badge variant="outline" className={
                                  node.type === 'primary' ? 'border-blue-300 text-blue-700' :
                                  node.type === 'backup' ? 'border-green-300 text-green-700' :
                                  'border-purple-300 text-purple-700'
                                }>
                                  {node.type}
                                </Badge>
                              </div>
                            </div>

                            {/* Health Metrics */}
                            <div className="space-y-3 mb-4">
                              <div className="flex items-center justify-between text-sm">
                                <div className="flex items-center space-x-2">
                                  <Cpu className="w-4 h-4 text-gray-500" />
                                  <span>CPU</span>
                                </div>
                                <div className="flex items-center space-x-2">
                                  <Progress value={node.health.cpu} className="w-16 h-2" />
                                  <span className={getHealthColor(100 - node.health.cpu)}>
                                    {node.health.cpu}%
                                  </span>
                                </div>
                              </div>

                              <div className="flex items-center justify-between text-sm">
                                <div className="flex items-center space-x-2">
                                  <MemoryStick className="w-4 h-4 text-gray-500" />
                                  <span>Memory</span>
                                </div>
                                <div className="flex items-center space-x-2">
                                  <Progress value={node.health.memory} className="w-16 h-2" />
                                  <span className={getHealthColor(100 - node.health.memory)}>
                                    {node.health.memory}%
                                  </span>
                                </div>
                              </div>

                              <div className="flex items-center justify-between text-sm">
                                <div className="flex items-center space-x-2">
                                  <HardDrive className="w-4 h-4 text-gray-500" />
                                  <span>Storage</span>
                                </div>
                                <div className="flex items-center space-x-2">
                                  <Progress value={node.storage.utilization} className="w-16 h-2" />
                                  <span className={
                                    node.storage.utilization > 80 ? 'text-red-600' :
                                    node.storage.utilization > 60 ? 'text-yellow-600' : 'text-green-600'
                                  }>
                                    {node.storage.utilization}%
                                  </span>
                                </div>
                              </div>

                              <div className="flex items-center justify-between text-sm">
                                <div className="flex items-center space-x-2">
                                  <Thermometer className="w-4 h-4 text-gray-500" />
                                  <span>Temperature</span>
                                </div>
                                <span className={
                                  node.health.temperature > 70 ? 'text-red-600' :
                                  node.health.temperature > 50 ? 'text-yellow-600' : 'text-green-600'
                                }>
                                  {node.health.temperature}°C
                                </span>
                              </div>
                            </div>

                            {/* Network Status */}
                            <div className="p-3 bg-gray-50 rounded-lg mb-4">
                              <div className="flex items-center justify-between text-sm">
                                <div className="flex items-center space-x-2">
                                  <Network className="w-4 h-4 text-gray-500" />
                                  <span>Network</span>
                                </div>
                                <div className="text-right">
                                  <div>{node.network.bandwidth.toFixed(1)} Gbps</div>
                                  <div className="text-xs text-gray-500">{node.network.latency}ms latency</div>
                                </div>
                              </div>
                            </div>

                            {/* Services */}
                            <div className="mb-4">
                              <div className="text-sm font-medium text-gray-900 mb-2">Services</div>
                              <div className="space-y-1">
                                {node.services.map(service => (
                                  <div key={service.id} className="flex items-center justify-between text-sm">
                                    <span>{service.name}</span>
                                    <Badge className={getStatusColor(service.status)} size="sm">
                                      {service.status}
                                    </Badge>
                                  </div>
                                ))}
                              </div>
                            </div>

                            {/* Alerts */}
                            {node.alerts.length > 0 && (
                              <div className="space-y-2">
                                {node.alerts.slice(0, 2).map(alert => (
                                  <div
                                    key={alert.id}
                                    className={`flex items-center space-x-2 text-xs p-2 rounded ${
                                      alert.severity === 'critical' || alert.severity === 'emergency'
                                        ? 'bg-red-50 text-red-700 border border-red-200'
                                        : 'bg-yellow-50 text-yellow-700 border border-yellow-200'
                                    }`}
                                  >
                                    <AlertTriangle className="w-3 h-3 flex-shrink-0" />
                                    <span className="flex-1">{alert.message}</span>
                                    {!alert.acknowledged && (
                                      <div className="w-2 h-2 bg-red-500 rounded-full animate-pulse" />
                                    )}
                                  </div>
                                ))}
                                {node.alerts.length > 2 && (
                                  <div className="text-xs text-gray-500 text-center">
                                    +{node.alerts.length - 2} more alerts
                                  </div>
                                )}
                              </div>
                            )}

                            {/* Last Update */}
                            <div className="mt-4 pt-3 border-t border-gray-200">
                              <div className="flex items-center justify-between text-xs text-gray-500">
                                <div className="flex items-center space-x-1">
                                  <Clock className="w-3 h-3" />
                                  <span>Last update: {Math.round((new Date().getTime() - node.lastUpdate.getTime()) / 1000)}s ago</span>
                                </div>
                                <Button variant="ghost" size="sm">
                                  <Eye className="w-3 h-3" />
                                </Button>
                              </div>
                            </div>
                          </div>
                        </Card>
                      )
                    })}
                  </div>
                </div>
              </ScrollArea>
            </TabsContent>

            <TabsContent value="deployments" className="h-full">
              <ScrollArea className="h-full">
                <div className="p-6">
                  <div className="space-y-6">
                    {/* Active Deployments */}
                    <div>
                      <h3 className="text-lg font-medium text-gray-900 mb-4">Active Deployments</h3>
                      <div className="space-y-4">
                        {deployments.filter(d => d.status !== 'completed' && d.status !== 'failed').map(deployment => (
                          <Card key={deployment.id} className="p-4">
                            <div className="flex items-center justify-between mb-3">
                              <div>
                                <div className="font-medium text-gray-900">{deployment.name}</div>
                                <div className="text-sm text-gray-600">
                                  Version: {deployment.version} • Size: {deployment.size}MB
                                </div>
                              </div>
                              <Badge className={getDeploymentStatusColor(deployment.status)}>
                                {deployment.status}
                              </Badge>
                            </div>

                            <div className="space-y-3">
                              <div>
                                <div className="flex items-center justify-between text-sm mb-1">
                                  <span>Progress</span>
                                  <span>{deployment.progress}%</span>
                                </div>
                                <Progress value={deployment.progress} className="w-full h-2" />
                              </div>

                              <div className="text-sm text-gray-600">
                                <div>Target Nodes: {deployment.targetNodes.join(', ')}</div>
                                <div>Started: {deployment.startTime.toLocaleString()}</div>
                                <div>Checksum: {deployment.checksum}</div>
                              </div>
                            </div>
                          </Card>
                        ))}
                      </div>
                    </div>

                    {/* Deployment History */}
                    <div>
                      <h3 className="text-lg font-medium text-gray-900 mb-4">Deployment History</h3>
                      <div className="space-y-4">
                        {deployments.filter(d => d.status === 'completed' || d.status === 'failed').map(deployment => (
                          <Card key={deployment.id} className="p-4">
                            <div className="flex items-center justify-between">
                              <div>
                                <div className="font-medium text-gray-900">{deployment.name}</div>
                                <div className="text-sm text-gray-600">
                                  {deployment.startTime.toLocaleString()} - {deployment.endTime?.toLocaleString()}
                                </div>
                              </div>
                              <div className="flex items-center space-x-2">
                                <Badge className={getDeploymentStatusColor(deployment.status)}>
                                  {deployment.status}
                                </Badge>
                                <Button variant="ghost" size="sm">
                                  <FileText className="w-4 h-4" />
                                </Button>
                              </div>
                            </div>
                          </Card>
                        ))}
                      </div>
                    </div>
                  </div>
                </div>
              </ScrollArea>
            </TabsContent>

            <TabsContent value="monitoring" className="h-full">
              <ScrollArea className="h-full">
                <div className="p-6">
                  <div className="text-center py-12">
                    <Activity className="w-16 h-16 text-gray-400 mx-auto mb-4" />
                    <h3 className="text-lg font-medium text-gray-900 mb-2">System Monitoring</h3>
                    <p className="text-gray-600">Real-time performance metrics and monitoring dashboards</p>
                  </div>
                </div>
              </ScrollArea>
            </TabsContent>

            <TabsContent value="security" className="h-full">
              <ScrollArea className="h-full">
                <div className="p-6">
                  <div className="text-center py-12">
                    <Shield className="w-16 h-16 text-gray-400 mx-auto mb-4" />
                    <h3 className="text-lg font-medium text-gray-900 mb-2">Security & Access Control</h3>
                    <p className="text-gray-600">Security policies, access management, and audit logs</p>
                  </div>
                </div>
              </ScrollArea>
            </TabsContent>
          </div>
        </Tabs>
      </div>

      {/* Node Detail Dialog */}
      <Dialog open={showNodeDetail} onOpenChange={setShowNodeDetail}>
        <DialogContent className="max-w-4xl max-h-[90vh] overflow-y-auto">
          <DialogHeader>
            <DialogTitle>
              Node Details: {selectedNode?.name}
            </DialogTitle>
          </DialogHeader>

          {selectedNode && (
            <div className="space-y-6">
              <Tabs defaultValue="overview">
                <TabsList>
                  <TabsTrigger value="overview">Overview</TabsTrigger>
                  <TabsTrigger value="services">Services</TabsTrigger>
                  <TabsTrigger value="storage">Storage</TabsTrigger>
                  <TabsTrigger value="network">Network</TabsTrigger>
                  <TabsTrigger value="alerts">Alerts</TabsTrigger>
                </TabsList>

                <TabsContent value="overview" className="space-y-4">
                  <div className="grid grid-cols-2 gap-6">
                    <Card className="p-4">
                      <h3 className="text-lg font-medium text-gray-900 mb-3">System Information</h3>
                      <div className="space-y-2 text-sm">
                        <div className="flex justify-between">
                          <span className="text-gray-600">CPU:</span>
                          <span className="font-medium">{selectedNode.specs.cpu}</span>
                        </div>
                        <div className="flex justify-between">
                          <span className="text-gray-600">Memory:</span>
                          <span className="font-medium">{selectedNode.specs.memory}</span>
                        </div>
                        <div className="flex justify-between">
                          <span className="text-gray-600">Storage:</span>
                          <span className="font-medium">{selectedNode.specs.storage}</span>
                        </div>
                        <div className="flex justify-between">
                          <span className="text-gray-600">Network:</span>
                          <span className="font-medium">{selectedNode.specs.network}</span>
                        </div>
                      </div>
                    </Card>

                    <Card className="p-4">
                      <h3 className="text-lg font-medium text-gray-900 mb-3">Current Status</h3>
                      <div className="space-y-3">
                        <div className="flex items-center justify-between">
                          <span className="text-gray-600">Overall Health:</span>
                          <div className="flex items-center space-x-2">
                            <Progress value={selectedNode.health.overall} className="w-20 h-2" />
                            <span className={getHealthColor(selectedNode.health.overall)}>
                              {selectedNode.health.overall}%
                            </span>
                          </div>
                        </div>
                        <div className="flex items-center justify-between">
                          <span className="text-gray-600">Status:</span>
                          <Badge className={getStatusColor(selectedNode.status)}>
                            {selectedNode.status}
                          </Badge>
                        </div>
                        <div className="flex items-center justify-between">
                          <span className="text-gray-600">Type:</span>
                          <Badge variant="outline">{selectedNode.type}</Badge>
                        </div>
                      </div>
                    </Card>
                  </div>
                </TabsContent>

                <TabsContent value="services" className="space-y-4">
                  <div className="space-y-4">
                    {selectedNode.services.map(service => (
                      <Card key={service.id} className="p-4">
                        <div className="flex items-center justify-between mb-3">
                          <div>
                            <div className="font-medium text-gray-900">{service.name}</div>
                            <div className="text-sm text-gray-600">
                              Port {service.port} • Version {service.version} • Uptime: {service.uptime}
                            </div>
                          </div>
                          <div className="flex items-center space-x-2">
                            <Badge className={getStatusColor(service.status)}>
                              {service.status}
                            </Badge>
                            <div className="flex space-x-1">
                              <TooltipProvider>
                                <Tooltip>
                                  <TooltipTrigger asChild>
                                    <Button 
                                      size="sm" 
                                      variant="outline"
                                      onClick={() => handleServiceAction(selectedNode.id, service.id, 'start')}
                                    >
                                      <Play className="w-3 h-3" />
                                    </Button>
                                  </TooltipTrigger>
                                  <TooltipContent>Start Service</TooltipContent>
                                </Tooltip>
                              </TooltipProvider>
                              
                              <TooltipProvider>
                                <Tooltip>
                                  <TooltipTrigger asChild>
                                    <Button 
                                      size="sm" 
                                      variant="outline"
                                      onClick={() => handleServiceAction(selectedNode.id, service.id, 'stop')}
                                    >
                                      <Pause className="w-3 h-3" />
                                    </Button>
                                  </TooltipTrigger>
                                  <TooltipContent>Stop Service</TooltipContent>
                                </Tooltip>
                              </TooltipProvider>
                              
                              <TooltipProvider>
                                <Tooltip>
                                  <TooltipTrigger asChild>
                                    <Button 
                                      size="sm" 
                                      variant="outline"
                                      onClick={() => handleServiceAction(selectedNode.id, service.id, 'restart')}
                                    >
                                      <RotateCcw className="w-3 h-3" />
                                    </Button>
                                  </TooltipTrigger>
                                  <TooltipContent>Restart Service</TooltipContent>
                                </Tooltip>
                              </TooltipProvider>
                            </div>
                          </div>
                        </div>
                        
                        <div className="grid grid-cols-2 gap-4 text-sm">
                          <div className="flex items-center justify-between">
                            <span className="text-gray-600">CPU Usage:</span>
                            <span className={getHealthColor(100 - service.cpu)}>{service.cpu}%</span>
                          </div>
                          <div className="flex items-center justify-between">
                            <span className="text-gray-600">Memory:</span>
                            <span>{service.memory}MB</span>
                          </div>
                        </div>
                      </Card>
                    ))}
                  </div>
                </TabsContent>

                <TabsContent value="storage" className="space-y-4">
                  <Card className="p-4">
                    <h3 className="text-lg font-medium text-gray-900 mb-3">Storage Overview</h3>
                    <div className="space-y-4">
                      <div className="flex items-center justify-between">
                        <span className="text-gray-600">Total Capacity:</span>
                        <span className="font-medium">{selectedNode.storage.total}GB</span>
                      </div>
                      <div className="flex items-center justify-between">
                        <span className="text-gray-600">Used:</span>
                        <span className="font-medium">{selectedNode.storage.used}GB</span>
                      </div>
                      <div className="flex items-center justify-between">
                        <span className="text-gray-600">Available:</span>
                        <span className="font-medium">{selectedNode.storage.available}GB</span>
                      </div>
                      <div className="flex items-center justify-between">
                        <span className="text-gray-600">Utilization:</span>
                        <div className="flex items-center space-x-2">
                          <Progress value={selectedNode.storage.utilization} className="w-32 h-3" />
                          <span className={
                            selectedNode.storage.utilization > 80 ? 'text-red-600 font-medium' :
                            selectedNode.storage.utilization > 60 ? 'text-yellow-600 font-medium' : 'text-green-600 font-medium'
                          }>
                            {selectedNode.storage.utilization}%
                          </span>
                        </div>
                      </div>
                    </div>
                  </Card>
                </TabsContent>

                <TabsContent value="network" className="space-y-4">
                  <Card className="p-4">
                    <h3 className="text-lg font-medium text-gray-900 mb-3">Network Status</h3>
                    <div className="grid grid-cols-2 gap-6">
                      <div className="space-y-3">
                        <div className="flex items-center justify-between">
                          <span className="text-gray-600">Bandwidth:</span>
                          <span className="font-medium">{selectedNode.network.bandwidth.toFixed(1)} Gbps</span>
                        </div>
                        <div className="flex items-center justify-between">
                          <span className="text-gray-600">Latency:</span>
                          <span className="font-medium">{selectedNode.network.latency}ms</span>
                        </div>
                      </div>
                      <div className="space-y-3">
                        <div className="flex items-center justify-between">
                          <span className="text-gray-600">Packet Loss:</span>
                          <span className={`font-medium ${
                            selectedNode.network.packetLoss > 1 ? 'text-red-600' :
                            selectedNode.network.packetLoss > 0.1 ? 'text-yellow-600' : 'text-green-600'
                          }`}>
                            {selectedNode.network.packetLoss}%
                          </span>
                        </div>
                        <div className="flex items-center justify-between">
                          <span className="text-gray-600">Connections:</span>
                          <span className="font-medium">{selectedNode.network.connections}</span>
                        </div>
                      </div>
                    </div>
                  </Card>
                </TabsContent>

                <TabsContent value="alerts" className="space-y-4">
                  {selectedNode.alerts.length > 0 ? (
                    <div className="space-y-3">
                      {selectedNode.alerts.map(alert => (
                        <Card key={alert.id} className="p-4">
                          <div className="flex items-start justify-between">
                            <div className="flex items-start space-x-3">
                              <div className={`mt-1 ${
                                alert.severity === 'critical' || alert.severity === 'emergency' 
                                  ? 'text-red-500' 
                                  : alert.severity === 'warning' 
                                    ? 'text-yellow-500' 
                                    : 'text-blue-500'
                              }`}>
                                <AlertTriangle className="w-5 h-5" />
                              </div>
                              <div>
                                <div className="font-medium text-gray-900">{alert.message}</div>
                                <div className="text-sm text-gray-600">
                                  {alert.type} • {alert.timestamp.toLocaleString()}
                                </div>
                              </div>
                            </div>
                            <div className="flex items-center space-x-2">
                              <Badge 
                                variant={
                                  alert.severity === 'critical' || alert.severity === 'emergency' 
                                    ? 'destructive' 
                                    : alert.severity === 'warning' 
                                      ? 'warning' 
                                      : 'secondary'
                                }
                              >
                                {alert.severity}
                              </Badge>
                              {!alert.acknowledged && (
                                <Button 
                                  size="sm" 
                                  variant="outline"
                                  onClick={() => handleAcknowledgeAlert(selectedNode.id, alert.id)}
                                >
                                  Acknowledge
                                </Button>
                              )}
                            </div>
                          </div>
                        </Card>
                      ))}
                    </div>
                  ) : (
                    <div className="text-center py-12">
                      <CheckCircle className="w-16 h-16 text-green-500 mx-auto mb-4" />
                      <h3 className="text-lg font-medium text-gray-900 mb-2">No Active Alerts</h3>
                      <p className="text-gray-600">System is operating normally</p>
                    </div>
                  )}
                </TabsContent>
              </Tabs>
            </div>
          )}
        </DialogContent>
      </Dialog>

      {/* Upload Image Dialog */}
      <Dialog open={showImageUpload} onOpenChange={setShowImageUpload}>
        <DialogContent>
          <DialogHeader>
            <DialogTitle>Upload System Image</DialogTitle>
          </DialogHeader>
          <div className="space-y-4">
            <div className="text-center py-8 border-2 border-dashed border-gray-300 rounded-lg">
              <Upload className="w-16 h-16 text-gray-400 mx-auto mb-4" />
              <h3 className="text-lg font-medium text-gray-900 mb-2">Upload Image File</h3>
              <p className="text-gray-600">Drag and drop or click to select image file</p>
            </div>
            <div className="flex justify-end space-x-3">
              <Button variant="outline" onClick={() => setShowImageUpload(false)}>
                Cancel
              </Button>
              <Button>Upload</Button>
            </div>
          </div>
        </DialogContent>
      </Dialog>

      {/* Deploy Dialog */}
      <Dialog open={showDeployDialog} onOpenChange={setShowDeployDialog}>
        <DialogContent>
          <DialogHeader>
            <DialogTitle>Deploy System Update</DialogTitle>
          </DialogHeader>
          <div className="space-y-4">
            <div className="text-center py-8">
              <RefreshCw className="w-16 h-16 text-blue-500 mx-auto mb-4" />
              <h3 className="text-lg font-medium text-gray-900 mb-2">Deploy Update</h3>
              <p className="text-gray-600">Select nodes and image version to deploy</p>
            </div>
            <div className="flex justify-end space-x-3">
              <Button variant="outline" onClick={() => setShowDeployDialog(false)}>
                Cancel
              </Button>
              <Button>Start Deployment</Button>
            </div>
          </div>
        </DialogContent>
      </Dialog>
    </div>
  )
}

export default GarageManagement
