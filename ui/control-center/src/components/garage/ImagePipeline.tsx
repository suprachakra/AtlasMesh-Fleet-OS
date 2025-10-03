import React, { useState, useCallback, useEffect, useMemo } from 'react'
import { 
  Package, Upload, Download, Shield, CheckCircle, XCircle, Clock, AlertTriangle,
  Eye, Settings, RefreshCw, Play, Pause, StopCircle, GitBranch, Hash,
  Server, HardDrive, Zap, Users, Lock, Unlock, FileText, BarChart3,
  Target, Gauge, Activity, Database, Cpu, Memory, Thermometer
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
import { Switch } from '../ui/Switch'

// Types
interface EdgeImage {
  id: string
  name: string
  version: string
  description: string
  buildDate: Date
  size: number // MB
  checksum: string
  signature: string
  signedBy: string
  status: 'building' | 'testing' | 'staging' | 'production' | 'deprecated' | 'failed'
  stability: 'stable' | 'beta' | 'alpha' | 'experimental'
  baseImage: string
  components: ImageComponent[]
  securityScan: SecurityScan
  testResults: TestResult[]
  deploymentStrategy: DeploymentStrategy
  rolloutStatus: RolloutStatus
  metadata: ImageMetadata
}

interface ImageComponent {
  name: string
  version: string
  type: 'os' | 'runtime' | 'application' | 'driver' | 'security'
  critical: boolean
  vulnerabilities: number
  licenses: string[]
}

interface SecurityScan {
  id: string
  scanDate: Date
  scanner: string
  status: 'passed' | 'failed' | 'warning'
  criticalVulns: number
  highVulns: number
  mediumVulns: number
  lowVulns: number
  totalVulns: number
  score: number
  details: SecurityVulnerability[]
}

interface SecurityVulnerability {
  id: string
  severity: 'critical' | 'high' | 'medium' | 'low'
  component: string
  description: string
  cve?: string
  fixAvailable: boolean
  fixVersion?: string
}

interface TestResult {
  id: string
  testSuite: string
  status: 'passed' | 'failed' | 'skipped'
  duration: number
  testDate: Date
  passedTests: number
  failedTests: number
  totalTests: number
  coverage: number
  details: string
}

interface DeploymentStrategy {
  type: 'blue_green' | 'canary' | 'rolling' | 'staged'
  stages: DeploymentStage[]
  rollbackStrategy: 'automatic' | 'manual'
  healthChecks: HealthCheck[]
  approvalRequired: boolean
}

interface DeploymentStage {
  name: string
  percentage: number
  duration: number // minutes
  criteria: string[]
  autoPromote: boolean
}

interface HealthCheck {
  name: string
  endpoint: string
  expectedStatus: number
  timeout: number
  interval: number
  retries: number
}

interface RolloutStatus {
  currentStage: number
  totalStages: number
  stageName: string
  stageProgress: number
  deployedNodes: number
  totalNodes: number
  successRate: number
  errors: RolloutError[]
  startTime: Date
  estimatedCompletion?: Date
}

interface RolloutError {
  nodeId: string
  error: string
  timestamp: Date
  resolved: boolean
}

interface ImageMetadata {
  buildPipeline: string
  buildCommit: string
  buildBranch: string
  buildNumber: number
  buildTrigger: string
  approvedBy?: string
  approvalDate?: Date
  releaseNotes: string
  tags: string[]
}

interface EdgeNode {
  id: string
  name: string
  location: string
  status: 'online' | 'offline' | 'updating' | 'error'
  currentImage: string
  targetImage?: string
  updateProgress?: number
  lastUpdate: Date
  hardware: {
    cpu: string
    memory: number
    storage: number
    architecture: string
  }
  health: {
    cpuUsage: number
    memoryUsage: number
    diskUsage: number
    temperature: number
  }
}

interface ImagePipelineProps {
  nodeId?: string
  nodeName?: string
  onImageDeployed?: (imageId: string, nodeId: string) => void
  className?: string
}

const ImagePipeline: React.FC<ImagePipelineProps> = ({
  nodeId,
  nodeName,
  onImageDeployed,
  className = ''
}) => {
  // State
  const [images, setImages] = useState<EdgeImage[]>([])
  const [nodes, setNodes] = useState<EdgeNode[]>([])
  const [showBuildDialog, setShowBuildDialog] = useState(false)
  const [showDeployDialog, setShowDeployDialog] = useState(false)
  const [showImageDetails, setShowImageDetails] = useState(false)
  const [selectedImage, setSelectedImage] = useState<EdgeImage | null>(null)
  const [activeTab, setActiveTab] = useState('overview')
  const [isBuilding, setIsBuilding] = useState(false)
  const [buildConfig, setBuildConfig] = useState({
    name: '',
    version: '',
    description: '',
    baseImage: 'ubuntu-22.04-lts',
    components: [] as string[],
    stability: 'beta' as const
  })

  // Mock data initialization
  const initializeMockData = useCallback(() => {
    const mockImages: EdgeImage[] = [
      {
        id: 'img-001',
        name: 'AtlasMesh Edge OS',
        version: 'v2.1.0',
        description: 'Production-ready edge operating system with ROS2 and ML stack',
        buildDate: new Date('2024-11-20'),
        size: 2048,
        checksum: 'sha256:a1b2c3d4e5f6789...',
        signature: 'rsa-pss:9f8e7d6c5b4a321...',
        signedBy: 'AtlasMesh CI/CD Pipeline',
        status: 'production',
        stability: 'stable',
        baseImage: 'ubuntu-22.04-lts',
        components: [
          { name: 'Ubuntu LTS', version: '22.04.3', type: 'os', critical: true, vulnerabilities: 0, licenses: ['GPL-2.0'] },
          { name: 'ROS2 Humble', version: '2.7.1', type: 'runtime', critical: true, vulnerabilities: 1, licenses: ['Apache-2.0'] },
          { name: 'AtlasMesh Core', version: '2.1.0', type: 'application', critical: true, vulnerabilities: 0, licenses: ['Proprietary'] },
          { name: 'NVIDIA Drivers', version: '535.129', type: 'driver', critical: true, vulnerabilities: 0, licenses: ['Proprietary'] },
          { name: 'Security Agent', version: '1.8.2', type: 'security', critical: true, vulnerabilities: 0, licenses: ['Proprietary'] }
        ],
        securityScan: {
          id: 'scan-001',
          scanDate: new Date('2024-11-20'),
          scanner: 'Trivy v0.47.0',
          status: 'passed',
          criticalVulns: 0,
          highVulns: 1,
          mediumVulns: 3,
          lowVulns: 8,
          totalVulns: 12,
          score: 92,
          details: [
            {
              id: 'vuln-001',
              severity: 'high',
              component: 'ROS2 Humble',
              description: 'Potential buffer overflow in message serialization',
              cve: 'CVE-2024-12345',
              fixAvailable: true,
              fixVersion: '2.7.2'
            }
          ]
        },
        testResults: [
          {
            id: 'test-001',
            testSuite: 'System Integration Tests',
            status: 'passed',
            duration: 1800,
            testDate: new Date('2024-11-20'),
            passedTests: 147,
            failedTests: 0,
            totalTests: 147,
            coverage: 94.5,
            details: 'All critical system functions validated successfully'
          },
          {
            id: 'test-002',
            testSuite: 'Performance Tests',
            status: 'passed',
            duration: 900,
            testDate: new Date('2024-11-20'),
            passedTests: 23,
            failedTests: 0,
            totalTests: 23,
            coverage: 89.2,
            details: 'Boot time: 45s, Memory footprint: 1.2GB, CPU overhead: 8%'
          }
        ],
        deploymentStrategy: {
          type: 'canary',
          stages: [
            { name: 'Canary', percentage: 10, duration: 60, criteria: ['health_check', 'performance'], autoPromote: false },
            { name: 'Staging', percentage: 50, duration: 120, criteria: ['health_check', 'performance', 'user_acceptance'], autoPromote: false },
            { name: 'Production', percentage: 100, duration: 180, criteria: ['health_check'], autoPromote: true }
          ],
          rollbackStrategy: 'automatic',
          healthChecks: [
            { name: 'System Health', endpoint: '/health', expectedStatus: 200, timeout: 30, interval: 60, retries: 3 },
            { name: 'ROS2 Status', endpoint: '/ros2/status', expectedStatus: 200, timeout: 15, interval: 30, retries: 2 }
          ],
          approvalRequired: true
        },
        rolloutStatus: {
          currentStage: 0,
          totalStages: 3,
          stageName: 'Production',
          stageProgress: 100,
          deployedNodes: 15,
          totalNodes: 15,
          successRate: 100,
          errors: [],
          startTime: new Date('2024-11-20T10:00:00Z')
        },
        metadata: {
          buildPipeline: 'edge-os-pipeline',
          buildCommit: 'abc123def456',
          buildBranch: 'release/v2.1.0',
          buildNumber: 1247,
          buildTrigger: 'scheduled',
          approvedBy: 'Fleet Manager',
          approvalDate: new Date('2024-11-20T09:00:00Z'),
          releaseNotes: 'Stable production release with enhanced security and performance improvements',
          tags: ['production', 'stable', 'ros2', 'security-hardened']
        }
      },
      {
        id: 'img-002',
        name: 'AtlasMesh Edge OS Beta',
        version: 'v2.2.0-beta.1',
        description: 'Beta release with experimental ML acceleration features',
        buildDate: new Date('2024-11-25'),
        size: 2156,
        checksum: 'sha256:b2c3d4e5f6a7890...',
        signature: 'rsa-pss:8e7d6c5b4a32109...',
        signedBy: 'AtlasMesh CI/CD Pipeline',
        status: 'staging',
        stability: 'beta',
        baseImage: 'ubuntu-22.04-lts',
        components: [
          { name: 'Ubuntu LTS', version: '22.04.3', type: 'os', critical: true, vulnerabilities: 0, licenses: ['GPL-2.0'] },
          { name: 'ROS2 Humble', version: '2.8.0-beta', type: 'runtime', critical: true, vulnerabilities: 2, licenses: ['Apache-2.0'] },
          { name: 'AtlasMesh Core', version: '2.2.0-beta', type: 'application', critical: true, vulnerabilities: 0, licenses: ['Proprietary'] },
          { name: 'TensorRT', version: '8.6.1', type: 'runtime', critical: false, vulnerabilities: 0, licenses: ['Proprietary'] },
          { name: 'CUDA Runtime', version: '12.0', type: 'runtime', critical: false, vulnerabilities: 1, licenses: ['Proprietary'] }
        ],
        securityScan: {
          id: 'scan-002',
          scanDate: new Date('2024-11-25'),
          scanner: 'Trivy v0.47.0',
          status: 'warning',
          criticalVulns: 0,
          highVulns: 3,
          mediumVulns: 7,
          lowVulns: 15,
          totalVulns: 25,
          score: 78,
          details: [
            {
              id: 'vuln-002',
              severity: 'high',
              component: 'ROS2 Humble Beta',
              description: 'Unvalidated input in new message types',
              cve: 'CVE-2024-54321',
              fixAvailable: false
            }
          ]
        },
        testResults: [
          {
            id: 'test-003',
            testSuite: 'System Integration Tests',
            status: 'passed',
            duration: 2100,
            testDate: new Date('2024-11-25'),
            passedTests: 142,
            failedTests: 5,
            totalTests: 147,
            coverage: 91.2,
            details: '5 non-critical tests failed in experimental features'
          }
        ],
        deploymentStrategy: {
          type: 'staged',
          stages: [
            { name: 'Development', percentage: 20, duration: 30, criteria: ['basic_health'], autoPromote: true },
            { name: 'Testing', percentage: 100, duration: 240, criteria: ['full_test_suite'], autoPromote: false }
          ],
          rollbackStrategy: 'manual',
          healthChecks: [
            { name: 'System Health', endpoint: '/health', expectedStatus: 200, timeout: 30, interval: 60, retries: 3 }
          ],
          approvalRequired: true
        },
        rolloutStatus: {
          currentStage: 1,
          totalStages: 2,
          stageName: 'Testing',
          stageProgress: 65,
          deployedNodes: 3,
          totalNodes: 5,
          successRate: 95,
          errors: [
            { nodeId: 'node-004', error: 'ML acceleration test timeout', timestamp: new Date(), resolved: false }
          ],
          startTime: new Date(Date.now() - 4 * 60 * 60 * 1000),
          estimatedCompletion: new Date(Date.now() + 2 * 60 * 60 * 1000)
        },
        metadata: {
          buildPipeline: 'edge-os-beta-pipeline',
          buildCommit: 'def456ghi789',
          buildBranch: 'develop',
          buildNumber: 1289,
          buildTrigger: 'pull_request',
          releaseNotes: 'Beta release with experimental ML acceleration and enhanced ROS2 features',
          tags: ['beta', 'experimental', 'ml-acceleration', 'ros2-beta']
        }
      }
    ]

    const mockNodes: EdgeNode[] = [
      {
        id: 'node-001',
        name: 'Depot-Edge-01',
        location: 'Main Depot - Bay 1',
        status: 'online',
        currentImage: 'v2.1.0',
        lastUpdate: new Date('2024-11-20'),
        hardware: {
          cpu: 'Intel i7-12700H',
          memory: 32,
          storage: 1000,
          architecture: 'x86_64'
        },
        health: {
          cpuUsage: 25,
          memoryUsage: 45,
          diskUsage: 60,
          temperature: 42
        }
      },
      {
        id: 'node-002',
        name: 'Depot-Edge-02',
        location: 'Main Depot - Bay 2',
        status: 'updating',
        currentImage: 'v2.0.8',
        targetImage: 'v2.1.0',
        updateProgress: 75,
        lastUpdate: new Date('2024-11-15'),
        hardware: {
          cpu: 'Intel i7-12700H',
          memory: 32,
          storage: 1000,
          architecture: 'x86_64'
        },
        health: {
          cpuUsage: 15,
          memoryUsage: 35,
          diskUsage: 55,
          temperature: 38
        }
      },
      {
        id: 'node-003',
        name: 'Depot-Edge-03',
        location: 'Service Center A',
        status: 'online',
        currentImage: 'v2.2.0-beta.1',
        lastUpdate: new Date('2024-11-25'),
        hardware: {
          cpu: 'NVIDIA Jetson AGX Orin',
          memory: 64,
          storage: 1000,
          architecture: 'aarch64'
        },
        health: {
          cpuUsage: 35,
          memoryUsage: 55,
          diskUsage: 40,
          temperature: 48
        }
      }
    ]

    setImages(mockImages)
    setNodes(mockNodes)
  }, [])

  useEffect(() => {
    initializeMockData()
  }, [initializeMockData])

  // Real-time updates
  useEffect(() => {
    const interval = setInterval(() => {
      // Update rollout progress
      setImages(prev => prev.map(img => {
        if (img.rolloutStatus.stageProgress < 100 && img.status !== 'production') {
          const newProgress = Math.min(100, img.rolloutStatus.stageProgress + Math.random() * 5)
          return {
            ...img,
            rolloutStatus: {
              ...img.rolloutStatus,
              stageProgress: newProgress,
              deployedNodes: Math.round((newProgress / 100) * img.rolloutStatus.totalNodes)
            }
          }
        }
        return img
      }))

      // Update node health
      setNodes(prev => prev.map(node => ({
        ...node,
        health: {
          cpuUsage: Math.max(10, Math.min(90, node.health.cpuUsage + (Math.random() - 0.5) * 10)),
          memoryUsage: Math.max(20, Math.min(95, node.health.memoryUsage + (Math.random() - 0.5) * 8)),
          diskUsage: Math.max(30, Math.min(95, node.health.diskUsage + (Math.random() - 0.5) * 2)),
          temperature: Math.max(25, Math.min(85, node.health.temperature + (Math.random() - 0.5) * 5))
        },
        updateProgress: node.updateProgress ? Math.min(100, node.updateProgress + Math.random() * 3) : undefined
      })))
    }, 3000)

    return () => clearInterval(interval)
  }, [])

  // Handlers
  const handleBuildImage = useCallback(async () => {
    setIsBuilding(true)
    
    const newImage: EdgeImage = {
      id: `img-${Date.now()}`,
      name: buildConfig.name,
      version: buildConfig.version,
      description: buildConfig.description,
      buildDate: new Date(),
      size: 1800 + Math.random() * 500,
      checksum: `sha256:${Math.random().toString(36).substring(2, 15)}...`,
      signature: `rsa-pss:${Math.random().toString(36).substring(2, 15)}...`,
      signedBy: 'AtlasMesh CI/CD Pipeline',
      status: 'building',
      stability: buildConfig.stability,
      baseImage: buildConfig.baseImage,
      components: [
        { name: 'Ubuntu LTS', version: '22.04.3', type: 'os', critical: true, vulnerabilities: 0, licenses: ['GPL-2.0'] },
        { name: 'AtlasMesh Core', version: buildConfig.version, type: 'application', critical: true, vulnerabilities: 0, licenses: ['Proprietary'] }
      ],
      securityScan: {
        id: `scan-${Date.now()}`,
        scanDate: new Date(),
        scanner: 'Trivy v0.47.0',
        status: 'passed',
        criticalVulns: 0,
        highVulns: 0,
        mediumVulns: 2,
        lowVulns: 5,
        totalVulns: 7,
        score: 95,
        details: []
      },
      testResults: [],
      deploymentStrategy: {
        type: 'canary',
        stages: [
          { name: 'Canary', percentage: 10, duration: 60, criteria: ['health_check'], autoPromote: false },
          { name: 'Production', percentage: 100, duration: 120, criteria: ['health_check'], autoPromote: true }
        ],
        rollbackStrategy: 'automatic',
        healthChecks: [
          { name: 'System Health', endpoint: '/health', expectedStatus: 200, timeout: 30, interval: 60, retries: 3 }
        ],
        approvalRequired: buildConfig.stability === 'stable'
      },
      rolloutStatus: {
        currentStage: 0,
        totalStages: 2,
        stageName: 'Building',
        stageProgress: 0,
        deployedNodes: 0,
        totalNodes: nodes.length,
        successRate: 0,
        errors: [],
        startTime: new Date()
      },
      metadata: {
        buildPipeline: 'edge-os-pipeline',
        buildCommit: Math.random().toString(36).substring(2, 15),
        buildBranch: 'feature/new-build',
        buildNumber: Math.floor(Math.random() * 1000) + 1300,
        buildTrigger: 'manual',
        releaseNotes: buildConfig.description,
        tags: [buildConfig.stability, 'custom-build']
      }
    }

    setImages(prev => [newImage, ...prev])
    setShowBuildDialog(false)
    setBuildConfig({
      name: '',
      version: '',
      description: '',
      baseImage: 'ubuntu-22.04-lts',
      components: [],
      stability: 'beta'
    })

    // Simulate build process
    setTimeout(() => {
      setImages(prev => prev.map(img => 
        img.id === newImage.id 
          ? { ...img, status: 'testing' as const, rolloutStatus: { ...img.rolloutStatus, stageName: 'Testing', stageProgress: 50 } }
          : img
      ))
    }, 5000)

    setTimeout(() => {
      setImages(prev => prev.map(img => 
        img.id === newImage.id 
          ? { ...img, status: 'staging' as const, rolloutStatus: { ...img.rolloutStatus, stageName: 'Staging', stageProgress: 100 } }
          : img
      ))
    }, 10000)

    setIsBuilding(false)
  }, [buildConfig, nodes.length])

  const handleDeployImage = useCallback((image: EdgeImage, targetNodes: string[]) => {
    // Start deployment simulation
    setImages(prev => prev.map(img => 
      img.id === image.id 
        ? { 
            ...img, 
            rolloutStatus: { 
              ...img.rolloutStatus, 
              stageName: 'Deploying',
              stageProgress: 0,
              startTime: new Date()
            }
          }
        : img
    ))

    setNodes(prev => prev.map(node => 
      targetNodes.includes(node.id)
        ? { ...node, status: 'updating' as const, targetImage: image.version, updateProgress: 0 }
        : node
    ))

    onImageDeployed?.(image.id, targetNodes[0])
  }, [onImageDeployed])

  const handleViewImageDetails = useCallback((image: EdgeImage) => {
    setSelectedImage(image)
    setShowImageDetails(true)
  }, [])

  // Computed values
  const stats = useMemo(() => {
    const filteredImages = nodeId ? 
      images.filter(img => nodes.some(node => node.id === nodeId && (node.currentImage === img.version || node.targetImage === img.version))) :
      images

    const total = images.length
    const production = images.filter(img => img.status === 'production').length
    const staging = images.filter(img => img.status === 'staging').length
    const building = images.filter(img => img.status === 'building').length
    const failed = images.filter(img => img.status === 'failed').length

    const totalNodes = nodeId ? 1 : nodes.length
    const onlineNodes = nodes.filter(node => !nodeId || node.id === nodeId).filter(node => node.status === 'online').length
    const updatingNodes = nodes.filter(node => !nodeId || node.id === nodeId).filter(node => node.status === 'updating').length

    const avgSecurityScore = images.length > 0 ? 
      images.reduce((sum, img) => sum + img.securityScan.score, 0) / images.length : 0

    return {
      total,
      production,
      staging,
      building,
      failed,
      totalNodes,
      onlineNodes,
      updatingNodes,
      avgSecurityScore
    }
  }, [images, nodes, nodeId])

  const getStatusColor = (status: string) => {
    switch (status) {
      case 'production': return 'bg-green-100 text-green-800'
      case 'staging': return 'bg-blue-100 text-blue-800'
      case 'testing': return 'bg-yellow-100 text-yellow-800'
      case 'building': return 'bg-purple-100 text-purple-800'
      case 'failed': return 'bg-red-100 text-red-800'
      case 'deprecated': return 'bg-gray-100 text-gray-800'
      default: return 'bg-gray-100 text-gray-800'
    }
  }

  const getStabilityColor = (stability: string) => {
    switch (stability) {
      case 'stable': return 'bg-green-100 text-green-800'
      case 'beta': return 'bg-blue-100 text-blue-800'
      case 'alpha': return 'bg-yellow-100 text-yellow-800'
      case 'experimental': return 'bg-red-100 text-red-800'
      default: return 'bg-gray-100 text-gray-800'
    }
  }

  const getSecurityScoreColor = (score: number) => {
    if (score >= 90) return 'text-green-600'
    if (score >= 70) return 'text-yellow-600'
    return 'text-red-600'
  }

  return (
    <div className={`space-y-6 ${className}`}>
      {/* Header */}
      <div className="flex items-center justify-between">
        <div className="flex items-center space-x-3">
          <Package className="w-6 h-6 text-blue-600" />
          <h2 className="text-2xl font-semibold text-gray-900">
            Garage PC Image Pipeline
            {nodeId && <span className="text-lg font-normal text-gray-600 ml-2">- {nodeName}</span>}
          </h2>
        </div>
        <div className="flex items-center space-x-2">
          <Button variant="outline" onClick={initializeMockData}>
            <RefreshCw className="w-4 h-4 mr-2" />
            Refresh
          </Button>
          <Button onClick={() => setShowBuildDialog(true)}>
            <Upload className="w-4 h-4 mr-2" />
            Build Image
          </Button>
        </div>
      </div>

      {/* Statistics */}
      <div className="grid grid-cols-2 md:grid-cols-4 lg:grid-cols-8 gap-4">
        <Card className="p-4">
          <div className="text-center">
            <div className="text-2xl font-bold text-gray-900">{stats.total}</div>
            <div className="text-sm text-gray-600">Total Images</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className="text-2xl font-bold text-green-600">{stats.production}</div>
            <div className="text-sm text-gray-600">Production</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className="text-2xl font-bold text-blue-600">{stats.staging}</div>
            <div className="text-sm text-gray-600">Staging</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className="text-2xl font-bold text-purple-600">{stats.building}</div>
            <div className="text-sm text-gray-600">Building</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className={`text-2xl font-bold ${stats.failed > 0 ? 'text-red-600' : 'text-green-600'}`}>
              {stats.failed}
            </div>
            <div className="text-sm text-gray-600">Failed</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className="text-2xl font-bold text-green-600">{stats.onlineNodes}</div>
            <div className="text-sm text-gray-600">Nodes Online</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className="text-2xl font-bold text-blue-600">{stats.updatingNodes}</div>
            <div className="text-sm text-gray-600">Updating</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className={`text-2xl font-bold ${getSecurityScoreColor(stats.avgSecurityScore)}`}>
              {Math.round(stats.avgSecurityScore)}
            </div>
            <div className="text-sm text-gray-600">Avg Security</div>
          </div>
        </Card>
      </div>

      {/* Main Content */}
      <Tabs value={activeTab} onValueChange={setActiveTab}>
        <TabsList>
          <TabsTrigger value="overview">Images</TabsTrigger>
          <TabsTrigger value="nodes">Edge Nodes</TabsTrigger>
          <TabsTrigger value="pipeline">Build Pipeline</TabsTrigger>
          <TabsTrigger value="security">Security</TabsTrigger>
        </TabsList>

        <TabsContent value="overview" className="space-y-4">
          <div className="space-y-6">
            {images
              .filter(img => !nodeId || nodes.some(node => node.id === nodeId && (node.currentImage === img.version || node.targetImage === img.version)))
              .map(image => (
              <Card key={image.id} className="p-6">
                <div className="flex items-start justify-between mb-4">
                  <div className="flex items-center space-x-3">
                    <div className="p-2 bg-blue-100 rounded">
                      <Package className="w-6 h-6 text-blue-600" />
                    </div>
                    <div>
                      <h3 className="text-lg font-medium text-gray-900">{image.name}</h3>
                      <p className="text-sm text-gray-600">{image.description}</p>
                    </div>
                  </div>
                  <div className="flex items-center space-x-2">
                    <Badge className={getStatusColor(image.status)}>
                      {image.status}
                    </Badge>
                    <Badge className={getStabilityColor(image.stability)}>
                      {image.stability}
                    </Badge>
                    {image.securityScan.status === 'passed' && (
                      <Badge className="bg-green-100 text-green-800">
                        <Shield className="w-3 h-3 mr-1" />
                        Secure
                      </Badge>
                    )}
                  </div>
                </div>

                <div className="grid grid-cols-2 md:grid-cols-4 gap-4 mb-4">
                  <div>
                    <div className="text-sm text-gray-600">Version</div>
                    <div className="font-medium text-gray-900">{image.version}</div>
                  </div>
                  <div>
                    <div className="text-sm text-gray-600">Size</div>
                    <div className="font-medium text-gray-900">{Math.round(image.size)}MB</div>
                  </div>
                  <div>
                    <div className="text-sm text-gray-600">Build Date</div>
                    <div className="font-medium text-gray-900">{image.buildDate.toLocaleDateString()}</div>
                  </div>
                  <div>
                    <div className="text-sm text-gray-600">Security Score</div>
                    <div className={`font-medium ${getSecurityScoreColor(image.securityScan.score)}`}>
                      {image.securityScan.score}/100
                    </div>
                  </div>
                </div>

                {/* Rollout Status */}
                {image.rolloutStatus.stageProgress < 100 && (
                  <div className="mb-4 p-3 bg-blue-50 rounded border border-blue-200">
                    <div className="flex items-center justify-between mb-2">
                      <div className="font-medium text-blue-900">
                        {image.rolloutStatus.stageName} - Stage {image.rolloutStatus.currentStage + 1}/{image.rolloutStatus.totalStages}
                      </div>
                      <div className="text-sm text-blue-700">
                        {image.rolloutStatus.deployedNodes}/{image.rolloutStatus.totalNodes} nodes
                      </div>
                    </div>
                    <Progress value={image.rolloutStatus.stageProgress} className="mb-2" />
                    <div className="text-sm text-blue-700">
                      Success Rate: {image.rolloutStatus.successRate}%
                      {image.rolloutStatus.estimatedCompletion && (
                        <span> • ETA: {image.rolloutStatus.estimatedCompletion.toLocaleTimeString()}</span>
                      )}
                    </div>
                  </div>
                )}

                {/* Components Summary */}
                <div className="mb-4">
                  <div className="text-sm font-medium text-gray-900 mb-2">Components</div>
                  <div className="flex flex-wrap gap-2">
                    {image.components.slice(0, 5).map(component => (
                      <Badge key={component.name} variant="outline" className={
                        component.critical ? 'border-red-200 text-red-800' : 'border-gray-200'
                      }>
                        {component.name} {component.version}
                        {component.vulnerabilities > 0 && (
                          <span className="ml-1 text-red-600">({component.vulnerabilities})</span>
                        )}
                      </Badge>
                    ))}
                    {image.components.length > 5 && (
                      <Badge variant="outline">+{image.components.length - 5} more</Badge>
                    )}
                  </div>
                </div>

                {/* Security & Test Status */}
                <div className="grid grid-cols-2 gap-4 mb-4">
                  <div className="p-3 bg-gray-50 rounded">
                    <div className="flex items-center justify-between mb-1">
                      <span className="text-sm font-medium text-gray-900">Security Scan</span>
                      <Badge className={
                        image.securityScan.status === 'passed' ? 'bg-green-100 text-green-800' :
                        image.securityScan.status === 'warning' ? 'bg-yellow-100 text-yellow-800' :
                        'bg-red-100 text-red-800'
                      }>
                        {image.securityScan.status}
                      </Badge>
                    </div>
                    <div className="text-xs text-gray-600">
                      {image.securityScan.totalVulns} vulnerabilities found
                      {image.securityScan.criticalVulns > 0 && (
                        <span className="text-red-600"> • {image.securityScan.criticalVulns} critical</span>
                      )}
                    </div>
                  </div>

                  <div className="p-3 bg-gray-50 rounded">
                    <div className="flex items-center justify-between mb-1">
                      <span className="text-sm font-medium text-gray-900">Test Results</span>
                      <Badge className={
                        image.testResults.every(t => t.status === 'passed') ? 'bg-green-100 text-green-800' :
                        image.testResults.some(t => t.status === 'failed') ? 'bg-red-100 text-red-800' :
                        'bg-yellow-100 text-yellow-800'
                      }>
                        {image.testResults.length > 0 ? 
                          image.testResults.every(t => t.status === 'passed') ? 'passed' : 'mixed' :
                          'pending'
                        }
                      </Badge>
                    </div>
                    <div className="text-xs text-gray-600">
                      {image.testResults.length} test suites
                      {image.testResults.length > 0 && (
                        <span> • {Math.round(image.testResults.reduce((sum, t) => sum + t.coverage, 0) / image.testResults.length)}% coverage</span>
                      )}
                    </div>
                  </div>
                </div>

                {/* Actions */}
                <div className="flex items-center justify-between pt-4 border-t">
                  <div className="flex items-center space-x-4 text-sm text-gray-600">
                    <div className="flex items-center space-x-1">
                      <Hash className="w-4 h-4" />
                      <span>Build #{image.metadata.buildNumber}</span>
                    </div>
                    <div className="flex items-center space-x-1">
                      <GitBranch className="w-4 h-4" />
                      <span>{image.metadata.buildBranch}</span>
                    </div>
                    {image.metadata.approvedBy && (
                      <div className="flex items-center space-x-1">
                        <CheckCircle className="w-4 h-4 text-green-600" />
                        <span>Approved by {image.metadata.approvedBy}</span>
                      </div>
                    )}
                  </div>
                  <div className="flex items-center space-x-2">
                    <Button
                      variant="outline"
                      size="sm"
                      onClick={() => handleViewImageDetails(image)}
                    >
                      <Eye className="w-4 h-4 mr-1" />
                      Details
                    </Button>
                    {image.status === 'production' || image.status === 'staging' ? (
                      <Button
                        size="sm"
                        onClick={() => setShowDeployDialog(true)}
                      >
                        <Download className="w-4 h-4 mr-1" />
                        Deploy
                      </Button>
                    ) : (
                      <Button size="sm" disabled>
                        <Clock className="w-4 h-4 mr-1" />
                        {image.status === 'building' ? 'Building...' : 'Testing...'}
                      </Button>
                    )}
                  </div>
                </div>
              </Card>
            ))}
          </div>
        </TabsContent>

        <TabsContent value="nodes" className="space-y-4">
          <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-6">
            {nodes
              .filter(node => !nodeId || node.id === nodeId)
              .map(node => (
              <Card key={node.id} className="p-6">
                <div className="flex items-start justify-between mb-4">
                  <div>
                    <h3 className="text-lg font-medium text-gray-900">{node.name}</h3>
                    <p className="text-sm text-gray-600">{node.location}</p>
                  </div>
                  <Badge className={
                    node.status === 'online' ? 'bg-green-100 text-green-800' :
                    node.status === 'updating' ? 'bg-blue-100 text-blue-800' :
                    node.status === 'error' ? 'bg-red-100 text-red-800' :
                    'bg-gray-100 text-gray-800'
                  }>
                    {node.status}
                  </Badge>
                </div>

                <div className="space-y-3 mb-4">
                  <div>
                    <div className="text-sm text-gray-600">Current Image</div>
                    <div className="font-medium text-gray-900">{node.currentImage}</div>
                    {node.targetImage && node.targetImage !== node.currentImage && (
                      <div className="text-sm text-blue-600">→ {node.targetImage}</div>
                    )}
                  </div>

                  {node.updateProgress !== undefined && (
                    <div>
                      <div className="flex items-center justify-between text-sm mb-1">
                        <span className="text-gray-600">Update Progress</span>
                        <span className="font-medium">{Math.round(node.updateProgress)}%</span>
                      </div>
                      <Progress value={node.updateProgress} className="h-2" />
                    </div>
                  )}

                  <div>
                    <div className="text-sm text-gray-600 mb-2">Hardware</div>
                    <div className="text-xs text-gray-500 space-y-0.5">
                      <div>CPU: {node.hardware.cpu}</div>
                      <div>Memory: {node.hardware.memory}GB</div>
                      <div>Storage: {node.hardware.storage}GB</div>
                      <div>Arch: {node.hardware.architecture}</div>
                    </div>
                  </div>
                </div>

                <div className="space-y-3">
                  <div className="text-sm font-medium text-gray-900">Health Metrics</div>
                  
                  <div className="space-y-2">
                    <div className="flex items-center justify-between">
                      <div className="flex items-center space-x-2">
                        <Cpu className="w-4 h-4 text-blue-600" />
                        <span className="text-sm text-gray-600">CPU</span>
                      </div>
                      <div className="flex items-center space-x-2">
                        <div className="w-16">
                          <Progress value={node.health.cpuUsage} className="h-1" />
                        </div>
                        <span className="text-sm font-medium">{Math.round(node.health.cpuUsage)}%</span>
                      </div>
                    </div>

                    <div className="flex items-center justify-between">
                      <div className="flex items-center space-x-2">
                        <Memory className="w-4 h-4 text-green-600" />
                        <span className="text-sm text-gray-600">Memory</span>
                      </div>
                      <div className="flex items-center space-x-2">
                        <div className="w-16">
                          <Progress value={node.health.memoryUsage} className="h-1" />
                        </div>
                        <span className="text-sm font-medium">{Math.round(node.health.memoryUsage)}%</span>
                      </div>
                    </div>

                    <div className="flex items-center justify-between">
                      <div className="flex items-center space-x-2">
                        <HardDrive className="w-4 h-4 text-purple-600" />
                        <span className="text-sm text-gray-600">Disk</span>
                      </div>
                      <div className="flex items-center space-x-2">
                        <div className="w-16">
                          <Progress value={node.health.diskUsage} className="h-1" />
                        </div>
                        <span className="text-sm font-medium">{Math.round(node.health.diskUsage)}%</span>
                      </div>
                    </div>

                    <div className="flex items-center justify-between">
                      <div className="flex items-center space-x-2">
                        <Thermometer className="w-4 h-4 text-orange-600" />
                        <span className="text-sm text-gray-600">Temp</span>
                      </div>
                      <div className="flex items-center space-x-2">
                        <span className={`text-sm font-medium ${
                          node.health.temperature > 70 ? 'text-red-600' :
                          node.health.temperature > 60 ? 'text-yellow-600' :
                          'text-green-600'
                        }`}>
                          {Math.round(node.health.temperature)}°C
                        </span>
                      </div>
                    </div>
                  </div>
                </div>

                <div className="pt-4 border-t mt-4">
                  <div className="text-xs text-gray-500">
                    Last updated: {node.lastUpdate.toLocaleString()}
                  </div>
                </div>
              </Card>
            ))}
          </div>
        </TabsContent>

        <TabsContent value="pipeline" className="space-y-4">
          <Card className="p-6">
            <h3 className="text-lg font-medium text-gray-900 mb-4">Build Pipeline Status</h3>
            <div className="space-y-4">
              <div className="flex items-center justify-between p-3 bg-green-50 rounded border border-green-200">
                <div className="flex items-center space-x-3">
                  <CheckCircle className="w-5 h-5 text-green-600" />
                  <div>
                    <div className="font-medium text-green-900">Production Pipeline</div>
                    <div className="text-sm text-green-700">Last successful build: 2 hours ago</div>
                  </div>
                </div>
                <Badge className="bg-green-100 text-green-800">Active</Badge>
              </div>

              <div className="flex items-center justify-between p-3 bg-blue-50 rounded border border-blue-200">
                <div className="flex items-center space-x-3">
                  <Activity className="w-5 h-5 text-blue-600" />
                  <div>
                    <div className="font-medium text-blue-900">Beta Pipeline</div>
                    <div className="text-sm text-blue-700">Currently building v2.2.0-beta.2</div>
                  </div>
                </div>
                <Badge className="bg-blue-100 text-blue-800">Building</Badge>
              </div>

              <div className="flex items-center justify-between p-3 bg-gray-50 rounded border border-gray-200">
                <div className="flex items-center space-x-3">
                  <Clock className="w-5 h-5 text-gray-600" />
                  <div>
                    <div className="font-medium text-gray-900">Development Pipeline</div>
                    <div className="text-sm text-gray-700">Waiting for commits</div>
                  </div>
                </div>
                <Badge className="bg-gray-100 text-gray-800">Idle</Badge>
              </div>
            </div>
          </Card>

          <Card className="p-6">
            <h3 className="text-lg font-medium text-gray-900 mb-4">Pipeline Configuration</h3>
            <div className="space-y-4">
              <div className="grid grid-cols-2 gap-4">
                <div>
                  <div className="text-sm font-medium text-gray-900 mb-2">Build Triggers</div>
                  <div className="space-y-1 text-sm text-gray-600">
                    <div className="flex items-center space-x-2">
                      <input type="checkbox" checked readOnly />
                      <span>Git push to main branch</span>
                    </div>
                    <div className="flex items-center space-x-2">
                      <input type="checkbox" checked readOnly />
                      <span>Scheduled nightly builds</span>
                    </div>
                    <div className="flex items-center space-x-2">
                      <input type="checkbox" readOnly />
                      <span>Manual trigger</span>
                    </div>
                  </div>
                </div>

                <div>
                  <div className="text-sm font-medium text-gray-900 mb-2">Quality Gates</div>
                  <div className="space-y-1 text-sm text-gray-600">
                    <div className="flex items-center space-x-2">
                      <CheckCircle className="w-4 h-4 text-green-600" />
                      <span>Security scan (required)</span>
                    </div>
                    <div className="flex items-center space-x-2">
                      <CheckCircle className="w-4 h-4 text-green-600" />
                      <span>Unit tests (required)</span>
                    </div>
                    <div className="flex items-center space-x-2">
                      <CheckCircle className="w-4 h-4 text-green-600" />
                      <span>Integration tests (required)</span>
                    </div>
                    <div className="flex items-center space-x-2">
                      <Clock className="w-4 h-4 text-yellow-600" />
                      <span>Performance tests (optional)</span>
                    </div>
                  </div>
                </div>
              </div>
            </div>
          </Card>
        </TabsContent>

        <TabsContent value="security" className="space-y-4">
          <div className="grid grid-cols-1 md:grid-cols-2 gap-6">
            {images.map(image => (
              <Card key={image.id} className="p-6">
                <div className="flex items-center justify-between mb-4">
                  <h3 className="text-lg font-medium text-gray-900">{image.name} {image.version}</h3>
                  <Badge className={
                    image.securityScan.score >= 90 ? 'bg-green-100 text-green-800' :
                    image.securityScan.score >= 70 ? 'bg-yellow-100 text-yellow-800' :
                    'bg-red-100 text-red-800'
                  }>
                    Score: {image.securityScan.score}/100
                  </Badge>
                </div>

                <div className="space-y-4">
                  <div>
                    <div className="flex items-center justify-between mb-2">
                      <span className="text-sm font-medium text-gray-900">Vulnerability Summary</span>
                      <Badge className={
                        image.securityScan.status === 'passed' ? 'bg-green-100 text-green-800' :
                        image.securityScan.status === 'warning' ? 'bg-yellow-100 text-yellow-800' :
                        'bg-red-100 text-red-800'
                      }>
                        {image.securityScan.status}
                      </Badge>
                    </div>
                    <div className="grid grid-cols-4 gap-2 text-center text-sm">
                      <div className={`p-2 rounded ${image.securityScan.criticalVulns > 0 ? 'bg-red-100' : 'bg-gray-100'}`}>
                        <div className={`font-bold ${image.securityScan.criticalVulns > 0 ? 'text-red-600' : 'text-gray-600'}`}>
                          {image.securityScan.criticalVulns}
                        </div>
                        <div className="text-xs text-gray-600">Critical</div>
                      </div>
                      <div className={`p-2 rounded ${image.securityScan.highVulns > 0 ? 'bg-orange-100' : 'bg-gray-100'}`}>
                        <div className={`font-bold ${image.securityScan.highVulns > 0 ? 'text-orange-600' : 'text-gray-600'}`}>
                          {image.securityScan.highVulns}
                        </div>
                        <div className="text-xs text-gray-600">High</div>
                      </div>
                      <div className={`p-2 rounded ${image.securityScan.mediumVulns > 0 ? 'bg-yellow-100' : 'bg-gray-100'}`}>
                        <div className={`font-bold ${image.securityScan.mediumVulns > 0 ? 'text-yellow-600' : 'text-gray-600'}`}>
                          {image.securityScan.mediumVulns}
                        </div>
                        <div className="text-xs text-gray-600">Medium</div>
                      </div>
                      <div className="p-2 rounded bg-gray-100">
                        <div className="font-bold text-gray-600">{image.securityScan.lowVulns}</div>
                        <div className="text-xs text-gray-600">Low</div>
                      </div>
                    </div>
                  </div>

                  <div>
                    <div className="text-sm font-medium text-gray-900 mb-2">Signature Verification</div>
                    <div className="flex items-center space-x-2">
                      <CheckCircle className="w-4 h-4 text-green-600" />
                      <span className="text-sm text-gray-600">Signed by {image.signedBy}</span>
                    </div>
                    <div className="text-xs text-gray-500 mt-1">
                      Signature: {image.signature.substring(0, 20)}...
                    </div>
                  </div>

                  <div>
                    <div className="text-sm font-medium text-gray-900 mb-2">Checksum</div>
                    <div className="text-xs font-mono text-gray-600 bg-gray-100 p-2 rounded">
                      {image.checksum}
                    </div>
                  </div>

                  <div>
                    <div className="text-sm font-medium text-gray-900 mb-2">Scan Details</div>
                    <div className="text-xs text-gray-600">
                      <div>Scanner: {image.securityScan.scanner}</div>
                      <div>Scan Date: {image.securityScan.scanDate.toLocaleString()}</div>
                      <div>Total Vulnerabilities: {image.securityScan.totalVulns}</div>
                    </div>
                  </div>
                </div>
              </Card>
            ))}
          </div>
        </TabsContent>
      </Tabs>

      {/* Build Image Dialog */}
      <Dialog open={showBuildDialog} onOpenChange={setShowBuildDialog}>
        <DialogContent className="max-w-2xl">
          <DialogHeader>
            <DialogTitle>Build New Edge Image</DialogTitle>
          </DialogHeader>
          
          <div className="space-y-4">
            <div className="grid grid-cols-2 gap-4">
              <div>
                <label className="block text-sm font-medium text-gray-700 mb-2">
                  Image Name *
                </label>
                <Input
                  value={buildConfig.name}
                  onChange={(e) => setBuildConfig(prev => ({ ...prev, name: e.target.value }))}
                  placeholder="e.g., AtlasMesh Edge OS Custom"
                  required
                />
              </div>
              <div>
                <label className="block text-sm font-medium text-gray-700 mb-2">
                  Version *
                </label>
                <Input
                  value={buildConfig.version}
                  onChange={(e) => setBuildConfig(prev => ({ ...prev, version: e.target.value }))}
                  placeholder="e.g., v2.1.1-custom"
                  required
                />
              </div>
            </div>

            <div>
              <label className="block text-sm font-medium text-gray-700 mb-2">
                Description
              </label>
              <Textarea
                value={buildConfig.description}
                onChange={(e) => setBuildConfig(prev => ({ ...prev, description: e.target.value }))}
                placeholder="Describe this custom build..."
                rows={3}
              />
            </div>

            <div className="grid grid-cols-2 gap-4">
              <div>
                <label className="block text-sm font-medium text-gray-700 mb-2">
                  Base Image
                </label>
                <Select
                  value={buildConfig.baseImage}
                  onValueChange={(value) => setBuildConfig(prev => ({ ...prev, baseImage: value }))}
                >
                  <option value="ubuntu-22.04-lts">Ubuntu 22.04 LTS</option>
                  <option value="ubuntu-20.04-lts">Ubuntu 20.04 LTS</option>
                  <option value="debian-12">Debian 12</option>
                  <option value="alpine-3.18">Alpine Linux 3.18</option>
                </Select>
              </div>
              <div>
                <label className="block text-sm font-medium text-gray-700 mb-2">
                  Stability
                </label>
                <Select
                  value={buildConfig.stability}
                  onValueChange={(value) => setBuildConfig(prev => ({ 
                    ...prev, 
                    stability: value as 'stable' | 'beta' | 'alpha' | 'experimental'
                  }))}
                >
                  <option value="experimental">Experimental</option>
                  <option value="alpha">Alpha</option>
                  <option value="beta">Beta</option>
                  <option value="stable">Stable</option>
                </Select>
              </div>
            </div>

            <Alert>
              <Package className="w-4 h-4" />
              <AlertDescription>
                The build process will include security scanning, automated testing, and digital signing. 
                Builds typically take 15-30 minutes depending on complexity.
              </AlertDescription>
            </Alert>

            <div className="flex justify-end space-x-3 pt-4">
              <Button
                variant="outline"
                onClick={() => setShowBuildDialog(false)}
              >
                Cancel
              </Button>
              <Button
                onClick={handleBuildImage}
                disabled={!buildConfig.name || !buildConfig.version || isBuilding}
              >
                {isBuilding ? (
                  <>
                    <RefreshCw className="w-4 h-4 mr-2 animate-spin" />
                    Building...
                  </>
                ) : (
                  <>
                    <Play className="w-4 h-4 mr-2" />
                    Start Build
                  </>
                )}
              </Button>
            </div>
          </div>
        </DialogContent>
      </Dialog>

      {/* Image Details Dialog */}
      <Dialog open={showImageDetails} onOpenChange={setShowImageDetails}>
        <DialogContent className="max-w-6xl max-h-[90vh] overflow-y-auto">
          <DialogHeader>
            <DialogTitle>Image Details: {selectedImage?.name}</DialogTitle>
          </DialogHeader>

          {selectedImage && (
            <Tabs defaultValue="overview">
              <TabsList>
                <TabsTrigger value="overview">Overview</TabsTrigger>
                <TabsTrigger value="components">Components</TabsTrigger>
                <TabsTrigger value="security">Security</TabsTrigger>
                <TabsTrigger value="tests">Tests</TabsTrigger>
                <TabsTrigger value="deployment">Deployment</TabsTrigger>
              </TabsList>

              <TabsContent value="overview" className="space-y-4">
                <div className="grid grid-cols-2 gap-6">
                  <Card className="p-4">
                    <h3 className="text-lg font-medium text-gray-900 mb-3">Image Information</h3>
                    <div className="space-y-2 text-sm">
                      <div className="flex justify-between">
                        <span className="text-gray-600">Name:</span>
                        <span className="font-medium">{selectedImage.name}</span>
                      </div>
                      <div className="flex justify-between">
                        <span className="text-gray-600">Version:</span>
                        <span className="font-medium">{selectedImage.version}</span>
                      </div>
                      <div className="flex justify-between">
                        <span className="text-gray-600">Size:</span>
                        <span className="font-medium">{Math.round(selectedImage.size)}MB</span>
                      </div>
                      <div className="flex justify-between">
                        <span className="text-gray-600">Status:</span>
                        <Badge className={getStatusColor(selectedImage.status)}>
                          {selectedImage.status}
                        </Badge>
                      </div>
                      <div className="flex justify-between">
                        <span className="text-gray-600">Stability:</span>
                        <Badge className={getStabilityColor(selectedImage.stability)}>
                          {selectedImage.stability}
                        </Badge>
                      </div>
                    </div>
                  </Card>

                  <Card className="p-4">
                    <h3 className="text-lg font-medium text-gray-900 mb-3">Build Metadata</h3>
                    <div className="space-y-2 text-sm">
                      <div className="flex justify-between">
                        <span className="text-gray-600">Build #:</span>
                        <span className="font-medium">{selectedImage.metadata.buildNumber}</span>
                      </div>
                      <div className="flex justify-between">
                        <span className="text-gray-600">Branch:</span>
                        <span className="font-medium">{selectedImage.metadata.buildBranch}</span>
                      </div>
                      <div className="flex justify-between">
                        <span className="text-gray-600">Commit:</span>
                        <span className="font-medium font-mono">{selectedImage.metadata.buildCommit.substring(0, 8)}</span>
                      </div>
                      <div className="flex justify-between">
                        <span className="text-gray-600">Trigger:</span>
                        <span className="font-medium">{selectedImage.metadata.buildTrigger}</span>
                      </div>
                      <div className="flex justify-between">
                        <span className="text-gray-600">Pipeline:</span>
                        <span className="font-medium">{selectedImage.metadata.buildPipeline}</span>
                      </div>
                    </div>
                  </Card>
                </div>

                <Card className="p-4">
                  <h3 className="text-lg font-medium text-gray-900 mb-3">Description</h3>
                  <p className="text-gray-600">{selectedImage.description}</p>
                </Card>

                <Card className="p-4">
                  <h3 className="text-lg font-medium text-gray-900 mb-3">Release Notes</h3>
                  <p className="text-gray-600">{selectedImage.metadata.releaseNotes}</p>
                </Card>

                <Card className="p-4">
                  <h3 className="text-lg font-medium text-gray-900 mb-3">Tags</h3>
                  <div className="flex flex-wrap gap-2">
                    {selectedImage.metadata.tags.map(tag => (
                      <Badge key={tag} variant="outline">{tag}</Badge>
                    ))}
                  </div>
                </Card>
              </TabsContent>

              <TabsContent value="components" className="space-y-4">
                <div className="space-y-3">
                  {selectedImage.components.map(component => (
                    <Card key={component.name} className="p-4">
                      <div className="flex items-center justify-between mb-2">
                        <div className="flex items-center space-x-3">
                          <h4 className="font-medium text-gray-900">{component.name}</h4>
                          <Badge variant="outline" className="capitalize">
                            {component.type}
                          </Badge>
                          {component.critical && (
                            <Badge className="bg-red-100 text-red-800">Critical</Badge>
                          )}
                        </div>
                        <div className="text-sm font-medium text-gray-900">{component.version}</div>
                      </div>

                      <div className="grid grid-cols-3 gap-4 text-sm">
                        <div>
                          <span className="text-gray-600">Vulnerabilities:</span>
                          <span className={`ml-2 font-medium ${
                            component.vulnerabilities > 0 ? 'text-red-600' : 'text-green-600'
                          }`}>
                            {component.vulnerabilities}
                          </span>
                        </div>
                        <div>
                          <span className="text-gray-600">Licenses:</span>
                          <span className="ml-2 font-medium">{component.licenses.join(', ')}</span>
                        </div>
                        <div>
                          <span className="text-gray-600">Type:</span>
                          <span className="ml-2 font-medium capitalize">{component.type}</span>
                        </div>
                      </div>
                    </Card>
                  ))}
                </div>
              </TabsContent>

              <TabsContent value="security" className="space-y-4">
                <Card className="p-4">
                  <h3 className="text-lg font-medium text-gray-900 mb-3">Security Scan Results</h3>
                  <div className="grid grid-cols-2 gap-6">
                    <div>
                      <div className="text-sm text-gray-600 mb-2">Overall Score</div>
                      <div className={`text-3xl font-bold ${getSecurityScoreColor(selectedImage.securityScan.score)}`}>
                        {selectedImage.securityScan.score}/100
                      </div>
                    </div>
                    <div>
                      <div className="text-sm text-gray-600 mb-2">Status</div>
                      <Badge className={
                        selectedImage.securityScan.status === 'passed' ? 'bg-green-100 text-green-800' :
                        selectedImage.securityScan.status === 'warning' ? 'bg-yellow-100 text-yellow-800' :
                        'bg-red-100 text-red-800'
                      }>
                        {selectedImage.securityScan.status}
                      </Badge>
                    </div>
                  </div>
                </Card>

                <Card className="p-4">
                  <h3 className="text-lg font-medium text-gray-900 mb-3">Vulnerability Breakdown</h3>
                  <div className="grid grid-cols-4 gap-4">
                    <div className="text-center p-3 bg-red-50 rounded">
                      <div className="text-2xl font-bold text-red-600">{selectedImage.securityScan.criticalVulns}</div>
                      <div className="text-sm text-red-800">Critical</div>
                    </div>
                    <div className="text-center p-3 bg-orange-50 rounded">
                      <div className="text-2xl font-bold text-orange-600">{selectedImage.securityScan.highVulns}</div>
                      <div className="text-sm text-orange-800">High</div>
                    </div>
                    <div className="text-center p-3 bg-yellow-50 rounded">
                      <div className="text-2xl font-bold text-yellow-600">{selectedImage.securityScan.mediumVulns}</div>
                      <div className="text-sm text-yellow-800">Medium</div>
                    </div>
                    <div className="text-center p-3 bg-blue-50 rounded">
                      <div className="text-2xl font-bold text-blue-600">{selectedImage.securityScan.lowVulns}</div>
                      <div className="text-sm text-blue-800">Low</div>
                    </div>
                  </div>
                </Card>

                {selectedImage.securityScan.details.length > 0 && (
                  <Card className="p-4">
                    <h3 className="text-lg font-medium text-gray-900 mb-3">Vulnerability Details</h3>
                    <div className="space-y-3">
                      {selectedImage.securityScan.details.map(vuln => (
                        <div key={vuln.id} className="p-3 border rounded">
                          <div className="flex items-center justify-between mb-2">
                            <div className="flex items-center space-x-2">
                              <Badge className={
                                vuln.severity === 'critical' ? 'bg-red-100 text-red-800' :
                                vuln.severity === 'high' ? 'bg-orange-100 text-orange-800' :
                                vuln.severity === 'medium' ? 'bg-yellow-100 text-yellow-800' :
                                'bg-blue-100 text-blue-800'
                              }>
                                {vuln.severity}
                              </Badge>
                              <span className="font-medium">{vuln.component}</span>
                              {vuln.cve && (
                                <Badge variant="outline">{vuln.cve}</Badge>
                              )}
                            </div>
                            {vuln.fixAvailable && (
                              <Badge className="bg-green-100 text-green-800">
                                Fix Available
                                {vuln.fixVersion && <span>: {vuln.fixVersion}</span>}
                              </Badge>
                            )}
                          </div>
                          <p className="text-sm text-gray-600">{vuln.description}</p>
                        </div>
                      ))}
                    </div>
                  </Card>
                )}
              </TabsContent>

              <TabsContent value="tests" className="space-y-4">
                <div className="space-y-4">
                  {selectedImage.testResults.map(test => (
                    <Card key={test.id} className="p-4">
                      <div className="flex items-center justify-between mb-3">
                        <h4 className="font-medium text-gray-900">{test.testSuite}</h4>
                        <Badge className={
                          test.status === 'passed' ? 'bg-green-100 text-green-800' :
                          test.status === 'failed' ? 'bg-red-100 text-red-800' :
                          'bg-yellow-100 text-yellow-800'
                        }>
                          {test.status}
                        </Badge>
                      </div>

                      <div className="grid grid-cols-4 gap-4 mb-3 text-sm">
                        <div>
                          <span className="text-gray-600">Passed:</span>
                          <span className="ml-2 font-medium text-green-600">{test.passedTests}</span>
                        </div>
                        <div>
                          <span className="text-gray-600">Failed:</span>
                          <span className={`ml-2 font-medium ${test.failedTests > 0 ? 'text-red-600' : 'text-green-600'}`}>
                            {test.failedTests}
                          </span>
                        </div>
                        <div>
                          <span className="text-gray-600">Duration:</span>
                          <span className="ml-2 font-medium">{Math.round(test.duration / 60)}m</span>
                        </div>
                        <div>
                          <span className="text-gray-600">Coverage:</span>
                          <span className="ml-2 font-medium">{test.coverage}%</span>
                        </div>
                      </div>

                      <div className="mb-3">
                        <Progress value={(test.passedTests / test.totalTests) * 100} className="h-2" />
                      </div>

                      <p className="text-sm text-gray-600">{test.details}</p>
                    </Card>
                  ))}
                </div>
              </TabsContent>

              <TabsContent value="deployment" className="space-y-4">
                <Card className="p-4">
                  <h3 className="text-lg font-medium text-gray-900 mb-3">Deployment Strategy</h3>
                  <div className="space-y-3">
                    <div>
                      <div className="text-sm text-gray-600">Strategy Type</div>
                      <div className="font-medium capitalize">{selectedImage.deploymentStrategy.type.replace('_', ' ')}</div>
                    </div>
                    
                    <div>
                      <div className="text-sm text-gray-600">Rollback Strategy</div>
                      <div className="font-medium capitalize">{selectedImage.deploymentStrategy.rollbackStrategy}</div>
                    </div>

                    <div>
                      <div className="text-sm text-gray-600">Approval Required</div>
                      <div className="font-medium">
                        {selectedImage.deploymentStrategy.approvalRequired ? 'Yes' : 'No'}
                      </div>
                    </div>
                  </div>
                </Card>

                <Card className="p-4">
                  <h3 className="text-lg font-medium text-gray-900 mb-3">Deployment Stages</h3>
                  <div className="space-y-3">
                    {selectedImage.deploymentStrategy.stages.map((stage, index) => (
                      <div key={stage.name} className="flex items-center justify-between p-3 bg-gray-50 rounded">
                        <div>
                          <div className="font-medium">{stage.name}</div>
                          <div className="text-sm text-gray-600">
                            {stage.percentage}% of nodes • {stage.duration}min duration
                          </div>
                        </div>
                        <div className="text-right">
                          <Badge variant="outline">
                            {stage.autoPromote ? 'Auto-promote' : 'Manual approval'}
                          </Badge>
                        </div>
                      </div>
                    ))}
                  </div>
                </Card>

                <Card className="p-4">
                  <h3 className="text-lg font-medium text-gray-900 mb-3">Health Checks</h3>
                  <div className="space-y-3">
                    {selectedImage.deploymentStrategy.healthChecks.map(check => (
                      <div key={check.name} className="p-3 bg-gray-50 rounded">
                        <div className="font-medium">{check.name}</div>
                        <div className="text-sm text-gray-600 mt-1">
                          Endpoint: {check.endpoint} • 
                          Timeout: {check.timeout}s • 
                          Interval: {check.interval}s • 
                          Retries: {check.retries}
                        </div>
                      </div>
                    ))}
                  </div>
                </Card>
              </TabsContent>
            </Tabs>
          )}
        </DialogContent>
      </Dialog>
    </div>
  )
}

export default ImagePipeline
