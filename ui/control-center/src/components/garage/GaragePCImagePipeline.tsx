import React, { useState, useCallback, useEffect, useMemo } from 'react'
import { 
  HardDrive, Package, Download, Upload, CheckCircle, XCircle, AlertTriangle,
  Clock, Zap, Shield, Key, Hash, Eye, EyeOff, Settings, RefreshCw,
  Play, Pause, SkipForward, Rewind, FileText, Users, Activity, Server,
  Monitor, Terminal, Bug, Wrench, History, MoreHorizontal, Copy, Trash2
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
import { DropdownMenu, DropdownMenuContent, DropdownMenuItem, DropdownMenuTrigger } from '../ui/DropdownMenu'

// Types
interface OSImage {
  id: string
  name: string
  version: string
  baseImage: string
  description: string
  size: number // bytes
  checksum: ImageChecksum
  signature: ImageSignature
  buildInfo: BuildInfo
  deploymentInfo: DeploymentInfo
  status: 'building' | 'built' | 'testing' | 'signed' | 'published' | 'deprecated' | 'failed'
  tags: string[]
  createdAt: Date
  updatedAt: Date
  createdBy: string
  approvedBy?: string
  approvedAt?: Date
  deployedNodes: number
  totalNodes: number
  rolloutStrategy: RolloutStrategy
  vulnerabilities: ImageVulnerability[]
  packages: InstalledPackage[]
  layers: ImageLayer[]
  metadata: ImageMetadata
}

interface ImageChecksum {
  sha256: string
  sha512: string
  md5: string
  verified: boolean
  verifiedAt?: Date
  verificationMethod: 'automatic' | 'manual'
}

interface ImageSignature {
  algorithm: 'rsa' | 'ecdsa' | 'ed25519'
  keyId: string
  signature: string
  publicKey: string
  signed: boolean
  signedAt?: Date
  signedBy?: string
  verified: boolean
  verifiedAt?: Date
  verificationChain: SignatureVerification[]
}

interface SignatureVerification {
  step: string
  status: 'passed' | 'failed' | 'pending'
  timestamp: Date
  details?: string
}

interface BuildInfo {
  buildId: string
  buildNumber: number
  gitCommit: string
  gitBranch: string
  gitTag?: string
  dockerfile: string
  buildArgs: Record<string, string>
  buildDuration: number // seconds
  buildLog: string
  buildAgent: string
  buildEnvironment: string
  dependencies: BuildDependency[]
}

interface BuildDependency {
  name: string
  version: string
  type: 'base_image' | 'package' | 'library'
  source: string
  checksum: string
  verified: boolean
}

interface DeploymentInfo {
  strategy: 'blue_green' | 'canary' | 'rolling' | 'staged'
  stages: DeploymentStage[]
  currentStage: number
  startedAt?: Date
  completedAt?: Date
  estimatedCompletion?: Date
  rollbackPlan: RollbackPlan
  healthChecks: HealthCheck[]
  approvals: DeploymentApproval[]
}

interface DeploymentStage {
  id: string
  name: string
  description: string
  targetNodes: string[]
  percentage: number
  status: 'pending' | 'in_progress' | 'completed' | 'failed' | 'paused'
  startedAt?: Date
  completedAt?: Date
  duration?: number
  successCriteria: SuccessCriteria
  rollbackTriggers: RollbackTrigger[]
  healthCheckResults: HealthCheckResult[]
}

interface SuccessCriteria {
  minSuccessRate: number
  maxFailureRate: number
  maxResponseTime: number
  minUptime: number
  customMetrics: CustomMetric[]
}

interface CustomMetric {
  name: string
  threshold: number
  operator: 'gt' | 'lt' | 'eq' | 'gte' | 'lte'
  currentValue?: number
  status?: 'pass' | 'fail'
}

interface RollbackTrigger {
  condition: string
  threshold: number
  timeWindow: number // minutes
  enabled: boolean
  autoRollback: boolean
}

interface RollbackPlan {
  enabled: boolean
  previousVersion: string
  rollbackSteps: RollbackStep[]
  estimatedDuration: number
  approvalRequired: boolean
  approvers: string[]
}

interface RollbackStep {
  id: string
  name: string
  description: string
  command: string
  timeout: number
  retries: number
}

interface HealthCheck {
  id: string
  name: string
  type: 'http' | 'tcp' | 'command' | 'custom'
  endpoint?: string
  command?: string
  expectedResponse?: string
  timeout: number
  interval: number
  retries: number
  enabled: boolean
}

interface HealthCheckResult {
  healthCheckId: string
  nodeId: string
  status: 'healthy' | 'unhealthy' | 'unknown'
  response?: string
  responseTime: number
  timestamp: Date
  error?: string
}

interface DeploymentApproval {
  stage: string
  approver: string
  status: 'pending' | 'approved' | 'rejected'
  timestamp: Date
  reason?: string
  conditions: string[]
}

interface RolloutStrategy {
  type: 'immediate' | 'scheduled' | 'manual'
  schedule?: Date
  batchSize: number
  batchInterval: number // minutes
  pauseBetweenBatches: boolean
  canaryPercentage?: number
  autoPromote: boolean
  promotionCriteria?: SuccessCriteria
}

interface ImageVulnerability {
  id: string
  cveId?: string
  severity: 'critical' | 'high' | 'medium' | 'low'
  package: string
  version: string
  fixedVersion?: string
  description: string
  vector: string
  cvssScore: number
  exploitAvailable: boolean
  patchAvailable: boolean
  status: 'open' | 'fixed' | 'ignored' | 'false_positive'
}

interface InstalledPackage {
  name: string
  version: string
  architecture: string
  size: number
  description: string
  maintainer: string
  source: string
  essential: boolean
  vulnerabilities: number
  lastUpdated: Date
}

interface ImageLayer {
  id: string
  digest: string
  size: number
  command: string
  createdAt: Date
  createdBy: string
  isEmpty: boolean
}

interface ImageMetadata {
  architecture: string
  os: string
  osVersion: string
  kernel: string
  environment: Record<string, string>
  labels: Record<string, string>
  exposedPorts: string[]
  volumes: string[]
  workingDirectory: string
  entrypoint: string[]
  command: string[]
  user: string
  healthCheck?: {
    test: string[]
    interval: string
    timeout: string
    retries: number
  }
}

interface GarageNode {
  id: string
  name: string
  location: string
  ipAddress: string
  currentImage: string
  targetImage?: string
  status: 'online' | 'offline' | 'updating' | 'failed' | 'maintenance'
  lastSeen: Date
  deploymentProgress?: number
  deploymentStage?: string
  healthStatus: 'healthy' | 'unhealthy' | 'unknown'
  systemInfo: {
    cpu: string
    memory: string
    disk: string
    architecture: string
    osVersion: string
  }
}

interface GaragePCImagePipelineProps {
  onImageDeployed?: (image: OSImage, nodes: string[]) => void
  onDeploymentFailed?: (image: OSImage, error: string) => void
  className?: string
}

const GaragePCImagePipeline: React.FC<GaragePCImagePipelineProps> = ({
  onImageDeployed,
  onDeploymentFailed,
  className = ''
}) => {
  // State
  const [images, setImages] = useState<OSImage[]>([])
  const [nodes, setNodes] = useState<GarageNode[]>([])
  const [selectedImage, setSelectedImage] = useState<OSImage | null>(null)
  const [showImageDialog, setShowImageDialog] = useState(false)
  const [showDeployDialog, setShowDeployDialog] = useState(false)
  const [activeTab, setActiveTab] = useState('images')
  const [filters, setFilters] = useState({
    status: '',
    tag: '',
    search: '',
    architecture: ''
  })
  const [deploymentInProgress, setDeploymentInProgress] = useState<string | null>(null)
  const [buildInProgress, setBuildInProgress] = useState(false)
  const [buildProgress, setBuildProgress] = useState(0)

  // Mock data initialization
  const initializeMockData = useCallback(() => {
    const mockImages: OSImage[] = [
      {
        id: 'img-ubuntu-22-04-v1-2-3',
        name: 'AtlasMesh Garage OS',
        version: '1.2.3',
        baseImage: 'ubuntu:22.04',
        description: 'Production-ready OS image for AtlasMesh garage PCs with security hardening and fleet management tools',
        size: 2.8 * 1024 * 1024 * 1024, // 2.8GB
        checksum: {
          sha256: 'e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855',
          sha512: 'cf83e1357eefb8bdf1542850d66d8007d620e4050b5715dc83f4a921d36ce9ce47d0d13c5d85f2b0ff8318d2877eec2f63b931bd47417a81a538327af927da3e',
          md5: 'd41d8cd98f00b204e9800998ecf8427e',
          verified: true,
          verifiedAt: new Date('2024-11-26T08:00:00Z'),
          verificationMethod: 'automatic'
        },
        signature: {
          algorithm: 'rsa',
          keyId: 'atlasmesh-signing-key-2024',
          signature: 'MEUCIQDKn7VfY...',
          publicKey: '-----BEGIN PUBLIC KEY-----\nMIIBIjANBg...',
          signed: true,
          signedAt: new Date('2024-11-26T08:30:00Z'),
          signedBy: 'build-system',
          verified: true,
          verifiedAt: new Date('2024-11-26T08:31:00Z'),
          verificationChain: [
            {
              step: 'Key validation',
              status: 'passed',
              timestamp: new Date('2024-11-26T08:30:30Z')
            },
            {
              step: 'Signature verification',
              status: 'passed',
              timestamp: new Date('2024-11-26T08:31:00Z')
            },
            {
              step: 'Chain of trust validation',
              status: 'passed',
              timestamp: new Date('2024-11-26T08:31:15Z')
            }
          ]
        },
        buildInfo: {
          buildId: 'build-20241126-083000',
          buildNumber: 1456,
          gitCommit: 'a1b2c3d4e5f6789012345678901234567890abcd',
          gitBranch: 'main',
          gitTag: 'v1.2.3',
          dockerfile: 'FROM ubuntu:22.04\nRUN apt-get update && apt-get install -y...',
          buildArgs: {
            'ATLASMESH_VERSION': '1.2.3',
            'BUILD_DATE': '2024-11-26T08:30:00Z',
            'SECURITY_PATCHES': 'enabled'
          },
          buildDuration: 1847, // ~30 minutes
          buildLog: 'Step 1/15 : FROM ubuntu:22.04\n ---> sha256:e3b0c44...\nStep 2/15 : RUN apt-get update...',
          buildAgent: 'jenkins-agent-01',
          buildEnvironment: 'production',
          dependencies: [
            {
              name: 'ubuntu',
              version: '22.04',
              type: 'base_image',
              source: 'docker.io/library/ubuntu',
              checksum: 'sha256:e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855',
              verified: true
            },
            {
              name: 'docker-ce',
              version: '24.0.7',
              type: 'package',
              source: 'docker.com/apt',
              checksum: 'sha256:f1c2d3e4f5a6b7c8d9e0f1a2b3c4d5e6f7a8b9c0d1e2f3a4b5c6d7e8f9a0b1c2',
              verified: true
            }
          ]
        },
        deploymentInfo: {
          strategy: 'staged',
          stages: [
            {
              id: 'stage-canary',
              name: 'Canary Deployment',
              description: 'Deploy to 2 canary nodes for validation',
              targetNodes: ['garage-pc-001', 'garage-pc-002'],
              percentage: 10,
              status: 'completed',
              startedAt: new Date('2024-11-26T09:00:00Z'),
              completedAt: new Date('2024-11-26T09:15:00Z'),
              duration: 15,
              successCriteria: {
                minSuccessRate: 95,
                maxFailureRate: 5,
                maxResponseTime: 5000,
                minUptime: 99,
                customMetrics: []
              },
              rollbackTriggers: [
                {
                  condition: 'error_rate > 5%',
                  threshold: 5,
                  timeWindow: 10,
                  enabled: true,
                  autoRollback: true
                }
              ],
              healthCheckResults: [
                {
                  healthCheckId: 'hc-system-health',
                  nodeId: 'garage-pc-001',
                  status: 'healthy',
                  responseTime: 150,
                  timestamp: new Date('2024-11-26T09:14:00Z')
                }
              ]
            },
            {
              id: 'stage-batch-1',
              name: 'Batch 1 - 50% Rollout',
              description: 'Deploy to 50% of remaining nodes',
              targetNodes: ['garage-pc-003', 'garage-pc-004', 'garage-pc-005'],
              percentage: 50,
              status: 'in_progress',
              startedAt: new Date('2024-11-26T10:00:00Z'),
              successCriteria: {
                minSuccessRate: 95,
                maxFailureRate: 5,
                maxResponseTime: 5000,
                minUptime: 99,
                customMetrics: []
              },
              rollbackTriggers: [
                {
                  condition: 'error_rate > 3%',
                  threshold: 3,
                  timeWindow: 15,
                  enabled: true,
                  autoRollback: false
                }
              ],
              healthCheckResults: []
            },
            {
              id: 'stage-final',
              name: 'Final Rollout',
              description: 'Deploy to all remaining nodes',
              targetNodes: ['garage-pc-006', 'garage-pc-007', 'garage-pc-008'],
              percentage: 100,
              status: 'pending',
              successCriteria: {
                minSuccessRate: 98,
                maxFailureRate: 2,
                maxResponseTime: 3000,
                minUptime: 99.5,
                customMetrics: []
              },
              rollbackTriggers: [],
              healthCheckResults: []
            }
          ],
          currentStage: 1,
          startedAt: new Date('2024-11-26T09:00:00Z'),
          estimatedCompletion: new Date('2024-11-26T12:00:00Z'),
          rollbackPlan: {
            enabled: true,
            previousVersion: '1.2.2',
            rollbackSteps: [
              {
                id: 'step-stop-services',
                name: 'Stop AtlasMesh services',
                description: 'Gracefully stop all AtlasMesh services',
                command: 'systemctl stop atlasmesh-*',
                timeout: 300,
                retries: 3
              },
              {
                id: 'step-restore-image',
                name: 'Restore previous image',
                description: 'Restore the previous working image',
                command: 'docker load < /backup/atlasmesh-1.2.2.tar',
                timeout: 600,
                retries: 1
              }
            ],
            estimatedDuration: 15,
            approvalRequired: true,
            approvers: ['ops-team', 'security-team']
          },
          healthChecks: [
            {
              id: 'hc-system-health',
              name: 'System Health Check',
              type: 'http',
              endpoint: 'http://localhost:8080/health',
              expectedResponse: '{"status":"healthy"}',
              timeout: 30,
              interval: 60,
              retries: 3,
              enabled: true
            },
            {
              id: 'hc-docker-daemon',
              name: 'Docker Daemon Check',
              type: 'command',
              command: 'docker ps',
              timeout: 10,
              interval: 30,
              retries: 2,
              enabled: true
            }
          ],
          approvals: [
            {
              stage: 'stage-canary',
              approver: 'ops-manager',
              status: 'approved',
              timestamp: new Date('2024-11-26T08:45:00Z'),
              conditions: ['Canary metrics look good', 'No security issues detected']
            },
            {
              stage: 'stage-batch-1',
              approver: 'security-team',
              status: 'approved',
              timestamp: new Date('2024-11-26T09:55:00Z'),
              conditions: ['Security scan passed', 'Vulnerability assessment complete']
            }
          ]
        },
        status: 'signed',
        tags: ['production', 'ubuntu-22.04', 'hardened', 'garage-os'],
        createdAt: new Date('2024-11-26T08:00:00Z'),
        updatedAt: new Date('2024-11-26T08:31:00Z'),
        createdBy: 'build-system',
        approvedBy: 'security-team',
        approvedAt: new Date('2024-11-26T08:45:00Z'),
        deployedNodes: 2,
        totalNodes: 8,
        rolloutStrategy: {
          type: 'scheduled',
          schedule: new Date('2024-11-26T09:00:00Z'),
          batchSize: 2,
          batchInterval: 30,
          pauseBetweenBatches: true,
          canaryPercentage: 10,
          autoPromote: false,
          promotionCriteria: {
            minSuccessRate: 95,
            maxFailureRate: 5,
            maxResponseTime: 5000,
            minUptime: 99,
            customMetrics: []
          }
        },
        vulnerabilities: [
          {
            id: 'vuln-001',
            cveId: 'CVE-2024-XXXX',
            severity: 'medium',
            package: 'libssl3',
            version: '3.0.2-0ubuntu1.10',
            fixedVersion: '3.0.2-0ubuntu1.12',
            description: 'OpenSSL vulnerability in certificate validation',
            vector: 'CVSS:3.1/AV:N/AC:L/PR:N/UI:N/S:U/C:N/I:L/A:N',
            cvssScore: 5.3,
            exploitAvailable: false,
            patchAvailable: true,
            status: 'open'
          }
        ],
        packages: [
          {
            name: 'docker-ce',
            version: '24.0.7-1~ubuntu.22.04~jammy',
            architecture: 'amd64',
            size: 25 * 1024 * 1024, // 25MB
            description: 'Docker Community Edition',
            maintainer: 'Docker <docker@docker.com>',
            source: 'docker-ce',
            essential: true,
            vulnerabilities: 0,
            lastUpdated: new Date('2024-11-20T00:00:00Z')
          }
        ],
        layers: [
          {
            id: 'layer-001',
            digest: 'sha256:e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855',
            size: 72.8 * 1024 * 1024, // 72.8MB
            command: 'FROM ubuntu:22.04',
            createdAt: new Date('2024-11-26T08:00:00Z'),
            createdBy: 'build-system',
            isEmpty: false
          }
        ],
        metadata: {
          architecture: 'amd64',
          os: 'linux',
          osVersion: 'Ubuntu 22.04.3 LTS',
          kernel: '5.15.0-91-generic',
          environment: {
            'PATH': '/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin',
            'ATLASMESH_VERSION': '1.2.3',
            'ATLASMESH_ENV': 'production'
          },
          labels: {
            'org.atlasmesh.version': '1.2.3',
            'org.atlasmesh.build-date': '2024-11-26T08:30:00Z',
            'org.atlasmesh.security-scan': 'passed'
          },
          exposedPorts: ['8080/tcp', '8443/tcp'],
          volumes: ['/var/lib/atlasmesh', '/var/log/atlasmesh'],
          workingDirectory: '/app',
          entrypoint: ['/usr/local/bin/atlasmesh-init'],
          command: ['start'],
          user: 'atlasmesh',
          healthCheck: {
            test: ['CMD', 'curl', '-f', 'http://localhost:8080/health'],
            interval: '30s',
            timeout: '10s',
            retries: 3
          }
        }
      },
      {
        id: 'img-ubuntu-22-04-v1-2-2',
        name: 'AtlasMesh Garage OS',
        version: '1.2.2',
        baseImage: 'ubuntu:22.04',
        description: 'Previous stable version - currently deployed to production',
        size: 2.7 * 1024 * 1024 * 1024, // 2.7GB
        checksum: {
          sha256: 'f4e5d6c7b8a9012345678901234567890123456789012345678901234567890123',
          sha512: 'a1b2c3d4e5f6789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456',
          md5: '123456789012345678901234567890ab',
          verified: true,
          verifiedAt: new Date('2024-11-20T10:00:00Z'),
          verificationMethod: 'automatic'
        },
        signature: {
          algorithm: 'rsa',
          keyId: 'atlasmesh-signing-key-2024',
          signature: 'MEUCIQCKn7VfY...',
          publicKey: '-----BEGIN PUBLIC KEY-----\nMIIBIjANBg...',
          signed: true,
          signedAt: new Date('2024-11-20T10:30:00Z'),
          signedBy: 'build-system',
          verified: true,
          verifiedAt: new Date('2024-11-20T10:31:00Z'),
          verificationChain: [
            {
              step: 'Key validation',
              status: 'passed',
              timestamp: new Date('2024-11-20T10:30:30Z')
            },
            {
              step: 'Signature verification',
              status: 'passed',
              timestamp: new Date('2024-11-20T10:31:00Z')
            }
          ]
        },
        buildInfo: {
          buildId: 'build-20241120-103000',
          buildNumber: 1455,
          gitCommit: 'b2c3d4e5f6a7890123456789012345678901bcdef',
          gitBranch: 'main',
          gitTag: 'v1.2.2',
          dockerfile: 'FROM ubuntu:22.04\nRUN apt-get update && apt-get install -y...',
          buildArgs: {
            'ATLASMESH_VERSION': '1.2.2',
            'BUILD_DATE': '2024-11-20T10:30:00Z'
          },
          buildDuration: 1789,
          buildLog: 'Step 1/15 : FROM ubuntu:22.04\n ---> sha256:f4e5d6c7...',
          buildAgent: 'jenkins-agent-02',
          buildEnvironment: 'production',
          dependencies: []
        },
        deploymentInfo: {
          strategy: 'rolling',
          stages: [
            {
              id: 'stage-production',
              name: 'Production Deployment',
              description: 'Deployed to all production nodes',
              targetNodes: ['garage-pc-001', 'garage-pc-002', 'garage-pc-003', 'garage-pc-004', 'garage-pc-005', 'garage-pc-006'],
              percentage: 100,
              status: 'completed',
              startedAt: new Date('2024-11-20T11:00:00Z'),
              completedAt: new Date('2024-11-20T12:30:00Z'),
              duration: 90,
              successCriteria: {
                minSuccessRate: 98,
                maxFailureRate: 2,
                maxResponseTime: 3000,
                minUptime: 99.5,
                customMetrics: []
              },
              rollbackTriggers: [],
              healthCheckResults: []
            }
          ],
          currentStage: 0,
          startedAt: new Date('2024-11-20T11:00:00Z'),
          completedAt: new Date('2024-11-20T12:30:00Z'),
          rollbackPlan: {
            enabled: false,
            previousVersion: '1.2.1',
            rollbackSteps: [],
            estimatedDuration: 0,
            approvalRequired: false,
            approvers: []
          },
          healthChecks: [],
          approvals: []
        },
        status: 'published',
        tags: ['production', 'stable', 'deployed'],
        createdAt: new Date('2024-11-20T10:00:00Z'),
        updatedAt: new Date('2024-11-20T12:30:00Z'),
        createdBy: 'build-system',
        approvedBy: 'ops-team',
        approvedAt: new Date('2024-11-20T10:45:00Z'),
        deployedNodes: 6,
        totalNodes: 8,
        rolloutStrategy: {
          type: 'immediate',
          batchSize: 3,
          batchInterval: 15,
          pauseBetweenBatches: false,
          autoPromote: true
        },
        vulnerabilities: [],
        packages: [],
        layers: [],
        metadata: {
          architecture: 'amd64',
          os: 'linux',
          osVersion: 'Ubuntu 22.04.3 LTS',
          kernel: '5.15.0-88-generic',
          environment: {},
          labels: {},
          exposedPorts: [],
          volumes: [],
          workingDirectory: '/app',
          entrypoint: [],
          command: [],
          user: 'root'
        }
      }
    ]

    const mockNodes: GarageNode[] = [
      {
        id: 'garage-pc-001',
        name: 'Depot-A-Station-01',
        location: 'Dubai Main Depot - Bay A',
        ipAddress: '192.168.10.101',
        currentImage: 'img-ubuntu-22-04-v1-2-3',
        status: 'online',
        lastSeen: new Date('2024-11-26T10:45:00Z'),
        healthStatus: 'healthy',
        systemInfo: {
          cpu: 'Intel Core i7-11700 @ 2.50GHz',
          memory: '32GB DDR4',
          disk: '512GB SSD',
          architecture: 'amd64',
          osVersion: 'Ubuntu 22.04.3 LTS'
        }
      },
      {
        id: 'garage-pc-002',
        name: 'Depot-A-Station-02',
        location: 'Dubai Main Depot - Bay A',
        ipAddress: '192.168.10.102',
        currentImage: 'img-ubuntu-22-04-v1-2-3',
        status: 'online',
        lastSeen: new Date('2024-11-26T10:44:00Z'),
        healthStatus: 'healthy',
        systemInfo: {
          cpu: 'Intel Core i7-11700 @ 2.50GHz',
          memory: '32GB DDR4',
          disk: '512GB SSD',
          architecture: 'amd64',
          osVersion: 'Ubuntu 22.04.3 LTS'
        }
      },
      {
        id: 'garage-pc-003',
        name: 'Depot-A-Station-03',
        location: 'Dubai Main Depot - Bay B',
        ipAddress: '192.168.10.103',
        currentImage: 'img-ubuntu-22-04-v1-2-2',
        targetImage: 'img-ubuntu-22-04-v1-2-3',
        status: 'updating',
        lastSeen: new Date('2024-11-26T10:43:00Z'),
        deploymentProgress: 65,
        deploymentStage: 'stage-batch-1',
        healthStatus: 'unknown',
        systemInfo: {
          cpu: 'Intel Core i7-11700 @ 2.50GHz',
          memory: '32GB DDR4',
          disk: '512GB SSD',
          architecture: 'amd64',
          osVersion: 'Ubuntu 22.04.3 LTS'
        }
      },
      {
        id: 'garage-pc-004',
        name: 'Depot-A-Station-04',
        location: 'Dubai Main Depot - Bay B',
        ipAddress: '192.168.10.104',
        currentImage: 'img-ubuntu-22-04-v1-2-2',
        targetImage: 'img-ubuntu-22-04-v1-2-3',
        status: 'updating',
        lastSeen: new Date('2024-11-26T10:42:00Z'),
        deploymentProgress: 35,
        deploymentStage: 'stage-batch-1',
        healthStatus: 'unknown',
        systemInfo: {
          cpu: 'Intel Core i7-11700 @ 2.50GHz',
          memory: '32GB DDR4',
          disk: '512GB SSD',
          architecture: 'amd64',
          osVersion: 'Ubuntu 22.04.3 LTS'
        }
      },
      {
        id: 'garage-pc-005',
        name: 'Depot-B-Station-01',
        location: 'Abu Dhabi Depot - Bay A',
        ipAddress: '192.168.20.101',
        currentImage: 'img-ubuntu-22-04-v1-2-2',
        status: 'online',
        lastSeen: new Date('2024-11-26T10:41:00Z'),
        healthStatus: 'healthy',
        systemInfo: {
          cpu: 'Intel Core i7-11700 @ 2.50GHz',
          memory: '32GB DDR4',
          disk: '512GB SSD',
          architecture: 'amd64',
          osVersion: 'Ubuntu 22.04.3 LTS'
        }
      }
    ]

    setImages(mockImages)
    setNodes(mockNodes)
  }, [])

  useEffect(() => {
    initializeMockData()
  }, [initializeMockData])

  // Real-time updates simulation
  useEffect(() => {
    const interval = setInterval(() => {
      // Update deployment progress for nodes that are updating
      setNodes(prev => prev.map(node => {
        if (node.status === 'updating' && node.deploymentProgress !== undefined) {
          const newProgress = Math.min(100, node.deploymentProgress + Math.random() * 5)
          if (newProgress >= 100) {
            return {
              ...node,
              status: 'online',
              currentImage: node.targetImage || node.currentImage,
              targetImage: undefined,
              deploymentProgress: undefined,
              deploymentStage: undefined,
              healthStatus: 'healthy'
            }
          }
          return {
            ...node,
            deploymentProgress: newProgress,
            lastSeen: new Date()
          }
        }
        return {
          ...node,
          lastSeen: new Date()
        }
      }))

      // Update image deployment info
      setImages(prev => prev.map(image => {
        if (image.status === 'signed' && image.deploymentInfo.currentStage < image.deploymentInfo.stages.length) {
          const currentStage = image.deploymentInfo.stages[image.deploymentInfo.currentStage]
          if (currentStage.status === 'in_progress') {
            // Simulate stage completion
            if (Math.random() > 0.95) {
              const updatedStages = [...image.deploymentInfo.stages]
              updatedStages[image.deploymentInfo.currentStage] = {
                ...currentStage,
                status: 'completed',
                completedAt: new Date()
              }
              
              return {
                ...image,
                deploymentInfo: {
                  ...image.deploymentInfo,
                  stages: updatedStages,
                  currentStage: Math.min(image.deploymentInfo.currentStage + 1, image.deploymentInfo.stages.length - 1)
                }
              }
            }
          }
        }
        return image
      }))
    }, 2000)

    return () => clearInterval(interval)
  }, [])

  // Handlers
  const handleImageSelect = useCallback((image: OSImage) => {
    setSelectedImage(image)
    setShowImageDialog(true)
  }, [])

  const handleBuildImage = useCallback(async () => {
    setBuildInProgress(true)
    setBuildProgress(0)

    // Simulate build progress
    const progressInterval = setInterval(() => {
      setBuildProgress(prev => {
        const newProgress = prev + Math.random() * 8
        if (newProgress >= 100) {
          clearInterval(progressInterval)
          setBuildInProgress(false)
          return 100
        }
        return newProgress
      })
    }, 1000)

    // Create new image after build completes
    setTimeout(() => {
      clearInterval(progressInterval)
      setBuildInProgress(false)
      setBuildProgress(100)
      
      const newImage: OSImage = {
        id: `img-ubuntu-22-04-v1-2-4`,
        name: 'AtlasMesh Garage OS',
        version: '1.2.4',
        baseImage: 'ubuntu:22.04',
        description: 'Latest build with security updates and new features',
        size: 2.9 * 1024 * 1024 * 1024,
        checksum: {
          sha256: 'new-checksum-' + Date.now(),
          sha512: 'new-sha512-' + Date.now(),
          md5: 'new-md5-' + Date.now(),
          verified: false,
          verificationMethod: 'automatic'
        },
        signature: {
          algorithm: 'rsa',
          keyId: 'atlasmesh-signing-key-2024',
          signature: '',
          publicKey: '',
          signed: false,
          verified: false,
          verificationChain: []
        },
        buildInfo: {
          buildId: `build-${Date.now()}`,
          buildNumber: 1457,
          gitCommit: 'new-commit-' + Date.now(),
          gitBranch: 'main',
          dockerfile: 'FROM ubuntu:22.04\n...',
          buildArgs: {},
          buildDuration: 1920,
          buildLog: 'Build completed successfully',
          buildAgent: 'jenkins-agent-01',
          buildEnvironment: 'production',
          dependencies: []
        },
        deploymentInfo: {
          strategy: 'staged',
          stages: [],
          currentStage: 0,
          rollbackPlan: {
            enabled: false,
            previousVersion: '',
            rollbackSteps: [],
            estimatedDuration: 0,
            approvalRequired: false,
            approvers: []
          },
          healthChecks: [],
          approvals: []
        },
        status: 'built',
        tags: ['latest', 'ubuntu-22.04'],
        createdAt: new Date(),
        updatedAt: new Date(),
        createdBy: 'build-system',
        deployedNodes: 0,
        totalNodes: 0,
        rolloutStrategy: {
          type: 'manual',
          batchSize: 2,
          batchInterval: 30,
          pauseBetweenBatches: true,
          autoPromote: false
        },
        vulnerabilities: [],
        packages: [],
        layers: [],
        metadata: {
          architecture: 'amd64',
          os: 'linux',
          osVersion: 'Ubuntu 22.04.3 LTS',
          kernel: '5.15.0-91-generic',
          environment: {},
          labels: {},
          exposedPorts: [],
          volumes: [],
          workingDirectory: '/app',
          entrypoint: [],
          command: [],
          user: 'root'
        }
      }

      setImages(prev => [newImage, ...prev])
    }, 15000)
  }, [])

  const handleSignImage = useCallback((imageId: string) => {
    setImages(prev => prev.map(image => 
      image.id === imageId 
        ? {
            ...image,
            signature: {
              ...image.signature,
              signed: true,
              signedAt: new Date(),
              signedBy: 'security-team',
              signature: 'MEUCIQDKn7VfY...',
              verified: true,
              verifiedAt: new Date(),
              verificationChain: [
                {
                  step: 'Key validation',
                  status: 'passed',
                  timestamp: new Date()
                },
                {
                  step: 'Signature verification',
                  status: 'passed',
                  timestamp: new Date()
                }
              ]
            },
            checksum: {
              ...image.checksum,
              verified: true,
              verifiedAt: new Date()
            },
            status: 'signed',
            approvedBy: 'security-team',
            approvedAt: new Date()
          }
        : image
    ))
  }, [])

  const handleDeployImage = useCallback((imageId: string, nodeIds: string[]) => {
    setDeploymentInProgress(imageId)
    
    // Update image deployment info
    setImages(prev => prev.map(image => 
      image.id === imageId 
        ? {
            ...image,
            deploymentInfo: {
              ...image.deploymentInfo,
              startedAt: new Date(),
              currentStage: 0,
              stages: [
                {
                  id: 'stage-deploy',
                  name: 'Deployment',
                  description: `Deploy to ${nodeIds.length} selected nodes`,
                  targetNodes: nodeIds,
                  percentage: 100,
                  status: 'in_progress',
                  startedAt: new Date(),
                  successCriteria: {
                    minSuccessRate: 95,
                    maxFailureRate: 5,
                    maxResponseTime: 5000,
                    minUptime: 99,
                    customMetrics: []
                  },
                  rollbackTriggers: [],
                  healthCheckResults: []
                }
              ]
            },
            deployedNodes: 0,
            totalNodes: nodeIds.length
          }
        : image
    ))

    // Update target nodes
    setNodes(prev => prev.map(node => 
      nodeIds.includes(node.id)
        ? {
            ...node,
            targetImage: imageId,
            status: 'updating',
            deploymentProgress: 0,
            deploymentStage: 'stage-deploy',
            healthStatus: 'unknown'
          }
        : node
    ))

    // Simulate deployment completion
    setTimeout(() => {
      setDeploymentInProgress(null)
      onImageDeployed?.(images.find(img => img.id === imageId)!, nodeIds)
    }, 10000)
  }, [images, onImageDeployed])

  // Filtered data
  const filteredImages = useMemo(() => {
    return images
      .filter(image => !filters.status || image.status === filters.status)
      .filter(image => !filters.tag || image.tags.some(tag => tag.includes(filters.tag)))
      .filter(image => !filters.architecture || image.metadata.architecture === filters.architecture)
      .filter(image => {
        if (!filters.search) return true
        const searchTerm = filters.search.toLowerCase()
        return (
          image.name.toLowerCase().includes(searchTerm) ||
          image.version.toLowerCase().includes(searchTerm) ||
          image.description.toLowerCase().includes(searchTerm)
        )
      })
      .sort((a, b) => b.createdAt.getTime() - a.createdAt.getTime())
  }, [images, filters])

  // Statistics
  const stats = useMemo(() => {
    const totalImages = images.length
    const signedImages = images.filter(img => img.status === 'signed' || img.status === 'published').length
    const deployedImages = images.filter(img => img.deployedNodes > 0).length
    const vulnerableImages = images.filter(img => img.vulnerabilities.length > 0).length
    
    const totalNodes = nodes.length
    const onlineNodes = nodes.filter(n => n.status === 'online').length
    const updatingNodes = nodes.filter(n => n.status === 'updating').length
    const healthyNodes = nodes.filter(n => n.healthStatus === 'healthy').length
    
    const totalDeployments = images.reduce((sum, img) => sum + img.deployedNodes, 0)
    const activeDeployments = deploymentInProgress ? 1 : 0

    return {
      totalImages,
      signedImages,
      deployedImages,
      vulnerableImages,
      totalNodes,
      onlineNodes,
      updatingNodes,
      healthyNodes,
      totalDeployments,
      activeDeployments
    }
  }, [images, nodes, deploymentInProgress])

  const getStatusColor = (status: string) => {
    switch (status) {
      case 'building': return 'bg-blue-100 text-blue-800'
      case 'built': return 'bg-yellow-100 text-yellow-800'
      case 'testing': return 'bg-purple-100 text-purple-800'
      case 'signed': return 'bg-green-100 text-green-800'
      case 'published': return 'bg-indigo-100 text-indigo-800'
      case 'deprecated': return 'bg-gray-100 text-gray-800'
      case 'failed': return 'bg-red-100 text-red-800'
      default: return 'bg-gray-100 text-gray-800'
    }
  }

  const getNodeStatusColor = (status: string) => {
    switch (status) {
      case 'online': return 'bg-green-100 text-green-800'
      case 'offline': return 'bg-gray-100 text-gray-800'
      case 'updating': return 'bg-blue-100 text-blue-800'
      case 'failed': return 'bg-red-100 text-red-800'
      case 'maintenance': return 'bg-yellow-100 text-yellow-800'
      default: return 'bg-gray-100 text-gray-800'
    }
  }

  const formatBytes = (bytes: number) => {
    const units = ['B', 'KB', 'MB', 'GB', 'TB']
    let size = bytes
    let unitIndex = 0
    
    while (size >= 1024 && unitIndex < units.length - 1) {
      size /= 1024
      unitIndex++
    }
    
    return `${size.toFixed(1)} ${units[unitIndex]}`
  }

  return (
    <div className={`space-y-6 ${className}`}>
      {/* Header */}
      <div className="flex items-center justify-between">
        <div className="flex items-center space-x-3">
          <HardDrive className="w-6 h-6 text-blue-600" />
          <h2 className="text-2xl font-semibold text-gray-900">Garage PC Image Pipeline</h2>
        </div>
        <div className="flex items-center space-x-2">
          <Button 
            variant="outline" 
            onClick={handleBuildImage}
            disabled={buildInProgress}
          >
            {buildInProgress ? (
              <>
                <Activity className="w-4 h-4 mr-2 animate-spin" />
                Building... {Math.round(buildProgress)}%
              </>
            ) : (
              <>
                <Package className="w-4 h-4 mr-2" />
                Build New Image
              </>
            )}
          </Button>
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

      {/* Statistics */}
      <div className="grid grid-cols-2 md:grid-cols-5 lg:grid-cols-10 gap-4">
        <Card className="p-4">
          <div className="text-center">
            <div className="text-2xl font-bold text-gray-900">{stats.totalImages}</div>
            <div className="text-sm text-gray-600">Total Images</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className="text-2xl font-bold text-green-600">{stats.signedImages}</div>
            <div className="text-sm text-gray-600">Signed</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className="text-2xl font-bold text-blue-600">{stats.deployedImages}</div>
            <div className="text-sm text-gray-600">Deployed</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className="text-2xl font-bold text-red-600">{stats.vulnerableImages}</div>
            <div className="text-sm text-gray-600">Vulnerable</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className="text-2xl font-bold text-purple-600">{stats.totalNodes}</div>
            <div className="text-sm text-gray-600">Total Nodes</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className="text-2xl font-bold text-indigo-600">{stats.onlineNodes}</div>
            <div className="text-sm text-gray-600">Online</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className="text-2xl font-bold text-orange-600">{stats.updatingNodes}</div>
            <div className="text-sm text-gray-600">Updating</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className="text-2xl font-bold text-teal-600">{stats.healthyNodes}</div>
            <div className="text-sm text-gray-600">Healthy</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className="text-2xl font-bold text-pink-600">{stats.totalDeployments}</div>
            <div className="text-sm text-gray-600">Deployments</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className="text-2xl font-bold text-amber-600">{stats.activeDeployments}</div>
            <div className="text-sm text-gray-600">Active</div>
          </div>
        </Card>
      </div>

      {/* Build Progress */}
      {buildInProgress && (
        <Card className="p-4">
          <div className="flex items-center justify-between mb-2">
            <span className="font-medium text-gray-900">Building New Image</span>
            <span className="text-sm text-gray-600">{Math.round(buildProgress)}%</span>
          </div>
          <Progress value={buildProgress} className="w-full" />
        </Card>
      )}

      <Tabs value={activeTab} onValueChange={setActiveTab}>
        <TabsList>
          <TabsTrigger value="images">OS Images</TabsTrigger>
          <TabsTrigger value="nodes">Garage Nodes</TabsTrigger>
          <TabsTrigger value="deployments">Active Deployments</TabsTrigger>
          <TabsTrigger value="pipeline">Build Pipeline</TabsTrigger>
        </TabsList>

        <TabsContent value="images" className="space-y-4">
          {/* Filters */}
          <Card className="p-4">
            <div className="flex flex-wrap items-center gap-4">
              <div className="flex items-center space-x-2">
                <Search className="w-4 h-4 text-gray-500" />
                <Input
                  placeholder="Search images..."
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
                <option value="building">Building</option>
                <option value="built">Built</option>
                <option value="testing">Testing</option>
                <option value="signed">Signed</option>
                <option value="published">Published</option>
                <option value="deprecated">Deprecated</option>
                <option value="failed">Failed</option>
              </Select>

              <Input
                placeholder="Tag filter..."
                value={filters.tag}
                onChange={(e) => setFilters(prev => ({ ...prev, tag: e.target.value }))}
                className="w-32"
              />

              <Select
                value={filters.architecture}
                onValueChange={(value) => setFilters(prev => ({ ...prev, architecture: value }))}
              >
                <option value="">All Architectures</option>
                <option value="amd64">amd64</option>
                <option value="arm64">arm64</option>
                <option value="armv7">armv7</option>
              </Select>
            </div>
          </Card>

          {/* Images List */}
          <div className="space-y-4">
            {filteredImages.map(image => (
              <Card key={image.id} className="p-6 hover:shadow-md transition-shadow cursor-pointer"
                    onClick={() => handleImageSelect(image)}>
                <div className="flex items-start justify-between mb-4">
                  <div className="flex items-center space-x-3">
                    <div className={`p-2 rounded ${
                      image.status === 'signed' || image.status === 'published' ? 'bg-green-100' :
                      image.status === 'building' ? 'bg-blue-100' :
                      image.status === 'failed' ? 'bg-red-100' : 'bg-yellow-100'
                    }`}>
                      <HardDrive className={`w-5 h-5 ${
                        image.status === 'signed' || image.status === 'published' ? 'text-green-600' :
                        image.status === 'building' ? 'text-blue-600' :
                        image.status === 'failed' ? 'text-red-600' : 'text-yellow-600'
                      }`} />
                    </div>
                    <div>
                      <h3 className="text-lg font-medium text-gray-900">{image.name}</h3>
                      <p className="text-sm text-gray-600">Version: {image.version}</p>
                      <p className="text-sm text-gray-600">{image.description}</p>
                    </div>
                  </div>
                  <div className="flex items-center space-x-2">
                    <Badge className={getStatusColor(image.status)}>
                      {image.status}
                    </Badge>
                    {image.signature.signed && (
                      <Badge className="bg-green-100 text-green-800">
                        <Shield className="w-3 h-3 mr-1" />
                        Signed
                      </Badge>
                    )}
                    {image.checksum.verified && (
                      <Badge className="bg-blue-100 text-blue-800">
                        <CheckCircle className="w-3 h-3 mr-1" />
                        Verified
                      </Badge>
                    )}
                  </div>
                </div>

                <div className="grid grid-cols-2 md:grid-cols-4 gap-4 mb-4">
                  <div>
                    <div className="text-sm text-gray-600">Size</div>
                    <div className="font-medium text-gray-900">
                      {formatBytes(image.size)}
                    </div>
                  </div>
                  <div>
                    <div className="text-sm text-gray-600">Architecture</div>
                    <div className="font-medium text-gray-900">
                      {image.metadata.architecture}
                    </div>
                  </div>
                  <div>
                    <div className="text-sm text-gray-600">Deployed Nodes</div>
                    <div className="font-medium text-gray-900">
                      {image.deployedNodes} / {image.totalNodes}
                    </div>
                  </div>
                  <div>
                    <div className="text-sm text-gray-600">Vulnerabilities</div>
                    <div className={`font-medium ${
                      image.vulnerabilities.length > 0 ? 'text-red-600' : 'text-green-600'
                    }`}>
                      {image.vulnerabilities.length}
                    </div>
                  </div>
                </div>

                {/* Build Info */}
                <div className="grid grid-cols-2 gap-6 mb-4">
                  <div>
                    <div className="text-sm font-medium text-gray-900 mb-2">Build Information</div>
                    <div className="text-sm text-gray-600">
                      <span className="font-medium">Build:</span> #{image.buildInfo.buildNumber}
                    </div>
                    <div className="text-sm text-gray-600">
                      <span className="font-medium">Git:</span> {image.buildInfo.gitCommit.substring(0, 8)}
                      {image.buildInfo.gitTag && ` (${image.buildInfo.gitTag})`}
                    </div>
                    <div className="text-sm text-gray-600">
                      <span className="font-medium">Duration:</span> {Math.round(image.buildInfo.buildDuration / 60)}m
                    </div>
                  </div>
                  <div>
                    <div className="text-sm font-medium text-gray-900 mb-2">Security</div>
                    <div className="text-sm text-gray-600">
                      <span className="font-medium">Checksum:</span> {image.checksum.verified ? 'Verified' : 'Pending'}
                    </div>
                    <div className="text-sm text-gray-600">
                      <span className="font-medium">Signature:</span> {image.signature.signed ? 'Signed' : 'Unsigned'}
                    </div>
                    {image.approvedBy && (
                      <div className="text-sm text-gray-600">
                        <span className="font-medium">Approved by:</span> {image.approvedBy}
                      </div>
                    )}
                  </div>
                </div>

                {/* Tags */}
                <div className="mb-4">
                  <div className="text-sm font-medium text-gray-900 mb-2">Tags</div>
                  <div className="flex flex-wrap gap-1">
                    {image.tags.map(tag => (
                      <Badge key={tag} variant="outline" size="sm">
                        {tag}
                      </Badge>
                    ))}
                  </div>
                </div>

                {/* Deployment Progress (if applicable) */}
                {image.deploymentInfo.startedAt && !image.deploymentInfo.completedAt && (
                  <div className="mb-4">
                    <div className="text-sm font-medium text-gray-900 mb-2">Deployment Progress</div>
                    <div className="space-y-2">
                      {image.deploymentInfo.stages.map((stage, index) => (
                        <div key={stage.id} className="flex items-center space-x-3">
                          <div className={`w-2 h-2 rounded-full ${
                            stage.status === 'completed' ? 'bg-green-500' :
                            stage.status === 'in_progress' ? 'bg-blue-500' :
                            stage.status === 'failed' ? 'bg-red-500' : 'bg-gray-300'
                          }`} />
                          <span className="text-sm text-gray-600">{stage.name}</span>
                          <Badge className={
                            stage.status === 'completed' ? 'bg-green-100 text-green-800' :
                            stage.status === 'in_progress' ? 'bg-blue-100 text-blue-800' :
                            stage.status === 'failed' ? 'bg-red-100 text-red-800' :
                            'bg-gray-100 text-gray-800'
                          } size="sm">
                            {stage.status.replace('_', ' ')}
                          </Badge>
                        </div>
                      ))}
                    </div>
                  </div>
                )}

                <div className="flex items-center justify-between pt-4 border-t">
                  <div className="text-sm text-gray-600">
                    Created: {image.createdAt.toLocaleDateString()} • 
                    By: {image.createdBy}
                  </div>
                  <div className="flex items-center space-x-2">
                    {image.status === 'built' && !image.signature.signed && (
                      <Button 
                        variant="outline" 
                        size="sm"
                        onClick={(e) => {
                          e.stopPropagation()
                          handleSignImage(image.id)
                        }}
                      >
                        <Key className="w-4 h-4 mr-1" />
                        Sign Image
                      </Button>
                    )}
                    {(image.status === 'signed' || image.status === 'published') && (
                      <Button 
                        variant="outline" 
                        size="sm"
                        onClick={(e) => {
                          e.stopPropagation()
                          setSelectedImage(image)
                          setShowDeployDialog(true)
                        }}
                        disabled={deploymentInProgress === image.id}
                      >
                        <Play className="w-4 h-4 mr-1" />
                        Deploy
                      </Button>
                    )}
                    <Button variant="outline" size="sm">
                      <Eye className="w-4 h-4 mr-1" />
                      View Details
                    </Button>
                  </div>
                </div>
              </Card>
            ))}

            {filteredImages.length === 0 && (
              <div className="text-center py-12">
                <HardDrive className="w-16 h-16 text-gray-400 mx-auto mb-4" />
                <h3 className="text-lg font-medium text-gray-900 mb-2">No Images Found</h3>
                <p className="text-gray-600">
                  {filters.search || filters.status || filters.tag || filters.architecture
                    ? 'No images match your current filters'
                    : 'No OS images available'
                  }
                </p>
              </div>
            )}
          </div>
        </TabsContent>

        <TabsContent value="nodes" className="space-y-4">
          <div className="grid gap-4 md:grid-cols-2 lg:grid-cols-3">
            {nodes.map(node => (
              <Card key={node.id} className="p-4">
                <div className="flex items-start justify-between mb-3">
                  <div className="flex items-center space-x-3">
                    <div className={`p-2 rounded ${
                      node.status === 'online' ? 'bg-green-100' :
                      node.status === 'updating' ? 'bg-blue-100' :
                      node.status === 'failed' ? 'bg-red-100' : 'bg-gray-100'
                    }`}>
                      <Monitor className={`w-4 h-4 ${
                        node.status === 'online' ? 'text-green-600' :
                        node.status === 'updating' ? 'text-blue-600' :
                        node.status === 'failed' ? 'text-red-600' : 'text-gray-600'
                      }`} />
                    </div>
                    <div>
                      <h4 className="font-medium text-gray-900">{node.name}</h4>
                      <p className="text-sm text-gray-600">{node.location}</p>
                      <p className="text-sm text-gray-600 font-mono">{node.ipAddress}</p>
                    </div>
                  </div>
                  <Badge className={getNodeStatusColor(node.status)}>
                    {node.status}
                  </Badge>
                </div>

                <div className="space-y-2 text-sm">
                  <div className="flex justify-between">
                    <span className="text-gray-600">Current Image:</span>
                    <span className="font-medium font-mono text-xs">
                      {images.find(img => img.id === node.currentImage)?.version || 'Unknown'}
                    </span>
                  </div>
                  {node.targetImage && (
                    <div className="flex justify-between">
                      <span className="text-gray-600">Target Image:</span>
                      <span className="font-medium font-mono text-xs">
                        {images.find(img => img.id === node.targetImage)?.version || 'Unknown'}
                      </span>
                    </div>
                  )}
                  <div className="flex justify-between">
                    <span className="text-gray-600">Health:</span>
                    <Badge className={
                      node.healthStatus === 'healthy' ? 'bg-green-100 text-green-800' :
                      node.healthStatus === 'unhealthy' ? 'bg-red-100 text-red-800' :
                      'bg-yellow-100 text-yellow-800'
                    } size="sm">
                      {node.healthStatus}
                    </Badge>
                  </div>
                  <div className="flex justify-between">
                    <span className="text-gray-600">Last Seen:</span>
                    <span className="font-medium">{node.lastSeen.toLocaleTimeString()}</span>
                  </div>
                </div>

                {node.deploymentProgress !== undefined && (
                  <div className="mt-3">
                    <div className="flex items-center justify-between mb-1">
                      <span className="text-sm font-medium text-gray-900">Deployment Progress</span>
                      <span className="text-sm text-gray-600">{Math.round(node.deploymentProgress)}%</span>
                    </div>
                    <Progress value={node.deploymentProgress} className="w-full h-2" />
                  </div>
                )}

                <div className="mt-3 pt-3 border-t">
                  <div className="text-xs text-gray-500">
                    {node.systemInfo.cpu} • {node.systemInfo.memory} • {node.systemInfo.disk}
                  </div>
                </div>
              </Card>
            ))}
          </div>
        </TabsContent>

        <TabsContent value="deployments" className="space-y-4">
          <div className="text-center py-8">
            <Activity className="w-16 h-16 text-gray-400 mx-auto mb-4" />
            <h3 className="text-lg font-medium text-gray-900 mb-2">Active Deployments</h3>
            <p className="text-gray-600">
              Real-time deployment monitoring and control interface would be implemented here.
            </p>
          </div>
        </TabsContent>

        <TabsContent value="pipeline" className="space-y-4">
          <div className="text-center py-8">
            <Package className="w-16 h-16 text-gray-400 mx-auto mb-4" />
            <h3 className="text-lg font-medium text-gray-900 mb-2">Build Pipeline</h3>
            <p className="text-gray-600">
              CI/CD pipeline configuration and monitoring interface would be implemented here.
            </p>
          </div>
        </TabsContent>
      </Tabs>

      {/* Image Details Dialog */}
      <Dialog open={showImageDialog} onOpenChange={setShowImageDialog}>
        <DialogContent className="max-w-4xl max-h-[90vh] overflow-y-auto">
          <DialogHeader>
            <DialogTitle>Image Details: {selectedImage?.name} v{selectedImage?.version}</DialogTitle>
          </DialogHeader>

          {selectedImage && (
            <div className="space-y-6">
              <div className="grid grid-cols-2 gap-6">
                <div>
                  <h3 className="font-medium text-gray-900 mb-2">Image Information</h3>
                  <div className="space-y-1 text-sm">
                    <div><span className="font-medium">Size:</span> {formatBytes(selectedImage.size)}</div>
                    <div><span className="font-medium">Base Image:</span> {selectedImage.baseImage}</div>
                    <div><span className="font-medium">Architecture:</span> {selectedImage.metadata.architecture}</div>
                    <div><span className="font-medium">OS:</span> {selectedImage.metadata.osVersion}</div>
                  </div>
                </div>
                <div>
                  <h3 className="font-medium text-gray-900 mb-2">Security</h3>
                  <div className="space-y-1 text-sm">
                    <div>
                      <span className="font-medium">Checksum:</span> 
                      <Badge className={selectedImage.checksum.verified ? 'bg-green-100 text-green-800 ml-2' : 'bg-yellow-100 text-yellow-800 ml-2'} size="sm">
                        {selectedImage.checksum.verified ? 'Verified' : 'Pending'}
                      </Badge>
                    </div>
                    <div>
                      <span className="font-medium">Signature:</span> 
                      <Badge className={selectedImage.signature.signed ? 'bg-green-100 text-green-800 ml-2' : 'bg-red-100 text-red-800 ml-2'} size="sm">
                        {selectedImage.signature.signed ? 'Signed' : 'Unsigned'}
                      </Badge>
                    </div>
                    <div><span className="font-medium">Vulnerabilities:</span> {selectedImage.vulnerabilities.length}</div>
                  </div>
                </div>
              </div>

              <div>
                <h3 className="font-medium text-gray-900 mb-2">Checksums</h3>
                <div className="space-y-1 text-xs font-mono bg-gray-50 p-3 rounded">
                  <div><span className="font-medium">SHA256:</span> {selectedImage.checksum.sha256}</div>
                  <div><span className="font-medium">SHA512:</span> {selectedImage.checksum.sha512}</div>
                  <div><span className="font-medium">MD5:</span> {selectedImage.checksum.md5}</div>
                </div>
              </div>

              {selectedImage.vulnerabilities.length > 0 && (
                <div>
                  <h3 className="font-medium text-gray-900 mb-2">Vulnerabilities ({selectedImage.vulnerabilities.length})</h3>
                  <div className="space-y-2">
                    {selectedImage.vulnerabilities.map(vuln => (
                      <div key={vuln.id} className="p-3 border rounded">
                        <div className="flex items-center justify-between mb-1">
                          <span className="font-medium">{vuln.cveId || vuln.id}</span>
                          <Badge className={
                            vuln.severity === 'critical' ? 'bg-red-100 text-red-800' :
                            vuln.severity === 'high' ? 'bg-orange-100 text-orange-800' :
                            vuln.severity === 'medium' ? 'bg-yellow-100 text-yellow-800' :
                            'bg-green-100 text-green-800'
                          } size="sm">
                            {vuln.severity}
                          </Badge>
                        </div>
                        <div className="text-sm text-gray-600">{vuln.description}</div>
                        <div className="text-xs text-gray-500 mt-1">
                          Package: {vuln.package} {vuln.version}
                          {vuln.fixedVersion && ` → ${vuln.fixedVersion}`}
                          • CVSS: {vuln.cvssScore}
                        </div>
                      </div>
                    ))}
                  </div>
                </div>
              )}
            </div>
          )}

          <DialogFooter>
            <Button variant="outline" onClick={() => setShowImageDialog(false)}>
              Close
            </Button>
            {selectedImage?.status === 'built' && !selectedImage.signature.signed && (
              <Button onClick={() => selectedImage && handleSignImage(selectedImage.id)}>
                <Key className="w-4 h-4 mr-2" />
                Sign Image
              </Button>
            )}
          </DialogFooter>
        </DialogContent>
      </Dialog>

      {/* Deploy Dialog */}
      <Dialog open={showDeployDialog} onOpenChange={setShowDeployDialog}>
        <DialogContent className="max-w-2xl">
          <DialogHeader>
            <DialogTitle>Deploy Image: {selectedImage?.name} v{selectedImage?.version}</DialogTitle>
          </DialogHeader>

          <div className="space-y-4">
            <div>
              <Label>Target Nodes</Label>
              <div className="mt-2 space-y-2 max-h-60 overflow-y-auto">
                {nodes.map(node => (
                  <div key={node.id} className="flex items-center space-x-2 p-2 border rounded">
                    <Checkbox id={node.id} />
                    <Label htmlFor={node.id} className="flex-grow">
                      <div className="font-medium">{node.name}</div>
                      <div className="text-sm text-gray-600">{node.location}</div>
                    </Label>
                    <Badge className={getNodeStatusColor(node.status)} size="sm">
                      {node.status}
                    </Badge>
                  </div>
                ))}
              </div>
            </div>

            <Alert>
              <AlertTriangle className="h-4 w-4" />
              <AlertDescription>
                This will deploy the selected image to the chosen nodes. The deployment will be staged with health checks and rollback capabilities.
              </AlertDescription>
            </Alert>
          </div>

          <DialogFooter>
            <Button variant="outline" onClick={() => setShowDeployDialog(false)}>
              Cancel
            </Button>
            <Button 
              onClick={() => {
                if (selectedImage) {
                  const selectedNodes = nodes.filter(node => 
                    document.getElementById(node.id)?.checked
                  ).map(node => node.id)
                  
                  if (selectedNodes.length > 0) {
                    handleDeployImage(selectedImage.id, selectedNodes)
                    setShowDeployDialog(false)
                  }
                }
              }}
              disabled={deploymentInProgress !== null}
            >
              <Play className="w-4 h-4 mr-2" />
              Deploy Image
            </Button>
          </DialogFooter>
        </DialogContent>
      </Dialog>
    </div>
  )
}

export default GaragePCImagePipeline
