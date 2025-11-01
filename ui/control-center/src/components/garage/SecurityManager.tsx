import React, { useState, useCallback, useEffect, useMemo } from 'react'
import { 
  Shield, Lock, Unlock, AlertTriangle, CheckCircle, XCircle, Eye, EyeOff,
  Wifi, WifiOff, Usb, HardDrive, Network, Activity, Settings, RefreshCw,
  Server, Database, FileText, Download, Upload, Play, Pause, StopCircle,
  Users, Clock, MapPin, Zap, Target, BarChart3, TrendingUp, TrendingDown
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
interface SecurityPolicy {
  id: string
  name: string
  category: 'os_patching' | 'edr' | 'usb_lockdown' | 'network_segmentation' | 'access_control' | 'data_protection'
  status: 'active' | 'inactive' | 'pending' | 'failed'
  severity: 'low' | 'medium' | 'high' | 'critical'
  description: string
  rules: SecurityRule[]
  lastUpdated: Date
  nextCheck: Date
  violations: SecurityViolation[]
  compliance: ComplianceStatus
  remediation: RemediationAction[]
}

interface SecurityRule {
  id: string
  name: string
  condition: string
  action: 'allow' | 'deny' | 'monitor' | 'alert'
  enabled: boolean
  priority: number
  exceptions: string[]
}

interface SecurityViolation {
  id: string
  ruleId: string
  ruleName: string
  severity: 'low' | 'medium' | 'high' | 'critical'
  description: string
  nodeId: string
  nodeName: string
  detectedAt: Date
  status: 'open' | 'investigating' | 'resolved' | 'false_positive'
  evidence: string[]
  remediation?: string
}

interface ComplianceStatus {
  framework: string
  score: number
  requirements: ComplianceRequirement[]
  lastAssessment: Date
  nextAssessment: Date
}

interface ComplianceRequirement {
  id: string
  requirement: string
  status: 'compliant' | 'non_compliant' | 'partial' | 'not_applicable'
  evidence: string[]
  gap?: string
}

interface RemediationAction {
  id: string
  type: 'patch' | 'configuration' | 'policy_update' | 'manual_intervention'
  description: string
  priority: 'low' | 'medium' | 'high' | 'critical'
  status: 'pending' | 'in_progress' | 'completed' | 'failed'
  estimatedTime: number // minutes
  assignedTo?: string
  dueDate: Date
}

interface EdgeNodeSecurity {
  id: string
  name: string
  location: string
  status: 'secure' | 'warning' | 'critical' | 'offline'
  lastSecurityScan: Date
  securityScore: number
  patches: PatchStatus
  edr: EDRStatus
  usb: USBStatus
  network: NetworkSecurity
  accessControl: AccessControlStatus
  dataProtection: DataProtectionStatus
  threats: ThreatDetection[]
}

interface PatchStatus {
  totalPatches: number
  installedPatches: number
  pendingPatches: number
  criticalPatches: number
  lastPatchDate: Date
  nextPatchWindow: Date
  autoPatching: boolean
  patchCompliance: number
}

interface EDRStatus {
  agent: string
  version: string
  status: 'active' | 'inactive' | 'error'
  lastUpdate: Date
  threatsDetected: number
  threatsBlocked: number
  quarantinedFiles: number
  realTimeProtection: boolean
  behaviorAnalysis: boolean
  cloudConnectivity: boolean
}

interface USBStatus {
  policy: 'disabled' | 'read_only' | 'whitelist' | 'unrestricted'
  whitelistedDevices: string[]
  blockedAttempts: number
  lastAttempt?: Date
  autoMount: boolean
  encryption: boolean
  auditLog: boolean
}

interface NetworkSecurity {
  segmentation: 'isolated' | 'vlan' | 'none'
  firewall: FirewallStatus
  vpn: VPNStatus
  intrusion: IntrusionDetection
  trafficMonitoring: boolean
  dnsFiltering: boolean
}

interface FirewallStatus {
  enabled: boolean
  rules: number
  blockedConnections: number
  allowedConnections: number
  lastRuleUpdate: Date
}

interface VPNStatus {
  enabled: boolean
  connected: boolean
  endpoint: string
  protocol: string
  lastConnection: Date
  dataTransferred: number
}

interface IntrusionDetection {
  enabled: boolean
  alertsToday: number
  threatsBlocked: number
  lastSignatureUpdate: Date
  signatureVersion: string
}

interface AccessControlStatus {
  authentication: 'disabled' | 'password' | 'mfa' | 'certificate'
  authorization: 'none' | 'rbac' | 'abac'
  sessionTimeout: number
  activeSessions: number
  failedAttempts: number
  lastAccess: Date
}

interface DataProtectionStatus {
  encryption: EncryptionStatus
  backup: BackupStatus
  dataClassification: boolean
  dlp: DLPStatus
}

interface EncryptionStatus {
  diskEncryption: boolean
  networkEncryption: boolean
  dataAtRest: boolean
  dataInTransit: boolean
  keyManagement: string
}

interface BackupStatus {
  enabled: boolean
  lastBackup: Date
  backupSize: number
  retention: number
  offsite: boolean
  encrypted: boolean
}

interface DLPStatus {
  enabled: boolean
  policies: number
  violations: number
  lastScan: Date
}

interface ThreatDetection {
  id: string
  type: 'malware' | 'intrusion' | 'anomaly' | 'policy_violation'
  severity: 'low' | 'medium' | 'high' | 'critical'
  description: string
  source: string
  detectedAt: Date
  status: 'active' | 'contained' | 'resolved'
  actions: string[]
}

interface SecurityManagerProps {
  nodeId?: string
  nodeName?: string
  onPolicyUpdated?: (policyId: string) => void
  className?: string
}

const SecurityManager: React.FC<SecurityManagerProps> = ({
  nodeId,
  nodeName,
  onPolicyUpdated,
  className = ''
}) => {
  // State
  const [policies, setPolicies] = useState<SecurityPolicy[]>([])
  const [nodes, setNodes] = useState<EdgeNodeSecurity[]>([])
  const [showPolicyDialog, setShowPolicyDialog] = useState(false)
  const [showViolationDialog, setShowViolationDialog] = useState(false)
  const [selectedPolicy, setSelectedPolicy] = useState<SecurityPolicy | null>(null)
  const [selectedViolation, setSelectedViolation] = useState<SecurityViolation | null>(null)
  const [activeTab, setActiveTab] = useState('overview')
  const [securityScanning, setSecurityScanning] = useState(false)

  // Mock data initialization
  const initializeMockData = useCallback(() => {
    const mockPolicies: SecurityPolicy[] = [
      {
        id: 'policy-os-patching',
        name: 'OS Patching Policy',
        category: 'os_patching',
        status: 'active',
        severity: 'critical',
        description: 'Automated OS security patching with scheduled maintenance windows',
        rules: [
          {
            id: 'rule-001',
            name: 'Critical Patch Installation',
            condition: 'patch.severity == "critical" AND patch.age > 24h',
            action: 'allow',
            enabled: true,
            priority: 1,
            exceptions: []
          },
          {
            id: 'rule-002',
            name: 'Maintenance Window Enforcement',
            condition: 'patch.type == "non-critical" AND time NOT IN maintenance_window',
            action: 'deny',
            enabled: true,
            priority: 2,
            exceptions: ['emergency_override']
          }
        ],
        lastUpdated: new Date('2024-11-25'),
        nextCheck: new Date(Date.now() + 24 * 60 * 60 * 1000),
        violations: [],
        compliance: {
          framework: 'ISO 27001',
          score: 95,
          requirements: [
            {
              id: 'req-001',
              requirement: 'A.12.6.1 - Management of technical vulnerabilities',
              status: 'compliant',
              evidence: ['patch_logs', 'vulnerability_scans']
            }
          ],
          lastAssessment: new Date('2024-11-20'),
          nextAssessment: new Date(Date.now() + 90 * 24 * 60 * 60 * 1000)
        },
        remediation: []
      },
      {
        id: 'policy-edr',
        name: 'Endpoint Detection & Response',
        category: 'edr',
        status: 'active',
        severity: 'critical',
        description: 'Real-time threat detection and automated response system',
        rules: [
          {
            id: 'rule-003',
            name: 'Malware Detection',
            condition: 'file.signature IN malware_signatures',
            action: 'deny',
            enabled: true,
            priority: 1,
            exceptions: []
          },
          {
            id: 'rule-004',
            name: 'Behavioral Analysis',
            condition: 'process.behavior == "suspicious"',
            action: 'monitor',
            enabled: true,
            priority: 2,
            exceptions: ['whitelisted_processes']
          }
        ],
        lastUpdated: new Date('2024-11-24'),
        nextCheck: new Date(Date.now() + 4 * 60 * 60 * 1000),
        violations: [
          {
            id: 'viol-001',
            ruleId: 'rule-003',
            ruleName: 'Malware Detection',
            severity: 'high',
            description: 'Suspicious executable detected and quarantined',
            nodeId: 'node-002',
            nodeName: 'Depot-Edge-02',
            detectedAt: new Date(Date.now() - 2 * 60 * 60 * 1000),
            status: 'resolved',
            evidence: ['/var/log/edr/quarantine_20241126_001.log'],
            remediation: 'File quarantined and threat neutralized'
          }
        ],
        compliance: {
          framework: 'NIST Cybersecurity Framework',
          score: 88,
          requirements: [
            {
              id: 'req-002',
              requirement: 'DE.CM-1 - Detect cybersecurity events',
              status: 'compliant',
              evidence: ['edr_logs', 'threat_reports']
            }
          ],
          lastAssessment: new Date('2024-11-22'),
          nextAssessment: new Date(Date.now() + 30 * 24 * 60 * 60 * 1000)
        },
        remediation: []
      },
      {
        id: 'policy-usb-lockdown',
        name: 'USB Port Security',
        category: 'usb_lockdown',
        status: 'active',
        severity: 'high',
        description: 'USB port access control and device whitelisting',
        rules: [
          {
            id: 'rule-005',
            name: 'USB Device Whitelist',
            condition: 'usb.device_id NOT IN whitelist',
            action: 'deny',
            enabled: true,
            priority: 1,
            exceptions: ['emergency_access']
          },
          {
            id: 'rule-006',
            name: 'Auto-Mount Prevention',
            condition: 'usb.auto_mount == true',
            action: 'deny',
            enabled: true,
            priority: 2,
            exceptions: []
          }
        ],
        lastUpdated: new Date('2024-11-23'),
        nextCheck: new Date(Date.now() + 12 * 60 * 60 * 1000),
        violations: [
          {
            id: 'viol-002',
            ruleId: 'rule-005',
            ruleName: 'USB Device Whitelist',
            severity: 'medium',
            description: 'Unauthorized USB device connection attempt',
            nodeId: 'node-001',
            nodeName: 'Depot-Edge-01',
            detectedAt: new Date(Date.now() - 6 * 60 * 60 * 1000),
            status: 'investigating',
            evidence: ['/var/log/usb/blocked_device_20241126_002.log']
          }
        ],
        compliance: {
          framework: 'ISO 27001',
          score: 92,
          requirements: [
            {
              id: 'req-003',
              requirement: 'A.11.2.6 - Secure disposal or reuse of equipment',
              status: 'compliant',
              evidence: ['usb_access_logs', 'device_whitelist']
            }
          ],
          lastAssessment: new Date('2024-11-20'),
          nextAssessment: new Date(Date.now() + 60 * 24 * 60 * 60 * 1000)
        },
        remediation: [
          {
            id: 'rem-001',
            type: 'manual_intervention',
            description: 'Review and update USB device whitelist',
            priority: 'medium',
            status: 'pending',
            estimatedTime: 30,
            dueDate: new Date(Date.now() + 48 * 60 * 60 * 1000)
          }
        ]
      },
      {
        id: 'policy-network-seg',
        name: 'Network Segmentation',
        category: 'network_segmentation',
        status: 'active',
        severity: 'high',
        description: 'Network isolation and micro-segmentation policies',
        rules: [
          {
            id: 'rule-007',
            name: 'VLAN Isolation',
            condition: 'network.vlan != "edge_operations"',
            action: 'deny',
            enabled: true,
            priority: 1,
            exceptions: ['management_network']
          },
          {
            id: 'rule-008',
            name: 'Lateral Movement Prevention',
            condition: 'connection.source_vlan != connection.dest_vlan',
            action: 'monitor',
            enabled: true,
            priority: 2,
            exceptions: ['approved_cross_vlan']
          }
        ],
        lastUpdated: new Date('2024-11-22'),
        nextCheck: new Date(Date.now() + 6 * 60 * 60 * 1000),
        violations: [],
        compliance: {
          framework: 'PCI DSS',
          score: 94,
          requirements: [
            {
              id: 'req-004',
              requirement: '1.3 - Prohibit direct public access between Internet and cardholder data environment',
              status: 'compliant',
              evidence: ['firewall_rules', 'network_topology']
            }
          ],
          lastAssessment: new Date('2024-11-18'),
          nextAssessment: new Date(Date.now() + 90 * 24 * 60 * 60 * 1000)
        },
        remediation: []
      }
    ]

    const mockNodes: EdgeNodeSecurity[] = [
      {
        id: 'node-001',
        name: 'Depot-Edge-01',
        location: 'Main Depot - Bay 1',
        status: 'secure',
        lastSecurityScan: new Date(Date.now() - 2 * 60 * 60 * 1000),
        securityScore: 94,
        patches: {
          totalPatches: 145,
          installedPatches: 142,
          pendingPatches: 3,
          criticalPatches: 0,
          lastPatchDate: new Date('2024-11-24'),
          nextPatchWindow: new Date(Date.now() + 48 * 60 * 60 * 1000),
          autoPatching: true,
          patchCompliance: 97.9
        },
        edr: {
          agent: 'CrowdStrike Falcon',
          version: '7.15.18203',
          status: 'active',
          lastUpdate: new Date(Date.now() - 30 * 60 * 1000),
          threatsDetected: 12,
          threatsBlocked: 12,
          quarantinedFiles: 3,
          realTimeProtection: true,
          behaviorAnalysis: true,
          cloudConnectivity: true
        },
        usb: {
          policy: 'whitelist',
          whitelistedDevices: ['VID_0781&PID_5567', 'VID_058F&PID_6387'],
          blockedAttempts: 2,
          lastAttempt: new Date(Date.now() - 6 * 60 * 60 * 1000),
          autoMount: false,
          encryption: true,
          auditLog: true
        },
        network: {
          segmentation: 'vlan',
          firewall: {
            enabled: true,
            rules: 47,
            blockedConnections: 156,
            allowedConnections: 8924,
            lastRuleUpdate: new Date('2024-11-23')
          },
          vpn: {
            enabled: true,
            connected: true,
            endpoint: 'vpn.atlasmesh.internal',
            protocol: 'WireGuard',
            lastConnection: new Date(Date.now() - 15 * 60 * 1000),
            dataTransferred: 2.4
          },
          intrusion: {
            enabled: true,
            alertsToday: 3,
            threatsBlocked: 1,
            lastSignatureUpdate: new Date(Date.now() - 4 * 60 * 60 * 1000),
            signatureVersion: '2024.11.26.001'
          },
          trafficMonitoring: true,
          dnsFiltering: true
        },
        accessControl: {
          authentication: 'certificate',
          authorization: 'rbac',
          sessionTimeout: 480,
          activeSessions: 2,
          failedAttempts: 0,
          lastAccess: new Date(Date.now() - 45 * 60 * 1000)
        },
        dataProtection: {
          encryption: {
            diskEncryption: true,
            networkEncryption: true,
            dataAtRest: true,
            dataInTransit: true,
            keyManagement: 'HashiCorp Vault'
          },
          backup: {
            enabled: true,
            lastBackup: new Date(Date.now() - 6 * 60 * 60 * 1000),
            backupSize: 15.7,
            retention: 30,
            offsite: true,
            encrypted: true
          },
          dataClassification: true,
          dlp: {
            enabled: true,
            policies: 8,
            violations: 0,
            lastScan: new Date(Date.now() - 2 * 60 * 60 * 1000)
          }
        },
        threats: []
      },
      {
        id: 'node-002',
        name: 'Depot-Edge-02',
        location: 'Main Depot - Bay 2',
        status: 'warning',
        lastSecurityScan: new Date(Date.now() - 30 * 60 * 1000),
        securityScore: 78,
        patches: {
          totalPatches: 145,
          installedPatches: 138,
          pendingPatches: 7,
          criticalPatches: 2,
          lastPatchDate: new Date('2024-11-20'),
          nextPatchWindow: new Date(Date.now() + 12 * 60 * 60 * 1000),
          autoPatching: false,
          patchCompliance: 95.2
        },
        edr: {
          agent: 'CrowdStrike Falcon',
          version: '7.15.18203',
          status: 'active',
          lastUpdate: new Date(Date.now() - 45 * 60 * 1000),
          threatsDetected: 18,
          threatsBlocked: 17,
          quarantinedFiles: 5,
          realTimeProtection: true,
          behaviorAnalysis: true,
          cloudConnectivity: true
        },
        usb: {
          policy: 'disabled',
          whitelistedDevices: [],
          blockedAttempts: 8,
          lastAttempt: new Date(Date.now() - 2 * 60 * 60 * 1000),
          autoMount: false,
          encryption: true,
          auditLog: true
        },
        network: {
          segmentation: 'vlan',
          firewall: {
            enabled: true,
            rules: 52,
            blockedConnections: 234,
            allowedConnections: 7845,
            lastRuleUpdate: new Date('2024-11-22')
          },
          vpn: {
            enabled: true,
            connected: false,
            endpoint: 'vpn.atlasmesh.internal',
            protocol: 'WireGuard',
            lastConnection: new Date(Date.now() - 2 * 60 * 60 * 1000),
            dataTransferred: 1.8
          },
          intrusion: {
            enabled: true,
            alertsToday: 7,
            threatsBlocked: 3,
            lastSignatureUpdate: new Date(Date.now() - 4 * 60 * 60 * 1000),
            signatureVersion: '2024.11.26.001'
          },
          trafficMonitoring: true,
          dnsFiltering: true
        },
        accessControl: {
          authentication: 'mfa',
          authorization: 'rbac',
          sessionTimeout: 480,
          activeSessions: 1,
          failedAttempts: 3,
          lastAccess: new Date(Date.now() - 2 * 60 * 60 * 1000)
        },
        dataProtection: {
          encryption: {
            diskEncryption: true,
            networkEncryption: true,
            dataAtRest: true,
            dataInTransit: true,
            keyManagement: 'HashiCorp Vault'
          },
          backup: {
            enabled: true,
            lastBackup: new Date(Date.now() - 12 * 60 * 60 * 1000),
            backupSize: 18.2,
            retention: 30,
            offsite: true,
            encrypted: true
          },
          dataClassification: true,
          dlp: {
            enabled: true,
            policies: 8,
            violations: 2,
            lastScan: new Date(Date.now() - 4 * 60 * 60 * 1000)
          }
        },
        threats: [
          {
            id: 'threat-001',
            type: 'malware',
            severity: 'high',
            description: 'Suspicious executable detected in /tmp directory',
            source: 'EDR Agent',
            detectedAt: new Date(Date.now() - 2 * 60 * 60 * 1000),
            status: 'resolved',
            actions: ['quarantine', 'scan', 'notify_admin']
          }
        ]
      }
    ]

    setPolicies(mockPolicies)
    setNodes(mockNodes)
  }, [])

  useEffect(() => {
    initializeMockData()
  }, [initializeMockData])

  // Real-time updates
  useEffect(() => {
    const interval = setInterval(() => {
      // Update node security metrics
      setNodes(prev => prev.map(node => ({
        ...node,
        edr: {
          ...node.edr,
          lastUpdate: new Date(),
          threatsDetected: node.edr.threatsDetected + (Math.random() > 0.95 ? 1 : 0),
          threatsBlocked: node.edr.threatsBlocked + (Math.random() > 0.98 ? 1 : 0)
        },
        network: {
          ...node.network,
          firewall: {
            ...node.network.firewall,
            blockedConnections: node.network.firewall.blockedConnections + Math.floor(Math.random() * 3),
            allowedConnections: node.network.firewall.allowedConnections + Math.floor(Math.random() * 10)
          },
          intrusion: {
            ...node.network.intrusion,
            alertsToday: node.network.intrusion.alertsToday + (Math.random() > 0.97 ? 1 : 0)
          }
        },
        usb: {
          ...node.usb,
          blockedAttempts: node.usb.blockedAttempts + (Math.random() > 0.99 ? 1 : 0)
        }
      })))
    }, 5000)

    return () => clearInterval(interval)
  }, [])

  // Handlers
  const handleRunSecurityScan = useCallback(async () => {
    setSecurityScanning(true)
    
    // Simulate security scan
    setTimeout(() => {
      setNodes(prev => prev.map(node => ({
        ...node,
        lastSecurityScan: new Date(),
        securityScore: Math.max(70, Math.min(100, node.securityScore + (Math.random() - 0.5) * 10))
      })))
      setSecurityScanning(false)
    }, 3000)
  }, [])

  const handlePolicyToggle = useCallback((policyId: string, enabled: boolean) => {
    setPolicies(prev => prev.map(policy => 
      policy.id === policyId 
        ? { ...policy, status: enabled ? 'active' : 'inactive', lastUpdated: new Date() }
        : policy
    ))
    onPolicyUpdated?.(policyId)
  }, [onPolicyUpdated])

  const handleViolationResolve = useCallback((violationId: string) => {
    setPolicies(prev => prev.map(policy => ({
      ...policy,
      violations: policy.violations.map(violation =>
        violation.id === violationId
          ? { ...violation, status: 'resolved' as const }
          : violation
      )
    })))
  }, [])

  // Computed values
  const stats = useMemo(() => {
    const filteredNodes = nodeId ? nodes.filter(node => node.id === nodeId) : nodes
    const filteredPolicies = policies

    const totalNodes = filteredNodes.length
    const secureNodes = filteredNodes.filter(node => node.status === 'secure').length
    const warningNodes = filteredNodes.filter(node => node.status === 'warning').length
    const criticalNodes = filteredNodes.filter(node => node.status === 'critical').length

    const totalPolicies = filteredPolicies.length
    const activePolicies = filteredPolicies.filter(policy => policy.status === 'active').length
    const totalViolations = filteredPolicies.reduce((sum, policy) => sum + policy.violations.length, 0)
    const openViolations = filteredPolicies.reduce((sum, policy) => 
      sum + policy.violations.filter(v => v.status === 'open' || v.status === 'investigating').length, 0
    )

    const avgSecurityScore = filteredNodes.length > 0 ? 
      filteredNodes.reduce((sum, node) => sum + node.securityScore, 0) / filteredNodes.length : 0

    const totalThreats = filteredNodes.reduce((sum, node) => sum + node.threats.length, 0)
    const activeThreats = filteredNodes.reduce((sum, node) => 
      sum + node.threats.filter(t => t.status === 'active').length, 0
    )

    return {
      totalNodes,
      secureNodes,
      warningNodes,
      criticalNodes,
      totalPolicies,
      activePolicies,
      totalViolations,
      openViolations,
      avgSecurityScore,
      totalThreats,
      activeThreats
    }
  }, [nodes, policies, nodeId])

  const getStatusColor = (status: string) => {
    switch (status) {
      case 'secure':
      case 'active':
      case 'compliant': return 'bg-green-100 text-green-800'
      case 'warning':
      case 'partial': return 'bg-yellow-100 text-yellow-800'
      case 'critical':
      case 'non_compliant':
      case 'failed': return 'bg-red-100 text-red-800'
      case 'inactive':
      case 'offline': return 'bg-gray-100 text-gray-800'
      default: return 'bg-blue-100 text-blue-800'
    }
  }

  const getSeverityColor = (severity: string) => {
    switch (severity) {
      case 'critical': return 'bg-red-100 text-red-800'
      case 'high': return 'bg-orange-100 text-orange-800'
      case 'medium': return 'bg-yellow-100 text-yellow-800'
      case 'low': return 'bg-blue-100 text-blue-800'
      default: return 'bg-gray-100 text-gray-800'
    }
  }

  const getSecurityScoreColor = (score: number) => {
    if (score >= 90) return 'text-green-600'
    if (score >= 75) return 'text-yellow-600'
    if (score >= 60) return 'text-orange-600'
    return 'text-red-600'
  }

  return (
    <div className={`space-y-6 ${className}`}>
      {/* Header */}
      <div className="flex items-center justify-between">
        <div className="flex items-center space-x-3">
          <Shield className="w-6 h-6 text-blue-600" />
          <h2 className="text-2xl font-semibold text-gray-900">
            Garage PC Security
            {nodeId && <span className="text-lg font-normal text-gray-600 ml-2">- {nodeName}</span>}
          </h2>
        </div>
        <div className="flex items-center space-x-2">
          <Button variant="outline" onClick={initializeMockData}>
            <RefreshCw className="w-4 h-4 mr-2" />
            Refresh
          </Button>
          <Button onClick={handleRunSecurityScan} disabled={securityScanning}>
            {securityScanning ? (
              <RefreshCw className="w-4 h-4 mr-2 animate-spin" />
            ) : (
              <Shield className="w-4 h-4 mr-2" />
            )}
            Security Scan
          </Button>
        </div>
      </div>

      {/* Statistics */}
      <div className="grid grid-cols-2 md:grid-cols-4 lg:grid-cols-7 gap-4">
        <Card className="p-4">
          <div className="text-center">
            <div className="text-2xl font-bold text-green-600">{stats.secureNodes}</div>
            <div className="text-sm text-gray-600">Secure Nodes</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className={`text-2xl font-bold ${stats.warningNodes > 0 ? 'text-yellow-600' : 'text-green-600'}`}>
              {stats.warningNodes}
            </div>
            <div className="text-sm text-gray-600">Warnings</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className={`text-2xl font-bold ${stats.criticalNodes > 0 ? 'text-red-600' : 'text-green-600'}`}>
              {stats.criticalNodes}
            </div>
            <div className="text-sm text-gray-600">Critical</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className="text-2xl font-bold text-blue-600">{stats.activePolicies}</div>
            <div className="text-sm text-gray-600">Active Policies</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className={`text-2xl font-bold ${stats.openViolations > 0 ? 'text-red-600' : 'text-green-600'}`}>
              {stats.openViolations}
            </div>
            <div className="text-sm text-gray-600">Open Violations</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className={`text-2xl font-bold ${stats.activeThreats > 0 ? 'text-red-600' : 'text-green-600'}`}>
              {stats.activeThreats}
            </div>
            <div className="text-sm text-gray-600">Active Threats</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className={`text-2xl font-bold ${getSecurityScoreColor(stats.avgSecurityScore)}`}>
              {Math.round(stats.avgSecurityScore)}
            </div>
            <div className="text-sm text-gray-600">Avg Score</div>
          </div>
        </Card>
      </div>

      {/* Critical Alerts */}
      {(stats.criticalNodes > 0 || stats.activeThreats > 0) && (
        <Alert className="border-red-200 bg-red-50">
          <AlertTriangle className="w-4 h-4 text-red-600" />
          <AlertDescription className="text-red-800">
            <strong>Security Alert:</strong> {stats.criticalNodes} node(s) in critical state, {stats.activeThreats} active threat(s) detected. Immediate attention required.
          </AlertDescription>
        </Alert>
      )}

      {/* Main Content */}
      <Tabs value={activeTab} onValueChange={setActiveTab}>
        <TabsList>
          <TabsTrigger value="overview">Overview</TabsTrigger>
          <TabsTrigger value="policies">Security Policies</TabsTrigger>
          <TabsTrigger value="nodes">Node Security</TabsTrigger>
          <TabsTrigger value="violations">Violations</TabsTrigger>
          <TabsTrigger value="compliance">Compliance</TabsTrigger>
        </TabsList>

        <TabsContent value="overview" className="space-y-6">
          <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-6">
            {/* OS Patching */}
            <Card className="p-6">
              <div className="flex items-center justify-between mb-4">
                <h3 className="text-lg font-medium text-gray-900">OS Patching</h3>
                <Badge className="bg-green-100 text-green-800">Active</Badge>
              </div>
              <div className="space-y-3">
                <div className="flex items-center justify-between">
                  <span className="text-sm text-gray-600">Patch Compliance</span>
                  <span className="font-medium">97.2%</span>
                </div>
                <Progress value={97.2} className="h-2" />
                <div className="grid grid-cols-2 gap-4 text-sm">
                  <div>
                    <div className="text-gray-600">Installed</div>
                    <div className="font-medium">280/287</div>
                  </div>
                  <div>
                    <div className="text-gray-600">Critical Pending</div>
                    <div className={`font-medium ${2 > 0 ? 'text-red-600' : 'text-green-600'}`}>2</div>
                  </div>
                </div>
              </div>
            </Card>

            {/* EDR Status */}
            <Card className="p-6">
              <div className="flex items-center justify-between mb-4">
                <h3 className="text-lg font-medium text-gray-900">Endpoint Protection</h3>
                <Badge className="bg-green-100 text-green-800">Active</Badge>
              </div>
              <div className="space-y-3">
                <div className="flex items-center justify-between">
                  <span className="text-sm text-gray-600">Threats Blocked</span>
                  <span className="font-medium text-green-600">29/30</span>
                </div>
                <Progress value={96.7} className="h-2" />
                <div className="grid grid-cols-2 gap-4 text-sm">
                  <div>
                    <div className="text-gray-600">Quarantined</div>
                    <div className="font-medium">8</div>
                  </div>
                  <div>
                    <div className="text-gray-600">Real-time</div>
                    <div className="font-medium text-green-600">Enabled</div>
                  </div>
                </div>
              </div>
            </Card>

            {/* Network Security */}
            <Card className="p-6">
              <div className="flex items-center justify-between mb-4">
                <h3 className="text-lg font-medium text-gray-900">Network Security</h3>
                <Badge className="bg-green-100 text-green-800">Protected</Badge>
              </div>
              <div className="space-y-3">
                <div className="flex items-center justify-between">
                  <span className="text-sm text-gray-600">Connections Blocked</span>
                  <span className="font-medium">390</span>
                </div>
                <div className="grid grid-cols-2 gap-4 text-sm">
                  <div>
                    <div className="text-gray-600">Firewall Rules</div>
                    <div className="font-medium">99</div>
                  </div>
                  <div>
                    <div className="text-gray-600">IDS Alerts</div>
                    <div className="font-medium">10</div>
                  </div>
                </div>
              </div>
            </Card>

            {/* USB Security */}
            <Card className="p-6">
              <div className="flex items-center justify-between mb-4">
                <h3 className="text-lg font-medium text-gray-900">USB Security</h3>
                <Badge className="bg-blue-100 text-blue-800">Restricted</Badge>
              </div>
              <div className="space-y-3">
                <div className="flex items-center justify-between">
                  <span className="text-sm text-gray-600">Policy</span>
                  <span className="font-medium">Whitelist Only</span>
                </div>
                <div className="grid grid-cols-2 gap-4 text-sm">
                  <div>
                    <div className="text-gray-600">Blocked Attempts</div>
                    <div className="font-medium">10</div>
                  </div>
                  <div>
                    <div className="text-gray-600">Whitelisted</div>
                    <div className="font-medium">2</div>
                  </div>
                </div>
              </div>
            </Card>

            {/* Access Control */}
            <Card className="p-6">
              <div className="flex items-center justify-between mb-4">
                <h3 className="text-lg font-medium text-gray-900">Access Control</h3>
                <Badge className="bg-green-100 text-green-800">Secure</Badge>
              </div>
              <div className="space-y-3">
                <div className="flex items-center justify-between">
                  <span className="text-sm text-gray-600">Authentication</span>
                  <span className="font-medium">Certificate + MFA</span>
                </div>
                <div className="grid grid-cols-2 gap-4 text-sm">
                  <div>
                    <div className="text-gray-600">Active Sessions</div>
                    <div className="font-medium">3</div>
                  </div>
                  <div>
                    <div className="text-gray-600">Failed Attempts</div>
                    <div className="font-medium">3</div>
                  </div>
                </div>
              </div>
            </Card>

            {/* Data Protection */}
            <Card className="p-6">
              <div className="flex items-center justify-between mb-4">
                <h3 className="text-lg font-medium text-gray-900">Data Protection</h3>
                <Badge className="bg-green-100 text-green-800">Encrypted</Badge>
              </div>
              <div className="space-y-3">
                <div className="flex items-center justify-between">
                  <span className="text-sm text-gray-600">Encryption</span>
                  <span className="font-medium text-green-600">Full</span>
                </div>
                <div className="grid grid-cols-2 gap-4 text-sm">
                  <div>
                    <div className="text-gray-600">Last Backup</div>
                    <div className="font-medium">6h ago</div>
                  </div>
                  <div>
                    <div className="text-gray-600">DLP Violations</div>
                    <div className="font-medium">2</div>
                  </div>
                </div>
              </div>
            </Card>
          </div>
        </TabsContent>

        <TabsContent value="policies" className="space-y-4">
          <div className="space-y-6">
            {policies.map(policy => (
              <Card key={policy.id} className="p-6">
                <div className="flex items-start justify-between mb-4">
                  <div className="flex items-center space-x-3">
                    <div className="p-2 bg-blue-100 rounded">
                      {policy.category === 'os_patching' && <Download className="w-5 h-5 text-blue-600" />}
                      {policy.category === 'edr' && <Shield className="w-5 h-5 text-blue-600" />}
                      {policy.category === 'usb_lockdown' && <Usb className="w-5 h-5 text-blue-600" />}
                      {policy.category === 'network_segmentation' && <Network className="w-5 h-5 text-blue-600" />}
                      {policy.category === 'access_control' && <Lock className="w-5 h-5 text-blue-600" />}
                      {policy.category === 'data_protection' && <Database className="w-5 h-5 text-blue-600" />}
                    </div>
                    <div>
                      <h3 className="text-lg font-medium text-gray-900">{policy.name}</h3>
                      <p className="text-sm text-gray-600">{policy.description}</p>
                    </div>
                  </div>
                  <div className="flex items-center space-x-2">
                    <Badge className={getStatusColor(policy.status)}>
                      {policy.status}
                    </Badge>
                    <Badge className={getSeverityColor(policy.severity)}>
                      {policy.severity}
                    </Badge>
                    <Switch
                      checked={policy.status === 'active'}
                      onCheckedChange={(checked) => handlePolicyToggle(policy.id, checked)}
                    />
                  </div>
                </div>

                <div className="grid grid-cols-2 md:grid-cols-4 gap-4 mb-4">
                  <div>
                    <div className="text-sm text-gray-600">Rules</div>
                    <div className="font-medium text-gray-900">{policy.rules.length}</div>
                  </div>
                  <div>
                    <div className="text-sm text-gray-600">Violations</div>
                    <div className={`font-medium ${policy.violations.length > 0 ? 'text-red-600' : 'text-green-600'}`}>
                      {policy.violations.length}
                    </div>
                  </div>
                  <div>
                    <div className="text-sm text-gray-600">Compliance Score</div>
                    <div className={`font-medium ${getSecurityScoreColor(policy.compliance.score)}`}>
                      {policy.compliance.score}%
                    </div>
                  </div>
                  <div>
                    <div className="text-sm text-gray-600">Last Updated</div>
                    <div className="font-medium text-gray-900">{policy.lastUpdated.toLocaleDateString()}</div>
                  </div>
                </div>

                {/* Rules Summary */}
                <div className="mb-4">
                  <div className="text-sm font-medium text-gray-900 mb-2">Active Rules</div>
                  <div className="space-y-1">
                    {policy.rules.slice(0, 2).map(rule => (
                      <div key={rule.id} className="flex items-center justify-between p-2 bg-gray-50 rounded">
                        <div className="flex items-center space-x-2">
                          <div className={`w-2 h-2 rounded-full ${rule.enabled ? 'bg-green-500' : 'bg-gray-400'}`} />
                          <span className="text-sm font-medium">{rule.name}</span>
                          <Badge variant="outline" size="sm" className={
                            rule.action === 'deny' ? 'text-red-700' :
                            rule.action === 'allow' ? 'text-green-700' :
                            rule.action === 'monitor' ? 'text-blue-700' :
                            'text-orange-700'
                          }>
                            {rule.action}
                          </Badge>
                        </div>
                        <div className="text-xs text-gray-500">Priority: {rule.priority}</div>
                      </div>
                    ))}
                    {policy.rules.length > 2 && (
                      <div className="text-xs text-gray-500 text-center py-1">
                        +{policy.rules.length - 2} more rules
                      </div>
                    )}
                  </div>
                </div>

                {/* Violations */}
                {policy.violations.length > 0 && (
                  <div className="mb-4">
                    <div className="text-sm font-medium text-gray-900 mb-2">Recent Violations</div>
                    <div className="space-y-2">
                      {policy.violations.slice(0, 2).map(violation => (
                        <div key={violation.id} className="p-2 bg-red-50 rounded border border-red-200">
                          <div className="flex items-center justify-between mb-1">
                            <div className="flex items-center space-x-2">
                              <Badge className={getSeverityColor(violation.severity)} size="sm">
                                {violation.severity}
                              </Badge>
                              <span className="text-sm font-medium">{violation.ruleName}</span>
                            </div>
                            <Badge className={getStatusColor(violation.status)} size="sm">
                              {violation.status.replace('_', ' ')}
                            </Badge>
                          </div>
                          <div className="text-xs text-gray-600">
                            {violation.nodeName} • {violation.detectedAt.toLocaleString()}
                          </div>
                        </div>
                      ))}
                    </div>
                  </div>
                )}

                {/* Remediation Actions */}
                {policy.remediation.length > 0 && (
                  <div className="mb-4">
                    <div className="text-sm font-medium text-gray-900 mb-2">Pending Actions</div>
                    <div className="space-y-1">
                      {policy.remediation.map(action => (
                        <div key={action.id} className="flex items-center justify-between p-2 bg-yellow-50 rounded">
                          <div>
                            <div className="text-sm font-medium">{action.description}</div>
                            <div className="text-xs text-gray-600">
                              {action.type.replace('_', ' ')} • Due: {action.dueDate.toLocaleDateString()}
                            </div>
                          </div>
                          <Badge className={getSeverityColor(action.priority)} size="sm">
                            {action.priority}
                          </Badge>
                        </div>
                      ))}
                    </div>
                  </div>
                )}

                <div className="flex items-center justify-between pt-4 border-t">
                  <div className="text-sm text-gray-600">
                    Next check: {policy.nextCheck.toLocaleString()}
                  </div>
                  <div className="flex items-center space-x-2">
                    <Button
                      variant="outline"
                      size="sm"
                      onClick={() => {
                        setSelectedPolicy(policy)
                        setShowPolicyDialog(true)
                      }}
                    >
                      <Eye className="w-4 h-4 mr-1" />
                      Details
                    </Button>
                    <Button variant="outline" size="sm">
                      <Settings className="w-4 h-4 mr-1" />
                      Configure
                    </Button>
                  </div>
                </div>
              </Card>
            ))}
          </div>
        </TabsContent>

        <TabsContent value="nodes" className="space-y-4">
          <div className="space-y-6">
            {nodes
              .filter(node => !nodeId || node.id === nodeId)
              .map(node => (
              <Card key={node.id} className="p-6">
                <div className="flex items-start justify-between mb-6">
                  <div>
                    <h3 className="text-lg font-medium text-gray-900">{node.name}</h3>
                    <p className="text-sm text-gray-600">{node.location}</p>
                    <div className="text-xs text-gray-500 mt-1">
                      Last scan: {node.lastSecurityScan.toLocaleString()}
                    </div>
                  </div>
                  <div className="flex items-center space-x-3">
                    <Badge className={getStatusColor(node.status)}>
                      {node.status}
                    </Badge>
                    <div className="text-right">
                      <div className={`text-2xl font-bold ${getSecurityScoreColor(node.securityScore)}`}>
                        {node.securityScore}
                      </div>
                      <div className="text-xs text-gray-600">Security Score</div>
                    </div>
                  </div>
                </div>

                <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-6">
                  {/* Patches */}
                  <div className="space-y-3">
                    <h4 className="text-sm font-medium text-gray-900 flex items-center">
                      <Download className="w-4 h-4 mr-2 text-blue-600" />
                      OS Patches
                    </h4>
                    <div className="space-y-2 text-sm">
                      <div className="flex items-center justify-between">
                        <span className="text-gray-600">Compliance</span>
                        <span className="font-medium">{node.patches.patchCompliance.toFixed(1)}%</span>
                      </div>
                      <Progress value={node.patches.patchCompliance} className="h-1" />
                      <div className="grid grid-cols-2 gap-2 text-xs">
                        <div>
                          <span className="text-gray-500">Installed:</span>
                          <span className="ml-1 font-medium">{node.patches.installedPatches}</span>
                        </div>
                        <div>
                          <span className="text-gray-500">Pending:</span>
                          <span className={`ml-1 font-medium ${node.patches.pendingPatches > 0 ? 'text-yellow-600' : 'text-green-600'}`}>
                            {node.patches.pendingPatches}
                          </span>
                        </div>
                        <div>
                          <span className="text-gray-500">Critical:</span>
                          <span className={`ml-1 font-medium ${node.patches.criticalPatches > 0 ? 'text-red-600' : 'text-green-600'}`}>
                            {node.patches.criticalPatches}
                          </span>
                        </div>
                        <div>
                          <span className="text-gray-500">Auto:</span>
                          <span className={`ml-1 font-medium ${node.patches.autoPatching ? 'text-green-600' : 'text-gray-600'}`}>
                            {node.patches.autoPatching ? 'Yes' : 'No'}
                          </span>
                        </div>
                      </div>
                    </div>
                  </div>

                  {/* EDR */}
                  <div className="space-y-3">
                    <h4 className="text-sm font-medium text-gray-900 flex items-center">
                      <Shield className="w-4 h-4 mr-2 text-green-600" />
                      Endpoint Protection
                    </h4>
                    <div className="space-y-2 text-sm">
                      <div className="flex items-center justify-between">
                        <span className="text-gray-600">Agent</span>
                        <Badge className={getStatusColor(node.edr.status)} size="sm">
                          {node.edr.status}
                        </Badge>
                      </div>
                      <div className="text-xs text-gray-600">
                        {node.edr.agent} v{node.edr.version}
                      </div>
                      <div className="grid grid-cols-2 gap-2 text-xs">
                        <div>
                          <span className="text-gray-500">Detected:</span>
                          <span className="ml-1 font-medium">{node.edr.threatsDetected}</span>
                        </div>
                        <div>
                          <span className="text-gray-500">Blocked:</span>
                          <span className="ml-1 font-medium text-green-600">{node.edr.threatsBlocked}</span>
                        </div>
                        <div>
                          <span className="text-gray-500">Quarantined:</span>
                          <span className="ml-1 font-medium">{node.edr.quarantinedFiles}</span>
                        </div>
                        <div>
                          <span className="text-gray-500">Real-time:</span>
                          <span className={`ml-1 font-medium ${node.edr.realTimeProtection ? 'text-green-600' : 'text-red-600'}`}>
                            {node.edr.realTimeProtection ? 'On' : 'Off'}
                          </span>
                        </div>
                      </div>
                    </div>
                  </div>

                  {/* Network */}
                  <div className="space-y-3">
                    <h4 className="text-sm font-medium text-gray-900 flex items-center">
                      <Network className="w-4 h-4 mr-2 text-purple-600" />
                      Network Security
                    </h4>
                    <div className="space-y-2 text-sm">
                      <div className="flex items-center justify-between">
                        <span className="text-gray-600">Segmentation</span>
                        <Badge variant="outline" size="sm" className="capitalize">
                          {node.network.segmentation}
                        </Badge>
                      </div>
                      <div className="grid grid-cols-2 gap-2 text-xs">
                        <div>
                          <span className="text-gray-500">Firewall:</span>
                          <span className={`ml-1 font-medium ${node.network.firewall.enabled ? 'text-green-600' : 'text-red-600'}`}>
                            {node.network.firewall.enabled ? 'On' : 'Off'}
                          </span>
                        </div>
                        <div>
                          <span className="text-gray-500">VPN:</span>
                          <span className={`ml-1 font-medium ${node.network.vpn.connected ? 'text-green-600' : 'text-red-600'}`}>
                            {node.network.vpn.connected ? 'Connected' : 'Disconnected'}
                          </span>
                        </div>
                        <div>
                          <span className="text-gray-500">Blocked:</span>
                          <span className="ml-1 font-medium">{node.network.firewall.blockedConnections}</span>
                        </div>
                        <div>
                          <span className="text-gray-500">IDS Alerts:</span>
                          <span className="ml-1 font-medium">{node.network.intrusion.alertsToday}</span>
                        </div>
                      </div>
                    </div>
                  </div>

                  {/* USB */}
                  <div className="space-y-3">
                    <h4 className="text-sm font-medium text-gray-900 flex items-center">
                      <Usb className="w-4 h-4 mr-2 text-orange-600" />
                      USB Security
                    </h4>
                    <div className="space-y-2 text-sm">
                      <div className="flex items-center justify-between">
                        <span className="text-gray-600">Policy</span>
                        <Badge variant="outline" size="sm" className="capitalize">
                          {node.usb.policy.replace('_', ' ')}
                        </Badge>
                      </div>
                      <div className="grid grid-cols-2 gap-2 text-xs">
                        <div>
                          <span className="text-gray-500">Whitelisted:</span>
                          <span className="ml-1 font-medium">{node.usb.whitelistedDevices.length}</span>
                        </div>
                        <div>
                          <span className="text-gray-500">Blocked:</span>
                          <span className="ml-1 font-medium text-red-600">{node.usb.blockedAttempts}</span>
                        </div>
                        <div>
                          <span className="text-gray-500">Auto-mount:</span>
                          <span className={`ml-1 font-medium ${node.usb.autoMount ? 'text-red-600' : 'text-green-600'}`}>
                            {node.usb.autoMount ? 'On' : 'Off'}
                          </span>
                        </div>
                        <div>
                          <span className="text-gray-500">Encryption:</span>
                          <span className={`ml-1 font-medium ${node.usb.encryption ? 'text-green-600' : 'text-red-600'}`}>
                            {node.usb.encryption ? 'Required' : 'Optional'}
                          </span>
                        </div>
                      </div>
                    </div>
                  </div>

                  {/* Access Control */}
                  <div className="space-y-3">
                    <h4 className="text-sm font-medium text-gray-900 flex items-center">
                      <Lock className="w-4 h-4 mr-2 text-indigo-600" />
                      Access Control
                    </h4>
                    <div className="space-y-2 text-sm">
                      <div className="flex items-center justify-between">
                        <span className="text-gray-600">Auth Method</span>
                        <Badge variant="outline" size="sm" className="capitalize">
                          {node.accessControl.authentication}
                        </Badge>
                      </div>
                      <div className="grid grid-cols-2 gap-2 text-xs">
                        <div>
                          <span className="text-gray-500">Sessions:</span>
                          <span className="ml-1 font-medium">{node.accessControl.activeSessions}</span>
                        </div>
                        <div>
                          <span className="text-gray-500">Failed:</span>
                          <span className={`ml-1 font-medium ${node.accessControl.failedAttempts > 0 ? 'text-red-600' : 'text-green-600'}`}>
                            {node.accessControl.failedAttempts}
                          </span>
                        </div>
                        <div>
                          <span className="text-gray-500">Timeout:</span>
                          <span className="ml-1 font-medium">{node.accessControl.sessionTimeout}m</span>
                        </div>
                        <div>
                          <span className="text-gray-500">RBAC:</span>
                          <span className={`ml-1 font-medium ${node.accessControl.authorization === 'rbac' ? 'text-green-600' : 'text-yellow-600'}`}>
                            {node.accessControl.authorization.toUpperCase()}
                          </span>
                        </div>
                      </div>
                    </div>
                  </div>

                  {/* Data Protection */}
                  <div className="space-y-3">
                    <h4 className="text-sm font-medium text-gray-900 flex items-center">
                      <Database className="w-4 h-4 mr-2 text-teal-600" />
                      Data Protection
                    </h4>
                    <div className="space-y-2 text-sm">
                      <div className="flex items-center justify-between">
                        <span className="text-gray-600">Encryption</span>
                        <Badge className={`${node.dataProtection.encryption.diskEncryption && node.dataProtection.encryption.dataAtRest ? 'bg-green-100 text-green-800' : 'bg-yellow-100 text-yellow-800'}`} size="sm">
                          {node.dataProtection.encryption.diskEncryption && node.dataProtection.encryption.dataAtRest ? 'Full' : 'Partial'}
                        </Badge>
                      </div>
                      <div className="grid grid-cols-2 gap-2 text-xs">
                        <div>
                          <span className="text-gray-500">Last Backup:</span>
                          <span className="ml-1 font-medium">
                            {Math.round((new Date().getTime() - node.dataProtection.backup.lastBackup.getTime()) / (60 * 60 * 1000))}h ago
                          </span>
                        </div>
                        <div>
                          <span className="text-gray-500">DLP:</span>
                          <span className={`ml-1 font-medium ${node.dataProtection.dlp.violations === 0 ? 'text-green-600' : 'text-red-600'}`}>
                            {node.dataProtection.dlp.violations} violations
                          </span>
                        </div>
                        <div>
                          <span className="text-gray-500">Offsite:</span>
                          <span className={`ml-1 font-medium ${node.dataProtection.backup.offsite ? 'text-green-600' : 'text-red-600'}`}>
                            {node.dataProtection.backup.offsite ? 'Yes' : 'No'}
                          </span>
                        </div>
                        <div>
                          <span className="text-gray-500">Key Mgmt:</span>
                          <span className="ml-1 font-medium">Vault</span>
                        </div>
                      </div>
                    </div>
                  </div>
                </div>

                {/* Active Threats */}
                {node.threats.length > 0 && (
                  <div className="mt-6 pt-6 border-t">
                    <h4 className="text-sm font-medium text-gray-900 mb-3">Active Threats</h4>
                    <div className="space-y-2">
                      {node.threats.map(threat => (
                        <div key={threat.id} className="p-3 bg-red-50 rounded border border-red-200">
                          <div className="flex items-center justify-between mb-2">
                            <div className="flex items-center space-x-2">
                              <Badge className={getSeverityColor(threat.severity)} size="sm">
                                {threat.severity}
                              </Badge>
                              <span className="text-sm font-medium capitalize">{threat.type}</span>
                            </div>
                            <Badge className={getStatusColor(threat.status)} size="sm">
                              {threat.status}
                            </Badge>
                          </div>
                          <div className="text-sm text-gray-700 mb-1">{threat.description}</div>
                          <div className="text-xs text-gray-600">
                            Source: {threat.source} • Detected: {threat.detectedAt.toLocaleString()}
                          </div>
                          {threat.actions.length > 0 && (
                            <div className="text-xs text-gray-600 mt-1">
                              Actions: {threat.actions.join(', ')}
                            </div>
                          )}
                        </div>
                      ))}
                    </div>
                  </div>
                )}
              </Card>
            ))}
          </div>
        </TabsContent>

        <TabsContent value="violations" className="space-y-4">
          <div className="space-y-4">
            {policies.flatMap(policy => 
              policy.violations.map(violation => (
                <Card key={violation.id} className="p-4">
                  <div className="flex items-start justify-between">
                    <div className="flex items-start space-x-3">
                      <AlertTriangle className={`w-5 h-5 mt-0.5 ${
                        violation.severity === 'critical' ? 'text-red-600' :
                        violation.severity === 'high' ? 'text-orange-600' :
                        violation.severity === 'medium' ? 'text-yellow-600' :
                        'text-blue-600'
                      }`} />
                      <div className="flex-1">
                        <div className="flex items-center space-x-3 mb-2">
                          <h4 className="font-medium text-gray-900">{violation.ruleName}</h4>
                          <Badge className={getSeverityColor(violation.severity)}>
                            {violation.severity}
                          </Badge>
                          <Badge className={getStatusColor(violation.status)}>
                            {violation.status.replace('_', ' ')}
                          </Badge>
                        </div>
                        
                        <p className="text-gray-600 text-sm mb-2">{violation.description}</p>
                        
                        <div className="grid grid-cols-3 gap-4 text-sm mb-3">
                          <div>
                            <span className="text-gray-500">Node:</span>
                            <span className="ml-2 font-medium">{violation.nodeName}</span>
                          </div>
                          <div>
                            <span className="text-gray-500">Detected:</span>
                            <span className="ml-2 font-medium">{violation.detectedAt.toLocaleString()}</span>
                          </div>
                          <div>
                            <span className="text-gray-500">Policy:</span>
                            <span className="ml-2 font-medium">{policy.name}</span>
                          </div>
                        </div>
                        
                        {violation.evidence.length > 0 && (
                          <div className="mb-3">
                            <div className="text-sm font-medium text-gray-900 mb-1">Evidence:</div>
                            <div className="flex flex-wrap gap-1">
                              {violation.evidence.map((evidence, index) => (
                                <Badge key={index} variant="outline" size="sm" className="font-mono text-xs">
                                  {evidence}
                                </Badge>
                              ))}
                            </div>
                          </div>
                        )}
                        
                        {violation.remediation && (
                          <div className="p-2 bg-green-50 rounded border border-green-200">
                            <div className="text-sm font-medium text-green-900">Remediation:</div>
                            <div className="text-sm text-green-700">{violation.remediation}</div>
                          </div>
                        )}
                      </div>
                    </div>
                    
                    <div className="flex items-center space-x-2">
                      <Button
                        variant="outline"
                        size="sm"
                        onClick={() => {
                          setSelectedViolation(violation)
                          setShowViolationDialog(true)
                        }}
                      >
                        <Eye className="w-4 h-4" />
                      </Button>
                      
                      {violation.status === 'open' || violation.status === 'investigating' ? (
                        <Button
                          size="sm"
                          onClick={() => handleViolationResolve(violation.id)}
                        >
                          Resolve
                        </Button>
                      ) : (
                        <CheckCircle className="w-5 h-5 text-green-600" />
                      )}
                    </div>
                  </div>
                </Card>
              ))
            )}
            
            {policies.every(policy => policy.violations.length === 0) && (
              <div className="text-center py-12">
                <CheckCircle className="w-16 h-16 text-green-500 mx-auto mb-4" />
                <h3 className="text-lg font-medium text-gray-900 mb-2">No Security Violations</h3>
                <p className="text-gray-600">All security policies are being followed correctly</p>
              </div>
            )}
          </div>
        </TabsContent>

        <TabsContent value="compliance" className="space-y-4">
          <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-6">
            {policies.map(policy => (
              <Card key={policy.id} className="p-6">
                <div className="flex items-center justify-between mb-4">
                  <h3 className="text-lg font-medium text-gray-900">{policy.name}</h3>
                  <div className={`text-2xl font-bold ${getSecurityScoreColor(policy.compliance.score)}`}>
                    {policy.compliance.score}%
                  </div>
                </div>

                <div className="space-y-3">
                  <div>
                    <div className="text-sm text-gray-600">Framework</div>
                    <div className="font-medium">{policy.compliance.framework}</div>
                  </div>

                  <div>
                    <div className="text-sm text-gray-600 mb-1">Compliance Score</div>
                    <Progress value={policy.compliance.score} className="h-2" />
                  </div>

                  <div className="space-y-2">
                    {policy.compliance.requirements.slice(0, 2).map(req => (
                      <div key={req.id} className="flex items-center justify-between p-2 bg-gray-50 rounded">
                        <div className="flex-1">
                          <div className="text-sm font-medium">{req.requirement}</div>
                          {req.gap && (
                            <div className="text-xs text-red-600 mt-1">{req.gap}</div>
                          )}
                        </div>
                        <Badge className={getStatusColor(req.status)} size="sm">
                          {req.status.replace('_', ' ')}
                        </Badge>
                      </div>
                    ))}
                    {policy.compliance.requirements.length > 2 && (
                      <div className="text-xs text-gray-500 text-center">
                        +{policy.compliance.requirements.length - 2} more requirements
                      </div>
                    )}
                  </div>

                  <div className="pt-3 border-t text-xs text-gray-500">
                    <div>Last Assessment: {policy.compliance.lastAssessment.toLocaleDateString()}</div>
                    <div>Next Assessment: {policy.compliance.nextAssessment.toLocaleDateString()}</div>
                  </div>
                </div>
              </Card>
            ))}
          </div>
        </TabsContent>
      </Tabs>
    </div>
  )
}

export default SecurityManager
