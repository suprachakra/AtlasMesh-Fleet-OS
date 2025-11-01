import React, { useState, useCallback, useEffect, useMemo } from 'react'
import { 
  Shield, Lock, Unlock, AlertTriangle, CheckCircle, XCircle, Wifi, 
  HardDrive, Usb, Network, Eye, EyeOff, Settings, RefreshCw, 
  Download, Upload, Activity, Zap, Clock, Server, Monitor,
  FileText, Key, Globe, Smartphone, Laptop, Database, Terminal,
  Bug, Wrench, Package, Users, History, MoreHorizontal, Play, Pause
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

// Types
interface GaragePC {
  id: string
  name: string
  location: string
  ipAddress: string
  macAddress: string
  osVersion: string
  lastSeen: Date
  status: 'online' | 'offline' | 'maintenance' | 'compromised'
  securityProfile: SecurityProfile
  patchingStatus: PatchingStatus
  edrStatus: EDRStatus
  usbPolicy: USBPolicy
  networkSegmentation: NetworkSegmentation
  complianceScore: number
  vulnerabilities: Vulnerability[]
  securityEvents: SecurityEvent[]
  hardwareInfo: HardwareInfo
  installedSoftware: InstalledSoftware[]
  networkConnections: NetworkConnection[]
  processes: ProcessInfo[]
}

interface SecurityProfile {
  id: string
  name: string
  level: 'basic' | 'standard' | 'hardened' | 'maximum'
  policies: SecurityPolicy[]
  lastUpdated: Date
  appliedAt?: Date
  status: 'active' | 'pending' | 'failed'
}

interface SecurityPolicy {
  id: string
  name: string
  category: 'os_hardening' | 'network' | 'application' | 'user_access' | 'logging'
  description: string
  enabled: boolean
  enforced: boolean
  lastChecked: Date
  compliant: boolean
  remediation?: string
}

interface PatchingStatus {
  lastUpdate: Date
  nextScheduledUpdate: Date
  pendingPatches: PatchInfo[]
  installedPatches: PatchInfo[]
  autoUpdateEnabled: boolean
  maintenanceWindow: MaintenanceWindow
  patchingPolicy: 'immediate' | 'scheduled' | 'manual'
  rebootRequired: boolean
}

interface PatchInfo {
  id: string
  title: string
  description: string
  severity: 'critical' | 'high' | 'medium' | 'low'
  releaseDate: Date
  installDate?: Date
  status: 'pending' | 'installed' | 'failed' | 'superseded'
  kbNumber?: string
  requiresReboot: boolean
  size: number
}

interface MaintenanceWindow {
  enabled: boolean
  dayOfWeek: string
  startTime: string
  endTime: string
  timezone: string
  allowEmergencyPatches: boolean
}

interface EDRStatus {
  agent: string
  version: string
  lastUpdate: Date
  status: 'running' | 'stopped' | 'error' | 'updating'
  threatDetection: boolean
  behaviorAnalysis: boolean
  networkMonitoring: boolean
  fileIntegrityMonitoring: boolean
  quarantinedFiles: QuarantinedFile[]
  detectedThreats: ThreatInfo[]
  scanResults: ScanResult[]
}

interface QuarantinedFile {
  id: string
  filePath: string
  threatType: string
  detectionTime: Date
  quarantineTime: Date
  status: 'quarantined' | 'restored' | 'deleted'
  riskLevel: 'critical' | 'high' | 'medium' | 'low'
}

interface ThreatInfo {
  id: string
  type: 'malware' | 'suspicious_behavior' | 'network_anomaly' | 'unauthorized_access'
  severity: 'critical' | 'high' | 'medium' | 'low'
  detectionTime: Date
  description: string
  source: string
  status: 'active' | 'mitigated' | 'false_positive'
  mitigation?: string
}

interface ScanResult {
  id: string
  scanType: 'full' | 'quick' | 'custom'
  startTime: Date
  endTime: Date
  filesScanned: number
  threatsFound: number
  status: 'completed' | 'running' | 'failed' | 'cancelled'
}

interface USBPolicy {
  enabled: boolean
  mode: 'block_all' | 'allow_read_only' | 'allow_specific' | 'monitor_only'
  allowedDevices: AllowedUSBDevice[]
  blockedDevices: BlockedUSBDevice[]
  monitoring: USBMonitoring
  violations: USBViolation[]
}

interface AllowedUSBDevice {
  id: string
  deviceId: string
  vendorId: string
  productId: string
  serialNumber?: string
  description: string
  permissions: 'read_only' | 'read_write'
  approvedBy: string
  approvedAt: Date
  expiresAt?: Date
}

interface BlockedUSBDevice {
  id: string
  deviceId: string
  reason: string
  blockedAt: Date
  blockedBy: string
}

interface USBMonitoring {
  logConnections: boolean
  logFileTransfers: boolean
  alertOnUnknownDevice: boolean
  quarantineUnknownFiles: boolean
}

interface USBViolation {
  id: string
  deviceId: string
  violationType: 'unauthorized_device' | 'policy_violation' | 'malware_detected'
  detectionTime: Date
  description: string
  action: 'blocked' | 'quarantined' | 'allowed_with_warning'
  user: string
}

interface NetworkSegmentation {
  enabled: boolean
  vlan: string
  subnet: string
  firewallRules: FirewallRule[]
  allowedConnections: AllowedConnection[]
  blockedConnections: BlockedConnection[]
  monitoring: NetworkMonitoring
}

interface FirewallRule {
  id: string
  name: string
  direction: 'inbound' | 'outbound'
  action: 'allow' | 'block' | 'log'
  protocol: 'tcp' | 'udp' | 'icmp' | 'any'
  sourceIP: string
  sourcePort: string
  destinationIP: string
  destinationPort: string
  enabled: boolean
  priority: number
  hitCount: number
  lastHit?: Date
}

interface AllowedConnection {
  id: string
  destination: string
  port: number
  protocol: string
  purpose: string
  approvedBy: string
  approvedAt: Date
}

interface BlockedConnection {
  id: string
  source: string
  destination: string
  port: number
  protocol: string
  reason: string
  blockedAt: Date
  attempts: number
}

interface NetworkMonitoring {
  enabled: boolean
  logConnections: boolean
  detectAnomalies: boolean
  alertOnUnauthorized: boolean
  bandwidthMonitoring: boolean
}

interface Vulnerability {
  id: string
  cveId?: string
  title: string
  description: string
  severity: 'critical' | 'high' | 'medium' | 'low'
  cvssScore: number
  affectedComponent: string
  discoveredAt: Date
  status: 'open' | 'mitigated' | 'false_positive' | 'accepted_risk'
  remediation?: string
  patchAvailable: boolean
  exploitAvailable: boolean
}

interface SecurityEvent {
  id: string
  timestamp: Date
  category: 'authentication' | 'authorization' | 'system_change' | 'network' | 'file_access'
  severity: 'critical' | 'high' | 'medium' | 'low' | 'info'
  source: string
  description: string
  user?: string
  ipAddress?: string
  status: 'open' | 'investigating' | 'resolved' | 'false_positive'
  responseActions: string[]
}

interface HardwareInfo {
  manufacturer: string
  model: string
  serialNumber: string
  cpu: string
  memory: string
  storage: StorageInfo[]
  networkAdapters: NetworkAdapter[]
  usbPorts: USBPort[]
  tpmEnabled: boolean
  secureBootEnabled: boolean
}

interface StorageInfo {
  device: string
  type: 'ssd' | 'hdd' | 'nvme'
  capacity: number
  used: number
  health: 'good' | 'warning' | 'critical'
  encrypted: boolean
  smartStatus: string
}

interface NetworkAdapter {
  name: string
  macAddress: string
  ipAddress: string
  status: 'connected' | 'disconnected'
  speed: string
  type: 'ethernet' | 'wifi' | 'cellular'
}

interface USBPort {
  port: string
  status: 'active' | 'disabled'
  connectedDevice?: string
  policy: 'allowed' | 'blocked' | 'monitored'
}

interface InstalledSoftware {
  name: string
  version: string
  vendor: string
  installDate: Date
  size: number
  category: 'system' | 'application' | 'driver' | 'security'
  vulnerabilities: number
  lastUpdated?: Date
}

interface NetworkConnection {
  localAddress: string
  localPort: number
  remoteAddress: string
  remotePort: number
  protocol: string
  state: string
  process: string
  established: Date
  bytesIn: number
  bytesOut: number
}

interface ProcessInfo {
  pid: number
  name: string
  user: string
  cpuUsage: number
  memoryUsage: number
  startTime: Date
  commandLine: string
  parentPid?: number
  status: 'running' | 'sleeping' | 'zombie'
}

interface GaragePCSecurityManagerProps {
  garagePCId?: string
  onSecurityEventDetected?: (event: SecurityEvent) => void
  onPolicyViolation?: (violation: any) => void
  className?: string
}

const GaragePCSecurityManager: React.FC<GaragePCSecurityManagerProps> = ({
  garagePCId,
  onSecurityEventDetected,
  onPolicyViolation,
  className = ''
}) => {
  // State
  const [garagePCs, setGaragePCs] = useState<GaragePC[]>([])
  const [selectedPC, setSelectedPC] = useState<GaragePC | null>(null)
  const [showDetailsDialog, setShowDetailsDialog] = useState(false)
  const [showPolicyDialog, setShowPolicyDialog] = useState(false)
  const [activeTab, setActiveTab] = useState('overview')
  const [filters, setFilters] = useState({
    status: '',
    location: '',
    search: '',
    securityLevel: ''
  })
  const [isScanning, setIsScanning] = useState(false)
  const [scanProgress, setScanProgress] = useState(0)

  // Mock data initialization
  const initializeMockData = useCallback(() => {
    const mockGaragePCs: GaragePC[] = [
      {
        id: 'garage-pc-001',
        name: 'Depot-A-Station-01',
        location: 'Dubai Main Depot - Bay A',
        ipAddress: '192.168.10.101',
        macAddress: '00:1B:44:11:3A:B7',
        osVersion: 'Ubuntu 22.04.3 LTS',
        lastSeen: new Date('2024-11-26T10:45:00Z'),
        status: 'online',
        securityProfile: {
          id: 'profile-hardened',
          name: 'Hardened Security Profile',
          level: 'hardened',
          policies: [
            {
              id: 'pol-firewall',
              name: 'Firewall Configuration',
              category: 'network',
              description: 'Strict inbound/outbound firewall rules',
              enabled: true,
              enforced: true,
              lastChecked: new Date('2024-11-26T10:00:00Z'),
              compliant: true
            },
            {
              id: 'pol-usb-lockdown',
              name: 'USB Device Lockdown',
              category: 'os_hardening',
              description: 'Block unauthorized USB devices',
              enabled: true,
              enforced: true,
              lastChecked: new Date('2024-11-26T10:00:00Z'),
              compliant: true
            },
            {
              id: 'pol-user-access',
              name: 'User Access Control',
              category: 'user_access',
              description: 'Restrict user privileges and sudo access',
              enabled: true,
              enforced: true,
              lastChecked: new Date('2024-11-26T10:00:00Z'),
              compliant: false,
              remediation: 'Remove unnecessary sudo privileges for service accounts'
            }
          ],
          lastUpdated: new Date('2024-11-20T14:30:00Z'),
          appliedAt: new Date('2024-11-20T15:00:00Z'),
          status: 'active'
        },
        patchingStatus: {
          lastUpdate: new Date('2024-11-25T02:00:00Z'),
          nextScheduledUpdate: new Date('2024-12-02T02:00:00Z'),
          pendingPatches: [
            {
              id: 'patch-001',
              title: 'Linux Kernel Security Update',
              description: 'Critical security patches for kernel vulnerabilities',
              severity: 'critical',
              releaseDate: new Date('2024-11-24T00:00:00Z'),
              status: 'pending',
              kbNumber: 'USN-6534-1',
              requiresReboot: true,
              size: 45 * 1024 * 1024 // 45MB
            },
            {
              id: 'patch-002',
              title: 'OpenSSL Security Update',
              description: 'Fix for CVE-2024-XXXX in OpenSSL library',
              severity: 'high',
              releaseDate: new Date('2024-11-23T00:00:00Z'),
              status: 'pending',
              kbNumber: 'USN-6533-1',
              requiresReboot: false,
              size: 8 * 1024 * 1024 // 8MB
            }
          ],
          installedPatches: [
            {
              id: 'patch-installed-001',
              title: 'System Security Update',
              description: 'Monthly security patches',
              severity: 'medium',
              releaseDate: new Date('2024-11-20T00:00:00Z'),
              installDate: new Date('2024-11-25T02:15:00Z'),
              status: 'installed',
              kbNumber: 'USN-6532-1',
              requiresReboot: false,
              size: 25 * 1024 * 1024 // 25MB
            }
          ],
          autoUpdateEnabled: true,
          maintenanceWindow: {
            enabled: true,
            dayOfWeek: 'sunday',
            startTime: '02:00',
            endTime: '04:00',
            timezone: 'Asia/Dubai',
            allowEmergencyPatches: true
          },
          patchingPolicy: 'scheduled',
          rebootRequired: true
        },
        edrStatus: {
          agent: 'CrowdStrike Falcon',
          version: '7.10.19303.0',
          lastUpdate: new Date('2024-11-26T08:00:00Z'),
          status: 'running',
          threatDetection: true,
          behaviorAnalysis: true,
          networkMonitoring: true,
          fileIntegrityMonitoring: true,
          quarantinedFiles: [
            {
              id: 'qf-001',
              filePath: '/tmp/suspicious_script.sh',
              threatType: 'Potentially Unwanted Program',
              detectionTime: new Date('2024-11-25T14:30:00Z'),
              quarantineTime: new Date('2024-11-25T14:30:30Z'),
              status: 'quarantined',
              riskLevel: 'medium'
            }
          ],
          detectedThreats: [
            {
              id: 'threat-001',
              type: 'suspicious_behavior',
              severity: 'medium',
              detectionTime: new Date('2024-11-26T09:15:00Z'),
              description: 'Unusual network activity detected from garage management process',
              source: '/opt/garage-mgmt/bin/data-sync',
              status: 'mitigated',
              mitigation: 'Process whitelisted after verification'
            }
          ],
          scanResults: [
            {
              id: 'scan-001',
              scanType: 'full',
              startTime: new Date('2024-11-26T01:00:00Z'),
              endTime: new Date('2024-11-26T03:45:00Z'),
              filesScanned: 1250000,
              threatsFound: 0,
              status: 'completed'
            }
          ]
        },
        usbPolicy: {
          enabled: true,
          mode: 'allow_specific',
          allowedDevices: [
            {
              id: 'usb-allow-001',
              deviceId: 'USB\\VID_0781&PID_5567',
              vendorId: '0781',
              productId: '5567',
              serialNumber: 'AA010203050813213455',
              description: 'SanDisk Ultra USB 3.0 - Maintenance Tools',
              permissions: 'read_only',
              approvedBy: 'security-admin',
              approvedAt: new Date('2024-11-15T10:00:00Z'),
              expiresAt: new Date('2025-11-15T10:00:00Z')
            }
          ],
          blockedDevices: [
            {
              id: 'usb-block-001',
              deviceId: 'USB\\VID_FFFF&PID_0001',
              reason: 'Unknown vendor - potential security risk',
              blockedAt: new Date('2024-11-24T16:30:00Z'),
              blockedBy: 'system-policy'
            }
          ],
          monitoring: {
            logConnections: true,
            logFileTransfers: true,
            alertOnUnknownDevice: true,
            quarantineUnknownFiles: true
          },
          violations: [
            {
              id: 'usb-viol-001',
              deviceId: 'USB\\VID_FFFF&PID_0001',
              violationType: 'unauthorized_device',
              detectionTime: new Date('2024-11-24T16:30:00Z'),
              description: 'Attempt to connect unauthorized USB device',
              action: 'blocked',
              user: 'technician-01'
            }
          ]
        },
        networkSegmentation: {
          enabled: true,
          vlan: 'VLAN-100-GARAGE',
          subnet: '192.168.10.0/24',
          firewallRules: [
            {
              id: 'fw-001',
              name: 'Allow Fleet Management',
              direction: 'outbound',
              action: 'allow',
              protocol: 'tcp',
              sourceIP: '192.168.10.101',
              sourcePort: 'any',
              destinationIP: '10.0.1.0/24',
              destinationPort: '443,8443',
              enabled: true,
              priority: 100,
              hitCount: 15847,
              lastHit: new Date('2024-11-26T10:44:00Z')
            },
            {
              id: 'fw-002',
              name: 'Block Internet Access',
              direction: 'outbound',
              action: 'block',
              protocol: 'any',
              sourceIP: '192.168.10.101',
              sourcePort: 'any',
              destinationIP: '0.0.0.0/0',
              destinationPort: 'any',
              enabled: true,
              priority: 1000,
              hitCount: 23,
              lastHit: new Date('2024-11-26T08:15:00Z')
            }
          ],
          allowedConnections: [
            {
              id: 'conn-allow-001',
              destination: '10.0.1.50',
              port: 443,
              protocol: 'HTTPS',
              purpose: 'Fleet Management API',
              approvedBy: 'network-admin',
              approvedAt: new Date('2024-11-15T10:00:00Z')
            }
          ],
          blockedConnections: [
            {
              id: 'conn-block-001',
              source: '192.168.10.101',
              destination: '8.8.8.8',
              port: 53,
              protocol: 'DNS',
              reason: 'Unauthorized external DNS query',
              blockedAt: new Date('2024-11-26T08:15:00Z'),
              attempts: 5
            }
          ],
          monitoring: {
            enabled: true,
            logConnections: true,
            detectAnomalies: true,
            alertOnUnauthorized: true,
            bandwidthMonitoring: true
          }
        },
        complianceScore: 87,
        vulnerabilities: [
          {
            id: 'vuln-001',
            cveId: 'CVE-2024-XXXX',
            title: 'Privilege Escalation in sudo',
            description: 'Local privilege escalation vulnerability in sudo command',
            severity: 'high',
            cvssScore: 7.8,
            affectedComponent: 'sudo (version 1.9.9)',
            discoveredAt: new Date('2024-11-24T12:00:00Z'),
            status: 'open',
            remediation: 'Update sudo to version 1.9.15 or later',
            patchAvailable: true,
            exploitAvailable: false
          }
        ],
        securityEvents: [
          {
            id: 'event-001',
            timestamp: new Date('2024-11-26T10:30:00Z'),
            category: 'authentication',
            severity: 'medium',
            source: 'SSH',
            description: 'Multiple failed SSH login attempts from internal IP',
            user: 'unknown',
            ipAddress: '192.168.10.200',
            status: 'investigating',
            responseActions: ['IP temporarily blocked', 'Security team notified']
          }
        ],
        hardwareInfo: {
          manufacturer: 'Dell',
          model: 'OptiPlex 7090',
          serialNumber: 'DELLPC001234',
          cpu: 'Intel Core i7-11700 @ 2.50GHz',
          memory: '32GB DDR4',
          storage: [
            {
              device: '/dev/sda',
              type: 'ssd',
              capacity: 512 * 1024 * 1024 * 1024, // 512GB
              used: 256 * 1024 * 1024 * 1024, // 256GB
              health: 'good',
              encrypted: true,
              smartStatus: 'PASSED'
            }
          ],
          networkAdapters: [
            {
              name: 'eth0',
              macAddress: '00:1B:44:11:3A:B7',
              ipAddress: '192.168.10.101',
              status: 'connected',
              speed: '1Gbps',
              type: 'ethernet'
            }
          ],
          usbPorts: [
            {
              port: 'USB1',
              status: 'disabled',
              policy: 'blocked'
            },
            {
              port: 'USB2',
              status: 'active',
              connectedDevice: 'SanDisk Ultra USB 3.0',
              policy: 'allowed'
            }
          ],
          tpmEnabled: true,
          secureBootEnabled: true
        },
        installedSoftware: [
          {
            name: 'Docker Engine',
            version: '24.0.7',
            vendor: 'Docker Inc.',
            installDate: new Date('2024-11-15T10:00:00Z'),
            size: 150 * 1024 * 1024, // 150MB
            category: 'application',
            vulnerabilities: 0,
            lastUpdated: new Date('2024-11-20T08:00:00Z')
          },
          {
            name: 'OpenSSL',
            version: '3.0.2',
            vendor: 'OpenSSL Software Foundation',
            installDate: new Date('2024-11-15T10:00:00Z'),
            size: 50 * 1024 * 1024, // 50MB
            category: 'system',
            vulnerabilities: 1
          }
        ],
        networkConnections: [
          {
            localAddress: '192.168.10.101',
            localPort: 22,
            remoteAddress: '192.168.10.50',
            remotePort: 54321,
            protocol: 'TCP',
            state: 'ESTABLISHED',
            process: 'sshd',
            established: new Date('2024-11-26T09:00:00Z'),
            bytesIn: 1024 * 1024, // 1MB
            bytesOut: 512 * 1024 // 512KB
          }
        ],
        processes: [
          {
            pid: 1234,
            name: 'garage-manager',
            user: 'garage-svc',
            cpuUsage: 2.5,
            memoryUsage: 128 * 1024 * 1024, // 128MB
            startTime: new Date('2024-11-26T08:00:00Z'),
            commandLine: '/opt/garage-mgmt/bin/garage-manager --config /etc/garage/config.yml',
            status: 'running'
          }
        ]
      }
    ]

    setGaragePCs(mockGaragePCs)
  }, [])

  useEffect(() => {
    initializeMockData()
  }, [initializeMockData])

  // Real-time updates simulation
  useEffect(() => {
    const interval = setInterval(() => {
      setGaragePCs(prev => prev.map(pc => ({
        ...pc,
        lastSeen: new Date(),
        complianceScore: Math.max(70, Math.min(100, pc.complianceScore + (Math.random() - 0.5) * 2))
      })))
    }, 30000) // Update every 30 seconds

    return () => clearInterval(interval)
  }, [])

  // Handlers
  const handlePCSelect = useCallback((pc: GaragePC) => {
    setSelectedPC(pc)
    setShowDetailsDialog(true)
  }, [])

  const handleSecurityScan = useCallback(async (pcId: string) => {
    setIsScanning(true)
    setScanProgress(0)

    // Simulate scan progress
    const progressInterval = setInterval(() => {
      setScanProgress(prev => {
        const newProgress = prev + Math.random() * 10
        if (newProgress >= 100) {
          clearInterval(progressInterval)
          setIsScanning(false)
          return 100
        }
        return newProgress
      })
    }, 500)

    // Simulate scan completion
    setTimeout(() => {
      clearInterval(progressInterval)
      setIsScanning(false)
      setScanProgress(100)
      
      // Add new scan result
      setGaragePCs(prev => prev.map(pc => 
        pc.id === pcId 
          ? {
              ...pc,
              edrStatus: {
                ...pc.edrStatus,
                scanResults: [
                  {
                    id: `scan-${Date.now()}`,
                    scanType: 'quick',
                    startTime: new Date(),
                    endTime: new Date(),
                    filesScanned: Math.floor(Math.random() * 100000) + 50000,
                    threatsFound: Math.floor(Math.random() * 3),
                    status: 'completed'
                  },
                  ...pc.edrStatus.scanResults
                ]
              }
            }
          : pc
      ))
    }, 8000)
  }, [])

  const handlePolicyUpdate = useCallback((pcId: string, policyId: string, enabled: boolean) => {
    setGaragePCs(prev => prev.map(pc => 
      pc.id === pcId 
        ? {
            ...pc,
            securityProfile: {
              ...pc.securityProfile,
              policies: pc.securityProfile.policies.map(policy =>
                policy.id === policyId 
                  ? { ...policy, enabled, lastChecked: new Date() }
                  : policy
              ),
              lastUpdated: new Date()
            }
          }
        : pc
    ))
  }, [])

  const handleUSBPolicyUpdate = useCallback((pcId: string, mode: USBPolicy['mode']) => {
    setGaragePCs(prev => prev.map(pc => 
      pc.id === pcId 
        ? {
            ...pc,
            usbPolicy: {
              ...pc.usbPolicy,
              mode
            }
          }
        : pc
    ))
  }, [])

  // Filtered PCs
  const filteredPCs = useMemo(() => {
    return garagePCs
      .filter(pc => !filters.status || pc.status === filters.status)
      .filter(pc => !filters.location || pc.location.toLowerCase().includes(filters.location.toLowerCase()))
      .filter(pc => !filters.securityLevel || pc.securityProfile.level === filters.securityLevel)
      .filter(pc => {
        if (!filters.search) return true
        const searchTerm = filters.search.toLowerCase()
        return (
          pc.name.toLowerCase().includes(searchTerm) ||
          pc.location.toLowerCase().includes(searchTerm) ||
          pc.ipAddress.includes(searchTerm)
        )
      })
      .sort((a, b) => {
        // Sort by status (online first) then by compliance score
        if (a.status !== b.status) {
          const statusOrder = { online: 4, maintenance: 3, offline: 2, compromised: 1 }
          return statusOrder[b.status] - statusOrder[a.status]
        }
        return b.complianceScore - a.complianceScore
      })
  }, [garagePCs, filters])

  // Statistics
  const stats = useMemo(() => {
    const totalPCs = garagePCs.length
    const onlinePCs = garagePCs.filter(pc => pc.status === 'online').length
    const compromisedPCs = garagePCs.filter(pc => pc.status === 'compromised').length
    const avgComplianceScore = garagePCs.reduce((sum, pc) => sum + pc.complianceScore, 0) / totalPCs || 0
    
    const totalVulnerabilities = garagePCs.reduce((sum, pc) => sum + pc.vulnerabilities.length, 0)
    const criticalVulns = garagePCs.reduce((sum, pc) => 
      sum + pc.vulnerabilities.filter(v => v.severity === 'critical').length, 0
    )
    
    const pendingPatches = garagePCs.reduce((sum, pc) => sum + pc.patchingStatus.pendingPatches.length, 0)
    const rebootRequired = garagePCs.filter(pc => pc.patchingStatus.rebootRequired).length
    
    const activeThreats = garagePCs.reduce((sum, pc) => 
      sum + pc.edrStatus.detectedThreats.filter(t => t.status === 'active').length, 0
    )
    
    const quarantinedFiles = garagePCs.reduce((sum, pc) => 
      sum + pc.edrStatus.quarantinedFiles.filter(f => f.status === 'quarantined').length, 0
    )

    return {
      totalPCs,
      onlinePCs,
      compromisedPCs,
      avgComplianceScore,
      totalVulnerabilities,
      criticalVulns,
      pendingPatches,
      rebootRequired,
      activeThreats,
      quarantinedFiles
    }
  }, [garagePCs])

  const getStatusColor = (status: string) => {
    switch (status) {
      case 'online': return 'bg-green-100 text-green-800'
      case 'offline': return 'bg-gray-100 text-gray-800'
      case 'maintenance': return 'bg-yellow-100 text-yellow-800'
      case 'compromised': return 'bg-red-100 text-red-800'
      default: return 'bg-gray-100 text-gray-800'
    }
  }

  const getSecurityLevelColor = (level: string) => {
    switch (level) {
      case 'maximum': return 'bg-purple-100 text-purple-800'
      case 'hardened': return 'bg-blue-100 text-blue-800'
      case 'standard': return 'bg-green-100 text-green-800'
      case 'basic': return 'bg-yellow-100 text-yellow-800'
      default: return 'bg-gray-100 text-gray-800'
    }
  }

  const getSeverityColor = (severity: string) => {
    switch (severity) {
      case 'critical': return 'bg-red-100 text-red-800'
      case 'high': return 'bg-orange-100 text-orange-800'
      case 'medium': return 'bg-yellow-100 text-yellow-800'
      case 'low': return 'bg-green-100 text-green-800'
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
          <Shield className="w-6 h-6 text-blue-600" />
          <h2 className="text-2xl font-semibold text-gray-900">Garage PC Security Manager</h2>
        </div>
        <div className="flex items-center space-x-2">
          <Button variant="outline" onClick={initializeMockData}>
            <RefreshCw className="w-4 h-4 mr-2" />
            Refresh
          </Button>
          <Button variant="outline">
            <Download className="w-4 h-4 mr-2" />
            Export Report
          </Button>
          <Button>
            <Settings className="w-4 h-4 mr-2" />
            Global Policies
          </Button>
        </div>
      </div>

      {/* Statistics */}
      <div className="grid grid-cols-2 md:grid-cols-5 lg:grid-cols-10 gap-4">
        <Card className="p-4">
          <div className="text-center">
            <div className="text-2xl font-bold text-gray-900">{stats.totalPCs}</div>
            <div className="text-sm text-gray-600">Total PCs</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className="text-2xl font-bold text-green-600">{stats.onlinePCs}</div>
            <div className="text-sm text-gray-600">Online</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className="text-2xl font-bold text-red-600">{stats.compromisedPCs}</div>
            <div className="text-sm text-gray-600">Compromised</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className="text-2xl font-bold text-blue-600">{Math.round(stats.avgComplianceScore)}</div>
            <div className="text-sm text-gray-600">Avg Compliance</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className="text-2xl font-bold text-orange-600">{stats.totalVulnerabilities}</div>
            <div className="text-sm text-gray-600">Vulnerabilities</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className="text-2xl font-bold text-red-600">{stats.criticalVulns}</div>
            <div className="text-sm text-gray-600">Critical</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className="text-2xl font-bold text-purple-600">{stats.pendingPatches}</div>
            <div className="text-sm text-gray-600">Pending Patches</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className="text-2xl font-bold text-yellow-600">{stats.rebootRequired}</div>
            <div className="text-sm text-gray-600">Reboot Required</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className="text-2xl font-bold text-pink-600">{stats.activeThreats}</div>
            <div className="text-sm text-gray-600">Active Threats</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className="text-2xl font-bold text-indigo-600">{stats.quarantinedFiles}</div>
            <div className="text-sm text-gray-600">Quarantined</div>
          </div>
        </Card>
      </div>

      {/* Filters */}
      <Card className="p-4">
        <div className="flex flex-wrap items-center gap-4">
          <div className="flex items-center space-x-2">
            <Search className="w-4 h-4 text-gray-500" />
            <Input
              placeholder="Search PCs..."
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
            <option value="online">Online</option>
            <option value="offline">Offline</option>
            <option value="maintenance">Maintenance</option>
            <option value="compromised">Compromised</option>
          </Select>

          <Select
            value={filters.securityLevel}
            onValueChange={(value) => setFilters(prev => ({ ...prev, securityLevel: value }))}
          >
            <option value="">All Security Levels</option>
            <option value="maximum">Maximum</option>
            <option value="hardened">Hardened</option>
            <option value="standard">Standard</option>
            <option value="basic">Basic</option>
          </Select>

          <Input
            placeholder="Location filter..."
            value={filters.location}
            onChange={(e) => setFilters(prev => ({ ...prev, location: e.target.value }))}
            className="w-48"
          />
        </div>
      </Card>

      {/* Garage PCs List */}
      <div className="space-y-4">
        {filteredPCs.map(pc => (
          <Card key={pc.id} className="p-6 hover:shadow-md transition-shadow cursor-pointer"
                onClick={() => handlePCSelect(pc)}>
            <div className="flex items-start justify-between mb-4">
              <div className="flex items-center space-x-3">
                <div className={`p-2 rounded ${
                  pc.status === 'online' ? 'bg-green-100' :
                  pc.status === 'compromised' ? 'bg-red-100' :
                  pc.status === 'maintenance' ? 'bg-yellow-100' : 'bg-gray-100'
                }`}>
                  <Monitor className={`w-5 h-5 ${
                    pc.status === 'online' ? 'text-green-600' :
                    pc.status === 'compromised' ? 'text-red-600' :
                    pc.status === 'maintenance' ? 'text-yellow-600' : 'text-gray-600'
                  }`} />
                </div>
                <div>
                  <h3 className="text-lg font-medium text-gray-900">{pc.name}</h3>
                  <p className="text-sm text-gray-600">{pc.location}</p>
                  <p className="text-sm text-gray-600 font-mono">{pc.ipAddress} • {pc.osVersion}</p>
                </div>
              </div>
              <div className="flex items-center space-x-2">
                <Badge className={getStatusColor(pc.status)}>
                  {pc.status}
                </Badge>
                <Badge className={getSecurityLevelColor(pc.securityProfile.level)}>
                  {pc.securityProfile.level}
                </Badge>
                <div className="text-right">
                  <div className="text-sm font-medium text-gray-900">
                    Compliance: {pc.complianceScore}%
                  </div>
                  <Progress value={pc.complianceScore} className="w-20 h-2" />
                </div>
              </div>
            </div>

            <div className="grid grid-cols-2 md:grid-cols-4 gap-4 mb-4">
              <div>
                <div className="text-sm text-gray-600">EDR Status</div>
                <div className="flex items-center space-x-1">
                  <div className={`w-2 h-2 rounded-full ${
                    pc.edrStatus.status === 'running' ? 'bg-green-500' : 'bg-red-500'
                  }`} />
                  <span className="font-medium text-gray-900 text-sm">{pc.edrStatus.agent}</span>
                </div>
              </div>
              <div>
                <div className="text-sm text-gray-600">Vulnerabilities</div>
                <div className="font-medium text-gray-900">
                  {pc.vulnerabilities.length} 
                  {pc.vulnerabilities.filter(v => v.severity === 'critical').length > 0 && (
                    <span className="text-red-600 ml-1">
                      ({pc.vulnerabilities.filter(v => v.severity === 'critical').length} critical)
                    </span>
                  )}
                </div>
              </div>
              <div>
                <div className="text-sm text-gray-600">Pending Patches</div>
                <div className="font-medium text-gray-900">
                  {pc.patchingStatus.pendingPatches.length}
                  {pc.patchingStatus.rebootRequired && (
                    <span className="text-orange-600 ml-1">(reboot req.)</span>
                  )}
                </div>
              </div>
              <div>
                <div className="text-sm text-gray-600">Last Seen</div>
                <div className="font-medium text-gray-900">
                  {pc.lastSeen.toLocaleTimeString()}
                </div>
              </div>
            </div>

            {/* Security Features Status */}
            <div className="grid grid-cols-2 gap-6 mb-4">
              <div>
                <div className="text-sm font-medium text-gray-900 mb-2">Security Features</div>
                <div className="flex flex-wrap gap-1">
                  <Badge className={pc.edrStatus.threatDetection ? 'bg-green-100 text-green-800' : 'bg-red-100 text-red-800'} size="sm">
                    {pc.edrStatus.threatDetection ? 'EDR Active' : 'EDR Inactive'}
                  </Badge>
                  <Badge className={pc.usbPolicy.enabled ? 'bg-green-100 text-green-800' : 'bg-red-100 text-red-800'} size="sm">
                    USB {pc.usbPolicy.enabled ? 'Locked' : 'Open'}
                  </Badge>
                  <Badge className={pc.networkSegmentation.enabled ? 'bg-green-100 text-green-800' : 'bg-red-100 text-red-800'} size="sm">
                    Network {pc.networkSegmentation.enabled ? 'Segmented' : 'Open'}
                  </Badge>
                  <Badge className={pc.hardwareInfo.tpmEnabled ? 'bg-green-100 text-green-800' : 'bg-red-100 text-red-800'} size="sm">
                    TPM {pc.hardwareInfo.tpmEnabled ? 'Enabled' : 'Disabled'}
                  </Badge>
                  <Badge className={pc.hardwareInfo.secureBootEnabled ? 'bg-green-100 text-green-800' : 'bg-red-100 text-red-800'} size="sm">
                    Secure Boot {pc.hardwareInfo.secureBootEnabled ? 'On' : 'Off'}
                  </Badge>
                </div>
              </div>
              <div>
                <div className="text-sm font-medium text-gray-900 mb-2">Recent Activity</div>
                <div className="text-sm text-gray-600">
                  {pc.securityEvents.length > 0 ? (
                    <>
                      Latest: {pc.securityEvents[0].description.substring(0, 50)}...
                      <br />
                      <span className="text-xs">
                        {pc.securityEvents[0].timestamp.toLocaleString()}
                      </span>
                    </>
                  ) : (
                    'No recent security events'
                  )}
                </div>
              </div>
            </div>

            <div className="flex items-center justify-between pt-4 border-t">
              <div className="text-sm text-gray-600">
                Profile: {pc.securityProfile.name} • 
                Policies: {pc.securityProfile.policies.filter(p => p.compliant).length}/{pc.securityProfile.policies.length} compliant
              </div>
              <div className="flex items-center space-x-2">
                <Button 
                  variant="outline" 
                  size="sm"
                  onClick={(e) => {
                    e.stopPropagation()
                    handleSecurityScan(pc.id)
                  }}
                  disabled={isScanning}
                >
                  {isScanning ? (
                    <>
                      <Activity className="w-4 h-4 mr-1 animate-spin" />
                      Scanning... {Math.round(scanProgress)}%
                    </>
                  ) : (
                    <>
                      <Bug className="w-4 h-4 mr-1" />
                      Security Scan
                    </>
                  )}
                </Button>
                <Button variant="outline" size="sm">
                  <Eye className="w-4 h-4 mr-1" />
                  View Details
                </Button>
              </div>
            </div>
          </Card>
        ))}

        {filteredPCs.length === 0 && (
          <div className="text-center py-12">
            <Monitor className="w-16 h-16 text-gray-400 mx-auto mb-4" />
            <h3 className="text-lg font-medium text-gray-900 mb-2">No Garage PCs Found</h3>
            <p className="text-gray-600">
              {filters.search || filters.status || filters.location || filters.securityLevel
                ? 'No garage PCs match your current filters'
                : 'No garage PCs configured'
              }
            </p>
          </div>
        )}
      </div>

      {/* PC Details Dialog */}
      <Dialog open={showDetailsDialog} onOpenChange={setShowDetailsDialog}>
        <DialogContent className="max-w-6xl max-h-[90vh] overflow-y-auto">
          <DialogHeader>
            <DialogTitle>Garage PC Security Details: {selectedPC?.name}</DialogTitle>
          </DialogHeader>

          {selectedPC && (
            <Tabs value={activeTab} onValueChange={setActiveTab}>
              <TabsList>
                <TabsTrigger value="overview">Overview</TabsTrigger>
                <TabsTrigger value="patching">Patching</TabsTrigger>
                <TabsTrigger value="edr">EDR & Threats</TabsTrigger>
                <TabsTrigger value="usb">USB Policy</TabsTrigger>
                <TabsTrigger value="network">Network Security</TabsTrigger>
                <TabsTrigger value="compliance">Compliance</TabsTrigger>
              </TabsList>

              <TabsContent value="overview" className="space-y-4">
                <div className="grid grid-cols-2 gap-6">
                  <Card className="p-4">
                    <h3 className="text-lg font-medium text-gray-900 mb-3">System Information</h3>
                    <div className="space-y-2 text-sm">
                      <div className="flex justify-between">
                        <span className="text-gray-600">Name:</span>
                        <span className="font-medium">{selectedPC.name}</span>
                      </div>
                      <div className="flex justify-between">
                        <span className="text-gray-600">Location:</span>
                        <span className="font-medium">{selectedPC.location}</span>
                      </div>
                      <div className="flex justify-between">
                        <span className="text-gray-600">IP Address:</span>
                        <span className="font-medium font-mono">{selectedPC.ipAddress}</span>
                      </div>
                      <div className="flex justify-between">
                        <span className="text-gray-600">OS Version:</span>
                        <span className="font-medium">{selectedPC.osVersion}</span>
                      </div>
                      <div className="flex justify-between">
                        <span className="text-gray-600">Status:</span>
                        <Badge className={getStatusColor(selectedPC.status)}>
                          {selectedPC.status}
                        </Badge>
                      </div>
                    </div>
                  </Card>

                  <Card className="p-4">
                    <h3 className="text-lg font-medium text-gray-900 mb-3">Hardware</h3>
                    <div className="space-y-2 text-sm">
                      <div className="flex justify-between">
                        <span className="text-gray-600">Manufacturer:</span>
                        <span className="font-medium">{selectedPC.hardwareInfo.manufacturer}</span>
                      </div>
                      <div className="flex justify-between">
                        <span className="text-gray-600">Model:</span>
                        <span className="font-medium">{selectedPC.hardwareInfo.model}</span>
                      </div>
                      <div className="flex justify-between">
                        <span className="text-gray-600">CPU:</span>
                        <span className="font-medium">{selectedPC.hardwareInfo.cpu}</span>
                      </div>
                      <div className="flex justify-between">
                        <span className="text-gray-600">Memory:</span>
                        <span className="font-medium">{selectedPC.hardwareInfo.memory}</span>
                      </div>
                      <div className="flex justify-between">
                        <span className="text-gray-600">TPM:</span>
                        <Badge className={selectedPC.hardwareInfo.tpmEnabled ? 'bg-green-100 text-green-800' : 'bg-red-100 text-red-800'} size="sm">
                          {selectedPC.hardwareInfo.tpmEnabled ? 'Enabled' : 'Disabled'}
                        </Badge>
                      </div>
                    </div>
                  </Card>
                </div>

                <Card className="p-4">
                  <h3 className="text-lg font-medium text-gray-900 mb-3">Security Policies</h3>
                  <div className="space-y-3">
                    {selectedPC.securityProfile.policies.map(policy => (
                      <div key={policy.id} className="flex items-center justify-between p-3 border rounded">
                        <div className="flex items-center space-x-3">
                          <div className={`p-1 rounded ${policy.compliant ? 'bg-green-100' : 'bg-red-100'}`}>
                            {policy.compliant ? (
                              <CheckCircle className="w-4 h-4 text-green-600" />
                            ) : (
                              <XCircle className="w-4 h-4 text-red-600" />
                            )}
                          </div>
                          <div>
                            <div className="font-medium text-gray-900">{policy.name}</div>
                            <div className="text-sm text-gray-600">{policy.description}</div>
                            {!policy.compliant && policy.remediation && (
                              <div className="text-xs text-orange-600 mt-1">
                                Remediation: {policy.remediation}
                              </div>
                            )}
                          </div>
                        </div>
                        <div className="flex items-center space-x-2">
                          <Badge className={policy.enabled ? 'bg-green-100 text-green-800' : 'bg-gray-100 text-gray-800'} size="sm">
                            {policy.enabled ? 'Enabled' : 'Disabled'}
                          </Badge>
                          <Switch
                            checked={policy.enabled}
                            onCheckedChange={(enabled) => handlePolicyUpdate(selectedPC.id, policy.id, enabled)}
                            size="sm"
                          />
                        </div>
                      </div>
                    ))}
                  </div>
                </Card>
              </TabsContent>

              <TabsContent value="patching" className="space-y-4">
                <div className="grid grid-cols-2 gap-6">
                  <Card className="p-4">
                    <h3 className="text-lg font-medium text-gray-900 mb-3">Patching Status</h3>
                    <div className="space-y-2 text-sm">
                      <div className="flex justify-between">
                        <span className="text-gray-600">Last Update:</span>
                        <span className="font-medium">{selectedPC.patchingStatus.lastUpdate.toLocaleDateString()}</span>
                      </div>
                      <div className="flex justify-between">
                        <span className="text-gray-600">Next Scheduled:</span>
                        <span className="font-medium">{selectedPC.patchingStatus.nextScheduledUpdate.toLocaleDateString()}</span>
                      </div>
                      <div className="flex justify-between">
                        <span className="text-gray-600">Auto Update:</span>
                        <Badge className={selectedPC.patchingStatus.autoUpdateEnabled ? 'bg-green-100 text-green-800' : 'bg-red-100 text-red-800'} size="sm">
                          {selectedPC.patchingStatus.autoUpdateEnabled ? 'Enabled' : 'Disabled'}
                        </Badge>
                      </div>
                      <div className="flex justify-between">
                        <span className="text-gray-600">Policy:</span>
                        <span className="font-medium capitalize">{selectedPC.patchingStatus.patchingPolicy}</span>
                      </div>
                      <div className="flex justify-between">
                        <span className="text-gray-600">Reboot Required:</span>
                        <Badge className={selectedPC.patchingStatus.rebootRequired ? 'bg-orange-100 text-orange-800' : 'bg-green-100 text-green-800'} size="sm">
                          {selectedPC.patchingStatus.rebootRequired ? 'Yes' : 'No'}
                        </Badge>
                      </div>
                    </div>
                  </Card>

                  <Card className="p-4">
                    <h3 className="text-lg font-medium text-gray-900 mb-3">Maintenance Window</h3>
                    <div className="space-y-2 text-sm">
                      <div className="flex justify-between">
                        <span className="text-gray-600">Enabled:</span>
                        <Badge className={selectedPC.patchingStatus.maintenanceWindow.enabled ? 'bg-green-100 text-green-800' : 'bg-gray-100 text-gray-800'} size="sm">
                          {selectedPC.patchingStatus.maintenanceWindow.enabled ? 'Yes' : 'No'}
                        </Badge>
                      </div>
                      <div className="flex justify-between">
                        <span className="text-gray-600">Day:</span>
                        <span className="font-medium capitalize">{selectedPC.patchingStatus.maintenanceWindow.dayOfWeek}</span>
                      </div>
                      <div className="flex justify-between">
                        <span className="text-gray-600">Time:</span>
                        <span className="font-medium">
                          {selectedPC.patchingStatus.maintenanceWindow.startTime} - {selectedPC.patchingStatus.maintenanceWindow.endTime}
                        </span>
                      </div>
                      <div className="flex justify-between">
                        <span className="text-gray-600">Timezone:</span>
                        <span className="font-medium">{selectedPC.patchingStatus.maintenanceWindow.timezone}</span>
                      </div>
                      <div className="flex justify-between">
                        <span className="text-gray-600">Emergency Patches:</span>
                        <Badge className={selectedPC.patchingStatus.maintenanceWindow.allowEmergencyPatches ? 'bg-green-100 text-green-800' : 'bg-red-100 text-red-800'} size="sm">
                          {selectedPC.patchingStatus.maintenanceWindow.allowEmergencyPatches ? 'Allowed' : 'Blocked'}
                        </Badge>
                      </div>
                    </div>
                  </Card>
                </div>

                <Card className="p-4">
                  <h3 className="text-lg font-medium text-gray-900 mb-3">Pending Patches ({selectedPC.patchingStatus.pendingPatches.length})</h3>
                  <div className="space-y-3">
                    {selectedPC.patchingStatus.pendingPatches.map(patch => (
                      <div key={patch.id} className="flex items-center justify-between p-3 border rounded">
                        <div className="flex items-center space-x-3">
                          <Badge className={getSeverityColor(patch.severity)}>
                            {patch.severity}
                          </Badge>
                          <div>
                            <div className="font-medium text-gray-900">{patch.title}</div>
                            <div className="text-sm text-gray-600">{patch.description}</div>
                            <div className="text-xs text-gray-500 mt-1">
                              KB: {patch.kbNumber} • Size: {formatBytes(patch.size)} • Released: {patch.releaseDate.toLocaleDateString()}
                            </div>
                          </div>
                        </div>
                        <div className="flex items-center space-x-2">
                          {patch.requiresReboot && (
                            <Badge className="bg-orange-100 text-orange-800" size="sm">Reboot Required</Badge>
                          )}
                          <Button variant="outline" size="sm">
                            Install
                          </Button>
                        </div>
                      </div>
                    ))}
                  </div>
                </Card>
              </TabsContent>

              <TabsContent value="edr" className="space-y-4">
                <div className="grid grid-cols-2 gap-6">
                  <Card className="p-4">
                    <h3 className="text-lg font-medium text-gray-900 mb-3">EDR Agent Status</h3>
                    <div className="space-y-2 text-sm">
                      <div className="flex justify-between">
                        <span className="text-gray-600">Agent:</span>
                        <span className="font-medium">{selectedPC.edrStatus.agent}</span>
                      </div>
                      <div className="flex justify-between">
                        <span className="text-gray-600">Version:</span>
                        <span className="font-medium">{selectedPC.edrStatus.version}</span>
                      </div>
                      <div className="flex justify-between">
                        <span className="text-gray-600">Status:</span>
                        <Badge className={selectedPC.edrStatus.status === 'running' ? 'bg-green-100 text-green-800' : 'bg-red-100 text-red-800'}>
                          {selectedPC.edrStatus.status}
                        </Badge>
                      </div>
                      <div className="flex justify-between">
                        <span className="text-gray-600">Last Update:</span>
                        <span className="font-medium">{selectedPC.edrStatus.lastUpdate.toLocaleDateString()}</span>
                      </div>
                    </div>
                  </Card>

                  <Card className="p-4">
                    <h3 className="text-lg font-medium text-gray-900 mb-3">Protection Features</h3>
                    <div className="space-y-2 text-sm">
                      <div className="flex justify-between">
                        <span className="text-gray-600">Threat Detection:</span>
                        <Badge className={selectedPC.edrStatus.threatDetection ? 'bg-green-100 text-green-800' : 'bg-red-100 text-red-800'} size="sm">
                          {selectedPC.edrStatus.threatDetection ? 'Active' : 'Inactive'}
                        </Badge>
                      </div>
                      <div className="flex justify-between">
                        <span className="text-gray-600">Behavior Analysis:</span>
                        <Badge className={selectedPC.edrStatus.behaviorAnalysis ? 'bg-green-100 text-green-800' : 'bg-red-100 text-red-800'} size="sm">
                          {selectedPC.edrStatus.behaviorAnalysis ? 'Active' : 'Inactive'}
                        </Badge>
                      </div>
                      <div className="flex justify-between">
                        <span className="text-gray-600">Network Monitoring:</span>
                        <Badge className={selectedPC.edrStatus.networkMonitoring ? 'bg-green-100 text-green-800' : 'bg-red-100 text-red-800'} size="sm">
                          {selectedPC.edrStatus.networkMonitoring ? 'Active' : 'Inactive'}
                        </Badge>
                      </div>
                      <div className="flex justify-between">
                        <span className="text-gray-600">File Integrity:</span>
                        <Badge className={selectedPC.edrStatus.fileIntegrityMonitoring ? 'bg-green-100 text-green-800' : 'bg-red-100 text-red-800'} size="sm">
                          {selectedPC.edrStatus.fileIntegrityMonitoring ? 'Active' : 'Inactive'}
                        </Badge>
                      </div>
                    </div>
                  </Card>
                </div>

                <Card className="p-4">
                  <h3 className="text-lg font-medium text-gray-900 mb-3">Detected Threats ({selectedPC.edrStatus.detectedThreats.length})</h3>
                  <div className="space-y-3">
                    {selectedPC.edrStatus.detectedThreats.map(threat => (
                      <div key={threat.id} className="flex items-center justify-between p-3 border rounded">
                        <div className="flex items-center space-x-3">
                          <Badge className={getSeverityColor(threat.severity)}>
                            {threat.severity}
                          </Badge>
                          <div>
                            <div className="font-medium text-gray-900">{threat.type.replace('_', ' ')}</div>
                            <div className="text-sm text-gray-600">{threat.description}</div>
                            <div className="text-xs text-gray-500 mt-1">
                              Source: {threat.source} • Detected: {threat.detectionTime.toLocaleString()}
                            </div>
                            {threat.mitigation && (
                              <div className="text-xs text-green-600 mt-1">
                                Mitigation: {threat.mitigation}
                              </div>
                            )}
                          </div>
                        </div>
                        <Badge className={
                          threat.status === 'active' ? 'bg-red-100 text-red-800' :
                          threat.status === 'mitigated' ? 'bg-green-100 text-green-800' :
                          'bg-yellow-100 text-yellow-800'
                        }>
                          {threat.status.replace('_', ' ')}
                        </Badge>
                      </div>
                    ))}
                  </div>
                </Card>

                <Card className="p-4">
                  <h3 className="text-lg font-medium text-gray-900 mb-3">Quarantined Files ({selectedPC.edrStatus.quarantinedFiles.length})</h3>
                  <div className="space-y-3">
                    {selectedPC.edrStatus.quarantinedFiles.map(file => (
                      <div key={file.id} className="flex items-center justify-between p-3 border rounded">
                        <div className="flex items-center space-x-3">
                          <Badge className={getSeverityColor(file.riskLevel)}>
                            {file.riskLevel}
                          </Badge>
                          <div>
                            <div className="font-medium text-gray-900 font-mono text-sm">{file.filePath}</div>
                            <div className="text-sm text-gray-600">{file.threatType}</div>
                            <div className="text-xs text-gray-500 mt-1">
                              Detected: {file.detectionTime.toLocaleString()} • Quarantined: {file.quarantineTime.toLocaleString()}
                            </div>
                          </div>
                        </div>
                        <div className="flex items-center space-x-2">
                          <Badge className={
                            file.status === 'quarantined' ? 'bg-orange-100 text-orange-800' :
                            file.status === 'restored' ? 'bg-green-100 text-green-800' :
                            'bg-red-100 text-red-800'
                          } size="sm">
                            {file.status}
                          </Badge>
                          <Button variant="outline" size="sm">
                            Actions
                          </Button>
                        </div>
                      </div>
                    ))}
                  </div>
                </Card>
              </TabsContent>

              <TabsContent value="usb" className="space-y-4">
                <div className="grid grid-cols-2 gap-6">
                  <Card className="p-4">
                    <h3 className="text-lg font-medium text-gray-900 mb-3">USB Policy Configuration</h3>
                    <div className="space-y-4">
                      <div>
                        <Label htmlFor="usb-policy-mode">Policy Mode</Label>
                        <Select
                          value={selectedPC.usbPolicy.mode}
                          onValueChange={(value: USBPolicy['mode']) => handleUSBPolicyUpdate(selectedPC.id, value)}
                        >
                          <option value="block_all">Block All Devices</option>
                          <option value="allow_read_only">Allow Read-Only</option>
                          <option value="allow_specific">Allow Specific Devices</option>
                          <option value="monitor_only">Monitor Only</option>
                        </Select>
                      </div>
                      
                      <div className="space-y-2">
                        <Label>Monitoring Options</Label>
                        <div className="space-y-2">
                          <div className="flex items-center space-x-2">
                            <Checkbox checked={selectedPC.usbPolicy.monitoring.logConnections} />
                            <Label className="text-sm">Log Connections</Label>
                          </div>
                          <div className="flex items-center space-x-2">
                            <Checkbox checked={selectedPC.usbPolicy.monitoring.logFileTransfers} />
                            <Label className="text-sm">Log File Transfers</Label>
                          </div>
                          <div className="flex items-center space-x-2">
                            <Checkbox checked={selectedPC.usbPolicy.monitoring.alertOnUnknownDevice} />
                            <Label className="text-sm">Alert on Unknown Device</Label>
                          </div>
                          <div className="flex items-center space-x-2">
                            <Checkbox checked={selectedPC.usbPolicy.monitoring.quarantineUnknownFiles} />
                            <Label className="text-sm">Quarantine Unknown Files</Label>
                          </div>
                        </div>
                      </div>
                    </div>
                  </Card>

                  <Card className="p-4">
                    <h3 className="text-lg font-medium text-gray-900 mb-3">USB Port Status</h3>
                    <div className="space-y-3">
                      {selectedPC.hardwareInfo.usbPorts.map(port => (
                        <div key={port.port} className="flex items-center justify-between p-2 border rounded">
                          <div className="flex items-center space-x-2">
                            <Usb className="w-4 h-4 text-gray-500" />
                            <div>
                              <div className="text-sm font-medium">{port.port}</div>
                              {port.connectedDevice && (
                                <div className="text-xs text-gray-600">{port.connectedDevice}</div>
                              )}
                            </div>
                          </div>
                          <div className="flex items-center space-x-2">
                            <Badge className={
                              port.status === 'active' ? 'bg-green-100 text-green-800' : 'bg-gray-100 text-gray-800'
                            } size="sm">
                              {port.status}
                            </Badge>
                            <Badge className={
                              port.policy === 'allowed' ? 'bg-green-100 text-green-800' :
                              port.policy === 'blocked' ? 'bg-red-100 text-red-800' :
                              'bg-yellow-100 text-yellow-800'
                            } size="sm">
                              {port.policy}
                            </Badge>
                          </div>
                        </div>
                      ))}
                    </div>
                  </Card>
                </div>

                <Card className="p-4">
                  <h3 className="text-lg font-medium text-gray-900 mb-3">Allowed Devices ({selectedPC.usbPolicy.allowedDevices.length})</h3>
                  <div className="space-y-3">
                    {selectedPC.usbPolicy.allowedDevices.map(device => (
                      <div key={device.id} className="flex items-center justify-between p-3 border rounded">
                        <div className="flex items-center space-x-3">
                          <Usb className="w-4 h-4 text-green-600" />
                          <div>
                            <div className="font-medium text-gray-900">{device.description}</div>
                            <div className="text-sm text-gray-600 font-mono">
                              VID: {device.vendorId} • PID: {device.productId}
                              {device.serialNumber && ` • SN: ${device.serialNumber}`}
                            </div>
                            <div className="text-xs text-gray-500 mt-1">
                              Approved: {device.approvedAt.toLocaleDateString()} by {device.approvedBy}
                              {device.expiresAt && ` • Expires: ${device.expiresAt.toLocaleDateString()}`}
                            </div>
                          </div>
                        </div>
                        <div className="flex items-center space-x-2">
                          <Badge className={device.permissions === 'read_write' ? 'bg-orange-100 text-orange-800' : 'bg-green-100 text-green-800'} size="sm">
                            {device.permissions.replace('_', ' ')}
                          </Badge>
                          <Button variant="outline" size="sm">
                            <XCircle className="w-3 h-3" />
                          </Button>
                        </div>
                      </div>
                    ))}
                  </div>
                </Card>

                {selectedPC.usbPolicy.violations.length > 0 && (
                  <Card className="p-4">
                    <h3 className="text-lg font-medium text-gray-900 mb-3">Recent Violations ({selectedPC.usbPolicy.violations.length})</h3>
                    <div className="space-y-3">
                      {selectedPC.usbPolicy.violations.map(violation => (
                        <div key={violation.id} className="flex items-center justify-between p-3 border rounded">
                          <div className="flex items-center space-x-3">
                            <AlertTriangle className="w-4 h-4 text-red-600" />
                            <div>
                              <div className="font-medium text-gray-900">{violation.violationType.replace('_', ' ')}</div>
                              <div className="text-sm text-gray-600">{violation.description}</div>
                              <div className="text-xs text-gray-500 mt-1">
                                Device: {violation.deviceId} • User: {violation.user} • {violation.detectionTime.toLocaleString()}
                              </div>
                            </div>
                          </div>
                          <Badge className={
                            violation.action === 'blocked' ? 'bg-red-100 text-red-800' :
                            violation.action === 'quarantined' ? 'bg-orange-100 text-orange-800' :
                            'bg-yellow-100 text-yellow-800'
                          }>
                            {violation.action.replace('_', ' ')}
                          </Badge>
                        </div>
                      ))}
                    </div>
                  </Card>
                )}
              </TabsContent>

              <TabsContent value="network" className="space-y-4">
                <div className="grid grid-cols-2 gap-6">
                  <Card className="p-4">
                    <h3 className="text-lg font-medium text-gray-900 mb-3">Network Segmentation</h3>
                    <div className="space-y-2 text-sm">
                      <div className="flex justify-between">
                        <span className="text-gray-600">Enabled:</span>
                        <Badge className={selectedPC.networkSegmentation.enabled ? 'bg-green-100 text-green-800' : 'bg-red-100 text-red-800'} size="sm">
                          {selectedPC.networkSegmentation.enabled ? 'Yes' : 'No'}
                        </Badge>
                      </div>
                      <div className="flex justify-between">
                        <span className="text-gray-600">VLAN:</span>
                        <span className="font-medium font-mono">{selectedPC.networkSegmentation.vlan}</span>
                      </div>
                      <div className="flex justify-between">
                        <span className="text-gray-600">Subnet:</span>
                        <span className="font-medium font-mono">{selectedPC.networkSegmentation.subnet}</span>
                      </div>
                      <div className="flex justify-between">
                        <span className="text-gray-600">Firewall Rules:</span>
                        <span className="font-medium">{selectedPC.networkSegmentation.firewallRules.length}</span>
                      </div>
                    </div>
                  </Card>

                  <Card className="p-4">
                    <h3 className="text-lg font-medium text-gray-900 mb-3">Monitoring</h3>
                    <div className="space-y-2 text-sm">
                      <div className="flex justify-between">
                        <span className="text-gray-600">Connection Logging:</span>
                        <Badge className={selectedPC.networkSegmentation.monitoring.logConnections ? 'bg-green-100 text-green-800' : 'bg-gray-100 text-gray-800'} size="sm">
                          {selectedPC.networkSegmentation.monitoring.logConnections ? 'On' : 'Off'}
                        </Badge>
                      </div>
                      <div className="flex justify-between">
                        <span className="text-gray-600">Anomaly Detection:</span>
                        <Badge className={selectedPC.networkSegmentation.monitoring.detectAnomalies ? 'bg-green-100 text-green-800' : 'bg-gray-100 text-gray-800'} size="sm">
                          {selectedPC.networkSegmentation.monitoring.detectAnomalies ? 'On' : 'Off'}
                        </Badge>
                      </div>
                      <div className="flex justify-between">
                        <span className="text-gray-600">Unauthorized Alerts:</span>
                        <Badge className={selectedPC.networkSegmentation.monitoring.alertOnUnauthorized ? 'bg-green-100 text-green-800' : 'bg-gray-100 text-gray-800'} size="sm">
                          {selectedPC.networkSegmentation.monitoring.alertOnUnauthorized ? 'On' : 'Off'}
                        </Badge>
                      </div>
                      <div className="flex justify-between">
                        <span className="text-gray-600">Bandwidth Monitoring:</span>
                        <Badge className={selectedPC.networkSegmentation.monitoring.bandwidthMonitoring ? 'bg-green-100 text-green-800' : 'bg-gray-100 text-gray-800'} size="sm">
                          {selectedPC.networkSegmentation.monitoring.bandwidthMonitoring ? 'On' : 'Off'}
                        </Badge>
                      </div>
                    </div>
                  </Card>
                </div>

                <Card className="p-4">
                  <h3 className="text-lg font-medium text-gray-900 mb-3">Firewall Rules</h3>
                  <div className="overflow-x-auto">
                    <Table>
                      <TableHeader>
                        <TableRow>
                          <TableHead>Name</TableHead>
                          <TableHead>Direction</TableHead>
                          <TableHead>Action</TableHead>
                          <TableHead>Protocol</TableHead>
                          <TableHead>Source</TableHead>
                          <TableHead>Destination</TableHead>
                          <TableHead>Hits</TableHead>
                          <TableHead>Status</TableHead>
                        </TableRow>
                      </TableHeader>
                      <TableBody>
                        {selectedPC.networkSegmentation.firewallRules.map(rule => (
                          <TableRow key={rule.id}>
                            <TableCell className="font-medium">{rule.name}</TableCell>
                            <TableCell>{rule.direction}</TableCell>
                            <TableCell>
                              <Badge className={rule.action === 'allow' ? 'bg-green-100 text-green-800' : 'bg-red-100 text-red-800'} size="sm">
                                {rule.action}
                              </Badge>
                            </TableCell>
                            <TableCell className="font-mono text-sm">{rule.protocol.toUpperCase()}</TableCell>
                            <TableCell className="font-mono text-sm">{rule.sourceIP}:{rule.sourcePort}</TableCell>
                            <TableCell className="font-mono text-sm">{rule.destinationIP}:{rule.destinationPort}</TableCell>
                            <TableCell>{rule.hitCount.toLocaleString()}</TableCell>
                            <TableCell>
                              <Badge className={rule.enabled ? 'bg-green-100 text-green-800' : 'bg-gray-100 text-gray-800'} size="sm">
                                {rule.enabled ? 'Enabled' : 'Disabled'}
                              </Badge>
                            </TableCell>
                          </TableRow>
                        ))}
                      </TableBody>
                    </Table>
                  </div>
                </Card>

                <Card className="p-4">
                  <h3 className="text-lg font-medium text-gray-900 mb-3">Active Network Connections</h3>
                  <div className="space-y-2">
                    {selectedPC.networkConnections.map((conn, index) => (
                      <div key={index} className="flex items-center justify-between p-2 bg-gray-50 rounded text-sm">
                        <div className="font-mono">
                          {conn.localAddress}:{conn.localPort} ↔ {conn.remoteAddress}:{conn.remotePort}
                        </div>
                        <div className="flex items-center space-x-2">
                          <span className="text-gray-600">{conn.protocol}</span>
                          <Badge className="bg-green-100 text-green-800" size="sm">{conn.state}</Badge>
                          <span className="text-gray-600">{conn.process}</span>
                        </div>
                      </div>
                    ))}
                  </div>
                </Card>
              </TabsContent>

              <TabsContent value="compliance" className="space-y-4">
                <Card className="p-4">
                  <h3 className="text-lg font-medium text-gray-900 mb-3">
                    Compliance Score: {selectedPC.complianceScore}%
                  </h3>
                  <Progress value={selectedPC.complianceScore} className="w-full h-4 mb-4" />
                  
                  <div className="grid grid-cols-2 gap-6">
                    <div>
                      <h4 className="font-medium text-gray-900 mb-2">Security Controls</h4>
                      <div className="space-y-1 text-sm">
                        <div className="flex justify-between">
                          <span>OS Hardening:</span>
                          <Badge className="bg-green-100 text-green-800" size="sm">Compliant</Badge>
                        </div>
                        <div className="flex justify-between">
                          <span>Patch Management:</span>
                          <Badge className="bg-yellow-100 text-yellow-800" size="sm">Partial</Badge>
                        </div>
                        <div className="flex justify-between">
                          <span>EDR Protection:</span>
                          <Badge className="bg-green-100 text-green-800" size="sm">Compliant</Badge>
                        </div>
                        <div className="flex justify-between">
                          <span>USB Controls:</span>
                          <Badge className="bg-green-100 text-green-800" size="sm">Compliant</Badge>
                        </div>
                        <div className="flex justify-between">
                          <span>Network Segmentation:</span>
                          <Badge className="bg-green-100 text-green-800" size="sm">Compliant</Badge>
                        </div>
                      </div>
                    </div>
                    
                    <div>
                      <h4 className="font-medium text-gray-900 mb-2">Hardware Security</h4>
                      <div className="space-y-1 text-sm">
                        <div className="flex justify-between">
                          <span>TPM Enabled:</span>
                          <Badge className="bg-green-100 text-green-800" size="sm">Yes</Badge>
                        </div>
                        <div className="flex justify-between">
                          <span>Secure Boot:</span>
                          <Badge className="bg-green-100 text-green-800" size="sm">Enabled</Badge>
                        </div>
                        <div className="flex justify-between">
                          <span>Disk Encryption:</span>
                          <Badge className="bg-green-100 text-green-800" size="sm">Active</Badge>
                        </div>
                        <div className="flex justify-between">
                          <span>BIOS Protection:</span>
                          <Badge className="bg-green-100 text-green-800" size="sm">Enabled</Badge>
                        </div>
                      </div>
                    </div>
                  </div>
                </Card>

                <Card className="p-4">
                  <h3 className="text-lg font-medium text-gray-900 mb-3">Vulnerabilities ({selectedPC.vulnerabilities.length})</h3>
                  <div className="space-y-3">
                    {selectedPC.vulnerabilities.map(vuln => (
                      <div key={vuln.id} className="flex items-center justify-between p-3 border rounded">
                        <div className="flex items-center space-x-3">
                          <Badge className={getSeverityColor(vuln.severity)}>
                            {vuln.severity}
                          </Badge>
                          <div>
                            <div className="font-medium text-gray-900">{vuln.title}</div>
                            <div className="text-sm text-gray-600">{vuln.description}</div>
                            <div className="text-xs text-gray-500 mt-1">
                              {vuln.cveId && `${vuln.cveId} • `}
                              CVSS: {vuln.cvssScore} • Component: {vuln.affectedComponent}
                            </div>
                            {vuln.remediation && (
                              <div className="text-xs text-blue-600 mt-1">
                                Remediation: {vuln.remediation}
                              </div>
                            )}
                          </div>
                        </div>
                        <div className="flex items-center space-x-2">
                          {vuln.patchAvailable && (
                            <Badge className="bg-green-100 text-green-800" size="sm">Patch Available</Badge>
                          )}
                          {vuln.exploitAvailable && (
                            <Badge className="bg-red-100 text-red-800" size="sm">Exploit Available</Badge>
                          )}
                          <Badge className={
                            vuln.status === 'open' ? 'bg-red-100 text-red-800' :
                            vuln.status === 'mitigated' ? 'bg-green-100 text-green-800' :
                            'bg-yellow-100 text-yellow-800'
                          }>
                            {vuln.status.replace('_', ' ')}
                          </Badge>
                        </div>
                      </div>
                    ))}
                  </div>
                </Card>
              </TabsContent>
            </Tabs>
          )}

          <DialogFooter>
            <Button variant="outline" onClick={() => setShowDetailsDialog(false)}>
              Close
            </Button>
            <Button>
              <Settings className="w-4 h-4 mr-2" />
              Configure Security
            </Button>
          </DialogFooter>
        </DialogContent>
      </Dialog>
    </div>
  )
}

export default GaragePCSecurityManager
