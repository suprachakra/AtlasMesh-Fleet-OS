import React, { useState, useCallback, useEffect, useMemo } from 'react'
import { 
  Shield, Users, Key, Lock, Unlock, Eye, EyeOff, AlertTriangle,
  CheckCircle, XCircle, Search, Filter, Settings, RefreshCw, FileText,
  Clock, User, UserCheck, UserX, Activity, Zap, History, Plus, Edit3,
  Trash2, Copy, Download, Upload, MoreHorizontal, Calendar, Globe
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
import { DropdownMenu, DropdownMenuContent, DropdownMenuItem, DropdownMenuTrigger } from '../ui/DropdownMenu'
import { Progress } from '../ui/Progress'

// Types
interface Role {
  id: string
  name: string
  displayName: string
  description: string
  type: 'system' | 'custom' | 'inherited'
  category: 'operational' | 'administrative' | 'technical' | 'safety' | 'compliance'
  permissions: Permission[]
  constraints: RoleConstraint[]
  inheritance: RoleInheritance[]
  users: string[]
  status: 'active' | 'deprecated' | 'disabled'
  riskLevel: 'low' | 'medium' | 'high' | 'critical'
  createdAt: Date
  updatedAt: Date
  lastUsed?: Date
  usageCount: number
  auditRequired: boolean
  dualAuthRequired: boolean
  sessionTimeout: number // minutes
  ipRestrictions: string[]
  timeRestrictions: TimeRestriction[]
  approvalWorkflow?: ApprovalWorkflow
}

interface Permission {
  id: string
  resource: string
  action: string
  effect: 'allow' | 'deny'
  conditions: PermissionCondition[]
  context: string[]
  priority: number
  description: string
  riskLevel: 'low' | 'medium' | 'high' | 'critical'
  auditRequired: boolean
  dualAuthRequired: boolean
  enabled: boolean
  expiresAt?: Date
  grantedAt: Date
  grantedBy: string
}

interface PermissionCondition {
  field: string
  operator: 'equals' | 'not_equals' | 'contains' | 'not_contains' | 'in' | 'not_in' | 'greater_than' | 'less_than' | 'exists' | 'not_exists'
  value: string | number | boolean | string[]
  description: string
}

interface RoleConstraint {
  type: 'time' | 'location' | 'device' | 'network' | 'concurrent_sessions' | 'custom'
  description: string
  parameters: Record<string, any>
  enabled: boolean
}

interface RoleInheritance {
  parentRoleId: string
  parentRoleName: string
  inheritedPermissions: string[]
  overriddenPermissions: string[]
}

interface TimeRestriction {
  days: string[] // ['monday', 'tuesday', etc.]
  startTime: string // HH:MM format
  endTime: string // HH:MM format
  timezone: string
  enabled: boolean
}

interface ApprovalWorkflow {
  required: boolean
  approvers: string[]
  minimumApprovals: number
  timeoutHours: number
  escalationRules: EscalationRule[]
}

interface EscalationRule {
  afterHours: number
  escalateTo: string[]
  notificationMethod: 'email' | 'sms' | 'push' | 'all'
}

interface User {
  id: string
  name: string
  email: string
  department: string
  roles: string[]
  status: 'active' | 'inactive' | 'suspended' | 'pending'
  lastLogin?: Date
  failedLogins: number
  mfaEnabled: boolean
  createdAt: Date
}

interface AuditEntry {
  id: string
  timestamp: Date
  userId: string
  userName: string
  action: string
  resource: string
  roleId?: string
  permissionId?: string
  outcome: 'success' | 'failure' | 'denied'
  reason?: string
  ipAddress: string
  userAgent: string
  riskScore: number
  requiresReview: boolean
  reviewedBy?: string
  reviewedAt?: Date
}

interface RBACManagerProps {
  userId?: string
  mode?: 'view' | 'configure' | 'audit'
  onRoleSelected?: (role: Role) => void
  onPermissionGranted?: (userId: string, permission: Permission) => void
  onAuditRequired?: (entry: AuditEntry) => void
  className?: string
}

const RBACManager: React.FC<RBACManagerProps> = ({
  userId,
  mode = 'view',
  onRoleSelected,
  onPermissionGranted,
  onAuditRequired,
  className = ''
}) => {
  // State
  const [roles, setRoles] = useState<Role[]>([])
  const [users, setUsers] = useState<User[]>([])
  const [auditEntries, setAuditEntries] = useState<AuditEntry[]>([])
  const [showRoleDialog, setShowRoleDialog] = useState(false)
  const [showUserDialog, setShowUserDialog] = useState(false)
  const [showPermissionDialog, setShowPermissionDialog] = useState(false)
  const [showAuditDialog, setShowAuditDialog] = useState(false)
  const [selectedRole, setSelectedRole] = useState<Role | null>(null)
  const [selectedUser, setSelectedUser] = useState<User | null>(null)
  const [selectedPermission, setSelectedPermission] = useState<Permission | null>(null)
  const [activeTab, setActiveTab] = useState('roles')
  const [filters, setFilters] = useState({
    category: '',
    riskLevel: '',
    status: '',
    search: '',
    type: ''
  })
  const [newRole, setNewRole] = useState<Partial<Role>>({})
  const [matrixView, setMatrixView] = useState(false)

  // Mock data initialization
  const initializeMockData = useCallback(() => {
    const mockRoles: Role[] = [
      {
        id: 'role-fleet-operator',
        name: 'fleet_operator',
        displayName: 'Fleet Operator',
        description: 'Operational control of fleet vehicles and basic monitoring capabilities',
        type: 'system',
        category: 'operational',
        permissions: [
          {
            id: 'perm-vehicle-control',
            resource: 'vehicle',
            action: 'control',
            effect: 'allow',
            conditions: [
              { field: 'vehicle.status', operator: 'equals', value: 'operational', description: 'Vehicle must be operational' },
              { field: 'vehicle.assigned_operator', operator: 'equals', value: '${user.id}', description: 'Must be assigned operator' }
            ],
            context: ['fleet_operations', 'emergency_response'],
            priority: 1,
            description: 'Control assigned vehicles (start/stop/navigate)',
            riskLevel: 'high',
            auditRequired: true,
            dualAuthRequired: false,
            enabled: true,
            grantedAt: new Date('2024-01-15'),
            grantedBy: 'system-admin'
          },
          {
            id: 'perm-telemetry-view',
            resource: 'telemetry',
            action: 'read',
            effect: 'allow',
            conditions: [
              { field: 'telemetry.vehicle_id', operator: 'in', value: '${user.assigned_vehicles}', description: 'Only assigned vehicles' }
            ],
            context: ['operations_dashboard', 'monitoring'],
            priority: 2,
            description: 'View telemetry data for assigned vehicles',
            riskLevel: 'low',
            auditRequired: false,
            dualAuthRequired: false,
            enabled: true,
            grantedAt: new Date('2024-01-15'),
            grantedBy: 'system-admin'
          },
          {
            id: 'perm-emergency-stop',
            resource: 'vehicle',
            action: 'emergency_stop',
            effect: 'allow',
            conditions: [],
            context: ['emergency_response'],
            priority: 1,
            description: 'Execute emergency stop on any vehicle',
            riskLevel: 'critical',
            auditRequired: true,
            dualAuthRequired: true,
            enabled: true,
            grantedAt: new Date('2024-01-15'),
            grantedBy: 'safety-manager'
          }
        ],
        constraints: [
          {
            type: 'time',
            description: 'Operational hours only',
            parameters: {
              days: ['monday', 'tuesday', 'wednesday', 'thursday', 'friday', 'saturday', 'sunday'],
              startTime: '06:00',
              endTime: '22:00',
              timezone: 'Asia/Dubai'
            },
            enabled: true
          },
          {
            type: 'concurrent_sessions',
            description: 'Single active session',
            parameters: {
              maxSessions: 1
            },
            enabled: true
          }
        ],
        inheritance: [],
        users: ['user-ahmed', 'user-fatima', 'user-omar'],
        status: 'active',
        riskLevel: 'high',
        createdAt: new Date('2024-01-15'),
        updatedAt: new Date('2024-11-20'),
        lastUsed: new Date('2024-11-26'),
        usageCount: 2847,
        auditRequired: true,
        dualAuthRequired: false,
        sessionTimeout: 480, // 8 hours
        ipRestrictions: ['192.168.1.0/24', '10.0.0.0/8'],
        timeRestrictions: [
          {
            days: ['monday', 'tuesday', 'wednesday', 'thursday', 'friday', 'saturday', 'sunday'],
            startTime: '06:00',
            endTime: '22:00',
            timezone: 'Asia/Dubai',
            enabled: true
          }
        ]
      },
      {
        id: 'role-safety-supervisor',
        name: 'safety_supervisor',
        displayName: 'Safety Supervisor',
        description: 'Safety oversight and emergency response coordination with elevated privileges',
        type: 'system',
        category: 'safety',
        permissions: [
          {
            id: 'perm-fleet-emergency-stop',
            resource: 'fleet',
            action: 'emergency_stop_all',
            effect: 'allow',
            conditions: [],
            context: ['emergency_response', 'safety_override'],
            priority: 1,
            description: 'Stop all vehicles in emergency situations',
            riskLevel: 'critical',
            auditRequired: true,
            dualAuthRequired: true,
            enabled: true,
            grantedAt: new Date('2024-01-15'),
            grantedBy: 'system-admin'
          },
          {
            id: 'perm-safety-policy-override',
            resource: 'policy',
            action: 'override',
            effect: 'allow',
            conditions: [
              { field: 'policy.category', operator: 'equals', value: 'safety', description: 'Safety policies only' },
              { field: 'emergency.level', operator: 'greater_than', value: 3, description: 'Emergency level 4+ only' }
            ],
            context: ['emergency_response'],
            priority: 1,
            description: 'Override safety policies in emergencies',
            riskLevel: 'critical',
            auditRequired: true,
            dualAuthRequired: true,
            enabled: true,
            grantedAt: new Date('2024-01-15'),
            grantedBy: 'system-admin'
          },
          {
            id: 'perm-incident-management',
            resource: 'incident',
            action: 'manage',
            effect: 'allow',
            conditions: [],
            context: ['incident_response', 'safety_investigation'],
            priority: 2,
            description: 'Create and manage safety incidents',
            riskLevel: 'medium',
            auditRequired: true,
            dualAuthRequired: false,
            enabled: true,
            grantedAt: new Date('2024-01-15'),
            grantedBy: 'system-admin'
          }
        ],
        constraints: [
          {
            type: 'device',
            description: 'Authorized devices only',
            parameters: {
              allowedDevices: ['certified_tablet', 'ops_workstation'],
              requireCertificate: true
            },
            enabled: true
          }
        ],
        inheritance: [
          {
            parentRoleId: 'role-fleet-operator',
            parentRoleName: 'Fleet Operator',
            inheritedPermissions: ['perm-telemetry-view'],
            overriddenPermissions: []
          }
        ],
        users: ['user-sarah', 'user-khalid'],
        status: 'active',
        riskLevel: 'critical',
        createdAt: new Date('2024-01-15'),
        updatedAt: new Date('2024-11-18'),
        lastUsed: new Date('2024-11-26'),
        usageCount: 892,
        auditRequired: true,
        dualAuthRequired: true,
        sessionTimeout: 240, // 4 hours
        ipRestrictions: ['192.168.1.0/24'],
        timeRestrictions: [
          {
            days: ['monday', 'tuesday', 'wednesday', 'thursday', 'friday', 'saturday', 'sunday'],
            startTime: '00:00',
            endTime: '23:59',
            timezone: 'Asia/Dubai',
            enabled: true
          }
        ],
        approvalWorkflow: {
          required: true,
          approvers: ['security-admin', 'safety-director'],
          minimumApprovals: 2,
          timeoutHours: 4,
          escalationRules: [
            {
              afterHours: 2,
              escalateTo: ['security-director'],
              notificationMethod: 'all'
            }
          ]
        }
      },
      {
        id: 'role-system-admin',
        name: 'system_admin',
        displayName: 'System Administrator',
        description: 'Full system administration with configuration and user management capabilities',
        type: 'system',
        category: 'administrative',
        permissions: [
          {
            id: 'perm-user-management',
            resource: 'user',
            action: 'manage',
            effect: 'allow',
            conditions: [
              { field: 'target_user.role', operator: 'not_equals', value: 'super_admin', description: 'Cannot manage super admins' }
            ],
            context: ['user_administration'],
            priority: 1,
            description: 'Create, modify, and delete user accounts',
            riskLevel: 'high',
            auditRequired: true,
            dualAuthRequired: true,
            enabled: true,
            grantedAt: new Date('2024-01-15'),
            grantedBy: 'super-admin'
          },
          {
            id: 'perm-role-management',
            resource: 'role',
            action: 'manage',
            effect: 'allow',
            conditions: [
              { field: 'role.type', operator: 'not_equals', value: 'system', description: 'Cannot modify system roles' }
            ],
            context: ['rbac_administration'],
            priority: 1,
            description: 'Create and modify custom roles',
            riskLevel: 'critical',
            auditRequired: true,
            dualAuthRequired: true,
            enabled: true,
            grantedAt: new Date('2024-01-15'),
            grantedBy: 'super-admin'
          },
          {
            id: 'perm-system-config',
            resource: 'system',
            action: 'configure',
            effect: 'allow',
            conditions: [],
            context: ['system_administration'],
            priority: 1,
            description: 'Modify system configuration',
            riskLevel: 'critical',
            auditRequired: true,
            dualAuthRequired: true,
            enabled: true,
            grantedAt: new Date('2024-01-15'),
            grantedBy: 'super-admin'
          }
        ],
        constraints: [
          {
            type: 'location',
            description: 'On-premise access only',
            parameters: {
              allowedLocations: ['headquarters', 'data_center'],
              requirePhysicalPresence: true
            },
            enabled: true
          },
          {
            type: 'network',
            description: 'Secure network only',
            parameters: {
              allowedNetworks: ['192.168.1.0/24'],
              requireVPN: true
            },
            enabled: true
          }
        ],
        inheritance: [],
        users: ['user-admin'],
        status: 'active',
        riskLevel: 'critical',
        createdAt: new Date('2024-01-15'),
        updatedAt: new Date('2024-11-22'),
        lastUsed: new Date('2024-11-26'),
        usageCount: 456,
        auditRequired: true,
        dualAuthRequired: true,
        sessionTimeout: 120, // 2 hours
        ipRestrictions: ['192.168.1.0/24'],
        timeRestrictions: [
          {
            days: ['monday', 'tuesday', 'wednesday', 'thursday', 'friday'],
            startTime: '08:00',
            endTime: '18:00',
            timezone: 'Asia/Dubai',
            enabled: true
          }
        ],
        approvalWorkflow: {
          required: true,
          approvers: ['security-director', 'cto'],
          minimumApprovals: 2,
          timeoutHours: 24,
          escalationRules: [
            {
              afterHours: 12,
              escalateTo: ['ceo'],
              notificationMethod: 'all'
            }
          ]
        }
      },
      {
        id: 'role-data-analyst',
        name: 'data_analyst',
        displayName: 'Data Analyst',
        description: 'Read-only access to anonymized data for business intelligence and reporting',
        type: 'custom',
        category: 'technical',
        permissions: [
          {
            id: 'perm-analytics-read',
            resource: 'analytics',
            action: 'read',
            effect: 'allow',
            conditions: [
              { field: 'data.anonymized', operator: 'equals', value: true, description: 'Only anonymized data' },
              { field: 'data.classification', operator: 'not_equals', value: 'restricted', description: 'No restricted data' }
            ],
            context: ['business_intelligence', 'reporting'],
            priority: 2,
            description: 'Access anonymized analytics data',
            riskLevel: 'low',
            auditRequired: false,
            dualAuthRequired: false,
            enabled: true,
            grantedAt: new Date('2024-02-01'),
            grantedBy: 'data-manager'
          },
          {
            id: 'perm-report-generate',
            resource: 'report',
            action: 'generate',
            effect: 'allow',
            conditions: [
              { field: 'report.type', operator: 'in', value: ['operational', 'performance', 'usage'], description: 'Standard report types only' }
            ],
            context: ['reporting'],
            priority: 2,
            description: 'Generate standard business reports',
            riskLevel: 'low',
            auditRequired: true,
            dualAuthRequired: false,
            enabled: true,
            grantedAt: new Date('2024-02-01'),
            grantedBy: 'data-manager'
          }
        ],
        constraints: [
          {
            type: 'time',
            description: 'Business hours only',
            parameters: {
              days: ['monday', 'tuesday', 'wednesday', 'thursday', 'friday'],
              startTime: '08:00',
              endTime: '18:00',
              timezone: 'Asia/Dubai'
            },
            enabled: true
          }
        ],
        inheritance: [],
        users: ['user-analyst1', 'user-analyst2'],
        status: 'active',
        riskLevel: 'low',
        createdAt: new Date('2024-02-01'),
        updatedAt: new Date('2024-11-10'),
        lastUsed: new Date('2024-11-25'),
        usageCount: 1234,
        auditRequired: false,
        dualAuthRequired: false,
        sessionTimeout: 480, // 8 hours
        ipRestrictions: [],
        timeRestrictions: [
          {
            days: ['monday', 'tuesday', 'wednesday', 'thursday', 'friday'],
            startTime: '08:00',
            endTime: '18:00',
            timezone: 'Asia/Dubai',
            enabled: true
          }
        ]
      },
      {
        id: 'role-compliance-officer',
        name: 'compliance_officer',
        displayName: 'Compliance Officer',
        description: 'Access to compliance data, audit trails, and regulatory reporting capabilities',
        type: 'system',
        category: 'compliance',
        permissions: [
          {
            id: 'perm-audit-access',
            resource: 'audit',
            action: 'read',
            effect: 'allow',
            conditions: [],
            context: ['compliance_monitoring', 'audit_review'],
            priority: 1,
            description: 'Access all audit logs and trails',
            riskLevel: 'medium',
            auditRequired: true,
            dualAuthRequired: false,
            enabled: true,
            grantedAt: new Date('2024-01-20'),
            grantedBy: 'system-admin'
          },
          {
            id: 'perm-compliance-report',
            resource: 'compliance',
            action: 'report',
            effect: 'allow',
            conditions: [],
            context: ['regulatory_reporting'],
            priority: 1,
            description: 'Generate compliance and regulatory reports',
            riskLevel: 'medium',
            auditRequired: true,
            dualAuthRequired: false,
            enabled: true,
            grantedAt: new Date('2024-01-20'),
            grantedBy: 'system-admin'
          },
          {
            id: 'perm-policy-review',
            resource: 'policy',
            action: 'review',
            effect: 'allow',
            conditions: [],
            context: ['compliance_review'],
            priority: 2,
            description: 'Review and assess policy compliance',
            riskLevel: 'low',
            auditRequired: false,
            dualAuthRequired: false,
            enabled: true,
            grantedAt: new Date('2024-01-20'),
            grantedBy: 'system-admin'
          }
        ],
        constraints: [],
        inheritance: [],
        users: ['user-compliance'],
        status: 'active',
        riskLevel: 'medium',
        createdAt: new Date('2024-01-20'),
        updatedAt: new Date('2024-11-15'),
        lastUsed: new Date('2024-11-24'),
        usageCount: 678,
        auditRequired: true,
        dualAuthRequired: false,
        sessionTimeout: 360, // 6 hours
        ipRestrictions: ['192.168.1.0/24'],
        timeRestrictions: [
          {
            days: ['monday', 'tuesday', 'wednesday', 'thursday', 'friday'],
            startTime: '08:00',
            endTime: '17:00',
            timezone: 'Asia/Dubai',
            enabled: true
          }
        ]
      }
    ]

    const mockUsers: User[] = [
      {
        id: 'user-ahmed',
        name: 'Ahmed Al-Mansouri',
        email: 'ahmed.mansouri@atlasmesh.com',
        department: 'Operations',
        roles: ['role-fleet-operator'],
        status: 'active',
        lastLogin: new Date('2024-11-26T08:30:00Z'),
        failedLogins: 0,
        mfaEnabled: true,
        createdAt: new Date('2024-01-15')
      },
      {
        id: 'user-fatima',
        name: 'Fatima Al-Zahra',
        email: 'fatima.zahra@atlasmesh.com',
        department: 'Operations',
        roles: ['role-fleet-operator'],
        status: 'active',
        lastLogin: new Date('2024-11-26T09:15:00Z'),
        failedLogins: 0,
        mfaEnabled: true,
        createdAt: new Date('2024-01-20')
      },
      {
        id: 'user-sarah',
        name: 'Sarah Johnson',
        email: 'sarah.johnson@atlasmesh.com',
        department: 'Safety',
        roles: ['role-safety-supervisor'],
        status: 'active',
        lastLogin: new Date('2024-11-26T07:45:00Z'),
        failedLogins: 0,
        mfaEnabled: true,
        createdAt: new Date('2024-01-15')
      },
      {
        id: 'user-admin',
        name: 'System Administrator',
        email: 'admin@atlasmesh.com',
        department: 'IT',
        roles: ['role-system-admin'],
        status: 'active',
        lastLogin: new Date('2024-11-26T10:00:00Z'),
        failedLogins: 0,
        mfaEnabled: true,
        createdAt: new Date('2024-01-15')
      },
      {
        id: 'user-compliance',
        name: 'Maria Rodriguez',
        email: 'maria.rodriguez@atlasmesh.com',
        department: 'Legal & Compliance',
        roles: ['role-compliance-officer'],
        status: 'active',
        lastLogin: new Date('2024-11-25T14:30:00Z'),
        failedLogins: 0,
        mfaEnabled: true,
        createdAt: new Date('2024-01-20')
      }
    ]

    const mockAuditEntries: AuditEntry[] = [
      {
        id: 'audit-001',
        timestamp: new Date('2024-11-26T10:30:00Z'),
        userId: 'user-ahmed',
        userName: 'Ahmed Al-Mansouri',
        action: 'vehicle_emergency_stop',
        resource: 'vehicle:atlas-001',
        roleId: 'role-fleet-operator',
        permissionId: 'perm-emergency-stop',
        outcome: 'success',
        reason: 'Obstacle detected by operator',
        ipAddress: '192.168.1.100',
        userAgent: 'Mozilla/5.0 (Windows NT 10.0; Win64; x64)',
        riskScore: 8.5,
        requiresReview: true
      },
      {
        id: 'audit-002',
        timestamp: new Date('2024-11-26T10:15:00Z'),
        userId: 'user-sarah',
        userName: 'Sarah Johnson',
        action: 'policy_override',
        resource: 'policy:safety-speed-limit',
        roleId: 'role-safety-supervisor',
        permissionId: 'perm-safety-policy-override',
        outcome: 'success',
        reason: 'Emergency evacuation scenario',
        ipAddress: '192.168.1.105',
        userAgent: 'Mozilla/5.0 (iPad; CPU OS 15_0)',
        riskScore: 9.2,
        requiresReview: true,
        reviewedBy: 'safety-director',
        reviewedAt: new Date('2024-11-26T11:00:00Z')
      },
      {
        id: 'audit-003',
        timestamp: new Date('2024-11-26T09:45:00Z'),
        userId: 'user-admin',
        userName: 'System Administrator',
        action: 'user_role_assignment',
        resource: 'user:new-operator-001',
        roleId: 'role-system-admin',
        permissionId: 'perm-user-management',
        outcome: 'success',
        reason: 'New operator onboarding',
        ipAddress: '192.168.1.50',
        userAgent: 'Mozilla/5.0 (Windows NT 10.0; Win64; x64)',
        riskScore: 7.1,
        requiresReview: false
      }
    ]

    setRoles(mockRoles)
    setUsers(mockUsers)
    setAuditEntries(mockAuditEntries)
  }, [])

  useEffect(() => {
    initializeMockData()
  }, [initializeMockData])

  // Handlers
  const handleRoleSelect = useCallback((role: Role) => {
    setSelectedRole(role)
    setShowRoleDialog(true)
    onRoleSelected?.(role)
  }, [onRoleSelected])

  const handlePermissionToggle = useCallback((roleId: string, permissionId: string, enabled: boolean) => {
    setRoles(prev => prev.map(role => 
      role.id === roleId 
        ? {
            ...role,
            permissions: role.permissions.map(permission =>
              permission.id === permissionId 
                ? { ...permission, enabled, grantedAt: new Date() }
                : permission
            ),
            updatedAt: new Date()
          }
        : role
    ))

    // Create audit entry
    const auditEntry: AuditEntry = {
      id: `audit-${Date.now()}`,
      timestamp: new Date(),
      userId: userId || 'current-user',
      userName: 'Current User',
      action: enabled ? 'permission_granted' : 'permission_revoked',
      resource: `permission:${permissionId}`,
      roleId,
      permissionId,
      outcome: 'success',
      reason: `Permission ${enabled ? 'granted' : 'revoked'} by administrator`,
      ipAddress: '192.168.1.100',
      userAgent: navigator.userAgent,
      riskScore: 6.5,
      requiresReview: enabled
    }

    setAuditEntries(prev => [auditEntry, ...prev])
    onAuditRequired?.(auditEntry)
  }, [userId, onAuditRequired])

  const handleUserRoleAssignment = useCallback((userId: string, roleId: string, assign: boolean) => {
    setUsers(prev => prev.map(user => 
      user.id === userId 
        ? {
            ...user,
            roles: assign 
              ? [...user.roles.filter(r => r !== roleId), roleId]
              : user.roles.filter(r => r !== roleId)
          }
        : user
    ))

    // Update role usage count
    setRoles(prev => prev.map(role => 
      role.id === roleId 
        ? {
            ...role,
            users: assign 
              ? [...role.users.filter(u => u !== userId), userId]
              : role.users.filter(u => u !== userId),
            usageCount: assign ? role.usageCount + 1 : Math.max(0, role.usageCount - 1),
            lastUsed: assign ? new Date() : role.lastUsed
          }
        : role
    ))

    // Create audit entry
    const auditEntry: AuditEntry = {
      id: `audit-${Date.now()}`,
      timestamp: new Date(),
      userId: userId || 'current-user',
      userName: 'Current User',
      action: assign ? 'role_assigned' : 'role_revoked',
      resource: `user:${userId}`,
      roleId,
      outcome: 'success',
      reason: `Role ${assign ? 'assigned to' : 'revoked from'} user`,
      ipAddress: '192.168.1.100',
      userAgent: navigator.userAgent,
      riskScore: assign ? 7.0 : 5.0,
      requiresReview: assign
    }

    setAuditEntries(prev => [auditEntry, ...prev])
    onAuditRequired?.(auditEntry)
  }, [userId, onAuditRequired])

  // Filtered data
  const filteredRoles = useMemo(() => {
    return roles
      .filter(role => !filters.category || role.category === filters.category)
      .filter(role => !filters.riskLevel || role.riskLevel === filters.riskLevel)
      .filter(role => !filters.status || role.status === filters.status)
      .filter(role => !filters.type || role.type === filters.type)
      .filter(role => {
        if (!filters.search) return true
        const searchTerm = filters.search.toLowerCase()
        return (
          role.name.toLowerCase().includes(searchTerm) ||
          role.displayName.toLowerCase().includes(searchTerm) ||
          role.description.toLowerCase().includes(searchTerm)
        )
      })
      .sort((a, b) => {
        // Sort by risk level (critical first) then by usage count
        const riskOrder = { critical: 4, high: 3, medium: 2, low: 1 }
        const aRisk = riskOrder[a.riskLevel]
        const bRisk = riskOrder[b.riskLevel]
        
        if (aRisk !== bRisk) {
          return bRisk - aRisk
        }
        
        return b.usageCount - a.usageCount
      })
  }, [roles, filters])

  // Statistics
  const stats = useMemo(() => {
    const totalRoles = roles.length
    const activeRoles = roles.filter(r => r.status === 'active').length
    const criticalRoles = roles.filter(r => r.riskLevel === 'critical').length
    const highRiskRoles = roles.filter(r => r.riskLevel === 'high').length
    
    const totalUsers = users.length
    const activeUsers = users.filter(u => u.status === 'active').length
    const mfaEnabledUsers = users.filter(u => u.mfaEnabled).length
    
    const totalPermissions = roles.reduce((sum, r) => sum + r.permissions.length, 0)
    const activePermissions = roles.reduce((sum, r) => sum + r.permissions.filter(p => p.enabled).length, 0)
    const highRiskPermissions = roles.reduce((sum, r) => 
      sum + r.permissions.filter(p => p.riskLevel === 'high' || p.riskLevel === 'critical').length, 0
    )
    
    const recentAudits = auditEntries.filter(a => 
      new Date().getTime() - a.timestamp.getTime() < 24 * 60 * 60 * 1000
    ).length
    
    const pendingReviews = auditEntries.filter(a => a.requiresReview && !a.reviewedAt).length
    
    return {
      totalRoles,
      activeRoles,
      criticalRoles,
      highRiskRoles,
      totalUsers,
      activeUsers,
      mfaEnabledUsers,
      totalPermissions,
      activePermissions,
      highRiskPermissions,
      recentAudits,
      pendingReviews
    }
  }, [roles, users, auditEntries])

  const getRiskColor = (riskLevel: string) => {
    switch (riskLevel) {
      case 'critical': return 'bg-red-100 text-red-800'
      case 'high': return 'bg-orange-100 text-orange-800'
      case 'medium': return 'bg-yellow-100 text-yellow-800'
      case 'low': return 'bg-green-100 text-green-800'
      default: return 'bg-gray-100 text-gray-800'
    }
  }

  const getCategoryColor = (category: string) => {
    switch (category) {
      case 'operational': return 'bg-blue-100 text-blue-800'
      case 'administrative': return 'bg-purple-100 text-purple-800'
      case 'technical': return 'bg-green-100 text-green-800'
      case 'safety': return 'bg-red-100 text-red-800'
      case 'compliance': return 'bg-yellow-100 text-yellow-800'
      default: return 'bg-gray-100 text-gray-800'
    }
  }

  const getStatusColor = (status: string) => {
    switch (status) {
      case 'active': return 'bg-green-100 text-green-800'
      case 'inactive': return 'bg-gray-100 text-gray-800'
      case 'suspended': return 'bg-red-100 text-red-800'
      case 'pending': return 'bg-yellow-100 text-yellow-800'
      case 'deprecated': return 'bg-orange-100 text-orange-800'
      case 'disabled': return 'bg-red-100 text-red-800'
      default: return 'bg-gray-100 text-gray-800'
    }
  }

  const formatNumber = (num: number) => {
    if (num >= 1000000) {
      return `${(num / 1000000).toFixed(1)}M`
    } else if (num >= 1000) {
      return `${(num / 1000).toFixed(1)}K`
    }
    return num.toString()
  }

  // Permission matrix data
  const permissionMatrix = useMemo(() => {
    const allResources = Array.from(new Set(
      roles.flatMap(role => role.permissions.map(p => p.resource))
    )).sort()
    
    const allActions = Array.from(new Set(
      roles.flatMap(role => role.permissions.map(p => p.action))
    )).sort()

    return {
      roles: filteredRoles,
      resources: allResources,
      actions: allActions,
      getPermission: (roleId: string, resource: string, action: string) => {
        const role = roles.find(r => r.id === roleId)
        return role?.permissions.find(p => p.resource === resource && p.action === action)
      }
    }
  }, [roles, filteredRoles])

  return (
    <div className={`space-y-6 ${className}`}>
      {/* Header */}
      <div className="flex items-center justify-between">
        <div className="flex items-center space-x-3">
          <Shield className="w-6 h-6 text-blue-600" />
          <h2 className="text-2xl font-semibold text-gray-900">
            RBAC Manager
            {userId && <span className="text-lg text-gray-600 ml-2">for {userId}</span>}
          </h2>
        </div>
        <div className="flex items-center space-x-2">
          <Button variant="outline" onClick={() => setMatrixView(!matrixView)}>
            {matrixView ? <Users className="w-4 h-4 mr-2" /> : <Key className="w-4 h-4 mr-2" />}
            {matrixView ? 'List View' : 'Matrix View'}
          </Button>
          {mode === 'configure' && (
            <>
              <Button variant="outline">
                <Plus className="w-4 h-4 mr-2" />
                Add Role
              </Button>
              <Button variant="outline">
                <Upload className="w-4 h-4 mr-2" />
                Import
              </Button>
            </>
          )}
          <Button variant="outline" onClick={initializeMockData}>
            <RefreshCw className="w-4 h-4 mr-2" />
            Refresh
          </Button>
          <Button variant="outline">
            <Download className="w-4 h-4 mr-2" />
            Export
          </Button>
        </div>
      </div>

      {/* Statistics */}
      <div className="grid grid-cols-2 md:grid-cols-4 lg:grid-cols-6 xl:grid-cols-12 gap-4">
        <Card className="p-4">
          <div className="text-center">
            <div className="text-2xl font-bold text-gray-900">{stats.totalRoles}</div>
            <div className="text-sm text-gray-600">Total Roles</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className="text-2xl font-bold text-green-600">{stats.activeRoles}</div>
            <div className="text-sm text-gray-600">Active</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className="text-2xl font-bold text-red-600">{stats.criticalRoles}</div>
            <div className="text-sm text-gray-600">Critical</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className="text-2xl font-bold text-orange-600">{stats.highRiskRoles}</div>
            <div className="text-sm text-gray-600">High Risk</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className="text-2xl font-bold text-blue-600">{stats.totalUsers}</div>
            <div className="text-sm text-gray-600">Users</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className="text-2xl font-bold text-purple-600">{stats.activeUsers}</div>
            <div className="text-sm text-gray-600">Active Users</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className="text-2xl font-bold text-indigo-600">{stats.mfaEnabledUsers}</div>
            <div className="text-sm text-gray-600">MFA Enabled</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className="text-2xl font-bold text-teal-600">{stats.totalPermissions}</div>
            <div className="text-sm text-gray-600">Permissions</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className="text-2xl font-bold text-cyan-600">{stats.activePermissions}</div>
            <div className="text-sm text-gray-600">Active Perms</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className="text-2xl font-bold text-pink-600">{stats.highRiskPermissions}</div>
            <div className="text-sm text-gray-600">High Risk</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className="text-2xl font-bold text-amber-600">{stats.recentAudits}</div>
            <div className="text-sm text-gray-600">Recent Audits</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className="text-2xl font-bold text-rose-600">{stats.pendingReviews}</div>
            <div className="text-sm text-gray-600">Pending Reviews</div>
          </div>
        </Card>
      </div>

      <Tabs value={activeTab} onValueChange={setActiveTab}>
        <TabsList>
          <TabsTrigger value="roles">Roles</TabsTrigger>
          <TabsTrigger value="users">Users</TabsTrigger>
          <TabsTrigger value="permissions">Permissions</TabsTrigger>
          <TabsTrigger value="audit">Audit Trail</TabsTrigger>
          <TabsTrigger value="matrix">Role Matrix</TabsTrigger>
        </TabsList>

        <TabsContent value="roles" className="space-y-4">
          {/* Filters */}
          <Card className="p-4">
            <div className="flex flex-wrap items-center gap-4">
              <div className="flex items-center space-x-2">
                <Search className="w-4 h-4 text-gray-500" />
                <Input
                  placeholder="Search roles..."
                  value={filters.search}
                  onChange={(e) => setFilters(prev => ({ ...prev, search: e.target.value }))}
                  className="w-64"
                />
              </div>
              
              <Select
                value={filters.category}
                onValueChange={(value) => setFilters(prev => ({ ...prev, category: value }))}
              >
                <option value="">All Categories</option>
                <option value="operational">Operational</option>
                <option value="administrative">Administrative</option>
                <option value="technical">Technical</option>
                <option value="safety">Safety</option>
                <option value="compliance">Compliance</option>
              </Select>

              <Select
                value={filters.riskLevel}
                onValueChange={(value) => setFilters(prev => ({ ...prev, riskLevel: value }))}
              >
                <option value="">All Risk Levels</option>
                <option value="critical">Critical</option>
                <option value="high">High</option>
                <option value="medium">Medium</option>
                <option value="low">Low</option>
              </Select>

              <Select
                value={filters.status}
                onValueChange={(value) => setFilters(prev => ({ ...prev, status: value }))}
              >
                <option value="">All Status</option>
                <option value="active">Active</option>
                <option value="deprecated">Deprecated</option>
                <option value="disabled">Disabled</option>
              </Select>

              <Select
                value={filters.type}
                onValueChange={(value) => setFilters(prev => ({ ...prev, type: value }))}
              >
                <option value="">All Types</option>
                <option value="system">System</option>
                <option value="custom">Custom</option>
                <option value="inherited">Inherited</option>
              </Select>
            </div>
          </Card>

          {/* Roles List */}
          <div className="space-y-4">
            {filteredRoles.map(role => (
              <Card key={role.id} className="p-6 hover:shadow-md transition-shadow cursor-pointer"
                    onClick={() => handleRoleSelect(role)}>
                <div className="flex items-start justify-between mb-4">
                  <div className="flex items-center space-x-3">
                    <div className="p-2 bg-blue-100 rounded">
                      <Shield className="w-5 h-5 text-blue-600" />
                    </div>
                    <div>
                      <h3 className="text-lg font-medium text-gray-900">{role.displayName}</h3>
                      <p className="text-sm text-gray-600 font-mono">{role.name}</p>
                      <p className="text-sm text-gray-600 mt-1">{role.description}</p>
                    </div>
                  </div>
                  <div className="flex items-center space-x-2">
                    <Badge className={getRiskColor(role.riskLevel)}>
                      {role.riskLevel}
                    </Badge>
                    <Badge className={getCategoryColor(role.category)}>
                      {role.category}
                    </Badge>
                    <Badge className={getStatusColor(role.status)}>
                      {role.status}
                    </Badge>
                    <Badge variant="outline">
                      {role.type}
                    </Badge>
                  </div>
                </div>

                <div className="grid grid-cols-2 md:grid-cols-4 gap-4 mb-4">
                  <div>
                    <div className="text-sm text-gray-600">Permissions</div>
                    <div className="font-medium text-gray-900">
                      {role.permissions.filter(p => p.enabled).length} / {role.permissions.length}
                    </div>
                  </div>
                  <div>
                    <div className="text-sm text-gray-600">Users</div>
                    <div className="font-medium text-gray-900">
                      {role.users.length}
                    </div>
                  </div>
                  <div>
                    <div className="text-sm text-gray-600">Usage Count</div>
                    <div className="font-medium text-gray-900">
                      {formatNumber(role.usageCount)}
                    </div>
                  </div>
                  <div>
                    <div className="text-sm text-gray-600">Last Used</div>
                    <div className="font-medium text-gray-900">
                      {role.lastUsed ? role.lastUsed.toLocaleDateString() : 'Never'}
                    </div>
                  </div>
                </div>

                {/* Key Permissions */}
                <div className="mb-4">
                  <div className="text-sm font-medium text-gray-900 mb-2">Key Permissions</div>
                  <div className="grid gap-2">
                    {role.permissions.slice(0, 3).map(permission => (
                      <div key={permission.id} className="flex items-center justify-between p-2 bg-gray-50 rounded">
                        <div className="flex items-center space-x-2">
                          <Badge className={getRiskColor(permission.riskLevel)} size="sm">
                            {permission.riskLevel}
                          </Badge>
                          <span className="text-sm font-medium">{permission.resource}:{permission.action}</span>
                          {permission.auditRequired && <FileText className="w-3 h-3 text-gray-500" />}
                          {permission.dualAuthRequired && <Key className="w-3 h-3 text-orange-500" />}
                        </div>
                        <div className="flex items-center space-x-2">
                          {mode === 'configure' && (
                            <Switch
                              checked={permission.enabled}
                              onCheckedChange={(enabled) => handlePermissionToggle(role.id, permission.id, enabled)}
                              size="sm"
                            />
                          )}
                          <Badge className={permission.enabled ? 'bg-green-100 text-green-800' : 'bg-gray-100 text-gray-800'} size="sm">
                            {permission.enabled ? 'Enabled' : 'Disabled'}
                          </Badge>
                        </div>
                      </div>
                    ))}
                    {role.permissions.length > 3 && (
                      <div className="text-sm text-gray-500 text-center py-1">
                        +{role.permissions.length - 3} more permissions
                      </div>
                    )}
                  </div>
                </div>

                {/* Constraints & Security */}
                <div className="grid grid-cols-2 gap-6 mb-4">
                  <div>
                    <div className="text-sm font-medium text-gray-900 mb-2">Security Features</div>
                    <div className="flex flex-wrap gap-1">
                      {role.auditRequired && <Badge variant="outline" size="sm">Audit Required</Badge>}
                      {role.dualAuthRequired && <Badge variant="outline" size="sm">Dual Auth</Badge>}
                      {role.sessionTimeout && <Badge variant="outline" size="sm">{role.sessionTimeout}min Session</Badge>}
                      {role.ipRestrictions.length > 0 && <Badge variant="outline" size="sm">IP Restricted</Badge>}
                      {role.timeRestrictions.length > 0 && <Badge variant="outline" size="sm">Time Restricted</Badge>}
                    </div>
                  </div>
                  <div>
                    <div className="text-sm font-medium text-gray-900 mb-2">Constraints</div>
                    <div className="flex flex-wrap gap-1">
                      {role.constraints.filter(c => c.enabled).map(constraint => (
                        <Badge key={constraint.type} variant="outline" size="sm">
                          {constraint.type.replace('_', ' ')}
                        </Badge>
                      ))}
                    </div>
                  </div>
                </div>

                <div className="flex items-center justify-between pt-4 border-t">
                  <div className="text-sm text-gray-600">
                    Updated: {role.updatedAt.toLocaleDateString()} • 
                    Session Timeout: {role.sessionTimeout}min
                  </div>
                  <div className="flex items-center space-x-2">
                    <Button variant="outline" size="sm">
                      <Eye className="w-4 h-4 mr-1" />
                      View Details
                    </Button>
                    {mode === 'configure' && (
                      <Button variant="outline" size="sm">
                        <Settings className="w-4 h-4 mr-1" />
                        Configure
                      </Button>
                    )}
                  </div>
                </div>
              </Card>
            ))}

            {filteredRoles.length === 0 && (
              <div className="text-center py-12">
                <Shield className="w-16 h-16 text-gray-400 mx-auto mb-4" />
                <h3 className="text-lg font-medium text-gray-900 mb-2">No Roles Found</h3>
                <p className="text-gray-600">
                  {filters.search || filters.category || filters.riskLevel || filters.status || filters.type
                    ? 'No roles match your current filters'
                    : 'No roles configured'
                  }
                </p>
              </div>
            )}
          </div>
        </TabsContent>

        <TabsContent value="users" className="space-y-4">
          <div className="space-y-4">
            {users.map(user => (
              <Card key={user.id} className="p-4">
                <div className="flex items-start justify-between mb-3">
                  <div className="flex items-center space-x-3">
                    <div className="p-2 bg-blue-100 rounded">
                      <User className="w-5 h-5 text-blue-600" />
                    </div>
                    <div>
                      <h3 className="text-lg font-medium text-gray-900">{user.name}</h3>
                      <p className="text-sm text-gray-600">{user.email}</p>
                      <p className="text-sm text-gray-600">{user.department}</p>
                    </div>
                  </div>
                  <div className="flex items-center space-x-2">
                    <Badge className={getStatusColor(user.status)}>
                      {user.status}
                    </Badge>
                    {user.mfaEnabled && <Badge className="bg-green-100 text-green-800">MFA</Badge>}
                  </div>
                </div>

                <div className="grid grid-cols-2 md:grid-cols-4 gap-4 mb-4">
                  <div>
                    <div className="text-sm text-gray-600">Roles</div>
                    <div className="font-medium text-gray-900">{user.roles.length}</div>
                  </div>
                  <div>
                    <div className="text-sm text-gray-600">Last Login</div>
                    <div className="font-medium text-gray-900">
                      {user.lastLogin ? user.lastLogin.toLocaleDateString() : 'Never'}
                    </div>
                  </div>
                  <div>
                    <div className="text-sm text-gray-600">Failed Logins</div>
                    <div className={`font-medium ${user.failedLogins > 0 ? 'text-red-600' : 'text-gray-900'}`}>
                      {user.failedLogins}
                    </div>
                  </div>
                  <div>
                    <div className="text-sm text-gray-600">Created</div>
                    <div className="font-medium text-gray-900">{user.createdAt.toLocaleDateString()}</div>
                  </div>
                </div>

                {/* User Roles */}
                <div className="mb-4">
                  <div className="text-sm font-medium text-gray-900 mb-2">Assigned Roles</div>
                  <div className="flex flex-wrap gap-2">
                    {user.roles.map(roleId => {
                      const role = roles.find(r => r.id === roleId)
                      return role ? (
                        <div key={roleId} className="flex items-center space-x-2 p-2 bg-gray-50 rounded">
                          <Badge className={getRiskColor(role.riskLevel)} size="sm">
                            {role.riskLevel}
                          </Badge>
                          <span className="text-sm font-medium">{role.displayName}</span>
                          {mode === 'configure' && (
                            <Button
                              variant="ghost"
                              size="sm"
                              onClick={() => handleUserRoleAssignment(user.id, roleId, false)}
                            >
                              <XCircle className="w-3 h-3 text-red-500" />
                            </Button>
                          )}
                        </div>
                      ) : null
                    })}
                  </div>
                </div>
              </Card>
            ))}
          </div>
        </TabsContent>

        <TabsContent value="permissions" className="space-y-4">
          <div className="text-center py-8">
            <Key className="w-16 h-16 text-gray-400 mx-auto mb-4" />
            <h3 className="text-lg font-medium text-gray-900 mb-2">Permission Management</h3>
            <p className="text-gray-600">
              Detailed permission configuration interface would be implemented here.
            </p>
          </div>
        </TabsContent>

        <TabsContent value="audit" className="space-y-4">
          <Card className="p-4">
            <h3 className="text-lg font-medium text-gray-900 mb-4">Recent Audit Entries</h3>
            <div className="space-y-3">
              {auditEntries.slice(0, 10).map(entry => (
                <div key={entry.id} className="flex items-center justify-between p-3 border rounded">
                  <div className="flex items-center space-x-3">
                    <div className={`p-1 rounded ${
                      entry.outcome === 'success' ? 'bg-green-100' :
                      entry.outcome === 'failure' ? 'bg-red-100' : 'bg-yellow-100'
                    }`}>
                      {entry.outcome === 'success' ? (
                        <CheckCircle className="w-4 h-4 text-green-600" />
                      ) : entry.outcome === 'failure' ? (
                        <XCircle className="w-4 h-4 text-red-600" />
                      ) : (
                        <AlertTriangle className="w-4 h-4 text-yellow-600" />
                      )}
                    </div>
                    <div>
                      <div className="text-sm font-medium text-gray-900">
                        {entry.userName} • {entry.action}
                      </div>
                      <div className="text-xs text-gray-600">
                        {entry.resource} • {entry.timestamp.toLocaleString()}
                      </div>
                      {entry.reason && (
                        <div className="text-xs text-gray-500">{entry.reason}</div>
                      )}
                    </div>
                  </div>
                  <div className="flex items-center space-x-2">
                    <Badge className={getRiskColor(
                      entry.riskScore >= 9 ? 'critical' :
                      entry.riskScore >= 7 ? 'high' :
                      entry.riskScore >= 5 ? 'medium' : 'low'
                    )} size="sm">
                      Risk: {entry.riskScore.toFixed(1)}
                    </Badge>
                    {entry.requiresReview && (
                      <Badge className={entry.reviewedAt ? 'bg-green-100 text-green-800' : 'bg-yellow-100 text-yellow-800'} size="sm">
                        {entry.reviewedAt ? 'Reviewed' : 'Pending Review'}
                      </Badge>
                    )}
                  </div>
                </div>
              ))}
            </div>
          </Card>
        </TabsContent>

        <TabsContent value="matrix" className="space-y-4">
          {matrixView ? (
            <Card className="p-4">
              <h3 className="text-lg font-medium text-gray-900 mb-4">Role-Permission Matrix</h3>
              <div className="overflow-x-auto">
                <table className="min-w-full">
                  <thead>
                    <tr>
                      <th className="text-left p-2 border-b">Role</th>
                      {permissionMatrix.resources.slice(0, 10).map(resource => (
                        <th key={resource} className="text-center p-2 border-b text-xs">
                          {resource}
                        </th>
                      ))}
                    </tr>
                  </thead>
                  <tbody>
                    {permissionMatrix.roles.slice(0, 10).map(role => (
                      <tr key={role.id}>
                        <td className="p-2 border-b">
                          <div className="text-sm font-medium">{role.displayName}</div>
                          <div className="text-xs text-gray-600">{role.category}</div>
                        </td>
                        {permissionMatrix.resources.slice(0, 10).map(resource => (
                          <td key={resource} className="p-2 border-b text-center">
                            {permissionMatrix.actions.some(action => {
                              const permission = permissionMatrix.getPermission(role.id, resource, action)
                              return permission && permission.enabled
                            }) ? (
                              <CheckCircle className="w-4 h-4 text-green-600 mx-auto" />
                            ) : (
                              <XCircle className="w-4 h-4 text-gray-300 mx-auto" />
                            )}
                          </td>
                        ))}
                      </tr>
                    ))}
                  </tbody>
                </table>
              </div>
            </Card>
          ) : (
            <div className="text-center py-8">
              <Key className="w-16 h-16 text-gray-400 mx-auto mb-4" />
              <h3 className="text-lg font-medium text-gray-900 mb-2">Role Matrix View</h3>
              <p className="text-gray-600">
                Click "Matrix View" in the header to see the role-permission matrix.
              </p>
            </div>
          )}
        </TabsContent>
      </Tabs>

      {/* Role Details Dialog */}
      <Dialog open={showRoleDialog} onOpenChange={setShowRoleDialog}>
        <DialogContent className="max-w-6xl max-h-[90vh] overflow-y-auto">
          <DialogHeader>
            <DialogTitle>Role Details: {selectedRole?.displayName}</DialogTitle>
          </DialogHeader>

          {selectedRole && (
            <div className="space-y-6">
              {/* Basic Information */}
              <div className="grid grid-cols-2 gap-6">
                <Card className="p-4">
                  <h3 className="text-lg font-medium text-gray-900 mb-3">Role Information</h3>
                  <div className="space-y-2 text-sm">
                    <div className="flex justify-between">
                      <span className="text-gray-600">Name:</span>
                      <span className="font-medium font-mono">{selectedRole.name}</span>
                    </div>
                    <div className="flex justify-between">
                      <span className="text-gray-600">Display Name:</span>
                      <span className="font-medium">{selectedRole.displayName}</span>
                    </div>
                    <div className="flex justify-between">
                      <span className="text-gray-600">Category:</span>
                      <Badge className={getCategoryColor(selectedRole.category)}>
                        {selectedRole.category}
                      </Badge>
                    </div>
                    <div className="flex justify-between">
                      <span className="text-gray-600">Type:</span>
                      <span className="font-medium">{selectedRole.type}</span>
                    </div>
                    <div className="flex justify-between">
                      <span className="text-gray-600">Risk Level:</span>
                      <Badge className={getRiskColor(selectedRole.riskLevel)}>
                        {selectedRole.riskLevel}
                      </Badge>
                    </div>
                  </div>
                </Card>

                <Card className="p-4">
                  <h3 className="text-lg font-medium text-gray-900 mb-3">Usage Statistics</h3>
                  <div className="space-y-2 text-sm">
                    <div className="flex justify-between">
                      <span className="text-gray-600">Users:</span>
                      <span className="font-medium">{selectedRole.users.length}</span>
                    </div>
                    <div className="flex justify-between">
                      <span className="text-gray-600">Usage Count:</span>
                      <span className="font-medium">{formatNumber(selectedRole.usageCount)}</span>
                    </div>
                    <div className="flex justify-between">
                      <span className="text-gray-600">Last Used:</span>
                      <span className="font-medium">
                        {selectedRole.lastUsed ? selectedRole.lastUsed.toLocaleDateString() : 'Never'}
                      </span>
                    </div>
                    <div className="flex justify-between">
                      <span className="text-gray-600">Session Timeout:</span>
                      <span className="font-medium">{selectedRole.sessionTimeout} minutes</span>
                    </div>
                  </div>
                </Card>
              </div>

              {/* Description */}
              <Card className="p-4">
                <h3 className="text-lg font-medium text-gray-900 mb-3">Description</h3>
                <p className="text-gray-600">{selectedRole.description}</p>
              </Card>

              {/* Permissions */}
              <Card className="p-4">
                <h3 className="text-lg font-medium text-gray-900 mb-3">Permissions</h3>
                <div className="space-y-4">
                  {selectedRole.permissions.map(permission => (
                    <div key={permission.id} className="p-4 border rounded">
                      <div className="flex items-center justify-between mb-3">
                        <div className="flex items-center space-x-3">
                          <Badge className={getRiskColor(permission.riskLevel)}>
                            {permission.riskLevel}
                          </Badge>
                          <div>
                            <h4 className="font-medium text-gray-900">
                              {permission.resource}:{permission.action}
                            </h4>
                            <p className="text-sm text-gray-600">{permission.description}</p>
                          </div>
                        </div>
                        <div className="flex items-center space-x-2">
                          {permission.auditRequired && (
                            <Badge variant="outline" size="sm">Audit</Badge>
                          )}
                          {permission.dualAuthRequired && (
                            <Badge variant="outline" size="sm">Dual Auth</Badge>
                          )}
                          <Badge className={permission.enabled ? 'bg-green-100 text-green-800' : 'bg-gray-100 text-gray-800'}>
                            {permission.enabled ? 'Enabled' : 'Disabled'}
                          </Badge>
                        </div>
                      </div>

                      {permission.conditions.length > 0 && (
                        <div className="mb-3">
                          <div className="text-sm font-medium text-gray-900 mb-2">Conditions:</div>
                          <div className="space-y-1">
                            {permission.conditions.map((condition, index) => (
                              <div key={index} className="text-xs text-gray-600 bg-gray-50 p-2 rounded">
                                {condition.field} {condition.operator} {JSON.stringify(condition.value)} - {condition.description}
                              </div>
                            ))}
                          </div>
                        </div>
                      )}

                      <div className="text-xs text-gray-500">
                        Granted: {permission.grantedAt.toLocaleDateString()} by {permission.grantedBy}
                        {permission.expiresAt && ` • Expires: ${permission.expiresAt.toLocaleDateString()}`}
                      </div>
                    </div>
                  ))}
                </div>
              </Card>

              {/* Security Features */}
              <div className="grid grid-cols-2 gap-6">
                <Card className="p-4">
                  <h3 className="text-lg font-medium text-gray-900 mb-3">Security Features</h3>
                  <div className="space-y-2 text-sm">
                    <div className="flex justify-between">
                      <span className="text-gray-600">Audit Required:</span>
                      <Badge className={selectedRole.auditRequired ? 'bg-green-100 text-green-800' : 'bg-gray-100 text-gray-800'}>
                        {selectedRole.auditRequired ? 'Yes' : 'No'}
                      </Badge>
                    </div>
                    <div className="flex justify-between">
                      <span className="text-gray-600">Dual Auth Required:</span>
                      <Badge className={selectedRole.dualAuthRequired ? 'bg-green-100 text-green-800' : 'bg-gray-100 text-gray-800'}>
                        {selectedRole.dualAuthRequired ? 'Yes' : 'No'}
                      </Badge>
                    </div>
                    <div className="flex justify-between">
                      <span className="text-gray-600">IP Restrictions:</span>
                      <span className="font-medium">{selectedRole.ipRestrictions.length} ranges</span>
                    </div>
                    <div className="flex justify-between">
                      <span className="text-gray-600">Time Restrictions:</span>
                      <span className="font-medium">{selectedRole.timeRestrictions.length} rules</span>
                    </div>
                  </div>
                </Card>

                <Card className="p-4">
                  <h3 className="text-lg font-medium text-gray-900 mb-3">Constraints</h3>
                  <div className="space-y-2">
                    {selectedRole.constraints.filter(c => c.enabled).map(constraint => (
                      <div key={constraint.type} className="p-2 bg-gray-50 rounded">
                        <div className="text-sm font-medium capitalize">{constraint.type.replace('_', ' ')}</div>
                        <div className="text-xs text-gray-600">{constraint.description}</div>
                      </div>
                    ))}
                  </div>
                </Card>
              </div>
            </div>
          )}

          <DialogFooter>
            <Button variant="outline" onClick={() => setShowRoleDialog(false)}>
              Close
            </Button>
            {mode === 'configure' && (
              <Button>
                <Edit3 className="w-4 h-4 mr-2" />
                Edit Role
              </Button>
            )}
          </DialogFooter>
        </DialogContent>
      </Dialog>
    </div>
  )
}

export default RBACManager
