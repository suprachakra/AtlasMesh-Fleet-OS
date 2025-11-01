import React, { useState, useCallback, useEffect, useMemo } from 'react'
import { 
  Shield, Eye, EyeOff, Lock, Unlock, Download, Upload, AlertTriangle,
  CheckCircle, XCircle, Search, Filter, Settings, RefreshCw, FileText,
  Database, MapPin, User, Phone, Mail, CreditCard, Hash, Calendar,
  Globe, Smartphone, Car, Route, Clock, Tag, Trash2, Plus, Edit3
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

// Types
interface PIIField {
  id: string
  name: string
  displayName: string
  category: PIICategory
  dataType: 'string' | 'number' | 'email' | 'phone' | 'date' | 'coordinate' | 'json' | 'binary'
  description: string
  sensitivity: 'low' | 'medium' | 'high' | 'critical'
  sources: string[] // Services/systems that collect this data
  redactionRules: RedactionRule[]
  retentionPolicy: RetentionPolicy
  complianceFrameworks: string[] // GDPR, CCPA, HIPAA, etc.
  status: 'active' | 'deprecated' | 'disabled'
  createdAt: Date
  updatedAt: Date
  lastAccessed?: Date
  accessCount: number
  examples: PIIExample[]
}

interface RedactionRule {
  id: string
  name: string
  description: string
  method: RedactionMethod
  parameters: RedactionParameters
  contexts: RedactionContext[]
  conditions: RedactionCondition[]
  priority: number
  enabled: boolean
  createdAt: Date
  updatedAt: Date
  lastApplied?: Date
  applicationCount: number
}

interface RedactionParameters {
  // For masking
  maskChar?: string
  visibleChars?: number
  visiblePosition?: 'start' | 'end' | 'middle'
  
  // For hashing
  algorithm?: 'sha256' | 'sha512' | 'md5' | 'bcrypt'
  salt?: string
  
  // For tokenization
  tokenFormat?: string
  preserveFormat?: boolean
  
  // For generalization
  precision?: number
  roundingMode?: 'up' | 'down' | 'nearest'
  
  // For synthetic data
  syntheticType?: 'random' | 'deterministic' | 'format_preserving'
  
  // For deletion
  replacementValue?: string
  
  // Custom parameters
  customParams?: Record<string, any>
}

interface RedactionContext {
  name: string
  description: string
  applies: boolean
  conditions?: string[]
}

interface RedactionCondition {
  field: string
  operator: 'equals' | 'not_equals' | 'contains' | 'not_contains' | 'regex' | 'exists' | 'not_exists'
  value?: string | number | boolean
  caseSensitive?: boolean
}

interface RetentionPolicy {
  retentionDays: number
  autoDelete: boolean
  archiveBefore: boolean
  archiveLocation?: string
  complianceRequirement?: string
}

interface PIIExample {
  id: string
  name: string
  originalValue: string
  redactedValue: string
  method: string
  context: string
}

type PIICategory = 
  | 'identity' 
  | 'contact' 
  | 'financial' 
  | 'location' 
  | 'biometric' 
  | 'health' 
  | 'behavioral' 
  | 'technical' 
  | 'employment' 
  | 'demographic'

type RedactionMethod = 
  | 'mask' 
  | 'hash' 
  | 'tokenize' 
  | 'encrypt' 
  | 'delete' 
  | 'generalize' 
  | 'synthetic' 
  | 'null'

interface ExportPolicy {
  id: string
  name: string
  description: string
  allowedFormats: string[]
  redactionLevel: 'none' | 'minimal' | 'standard' | 'strict' | 'complete'
  approvalRequired: boolean
  approvers: string[]
  auditRequired: boolean
  retentionDays: number
  conditions: ExportCondition[]
  enabled: boolean
}

interface ExportCondition {
  field: string
  operator: string
  value: string
  required: boolean
}

interface PIIRedactionManagerProps {
  entityId?: string
  entityType?: 'vehicle' | 'trip' | 'user' | 'system'
  mode?: 'view' | 'configure'
  onFieldSelected?: (field: PIIField) => void
  onRuleApplied?: (field: PIIField, rule: RedactionRule) => void
  className?: string
}

const PIIRedactionManager: React.FC<PIIRedactionManagerProps> = ({
  entityId,
  entityType = 'system',
  mode = 'view',
  onFieldSelected,
  onRuleApplied,
  className = ''
}) => {
  // State
  const [piiFields, setPiiFields] = useState<PIIField[]>([])
  const [exportPolicies, setExportPolicies] = useState<ExportPolicy[]>([])
  const [showFieldDialog, setShowFieldDialog] = useState(false)
  const [showRuleDialog, setShowRuleDialog] = useState(false)
  const [showExportDialog, setShowExportDialog] = useState(false)
  const [selectedField, setSelectedField] = useState<PIIField | null>(null)
  const [selectedRule, setSelectedRule] = useState<RedactionRule | null>(null)
  const [activeTab, setActiveTab] = useState('fields')
  const [filters, setFilters] = useState({
    category: '',
    sensitivity: '',
    status: '',
    search: '',
    source: ''
  })
  const [newField, setNewField] = useState<Partial<PIIField>>({})
  const [newRule, setNewRule] = useState<Partial<RedactionRule>>({})

  // Mock data initialization
  const initializeMockData = useCallback(() => {
    const mockFields: PIIField[] = [
      {
        id: 'field-email',
        name: 'user_email',
        displayName: 'User Email Address',
        category: 'contact',
        dataType: 'email',
        description: 'Primary email address for user communication and identification',
        sensitivity: 'high',
        sources: ['user-service', 'authentication-service', 'notification-service'],
        redactionRules: [
          {
            id: 'rule-email-mask',
            name: 'Email Masking for UI',
            description: 'Mask email addresses in user interfaces',
            method: 'mask',
            parameters: {
              maskChar: '*',
              visibleChars: 3,
              visiblePosition: 'start'
            },
            contexts: [
              { name: 'ui_display', description: 'User interface display contexts', applies: true },
              { name: 'logs', description: 'Application logs', applies: true },
              { name: 'exports', description: 'Data exports', applies: false },
              { name: 'analytics', description: 'Analytics processing', applies: true }
            ],
            conditions: [
              { field: 'user_role', operator: 'not_equals', value: 'admin' }
            ],
            priority: 1,
            enabled: true,
            createdAt: new Date('2024-01-15'),
            updatedAt: new Date('2024-11-20'),
            lastApplied: new Date('2024-11-26'),
            applicationCount: 15847
          },
          {
            id: 'rule-email-hash',
            name: 'Email Hashing for Analytics',
            description: 'Hash email addresses for analytics while preserving uniqueness',
            method: 'hash',
            parameters: {
              algorithm: 'sha256',
              salt: 'analytics-salt-2024'
            },
            contexts: [
              { name: 'analytics', description: 'Analytics processing', applies: true },
              { name: 'reporting', description: 'Business reporting', applies: true }
            ],
            conditions: [],
            priority: 2,
            enabled: true,
            createdAt: new Date('2024-02-01'),
            updatedAt: new Date('2024-11-15'),
            lastApplied: new Date('2024-11-26'),
            applicationCount: 8923
          }
        ],
        retentionPolicy: {
          retentionDays: 2555, // 7 years for GDPR compliance
          autoDelete: true,
          archiveBefore: true,
          archiveLocation: 's3://pii-archive/emails/',
          complianceRequirement: 'GDPR Article 17 - Right to erasure'
        },
        complianceFrameworks: ['GDPR', 'CCPA', 'CAN-SPAM'],
        status: 'active',
        createdAt: new Date('2024-01-15'),
        updatedAt: new Date('2024-11-20'),
        lastAccessed: new Date('2024-11-26'),
        accessCount: 24770,
        examples: [
          {
            id: 'example-email-1',
            name: 'Standard Email Masking',
            originalValue: 'ahmed.mansouri@example.com',
            redactedValue: 'ahm*****@example.com',
            method: 'mask',
            context: 'ui_display'
          },
          {
            id: 'example-email-2',
            name: 'Email Hashing for Analytics',
            originalValue: 'ahmed.mansouri@example.com',
            redactedValue: 'sha256:a1b2c3d4e5f6...',
            method: 'hash',
            context: 'analytics'
          }
        ]
      },
      {
        id: 'field-phone',
        name: 'user_phone',
        displayName: 'Phone Number',
        category: 'contact',
        dataType: 'phone',
        description: 'User phone number for emergency contact and SMS notifications',
        sensitivity: 'high',
        sources: ['user-service', 'emergency-service', 'notification-service'],
        redactionRules: [
          {
            id: 'rule-phone-mask',
            name: 'Phone Number Masking',
            description: 'Mask phone numbers for privacy protection',
            method: 'mask',
            parameters: {
              maskChar: 'X',
              visibleChars: 4,
              visiblePosition: 'end'
            },
            contexts: [
              { name: 'ui_display', description: 'User interface display', applies: true },
              { name: 'logs', description: 'Application logs', applies: true }
            ],
            conditions: [],
            priority: 1,
            enabled: true,
            createdAt: new Date('2024-01-20'),
            updatedAt: new Date('2024-11-18'),
            lastApplied: new Date('2024-11-26'),
            applicationCount: 9834
          }
        ],
        retentionPolicy: {
          retentionDays: 2555,
          autoDelete: true,
          archiveBefore: true,
          complianceRequirement: 'GDPR compliance'
        },
        complianceFrameworks: ['GDPR', 'CCPA', 'TCPA'],
        status: 'active',
        createdAt: new Date('2024-01-20'),
        updatedAt: new Date('2024-11-18'),
        lastAccessed: new Date('2024-11-26'),
        accessCount: 12456,
        examples: [
          {
            id: 'example-phone-1',
            name: 'Phone Number Masking',
            originalValue: '+971-50-123-4567',
            redactedValue: '+971-XX-XXX-4567',
            method: 'mask',
            context: 'ui_display'
          }
        ]
      },
      {
        id: 'field-location',
        name: 'vehicle_location',
        displayName: 'Vehicle GPS Coordinates',
        category: 'location',
        dataType: 'coordinate',
        description: 'Precise GPS coordinates of vehicle location',
        sensitivity: 'critical',
        sources: ['vehicle-telemetry', 'fleet-manager', 'route-planner'],
        redactionRules: [
          {
            id: 'rule-location-generalize',
            name: 'Location Generalization',
            description: 'Reduce precision of GPS coordinates for privacy',
            method: 'generalize',
            parameters: {
              precision: 3, // 3 decimal places (~100m accuracy)
              roundingMode: 'nearest'
            },
            contexts: [
              { name: 'analytics', description: 'Analytics processing', applies: true },
              { name: 'reporting', description: 'Business reporting', applies: true },
              { name: 'third_party', description: 'Third-party integrations', applies: true }
            ],
            conditions: [
              { field: 'location_type', operator: 'not_equals', value: 'emergency' }
            ],
            priority: 1,
            enabled: true,
            createdAt: new Date('2024-02-15'),
            updatedAt: new Date('2024-11-10'),
            lastApplied: new Date('2024-11-26'),
            applicationCount: 45623
          },
          {
            id: 'rule-location-delete',
            name: 'Location Deletion for Exports',
            description: 'Remove location data from certain exports',
            method: 'delete',
            parameters: {
              replacementValue: '[LOCATION_REDACTED]'
            },
            contexts: [
              { name: 'public_exports', description: 'Public data exports', applies: true }
            ],
            conditions: [],
            priority: 2,
            enabled: true,
            createdAt: new Date('2024-03-01'),
            updatedAt: new Date('2024-11-05'),
            lastApplied: new Date('2024-11-25'),
            applicationCount: 1234
          }
        ],
        retentionPolicy: {
          retentionDays: 1095, // 3 years
          autoDelete: true,
          archiveBefore: false,
          complianceRequirement: 'Location privacy regulations'
        },
        complianceFrameworks: ['GDPR', 'CCPA', 'Location Privacy Laws'],
        status: 'active',
        createdAt: new Date('2024-02-15'),
        updatedAt: new Date('2024-11-10'),
        lastAccessed: new Date('2024-11-26'),
        accessCount: 89234,
        examples: [
          {
            id: 'example-location-1',
            name: 'Coordinate Generalization',
            originalValue: '25.204816, 55.270783',
            redactedValue: '25.205, 55.271',
            method: 'generalize',
            context: 'analytics'
          },
          {
            id: 'example-location-2',
            name: 'Location Deletion',
            originalValue: '25.204816, 55.270783',
            redactedValue: '[LOCATION_REDACTED]',
            method: 'delete',
            context: 'public_exports'
          }
        ]
      },
      {
        id: 'field-payment',
        name: 'payment_card_number',
        displayName: 'Payment Card Number',
        category: 'financial',
        dataType: 'string',
        description: 'Credit/debit card number for payment processing',
        sensitivity: 'critical',
        sources: ['payment-service', 'billing-service'],
        redactionRules: [
          {
            id: 'rule-card-tokenize',
            name: 'Card Tokenization',
            description: 'Replace card numbers with secure tokens',
            method: 'tokenize',
            parameters: {
              tokenFormat: 'tok_****_****_****_####',
              preserveFormat: true
            },
            contexts: [
              { name: 'ui_display', description: 'User interface display', applies: true },
              { name: 'logs', description: 'Application logs', applies: true },
              { name: 'database', description: 'Database storage', applies: true }
            ],
            conditions: [],
            priority: 1,
            enabled: true,
            createdAt: new Date('2024-01-10'),
            updatedAt: new Date('2024-11-22'),
            lastApplied: new Date('2024-11-26'),
            applicationCount: 5678
          }
        ],
        retentionPolicy: {
          retentionDays: 90, // PCI DSS requirement
          autoDelete: true,
          archiveBefore: false,
          complianceRequirement: 'PCI DSS data retention'
        },
        complianceFrameworks: ['PCI DSS', 'GDPR'],
        status: 'active',
        createdAt: new Date('2024-01-10'),
        updatedAt: new Date('2024-11-22'),
        lastAccessed: new Date('2024-11-26'),
        accessCount: 3456,
        examples: [
          {
            id: 'example-card-1',
            name: 'Card Number Tokenization',
            originalValue: '4532-1234-5678-9012',
            redactedValue: 'tok_****_****_****_9012',
            method: 'tokenize',
            context: 'ui_display'
          }
        ]
      },
      {
        id: 'field-device-id',
        name: 'device_identifier',
        displayName: 'Device Identifier',
        category: 'technical',
        dataType: 'string',
        description: 'Unique identifier for user devices and vehicles',
        sensitivity: 'medium',
        sources: ['device-manager', 'analytics-service', 'session-service'],
        redactionRules: [
          {
            id: 'rule-device-hash',
            name: 'Device ID Hashing',
            description: 'Hash device identifiers for analytics',
            method: 'hash',
            parameters: {
              algorithm: 'sha256',
              salt: 'device-analytics-2024'
            },
            contexts: [
              { name: 'analytics', description: 'Analytics processing', applies: true },
              { name: 'third_party', description: 'Third-party sharing', applies: true }
            ],
            conditions: [],
            priority: 1,
            enabled: true,
            createdAt: new Date('2024-03-10'),
            updatedAt: new Date('2024-11-12'),
            lastApplied: new Date('2024-11-26'),
            applicationCount: 23456
          }
        ],
        retentionPolicy: {
          retentionDays: 730, // 2 years
          autoDelete: true,
          archiveBefore: true,
          complianceRequirement: 'Device tracking privacy'
        },
        complianceFrameworks: ['GDPR', 'CCPA'],
        status: 'active',
        createdAt: new Date('2024-03-10'),
        updatedAt: new Date('2024-11-12'),
        lastAccessed: new Date('2024-11-26'),
        accessCount: 67890,
        examples: [
          {
            id: 'example-device-1',
            name: 'Device ID Hashing',
            originalValue: 'IMEI:123456789012345',
            redactedValue: 'sha256:f7c3bc1d808e04732adf679965ccc34ca7ae3441b...',
            method: 'hash',
            context: 'analytics'
          }
        ]
      }
    ]

    const mockExportPolicies: ExportPolicy[] = [
      {
        id: 'export-policy-public',
        name: 'Public Data Export',
        description: 'Policy for public-facing data exports with strict redaction',
        allowedFormats: ['csv', 'json'],
        redactionLevel: 'complete',
        approvalRequired: false,
        approvers: [],
        auditRequired: true,
        retentionDays: 30,
        conditions: [],
        enabled: true
      },
      {
        id: 'export-policy-internal',
        name: 'Internal Analytics Export',
        description: 'Policy for internal analytics with standard redaction',
        allowedFormats: ['csv', 'json', 'parquet'],
        redactionLevel: 'standard',
        approvalRequired: true,
        approvers: ['data-protection-officer', 'analytics-lead'],
        auditRequired: true,
        retentionDays: 90,
        conditions: [
          { field: 'department', operator: 'equals', value: 'analytics', required: true },
          { field: 'purpose', operator: 'contains', value: 'business_intelligence', required: true }
        ],
        enabled: true
      },
      {
        id: 'export-policy-compliance',
        name: 'Compliance Audit Export',
        description: 'Policy for compliance audits with minimal redaction',
        allowedFormats: ['json', 'xml'],
        redactionLevel: 'minimal',
        approvalRequired: true,
        approvers: ['compliance-officer', 'legal-counsel'],
        auditRequired: true,
        retentionDays: 365,
        conditions: [
          { field: 'audit_type', operator: 'exists', value: '', required: true },
          { field: 'requester_role', operator: 'equals', value: 'auditor', required: true }
        ],
        enabled: true
      }
    ]

    setPiiFields(mockFields)
    setExportPolicies(mockExportPolicies)
  }, [])

  useEffect(() => {
    initializeMockData()
  }, [initializeMockData])

  // Handlers
  const handleFieldSelect = useCallback((field: PIIField) => {
    setSelectedField(field)
    setShowFieldDialog(true)
    onFieldSelected?.(field)
  }, [onFieldSelected])

  const handleRuleToggle = useCallback((fieldId: string, ruleId: string, enabled: boolean) => {
    setPiiFields(prev => prev.map(field => 
      field.id === fieldId 
        ? {
            ...field,
            redactionRules: field.redactionRules.map(rule =>
              rule.id === ruleId 
                ? { ...rule, enabled, updatedAt: new Date() }
                : rule
            )
          }
        : field
    ))
  }, [])

  const handleApplyRule = useCallback((field: PIIField, rule: RedactionRule) => {
    // Simulate applying the rule
    setPiiFields(prev => prev.map(f => 
      f.id === field.id 
        ? {
            ...f,
            redactionRules: f.redactionRules.map(r =>
              r.id === rule.id 
                ? { ...r, lastApplied: new Date(), applicationCount: r.applicationCount + 1 }
                : r
            ),
            lastAccessed: new Date(),
            accessCount: f.accessCount + 1
          }
        : f
    ))
    onRuleApplied?.(field, rule)
  }, [onRuleApplied])

  const handleTestRedaction = useCallback((field: PIIField, rule: RedactionRule, testValue: string) => {
    // Simulate redaction based on rule method
    let redactedValue = testValue
    
    switch (rule.method) {
      case 'mask':
        const maskChar = rule.parameters.maskChar || '*'
        const visibleChars = rule.parameters.visibleChars || 3
        const position = rule.parameters.visiblePosition || 'start'
        
        if (position === 'start') {
          redactedValue = testValue.substring(0, visibleChars) + maskChar.repeat(Math.max(0, testValue.length - visibleChars))
        } else if (position === 'end') {
          redactedValue = maskChar.repeat(Math.max(0, testValue.length - visibleChars)) + testValue.substring(testValue.length - visibleChars)
        }
        break
        
      case 'hash':
        redactedValue = `${rule.parameters.algorithm}:${Math.random().toString(36).substring(2, 15)}...`
        break
        
      case 'tokenize':
        redactedValue = rule.parameters.tokenFormat?.replace(/[#*]/g, () => 
          Math.random() < 0.5 ? '*' : Math.floor(Math.random() * 10).toString()
        ) || `tok_${Math.random().toString(36).substring(2, 8)}`
        break
        
      case 'delete':
        redactedValue = rule.parameters.replacementValue || '[REDACTED]'
        break
        
      case 'generalize':
        if (field.dataType === 'coordinate') {
          const coords = testValue.split(',').map(c => parseFloat(c.trim()))
          const precision = rule.parameters.precision || 3
          redactedValue = coords.map(c => c.toFixed(precision)).join(', ')
        }
        break
        
      default:
        redactedValue = '[REDACTED]'
    }
    
    return redactedValue
  }, [])

  // Filtered fields
  const filteredFields = useMemo(() => {
    return piiFields
      .filter(field => !entityId || field.sources.some(s => s.includes(entityType)))
      .filter(field => !filters.category || field.category === filters.category)
      .filter(field => !filters.sensitivity || field.sensitivity === filters.sensitivity)
      .filter(field => !filters.status || field.status === filters.status)
      .filter(field => !filters.source || field.sources.some(s => s.includes(filters.source)))
      .filter(field => {
        if (!filters.search) return true
        const searchTerm = filters.search.toLowerCase()
        return (
          field.name.toLowerCase().includes(searchTerm) ||
          field.displayName.toLowerCase().includes(searchTerm) ||
          field.description.toLowerCase().includes(searchTerm)
        )
      })
      .sort((a, b) => {
        // Sort by sensitivity (critical first) then by access count
        const sensitivityOrder = { critical: 4, high: 3, medium: 2, low: 1 }
        const aSensitivity = sensitivityOrder[a.sensitivity]
        const bSensitivity = sensitivityOrder[b.sensitivity]
        
        if (aSensitivity !== bSensitivity) {
          return bSensitivity - aSensitivity
        }
        
        return b.accessCount - a.accessCount
      })
  }, [piiFields, entityId, entityType, filters])

  // Statistics
  const stats = useMemo(() => {
    const totalFields = piiFields.length
    const activeFields = piiFields.filter(f => f.status === 'active').length
    const criticalFields = piiFields.filter(f => f.sensitivity === 'critical').length
    const highFields = piiFields.filter(f => f.sensitivity === 'high').length
    
    const totalRules = piiFields.reduce((sum, f) => sum + f.redactionRules.length, 0)
    const activeRules = piiFields.reduce((sum, f) => sum + f.redactionRules.filter(r => r.enabled).length, 0)
    const totalApplications = piiFields.reduce((sum, f) => 
      sum + f.redactionRules.reduce((ruleSum, r) => ruleSum + r.applicationCount, 0), 0
    )
    
    return {
      totalFields,
      activeFields,
      criticalFields,
      highFields,
      totalRules,
      activeRules,
      totalApplications
    }
  }, [piiFields])

  const getSensitivityColor = (sensitivity: string) => {
    switch (sensitivity) {
      case 'critical': return 'bg-red-100 text-red-800'
      case 'high': return 'bg-orange-100 text-orange-800'
      case 'medium': return 'bg-yellow-100 text-yellow-800'
      case 'low': return 'bg-green-100 text-green-800'
      default: return 'bg-gray-100 text-gray-800'
    }
  }

  const getCategoryColor = (category: string) => {
    switch (category) {
      case 'identity': return 'bg-purple-100 text-purple-800'
      case 'contact': return 'bg-blue-100 text-blue-800'
      case 'financial': return 'bg-green-100 text-green-800'
      case 'location': return 'bg-red-100 text-red-800'
      case 'biometric': return 'bg-pink-100 text-pink-800'
      case 'health': return 'bg-teal-100 text-teal-800'
      case 'behavioral': return 'bg-indigo-100 text-indigo-800'
      case 'technical': return 'bg-gray-100 text-gray-800'
      case 'employment': return 'bg-yellow-100 text-yellow-800'
      case 'demographic': return 'bg-orange-100 text-orange-800'
      default: return 'bg-gray-100 text-gray-800'
    }
  }

  const getMethodIcon = (method: RedactionMethod) => {
    switch (method) {
      case 'mask': return <EyeOff className="w-4 h-4" />
      case 'hash': return <Hash className="w-4 h-4" />
      case 'tokenize': return <Tag className="w-4 h-4" />
      case 'encrypt': return <Lock className="w-4 h-4" />
      case 'delete': return <Trash2 className="w-4 h-4" />
      case 'generalize': return <Globe className="w-4 h-4" />
      case 'synthetic': return <RefreshCw className="w-4 h-4" />
      case 'null': return <XCircle className="w-4 h-4" />
      default: return <Shield className="w-4 h-4" />
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

  return (
    <div className={`space-y-6 ${className}`}>
      {/* Header */}
      <div className="flex items-center justify-between">
        <div className="flex items-center space-x-3">
          <Shield className="w-6 h-6 text-blue-600" />
          <h2 className="text-2xl font-semibold text-gray-900">
            PII Redaction Manager
            {entityId && <span className="text-lg text-gray-600 ml-2">for {entityId}</span>}
          </h2>
        </div>
        <div className="flex items-center space-x-2">
          {mode === 'configure' && (
            <>
              <Button variant="outline">
                <Plus className="w-4 h-4 mr-2" />
                Add Field
              </Button>
              <Button variant="outline">
                <Upload className="w-4 h-4 mr-2" />
                Import Rules
              </Button>
            </>
          )}
          <Button variant="outline" onClick={initializeMockData}>
            <RefreshCw className="w-4 h-4 mr-2" />
            Refresh
          </Button>
          <Button variant="outline" onClick={() => setShowExportDialog(true)}>
            <Download className="w-4 h-4 mr-2" />
            Export Policy
          </Button>
        </div>
      </div>

      {/* Statistics */}
      <div className="grid grid-cols-2 md:grid-cols-4 lg:grid-cols-7 gap-4">
        <Card className="p-4">
          <div className="text-center">
            <div className="text-2xl font-bold text-gray-900">{stats.totalFields}</div>
            <div className="text-sm text-gray-600">Total Fields</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className="text-2xl font-bold text-green-600">{stats.activeFields}</div>
            <div className="text-sm text-gray-600">Active</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className="text-2xl font-bold text-red-600">{stats.criticalFields}</div>
            <div className="text-sm text-gray-600">Critical</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className="text-2xl font-bold text-orange-600">{stats.highFields}</div>
            <div className="text-sm text-gray-600">High Risk</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className="text-2xl font-bold text-blue-600">{stats.totalRules}</div>
            <div className="text-sm text-gray-600">Total Rules</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className="text-2xl font-bold text-purple-600">{stats.activeRules}</div>
            <div className="text-sm text-gray-600">Active Rules</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className="text-2xl font-bold text-indigo-600">{formatNumber(stats.totalApplications)}</div>
            <div className="text-sm text-gray-600">Applications</div>
          </div>
        </Card>
      </div>

      <Tabs value={activeTab} onValueChange={setActiveTab}>
        <TabsList>
          <TabsTrigger value="fields">PII Fields</TabsTrigger>
          <TabsTrigger value="rules">Redaction Rules</TabsTrigger>
          <TabsTrigger value="policies">Export Policies</TabsTrigger>
          <TabsTrigger value="compliance">Compliance</TabsTrigger>
        </TabsList>

        <TabsContent value="fields" className="space-y-4">
          {/* Filters */}
          <Card className="p-4">
            <div className="flex flex-wrap items-center gap-4">
              <div className="flex items-center space-x-2">
                <Search className="w-4 h-4 text-gray-500" />
                <Input
                  placeholder="Search PII fields..."
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
                <option value="identity">Identity</option>
                <option value="contact">Contact</option>
                <option value="financial">Financial</option>
                <option value="location">Location</option>
                <option value="biometric">Biometric</option>
                <option value="health">Health</option>
                <option value="behavioral">Behavioral</option>
                <option value="technical">Technical</option>
                <option value="employment">Employment</option>
                <option value="demographic">Demographic</option>
              </Select>

              <Select
                value={filters.sensitivity}
                onValueChange={(value) => setFilters(prev => ({ ...prev, sensitivity: value }))}
              >
                <option value="">All Sensitivity</option>
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

              <Input
                placeholder="Source filter..."
                value={filters.source}
                onChange={(e) => setFilters(prev => ({ ...prev, source: e.target.value }))}
                className="w-48"
              />
            </div>
          </Card>

          {/* PII Fields List */}
          <div className="space-y-4">
            {filteredFields.map(field => (
              <Card key={field.id} className="p-6 hover:shadow-md transition-shadow cursor-pointer"
                    onClick={() => handleFieldSelect(field)}>
                <div className="flex items-start justify-between mb-4">
                  <div className="flex items-center space-x-3">
                    <div className="p-2 bg-blue-100 rounded">
                      <Shield className="w-5 h-5 text-blue-600" />
                    </div>
                    <div>
                      <h3 className="text-lg font-medium text-gray-900">{field.displayName}</h3>
                      <p className="text-sm text-gray-600 font-mono">{field.name}</p>
                      <p className="text-sm text-gray-600 mt-1">{field.description}</p>
                    </div>
                  </div>
                  <div className="flex items-center space-x-2">
                    <Badge className={getSensitivityColor(field.sensitivity)}>
                      {field.sensitivity}
                    </Badge>
                    <Badge className={getCategoryColor(field.category)}>
                      {field.category}
                    </Badge>
                    <Badge variant="outline">
                      {field.dataType}
                    </Badge>
                  </div>
                </div>

                <div className="grid grid-cols-2 md:grid-cols-4 gap-4 mb-4">
                  <div>
                    <div className="text-sm text-gray-600">Active Rules</div>
                    <div className="font-medium text-gray-900">
                      {field.redactionRules.filter(r => r.enabled).length} / {field.redactionRules.length}
                    </div>
                  </div>
                  <div>
                    <div className="text-sm text-gray-600">Applications</div>
                    <div className="font-medium text-gray-900">
                      {formatNumber(field.redactionRules.reduce((sum, r) => sum + r.applicationCount, 0))}
                    </div>
                  </div>
                  <div>
                    <div className="text-sm text-gray-600">Last Accessed</div>
                    <div className="font-medium text-gray-900">
                      {field.lastAccessed ? field.lastAccessed.toLocaleDateString() : 'Never'}
                    </div>
                  </div>
                  <div>
                    <div className="text-sm text-gray-600">Retention</div>
                    <div className="font-medium text-gray-900">
                      {Math.round(field.retentionPolicy.retentionDays / 365 * 10) / 10}y
                    </div>
                  </div>
                </div>

                {/* Redaction Rules */}
                <div className="mb-4">
                  <div className="text-sm font-medium text-gray-900 mb-2">Redaction Rules</div>
                  <div className="grid gap-2">
                    {field.redactionRules.map(rule => (
                      <div key={rule.id} className="flex items-center justify-between p-3 bg-gray-50 rounded">
                        <div className="flex items-center space-x-3">
                          {getMethodIcon(rule.method)}
                          <div>
                            <div className="text-sm font-medium">{rule.name}</div>
                            <div className="text-xs text-gray-600">{rule.description}</div>
                          </div>
                        </div>
                        <div className="flex items-center space-x-3">
                          <div className="text-xs text-gray-600">
                            {formatNumber(rule.applicationCount)} applications
                          </div>
                          {mode === 'configure' && (
                            <Switch
                              checked={rule.enabled}
                              onCheckedChange={(enabled) => handleRuleToggle(field.id, rule.id, enabled)}
                            />
                          )}
                          <Badge className={rule.enabled ? 'bg-green-100 text-green-800' : 'bg-gray-100 text-gray-800'}>
                            {rule.enabled ? 'Enabled' : 'Disabled'}
                          </Badge>
                        </div>
                      </div>
                    ))}
                  </div>
                </div>

                {/* Sources & Compliance */}
                <div className="grid grid-cols-2 gap-6 mb-4">
                  <div>
                    <div className="text-sm font-medium text-gray-900 mb-2">Data Sources</div>
                    <div className="flex flex-wrap gap-1">
                      {field.sources.slice(0, 3).map(source => (
                        <Badge key={source} variant="outline" size="sm">
                          {source}
                        </Badge>
                      ))}
                      {field.sources.length > 3 && (
                        <Badge variant="outline" size="sm">+{field.sources.length - 3}</Badge>
                      )}
                    </div>
                  </div>
                  <div>
                    <div className="text-sm font-medium text-gray-900 mb-2">Compliance</div>
                    <div className="flex flex-wrap gap-1">
                      {field.complianceFrameworks.map(framework => (
                        <Badge key={framework} variant="outline" size="sm">
                          {framework}
                        </Badge>
                      ))}
                    </div>
                  </div>
                </div>

                <div className="flex items-center justify-between pt-4 border-t">
                  <div className="text-sm text-gray-600">
                    Updated: {field.updatedAt.toLocaleDateString()} • 
                    Access Count: {formatNumber(field.accessCount)}
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

            {filteredFields.length === 0 && (
              <div className="text-center py-12">
                <Shield className="w-16 h-16 text-gray-400 mx-auto mb-4" />
                <h3 className="text-lg font-medium text-gray-900 mb-2">No PII Fields Found</h3>
                <p className="text-gray-600">
                  {filters.search || filters.category || filters.sensitivity || filters.status || filters.source
                    ? 'No PII fields match your current filters'
                    : 'No PII fields configured for this entity'
                  }
                </p>
              </div>
            )}
          </div>
        </TabsContent>

        <TabsContent value="rules" className="space-y-4">
          <div className="text-center py-8">
            <Settings className="w-16 h-16 text-gray-400 mx-auto mb-4" />
            <h3 className="text-lg font-medium text-gray-900 mb-2">Redaction Rules Management</h3>
            <p className="text-gray-600">
              Detailed rule configuration and management interface would be implemented here.
            </p>
          </div>
        </TabsContent>

        <TabsContent value="policies" className="space-y-4">
          <div className="grid gap-4">
            {exportPolicies.map(policy => (
              <Card key={policy.id} className="p-4">
                <div className="flex items-start justify-between mb-3">
                  <div>
                    <h3 className="text-lg font-medium text-gray-900">{policy.name}</h3>
                    <p className="text-sm text-gray-600">{policy.description}</p>
                  </div>
                  <div className="flex items-center space-x-2">
                    <Badge className={policy.enabled ? 'bg-green-100 text-green-800' : 'bg-gray-100 text-gray-800'}>
                      {policy.enabled ? 'Enabled' : 'Disabled'}
                    </Badge>
                    <Badge variant="outline">
                      {policy.redactionLevel}
                    </Badge>
                  </div>
                </div>

                <div className="grid grid-cols-2 md:grid-cols-4 gap-4 mb-4">
                  <div>
                    <div className="text-sm text-gray-600">Formats</div>
                    <div className="font-medium text-gray-900">
                      {policy.allowedFormats.join(', ')}
                    </div>
                  </div>
                  <div>
                    <div className="text-sm text-gray-600">Approval Required</div>
                    <div className="font-medium text-gray-900">
                      {policy.approvalRequired ? 'Yes' : 'No'}
                    </div>
                  </div>
                  <div>
                    <div className="text-sm text-gray-600">Audit Required</div>
                    <div className="font-medium text-gray-900">
                      {policy.auditRequired ? 'Yes' : 'No'}
                    </div>
                  </div>
                  <div>
                    <div className="text-sm text-gray-600">Retention</div>
                    <div className="font-medium text-gray-900">
                      {policy.retentionDays} days
                    </div>
                  </div>
                </div>

                {policy.approvers.length > 0 && (
                  <div className="mb-3">
                    <div className="text-sm font-medium text-gray-900 mb-1">Approvers</div>
                    <div className="flex flex-wrap gap-1">
                      {policy.approvers.map(approver => (
                        <Badge key={approver} variant="outline" size="sm">
                          {approver}
                        </Badge>
                      ))}
                    </div>
                  </div>
                )}
              </Card>
            ))}
          </div>
        </TabsContent>

        <TabsContent value="compliance" className="space-y-4">
          <div className="text-center py-8">
            <FileText className="w-16 h-16 text-gray-400 mx-auto mb-4" />
            <h3 className="text-lg font-medium text-gray-900 mb-2">Compliance Dashboard</h3>
            <p className="text-gray-600">
              Compliance reporting and audit trail interface would be implemented here.
            </p>
          </div>
        </TabsContent>
      </Tabs>

      {/* Field Details Dialog */}
      <Dialog open={showFieldDialog} onOpenChange={setShowFieldDialog}>
        <DialogContent className="max-w-4xl max-h-[90vh] overflow-y-auto">
          <DialogHeader>
            <DialogTitle>PII Field Details: {selectedField?.displayName}</DialogTitle>
          </DialogHeader>

          {selectedField && (
            <div className="space-y-6">
              {/* Basic Information */}
              <div className="grid grid-cols-2 gap-6">
                <Card className="p-4">
                  <h3 className="text-lg font-medium text-gray-900 mb-3">Field Information</h3>
                  <div className="space-y-2 text-sm">
                    <div className="flex justify-between">
                      <span className="text-gray-600">Name:</span>
                      <span className="font-medium font-mono">{selectedField.name}</span>
                    </div>
                    <div className="flex justify-between">
                      <span className="text-gray-600">Display Name:</span>
                      <span className="font-medium">{selectedField.displayName}</span>
                    </div>
                    <div className="flex justify-between">
                      <span className="text-gray-600">Category:</span>
                      <Badge className={getCategoryColor(selectedField.category)}>
                        {selectedField.category}
                      </Badge>
                    </div>
                    <div className="flex justify-between">
                      <span className="text-gray-600">Data Type:</span>
                      <span className="font-medium">{selectedField.dataType}</span>
                    </div>
                    <div className="flex justify-between">
                      <span className="text-gray-600">Sensitivity:</span>
                      <Badge className={getSensitivityColor(selectedField.sensitivity)}>
                        {selectedField.sensitivity}
                      </Badge>
                    </div>
                  </div>
                </Card>

                <Card className="p-4">
                  <h3 className="text-lg font-medium text-gray-900 mb-3">Usage Statistics</h3>
                  <div className="space-y-2 text-sm">
                    <div className="flex justify-between">
                      <span className="text-gray-600">Access Count:</span>
                      <span className="font-medium">{formatNumber(selectedField.accessCount)}</span>
                    </div>
                    <div className="flex justify-between">
                      <span className="text-gray-600">Last Accessed:</span>
                      <span className="font-medium">
                        {selectedField.lastAccessed ? selectedField.lastAccessed.toLocaleDateString() : 'Never'}
                      </span>
                    </div>
                    <div className="flex justify-between">
                      <span className="text-gray-600">Active Rules:</span>
                      <span className="font-medium">
                        {selectedField.redactionRules.filter(r => r.enabled).length}
                      </span>
                    </div>
                    <div className="flex justify-between">
                      <span className="text-gray-600">Total Applications:</span>
                      <span className="font-medium">
                        {formatNumber(selectedField.redactionRules.reduce((sum, r) => sum + r.applicationCount, 0))}
                      </span>
                    </div>
                  </div>
                </Card>
              </div>

              {/* Description */}
              <Card className="p-4">
                <h3 className="text-lg font-medium text-gray-900 mb-3">Description</h3>
                <p className="text-gray-600">{selectedField.description}</p>
              </Card>

              {/* Redaction Rules */}
              <Card className="p-4">
                <h3 className="text-lg font-medium text-gray-900 mb-3">Redaction Rules</h3>
                <div className="space-y-4">
                  {selectedField.redactionRules.map(rule => (
                    <div key={rule.id} className="p-4 border rounded">
                      <div className="flex items-center justify-between mb-3">
                        <div className="flex items-center space-x-3">
                          {getMethodIcon(rule.method)}
                          <div>
                            <h4 className="font-medium text-gray-900">{rule.name}</h4>
                            <p className="text-sm text-gray-600">{rule.description}</p>
                          </div>
                        </div>
                        <div className="flex items-center space-x-2">
                          <Badge variant="outline">{rule.method}</Badge>
                          <Badge className={rule.enabled ? 'bg-green-100 text-green-800' : 'bg-gray-100 text-gray-800'}>
                            {rule.enabled ? 'Enabled' : 'Disabled'}
                          </Badge>
                        </div>
                      </div>

                      <div className="grid grid-cols-3 gap-4 mb-3 text-sm">
                        <div>
                          <span className="text-gray-600">Priority:</span>
                          <div className="font-medium">{rule.priority}</div>
                        </div>
                        <div>
                          <span className="text-gray-600">Applications:</span>
                          <div className="font-medium">{formatNumber(rule.applicationCount)}</div>
                        </div>
                        <div>
                          <span className="text-gray-600">Last Applied:</span>
                          <div className="font-medium">
                            {rule.lastApplied ? rule.lastApplied.toLocaleDateString() : 'Never'}
                          </div>
                        </div>
                      </div>

                      {/* Contexts */}
                      <div className="mb-3">
                        <div className="text-sm font-medium text-gray-900 mb-2">Applies to Contexts:</div>
                        <div className="flex flex-wrap gap-1">
                          {rule.contexts.filter(c => c.applies).map(context => (
                            <Badge key={context.name} variant="outline" size="sm">
                              {context.name}
                            </Badge>
                          ))}
                        </div>
                      </div>

                      {/* Parameters */}
                      <div className="mb-3">
                        <div className="text-sm font-medium text-gray-900 mb-2">Parameters:</div>
                        <div className="text-xs text-gray-600 bg-gray-50 p-2 rounded font-mono">
                          {JSON.stringify(rule.parameters, null, 2)}
                        </div>
                      </div>

                      {mode === 'configure' && (
                        <div className="flex items-center justify-between pt-3 border-t">
                          <Button
                            variant="outline"
                            size="sm"
                            onClick={() => handleApplyRule(selectedField, rule)}
                          >
                            Test Rule
                          </Button>
                          <Switch
                            checked={rule.enabled}
                            onCheckedChange={(enabled) => handleRuleToggle(selectedField.id, rule.id, enabled)}
                          />
                        </div>
                      )}
                    </div>
                  ))}
                </div>
              </Card>

              {/* Examples */}
              {selectedField.examples.length > 0 && (
                <Card className="p-4">
                  <h3 className="text-lg font-medium text-gray-900 mb-3">Examples</h3>
                  <div className="space-y-3">
                    {selectedField.examples.map(example => (
                      <div key={example.id} className="p-3 bg-gray-50 rounded">
                        <div className="flex items-center justify-between mb-2">
                          <span className="font-medium text-sm">{example.name}</span>
                          <Badge variant="outline" size="sm">{example.method}</Badge>
                        </div>
                        <div className="grid grid-cols-2 gap-4 text-sm">
                          <div>
                            <span className="text-gray-600">Original:</span>
                            <div className="font-mono bg-white p-2 rounded mt-1">{example.originalValue}</div>
                          </div>
                          <div>
                            <span className="text-gray-600">Redacted:</span>
                            <div className="font-mono bg-white p-2 rounded mt-1">{example.redactedValue}</div>
                          </div>
                        </div>
                      </div>
                    ))}
                  </div>
                </Card>
              )}
            </div>
          )}

          <DialogFooter>
            <Button variant="outline" onClick={() => setShowFieldDialog(false)}>
              Close
            </Button>
            {mode === 'configure' && (
              <Button>
                <Edit3 className="w-4 h-4 mr-2" />
                Edit Field
              </Button>
            )}
          </DialogFooter>
        </DialogContent>
      </Dialog>

      {/* Export Policy Dialog */}
      <Dialog open={showExportDialog} onOpenChange={setShowExportDialog}>
        <DialogContent className="max-w-2xl">
          <DialogHeader>
            <DialogTitle>Export with PII Redaction</DialogTitle>
          </DialogHeader>

          <div className="space-y-4">
            <div>
              <Label htmlFor="export-policy">Select Export Policy</Label>
              <Select>
                <option value="">Choose a policy...</option>
                {exportPolicies.filter(p => p.enabled).map(policy => (
                  <option key={policy.id} value={policy.id}>
                    {policy.name} - {policy.redactionLevel} redaction
                  </option>
                ))}
              </Select>
            </div>

            <div>
              <Label htmlFor="export-format">Export Format</Label>
              <Select>
                <option value="csv">CSV</option>
                <option value="json">JSON</option>
                <option value="xml">XML</option>
                <option value="parquet">Parquet</option>
              </Select>
            </div>

            <div>
              <Label htmlFor="export-purpose">Purpose of Export</Label>
              <Textarea
                id="export-purpose"
                placeholder="Describe the purpose and intended use of this data export..."
                rows={3}
              />
            </div>

            <Alert>
              <AlertTriangle className="h-4 w-4" />
              <AlertDescription>
                This export will apply the selected redaction policy. All PII will be processed according to the policy rules. 
                The export will be audited and retained according to compliance requirements.
              </AlertDescription>
            </Alert>
          </div>

          <DialogFooter>
            <Button variant="outline" onClick={() => setShowExportDialog(false)}>
              Cancel
            </Button>
            <Button>
              <Download className="w-4 h-4 mr-2" />
              Export with Redaction
            </Button>
          </DialogFooter>
        </DialogContent>
      </Dialog>
    </div>
  )
}

export default PIIRedactionManager
