import React, { useState, useCallback, useEffect, useMemo } from 'react'
import { 
  Shield, Lock, Unlock, Globe, Key, Clock, AlertTriangle, CheckCircle, 
  XCircle, Wifi, Server, Database, Eye, EyeOff, Settings, RefreshCw,
  Download, Upload, Activity, Zap, FileText, Users, Network, Terminal,
  Bug, Wrench, Package, History, MoreHorizontal, Play, Pause, Info
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
interface SecurityNFR {
  id: string
  name: string
  category: 'authentication' | 'authorization' | 'encryption' | 'network' | 'application' | 'compliance'
  description: string
  requirement: string
  currentStatus: 'compliant' | 'partial' | 'non_compliant' | 'not_implemented'
  targetStatus: 'compliant'
  priority: 'critical' | 'high' | 'medium' | 'low'
  implementation: SecurityImplementation
  metrics: SecurityMetrics
  tests: SecurityTest[]
  remediation?: string
  lastChecked: Date
  nextCheck: Date
  owner: string
  dependencies: string[]
  riskLevel: 'critical' | 'high' | 'medium' | 'low'
}

interface SecurityImplementation {
  technology: string
  configuration: Record<string, any>
  enabled: boolean
  version?: string
  lastUpdated: Date
  endpoints: string[]
  certificates?: CertificateInfo[]
  policies?: PolicyConfiguration[]
}

interface CertificateInfo {
  id: string
  commonName: string
  issuer: string
  validFrom: Date
  validTo: Date
  algorithm: string
  keySize: number
  status: 'valid' | 'expiring' | 'expired' | 'revoked'
  daysUntilExpiry: number
  usage: string[]
}

interface PolicyConfiguration {
  id: string
  name: string
  type: string
  rules: PolicyRule[]
  enabled: boolean
  lastModified: Date
}

interface PolicyRule {
  id: string
  condition: string
  action: 'allow' | 'deny' | 'log' | 'rate_limit'
  parameters: Record<string, any>
  enabled: boolean
}

interface SecurityMetrics {
  uptime: number // percentage
  responseTime: number // milliseconds
  throughput: number // requests per second
  errorRate: number // percentage
  lastIncident?: Date
  mtbf: number // mean time between failures in hours
  mttr: number // mean time to recovery in minutes
  slaCompliance: number // percentage
}

interface SecurityTest {
  id: string
  name: string
  type: 'automated' | 'manual' | 'penetration' | 'vulnerability_scan'
  status: 'passed' | 'failed' | 'running' | 'pending'
  lastRun: Date
  nextRun: Date
  result?: TestResult
  frequency: 'daily' | 'weekly' | 'monthly' | 'quarterly'
}

interface TestResult {
  score: number
  findings: SecurityFinding[]
  recommendations: string[]
  executionTime: number
  coverage: number
}

interface SecurityFinding {
  id: string
  severity: 'critical' | 'high' | 'medium' | 'low' | 'info'
  title: string
  description: string
  evidence: string
  remediation: string
  cveId?: string
  cvssScore?: number
}

interface mTLSConfiguration {
  enabled: boolean
  caCertificate: string
  clientCertificates: CertificateInfo[]
  serverCertificates: CertificateInfo[]
  mutualAuthRequired: boolean
  certRotationPolicy: CertRotationPolicy
  trustedCAs: string[]
  revocationCheck: boolean
  ocspEnabled: boolean
}

interface CertRotationPolicy {
  autoRotate: boolean
  rotationThreshold: number // days before expiry
  notificationThreshold: number // days before expiry to notify
  backupCount: number
  validationRequired: boolean
}

interface SSOConfiguration {
  provider: 'saml' | 'oidc' | 'oauth2' | 'ldap'
  enabled: boolean
  entityId: string
  ssoUrl: string
  certificateFingerprint: string
  attributeMapping: Record<string, string>
  sessionTimeout: number
  forceReauth: boolean
  mfaRequired: boolean
  allowedDomains: string[]
}

interface CSPConfiguration {
  enabled: boolean
  directives: CSPDirective[]
  reportUri?: string
  reportOnly: boolean
  violations: CSPViolation[]
}

interface CSPDirective {
  directive: string
  sources: string[]
  enabled: boolean
}

interface CSPViolation {
  id: string
  timestamp: Date
  directive: string
  violatedDirective: string
  blockedUri: string
  sourceFile?: string
  lineNumber?: number
  userAgent: string
  ipAddress: string
}

interface RateLimitConfiguration {
  enabled: boolean
  policies: RateLimitPolicy[]
  globalLimits: RateLimit[]
  bypassRules: BypassRule[]
}

interface RateLimitPolicy {
  id: string
  name: string
  path: string
  method: string
  limit: number
  window: number // seconds
  burst: number
  keyBy: 'ip' | 'user' | 'api_key' | 'custom'
  enabled: boolean
  action: 'block' | 'delay' | 'log'
}

interface RateLimit {
  type: 'global' | 'per_endpoint' | 'per_user'
  limit: number
  window: number
  currentUsage: number
  resetTime: Date
}

interface BypassRule {
  id: string
  condition: string
  reason: string
  enabled: boolean
  expiresAt?: Date
}

interface SessionConfiguration {
  timeout: number // minutes
  slidingExpiration: boolean
  secureOnly: boolean
  httpOnly: boolean
  sameSite: 'strict' | 'lax' | 'none'
  domain?: string
  path: string
  renewalThreshold: number // minutes before expiry
  concurrentSessions: number
  deviceTracking: boolean
}

interface SecurityNFRManagerProps {
  serviceId?: string
  onComplianceChanged?: (nfr: SecurityNFR, status: string) => void
  onSecurityIncident?: (incident: any) => void
  className?: string
}

const SecurityNFRManager: React.FC<SecurityNFRManagerProps> = ({
  serviceId,
  onComplianceChanged,
  onSecurityIncident,
  className = ''
}) => {
  // State
  const [securityNFRs, setSecurityNFRs] = useState<SecurityNFR[]>([])
  const [selectedNFR, setSelectedNFR] = useState<SecurityNFR | null>(null)
  const [showDetailsDialog, setShowDetailsDialog] = useState(false)
  const [showConfigDialog, setShowConfigDialog] = useState(false)
  const [activeTab, setActiveTab] = useState('overview')
  const [filters, setFilters] = useState({
    category: '',
    status: '',
    priority: '',
    search: ''
  })
  const [isRunningTests, setIsRunningTests] = useState(false)
  const [testProgress, setTestProgress] = useState(0)

  // Mock data initialization
  const initializeMockData = useCallback(() => {
    const mockNFRs: SecurityNFR[] = [
      {
        id: 'nfr-mtls',
        name: 'Mutual TLS (mTLS) Communication',
        category: 'encryption',
        description: 'All service-to-service communication must use mutual TLS authentication',
        requirement: 'mTLS with certificate validation for all internal API communications',
        currentStatus: 'compliant',
        targetStatus: 'compliant',
        priority: 'critical',
        implementation: {
          technology: 'Istio Service Mesh + cert-manager',
          configuration: {
            mtls: {
              enabled: true,
              caCertificate: 'LS0tLS1CRUdJTi...',
              clientCertificates: [],
              serverCertificates: [],
              mutualAuthRequired: true,
              certRotationPolicy: {
                autoRotate: true,
                rotationThreshold: 30,
                notificationThreshold: 7,
                backupCount: 3,
                validationRequired: true
              },
              trustedCAs: ['internal-ca', 'istio-ca'],
              revocationCheck: true,
              ocspEnabled: true
            } as mTLSConfiguration
          },
          enabled: true,
          version: '1.19.3',
          lastUpdated: new Date('2024-11-20T10:00:00Z'),
          endpoints: [
            'https://fleet-manager.internal:8443',
            'https://vehicle-service.internal:8443',
            'https://telemetry-service.internal:8443'
          ],
          certificates: [
            {
              id: 'cert-fleet-manager',
              commonName: 'fleet-manager.atlasmesh.internal',
              issuer: 'AtlasMesh Internal CA',
              validFrom: new Date('2024-11-01T00:00:00Z'),
              validTo: new Date('2025-11-01T00:00:00Z'),
              algorithm: 'RSA-SHA256',
              keySize: 2048,
              status: 'valid',
              daysUntilExpiry: 340,
              usage: ['digital_signature', 'key_encipherment', 'server_auth', 'client_auth']
            },
            {
              id: 'cert-vehicle-service',
              commonName: 'vehicle-service.atlasmesh.internal',
              issuer: 'AtlasMesh Internal CA',
              validFrom: new Date('2024-10-15T00:00:00Z'),
              validTo: new Date('2024-12-15T00:00:00Z'),
              algorithm: 'RSA-SHA256',
              keySize: 2048,
              status: 'expiring',
              daysUntilExpiry: 19,
              usage: ['digital_signature', 'key_encipherment', 'server_auth', 'client_auth']
            }
          ]
        },
        metrics: {
          uptime: 99.98,
          responseTime: 45,
          throughput: 1250,
          errorRate: 0.02,
          lastIncident: new Date('2024-11-15T08:30:00Z'),
          mtbf: 720, // 30 days
          mttr: 15, // 15 minutes
          slaCompliance: 99.95
        },
        tests: [
          {
            id: 'test-mtls-handshake',
            name: 'mTLS Handshake Validation',
            type: 'automated',
            status: 'passed',
            lastRun: new Date('2024-11-26T08:00:00Z'),
            nextRun: new Date('2024-11-26T20:00:00Z'),
            frequency: 'daily',
            result: {
              score: 98,
              findings: [],
              recommendations: ['Consider upgrading to TLS 1.3 for better performance'],
              executionTime: 1200,
              coverage: 95
            }
          }
        ],
        lastChecked: new Date('2024-11-26T10:00:00Z'),
        nextCheck: new Date('2024-11-26T22:00:00Z'),
        owner: 'security-team',
        dependencies: ['cert-manager', 'istio-system'],
        riskLevel: 'critical'
      },
      {
        id: 'nfr-sso-saml',
        name: 'Single Sign-On (SSO) Authentication',
        category: 'authentication',
        description: 'All user authentication must use enterprise SSO with SAML/OIDC',
        requirement: 'SAML 2.0 or OIDC integration with multi-factor authentication',
        currentStatus: 'compliant',
        targetStatus: 'compliant',
        priority: 'high',
        implementation: {
          technology: 'Auth0 + SAML/OIDC',
          configuration: {
            sso: {
              provider: 'saml',
              enabled: true,
              entityId: 'https://atlasmesh.auth0.com',
              ssoUrl: 'https://atlasmesh.auth0.com/samlp/xyz123',
              certificateFingerprint: 'A1:B2:C3:D4:E5:F6:...',
              attributeMapping: {
                'email': 'http://schemas.xmlsoap.org/ws/2005/05/identity/claims/emailaddress',
                'firstName': 'http://schemas.xmlsoap.org/ws/2005/05/identity/claims/givenname',
                'lastName': 'http://schemas.xmlsoap.org/ws/2005/05/identity/claims/surname',
                'role': 'http://schemas.microsoft.com/ws/2008/06/identity/claims/role'
              },
              sessionTimeout: 480, // 8 hours
              forceReauth: false,
              mfaRequired: true,
              allowedDomains: ['atlasmesh.com', 'contractor.atlasmesh.com']
            } as SSOConfiguration
          },
          enabled: true,
          version: '2.0',
          lastUpdated: new Date('2024-11-18T14:30:00Z'),
          endpoints: [
            'https://control-center.atlasmesh.com/auth/saml',
            'https://api.atlasmesh.com/auth/oidc'
          ]
        },
        metrics: {
          uptime: 99.95,
          responseTime: 850,
          throughput: 125,
          errorRate: 0.05,
          mtbf: 1440, // 60 days
          mttr: 30, // 30 minutes
          slaCompliance: 99.90
        },
        tests: [
          {
            id: 'test-sso-auth',
            name: 'SSO Authentication Flow',
            type: 'automated',
            status: 'passed',
            lastRun: new Date('2024-11-26T06:00:00Z'),
            nextRun: new Date('2024-11-27T06:00:00Z'),
            frequency: 'daily'
          }
        ],
        lastChecked: new Date('2024-11-26T09:00:00Z'),
        nextCheck: new Date('2024-11-26T21:00:00Z'),
        owner: 'identity-team',
        dependencies: ['auth0-service'],
        riskLevel: 'high'
      },
      {
        id: 'nfr-csp-csrf',
        name: 'Content Security Policy & CSRF Protection',
        category: 'application',
        description: 'Web applications must implement CSP headers and CSRF protection',
        requirement: 'Strict CSP policy with nonce-based script execution and CSRF tokens',
        currentStatus: 'partial',
        targetStatus: 'compliant',
        priority: 'high',
        implementation: {
          technology: 'Nginx + Application Middleware',
          configuration: {
            csp: {
              enabled: true,
              directives: [
                {
                  directive: 'default-src',
                  sources: ["'self'"],
                  enabled: true
                },
                {
                  directive: 'script-src',
                  sources: ["'self'", "'nonce-{random}'", 'https://cdn.atlasmesh.com'],
                  enabled: true
                },
                {
                  directive: 'style-src',
                  sources: ["'self'", "'unsafe-inline'", 'https://fonts.googleapis.com'],
                  enabled: true
                },
                {
                  directive: 'img-src',
                  sources: ["'self'", 'data:', 'https:'],
                  enabled: true
                },
                {
                  directive: 'connect-src',
                  sources: ["'self'", 'https://api.atlasmesh.com'],
                  enabled: true
                }
              ],
              reportUri: 'https://api.atlasmesh.com/csp-report',
              reportOnly: false,
              violations: [
                {
                  id: 'viol-001',
                  timestamp: new Date('2024-11-26T09:15:00Z'),
                  directive: 'script-src',
                  violatedDirective: "script-src 'self' 'nonce-abc123'",
                  blockedUri: 'https://malicious-site.com/script.js',
                  sourceFile: 'https://control-center.atlasmesh.com/dashboard',
                  lineNumber: 42,
                  userAgent: 'Mozilla/5.0...',
                  ipAddress: '192.168.1.100'
                }
              ]
            } as CSPConfiguration,
            csrf: {
              enabled: true,
              tokenLength: 32,
              cookieName: '__csrf_token',
              headerName: 'X-CSRF-Token',
              sameSite: 'strict',
              secure: true,
              httpOnly: true
            }
          },
          enabled: true,
          lastUpdated: new Date('2024-11-22T16:00:00Z'),
          endpoints: [
            'https://control-center.atlasmesh.com',
            'https://api.atlasmesh.com'
          ]
        },
        metrics: {
          uptime: 99.92,
          responseTime: 120,
          throughput: 2500,
          errorRate: 0.08,
          mtbf: 360, // 15 days
          mttr: 10, // 10 minutes
          slaCompliance: 99.85
        },
        tests: [
          {
            id: 'test-csp-policy',
            name: 'CSP Policy Validation',
            type: 'automated',
            status: 'failed',
            lastRun: new Date('2024-11-26T07:30:00Z'),
            nextRun: new Date('2024-11-26T19:30:00Z'),
            frequency: 'daily',
            result: {
              score: 75,
              findings: [
                {
                  id: 'find-001',
                  severity: 'medium',
                  title: 'Unsafe inline styles detected',
                  description: 'Application uses unsafe-inline for styles which reduces CSP effectiveness',
                  evidence: 'style-src directive contains unsafe-inline',
                  remediation: 'Replace inline styles with external stylesheets or use nonce-based approach',
                  cvssScore: 4.5
                }
              ],
              recommendations: [
                'Remove unsafe-inline from style-src directive',
                'Implement nonce-based style loading',
                'Add Content-Security-Policy-Report-Only header for testing'
              ],
              executionTime: 800,
              coverage: 85
            }
          }
        ],
        remediation: 'Remove unsafe-inline directive and implement nonce-based CSP for styles',
        lastChecked: new Date('2024-11-26T07:30:00Z'),
        nextCheck: new Date('2024-11-26T19:30:00Z'),
        owner: 'frontend-team',
        dependencies: ['nginx-ingress', 'web-application'],
        riskLevel: 'medium'
      },
      {
        id: 'nfr-rate-limiting',
        name: 'API Rate Limiting',
        category: 'network',
        description: 'All public APIs must implement rate limiting to prevent abuse',
        requirement: 'Configurable rate limits per endpoint with burst handling and monitoring',
        currentStatus: 'compliant',
        targetStatus: 'compliant',
        priority: 'medium',
        implementation: {
          technology: 'Kong API Gateway',
          configuration: {
            rateLimiting: {
              enabled: true,
              policies: [
                {
                  id: 'policy-auth-api',
                  name: 'Authentication API',
                  path: '/api/v1/auth/*',
                  method: 'POST',
                  limit: 10,
                  window: 60, // 1 minute
                  burst: 5,
                  keyBy: 'ip',
                  enabled: true,
                  action: 'block'
                },
                {
                  id: 'policy-fleet-api',
                  name: 'Fleet Management API',
                  path: '/api/v1/fleet/*',
                  method: 'GET',
                  limit: 1000,
                  window: 60,
                  burst: 100,
                  keyBy: 'api_key',
                  enabled: true,
                  action: 'delay'
                }
              ],
              globalLimits: [
                {
                  type: 'global',
                  limit: 10000,
                  window: 60,
                  currentUsage: 3247,
                  resetTime: new Date('2024-11-26T10:31:00Z')
                }
              ],
              bypassRules: [
                {
                  id: 'bypass-monitoring',
                  condition: 'source_ip == "192.168.1.0/24"',
                  reason: 'Internal monitoring systems',
                  enabled: true
                }
              ]
            } as RateLimitConfiguration
          },
          enabled: true,
          version: '3.4.2',
          lastUpdated: new Date('2024-11-19T11:00:00Z'),
          endpoints: [
            'https://api.atlasmesh.com'
          ]
        },
        metrics: {
          uptime: 99.99,
          responseTime: 25,
          throughput: 8500,
          errorRate: 0.01,
          mtbf: 2160, // 90 days
          mttr: 5, // 5 minutes
          slaCompliance: 99.98
        },
        tests: [
          {
            id: 'test-rate-limits',
            name: 'Rate Limit Enforcement',
            type: 'automated',
            status: 'passed',
            lastRun: new Date('2024-11-26T08:30:00Z'),
            nextRun: new Date('2024-11-26T20:30:00Z'),
            frequency: 'daily'
          }
        ],
        lastChecked: new Date('2024-11-26T10:30:00Z'),
        nextCheck: new Date('2024-11-26T22:30:00Z'),
        owner: 'api-team',
        dependencies: ['kong-gateway'],
        riskLevel: 'medium'
      },
      {
        id: 'nfr-session-mgmt',
        name: 'Secure Session Management',
        category: 'authentication',
        description: 'Web applications must implement secure session management with proper timeouts',
        requirement: 'Secure session cookies with configurable timeouts and concurrent session limits',
        currentStatus: 'compliant',
        targetStatus: 'compliant',
        priority: 'medium',
        implementation: {
          technology: 'Redis Session Store + Express Session',
          configuration: {
            session: {
              timeout: 480, // 8 hours
              slidingExpiration: true,
              secureOnly: true,
              httpOnly: true,
              sameSite: 'strict',
              domain: '.atlasmesh.com',
              path: '/',
              renewalThreshold: 60, // 1 hour before expiry
              concurrentSessions: 3,
              deviceTracking: true
            } as SessionConfiguration
          },
          enabled: true,
          version: '1.17.3',
          lastUpdated: new Date('2024-11-21T09:00:00Z'),
          endpoints: [
            'https://control-center.atlasmesh.com'
          ]
        },
        metrics: {
          uptime: 99.96,
          responseTime: 15,
          throughput: 450,
          errorRate: 0.04,
          mtbf: 1080, // 45 days
          mttr: 8, // 8 minutes
          slaCompliance: 99.92
        },
        tests: [
          {
            id: 'test-session-security',
            name: 'Session Security Validation',
            type: 'automated',
            status: 'passed',
            lastRun: new Date('2024-11-26T09:00:00Z'),
            nextRun: new Date('2024-11-26T21:00:00Z'),
            frequency: 'daily'
          }
        ],
        lastChecked: new Date('2024-11-26T10:15:00Z'),
        nextCheck: new Date('2024-11-26T22:15:00Z'),
        owner: 'frontend-team',
        dependencies: ['redis-cluster', 'web-application'],
        riskLevel: 'low'
      }
    ]

    setSecurityNFRs(mockNFRs)
  }, [])

  useEffect(() => {
    initializeMockData()
  }, [initializeMockData])

  // Real-time metrics updates
  useEffect(() => {
    const interval = setInterval(() => {
      setSecurityNFRs(prev => prev.map(nfr => ({
        ...nfr,
        metrics: {
          ...nfr.metrics,
          responseTime: Math.max(10, nfr.metrics.responseTime + (Math.random() - 0.5) * 10),
          throughput: Math.max(0, nfr.metrics.throughput + (Math.random() - 0.5) * nfr.metrics.throughput * 0.1),
          errorRate: Math.max(0, Math.min(5, nfr.metrics.errorRate + (Math.random() - 0.5) * 0.02)),
          uptime: Math.max(95, Math.min(100, nfr.metrics.uptime + (Math.random() - 0.5) * 0.1))
        }
      })))
    }, 5000)

    return () => clearInterval(interval)
  }, [])

  // Handlers
  const handleNFRSelect = useCallback((nfr: SecurityNFR) => {
    setSelectedNFR(nfr)
    setShowDetailsDialog(true)
  }, [])

  const handleRunSecurityTests = useCallback(async () => {
    setIsRunningTests(true)
    setTestProgress(0)

    // Simulate test execution
    const progressInterval = setInterval(() => {
      setTestProgress(prev => {
        const newProgress = prev + Math.random() * 15
        if (newProgress >= 100) {
          clearInterval(progressInterval)
          setIsRunningTests(false)
          return 100
        }
        return newProgress
      })
    }, 800)

    // Update test results after completion
    setTimeout(() => {
      clearInterval(progressInterval)
      setIsRunningTests(false)
      setTestProgress(100)
      
      setSecurityNFRs(prev => prev.map(nfr => ({
        ...nfr,
        tests: nfr.tests.map(test => ({
          ...test,
          status: Math.random() > 0.2 ? 'passed' : 'failed',
          lastRun: new Date(),
          nextRun: new Date(Date.now() + 24 * 60 * 60 * 1000)
        })),
        lastChecked: new Date()
      })))
    }, 12000)
  }, [])

  const handleToggleNFR = useCallback((nfrId: string, enabled: boolean) => {
    setSecurityNFRs(prev => prev.map(nfr => 
      nfr.id === nfrId 
        ? {
            ...nfr,
            implementation: {
              ...nfr.implementation,
              enabled
            },
            currentStatus: enabled ? 'compliant' : 'non_compliant',
            lastChecked: new Date()
          }
        : nfr
    ))

    const nfr = securityNFRs.find(n => n.id === nfrId)
    if (nfr) {
      onComplianceChanged?.(nfr, enabled ? 'compliant' : 'non_compliant')
    }
  }, [securityNFRs, onComplianceChanged])

  // Filtered NFRs
  const filteredNFRs = useMemo(() => {
    return securityNFRs
      .filter(nfr => !serviceId || nfr.implementation.endpoints.some(e => e.includes(serviceId)))
      .filter(nfr => !filters.category || nfr.category === filters.category)
      .filter(nfr => !filters.status || nfr.currentStatus === filters.status)
      .filter(nfr => !filters.priority || nfr.priority === filters.priority)
      .filter(nfr => {
        if (!filters.search) return true
        const searchTerm = filters.search.toLowerCase()
        return (
          nfr.name.toLowerCase().includes(searchTerm) ||
          nfr.description.toLowerCase().includes(searchTerm) ||
          nfr.requirement.toLowerCase().includes(searchTerm)
        )
      })
      .sort((a, b) => {
        // Sort by priority (critical first) then by status (non-compliant first)
        const priorityOrder = { critical: 4, high: 3, medium: 2, low: 1 }
        const statusOrder = { non_compliant: 4, not_implemented: 3, partial: 2, compliant: 1 }
        
        const aPriority = priorityOrder[a.priority]
        const bPriority = priorityOrder[b.priority]
        
        if (aPriority !== bPriority) {
          return bPriority - aPriority
        }
        
        const aStatus = statusOrder[a.currentStatus]
        const bStatus = statusOrder[b.currentStatus]
        
        return bStatus - aStatus
      })
  }, [securityNFRs, serviceId, filters])

  // Statistics
  const stats = useMemo(() => {
    const totalNFRs = securityNFRs.length
    const compliantNFRs = securityNFRs.filter(nfr => nfr.currentStatus === 'compliant').length
    const nonCompliantNFRs = securityNFRs.filter(nfr => nfr.currentStatus === 'non_compliant').length
    const criticalNFRs = securityNFRs.filter(nfr => nfr.priority === 'critical').length
    
    const avgUptime = securityNFRs.reduce((sum, nfr) => sum + nfr.metrics.uptime, 0) / totalNFRs || 0
    const avgResponseTime = securityNFRs.reduce((sum, nfr) => sum + nfr.metrics.responseTime, 0) / totalNFRs || 0
    const totalThroughput = securityNFRs.reduce((sum, nfr) => sum + nfr.metrics.throughput, 0)
    const avgErrorRate = securityNFRs.reduce((sum, nfr) => sum + nfr.metrics.errorRate, 0) / totalNFRs || 0
    
    const failedTests = securityNFRs.reduce((sum, nfr) => 
      sum + nfr.tests.filter(t => t.status === 'failed').length, 0
    )
    const totalTests = securityNFRs.reduce((sum, nfr) => sum + nfr.tests.length, 0)
    
    const expiringCerts = securityNFRs.reduce((sum, nfr) => 
      sum + (nfr.implementation.certificates?.filter(c => c.status === 'expiring').length || 0), 0
    )

    return {
      totalNFRs,
      compliantNFRs,
      nonCompliantNFRs,
      criticalNFRs,
      avgUptime,
      avgResponseTime,
      totalThroughput,
      avgErrorRate,
      failedTests,
      totalTests,
      expiringCerts
    }
  }, [securityNFRs])

  const getStatusColor = (status: string) => {
    switch (status) {
      case 'compliant': return 'bg-green-100 text-green-800'
      case 'partial': return 'bg-yellow-100 text-yellow-800'
      case 'non_compliant': return 'bg-red-100 text-red-800'
      case 'not_implemented': return 'bg-gray-100 text-gray-800'
      default: return 'bg-gray-100 text-gray-800'
    }
  }

  const getPriorityColor = (priority: string) => {
    switch (priority) {
      case 'critical': return 'bg-red-100 text-red-800'
      case 'high': return 'bg-orange-100 text-orange-800'
      case 'medium': return 'bg-yellow-100 text-yellow-800'
      case 'low': return 'bg-green-100 text-green-800'
      default: return 'bg-gray-100 text-gray-800'
    }
  }

  const getCategoryColor = (category: string) => {
    switch (category) {
      case 'authentication': return 'bg-blue-100 text-blue-800'
      case 'authorization': return 'bg-purple-100 text-purple-800'
      case 'encryption': return 'bg-green-100 text-green-800'
      case 'network': return 'bg-orange-100 text-orange-800'
      case 'application': return 'bg-pink-100 text-pink-800'
      case 'compliance': return 'bg-indigo-100 text-indigo-800'
      default: return 'bg-gray-100 text-gray-800'
    }
  }

  const formatNumber = (num: number) => {
    if (num >= 1000000) {
      return `${(num / 1000000).toFixed(1)}M`
    } else if (num >= 1000) {
      return `${(num / 1000).toFixed(1)}K`
    }
    return Math.round(num).toString()
  }

  return (
    <div className={`space-y-6 ${className}`}>
      {/* Header */}
      <div className="flex items-center justify-between">
        <div className="flex items-center space-x-3">
          <Shield className="w-6 h-6 text-blue-600" />
          <h2 className="text-2xl font-semibold text-gray-900">
            Security NFRs Manager
            {serviceId && <span className="text-lg text-gray-600 ml-2">for {serviceId}</span>}
          </h2>
        </div>
        <div className="flex items-center space-x-2">
          <Button 
            variant="outline" 
            onClick={handleRunSecurityTests}
            disabled={isRunningTests}
          >
            {isRunningTests ? (
              <>
                <Activity className="w-4 h-4 mr-2 animate-spin" />
                Running Tests... {Math.round(testProgress)}%
              </>
            ) : (
              <>
                <Bug className="w-4 h-4 mr-2" />
                Run Security Tests
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
      <div className="grid grid-cols-2 md:grid-cols-6 lg:grid-cols-11 gap-4">
        <Card className="p-4">
          <div className="text-center">
            <div className="text-2xl font-bold text-gray-900">{stats.totalNFRs}</div>
            <div className="text-sm text-gray-600">Total NFRs</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className="text-2xl font-bold text-green-600">{stats.compliantNFRs}</div>
            <div className="text-sm text-gray-600">Compliant</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className="text-2xl font-bold text-red-600">{stats.nonCompliantNFRs}</div>
            <div className="text-sm text-gray-600">Non-Compliant</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className="text-2xl font-bold text-orange-600">{stats.criticalNFRs}</div>
            <div className="text-sm text-gray-600">Critical</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className="text-2xl font-bold text-blue-600">{stats.avgUptime.toFixed(2)}%</div>
            <div className="text-sm text-gray-600">Avg Uptime</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className="text-2xl font-bold text-purple-600">{Math.round(stats.avgResponseTime)}ms</div>
            <div className="text-sm text-gray-600">Avg Response</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className="text-2xl font-bold text-indigo-600">{formatNumber(stats.totalThroughput)}</div>
            <div className="text-sm text-gray-600">Total RPS</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className="text-2xl font-bold text-pink-600">{stats.avgErrorRate.toFixed(2)}%</div>
            <div className="text-sm text-gray-600">Error Rate</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className="text-2xl font-bold text-yellow-600">{stats.failedTests}</div>
            <div className="text-sm text-gray-600">Failed Tests</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className="text-2xl font-bold text-teal-600">{stats.totalTests}</div>
            <div className="text-sm text-gray-600">Total Tests</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className="text-2xl font-bold text-amber-600">{stats.expiringCerts}</div>
            <div className="text-sm text-gray-600">Expiring Certs</div>
          </div>
        </Card>
      </div>

      {/* Test Progress */}
      {isRunningTests && (
        <Card className="p-4">
          <div className="flex items-center justify-between mb-2">
            <span className="font-medium text-gray-900">Running Security Tests</span>
            <span className="text-sm text-gray-600">{Math.round(testProgress)}%</span>
          </div>
          <Progress value={testProgress} className="w-full" />
        </Card>
      )}

      {/* Filters */}
      <Card className="p-4">
        <div className="flex flex-wrap items-center gap-4">
          <div className="flex items-center space-x-2">
            <Search className="w-4 h-4 text-gray-500" />
            <Input
              placeholder="Search NFRs..."
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
            <option value="authentication">Authentication</option>
            <option value="authorization">Authorization</option>
            <option value="encryption">Encryption</option>
            <option value="network">Network</option>
            <option value="application">Application</option>
            <option value="compliance">Compliance</option>
          </Select>

          <Select
            value={filters.status}
            onValueChange={(value) => setFilters(prev => ({ ...prev, status: value }))}
          >
            <option value="">All Status</option>
            <option value="compliant">Compliant</option>
            <option value="partial">Partial</option>
            <option value="non_compliant">Non-Compliant</option>
            <option value="not_implemented">Not Implemented</option>
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
        </div>
      </Card>

      {/* NFRs List */}
      <div className="space-y-4">
        {filteredNFRs.map(nfr => (
          <Card key={nfr.id} className="p-6 hover:shadow-md transition-shadow cursor-pointer"
                onClick={() => handleNFRSelect(nfr)}>
            <div className="flex items-start justify-between mb-4">
              <div className="flex items-center space-x-3">
                <div className={`p-2 rounded ${
                  nfr.currentStatus === 'compliant' ? 'bg-green-100' :
                  nfr.currentStatus === 'partial' ? 'bg-yellow-100' :
                  nfr.currentStatus === 'non_compliant' ? 'bg-red-100' : 'bg-gray-100'
                }`}>
                  <Shield className={`w-5 h-5 ${
                    nfr.currentStatus === 'compliant' ? 'text-green-600' :
                    nfr.currentStatus === 'partial' ? 'text-yellow-600' :
                    nfr.currentStatus === 'non_compliant' ? 'text-red-600' : 'text-gray-600'
                  }`} />
                </div>
                <div>
                  <h3 className="text-lg font-medium text-gray-900">{nfr.name}</h3>
                  <p className="text-sm text-gray-600">{nfr.description}</p>
                  <p className="text-sm text-gray-600 mt-1">
                    <span className="font-medium">Requirement:</span> {nfr.requirement}
                  </p>
                </div>
              </div>
              <div className="flex items-center space-x-2">
                <Badge className={getStatusColor(nfr.currentStatus)}>
                  {nfr.currentStatus.replace('_', ' ')}
                </Badge>
                <Badge className={getPriorityColor(nfr.priority)}>
                  {nfr.priority}
                </Badge>
                <Badge className={getCategoryColor(nfr.category)}>
                  {nfr.category}
                </Badge>
              </div>
            </div>

            <div className="grid grid-cols-2 md:grid-cols-4 gap-4 mb-4">
              <div>
                <div className="text-sm text-gray-600">Uptime</div>
                <div className="font-medium text-gray-900">
                  {nfr.metrics.uptime.toFixed(2)}%
                </div>
              </div>
              <div>
                <div className="text-sm text-gray-600">Response Time</div>
                <div className="font-medium text-gray-900">
                  {Math.round(nfr.metrics.responseTime)}ms
                </div>
              </div>
              <div>
                <div className="text-sm text-gray-600">Throughput</div>
                <div className="font-medium text-gray-900">
                  {formatNumber(nfr.metrics.throughput)} RPS
                </div>
              </div>
              <div>
                <div className="text-sm text-gray-600">Error Rate</div>
                <div className={`font-medium ${
                  nfr.metrics.errorRate > 1 ? 'text-red-600' :
                  nfr.metrics.errorRate > 0.5 ? 'text-yellow-600' : 'text-green-600'
                }`}>
                  {nfr.metrics.errorRate.toFixed(2)}%
                </div>
              </div>
            </div>

            {/* Implementation Details */}
            <div className="grid grid-cols-2 gap-6 mb-4">
              <div>
                <div className="text-sm font-medium text-gray-900 mb-2">Implementation</div>
                <div className="text-sm text-gray-600">
                  <span className="font-medium">Technology:</span> {nfr.implementation.technology}
                  {nfr.implementation.version && (
                    <span className="ml-2">v{nfr.implementation.version}</span>
                  )}
                </div>
                <div className="text-sm text-gray-600">
                  <span className="font-medium">Endpoints:</span> {nfr.implementation.endpoints.length}
                </div>
              </div>
              <div>
                <div className="text-sm font-medium text-gray-900 mb-2">Tests & Compliance</div>
                <div className="text-sm text-gray-600">
                  <span className="font-medium">Tests:</span> {nfr.tests.filter(t => t.status === 'passed').length}/{nfr.tests.length} passed
                </div>
                <div className="text-sm text-gray-600">
                  <span className="font-medium">Last Checked:</span> {nfr.lastChecked.toLocaleString()}
                </div>
              </div>
            </div>

            {/* Test Results */}
            <div className="mb-4">
              <div className="text-sm font-medium text-gray-900 mb-2">Recent Test Results</div>
              <div className="flex flex-wrap gap-2">
                {nfr.tests.map(test => (
                  <div key={test.id} className="flex items-center space-x-1 p-2 bg-gray-50 rounded text-xs">
                    <div className={`w-2 h-2 rounded-full ${
                      test.status === 'passed' ? 'bg-green-500' :
                      test.status === 'failed' ? 'bg-red-500' :
                      test.status === 'running' ? 'bg-blue-500' : 'bg-gray-500'
                    }`} />
                    <span>{test.name}</span>
                    <span className="text-gray-500">({test.status})</span>
                  </div>
                ))}
              </div>
            </div>

            {/* Certificates Status (if applicable) */}
            {nfr.implementation.certificates && nfr.implementation.certificates.length > 0 && (
              <div className="mb-4">
                <div className="text-sm font-medium text-gray-900 mb-2">Certificates</div>
                <div className="flex flex-wrap gap-2">
                  {nfr.implementation.certificates.map(cert => (
                    <div key={cert.id} className="flex items-center space-x-1 p-2 bg-gray-50 rounded text-xs">
                      <div className={`w-2 h-2 rounded-full ${
                        cert.status === 'valid' ? 'bg-green-500' :
                        cert.status === 'expiring' ? 'bg-yellow-500' : 'bg-red-500'
                      }`} />
                      <span>{cert.commonName}</span>
                      <span className="text-gray-500">({cert.daysUntilExpiry}d)</span>
                    </div>
                  ))}
                </div>
              </div>
            )}

            {/* Remediation (if needed) */}
            {nfr.remediation && (
              <Alert className="mb-4">
                <AlertTriangle className="h-4 w-4" />
                <AlertDescription>
                  <span className="font-medium">Remediation Required:</span> {nfr.remediation}
                </AlertDescription>
              </Alert>
            )}

            <div className="flex items-center justify-between pt-4 border-t">
              <div className="text-sm text-gray-600">
                Owner: {nfr.owner} • 
                Risk Level: {nfr.riskLevel} • 
                Dependencies: {nfr.dependencies.length}
              </div>
              <div className="flex items-center space-x-2">
                <Switch
                  checked={nfr.implementation.enabled}
                  onCheckedChange={(enabled) => handleToggleNFR(nfr.id, enabled)}
                />
                <Button variant="outline" size="sm">
                  <Eye className="w-4 h-4 mr-1" />
                  View Details
                </Button>
                <Button variant="outline" size="sm">
                  <Settings className="w-4 h-4 mr-1" />
                  Configure
                </Button>
              </div>
            </div>
          </Card>
        ))}

        {filteredNFRs.length === 0 && (
          <div className="text-center py-12">
            <Shield className="w-16 h-16 text-gray-400 mx-auto mb-4" />
            <h3 className="text-lg font-medium text-gray-900 mb-2">No Security NFRs Found</h3>
            <p className="text-gray-600">
              {filters.search || filters.category || filters.status || filters.priority
                ? 'No security NFRs match your current filters'
                : 'No security NFRs configured'
              }
            </p>
          </div>
        )}
      </div>

      {/* NFR Details Dialog */}
      <Dialog open={showDetailsDialog} onOpenChange={setShowDetailsDialog}>
        <DialogContent className="max-w-6xl max-h-[90vh] overflow-y-auto">
          <DialogHeader>
            <DialogTitle>Security NFR Details: {selectedNFR?.name}</DialogTitle>
          </DialogHeader>

          {selectedNFR && (
            <Tabs value={activeTab} onValueChange={setActiveTab}>
              <TabsList>
                <TabsTrigger value="overview">Overview</TabsTrigger>
                <TabsTrigger value="implementation">Implementation</TabsTrigger>
                <TabsTrigger value="metrics">Metrics</TabsTrigger>
                <TabsTrigger value="tests">Tests</TabsTrigger>
                <TabsTrigger value="certificates">Certificates</TabsTrigger>
              </TabsList>

              <TabsContent value="overview" className="space-y-4">
                <div className="grid grid-cols-2 gap-6">
                  <Card className="p-4">
                    <h3 className="text-lg font-medium text-gray-900 mb-3">NFR Information</h3>
                    <div className="space-y-2 text-sm">
                      <div className="flex justify-between">
                        <span className="text-gray-600">Name:</span>
                        <span className="font-medium">{selectedNFR.name}</span>
                      </div>
                      <div className="flex justify-between">
                        <span className="text-gray-600">Category:</span>
                        <Badge className={getCategoryColor(selectedNFR.category)}>
                          {selectedNFR.category}
                        </Badge>
                      </div>
                      <div className="flex justify-between">
                        <span className="text-gray-600">Priority:</span>
                        <Badge className={getPriorityColor(selectedNFR.priority)}>
                          {selectedNFR.priority}
                        </Badge>
                      </div>
                      <div className="flex justify-between">
                        <span className="text-gray-600">Status:</span>
                        <Badge className={getStatusColor(selectedNFR.currentStatus)}>
                          {selectedNFR.currentStatus.replace('_', ' ')}
                        </Badge>
                      </div>
                      <div className="flex justify-between">
                        <span className="text-gray-600">Risk Level:</span>
                        <Badge className={getPriorityColor(selectedNFR.riskLevel)}>
                          {selectedNFR.riskLevel}
                        </Badge>
                      </div>
                    </div>
                  </Card>

                  <Card className="p-4">
                    <h3 className="text-lg font-medium text-gray-900 mb-3">Current Metrics</h3>
                    <div className="space-y-2 text-sm">
                      <div className="flex justify-between">
                        <span className="text-gray-600">Uptime:</span>
                        <span className="font-medium">{selectedNFR.metrics.uptime.toFixed(2)}%</span>
                      </div>
                      <div className="flex justify-between">
                        <span className="text-gray-600">Response Time:</span>
                        <span className="font-medium">{Math.round(selectedNFR.metrics.responseTime)}ms</span>
                      </div>
                      <div className="flex justify-between">
                        <span className="text-gray-600">Throughput:</span>
                        <span className="font-medium">{formatNumber(selectedNFR.metrics.throughput)} RPS</span>
                      </div>
                      <div className="flex justify-between">
                        <span className="text-gray-600">Error Rate:</span>
                        <span className="font-medium">{selectedNFR.metrics.errorRate.toFixed(2)}%</span>
                      </div>
                      <div className="flex justify-between">
                        <span className="text-gray-600">SLA Compliance:</span>
                        <span className="font-medium">{selectedNFR.metrics.slaCompliance.toFixed(2)}%</span>
                      </div>
                    </div>
                  </Card>
                </div>

                <Card className="p-4">
                  <h3 className="text-lg font-medium text-gray-900 mb-3">Description</h3>
                  <p className="text-gray-600 mb-4">{selectedNFR.description}</p>
                  <h4 className="font-medium text-gray-900 mb-2">Requirement</h4>
                  <p className="text-gray-600">{selectedNFR.requirement}</p>
                </Card>

                <div className="grid grid-cols-2 gap-6">
                  <Card className="p-4">
                    <h3 className="text-lg font-medium text-gray-900 mb-3">Dependencies</h3>
                    <div className="space-y-2">
                      {selectedNFR.dependencies.map(dep => (
                        <div key={dep} className="p-2 bg-gray-50 rounded text-sm">
                          {dep}
                        </div>
                      ))}
                    </div>
                  </Card>

                  <Card className="p-4">
                    <h3 className="text-lg font-medium text-gray-900 mb-3">Ownership</h3>
                    <div className="space-y-2 text-sm">
                      <div className="flex justify-between">
                        <span className="text-gray-600">Owner:</span>
                        <span className="font-medium">{selectedNFR.owner}</span>
                      </div>
                      <div className="flex justify-between">
                        <span className="text-gray-600">Last Checked:</span>
                        <span className="font-medium">{selectedNFR.lastChecked.toLocaleDateString()}</span>
                      </div>
                      <div className="flex justify-between">
                        <span className="text-gray-600">Next Check:</span>
                        <span className="font-medium">{selectedNFR.nextCheck.toLocaleDateString()}</span>
                      </div>
                    </div>
                  </Card>
                </div>
              </TabsContent>

              <TabsContent value="implementation" className="space-y-4">
                <Card className="p-4">
                  <h3 className="text-lg font-medium text-gray-900 mb-3">Implementation Details</h3>
                  <div className="space-y-4">
                    <div className="grid grid-cols-2 gap-4">
                      <div>
                        <Label className="text-sm font-medium text-gray-900">Technology</Label>
                        <div className="text-sm text-gray-600">{selectedNFR.implementation.technology}</div>
                      </div>
                      <div>
                        <Label className="text-sm font-medium text-gray-900">Version</Label>
                        <div className="text-sm text-gray-600">{selectedNFR.implementation.version || 'N/A'}</div>
                      </div>
                      <div>
                        <Label className="text-sm font-medium text-gray-900">Status</Label>
                        <Badge className={selectedNFR.implementation.enabled ? 'bg-green-100 text-green-800' : 'bg-red-100 text-red-800'}>
                          {selectedNFR.implementation.enabled ? 'Enabled' : 'Disabled'}
                        </Badge>
                      </div>
                      <div>
                        <Label className="text-sm font-medium text-gray-900">Last Updated</Label>
                        <div className="text-sm text-gray-600">{selectedNFR.implementation.lastUpdated.toLocaleDateString()}</div>
                      </div>
                    </div>

                    <div>
                      <Label className="text-sm font-medium text-gray-900">Endpoints</Label>
                      <div className="mt-1 space-y-1">
                        {selectedNFR.implementation.endpoints.map(endpoint => (
                          <div key={endpoint} className="text-sm text-gray-600 font-mono bg-gray-50 p-2 rounded">
                            {endpoint}
                          </div>
                        ))}
                      </div>
                    </div>

                    <div>
                      <Label className="text-sm font-medium text-gray-900">Configuration</Label>
                      <div className="mt-1 bg-gray-50 p-4 rounded">
                        <pre className="text-xs text-gray-600 overflow-x-auto">
                          {JSON.stringify(selectedNFR.implementation.configuration, null, 2)}
                        </pre>
                      </div>
                    </div>
                  </div>
                </Card>
              </TabsContent>

              <TabsContent value="metrics" className="space-y-4">
                <div className="grid grid-cols-2 md:grid-cols-4 gap-4">
                  <Card className="p-4 text-center">
                    <Activity className="w-8 h-8 text-green-600 mx-auto mb-2" />
                    <div className="text-2xl font-bold text-gray-900">
                      {selectedNFR.metrics.uptime.toFixed(2)}%
                    </div>
                    <div className="text-sm text-gray-600">Uptime</div>
                  </Card>

                  <Card className="p-4 text-center">
                    <Clock className="w-8 h-8 text-blue-600 mx-auto mb-2" />
                    <div className="text-2xl font-bold text-gray-900">
                      {Math.round(selectedNFR.metrics.responseTime)}ms
                    </div>
                    <div className="text-sm text-gray-600">Response Time</div>
                  </Card>

                  <Card className="p-4 text-center">
                    <Zap className="w-8 h-8 text-purple-600 mx-auto mb-2" />
                    <div className="text-2xl font-bold text-gray-900">
                      {formatNumber(selectedNFR.metrics.throughput)}
                    </div>
                    <div className="text-sm text-gray-600">RPS</div>
                  </Card>

                  <Card className="p-4 text-center">
                    <AlertTriangle className="w-8 h-8 text-orange-600 mx-auto mb-2" />
                    <div className="text-2xl font-bold text-gray-900">
                      {selectedNFR.metrics.errorRate.toFixed(2)}%
                    </div>
                    <div className="text-sm text-gray-600">Error Rate</div>
                  </Card>
                </div>

                <div className="grid grid-cols-2 gap-6">
                  <Card className="p-4">
                    <h3 className="text-lg font-medium text-gray-900 mb-3">Reliability Metrics</h3>
                    <div className="space-y-2 text-sm">
                      <div className="flex justify-between">
                        <span className="text-gray-600">MTBF:</span>
                        <span className="font-medium">{selectedNFR.metrics.mtbf} hours</span>
                      </div>
                      <div className="flex justify-between">
                        <span className="text-gray-600">MTTR:</span>
                        <span className="font-medium">{selectedNFR.metrics.mttr} minutes</span>
                      </div>
                      <div className="flex justify-between">
                        <span className="text-gray-600">SLA Compliance:</span>
                        <span className="font-medium">{selectedNFR.metrics.slaCompliance.toFixed(2)}%</span>
                      </div>
                      {selectedNFR.metrics.lastIncident && (
                        <div className="flex justify-between">
                          <span className="text-gray-600">Last Incident:</span>
                          <span className="font-medium">{selectedNFR.metrics.lastIncident.toLocaleDateString()}</span>
                        </div>
                      )}
                    </div>
                  </Card>

                  <Card className="p-4">
                    <h3 className="text-lg font-medium text-gray-900 mb-3">Performance Trends</h3>
                    <div className="text-center text-gray-600 py-8">
                      <BarChart3 className="w-16 h-16 mx-auto mb-4 text-gray-400" />
                      <p>Performance charts would be displayed here</p>
                    </div>
                  </Card>
                </div>
              </TabsContent>

              <TabsContent value="tests" className="space-y-4">
                <Card className="p-4">
                  <h3 className="text-lg font-medium text-gray-900 mb-3">Security Tests</h3>
                  <div className="space-y-4">
                    {selectedNFR.tests.map(test => (
                      <div key={test.id} className="p-4 border rounded">
                        <div className="flex items-center justify-between mb-3">
                          <div className="flex items-center space-x-3">
                            <div className={`p-1 rounded ${
                              test.status === 'passed' ? 'bg-green-100' :
                              test.status === 'failed' ? 'bg-red-100' :
                              test.status === 'running' ? 'bg-blue-100' : 'bg-gray-100'
                            }`}>
                              {test.status === 'passed' ? (
                                <CheckCircle className="w-4 h-4 text-green-600" />
                              ) : test.status === 'failed' ? (
                                <XCircle className="w-4 h-4 text-red-600" />
                              ) : test.status === 'running' ? (
                                <Activity className="w-4 h-4 text-blue-600 animate-spin" />
                              ) : (
                                <Clock className="w-4 h-4 text-gray-600" />
                              )}
                            </div>
                            <div>
                              <h4 className="font-medium text-gray-900">{test.name}</h4>
                              <div className="text-sm text-gray-600">Type: {test.type} • Frequency: {test.frequency}</div>
                            </div>
                          </div>
                          <Badge className={
                            test.status === 'passed' ? 'bg-green-100 text-green-800' :
                            test.status === 'failed' ? 'bg-red-100 text-red-800' :
                            test.status === 'running' ? 'bg-blue-100 text-blue-800' :
                            'bg-gray-100 text-gray-800'
                          }>
                            {test.status}
                          </Badge>
                        </div>

                        <div className="grid grid-cols-3 gap-4 text-sm">
                          <div>
                            <span className="text-gray-600">Last Run:</span>
                            <div className="font-medium">{test.lastRun.toLocaleString()}</div>
                          </div>
                          <div>
                            <span className="text-gray-600">Next Run:</span>
                            <div className="font-medium">{test.nextRun.toLocaleString()}</div>
                          </div>
                          <div>
                            <span className="text-gray-600">Score:</span>
                            <div className="font-medium">{test.result?.score || 'N/A'}</div>
                          </div>
                        </div>

                        {test.result && (
                          <div className="mt-3">
                            <div className="text-sm font-medium text-gray-900 mb-2">Test Results</div>
                            {test.result.findings.length > 0 && (
                              <div className="space-y-2">
                                {test.result.findings.map(finding => (
                                  <div key={finding.id} className="p-2 bg-gray-50 rounded text-sm">
                                    <div className="flex items-center justify-between mb-1">
                                      <span className="font-medium">{finding.title}</span>
                                      <Badge className={getSeverityColor(finding.severity)} size="sm">
                                        {finding.severity}
                                      </Badge>
                                    </div>
                                    <div className="text-gray-600">{finding.description}</div>
                                    {finding.remediation && (
                                      <div className="text-blue-600 text-xs mt-1">
                                        Remediation: {finding.remediation}
                                      </div>
                                    )}
                                  </div>
                                ))}
                              </div>
                            )}
                          </div>
                        )}
                      </div>
                    ))}
                  </div>
                </Card>
              </TabsContent>

              <TabsContent value="certificates" className="space-y-4">
                {selectedNFR.implementation.certificates && selectedNFR.implementation.certificates.length > 0 ? (
                  <Card className="p-4">
                    <h3 className="text-lg font-medium text-gray-900 mb-3">SSL/TLS Certificates</h3>
                    <div className="space-y-4">
                      {selectedNFR.implementation.certificates.map(cert => (
                        <div key={cert.id} className="p-4 border rounded">
                          <div className="flex items-center justify-between mb-3">
                            <div>
                              <h4 className="font-medium text-gray-900">{cert.commonName}</h4>
                              <div className="text-sm text-gray-600">Issued by: {cert.issuer}</div>
                            </div>
                            <div className="flex items-center space-x-2">
                              <Badge className={
                                cert.status === 'valid' ? 'bg-green-100 text-green-800' :
                                cert.status === 'expiring' ? 'bg-yellow-100 text-yellow-800' :
                                'bg-red-100 text-red-800'
                              }>
                                {cert.status}
                              </Badge>
                              <Badge variant="outline">
                                {cert.daysUntilExpiry} days
                              </Badge>
                            </div>
                          </div>

                          <div className="grid grid-cols-2 md:grid-cols-4 gap-4 text-sm">
                            <div>
                              <span className="text-gray-600">Valid From:</span>
                              <div className="font-medium">{cert.validFrom.toLocaleDateString()}</div>
                            </div>
                            <div>
                              <span className="text-gray-600">Valid To:</span>
                              <div className="font-medium">{cert.validTo.toLocaleDateString()}</div>
                            </div>
                            <div>
                              <span className="text-gray-600">Algorithm:</span>
                              <div className="font-medium">{cert.algorithm}</div>
                            </div>
                            <div>
                              <span className="text-gray-600">Key Size:</span>
                              <div className="font-medium">{cert.keySize} bits</div>
                            </div>
                          </div>

                          <div className="mt-3">
                            <div className="text-sm font-medium text-gray-900 mb-2">Usage</div>
                            <div className="flex flex-wrap gap-1">
                              {cert.usage.map(usage => (
                                <Badge key={usage} variant="outline" size="sm">
                                  {usage.replace('_', ' ')}
                                </Badge>
                              ))}
                            </div>
                          </div>
                        </div>
                      ))}
                    </div>
                  </Card>
                ) : (
                  <div className="text-center py-8">
                    <Key className="w-16 h-16 text-gray-400 mx-auto mb-4" />
                    <h3 className="text-lg font-medium text-gray-900 mb-2">No Certificates</h3>
                    <p className="text-gray-600">This security NFR does not involve certificate management.</p>
                  </div>
                )}
              </TabsContent>
            </Tabs>
          )}

          <DialogFooter>
            <Button variant="outline" onClick={() => setShowDetailsDialog(false)}>
              Close
            </Button>
            <Button onClick={() => setShowConfigDialog(true)}>
              <Settings className="w-4 h-4 mr-2" />
              Configure
            </Button>
          </DialogFooter>
        </DialogContent>
      </Dialog>
    </div>
  )
}

export default SecurityNFRManager
