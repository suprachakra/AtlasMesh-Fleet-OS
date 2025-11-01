import React, { useState, useCallback, useEffect, useMemo } from 'react'
import { 
  Radio, Database, Clock, Tag, Search, Filter, Eye, Settings, RefreshCw,
  BarChart3, TrendingUp, AlertTriangle, CheckCircle, Activity, Zap,
  GitBranch, Hash, Calendar, Users, MapPin, FileText, Download, Upload
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
interface EventTopic {
  id: string
  name: string
  displayName: string
  category: 'telemetry' | 'command' | 'status' | 'alert' | 'audit' | 'metrics'
  description: string
  schema: EventSchema
  producer: string[]
  consumer: string[]
  status: 'active' | 'deprecated' | 'experimental' | 'disabled'
  version: string
  retention: RetentionPolicy
  partitions: number
  replicationFactor: number
  compression: 'none' | 'gzip' | 'snappy' | 'lz4' | 'zstd'
  correlationIds: CorrelationConfig
  timeSync: TimeSyncConfig
  sla: TopicSLA
  metrics: TopicMetrics
  tags: string[]
  createdAt: Date
  updatedAt: Date
  examples: EventExample[]
}

interface EventSchema {
  format: 'json' | 'avro' | 'protobuf'
  version: string
  fields: SchemaField[]
  required: string[]
  additionalProperties: boolean
  compatibility: 'backward' | 'forward' | 'full' | 'none'
}

interface SchemaField {
  name: string
  type: 'string' | 'number' | 'boolean' | 'object' | 'array' | 'timestamp'
  description: string
  required: boolean
  format?: string
  enum?: string[]
  pattern?: string
  minimum?: number
  maximum?: number
  items?: SchemaField
  properties?: Record<string, SchemaField>
}

interface RetentionPolicy {
  timeMs: number // milliseconds
  sizeBytes: number
  compactionPolicy: 'delete' | 'compact'
  cleanupPolicy: string
}

interface CorrelationConfig {
  traceId: boolean
  spanId: boolean
  parentSpanId: boolean
  sessionId: boolean
  requestId: boolean
  vehicleId: boolean
  tripId: boolean
  userId: boolean
  customFields: string[]
}

interface TimeSyncConfig {
  clockSyncRequired: boolean
  maxClockSkewMs: number
  timestampField: string
  timestampFormat: 'iso8601' | 'unix' | 'unix_ms' | 'unix_ns'
  timezone: string
}

interface TopicSLA {
  maxLatencyMs: number
  minThroughputMps: number // messages per second
  availabilityPercent: number
  durabilityLevel: 'at_most_once' | 'at_least_once' | 'exactly_once'
  orderingGuarantee: 'none' | 'partition' | 'global'
}

interface TopicMetrics {
  messagesPerSecond: number
  bytesPerSecond: number
  totalMessages: number
  totalBytes: number
  avgMessageSize: number
  lagMs: number
  errorRate: number
  lastActivity: Date
  peakThroughput: number
  consumers: ConsumerMetrics[]
  producers: ProducerMetrics[]
}

interface ConsumerMetrics {
  consumerId: string
  consumerGroup: string
  lag: number
  throughput: number
  lastOffset: number
  status: 'active' | 'idle' | 'error'
}

interface ProducerMetrics {
  producerId: string
  throughput: number
  errorRate: number
  lastProduced: Date
  status: 'active' | 'idle' | 'error'
}

interface EventExample {
  id: string
  name: string
  description: string
  payload: Record<string, any>
  timestamp: Date
  correlationIds: Record<string, string>
}

interface EventCatalogProps {
  topicFilter?: string
  onTopicSelected?: (topic: EventTopic) => void
  className?: string
}

const EventCatalog: React.FC<EventCatalogProps> = ({
  topicFilter,
  onTopicSelected,
  className = ''
}) => {
  // State
  const [topics, setTopics] = useState<EventTopic[]>([])
  const [showTopicDialog, setShowTopicDialog] = useState(false)
  const [selectedTopic, setSelectedTopic] = useState<EventTopic | null>(null)
  const [activeTab, setActiveTab] = useState('overview')
  const [filters, setFilters] = useState({
    category: '',
    status: '',
    search: '',
    producer: '',
    consumer: ''
  })

  // Mock data initialization
  const initializeMockData = useCallback(() => {
    const mockTopics: EventTopic[] = [
      {
        id: 'topic-vehicle-telemetry',
        name: 'vehicle.telemetry.sensors',
        displayName: 'Vehicle Sensor Telemetry',
        category: 'telemetry',
        description: 'Real-time sensor data from vehicle systems including GPS, IMU, cameras, and LIDAR',
        schema: {
          format: 'avro',
          version: '2.1.0',
          fields: [
            {
              name: 'vehicleId',
              type: 'string',
              description: 'Unique identifier for the vehicle',
              required: true,
              pattern: '^[A-Za-z0-9-]+$'
            },
            {
              name: 'timestamp',
              type: 'timestamp',
              description: 'UTC timestamp when data was collected',
              required: true,
              format: 'iso8601'
            },
            {
              name: 'location',
              type: 'object',
              description: 'GPS coordinates and accuracy',
              required: true,
              properties: {
                'latitude': { name: 'latitude', type: 'number', description: 'Latitude in decimal degrees', required: true, minimum: -90, maximum: 90 },
                'longitude': { name: 'longitude', type: 'number', description: 'Longitude in decimal degrees', required: true, minimum: -180, maximum: 180 },
                'altitude': { name: 'altitude', type: 'number', description: 'Altitude in meters', required: false },
                'accuracy': { name: 'accuracy', type: 'number', description: 'GPS accuracy in meters', required: true }
              }
            },
            {
              name: 'speed',
              type: 'number',
              description: 'Vehicle speed in m/s',
              required: true,
              minimum: 0,
              maximum: 50
            },
            {
              name: 'heading',
              type: 'number',
              description: 'Vehicle heading in degrees (0-360)',
              required: true,
              minimum: 0,
              maximum: 360
            },
            {
              name: 'sensors',
              type: 'object',
              description: 'Sensor readings',
              required: true,
              properties: {
                'lidar': { name: 'lidar', type: 'object', description: 'LIDAR point cloud metadata', required: false },
                'cameras': { name: 'cameras', type: 'array', description: 'Camera frame metadata', required: false },
                'radar': { name: 'radar', type: 'object', description: 'Radar detection data', required: false }
              }
            }
          ],
          required: ['vehicleId', 'timestamp', 'location', 'speed', 'heading'],
          additionalProperties: false,
          compatibility: 'backward'
        },
        producer: ['vehicle-edge-agent', 'simulator'],
        consumer: ['fleet-manager', 'analytics-engine', 'safety-monitor'],
        status: 'active',
        version: '2.1.0',
        retention: {
          timeMs: 7 * 24 * 60 * 60 * 1000, // 7 days
          sizeBytes: 10 * 1024 * 1024 * 1024, // 10GB
          compactionPolicy: 'delete',
          cleanupPolicy: 'delete'
        },
        partitions: 12,
        replicationFactor: 3,
        compression: 'snappy',
        correlationIds: {
          traceId: true,
          spanId: true,
          parentSpanId: false,
          sessionId: false,
          requestId: false,
          vehicleId: true,
          tripId: true,
          userId: false,
          customFields: ['route_id', 'mission_id']
        },
        timeSync: {
          clockSyncRequired: true,
          maxClockSkewMs: 1000,
          timestampField: 'timestamp',
          timestampFormat: 'iso8601',
          timezone: 'UTC'
        },
        sla: {
          maxLatencyMs: 500,
          minThroughputMps: 1000,
          availabilityPercent: 99.9,
          durabilityLevel: 'at_least_once',
          orderingGuarantee: 'partition'
        },
        metrics: {
          messagesPerSecond: 1247,
          bytesPerSecond: 2.4 * 1024 * 1024,
          totalMessages: 45892341,
          totalBytes: 89.7 * 1024 * 1024 * 1024,
          avgMessageSize: 2048,
          lagMs: 45,
          errorRate: 0.02,
          lastActivity: new Date(),
          peakThroughput: 2100,
          consumers: [
            {
              consumerId: 'fleet-manager-01',
              consumerGroup: 'fleet-management',
              lag: 12,
              throughput: 450,
              lastOffset: 45892341,
              status: 'active'
            },
            {
              consumerId: 'analytics-engine-01',
              consumerGroup: 'analytics',
              lag: 89,
              throughput: 800,
              lastOffset: 45892252,
              status: 'active'
            }
          ],
          producers: [
            {
              producerId: 'vehicle-atlas-001',
              throughput: 12.5,
              errorRate: 0.01,
              lastProduced: new Date(),
              status: 'active'
            }
          ]
        },
        tags: ['telemetry', 'real-time', 'high-volume', 'critical'],
        createdAt: new Date('2024-01-15'),
        updatedAt: new Date('2024-11-20'),
        examples: [
          {
            id: 'example-001',
            name: 'Standard Telemetry Message',
            description: 'Typical sensor telemetry from vehicle during normal operation',
            payload: {
              vehicleId: 'atlas-001',
              timestamp: '2024-11-26T10:30:45.123Z',
              location: {
                latitude: 25.2048,
                longitude: 55.2708,
                altitude: 12.5,
                accuracy: 1.2
              },
              speed: 15.6,
              heading: 87.5,
              sensors: {
                lidar: { points: 65536, range_max: 120.0 },
                cameras: [
                  { camera_id: 'front', resolution: '1920x1080', fps: 30 },
                  { camera_id: 'rear', resolution: '1920x1080', fps: 30 }
                ],
                radar: { targets: 3, max_range: 200.0 }
              }
            },
            timestamp: new Date(),
            correlationIds: {
              traceId: '1a2b3c4d5e6f7890',
              spanId: '9876543210abcdef',
              vehicleId: 'atlas-001',
              tripId: 'trip-20241126-001',
              route_id: 'route-dubai-mall-001',
              mission_id: 'mission-passenger-pickup-001'
            }
          }
        ]
      },
      {
        id: 'topic-vehicle-commands',
        name: 'vehicle.commands.control',
        displayName: 'Vehicle Control Commands',
        category: 'command',
        description: 'Commands sent to vehicles for navigation, speed control, and operational directives',
        schema: {
          format: 'json',
          version: '1.8.2',
          fields: [
            {
              name: 'commandId',
              type: 'string',
              description: 'Unique identifier for the command',
              required: true
            },
            {
              name: 'vehicleId',
              type: 'string',
              description: 'Target vehicle identifier',
              required: true
            },
            {
              name: 'commandType',
              type: 'string',
              description: 'Type of command being issued',
              required: true,
              enum: ['navigate', 'stop', 'resume', 'emergency_stop', 'change_route', 'park', 'return_to_depot']
            },
            {
              name: 'parameters',
              type: 'object',
              description: 'Command-specific parameters',
              required: false
            },
            {
              name: 'priority',
              type: 'string',
              description: 'Command priority level',
              required: true,
              enum: ['low', 'normal', 'high', 'emergency']
            },
            {
              name: 'timeout',
              type: 'number',
              description: 'Command timeout in seconds',
              required: false,
              minimum: 1,
              maximum: 3600
            }
          ],
          required: ['commandId', 'vehicleId', 'commandType', 'priority'],
          additionalProperties: true,
          compatibility: 'forward'
        },
        producer: ['fleet-manager', 'safety-monitor', 'operator-console'],
        consumer: ['vehicle-edge-agent'],
        status: 'active',
        version: '1.8.2',
        retention: {
          timeMs: 30 * 24 * 60 * 60 * 1000, // 30 days
          sizeBytes: 1 * 1024 * 1024 * 1024, // 1GB
          compactionPolicy: 'delete',
          cleanupPolicy: 'delete'
        },
        partitions: 6,
        replicationFactor: 3,
        compression: 'gzip',
        correlationIds: {
          traceId: true,
          spanId: true,
          parentSpanId: true,
          sessionId: false,
          requestId: true,
          vehicleId: true,
          tripId: true,
          userId: true,
          customFields: ['operator_id', 'safety_context']
        },
        timeSync: {
          clockSyncRequired: true,
          maxClockSkewMs: 100,
          timestampField: 'issuedAt',
          timestampFormat: 'iso8601',
          timezone: 'UTC'
        },
        sla: {
          maxLatencyMs: 50,
          minThroughputMps: 100,
          availabilityPercent: 99.99,
          durabilityLevel: 'exactly_once',
          orderingGuarantee: 'partition'
        },
        metrics: {
          messagesPerSecond: 23,
          bytesPerSecond: 18 * 1024,
          totalMessages: 892341,
          totalBytes: 1.2 * 1024 * 1024 * 1024,
          avgMessageSize: 1024,
          lagMs: 8,
          errorRate: 0.001,
          lastActivity: new Date(),
          peakThroughput: 150,
          consumers: [
            {
              consumerId: 'vehicle-agent-atlas-001',
              consumerGroup: 'vehicle-agents',
              lag: 0,
              throughput: 12,
              lastOffset: 892341,
              status: 'active'
            }
          ],
          producers: [
            {
              producerId: 'fleet-manager-primary',
              throughput: 15,
              errorRate: 0.0005,
              lastProduced: new Date(),
              status: 'active'
            }
          ]
        },
        tags: ['commands', 'critical', 'low-latency', 'safety'],
        createdAt: new Date('2024-01-15'),
        updatedAt: new Date('2024-11-18'),
        examples: [
          {
            id: 'example-002',
            name: 'Emergency Stop Command',
            description: 'Critical command to immediately stop a vehicle',
            payload: {
              commandId: 'cmd-20241126-emergency-001',
              vehicleId: 'atlas-001',
              commandType: 'emergency_stop',
              priority: 'emergency',
              parameters: {
                reason: 'obstacle_detected',
                location: {
                  latitude: 25.2048,
                  longitude: 55.2708
                }
              },
              timeout: 5,
              issuedAt: '2024-11-26T10:30:45.123Z'
            },
            timestamp: new Date(),
            correlationIds: {
              traceId: '2b3c4d5e6f7890a1',
              spanId: '876543210abcdef9',
              requestId: 'req-emergency-001',
              vehicleId: 'atlas-001',
              tripId: 'trip-20241126-001',
              userId: 'operator-ahmed',
              operator_id: 'op-001',
              safety_context: 'obstacle_avoidance'
            }
          }
        ]
      },
      {
        id: 'topic-system-alerts',
        name: 'system.alerts.notifications',
        displayName: 'System Alerts & Notifications',
        category: 'alert',
        description: 'Critical system alerts, warnings, and notifications from all fleet components',
        schema: {
          format: 'json',
          version: '1.5.1',
          fields: [
            {
              name: 'alertId',
              type: 'string',
              description: 'Unique identifier for the alert',
              required: true
            },
            {
              name: 'severity',
              type: 'string',
              description: 'Alert severity level',
              required: true,
              enum: ['info', 'warning', 'error', 'critical']
            },
            {
              name: 'source',
              type: 'string',
              description: 'System component that generated the alert',
              required: true
            },
            {
              name: 'category',
              type: 'string',
              description: 'Alert category',
              required: true,
              enum: ['safety', 'operational', 'technical', 'security', 'compliance']
            },
            {
              name: 'title',
              type: 'string',
              description: 'Human-readable alert title',
              required: true
            },
            {
              name: 'description',
              type: 'string',
              description: 'Detailed alert description',
              required: true
            },
            {
              name: 'metadata',
              type: 'object',
              description: 'Additional context-specific data',
              required: false
            },
            {
              name: 'acknowledged',
              type: 'boolean',
              description: 'Whether alert has been acknowledged',
              required: false
            }
          ],
          required: ['alertId', 'severity', 'source', 'category', 'title', 'description'],
          additionalProperties: true,
          compatibility: 'full'
        },
        producer: ['safety-monitor', 'fleet-manager', 'vehicle-agents', 'infrastructure'],
        consumer: ['alert-manager', 'notification-service', 'operator-console', 'incident-manager'],
        status: 'active',
        version: '1.5.1',
        retention: {
          timeMs: 90 * 24 * 60 * 60 * 1000, // 90 days
          sizeBytes: 5 * 1024 * 1024 * 1024, // 5GB
          compactionPolicy: 'compact',
          cleanupPolicy: 'compact'
        },
        partitions: 8,
        replicationFactor: 3,
        compression: 'lz4',
        correlationIds: {
          traceId: true,
          spanId: true,
          parentSpanId: false,
          sessionId: false,
          requestId: false,
          vehicleId: false,
          tripId: false,
          userId: false,
          customFields: ['alert_chain_id', 'incident_id']
        },
        timeSync: {
          clockSyncRequired: true,
          maxClockSkewMs: 5000,
          timestampField: 'timestamp',
          timestampFormat: 'iso8601',
          timezone: 'UTC'
        },
        sla: {
          maxLatencyMs: 1000,
          minThroughputMps: 50,
          availabilityPercent: 99.95,
          durabilityLevel: 'at_least_once',
          orderingGuarantee: 'none'
        },
        metrics: {
          messagesPerSecond: 45,
          bytesPerSecond: 67 * 1024,
          totalMessages: 2341892,
          totalBytes: 3.1 * 1024 * 1024 * 1024,
          avgMessageSize: 1536,
          lagMs: 120,
          errorRate: 0.01,
          lastActivity: new Date(),
          peakThroughput: 200,
          consumers: [
            {
              consumerId: 'alert-manager-primary',
              consumerGroup: 'alert-processing',
              lag: 5,
              throughput: 25,
              lastOffset: 2341892,
              status: 'active'
            },
            {
              consumerId: 'notification-service-01',
              consumerGroup: 'notifications',
              lag: 45,
              throughput: 20,
              lastOffset: 2341847,
              status: 'active'
            }
          ],
          producers: [
            {
              producerId: 'safety-monitor-primary',
              throughput: 20,
              errorRate: 0.005,
              lastProduced: new Date(),
              status: 'active'
            }
          ]
        },
        tags: ['alerts', 'notifications', 'monitoring', 'operational'],
        createdAt: new Date('2024-02-01'),
        updatedAt: new Date('2024-11-15'),
        examples: [
          {
            id: 'example-003',
            name: 'Safety Alert Example',
            description: 'Critical safety alert from vehicle safety monitoring system',
            payload: {
              alertId: 'alert-20241126-safety-001',
              severity: 'critical',
              source: 'safety-monitor',
              category: 'safety',
              title: 'Emergency Stop Activated',
              description: 'Vehicle atlas-001 executed emergency stop due to obstacle detection failure',
              timestamp: '2024-11-26T10:30:45.123Z',
              metadata: {
                vehicleId: 'atlas-001',
                location: {
                  latitude: 25.2048,
                  longitude: 55.2708
                },
                sensor: 'lidar_front',
                confidence: 0.95
              },
              acknowledged: false
            },
            timestamp: new Date(),
            correlationIds: {
              traceId: '3c4d5e6f7890a1b2',
              spanId: '76543210abcdef98',
              alert_chain_id: 'chain-safety-001',
              incident_id: 'inc-20241126-001'
            }
          }
        ]
      },
      {
        id: 'topic-audit-logs',
        name: 'system.audit.operations',
        displayName: 'System Audit Logs',
        category: 'audit',
        description: 'Comprehensive audit trail of all system operations, user actions, and security events',
        schema: {
          format: 'json',
          version: '2.0.3',
          fields: [
            {
              name: 'auditId',
              type: 'string',
              description: 'Unique identifier for the audit event',
              required: true
            },
            {
              name: 'eventType',
              type: 'string',
              description: 'Type of audited event',
              required: true,
              enum: ['user_action', 'system_operation', 'security_event', 'data_access', 'configuration_change']
            },
            {
              name: 'actor',
              type: 'object',
              description: 'Entity that performed the action',
              required: true,
              properties: {
                'type': { name: 'type', type: 'string', description: 'Actor type (user, system, service)', required: true, enum: ['user', 'system', 'service'] },
                'id': { name: 'id', type: 'string', description: 'Actor identifier', required: true },
                'name': { name: 'name', type: 'string', description: 'Actor display name', required: false }
              }
            },
            {
              name: 'action',
              type: 'string',
              description: 'Action that was performed',
              required: true
            },
            {
              name: 'resource',
              type: 'object',
              description: 'Resource that was acted upon',
              required: false,
              properties: {
                'type': { name: 'type', type: 'string', description: 'Resource type', required: true },
                'id': { name: 'id', type: 'string', description: 'Resource identifier', required: true },
                'name': { name: 'name', type: 'string', description: 'Resource name', required: false }
              }
            },
            {
              name: 'outcome',
              type: 'string',
              description: 'Result of the action',
              required: true,
              enum: ['success', 'failure', 'partial']
            },
            {
              name: 'details',
              type: 'object',
              description: 'Additional event-specific details',
              required: false
            }
          ],
          required: ['auditId', 'eventType', 'actor', 'action', 'outcome'],
          additionalProperties: true,
          compatibility: 'backward'
        },
        producer: ['audit-service', 'authentication-service', 'authorization-service', 'all-services'],
        consumer: ['audit-processor', 'compliance-monitor', 'security-analyzer'],
        status: 'active',
        version: '2.0.3',
        retention: {
          timeMs: 365 * 24 * 60 * 60 * 1000, // 365 days (1 year)
          sizeBytes: 50 * 1024 * 1024 * 1024, // 50GB
          compactionPolicy: 'compact',
          cleanupPolicy: 'compact'
        },
        partitions: 16,
        replicationFactor: 3,
        compression: 'zstd',
        correlationIds: {
          traceId: true,
          spanId: true,
          parentSpanId: false,
          sessionId: true,
          requestId: true,
          vehicleId: false,
          tripId: false,
          userId: true,
          customFields: ['session_id', 'ip_address', 'user_agent']
        },
        timeSync: {
          clockSyncRequired: true,
          maxClockSkewMs: 1000,
          timestampField: 'timestamp',
          timestampFormat: 'iso8601',
          timezone: 'UTC'
        },
        sla: {
          maxLatencyMs: 2000,
          minThroughputMps: 200,
          availabilityPercent: 99.99,
          durabilityLevel: 'exactly_once',
          orderingGuarantee: 'partition'
        },
        metrics: {
          messagesPerSecond: 156,
          bytesPerSecond: 234 * 1024,
          totalMessages: 8923451,
          totalBytes: 12.7 * 1024 * 1024 * 1024,
          avgMessageSize: 1500,
          lagMs: 67,
          errorRate: 0.005,
          lastActivity: new Date(),
          peakThroughput: 500,
          consumers: [
            {
              consumerId: 'audit-processor-01',
              consumerGroup: 'audit-processing',
              lag: 23,
              throughput: 80,
              lastOffset: 8923451,
              status: 'active'
            },
            {
              consumerId: 'compliance-monitor-01',
              consumerGroup: 'compliance',
              lag: 156,
              throughput: 40,
              lastOffset: 8923295,
              status: 'active'
            }
          ],
          producers: [
            {
              producerId: 'audit-service-primary',
              throughput: 100,
              errorRate: 0.002,
              lastProduced: new Date(),
              status: 'active'
            }
          ]
        },
        tags: ['audit', 'compliance', 'security', 'long-retention'],
        createdAt: new Date('2024-01-15'),
        updatedAt: new Date('2024-11-22'),
        examples: [
          {
            id: 'example-004',
            name: 'User Login Audit',
            description: 'Audit record for user authentication event',
            payload: {
              auditId: 'audit-20241126-auth-001',
              eventType: 'security_event',
              actor: {
                type: 'user',
                id: 'user-ahmed-001',
                name: 'Ahmed Al-Mansouri'
              },
              action: 'login',
              resource: {
                type: 'application',
                id: 'control-center',
                name: 'Fleet Control Center'
              },
              outcome: 'success',
              timestamp: '2024-11-26T10:30:45.123Z',
              details: {
                method: 'certificate',
                ip_address: '192.168.1.100',
                user_agent: 'Mozilla/5.0 (Windows NT 10.0; Win64; x64)',
                mfa_used: true
              }
            },
            timestamp: new Date(),
            correlationIds: {
              traceId: '4d5e6f7890a1b2c3',
              spanId: '6543210abcdef987',
              sessionId: 'session-20241126-001',
              requestId: 'req-auth-001',
              userId: 'user-ahmed-001',
              session_id: 'session-20241126-001',
              ip_address: '192.168.1.100',
              user_agent: 'Mozilla/5.0'
            }
          }
        ]
      }
    ]

    setTopics(mockTopics)
  }, [])

  useEffect(() => {
    initializeMockData()
  }, [initializeMockData])

  // Real-time metrics updates
  useEffect(() => {
    const interval = setInterval(() => {
      setTopics(prev => prev.map(topic => ({
        ...topic,
        metrics: {
          ...topic.metrics,
          messagesPerSecond: Math.max(0, topic.metrics.messagesPerSecond + (Math.random() - 0.5) * topic.metrics.messagesPerSecond * 0.1),
          bytesPerSecond: Math.max(0, topic.metrics.bytesPerSecond + (Math.random() - 0.5) * topic.metrics.bytesPerSecond * 0.1),
          totalMessages: topic.metrics.totalMessages + Math.floor(Math.random() * 100),
          lagMs: Math.max(0, topic.metrics.lagMs + (Math.random() - 0.5) * 20),
          lastActivity: new Date()
        }
      })))
    }, 5000)

    return () => clearInterval(interval)
  }, [])

  // Handlers
  const handleTopicSelect = useCallback((topic: EventTopic) => {
    setSelectedTopic(topic)
    setShowTopicDialog(true)
    onTopicSelected?.(topic)
  }, [onTopicSelected])

  // Filtered topics
  const filteredTopics = useMemo(() => {
    return topics
      .filter(topic => !topicFilter || topic.name.includes(topicFilter))
      .filter(topic => !filters.category || topic.category === filters.category)
      .filter(topic => !filters.status || topic.status === filters.status)
      .filter(topic => !filters.producer || topic.producer.some(p => p.includes(filters.producer)))
      .filter(topic => !filters.consumer || topic.consumer.some(c => c.includes(filters.consumer)))
      .filter(topic => {
        if (!filters.search) return true
        const searchTerm = filters.search.toLowerCase()
        return (
          topic.name.toLowerCase().includes(searchTerm) ||
          topic.displayName.toLowerCase().includes(searchTerm) ||
          topic.description.toLowerCase().includes(searchTerm) ||
          topic.tags.some(tag => tag.toLowerCase().includes(searchTerm))
        )
      })
      .sort((a, b) => {
        // Sort by activity (messages per second)
        return b.metrics.messagesPerSecond - a.metrics.messagesPerSecond
      })
  }, [topics, topicFilter, filters])

  // Statistics
  const stats = useMemo(() => {
    const totalTopics = topics.length
    const activeTopics = topics.filter(t => t.status === 'active').length
    const experimentalTopics = topics.filter(t => t.status === 'experimental').length
    const deprecatedTopics = topics.filter(t => t.status === 'deprecated').length

    const totalMessages = topics.reduce((sum, t) => sum + t.metrics.totalMessages, 0)
    const totalThroughput = topics.reduce((sum, t) => sum + t.metrics.messagesPerSecond, 0)
    const avgLatency = topics.length > 0 ? topics.reduce((sum, t) => sum + t.metrics.lagMs, 0) / topics.length : 0
    const totalConsumers = topics.reduce((sum, t) => sum + t.metrics.consumers.length, 0)

    return {
      totalTopics,
      activeTopics,
      experimentalTopics,
      deprecatedTopics,
      totalMessages,
      totalThroughput,
      avgLatency,
      totalConsumers
    }
  }, [topics])

  const getStatusColor = (status: string) => {
    switch (status) {
      case 'active': return 'bg-green-100 text-green-800'
      case 'experimental': return 'bg-blue-100 text-blue-800'
      case 'deprecated': return 'bg-yellow-100 text-yellow-800'
      case 'disabled': return 'bg-red-100 text-red-800'
      default: return 'bg-gray-100 text-gray-800'
    }
  }

  const getCategoryColor = (category: string) => {
    switch (category) {
      case 'telemetry': return 'bg-blue-100 text-blue-800'
      case 'command': return 'bg-green-100 text-green-800'
      case 'status': return 'bg-purple-100 text-purple-800'
      case 'alert': return 'bg-red-100 text-red-800'
      case 'audit': return 'bg-gray-100 text-gray-800'
      case 'metrics': return 'bg-orange-100 text-orange-800'
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
          <Radio className="w-6 h-6 text-blue-600" />
          <h2 className="text-2xl font-semibold text-gray-900">Event & Topic Catalog</h2>
        </div>
        <Button variant="outline" onClick={initializeMockData}>
          <RefreshCw className="w-4 h-4 mr-2" />
          Refresh
        </Button>
      </div>

      {/* Statistics */}
      <div className="grid grid-cols-2 md:grid-cols-4 lg:grid-cols-8 gap-4">
        <Card className="p-4">
          <div className="text-center">
            <div className="text-2xl font-bold text-gray-900">{stats.totalTopics}</div>
            <div className="text-sm text-gray-600">Total Topics</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className="text-2xl font-bold text-green-600">{stats.activeTopics}</div>
            <div className="text-sm text-gray-600">Active</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className="text-2xl font-bold text-blue-600">{stats.experimentalTopics}</div>
            <div className="text-sm text-gray-600">Experimental</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className="text-2xl font-bold text-yellow-600">{stats.deprecatedTopics}</div>
            <div className="text-sm text-gray-600">Deprecated</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className="text-2xl font-bold text-purple-600">{formatNumber(stats.totalMessages)}</div>
            <div className="text-sm text-gray-600">Total Messages</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className="text-2xl font-bold text-indigo-600">{Math.round(stats.totalThroughput)}</div>
            <div className="text-sm text-gray-600">Msg/sec</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className="text-2xl font-bold text-orange-600">{Math.round(stats.avgLatency)}ms</div>
            <div className="text-sm text-gray-600">Avg Latency</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className="text-2xl font-bold text-teal-600">{stats.totalConsumers}</div>
            <div className="text-sm text-gray-600">Consumers</div>
          </div>
        </Card>
      </div>

      {/* Filters */}
      <Card className="p-4">
        <div className="flex flex-wrap items-center gap-4">
          <div className="flex items-center space-x-2">
            <Search className="w-4 h-4 text-gray-500" />
            <Input
              placeholder="Search topics..."
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
            <option value="telemetry">Telemetry</option>
            <option value="command">Command</option>
            <option value="status">Status</option>
            <option value="alert">Alert</option>
            <option value="audit">Audit</option>
            <option value="metrics">Metrics</option>
          </Select>

          <Select
            value={filters.status}
            onValueChange={(value) => setFilters(prev => ({ ...prev, status: value }))}
          >
            <option value="">All Status</option>
            <option value="active">Active</option>
            <option value="experimental">Experimental</option>
            <option value="deprecated">Deprecated</option>
            <option value="disabled">Disabled</option>
          </Select>

          <Input
            placeholder="Producer filter..."
            value={filters.producer}
            onChange={(e) => setFilters(prev => ({ ...prev, producer: e.target.value }))}
            className="w-48"
          />

          <Input
            placeholder="Consumer filter..."
            value={filters.consumer}
            onChange={(e) => setFilters(prev => ({ ...prev, consumer: e.target.value }))}
            className="w-48"
          />
        </div>
      </Card>

      {/* Topics List */}
      <div className="space-y-4">
        {filteredTopics.map(topic => (
          <Card key={topic.id} className="p-6 hover:shadow-md transition-shadow cursor-pointer"
                onClick={() => handleTopicSelect(topic)}>
            <div className="flex items-start justify-between mb-4">
              <div className="flex items-center space-x-3">
                <div className="p-2 bg-blue-100 rounded">
                  <Radio className="w-5 h-5 text-blue-600" />
                </div>
                <div>
                  <h3 className="text-lg font-medium text-gray-900">{topic.displayName}</h3>
                  <p className="text-sm text-gray-600 font-mono">{topic.name}</p>
                  <p className="text-sm text-gray-600 mt-1">{topic.description}</p>
                </div>
              </div>
              <div className="flex items-center space-x-2">
                <Badge className={getStatusColor(topic.status)}>
                  {topic.status}
                </Badge>
                <Badge className={getCategoryColor(topic.category)}>
                  {topic.category}
                </Badge>
                <Badge variant="outline">
                  v{topic.version}
                </Badge>
              </div>
            </div>

            <div className="grid grid-cols-2 md:grid-cols-4 gap-4 mb-4">
              <div>
                <div className="text-sm text-gray-600">Throughput</div>
                <div className="font-medium text-gray-900">
                  {Math.round(topic.metrics.messagesPerSecond)} msg/s
                </div>
              </div>
              <div>
                <div className="text-sm text-gray-600">Data Rate</div>
                <div className="font-medium text-gray-900">
                  {formatBytes(topic.metrics.bytesPerSecond)}/s
                </div>
              </div>
              <div>
                <div className="text-sm text-gray-600">Lag</div>
                <div className={`font-medium ${
                  topic.metrics.lagMs > 1000 ? 'text-red-600' :
                  topic.metrics.lagMs > 500 ? 'text-yellow-600' : 'text-green-600'
                }`}>
                  {Math.round(topic.metrics.lagMs)}ms
                </div>
              </div>
              <div>
                <div className="text-sm text-gray-600">Total Messages</div>
                <div className="font-medium text-gray-900">
                  {formatNumber(topic.metrics.totalMessages)}
                </div>
              </div>
            </div>

            {/* Schema & Configuration */}
            <div className="grid grid-cols-2 md:grid-cols-4 gap-4 mb-4 text-sm">
              <div>
                <div className="text-gray-600">Schema</div>
                <div className="font-medium">{topic.schema.format.toUpperCase()} v{topic.schema.version}</div>
              </div>
              <div>
                <div className="text-gray-600">Partitions</div>
                <div className="font-medium">{topic.partitions}</div>
              </div>
              <div>
                <div className="text-gray-600">Replication</div>
                <div className="font-medium">{topic.replicationFactor}x</div>
              </div>
              <div>
                <div className="text-gray-600">Compression</div>
                <div className="font-medium capitalize">{topic.compression}</div>
              </div>
            </div>

            {/* Producers & Consumers */}
            <div className="grid grid-cols-2 gap-6 mb-4">
              <div>
                <div className="text-sm font-medium text-gray-900 mb-2">Producers ({topic.producer.length})</div>
                <div className="flex flex-wrap gap-1">
                  {topic.producer.slice(0, 3).map(producer => (
                    <Badge key={producer} variant="outline" size="sm">
                      {producer}
                    </Badge>
                  ))}
                  {topic.producer.length > 3 && (
                    <Badge variant="outline" size="sm">+{topic.producer.length - 3}</Badge>
                  )}
                </div>
              </div>
              <div>
                <div className="text-sm font-medium text-gray-900 mb-2">Consumers ({topic.consumer.length})</div>
                <div className="flex flex-wrap gap-1">
                  {topic.consumer.slice(0, 3).map(consumer => (
                    <Badge key={consumer} variant="outline" size="sm">
                      {consumer}
                    </Badge>
                  ))}
                  {topic.consumer.length > 3 && (
                    <Badge variant="outline" size="sm">+{topic.consumer.length - 3}</Badge>
                  )}
                </div>
              </div>
            </div>

            {/* SLA & Correlation IDs */}
            <div className="grid grid-cols-2 gap-6 mb-4">
              <div>
                <div className="text-sm font-medium text-gray-900 mb-2">SLA Requirements</div>
                <div className="text-xs text-gray-600 space-y-0.5">
                  <div>Max Latency: {topic.sla.maxLatencyMs}ms</div>
                  <div>Min Throughput: {topic.sla.minThroughputMps} msg/s</div>
                  <div>Availability: {topic.sla.availabilityPercent}%</div>
                  <div>Durability: {topic.sla.durabilityLevel.replace('_', ' ')}</div>
                </div>
              </div>
              <div>
                <div className="text-sm font-medium text-gray-900 mb-2">Correlation IDs</div>
                <div className="flex flex-wrap gap-1">
                  {topic.correlationIds.traceId && <Badge variant="outline" size="sm">trace</Badge>}
                  {topic.correlationIds.vehicleId && <Badge variant="outline" size="sm">vehicle</Badge>}
                  {topic.correlationIds.tripId && <Badge variant="outline" size="sm">trip</Badge>}
                  {topic.correlationIds.userId && <Badge variant="outline" size="sm">user</Badge>}
                  {topic.correlationIds.customFields.slice(0, 2).map(field => (
                    <Badge key={field} variant="outline" size="sm">{field}</Badge>
                  ))}
                </div>
              </div>
            </div>

            {/* Tags */}
            {topic.tags.length > 0 && (
              <div className="mb-4">
                <div className="text-sm font-medium text-gray-900 mb-2">Tags</div>
                <div className="flex flex-wrap gap-1">
                  {topic.tags.map(tag => (
                    <Badge key={tag} variant="outline" size="sm">
                      {tag}
                    </Badge>
                  ))}
                </div>
              </div>
            )}

            <div className="flex items-center justify-between pt-4 border-t">
              <div className="text-sm text-gray-600">
                Updated: {topic.updatedAt.toLocaleDateString()} • 
                Retention: {Math.round(topic.retention.timeMs / (24 * 60 * 60 * 1000))} days
              </div>
              <div className="flex items-center space-x-2">
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

        {filteredTopics.length === 0 && (
          <div className="text-center py-12">
            <Radio className="w-16 h-16 text-gray-400 mx-auto mb-4" />
            <h3 className="text-lg font-medium text-gray-900 mb-2">No Topics Found</h3>
            <p className="text-gray-600">
              {filters.search || filters.category || filters.status || filters.producer || filters.consumer
                ? 'No topics match your current filters'
                : 'No event topics available'
              }
            </p>
          </div>
        )}
      </div>

      {/* Topic Details Dialog */}
      <Dialog open={showTopicDialog} onOpenChange={setShowTopicDialog}>
        <DialogContent className="max-w-6xl max-h-[90vh] overflow-y-auto">
          <DialogHeader>
            <DialogTitle>Topic Details: {selectedTopic?.displayName}</DialogTitle>
          </DialogHeader>

          {selectedTopic && (
            <Tabs value={activeTab} onValueChange={setActiveTab}>
              <TabsList>
                <TabsTrigger value="overview">Overview</TabsTrigger>
                <TabsTrigger value="schema">Schema</TabsTrigger>
                <TabsTrigger value="metrics">Metrics</TabsTrigger>
                <TabsTrigger value="examples">Examples</TabsTrigger>
                <TabsTrigger value="configuration">Configuration</TabsTrigger>
              </TabsList>

              <TabsContent value="overview" className="space-y-4">
                <div className="grid grid-cols-2 gap-6">
                  <Card className="p-4">
                    <h3 className="text-lg font-medium text-gray-900 mb-3">Topic Information</h3>
                    <div className="space-y-2 text-sm">
                      <div className="flex justify-between">
                        <span className="text-gray-600">Name:</span>
                        <span className="font-medium font-mono">{selectedTopic.name}</span>
                      </div>
                      <div className="flex justify-between">
                        <span className="text-gray-600">Display Name:</span>
                        <span className="font-medium">{selectedTopic.displayName}</span>
                      </div>
                      <div className="flex justify-between">
                        <span className="text-gray-600">Category:</span>
                        <Badge className={getCategoryColor(selectedTopic.category)}>
                          {selectedTopic.category}
                        </Badge>
                      </div>
                      <div className="flex justify-between">
                        <span className="text-gray-600">Status:</span>
                        <Badge className={getStatusColor(selectedTopic.status)}>
                          {selectedTopic.status}
                        </Badge>
                      </div>
                      <div className="flex justify-between">
                        <span className="text-gray-600">Version:</span>
                        <span className="font-medium">v{selectedTopic.version}</span>
                      </div>
                    </div>
                  </Card>

                  <Card className="p-4">
                    <h3 className="text-lg font-medium text-gray-900 mb-3">Current Metrics</h3>
                    <div className="space-y-2 text-sm">
                      <div className="flex justify-between">
                        <span className="text-gray-600">Messages/sec:</span>
                        <span className="font-medium">{Math.round(selectedTopic.metrics.messagesPerSecond)}</span>
                      </div>
                      <div className="flex justify-between">
                        <span className="text-gray-600">Bytes/sec:</span>
                        <span className="font-medium">{formatBytes(selectedTopic.metrics.bytesPerSecond)}</span>
                      </div>
                      <div className="flex justify-between">
                        <span className="text-gray-600">Total Messages:</span>
                        <span className="font-medium">{formatNumber(selectedTopic.metrics.totalMessages)}</span>
                      </div>
                      <div className="flex justify-between">
                        <span className="text-gray-600">Avg Message Size:</span>
                        <span className="font-medium">{formatBytes(selectedTopic.metrics.avgMessageSize)}</span>
                      </div>
                      <div className="flex justify-between">
                        <span className="text-gray-600">Lag:</span>
                        <span className={`font-medium ${
                          selectedTopic.metrics.lagMs > 1000 ? 'text-red-600' :
                          selectedTopic.metrics.lagMs > 500 ? 'text-yellow-600' : 'text-green-600'
                        }`}>
                          {Math.round(selectedTopic.metrics.lagMs)}ms
                        </span>
                      </div>
                    </div>
                  </Card>
                </div>

                <Card className="p-4">
                  <h3 className="text-lg font-medium text-gray-900 mb-3">Description</h3>
                  <p className="text-gray-600">{selectedTopic.description}</p>
                </Card>

                <div className="grid grid-cols-2 gap-6">
                  <Card className="p-4">
                    <h3 className="text-lg font-medium text-gray-900 mb-3">Producers</h3>
                    <div className="space-y-2">
                      {selectedTopic.producer.map(producer => (
                        <div key={producer} className="flex items-center justify-between p-2 bg-gray-50 rounded">
                          <span className="text-sm font-medium">{producer}</span>
                          <Badge variant="outline" size="sm">Producer</Badge>
                        </div>
                      ))}
                    </div>
                  </Card>

                  <Card className="p-4">
                    <h3 className="text-lg font-medium text-gray-900 mb-3">Consumers</h3>
                    <div className="space-y-2">
                      {selectedTopic.consumer.map(consumer => (
                        <div key={consumer} className="flex items-center justify-between p-2 bg-gray-50 rounded">
                          <span className="text-sm font-medium">{consumer}</span>
                          <Badge variant="outline" size="sm">Consumer</Badge>
                        </div>
                      ))}
                    </div>
                  </Card>
                </div>
              </TabsContent>

              <TabsContent value="schema" className="space-y-4">
                <div className="grid grid-cols-2 gap-6">
                  <Card className="p-4">
                    <h3 className="text-lg font-medium text-gray-900 mb-3">Schema Information</h3>
                    <div className="space-y-2 text-sm">
                      <div className="flex justify-between">
                        <span className="text-gray-600">Format:</span>
                        <span className="font-medium">{selectedTopic.schema.format.toUpperCase()}</span>
                      </div>
                      <div className="flex justify-between">
                        <span className="text-gray-600">Version:</span>
                        <span className="font-medium">v{selectedTopic.schema.version}</span>
                      </div>
                      <div className="flex justify-between">
                        <span className="text-gray-600">Fields:</span>
                        <span className="font-medium">{selectedTopic.schema.fields.length}</span>
                      </div>
                      <div className="flex justify-between">
                        <span className="text-gray-600">Required Fields:</span>
                        <span className="font-medium">{selectedTopic.schema.required.length}</span>
                      </div>
                      <div className="flex justify-between">
                        <span className="text-gray-600">Compatibility:</span>
                        <Badge variant="outline" className="capitalize">
                          {selectedTopic.schema.compatibility}
                        </Badge>
                      </div>
                    </div>
                  </Card>

                  <Card className="p-4">
                    <h3 className="text-lg font-medium text-gray-900 mb-3">Time Sync Configuration</h3>
                    <div className="space-y-2 text-sm">
                      <div className="flex justify-between">
                        <span className="text-gray-600">Clock Sync Required:</span>
                        <Badge className={selectedTopic.timeSync.clockSyncRequired ? 'bg-green-100 text-green-800' : 'bg-red-100 text-red-800'}>
                          {selectedTopic.timeSync.clockSyncRequired ? 'Yes' : 'No'}
                        </Badge>
                      </div>
                      <div className="flex justify-between">
                        <span className="text-gray-600">Max Clock Skew:</span>
                        <span className="font-medium">{selectedTopic.timeSync.maxClockSkewMs}ms</span>
                      </div>
                      <div className="flex justify-between">
                        <span className="text-gray-600">Timestamp Field:</span>
                        <span className="font-medium font-mono">{selectedTopic.timeSync.timestampField}</span>
                      </div>
                      <div className="flex justify-between">
                        <span className="text-gray-600">Format:</span>
                        <span className="font-medium">{selectedTopic.timeSync.timestampFormat}</span>
                      </div>
                      <div className="flex justify-between">
                        <span className="text-gray-600">Timezone:</span>
                        <span className="font-medium">{selectedTopic.timeSync.timezone}</span>
                      </div>
                    </div>
                  </Card>
                </div>

                <Card className="p-4">
                  <h3 className="text-lg font-medium text-gray-900 mb-3">Schema Fields</h3>
                  <div className="space-y-3">
                    {selectedTopic.schema.fields.map(field => (
                      <div key={field.name} className="p-3 border rounded">
                        <div className="flex items-center justify-between mb-2">
                          <div className="flex items-center space-x-2">
                            <span className="font-medium font-mono">{field.name}</span>
                            <Badge variant="outline" size="sm">{field.type}</Badge>
                            {field.required && (
                              <Badge className="bg-red-100 text-red-800" size="sm">Required</Badge>
                            )}
                          </div>
                        </div>
                        <p className="text-sm text-gray-600 mb-2">{field.description}</p>
                        {(field.minimum !== undefined || field.maximum !== undefined || field.pattern || field.enum) && (
                          <div className="text-xs text-gray-500">
                            {field.minimum !== undefined && <span>Min: {field.minimum} </span>}
                            {field.maximum !== undefined && <span>Max: {field.maximum} </span>}
                            {field.pattern && <span>Pattern: {field.pattern} </span>}
                            {field.enum && <span>Values: {field.enum.join(', ')}</span>}
                          </div>
                        )}
                      </div>
                    ))}
                  </div>
                </Card>
              </TabsContent>

              <TabsContent value="metrics" className="space-y-4">
                <div className="grid grid-cols-2 md:grid-cols-4 gap-4">
                  <Card className="p-4 text-center">
                    <BarChart3 className="w-8 h-8 text-blue-600 mx-auto mb-2" />
                    <div className="text-2xl font-bold text-gray-900">
                      {Math.round(selectedTopic.metrics.messagesPerSecond)}
                    </div>
                    <div className="text-sm text-gray-600">Messages/sec</div>
                  </Card>

                  <Card className="p-4 text-center">
                    <Activity className="w-8 h-8 text-green-600 mx-auto mb-2" />
                    <div className="text-2xl font-bold text-gray-900">
                      {formatBytes(selectedTopic.metrics.bytesPerSecond)}
                    </div>
                    <div className="text-sm text-gray-600">Bytes/sec</div>
                  </Card>

                  <Card className="p-4 text-center">
                    <Clock className="w-8 h-8 text-orange-600 mx-auto mb-2" />
                    <div className={`text-2xl font-bold ${
                      selectedTopic.metrics.lagMs > 1000 ? 'text-red-600' :
                      selectedTopic.metrics.lagMs > 500 ? 'text-yellow-600' : 'text-green-600'
                    }`}>
                      {Math.round(selectedTopic.metrics.lagMs)}ms
                    </div>
                    <div className="text-sm text-gray-600">Lag</div>
                  </Card>

                  <Card className="p-4 text-center">
                    <TrendingUp className="w-8 h-8 text-purple-600 mx-auto mb-2" />
                    <div className="text-2xl font-bold text-gray-900">
                      {Math.round(selectedTopic.metrics.peakThroughput)}
                    </div>
                    <div className="text-sm text-gray-600">Peak Msg/sec</div>
                  </Card>
                </div>

                <div className="grid grid-cols-1 md:grid-cols-2 gap-6">
                  <Card className="p-4">
                    <h3 className="text-lg font-medium text-gray-900 mb-3">Consumer Metrics</h3>
                    <div className="space-y-3">
                      {selectedTopic.metrics.consumers.map(consumer => (
                        <div key={consumer.consumerId} className="p-3 bg-gray-50 rounded">
                          <div className="flex items-center justify-between mb-2">
                            <span className="font-medium text-sm">{consumer.consumerId}</span>
                            <Badge className={
                              consumer.status === 'active' ? 'bg-green-100 text-green-800' :
                              consumer.status === 'error' ? 'bg-red-100 text-red-800' :
                              'bg-gray-100 text-gray-800'
                            } size="sm">
                              {consumer.status}
                            </Badge>
                          </div>
                          <div className="grid grid-cols-3 gap-2 text-xs">
                            <div>
                              <span className="text-gray-500">Group:</span>
                              <div className="font-medium">{consumer.consumerGroup}</div>
                            </div>
                            <div>
                              <span className="text-gray-500">Lag:</span>
                              <div className="font-medium">{consumer.lag}</div>
                            </div>
                            <div>
                              <span className="text-gray-500">Throughput:</span>
                              <div className="font-medium">{consumer.throughput} msg/s</div>
                            </div>
                          </div>
                        </div>
                      ))}
                    </div>
                  </Card>

                  <Card className="p-4">
                    <h3 className="text-lg font-medium text-gray-900 mb-3">Producer Metrics</h3>
                    <div className="space-y-3">
                      {selectedTopic.metrics.producers.map(producer => (
                        <div key={producer.producerId} className="p-3 bg-gray-50 rounded">
                          <div className="flex items-center justify-between mb-2">
                            <span className="font-medium text-sm">{producer.producerId}</span>
                            <Badge className={
                              producer.status === 'active' ? 'bg-green-100 text-green-800' :
                              producer.status === 'error' ? 'bg-red-100 text-red-800' :
                              'bg-gray-100 text-gray-800'
                            } size="sm">
                              {producer.status}
                            </Badge>
                          </div>
                          <div className="grid grid-cols-3 gap-2 text-xs">
                            <div>
                              <span className="text-gray-500">Throughput:</span>
                              <div className="font-medium">{producer.throughput} msg/s</div>
                            </div>
                            <div>
                              <span className="text-gray-500">Error Rate:</span>
                              <div className="font-medium">{(producer.errorRate * 100).toFixed(2)}%</div>
                            </div>
                            <div>
                              <span className="text-gray-500">Last Produced:</span>
                              <div className="font-medium">{producer.lastProduced.toLocaleTimeString()}</div>
                            </div>
                          </div>
                        </div>
                      ))}
                    </div>
                  </Card>
                </div>
              </TabsContent>

              <TabsContent value="examples" className="space-y-4">
                <div className="space-y-6">
                  {selectedTopic.examples.map(example => (
                    <Card key={example.id} className="p-4">
                      <div className="flex items-center justify-between mb-3">
                        <h4 className="font-medium text-gray-900">{example.name}</h4>
                        <div className="flex items-center space-x-2">
                          <Button variant="outline" size="sm">
                            <Download className="w-4 h-4 mr-1" />
                            Export
                          </Button>
                          <Button variant="outline" size="sm">
                            <Eye className="w-4 h-4 mr-1" />
                            View Raw
                          </Button>
                        </div>
                      </div>
                      
                      <p className="text-sm text-gray-600 mb-4">{example.description}</p>
                      
                      <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
                        <div>
                          <div className="text-sm font-medium text-gray-900 mb-2">Message Payload</div>
                          <pre className="text-xs bg-gray-100 p-3 rounded overflow-x-auto">
                            {JSON.stringify(example.payload, null, 2)}
                          </pre>
                        </div>
                        
                        <div>
                          <div className="text-sm font-medium text-gray-900 mb-2">Correlation IDs</div>
                          <div className="space-y-1">
                            {Object.entries(example.correlationIds).map(([key, value]) => (
                              <div key={key} className="flex items-center justify-between p-2 bg-gray-50 rounded text-xs">
                                <span className="font-medium">{key}:</span>
                                <span className="font-mono">{value}</span>
                              </div>
                            ))}
                          </div>
                        </div>
                      </div>
                    </Card>
                  ))}
                </div>
              </TabsContent>

              <TabsContent value="configuration" className="space-y-4">
                <div className="grid grid-cols-1 md:grid-cols-2 gap-6">
                  <Card className="p-4">
                    <h3 className="text-lg font-medium text-gray-900 mb-3">Topic Configuration</h3>
                    <div className="space-y-2 text-sm">
                      <div className="flex justify-between">
                        <span className="text-gray-600">Partitions:</span>
                        <span className="font-medium">{selectedTopic.partitions}</span>
                      </div>
                      <div className="flex justify-between">
                        <span className="text-gray-600">Replication Factor:</span>
                        <span className="font-medium">{selectedTopic.replicationFactor}</span>
                      </div>
                      <div className="flex justify-between">
                        <span className="text-gray-600">Compression:</span>
                        <span className="font-medium capitalize">{selectedTopic.compression}</span>
                      </div>
                      <div className="flex justify-between">
                        <span className="text-gray-600">Cleanup Policy:</span>
                        <span className="font-medium">{selectedTopic.retention.cleanupPolicy}</span>
                      </div>
                    </div>
                  </Card>

                  <Card className="p-4">
                    <h3 className="text-lg font-medium text-gray-900 mb-3">Retention Policy</h3>
                    <div className="space-y-2 text-sm">
                      <div className="flex justify-between">
                        <span className="text-gray-600">Time Retention:</span>
                        <span className="font-medium">
                          {Math.round(selectedTopic.retention.timeMs / (24 * 60 * 60 * 1000))} days
                        </span>
                      </div>
                      <div className="flex justify-between">
                        <span className="text-gray-600">Size Retention:</span>
                        <span className="font-medium">{formatBytes(selectedTopic.retention.sizeBytes)}</span>
                      </div>
                      <div className="flex justify-between">
                        <span className="text-gray-600">Compaction Policy:</span>
                        <span className="font-medium capitalize">{selectedTopic.retention.compactionPolicy}</span>
                      </div>
                    </div>
                  </Card>

                  <Card className="p-4">
                    <h3 className="text-lg font-medium text-gray-900 mb-3">SLA Requirements</h3>
                    <div className="space-y-2 text-sm">
                      <div className="flex justify-between">
                        <span className="text-gray-600">Max Latency:</span>
                        <span className="font-medium">{selectedTopic.sla.maxLatencyMs}ms</span>
                      </div>
                      <div className="flex justify-between">
                        <span className="text-gray-600">Min Throughput:</span>
                        <span className="font-medium">{selectedTopic.sla.minThroughputMps} msg/s</span>
                      </div>
                      <div className="flex justify-between">
                        <span className="text-gray-600">Availability:</span>
                        <span className="font-medium">{selectedTopic.sla.availabilityPercent}%</span>
                      </div>
                      <div className="flex justify-between">
                        <span className="text-gray-600">Durability:</span>
                        <span className="font-medium">{selectedTopic.sla.durabilityLevel.replace('_', ' ')}</span>
                      </div>
                      <div className="flex justify-between">
                        <span className="text-gray-600">Ordering:</span>
                        <span className="font-medium capitalize">{selectedTopic.sla.orderingGuarantee}</span>
                      </div>
                    </div>
                  </Card>

                  <Card className="p-4">
                    <h3 className="text-lg font-medium text-gray-900 mb-3">Correlation Configuration</h3>
                    <div className="space-y-2 text-sm">
                      <div className="flex justify-between">
                        <span className="text-gray-600">Trace ID:</span>
                        <Badge className={selectedTopic.correlationIds.traceId ? 'bg-green-100 text-green-800' : 'bg-gray-100 text-gray-800'} size="sm">
                          {selectedTopic.correlationIds.traceId ? 'Enabled' : 'Disabled'}
                        </Badge>
                      </div>
                      <div className="flex justify-between">
                        <span className="text-gray-600">Vehicle ID:</span>
                        <Badge className={selectedTopic.correlationIds.vehicleId ? 'bg-green-100 text-green-800' : 'bg-gray-100 text-gray-800'} size="sm">
                          {selectedTopic.correlationIds.vehicleId ? 'Enabled' : 'Disabled'}
                        </Badge>
                      </div>
                      <div className="flex justify-between">
                        <span className="text-gray-600">Trip ID:</span>
                        <Badge className={selectedTopic.correlationIds.tripId ? 'bg-green-100 text-green-800' : 'bg-gray-100 text-gray-800'} size="sm">
                          {selectedTopic.correlationIds.tripId ? 'Enabled' : 'Disabled'}
                        </Badge>
                      </div>
                      <div className="flex justify-between">
                        <span className="text-gray-600">User ID:</span>
                        <Badge className={selectedTopic.correlationIds.userId ? 'bg-green-100 text-green-800' : 'bg-gray-100 text-gray-800'} size="sm">
                          {selectedTopic.correlationIds.userId ? 'Enabled' : 'Disabled'}
                        </Badge>
                      </div>
                    </div>
                    
                    {selectedTopic.correlationIds.customFields.length > 0 && (
                      <div className="mt-3">
                        <div className="text-sm font-medium text-gray-900 mb-2">Custom Fields</div>
                        <div className="flex flex-wrap gap-1">
                          {selectedTopic.correlationIds.customFields.map(field => (
                            <Badge key={field} variant="outline" size="sm">{field}</Badge>
                          ))}
                        </div>
                      </div>
                    )}
                  </Card>
                </div>
              </TabsContent>
            </Tabs>
          )}
        </DialogContent>
      </Dialog>
    </div>
  )
}

export default EventCatalog
