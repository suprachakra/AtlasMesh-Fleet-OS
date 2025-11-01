import React, { useState, useCallback, useEffect, useMemo } from 'react'
import { 
  Map, MapPin, Navigation, Compass, Target, Route, Shield, AlertTriangle,
  CheckCircle, XCircle, Edit3, Trash2, Copy, Eye, EyeOff, Settings,
  RefreshCw, Download, Upload, Play, Pause, SkipForward, Rewind,
  Layers, Filter, Search, Plus, Save, X, Clock, Users, Activity
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
import { Slider } from '../ui/Slider'

// Types
interface RouteDefinition {
  id: string
  name: string
  description: string
  type: 'fixed' | 'dynamic' | 'emergency' | 'maintenance'
  priority: 'low' | 'normal' | 'high' | 'critical'
  status: 'draft' | 'active' | 'inactive' | 'deprecated' | 'testing'
  waypoints: Waypoint[]
  geofences: string[] // References to geofence IDs
  constraints: RouteConstraint[]
  metadata: RouteMetadata
  usage: RouteUsage
  validation: RouteValidation
  createdAt: Date
  updatedAt: Date
  createdBy: string
  lastUsedAt?: Date
  version: string
  publishedVersion?: string
}

interface Waypoint {
  id: string
  name: string
  coordinates: Coordinates
  type: 'start' | 'end' | 'checkpoint' | 'stop' | 'charging' | 'pickup' | 'dropoff'
  dwellTime?: number // seconds
  speed?: number // m/s
  heading?: number // degrees
  instructions?: string
  required: boolean
  order: number
  geofence?: string // Associated geofence ID
}

interface Coordinates {
  latitude: number
  longitude: number
  altitude?: number
  accuracy?: number
}

interface GeofenceDefinition {
  id: string
  name: string
  description: string
  type: 'circular' | 'polygon' | 'corridor'
  geometry: GeofenceGeometry
  rules: GeofenceRule[]
  constraints: GeofenceConstraint[]
  triggers: GeofenceTrigger[]
  metadata: GeofenceMetadata
  usage: GeofenceUsage
  status: 'draft' | 'active' | 'inactive' | 'testing'
  createdAt: Date
  updatedAt: Date
  createdBy: string
  version: string
  publishedVersion?: string
}

interface GeofenceGeometry {
  center?: Coordinates // For circular geofences
  radius?: number // meters, for circular geofences
  points?: Coordinates[] // For polygon geofences
  width?: number // meters, for corridor geofences
  buffer?: number // meters, buffer zone around geometry
}

interface GeofenceRule {
  id: string
  name: string
  condition: 'entry' | 'exit' | 'dwell' | 'speed' | 'heading'
  operator: 'equals' | 'not_equals' | 'greater_than' | 'less_than' | 'between'
  value: number | string | [number, number]
  action: GeofenceAction
  priority: number
  enabled: boolean
}

interface GeofenceAction {
  type: 'alert' | 'speed_limit' | 'stop' | 'reroute' | 'log' | 'notification'
  parameters: Record<string, any>
  severity: 'info' | 'warning' | 'critical'
  escalation?: EscalationRule[]
}

interface EscalationRule {
  afterSeconds: number
  action: GeofenceAction
}

interface GeofenceTrigger {
  id: string
  event: 'vehicle_enter' | 'vehicle_exit' | 'vehicle_dwell' | 'time_based' | 'condition_met'
  condition?: string
  action: string
  enabled: boolean
  lastTriggered?: Date
  triggerCount: number
}

interface RouteConstraint {
  type: 'speed' | 'time' | 'vehicle_type' | 'weather' | 'traffic' | 'custom'
  condition: string
  value: number | string | boolean
  operator: 'equals' | 'not_equals' | 'greater_than' | 'less_than' | 'contains'
  enabled: boolean
  description: string
}

interface GeofenceConstraint {
  type: 'time' | 'weather' | 'vehicle_type' | 'user_role' | 'custom'
  condition: string
  value: number | string | boolean
  operator: 'equals' | 'not_equals' | 'greater_than' | 'less_than' | 'contains'
  enabled: boolean
  description: string
}

interface RouteMetadata {
  distance: number // meters
  estimatedDuration: number // seconds
  difficulty: 'easy' | 'medium' | 'hard'
  roadTypes: string[]
  weatherSuitability: string[]
  timeOfDayRestrictions?: TimeRestriction[]
  vehicleTypes: string[]
  maxSpeed: number
  minSpeed: number
  elevationGain?: number
  elevationLoss?: number
}

interface GeofenceMetadata {
  area?: number // square meters for polygon/circular geofences
  perimeter?: number // meters
  elevation?: {
    min: number
    max: number
    average: number
  }
  landUse?: string[]
  restrictions?: string[]
  hazards?: string[]
}

interface TimeRestriction {
  days: string[] // ['monday', 'tuesday', etc.]
  startTime: string // HH:MM format
  endTime: string // HH:MM format
  timezone: string
}

interface RouteUsage {
  totalTrips: number
  activeTrips: number
  averageRating: number
  lastUsed?: Date
  popularTimes: PopularTime[]
  vehicles: string[]
  incidents: number
  successRate: number
}

interface GeofenceUsage {
  totalVehicles: number
  currentVehicles: number
  totalTriggers: number
  recentTriggers: GeofenceTriggerEvent[]
  violationCount: number
  lastViolation?: Date
}

interface PopularTime {
  hour: number
  usage: number
  averageSpeed: number
  incidents: number
}

interface GeofenceTriggerEvent {
  id: string
  vehicleId: string
  event: string
  timestamp: Date
  coordinates: Coordinates
  metadata: Record<string, any>
}

interface RouteValidation {
  status: 'valid' | 'invalid' | 'warning' | 'pending'
  issues: ValidationIssue[]
  lastValidated: Date
  validatedBy: string
  score: number // 0-100
}

interface ValidationIssue {
  type: 'error' | 'warning' | 'info'
  code: string
  message: string
  location?: Coordinates
  severity: 'low' | 'medium' | 'high' | 'critical'
  suggestion?: string
}

interface MapLayer {
  id: string
  name: string
  type: 'routes' | 'geofences' | 'waypoints' | 'traffic' | 'weather' | 'incidents' | 'vehicles'
  visible: boolean
  opacity: number
  color?: string
  strokeWidth?: number
  fillOpacity?: number
}

interface RoutesGeofencesEditorProps {
  initialView?: 'routes' | 'geofences'
  selectedRouteId?: string
  selectedGeofenceId?: string
  onRouteSelected?: (route: RouteDefinition) => void
  onGeofenceSelected?: (geofence: GeofenceDefinition) => void
  onRoutePublished?: (route: RouteDefinition) => void
  onGeofencePublished?: (geofence: GeofenceDefinition) => void
  readOnly?: boolean
  className?: string
}

const RoutesGeofencesEditor: React.FC<RoutesGeofencesEditorProps> = ({
  initialView = 'routes',
  selectedRouteId,
  selectedGeofenceId,
  onRouteSelected,
  onGeofenceSelected,
  onRoutePublished,
  onGeofencePublished,
  readOnly = false,
  className = ''
}) => {
  // State
  const [routes, setRoutes] = useState<RouteDefinition[]>([])
  const [geofences, setGeofences] = useState<GeofenceDefinition[]>([])
  const [activeView, setActiveView] = useState(initialView)
  const [selectedRoute, setSelectedRoute] = useState<RouteDefinition | null>(null)
  const [selectedGeofence, setSelectedGeofence] = useState<GeofenceDefinition | null>(null)
  const [showRouteDialog, setShowRouteDialog] = useState(false)
  const [showGeofenceDialog, setShowGeofenceDialog] = useState(false)
  const [showPublishDialog, setShowPublishDialog] = useState(false)
  const [mapLayers, setMapLayers] = useState<MapLayer[]>([])
  const [filters, setFilters] = useState({
    status: '',
    type: '',
    search: '',
    priority: ''
  })
  const [isEditing, setIsEditing] = useState(false)
  const [validationInProgress, setValidationInProgress] = useState(false)

  // Mock data initialization
  const initializeMockData = useCallback(() => {
    const mockRoutes: RouteDefinition[] = [
      {
        id: 'route-depot-to-mall',
        name: 'Depot A to Dubai Mall',
        description: 'Primary route from main depot to Dubai Mall pickup/dropoff zone',
        type: 'fixed',
        priority: 'high',
        status: 'active',
        waypoints: [
          {
            id: 'wp-depot-start',
            name: 'Depot A - Exit Gate',
            coordinates: { latitude: 25.2048, longitude: 55.2708, accuracy: 1.0 },
            type: 'start',
            dwellTime: 30,
            speed: 0,
            heading: 90,
            instructions: 'Exit depot via main gate, proceed to Sheikh Zayed Road',
            required: true,
            order: 1
          },
          {
            id: 'wp-szr-entry',
            name: 'Sheikh Zayed Road Entry',
            coordinates: { latitude: 25.2156, longitude: 55.2794, accuracy: 2.0 },
            type: 'checkpoint',
            speed: 15,
            heading: 45,
            instructions: 'Merge onto Sheikh Zayed Road northbound',
            required: true,
            order: 2
          },
          {
            id: 'wp-mall-dropoff',
            name: 'Dubai Mall - Dropoff Zone',
            coordinates: { latitude: 25.1972, longitude: 55.2796, accuracy: 1.5 },
            type: 'end',
            dwellTime: 120,
            speed: 0,
            instructions: 'Enter designated AV dropoff zone, Level P1',
            required: true,
            order: 3,
            geofence: 'geofence-mall-zone'
          }
        ],
        geofences: ['geofence-depot-exit', 'geofence-szr-corridor', 'geofence-mall-zone'],
        constraints: [
          {
            type: 'speed',
            condition: 'max_speed',
            value: 60,
            operator: 'less_than',
            enabled: true,
            description: 'Maximum speed limit for safety'
          },
          {
            type: 'time',
            condition: 'operation_hours',
            value: '06:00-22:00',
            operator: 'contains',
            enabled: true,
            description: 'Route only available during operational hours'
          },
          {
            type: 'weather',
            condition: 'visibility',
            value: 500,
            operator: 'greater_than',
            enabled: true,
            description: 'Minimum visibility requirement in meters'
          }
        ],
        metadata: {
          distance: 15420, // 15.42 km
          estimatedDuration: 1680, // 28 minutes
          difficulty: 'medium',
          roadTypes: ['highway', 'arterial', 'local'],
          weatherSuitability: ['clear', 'partly_cloudy', 'light_rain'],
          timeOfDayRestrictions: [
            {
              days: ['monday', 'tuesday', 'wednesday', 'thursday', 'friday', 'saturday', 'sunday'],
              startTime: '06:00',
              endTime: '22:00',
              timezone: 'Asia/Dubai'
            }
          ],
          vehicleTypes: ['sedan', 'suv', 'van'],
          maxSpeed: 60,
          minSpeed: 5,
          elevationGain: 45,
          elevationLoss: 32
        },
        usage: {
          totalTrips: 2847,
          activeTrips: 12,
          averageRating: 4.6,
          lastUsed: new Date('2024-11-26T10:30:00Z'),
          popularTimes: [
            { hour: 8, usage: 45, averageSpeed: 32, incidents: 0 },
            { hour: 9, usage: 67, averageSpeed: 28, incidents: 1 },
            { hour: 17, usage: 89, averageSpeed: 25, incidents: 2 },
            { hour: 18, usage: 92, averageSpeed: 22, incidents: 1 }
          ],
          vehicles: ['atlas-001', 'atlas-002', 'atlas-003'],
          incidents: 8,
          successRate: 98.7
        },
        validation: {
          status: 'valid',
          issues: [
            {
              type: 'warning',
              code: 'TRAFFIC_CONGESTION',
              message: 'High traffic congestion expected during peak hours (17:00-19:00)',
              location: { latitude: 25.2156, longitude: 55.2794 },
              severity: 'medium',
              suggestion: 'Consider alternative route or time-based routing'
            }
          ],
          lastValidated: new Date('2024-11-26T08:00:00Z'),
          validatedBy: 'route-validation-service',
          score: 87
        },
        createdAt: new Date('2024-10-15T10:00:00Z'),
        updatedAt: new Date('2024-11-20T14:30:00Z'),
        createdBy: 'route-planner',
        lastUsedAt: new Date('2024-11-26T10:30:00Z'),
        version: '2.1.0',
        publishedVersion: '2.1.0'
      },
      {
        id: 'route-emergency-hospital',
        name: 'Emergency Route to Dubai Hospital',
        description: 'High-priority emergency route with traffic signal override capabilities',
        type: 'emergency',
        priority: 'critical',
        status: 'active',
        waypoints: [
          {
            id: 'wp-emergency-start',
            name: 'Emergency Pickup Point',
            coordinates: { latitude: 25.2048, longitude: 55.2708 },
            type: 'start',
            dwellTime: 60,
            required: true,
            order: 1
          },
          {
            id: 'wp-hospital-entrance',
            name: 'Dubai Hospital - Emergency Entrance',
            coordinates: { latitude: 25.2285, longitude: 55.2975 },
            type: 'end',
            dwellTime: 0,
            required: true,
            order: 2
          }
        ],
        geofences: ['geofence-emergency-corridor', 'geofence-hospital-zone'],
        constraints: [
          {
            type: 'speed',
            condition: 'max_speed',
            value: 80,
            operator: 'less_than',
            enabled: true,
            description: 'Emergency speed limit with traffic override'
          }
        ],
        metadata: {
          distance: 8750,
          estimatedDuration: 420, // 7 minutes
          difficulty: 'easy',
          roadTypes: ['highway', 'arterial'],
          weatherSuitability: ['clear', 'partly_cloudy', 'light_rain', 'heavy_rain'],
          vehicleTypes: ['emergency'],
          maxSpeed: 80,
          minSpeed: 10
        },
        usage: {
          totalTrips: 23,
          activeTrips: 0,
          averageRating: 4.9,
          lastUsed: new Date('2024-11-24T15:45:00Z'),
          popularTimes: [],
          vehicles: ['atlas-emergency-001'],
          incidents: 0,
          successRate: 100
        },
        validation: {
          status: 'valid',
          issues: [],
          lastValidated: new Date('2024-11-26T06:00:00Z'),
          validatedBy: 'emergency-route-validator',
          score: 98
        },
        createdAt: new Date('2024-09-01T09:00:00Z'),
        updatedAt: new Date('2024-11-15T11:00:00Z'),
        createdBy: 'emergency-planner',
        lastUsedAt: new Date('2024-11-24T15:45:00Z'),
        version: '1.3.0',
        publishedVersion: '1.3.0'
      }
    ]

    const mockGeofences: GeofenceDefinition[] = [
      {
        id: 'geofence-mall-zone',
        name: 'Dubai Mall Pickup/Dropoff Zone',
        description: 'Designated area for autonomous vehicle passenger pickup and dropoff at Dubai Mall',
        type: 'polygon',
        geometry: {
          points: [
            { latitude: 25.1970, longitude: 55.2794 },
            { latitude: 25.1974, longitude: 55.2794 },
            { latitude: 25.1974, longitude: 55.2798 },
            { latitude: 25.1970, longitude: 55.2798 }
          ],
          buffer: 5 // 5 meter buffer zone
        },
        rules: [
          {
            id: 'rule-speed-limit',
            name: 'Speed Limit in Zone',
            condition: 'speed',
            operator: 'less_than',
            value: 10,
            action: {
              type: 'speed_limit',
              parameters: { max_speed: 10 },
              severity: 'warning'
            },
            priority: 1,
            enabled: true
          },
          {
            id: 'rule-dwell-time',
            name: 'Maximum Dwell Time',
            condition: 'dwell',
            operator: 'less_than',
            value: 300, // 5 minutes
            action: {
              type: 'alert',
              parameters: { message: 'Vehicle exceeded maximum dwell time' },
              severity: 'warning',
              escalation: [
                {
                  afterSeconds: 600, // 10 minutes
                  action: {
                    type: 'reroute',
                    parameters: { destination: 'depot' },
                    severity: 'critical'
                  }
                }
              ]
            },
            priority: 2,
            enabled: true
          }
        ],
        constraints: [
          {
            type: 'time',
            condition: 'operation_hours',
            value: '08:00-22:00',
            operator: 'contains',
            enabled: true,
            description: 'Mall operating hours only'
          },
          {
            type: 'vehicle_type',
            condition: 'vehicle_category',
            value: 'passenger',
            operator: 'equals',
            enabled: true,
            description: 'Passenger vehicles only'
          }
        ],
        triggers: [
          {
            id: 'trigger-entry-log',
            event: 'vehicle_enter',
            action: 'log_entry',
            enabled: true,
            triggerCount: 1247,
            lastTriggered: new Date('2024-11-26T10:25:00Z')
          },
          {
            id: 'trigger-exit-cleanup',
            event: 'vehicle_exit',
            action: 'cleanup_session',
            enabled: true,
            triggerCount: 1245,
            lastTriggered: new Date('2024-11-26T10:20:00Z')
          }
        ],
        metadata: {
          area: 1600, // 40m x 40m = 1600 sq meters
          perimeter: 160, // 40m x 4 = 160m
          elevation: {
            min: 12,
            max: 14,
            average: 13
          },
          landUse: ['commercial', 'parking'],
          restrictions: ['no_stopping_outside_zone', 'passenger_only'],
          hazards: ['pedestrian_crossing', 'construction_nearby']
        },
        usage: {
          totalVehicles: 156,
          currentVehicles: 3,
          totalTriggers: 2492,
          recentTriggers: [
            {
              id: 'trigger-event-001',
              vehicleId: 'atlas-003',
              event: 'vehicle_enter',
              timestamp: new Date('2024-11-26T10:25:00Z'),
              coordinates: { latitude: 25.1972, longitude: 55.2796 },
              metadata: { passenger_count: 2, trip_id: 'trip-20241126-001' }
            },
            {
              id: 'trigger-event-002',
              vehicleId: 'atlas-001',
              event: 'vehicle_exit',
              timestamp: new Date('2024-11-26T10:20:00Z'),
              coordinates: { latitude: 25.1970, longitude: 55.2795 },
              metadata: { dwell_time: 180, passenger_count: 0 }
            }
          ],
          violationCount: 12,
          lastViolation: new Date('2024-11-25T16:30:00Z')
        },
        status: 'active',
        createdAt: new Date('2024-10-01T12:00:00Z'),
        updatedAt: new Date('2024-11-18T09:30:00Z'),
        createdBy: 'geofence-admin',
        version: '1.2.0',
        publishedVersion: '1.2.0'
      },
      {
        id: 'geofence-depot-exit',
        name: 'Depot A Exit Zone',
        description: 'Controlled exit zone from main depot with speed and safety checks',
        type: 'circular',
        geometry: {
          center: { latitude: 25.2048, longitude: 55.2708 },
          radius: 50, // 50 meters
          buffer: 10
        },
        rules: [
          {
            id: 'rule-exit-speed',
            name: 'Exit Speed Control',
            condition: 'speed',
            operator: 'less_than',
            value: 15,
            action: {
              type: 'speed_limit',
              parameters: { max_speed: 15 },
              severity: 'warning'
            },
            priority: 1,
            enabled: true
          },
          {
            id: 'rule-safety-check',
            name: 'Pre-Exit Safety Check',
            condition: 'entry',
            operator: 'equals',
            value: 'true',
            action: {
              type: 'log',
              parameters: { event: 'safety_check_required' },
              severity: 'info'
            },
            priority: 2,
            enabled: true
          }
        ],
        constraints: [
          {
            type: 'vehicle_type',
            condition: 'authorized_vehicle',
            value: 'true',
            operator: 'equals',
            enabled: true,
            description: 'Only authorized fleet vehicles'
          }
        ],
        triggers: [
          {
            id: 'trigger-exit-authorization',
            event: 'vehicle_enter',
            action: 'check_authorization',
            enabled: true,
            triggerCount: 892,
            lastTriggered: new Date('2024-11-26T10:15:00Z')
          }
        ],
        metadata: {
          area: 7854, // π * 50² ≈ 7854 sq meters
          perimeter: 314, // 2π * 50 ≈ 314 meters
          elevation: {
            min: 8,
            max: 10,
            average: 9
          },
          landUse: ['industrial', 'transportation'],
          restrictions: ['authorized_vehicles_only'],
          hazards: ['gate_mechanism', 'security_checkpoint']
        },
        usage: {
          totalVehicles: 45,
          currentVehicles: 0,
          totalTriggers: 892,
          recentTriggers: [],
          violationCount: 3,
          lastViolation: new Date('2024-11-20T14:15:00Z')
        },
        status: 'active',
        createdAt: new Date('2024-09-15T08:00:00Z'),
        updatedAt: new Date('2024-11-22T16:00:00Z'),
        createdBy: 'depot-admin',
        version: '1.1.0',
        publishedVersion: '1.1.0'
      }
    ]

    const mockLayers: MapLayer[] = [
      {
        id: 'layer-routes',
        name: 'Routes',
        type: 'routes',
        visible: true,
        opacity: 0.8,
        color: '#3B82F6',
        strokeWidth: 3
      },
      {
        id: 'layer-geofences',
        name: 'Geofences',
        type: 'geofences',
        visible: true,
        opacity: 0.6,
        color: '#EF4444',
        strokeWidth: 2,
        fillOpacity: 0.2
      },
      {
        id: 'layer-waypoints',
        name: 'Waypoints',
        type: 'waypoints',
        visible: true,
        opacity: 1.0,
        color: '#10B981'
      },
      {
        id: 'layer-traffic',
        name: 'Traffic',
        type: 'traffic',
        visible: false,
        opacity: 0.7
      },
      {
        id: 'layer-weather',
        name: 'Weather',
        type: 'weather',
        visible: false,
        opacity: 0.5
      },
      {
        id: 'layer-vehicles',
        name: 'Live Vehicles',
        type: 'vehicles',
        visible: true,
        opacity: 1.0,
        color: '#8B5CF6'
      }
    ]

    setRoutes(mockRoutes)
    setGeofences(mockGeofences)
    setMapLayers(mockLayers)
  }, [])

  useEffect(() => {
    initializeMockData()
  }, [initializeMockData])

  // Set initial selections
  useEffect(() => {
    if (selectedRouteId && routes.length > 0) {
      const route = routes.find(r => r.id === selectedRouteId)
      if (route) {
        setSelectedRoute(route)
        onRouteSelected?.(route)
      }
    }
  }, [selectedRouteId, routes, onRouteSelected])

  useEffect(() => {
    if (selectedGeofenceId && geofences.length > 0) {
      const geofence = geofences.find(g => g.id === selectedGeofenceId)
      if (geofence) {
        setSelectedGeofence(geofence)
        onGeofenceSelected?.(geofence)
      }
    }
  }, [selectedGeofenceId, geofences, onGeofenceSelected])

  // Handlers
  const handleRouteSelect = useCallback((route: RouteDefinition) => {
    setSelectedRoute(route)
    onRouteSelected?.(route)
  }, [onRouteSelected])

  const handleGeofenceSelect = useCallback((geofence: GeofenceDefinition) => {
    setSelectedGeofence(geofence)
    onGeofenceSelected?.(geofence)
  }, [onGeofenceSelected])

  const handleLayerToggle = useCallback((layerId: string, visible: boolean) => {
    setMapLayers(prev => prev.map(layer => 
      layer.id === layerId ? { ...layer, visible } : layer
    ))
  }, [])

  const handleValidateRoute = useCallback(async (routeId: string) => {
    setValidationInProgress(true)
    
    // Simulate validation process
    setTimeout(() => {
      setRoutes(prev => prev.map(route => 
        route.id === routeId 
          ? {
              ...route,
              validation: {
                ...route.validation,
                status: Math.random() > 0.8 ? 'invalid' : 'valid',
                lastValidated: new Date(),
                validatedBy: 'validation-service',
                score: Math.floor(Math.random() * 30) + 70 // 70-100
              }
            }
          : route
      ))
      setValidationInProgress(false)
    }, 3000)
  }, [])

  const handlePublishRoute = useCallback((route: RouteDefinition) => {
    setRoutes(prev => prev.map(r => 
      r.id === route.id 
        ? {
            ...r,
            status: 'active',
            publishedVersion: r.version,
            updatedAt: new Date()
          }
        : r
    ))
    onRoutePublished?.(route)
  }, [onRoutePublished])

  const handlePublishGeofence = useCallback((geofence: GeofenceDefinition) => {
    setGeofences(prev => prev.map(g => 
      g.id === geofence.id 
        ? {
            ...g,
            status: 'active',
            publishedVersion: g.version,
            updatedAt: new Date()
          }
        : g
    ))
    onGeofencePublished?.(geofence)
  }, [onGeofencePublished])

  // Filtered data
  const filteredRoutes = useMemo(() => {
    return routes
      .filter(route => !filters.status || route.status === filters.status)
      .filter(route => !filters.type || route.type === filters.type)
      .filter(route => !filters.priority || route.priority === filters.priority)
      .filter(route => {
        if (!filters.search) return true
        const searchTerm = filters.search.toLowerCase()
        return (
          route.name.toLowerCase().includes(searchTerm) ||
          route.description.toLowerCase().includes(searchTerm)
        )
      })
      .sort((a, b) => {
        // Sort by priority (critical first) then by last used
        const priorityOrder = { critical: 4, high: 3, normal: 2, low: 1 }
        const aPriority = priorityOrder[a.priority]
        const bPriority = priorityOrder[b.priority]
        
        if (aPriority !== bPriority) {
          return bPriority - aPriority
        }
        
        const aLastUsed = a.lastUsedAt?.getTime() || 0
        const bLastUsed = b.lastUsedAt?.getTime() || 0
        return bLastUsed - aLastUsed
      })
  }, [routes, filters])

  const filteredGeofences = useMemo(() => {
    return geofences
      .filter(geofence => !filters.status || geofence.status === filters.status)
      .filter(geofence => !filters.type || geofence.type === filters.type)
      .filter(geofence => {
        if (!filters.search) return true
        const searchTerm = filters.search.toLowerCase()
        return (
          geofence.name.toLowerCase().includes(searchTerm) ||
          geofence.description.toLowerCase().includes(searchTerm)
        )
      })
      .sort((a, b) => b.updatedAt.getTime() - a.updatedAt.getTime())
  }, [geofences, filters])

  // Statistics
  const stats = useMemo(() => {
    const totalRoutes = routes.length
    const activeRoutes = routes.filter(r => r.status === 'active').length
    const totalGeofences = geofences.length
    const activeGeofences = geofences.filter(g => g.status === 'active').length
    
    const totalTrips = routes.reduce((sum, r) => sum + r.usage.totalTrips, 0)
    const activeTrips = routes.reduce((sum, r) => sum + r.usage.activeTrips, 0)
    const avgSuccessRate = routes.reduce((sum, r) => sum + r.usage.successRate, 0) / totalRoutes || 0
    
    const totalViolations = geofences.reduce((sum, g) => sum + g.usage.violationCount, 0)
    const currentVehiclesInGeofences = geofences.reduce((sum, g) => sum + g.usage.currentVehicles, 0)
    
    const validationIssues = routes.reduce((sum, r) => sum + r.validation.issues.length, 0)

    return {
      totalRoutes,
      activeRoutes,
      totalGeofences,
      activeGeofences,
      totalTrips,
      activeTrips,
      avgSuccessRate,
      totalViolations,
      currentVehiclesInGeofences,
      validationIssues
    }
  }, [routes, geofences])

  const getStatusColor = (status: string) => {
    switch (status) {
      case 'active': return 'bg-green-100 text-green-800'
      case 'draft': return 'bg-yellow-100 text-yellow-800'
      case 'inactive': return 'bg-gray-100 text-gray-800'
      case 'testing': return 'bg-blue-100 text-blue-800'
      case 'deprecated': return 'bg-red-100 text-red-800'
      default: return 'bg-gray-100 text-gray-800'
    }
  }

  const getPriorityColor = (priority: string) => {
    switch (priority) {
      case 'critical': return 'bg-red-100 text-red-800'
      case 'high': return 'bg-orange-100 text-orange-800'
      case 'normal': return 'bg-blue-100 text-blue-800'
      case 'low': return 'bg-green-100 text-green-800'
      default: return 'bg-gray-100 text-gray-800'
    }
  }

  const getTypeColor = (type: string) => {
    switch (type) {
      case 'fixed': return 'bg-blue-100 text-blue-800'
      case 'dynamic': return 'bg-purple-100 text-purple-800'
      case 'emergency': return 'bg-red-100 text-red-800'
      case 'maintenance': return 'bg-yellow-100 text-yellow-800'
      case 'circular': return 'bg-green-100 text-green-800'
      case 'polygon': return 'bg-indigo-100 text-indigo-800'
      case 'corridor': return 'bg-pink-100 text-pink-800'
      default: return 'bg-gray-100 text-gray-800'
    }
  }

  const formatDistance = (meters: number) => {
    if (meters >= 1000) {
      return `${(meters / 1000).toFixed(1)} km`
    }
    return `${meters} m`
  }

  const formatDuration = (seconds: number) => {
    const minutes = Math.floor(seconds / 60)
    const hours = Math.floor(minutes / 60)
    
    if (hours > 0) {
      return `${hours}h ${minutes % 60}m`
    }
    return `${minutes}m`
  }

  return (
    <div className={`space-y-6 ${className}`}>
      {/* Header */}
      <div className="flex items-center justify-between">
        <div className="flex items-center space-x-3">
          <Map className="w-6 h-6 text-blue-600" />
          <h2 className="text-2xl font-semibold text-gray-900">Routes & Geofences Editor</h2>
          {readOnly && (
            <Badge variant="outline">Read Only</Badge>
          )}
        </div>
        <div className="flex items-center space-x-2">
          {!readOnly && (
            <>
              <Button variant="outline">
                <Plus className="w-4 h-4 mr-2" />
                Create {activeView === 'routes' ? 'Route' : 'Geofence'}
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
      <div className="grid grid-cols-2 md:grid-cols-5 lg:grid-cols-10 gap-4">
        <Card className="p-4">
          <div className="text-center">
            <div className="text-2xl font-bold text-gray-900">{stats.totalRoutes}</div>
            <div className="text-sm text-gray-600">Total Routes</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className="text-2xl font-bold text-green-600">{stats.activeRoutes}</div>
            <div className="text-sm text-gray-600">Active Routes</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className="text-2xl font-bold text-blue-600">{stats.totalGeofences}</div>
            <div className="text-sm text-gray-600">Geofences</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className="text-2xl font-bold text-purple-600">{stats.activeGeofences}</div>
            <div className="text-sm text-gray-600">Active Zones</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className="text-2xl font-bold text-indigo-600">{stats.totalTrips.toLocaleString()}</div>
            <div className="text-sm text-gray-600">Total Trips</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className="text-2xl font-bold text-orange-600">{stats.activeTrips}</div>
            <div className="text-sm text-gray-600">Active Trips</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className="text-2xl font-bold text-teal-600">{stats.avgSuccessRate.toFixed(1)}%</div>
            <div className="text-sm text-gray-600">Success Rate</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className="text-2xl font-bold text-red-600">{stats.totalViolations}</div>
            <div className="text-sm text-gray-600">Violations</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className="text-2xl font-bold text-pink-600">{stats.currentVehiclesInGeofences}</div>
            <div className="text-sm text-gray-600">In Zones</div>
          </div>
        </Card>
        <Card className="p-4">
          <div className="text-center">
            <div className="text-2xl font-bold text-amber-600">{stats.validationIssues}</div>
            <div className="text-sm text-gray-600">Issues</div>
          </div>
        </Card>
      </div>

      <div className="grid grid-cols-1 lg:grid-cols-3 gap-6">
        {/* Left Panel - List & Controls */}
        <div className="lg:col-span-1 space-y-4">
          {/* View Toggle */}
          <Card className="p-4">
            <Tabs value={activeView} onValueChange={setActiveView}>
              <TabsList className="grid w-full grid-cols-2">
                <TabsTrigger value="routes">
                  <Route className="w-4 h-4 mr-2" />
                  Routes
                </TabsTrigger>
                <TabsTrigger value="geofences">
                  <Shield className="w-4 h-4 mr-2" />
                  Geofences
                </TabsTrigger>
              </TabsList>
            </Tabs>
          </div>

          {/* Filters */}
          <Card className="p-4">
            <div className="space-y-3">
              <div className="flex items-center space-x-2">
                <Search className="w-4 h-4 text-gray-500" />
                <Input
                  placeholder={`Search ${activeView}...`}
                  value={filters.search}
                  onChange={(e) => setFilters(prev => ({ ...prev, search: e.target.value }))}
                  className="flex-1"
                />
              </div>
              
              <Select
                value={filters.status}
                onValueChange={(value) => setFilters(prev => ({ ...prev, status: value }))}
              >
                <option value="">All Status</option>
                <option value="active">Active</option>
                <option value="draft">Draft</option>
                <option value="inactive">Inactive</option>
                <option value="testing">Testing</option>
                <option value="deprecated">Deprecated</option>
              </Select>

              <Select
                value={filters.type}
                onValueChange={(value) => setFilters(prev => ({ ...prev, type: value }))}
              >
                <option value="">All Types</option>
                {activeView === 'routes' ? (
                  <>
                    <option value="fixed">Fixed</option>
                    <option value="dynamic">Dynamic</option>
                    <option value="emergency">Emergency</option>
                    <option value="maintenance">Maintenance</option>
                  </>
                ) : (
                  <>
                    <option value="circular">Circular</option>
                    <option value="polygon">Polygon</option>
                    <option value="corridor">Corridor</option>
                  </>
                )}
              </Select>

              {activeView === 'routes' && (
                <Select
                  value={filters.priority}
                  onValueChange={(value) => setFilters(prev => ({ ...prev, priority: value }))}
                >
                  <option value="">All Priorities</option>
                  <option value="critical">Critical</option>
                  <option value="high">High</option>
                  <option value="normal">Normal</option>
                  <option value="low">Low</option>
                </Select>
              )}
            </div>
          </Card>

          {/* Map Layers */}
          <Card className="p-4">
            <h3 className="font-medium text-gray-900 mb-3 flex items-center">
              <Layers className="w-4 h-4 mr-2" />
              Map Layers
            </h3>
            <div className="space-y-2">
              {mapLayers.map(layer => (
                <div key={layer.id} className="flex items-center justify-between">
                  <div className="flex items-center space-x-2">
                    <div 
                      className="w-3 h-3 rounded" 
                      style={{ backgroundColor: layer.color || '#6B7280' }}
                    />
                    <span className="text-sm">{layer.name}</span>
                  </div>
                  <Switch
                    checked={layer.visible}
                    onCheckedChange={(visible) => handleLayerToggle(layer.id, visible)}
                    size="sm"
                  />
                </div>
              ))}
            </div>
          </Card>

          {/* Items List */}
          <Card className="p-4">
            <div className="space-y-3 max-h-96 overflow-y-auto">
              {activeView === 'routes' ? (
                filteredRoutes.map(route => (
                  <div
                    key={route.id}
                    className={`p-3 border rounded cursor-pointer transition-colors ${
                      selectedRoute?.id === route.id 
                        ? 'border-blue-500 bg-blue-50' 
                        : 'border-gray-200 hover:border-gray-300'
                    }`}
                    onClick={() => handleRouteSelect(route)}
                  >
                    <div className="flex items-start justify-between mb-2">
                      <h4 className="font-medium text-gray-900 text-sm">{route.name}</h4>
                      <div className="flex items-center space-x-1">
                        <Badge className={getStatusColor(route.status)} size="sm">
                          {route.status}
                        </Badge>
                        <Badge className={getPriorityColor(route.priority)} size="sm">
                          {route.priority}
                        </Badge>
                      </div>
                    </div>
                    <p className="text-xs text-gray-600 mb-2">{route.description}</p>
                    <div className="grid grid-cols-2 gap-2 text-xs">
                      <div>
                        <span className="text-gray-500">Distance:</span>
                        <span className="font-medium ml-1">{formatDistance(route.metadata.distance)}</span>
                      </div>
                      <div>
                        <span className="text-gray-500">Duration:</span>
                        <span className="font-medium ml-1">{formatDuration(route.metadata.estimatedDuration)}</span>
                      </div>
                      <div>
                        <span className="text-gray-500">Trips:</span>
                        <span className="font-medium ml-1">{route.usage.totalTrips}</span>
                      </div>
                      <div>
                        <span className="text-gray-500">Success:</span>
                        <span className="font-medium ml-1">{route.usage.successRate}%</span>
                      </div>
                    </div>
                    {route.validation.issues.length > 0 && (
                      <div className="mt-2 flex items-center space-x-1">
                        <AlertTriangle className="w-3 h-3 text-yellow-500" />
                        <span className="text-xs text-yellow-600">{route.validation.issues.length} validation issues</span>
                      </div>
                    )}
                  </div>
                ))
              ) : (
                filteredGeofences.map(geofence => (
                  <div
                    key={geofence.id}
                    className={`p-3 border rounded cursor-pointer transition-colors ${
                      selectedGeofence?.id === geofence.id 
                        ? 'border-blue-500 bg-blue-50' 
                        : 'border-gray-200 hover:border-gray-300'
                    }`}
                    onClick={() => handleGeofenceSelect(geofence)}
                  >
                    <div className="flex items-start justify-between mb-2">
                      <h4 className="font-medium text-gray-900 text-sm">{geofence.name}</h4>
                      <div className="flex items-center space-x-1">
                        <Badge className={getStatusColor(geofence.status)} size="sm">
                          {geofence.status}
                        </Badge>
                        <Badge className={getTypeColor(geofence.type)} size="sm">
                          {geofence.type}
                        </Badge>
                      </div>
                    </div>
                    <p className="text-xs text-gray-600 mb-2">{geofence.description}</p>
                    <div className="grid grid-cols-2 gap-2 text-xs">
                      <div>
                        <span className="text-gray-500">Area:</span>
                        <span className="font-medium ml-1">
                          {geofence.metadata.area ? `${geofence.metadata.area.toLocaleString()} m²` : 'N/A'}
                        </span>
                      </div>
                      <div>
                        <span className="text-gray-500">Vehicles:</span>
                        <span className="font-medium ml-1">{geofence.usage.currentVehicles}</span>
                      </div>
                      <div>
                        <span className="text-gray-500">Rules:</span>
                        <span className="font-medium ml-1">{geofence.rules.length}</span>
                      </div>
                      <div>
                        <span className="text-gray-500">Violations:</span>
                        <span className="font-medium ml-1">{geofence.usage.violationCount}</span>
                      </div>
                    </div>
                  </div>
                ))
              )}
            </div>
          </Card>
        </div>

        {/* Center Panel - Map View */}
        <div className="lg:col-span-1">
          <Card className="p-4 h-[600px]">
            <div className="flex items-center justify-between mb-4">
              <h3 className="font-medium text-gray-900">Map View</h3>
              <div className="flex items-center space-x-2">
                <Button variant="outline" size="sm">
                  <Compass className="w-4 h-4" />
                </Button>
                <Button variant="outline" size="sm">
                  <Target className="w-4 h-4" />
                </Button>
              </div>
            </div>
            <div className="h-full bg-gray-100 rounded flex items-center justify-center">
              <div className="text-center">
                <Map className="w-16 h-16 text-gray-400 mx-auto mb-4" />
                <p className="text-gray-600 mb-2">Interactive Map</p>
                <p className="text-sm text-gray-500">
                  Routes, geofences, and live vehicle positions would be displayed here
                </p>
                <div className="mt-4 space-y-2">
                  {mapLayers.filter(l => l.visible).map(layer => (
                    <div key={layer.id} className="flex items-center justify-center space-x-2">
                      <div 
                        className="w-2 h-2 rounded-full" 
                        style={{ backgroundColor: layer.color || '#6B7280' }}
                      />
                      <span className="text-xs text-gray-600">{layer.name} Layer Active</span>
                    </div>
                  ))}
                </div>
              </div>
            </div>
          </Card>
        </div>

        {/* Right Panel - Details */}
        <div className="lg:col-span-1">
          {selectedRoute ? (
            <Card className="p-4">
              <div className="flex items-center justify-between mb-4">
                <h3 className="font-medium text-gray-900">Route Details</h3>
                {!readOnly && (
                  <div className="flex items-center space-x-2">
                    <Button variant="outline" size="sm">
                      <Edit3 className="w-4 h-4" />
                    </Button>
                    <Button 
                      variant="outline" 
                      size="sm"
                      onClick={() => handleValidateRoute(selectedRoute.id)}
                      disabled={validationInProgress}
                    >
                      {validationInProgress ? (
                        <Activity className="w-4 h-4 animate-spin" />
                      ) : (
                        <CheckCircle className="w-4 h-4" />
                      )}
                    </Button>
                  </div>
                )}
              </div>

              <div className="space-y-4">
                <div>
                  <h4 className="font-medium text-gray-900 mb-2">{selectedRoute.name}</h4>
                  <p className="text-sm text-gray-600 mb-3">{selectedRoute.description}</p>
                  
                  <div className="flex flex-wrap gap-2 mb-3">
                    <Badge className={getStatusColor(selectedRoute.status)}>
                      {selectedRoute.status}
                    </Badge>
                    <Badge className={getPriorityColor(selectedRoute.priority)}>
                      {selectedRoute.priority}
                    </Badge>
                    <Badge className={getTypeColor(selectedRoute.type)}>
                      {selectedRoute.type}
                    </Badge>
                  </div>
                </div>

                <div className="grid grid-cols-2 gap-4 text-sm">
                  <div>
                    <span className="text-gray-600">Distance:</span>
                    <div className="font-medium">{formatDistance(selectedRoute.metadata.distance)}</div>
                  </div>
                  <div>
                    <span className="text-gray-600">Duration:</span>
                    <div className="font-medium">{formatDuration(selectedRoute.metadata.estimatedDuration)}</div>
                  </div>
                  <div>
                    <span className="text-gray-600">Waypoints:</span>
                    <div className="font-medium">{selectedRoute.waypoints.length}</div>
                  </div>
                  <div>
                    <span className="text-gray-600">Geofences:</span>
                    <div className="font-medium">{selectedRoute.geofences.length}</div>
                  </div>
                </div>

                <div>
                  <h5 className="font-medium text-gray-900 mb-2">Usage Statistics</h5>
                  <div className="grid grid-cols-2 gap-4 text-sm">
                    <div>
                      <span className="text-gray-600">Total Trips:</span>
                      <div className="font-medium">{selectedRoute.usage.totalTrips.toLocaleString()}</div>
                    </div>
                    <div>
                      <span className="text-gray-600">Active Trips:</span>
                      <div className="font-medium">{selectedRoute.usage.activeTrips}</div>
                    </div>
                    <div>
                      <span className="text-gray-600">Success Rate:</span>
                      <div className="font-medium">{selectedRoute.usage.successRate}%</div>
                    </div>
                    <div>
                      <span className="text-gray-600">Rating:</span>
                      <div className="font-medium">{selectedRoute.usage.averageRating}/5</div>
                    </div>
                  </div>
                </div>

                <div>
                  <h5 className="font-medium text-gray-900 mb-2">Validation</h5>
                  <div className="flex items-center space-x-2 mb-2">
                    <Badge className={
                      selectedRoute.validation.status === 'valid' ? 'bg-green-100 text-green-800' :
                      selectedRoute.validation.status === 'invalid' ? 'bg-red-100 text-red-800' :
                      'bg-yellow-100 text-yellow-800'
                    }>
                      {selectedRoute.validation.status}
                    </Badge>
                    <span className="text-sm text-gray-600">Score: {selectedRoute.validation.score}/100</span>
                  </div>
                  {selectedRoute.validation.issues.length > 0 && (
                    <div className="space-y-2">
                      <span className="text-sm font-medium text-gray-900">Issues:</span>
                      {selectedRoute.validation.issues.map((issue, index) => (
                        <div key={index} className="p-2 bg-yellow-50 border border-yellow-200 rounded text-sm">
                          <div className="flex items-center space-x-2">
                            <AlertTriangle className="w-3 h-3 text-yellow-600" />
                            <span className="font-medium text-yellow-800">{issue.code}</span>
                          </div>
                          <p className="text-yellow-700 mt-1">{issue.message}</p>
                          {issue.suggestion && (
                            <p className="text-yellow-600 mt-1 text-xs">Suggestion: {issue.suggestion}</p>
                          )}
                        </div>
                      ))}
                    </div>
                  )}
                </div>

                {!readOnly && selectedRoute.status !== 'active' && (
                  <Button 
                    className="w-full"
                    onClick={() => handlePublishRoute(selectedRoute)}
                  >
                    <Play className="w-4 h-4 mr-2" />
                    Publish Route
                  </Button>
                )}
              </div>
            </Card>
          ) : selectedGeofence ? (
            <Card className="p-4">
              <div className="flex items-center justify-between mb-4">
                <h3 className="font-medium text-gray-900">Geofence Details</h3>
                {!readOnly && (
                  <div className="flex items-center space-x-2">
                    <Button variant="outline" size="sm">
                      <Edit3 className="w-4 h-4" />
                    </Button>
                  </div>
                )}
              </div>

              <div className="space-y-4">
                <div>
                  <h4 className="font-medium text-gray-900 mb-2">{selectedGeofence.name}</h4>
                  <p className="text-sm text-gray-600 mb-3">{selectedGeofence.description}</p>
                  
                  <div className="flex flex-wrap gap-2 mb-3">
                    <Badge className={getStatusColor(selectedGeofence.status)}>
                      {selectedGeofence.status}
                    </Badge>
                    <Badge className={getTypeColor(selectedGeofence.type)}>
                      {selectedGeofence.type}
                    </Badge>
                  </div>
                </div>

                <div className="grid grid-cols-2 gap-4 text-sm">
                  {selectedGeofence.metadata.area && (
                    <div>
                      <span className="text-gray-600">Area:</span>
                      <div className="font-medium">{selectedGeofence.metadata.area.toLocaleString()} m²</div>
                    </div>
                  )}
                  {selectedGeofence.metadata.perimeter && (
                    <div>
                      <span className="text-gray-600">Perimeter:</span>
                      <div className="font-medium">{selectedGeofence.metadata.perimeter} m</div>
                    </div>
                  )}
                  <div>
                    <span className="text-gray-600">Rules:</span>
                    <div className="font-medium">{selectedGeofence.rules.length}</div>
                  </div>
                  <div>
                    <span className="text-gray-600">Triggers:</span>
                    <div className="font-medium">{selectedGeofence.triggers.length}</div>
                  </div>
                </div>

                <div>
                  <h5 className="font-medium text-gray-900 mb-2">Current Activity</h5>
                  <div className="grid grid-cols-2 gap-4 text-sm">
                    <div>
                      <span className="text-gray-600">Vehicles Inside:</span>
                      <div className="font-medium">{selectedGeofence.usage.currentVehicles}</div>
                    </div>
                    <div>
                      <span className="text-gray-600">Total Triggers:</span>
                      <div className="font-medium">{selectedGeofence.usage.totalTriggers.toLocaleString()}</div>
                    </div>
                    <div>
                      <span className="text-gray-600">Violations:</span>
                      <div className="font-medium">{selectedGeofence.usage.violationCount}</div>
                    </div>
                    <div>
                      <span className="text-gray-600">Last Violation:</span>
                      <div className="font-medium text-xs">
                        {selectedGeofence.usage.lastViolation 
                          ? selectedGeofence.usage.lastViolation.toLocaleDateString()
                          : 'None'
                        }
                      </div>
                    </div>
                  </div>
                </div>

                <div>
                  <h5 className="font-medium text-gray-900 mb-2">Rules</h5>
                  <div className="space-y-2">
                    {selectedGeofence.rules.map(rule => (
                      <div key={rule.id} className="p-2 bg-gray-50 rounded text-sm">
                        <div className="flex items-center justify-between">
                          <span className="font-medium">{rule.name}</span>
                          <Badge className={rule.enabled ? 'bg-green-100 text-green-800' : 'bg-gray-100 text-gray-800'} size="sm">
                            {rule.enabled ? 'Enabled' : 'Disabled'}
                          </Badge>
                        </div>
                        <p className="text-gray-600 mt-1">
                          {rule.condition} {rule.operator} {rule.value} → {rule.action.type}
                        </p>
                      </div>
                    ))}
                  </div>
                </div>

                {selectedGeofence.usage.recentTriggers.length > 0 && (
                  <div>
                    <h5 className="font-medium text-gray-900 mb-2">Recent Triggers</h5>
                    <div className="space-y-2">
                      {selectedGeofence.usage.recentTriggers.slice(0, 3).map(trigger => (
                        <div key={trigger.id} className="p-2 bg-blue-50 border border-blue-200 rounded text-sm">
                          <div className="flex items-center justify-between">
                            <span className="font-medium">{trigger.vehicleId}</span>
                            <span className="text-xs text-gray-600">{trigger.timestamp.toLocaleTimeString()}</span>
                          </div>
                          <p className="text-blue-700 mt-1">{trigger.event}</p>
                        </div>
                      ))}
                    </div>
                  </div>
                )}

                {!readOnly && selectedGeofence.status !== 'active' && (
                  <Button 
                    className="w-full"
                    onClick={() => handlePublishGeofence(selectedGeofence)}
                  >
                    <Play className="w-4 h-4 mr-2" />
                    Publish Geofence
                  </Button>
                )}
              </div>
            </Card>
          ) : (
            <Card className="p-4 h-[600px] flex items-center justify-center">
              <div className="text-center">
                <MapPin className="w-16 h-16 text-gray-400 mx-auto mb-4" />
                <h3 className="text-lg font-medium text-gray-900 mb-2">No Selection</h3>
                <p className="text-gray-600">
                  Select a {activeView === 'routes' ? 'route' : 'geofence'} from the list to view details
                </p>
              </div>
            </Card>
          )}
        </div>
      </div>
    </div>
  )
}

export default RoutesGeofencesEditor
