import { z } from 'zod'

// Base schemas
export const coordinatesSchema = z.object({
  lat: z.number().min(-90).max(90),
  lng: z.number().min(-180).max(180),
})

export const locationSchema = coordinatesSchema.extend({
  address: z.string().optional(),
  accuracy: z.number().positive().optional(),
})

// Vehicle schemas
export const autonomyLevelSchema = z.enum(['L0', 'L1', 'L2', 'L3', 'L4', 'L5'])

export const vehicleStatusSchema = z.enum([
  'driving_av',
  'waiting',
  'remote_assist',
  'fallback',
  'safed',
  'offline',
  'maintenance',
  'charging',
])

export const autonomyStatusSchema = z.enum([
  'active',
  'degraded',
  'fallback',
  'manual',
  'offline',
])

export const sensorConfigSchema = z.object({
  lidarRange: z.number().positive().optional(),
  cameraCount: z.number().int().positive().optional(),
  radarCount: z.number().int().positive().optional(),
  imuCount: z.number().int().positive().optional(),
  gnssCount: z.number().int().positive().optional(),
})

export const vehicleCapabilitiesSchema = z.object({
  maxSpeed: z.number().positive(),
  passengerCapacity: z.number().int().nonnegative(),
  cargoCapacity: z.number().nonnegative(), // kg
  autonomyCapable: z.boolean().default(true),
  weatherLimitations: z.array(z.string()).optional(),
  roadTypeLimitations: z.array(z.string()).optional(),
})

export const vehicleProfileSchema = z.object({
  type: z.enum(['passenger', 'cargo', 'shuttle', 'emergency', 'maintenance']),
  category: z.enum(['sedan', 'suv', 'van', 'truck', 'bus', 'specialized']),
  fuelType: z.enum(['electric', 'hybrid', 'gasoline', 'diesel', 'hydrogen']),
  dimensions: z.object({
    length: z.number().positive(),
    width: z.number().positive(),
    height: z.number().positive(),
    wheelbase: z.number().positive().optional(),
  }).optional(),
  weight: z.object({
    curb: z.number().positive(),
    gross: z.number().positive(),
  }).optional(),
})

export const vehicleSchema = z.object({
  vehicleId: z.string().uuid(),
  assetTag: z.string().min(1).max(50),
  fleetId: z.string().uuid(),
  manufacturer: z.string().min(1).max(100),
  model: z.string().min(1).max(100),
  serialNumber: z.string().min(1).max(100),
  year: z.number().int().min(2000).max(new Date().getFullYear() + 2).optional(),
  
  // Operational status
  operationalStatus: vehicleStatusSchema,
  autonomyLevel: autonomyLevelSchema,
  autonomyStatus: autonomyStatusSchema,
  autonomyConfidence: z.number().min(0).max(1).optional(),
  oddCompliance: z.boolean().default(true),
  
  // Location and movement
  currentLocation: locationSchema.optional(),
  heading: z.number().min(0).max(360).optional(),
  speed: z.number().nonnegative().optional(),
  
  // Power and health
  batteryLevel: z.number().min(0).max(100).optional(),
  fuelLevel: z.number().min(0).max(100).optional(),
  odometer: z.number().nonnegative().optional(),
  engineHours: z.number().nonnegative().optional(),
  
  // Timestamps
  lastSeen: z.string().datetime(),
  createdAt: z.string().datetime(),
  updatedAt: z.string().datetime(),
  
  // Relationships
  currentTripId: z.string().uuid().optional(),
  assignedDepotId: z.string().uuid().optional(),
  
  // Configuration
  vehicleProfile: vehicleProfileSchema.optional(),
  capabilities: vehicleCapabilitiesSchema.optional(),
  sensorConfig: sensorConfigSchema.optional(),
  
  // Metadata
  tags: z.array(z.string()).default([]),
  metadata: z.record(z.unknown()).default({}),
})

export const vehicleCreateSchema = vehicleSchema.omit({
  vehicleId: true,
  createdAt: true,
  updatedAt: true,
  lastSeen: true,
}).extend({
  // Override required fields for creation
  operationalStatus: vehicleStatusSchema.default('offline'),
  autonomyLevel: autonomyLevelSchema.default('L0'),
  autonomyStatus: autonomyStatusSchema.default('offline'),
})

export const vehicleUpdateSchema = vehicleSchema.partial().omit({
  vehicleId: true,
  createdAt: true,
})

export const vehicleFilterSchema = z.object({
  search: z.string().optional(),
  status: vehicleStatusSchema.optional(),
  autonomyLevel: autonomyLevelSchema.optional(),
  fleetId: z.string().uuid().optional(),
  manufacturer: z.string().optional(),
  model: z.string().optional(),
  tags: z.array(z.string()).optional(),
  location: z.object({
    center: coordinatesSchema,
    radius: z.number().positive(), // meters
  }).optional(),
  batteryLevel: z.object({
    min: z.number().min(0).max(100).optional(),
    max: z.number().min(0).max(100).optional(),
  }).optional(),
  lastSeenAfter: z.string().datetime().optional(),
  lastSeenBefore: z.string().datetime().optional(),
  includeInactive: z.boolean().default(false),
  includeOffline: z.boolean().default(true),
  limit: z.number().int().positive().max(1000).default(50),
  offset: z.number().int().nonnegative().default(0),
  sortBy: z.enum(['assetTag', 'lastSeen', 'batteryLevel', 'createdAt']).default('assetTag'),
  sortOrder: z.enum(['asc', 'desc']).default('asc'),
})

// Command schemas
export const vehicleCommandSchema = z.object({
  commandId: z.string().uuid().optional(),
  vehicleId: z.string().uuid(),
  type: z.enum([
    'start',
    'stop',
    'pause',
    'resume',
    'emergency_stop',
    'safe_stop',
    'maintenance_mode',
    'charging_mode',
    'return_to_depot',
    'update_route',
    'cancel_trip',
  ]),
  payload: z.record(z.unknown()).default({}),
  priority: z.enum(['low', 'normal', 'high', 'critical']).default('normal'),
  executedBy: z.string().uuid(),
  executedAt: z.string().datetime(),
  reason: z.string().min(1).max(500).optional(),
  reasonCategory: z.enum([
    'safety_concern',
    'technical_issue',
    'passenger_request',
    'traffic_incident',
    'weather_conditions',
    'regulatory_compliance',
    'operational_efficiency',
    'maintenance_required',
    'other',
  ]).optional(),
})

// Export types
export type Vehicle = z.infer<typeof vehicleSchema>
export type VehicleCreate = z.infer<typeof vehicleCreateSchema>
export type VehicleUpdate = z.infer<typeof vehicleUpdateSchema>
export type VehicleFilter = z.infer<typeof vehicleFilterSchema>
export type VehicleCommand = z.infer<typeof vehicleCommandSchema>
export type VehicleStatus = z.infer<typeof vehicleStatusSchema>
export type AutonomyLevel = z.infer<typeof autonomyLevelSchema>
export type AutonomyStatus = z.infer<typeof autonomyStatusSchema>
export type Location = z.infer<typeof locationSchema>
export type Coordinates = z.infer<typeof coordinatesSchema>
