import { z } from 'zod'
import { coordinatesSchema, locationSchema } from './vehicle'

// Trip status schema
export const tripStatusSchema = z.enum([
  'scheduled',
  'assigned',
  'en_route_pickup',
  'arrived_pickup',
  'pickup_complete',
  'en_route_destination',
  'arrived_destination',
  'completed',
  'cancelled',
  'failed',
  'paused',
])

// Mission type schema
export const missionTypeSchema = z.enum([
  'passenger_transport',
  'cargo_delivery',
  'shuttle_service',
  'emergency_response',
  'maintenance_patrol',
  'security_patrol',
  'cleaning_service',
  'inspection_round',
  'custom',
])

// Trip classification schema (P0-P3 regulatory/safety levels)
export const tripClassificationSchema = z.enum([
  'P0_demo',                    // P0: Demo - Non-operational demonstration runs
  'P1_operation_run',           // P1: Operation Run - Standard supervised operations
  'P1_driverless_operation',    // P1: Driverless Operation - L4/L5 autonomous
  'P1_chery_run',              // P1: Chery Run - Specific test scenarios
  'P2_release_run',            // P2: Release Run - Pre-release validation
  'P3_engineering_run',        // P3: Engineering Run - Development and testing
])

// Onboard stage schema
export const onboardStageSchema = z.enum([
  'with_co_drive',             // With co-driver (safety operator present)
  'driverless_operation',      // Fully driverless operation
  'remote_monitored',          // Remote operator monitoring
  'safety_chase',              // Following car with safety driver
])

// Priority schema
export const tripPrioritySchema = z.enum([
  'low',
  'normal',
  'high',
  'urgent',
  'emergency',
])

// Route schema
export const waypointSchema = z.object({
  id: z.string().uuid().optional(),
  location: locationSchema,
  type: z.enum(['pickup', 'dropoff', 'waypoint', 'charging', 'maintenance']),
  arrivalTime: z.string().datetime().optional(),
  departureTime: z.string().datetime().optional(),
  duration: z.number().nonnegative().optional(), // seconds
  instructions: z.string().optional(),
  metadata: z.record(z.unknown()).default({}),
})

export const routeSchema = z.object({
  routeId: z.string().uuid().optional(),
  coordinates: z.array(z.tuple([z.number(), z.number()])), // [lng, lat] pairs
  waypoints: z.array(waypointSchema),
  distance: z.number().nonnegative(), // meters
  duration: z.number().nonnegative(), // seconds
  trafficDuration: z.number().nonnegative().optional(), // seconds with traffic
  geometry: z.string().optional(), // encoded polyline
  instructions: z.array(z.object({
    text: z.string(),
    distance: z.number().nonnegative(),
    duration: z.number().nonnegative(),
    location: coordinatesSchema,
  })).optional(),
  alternatives: z.array(z.object({
    coordinates: z.array(z.tuple([z.number(), z.number()])),
    distance: z.number().nonnegative(),
    duration: z.number().nonnegative(),
    reason: z.string(),
  })).default([]),
})

// Passenger schema
export const passengerSchema = z.object({
  passengerId: z.string().uuid().optional(),
  name: z.string().min(1).max(100).optional(), // May be redacted
  phone: z.string().optional(), // May be redacted
  email: z.string().email().optional(), // May be redacted
  accessibilityNeeds: z.array(z.enum([
    'wheelchair',
    'mobility_aid',
    'visual_impairment',
    'hearing_impairment',
    'service_animal',
    'child_seat',
    'booster_seat',
    'other',
  ])).default([]),
  preferences: z.object({
    temperature: z.number().min(16).max(30).optional(), // Celsius
    music: z.boolean().default(false),
    conversation: z.boolean().default(true),
    route_preference: z.enum(['fastest', 'shortest', 'scenic', 'eco']).default('fastest'),
  }).optional(),
  metadata: z.record(z.unknown()).default({}),
})

// Cargo schema
export const cargoSchema = z.object({
  cargoId: z.string().uuid().optional(),
  description: z.string().min(1).max(500),
  weight: z.number().nonnegative(), // kg
  volume: z.number().nonnegative().optional(), // cubic meters
  dimensions: z.object({
    length: z.number().positive(),
    width: z.number().positive(),
    height: z.number().positive(),
  }).optional(),
  type: z.enum([
    'general',
    'fragile',
    'hazardous',
    'perishable',
    'valuable',
    'oversized',
    'temperature_controlled',
  ]).default('general'),
  specialHandling: z.array(z.string()).default([]),
  temperatureRequirements: z.object({
    min: z.number(),
    max: z.number(),
  }).optional(),
  metadata: z.record(z.unknown()).default({}),
})

// Trip parameters schema
export const tripParametersSchema = z.object({
  maxSpeed: z.number().positive().optional(),
  comfortLevel: z.enum(['eco', 'comfort', 'sport']).default('comfort'),
  routePreference: z.enum(['fastest', 'shortest', 'scenic', 'eco']).default('fastest'),
  avoidTolls: z.boolean().default(false),
  avoidHighways: z.boolean().default(false),
  allowUTurns: z.boolean().default(true),
  maxDetourDistance: z.number().nonnegative().optional(), // meters
  maxDetourTime: z.number().nonnegative().optional(), // seconds
  weatherRestrictions: z.array(z.enum([
    'heavy_rain',
    'snow',
    'ice',
    'fog',
    'high_winds',
    'extreme_heat',
    'extreme_cold',
  ])).default([]),
  timeRestrictions: z.object({
    earliestStart: z.string().optional(), // HH:MM format
    latestEnd: z.string().optional(), // HH:MM format
    allowNighttime: z.boolean().default(true),
  }).optional(),
})

// Main trip schema
export const tripSchema = z.object({
  tripId: z.string().uuid(),
  vehicleId: z.string().uuid(),
  fleetId: z.string().uuid(),
  
  // Trip details
  status: tripStatusSchema,
  missionType: missionTypeSchema,
  priority: tripPrioritySchema.default('normal'),
  classification: tripClassificationSchema.default('P1_operation_run'), // Regulatory classification
  onboardStage: onboardStageSchema.default('with_co_drive'), // Safety operator configuration
  
  // Route and locations
  origin: locationSchema,
  destination: locationSchema,
  route: routeSchema.optional(),
  
  // Timing
  scheduledStart: z.string().datetime(),
  scheduledEnd: z.string().datetime(),
  actualStart: z.string().datetime().optional(),
  actualEnd: z.string().datetime().optional(),
  estimatedArrival: z.string().datetime().optional(),
  
  // Progress
  progress: z.number().min(0).max(100).default(0),
  currentWaypointIndex: z.number().int().nonnegative().default(0),
  distanceRemaining: z.number().nonnegative().optional(), // meters
  timeRemaining: z.number().nonnegative().optional(), // seconds
  
  // Passengers and cargo
  passengers: z.array(passengerSchema).default([]),
  cargo: z.array(cargoSchema).default([]),
  maxPassengers: z.number().int().nonnegative().default(0),
  maxCargo: z.number().nonnegative().default(0), // kg
  
  // Configuration
  parameters: tripParametersSchema.default({}),
  
  // Metadata
  createdBy: z.string().uuid(),
  assignedBy: z.string().uuid().optional(),
  createdAt: z.string().datetime(),
  updatedAt: z.string().datetime(),
  
  // External references
  externalId: z.string().optional(), // Third-party booking ID
  parentTripId: z.string().uuid().optional(), // For multi-leg trips
  childTripIds: z.array(z.string().uuid()).default([]),
  jiraTicket: z.string().optional(), // JIRA ticket reference
  
  // Multi-operator configuration
  driverEmail: z.string().email().optional(),
  coDriverEmail: z.string().email().optional(),
  remoteOperatorEmail: z.string().email().optional(),
  followingCarDriverEmail: z.string().email().optional(),
  issueOwnerEmail: z.string().email().optional(),
  
  // Advanced configuration
  tripObsoleteDuration: z.number().positive().default(30), // Minutes until trip is considered obsolete
  onboardSuiteUrl: z.string().url().optional(), // URL for onboard test suite
  roadGraphVersion: z.string().optional(), // yyyy-mm-dd-hhmmss format
  
  // Billing and cost
  estimatedCost: z.number().nonnegative().optional(),
  actualCost: z.number().nonnegative().optional(),
  currency: z.string().length(3).default('USD'),
  
  // Quality metrics
  rating: z.number().min(1).max(5).optional(),
  feedback: z.string().max(1000).optional(),
  
  // Tags and metadata
  tags: z.array(z.string()).default([]),
  metadata: z.record(z.unknown()).default({}),
})

export const tripCreateSchema = tripSchema.omit({
  tripId: true,
  createdAt: true,
  updatedAt: true,
  progress: true,
  currentWaypointIndex: true,
  actualStart: true,
  actualEnd: true,
}).extend({
  // Override defaults for creation
  status: tripStatusSchema.default('scheduled'),
})

export const tripUpdateSchema = tripSchema.partial().omit({
  tripId: true,
  createdAt: true,
  createdBy: true,
})

export const tripFilterSchema = z.object({
  search: z.string().optional(),
  status: tripStatusSchema.optional(),
  missionType: missionTypeSchema.optional(),
  priority: tripPrioritySchema.optional(),
  vehicleId: z.string().uuid().optional(),
  fleetId: z.string().uuid().optional(),
  createdBy: z.string().uuid().optional(),
  assignedBy: z.string().uuid().optional(),
  
  // Time filters
  scheduledAfter: z.string().datetime().optional(),
  scheduledBefore: z.string().datetime().optional(),
  createdAfter: z.string().datetime().optional(),
  createdBefore: z.string().datetime().optional(),
  
  // Location filters
  originArea: z.object({
    center: coordinatesSchema,
    radius: z.number().positive(), // meters
  }).optional(),
  destinationArea: z.object({
    center: coordinatesSchema,
    radius: z.number().positive(), // meters
  }).optional(),
  
  // Progress filters
  progressMin: z.number().min(0).max(100).optional(),
  progressMax: z.number().min(0).max(100).optional(),
  
  // Passenger/cargo filters
  hasPassengers: z.boolean().optional(),
  hasCargo: z.boolean().optional(),
  passengerCount: z.object({
    min: z.number().int().nonnegative().optional(),
    max: z.number().int().nonnegative().optional(),
  }).optional(),
  
  // Tags and metadata
  tags: z.array(z.string()).optional(),
  externalId: z.string().optional(),
  
  // Pagination and sorting
  limit: z.number().int().positive().max(1000).default(50),
  offset: z.number().int().nonnegative().default(0),
  sortBy: z.enum([
    'scheduledStart',
    'createdAt',
    'priority',
    'status',
    'progress',
    'estimatedArrival',
  ]).default('scheduledStart'),
  sortOrder: z.enum(['asc', 'desc']).default('desc'),
  
  // Include related data
  includeRoute: z.boolean().default(false),
  includePassengers: z.boolean().default(false),
  includeCargo: z.boolean().default(false),
  includeVehicle: z.boolean().default(false),
})

// Trip action schemas
export const tripActionSchema = z.object({
  actionId: z.string().uuid().optional(),
  tripId: z.string().uuid(),
  type: z.enum([
    'start',
    'pause',
    'resume',
    'complete',
    'cancel',
    'reassign',
    'update_route',
    'add_waypoint',
    'remove_waypoint',
    'update_priority',
    'add_passenger',
    'remove_passenger',
    'add_cargo',
    'remove_cargo',
  ]),
  payload: z.record(z.unknown()).default({}),
  executedBy: z.string().uuid(),
  executedAt: z.string().datetime(),
  reason: z.string().min(1).max(500).optional(),
})

// Export types
export type Trip = z.infer<typeof tripSchema>
export type TripCreate = z.infer<typeof tripCreateSchema>
export type TripUpdate = z.infer<typeof tripUpdateSchema>
export type TripFilter = z.infer<typeof tripFilterSchema>
export type TripAction = z.infer<typeof tripActionSchema>
export type TripStatus = z.infer<typeof tripStatusSchema>
export type MissionType = z.infer<typeof missionTypeSchema>
export type TripPriority = z.infer<typeof tripPrioritySchema>
export type TripClassification = z.infer<typeof tripClassificationSchema>
export type OnboardStage = z.infer<typeof onboardStageSchema>
export type Route = z.infer<typeof routeSchema>
export type Waypoint = z.infer<typeof waypointSchema>
export type Passenger = z.infer<typeof passengerSchema>
export type Cargo = z.infer<typeof cargoSchema>
export type TripParameters = z.infer<typeof tripParametersSchema>
