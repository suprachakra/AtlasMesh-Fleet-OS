import { z } from 'zod'

// Worklog type schema
export const worklogTypeSchema = z.enum([
  'maintenance',
  'repair',
  'inspection',
  'upgrade',
  'incident',
  'testing',
  'calibration',
  'cleaning',
  'documentation',
  'other',
])

// Worklog status schema
export const worklogStatusSchema = z.enum([
  'scheduled',
  'in_progress',
  'completed',
  'cancelled',
  'blocked',
])

// Worklog priority schema
export const worklogPrioritySchema = z.enum([
  'low',
  'medium',
  'high',
  'critical',
])

// Part schema
export const partSchema = z.object({
  partId: z.string().uuid().optional(),
  partNumber: z.string().min(1).max(100),
  description: z.string().min(1).max(500),
  quantity: z.number().int().positive(),
  unitCost: z.number().nonnegative(),
  totalCost: z.number().nonnegative(),
  supplier: z.string().optional(),
  serialNumber: z.string().optional(),
})

// Attachment schema
export const attachmentSchema = z.object({
  attachmentId: z.string().uuid().optional(),
  name: z.string().min(1).max(255),
  url: z.string().url(),
  type: z.string(), // MIME type
  size: z.number().nonnegative(), // bytes
  uploadedBy: z.string().uuid(),
  uploadedAt: z.string().datetime(),
})

// Main worklog schema
export const worklogSchema = z.object({
  worklogId: z.string().uuid(),
  vehicleId: z.string().uuid(),
  
  // Worklog details
  type: worklogTypeSchema,
  title: z.string().min(1).max(200),
  description: z.string().min(1).max(5000), // Rich text HTML content
  status: worklogStatusSchema,
  priority: worklogPrioritySchema.default('medium'),
  
  // Timing
  startTime: z.string().datetime(),
  endTime: z.string().datetime().optional(),
  estimatedDuration: z.number().positive().optional(), // minutes
  actualDuration: z.number().positive().optional(), // minutes
  
  // Personnel
  technician: z.string().min(1).max(100), // Primary technician
  owner: z.string().email(), // Work owner email
  approver: z.string().email().optional(), // Approver email (for sign-off)
  assignedTo: z.array(z.string().email()).default([]), // Additional assigned personnel
  
  // External tracking
  jiraTicket: z.string().optional(), // JIRA ticket reference (e.g., "FLEET-1234")
  ecoNumber: z.string().optional(), // Engineering Change Order number
  serviceOrder: z.string().optional(), // Service order number
  purchaseOrder: z.string().optional(), // PO number for parts
  
  // Parts and costs
  parts: z.array(partSchema).default([]),
  laborCost: z.number().nonnegative().default(0),
  partsCost: z.number().nonnegative().default(0),
  totalCost: z.number().nonnegative().default(0),
  currency: z.string().length(3).default('USD'),
  
  // Attachments
  attachments: z.array(attachmentSchema).default([]),
  
  // Safety and compliance
  safetyChecksCompleted: z.boolean().default(false),
  qualityCheckCompleted: z.boolean().default(false),
  certificationRequired: z.boolean().default(false),
  certificationNumber: z.string().optional(),
  
  // Timestamps
  createdBy: z.string().uuid(),
  createdAt: z.string().datetime(),
  updatedAt: z.string().datetime(),
  approvedAt: z.string().datetime().optional(),
  approvedBy: z.string().uuid().optional(),
  
  // Metadata
  tags: z.array(z.string()).default([]),
  notes: z.string().max(1000).optional(), // Internal notes (not shown to customer)
  metadata: z.record(z.unknown()).default({}),
})

export const worklogCreateSchema = worklogSchema.omit({
  worklogId: true,
  createdAt: true,
  updatedAt: true,
  actualDuration: true,
  approvedAt: true,
  approvedBy: true,
}).extend({
  status: worklogStatusSchema.default('scheduled'),
})

export const worklogUpdateSchema = worklogSchema.partial().omit({
  worklogId: true,
  createdAt: true,
  createdBy: true,
})

export const worklogFilterSchema = z.object({
  search: z.string().optional(),
  type: worklogTypeSchema.optional(),
  status: worklogStatusSchema.optional(),
  priority: worklogPrioritySchema.optional(),
  vehicleId: z.string().uuid().optional(),
  technician: z.string().optional(),
  owner: z.string().email().optional(),
  jiraTicket: z.string().optional(),
  ecoNumber: z.string().optional(),
  
  // Time filters
  startedAfter: z.string().datetime().optional(),
  startedBefore: z.string().datetime().optional(),
  completedAfter: z.string().datetime().optional(),
  completedBefore: z.string().datetime().optional(),
  
  // Cost filters
  minCost: z.number().nonnegative().optional(),
  maxCost: z.number().nonnegative().optional(),
  
  // Compliance filters
  certificationRequired: z.boolean().optional(),
  safetyChecksCompleted: z.boolean().optional(),
  
  // Pagination and sorting
  limit: z.number().int().positive().max(1000).default(50),
  offset: z.number().int().nonnegative().default(0),
  sortBy: z.enum([
    'startTime',
    'createdAt',
    'priority',
    'status',
    'totalCost',
  ]).default('startTime'),
  sortOrder: z.enum(['asc', 'desc']).default('desc'),
})

// Export types
export type Worklog = z.infer<typeof worklogSchema>
export type WorklogCreate = z.infer<typeof worklogCreateSchema>
export type WorklogUpdate = z.infer<typeof worklogUpdateSchema>
export type WorklogFilter = z.infer<typeof worklogFilterSchema>
export type WorklogType = z.infer<typeof worklogTypeSchema>
export type WorklogStatus = z.infer<typeof worklogStatusSchema>
export type WorklogPriority = z.infer<typeof worklogPrioritySchema>
export type Part = z.infer<typeof partSchema>
export type Attachment = z.infer<typeof attachmentSchema>

