import { setupServer } from 'msw/node'
import { rest } from 'msw'
import { mockVehicles, mockTrips, mockAlerts, mockIncidents } from './data'

// Mock API handlers
export const handlers = [
  // Fleet API
  rest.get('/api/v1/fleet/vehicles', (req, res, ctx) => {
    const search = req.url.searchParams.get('search')
    const status = req.url.searchParams.get('status')
    
    let filteredVehicles = mockVehicles
    
    if (search) {
      filteredVehicles = filteredVehicles.filter(v => 
        v.assetTag.toLowerCase().includes(search.toLowerCase()) ||
        v.model.toLowerCase().includes(search.toLowerCase())
      )
    }
    
    if (status && status !== 'all') {
      filteredVehicles = filteredVehicles.filter(v => v.operationalStatus === status)
    }
    
    return res(
      ctx.status(200),
      ctx.json({
        data: filteredVehicles,
        total: filteredVehicles.length,
        page: 1,
        pageSize: 50,
      })
    )
  }),

  rest.get('/api/v1/fleet/vehicles/:vehicleId', (req, res, ctx) => {
    const { vehicleId } = req.params
    const vehicle = mockVehicles.find(v => v.vehicleId === vehicleId)
    
    if (!vehicle) {
      return res(ctx.status(404), ctx.json({ error: 'Vehicle not found' }))
    }
    
    return res(ctx.status(200), ctx.json({ data: vehicle }))
  }),

  // Trips API
  rest.get('/api/v1/trips', (req, res, ctx) => {
    const status = req.url.searchParams.get('status')
    
    let filteredTrips = mockTrips
    
    if (status && status !== 'all') {
      filteredTrips = filteredTrips.filter(t => t.status === status)
    }
    
    return res(
      ctx.status(200),
      ctx.json({
        data: filteredTrips,
        total: filteredTrips.length,
        page: 1,
        pageSize: 50,
      })
    )
  }),

  rest.get('/api/v1/trips/:tripId', (req, res, ctx) => {
    const { tripId } = req.params
    const trip = mockTrips.find(t => t.tripId === tripId)
    
    if (!trip) {
      return res(ctx.status(404), ctx.json({ error: 'Trip not found' }))
    }
    
    return res(ctx.status(200), ctx.json({ data: trip }))
  }),

  // Alerts API
  rest.get('/api/v1/alerts', (req, res, ctx) => {
    const severity = req.url.searchParams.get('severity')
    
    let filteredAlerts = mockAlerts
    
    if (severity && severity !== 'all') {
      filteredAlerts = filteredAlerts.filter(a => a.severity === severity)
    }
    
    return res(
      ctx.status(200),
      ctx.json({
        data: filteredAlerts,
        total: filteredAlerts.length,
        page: 1,
        pageSize: 50,
      })
    )
  }),

  // Incidents API
  rest.get('/api/v1/incidents', (req, res, ctx) => {
    return res(
      ctx.status(200),
      ctx.json({
        data: mockIncidents,
        total: mockIncidents.length,
        page: 1,
        pageSize: 50,
      })
    )
  }),

  rest.post('/api/v1/incidents', (req, res, ctx) => {
    return res(
      ctx.status(201),
      ctx.json({
        data: {
          incidentId: 'inc_' + Math.random().toString(36).substr(2, 9),
          ...req.body,
          reportedAt: new Date().toISOString(),
          status: 'new',
        }
      })
    )
  }),

  // Emergency Actions API
  rest.post('/api/v1/emergency/actions', (req, res, ctx) => {
    return res(
      ctx.status(200),
      ctx.json({
        success: true,
        actionId: 'action_' + Math.random().toString(36).substr(2, 9),
        executedAt: new Date().toISOString(),
      })
    )
  }),

  // Teleoperation API
  rest.post('/api/v1/teleoperation/sessions', (req, res, ctx) => {
    return res(
      ctx.status(201),
      ctx.json({
        data: {
          sessionId: 'session_' + Math.random().toString(36).substr(2, 9),
          status: 'connecting',
          startTime: new Date().toISOString(),
          maxDuration: 30,
          latency: 45,
          bandwidth: 8500,
          quality: 'good',
        }
      })
    )
  }),

  // Settings API
  rest.get('/api/v1/settings', (req, res, ctx) => {
    return res(
      ctx.status(200),
      ctx.json({
        data: {
          firstName: 'John',
          lastName: 'Doe',
          email: 'john.doe@atlasmesh.com',
          notifications: {
            criticalAlerts: true,
            tripUpdates: true,
            maintenanceReminders: true,
            systemUpdates: false,
          },
          appearance: {
            theme: 'system',
            language: 'en',
            timezone: 'UTC',
          },
          security: {
            twoFactorEnabled: true,
          },
        }
      })
    )
  }),

  rest.put('/api/v1/settings', (req, res, ctx) => {
    return res(
      ctx.status(200),
      ctx.json({
        data: req.body,
        updatedAt: new Date().toISOString(),
      })
    )
  }),

  // Auth API
  rest.post('/api/v1/auth/login', (req, res, ctx) => {
    return res(
      ctx.status(200),
      ctx.json({
        data: {
          user: {
            id: 'user_123',
            email: 'john.doe@atlasmesh.com',
            firstName: 'John',
            lastName: 'Doe',
            role: 'operator',
            permissions: [
              'fleet:read',
              'fleet:command',
              'trips:read',
              'trips:manage',
              'alerts:read',
              'alerts:manage',
              'incidents:read',
              'incidents:create',
              'emergency:execute',
              'teleoperation:access',
              'teleoperation:certified',
            ],
            twoFactorEnabled: true,
          },
          token: 'mock_jwt_token',
          refreshToken: 'mock_refresh_token',
        }
      })
    )
  }),

  // WebSocket connection mock
  rest.get('/api/v1/ws/status', (req, res, ctx) => {
    return res(
      ctx.status(200),
      ctx.json({
        connected: true,
        lastUpdate: new Date().toISOString(),
      })
    )
  }),
]

// Setup server
export const server = setupServer(...handlers)
