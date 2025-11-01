# AtlasMesh Fleet Management System - API Reference Guide

## Overview

This comprehensive API reference guide covers all available endpoints, authentication methods, data models, and integration patterns for AtlasMesh Fleet Management System. The system provides RESTful APIs with OpenAPI 3.0 specifications for all fleet management services.

### Quick Stats
- **72 Microservices** with RESTful APIs
- **OpenAPI 3.0** compliant specifications
- **Rate Limiting**: 10,000 requests/minute (production)
- **Response Time**: P95 < 100ms, P99 < 500ms
- **Uptime SLA**: 99.9% availability guarantee

### Base URLs by Environment
| Environment | Base URL | Purpose |
|-------------|----------|---------|
| **Production** | `https://api.atlasmesh.ae` | Live production environment |
| **Staging** | `https://api-staging.atlasmesh.ae` | Pre-production testing |
| **Sandbox** | `https://api-sandbox.atlasmesh.ae` | Developer testing |
| **Local** | `http://localhost:8080` | Local development |

## Table of Contents

1. [Authentication & Authorization](#authentication--authorization)
2. [Fleet Management APIs](#fleet-management-apis)
3. [Data Models](#data-models)
4. [Error Handling](#error-handling)
5. [Rate Limiting](#rate-limiting)
6. [SDK & Client Libraries](#sdk--client-libraries)
7. [Webhooks & Events](#webhooks--events)
8. [Fleet Management Integration Examples](#fleet-management-integration-examples)

## Authentication & Authorization

### Authentication Methods

#### JWT Bearer Token
```http
Authorization: Bearer eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...
```

#### API Key Authentication
```http
X-API-Key: your-api-key-here
X-API-Secret: your-api-secret-here
```

#### OAuth 2.0 (Recommended for third-party integrations)
```http
Authorization: Bearer oauth-access-token
```

### Authentication Endpoints

#### Login
```http
POST /api/v1/auth/login
Content-Type: application/json

{
  "username": "user@atlasmesh.ae",
  "password": "secure-password",
  "mfa_code": "123456"
}
```

**Response:**
```json
{
  "access_token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...",
  "refresh_token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...",
  "token_type": "Bearer",
  "expires_in": 3600,
  "user": {
    "id": "user-123",
    "username": "user@atlasmesh.ae",
    "roles": ["fleet_manager"],
    "permissions": ["fleet:read", "fleet:write"]
  }
}
```

#### Token Refresh
```http
POST /api/v1/auth/refresh
Content-Type: application/json

{
  "refresh_token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9..."
}
```

#### Logout
```http
POST /api/v1/auth/logout
Authorization: Bearer your-access-token
```

### Authorization Scopes

| Scope | Description |
|-------|-------------|
| `fleet:read` | Read fleet information and status |
| `fleet:write` | Modify fleet configuration and dispatch |
| `vehicle:read` | Read vehicle data and diagnostics |
| `vehicle:write` | Control vehicles and update settings |
| `analytics:read` | Access analytics and reports |
| `admin:read` | Read system configuration |
| `admin:write` | Modify system settings |

## Fleet Management APIs

### Fleet Manager API

#### Base URL
```
https://api.atlasmesh.ae/fleet-manager/v1
```

#### Endpoints

##### Get Fleet Status
```http
GET /fleets/{fleet_id}
Authorization: Bearer your-token
```

**Response:**
```json
{
  "fleet_id": "fleet-001",
  "name": "Abu Dhabi Downtown Fleet",
  "status": "active",
  "total_vehicles": 125,
  "active_vehicles": 89,
  "available_vehicles": 23,
  "maintenance_vehicles": 13,
  "location": {
    "latitude": 24.4539,
    "longitude": 54.3773,
    "address": "Abu Dhabi, UAE"
  },
  "performance": {
    "efficiency": 87.5,
    "utilization": 73.2,
    "uptime": 96.8
  },
  "last_updated": "2023-12-01T10:30:00Z"
}
```

##### Create Vehicle
```http
POST /vehicles
Authorization: Bearer your-token
Content-Type: application/json

{
  "vehicle_id": "AV-001",
  "make": "Tesla",
  "model": "Model Y",
  "year": 2023,
  "vin": "5YJ3E1EA...",
  "license_plate": "AD-12345",
  "fleet_id": "fleet-001",
  "vehicle_type": "sedan",
  "autonomy_level": "L4",
  "specifications": {
    "battery_capacity": 75,
    "range": 525,
    "charging_type": "CCS2",
    "sensors": ["lidar", "camera", "radar", "ultrasonic"]
  }
}
```

##### Dispatch Vehicle
```http
POST /trips
Authorization: Bearer your-token
Content-Type: application/json

{
  "trip_type": "passenger",
  "priority": "normal",
  "pickup_location": {
    "latitude": 24.4539,
    "longitude": 54.3773,
    "address": "Marina Mall, Abu Dhabi"
  },
  "destination": {
    "latitude": 25.2048,
    "longitude": 55.2708,
    "address": "Dubai International Airport"
  },
  "scheduled_time": "2023-12-01T14:30:00Z",
  "passenger_count": 2,
  "special_requirements": ["wheelchair_access"],
  "vehicle_preferences": {
    "vehicle_type": "suv",
    "autonomy_level": "L4"
  }
}
```

**Response:**
```json
{
  "trip_id": "trip-123456",
  "status": "scheduled",
  "assigned_vehicle": "AV-045",
  "estimated_pickup_time": "2023-12-01T14:25:00Z",
  "estimated_arrival_time": "2023-12-01T15:45:00Z",
  "route": {
    "distance": 145.2,
    "duration": 4800,
    "waypoints": [
      {
        "latitude": 24.4539,
        "longitude": 54.3773,
        "type": "pickup"
      },
      {
        "latitude": 25.2048,
        "longitude": 55.2708,
        "type": "destination"
      }
    ]
  },
  "cost_estimate": {
    "base_fare": 45.0,
    "distance_fare": 87.6,
    "time_fare": 24.0,
    "total": 156.6,
    "currency": "AED"
  }
}
```

### Vehicle Gateway API

#### Base URL
```
https://api.atlasmesh.ae/vehicle-gateway/v1
```

#### Real-time Vehicle Data
```http
GET /vehicles/{vehicle_id}/telemetry
Authorization: Bearer your-token
```

**Response:**
```json
{
  "vehicle_id": "AV-001",
  "timestamp": "2023-12-01T10:30:00Z",
  "location": {
    "latitude": 24.4539,
    "longitude": 54.3773,
    "altitude": 12.5,
    "heading": 45.2,
    "speed": 65.5
  },
  "battery": {
    "level": 78.5,
    "voltage": 402.5,
    "current": -125.3,
    "temperature": 32.1,
    "estimated_range": 410.2
  },
  "systems": {
    "autonomy_mode": "L4_active",
    "safety_systems": "operational",
    "communication": "connected",
    "sensors": {
      "lidar": "operational",
      "cameras": "operational",
      "radar": "operational",
      "gps": "operational"
    }
  },
  "diagnostics": {
    "health_score": 92,
    "fault_codes": [],
    "maintenance_due": false,
    "next_service": "2023-12-15T00:00:00Z"
  }
}
```

#### Vehicle Commands
```http
POST /vehicles/{vehicle_id}/commands
Authorization: Bearer your-token
Content-Type: application/json

{
  "command": "emergency_stop",
  "parameters": {
    "reason": "obstacle_detected",
    "location": {
      "latitude": 24.4539,
      "longitude": 54.3773
    }
  },
  "priority": "critical"
}
```

### Policy Engine API

#### Base URL
```
https://api.atlasmesh.ae/policy-engine/v1
```

#### Evaluate Policy
```http
POST /policies/evaluate
Authorization: Bearer your-token
Content-Type: application/json

{
  "policy_type": "route_approval",
  "context": {
    "vehicle_id": "AV-001",
    "route": {
      "origin": {"lat": 24.4539, "lng": 54.3773},
      "destination": {"lat": 25.2048, "lng": 55.2708}
    },
    "weather": {
      "condition": "clear",
      "temperature": 32,
      "wind_speed": 15
    },
    "traffic": {
      "congestion_level": "moderate",
      "incidents": []
    }
  }
}
```

**Response:**
```json
{
  "policy_id": "route-approval-001",
  "decision": "approved",
  "confidence": 0.95,
  "reasoning": [
    "Weather conditions are favorable",
    "No traffic incidents on route",
    "Vehicle health score above threshold"
  ],
  "conditions": [
    "Monitor weather updates",
    "Check traffic every 15 minutes"
  ],
  "valid_until": "2023-12-01T18:00:00Z"
}
```

### Weather Fusion API

#### Base URL
```
https://api.atlasmesh.ae/weather-fusion/v1
```

#### Current Weather
```http
GET /weather/current?lat=24.4539&lng=54.3773
Authorization: Bearer your-token
```

**Response:**
```json
{
  "location": {
    "latitude": 24.4539,
    "longitude": 54.3773,
    "city": "Abu Dhabi",
    "country": "UAE"
  },
  "current": {
    "temperature": 32.5,
    "humidity": 65,
    "wind_speed": 15.2,
    "wind_direction": 245,
    "visibility": 10.0,
    "condition": "clear",
    "uv_index": 8,
    "air_quality": "good"
  },
  "fusion_data": {
    "sources": ["openweather", "weatherapi", "local_sensors"],
    "confidence": 0.92,
    "last_updated": "2023-12-01T10:25:00Z"
  },
  "alerts": [
    {
      "type": "high_temperature",
      "severity": "moderate",
      "message": "High temperature expected, monitor vehicle cooling systems"
    }
  ]
}
```

### Telemetry Ingestion API

#### Base URL
```
https://api.atlasmesh.ae/telemetry/v1
```

#### Submit Telemetry Data
```http
POST /telemetry/batch
Authorization: Bearer your-token
Content-Type: application/json

{
  "vehicle_id": "AV-001",
  "batch_id": "batch-123456",
  "timestamp": "2023-12-01T10:30:00Z",
  "data_points": [
    {
      "timestamp": "2023-12-01T10:30:00Z",
      "type": "location",
      "data": {
        "latitude": 24.4539,
        "longitude": 54.3773,
        "speed": 65.5,
        "heading": 45.2
      }
    },
    {
      "timestamp": "2023-12-01T10:30:01Z",
      "type": "battery",
      "data": {
        "level": 78.5,
        "voltage": 402.5,
        "current": -125.3,
        "temperature": 32.1
      }
    }
  ]
}
```

## Data Models

### Vehicle Model
```json
{
  "vehicle_id": "string",
  "make": "string",
  "model": "string",
  "year": "integer",
  "vin": "string",
  "license_plate": "string",
  "fleet_id": "string",
  "vehicle_type": "sedan|suv|van|bus",
  "autonomy_level": "L2|L3|L4|L5",
  "status": "active|maintenance|idle|charging|offline",
  "location": {
    "latitude": "number",
    "longitude": "number",
    "altitude": "number",
    "heading": "number",
    "speed": "number",
    "timestamp": "string (ISO 8601)"
  },
  "specifications": {
    "battery_capacity": "number",
    "range": "number",
    "charging_type": "string",
    "sensors": ["string"]
  },
  "health": {
    "score": "number (0-100)",
    "battery_health": "number (0-100)",
    "system_health": "number (0-100)",
    "last_service": "string (ISO 8601)",
    "next_service": "string (ISO 8601)"
  },
  "created_at": "string (ISO 8601)",
  "updated_at": "string (ISO 8601)"
}
```

### Trip Model
```json
{
  "trip_id": "string",
  "trip_type": "passenger|cargo|service|emergency",
  "status": "scheduled|dispatched|en_route|pickup|in_progress|completed|cancelled",
  "priority": "low|normal|high|critical",
  "vehicle_id": "string",
  "driver_id": "string",
  "passenger_count": "integer",
  "pickup_location": {
    "latitude": "number",
    "longitude": "number",
    "address": "string",
    "landmark": "string"
  },
  "destination": {
    "latitude": "number",
    "longitude": "number",
    "address": "string",
    "landmark": "string"
  },
  "waypoints": [
    {
      "latitude": "number",
      "longitude": "number",
      "type": "pickup|dropoff|waypoint",
      "eta": "string (ISO 8601)"
    }
  ],
  "schedule": {
    "requested_time": "string (ISO 8601)",
    "scheduled_time": "string (ISO 8601)",
    "estimated_pickup": "string (ISO 8601)",
    "estimated_arrival": "string (ISO 8601)",
    "actual_pickup": "string (ISO 8601)",
    "actual_arrival": "string (ISO 8601)"
  },
  "route": {
    "distance": "number",
    "duration": "number",
    "polyline": "string",
    "traffic_conditions": "light|moderate|heavy|severe"
  },
  "cost": {
    "base_fare": "number",
    "distance_fare": "number",
    "time_fare": "number",
    "surge_multiplier": "number",
    "total": "number",
    "currency": "string"
  },
  "special_requirements": ["string"],
  "created_at": "string (ISO 8601)",
  "updated_at": "string (ISO 8601)"
}
```

### Fleet Model
```json
{
  "fleet_id": "string",
  "name": "string",
  "description": "string",
  "status": "active|inactive|maintenance",
  "location": {
    "latitude": "number",
    "longitude": "number",
    "address": "string",
    "geofence": {
      "type": "polygon",
      "coordinates": [[[number]]]
    }
  },
  "vehicles": {
    "total": "integer",
    "active": "integer",
    "available": "integer",
    "maintenance": "integer",
    "charging": "integer"
  },
  "performance": {
    "efficiency": "number",
    "utilization": "number",
    "uptime": "number",
    "revenue": "number",
    "cost": "number"
  },
  "configuration": {
    "operating_hours": {
      "start": "string (HH:MM)",
      "end": "string (HH:MM)",
      "timezone": "string"
    },
    "service_area": {
      "type": "polygon",
      "coordinates": [[[number]]]
    },
    "vehicle_types": ["string"],
    "max_vehicles": "integer"
  },
  "created_at": "string (ISO 8601)",
  "updated_at": "string (ISO 8601)"
}
```

## Error Handling

### Standard Error Response
```json
{
  "error": {
    "code": "INVALID_REQUEST",
    "message": "The request is invalid or malformed",
    "details": "Missing required field: vehicle_id",
    "timestamp": "2023-12-01T10:30:00Z",
    "request_id": "req-123456789"
  }
}
```

### Error Codes

| Code | HTTP Status | Description |
|------|-------------|-------------|
| `INVALID_REQUEST` | 400 | Request is malformed or missing required fields |
| `UNAUTHORIZED` | 401 | Authentication required or invalid token |
| `FORBIDDEN` | 403 | Insufficient permissions for the requested action |
| `NOT_FOUND` | 404 | Requested resource does not exist |
| `CONFLICT` | 409 | Resource already exists or conflict with current state |
| `RATE_LIMITED` | 429 | Rate limit exceeded |
| `INTERNAL_ERROR` | 500 | Internal server error |
| `SERVICE_UNAVAILABLE` | 503 | Service temporarily unavailable |

### Validation Errors
```json
{
  "error": {
    "code": "VALIDATION_ERROR",
    "message": "Request validation failed",
    "details": {
      "field_errors": [
        {
          "field": "pickup_location.latitude",
          "error": "must be between -90 and 90"
        },
        {
          "field": "passenger_count",
          "error": "must be a positive integer"
        }
      ]
    },
    "timestamp": "2023-12-01T10:30:00Z",
    "request_id": "req-123456789"
  }
}
```

## Rate Limiting

### Rate Limit Headers
```http
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 999
X-RateLimit-Reset: 1609459200
X-RateLimit-Window: 3600
```

### Rate Limits by Endpoint Category

| Category | Requests per Hour | Burst Limit |
|----------|-------------------|-------------|
| Authentication | 100 | 10 |
| Fleet Operations | 1000 | 50 |
| Vehicle Data | 5000 | 100 |
| Telemetry Ingestion | 10000 | 500 |
| Analytics | 500 | 25 |

### Rate Limit Exceeded Response
```json
{
  "error": {
    "code": "RATE_LIMITED",
    "message": "Rate limit exceeded",
    "details": "Too many requests. Limit: 1000/hour, Reset: 2023-12-01T11:00:00Z",
    "timestamp": "2023-12-01T10:30:00Z",
    "request_id": "req-123456789"
  }
}
```

## SDK & Client Libraries

### Official SDKs

#### JavaScript/TypeScript
```bash
npm install @atlasmesh/fleet-os-sdk
```

```javascript
import { FleetOSClient } from '@atlasmesh/fleet-os-sdk';

const client = new FleetOSClient({
  baseUrl: 'https://api.atlasmesh.ae',
  apiKey: 'your-api-key',
  apiSecret: 'your-api-secret'
});

// Get fleet status
const fleet = await client.fleets.get('fleet-001');

// Dispatch vehicle
const trip = await client.trips.create({
  tripType: 'passenger',
  pickupLocation: { lat: 24.4539, lng: 54.3773 },
  destination: { lat: 25.2048, lng: 55.2708 }
});
```

#### Python
```bash
pip install atlasmesh-fleet-os-sdk
```

```python
from atlasmesh import FleetOSClient

client = FleetOSClient(
    base_url='https://api.atlasmesh.ae',
    api_key='your-api-key',
    api_secret='your-api-secret'
)

# Get fleet status
fleet = client.fleets.get('fleet-001')

# Dispatch vehicle
trip = client.trips.create({
    'trip_type': 'passenger',
    'pickup_location': {'lat': 24.4539, 'lng': 54.3773},
    'destination': {'lat': 25.2048, 'lng': 55.2708}
})
```

#### Go
```bash
go get github.com/atlasmesh/fleet-os-sdk-go
```

```go
package main

import (
    "github.com/atlasmesh/fleet-os-sdk-go"
)

func main() {
    client := fleetossdk.NewClient(&fleetossdk.Config{
        BaseURL:   "https://api.atlasmesh.ae",
        APIKey:    "your-api-key",
        APISecret: "your-api-secret",
    })

    // Get fleet status
    fleet, err := client.Fleets.Get("fleet-001")
    if err != nil {
        panic(err)
    }

    // Dispatch vehicle
    trip, err := client.Trips.Create(&fleetossdk.CreateTripRequest{
        TripType: "passenger",
        PickupLocation: &fleetossdk.Location{
            Latitude:  24.4539,
            Longitude: 54.3773,
        },
        Destination: &fleetossdk.Location{
            Latitude:  25.2048,
            Longitude: 55.2708,
        },
    })
}
```

## Webhooks & Events

### Webhook Configuration
```http
POST /api/v1/webhooks
Authorization: Bearer your-token
Content-Type: application/json

{
  "url": "https://your-app.com/webhooks/atlasmesh",
  "events": [
    "trip.created",
    "trip.completed",
    "vehicle.emergency",
    "vehicle.maintenance_due"
  ],
  "secret": "your-webhook-secret",
  "active": true
}
```

### Event Types

#### Trip Events
- `trip.created` - New trip created
- `trip.dispatched` - Vehicle assigned to trip
- `trip.started` - Trip started (passenger pickup)
- `trip.completed` - Trip completed successfully
- `trip.cancelled` - Trip cancelled
- `trip.delayed` - Trip delayed beyond threshold

#### Vehicle Events
- `vehicle.online` - Vehicle came online
- `vehicle.offline` - Vehicle went offline
- `vehicle.emergency` - Emergency situation detected
- `vehicle.maintenance_due` - Maintenance required
- `vehicle.low_battery` - Battery level below threshold
- `vehicle.geofence_exit` - Vehicle left authorized area

#### Fleet Events
- `fleet.status_changed` - Fleet status updated
- `fleet.performance_alert` - Performance threshold breached
- `fleet.capacity_alert` - Fleet capacity issues

### Webhook Payload Example
```json
{
  "event_id": "evt_123456789",
  "event_type": "trip.completed",
  "timestamp": "2023-12-01T10:30:00Z",
  "data": {
    "trip_id": "trip-123456",
    "vehicle_id": "AV-001",
    "status": "completed",
    "duration": 3600,
    "distance": 45.2,
    "cost": 156.6,
    "pickup_time": "2023-12-01T09:30:00Z",
    "dropoff_time": "2023-12-01T10:30:00Z"
  },
  "metadata": {
    "fleet_id": "fleet-001",
    "region": "abu_dhabi"
  }
}
```

### Webhook Verification
```javascript
const crypto = require('crypto');

function verifyWebhook(payload, signature, secret) {
  const expectedSignature = crypto
    .createHmac('sha256', secret)
    .update(payload)
    .digest('hex');
  
  return crypto.timingSafeEqual(
    Buffer.from(signature),
    Buffer.from(expectedSignature)
  );
}
```

## Fleet Management APIs

### Multi-Fleet Coordination

#### Base URL
```
https://api.atlasmesh.ae/fleet-coordination/v1
```

#### Endpoints

##### Get Fleet Federation Status
```http
GET /federation/{federation_id}
Authorization: Bearer your-token
```

**Response:**
```json
{
  "federation_id": "fed-001",
  "name": "UAE Fleet Federation",
  "status": "active",
  "fleets": [
    {
      "fleet_id": "fleet-001",
      "name": "Abu Dhabi Fleet",
      "status": "active",
      "isolation_level": "high",
      "resource_sharing": true
    },
    {
      "fleet_id": "fleet-002", 
      "name": "Dubai Fleet",
      "status": "active",
      "isolation_level": "high",
      "resource_sharing": true
    }
  ],
  "coordination_metrics": {
    "efficiency": 87.5,
    "resource_utilization": 73.2,
    "communication_health": 99.1
  }
}
```

##### Cross-Fleet Resource Sharing
```http
POST /federation/{federation_id}/resources/share
Authorization: Bearer your-token
Content-Type: application/json

{
  "source_fleet_id": "fleet-001",
  "target_fleet_id": "fleet-002",
  "resource_type": "vehicle",
  "resource_id": "vehicle-001",
  "duration": 3600,
  "priority": "high"
}
```

### Mission Management

#### Base URL
```
https://api.atlasmesh.ae/mission-manager/v1
```

#### Endpoints

##### Create Mission Template
```http
POST /templates
Authorization: Bearer your-token
Content-Type: application/json

{
  "template_id": "template-001",
  "name": "Standard Delivery Mission",
  "description": "Standard delivery mission template",
  "parameters": {
    "pickup_location": "required",
    "delivery_location": "required",
    "priority": "optional",
    "deadline": "optional"
  },
  "dependencies": [],
  "safety_requirements": {
    "weather_conditions": "clear",
    "traffic_conditions": "normal"
  }
}
```

##### Execute Mission
```http
POST /missions
Authorization: Bearer your-token
Content-Type: application/json

{
  "mission_id": "mission-001",
  "template_id": "template-001",
  "parameters": {
    "pickup_location": {
      "latitude": 24.4539,
      "longitude": 54.3773,
      "address": "Abu Dhabi, UAE"
    },
    "delivery_location": {
      "latitude": 24.4539,
      "longitude": 54.3773,
      "address": "Dubai, UAE"
    },
    "priority": "high",
    "deadline": "2023-12-01T18:00:00Z"
  },
  "fleet_id": "fleet-001",
  "vehicle_id": "vehicle-001"
}
```

### Fleet Optimization

#### Base URL
```
https://api.atlasmesh.ae/fleet-optimizer/v1
```

#### Endpoints

##### Run Fleet Optimization
```http
POST /optimize
Authorization: Bearer your-token
Content-Type: application/json

{
  "fleet_id": "fleet-001",
  "optimization_type": "multi_objective",
  "objectives": [
    {
      "type": "cost",
      "weight": 0.4,
      "target": "minimize"
    },
    {
      "type": "energy",
      "weight": 0.3,
      "target": "minimize"
    },
    {
      "type": "utilization",
      "weight": 0.3,
      "target": "maximize"
    }
  ],
  "constraints": {
    "max_cost": 1000,
    "min_utilization": 0.8,
    "max_energy_consumption": 500
  }
}
```

##### Get Optimization Results
```http
GET /optimize/{optimization_id}/results
Authorization: Bearer your-token
```

**Response:**
```json
{
  "optimization_id": "opt-001",
  "status": "completed",
  "results": {
    "cost_reduction": 15.2,
    "energy_savings": 23.1,
    "utilization_improvement": 12.5,
    "recommendations": [
      {
        "type": "rebalancing",
        "description": "Rebalance vehicles between zones A and B",
        "impact": "high",
        "implementation_time": "2 hours"
      }
    ]
  }
}
```

### Fleet Analytics

#### Base URL
```
https://api.atlasmesh.ae/fleet-analytics/v1
```

#### Endpoints

##### Get Fleet Health Score
```http
GET /fleets/{fleet_id}/health
Authorization: Bearer your-token
```

**Response:**
```json
{
  "fleet_id": "fleet-001",
  "health_score": 87.5,
  "components": {
    "vehicle_health": 92.3,
    "operational_health": 85.1,
    "safety_health": 94.7,
    "efficiency_health": 78.9
  },
  "trends": {
    "health_trend": "improving",
    "change_7d": 2.3,
    "change_30d": 5.7
  },
  "recommendations": [
    {
      "priority": "high",
      "category": "maintenance",
      "description": "Schedule maintenance for 3 vehicles",
      "impact": "prevent_failure"
    }
  ]
}
```

##### Get Predictive Analytics
```http
GET /fleets/{fleet_id}/predictions
Authorization: Bearer your-token
```

**Response:**
```json
{
  "fleet_id": "fleet-001",
  "predictions": {
    "demand_forecast": {
      "next_24h": {
        "peak_demand": "14:00-16:00",
        "expected_vehicles": 45,
        "confidence": 0.87
      },
      "next_7d": {
        "average_daily_demand": 38,
        "peak_day": "Friday",
        "confidence": 0.82
      }
    },
    "maintenance_forecast": {
      "next_30d": [
        {
          "vehicle_id": "vehicle-001",
          "component": "brakes",
          "predicted_failure_date": "2023-12-15",
          "confidence": 0.91
        }
      ]
    }
  }
}
```

### Fleet Resource Management

#### Base URL
```
https://api.atlasmesh.ae/fleet-resources/v1
```

#### Endpoints

##### Allocate Resources
```http
POST /allocate
Authorization: Bearer your-token
Content-Type: application/json

{
  "fleet_id": "fleet-001",
  "resource_type": "vehicle",
  "quantity": 5,
  "duration": 3600,
  "priority": "high",
  "constraints": {
    "vehicle_type": "sedan",
    "max_distance": 50,
    "availability": "immediate"
  }
}
```

##### Get Resource Utilization
```http
GET /fleets/{fleet_id}/utilization
Authorization: Bearer your-token
```

**Response:**
```json
{
  "fleet_id": "fleet-001",
  "utilization": {
    "overall": 73.2,
    "by_type": {
      "vehicles": 78.5,
      "drivers": 65.3,
      "charging_stations": 82.1
    },
    "by_zone": {
      "zone_a": 85.2,
      "zone_b": 61.7,
      "zone_c": 72.9
    }
  },
  "trends": {
    "utilization_trend": "stable",
    "change_7d": -1.2,
    "change_30d": 3.4
  }
}
```

### Fleet Performance Management

#### Base URL
```
https://api.atlasmesh.ae/fleet-performance/v1
```

#### Endpoints

##### Get Fleet Performance Score
```http
GET /fleets/{fleet_id}/performance
Authorization: Bearer your-token
```

**Response:**
```json
{
  "fleet_id": "fleet-001",
  "performance_score": 89.3,
  "metrics": {
    "safety": 94.7,
    "efficiency": 87.2,
    "reliability": 91.8,
    "cost_effectiveness": 83.5
  },
  "benchmarks": {
    "industry_average": 78.9,
    "top_percentile": 95.2,
    "ranking": "top_15_percent"
  },
  "improvements": [
    {
      "metric": "efficiency",
      "current": 87.2,
      "target": 92.0,
      "improvement_potential": 4.8
    }
  ]
}
```

##### Get Performance Benchmarking
```http
GET /fleets/{fleet_id}/benchmarks
Authorization: Bearer your-token
```

**Response:**
```json
{
  "fleet_id": "fleet-001",
  "benchmarks": {
    "industry_comparison": {
      "rank": 15,
      "percentile": 85,
      "total_fleets": 1000
    },
    "peer_comparison": {
      "similar_size_fleets": {
        "rank": 8,
        "percentile": 92,
        "total_peers": 100
      }
    },
    "historical_comparison": {
      "vs_last_quarter": 5.2,
      "vs_last_year": 12.7
    }
  }
}
```

## Integration Examples

### Real-time Fleet Monitoring
```javascript
// WebSocket connection for real-time updates
const ws = new WebSocket('wss://api.atlasmesh.ae/ws/fleet/fleet-001');

ws.onmessage = function(event) {
  const update = JSON.parse(event.data);
  
  switch(update.type) {
    case 'vehicle_location':
      updateVehicleOnMap(update.vehicle_id, update.location);
      break;
    case 'trip_status':
      updateTripStatus(update.trip_id, update.status);
      break;
    case 'alert':
      showAlert(update.message, update.severity);
      break;
  }
};
```

### Batch Trip Creation
```python
import asyncio
from atlasmesh import FleetOSClient

async def create_multiple_trips(client, trip_requests):
    tasks = []
    for request in trip_requests:
        task = client.trips.create_async(request)
        tasks.append(task)
    
    results = await asyncio.gather(*tasks, return_exceptions=True)
    
    successful = [r for r in results if not isinstance(r, Exception)]
    failed = [r for r in results if isinstance(r, Exception)]
    
    return successful, failed
```

### Custom Analytics Integration
```go
package main

import (
    "time"
    "github.com/atlasmesh/fleet-os-sdk-go"
)

func generateFleetReport(client *fleetossdk.Client, fleetID string) {
    // Get fleet performance data
    performance, err := client.Analytics.GetFleetPerformance(fleetID, &fleetossdk.TimeRange{
        Start: time.Now().AddDate(0, -1, 0), // Last month
        End:   time.Now(),
    })
    if err != nil {
        panic(err)
    }

    // Get vehicle utilization
    utilization, err := client.Analytics.GetVehicleUtilization(fleetID, &fleetossdk.TimeRange{
        Start: time.Now().AddDate(0, -1, 0),
        End:   time.Now(),
    })
    if err != nil {
        panic(err)
    }

    // Generate report
    report := generateReport(performance, utilization)
    
    // Send to external system
    sendToBI(report)
}
```

## Best Practices

### API Usage Guidelines

1. **Authentication**
   - Use OAuth 2.0 for production integrations
   - Implement token refresh logic
   - Store credentials securely

2. **Error Handling**
   - Implement exponential backoff for retries
   - Handle rate limiting gracefully
   - Log errors with request IDs

3. **Performance**
   - Use batch operations when possible
   - Implement caching for frequently accessed data
   - Use WebSockets for real-time updates

4. **Security**
   - Validate webhook signatures
   - Use HTTPS for all API calls
   - Implement proper CORS policies

### Rate Limiting Best Practices

```javascript
class RateLimitedClient {
  constructor(client) {
    this.client = client;
    this.requestQueue = [];
    this.processing = false;
  }

  async makeRequest(request) {
    return new Promise((resolve, reject) => {
      this.requestQueue.push({ request, resolve, reject });
      this.processQueue();
    });
  }

  async processQueue() {
    if (this.processing || this.requestQueue.length === 0) {
      return;
    }

    this.processing = true;

    while (this.requestQueue.length > 0) {
      const { request, resolve, reject } = this.requestQueue.shift();

      try {
        const response = await this.client.makeRequest(request);
        resolve(response);
      } catch (error) {
        if (error.status === 429) {
          // Rate limited - wait and retry
          const retryAfter = error.headers['retry-after'] || 60;
          await this.sleep(retryAfter * 1000);
          this.requestQueue.unshift({ request, resolve, reject });
        } else {
          reject(error);
        }
      }

      // Small delay between requests
      await this.sleep(100);
    }

    this.processing = false;
  }

  sleep(ms) {
    return new Promise(resolve => setTimeout(resolve, ms));
  }
}
```

## Support & Resources

### Documentation Links
- **OpenAPI Specifications**: https://api.atlasmesh.ae/docs
- **SDK Documentation**: https://docs.atlasmesh.ae/sdks
- **Integration Guides**: https://docs.atlasmesh.ae/integrations
- **Changelog**: https://docs.atlasmesh.ae/changelog

### Support Channels
- **Developer Support**: dev-support@atlasmesh.ae
- **API Issues**: api-support@atlasmesh.ae
- **Community Forum**: https://community.atlasmesh.ae
- **Status Page**: https://status.atlasmesh.ae

### Rate Limits & Quotas
- **Sandbox Environment**: 1,000 requests/hour
- **Production Environment**: Contact sales for custom limits
- **WebSocket Connections**: 100 concurrent per API key

---

**API Version**: v1.0.0  
**Last Updated**: December 2023  
**Next Review**: March 2024

For the most up-to-date API specifications, please refer to the OpenAPI documentation at https://api.atlasmesh.ae/docs
