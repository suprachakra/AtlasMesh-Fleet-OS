# AtlasMesh Fleet OS API Documentation

## Overview

The AtlasMesh Fleet OS provides a comprehensive REST API for managing autonomous vehicle fleets. This documentation covers all available endpoints, authentication, data models, and integration examples.

## Base URL

```
Production: https://api.atlasmesh.com
Staging: https://staging-api.atlasmesh.com
Development: http://localhost:8080
```

## Authentication

All API requests require authentication using JWT tokens. Include the token in the Authorization header:

```http
Authorization: Bearer <your-jwt-token>
```

### Getting an Access Token

```http
POST /api/v1/auth/login
Content-Type: application/json

{
  "email": "user@example.com",
  "password": "your-password"
}
```

**Response:**
```json
{
  "access_token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...",
  "token_type": "bearer",
  "expires_in": 3600,
  "refresh_token": "refresh-token-here"
}
```

## API Endpoints

### Fleet Manager API

#### Organizations

##### Create Organization
```http
POST /api/v1/organizations
```

**Request Body:**
```json
{
  "name": "Dubai Fleet Management",
  "description": "Autonomous fleet management for Dubai",
  "contact_info": {
    "email": "contact@dubaifleet.com",
    "phone": "+971501234567",
    "address": "Dubai, UAE"
  },
  "settings": {
    "timezone": "Asia/Dubai",
    "currency": "AED",
    "language": "en"
  }
}
```

**Response:**
```json
{
  "organization_id": "org-123456",
  "name": "Dubai Fleet Management",
  "status": "active",
  "created_at": "2024-01-15T10:30:00Z",
  "message": "Organization created successfully"
}
```

##### Get Organization
```http
GET /api/v1/organizations/{organization_id}
```

##### Update Organization
```http
PUT /api/v1/organizations/{organization_id}
```

##### Delete Organization
```http
DELETE /api/v1/organizations/{organization_id}
```

#### Fleets

##### Create Fleet
```http
POST /api/v1/fleets
```

**Request Body:**
```json
{
  "organization_id": "org-123456",
  "name": "Dubai Taxi Fleet",
  "description": "Autonomous taxi fleet for Dubai",
  "fleet_type": "autonomous",
  "capacity": 100,
  "operational_area": {
    "center": {
      "latitude": 25.2048,
      "longitude": 55.2708
    },
    "radius": 50
  },
  "settings": {
    "autonomy_level": "L4",
    "max_speed": 60,
    "safety_mode": "strict"
  }
}
```

##### Get Fleet
```http
GET /api/v1/fleets/{fleet_id}
```

##### Get Fleet Vehicles
```http
GET /api/v1/fleets/{fleet_id}/vehicles?page=1&limit=50&status=active
```

**Query Parameters:**
- `page`: Page number (default: 1)
- `limit`: Items per page (default: 50, max: 100)
- `status`: Filter by vehicle status (active, maintenance, offline)
- `sort`: Sort field (created_at, updated_at, asset_tag)
- `order`: Sort order (asc, desc)

#### Vehicles

##### Create Vehicle
```http
POST /api/v1/fleet/vehicles
```

**Request Body:**
```json
{
  "fleet_id": "fleet-123456",
  "asset_tag": "DT-001",
  "vin": "1HGBH41JXMN109186",
  "license_plate": "DXB-001",
  "manufacturer": "Tesla",
  "model": "Model 3",
  "year": 2023,
  "serial_number": "TSL-001",
  "vehicle_profile": {
    "battery_capacity": 75,
    "max_range": 400,
    "passenger_capacity": 4,
    "cargo_capacity": 0
  },
  "capabilities": {
    "autonomous_driving": true,
    "remote_operation": true,
    "v2x_communication": true,
    "emergency_braking": true
  },
  "sensor_configuration": {
    "cameras": 8,
    "lidar": 1,
    "radar": 12,
    "ultrasonic": 12
  },
  "autonomy_level": "L4",
  "autonomy_capabilities": {
    "highway_driving": true,
    "city_driving": true,
    "parking": true,
    "weather_conditions": ["clear", "rain"]
  },
  "odd_configuration": {
    "operational_design_domain": {
      "geofence": {
        "center": {"latitude": 25.2048, "longitude": 55.2708},
        "radius": 50
      },
      "speed_limits": {
        "highway": 120,
        "city": 60,
        "residential": 30
      },
      "weather_conditions": ["clear", "rain", "fog"]
    }
  },
  "tags": ["taxi", "autonomous", "electric"],
  "metadata": {
    "purchase_date": "2024-01-01",
    "warranty_expiry": "2027-01-01",
    "insurance_policy": "POL-123456"
  }
}
```

##### Get Vehicle
```http
GET /api/v1/fleet/vehicles/{vehicle_id}
```

##### Update Vehicle
```http
PUT /api/v1/fleet/vehicles/{vehicle_id}
```

##### Delete Vehicle
```http
DELETE /api/v1/fleet/vehicles/{vehicle_id}
```

##### Get Vehicle Status
```http
GET /api/v1/fleet/vehicles/{vehicle_id}/status
```

**Response:**
```json
{
  "vehicle_id": "vehicle-123456",
  "operational_status": "online",
  "autonomy_status": "autonomous",
  "location": {
    "latitude": 25.2048,
    "longitude": 55.2708,
    "altitude": 10.5,
    "heading": 180.0,
    "speed": 45.0,
    "accuracy": 5.0
  },
  "battery_level": 85.5,
  "fuel_level": 0.0,
  "health_score": 95.0,
  "last_seen": "2024-01-15T10:30:00Z",
  "current_trip": {
    "trip_id": "trip-123456",
    "status": "in_progress",
    "passenger_count": 2
  },
  "alerts": [],
  "maintenance_due": false,
  "next_inspection": "2024-02-15T00:00:00Z"
}
```

##### Update Vehicle Location
```http
PUT /api/v1/fleet/vehicles/{vehicle_id}/location
```

**Request Body:**
```json
{
  "latitude": 25.2048,
  "longitude": 55.2708,
  "altitude": 10.5,
  "heading": 180.0,
  "speed": 45.0,
  "accuracy": 5.0,
  "timestamp": "2024-01-15T10:30:00Z"
}
```

##### Dispatch Vehicle
```http
POST /api/v1/fleet/vehicles/{vehicle_id}/dispatch
```

**Request Body:**
```json
{
  "trip_id": "trip-123456",
  "origin": {
    "latitude": 25.2048,
    "longitude": 55.2708,
    "address": "Dubai Marina"
  },
  "destination": {
    "latitude": 25.2582,
    "longitude": 55.3047,
    "address": "Dubai Mall"
  },
  "passenger_id": "passenger-123456",
  "priority": "normal",
  "estimated_duration": 25,
  "estimated_distance": 15.5,
  "instructions": "Pick up at main entrance",
  "constraints": {
    "max_speed": 60,
    "avoid_tolls": false,
    "preferred_route": "fastest"
  }
}
```

##### Recall Vehicle
```http
POST /api/v1/fleet/vehicles/{vehicle_id}/recall
```

**Request Body:**
```json
{
  "reason": "emergency",
  "priority": "high",
  "destination": {
    "latitude": 25.2048,
    "longitude": 55.2708,
    "address": "Service Center"
  },
  "instructions": "Return to service center immediately"
}
```

#### Fleet Analytics

##### Get Fleet Utilization
```http
GET /api/v1/fleet/fleets/{fleet_id}/utilization?timeframe=24h
```

**Query Parameters:**
- `timeframe`: Time range (1h, 24h, 7d, 30d)

**Response:**
```json
{
  "fleet_id": "fleet-123456",
  "timeframe": "24h",
  "utilization_rate": 85.5,
  "active_vehicles": 85,
  "total_vehicles": 100,
  "average_trip_time": 25.5,
  "total_distance": 1250.5,
  "total_trips": 150,
  "revenue": 2500.0,
  "cost_per_km": 0.15,
  "efficiency_score": 92.0,
  "breakdown": {
    "by_status": {
      "active": 85,
      "maintenance": 10,
      "offline": 5
    },
    "by_vehicle_type": {
      "sedan": 60,
      "suv": 25,
      "van": 15
    }
  },
  "trends": {
    "utilization_trend": [80, 82, 85, 87, 85],
    "revenue_trend": [2000, 2200, 2400, 2500, 2500]
  }
}
```

##### Get Vehicle Health Metrics
```http
GET /api/v1/fleet/vehicles/{vehicle_id}/health
```

**Response:**
```json
{
  "vehicle_id": "vehicle-123456",
  "overall_health_score": 95.0,
  "component_health": {
    "battery": {
      "score": 98.0,
      "status": "excellent",
      "capacity": 95.5,
      "cycles": 150,
      "temperature": 25.0
    },
    "engine": {
      "score": 92.0,
      "status": "good",
      "temperature": 75.0,
      "oil_level": "normal",
      "rpm": 1200
    },
    "tires": {
      "score": 88.0,
      "status": "good",
      "pressure": [32, 32, 31, 32],
      "tread_depth": [8.5, 8.3, 8.4, 8.6]
    },
    "brakes": {
      "score": 94.0,
      "status": "excellent",
      "pad_thickness": 8.5,
      "fluid_level": "normal"
    }
  },
  "maintenance_schedule": {
    "next_service": "2024-02-15T00:00:00Z",
    "overdue_items": [],
    "recommendations": [
      "Check tire pressure",
      "Rotate tires"
    ]
  },
  "alerts": [],
  "last_updated": "2024-01-15T10:30:00Z"
}
```

##### Get Operational Metrics
```http
GET /api/v1/fleet/operational-metrics?fleet_id=fleet-123456&timeframe=24h
```

### Analytics API

#### Fleet KPIs
```http
GET /api/v1/analytics/kpis?timeframe=24h
```

**Response:**
```json
{
  "utilization_rate": 85.5,
  "average_trip_time": 25.5,
  "fuel_efficiency": 0.85,
  "on_time_performance": 92.0,
  "total_revenue": 25000.0,
  "cost_per_kilometer": 0.15,
  "customer_satisfaction": 4.5,
  "fleet_availability": 95.0,
  "timestamp": "2024-01-15T10:30:00Z"
}
```

#### Operational Metrics
```http
GET /api/v1/analytics/operational
```

#### Financial Metrics
```http
GET /api/v1/analytics/financial
```

#### Safety Metrics
```http
GET /api/v1/analytics/safety
```

#### Custom Query
```http
POST /api/v1/analytics/query
```

**Request Body:**
```json
{
  "query": "SELECT AVG(utilization_rate) as avg_utilization FROM fleet_analytics.utilization_metrics WHERE timestamp >= now() - INTERVAL 24 HOUR",
  "parameters": {}
}
```

#### Real-time Metrics
```http
GET /api/v1/analytics/realtime
```

#### Analytics Stream
```http
GET /api/v1/analytics/stream
```

### External Integrations API

#### UAE Government Integration

##### Get Vehicle Registration
```http
GET /api/v1/uae/vehicle-registration/{vehicle_id}
```

##### Check Compliance
```http
GET /api/v1/uae/compliance-check/{vehicle_id}
```

##### Generate Compliance Report
```http
POST /api/v1/uae/generate-report
```

**Request Body:**
```json
{
  "vehicle_ids": ["vehicle-123456", "vehicle-789012"],
  "report_type": "monthly",
  "format": "pdf"
}
```

#### Weather Integration

##### Get Current Weather
```http
GET /api/v1/weather/current/{location}
```

##### Get Weather Forecast
```http
GET /api/v1/weather/forecast/{location}
```

##### Get Weather Alerts
```http
GET /api/v1/weather/alerts/{location}
```

#### Maps Integration

##### Geocode Address
```http
GET /api/v1/maps/geocode/{address}
```

##### Get Route
```http
POST /api/v1/maps/route
```

**Request Body:**
```json
{
  "origin": "Dubai Marina",
  "destination": "Dubai Mall",
  "mode": "driving"
}
```

##### Get Traffic Info
```http
GET /api/v1/maps/traffic/{location}
```

#### ERP Integration

##### Get ERP Orders
```http
GET /api/v1/erp/orders
```

##### Get ERP Order
```http
GET /api/v1/erp/orders/{order_id}
```

##### Sync ERP Data
```http
POST /api/v1/erp/sync
```

#### WMS Integration

##### Get WMS Inventory
```http
GET /api/v1/wms/inventory/{warehouse_id}
```

##### Get WMS Warehouses
```http
GET /api/v1/wms/warehouses
```

##### Sync WMS Data
```http
POST /api/v1/wms/sync
```

### ML Pipeline API

#### Models

##### Get Models
```http
GET /api/v1/models
```

##### Get Model
```http
GET /api/v1/models/{model_id}
```

##### Train Model
```http
POST /api/v1/models/{model_id}/train
```

##### Update Model
```http
POST /api/v1/models/{model_id}/update
```

#### Predictions

##### Predict Maintenance
```http
POST /api/v1/predict/maintenance
```

**Request Body:**
```json
{
  "vehicle_id": "vehicle-123456",
  "data": {
    "battery_level": 85.5,
    "engine_temp": 75.2,
    "mileage": 15000,
    "age": 2,
    "usage_pattern": "normal"
  }
}
```

**Response:**
```json
{
  "vehicle_id": "vehicle-123456",
  "component": "battery",
  "failure_probability": 0.15,
  "time_to_failure_hours": 72,
  "confidence": 0.92,
  "recommendations": [
    "Schedule battery inspection",
    "Monitor charging patterns"
  ],
  "timestamp": "2024-01-15T10:30:00Z"
}
```

##### Optimize Route
```http
POST /api/v1/predict/route
```

**Request Body:**
```json
{
  "origin": {
    "latitude": 25.2048,
    "longitude": 55.2708,
    "address": "Dubai Marina"
  },
  "destination": {
    "latitude": 25.2582,
    "longitude": 55.3047,
    "address": "Dubai Mall"
  },
  "constraints": {
    "max_duration": 30,
    "avoid_tolls": true,
    "preferred_route": "fastest"
  }
}
```

##### Forecast Demand
```http
POST /api/v1/predict/demand
```

**Request Body:**
```json
{
  "location_id": "location-001",
  "time_window": "next_hour",
  "factors": {
    "weather": "clear",
    "time_of_day": "morning",
    "events": "none"
  }
}
```

##### Detect Anomaly
```http
POST /api/v1/predict/anomaly
```

**Request Body:**
```json
{
  "vehicle_id": "vehicle-123456",
  "data": {
    "sensor_readings": {
      "accelerometer": [0.1, 0.2, 9.8],
      "gyroscope": [0.01, 0.02, 0.03],
      "temperature": 25.5
    },
    "behavior_patterns": {
      "speed_variance": 0.15,
      "braking_frequency": 0.05,
      "acceleration_pattern": "normal"
    }
  }
}
```

#### Batch Processing

##### Batch Predict
```http
POST /api/v1/batch/predict
```

**Request Body:**
```json
{
  "model_id": "maintenance-predictor",
  "data": [
    {
      "vehicle_id": "vehicle-001",
      "data": {"battery_level": 85.5}
    },
    {
      "vehicle_id": "vehicle-002",
      "data": {"battery_level": 75.2}
    }
  ]
}
```

##### Batch Train
```http
POST /api/v1/batch/train
```

#### Model Performance

##### Get Model Performance
```http
GET /api/v1/models/{model_id}/performance
```

##### Get Model Metrics
```http
GET /api/v1/models/{model_id}/metrics
```

## Error Handling

### HTTP Status Codes

- `200 OK`: Request successful
- `201 Created`: Resource created successfully
- `400 Bad Request`: Invalid request data
- `401 Unauthorized`: Authentication required
- `403 Forbidden`: Insufficient permissions
- `404 Not Found`: Resource not found
- `409 Conflict`: Resource conflict (e.g., duplicate)
- `422 Unprocessable Entity`: Validation error
- `429 Too Many Requests`: Rate limit exceeded
- `500 Internal Server Error`: Server error
- `503 Service Unavailable`: Service temporarily unavailable

### Error Response Format

```json
{
  "error": {
    "code": "VALIDATION_ERROR",
    "message": "Invalid request data",
    "details": [
      {
        "field": "email",
        "message": "Email format is invalid"
      }
    ],
    "request_id": "req-123456",
    "timestamp": "2024-01-15T10:30:00Z"
  }
}
```

### Common Error Codes

- `INVALID_TOKEN`: Invalid or expired JWT token
- `INSUFFICIENT_PERMISSIONS`: User lacks required permissions
- `RESOURCE_NOT_FOUND`: Requested resource doesn't exist
- `VALIDATION_ERROR`: Request data validation failed
- `RATE_LIMIT_EXCEEDED`: Too many requests
- `SERVICE_UNAVAILABLE`: Service temporarily unavailable
- `INTERNAL_ERROR`: Internal server error

## Rate Limiting

API requests are rate limited to ensure fair usage:

- **Standard users**: 1000 requests per hour
- **Premium users**: 5000 requests per hour
- **Enterprise users**: 10000 requests per hour

Rate limit headers are included in responses:

```http
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 999
X-RateLimit-Reset: 1642248600
```

## Pagination

List endpoints support pagination:

```http
GET /api/v1/fleet/vehicles?page=1&limit=50
```

**Response:**
```json
{
  "data": [...],
  "pagination": {
    "page": 1,
    "limit": 50,
    "total": 150,
    "pages": 3,
    "has_next": true,
    "has_prev": false
  }
}
```

## Webhooks

Subscribe to real-time events:

```http
POST /api/v1/webhooks
```

**Request Body:**
```json
{
  "url": "https://your-app.com/webhooks",
  "events": ["vehicle.status_changed", "trip.completed"],
  "secret": "webhook-secret"
}
```

### Webhook Events

- `vehicle.created`: Vehicle created
- `vehicle.status_changed`: Vehicle status changed
- `vehicle.location_updated`: Vehicle location updated
- `trip.started`: Trip started
- `trip.completed`: Trip completed
- `trip.cancelled`: Trip cancelled
- `maintenance.scheduled`: Maintenance scheduled
- `alert.triggered`: Alert triggered
- `fleet.utilization_updated`: Fleet utilization updated

### Webhook Payload

```json
{
  "event": "vehicle.status_changed",
  "data": {
    "vehicle_id": "vehicle-123456",
    "old_status": "offline",
    "new_status": "online",
    "timestamp": "2024-01-15T10:30:00Z"
  },
  "webhook_id": "webhook-123456"
}
```

## SDKs and Libraries

### JavaScript/Node.js
```bash
npm install @atlasmesh/fleet-os-sdk
```

```javascript
import { AtlasMeshClient } from '@atlasmesh/fleet-os-sdk';

const client = new AtlasMeshClient({
  apiKey: 'your-api-key',
  baseURL: 'https://api.atlasmesh.com'
});

// Get vehicles
const vehicles = await client.vehicles.list();
```

### Python
```bash
pip install atlasmesh-fleet-os
```

```python
from atlasmesh import AtlasMeshClient

client = AtlasMeshClient(
    api_key='your-api-key',
    base_url='https://api.atlasmesh.com'
)

# Get vehicles
vehicles = client.vehicles.list()
```

### Go
```bash
go get github.com/atlasmesh/fleet-os-sdk
```

```go
import "github.com/atlasmesh/fleet-os-sdk"

client := atlasmesh.NewClient("your-api-key", "https://api.atlasmesh.com")

vehicles, err := client.Vehicles.List()
```

## Examples

### Complete Fleet Management Workflow

```javascript
// 1. Create organization
const org = await client.organizations.create({
  name: "Dubai Fleet Management",
  description: "Autonomous fleet for Dubai",
  contact_info: {
    email: "contact@dubaifleet.com",
    phone: "+971501234567"
  }
});

// 2. Create fleet
const fleet = await client.fleets.create({
  organization_id: org.organization_id,
  name: "Dubai Taxi Fleet",
  fleet_type: "autonomous",
  capacity: 100
});

// 3. Create vehicle
const vehicle = await client.vehicles.create({
  fleet_id: fleet.fleet_id,
  asset_tag: "DT-001",
  manufacturer: "Tesla",
  model: "Model 3",
  autonomy_level: "L4"
});

// 4. Update vehicle location
await client.vehicles.updateLocation(vehicle.vehicle_id, {
  latitude: 25.2048,
  longitude: 55.2708,
  speed: 45.0
});

// 5. Dispatch vehicle
const trip = await client.vehicles.dispatch(vehicle.vehicle_id, {
  trip_id: "trip-001",
  origin: { latitude: 25.2048, longitude: 55.2708 },
  destination: { latitude: 25.2582, longitude: 55.3047 }
});

// 6. Get analytics
const kpis = await client.analytics.getKPIs({ timeframe: "24h" });
console.log(`Fleet utilization: ${kpis.utilization_rate}%`);

// 7. Get ML predictions
const prediction = await client.ml.predictMaintenance({
  vehicle_id: vehicle.vehicle_id,
  data: {
    battery_level: 85.5,
    engine_temp: 75.2
  }
});
console.log(`Maintenance needed: ${prediction.failure_probability > 0.5}`);
```

### Real-time Monitoring

```javascript
// Subscribe to vehicle updates
client.websocket.subscribe('vehicle.telemetry', (data) => {
  console.log(`Vehicle ${data.vehicle_id} location:`, data.location);
});

// Subscribe to fleet events
client.websocket.subscribe('fleet.events', (event) => {
  console.log('Fleet event:', event);
});
```

## Support

For API support and questions:

- **Documentation**: https://docs.atlasmesh.com
- **Status Page**: https://status.atlasmesh.com
- **Support Email**: api-support@atlasmesh.com
- **Community Forum**: https://community.atlasmesh.com

## Changelog

### v1.2.0 (2024-01-15)
- Added ML Pipeline API endpoints
- Enhanced analytics with real-time metrics
- Improved error handling and validation
- Added webhook support

### v1.1.0 (2024-01-01)
- Added External Integrations API
- Enhanced vehicle management endpoints
- Improved authentication and security
- Added comprehensive analytics

### v1.0.0 (2023-12-01)
- Initial API release
- Core fleet management functionality
- Basic analytics and reporting
- Authentication and authorization
