# Wenco Mining Adapter

## Purpose
The Wenco Mining Adapter provides integration with Wenco Fleet Management System for autonomous vehicle fleet management in mining operations. This adapter enables real-time equipment monitoring, production reporting, and fleet optimization for mining vehicles.

## Features
- **Equipment Integration**: Real-time integration with Wenco FMS
- **Production Reporting**: Automated production data collection and reporting
- **Fleet Optimization**: Integration with Wenco's optimization algorithms
- **Equipment Health**: Real-time equipment health monitoring
- **Location Tracking**: GPS-based location tracking for mining vehicles
- **Payload Data**: Integration with payload monitoring systems

## Architecture
```
┌─────────────────────────────────────────────────────────────┐
│ Wenco Mining Adapter                                        │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│ ┌──────────────┐ ┌──────────────┐ ┌─────────────┐          │
│ │ Equipment    │ │ Production   │ │ Location    │          │
│ │ Interface    │ │ Reporting    │ │ Tracking    │          │
│ └──────────────┘ └──────────────┘ └─────────────┘          │
│                                                             │
│ ┌──────────────┐ ┌──────────────┐ ┌─────────────┐          │
│ │ Fleet        │ │ Payload      │ │ Health      │          │
│ │ Optimization │ │ Monitoring   │ │ Monitoring  │          │
│ └──────────────┘ └──────────────┘ └─────────────┘          │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

## Configuration
The adapter supports configuration through YAML files and environment variables:

```yaml
wenco:
  enabled: true
  endpoint: "https://wenco.example.com/api"
  api_key: "${WENCO_API_KEY}"
  site_id: "mine_001"
  equipment_types:
    - "haul_truck"
    - "bulldozer"
    - "excavator"
    - "loader"
  production_reporting:
    enabled: true
    interval_minutes: 15
  location_tracking:
    enabled: true
    update_interval_seconds: 30
  payload_monitoring:
    enabled: true
    sensors: ["weight", "volume", "density"]
```

## API Endpoints
- `GET /api/v1/wenco/equipment` - Get equipment status
- `GET /api/v1/wenco/production` - Get production data
- `GET /api/v1/wenco/locations` - Get equipment locations
- `POST /api/v1/wenco/optimization` - Trigger fleet optimization
- `GET /api/v1/wenco/health` - Get equipment health status

## Deployment
The adapter can be deployed using Docker or Kubernetes:

```bash
# Docker
docker build -t wenco-mining-adapter .
docker run -p 8080:8080 wenco-mining-adapter

# Kubernetes
kubectl apply -f k8s/deployment.yaml
```

## Monitoring
The adapter provides Prometheus metrics for monitoring:
- `wenco_equipment_status_total` - Equipment status count
- `wenco_production_tonnes` - Production in tonnes
- `wenco_optimization_time_seconds` - Optimization duration
- `wenco_equipment_health_score` - Equipment health score

## Testing
Run the test suite:

```bash
go test ./...
```

## License
This adapter is part of the AtlasMesh Fleet OS project and is subject to the project's license terms.
