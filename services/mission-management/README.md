# Mission Management Service

Mission templates, orchestration, execution, and trip management service for AtlasMesh Fleet OS.

## Overview

The Mission Management Service provides comprehensive mission and trip lifecycle management, from template creation to execution monitoring. It enables standardized mission operations and trip management across fleets with dependency management, safety monitoring, and performance tracking.

## Features

- **Mission Templates**: Create reusable mission templates with parameters and safety requirements
- **Mission Orchestration**: Execute missions with dependency management and status tracking
- **Trip Management**: Comprehensive trip planning, execution, tracking, and analytics
- **Route Optimization**: Multi-objective route optimization with real-time traffic integration
- **Real-time Tracking**: GPS tracking, ETA updates, and progress notifications
- **Safety Monitoring**: Real-time safety event detection and resolution
- **Performance Metrics**: Track mission and trip performance and efficiency
- **Dependency Management**: Handle complex mission dependencies and execution order
- **Logging & Analytics**: Comprehensive mission and trip logging and analytics

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                  Mission Management Service                  │
├─────────────────────────────────────────────────────────────┤
│  API Layer                                                  │
│  ├── Mission Template Management                           │
│  ├── Mission Execution Management                          │
│  ├── Dependency Management                                 │
│  ├── Safety Event Management                              │
│  └── Metrics & Analytics                                   │
├─────────────────────────────────────────────────────────────┤
│  Business Logic Layer                                       │
│  ├── Mission Orchestrator                                 │
│  ├── Dependency Resolver                                   │
│  ├── Safety Monitor                                        │
│  ├── Performance Tracker                                  │
│  └── Template Engine                                       │
├─────────────────────────────────────────────────────────────┤
│  Data Layer                                                 │
│  ├── PostgreSQL (Missions, Templates, Dependencies)       │
│  ├── Redis (Caching, Sessions)                           │
│  ├── Kafka (Events, Notifications)                        │
│  └── ClickHouse (Analytics, Metrics)                      │
└─────────────────────────────────────────────────────────────┘
```

## API Endpoints

### Mission Templates
- `POST /api/v1/mission-management/templates` - Create template
- `GET /api/v1/mission-management/templates` - List templates
- `GET /api/v1/mission-management/templates/{id}` - Get template
- `PUT /api/v1/mission-management/templates/{id}` - Update template
- `DELETE /api/v1/mission-management/templates/{id}` - Delete template

### Missions
- `POST /api/v1/mission-management/missions` - Create mission
- `GET /api/v1/mission-management/missions` - List missions
- `GET /api/v1/mission-management/missions/{id}` - Get mission
- `PUT /api/v1/mission-management/missions/{id}` - Update mission
- `DELETE /api/v1/mission-management/missions/{id}` - Delete mission
- `POST /api/v1/mission-management/missions/{id}/start` - Start mission
- `POST /api/v1/mission-management/missions/{id}/complete` - Complete mission

### Mission Dependencies
- `POST /api/v1/mission-management/missions/{id}/dependencies` - Add dependency
- `GET /api/v1/mission-management/missions/{id}/dependencies` - List dependencies
- `DELETE /api/v1/mission-management/missions/{id}/dependencies/{dep_id}` - Remove dependency

### Mission Logs
- `GET /api/v1/mission-management/missions/{id}/logs` - Get mission logs
- `POST /api/v1/mission-management/missions/{id}/logs` - Create mission log

### Mission Metrics
- `GET /api/v1/mission-management/missions/{id}/metrics` - Get mission metrics
- `POST /api/v1/mission-management/missions/{id}/metrics` - Create mission metric

### Safety Events
- `GET /api/v1/mission-management/safety-events` - List safety events
- `POST /api/v1/mission-management/safety-events` - Create safety event
- `POST /api/v1/mission-management/safety-events/{id}/resolve` - Resolve safety event

### Trip Management
- `POST /api/v1/mission-management/trips` - Create new trip request
- `GET /api/v1/mission-management/trips` - List trips
- `GET /api/v1/mission-management/trips/{id}` - Get trip details
- `PUT /api/v1/mission-management/trips/{id}` - Update trip
- `DELETE /api/v1/mission-management/trips/{id}` - Cancel trip
- `GET /api/v1/mission-management/trips/{id}/track` - Get real-time trip tracking
- `POST /api/v1/mission-management/trips/{id}/start` - Start trip
- `POST /api/v1/mission-management/trips/{id}/complete` - Complete trip

### Trip Analytics
- `GET /api/v1/mission-management/trips/analytics` - Get trip analytics
- `GET /api/v1/mission-management/trips/{id}/metrics` - Get trip metrics

## Database Schema

### Mission Templates
- `mission_templates` - Template definitions with parameters and safety requirements
- `missions` - Mission instances with execution status
- `mission_dependencies` - Mission dependency relationships
- `mission_logs` - Mission execution logs
- `mission_metrics` - Mission performance metrics
- `safety_events` - Safety event tracking and resolution

### Trip Management
- `trips` - Trip instances with status and tracking data
- `trip_routes` - Route information and optimization data
- `trip_tracking` - Real-time trip tracking and location data
- `trip_analytics` - Trip performance metrics and analytics
- `trip_logs` - Trip execution logs and events

## Configuration

### Environment Variables
- `DATABASE_URL` - PostgreSQL connection string
- `REDIS_ADDR` - Redis server address
- `KAFKA_BROKERS` - Kafka broker addresses
- `CLICKHOUSE_ADDR` - ClickHouse server address
- `PORT` - Service port (default: 8080)

### Dependencies
- PostgreSQL 13+
- Redis 6+
- Kafka 2.8+
- ClickHouse 22+

## Development

### Prerequisites
- Go 1.21+
- Docker & Docker Compose
- Make

### Running Locally
```bash
# Start dependencies
make dev-deps

# Run migrations
make migrate

# Start service
make run
```

### Testing
```bash
# Run unit tests
make test

# Run integration tests
make test-integration

# Run load tests
make test-load
```

## Deployment

### Docker
```bash
# Build image
docker build -t mission-management .

# Run container
docker run -p 8080:8080 mission-management
```

### Kubernetes
```bash
# Apply manifests
kubectl apply -f k8s/

# Check status
kubectl get pods -l app=mission-management
```

## Monitoring

### Health Checks
- `GET /health` - Service health status
- Database connectivity
- Redis connectivity
- Kafka connectivity
- ClickHouse connectivity

### Metrics
- Mission success rate
- Mission execution time
- Safety event count
- Dependency resolution time
- Template usage statistics

### Logging
- Structured JSON logging
- Request/response logging
- Error tracking
- Performance monitoring

## Security

### Authentication
- JWT token validation
- API key authentication
- Role-based access control

### Authorization
- Mission-level permissions
- Template-level permissions
- Safety event permissions
- Metrics access permissions

### Data Protection
- Encryption in transit (TLS)
- Encryption at rest
- PII data handling
- Audit logging

## Integration

### Event Bus
- Kafka integration for events
- Event sourcing for state changes
- Real-time notifications
- Event replay capabilities

### Service Mesh
- Istio integration
- mTLS between services
- Traffic management
- Circuit breakers

### API Gateway
- Kong/Envoy integration
- Rate limiting
- Request routing
- Authentication

## Troubleshooting

### Common Issues
1. **Database Connection**: Check DATABASE_URL and network connectivity
2. **Redis Connection**: Verify REDIS_ADDR and Redis server status
3. **Kafka Connection**: Ensure KAFKA_BROKERS are accessible
4. **ClickHouse Connection**: Verify CLICKHOUSE_ADDR and server status

### Debug Mode
```bash
# Enable debug logging
export LOG_LEVEL=debug
make run
```

### Performance Tuning
- Database connection pooling
- Redis connection pooling
- Kafka batch processing
- ClickHouse query optimization

## Contributing

1. Fork the repository
2. Create feature branch
3. Make changes
4. Add tests
5. Submit pull request

## License

MIT License - see LICENSE file for details.
