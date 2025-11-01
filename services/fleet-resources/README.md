# Fleet Resource Management Service

Resource allocation, optimization, monitoring, and planning service for AtlasMesh Fleet OS.

## Overview

The Fleet Resource Management Service provides comprehensive resource management capabilities for fleet operations, including resource allocation, optimization, monitoring, and planning. It enables efficient resource utilization across fleets and vehicles.

## Features

- **Resource Pools**: Manage resource pools with capacity tracking
- **Resource Allocation**: Allocate resources to vehicles and operations
- **Resource Requests**: Handle resource requests with priority queuing
- **Resource Utilization**: Monitor resource utilization and efficiency
- **Resource Planning**: Plan resource allocation across time horizons
- **Resource Conflicts**: Detect and resolve resource conflicts

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                Fleet Resource Management Service              │
├─────────────────────────────────────────────────────────────┤
│  API Layer                                                  │
│  ├── Resource Pool Management                              │
│  ├── Resource Allocation Management                        │
│  ├── Resource Request Management                           │
│  ├── Resource Utilization Management                       │
│  ├── Resource Planning Management                          │
│  └── Resource Conflict Management                          │
├─────────────────────────────────────────────────────────────┤
│  Business Logic Layer                                       │
│  ├── Resource Pool Manager                                │
│  ├── Resource Allocator                                    │
│  ├── Resource Request Handler                              │
│  ├── Resource Utilization Tracker                          │
│  ├── Resource Planner                                       │
│  └── Resource Conflict Resolver                            │
├─────────────────────────────────────────────────────────────┤
│  Data Layer                                                 │
│  ├── PostgreSQL (Resource Pools, Allocations, Requests)   │
│  ├── Redis (Caching, Sessions)                           │
│  ├── Kafka (Events, Notifications)                        │
│  └── ClickHouse (Analytics, Metrics)                      │
└─────────────────────────────────────────────────────────────┘
```

## API Endpoints

### Resource Pools
- `POST /api/v1/fleet-resources/pools` - Create resource pool
- `GET /api/v1/fleet-resources/pools` - List resource pools
- `GET /api/v1/fleet-resources/pools/{id}` - Get resource pool
- `PUT /api/v1/fleet-resources/pools/{id}` - Update resource pool
- `DELETE /api/v1/fleet-resources/pools/{id}` - Delete resource pool

### Resource Allocations
- `POST /api/v1/fleet-resources/allocations` - Create resource allocation
- `GET /api/v1/fleet-resources/allocations` - List resource allocations
- `GET /api/v1/fleet-resources/allocations/{id}` - Get resource allocation
- `PUT /api/v1/fleet-resources/allocations/{id}` - Update resource allocation
- `DELETE /api/v1/fleet-resources/allocations/{id}` - Delete resource allocation

### Resource Requests
- `POST /api/v1/fleet-resources/requests` - Create resource request
- `GET /api/v1/fleet-resources/requests` - List resource requests
- `GET /api/v1/fleet-resources/requests/{id}` - Get resource request
- `PUT /api/v1/fleet-resources/requests/{id}` - Update resource request
- `DELETE /api/v1/fleet-resources/requests/{id}` - Delete resource request

### Resource Utilization
- `POST /api/v1/fleet-resources/utilization` - Create resource utilization
- `GET /api/v1/fleet-resources/utilization` - List resource utilization
- `GET /api/v1/fleet-resources/utilization/{pool_id}` - Get utilization by pool

### Resource Planning
- `POST /api/v1/fleet-resources/planning` - Create resource planning
- `GET /api/v1/fleet-resources/planning` - List resource planning
- `GET /api/v1/fleet-resources/planning/{id}` - Get resource planning
- `PUT /api/v1/fleet-resources/planning/{id}` - Update resource planning
- `DELETE /api/v1/fleet-resources/planning/{id}` - Delete resource planning

### Resource Conflicts
- `GET /api/v1/fleet-resources/conflicts` - List resource conflicts
- `POST /api/v1/fleet-resources/conflicts` - Create resource conflict
- `POST /api/v1/fleet-resources/conflicts/{id}/resolve` - Resolve resource conflict

## Database Schema

### Resource Management
- `resource_pools` - Resource pool definitions with capacity tracking
- `resource_allocations` - Resource allocations to vehicles and operations
- `resource_requests` - Resource requests with priority queuing
- `resource_utilization` - Resource utilization metrics and analytics
- `resource_planning` - Resource planning across time horizons
- `resource_conflicts` - Resource conflict detection and resolution

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
docker build -t fleet-resources .

# Run container
docker run -p 8080:8080 fleet-resources
```

### Kubernetes
```bash
# Apply manifests
kubectl apply -f k8s/

# Check status
kubectl get pods -l app=fleet-resources
```

## Monitoring

### Health Checks
- `GET /health` - Service health status
- Database connectivity
- Redis connectivity
- Kafka connectivity
- ClickHouse connectivity

### Metrics
- Resource utilization rate
- Resource allocation efficiency
- Resource request fulfillment time
- Resource conflict resolution time
- Resource planning accuracy

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
- Fleet-level permissions
- Resource-level permissions
- Allocation permissions
- Planning permissions

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
