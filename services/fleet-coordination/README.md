# Fleet Coordination Service

Multi-fleet coordination and federation management service for AtlasMesh Fleet OS.

## Overview

The Fleet Coordination Service enables multiple fleets to work together through federations, resource sharing, and coordinated operations. It provides the foundation for enterprise-scale fleet management across different organizations and operational domains.

## Features

- **Fleet Federations**: Create and manage federations of multiple fleets
- **Resource Sharing**: Enable cross-fleet resource sharing with isolation controls
- **Communication**: Secure inter-fleet communication and coordination
- **Metrics**: Track coordination efficiency and resource utilization
- **Isolation**: Maintain fleet isolation while enabling collaboration

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    Fleet Coordination Service                │
├─────────────────────────────────────────────────────────────┤
│  API Layer                                                  │
│  ├── Fleet Federation Management                           │
│  ├── Resource Sharing Management                           │
│  ├── Communication Management                              │
│  └── Metrics & Analytics                                   │
├─────────────────────────────────────────────────────────────┤
│  Business Logic Layer                                       │
│  ├── Federation Orchestrator                              │
│  ├── Resource Allocation Engine                           │
│  ├── Communication Router                                 │
│  └── Metrics Calculator                                   │
├─────────────────────────────────────────────────────────────┤
│  Data Layer                                                 │
│  ├── PostgreSQL (Federations, Members, Sharing)          │
│  ├── Redis (Caching, Sessions)                           │
│  ├── Kafka (Events, Communication)                       │
│  └── ClickHouse (Analytics, Metrics)                      │
└─────────────────────────────────────────────────────────────┘
```

## API Endpoints

### Fleet Federations
- `POST /api/v1/fleet-coordination/federations` - Create federation
- `GET /api/v1/fleet-coordination/federations` - List federations
- `GET /api/v1/fleet-coordination/federations/{id}` - Get federation
- `PUT /api/v1/fleet-coordination/federations/{id}` - Update federation
- `DELETE /api/v1/fleet-coordination/federations/{id}` - Delete federation

### Fleet Members
- `POST /api/v1/fleet-coordination/federations/{id}/members` - Add fleet member
- `GET /api/v1/fleet-coordination/federations/{id}/members` - List fleet members
- `DELETE /api/v1/fleet-coordination/federations/{id}/members/{fleet_id}` - Remove fleet member

### Resource Sharing
- `POST /api/v1/fleet-coordination/resource-sharing` - Create resource sharing
- `GET /api/v1/fleet-coordination/resource-sharing` - List resource sharing
- `PUT /api/v1/fleet-coordination/resource-sharing/{id}` - Update resource sharing
- `DELETE /api/v1/fleet-coordination/resource-sharing/{id}` - Delete resource sharing

### Metrics
- `GET /api/v1/fleet-coordination/metrics` - Get coordination metrics
- `POST /api/v1/fleet-coordination/metrics` - Create coordination metrics

## Database Schema

### Fleet Federations
- `fleet_federations` - Federation definitions
- `fleet_federation_members` - Fleet membership in federations
- `resource_sharing` - Cross-fleet resource sharing
- `coordination_metrics` - Federation performance metrics
- `communication_logs` - Inter-fleet communication logs

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
docker build -t fleet-coordination .

# Run container
docker run -p 8080:8080 fleet-coordination
```

### Kubernetes
```bash
# Apply manifests
kubectl apply -f k8s/

# Check status
kubectl get pods -l app=fleet-coordination
```

## Monitoring

### Health Checks
- `GET /health` - Service health status
- Database connectivity
- Redis connectivity
- Kafka connectivity
- ClickHouse connectivity

### Metrics
- Federation count
- Resource sharing efficiency
- Communication latency
- Coordination success rate

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
- Federation-level permissions
- Resource sharing permissions
- Communication permissions

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
