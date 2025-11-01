# Fleet Optimization Service

Multi-objective optimization and fleet rebalancing service for AtlasMesh Fleet OS.

## Overview

The Fleet Optimization Service provides advanced optimization capabilities for fleet operations, including multi-objective optimization, dynamic rebalancing, and cost optimization. It enables data-driven decision making for fleet management across different operational scenarios.

## Features

- **Multi-Objective Optimization**: Optimize multiple objectives simultaneously (cost, efficiency, coverage)
- **Fleet Rebalancing**: Dynamic vehicle redistribution across zones and time periods
- **Cost Optimization**: Minimize operational costs while maintaining service levels
- **Performance Analytics**: Track optimization performance and ROI
- **Recommendation Engine**: Generate actionable optimization recommendations
- **Real-time Optimization**: Continuous optimization based on real-time data

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                  Fleet Optimization Service                  │
├─────────────────────────────────────────────────────────────┤
│  API Layer                                                  │
│  ├── Optimization Run Management                          │
│  ├── Fleet Rebalancing Management                          │
│  ├── Results & Recommendations                             │
│  └── Metrics & Analytics                                   │
├─────────────────────────────────────────────────────────────┤
│  Business Logic Layer                                       │
│  ├── Optimization Engine                                   │
│  ├── Rebalancing Algorithm                                 │
│  ├── Cost Calculator                                        │
│  ├── Performance Tracker                                   │
│  └── Recommendation Generator                               │
├─────────────────────────────────────────────────────────────┤
│  Data Layer                                                 │
│  ├── PostgreSQL (Optimization Runs, Results)              │
│  ├── Redis (Caching, Sessions)                           │
│  ├── Kafka (Events, Notifications)                        │
│  └── ClickHouse (Analytics, Metrics)                      │
└─────────────────────────────────────────────────────────────┘
```

## API Endpoints

### Optimization Runs
- `POST /api/v1/fleet-optimization/runs` - Create optimization run
- `GET /api/v1/fleet-optimization/runs` - List optimization runs
- `GET /api/v1/fleet-optimization/runs/{id}` - Get optimization run
- `PUT /api/v1/fleet-optimization/runs/{id}` - Update optimization run
- `DELETE /api/v1/fleet-optimization/runs/{id}` - Delete optimization run

### Fleet Rebalancing
- `POST /api/v1/fleet-optimization/rebalancing` - Create rebalancing
- `GET /api/v1/fleet-optimization/rebalancing` - List rebalancing
- `GET /api/v1/fleet-optimization/rebalancing/{id}` - Get rebalancing
- `PUT /api/v1/fleet-optimization/rebalancing/{id}` - Update rebalancing
- `DELETE /api/v1/fleet-optimization/rebalancing/{id}` - Delete rebalancing

### Optimization Results
- `GET /api/v1/fleet-optimization/runs/{id}/results` - Get optimization results
- `POST /api/v1/fleet-optimization/runs/{id}/results` - Create optimization result

### Optimization Recommendations
- `GET /api/v1/fleet-optimization/runs/{id}/recommendations` - Get recommendations
- `POST /api/v1/fleet-optimization/runs/{id}/recommendations` - Create recommendation

### Optimization Metrics
- `GET /api/v1/fleet-optimization/metrics` - Get optimization metrics
- `POST /api/v1/fleet-optimization/metrics` - Create optimization metric

## Database Schema

### Optimization Runs
- `optimization_runs` - Optimization run definitions and status
- `fleet_rebalancing` - Fleet rebalancing operations
- `optimization_results` - Optimization results and metrics
- `optimization_recommendations` - Optimization recommendations
- `optimization_metrics` - Fleet optimization metrics

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
docker build -t fleet-optimization .

# Run container
docker run -p 8080:8080 fleet-optimization
```

### Kubernetes
```bash
# Apply manifests
kubectl apply -f k8s/

# Check status
kubectl get pods -l app=fleet-optimization
```

## Monitoring

### Health Checks
- `GET /health` - Service health status
- Database connectivity
- Redis connectivity
- Kafka connectivity
- ClickHouse connectivity

### Metrics
- Optimization success rate
- Optimization execution time
- Cost reduction percentage
- Fleet efficiency improvement
- Recommendation adoption rate

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
- Optimization-level permissions
- Results access permissions
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
