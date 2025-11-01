# Fleet Performance Management Service

Performance monitoring, optimization, reporting, and benchmarking service for AtlasMesh Fleet OS.

## Overview

The Fleet Performance Management Service provides comprehensive performance monitoring and optimization capabilities for fleet operations, including performance scoring, benchmarking, reporting, and trend analysis. It enables data-driven performance optimization across fleets.

## Features

- **Performance Scoring**: Comprehensive fleet performance assessment with component analysis
- **Performance Metrics**: Track performance metrics across multiple dimensions
- **Benchmarking**: Compare fleet performance against industry standards
- **Performance Reports**: Generate comprehensive performance reports
- **Performance Alerts**: Real-time performance monitoring and alerting
- **Performance Trends**: Analyze performance trends and patterns

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│              Fleet Performance Management Service             │
├─────────────────────────────────────────────────────────────┤
│  API Layer                                                  │
│  ├── Performance Score Management                          │
│  ├── Performance Metrics Management                         │
│  ├── Benchmark Management                                  │
│  ├── Performance Report Management                         │
│  ├── Performance Alert Management                          │
│  └── Performance Trend Management                          │
├─────────────────────────────────────────────────────────────┤
│  Business Logic Layer                                       │
│  ├── Performance Score Calculator                          │
│  ├── Performance Metrics Engine                            │
│  ├── Benchmark Manager                                     │
│  ├── Performance Report Generator                          │
│  ├── Performance Alert Monitor                            │
│  └── Performance Trend Analyzer                            │
├─────────────────────────────────────────────────────────────┤
│  Data Layer                                                 │
│  ├── PostgreSQL (Performance Scores, Metrics, Reports)    │
│  ├── Redis (Caching, Sessions)                           │
│  ├── Kafka (Events, Notifications)                        │
│  └── ClickHouse (Analytics, Metrics)                      │
└─────────────────────────────────────────────────────────────┘
```

## API Endpoints

### Fleet Performance Scores
- `POST /api/v1/fleet-performance/scores` - Create performance score
- `GET /api/v1/fleet-performance/scores` - List performance scores
- `GET /api/v1/fleet-performance/scores/{fleet_id}` - Get performance score by fleet

### Fleet Performance Metrics
- `POST /api/v1/fleet-performance/metrics` - Create performance metric
- `GET /api/v1/fleet-performance/metrics` - List performance metrics
- `GET /api/v1/fleet-performance/metrics/{fleet_id}` - Get metrics by fleet

### Fleet Benchmarks
- `POST /api/v1/fleet-performance/benchmarks` - Create benchmark
- `GET /api/v1/fleet-performance/benchmarks` - List benchmarks
- `GET /api/v1/fleet-performance/benchmarks/{id}` - Get benchmark
- `PUT /api/v1/fleet-performance/benchmarks/{id}` - Update benchmark
- `DELETE /api/v1/fleet-performance/benchmarks/{id}` - Delete benchmark

### Fleet Performance Reports
- `POST /api/v1/fleet-performance/reports` - Create performance report
- `GET /api/v1/fleet-performance/reports` - List performance reports
- `GET /api/v1/fleet-performance/reports/{fleet_id}` - Get reports by fleet

### Fleet Performance Alerts
- `GET /api/v1/fleet-performance/alerts` - List performance alerts
- `POST /api/v1/fleet-performance/alerts` - Create performance alert
- `POST /api/v1/fleet-performance/alerts/{id}/resolve` - Resolve performance alert

### Fleet Performance Trends
- `POST /api/v1/fleet-performance/trends` - Create performance trend
- `GET /api/v1/fleet-performance/trends` - List performance trends
- `GET /api/v1/fleet-performance/trends/{fleet_id}` - Get trends by fleet

## Database Schema

### Fleet Performance
- `fleet_performance_scores` - Fleet performance assessments with component analysis
- `fleet_performance_metrics` - Fleet performance measurements
- `fleet_benchmarks` - Performance benchmarks and industry standards
- `fleet_performance_reports` - Comprehensive performance reports
- `fleet_performance_alerts` - Performance monitoring and alerting
- `fleet_performance_trends` - Performance trend analysis

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
docker build -t fleet-performance .

# Run container
docker run -p 8080:8080 fleet-performance
```

### Kubernetes
```bash
# Apply manifests
kubectl apply -f k8s/

# Check status
kubectl get pods -l app=fleet-performance
```

## Monitoring

### Health Checks
- `GET /health` - Service health status
- Database connectivity
- Redis connectivity
- Kafka connectivity
- ClickHouse connectivity

### Metrics
- Performance score accuracy
- Benchmark comparison accuracy
- Report generation time
- Alert response time
- Trend analysis accuracy

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
- Performance-level permissions
- Report access permissions
- Alert management permissions

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
