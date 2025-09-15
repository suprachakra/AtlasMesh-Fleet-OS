# Services

Core FMS microservices for the AtlasMesh Fleet Operating System.

## Architecture

Each service follows the same patterns:
- **API-first**: OpenAPI 3.0 specifications
- **Event-driven**: Event sourcing with CQRS
- **Observability**: Structured logging, metrics, tracing
- **Resilience**: Circuit breakers, retries, timeouts

## Core Services

### Core Platform Services
- `api-gateway/` - Main API gateway with authentication and routing
- `dispatch-service/` - Trip dispatch and vehicle assignment  
- `routing-service/` - Route planning and optimization
- `policy-service/` - Policy engine with ODD and compliance rules
- `event-bus/` - Event streaming and message routing

### Vehicle & Fleet Management
- `fleet-manager/` - Fleet status and vehicle lifecycle
- `vehicle-gateway/` - Vehicle communication and telemetry
- `energy-service/` - Battery/fuel management and charging optimization
- `ota-service/` - Over-the-air updates and configuration management

### Operations & Safety
- `safety-monitor/` - Safety case validation and incident tracking
- `tele-assist-service/` - Remote assistance and human oversight
- `weather-service/` - Weather data fusion and extreme condition handling
- `compliance-service/` - Regulatory compliance and audit trails

### Data & Analytics
- `analytics-service/` - Fleet KPIs and performance metrics
- `map-service/` - Map data management with provenance tracking
- `ml-pipeline/` - Machine learning training and inference
- `data-warehouse/` - Data storage and lineage tracking

## Development Guidelines

### Service Structure
```
service-name/
├── api/                 # OpenAPI specs
├── src/                 # Source code
├── tests/               # Unit and integration tests
├── deploy/              # Kubernetes manifests
├── docs/                # Service documentation
└── package.json         # Dependencies and scripts
```

### Standards
- **Language**: TypeScript (Node.js) or Go for high-performance services
- **Database**: PostgreSQL primary, Redis for caching
- **Messaging**: Apache Kafka for events, gRPC for synchronous calls
- **Monitoring**: Prometheus metrics, Jaeger tracing

## Getting Started

```bash
# Start all services in development mode
make dev

# Build all services
make build

# Run service tests
make test

# Generate API documentation
nx run-many --target=api-docs --all
```

## Service Dependencies

Core dependency graph:
- `api-gateway` → all services
- `dispatch-service` → `routing-service`, `policy-service`, `fleet-manager`
- `routing-service` → `map-service`, `weather-service`
- `policy-service` → `compliance-service`
- `fleet-manager` → `vehicle-gateway`, `energy-service`

## Security

All services implement:
- mTLS for inter-service communication
- JWT-based authentication
- RBAC authorization
- Request/response signing for critical operations
- Audit logging for all state changes
