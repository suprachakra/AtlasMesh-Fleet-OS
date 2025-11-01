# Oracle WMS Adapter

## Overview

The Oracle WMS adapter provides integration with Oracle's Warehouse Management System for the AtlasMesh Fleet OS. This adapter enables seamless communication between the fleet management system and Oracle WMS.

## Features

- **Task Synchronization**: Real-time synchronization of warehouse tasks from Oracle WMS
- **Inventory Management**: Continuous inventory tracking and updates
- **Resource Management**: Warehouse resource status monitoring
- **RESTful API**: Modern REST API for external integrations
- **Health Monitoring**: Built-in health checks and metrics
- **Configurable**: Flexible configuration for different warehouse setups

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    Oracle WMS Adapter                      │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐        │
│  │   Task      │  │  Inventory  │  │  Resource   │        │
│  │  Manager    │  │  Manager    │  │  Manager    │        │
│  └─────────────┘  └─────────────┘  └─────────────┘        │
│         │               │               │                  │
│         └───────────────┼───────────────┘                  │
│                         │                                  │
│  ┌─────────────────────────────────────────────────────────┐ │
│  │              Oracle WMS Client                         │ │
│  └─────────────────────────────────────────────────────────┘ │
│                         │                                  │
│  ┌─────────────────────────────────────────────────────────┐ │
│  │                HTTP Server                              │ │
│  └─────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────┘
```

## Configuration

The adapter can be configured using environment variables or a YAML configuration file:

### Environment Variables

```bash
# Server Configuration
SERVER_HOST=0.0.0.0
SERVER_PORT=8080

# Database Configuration
DB_HOST=localhost
DB_PORT=5432
DB_USER=atlasmesh
DB_PASSWORD=atlasmesh123
DB_NAME=atlasmesh_oracle

# Oracle WMS Configuration
ORACLE_WMS_HOST=localhost
ORACLE_WMS_PORT=1521
ORACLE_WMS_USER=admin
ORACLE_WMS_PASSWORD=admin123
ORACLE_WMS_WAREHOUSE_ID=WH001
```

### YAML Configuration

```yaml
server:
  host: "0.0.0.0"
  port: 8080

database:
  host: "localhost"
  port: 5432
  user: "atlasmesh"
  password: "atlasmesh123"
  dbname: "atlasmesh_oracle"

oracle_wms:
  host: "localhost"
  port: 1521
  user: "admin"
  password: "admin123"
  warehouse_id: "WH001"
```

## API Endpoints

### Health Check
- `GET /health` - Service health status

### Tasks
- `GET /api/v1/tasks` - Get all tasks
- `GET /api/v1/tasks/:id` - Get specific task
- `PUT /api/v1/tasks/:id/status` - Update task status

### Inventory
- `GET /api/v1/inventory` - Get inventory items

### Resources
- `GET /api/v1/resources` - Get warehouse resources

## Deployment

### Docker

```bash
# Build image
docker build -t atlasmesh/oracle-wms-adapter .

# Run container
docker run -p 8080:8080 atlasmesh/oracle-wms-adapter
```

### Docker Compose

```bash
# Start all services
docker-compose up -d

# View logs
docker-compose logs -f

# Stop services
docker-compose down
```

### Kubernetes

```bash
# Deploy to Kubernetes
kubectl apply -f k8s/

# View logs
kubectl logs -f deployment/oracle-wms-adapter -n atlasmesh

# Delete deployment
kubectl delete -f k8s/
```

## Development

### Prerequisites

- Go 1.21+
- Docker
- PostgreSQL 15+

### Local Development

```bash
# Install dependencies
go mod download

# Run tests
go test ./...

# Run with hot reload
make dev

# Build
make build

# Run
make run
```

### Testing

```bash
# Run all tests
make test

# Run tests with coverage
make test-coverage

# Run specific test
go test ./internal/task/...
```

## Monitoring

The adapter exposes Prometheus metrics at `/metrics`:

- `oracle_wms_tasks_total` - Total number of tasks processed
- `oracle_wms_tasks_processed_total` - Successfully processed tasks
- `oracle_wms_tasks_failed_total` - Failed tasks
- `oracle_wms_task_duration_seconds` - Task processing duration
- `oracle_wms_active_tasks` - Currently active tasks
- `oracle_wms_inventory_items` - Number of inventory items
- `oracle_wms_resources_total` - Total warehouse resources

## Logging

The adapter uses structured JSON logging with the following levels:
- `debug` - Detailed debug information
- `info` - General information
- `warn` - Warning messages
- `error` - Error messages

## Security

- Authentication via Oracle WMS credentials
- HTTPS support for secure communication
- Input validation and sanitization
- Rate limiting to prevent abuse

## Troubleshooting

### Common Issues

1. **Connection to Oracle WMS fails**
   - Check network connectivity
   - Verify credentials
   - Check firewall settings

2. **Database connection issues**
   - Verify PostgreSQL is running
   - Check connection parameters
   - Ensure database exists

3. **Task synchronization problems**
   - Check Oracle WMS API status
   - Verify warehouse configuration
   - Review logs for errors

### Logs

```bash
# View application logs
docker logs oracle-wms-adapter

# View with follow
docker logs -f oracle-wms-adapter

# View specific log level
docker logs oracle-wms-adapter 2>&1 | grep ERROR
```

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Add tests
5. Submit a pull request

## License

This project is licensed under the MIT License - see the LICENSE file for details.
