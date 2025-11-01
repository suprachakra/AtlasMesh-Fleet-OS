# Manhattan SCALE Adapter

## Overview

The Manhattan SCALE adapter provides integration with Manhattan Associates' SCALE warehouse management system for the AtlasMesh Fleet OS. This adapter enables seamless communication between the fleet management system and Manhattan SCALE WMS.

## Features

- **Task Synchronization**: Real-time synchronization of warehouse tasks from Manhattan SCALE
- **Inventory Management**: Continuous inventory tracking and updates
- **Resource Management**: Warehouse resource status monitoring
- **RESTful API**: Modern REST API for external integrations
- **Health Monitoring**: Built-in health checks and metrics
- **Configurable**: Flexible configuration for different warehouse setups

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    Manhattan SCALE Adapter                  │
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
│  │              Manhattan SCALE Client                     │ │
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
DB_NAME=atlasmesh_manhattan

# Manhattan SCALE Configuration
MANHATTAN_HOST=localhost
MANHATTAN_PORT=8080
MANHATTAN_USER=admin
MANHATTAN_PASSWORD=admin123
MANHATTAN_WAREHOUSE_ID=WH001
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
  dbname: "atlasmesh_manhattan"

manhattan:
  host: "localhost"
  port: 8080
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
docker build -t atlasmesh/manhattan-scale-adapter .

# Run container
docker run -p 8080:8080 atlasmesh/manhattan-scale-adapter
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
kubectl logs -f deployment/manhattan-scale-adapter -n atlasmesh

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

- `manhattan_tasks_total` - Total number of tasks processed
- `manhattan_tasks_processed_total` - Successfully processed tasks
- `manhattan_tasks_failed_total` - Failed tasks
- `manhattan_task_duration_seconds` - Task processing duration
- `manhattan_active_tasks` - Currently active tasks
- `manhattan_inventory_items` - Number of inventory items
- `manhattan_resources_total` - Total warehouse resources

## Logging

The adapter uses structured JSON logging with the following levels:
- `debug` - Detailed debug information
- `info` - General information
- `warn` - Warning messages
- `error` - Error messages

## Security

- Authentication via Manhattan SCALE credentials
- HTTPS support for secure communication
- Input validation and sanitization
- Rate limiting to prevent abuse

## Troubleshooting

### Common Issues

1. **Connection to Manhattan SCALE fails**
   - Check network connectivity
   - Verify credentials
   - Check firewall settings

2. **Database connection issues**
   - Verify PostgreSQL is running
   - Check connection parameters
   - Ensure database exists

3. **Task synchronization problems**
   - Check Manhattan SCALE API status
   - Verify warehouse configuration
   - Review logs for errors

### Logs

```bash
# View application logs
docker logs manhattan-scale-adapter

# View with follow
docker logs -f manhattan-scale-adapter

# View specific log level
docker logs manhattan-scale-adapter 2>&1 | grep ERROR
```

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Add tests
5. Submit a pull request

## License

This project is licensed under the MIT License - see the LICENSE file for details.
