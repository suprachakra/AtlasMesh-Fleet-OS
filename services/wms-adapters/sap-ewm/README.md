# SAP EWM Adapter

> **SAP Extended Warehouse Management (EWM) integration adapter for AtlasMesh Fleet OS**

## 📋 **Overview**

The SAP EWM Adapter provides seamless integration between AtlasMesh Fleet OS and SAP Extended Warehouse Management systems. This adapter enables real-time task synchronization, inventory location mapping, and pick/place confirmations for autonomous warehouse operations.

## 🎯 **Features**

### **Core Integration**
- **Real-time Task Synchronization**: Automatic synchronization of warehouse tasks from SAP EWM
- **Inventory Location Mapping**: Dynamic mapping of inventory locations to physical coordinates
- **Pick/Place Confirmations**: Automated confirmation of completed warehouse operations
- **Error Handling & Retry Logic**: Robust error handling with automatic retry mechanisms

### **SAP EWM Specific Features**
- **Task Management**: Create, update, and complete warehouse tasks
- **Resource Management**: Manage warehouse resources and equipment
- **Inventory Tracking**: Real-time inventory level monitoring
- **Performance Analytics**: Warehouse operation performance metrics

## 🏗️ **Architecture**

```
┌─────────────────────────────────────────────────────────────┐
│                    AtlasMesh Fleet OS                      │
├─────────────────────────────────────────────────────────────┤
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐        │
│  │   Vehicle   │  │   Fleet     │  │   Dispatch  │        │
│  │    Agent    │  │  Manager    │  │   Service   │        │
│  └─────────────┘  └─────────────┘  └─────────────┘        │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                 SAP EWM Adapter                            │
├─────────────────────────────────────────────────────────────┤
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐        │
│  │   Task      │  │  Inventory  │  │  Resource   │        │
│  │  Manager    │  │  Manager    │  │  Manager    │        │
│  └─────────────┘  └─────────────┘  └─────────────┘        │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                    SAP EWM System                          │
├─────────────────────────────────────────────────────────────┤
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐        │
│  │   Task      │  │  Inventory  │  │  Warehouse  │        │
│  │  Management │  │  Management │  │  Management │        │
│  └─────────────┘  └─────────────┘  └─────────────┘        │
└─────────────────────────────────────────────────────────────┘
```

## 🔧 **Configuration**

### **Environment Variables**
```bash
# SAP EWM Connection
SAP_EWM_HOST=your-sap-ewm-host.com
SAP_EWM_PORT=8000
SAP_EWM_CLIENT=100
SAP_EWM_USER=your-username
SAP_EWM_PASSWORD=your-password

# Authentication
SAP_EWM_AUTH_TYPE=basic  # basic, oauth2, certificate
SAP_EWM_SSL_ENABLED=true
SAP_EWM_SSL_VERIFY=true

# API Configuration
SAP_EWM_API_VERSION=v1
SAP_EWM_TIMEOUT=30s
SAP_EWM_RETRY_ATTEMPTS=3
SAP_EWM_RETRY_DELAY=5s

# Polling Configuration
SAP_EWM_POLL_INTERVAL=5s
SAP_EWM_BATCH_SIZE=100
SAP_EWM_QUEUE_SIZE=1000
```

### **Configuration File**
```yaml
# config/sap-ewm.yaml
sap_ewm:
  connection:
    host: "your-sap-ewm-host.com"
    port: 8000
    client: 100
    user: "your-username"
    password: "your-password"
    auth_type: "basic"
    ssl_enabled: true
    ssl_verify: true
  
  api:
    version: "v1"
    timeout: "30s"
    retry_attempts: 3
    retry_delay: "5s"
  
  polling:
    interval: "5s"
    batch_size: 100
    queue_size: 1000
  
  warehouse:
    warehouse_id: "WH001"
    zones:
      - zone_id: "ZONE_A"
        coordinates:
          x_min: 0
          x_max: 100
          y_min: 0
          y_max: 50
      - zone_id: "ZONE_B"
        coordinates:
          x_min: 100
          x_max: 200
          y_min: 0
          y_max: 50
```

## 📡 **API Endpoints**

### **Task Management**
- `GET /api/v1/tasks` - Get all warehouse tasks
- `GET /api/v1/tasks/{task_id}` - Get specific task
- `POST /api/v1/tasks` - Create new task
- `PUT /api/v1/tasks/{task_id}` - Update task
- `DELETE /api/v1/tasks/{task_id}` - Delete task
- `POST /api/v1/tasks/{task_id}/complete` - Complete task

### **Inventory Management**
- `GET /api/v1/inventory` - Get inventory levels
- `GET /api/v1/inventory/{location}` - Get inventory at location
- `POST /api/v1/inventory/update` - Update inventory levels
- `GET /api/v1/locations` - Get all warehouse locations

### **Resource Management**
- `GET /api/v1/resources` - Get warehouse resources
- `GET /api/v1/resources/{resource_id}` - Get specific resource
- `POST /api/v1/resources/{resource_id}/assign` - Assign resource to task
- `POST /api/v1/resources/{resource_id}/release` - Release resource

## 🔄 **Data Flow**

### **Task Synchronization**
1. **Polling**: Adapter polls SAP EWM for new tasks every 5 seconds
2. **Processing**: New tasks are processed and validated
3. **Mapping**: Task locations are mapped to physical coordinates
4. **Dispatch**: Tasks are sent to AtlasMesh Dispatch Service
5. **Execution**: Vehicles execute tasks autonomously
6. **Confirmation**: Task completion is confirmed back to SAP EWM

### **Inventory Updates**
1. **Monitoring**: Adapter monitors inventory level changes
2. **Validation**: Changes are validated against business rules
3. **Synchronization**: Updates are synchronized with AtlasMesh
4. **Notification**: Stakeholders are notified of significant changes

## 🛡️ **Security**

### **Authentication**
- **Basic Authentication**: Username/password authentication
- **OAuth2**: Token-based authentication for enhanced security
- **Certificate Authentication**: Client certificate authentication

### **Data Protection**
- **Encryption**: All data encrypted in transit (TLS 1.3)
- **Data Masking**: Sensitive data masked in logs
- **Access Control**: Role-based access control (RBAC)
- **Audit Logging**: Comprehensive audit trail

## 📊 **Monitoring & Metrics**

### **Health Metrics**
- **Connection Status**: SAP EWM connection health
- **API Response Time**: Average response time for API calls
- **Error Rate**: Percentage of failed API calls
- **Task Processing Rate**: Tasks processed per minute

### **Business Metrics**
- **Task Completion Rate**: Percentage of tasks completed successfully
- **Inventory Accuracy**: Accuracy of inventory level synchronization
- **Resource Utilization**: Warehouse resource utilization rate
- **Performance Trends**: Historical performance data

## 🚀 **Deployment**

### **Docker Deployment**
```bash
# Build image
docker build -t atlasmesh/sap-ewm-adapter:latest .

# Run container
docker run -d \
  --name sap-ewm-adapter \
  -p 8080:8080 \
  -e SAP_EWM_HOST=your-sap-ewm-host.com \
  -e SAP_EWM_USER=your-username \
  -e SAP_EWM_PASSWORD=your-password \
  atlasmesh/sap-ewm-adapter:latest
```

### **Kubernetes Deployment**
```yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: sap-ewm-adapter
spec:
  replicas: 2
  selector:
    matchLabels:
      app: sap-ewm-adapter
  template:
    metadata:
      labels:
        app: sap-ewm-adapter
    spec:
      containers:
      - name: sap-ewm-adapter
        image: atlasmesh/sap-ewm-adapter:latest
        ports:
        - containerPort: 8080
        env:
        - name: SAP_EWM_HOST
          value: "your-sap-ewm-host.com"
        - name: SAP_EWM_USER
          valueFrom:
            secretKeyRef:
              name: sap-ewm-secret
              key: username
        - name: SAP_EWM_PASSWORD
          valueFrom:
            secretKeyRef:
              name: sap-ewm-secret
              key: password
```

## 🧪 **Testing**

### **Unit Tests**
```bash
# Run unit tests
go test ./internal/...

# Run with coverage
go test -cover ./internal/...
```

### **Integration Tests**
```bash
# Run integration tests
go test ./tests/integration/...

# Run with SAP EWM mock
go test -tags=integration ./tests/integration/...
```

### **Load Tests**
```bash
# Run load tests
go test ./tests/load/...

# Run with specific load
go test -load=1000 ./tests/load/...
```

## 📚 **Documentation**

- [API Documentation](docs/api.md)
- [Configuration Guide](docs/configuration.md)
- [Deployment Guide](docs/deployment.md)
- [Troubleshooting](docs/troubleshooting.md)
- [SAP EWM Integration](docs/sap-ewm-integration.md)

## 🤝 **Contributing**

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Add tests
5. Submit a pull request

## 📄 **License**

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🆘 **Support**

For support and questions:
- **Documentation**: [docs.atlasmesh.com](https://docs.atlasmesh.com)
- **Issues**: [GitHub Issues](https://github.com/atlasmesh/fleet-os/issues)
- **Email**: support@atlasmesh.com
- **Slack**: #sap-ewm-adapter
