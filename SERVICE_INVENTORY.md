# AtlasMesh Fleet OS - Complete Service Inventory

## 🎯 **72 Services Categorized by Layer & Function**

### **TIER 1: Core Business Services (8 services)**
| Service | Port | Purpose | Dependencies |
|---------|------|---------|--------------|
| `api-gateway` | 8080 | Main API entry point, routing, auth | Redis, Auth Service |
| `fleet-manager` | 8081 | Fleet operations, vehicle assignment | PostgreSQL, Event Bus |
| `vehicle-gateway` | 8082 | Vehicle communication hub | MQTT, Kafka, Redis |
| `mission-management` | 8083 | Mission and trip lifecycle management | PostgreSQL, Event Bus |
| `dispatch-service` | 8084 | Vehicle dispatch logic | Fleet Manager, Policy Engine |
| `routing-service` | 8085 | Route calculation and optimization | Map Data Contract |
| `policy-engine` | 8086 | Business rules and compliance (OPA) | PostgreSQL, Redis |
| `auth-service` | 8087 | Authentication and authorization | PostgreSQL, Vault |

### **TIER 2: Data & Analytics Services (8 services)**
| Service | Port | Purpose | Dependencies |
|---------|------|---------|--------------|
| `telemetry-ingestion` | 8090 | Real-time telemetry collection | Kafka, ClickHouse |
| `telemetry-ingestion` | 8091 | Telemetry processing pipeline | Kafka, TimescaleDB |
| `analytics-service` | 8092 | Fleet analytics and reporting | ClickHouse, PostgreSQL |
| `data-lineage` | 8093 | Data provenance tracking | Neo4j, Event Bus |
| `feature-store-registry` | 8094 | ML feature management | PostgreSQL, MinIO |
| `telemetry-lakehouse` | 8095 | Hot/cold data management | ClickHouse, MinIO |
| `sensor-data-collector` | 8096 | Edge sensor data collection | Kafka, Redis |
| `predictive-maintenance` | 8097 | RUL prediction and work orders | Feature Store, ML Pipeline |

### **TIER 3: Integration & External Services (7 services)**
| Service | Port | Purpose | Dependencies |
|---------|------|---------|--------------|
| `uae-government-integration` | 8100 | ADTA, emergency services APIs | External APIs |
| `business-systems-connector` | 8101 | SAP orders, Oracle WMS, IBM Maximo, Salesforce CRM | External Business Systems |
| `erp-financial-connector` | 8102 | ERP financial, HR, procurement | SAP ERP, Oracle ERP, NetSuite |
| `arabic-localization` | 8103 | RTL UI and cultural adaptations | Redis, Content DB |
| `comms-orchestration` | 8104 | Multi-path communication | LTE/5G/Wi-Fi/SAT |
| `weather-fusion` | 8105 | Multi-source weather aggregation | External Weather APIs |
| `map-data-contract` | 8106 | Map versioning and validation | External Map Providers |

### **TIER 4: Security & Compliance Services (8 services)**
| Service | Port | Purpose | Dependencies |
|---------|------|---------|--------------|
| `zero-trust-iam` | 8110 | SPIFFE/SPIRE identity management | Vault, PKI |
| `key-secret-management` | 8111 | Vault/KMS integration | HashiCorp Vault |
| `security-hardening` | 8112 | Security scanning and hardening | Security Tools |
| `auditability` | 8113 | Cryptographic audit logs | PostgreSQL, Vault |
| `compliance-automation` | 8114 | Regulatory reporting | Evidence Engine |
| `evidence-engine` | 8115 | Safety case and audit trails | MinIO, PostgreSQL |
| `purpose-binding-residency` | 8116 | Data privacy controls | PostgreSQL, Policy Engine |
| `tenant-entitlements` | 8117 | Multi-tenant access control | Auth Service, PostgreSQL |

### **TIER 5: ML & AI Services (4 services)**
| Service | Port | Purpose | Dependencies |
|---------|------|---------|--------------|
| `ml-pipeline` | 8120 | ML model training and inference | Feature Store, MinIO |
| `digital-twin` | 8121 | CARLA/Gazebo simulation | Simulation Cluster |
| `scenario-testing` | 8122 | Testing & validation simulation | CARLA, Gazebo, scenario banking, fault injection |
| `feature-flags` | 8123 | A/B testing and feature rollouts | Redis, PostgreSQL |

### **TIER 6: Operations & Infrastructure Services (12 services)**
| Service | Port | Purpose | Dependencies |
|---------|------|---------|--------------|
| `monitoring-alerting` | 8130 | SLO monitoring and alerting | Prometheus, Grafana |
| `observability` | 8131 | Distributed tracing and logging | Jaeger, OpenTelemetry |
| `event-bus` | 8132 | Event streaming and messaging | Kafka, NATS |
| `cache-manager` | 8133 | Distributed caching | Redis Cluster |
| `cost-optimization` | 8134 | FinOps and resource optimization | Prometheus, Cloud APIs |
| `chaos-engineering` | 8135 | Resilience testing | Chaos Mesh, Kubernetes |
| `garage-tools` | 8136 | Vehicle provisioning and maintenance | OTA Manager, Depot Systems |
| `degraded-modes` | 8137 | Graceful degradation management | Policy Engine, Event Bus |
| `time-sync-calibration` | 8138 | PTP/GNSS time synchronization | Hardware Interfaces |
| `oncall-runbooks` | 8139 | Incident response automation | PagerDuty, Slack |
| `sector-overlays` | 8140 | Sector-specific customizations | Policy Engine, UI Tokens |
| `schema-registry` | 8141 | Avro schema management | Kafka, Event Bus |

## 🏗️ **Service Dependencies Map**

### **Critical Path Services (Must Start First)**
1. **Infrastructure**: PostgreSQL, Redis, Kafka, Prometheus
2. **Core Auth**: `auth-service`, `key-secret-management`
3. **API Layer**: `api-gateway`
4. **Business Core**: `fleet-manager`, `vehicle-gateway`, `policy-engine`

### **Service Startup Order**
```
Level 1: Infrastructure (PostgreSQL, Redis, Kafka, Prometheus, Grafana, Jaeger)
Level 2: Auth & Security (auth-service, key-secret-management, zero-trust-iam)
Level 3: Core Services (api-gateway, fleet-manager, vehicle-gateway, policy-engine)
Level 4: Business Services (mission-management, dispatch-service, routing-service)
Level 5: Data Services (telemetry-ingestion, analytics-service, data-lineage)
Level 6: Integration Services (uae-government-integration, erp-wms-adapters)
Level 7: ML & AI Services (ml-pipeline, digital-twin, predictive-maintenance)
Level 8: Operations Services (monitoring-alerting, observability, chaos-engineering)
```

## 🔍 **Service Health Check Endpoints**

All services should implement:
- `GET /health` - Basic health check
- `GET /health/ready` - Readiness probe
- `GET /health/live` - Liveness probe
- `GET /metrics` - Prometheus metrics
- `GET /debug/vars` - Runtime variables (dev only)

## 🚀 **Quick Start Commands**

### **Start Infrastructure**
```bash
docker-compose -f infrastructure/docker-compose.yml up -d
```

### **Start Core Services**
```bash
# Terminal 1: Auth Service
cd services/auth-service && go run cmd/main.go

# Terminal 2: API Gateway  
cd services/api-gateway && go run cmd/main.go

# Terminal 3: Fleet Manager
cd services/fleet-manager && go run cmd/main.go

# Terminal 4: Vehicle Gateway
cd services/vehicle-gateway && go run cmd/main.go
```

### **Validate System**
```bash
# Check all service health
make health-check-all

# Run smoke tests
python testing/smoke/smoke-tests.py --environment development

# Check service mesh
make service-mesh-status
```

## 📊 **Resource Requirements**

### **Development Environment**
- **CPU**: 8 cores minimum (16 cores recommended)
- **Memory**: 16GB minimum (32GB recommended)  
- **Storage**: 100GB SSD minimum
- **Network**: Stable internet for external integrations

### **Production Environment**
- **Kubernetes Cluster**: 3+ nodes, 16 cores each, 64GB RAM
- **Database**: PostgreSQL cluster with read replicas
- **Message Bus**: Kafka cluster (3+ brokers)
- **Storage**: 1TB+ for telemetry data, evidence, and logs
- **Monitoring**: Dedicated monitoring stack

## 🎯 **Next Steps**

1. **Create service-specific Makefiles** for build/test/run
2. **Set up Docker Compose** for local development
3. **Implement health checks** for all services
4. **Create integration tests** between services
5. **Set up CI/CD pipeline** for automated deployment
6. **Configure monitoring** and alerting for all services
7. **Document service-specific troubleshooting** guides
