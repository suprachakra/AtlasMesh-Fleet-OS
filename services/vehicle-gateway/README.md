# Vehicle Gateway

> **TL;DR:** Real-time vehicle communication gateway for bidirectional command and telemetry exchange

## 📊 **Architecture Overview**

### 🌐 **Where it fits** - Communication Hub
```mermaid
graph TB
    subgraph "Cloud Services"
        FleetManager[🚛 Fleet Manager]
        Auth[🔐 Auth Service]
        TelemetryIngest[📊 Telemetry Ingest]
    end
    
    subgraph "Vehicle Gateway"
        WSServer[🔌 WebSocket Server]
        ConnMgr[📋 Connection Manager]
        MsgRouter[📨 Message Router]
        Cache[(🗄️ Redis Cache)]
    end
    
    subgraph "Edge Devices"
        Vehicle1[🚗 Vehicle AV-001]
        Vehicle2[🚗 Vehicle AV-002]
        VehicleN[🚗 Vehicle AV-xxx]
    end
    
    subgraph "Event Streaming"
        Kafka[📨 Kafka Event Bus]
    end
    
    FleetManager --> WSServer
    Auth --> WSServer
    WSServer --> ConnMgr
    ConnMgr --> MsgRouter
    MsgRouter --> Cache
    
    Vehicle1 <--> WSServer
    Vehicle2 <--> WSServer
    VehicleN <--> WSServer
    
    MsgRouter --> Kafka
    Kafka --> TelemetryIngest
    
    classDef gateway fill:#e1f5fe,stroke:#01579b,stroke-width:3px
    classDef cloud fill:#f3e5f5,stroke:#4a148c,stroke-width:2px
    classDef vehicle fill:#e8f5e8,stroke:#1b5e20,stroke-width:2px
    classDef data fill:#fff3e0,stroke:#f57c00,stroke-width:2px
    
    class WSServer,ConnMgr,MsgRouter gateway
    class FleetManager,Auth,TelemetryIngest cloud
    class Vehicle1,Vehicle2,VehicleN vehicle
    class Cache,Kafka data
```

### ⚡ **How it talks** - Emergency Stop Command
```mermaid
sequenceDiagram
    autonumber
    actor SafetyOp as 🚨 Safety Operator
    participant UI as 🖥️ Control Center
    participant GW as 🌐 Vehicle Gateway
    participant Cache as 🗄️ Redis Cache
    participant Vehicle as 🚗 Vehicle Agent
    participant Kafka as 📨 Event Bus
    
    SafetyOp->>UI: Click EMERGENCY STOP
    Note right of SafetyOp: Critical safety action
    
    UI->>GW: WebSocket: emergency_stop_command
    Note right of UI: Real-time command dispatch
    
    GW->>Cache: HGET vehicle:status
    Note right of GW: Verify vehicle connection
    
    GW->>Vehicle: WebSocket: EMERGENCY_STOP
    Note right of GW: <50ms safety command
    
    Vehicle->>GW: Command acknowledged
    Note right of Vehicle: Vehicle executing stop
    
    GW->>Kafka: Publish emergency_event
    Note right of GW: Audit trail for compliance
    
    GW->>UI: Command completed
    UI->>SafetyOp: ✅ Vehicle stopped
    
    Note over SafetyOp,Kafka: End-to-end latency: <100ms
```

### 📡 **What it owns** - Telemetry Pipeline
```mermaid
flowchart TB
    subgraph "Vehicle Telemetry Sources"
        GPS[📍 GPS Location]
        Sensors[🔧 Vehicle Sensors]
        Status[⚡ System Status]
        Camera[📷 Camera Data]
    end
    
    subgraph "Vehicle Gateway Processing"
        WSEndpoint[🔌 WebSocket /ws/{vehicle_id}]
        Validator[✅ Message Validator]
        Buffer[📦 Telemetry Buffer]
        Enricher[🏷️ Data Enricher]
    end
    
    subgraph "Data Routing"
        HotPath[🔥 Hot Path - Real-time]
        ColdPath[❄️ Cold Path - Batch]
        EventBus[📨 Kafka Topics]
    end
    
    subgraph "Downstream Consumers"
        Analytics[📊 Real-time Analytics]
        Storage[🗄️ Data Warehouse]
        Alerts[🚨 Alert Engine]
        ML[🤖 ML Pipeline]
    end
    
    GPS --> WSEndpoint
    Sensors --> WSEndpoint
    Status --> WSEndpoint
    Camera --> WSEndpoint
    
    WSEndpoint --> Validator
    Validator --> Buffer
    Buffer --> Enricher
    
    Enricher --> HotPath
    Enricher --> ColdPath
    
    HotPath --> EventBus
    ColdPath --> EventBus
    
    EventBus --> Analytics
    EventBus --> Storage
    EventBus --> Alerts
    EventBus --> ML
    
    classDef source fill:#e8f5e8,stroke:#1b5e20,stroke-width:2px
    classDef processing fill:#e1f5fe,stroke:#01579b,stroke-width:2px
    classDef routing fill:#fff3e0,stroke:#f57c00,stroke-width:2px
    classDef consumer fill:#f3e5f5,stroke:#4a148c,stroke-width:2px
    
    class GPS,Sensors,Status,Camera source
    class WSEndpoint,Validator,Buffer,Enricher processing
    class HotPath,ColdPath,EventBus routing
    class Analytics,Storage,Alerts,ML consumer
```

## 🔗 **API Contracts**

| Type | Endpoint | Description |
|------|----------|-------------|
| **WebSocket** | `/ws/{vehicle_id}` | Real-time vehicle communication |
| **REST** | `/api/v1/connections` | Active connection management |
| **Events** | `vehicle.telemetry.*` | Kafka telemetry topics |

## 🚀 **Quick Start**

```bash
# Start service locally
make dev.vehicle-gateway

# Test WebSocket connection
wscat -c ws://localhost:8080/ws/test-vehicle-001

# Health check
curl http://localhost:8080/health
```

## 📈 **SLOs & Performance**

| Metric | Target | Current |
|--------|--------|---------|
| **P95 Latency** | 50ms | 35ms ✅ |
| **Availability** | 99.95% | 99.98% ✅ |
| **Concurrent Connections** | 10K | 8.5K ✅ |
| **Throughput** | 10K msg/s | 8.5K msg/s ✅ |

## 🛡️ **Security & Monitoring**

- **Authentication:** JWT token validation for vehicle connections
- **Encryption:** WSS (WebSocket Secure) with TLS 1.3
- **Monitoring:** [Vehicle Gateway Dashboard](https://grafana.atlasmesh.com/d/vehicle-gateway)

## 🆘 **Troubleshooting**

| Issue | Solution |
|-------|----------|
| Connection drops | Check network stability, review heartbeat config |
| High latency | Verify Redis performance, check buffer sizes |
| Memory leaks | Review connection cleanup, check goroutine leaks |

---

**🎯 Owner:** Edge Platform Team | **📧 Contact:** edge-team@atlasmesh.com