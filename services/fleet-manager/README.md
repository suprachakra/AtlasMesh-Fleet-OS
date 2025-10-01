# Fleet Manager

> **TL;DR:** Core fleet management service for vehicle operations, trip coordination, and command dispatch

## 📊 **Architecture Overview**

### 🏗️ **Where it fits** - System Context
```mermaid
graph TB
    subgraph "External Users"
        Operator[👤 Fleet Operator]
        SafetyOp[🚨 Safety Operator]
    end
    
    subgraph "AtlasMesh Fleet OS"
        UI[🖥️ Control Center UI]
        Gateway[🚪 API Gateway]
        FleetManager[🚛 Fleet Manager]
        Auth[🔐 Auth Service]
        Policy[📋 Policy Engine]
        VGW[🌐 Vehicle Gateway]
    end
    
    subgraph "External Systems"
        Vehicles[🚗 Vehicle Fleet]
        Database[(🗄️ PostgreSQL)]
    end
    
    Operator --> UI
    SafetyOp --> UI
    UI --> Gateway
    Gateway --> FleetManager
    FleetManager --> Auth
    FleetManager --> Policy
    FleetManager --> VGW
    VGW --> Vehicles
    FleetManager --> Database
    
    classDef service fill:#e1f5fe,stroke:#01579b,stroke-width:2px
    classDef external fill:#f3e5f5,stroke:#4a148c,stroke-width:2px
    classDef data fill:#e8f5e8,stroke:#1b5e20,stroke-width:2px
    
    class FleetManager service
    class Operator,SafetyOp,UI,Vehicles external
    class Database data
```

### ⚡ **How it talks** - Trip Creation Flow
```mermaid
sequenceDiagram
    autonumber
    actor Operator
    participant API as Fleet Manager API
    participant Core as Business Logic
    participant DB as PostgreSQL
    participant VGW as Vehicle Gateway
    
    Operator->>API: POST /api/v1/trips
    Note right of Operator: Create new trip request
    
    API->>Core: validateTripRequest()
    Note right of API: Business rules validation
    
    Core->>DB: SELECT available_vehicles
    Note right of Core: Find suitable vehicles
    
    Core->>DB: INSERT trip, UPDATE vehicle_status
    Note right of Core: Persist trip and assign vehicle
    
    Core->>VGW: sendVehicleCommand(trip_assignment)
    Note right of Core: Notify vehicle of new trip
    
    VGW-->>Core: Command acknowledged
    Core-->>API: Trip created successfully
    API-->>Operator: 201 + trip_id
    
    Note over Operator,VGW: End-to-end latency: <200ms
```

### 🗄️ **What it owns** - Data Model
```mermaid
erDiagram
    ORGANIZATIONS {
        uuid organization_id PK
        varchar name
        varchar sector
        jsonb configuration
        varchar status
        timestamptz created_at
    }
    
    FLEETS {
        uuid fleet_id PK
        uuid organization_id FK
        varchar name
        varchar fleet_type
        integer max_vehicles
        varchar status
        geometry service_area
    }
    
    VEHICLES {
        uuid vehicle_id PK
        uuid fleet_id FK
        varchar asset_tag UK
        varchar vin UK
        varchar operational_status
        geometry current_location
        decimal health_score
        timestamptz last_seen
    }
    
    TRIPS {
        uuid trip_id PK
        uuid vehicle_id FK
        uuid fleet_id FK
        varchar status
        geometry origin
        geometry destination
        timestamptz scheduled_start
        timestamptz actual_start
    }
    
    ORGANIZATIONS ||--o{ FLEETS : "owns"
    FLEETS ||--o{ VEHICLES : "contains"
    VEHICLES ||--o{ TRIPS : "performs"
```

## 🔗 **API Contracts**

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/api/v1/fleets` | `GET` | List all fleets |
| `/api/v1/trips` | `POST` | Create new trip |
| `/api/v1/vehicles/{id}/commands` | `POST` | Send vehicle command |

## 🚀 **Quick Start**

```bash
# Start service locally
make dev.fleet-manager

# Run tests
make test.fleet-manager

# Health check
curl http://localhost:8080/health
```

## 📈 **SLOs & Performance**

| Metric | Target | Current |
|--------|--------|---------|
| **P95 Latency** | 120ms | 95ms ✅ |
| **Availability** | 99.9% | 99.95% ✅ |
| **Throughput** | 1000 req/s | 850 req/s ⚠️ |

## 🛡️ **Security & Compliance**

- **Authentication:** JWT tokens via Auth Service
- **Authorization:** RBAC/ABAC via Policy Engine  
- **Compliance:** UAE AV regulations, ISO 26262

## 📊 **Monitoring**

- **Health Dashboard:** [Fleet Manager Health](https://grafana.atlasmesh.com/d/fleet-manager)
- **Logs:** `kubectl logs -f deployment/fleet-manager -n fleet-os`

---

**🎯 Owner:** Fleet Platform Team | **📧 Contact:** fleet-team@atlasmesh.com