# AtlasMesh Fleet OS — Architecture


## 📋 Table of Contents

<div align="center">

| 🎯 **[Executive Summary](#-executive-summary-qualified-agnosticism)** | 🏗️ **[System Context](#%EF%B8%8F-1-system-context-c4-model)** | 🔧 **[Core Components](#2-core-components)** | 📊 **[Data Architecture](#3-data-architecture)** |
|:---:|:---:|:---:|:---:|
| 🚀 **[Deployment](#4-deployment-architecture)** | 🔒 **[Security](#5-security-architecture)** | 📈 **[Monitoring](#6-monitoring--observability)** | 🧪 **[Testing](#7-testing-strategy)** |


</div>

---

## 🎯 Executive Summary: Qualified Agnosticism

AtlasMesh Fleet OS implements **"qualified agnosticism"** - a pragmatic approach to vehicle-agnostic, sector-agnostic, and platform-agnostic autonomous fleet operations. Rather than claiming universal compatibility, we achieve **bounded agnosticism** through:

- **🚗 Vehicle-Agnostic**: Class/model-bounded with certified profiles (≤5% code delta)
- **🏢 Sector-Agnostic**: Policy overlays targeting ≥90% code reuse
- **☁️ Platform-Agnostic**: Contract-driven infrastructure with conformance testing

### 🎯 **Agnosticism Reality Check**

<div align="center">

| 🚙 **Vehicle-Agnostic** | 🏭 **Sector-Agnostic** | ☁️ **Platform-Agnostic** |
|:---:|:---:|:---:|
| **What "Yes" Looks Like** | **What "Yes" Looks Like** | **What "Yes" Looks Like** |
| Shared core; per-model profiles; HiL & track re-cert | Shared backbone; policy/UX overlays; evidence mappers | K8s-first; provider adapters; conformance suite |
| **Feasibility: Medium-High** | **Feasibility: High** | **Feasibility: High** |
| **Implementation: Vehicle HAL + Profile System** | **Implementation: Policy Engine + Sector Overlays** | **Implementation: Infrastructure Adapters** |

</div>

## 🏗️ **1) System Context (C4 Model)**

AtlasMesh Fleet OS operates as a **qualified agnostic** platform that orchestrates autonomous fleet operations across multiple vehicle classes, sectors, and infrastructure platforms. The system context diagram below illustrates the high-level interactions between AtlasMesh and external entities.

```mermaid
C4Context
    title AtlasMesh Fleet OS - System Context

    Person(operator, "Fleet Operator", "Manages fleet operations")
    Person(maintainer, "Maintenance Technician", "Services vehicles")
    Person(regulator, "Regulator/Auditor", "Verifies compliance")
    Person(rider, "Rider/End User", "Uses service (ride-hail)")
    
    System_Boundary(atlasmesh, "AtlasMesh Fleet OS") {
        System(fms, "Fleet Management System", "Orchestrates fleet operations")
        System(avkit, "CoreX AV Kit", "Vehicle retrofit hardware/software")
    }
    
    System_Ext(enterprise, "Enterprise Systems", "WMS/TOS/ERP")
    System_Ext(maps, "Map Providers", "HERE/OSM/etc.")
    System_Ext(weather, "Weather Services", "Forecast/nowcast")
    System_Ext(cloud, "Cloud Infrastructure", "AWS/Azure/GCP/On-prem")
    
    Rel(operator, fms, "Uses")
    Rel(maintainer, avkit, "Services")
    Rel(regulator, fms, "Audits")
    Rel(rider, fms, "Interacts with")
    
    Rel(fms, avkit, "Controls")
    Rel(fms, enterprise, "Integrates with")
    Rel(fms, maps, "Consumes data from")
    Rel(fms, weather, "Consumes data from")
    Rel(fms, cloud, "Deployed on")
```

## 🎯 **1.1) Qualified Agnosticism Architecture**

### Vehicle Hardware Abstraction Layer (HAL)

The Vehicle HAL enables **vehicle-agnostic** operations through config-driven profiles:

```mermaid
graph TB
    subgraph "Vehicle Classes"
        LightUTV[🚜 Light Industrial UTV]
        TerminalTractor[🚛 Terminal Tractor]
        MineHaul[⛏️ Mine Haul Truck]
        DefenseVehicle[🛡️ Defense Vehicle]
        RideHailSedan[🚗 Ride-hail Sedan]
        PublicBus[🚌 Public Transit Bus]
    end
    
    subgraph "Vehicle HAL"
        ProfileLoader[📋 Profile Loader]
        HALInterface[🔌 HAL Interface]
        SafetyMonitor[🛡️ Safety Monitor]
    end
    
    subgraph "Standardized Control"
        MotionPlanner[🗺️ Motion Planner]
        PathFollower[🛣️ Path Follower]
        VehicleController[🎮 Vehicle Controller]
    end
    
    LightUTV --> ProfileLoader
    TerminalTractor --> ProfileLoader
    MineHaul --> ProfileLoader
    DefenseVehicle --> ProfileLoader
    RideHailSedan --> ProfileLoader
    PublicBus --> ProfileLoader
    
    ProfileLoader --> HALInterface
    HALInterface --> SafetyMonitor
    SafetyMonitor --> MotionPlanner
    MotionPlanner --> PathFollower
    PathFollower --> VehicleController
    
    classDef vehicle fill:#e8f5e8,stroke:#1b5e20,stroke-width:2px
    classDef hal fill:#e1f5fe,stroke:#01579b,stroke-width:3px
    classDef control fill:#fff3e0,stroke:#f57c00,stroke-width:2px
    
    class LightUTV,TerminalTractor,MineHaul,DefenseVehicle,RideHailSedan,PublicBus vehicle
    class ProfileLoader,HALInterface,SafetyMonitor hal
    class MotionPlanner,PathFollower,VehicleController control
```

### Variant Budget Enforcement

The Variant Budget system ensures **qualified agnosticism** stays within defined limits:

```mermaid
graph LR
    CodeChanges[📝 Code Changes] --> DeltaAnalyzer[📊 Delta Analyzer]
    DeltaAnalyzer --> BudgetChecker{Budget Check}
    
    BudgetChecker -->|≤5% Code Delta| BuildPass[✅ Build Pass]
    BudgetChecker -->|>5% Code Delta| BuildBlock[❌ Build Block]
    
    BuildBlock --> CCBReview[📋 CCB Review]
    CCBReview --> RefactorPlan[🔧 Refactor Plan]
    
    classDef pass fill:#e8f5e8,stroke:#388e3c,stroke-width:2px
    classDef block fill:#ffebee,stroke:#c62828,stroke-width:2px
    classDef process fill:#e1f5fe,stroke:#01579b,stroke-width:2px
    
    class BuildPass pass
    class BuildBlock,CCBReview,RefactorPlan block
    class CodeChanges,DeltaAnalyzer,BudgetChecker process
```

## 🔧 **2) Container Diagram (Services & Components)**

The container diagram below shows the major components of the AtlasMesh Fleet OS and their interactions, including the new qualified agnosticism services.

```mermaid
C4Container
    title AtlasMesh Fleet OS - Container Diagram

    Person(operator, "Fleet Operator", "Manages fleet operations")
    
    System_Boundary(atlasmesh, "AtlasMesh Fleet OS") {
        Container(control_center, "Control Center UI", "React, TypeScript", "Web interface for fleet operations")
        Container(api_gateway, "API Gateway", "Go, gRPC", "Entry point for all API requests")
        
        Container_Boundary(core_services, "Core Services") {
            Container(trip_service, "Trip Service", "Go", "Trip lifecycle management")
            Container(dispatch, "Dispatch Service", "Go, Python", "Assignment and scheduling")
            Container(routing, "Routing Service", "Rust, C++", "Route planning and optimization")
            Container(policy_engine, "Policy Engine", "Go, Rego", "Rules evaluation and enforcement")
            Container(fleet_manager, "Fleet Manager", "Go", "Vehicle lifecycle and health")
        }
        
        Container_Boundary(operational_services, "Operational Services") {
            Container(energy_manager, "Energy Manager", "Python", "Energy optimization and charging")
            Container(predictive_maint, "Predictive Maintenance", "Go", "RUL models, feature store, work orders")
            Container(weather_fusion, "Weather Fusion", "Go", "Multi-source weather integration")
            Container(alerts_incident, "Alerts & Incidents", "Go", "Alert management and response")
        }
        
        Container_Boundary(data_services, "Data Services") {
            Container(telemetry_ingest, "Telemetry Ingest", "Go, Kafka", "Data ingestion and validation")
            Container(analytics_api, "Analytics API", "Python, dbt", "KPI calculation and reporting")
            Container(map_service, "Map Service", "Go, PostGIS", "Geospatial data management")
            Container(feature_store, "Feature Store & Registry", "Go", "ML features, model registry, drift detection")
            Container(telemetry_lakehouse, "Telemetry Lakehouse", "Go", "Hot/cold data paths, streaming ETL")
        }
        
        Container_Boundary(security_services, "Security Services") {
            Container(authn_authz, "AuthN/AuthZ", "Go, OIDC", "Authentication and authorization")
            Container(ota_manager, "OTA Manager", "Rust", "Secure software updates")
            Container(v2x_service, "V2X Service", "Rust, C", "Vehicle-to-everything communication")
            Container(key_secret_mgmt, "Key & Secret Management", "Go", "Vault/KMS, rotation SLAs, envelope encryption")
            Container(zero_trust_iam, "Zero-Trust IAM", "Go", "SPIFFE/SPIRE, mTLS, policy centralization")
        }
        
        Container_Boundary(integration_services, "Integration Services") {
            Container(adapter_sdk, "Adapter SDK", "TypeScript, Go", "Integration framework")
            Container(adapter_registry, "Adapter Registry", "Go", "Adapter management and discovery")
            Container(sector_overlays, "Sector Overlays", "Go", "Policy/UI tokens, ERP/WMS/TOS adapters")
            Container(tenant_entitlements, "Tenant Entitlements", "Go", "Token-based overlays, entitlement model")
        }
        
        Container_Boundary(compliance_services, "Compliance & Governance") {
            Container(evidence_engine, "Evidence Engine", "Go", "Continuous evidence, audit bundles")
            Container(auditability, "Auditability", "Go", "Cryptographic decision logs, retention automation")
        }
        
        Container_Boundary(simulation_services, "Simulation & Digital Twin") {
            Container(digital_twin_sim, "Digital Twin & Simulation", "Go", "Scenario bank, golden replays, fault injection")
        }
        
        Container_Boundary(edge_components, "Edge Components") {
            Container(vehicle_agent, "Vehicle Agent", "Rust, C++", "On-vehicle control and monitoring")
            Container(tele_assist, "Tele-Assist Client", "Rust, WebRTC", "Remote assistance interface")
            Container(diag_agent, "Diagnostics Agent", "Rust", "Vehicle diagnostics and logging")
        }
        
        ContainerDb(event_store, "Event Store", "Kafka, TimescaleDB", "Event sourcing and time-series data")
        ContainerDb(config_store, "Config Store", "PostgreSQL, Redis", "Configuration and state")
        ContainerDb(geo_store, "Geospatial Store", "PostGIS, S3", "Maps and geospatial data")
        ContainerDb(analytics_store, "Analytics Store", "ClickHouse, MinIO", "Hot/cold path analytics")
        ContainerDb(ml_store, "ML Store", "Neo4j, S3", "Feature store, model registry, lineage")
        ContainerDb(vault_store, "Vault Store", "HashiCorp Vault", "Secrets, keys, certificates")
    }
    
    System_Ext(enterprise, "Enterprise Systems", "WMS/TOS/ERP")
    System_Ext(maps, "Map Providers", "HERE/OSM/etc.")
    System_Ext(weather, "Weather Services", "Forecast/nowcast")
    
    Rel(operator, control_center, "Uses")
    Rel(control_center, api_gateway, "API calls", "HTTPS")
    
    Rel(api_gateway, trip_service, "Routes requests", "gRPC")
    Rel(api_gateway, fleet_manager, "Routes requests", "gRPC")
    Rel(api_gateway, analytics_api, "Routes requests", "gRPC")
    
    Rel(trip_service, dispatch, "Creates assignments", "gRPC")
    Rel(dispatch, routing, "Requests routes", "gRPC")
    Rel(dispatch, policy_engine, "Evaluates rules", "gRPC")
    
    Rel(fleet_manager, vehicle_agent, "Controls", "MQTT/gRPC")
    Rel(fleet_manager, ota_manager, "Initiates updates", "gRPC")
    
    Rel(vehicle_agent, telemetry_ingest, "Sends telemetry", "MQTT")
    Rel(telemetry_ingest, event_store, "Stores events", "Kafka")
    
    Rel(tele_assist, vehicle_agent, "Provides assistance", "WebRTC/gRPC")
    
    Rel(weather_fusion, weather, "Consumes data", "HTTPS/MQTT")
    Rel(map_service, maps, "Consumes data", "HTTPS")
    
    Rel(adapter_sdk, enterprise, "Integrates with", "Custom protocols")
    
    Rel_Back(analytics_api, event_store, "Analyzes data", "SQL/Spark")
    Rel(core_services, config_store, "Reads/writes configuration", "SQL/Redis")
    Rel(map_service, geo_store, "Manages geospatial data", "PostGIS/S3")
    
    Rel(telemetry_lakehouse, analytics_store, "Manages hot/cold paths", "ClickHouse/MinIO")
    Rel(feature_store, ml_store, "Stores ML features/models", "Neo4j/S3")
    Rel(predictive_maint, feature_store, "Uses ML features", "gRPC")
    
    Rel(key_secret_mgmt, vault_store, "Manages secrets", "Vault API")
    Rel(zero_trust_iam, key_secret_mgmt, "Uses certificates", "gRPC")
    
    Rel(evidence_engine, event_store, "Collects evidence", "Kafka")
    Rel(auditability, evidence_engine, "Signs decisions", "gRPC")
    
    Rel(sector_overlays, tenant_entitlements, "Applies policies", "gRPC")
    Rel(digital_twin_sim, predictive_maint, "Validates models", "gRPC")
```

## 🏗️ **3) Component Architecture Principles**

AtlasMesh Fleet OS is built on the following architectural principles:

### 3.1 Agnostic by Design

The system is designed to be agnostic across multiple dimensions:

- **Vehicle-agnostic**: Abstracted drive-by-wire interfaces, sensor fusion, and vehicle capabilities
- **Platform-agnostic**: Cloud-neutral deployment, containerized services, infrastructure as code
- **Sector-agnostic**: Policy-driven behavior, configurable workflows, domain-specific overlays
- **Sensor-agnostic**: Abstracted sensor interfaces, fusion algorithms, and degraded mode operations
- **Map-agnostic**: Multiple map provider support, provenance tracking, freshness/credibility tradeoffs
- **Weather-agnostic**: Multi-source weather data fusion, confidence scoring, and fallback strategies
- **Comms-agnostic**: Multi-layer communication stack, store-and-forward capabilities, offline operation

### 3.2 Service-Oriented Architecture

The system follows a service-oriented architecture with the following characteristics:

- **Microservices**: Bounded contexts with clear interfaces and responsibilities
- **Event-driven**: Event sourcing for state changes, event-based integration patterns
- **API-first**: Well-defined APIs with versioning, contract tests, and documentation
- **Stateless services**: Externalized state for scalability and resilience
- **Idempotent operations**: Safe retries and exactly-once semantics where needed

### 3.3 Security & Compliance by Design

Security and compliance are fundamental to the architecture:

- **Zero-trust networking**: mTLS for all service communication, SPIFFE/SPIRE for identity
- **Policy as code**: OPA/Rego policies for authorization and business rules
- **Evidence generation**: Automated safety case artifacts, audit trails, and compliance documentation
- **Secure by default**: Least privilege, encryption in transit and at rest, secrets management
- **Verifiable builds**: Reproducible builds, signed artifacts, SBOM generation

## 📊 **4) Data Flow Architecture**

The diagram below illustrates the key data flows within the AtlasMesh Fleet OS:

```mermaid
flowchart TD
    subgraph "Edge Layer"
        VA[Vehicle Agent]
        TA[Tele-Assist Client]
        DA[Diagnostics Agent]
    end

    subgraph "Ingestion Layer"
        TI[Telemetry Ingest]
        ES[Event Store]
    end

    subgraph "Processing Layer"
        TS[Trip Service]
        DS[Dispatch]
        RS[Routing]
        PE[Policy Engine]
        FM[Fleet Manager]
        WF[Weather Fusion]
        MS[Map Service]
    end

    subgraph "Analytics Layer"
        AA[Analytics API]
        PM[Predictive Maintenance]
        EM[Energy Manager]
    end

    subgraph "External Sources"
        Maps[Map Providers]
        Weather[Weather Services]
        Enterprise[Enterprise Systems]
    end

    VA -->|Telemetry| TI
    VA -->|Diagnostics| DA
    TA -->|Assistance Data| VA
    
    TI -->|Events| ES
    
    ES -->|Historical Data| AA
    ES -->|Sensor Data| PM
    ES -->|Energy Data| EM
    
    TS -->|Trip Events| ES
    DS -->|Assignment Events| ES
    RS -->|Route Events| ES
    PE -->|Policy Decisions| ES
    FM -->|Vehicle Events| ES
    
    Maps -->|Map Data| MS
    Weather -->|Weather Data| WF
    Enterprise -->|Business Data| TS
    
    MS -->|Map Features| RS
    WF -->|Weather Conditions| RS
    PE -->|Policy Constraints| DS
    PE -->|Policy Constraints| RS
    
    FM -->|Commands| VA
    DS -->|Assignments| VA
    RS -->|Routes| VA
```

### 4.1 ROS2-Based Edge Architecture

```mermaid
flowchart TD
    subgraph "Vehicle Hardware"
        Sensors[Sensors]
        Actuators[Actuators]
        Compute[Edge Compute]
    end

    subgraph "ROS2 Middleware"
        DDS[DDS/RTPS]
        NodeMgr[Node Lifecycle Manager]
        QoS[QoS Policies]
    end

    subgraph "ROS2 Nodes"
        PerceptionNodes[Perception Nodes]
        LocalizationNodes[Localization Nodes]
        DecisionNodes[Decision Nodes]
        ControlNodes[Control Nodes]
        DiagNodes[Diagnostics Nodes]
        CommsNodes[Communication Nodes]
    end

    subgraph "Cloud Bridge"
        MsgTranslator[Message Translator]
        SecurityBridge[Security Bridge]
        StoreForward[Store & Forward]
    end

    Sensors -->|Driver Nodes| PerceptionNodes
    PerceptionNodes -->|Object Data| DecisionNodes
    Sensors -->|Raw Data| LocalizationNodes
    LocalizationNodes -->|Position| DecisionNodes
    DecisionNodes -->|Trajectory| ControlNodes
    ControlNodes -->|Commands| Actuators
    
    PerceptionNodes -.->|Messages| DDS
    LocalizationNodes -.->|Messages| DDS
    DecisionNodes -.->|Messages| DDS
    ControlNodes -.->|Messages| DDS
    DiagNodes -.->|Messages| DDS
    CommsNodes -.->|Messages| DDS
    
    NodeMgr -->|Lifecycle| PerceptionNodes
    NodeMgr -->|Lifecycle| LocalizationNodes
    NodeMgr -->|Lifecycle| DecisionNodes
    NodeMgr -->|Lifecycle| ControlNodes
    NodeMgr -->|Lifecycle| DiagNodes
    NodeMgr -->|Lifecycle| CommsNodes
    
    QoS -.->|Policies| DDS
    
    CommsNodes -->|Telemetry| MsgTranslator
    MsgTranslator -->|Protocol Translation| SecurityBridge
    SecurityBridge -->|Secure Comms| Cloud[Cloud Services]
    CommsNodes -->|Offline Data| StoreForward
    StoreForward -.->|Reconnection| SecurityBridge
```

### 4.2 Behavior Tree Decision Framework

```mermaid
flowchart TD
    subgraph "Decision Framework"
        BT[Behavior Tree Engine]
        SA[Safety Arbitrator]
        ML[ML Priors]
        PE[Policy Engine]
    end

    subgraph "Inputs"
        Perception[Perception Data]
        Localization[Localization Data]
        Mission[Mission Parameters]
        Map[Map Data]
        Weather[Weather Data]
    end

    subgraph "Outputs"
        Trajectory[Trajectory]
        VehicleState[Vehicle State]
        Diagnostics[Diagnostics]
    end

    Perception --> BT
    Localization --> BT
    Mission --> BT
    Map --> BT
    Weather --> BT
    
    BT -->|Decision| SA
    ML -->|Optimization Hints| BT
    PE -->|Constraints| SA
    
    SA -->|Verified Decision| Trajectory
    SA -->|State Transition| VehicleState
    BT -->|Performance Metrics| Diagnostics
    
    Trajectory -->|Feedback| ML
    VehicleState -->|Feedback| ML
```

## 🚀 **5) Deployment Architecture**

AtlasMesh Fleet OS supports multiple deployment topologies to accommodate different operational requirements:

### 5.1 Cloud-Based Deployment

```mermaid
flowchart TD
    subgraph "Cloud Infrastructure"
        API[API Gateway]
        Core[Core Services]
        Data[Data Services]
        Ops[Operational Services]
        DB[(Databases)]
    end
    
    subgraph "Edge Devices"
        VA[Vehicle Agent]
        TA[Tele-Assist Client]
        DA[Diagnostics Agent]
    end
    
    subgraph "Operations Center"
        CC[Control Center UI]
        GA[Garage PC App]
    end
    
    CC -->|HTTPS| API
    GA -->|HTTPS| API
    
    API --> Core
    API --> Data
    API --> Ops
    
    Core --> DB
    Data --> DB
    Ops --> DB
    
    VA <-->|MQTT/gRPC| API
    TA <-->|WebRTC/gRPC| API
    DA -->|HTTPS| Data
```

### 5.2 Hybrid Deployment

```mermaid
flowchart TD
    subgraph "Cloud Infrastructure"
        API[API Gateway]
        Core[Core Services]
        Analytics[Analytics Services]
        CloudDB[(Cloud Databases)]
    end
    
    subgraph "On-Premises Infrastructure"
        EdgeAPI[Edge API Gateway]
        EdgeCore[Edge Core Services]
        EdgeDB[(Edge Databases)]
    end
    
    subgraph "Edge Devices"
        VA[Vehicle Agent]
        TA[Tele-Assist Client]
        DA[Diagnostics Agent]
    end
    
    subgraph "Operations Center"
        CC[Control Center UI]
        GA[Garage PC App]
    end
    
    CC -->|HTTPS| API
    CC -.->|Fallback| EdgeAPI
    GA -->|HTTPS| EdgeAPI
    
    API --> Core
    API --> Analytics
    Core --> CloudDB
    Analytics --> CloudDB
    
    EdgeAPI --> EdgeCore
    EdgeCore --> EdgeDB
    
    VA <-->|MQTT/gRPC| EdgeAPI
    TA <-->|WebRTC/gRPC| EdgeAPI
    DA -->|HTTPS| EdgeAPI
    
    EdgeAPI <-->|Sync| API
    EdgeDB <-->|Replication| CloudDB
```

### 5.3 Air-Gapped Deployment

```mermaid
flowchart TD
    subgraph "On-Premises Infrastructure"
        API[API Gateway]
        Core[Core Services]
        Data[Data Services]
        Ops[Operational Services]
        DB[(Databases)]
    end
    
    subgraph "Edge Devices"
        VA[Vehicle Agent]
        TA[Tele-Assist Client]
        DA[Diagnostics Agent]
    end
    
    subgraph "Operations Center"
        CC[Control Center UI]
        GA[Garage PC App]
    end
    
    CC -->|HTTPS| API
    GA -->|HTTPS| API
    
    API --> Core
    API --> Data
    API --> Ops
    
    Core --> DB
    Data --> DB
    Ops --> DB
    
    VA <-->|MQTT/gRPC| API
    TA <-->|WebRTC/gRPC| API
    DA -->|HTTPS| Data
    
    subgraph "Manual Data Transfer"
        Export[Export Package]
        Import[Import Package]
    end
    
    DB -->|Scheduled| Export
    Import -->|Controlled| DB
```

## 🛡️ **6) Failure Domains & Resilience**

AtlasMesh Fleet OS is designed with resilience in mind, with the following failure domains and mitigation strategies:

### 6.1 Communication Failures

| Failure Mode | Impact | Mitigation |
|--------------|--------|------------|
| Cloud connectivity loss | Remote monitoring and control unavailable | Edge autonomy for 45-60 minutes; store-and-forward telemetry |
| V2V/V2X disruption | Reduced coordination between vehicles | Independent operation modes; conservative safety parameters |
| GPS/GNSS denial | Position uncertainty | Multi-modal localization (SLAM, visual, inertial); degraded operation modes |

### 6.2 Service Failures

| Failure Mode | Impact | Mitigation |
|--------------|--------|------------|
| Trip Service outage | New trips cannot be created | Existing trips continue; graceful degradation |
| Routing Service outage | New routes cannot be calculated | Cached routes used; simplified routing fallbacks |
| Policy Engine outage | Policy decisions unavailable | Conservative default policies; cached decisions |

### 6.3 Environmental Challenges

| Failure Mode | Impact | Mitigation |
|--------------|--------|------------|
| Extreme heat | Sensor and compute degradation | Thermal management; derating curves; shade-seeking behavior |
| Dust/sand storms | Sensor occlusion | Multi-modal sensing; cleaning systems; confidence-aware fusion |
| Heavy precipitation | Reduced visibility and traction | Weather-aware routing; speed adaptation; safe harbor protocols |

## 🔗 **7) Key Interfaces & Integration Points**

### 7.1 External Interfaces

| Interface | Purpose | Protocol | Notes |
|-----------|---------|----------|-------|
| Enterprise Adapter | Integration with WMS/TOS/ERP | REST/SOAP/EDI | Contract-tested; version-pinned |
| Map Provider | Geospatial data ingestion | REST/WMTS/Vector Tiles | Multi-provider; provenance tracking |
| Weather Service | Weather data ingestion | REST/MQTT | Multi-source fusion; confidence scoring |
| Regulatory Reporting | Compliance documentation | REST/SFTP | Jurisdiction-specific formats |

### 7.2 Internal Interfaces

| Interface | Purpose | Protocol | Notes |
|-----------|---------|----------|-------|
| Vehicle Control | Command and telemetry | MQTT/gRPC | Secure; efficient; resilient |
| Service-to-Service | Inter-service communication | gRPC | mTLS; observability; rate limiting |
| Event Bus | Event distribution | Kafka | Exactly-once; ordered; durable |
| Policy Evaluation | Rule enforcement | OPA/Rego | Versioned; auditable; testable |

### 7.3 Edge-Cloud Interface

```mermaid
sequenceDiagram
    participant VA as Vehicle Agent
    participant CB as Cloud Bridge
    participant API as API Gateway
    participant FM as Fleet Manager
    participant TI as Telemetry Ingest
    participant ES as Event Store

    Note over VA,ES: Normal Operation (Connected)
    VA->>CB: Telemetry (gRPC stream)
    CB->>TI: Forward telemetry (MQTT)
    TI->>ES: Store events (Kafka)
    FM->>API: Command request
    API->>CB: Command (gRPC)
    CB->>VA: Forward command
    VA-->>CB: Command ACK
    CB-->>API: Forward ACK
    API-->>FM: Command result

    Note over VA,ES: Degraded Operation (Intermittent)
    VA->>VA: Store telemetry locally
    VA->>VA: Execute cached policies
    VA->>CB: Connectivity restored
    CB->>TI: Buffered telemetry (batch)
    TI->>ES: Store events (Kafka)

    Note over VA,ES: Offline Operation
    VA->>VA: Store telemetry locally
    VA->>VA: Execute cached policies
    VA->>VA: Autonomous decision-making
    VA->>VA: Safe operational boundaries
```

### 7.4 Data Contracts

#### Vehicle Telemetry Contract

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "Vehicle Telemetry",
  "type": "object",
  "required": ["vehicle_id", "timestamp", "sequence", "position", "status"],
  "properties": {
    "vehicle_id": {
      "type": "string",
      "description": "Unique identifier for the vehicle"
    },
    "timestamp": {
      "type": "string",
      "format": "date-time",
      "description": "ISO 8601 timestamp of the telemetry data"
    },
    "sequence": {
      "type": "integer",
      "description": "Monotonically increasing sequence number"
    },
    "position": {
      "type": "object",
      "required": ["latitude", "longitude", "heading"],
      "properties": {
        "latitude": { "type": "number" },
        "longitude": { "type": "number" },
        "heading": { "type": "number" },
        "altitude": { "type": "number" },
        "speed": { "type": "number" },
        "accuracy": { "type": "number" }
      }
    },
    "status": {
      "type": "object",
      "required": ["operational_state", "energy_level"],
      "properties": {
        "operational_state": {
          "type": "string",
          "enum": ["READY", "BUSY", "CHARGING", "MAINTENANCE", "ERROR"]
        },
        "energy_level": { "type": "number", "minimum": 0, "maximum": 100 },
        "health_score": { "type": "number", "minimum": 0, "maximum": 100 },
        "assist_state": {
          "type": "string",
          "enum": ["NONE", "REQUESTED", "ACTIVE"]
        }
      }
    },
    "diagnostics": {
      "type": "object",
      "properties": {
        "cpu_usage": { "type": "number" },
        "memory_usage": { "type": "number" },
        "disk_usage": { "type": "number" },
        "temperature": { "type": "number" },
        "network_quality": { "type": "number" }
      }
    },
    "mission": {
      "type": "object",
      "properties": {
        "mission_id": { "type": "string" },
        "route_id": { "type": "string" },
        "progress": { "type": "number", "minimum": 0, "maximum": 100 },
        "eta": { "type": "string", "format": "date-time" }
      }
    }
  }
}
```

#### Vehicle Command Contract

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "Vehicle Command",
  "type": "object",
  "required": ["command_id", "vehicle_id", "timestamp", "command_type"],
  "properties": {
    "command_id": {
      "type": "string",
      "description": "Unique identifier for the command"
    },
    "vehicle_id": {
      "type": "string",
      "description": "Target vehicle identifier"
    },
    "timestamp": {
      "type": "string",
      "format": "date-time",
      "description": "ISO 8601 timestamp of the command"
    },
    "command_type": {
      "type": "string",
      "enum": [
        "ASSIGN_MISSION", "CANCEL_MISSION", "PAUSE", "RESUME", 
        "SAFE_STOP", "EMERGENCY_STOP", "CHANGE_OPERATIONAL_MODE", 
        "UPDATE_ROUTE", "DIAGNOSTICS", "OTA_UPDATE"
      ]
    },
    "priority": {
      "type": "string",
      "enum": ["LOW", "NORMAL", "HIGH", "CRITICAL"],
      "default": "NORMAL"
    },
    "expiration": {
      "type": "string",
      "format": "date-time",
      "description": "ISO 8601 timestamp when the command expires"
    },
    "payload": {
      "type": "object",
      "description": "Command-specific parameters"
    },
    "authentication": {
      "type": "object",
      "properties": {
        "issuer": { "type": "string" },
        "signature": { "type": "string" }
      }
    }
  }
}
```

## 🚀 **8) Phase 2 Advanced Services**

AtlasMesh Fleet OS Phase 2 introduces advanced capabilities for production-scale operations:

### 8.1 Advanced Analytics & ML
- **Predictive Maintenance**: RUL models, feature store integration, work-order automation
- **Feature Store & Registry**: ML lifecycle management, drift detection, model versioning
- **Telemetry Lakehouse**: Hot/cold data paths, streaming ETL, cost optimization

### 8.2 Enhanced Security & Compliance
- **Zero-Trust IAM**: SPIFFE/SPIRE service identity, mTLS enforcement, policy centralization
- **Key & Secret Management**: Vault/KMS integration, rotation SLAs, envelope encryption
- **Evidence Engine**: Continuous evidence collection, automated audit bundles
- **Auditability**: Cryptographically signed decision logs, retention automation

### 8.3 Sector Customization & Multi-Tenancy
- **Sector Overlays**: Policy/UI tokens for sector-specific customizations without code forks
- **Tenant Entitlements**: Token-based entitlement model, multi-tenant isolation

### 8.4 Digital Twin & Simulation
- **Digital Twin & Simulation**: Scenario banks, golden replays, fault injection for robust testing

## 📈 **9) Observability & Monitoring**

AtlasMesh Fleet OS implements a comprehensive observability strategy:

- **Metrics**: Prometheus for service and business metrics; Grafana for dashboards
- **Logging**: Structured logging with correlation IDs; centralized log aggregation
- **Tracing**: OpenTelemetry for distributed tracing; Jaeger for visualization
- **Alerting**: Multi-tier alert definition; runbook integration; on-call rotation
- **SLOs**: Service Level Objectives with error budgets; SLI monitoring
- **Unified Observability**: Correlation IDs across all services, SLO dashboards, alert fatigue prevention

## 📡 **10) Eventing Backbone**

AtlasMesh Fleet OS uses a robust eventing architecture for system-wide communication:

- **Kafka Topics**: 16 specialized topics for different event types (vehicle events, fleet commands, system audits, DLQ patterns)
- **Schema Registry**: Avro schema management for message validation and evolution
- **Dead Letter Queues**: Comprehensive DLQ patterns for failed message processing
- **Event Bus Service**: Central event hub with routing, validation, and correlation ID propagation

## 🔒 **11) Security Architecture**

The security architecture follows defense-in-depth principles:

- **Identity**: SPIFFE/SPIRE for service identity; OIDC for user authentication
- **Access Control**: RBAC/ABAC with OPA; least privilege principle
- **Network Security**: mTLS for all traffic; network policies; segmentation
- **Data Protection**: Encryption at rest and in transit; key management
- **Supply Chain**: Verified builds; SBOM generation; vulnerability scanning
- **Incident Response**: Detection, containment, eradication, recovery procedures
- **Zero-Trust**: Service-to-service mTLS, policy centralization, continuous verification

## 📋 **12) Compliance Architecture**

The compliance architecture ensures regulatory adherence:

- **Safety Case**: Automated evidence generation; traceability; verification
- **Regulatory Mapping**: Jurisdiction-specific requirements and documentation
- **Audit Trail**: Immutable event logs; decision provenance; access records
- **Privacy Controls**: Data minimization; purpose limitation; retention policies
- **Evidence-as-Code**: Automated audit bundle generation with cryptographic signing

## 🎯 **13) Key Architectural Decisions**

| ADR ID | Decision | Rationale | Alternatives Considered |
|--------|----------|-----------|-------------------------|
| ADR-0001 | Vehicle-Agnostic Architecture | Support multiple vehicle classes; reduce dependency on specific hardware | Vehicle-specific implementations; fork per vehicle type |
| ADR-0002 | Platform-Agnostic Architecture | Cloud-neutral deployment; avoid vendor lock-in | Cloud-specific optimizations; native services |
| ADR-0003 | Sector-Agnostic Architecture | Shared backbone with sector overlays; code reuse | Sector-specific forks; separate products |
| ADR-0004 | Sensor-Agnostic Architecture | Support multiple sensor configurations; certified packs | Hard-coded sensor integration; fixed sensor suite |
| ADR-0005 | Map-Source-Agnostic Architecture | Multiple map providers; provenance tracking | Single map provider; custom map format |
| ADR-0006 | Weather-Source-Agnostic Architecture | Multi-source weather fusion; confidence scoring | Single weather source; simple integration |
| ADR-0007 | Communications-Agnostic Architecture | Multi-path communications; offline-first | Single communication channel; cloud dependency |
| ADR-0008 | ROS2-Based Edge Stack | Industry standard; component isolation; real-time | Custom middleware; direct hardware access |
| ADR-0009 | Hybrid Decision Framework | Behavior trees + rules + ML; explainability | Pure ML approach; pure rule-based system |
| ADR-0010 | Simulation Strategy | CARLA + Gazebo; scenario-based validation | Single simulator; minimal simulation |
| ADR-011 | Event Sourcing | Audit trail; temporal queries; resilience | CRUD; synchronous APIs |
| ADR-012 | Policy as Code | Versioned rules; testable policies; separation of concerns | Hardcoded rules; config files |

## 🗺️ **14) Architecture Evolution & Roadmap**

| Phase | Architectural Focus | Key Deliverables | Status |
|-------|---------------------|------------------|--------|
| Phase 1 | Core services; agnostic foundations | Trip service; dispatch; routing; policy engine; edge stack; UI | ✅ **COMPLETED** |
| Phase 2 | Advanced features; production readiness | Predictive maintenance; zero-trust security; evidence engine; digital twin | ✅ **COMPLETED** |
| Phase 3 | Qualified agnosticism implementation | Vehicle HAL; sensor packs; variant budget; conformance testing | ✅ **COMPLETED** |
| Phase 4 | Operational excellence; resilience | Chaos engineering; degraded modes; on-call runbooks; time sync | 🔄 **NEXT** |
| Future | Advanced intelligence; self-optimization | ML-driven optimization; predictive operations; autonomous healing | 📋 **PLANNED** |

## 🏛️ **15) Architecture Governance**

The architecture governance process ensures quality and alignment:

- **Architecture Review Board**: Weekly meetings; decision authority
- **RFC Process**: Formal proposal; review period; approval workflow
- **Architecture Principles**: Documented guidelines; compliance checks
- **Technical Debt Management**: Regular assessment; remediation planning
- **Innovation Pipeline**: Exploration; prototyping; evaluation; integration

## 📚 **16) References & Related Documents**

### Core Technical Documentation
- [System Requirements](03_Requirements_FRs_NFRs.md)
- [Security & Compliance](05_Security_and_Compliance.md)
- [Operations & Runbooks](06_Operations_and_Runbooks.md)
- [Data & Analytics Integration](04_Data_and_Analytics_Integration.md)
- [Architecture Decision Records](../ADR/README.md)

### Qualified Agnosticism Documentation
- [Qualified Agnosticism Overview](08_Qualified_Agnosticism.md)
- [Five Constraining Realities](09_Five_Constraining_Realities.md)
- [Feasibility Scorecard](10_Feasibility_Scorecard.md)
- [Agnostic By Contract](11_Agnostic_By_Contract.md)
- [Cross-Department Checklist](12_Cross_Department_Checklist.md)
- [Programmatic Proof Points](13_Programmatic_Proof_Points.md)
- [Architecture Reality Check](14_Architecture_Reality_Check.md)



