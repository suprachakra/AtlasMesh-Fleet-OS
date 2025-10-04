# AtlasMesh Fleet OS — Data Models & Entity Relationship Diagrams

## 1) Overview

This document provides comprehensive data models, Entity Relationship Diagrams (ERDs), and data flow diagrams for the AtlasMesh Fleet OS. The system uses a microservices architecture with domain-driven design principles, where each service manages its own data models.

## 2) Core Domain ERD

```mermaid
erDiagram
    %% Core Fleet Management Entities
    FLEET {
        uuid fleet_id PK
        string name
        string organization_id FK
        string sector
        jsonb configuration
        timestamp created_at
        timestamp updated_at
        string status
        jsonb metadata
    }
    
    VEHICLE {
        uuid vehicle_id PK
        uuid fleet_id FK
        string asset_tag
        string manufacturer
        string model
        string serial_number
        jsonb vehicle_profile
        string current_status
        geometry current_location
        timestamp last_seen
        jsonb capabilities
        jsonb sensor_config
        timestamp created_at
        timestamp updated_at
    }
    
    TRIP {
        uuid trip_id PK
        uuid vehicle_id FK
        uuid route_id FK
        string mission_type
        string status
        geometry origin
        geometry destination
        timestamp scheduled_start
        timestamp actual_start
        timestamp scheduled_end
        timestamp actual_end
        jsonb trip_parameters
        jsonb telemetry_summary
        timestamp created_at
        timestamp updated_at
    }
    
    ROUTE {
        uuid route_id PK
        string name
        geometry path
        jsonb waypoints
        float distance_km
        int estimated_duration_minutes
        jsonb constraints
        jsonb optimization_params
        string created_by
        timestamp created_at
        timestamp updated_at
        string status
    }
    
    POLICY {
        uuid policy_id PK
        string name
        string version
        text rego_code
        jsonb schema
        string sector
        string environment
        boolean is_active
        timestamp effective_from
        timestamp effective_until
        string created_by
        timestamp created_at
        timestamp updated_at
    }
    
    %% Relationships
    FLEET ||--o{ VEHICLE : contains
    VEHICLE ||--o{ TRIP : executes
    TRIP }o--|| ROUTE : follows
    VEHICLE }o--o{ POLICY : governed_by
    FLEET }o--o{ POLICY : applies_to
```

## 3) Telemetry & Analytics ERD

```mermaid
erDiagram
    %% Telemetry and Analytics Data Models
    VEHICLE_TELEMETRY {
        uuid telemetry_id PK
        uuid vehicle_id FK
        timestamp recorded_at
        geometry location
        float speed_kmh
        float heading_degrees
        float battery_level_percent
        float fuel_level_percent
        jsonb sensor_readings
        jsonb diagnostic_codes
        jsonb environmental_conditions
        string operational_status
        jsonb custom_data
        timestamp ingested_at
    }
    
    VEHICLE_COMMAND {
        uuid command_id PK
        uuid vehicle_id FK
        string command_type
        jsonb parameters
        string status
        timestamp issued_at
        timestamp executed_at
        timestamp expires_at
        string issued_by
        jsonb response_data
        string priority
    }
    
    ALERT {
        uuid alert_id PK
        uuid vehicle_id FK
        uuid trip_id FK
        string alert_type
        string severity
        text message
        jsonb context_data
        timestamp triggered_at
        timestamp acknowledged_at
        timestamp resolved_at
        string acknowledged_by
        string resolved_by
        string status
    }
    
    MAINTENANCE_RECORD {
        uuid maintenance_id PK
        uuid vehicle_id FK
        string maintenance_type
        text description
        timestamp scheduled_date
        timestamp completed_date
        string performed_by
        jsonb parts_used
        float cost
        jsonb diagnostic_results
        string status
        timestamp created_at
        timestamp updated_at
    }
    
    %% Feature Store for ML
    FEATURE {
        uuid feature_id PK
        string feature_name
        string feature_group
        string data_type
        text description
        jsonb schema
        timestamp created_at
        timestamp updated_at
        string created_by
        boolean is_active
    }
    
    FEATURE_VALUE {
        uuid value_id PK
        uuid feature_id FK
        uuid entity_id
        string entity_type
        jsonb feature_value
        timestamp computed_at
        timestamp valid_until
        float confidence_score
    }
    
    %% Relationships
    VEHICLE ||--o{ VEHICLE_TELEMETRY : generates
    VEHICLE ||--o{ VEHICLE_COMMAND : receives
    VEHICLE ||--o{ ALERT : triggers
    VEHICLE ||--o{ MAINTENANCE_RECORD : requires
    TRIP ||--o{ ALERT : may_trigger
    FEATURE ||--o{ FEATURE_VALUE : has_values
```

## 4) Security & Compliance ERD

```mermaid
erDiagram
    %% Security and Compliance Data Models
    USER {
        uuid user_id PK
        string username
        string email
        string password_hash
        jsonb roles
        jsonb permissions
        string organization_id FK
        boolean is_active
        timestamp last_login
        timestamp created_at
        timestamp updated_at
    }
    
    ORGANIZATION {
        uuid organization_id PK
        string name
        string sector
        jsonb configuration
        jsonb entitlements
        string subscription_tier
        timestamp created_at
        timestamp updated_at
        boolean is_active
    }
    
    AUDIT_LOG {
        uuid audit_id PK
        uuid user_id FK
        uuid resource_id
        string resource_type
        string action
        jsonb before_state
        jsonb after_state
        timestamp performed_at
        string ip_address
        string user_agent
        jsonb context
        string correlation_id
    }
    
    EVIDENCE_BUNDLE {
        uuid bundle_id PK
        string bundle_type
        string version
        jsonb evidence_items
        string cryptographic_hash
        string digital_signature
        timestamp generated_at
        timestamp valid_until
        string generated_by
        string compliance_framework
        jsonb metadata
    }
    
    CERTIFICATE {
        uuid certificate_id PK
        uuid vehicle_id FK
        string certificate_type
        text certificate_pem
        text private_key_pem
        timestamp issued_at
        timestamp expires_at
        string issuer
        string subject
        boolean is_revoked
        timestamp revoked_at
    }
    
    SECRET {
        uuid secret_id PK
        string secret_name
        string secret_type
        text encrypted_value
        string encryption_key_id
        timestamp created_at
        timestamp updated_at
        timestamp expires_at
        string created_by
        jsonb access_policy
    }
    
    %% Relationships
    ORGANIZATION ||--o{ USER : employs
    USER ||--o{ AUDIT_LOG : performs_actions
    VEHICLE ||--o{ CERTIFICATE : has_certificates
    ORGANIZATION ||--o{ EVIDENCE_BUNDLE : generates
    ORGANIZATION ||--o{ SECRET : owns
```

## 5) Data Flow Diagram - Telemetry Pipeline

```mermaid
flowchart TD
    %% Data Sources
    V1[Vehicle Agent 1] --> |MQTT/gRPC| TI[Telemetry Ingestion]
    V2[Vehicle Agent 2] --> |MQTT/gRPC| TI
    VN[Vehicle Agent N] --> |MQTT/gRPC| TI
    
    %% Ingestion Layer
    TI --> |Validate Schema| SR[Schema Registry]
    TI --> |Raw Events| K1[Kafka: vehicle.telemetry.raw]
    
    %% Processing Layer
    K1 --> |Stream Processing| TP[Telemetry Processor]
    TP --> |Validated Events| K2[Kafka: vehicle.telemetry.processed]
    TP --> |Failed Events| DLQ[Dead Letter Queue]
    
    %% Storage Layer - Hot Path
    K2 --> |Real-time Analytics| CH[ClickHouse]
    CH --> |Dashboards| G[Grafana]
    CH --> |Alerts| AM[Alert Manager]
    
    %% Storage Layer - Cold Path
    K2 --> |Batch Processing| ETL[Streaming ETL]
    ETL --> |Historical Data| S3[MinIO/S3]
    
    %% Feature Store
    K2 --> |Feature Extraction| FS[Feature Store]
    FS --> |ML Features| ML[ML Pipeline]
    
    %% Data Lineage
    TI --> |Lineage Events| DL[Data Lineage Service]
    DL --> |Metadata| NEO[Neo4j]
    
    %% Audit & Compliance
    K2 --> |Audit Events| AE[Audit Engine]
    AE --> |Evidence| EB[Evidence Bundle]
    
    style V1 fill:#e1f5fe
    style V2 fill:#e1f5fe
    style VN fill:#e1f5fe
    style TI fill:#f3e5f5
    style CH fill:#e8f5e8
    style S3 fill:#fff3e0
    style FS fill:#f1f8e9
```

## 6) Sequence Diagram - Policy Evaluation Flow

```mermaid
sequenceDiagram
    participant VA as Vehicle Agent
    participant PE as Policy Engine
    participant DS as Dispatch Service
    participant TS as Trip Service
    participant AL as Audit Log
    
    Note over VA,AL: Policy Evaluation for Trip Assignment
    
    VA->>+PE: EvaluatePolicy(vehicleId, context)
    PE->>PE: Load applicable policies
    PE->>PE: Execute Rego evaluation
    
    alt Policy Allows Action
        PE->>AL: Log policy decision (ALLOW)
        PE->>-VA: PolicyResponse(allowed=true, constraints)
        VA->>+DS: RequestTripAssignment(vehicleId, constraints)
        DS->>+TS: CreateTrip(tripRequest)
        TS->>TS: Validate trip parameters
        TS->>-DS: TripCreated(tripId)
        DS->>-VA: TripAssigned(tripId, route)
    else Policy Denies Action
        PE->>AL: Log policy decision (DENY)
        PE->>-VA: PolicyResponse(allowed=false, reason)
        VA->>VA: Enter degraded mode
    end
    
    Note over VA,AL: All decisions are audited for compliance
```

## 7) UML Class Diagram - Core Services

```mermaid
classDiagram
    %% Core Service Classes
    class PolicyEngine {
        +string serviceId
        +Config config
        +PolicyRepository policyRepo
        +AuditRepository auditRepo
        +EvaluatePolicy(request) PolicyResponse
        +CreatePolicy(policy) Policy
        +UpdatePolicy(id, policy) Policy
        +DeletePolicy(id) bool
        +ListPolicies(filter) []Policy
        +HealthCheck() HealthStatus
    }
    
    class TripService {
        +string serviceId
        +TripRepository tripRepo
        +RouteService routeService
        +CreateTrip(request) Trip
        +UpdateTrip(id, updates) Trip
        +CancelTrip(id) bool
        +GetTrip(id) Trip
        +ListTrips(filter) []Trip
        +GetTripStatus(id) TripStatus
    }
    
    class DispatchService {
        +string serviceId
        +VehicleRepository vehicleRepo
        +PolicyEngine policyEngine
        +DispatchTrip(tripId, vehicleId) Assignment
        +OptimizeAssignments(constraints) []Assignment
        +GetAssignmentStatus(id) AssignmentStatus
        +CancelAssignment(id) bool
    }
    
    class FleetManager {
        +string serviceId
        +VehicleRepository vehicleRepo
        +RegisterVehicle(vehicle) Vehicle
        +UpdateVehicleStatus(id, status) bool
        +GetVehicleHealth(id) HealthMetrics
        +ListVehicles(filter) []Vehicle
        +DecommissionVehicle(id) bool
    }
    
    %% Data Models
    class Vehicle {
        +UUID vehicleId
        +string assetTag
        +VehicleProfile profile
        +Location currentLocation
        +VehicleStatus status
        +Capabilities capabilities
        +DateTime lastSeen
        +UpdateLocation(location) bool
        +UpdateStatus(status) bool
        +GetTelemetry() TelemetryData
    }
    
    class Trip {
        +UUID tripId
        +UUID vehicleId
        +UUID routeId
        +Location origin
        +Location destination
        +TripStatus status
        +DateTime scheduledStart
        +DateTime actualStart
        +Parameters parameters
        +Start() bool
        +Complete() bool
        +Cancel() bool
    }
    
    class Policy {
        +UUID policyId
        +string name
        +string version
        +string regoCode
        +Schema schema
        +DateTime effectiveFrom
        +DateTime effectiveUntil
        +Evaluate(context) PolicyResult
        +Validate() bool
    }
    
    %% Relationships
    PolicyEngine --> Policy : manages
    TripService --> Trip : manages
    DispatchService --> PolicyEngine : uses
    DispatchService --> TripService : coordinates
    FleetManager --> Vehicle : manages
    Trip --> Vehicle : assigned_to
```

## 8) Data Governance & Quality

### 8.1 Data Classification

| Classification | Description | Examples | Retention |
|---------------|-------------|----------|-----------|
| **Public** | Non-sensitive operational data | Vehicle models, route templates | 7 years |
| **Internal** | Business operational data | Trip summaries, fleet metrics | 5 years |
| **Confidential** | Sensitive business data | Customer data, financial records | 7 years |
| **Restricted** | Highly sensitive data | Security logs, audit trails | 10 years |

### 8.2 Data Quality Metrics

```mermaid
graph LR
    subgraph "Data Quality Dimensions"
        A[Completeness] --> |90%+ required fields| QS[Quality Score]
        B[Accuracy] --> |<1% error rate| QS
        C[Consistency] --> |Cross-system validation| QS
        D[Timeliness] --> |<30s ingestion latency| QS
        E[Validity] --> |Schema compliance| QS
        F[Uniqueness] --> |Duplicate detection| QS
    end
    
    QS --> |Automated Monitoring| DM[Data Monitoring]
    QS --> |Quality Gates| CI[CI/CD Pipeline]
    QS --> |Alerts| ON[On-Call Team]
```

## 9) Database Schemas

### 9.1 PostgreSQL Schema (Core Services)

```sql
-- Core Fleet Management Schema
CREATE SCHEMA fleet_core;

-- Organizations and Multi-tenancy
CREATE TABLE fleet_core.organizations (
    organization_id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    name VARCHAR(255) NOT NULL,
    sector VARCHAR(50) NOT NULL,
    configuration JSONB DEFAULT '{}',
    entitlements JSONB DEFAULT '{}',
    subscription_tier VARCHAR(50) DEFAULT 'basic',
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    is_active BOOLEAN DEFAULT true
);

-- Fleets
CREATE TABLE fleet_core.fleets (
    fleet_id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    organization_id UUID NOT NULL REFERENCES fleet_core.organizations(organization_id),
    name VARCHAR(255) NOT NULL,
    sector VARCHAR(50) NOT NULL,
    configuration JSONB DEFAULT '{}',
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    status VARCHAR(50) DEFAULT 'active',
    metadata JSONB DEFAULT '{}'
);

-- Vehicles
CREATE TABLE fleet_core.vehicles (
    vehicle_id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    fleet_id UUID NOT NULL REFERENCES fleet_core.fleets(fleet_id),
    asset_tag VARCHAR(100) UNIQUE NOT NULL,
    manufacturer VARCHAR(100) NOT NULL,
    model VARCHAR(100) NOT NULL,
    serial_number VARCHAR(100) UNIQUE NOT NULL,
    vehicle_profile JSONB NOT NULL,
    current_status VARCHAR(50) DEFAULT 'offline',
    current_location GEOMETRY(POINT, 4326),
    last_seen TIMESTAMP WITH TIME ZONE,
    capabilities JSONB DEFAULT '{}',
    sensor_config JSONB DEFAULT '{}',
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);

-- Create indexes for performance
CREATE INDEX idx_vehicles_fleet_id ON fleet_core.vehicles(fleet_id);
CREATE INDEX idx_vehicles_status ON fleet_core.vehicles(current_status);
CREATE INDEX idx_vehicles_location ON fleet_core.vehicles USING GIST(current_location);
CREATE INDEX idx_vehicles_last_seen ON fleet_core.vehicles(last_seen);
```

### 9.2 ClickHouse Schema (Analytics)

```sql
-- Telemetry Hot Path Schema
CREATE DATABASE IF NOT EXISTS atlasmesh_analytics;

-- Vehicle Telemetry Table (Optimized for time-series queries)
CREATE TABLE atlasmesh_analytics.vehicle_telemetry (
    vehicle_id UUID,
    recorded_at DateTime64(3),
    ingested_at DateTime64(3) DEFAULT now64(),
    location Tuple(Float64, Float64), -- (latitude, longitude)
    speed_kmh Float32,
    heading_degrees Float32,
    battery_level_percent Float32,
    fuel_level_percent Float32,
    sensor_readings String, -- JSON string for flexibility
    diagnostic_codes Array(String),
    operational_status LowCardinality(String),
    environmental_conditions String, -- JSON string
    custom_data String -- JSON string for extensibility
) ENGINE = MergeTree()
PARTITION BY toYYYYMM(recorded_at)
ORDER BY (vehicle_id, recorded_at)
TTL recorded_at + INTERVAL 90 DAY; -- Hot data retention

-- Aggregated Metrics Table
CREATE MATERIALIZED VIEW atlasmesh_analytics.vehicle_metrics_hourly
ENGINE = SummingMergeTree()
PARTITION BY toYYYYMM(hour)
ORDER BY (vehicle_id, hour)
AS SELECT
    vehicle_id,
    toStartOfHour(recorded_at) as hour,
    count() as telemetry_count,
    avg(speed_kmh) as avg_speed,
    max(speed_kmh) as max_speed,
    avg(battery_level_percent) as avg_battery,
    min(battery_level_percent) as min_battery
FROM atlasmesh_analytics.vehicle_telemetry
GROUP BY vehicle_id, hour;
```

## 10) Data Migration & Evolution

### 10.1 Schema Migration Strategy

```mermaid
flowchart TD
    A[Schema Change Request] --> B{Breaking Change?}
    B -->|Yes| C[Create Migration Plan]
    B -->|No| D[Direct Migration]
    
    C --> E[Backward Compatibility Check]
    E --> F[Blue-Green Deployment]
    F --> G[Data Validation]
    
    D --> H[Apply Migration]
    H --> I[Validate Schema]
    
    G --> J[Rollback Plan Ready]
    I --> J
    J --> K[Production Deployment]
    
    K --> L[Monitor Data Quality]
    L --> M[Migration Complete]
```

### 10.2 Data Versioning

- **Schema Versioning**: Semantic versioning for all schema changes
- **API Versioning**: Backward compatibility for 2 major versions
- **Data Contracts**: Formal contracts between services with validation
- **Evolution Strategy**: Additive changes preferred, breaking changes planned

This comprehensive data model documentation ensures proper understanding of the AtlasMesh Fleet OS data architecture, relationships, and governance policies.
