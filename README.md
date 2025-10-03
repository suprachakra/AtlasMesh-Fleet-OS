# AtlasMesh Fleet OS

**AtlasMesh Fleet OS** implements **qualified agnosticism** - a pragmatic, engineering-grounded operating system for Level-4 autonomous fleets. Rather than claiming universal compatibility, we achieve **bounded agnosticism** through:
- **Vehicle-Agnostic** (≤5% code delta): Class/model-bounded with certified profiles
- **Sector-Agnostic** (≥90% code reuse): Policy overlays for Defense → Mining → Logistics → Ride-hail
- **Platform-Agnostic** (100% conformance): Contract-driven infrastructure (Azure EKS → multi-cloud)
- **Variant Budget Enforcement**: Automated delta tracking with CI/CD gates and CCB workflow

[![License](https://img.shields.io/badge/License-BUSL%201.1-blue.svg)](LICENSE)
[![CI Status](https://img.shields.io/badge/CI-Passing-brightgreen.svg)](https://github.com/atlasmesh/fleet-os/actions)
[![Safety Case](https://img.shields.io/badge/Safety%20Case-Verified-brightgreen.svg)](docs/safety/safety_case_structure.md)
[![Documentation](https://img.shields.io/badge/Docs-Latest-blue.svg)](docs/README.md)

## What Sets Us Apart

- **Qualified Agnosticism**: Engineering-grounded bounded abstraction with automated variant budget enforcement (≤5% code delta)
- **Programmatic Proof Points**: 3-vehicle demo, 2-sector pilot, 2-cloud deploy validation within 90-180 days
- **Safety & Compliance as Code**: Automated evidence generation for ISO 26262/SOTIF/R155/R156
- **Contract-Driven Interfaces**: Vehicle HAL, sector policies, platform adapters with conformance testing
- **Multi-Dimensional Validation**: Vehicle × Sector × Platform test matrix with priority-based execution

## Overall Project Flow

```mermaid
graph TB
    subgraph "Phase 1: Core Platform (COMPLETED)"
        A1[CI/CD + Evidence Spine] --> A2[Core Services]
        A2 --> A3[Vehicle-Agnostic Edge]
        A3 --> A4[Control Center UI]
        A4 --> A5[Telemetry & Analytics]
        A5 --> A6[Security Foundations]
        A6 --> A7[API Contracts & SDKs]
        A7 --> A8[Performance Budgets]
        A8 --> A9[Feature Flags & Cohorting]
        A9 --> A10[Accessibility & UX Gates]
    end
    
    subgraph "Phase 2: Advanced Features (COMPLETED)"
        B1[Weather Fusion] --> B2[Predictive Maintenance]
        B2 --> B3[Evidence Engine]
        B3 --> B4[Sector Overlays]
        B4 --> B5[Digital Twin & Simulation]
        B5 --> B6[Scale & Resilience]
    end
    
    subgraph "Phase 3: Qualified Agnosticism (COMPLETED)"
        D1[Vehicle HAL] --> D2[Sensor Pack Registry]
        D2 --> D3[Variant Budget Enforcement]
        D3 --> D4[Conformance Testing]
        D4 --> D5[Platform Adapters]
        D5 --> D6[Evidence Automation]
    end
    
    subgraph "Core Operations Flow"
        C1[Mission Planning] --> C2[Policy Evaluation]
        C2 --> C3[Dispatch & Assignment]
        C3 --> C4[Vehicle Execution]
        C4 --> C5[Real-time Monitoring]
        C5 --> C6[Mission Completion]
        C5 -.-> C7[Tele-Assist Q&A]
        C7 -.-> C4
        C5 --> C8[Analytics & Learning]
        C8 --> C1
    end
    
    A10 --> B1
    B6 --> D1
    C1 -.-> A2
    C2 -.-> A6
    C3 -.-> A3
    C4 -.-> A3
    C5 -.-> A5
    D1 -.-> C4
    D3 -.-> A1
    D4 -.-> C1
    
    classDef completed fill:#d4edda,stroke:#28a745,stroke-width:2px
    classDef pending fill:#fff3cd,stroke:#ffc107,stroke-width:2px
    classDef operations fill:#e7f3ff,stroke:#007bff,stroke-width:2px
    
    class A1,A2,A3,A4,A5,A6,A7,A8,A9,A10 completed
    class B1,B2,B3,B4,B5,B6 completed
    class D1,D2,D3,D4,D5,D6 completed
    class C1,C2,C3,C4,C5,C6,C7,C8 operations
```

## User Journey - Complete System Interactions

```mermaid
sequenceDiagram
    participant U as Fleet Operator
    participant CC as Control Center UI
    participant AG as API Gateway
    participant AS as Auth Service
    participant PS as Policy Service
    participant DS as Dispatch Service
    participant TS as Trip Service
    participant RS as Routing Service
    participant FM as Fleet Manager
    participant VA as Vehicle Agent
    participant TI as Telemetry Ingestion
    participant FF as Feature Flags
    
    Note over U,FF: Mission Planning & Dispatch
    U->>CC: Login to Control Center
    CC->>AG: Authenticate user
    AG->>AS: Validate credentials
    AS-->>AG: JWT token + permissions
    AG-->>CC: Authentication success
    
    U->>CC: Create new mission
    CC->>AG: POST /api/v1/trips
    AG->>FF: Check feature flags
    FF-->>AG: Feature availability
    AG->>PS: Validate mission against policies
    PS-->>AG: Policy compliance check
    AG->>TS: Create trip request
    TS-->>AG: Trip created (ID: trip-123)
    AG-->>CC: Trip creation success
    
    Note over U,FF: Vehicle Assignment & Route Planning
    CC->>AG: Request vehicle assignment
    AG->>DS: Dispatch trip-123
    DS->>FM: Query available vehicles
    FM-->>DS: Vehicle candidates
    DS->>PS: Check vehicle-mission compatibility
    PS-->>DS: Compatibility matrix
    DS->>RS: Calculate optimal routes
    RS-->>DS: Route options with ETAs
    DS-->>AG: Assignment: vehicle-001 → trip-123
    AG-->>CC: Assignment confirmed
    
    Note over U,FF: Mission Execution
    DS->>VA: Send mission to vehicle-001
    VA->>VA: Load vehicle profile & policies
    VA->>RS: Get detailed route
    RS-->>VA: Turn-by-turn navigation
    VA->>FM: Report mission start
    FM-->>CC: Live status update
    
    loop Real-time Monitoring
        VA->>TI: Stream telemetry data
        TI->>TI: Validate against Avro schema
        TI->>CC: Real-time dashboard updates
        
        alt Policy Violation Detected
            VA->>PS: Policy violation alert
            PS->>CC: Alert notification
            CC->>U: Display alert
            U->>CC: Acknowledge/take action
        end
        
        alt Emergency Situation
            U->>CC: Emergency stop
            CC->>AG: Emergency command
            AG->>VA: Immediate stop
            VA-->>AG: Stop confirmed
            AG-->>CC: Emergency stop executed
        end
    end
    
    Note over U,FF: Mission Completion
    VA->>TS: Mission completed
    TS->>TI: Log completion event
    TS-->>CC: Mission status update
    CC->>U: Mission completion notification
    
    Note over U,FF: Analytics & Reporting
    U->>CC: View mission analytics
    CC->>AG: GET /api/v1/analytics/trips/trip-123
    AG->>TI: Query processed telemetry
    TI-->>AG: Mission metrics & KPIs
    AG-->>CC: Analytics data
    CC->>U: Display performance dashboard
```

## Data Flow Diagram - Complete Platform Data Movement

```mermaid
graph TB
    subgraph "Edge Layer"
        V1[Vehicle Sensors] --> VA[Vehicle Agent]
        V2[Vehicle Actuators] <-- VA
        VA --> CB[Cloud Bridge]
        CB --> MQ1[MQTT/WebSocket]
    end
    
    subgraph "Ingestion Layer"
        MQ1 --> KF[Kafka Event Bus]
        KF --> SR[Schema Registry]
        SR --> TI[Telemetry Ingestion]
        TI --> DV[Data Validation]
    end
    
    subgraph "Storage Layer - Hot Path"
        DV --> CH[ClickHouse]
        DV --> RD[Redis Cache]
        CH --> GF[Grafana Dashboards]
        RD --> CC[Control Center UI]
    end
    
    subgraph "Storage Layer - Cold Path"
        DV --> MO[MinIO Object Storage]
        MO --> DL[Data Lineage Service]
        DL --> NG[Neo4j Graph DB]
    end
    
    subgraph "Core Services Data Flow"
        AG[API Gateway] <--> PS[Policy Service]
        AG <--> TS[Trip Service]
        AG <--> DS[Dispatch Service]
        AG <--> RS[Routing Service]
        AG <--> FM[Fleet Manager]
        AG <--> AS[Auth Service]
        
        PS --> PG[(PostgreSQL)]
        TS --> PG
        DS --> PG
        RS --> PG
        FM --> PG
        AS --> PG
    end
    
    subgraph "Feature Management"
        FF[Feature Flags] --> RD
        FF --> AG
    end
    
    subgraph "Security & Compliance"
        VT[Vault Secrets] --> AS
        VT --> PS
        OPA[Open Policy Agent] --> PS
        OPA --> AG
    end
    
    subgraph "Monitoring & Observability"
        PM[Prometheus Metrics] --> GF
        JG[Jaeger Tracing] --> GF
        LG[Log Aggregation] --> GF
        
        TI --> PM
        AG --> PM
        PS --> PM
        TS --> PM
        DS --> PM
        RS --> PM
        FM --> PM
    end
    
    subgraph "External Integrations"
        ERP[ERP/WMS/TOS] <--> AG
        MAP[Map Providers] --> RS
        WX[Weather Services] --> PS
        PKI[Certificate Authority] --> VT
    end
    
    subgraph "Analytics & ML Pipeline"
        CH --> ML[ML Pipeline]
        MO --> ML
        ML --> FS[Feature Store]
        ML --> MR[Model Registry]
        FS --> PM[Predictive Models]
        MR --> PM
    end
    
    %% Data Flow Annotations
    KF -.->|"Real-time Events"| TI
    CH -.->|"Time-series Queries"| GF
    MO -.->|"Historical Analysis"| ML
    NG -.->|"Data Lineage"| DL
    RD -.->|"Session Data"| CC
    PG -.->|"Transactional Data"| AG
    
    %% Styling
    classDef edge fill:#e1f5fe,stroke:#0277bd,stroke-width:2px
    classDef ingestion fill:#f3e5f5,stroke:#7b1fa2,stroke-width:2px
    classDef storage fill:#e8f5e8,stroke:#2e7d32,stroke-width:2px
    classDef services fill:#fff3e0,stroke:#ef6c00,stroke-width:2px
    classDef security fill:#ffebee,stroke:#c62828,stroke-width:2px
    classDef monitoring fill:#f1f8e9,stroke:#33691e,stroke-width:2px
    classDef external fill:#fafafa,stroke:#424242,stroke-width:2px
    classDef analytics fill:#e0f2f1,stroke:#00695c,stroke-width:2px
    
    class V1,V2,VA,CB,MQ1 edge
    class KF,SR,TI,DV ingestion
    class CH,RD,MO,DL,NG,PG storage
    class AG,PS,TS,DS,RS,FM,AS,FF services
    class VT,OPA,PKI security
    class PM,JG,LG,GF monitoring
    class ERP,MAP,WX external
    class ML,FS,MR,PM analytics
```

## Key Features

### Qualified Agnosticism Services (Phase 3)
- **Vehicle HAL**: Profile-driven hardware abstraction (Terminal Tractor, Mine Haul, UTV, Defense, Passenger, Transit)
- **Variant Budget**: Automated delta tracking with ≤5% code delta enforcement and CI/CD gates
- **Conformance Testing**: Multi-dimensional test matrix (3 vehicles × 4 sectors × 3 platforms = 36 critical paths)
- **Sensor Pack Registry**: Certified packs (Rugged-A, Urban-B, Highway-C) with calibration automation
- **Platform Adapters**: Storage/messaging/security abstraction for Azure/AWS/on-prem

### Core Platform
- **Policy Engine**: Rules-as-code for dispatch, ROE, degraded modes, ODD limits, and sector overlays
- **Trip Service**: Idempotent, sector-neutral lifecycle with FSM
- **Dispatch & Rebalancing**: Assignment/pooling with oscillation damping
- **Routing & ETA**: Constraint-aware routing with GNSS-deny resilience
- **Energy Manager**: SOC forecasts, charger queues, and tariff optimization

### Operations & Safety
- **Fleet Health & Alerts**: Live KPIs and SLOs with runbooks
- **Predictive Maintenance**: RUL models driving work orders
- **Evidence Engine**: Safety bundles and audit kits for regulators
- **Weather Fusion**: Multi-source weather with credibility/freshness tradeoffs
- **Geospatial DB**: Data provenance tracking with lineage

### User Interfaces
- **Control Center**: Map-first operations console with trip timeline
- **Vehicle Management**: Fleet-wide health monitoring and management
- **Garage PC**: Bay/drive status, bulk OTA, and pre-provisioning
- **Tele-Assist**: Q&A interface (no direct driving) with assist budgets

### Integration & Security
- **Adapter SDK**: Certified connectors for enterprise systems
- **OTA Manager**: Signed, staged, and attested updates
- **V2X/PKI**: Secure vehicle-to-everything communication
- **Telemetry Ingest**: Schema-validated data pipeline

## Architecture & Tech Stack

### System Context (C4)

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

### Technology Stack

| Component | Technologies |
|-----------|-------------|
| **Backend Services** | Go, Rust, Python, Node.js |
| **Frontend** | React, TypeScript, WebGL |
| **Data Pipeline** | Kafka, Avro/Protobuf, dbt |
| **Storage** | PostgreSQL, TimescaleDB, S3/MinIO |
| **ML/Analytics** | PyTorch, MLflow, Feast |
| **Deployment** | Kubernetes, Helm, Terraform |
| **Observability** | Prometheus, Grafana, OpenTelemetry |
| **Security** | mTLS, Vault, SPIFFE/SPIRE |
| **Policy** | OPA/Rego, Cedar |

## Repository Structure

```
atlasmesh-fleet-os/
├─ README.md                             # This file
├─ CONTRIBUTING.md                       # Contribution guidelines
├─ CODEOWNERS                            # Per-folder owners
├─ LICENSE                               # BUSL 1.1
├─ SECURITY.md                           # Security policy
├─ PRODUCTION_RUNBOOK.md                 # Production operations guide
├─ .github/                              # GitHub workflows and templates
├─ docs/                                 # Documentation
│  ├─ strategy/                          # Vision, market, OKRs, product strategy
│  ├─ Technical/                         # Architecture, requirements, APIs
│  │  ├─ 01_Architecture.md              # System architecture
│  │  ├─ 08_Qualified_Agnosticism.md     # Qualified agnosticism guide
│  │  ├─ 09_Five_Constraining_Realities.md  # Technical constraints
│  │  ├─ 10_Feasibility_Scorecard.md     # Feasibility analysis
│  │  ├─ 11_Agnostic_By_Contract.md      # Contract boundaries
│  │  ├─ 12_Cross_Department_Checklist.md   # Organizational checklist
│  │  ├─ 13_Programmatic_Proof_Points.md    # 90-180 day validation plan
│  │  └─ 14_Architecture_Reality_Check.md   # Implementation guide
│  ├─ ADR/                               # Architecture Decision Records
│  │  ├─ 0011-qualified-agnosticism.md   # ADR for qualified agnosticism
│  │  ├─ 0012-variant-budget-enforcement.md  # ADR for variant budgets
│  │  └─ 0013-conformance-testing.md     # ADR for conformance testing
│  ├─ prd/use-cases/                     # Use case specifications by sector
│  ├─ architecture/                      # Diagrams and technical designs
│  ├─ diagrams/                          # Mermaid diagrams-as-code
│  └─ troubleshooting/                   # Troubleshooting guides
├─ configs/                              # Configuration overlays
│  ├─ base/                              # Base configurations
│  ├─ sectors/                           # Sector-specific overlays
│  ├─ vehicles/                          # Vehicle profiles (YAML)
│  │  ├─ terminal_tractor_v2.yaml        # Terminal Tractor profile
│  │  └─ mine_haul_400t.yaml             # Mine Haul truck profile
│  ├─ sensor-packs/                      # Sensor pack definitions (NEW)
│  │  ├─ schema.json                     # Sensor pack schema
│  │  ├─ rugged-a.json                   # Rugged Pack A (mining/defense)
│  │  ├─ urban-b.json                    # Urban Pack B (ride-hail/transit)
│  │  └─ highway-c.json                  # Highway Pack C (logistics)
│  ├─ cities/                            # City/region configurations
│  ├─ odd/                               # ODD rules by sector
│  └─ nfrs/                              # Non-functional requirements
├─ services/                             # 50+ microservices
│  ├─ vehicle-hal/                       # Vehicle Hardware Abstraction Layer (NEW)
│  ├─ variant-budget/                    # Variant Budget Enforcement (NEW)
│  ├─ conformance-testing/               # Conformance Testing Framework (NEW)
│  ├─ sensor-pack-registry/              # Sensor Pack Registry (NEW)
│  ├─ platform-adapters/                 # Platform Adapters (NEW)
│  ├─ policy-engine/                     # Policy Engine (Enhanced for sector overlays)
│  ├─ sector-overlays/                   # Sector Overlay Management
│  ├─ fleet-manager/                     # Fleet management
│  ├─ vehicle-gateway/                   # Vehicle communication
│  ├─ auth-service/                      # Authentication & authorization
│  └─ ... 40+ more services
├─ edge-stack/                           # Edge/vehicle components
│  └─ vehicle-agent/                     # ROS2-based vehicle agent
├─ ui/                                   # User interfaces
│  └─ control-center/                    # React-based control center
├─ testing/                              # Test suites
│  ├─ conformance/                       # Conformance test runner (NEW)
│  │  └─ runner.py                       # Multi-dimensional test matrix
│  ├─ e2e/                               # End-to-end tests
│  ├─ integration/                       # Integration tests
│  ├─ performance/                       # Performance tests
│  └─ chaos/                             # Chaos engineering tests
├─ database/                             # Database schemas and migrations
├─ infrastructure/                       # Infrastructure as code
└─ tools/                                # Development tools
```

## Roadmap

| Phase | Timeline | Focus | Milestones | Status |
|-------|----------|-------|------------|--------|
| **Phase 1** | Months 1-6 | Core Platform, Safety Framework, Edge Stack | Trip service, dispatch, routing, policy engine, control center | ✅ **COMPLETED** |
| **Phase 2** | Months 7-12 | Advanced Features, Production Readiness | Weather fusion, PdM, evidence engine, digital twin | ✅ **COMPLETED** |
| **Phase 3** | Months 13-18 | Qualified Agnosticism Implementation | Vehicle HAL, sensor packs, variant budget, conformance testing | ✅ **COMPLETED** |
| **Phase 4** | Months 19-24 | Programmatic Validation & Scale | 3-vehicle demo, 2-sector pilot, 2-cloud deploy, evidence bundle | 🔄 **IN PROGRESS** |

**Key Deliverables (Phase 3 - Completed)**:
- ✅ Vehicle HAL with certified profiles (Terminal Tractor V2, Mine Haul 400T)
- ✅ Sensor Pack Registry with 3 certified packs (Rugged-A, Urban-B, Highway-C)
- ✅ Variant Budget service with automated delta tracking and CI/CD enforcement
- ✅ Conformance Testing framework with multi-dimensional test matrix
- ✅ Platform Adapters for storage/messaging/security abstraction
- ✅ Complete technical documentation (7 new docs + 3 ADRs)

**Next Milestones (Phase 4 - 90-180 Days)**:
- 🎯 3-Vehicle Demo: UTV + Terminal Tractor + Mine Haul (≥95% code reuse)
- 🎯 2-Sector Pilot: Defense + Mining overlays (≥90% code share)
- 🎯 2-Cloud Deploy: Azure EKS + on-prem K3s (100% conformance)
- 🎯 Pack Swap: Rugged-A ↔ Urban-B (≤30min swap time)
- 🎯 Evidence Bundle: Automated regulatory compliance artifacts

See [Product Roadmap](docs/strategy/12_Product_Roadmap_and_Milestones.md) for the detailed roadmap and release plan.

## Prerequisites

- Docker 20.10+
- Kubernetes 1.24+ (for production deployment)
- Go 1.21+
- Node.js 18+
- Python 3.10+
- Rust 1.70+ (optional)

## Environment Setup

1. Clone the repository:
   ```bash
   git clone https://github.com/atlasmesh/fleet-os.git
   cd fleet-os
   ```

2. Run the bootstrap script:
   ```bash
   ./scripts/bootstrap.sh
   ```

3. Set up environment variables:
   ```bash
   cp configs/env/.env.example configs/env/.env
   # Edit .env with your configuration
   ```

## Running Locally

Start the core services with demo data:

```bash
./scripts/run_local.sh
```

This will:
- Start required services in Docker containers
- Seed demo data (vehicles, trips, maps)
- Launch the Control Center UI at http://localhost:3000

For sector-specific configurations:

```bash
./scripts/run_local.sh --sector defense
./scripts/run_local.sh --sector mining
./scripts/run_local.sh --sector logistics
./scripts/run_local.sh --sector ride-hail
```

## Testing

Run the test suite:

```bash
make test
```

Run specific test categories:

```bash
make test-unit        # Unit tests
make test-contract    # Contract tests for adapters
make test-integration # Integration tests
make test-e2e         # End-to-end tests
make test-sim         # Simulation tests
make test-conformance # Multi-dimensional conformance testing (NEW)
```

### Qualified Agnosticism Conformance Testing

Run the multi-dimensional conformance test matrix:

```bash
# Dry run to validate configuration
python testing/conformance/runner.py --dry-run

# Execute full conformance suite (3 vehicles × 4 sectors × 3 platforms = 36 tests)
python testing/conformance/runner.py

# Run specific priority level
python testing/conformance/runner.py --priority critical
python testing/conformance/runner.py --priority high
```

**Test Matrix Dimensions:**
- **Vehicles**: ClassA_LightIndustrial, ClassB_HeavyDuty, ClassC_Mining
- **Sectors**: defense, mining, logistics, ride_hail
- **Platforms**: azure_eks, aws_eks, on_prem_k3s

**Evidence Output**: `testing/conformance/output/conformance_results_*.json`

## Contributing

Please read [CONTRIBUTING.md](CONTRIBUTING.md) for details on our code of conduct, branch naming conventions, commit message format, and pull request process.

Key points:
- Fork the repository and create a feature branch
- Follow the coding style and documentation standards
- Add tests for new functionality
- Update documentation as needed
- Submit a pull request with a clear description

## Security

See [SECURITY.md](SECURITY.md) for details on:
- Vulnerability disclosure policy
- Security SLAs
- Reporting process
- PGP key for encrypted communication

## Compliance & Safety Certification

AtlasMesh Fleet OS is designed to meet regulatory requirements across all supported sectors through **automated evidence generation**:

### Automotive Safety Standards
- **ISO 26262**: Functional safety (per-vehicle model certification with automated evidence)
- **ISO 21448 (SOTIF)**: Safety of intended functionality (scenario-based validation)
- **UNECE R155**: Cybersecurity requirements (mTLS, PKI, threat modeling)
- **UNECE R156**: Software update security (signed OTA, rollback, attestation)
- **ISO 21434**: Automotive cybersecurity engineering

### Sector-Specific Compliance
- **Defense**: NIST 800-53, Common Criteria, STIG compliance
- **Mining**: MSHA Part 56, ISO 19296 (mining machinery safety)
- **Logistics**: DOT regulations, FMCSA compliance
- **Ride-hail**: Local transportation authority requirements (Dubai RTA, Abu Dhabi DOT)

### Data Privacy & Residency
- **GDPR**: European data protection regulation
- **UAE PDPL**: UAE Personal Data Protection Law
- **Purpose Binding**: Data pipeline controls with DPIA workflows
- **Data Residency**: Regional deployment carve-outs for compliance

### Automated Evidence Generation
Compliance artifacts are generated automatically as part of the release process:
- **Safety Case Deltas**: Per-vehicle profile evidence generation
- **SOTIF Validation**: Scenario coverage and validation evidence
- **Cybersecurity Evidence**: UN R155/R156 compliance documentation
- **Audit Trail**: Cryptographically signed decision logs

**Evidence Bundle Export**: One-click regulatory package export in `testing/conformance/output/`

## License

This project is licensed under the Business Source License 1.1 - see the [LICENSE](LICENSE) file for details.

## Documentation

### Quick Links
- **[Architecture Overview](docs/Technical/01_Architecture.md)** - Complete system architecture
- **[Qualified Agnosticism Guide](docs/Technical/08_Qualified_Agnosticism.md)** - Implementation approach
- **[Feasibility Scorecard](docs/Technical/10_Feasibility_Scorecard.md)** - Detailed feasibility analysis
- **[Programmatic Proof Points](docs/Technical/13_Programmatic_Proof_Points.md)** - 90-180 day validation plan
- **[Service Registry](docs/Technical/07_Service_Registry.md)** - All 50+ microservices documented
- **[API Reference](docs/api/API_REFERENCE.md)** - Complete API documentation
- **[Deployment Guide](docs/deployment/DEPLOYMENT_GUIDE.md)** - Production deployment procedures

### Qualified Agnosticism Documentation
- **[Five Constraining Realities](docs/Technical/09_Five_Constraining_Realities.md)** - Physics, safety, ODD, sensors, regulations
- **[Agnostic By Contract](docs/Technical/11_Agnostic_By_Contract.md)** - Contract-driven boundaries
- **[Cross-Department Checklist](docs/Technical/12_Cross_Department_Checklist.md)** - Organizational alignment
- **[Architecture Reality Check](docs/Technical/14_Architecture_Reality_Check.md)** - Concrete implementation

### Architecture Decision Records
- **[ADR-0011: Qualified Agnosticism](docs/ADR/0011-qualified-agnosticism.md)** - Framework decision
- **[ADR-0012: Variant Budget Enforcement](docs/ADR/0012-variant-budget-enforcement.md)** - Automated enforcement
- **[ADR-0013: Conformance Testing](docs/ADR/0013-conformance-testing.md)** - Multi-dimensional validation

## Contact

- Website: [atlasmesh.io](https://atlasmesh.io)
- Email: [info@atlasmesh.io](mailto:info@atlasmesh.io)
- Documentation: [docs.atlasmesh.io](https://docs.atlasmesh.io)

---

**AtlasMesh Fleet OS - Engineering-Grounded Qualified Agnosticism for Autonomous Fleets** 🚗💨
