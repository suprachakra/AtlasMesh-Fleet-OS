<div align="center">

# 🚙 AtlasMesh Fleet OS

**The Universal Fleet Management System That Actually Works - One Codebase, Any Vehicle, Every Sector, Any Cloud** 🚀

[![License](https://img.shields.io/badge/License-BUSL%201.1-blue.svg)](LICENSE)
[![CI Status](https://img.shields.io/badge/CI-Passing-brightgreen.svg)](https://github.com/atlasmesh/fleet-os/actions)
[![Safety Case](https://img.shields.io/badge/Safety%20Case-Verified-brightgreen.svg)](docs/safety/safety_case_structure.md)
[![Documentation](https://img.shields.io/badge/Docs-Latest-blue.svg)](docs/README.md)
[![Product Management](https://img.shields.io/badge/PM%20Framework-Evidence--First-purple.svg)](docs/strategy/PM_COP_MAPPING.md)

</div>

---

## 📋 Table of Contents

<div align="center">

| 🎯 **[What Makes AtlasMesh Different](#-what-makes-atlasmesh-different)** | ✨ **[Key Features](#-key-features)** | 🗺️ **[Roadmap](#-roadmap)** | 🚀 **[Quick Start](#-quick-start)** |
|:---:|:---:|:---:|:---:|
| **Core Value Proposition** | **Technical Capabilities** | **Development Timeline** | **Get Started in Minutes** |

| 🏃‍♂️ **[Running Locally](#-running-locally)** | 🧪 **[Testing](#-testing)** | 🤝 **[Contributing](#-contributing)** | 🔒 **[Security & Compliance](#-security)** |
|:---:|:---:|:---:|:---:|
| **Local Development** | **Test Suite & Conformance** | **How to Contribute** | **Safety & Regulatory** |

| 📚 **[Documentation](#documentation)** | 🛡️ **[PM Framework](#product-management-framework)** | 📄 **[License](#-license)** |
|:---:|:---:|:---:|
| **Technical Documentation** | **Product Management** | **Business Source License** |

</div>

---

## 🎯 What Makes AtlasMesh Different?

AtlasMesh Fleet Management System implements **qualified agnosticism** - a pragmatic, engineering-grounded approach to autonomous fleet management. Rather than claiming universal compatibility, we achieve **bounded agnosticism** through:

### **🔄 Fleet Management Workflow**

```mermaid
flowchart TB
 subgraph subGraph0["Fleet Management Workflow"]
        B["Fleet Manager"]
        A["Vehicle Fleet"]
        C["Multi-Fleet Coordination"]
        D["Fleet Optimization"]
        E["Fleet Analytics"]
        F["Resource Sharing"]
        G["Cross-Fleet Optimization"]
        H["AI-Powered Algorithms"]
        I["Performance Tuning"]
        J["Real-Time Insights"]
        K["Health Scoring"]
        L["Predictive Maintenance"]
        M["Fleet Performance"]
        N["Continuous Improvement"]
  end
 subgraph subGraph1["Vehicle Retrofit Process"]
        P["Assessment"]
        O["Existing Fleet"]
        Q["Retrofit Planning"]
        R["AtlasMesh Integration"]
        S["Testing & Validation"]
        T["Deployment"]
        U["Autonomous Operations"]
  end
    A --> B
    B --> C & D & E
    C --> F & G
    D --> H & I
    E --> J & K & L
    F --> M
    G --> M
    H --> M
    I --> M
    J --> M
    K --> M
    L --> M
    M --> N
    N --> B
    O --> P
    P --> Q
    Q --> R
    R --> S
    S --> T
    T --> U

     B:::Class_01
     A:::Class_01
     C:::Class_01
     D:::Class_01
     E:::Class_01
     F:::Class_01
     G:::Class_01
     H:::Class_01
     I:::Class_01
     J:::Class_01
     K:::Class_01
     L:::Class_01
     M:::Class_01
     N:::Class_01
     P:::Sky
     O:::Sky
     Q:::Sky
     R:::Sky
     S:::Sky
     T:::Sky
     U:::Sky
    classDef Sky stroke-width:1px, stroke-dasharray:none, stroke:#374D7C, fill:#E2EBFF, color:#374D7C
    classDef Class_01 stroke:#d0ecb4, fill:#d0ecb4, color:#64a028
    style subGraph0 fill:transparent
    style subGraph1 fill:transparent
```

<div align="center">

| 🚚 **Vehicle-Agnostic** | 🏭 **Sector-Agnostic** | ☁️ **Platform-Agnostic** | 📊 **Variant Budget** |
|:---:|:---:|:---:|:---:|
| ≤5% code delta | ≥90% code reuse | 100% conformance | Automated tracking |
| Certified profiles | Policy overlays | Contract-driven | CI/CD gates |

</div>

**Multi-Sector Support**: Defense → Mining → Logistics → Ride-hail  
**Multi-Platform**: Azure EKS → AWS EKS → On-prem K3s  
**Multi-Vehicle**: Terminal Tractors → Mine Hauls → UGVs → Passenger Vehicles 

---

## 🚨 **What's the Actual Need?**

Autonomous and semi-autonomous operations are scaling across **multiple vehicle types, sectors, and infrastructures** - but ops today are stitched together with single-OEM tools, bespoke scripts and manual evidence collection. That creates five systemic gaps:

<div align="center">

| 🎯 **1. Fragmented Control** | 📋 **2. Safety & Compliance Drag** | 🏗️ **3. Site Onboarding Friction** | 👥 **4. Operator Overload** | 🔒 **5. Vendor/Platform Lock-in** |
|:---:|:---:|:---:|:---:|:---:|
| **Problem** | **Problem** | **Problem** | **Problem** | **Problem** |
| No single place to monitor, command, and troubleshoot mixed fleets | Evidence and policy conformance are manual, slow, and error-prone | Every new depot/city/sector feels "custom," inflating cost and time-to-value | Alerts, assists, and incidents lack context; triage is slow; handovers are risky | Cloud/provider specifics and vehicle peculiarities creep into the app layer |

</div>

**🎯 Bottom line:** Operators need a **single, autonomy-grade Fleet Management System** that runs mixed fleets safely, proves compliance automatically, and scales across sectors and platforms without forking core code.

---

## 💎 **Value Proposition**

**AtlasMesh Fleet Management System** is the **autonomy-grade, agnostic Fleet Management System** that lets you operate **mixed-OEM, multi-sector fleets** with **built-in safety, compliance evidence, and site-ready tooling**—without forking your core.

### 🏗️ **The 7 Value Pillars (with Measurable Outcomes)**

<div align="center">

| 🎯 **1. Agnostic by Design** | 🛡️ **2. Safety & Compliance, Automated** | 🚀 **3. Live Operations that Scale** | 🏗️ **4. Rapid Site Onboarding** |
|:---:|:---:|:---:|:---:|
| **Outcome** | **Outcome** | **Outcome** | **Outcome** |
| New model/sector onboarded in **≤4–6 weeks** with zero core forks | **100% audit-ready** builds; evidence pack creation **↓ 80–90%** effort | **Assist rate ≤ 2 / 1,000 km**; **availability ≥ 99.0–99.5%** | First-site time-to-ops **≤ 90 days**; additional sites **≤ 30–45 days** |

| 💰 **5. Cost & Performance Control** | 🚀 **6. Developer & Partner Velocity** | 🤝 **7. Trust & Transparency** |
|:---:|:---:|:---:|
| **Outcome** | **Outcome** | **Outcome** |
| **TCO ↓ 15–25%**, energy spend **↓ 10–15%**, MTTR **≤ 1 hour** | Feature lead time **↓ 30–40%**; rollback ≤ **15 minutes** | Operator SUS **≥ 80**, faster investigations, fewer escalations |

</div>

### 🎯 **Business Impact You Can Commit To**

<div align="center">

| ⏰ **Time-to-Deploy** | 💰 **OpEx Reduction** | 📊 **Availability** | 📋 **Compliance** | 🚀 **Engineering** |
|:---:|:---:|:---:|:---:|:---:|
| ≤ 90 days (then ≤ 45 days) | 15–25% via assists ↓, energy/predictive maintenance | ≥ 99.0% (path to 99.5%) | Evidence generation **days → hours** | +30–40% without quality regressions |

</div>

---

## ✨ What Sets Us Apart

<div align="center">

| 🎯 **Qualified Agnosticism** | 🧪 **Programmatic Proof** | 🛡️ **Safety as Code** | 🔗 **Contract-Driven** | 🧪 **Multi-Dimensional** |
|:---:|:---:|:---:|:---:|:---:|
| Engineering-grounded bounded abstraction | 3-vehicle demo, 2-sector pilot | Automated evidence generation | Vehicle HAL, sector policies | Vehicle × Sector × Platform |
| ≤5% code delta enforcement | 90-180 day validation | ISO 26262/SOTIF/R155/R156 | Platform adapters | Test matrix with priority execution |

</div>

### 🚀 **Key Differentiators**

- **🎯 Evidence-First Product Management**: No PRD without evidence pack (5-7 interviews + telemetry + sim)
- **📊 Automated Variant Budget**: Real-time tracking with CI/CD gates and CCB workflow
- **🛡️ Safety-Certified**: Per-vehicle ISO 26262/SOTIF compliance with automated evidence
- **🌍 Extreme Weather Ready**: 55°C desert operations with thermal derating and dust protocols
- **🔄 Continuous Validation**: Multi-dimensional test matrix (3 vehicles × 4 sectors × 3 platforms = 36 critical paths)

## Customer Journey & Success Process

### **🎯 Customer Success Journey**

```mermaid
flowchart TD
    A[Customer Inquiry] --> B[Discovery Call]
    B --> C[Technical Assessment]
    C --> D[Pilot Proposal]
    D --> E[90-Day Pilot]
    E --> F{Pilot Success?}
    
    F -->|Yes| G[Production Deployment]
    F -->|No| H[Iteration & Improvement]
    H --> E
    
    G --> I[Fleet Integration]
    I --> J[WMS/Mining FMS/Military Integration]
    J --> K[Go-Live]
    K --> L[Continuous Optimization]
    L --> M[ROI Achievement]
    M --> N[Expansion & Scaling]
    
    subgraph "Success Metrics"
        O[99.3% Uptime]
        P[15% Efficiency Gain]
        Q[40% Cost Reduction]
        R[14-Month Payback]
    end
    
    M --> O
    M --> P
    M --> Q
    M --> R
```

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
    
    %% SubGraph styling
    subgraph "Phase 1: Core Platform (COMPLETED)" fill:transparent
    end
    subgraph "Phase 2: Advanced Features (COMPLETED)" fill:transparent
    end
    subgraph "Phase 3: Qualified Agnosticism (COMPLETED)" fill:transparent
    end
    subgraph "Core Operations Flow" fill:transparent
    end
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
    participant TS as Mission Management
    participant RS as Routing Service
    participant FM as Fleet Manager
    participant VA as Vehicle Agent
    participant TI as Telemetry Ingestion
    participant FF as Feature Flags
    
    rect rgb(255, 248, 240)
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
    end
    
    rect rgb(255, 248, 240)
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
    end
    
    rect rgb(255, 248, 240)
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
    end
    
    rect rgb(255, 248, 240)
    Note over U,FF: Mission Completion
    VA->>TS: Mission completed
    TS->>TI: Log completion event
    TS-->>CC: Mission status update
    CC->>U: Mission completion notification
    end
    
    rect rgb(255, 248, 240)
    Note over U,FF: Analytics & Reporting
    U->>CC: View mission analytics
    CC->>AG: GET /api/v1/analytics/trips/trip-123
    AG->>TI: Query processed telemetry
    TI-->>AG: Mission metrics & KPIs
    AG-->>CC: Analytics data
    CC->>U: Display performance dashboard
    end
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
        AG <--> TS[Mission Management]
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
    
    %% SubGraph styling
    subgraph "Edge Layer" fill:transparent
    end
    subgraph "Ingestion Layer" fill:transparent
    end
    subgraph "Storage Layer - Hot Path" fill:transparent
    end
    subgraph "Storage Layer - Cold Path" fill:transparent
    end
    subgraph "Core Services Data Flow" fill:transparent
    end
    subgraph "Feature Management" fill:transparent
    end
    subgraph "Security & Compliance" fill:transparent
    end
    subgraph "Monitoring & Observability" fill:transparent
    end
    subgraph "External Integrations" fill:transparent
    end
    subgraph "Analytics & ML Pipeline" fill:transparent
    end
```

## 🚀 Key Features

<div align="center">

### 🎯 **Qualified Agnosticism Services** *(Phase 3 - Completed)*

| 🚚 **Vehicle HAL** | 📊 **Variant Budget** | 🧪 **Conformance Testing** | 📦 **Sensor Packs** | ☁️ **Platform Adapters** |
|:---:|:---:|:---:|:---:|:---:|
| Profile-driven hardware abstraction | Automated delta tracking | Multi-dimensional test matrix | Certified packs (Rugged/Urban/Highway) | Storage/messaging/security abstraction |
| Terminal Tractor, Mine Haul, UTV | ≤5% code delta enforcement | 3×4×3 = 36 critical paths | Calibration automation | Azure/AWS/on-prem |

</div>

### 🏗️ **Core Platform**

<div align="center">

| ⚖️ **Policy Engine** | 🚌 **Mission Management** | 🎯 **Dispatch & Rebalancing** | 🗺️ **Routing & ETA** | ⚡ **Energy Manager** |
|:---:|:---:|:---:|:---:|:---:|
| Rules-as-code for dispatch | Idempotent, sector-neutral lifecycle | Assignment/pooling with oscillation damping | Constraint-aware routing | SOC forecasts, charger queues |
| ROE, degraded modes, ODD limits | FSM-based state management | Real-time optimization | GNSS-deny resilience | Tariff optimization |

</div>

### 🛡️ **Operations & Safety**

<div align="center">

| 📊 **Fleet Health** | 🔧 **Predictive Maintenance** | 📋 **Evidence Engine** | 🌤️ **Weather Fusion** | 🗺️ **Geospatial DB** |
|:---:|:---:|:---:|:---:|:---:|
| Live KPIs and SLOs | RUL models driving work orders | Safety bundles and audit kits | Multi-source weather | Data provenance tracking |
| Runbooks integration | Predictive analytics | Regulatory compliance | Credibility/freshness tradeoffs | Lineage tracking |

</div>

### 🖥️ **User Interfaces**

<div align="center">

| 🎮 **Control Center** | 🚚 **Vehicle Management** | 🔧 **Garage PC** | 🆘 **Tele-Assist** |
|:---:|:---:|:---:|:---:|
| Map-first operations console | Fleet-wide health monitoring | Bay/drive status, bulk OTA | Q&A interface (no direct driving) |
| Trip timeline visualization | Real-time diagnostics | Pre-provisioning | Assist budgets |

</div>

### 🔗 **Integration & Security**

<div align="center">

| 🔌 **Adapter SDK** | 📱 **OTA Manager** | 🔐 **V2X/PKI** | 📊 **Telemetry Ingest** |
|:---:|:---:|:---:|:---:|
| Certified connectors | Signed, staged, attested updates | Secure vehicle-to-everything | Schema-validated data pipeline |
| Enterprise systems | Automatic rollback | PKI-based authentication | Real-time processing |

</div>

## Architecture & Tech Stack

### System Context (C4)

```mermaid
C4Context
    title AtlasMesh Fleet Management System - System Context

    Person(operator, "Fleet Operator", "Manages fleet operations")
    Person(maintainer, "Maintenance Technician", "Services vehicles")
    Person(regulator, "Regulator/Auditor", "Verifies compliance")
    Person(rider, "Rider/End User", "Uses service (ride-hail)")
    
    System_Boundary(atlasmesh, "AtlasMesh Fleet Management System") {
        System(fms, "Fleet Management System", "Orchestrates fleet operations")
        System(vehicle_agent, "Vehicle Agent", "On-vehicle edge software")
    }
    
    System_Ext(enterprise, "Enterprise Systems", "WMS/TOS/ERP")
    System_Ext(maps, "Map Providers", "HERE/OSM/etc.")
    System_Ext(weather, "Weather Services", "Forecast/nowcast")
    System_Ext(cloud, "Cloud Infrastructure", "AWS/Azure/GCP/On-prem")
    
    Rel(operator, fms, "Uses")
    Rel(maintainer, vehicle_agent, "Services")
    Rel(regulator, fms, "Audits")
    Rel(rider, fms, "Interacts with")
    
    Rel(fms, vehicle_agent, "Controls")
    Rel(fms, enterprise, "Integrates with")
    Rel(fms, maps, "Consumes data from")
    Rel(fms, weather, "Consumes data from")
    Rel(fms, cloud, "Deployed on")
```

### **🏗️ Complete System Architecture**

```mermaid
graph TB
    subgraph "AtlasMesh Fleet Management System Architecture"
        subgraph "Cloud Platform"
            A[Fleet Manager] --> B[Dispatch Service]
            A --> C[Mission Management]
            A --> D[Policy Engine]
            A --> E[Fleet Analytics]
            
            B --> F[WMS Adapter]
            B --> G[Mining FMS Adapter]
            B --> H[Military Adapter]
            
            C --> I[Control Center UI]
            D --> J[Auth Service]
            E --> K[Telemetry Ingestion]
        end
        
        subgraph "Edge Layer (Vehicle Agent)"
            L[Vehicle HAL] --> M[ROS2 Compute]
            M --> N[Sensor Fusion]
            N --> O[Local AI/ML]
            O --> P[Autonomous Control]
        end
        
        subgraph "Integration Layer"
            F --> Q[SAP EWM]
            F --> R[Manhattan SCALE]
            G --> S[Wenco]
            G --> T[MineStar]
            H --> U[Military Systems]
            H --> V[ISR Payloads]
        end
        
        A -.->|Telemetry| L
        L -.->|Commands| A
        I -.->|Remote Ops| P
    end
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
├─ services/                             # 72 microservices
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
├─ edge/                                 # Edge/vehicle components
│  ├─ vehicle-agent/                     # ROS2-based vehicle agent
│  ├─ cloud-bridge/                      # Cloud communication bridge
│  └─ ota-manager/                       # Over-the-air update manager
├─ ui/                                   # User interfaces
│  └─ control-center/                    # React-based control center (EN + AR/RTL)
├─ testing/                              # Test suites
│  ├─ conformance/                       # Conformance test runner (NEW)
│  │  └─ runner.py                       # Multi-dimensional test matrix
│  ├─ e2e/                               # End-to-end tests
│  ├─ integration/                       # Integration tests
│  ├─ performance/                       # Performance tests
│  └─ chaos/                             # Chaos engineering tests
`├─ data/                                 # Data contracts, schemas, and KPI definitions
├─ database/                             # Database optimization configs and stored procedures
├─ infrastructure/                       # Infrastructure as code
└─ tools/                                # Development tools
```

## 🗺️ Roadmap

<div align="center">

### 📅 **Development Phases**

| 🏗️ **Phase 1** | 🚀 **Phase 2** | 🎯 **Phase 3** | 🌟 **Phase 4** |
|:---:|:---:|:---:|:---:|
| **Months 1-6** | **Months 7-12** | **Months 13-18** | **Months 19-24** |
| Core Platform | Advanced Features | Qualified Agnosticism | Programmatic Validation |
| ✅ **COMPLETED** | ✅ **COMPLETED** | ✅ **COMPLETED** | 🔄 **IN PROGRESS** |

</div>

### 🎯 **Phase 3 Deliverables** *(Completed)*

<div align="center">

| 🚚 **Vehicle HAL** | 📦 **Sensor Packs** | 📊 **Variant Budget** | 🧪 **Conformance Testing** | ☁️ **Platform Adapters** | 📚 **Documentation** |
|:---:|:---:|:---:|:---:|:---:|:---:|
| Certified profiles | 3 certified packs | Automated delta tracking | Multi-dimensional matrix | Storage/messaging abstraction | 7 new docs + 3 ADRs |
| Terminal Tractor V2 | Rugged-A, Urban-B, Highway-C | CI/CD enforcement | 3×4×3 = 36 critical paths | Azure/AWS/on-prem | Complete technical docs |

</div>

### **📊 Implementation Timeline**

```mermaid
gantt
    title AtlasMesh Fleet Management System Implementation Timeline
    dateFormat  YYYY-MM-DD
    section Phase 1: MVP Foundation (Months 1-6)
    Core Platform (M1-M2)     :2024-01-01, 60d
    Safety Framework (M2-M3)  :2024-03-01, 30d
    Vehicle Abstraction (M3-M4) :2024-04-01, 30d
    Policy Engine (M4-M5)     :2024-05-01, 30d
    Control Center (M5-M6)    :2024-06-01, 30d
    
    section Phase 2: Pilots & Validation (Months 7-12)
    Lighthouse Customers (M7-M9) :2024-07-01, 90d
    Enterprise Integrations (M8-M10) :2024-08-01, 60d
    Weather-Aware Routing (M9-M11) :2024-09-01, 60d
    Multi-Site Operations (M10-M12) :2024-10-01, 60d
    
    section Phase 3: Scale & Expansion (Months 13-18)
    100+ Vehicles Production (M13-M15) :2025-01-01, 90d
    Advanced AI/ML (M14-M16) :2025-02-01, 60d
    Internationalization (M15-M17) :2025-03-01, 60d
    Market Leadership (M16-M18) :2025-04-01, 60d
    
    section Phase 4: Qualified Agnosticism (Months 19-24)
    Vehicle HAL (M19-M21) :2025-07-01, 90d
    Sensor Packs (M20-M22) :2025-08-01, 60d
    Variant Budget Enforcement (M21-M23) :2025-09-01, 60d
    Conformance Testing (M22-M24) :2025-10-01, 60d
```

### 🚀 **Phase 4 Milestones** *(90-180 Days)*

<div align="center">

| 🚙 **3-Vehicle Demo** | 🏭 **2-Sector Pilot** | ☁️ **2-Cloud Deploy** | 🔄 **Pack Swap** | 📋 **Evidence Bundle** |
|:---:|:---:|:---:|:---:|:---:|
| UTV + Terminal Tractor + Mine Haul | Defense + Mining overlays | Azure EKS + on-prem K3s | Rugged-A ↔ Urban-B | Automated regulatory compliance |
| ≥95% code reuse | ≥90% code share | 100% conformance | ≤30min swap time | Compliance artifacts |

</div>

> 📖 **Detailed Roadmap**: [Product Roadmap](docs/strategy/12_Product_Roadmap_and_Milestones.md) with PM CoP cadence and release plan

## 🛠️ Prerequisites

<div align="center">

| 🐳 **Docker** | ☸️ **Kubernetes** | 🐹 **Go** | 🟢 **Node.js** | 🐍 **Python** | 🦀 **Rust** |
|:---:|:---:|:---:|:---:|:---:|:---:|
| 20.10+ | 1.24+ (production) | 1.21+ | 18+ | 3.10+ | 1.70+ (optional) |

</div>

## 🚀 Quick Start

### 1️⃣ **Clone & Setup**

```bash
git clone https://github.com/atlasmesh/fleet-os.git
cd fleet-os
./scripts/bootstrap.sh
```

### 2️⃣ **Configure Environment**

```bash
cp configs/env/.env.example configs/env/.env
# Edit .env with your configuration
```

### 3️⃣ **Start Development Environment**

```bash
./scripts/run_local.sh
```

**This will start:**
- 🎮 Control Center UI: http://localhost:3000
- 📱 Mobile App: http://localhost:3001  
- 🔌 API Gateway: http://localhost:8080
- 📊 Grafana Dashboard: http://localhost:3001
- 📈 Prometheus Metrics: http://localhost:9090

## 🏃‍♂️ Running Locally

### **Full Stack Demo**

```bash
# Start all services with demo data
./scripts/run_local.sh
```

**This will:**
- 🐳 Start required services in Docker containers
- 🌱 Seed demo data (vehicles, trips, maps)
- 🎮 Launch the Control Center UI at http://localhost:3000

### **🏭 Sector-Specific Configurations**

<div align="center">

| 🛡️ **Defense** | ⛏️ **Mining** | 📦 **Logistics** | 🚕 **Ride-hail** |
|:---:|:---:|:---:|:---:|
| `--sector defense` | `--sector mining` | `--sector logistics` | `--sector ride-hail` |
| Military protocols | Heavy-duty operations | Supply chain optimization | Passenger transport |

</div>

```bash
./scripts/run_local.sh --sector defense
./scripts/run_local.sh --sector mining
./scripts/run_local.sh --sector logistics
./scripts/run_local.sh --sector ride-hail
```

## 🧪 Testing

### **Comprehensive Test Suite**

```bash
# Run all tests
make test
```

### **📊 Test Categories**

<div align="center">

| 🧪 **Unit Tests** | 📋 **Contract Tests** | 🔗 **Integration** | 🎯 **E2E Tests** | 🎮 **Simulation** | ✅ **Conformance** |
|:---:|:---:|:---:|:---:|:---:|:---:|
| `make test-unit` | `make test-contract` | `make test-integration` | `make test-e2e` | `make test-sim` | `make test-conformance` |
| Service logic | Adapter contracts | Cross-service | Full workflows | Scenario testing | Multi-dimensional |

</div>

```bash
make test-unit        # Unit tests
make test-contract    # Contract tests for adapters
make test-integration # Integration tests
make test-e2e         # End-to-end tests
make test-sim         # Simulation tests
make test-conformance # Multi-dimensional conformance testing (NEW)
```

### 🎯 **Qualified Agnosticism Conformance Testing**

**Multi-dimensional test matrix (3×4×3 = 36 critical paths)**

```bash
# Dry run to validate configuration
python testing/conformance/runner.py --dry-run

# Execute full conformance suite
python testing/conformance/runner.py

# Run specific priority level
python testing/conformance/runner.py --priority critical
python testing/conformance/runner.py --priority high
```

### **📊 Test Matrix Dimensions**

<div align="center">

| 🚚 **3 Vehicles** | 🏭 **4 Sectors** | ☁️ **3 Platforms** | 🎯 **36 Critical Paths** |
|:---:|:---:|:---:|:---:|
| ClassA_LightIndustrial | Defense | Azure EKS | Full test coverage |
| ClassB_HeavyDuty | Mining | AWS EKS | Priority execution |
| ClassC_Mining | Logistics | On-prem K3s | Evidence generation |
| | Ride-hail | | |

</div>

**📋 Evidence Output**: `testing/conformance/output/conformance_results_*.json`

## 🤝 Contributing

We welcome contributions! Please read [CONTRIBUTING.md](CONTRIBUTING.md) for details on our code of conduct, branch naming conventions, commit message format, and pull request process.

### **🚀 Quick Start for Contributors**

<div align="center">

| 🔀 **Fork & Branch** | 📝 **Code Standards** | 🧪 **Testing** | 📚 **Documentation** | 🔄 **Pull Request** |
|:---:|:---:|:---:|:---:|:---:|
| Fork repo → feature branch | Follow style guides | Write tests for new code | Update docs as needed | Clear description + tests |

</div>

### **📋 Key Requirements**

- ✅ Fork the repository and create a feature branch
- ✅ Follow the coding style and documentation standards  
- ✅ Add tests for new functionality
- ✅ Update documentation as needed
- ✅ Submit a pull request with a clear description

## 🔒 Security

<div align="center">

| 🛡️ **Vulnerability Disclosure** | ⏱️ **Security SLAs** | 📧 **Reporting Process** | 🔐 **PGP Encryption** |
|:---:|:---:|:---:|:---:|
| Coordinated disclosure | 24h response time | security@atlasmesh.io | Public key available |
| Responsible reporting | 7d patch timeline | Encrypted communication | Signed releases |

</div>

**📋 Full Details**: [SECURITY.md](SECURITY.md)

## 🛡️ Compliance & Safety Certification

AtlasMesh Fleet OS is designed to meet regulatory requirements across all supported sectors through **automated evidence generation**:

### **🚙 Automotive Safety Standards**

<div align="center">

| 📋 **ISO 26262** | 🎯 **ISO 21448 (SOTIF)** | 🔐 **UNECE R155** | 🔄 **UNECE R156** | 🛡️ **ISO 21434** |
|:---:|:---:|:---:|:---:|:---:|
| Functional safety | Safety of intended functionality | Cybersecurity requirements | Software update security | Automotive cybersecurity |
| Per-vehicle certification | Scenario-based validation | mTLS, PKI, threat modeling | Signed OTA, rollback | Engineering lifecycle |

</div>

### **🏭 Sector-Specific Compliance**

<div align="center">

| 🛡️ **Defense** | ⛏️ **Mining** | 📦 **Logistics** | 🚕 **Ride-hail** |
|:---:|:---:|:---:|:---:|
| NIST 800-53 | MSHA Part 56 | DOT regulations | Dubai RTA |
| Common Criteria | ISO 19296 | FMCSA compliance | Abu Dhabi DOT |
| STIG compliance | Mining machinery safety | | Local transport authority |

</div>

### **🔒 Data Privacy & Residency**

<div align="center">

| 🇪🇺 **GDPR** | 🇦🇪 **UAE PDPL** | 🎯 **Purpose Binding** | 🌍 **Data Residency** |
|:---:|:---:|:---:|:---:|
| European data protection | UAE Personal Data Protection | Data pipeline controls | Regional deployment |
| | | DPIA workflows | Compliance carve-outs |

</div>

### **🤖 Automated Evidence Generation**

**Compliance artifacts are generated automatically as part of the release process:**

<div align="center">

| 🛡️ **Safety Case Deltas** | 🎯 **SOTIF Validation** | 🔐 **Cybersecurity Evidence** | 📋 **Audit Trail** |
|:---:|:---:|:---:|:---:|
| Per-vehicle profile evidence | Scenario coverage & validation | UN R155/R156 compliance | Cryptographically signed logs |
| | | | |

</div>

## Product Management Framework

**AtlasMesh follows an evidence-first, outcome-driven product management framework** that ensures all features are strategically aligned, safely implemented, and measurably successful.

### **Framework Flow**

```mermaid
graph LR
    Intake[📥 Intake Form] --> Discovery[🔍 Discovery<br/>Evidence Pack]
    Discovery --> Canvas{Opportunity<br/>Canvas}
    Canvas -->|Go| PRD[📋 PRD<br/>Evidence-First]
    Canvas -->|No-Go| Archive[📁 Archive<br/>w/ Learnings]
    PRD --> DoR{Definition<br/>of Ready?}
    DoR -->|Yes| Delivery[🚀 Delivery<br/>Flags + Canary]
    DoR -->|No| PRD
    Delivery --> Evidence[📊 Evidence<br/>30-Day OQ]
    Evidence --> Decision{Continue?}
    Decision -->|Yes| Scale[📈 Scale to GA]
    Decision -->|Iterate| PRD
    Decision -->|Kill| Deprecate[🗑️ Deprecation<br/>Playbook]
    
    classDef intake fill:#e8f5e8,stroke:#388e3c,stroke-width:2px
    classDef discovery fill:#e3f2fd,stroke:#1976d2,stroke-width:2px
    classDef delivery fill:#fff3e0,stroke:#f57c00,stroke-width:2px
    classDef evidence fill:#f3e5f5,stroke:#7b1fa2,stroke-width:2px
    
    class Intake,Archive intake
    class Discovery,Canvas discovery
    class PRD,DoR,Delivery,Scale delivery
    class Evidence,Decision,Deprecate evidence
```

### **10 Ground Rules**
1. **Evidence before opinion** - No PRD without evidence pack (5-7 interviews + telemetry + sim)
2. **Traceability is mandatory** - Every FR/NFR → OKRs → Tests → SLIs → Evidence
3. **Agnostic by contract** - Profiles, packs, policies—not forks
4. **Variant budget is a constraint** - ≤5% code delta, ≤25% test delta per dimension
5. **Rollout ≠ release** - Flags, canaries, kill-switches required
6. **Safety & compliance are features** - They have owners, SLIs, gates
7. **Design is operational** - UI must perform under stress (WCAG 2.2 AA)
8. **Post-launch learning required** - 30-day OQ review closes loop
9. **Async excellence** - Comment in docs, decide in meetings
10. **Respect the clock** - Timeboxed reviews, documented decisions

### **PM Community of Practice (CoP)**

**Charter**: Standardize product craft, enforce traceability, govern variant budgets  
**Cadence**: Weekly intake, bi-weekly discovery, monthly roadmap/craft reviews, quarterly strategy alignment  
**Decision Model**: DACI (Driver-Approver-Contributors-Informed) for all cross-team decisions

**Full Charter**: [PM CoP in Executive Summary](docs/strategy/01_Executive_Summary_and_Vision.md#product-management-community-of-practice-pm-cop)

### **Key Documents & Templates**

**Strategy & Planning**:
- **[OKRs](docs/strategy/03_Objectives_and_Key_Results_OKRs.md)** - Company objectives and key results
- **[Metrics Canon](docs/strategy/04_Metrics_Canon.md)** - Single source of truth for all metrics
- **[Product Roadmap](docs/strategy/12_Product_Roadmap_and_Milestones.md)** - 18-month roadmap with PM CoP cadence
- **[Decision Log (DACI)](docs/strategy/Decision_Log_DACI.md)** - All product decisions recorded

**Templates & Playbooks**:
- **[Intake Form](docs/strategy/templates/Intake_Form.yaml)** - Single funnel for all requests
- **[Opportunity Canvas](docs/strategy/templates/Opportunity_Canvas.md)** - Evidence-based discovery
- **[Evidence-First PRD](docs/strategy/templates/PRD_Evidence_First.md)** - Comprehensive PRD template
- **[30-Day OQ Review](docs/strategy/templates/OQ_Review.md)** - Outcome quality assessment

**Playbooks**:
- **[Discovery](docs/strategy/playbooks/Discovery_Playbook.md)** - Evidence pack requirements
- **[Prioritization](docs/strategy/playbooks/Prioritization_Playbook.md)** - RICE × Safety × Variant-cost
- **[Beta Program](docs/strategy/playbooks/Beta_Program_Playbook.md)** - Safe rollout procedures
- **[Deprecation](docs/strategy/playbooks/Deprecation_Playbook.md)** - Feature sunset process

**Requirements & Governance**:
- **[Requirements (FRs/NFRs)](docs/Technical/03_Requirements_FRs_NFRs.md)** - With DoR/DoD checklists
- **[Epics & Alignment](docs/Technical/02_Epics_And_Strategic_Alignment.md)** - Epic-to-OKR mapping
- **[Cross-Department Checklist](docs/Technical/12_Cross_Department_Checklist.md)** - No-loopholes governance

---

## Documentation

### Quick Links
- **[Architecture Overview](docs/Technical/01_Architecture.md)** - Complete system architecture
- **[Qualified Agnosticism Guide](docs/Technical/08_Qualified_Agnosticism.md)** - Implementation approach
- **[Feasibility Scorecard](docs/Technical/10_Feasibility_Scorecard.md)** - Detailed feasibility analysis
- **[Programmatic Proof Points](docs/Technical/13_Programmatic_Proof_Points.md)** - 90-180 day validation plan
- **[Service Registry](docs/Technical/20_Service_Registry.md)** - All 72 microservices documented
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

---

**AtlasMesh Fleet OS - Engineering-Grounded Qualified Agnosticism for Autonomous Fleets** 🚙💨
**📋 Evidence Output**: `testing/conformance/output/conformance_results_*.json`

## 🤝 Contributing

We welcome contributions! Please read [CONTRIBUTING.md](CONTRIBUTING.md) for details on our code of conduct, branch naming conventions, commit message format, and pull request process.

### **🚀 Quick Start for Contributors**

<div align="center">

| 🔀 **Fork & Branch** | 📝 **Code Standards** | 🧪 **Testing** | 📚 **Documentation** | 🔄 **Pull Request** |
|:---:|:---:|:---:|:---:|:---:|
| Fork repo → feature branch | Follow style guides | Write tests for new code | Update docs as needed | Clear description + tests |

</div>

### **📋 Key Requirements**

- ✅ Fork the repository and create a feature branch
- ✅ Follow the coding style and documentation standards  
- ✅ Add tests for new functionality
- ✅ Update documentation as needed
- ✅ Submit a pull request with a clear description

## 🔒 Security

<div align="center">

| 🛡️ **Vulnerability Disclosure** | ⏱️ **Security SLAs** | 📧 **Reporting Process** | 🔐 **PGP Encryption** |
|:---:|:---:|:---:|:---:|
| Coordinated disclosure | 24h response time | security@atlasmesh.io | Public key available |
| Responsible reporting | 7d patch timeline | Encrypted communication | Signed releases |

</div>

**📋 Full Details**: [SECURITY.md](SECURITY.md)

## 🛡️ Compliance & Safety Certification

AtlasMesh Fleet OS is designed to meet regulatory requirements across all supported sectors through **automated evidence generation**:

### **🚙 Automotive Safety Standards**

<div align="center">

| 📋 **ISO 26262** | 🎯 **ISO 21448 (SOTIF)** | 🔐 **UNECE R155** | 🔄 **UNECE R156** | 🛡️ **ISO 21434** |
|:---:|:---:|:---:|:---:|:---:|
| Functional safety | Safety of intended functionality | Cybersecurity requirements | Software update security | Automotive cybersecurity |
| Per-vehicle certification | Scenario-based validation | mTLS, PKI, threat modeling | Signed OTA, rollback | Engineering lifecycle |

</div>

### **🏭 Sector-Specific Compliance**

<div align="center">

| 🛡️ **Defense** | ⛏️ **Mining** | 📦 **Logistics** | 🚕 **Ride-hail** |
|:---:|:---:|:---:|:---:|
| NIST 800-53 | MSHA Part 56 | DOT regulations | Dubai RTA |
| Common Criteria | ISO 19296 | FMCSA compliance | Abu Dhabi DOT |
| STIG compliance | Mining machinery safety | | Local transport authority |

</div>

### **🔒 Data Privacy & Residency**

<div align="center">

| 🇪🇺 **GDPR** | 🇦🇪 **UAE PDPL** | 🎯 **Purpose Binding** | 🌍 **Data Residency** |
|:---:|:---:|:---:|:---:|
| European data protection | UAE Personal Data Protection | Data pipeline controls | Regional deployment |
| | | DPIA workflows | Compliance carve-outs |

</div>

### **🤖 Automated Evidence Generation**

**Compliance artifacts are generated automatically as part of the release process:**

<div align="center">

| 🛡️ **Safety Case Deltas** | 🎯 **SOTIF Validation** | 🔐 **Cybersecurity Evidence** | 📋 **Audit Trail** |
|:---:|:---:|:---:|:---:|
| Per-vehicle profile evidence | Scenario coverage & validation | UN R155/R156 compliance | Cryptographically signed logs |
| | | | |

</div>

**📦 Evidence Bundle Export**: One-click regulatory package export in `testing/conformance/output/`

## 📄 License

<div align="center">

**This project is licensed under the Business Source License 1.1**

📋 **Full License Details**: [LICENSE](LICENSE) file

</div>

## Product Management Framework

**AtlasMesh follows an evidence-first, outcome-driven product management framework** that ensures all features are strategically aligned, safely implemented, and measurably successful.

### **Framework Flow**

```mermaid
graph LR
    Intake[📥 Intake Form] --> Discovery[🔍 Discovery<br/>Evidence Pack]
    Discovery --> Canvas{Opportunity<br/>Canvas}
    Canvas -->|Go| PRD[📋 PRD<br/>Evidence-First]
    Canvas -->|No-Go| Archive[📁 Archive<br/>w/ Learnings]
    PRD --> DoR{Definition<br/>of Ready?}
    DoR -->|Yes| Delivery[🚀 Delivery<br/>Flags + Canary]
    DoR -->|No| PRD
    Delivery --> Evidence[📊 Evidence<br/>30-Day OQ]
    Evidence --> Decision{Continue?}
    Decision -->|Yes| Scale[📈 Scale to GA]
    Decision -->|Iterate| PRD
    Decision -->|Kill| Deprecate[🗑️ Deprecation<br/>Playbook]
    
    classDef intake fill:#e8f5e8,stroke:#388e3c,stroke-width:2px
    classDef discovery fill:#e3f2fd,stroke:#1976d2,stroke-width:2px
    classDef delivery fill:#fff3e0,stroke:#f57c00,stroke-width:2px
    classDef evidence fill:#f3e5f5,stroke:#7b1fa2,stroke-width:2px
    
    class Intake,Archive intake
    class Discovery,Canvas discovery
    class PRD,DoR,Delivery,Scale delivery
    class Evidence,Decision,Deprecate evidence
```

### **10 Ground Rules**
1. **Evidence before opinion** - No PRD without evidence pack (5-7 interviews + telemetry + sim)
2. **Traceability is mandatory** - Every FR/NFR → OKRs → Tests → SLIs → Evidence
3. **Agnostic by contract** - Profiles, packs, policies—not forks
4. **Variant budget is a constraint** - ≤5% code delta, ≤25% test delta per dimension
5. **Rollout ≠ release** - Flags, canaries, kill-switches required
6. **Safety & compliance are features** - They have owners, SLIs, gates
7. **Design is operational** - UI must perform under stress (WCAG 2.2 AA)
8. **Post-launch learning required** - 30-day OQ review closes loop
9. **Async excellence** - Comment in docs, decide in meetings
10. **Respect the clock** - Timeboxed reviews, documented decisions

### **PM Community of Practice (CoP)**

**Charter**: Standardize product craft, enforce traceability, govern variant budgets  
**Cadence**: Weekly intake, bi-weekly discovery, monthly roadmap/craft reviews, quarterly strategy alignment  
**Decision Model**: DACI (Driver-Approver-Contributors-Informed) for all cross-team decisions

**Full Charter**: [PM CoP in Executive Summary](docs/strategy/01_Executive_Summary_and_Vision.md#product-management-community-of-practice-pm-cop)

### **Key Documents & Templates**

**Strategy & Planning**:
- **[OKRs](docs/strategy/03_Objectives_and_Key_Results_OKRs.md)** - Company objectives and key results
- **[Metrics Canon](docs/strategy/04_Metrics_Canon.md)** - Single source of truth for all metrics
- **[Product Roadmap](docs/strategy/12_Product_Roadmap_and_Milestones.md)** - 18-month roadmap with PM CoP cadence
- **[Decision Log (DACI)](docs/strategy/Decision_Log_DACI.md)** - All product decisions recorded

**Templates & Playbooks**:
- **[Intake Form](docs/strategy/templates/Intake_Form.yaml)** - Single funnel for all requests
- **[Opportunity Canvas](docs/strategy/templates/Opportunity_Canvas.md)** - Evidence-based discovery
- **[Evidence-First PRD](docs/strategy/templates/PRD_Evidence_First.md)** - Comprehensive PRD template
- **[30-Day OQ Review](docs/strategy/templates/OQ_Review.md)** - Outcome quality assessment

**Playbooks**:
- **[Discovery](docs/strategy/playbooks/Discovery_Playbook.md)** - Evidence pack requirements
- **[Prioritization](docs/strategy/playbooks/Prioritization_Playbook.md)** - RICE × Safety × Variant-cost
- **[Beta Program](docs/strategy/playbooks/Beta_Program_Playbook.md)** - Safe rollout procedures
- **[Deprecation](docs/strategy/playbooks/Deprecation_Playbook.md)** - Feature sunset process

**Requirements & Governance**:
- **[Requirements (FRs/NFRs)](docs/Technical/03_Requirements_FRs_NFRs.md)** - With DoR/DoD checklists
- **[Epics & Alignment](docs/Technical/02_Epics_And_Strategic_Alignment.md)** - Epic-to-OKR mapping
- **[Cross-Department Checklist](docs/Technical/12_Cross_Department_Checklist.md)** - No-loopholes governance

---

## Documentation

### Quick Links
- **[Architecture Overview](docs/Technical/01_Architecture.md)** - Complete system architecture
- **[Qualified Agnosticism Guide](docs/Technical/08_Qualified_Agnosticism.md)** - Implementation approach
- **[Feasibility Scorecard](docs/Technical/10_Feasibility_Scorecard.md)** - Detailed feasibility analysis
- **[Programmatic Proof Points](docs/Technical/13_Programmatic_Proof_Points.md)** - 90-180 day validation plan
- **[Service Registry](docs/Technical/20_Service_Registry.md)** - All 72 microservices documented
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

---

**AtlasMesh Fleet OS - Engineering-Grounded Qualified Agnosticism for Autonomous Fleets** 🚙💨
