# AtlasMesh Fleet Operating System

Welcome to the AtlasMesh Fleet OS documentation - your comprehensive guide to deploying and operating **vehicle-agnostic, platform-agnostic, sector-agnostic L4 autonomous fleet management systems**.

## 🎯 What is AtlasMesh Fleet OS?

AtlasMesh Fleet OS is a unified **Fleet Management System (FMS) + AV retrofit kit** implementing **qualified agnosticism** - a pragmatic, engineering-grounded approach to vehicle-agnostic, platform-agnostic, and sector-agnostic L4 autonomous fleet operations. Rather than claiming universal compatibility, we achieve **bounded agnosticism** through contract-driven interfaces, automated variant budgets, and safety-certified profiles.

### Core Capabilities

=== "Multi-Sector Support"
    - **Defense**: Convoy operations, tactical missions, secure base logistics
    - **Mining**: Haul trucks, equipment coordination, safety protocols  
    - **Logistics**: Port operations, last-mile delivery, warehouse automation
    - **Ride-hailing**: Urban transport, passenger service, fleet optimization

=== "Extreme Weather Ready"
    - **55°C desert operations** with thermal derating
    - **Dust storm protocols** with sensor degradation handling
    - **Sand/salt corrosion resistance** for coastal operations
    - **Intermittent connectivity** with offline capability

=== "Qualified Agnosticism"
    - **Vehicle-Agnostic**: Class/model-bounded with certified profiles (≤5% code delta)
    - **Sector-Agnostic**: Policy overlays targeting ≥90% code reuse across sectors
    - **Platform-Agnostic**: Contract-driven infrastructure with conformance testing
    - **Variant Budget Enforcement**: Automated tracking and CI/CD gates
    - **Safety Certified**: Per-model ISO 26262/SOTIF/R155/R156 compliance

## 🏗️ System Architecture

```mermaid
graph TB
    subgraph "Control Center UI"
        UI[Fleet Operations Dashboard]
        MOB[Mobile App]
    end
    
    subgraph "API Gateway & Load Balancer"
        LB[Load Balancer/Nginx]
        API[API Gateway]
    end
    
    subgraph "Core Services"
        FM[Fleet Manager]
        VG[Vehicle Gateway]
        PE[Policy Engine]
        WF[Weather Fusion]
        PM[Predictive Maintenance]
        GT[Garage Tools]
    end
    
    subgraph "Advanced Services"
        TI[Telemetry Ingest]
        CO[Cost Optimization]
        UAE[UAE Gov Integration]
        ERP[ERP/WMS Adapters]
        ML[ML Pipeline]
        DT[Digital Twin]
    end
    
    subgraph "Qualified Agnosticism Services"
        VHAL[Vehicle HAL]
        VB[Variant Budget]
        CT[Conformance Testing]
        SPR[Sensor Pack Registry]
        PA[Platform Adapters]
    end
    
    subgraph "Data Layer"
        PG[(PostgreSQL)]
        RD[(Redis)]
        KF[(Kafka)]
        CH[(ClickHouse)]
        MN[(MinIO)]
    end
    
    subgraph "Monitoring & Observability"
        PR[Prometheus]
        GR[Grafana]
        JG[Jaeger]
    end
    
    UI --> LB
    MOB --> LB
    LB --> API
    API --> FM
    API --> VG
    API --> PE
    API --> WF
    
    FM --> PG
    VG --> RD
    PE --> KF
    TI --> CH
    CO --> MN
    
    FM --> PR
    VG --> PR
    PE --> PR
    PR --> GR
    API --> JG
```

## 🚀 Quick Start

### Prerequisites

- **Docker** 24.0+ and **Kubernetes** 1.28+
- **Node.js** 18+ and **npm** 9+
- **Middle East deployment region** (UAE, Saudi Arabia, Qatar)

### 1. Clone and Setup

```bash
git clone https://github.com/atlasmesh/fleet-os.git
cd fleet-os
make setup
```

### 2. Start Development Environment

```bash
# Start all services with Docker Compose
docker-compose up -d

# Or use deployment scripts
./scripts/deploy.sh development
```

This starts:
- **Control Center UI**: http://localhost:3000
- **Mobile App**: http://localhost:3001  
- **API Gateway**: http://localhost:8080
- **Fleet Manager**: http://localhost:8081
- **Vehicle Gateway**: http://localhost:8082
- **Policy Engine**: http://localhost:8083
- **Grafana Dashboard**: http://localhost:3001
- **Prometheus Metrics**: http://localhost:9090

### 3. Configure Your Fleet

1. **Define vehicle capabilities** in `configs/vehicles/`
2. **Set sector policies** in `rules/policy/sector/`
3. **Configure city operations** in `configs/cities/`

## 📋 Documentation Structure

### Strategy & Planning
Get familiar with the business model, market positioning, and implementation roadmap.

[:material-strategy: Strategy Docs](strategy/01_executive_summary_and_vision.md){ .md-button .md-button--primary }

### Technical Implementation  
Deep dive into architecture, APIs, security, and development processes.

[:material-code-braces: Technical Docs](technical/01_architecture.md){ .md-button }

### Sector Specialization
Learn how to configure the platform for different industry sectors.

[:material-factory: Sector Packs](sector-packs/defense.md){ .md-button }

### Vehicle Integration
Integrate new vehicle types using the adapter SDK and capability manifests.

[:material-car: Vehicle Integration](vehicle-packs/ugv_themis.md){ .md-button }

### Safety & Compliance
Understand safety cases, regulatory compliance, and audit requirements.

[:material-shield-check: Safety & Compliance](safety/README.md){ .md-button }

### Operations
Production deployment guides, troubleshooting, and operational procedures.

[:material-cog: Operations Guide](runbooks/README.md){ .md-button }

## 🌟 Key Features

### Intelligent Fleet Management

- **Multi-objective dispatch** balancing safety, efficiency, and customer SLAs
- **Real-time optimization** with traffic, weather, and energy constraints
- **Predictive maintenance** preventing unexpected downtime
- **Energy optimization** for mixed EV/hybrid/diesel fleets

### Extreme Conditions Handling

- **Weather data fusion** from multiple providers
- **Automatic degradation** when conditions exceed safety thresholds  
- **Thermal management** with speed/route derating at high temperatures
- **Dust/sand mode** with enhanced sensor cleaning and conservative planning

### Policy-Driven Operations

- **Precedence hierarchy**: Safety > Law > Tenant > Sector > City > Fleet
- **Rules as code** with automated testing and deployment
- **ODD enforcement** preventing operations outside safe boundaries
- **Compliance automation** with audit trail generation

### Safety & Security

- **ISO 26262/21448** functional safety compliance
- **UNECE R155/R156** cybersecurity standards
- **PKI-based authentication** for all vehicle communications
- **Tamper-evident OTA updates** with automatic rollback

## 📊 Supported Metrics

Track performance across multiple dimensions:

| Metric Category | Examples |
| --- | --- |
| **Safety** | Incidents per million km, Near-miss rate, Emergency stops |
| **Efficiency** | Trip completion rate, ETA accuracy, Fuel/energy efficiency |
| **Availability** | Vehicle uptime, Service availability, Maintenance scheduling |
| **Customer** | On-time performance, Customer satisfaction, Response time |

## 🔗 Quick Links

### Core Documentation
- [**Architecture Overview**](Technical/01_Architecture.md) - System design and service breakdown
- [**API Documentation**](api/API_REFERENCE.md) - Comprehensive REST API reference  
- [**Deployment Guide**](deployment/DEPLOYMENT_GUIDE.md) - Production deployment procedures
- [**Troubleshooting**](troubleshooting/TROUBLESHOOTING_GUIDE.md) - Common issues and solutions
- [**User Manual**](user-manuals/USER_MANUAL.md) - Complete user guide
- [**Executive Summary**](strategy/01_Executive_Summary_and_Vision.md) - Business vision and strategy

### Qualified Agnosticism
- [**Qualified Agnosticism Overview**](Technical/08_Qualified_Agnosticism.md) - Implementation guide
- [**Five Constraining Realities**](Technical/09_Five_Constraining_Realities.md) - Technical constraints
- [**Feasibility Scorecard**](Technical/10_Feasibility_Scorecard.md) - Detailed feasibility analysis
- [**Programmatic Proof Points**](Technical/13_Programmatic_Proof_Points.md) - 90-180 day validation plan

## 🎯 Next Steps

1. **Read the [Executive Summary](strategy/01_Executive_Summary_and_Vision.md)** to understand the business vision
2. **Review [Architecture](Technical/01_Architecture.md)** to understand the technical approach  
3. **Explore [Use Cases](prd/use-cases/)** to see sector-specific implementations
4. **Follow the [Deployment Guide](deployment/DEPLOYMENT_GUIDE.md)** for production setup
5. **Check the [User Manual](user-manuals/USER_MANUAL.md)** for operational procedures

---

**Questions?** Reach out to our team at [team@atlasmesh.ai](mailto:team@atlasmesh.ai)
