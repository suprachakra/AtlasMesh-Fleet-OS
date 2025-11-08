# AtlasMesh Fleet OS - Use Cases Documentation

## 🎯 Overview

This directory contains comprehensive use case documentation for the **AtlasMesh Fleet OS** across all supported sectors. Each use case represents a specific operational scenario that the platform must support, with detailed specifications for implementation, validation and deployment.

## 📁 Directory Structure

```
docs/prd/use-cases/
├── README.md                           # This file
├── 00_taxonomy.md                      # Use case taxonomy and classification
├── _TEMPLATE.md                        # Template for new use cases
├── defense/                            # Defense sector use cases (27 files)
│   ├── 00_Problem_Statement_and_Solution-Defense.md
│   ├── D1_autonomous_fob_resupply_convoy.md
│   ├── D2_last_mile_critical_drop.md
│   └── ... (24 more defense use cases)
├── logistics/                          # Logistics sector use cases (24 files)
│   ├── 00_Problem_Statement_and_Solution-Logistics.md
│   ├── L1_yard_switcher_dock_yard.md
│   ├── L2_container_terminal_berth_shuttle.md
│   └── ... (22 more logistics use cases)
├── mining/                             # Mining sector use cases (26 files)
│   ├── 00_Problem_Statement_and_Solution-Mining.md
│   ├── M1_pit_to_crusher_autonomous_haul.md
│   ├── M2_overburden_removal_cycle.md
│   └── ... (23 more mining use cases)
└── ride-hail/                          # Ride-hail sector use cases (21 files)
    ├── 00_Problem_Statement_and_Solution-Ride-hail.md
    ├── R1_standard_point_to_point_ride.md
    ├── R2_shared_ride_pool.md
    └── ... (18 more ride-hail use cases)
```

## 🏗️ Use Case Architecture

### Mission Families
All use cases are organized into five core mission families:

1. **🚛 Move Things** - Transport people, cargo, fuel, water, tools
2. **🔍 Sense & Inspect** - Patrol, survey, asset inspection, mapping
3. **🔧 Maintain & Service** - Repair, refuel/recharge, swap, cleaning
4. **🎯 Control & Coordinate** - Convoy, platoon, berth/dock/stand control
5. **🛡️ Protect & Respond** - Safety, recovery, evacuation, emergency runs

### Trip Types (UI Integration)
Each use case maps to specific trip types in the user interface:

- **`OP_RUN`** - Normal operational trips
- **`RELEASE_RUN`** - Fleet launch window deployments
- **`MAPPING_RUN`** - Dedicated map data collection
- **`CALIB_RUN`** - Sensor calibration and system tuning
- **`MAINT_RUN`** - Maintenance-related trips
- **`TEST_RUN`** - Experimental or validation runs
- **`EMERGENCY_RUN`** - Emergency response operations
- **`TRAINING_RUN`** - Training and simulation
- **`BACKHAUL_RUN`** - Return logistics
- **`RELOCATION_RUN`** - Asset repositioning

### Trip Status Flow
```
PLANNED → QUEUED → DISPATCHED → ON_TRIP → COMPLETED
    ↓         ↓         ↓          ↓
  HOLD_SAFE ← ABORTED ← FAILED ← ON_TRIP
```

## 📋 Use Case Template Structure

Each use case follows a standardized template with these sections:

### Basic Information
- **ID**: Unique identifier (Sector + Number)
- **Name**: Descriptive use case name
- **Sector**: Defense | Mining | Logistics | Ride-hail
- **Status**: Draft | In Review | Approved | Implemented
- **Version**: Semantic versioning
- **Owner**: Responsible role

### Use Case Definition
- **Actors**: Primary and supporting actors
- **Trip Type**: Maps to UI trip types
- **ODD**: Operational Design Domain constraints
- **Trigger**: Event/state/SLA that initiates the use case
- **Nominal Flow**: 5-8 step primary flow
- **Variants/Edge Cases**: Key variations and edge conditions

### Performance & Integration
- **KPIs**: P0 (Must have) and P1 (Should have) metrics with thresholds
- **Dependencies**: Services, rules, and external systems required
- **Risks & Mitigations**: Key risks and designed-in mitigation strategies

### Validation & Deployment
- **Acceptance/Test Hooks**: Simulation scenarios, logs, gates
- **Rollout Plan**: Phased deployment strategy
- **Related Use Cases**: Cross-references to related scenarios

## 🎯 Sector-Specific Focus Areas

### 🛡️ Defense Sector (27 Use Cases)
**Focus**: Rugged terrain, contested environments, mission-critical operations
- **Key Challenges**: GNSS/Comms contested, environmental extremes, operational complexity
- **Mission Types**: Convoy operations, perimeter patrol, route clearance, medevac, base logistics
- **Special Requirements**: Security protocols, evidence generation, audit trails

### 🚛 Logistics Sector (24 Use Cases)
**Focus**: Port terminals, cross-dock operations, intermodal connections
- **Key Challenges**: Mixed traffic, V2I integration, operational efficiency
- **Mission Types**: Yard operations, container handling, cross-border transit, cold chain
- **Special Requirements**: Customs compliance, hazardous materials, scheduling optimization

### ⛏️ Mining Sector (26 Use Cases)
**Focus**: Heavy-duty operations, harsh environments, continuous operations
- **Key Challenges**: Dust, vibration, extreme temperatures, 24/7 operations
- **Mission Types**: Haul cycles, overburden removal, maintenance support, safety patrols
- **Special Requirements**: Load optimization, fuel efficiency, predictive maintenance

### 🚗 Ride-Hail Sector (21 Use Cases)
**Focus**: Urban mobility, passenger safety, accessibility, shared services
- **Key Challenges**: Mixed traffic, pedestrian safety, accessibility requirements
- **Mission Types**: Point-to-point rides, shared pools, airport services, accessibility
- **Special Requirements**: Passenger comfort, accessibility compliance, dynamic pricing

## 🔗 Integration with Product Development

Each use case integrates with the broader AtlasMesh Fleet OS development process:

### 1. **KPI Mapping**
- Links to `data/contracts/kpis.yaml` for performance tracking
- P0/P1 KPI definitions with measurable thresholds
- Real-time monitoring and alerting integration

### 2. **UI/UX Integration**
- Wireframes for key UI components
- User journey mapping
- Accessibility compliance (WCAG 2.2 AA)

### 3. **Service Dependencies**
- API contract definitions
- Service mesh integration
- Cross-service communication patterns

### 4. **Telemetry & Analytics**
- Telemetry schema updates
- Real-time data collection
- Analytics and reporting integration

### 5. **Testing & Validation**
- Simulation scenario definitions
- CI/CD integration with twin-gates
- Automated testing frameworks

## 🚀 Getting Started

### For Product Managers
1. **Review Sector Problem Statements**: Start with `00_Problem_Statement_and_Solution-[Sector].md`
2. **Understand Use Case Taxonomy**: Read `00_taxonomy.md` for classification system
3. **Map to Business Objectives**: Link use cases to OKRs and strategic goals

### For Engineers
1. **Review Use Case Dependencies**: Check service and rule requirements
2. **Understand ODD Constraints**: Review operational design domain limitations
3. **Plan Implementation**: Use acceptance criteria and test hooks for development

### For Testers
1. **Review Acceptance Criteria**: Understand simulation scenarios and test gates
2. **Plan Test Coverage**: Map use cases to test scenarios
3. **Validate KPIs**: Ensure measurable performance criteria

## 📊 Use Case Status Tracking

### Status Definitions
- **Draft**: Initial creation, under development
- **In Review**: Awaiting stakeholder approval
- **Approved**: Ready for implementation
- **Implemented**: Development complete, in testing

### Quality Gates
Each use case must pass these gates before approval:
- [ ] **Evidence Pack**: 5-7 contextual interviews OR telemetry analysis
- [ ] **Traceability**: Links to OKRs, Epics, FRs/NFRs
- [ ] **Variant Budget**: ≤5% code delta, ≤25% test delta
- [ ] **Safety Compliance**: All safety gates passed
- [ ] **Performance Validation**: KPI thresholds defined and measurable

## 🔄 Maintenance & Updates

### Regular Reviews
- **Monthly**: Status updates and progress tracking
- **Quarterly**: Use case validation and performance review
- **Annually**: Complete taxonomy review and sector analysis

### Change Management
- **Version Control**: Semantic versioning for all changes
- **Change Log**: Document all modifications and rationale
- **Stakeholder Approval**: Required for significant changes

## 📚 Additional Resources

### Related Documentation
- **[Technical Requirements](../Technical/03_Requirements_FRs_NFRs.md)**: Functional and non-functional requirements
- **[Architecture Documentation](../Technical/01_Architecture.md)**: System architecture and design
- **[API Reference](../../api/API_REFERENCE.md)**: Service API documentation
- **[Database Schema](../../database/DATABASE_COMPREHENSIVE_DOCUMENTATION.md)**: Data models and schemas

### Templates & Tools
- **Use Case Template**: `_TEMPLATE.md`
- **Problem Statement Template**: Available in each sector directory
- **Validation Checklists**: Integrated into each use case

### Support & Contact
- **Product Management**: [PM CoP Documentation](../../strategy/)
- **Technical Support**: [Architecture Decision Records](../../ADR/)
- **Implementation Guide**: [Technical Documentation](../Technical/)

---

## 🎯 Quick Navigation

| Sector | Use Cases | Key Focus | Status |
|--------|-----------|-----------|---------|
| **Defense** | 27 | Rugged terrain, contested environments | ✅ Complete |
| **Logistics** | 24 | Port operations, intermodal connections | ✅ Complete |
| **Mining** | 26 | Heavy-duty, continuous operations | ✅ Complete |
| **Ride-hail** | 21 | Urban mobility, passenger safety | ✅ Complete |

**Total Use Cases**: **98** across all sectors

---

*This documentation is maintained by the AtlasMesh Fleet OS Product Management Community of Practice (PM CoP) and follows evidence-first, traceability-enforced development practices.*

