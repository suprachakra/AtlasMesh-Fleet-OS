# Architecture Decision Records (ADRs)

This directory contains Architecture Decision Records (ADRs) for AtlasMesh Fleet OS, documenting significant architectural decisions made during the development of the system.

## What is an ADR?

An Architecture Decision Record is a document that captures an important architectural decision made along with its context and consequences. ADRs are immutable once accepted, though they may be superseded by later decisions.

## ADR Index

| ID | Title | Status | Date | Summary |
| --- | --- | --- | --- | --- |
| [ADR-0001](0001-vehicle-agnostic.md) | Vehicle-Agnostic Architecture | Accepted | 2025-09-14 | Defines the approach for supporting multiple vehicle classes through a profile-based abstraction layer |
| [ADR-0002](0002-platform-agnostic.md) | Platform-Agnostic Architecture | Accepted | 2025-09-14 | Establishes a Kubernetes-first approach with abstraction layers for cloud-agnostic deployment |
| [ADR-0003](0003-sector-agnostic.md) | Sector-Agnostic Architecture | Accepted | 2025-09-14 | Defines how the system supports multiple sectors through policy overlays and shared core capabilities |
| [ADR-0004](0004-sensor-agnostic.md) | Sensor-Agnostic Architecture | Accepted | 2025-09-14 | Establishes the approach for supporting different sensor configurations through certified sensor packs |
| [ADR-0005](0005-map-source-agnostic.md) | Map-Source-Agnostic Architecture | Accepted | 2025-09-14 | Defines how the system integrates multiple map sources with provenance tracking and conflict resolution |
| [ADR-0006](0006-weather-source-agnostic.md) | Weather-Source-Agnostic Architecture | Accepted | 2025-09-14 | Establishes the approach for fusing multiple weather data sources with confidence scoring |
| [ADR-0007](0007-comms-agnostic.md) | Communications-Agnostic Architecture | Accepted | 2025-09-14 | Defines the multi-path communications architecture with offline-first capabilities |
| [ADR-0008](0008-ros2-edge-stack.md) | ROS2-Based Edge Stack Architecture | Accepted | 2025-09-15 | Establishes ROS2 as the foundation for on-vehicle software with containerization and standardized interfaces |
| [ADR-0009](0009-hybrid-decision-framework.md) | Hybrid Decision Framework | Accepted | 2025-09-15 | Defines a combined approach using behavior trees, rule-based safety arbitration, and learned priors |
| [ADR-0010](0010-simulation-strategy.md) | Simulation Strategy | Accepted | 2025-09-15 | Establishes a comprehensive simulation approach using CARLA and Gazebo with twin-gated CI/CD |

## ADR Template

For creating new ADRs, use the [ADR template](adr-enhancement-template.md) which includes all required sections.

## ADR Process

1. **Proposal**: Create a new ADR using the template with status "Proposed"
2. **Review**: Share with stakeholders for review and feedback
3. **Decision**: Update status to "Accepted" once approved
4. **Implementation**: Reference the ADR in implementation work
5. **Supersession**: If a decision is replaced, create a new ADR and update the old one's status

## ADR Sections

Each ADR should include:

- **Header**: ID, title, status, date, proposer, approvers, etc.
- **Context**: Problem statement and business drivers
- **Decision**: The architectural decision and its components
- **Guardrails**: Constraints and enforcement mechanisms
- **KPIs/SLOs**: Measurable success criteria
- **Implementation**: Technical implementation details
- **Consequences**: Positive and negative impacts
- **Alternatives**: Options considered and reasons for rejection
- **Compliance & Monitoring**: Regulatory considerations and metrics

## Relationship to Other Documents

ADRs are referenced by and support:
- Strategic documents in `docs/strategy/`
- Technical specifications in `docs/Technical/`
- Implementation guides in `docs/implementation/`
- Safety and compliance documentation in `docs/compliance/`
