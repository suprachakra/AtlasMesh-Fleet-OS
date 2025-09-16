# ADR-0008: ROS2-Based Edge Stack Architecture

* **Status:** Accepted
* **Date:** 2025-09-15
* **Proposer:** Edge Platform Team
* **Approvers:** CTO, SVP Engineering, SVP Safety
* **Consulted:** Perception Team, Localization Team, Decision Team, Vehicle Integration Team
* **Informed:** All Engineering, Product Management
* **Related Issues:** ATLAS-245, ATLAS-267, ATLAS-289
* **Supersedes:** None
* **Superseded by:** None

## Context

AtlasMesh Fleet OS requires a robust, modular, and maintainable architecture for on-vehicle software components that can operate across multiple vehicle types and in challenging environments. We need a framework that supports real-time performance, component isolation, and standardized communication patterns.

### Problem Statement

1. On-vehicle software needs to handle multiple sensors, actuators, and processing modules
2. Components need to communicate with standardized interfaces and message formats
3. System must support real-time performance for safety-critical functions
4. Architecture must enable component isolation for fault containment
5. Software needs to be maintainable and testable across multiple vehicle types

### Market & Business Drivers

1. Industry convergence around ROS2 for autonomous systems
2. Availability of skilled robotics engineers familiar with ROS2
3. Need for rapid development and integration of new capabilities
4. Requirement for reproducible behavior across development and production
5. Desire to leverage open-source ecosystem while maintaining control of critical components

## Decision

We will implement a **ROS2-based Edge Stack** architecture with the following key components:

1. **ROS2 Framework** - Adopt ROS2 Humble/Iron as the foundation for our on-vehicle software:
   - Use DDS (Data Distribution Service) for reliable, real-time communication
   - Leverage ROS2's Quality of Service (QoS) settings for different message types
   - Utilize ROS2's node lifecycle management for controlled startup/shutdown
   - Implement component-based design with well-defined interfaces

2. **Node Structure** - Organize the system into the following primary nodes:
   - **Perception Nodes**: Sensor drivers, fusion, object detection and tracking
   - **Localization Nodes**: SLAM, VIO, sensor fusion, map alignment
   - **Decision Nodes**: Behavior trees, safety arbitration, trajectory planning
   - **Control Nodes**: Vehicle control, actuation, feedback loops
   - **Diagnostics Nodes**: System monitoring, health checks, fault detection
   - **Communication Nodes**: Vehicle-to-cloud bridge, local mesh networking

3. **Message Contracts** - Define standardized message interfaces:
   - Use ROS2 messages for on-vehicle communication
   - Define Protobuf mirrors of critical messages for cloud communication
   - Version all message definitions with backward compatibility
   - Document message semantics and quality expectations

4. **Containerization** - Deploy components in a containerized environment:
   - Use Docker containers for reproducible development and deployment
   - Implement resource isolation and constraints
   - Enable component-level updates and rollbacks
   - Support both development and production environments

5. **Operating Environment** - Standardize on:
   - Ubuntu 22.04 LTS as the base operating system
   - NVIDIA Jetson Orin as the primary compute platform
   - Optimized ROS2 packages for target hardware

## Guardrails

| Guardrail | Description | Enforcement Mechanism | Violation Response |
| --- | --- | --- | --- |
| **Real-time Performance** | Critical paths must meet timing requirements | Latency monitoring; performance testing | Performance optimization; architecture review |
| **Component Isolation** | Failures in one component must not cascade | Process isolation; watchdogs; fault injection testing | Component redesign; isolation enhancement |
| **Message Versioning** | All messages must be versioned and backward compatible | Schema validation; compatibility tests | Release block; interface redesign |
| **Resource Constraints** | Components must operate within defined resource budgets | Resource monitoring; stress testing | Optimization sprint; resource reallocation |
| **Safety Verification** | Safety-critical components require formal verification | Static analysis; runtime monitoring | Component redesign; additional verification |

## KPIs/SLOs

| KPI/SLO | Description | Target | Measurement Method | Reporting Frequency | Owner |
| --- | --- | --- | --- | --- | --- |
| **Control Loop Latency** | End-to-end processing time for critical paths | P95 ≤ 50ms | Instrumented timing; tracing | Continuous | Platform Lead |
| **CPU Utilization** | Processor usage under normal load | Peak ≤ 70%, Average ≤ 50% | System monitoring | Continuous | Performance Lead |
| **Memory Usage** | RAM consumption | Peak ≤ 80%, Leak rate = 0 | Memory profiling | Continuous | Performance Lead |
| **Node Restart Success** | Ability to recover from node failures | ≥ 99.9% | Fault injection testing | Weekly | Reliability Lead |
| **Message Throughput** | Message processing capacity | ≥ 2x peak expected load | Load testing | Monthly | Performance Lead |
| **Boot Time** | Time from power-on to operational | ≤ 60 seconds | Boot sequence timing | Per release | Platform Lead |

## Implementation Paths

| Component | Path | Description | Responsible Team | Dependencies |
| --- | --- | --- | --- | --- |
| **ROS2 Core** | `/edge/ros2-core/` | Base ROS2 configuration and extensions | Platform | ROS2 Humble/Iron; Ubuntu 22.04 |
| **Vehicle Agent** | `/edge/vehicle-agent/` | Main vehicle control application | Vehicle Integration | ROS2 Core; Vehicle interfaces |
| **Perception Modules** | `/edge/vehicle-agent/modules/perception/` | Sensor processing and fusion | Perception | Sensor drivers; ML models |
| **Localization Modules** | `/edge/vehicle-agent/modules/localization/` | SLAM, VIO, and positioning | Localization | Sensor fusion; Map service |
| **Decision Modules** | `/edge/vehicle-agent/modules/decision/` | Behavior trees and planning | Decision | Perception; Localization |
| **Control Modules** | `/edge/vehicle-agent/modules/control/` | Vehicle control algorithms | Control | Vehicle interfaces |
| **Diagnostics Agent** | `/edge/diag-agent/` | System monitoring and diagnostics | Platform | All components |
| **Cloud Bridge** | `/edge/cloud-bridge/` | Vehicle-to-cloud communication | Communications | All components; Cloud services |

### Integration Points

1. **Vehicle Hardware** - Integration with drive-by-wire systems, sensors, and compute
2. **Cloud Services** - Communication with fleet management and backend services
3. **Development Tools** - Integration with CI/CD, testing, and simulation environments
4. **Monitoring Systems** - Integration with diagnostics and observability platforms
5. **Update Systems** - Integration with OTA update mechanisms

## Consequences

### Positive

| Consequence | Description | Beneficiaries | Value Proposition |
| --- | --- | --- | --- |
| **Component Reusability** | Modular design enables reuse across vehicle types | Engineering, Product | Faster development; consistent behavior |
| **Ecosystem Leverage** | Access to ROS2 tools, libraries, and community | Engineering | Reduced development effort; shared knowledge |
| **Talent Availability** | Larger pool of engineers familiar with ROS2 | Hiring, Engineering | Easier recruitment; faster onboarding |
| **Standardized Communication** | Well-defined interfaces between components | Engineering, QA | Improved testability; clearer contracts |
| **Improved Maintainability** | Modular architecture with clear boundaries | Engineering, Operations | Easier updates; better fault isolation |

### Negative

| Consequence | Description | Mitigation Strategy | Residual Risk |
| --- | --- | --- | --- |
| **Learning Curve** | Some engineers may be unfamiliar with ROS2 | Training program; documentation; mentoring | Medium - Initial productivity impact |
| **Performance Overhead** | ROS2 middleware adds some processing overhead | Performance optimization; critical path analysis | Low - Acceptable for our use case |
| **Dependency Management** | Managing ROS2 package dependencies | Containerization; dependency lockfiles | Low - Managed through tooling |
| **Version Compatibility** | Ensuring compatibility across ROS2 versions | Version pinning; compatibility testing | Medium - Requires ongoing attention |
| **Community Alignment** | Balancing community standards with custom needs | Contribution to upstream; careful customization | Low - Manageable through governance |

## Kill-Switch Criteria

If any of the following occur, we will re-evaluate the ROS2-based architecture:

1. Critical performance targets cannot be met after optimization efforts
2. Safety certification becomes infeasible due to framework limitations
3. ROS2 community support significantly degrades or changes direction

## Alternatives Considered

| Alternative | Description | Pros | Cons | Why Rejected |
| --- | --- | --- | --- | --- |
| **Custom Middleware** | Develop proprietary communication framework | Complete control; potential performance gains | Development overhead; maintenance burden; talent constraints | Reinventing the wheel; higher development cost; limited ecosystem |
| **ROS1** | Use older ROS framework | Mature; widely used | Limited real-time support; less security; approaching end-of-life | Lacks modern features needed for production systems |
| **AUTOSAR** | Automotive standard architecture | Industry standard for automotive; safety certification | Complexity; development overhead; limited flexibility | Excessive overhead for our use case; limited ecosystem for autonomy |
| **DDS-only** | Use DDS directly without ROS2 | Reduced overhead; more control | Higher development effort; fewer tools and libraries | Lost benefits of ROS2 ecosystem; increased development time |

## Compliance & Regulatory Considerations

1. **Safety Standards**: ROS2 components in safety-critical paths will require additional verification per ISO 26262
2. **Real-time Performance**: Critical control loops must meet timing requirements with evidence
3. **Cybersecurity**: Communication security must be implemented per ISO 21434 guidelines
4. **Traceability**: Component behavior must be traceable to requirements and test cases
5. **Certification**: Safety-critical components may require certification evidence beyond ROS2 guarantees

## Monitoring & Metrics

| Metric | Description | Target | Warning Threshold | Critical Threshold | Data Source |
| --- | --- | --- | --- | --- | --- |
| **Node Health** | Status of ROS2 nodes | 100% operational | Any node degraded | Any critical node failed | Node lifecycle events |
| **Message Latency** | Time from publication to consumption | ≤ 10ms for critical paths | > 10ms | > 20ms | ROS2 instrumentation |
| **CPU Load** | Processor utilization | ≤ 50% average | > 70% sustained | > 90% peak | System monitoring |
| **Memory Usage** | RAM consumption | ≤ 70% | > 80% | > 90% | System monitoring |
| **Message Drop Rate** | Lost messages | < 0.01% | > 0.1% | > 1% | DDS statistics |
| **Node Restart Count** | Unexpected node restarts | 0 | Any non-critical | Any critical | System logs |

## Implementation Milestones

1. **Q1**: Base ROS2 environment with containerization; core node structure
2. **Q2**: Complete perception and localization nodes; initial decision framework
3. **Q3**: Full vehicle control integration; diagnostics system
4. **Q4**: Performance optimization; safety verification; certification support

## References

1. ROS2 Design Documentation (https://design.ros2.org/)
2. DDS Specification (https://www.omg.org/spec/DDS/)
3. AtlasMesh Vehicle Abstraction Layer (`docs/ADR/0001-vehicle-agnostic.md`)
4. AtlasMesh Sensor Abstraction Layer (`docs/ADR/0004-sensor-agnostic.md`)
5. AtlasMesh Communications Architecture (`docs/ADR/0007-comms-agnostic.md`)
