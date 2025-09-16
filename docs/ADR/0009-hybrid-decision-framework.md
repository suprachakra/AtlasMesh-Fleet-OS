# ADR-0009: Hybrid Decision Framework

* **Status:** Accepted
* **Date:** 2025-09-15
* **Proposer:** Decision Systems Team
* **Approvers:** CTO, SVP Engineering, SVP Safety
* **Consulted:** Perception Team, Localization Team, Control Team, Safety Team
* **Informed:** All Engineering, Product Management
* **Related Issues:** ATLAS-301, ATLAS-312, ATLAS-328
* **Supersedes:** None
* **Superseded by:** None

## Context

AtlasMesh Fleet OS requires a decision-making framework that can handle complex scenarios across multiple sectors (defense, mining, logistics, ride-hail) while maintaining safety, explainability, and adaptability. The framework must balance deterministic behavior with the ability to learn from experience and handle edge cases.

### Problem Statement

1. Decision-making must be robust, explainable, and verifiable for safety-critical applications
2. Different sectors and operational domains have unique behavioral requirements
3. Complex scenarios require sophisticated decision logic that is maintainable and testable
4. The system must adapt to new environments while maintaining safety guarantees
5. Performance must meet real-time constraints in resource-constrained environments

### Market & Business Drivers

1. Safety certification requirements across sectors
2. Need for rapid adaptation to new operational domains
3. Requirement for explainable decisions for regulatory compliance
4. Desire to leverage machine learning for improved performance
5. Competitive advantage through robust behavior in edge cases

## Decision

We will implement a **Hybrid Decision Framework** with the following key components:

1. **Behavior Tree Core** - A hierarchical behavior tree architecture that:
   - Provides a modular, composable structure for decision logic
   - Enables clear visualization and debugging of decision processes
   - Supports conditional execution, fallbacks, and parallel behaviors
   - Facilitates reuse of behavior components across sectors

2. **Rule-Based Safety Arbitration** - A formal rule system that:
   - Enforces safety constraints on all decisions
   - Implements sector-specific regulatory requirements
   - Provides verifiable safety guarantees
   - Overrides other components when safety is at risk

3. **Learned Priors** - Machine learning components that:
   - Provide optimization hints to the behavior tree
   - Learn from experience to improve performance
   - Operate within constraints defined by safety arbitration
   - Adapt to environmental variations and edge cases

4. **Policy Engine Integration** - Connection to the central policy engine that:
   - Enforces organization-wide policies and constraints
   - Adapts behavior based on sector, region, and vehicle type
   - Provides audit trails for all policy-influenced decisions
   - Enables remote updates to decision parameters

5. **Profiling & Validation** - A comprehensive testing framework that:
   - Validates decision behavior in simulation
   - Profiles performance and resource usage
   - Verifies compliance with safety requirements
   - Supports regression testing and continuous improvement

## Guardrails

| Guardrail | Description | Enforcement Mechanism | Violation Response |
| --- | --- | --- | --- |
| **Safety Override** | Safety arbitration must be able to override any decision | Runtime verification; formal methods | Immediate redesign; safety review |
| **Explainability** | All decisions must be traceable and explainable | Decision logging; visualization tools | Component redesign; additional documentation |
| **Determinism** | Critical paths must have deterministic behavior | Replay testing; formal verification | Refactoring; additional constraints |
| **ML Constraints** | Machine learning components must operate within defined bounds | Runtime monitoring; validation gates | Retrain with constraints; fallback to rules |
| **Performance Budget** | Decision cycle must complete within timing budget | Performance testing; profiling | Optimization; simplification |

## KPIs/SLOs

| KPI/SLO | Description | Target | Measurement Method | Reporting Frequency | Owner |
| --- | --- | --- | --- | --- | --- |
| **Decision Latency** | Time to make critical decisions | P95 ≤ 40ms | Instrumented timing; profiling | Continuous | Performance Lead |
| **Safety Violations** | Instances of safety rule violations | 0 | Runtime monitoring; simulation | Continuous | Safety Lead |
| **Decision Quality** | Optimality of decisions vs. baseline | ≥ 15% improvement | Simulation benchmarks | Weekly | Decision Lead |
| **Resource Usage** | CPU and memory consumption | ≤ 30% of budget | System monitoring | Continuous | Performance Lead |
| **Explainability Score** | Percentage of decisions with clear explanation | 100% | Automated analysis | Per release | Safety Lead |
| **Adaptation Rate** | Time to adapt to new conditions | ≤ 100 operational hours | Learning curve analysis | Monthly | ML Lead |

## Implementation Paths

| Component | Path | Description | Responsible Team | Dependencies |
| --- | --- | --- | --- | --- |
| **Behavior Tree Engine** | `/edge/vehicle-agent/modules/decision/bt-engine/` | Core behavior tree implementation | Decision | ROS2; Control interface |
| **Behavior Catalog** | `/edge/vehicle-agent/modules/decision/behaviors/` | Library of behavior nodes | Decision | Behavior Tree Engine |
| **Safety Arbitrator** | `/edge/vehicle-agent/modules/decision/safety-arbitrator/` | Safety rule enforcement | Safety | Behavior Tree Engine |
| **ML Components** | `/edge/vehicle-agent/modules/decision/ml-priors/` | Learned optimization components | ML | Behavior Tree Engine; Training data |
| **Policy Connector** | `/edge/vehicle-agent/modules/decision/policy-connector/` | Integration with policy engine | Decision | Policy Engine; Behavior Tree Engine |
| **Profiling Tools** | `/tools/decision-profiler/` | Performance analysis tools | Performance | Behavior Tree Engine |
| **Validation Framework** | `/sim/decision-validation/` | Simulation-based validation | QA | Simulation environment; Behavior Tree Engine |
| **Visualization Tools** | `/tools/bt-visualizer/` | Behavior tree visualization | UI | Behavior Tree Engine |

### Integration Points

1. **Perception System** - Integration with object detection and scene understanding
2. **Localization System** - Integration with positioning and mapping
3. **Control System** - Output to vehicle control modules
4. **Policy Engine** - Connection to centralized policy management
5. **Simulation Environment** - Integration with testing and validation tools

## Consequences

### Positive

| Consequence | Description | Beneficiaries | Value Proposition |
| --- | --- | --- | --- |
| **Modularity** | Reusable behavior components across sectors | Engineering, Product | Faster development; consistent behavior |
| **Explainability** | Clear decision traces for debugging and compliance | Safety, Compliance, Operations | Easier certification; better incident analysis |
| **Safety Assurance** | Formal verification of safety properties | Safety, Customers, Regulators | Higher confidence; easier certification |
| **Adaptability** | Learning from experience while maintaining safety | Product, Customers | Better performance over time; competitive advantage |
| **Maintainability** | Structured approach to complex decision logic | Engineering | Easier updates; better collaboration |

### Negative

| Consequence | Description | Mitigation Strategy | Residual Risk |
| --- | --- | --- | --- |
| **Complexity** | Hybrid approach increases system complexity | Comprehensive documentation; visualization tools | Medium - Requires ongoing attention to architecture |
| **Performance Overhead** | Multiple decision components add processing overhead | Performance optimization; profiling | Low - Acceptable for our use case |
| **Integration Challenges** | Coordinating between rule-based and ML components | Clear interfaces; extensive testing | Medium - Requires careful design |
| **Training Requirements** | Engineers need to understand multiple paradigms | Training program; documentation; mentoring | Low - Manageable through knowledge sharing |
| **Validation Burden** | More complex testing and validation requirements | Automated testing; simulation coverage | Medium - Requires significant test infrastructure |

## Kill-Switch Criteria

If any of the following occur, we will re-evaluate the hybrid decision framework:

1. Safety arbitration fails to prevent unsafe behavior in critical scenarios
2. Decision latency consistently exceeds performance budget after optimization
3. Complexity becomes unmanageable for engineering team

## Alternatives Considered

| Alternative | Description | Pros | Cons | Why Rejected |
| --- | --- | --- | --- | --- |
| **Pure Rule-Based** | Decision logic entirely based on explicit rules | Explainable; verifiable; deterministic | Limited adaptability; complexity explosion for edge cases | Insufficient for complex environments; difficult to scale to all scenarios |
| **Pure ML-Based** | Decision logic entirely based on learned models | Adaptable; potentially optimal; handles edge cases | Limited explainability; difficult to verify safety; data hungry | Safety certification challenges; explainability requirements |
| **State Machines** | Decision logic based on explicit state transitions | Simple; verifiable; well-understood | State explosion for complex scenarios; limited adaptability | Insufficient for complex behaviors; maintenance challenges at scale |
| **Utility-Based** | Decision logic based on utility functions | Flexible; mathematically grounded | Difficult to tune; complex interactions; limited explainability | Challenging to implement and maintain at scale |

## Compliance & Regulatory Considerations

1. **ISO 26262**: Safety-critical decision components require ASIL-appropriate development processes
2. **ISO 21448 (SOTIF)**: Decision framework must address performance limitations and edge cases
3. **Explainability Requirements**: Decisions must be traceable for regulatory compliance
4. **Sector-Specific Regulations**: Framework must adapt to different regulatory environments
5. **ML Governance**: Learning components require appropriate governance and validation

## Monitoring & Metrics

| Metric | Description | Target | Warning Threshold | Critical Threshold | Data Source |
| --- | --- | --- | --- | --- | --- |
| **Decision Time** | Processing time per decision cycle | ≤ 40ms | > 40ms | > 60ms | Performance profiling |
| **Safety Interventions** | Count of safety arbitrator overrides | < 0.1% of decisions | > 0.5% | > 1% | Decision logs |
| **Behavior Tree Depth** | Maximum depth of active behavior tree | ≤ 8 levels | > 10 levels | > 12 levels | Behavior tree metrics |
| **ML Confidence** | Confidence scores of ML predictions | ≥ 90% | < 80% | < 70% | ML component logs |
| **Decision Conflicts** | Conflicts between components requiring arbitration | < 1% of decisions | > 3% | > 5% | Decision logs |
| **Memory Usage** | RAM consumption of decision framework | ≤ 500MB | > 700MB | > 900MB | System monitoring |

## Implementation Milestones

1. **Q1**: Core behavior tree engine with basic behaviors; safety arbitrator prototype
2. **Q2**: Complete sector-specific behavior libraries; policy engine integration
3. **Q3**: ML prior integration; comprehensive validation framework
4. **Q4**: Performance optimization; safety verification; certification support

## References

1. Behavior Trees in Robotics and AI (Colledanchise & Ögren)
2. AtlasMesh ROS2 Edge Stack (`docs/ADR/0008-ros2-edge-stack.md`)
3. AtlasMesh Policy Engine Design (`docs/Technical/01_Architecture.md#policy-engine`)
4. AtlasMesh Safety Case Framework (`docs/safety/safety-case-framework.md`)
5. ISO 21448 (SOTIF) Implementation Guide (`docs/compliance/sotif-guide.md`)
