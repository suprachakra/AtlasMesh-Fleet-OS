# ADR-0010: Simulation Strategy

* **Status:** Accepted
* **Date:** 2025-09-15
* **Proposer:** Simulation & Validation Team
* **Approvers:** CTO, SVP Engineering, SVP Safety
* **Consulted:** Decision Team, Perception Team, Localization Team, QA Team
* **Informed:** All Engineering, Product Management
* **Related Issues:** ATLAS-356, ATLAS-372, ATLAS-389
* **Supersedes:** None
* **Superseded by:** None

## Context

AtlasMesh Fleet OS requires comprehensive testing and validation of autonomous capabilities across multiple vehicle types, sectors, and environmental conditions. Real-world testing alone is insufficient for achieving the necessary coverage, safety validation, and regression prevention. A robust simulation strategy is needed to enable thorough validation, rapid development cycles, and continuous integration.

### Problem Statement

1. Real-world testing is limited in coverage, repeatability, and edge case exploration
2. Safety validation requires exhaustive testing of critical scenarios
3. Development velocity depends on rapid feedback cycles for new features
4. Regression testing needs automation and comprehensive coverage
5. Cross-sector validation requires diverse environmental and operational conditions

### Market & Business Drivers

1. Safety certification requirements across sectors
2. Need for rapid development and validation cycles
3. Cost and time constraints of real-world testing
4. Requirement for edge case and failure mode testing
5. CI/CD integration for quality assurance

## Decision

We will implement a **Comprehensive Simulation Strategy** with the following key components:

1. **Multi-Level Simulation Framework** - A tiered approach to simulation:
   - **Unit-Level Simulation**: Component-specific testing with mocked interfaces
   - **Integration-Level Simulation**: Module interaction testing with simplified physics
   - **System-Level Simulation**: Full-stack testing with realistic physics and environments
   - **Hardware-in-the-Loop (HIL)**: Physical hardware integrated with simulated environments

2. **Primary Simulation Environments** - Adopt industry-standard simulators:
   - **CARLA**: Primary environment for urban and highway scenarios
   - **Gazebo**: Primary environment for off-road, industrial, and specialized scenarios
   - **Custom Extensions**: AtlasMesh-specific extensions for sector-specific requirements

3. **Scenario Management System** - A framework for defining, executing, and analyzing test scenarios:
   - Standardized scenario format (compatible with OpenSCENARIO)
   - Parameterized scenario templates for systematic variation
   - Scenario libraries organized by sector, capability, and risk level
   - Automated scenario generation from real-world data and edge cases

4. **Twin-Gated CI/CD Integration** - Simulation-based validation in the development pipeline:
   - Automated scenario execution for all code changes
   - Pass/fail criteria based on safety and performance metrics
   - Regression prevention through comprehensive test suites
   - Performance and resource usage benchmarking

5. **Digital Twin Capabilities** - Environment and vehicle modeling:
   - High-fidelity sensor simulation (cameras, LiDAR, radar, etc.)
   - Accurate vehicle dynamics models for supported vehicle classes
   - Environmental condition simulation (weather, lighting, terrain)
   - Infrastructure and actor behavior modeling

## Guardrails

| Guardrail | Description | Enforcement Mechanism | Violation Response |
| --- | --- | --- | --- |
| **Scenario Coverage** | Critical scenarios must be covered in simulation | Coverage analysis; scenario mapping | Test expansion; release block |
| **Fidelity Requirements** | Simulation must meet minimum fidelity standards | Validation against real-world data | Simulator enhancement; calibration |
| **Performance Budget** | Simulation execution within CI time constraints | Performance monitoring; optimization | Pipeline optimization; prioritization |
| **Reproducibility** | Scenarios must produce consistent results | Deterministic execution; seed control | Simulation framework fixes; isolation |
| **Real-World Correlation** | Simulation behavior must correlate with reality | Sim-to-real validation; calibration | Model refinement; additional validation |

## KPIs/SLOs

| KPI/SLO | Description | Target | Measurement Method | Reporting Frequency | Owner |
| --- | --- | --- | --- | --- | --- |
| **Scenario Coverage** | Percentage of required scenarios covered | ≥95% | Coverage analysis | Weekly | Simulation Lead |
| **Simulation Fidelity** | Correlation with real-world data | ≥90% match on key metrics | Sim-to-real comparison | Monthly | Validation Lead |
| **CI Execution Time** | Time to complete simulation tests in CI | ≤4 hours for full suite | Pipeline metrics | Daily | DevOps Lead |
| **Issue Detection Rate** | Percentage of issues caught in simulation | ≥85% | Issue root cause analysis | Monthly | QA Lead |
| **Scenario Execution Success** | Reliability of scenario execution | ≥99.5% | Execution logs | Daily | Simulation Lead |
| **Simulation Performance** | Real-time factor for simulation execution | ≥2x real-time for CI scenarios | Performance metrics | Weekly | Performance Lead |

## Implementation Paths

| Component | Path | Description | Responsible Team | Dependencies |
| --- | --- | --- | --- | --- |
| **CARLA Integration** | `/sim/carla/` | CARLA simulator integration and extensions | Simulation | CARLA; ROS2 bridge |
| **Gazebo Integration** | `/sim/gazebo/` | Gazebo simulator integration and extensions | Simulation | Gazebo; ROS2 bridge |
| **Scenario Bank** | `/sim/scenario-bank/` | Library of test scenarios | Validation | Scenario format; Domain expertise |
| **Scenario Runner** | `/sim/scenario-runner/` | Execution framework for scenarios | Simulation | Simulators; Scenario bank |
| **CI Integration** | `/sim/ci-gates/` | CI pipeline integration for simulation | DevOps | CI/CD system; Scenario runner |
| **Sensor Models** | `/sim/sensor-models/` | High-fidelity sensor simulation | Perception | Simulators; Sensor specifications |
| **Vehicle Models** | `/sim/vehicle-models/` | Vehicle dynamics and behavior models | Vehicle | Simulators; Vehicle specifications |
| **Environment Models** | `/sim/environment-models/` | Weather, terrain, and infrastructure models | Simulation | Simulators; Environmental data |

### Integration Points

1. **CI/CD Pipeline** - Integration with development workflow
2. **ROS2 Framework** - Connection to on-vehicle software stack
3. **Scenario Generation** - Integration with real-world data collection
4. **Metrics Collection** - Connection to analytics and reporting systems
5. **Hardware-in-the-Loop** - Integration with physical hardware components

## Consequences

### Positive

| Consequence | Description | Beneficiaries | Value Proposition |
| --- | --- | --- | --- |
| **Comprehensive Testing** | Broader coverage of scenarios and edge cases | Safety, QA, Engineering | Higher confidence; fewer field issues |
| **Accelerated Development** | Faster feedback cycles and validation | Engineering, Product | Reduced time-to-market; higher velocity |
| **Regression Prevention** | Automated detection of regressions | Engineering, QA | Improved quality; reduced maintenance |
| **Edge Case Exploration** | Systematic testing of rare but critical scenarios | Safety, Engineering | Improved robustness; risk reduction |
| **Cost Efficiency** | Reduced reliance on physical testing | Finance, Operations | Lower validation costs; faster iterations |

### Negative

| Consequence | Description | Mitigation Strategy | Residual Risk |
| --- | --- | --- | --- |
| **Simulation Gap** | Differences between simulation and reality | Continuous sim-to-real validation; conservative margins | Medium - Requires ongoing attention |
| **Infrastructure Costs** | Compute resources for simulation | Cloud optimization; prioritization; incremental testing | Low - Manageable through infrastructure planning |
| **Maintenance Burden** | Keeping simulators and models up-to-date | Automated testing; modular design; version control | Medium - Requires dedicated resources |
| **Complexity** | Managing diverse simulation environments | Unified interfaces; documentation; training | Low - Addressed through abstraction layers |
| **Over-reliance** | Potential to miss real-world issues | Balanced approach with field testing; sim-to-real validation | Low - Mitigated by validation strategy |

## Kill-Switch Criteria

If any of the following occur, we will re-evaluate the simulation strategy:

1. Significant simulation-to-reality gap persists after calibration efforts
2. CI execution time consistently exceeds budget, impacting development velocity
3. Maintenance burden becomes unsustainable for the team

## Alternatives Considered

| Alternative | Description | Pros | Cons | Why Rejected |
| --- | --- | --- | --- | --- |
| **Single Simulator** | Standardize on one simulation environment | Simplified maintenance; consistent interface | Limited coverage of all domains; vendor lock-in | Insufficient for cross-sector requirements |
| **Custom Simulator** | Build proprietary simulation environment | Complete control; tailored to needs | Development cost; maintenance burden; limited ecosystem | Excessive development overhead; reinventing the wheel |
| **Minimal Simulation** | Focus primarily on real-world testing | Higher fidelity; direct validation | Limited coverage; slow iteration; cost | Insufficient for safety validation and development velocity |
| **Unity-based Simulation** | Use Unity as primary simulation platform | Visual fidelity; ecosystem | Physics limitations; licensing costs | Less mature for autonomous systems; physics limitations |

## Compliance & Regulatory Considerations

1. **ISO 26262**: Simulation evidence as part of safety case
2. **ISO 21448 (SOTIF)**: Systematic coverage of performance limitations and edge cases
3. **Simulation Validation**: Documentation of simulation fidelity and limitations
4. **Evidence Generation**: Simulation results as part of regulatory submissions
5. **Traceability**: Linking scenarios to requirements and hazard analysis

## Monitoring & Metrics

| Metric | Description | Target | Warning Threshold | Critical Threshold | Data Source |
| --- | --- | --- | --- | --- | --- |
| **Scenario Pass Rate** | Percentage of scenarios passing | ≥98% | < 95% | < 90% | CI metrics |
| **Sim-to-Real Correlation** | Correlation coefficient for key metrics | ≥0.9 | < 0.85 | < 0.8 | Validation reports |
| **Simulation Performance** | Real-time factor | ≥2x | < 1.5x | < 1x | Performance metrics |
| **Coverage Gaps** | Uncovered critical scenarios | 0 | Any critical | Multiple critical | Coverage analysis |
| **Resource Utilization** | Compute resources per simulation | Within budget | > 120% budget | > 150% budget | Infrastructure metrics |
| **Scenario Flakiness** | Inconsistency in scenario results | < 0.1% | > 0.5% | > 1% | Execution logs |

## Implementation Milestones

1. **Q1**: Basic CARLA and Gazebo integration; initial scenario bank; CI prototype
2. **Q2**: Comprehensive scenario coverage for core capabilities; full CI integration
3. **Q3**: Advanced sensor and environment models; automated scenario generation
4. **Q4**: Complete digital twin capabilities; hardware-in-the-loop integration

## References

1. CARLA Simulator Documentation (https://carla.readthedocs.io/)
2. Gazebo Simulator Documentation (https://gazebosim.org/docs)
3. OpenSCENARIO Specification (https://www.asam.net/standards/detail/openscenario/)
4. AtlasMesh ROS2 Edge Stack (`docs/ADR/0008-ros2-edge-stack.md`)
5. AtlasMesh Hybrid Decision Framework (`docs/ADR/0009-hybrid-decision-framework.md`)
6. AtlasMesh Safety Validation Strategy (`docs/safety/validation-strategy.md`)
