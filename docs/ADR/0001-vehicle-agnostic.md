# ADR-0001: Vehicle-Agnostic Architecture

* **Status:** Accepted
* **Date:** 2025-09-14
* **Proposer:** Architecture Team
* **Approvers:** CTO, SVP Engineering, SVP Safety
* **Consulted:** Vehicle Integration Team, Safety Team, Regulatory Affairs
* **Informed:** All Engineering, Product Management
* **Related Issues:** ATLAS-123, ATLAS-145, ATLAS-167
* **Supersedes:** None
* **Superseded by:** None

## Context

AtlasMesh Fleet OS aims to operate across multiple vehicle classes and models without requiring code forks or custom implementations per vehicle. This requires a clear architecture that separates vehicle-specific concerns from core platform capabilities.

### Problem Statement

Supporting multiple vehicle types traditionally requires significant customization, leading to:
1. Code forks and maintenance overhead
2. Inconsistent behavior across vehicle types
3. Lengthy onboarding time for new vehicles
4. Difficulty maintaining safety certifications across variants
5. Increased testing matrix complexity

### Market & Business Drivers

1. Customers demand multi-vehicle fleet support (heterogeneous fleets)
2. Vehicle OEMs frequently update models and control interfaces
3. Cross-sector operations require different vehicle classes
4. Retrofit market demands flexibility across vehicle ages and types
5. TCO improvements require standardized maintenance and operations

## Decision

We will implement a **vehicle-agnostic architecture** with the following key components:

1. **Vehicle Profile System** - A configuration-driven approach where each supported vehicle has a profile defining:
   - Physical characteristics (mass, wheelbase, CG height, etc.)
   - Actuation parameters (brake curves, steering ratio, etc.)
   - Sensor placement and calibration
   - Control limits (max acceleration, jerk, lateral forces)

2. **Drive-by-Wire Abstraction Layer** - A hardware abstraction layer that:
   - Provides a unified API for vehicle control
   - Handles protocol translation (CAN/J1939/FlexRay)
   - Manages redundancy and failover for safety-critical systems
   - Enforces latency budgets (≤20ms link, ≤80ms end-to-end)

3. **Safety Bundle Requirements** - Each supported vehicle must have:
   - HARA/FMEA documentation
   - Stopping distance certification
   - HiL test results
   - Formal safety case deltas

4. **Conformance Testing Framework** - Automated validation that:
   - Verifies profile accuracy through closed-course testing
   - Validates control performance across operating conditions
   - Tests degraded mode behaviors

## Guardrails

| Guardrail | Description | Enforcement Mechanism | Violation Response |
| --- | --- | --- | --- |
| **Variant Budget** | No more than 5% core code delta and 25% test-time delta per release for vehicle-specific code | Static code analysis in CI pipeline; test runtime metrics | PR rejection; architecture review; refactoring requirement |
| **No Core Forks** | Vehicle differences must be expressed through configuration, not code forks | Code structure validation; config schema enforcement | PR rejection; architecture review; design pattern guidance |
| **Safety Case Updates** | Each new vehicle requires safety case deltas and certification | Release gate checks for safety artifacts; compliance validation | Release block; safety review board escalation |
| **Mandatory Testing** | HiL and closed-course validation for each supported vehicle class | Test coverage reports; validation checklist enforcement | Release block; test plan review; validation requirement |

## KPIs/SLOs

| KPI/SLO | Description | Target | Measurement Method | Reporting Frequency | Owner |
| --- | --- | --- | --- | --- | --- |
| **Availability** | Percentage of time vehicles are operational and mission-capable | ≥99.0% across ≥3 vehicle classes | (Operational hours - Downtime) / Scheduled hours | Daily, Weekly rollup | Operations Lead |
| **Assist Rate** | Number of human interventions required per distance traveled | ≤2 per 1,000 km | Count of assist events / Distance traveled | Daily, Weekly trend | Safety Lead |
| **Control Accuracy** | Deviation from expected behavior for critical maneuvers | Controlled stop distance within ±5% of profile spec | Closed-course testing; Telemetry analysis | Per release, Monthly validation | Vehicle Integration Lead |
| **Telemetry Completeness** | Percentage of expected telemetry data successfully captured | Gap <0.5% per 24h | (Received data points / Expected data points) | Daily | Data Platform Lead |
| **Onboarding Efficiency** | Time required to integrate new vehicle models | ≤14 days for supported classes | Project tracking from start to production-ready | Per vehicle onboarding | Integration Team Lead |
| **Profile Accuracy** | Correlation between profile parameters and actual behavior | ≥95% match on critical parameters | Validation testing; Telemetry comparison | Per profile version | Vehicle Dynamics Lead |

## Implementation Paths

| Component | Path | Description | Responsible Team | Dependencies |
| --- | --- | --- | --- | --- |
| **Vehicle Profiles** | `/configs/vehicles/*` | YAML profiles defining physical and operational characteristics of each supported vehicle | Vehicle Integration | Vehicle specifications; Calibration data |
| **ODD Limits** | `/rules/odd/vehicle_limits.rego` | Policy rules defining operational design domain limits specific to vehicle classes | Safety & Compliance | ODD framework; Policy engine |
| **Simulation Scenarios** | `/sim/scenario-bank/vehicle/*` | Simulation scenarios for validating vehicle-specific behavior | Simulation | Scenario framework; Vehicle dynamics models |
| **HAL Implementation** | `/edge/vehicle-agent/hal/*` | Hardware abstraction layer implementing the vehicle-agnostic interface | Edge Platform | Vehicle interfaces; Control systems |
| **Profile Validator** | `/tools/profile-validator/*` | Tools for validating vehicle profiles against specifications and real-world data | QA & Validation | Test framework; Telemetry processing |
| **Safety Case Generator** | `/compliance/safety-case/vehicle/*` | Tools for generating vehicle-specific safety case artifacts | Safety & Compliance | Safety framework; Evidence collection |

### Integration Points

1. **Control Systems** - The HAL integrates with vehicle control systems via standardized interfaces
2. **Perception System** - Vehicle profiles inform perception parameters (sensor placement, field of view)
3. **Planning & Control** - Vehicle dynamics parameters feed into motion planning and control algorithms
4. **Diagnostics** - Vehicle-specific diagnostic codes and health monitoring
5. **OTA Updates** - Vehicle-specific firmware and configuration management

## Consequences

### Positive

| Consequence | Description | Beneficiaries | Value Proposition |
| --- | --- | --- | --- |
| **Multi-vehicle Support** | Enables support for multiple vehicle classes without code forks | Product, Sales, Customers | Broader market reach; fleet flexibility; reduced time-to-market |
| **Maintenance Efficiency** | Reduces long-term maintenance burden through configuration-driven approach | Engineering, Operations | Lower TCO; faster updates; more reliable releases |
| **Safety Validation** | Provides clear safety validation path for new vehicles | Safety, Compliance, Regulators | Streamlined certification; consistent safety properties; audit readiness |
| **Fleet Heterogeneity** | Allows customers to operate mixed fleets with consistent behavior | Customers, Operations | Operational flexibility; phased deployment; resilience to supply chain issues |
| **Knowledge Consolidation** | Centralizes vehicle expertise in profiles rather than scattered code | Engineering, Documentation | Better knowledge management; improved onboarding; reduced bus factor |

### Negative

| Consequence | Description | Mitigation Strategy | Residual Risk |
| --- | --- | --- | --- |
| **Initial Development Overhead** | Additional effort (+30-40%) for profile system and testing framework | Phased implementation; reusable components; automation | Medium - Initial investment required but amortized across vehicle types |
| **QA Complexity** | Increased testing matrix complexity (+10-15%) | Automated test generation; prioritized test paths; simulation | Low - Managed through automation and simulation |
| **Profile Accuracy Risk** | Potential for subtle vehicle-specific bugs if profiles are inaccurate | Validation testing; telemetry verification; gradual deployment | Low - Caught by validation process and monitoring |
| **Performance Overhead** | Small runtime overhead from abstraction layer | Performance optimization; critical path analysis; caching | Very Low - Negligible impact on control loop timing |
| **Expertise Requirements** | Need for specialized knowledge in vehicle dynamics and control systems | Training program; documentation; expert consulting | Medium - Requires investment in team capabilities |

## Kill-Switch Criteria

If any of the following occur, we will pause new vehicle onboarding and revert to Tier-A vehicles only:
1. ≥2 critical incidents attributable to profile gaps in 2 quarters
2. >25% schedule slip due to vehicle variance

## Alternatives Considered

| Alternative | Description | Pros | Cons | Why Rejected |
| --- | --- | --- | --- | --- |
| **Vehicle-specific code branches** | Maintain separate code branches for each vehicle type | Perfect optimization for each vehicle; Direct mapping to OEM specs | Maintenance burden; Code divergence; Exponential testing matrix | Unsustainable maintenance burden; High risk of inconsistent behavior |
| **Fully generic approach without profiles** | Single codebase with runtime parameter adjustment | Maximum code reuse; Simplified maintenance | Safety concerns with generic parameters; Performance limitations; Difficult certification | Cannot guarantee safety properties across diverse vehicle dynamics |
| **OEM-provided control systems only** | Rely on OEM interfaces and control systems | Leverages OEM expertise; Simplified integration | Vendor lock-in; Inconsistent capabilities; Limited retrofit options | Strategic constraint; Limits market reach; Inconsistent feature set |

## Compliance & Regulatory Considerations

1. **ISO 26262 (Functional Safety)**: Each vehicle profile requires specific HARA/FMEA documentation and safety case deltas
2. **SOTIF (ISO 21448)**: Performance boundary conditions must be defined per vehicle class
3. **UNECE R155/R156**: Cybersecurity and OTA updates must account for vehicle-specific parameters
4. **Regional Certifications**: Profile-specific evidence bundles support regional type approvals

## Monitoring & Metrics

| Metric | Description | Target | Warning Threshold | Critical Threshold | Data Source |
| --- | --- | --- | --- | --- | --- |
| Profile Accuracy | Deviation between expected and actual vehicle behavior | <3% | 3-5% | >5% | Telemetry comparison |
| Vehicle Onboarding Time | Days from vehicle delivery to production readiness | <14 days | 14-21 days | >21 days | Project tracking |
| Code Variance | Percentage of vehicle-specific code | <5% | 5-7% | >7% | Static code analysis |
| Test Coverage | Percentage of vehicle-specific code paths tested | >95% | 90-95% | <90% | Test coverage reports |
| Assist Rate | Assists required per 1,000 km attributable to vehicle interface | <1 | 1-2 | >2 | Assist classification data |

## Implementation Milestones

1. **Q1**: Base vehicle abstraction layer with support for 2 vehicle classes
2. **Q2**: Profile system with automated validation; support for 3 vehicle classes
3. **Q3**: Full safety case integration; support for 4 vehicle classes
4. **Q4**: Performance optimization; support for 5+ vehicle classes

## References

1. Vehicle Abstraction Layer Design Document (`docs/design/vehicle-abstraction-layer.md`)
2. Vehicle Profile Schema (`configs/vehicles/schema.json`)
3. ISO 26262 Compliance Strategy (`docs/safety/iso-26262-strategy.md`)
4. Vehicle Onboarding Procedure (`docs/operations/vehicle-onboarding.md`)
