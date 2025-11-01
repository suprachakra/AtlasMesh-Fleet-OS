# ADR-0004: Sensor-Agnostic Architecture

* **Status:** Accepted
* **Date:** 2025-09-14
* **Proposer:** Perception Team

## Context

AtlasMesh Fleet OS must operate with different sensor configurations across various environments and use cases. Different sectors and operational domains have varying requirements for sensor capabilities, environmental resilience, and cost constraints. We need to support multiple sensor modalities and vendors without maintaining separate perception stacks.

## Decision

We will implement a **sensor-agnostic architecture** with the following key components:

1. **Sensor Hardware Abstraction Layer (HAL)** - A unified interface that:
   - Abstracts vendor-specific APIs and protocols
   - Provides consistent data formats and coordinate systems
   - Handles timing and synchronization
   - Manages sensor health monitoring and degradation

2. **Certified Sensor Packs** - Pre-defined sensor configurations:
   - **Rugged-A**: 64-beam LiDAR + radars + thermal + HDR cameras (mining/defense)
   - **Urban-B**: Solid-state LiDAR + cameras + radar (ride-hail/logistics)
   - Each pack with defined performance characteristics and environmental ratings
   - Validated calibration procedures and mounting specifications

3. **Pack-Specific Perception Models** - ML models optimized for:
   - Specific sensor combinations within each pack
   - Environmental conditions relevant to the pack's use case
   - Degraded operation when sensors fail or perform sub-optimally

4. **Sensor Fusion Framework** - A modular system that:
   - Combines data from multiple sensor modalities
   - Adapts to available sensors and their health status
   - Provides confidence metrics for perception outputs
   - Implements fail-silent policies for unreliable data

## Guardrails

1. **Certified Packs Only** - No ad-hoc sensor combinations in production
2. **Latency Budgets** - Strict performance requirements for each processing stage
3. **Calibration Requirements** - Formal procedures and validation for sensor alignment
4. **Fail-Silent Policy** - Clear handling of sensor degradation and failure

## KPIs/SLOs

1. Perception latency p95 ≤60 ms (pack-specific)
2. Precision/recall ≥ pack-specific targets
3. False-stop rate ≤1/2,000 km
4. Pack swap without application code changes

## Implementation Paths

1. `/adapters/sensors/*` - Vendor-specific adapters for the HAL
2. `/ml/models/perception/*` - Pack-specific perception models
3. `/data/quality/drift/*` - Sensor drift detection and monitoring

## Consequences

### Positive

1. Enables support for multiple sensor configurations
2. Provides clear upgrade paths as sensor technology evolves
3. Allows for cost/performance optimization per deployment
4. Simplifies vendor management and supply chain resilience

### Negative

1. Initial development overhead (+25-35%) for abstraction and pack validation
2. Increased test matrix complexity (+15%)
3. Potential for sub-optimal perception compared to fully customized solutions
4. Calibration and maintenance complexity

## Kill-Switch Criteria

If 2+ perception-attributable incidents occur per quarter for a specific sensor pack, that pack will be frozen from further deployment until root causes are addressed.

## Alternatives Considered

1. **Single reference sensor suite** - Rejected due to inflexibility and supply chain risk
2. **Fully customizable sensor configurations** - Rejected due to validation complexity
3. **Vendor-specific perception stacks** - Rejected due to maintenance burden and lock-in
