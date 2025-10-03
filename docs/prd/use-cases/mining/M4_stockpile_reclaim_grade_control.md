# M4 — Stockpile Reclaim & Grade Control

## Basic Information

**ID:** M4  
**Name:** Stockpile Reclaim & Grade Control  
**Sector:** Mining  
**Status:** Approved  
**Version:** 1.0  
**Last Updated:** 2025-09-13  
**Owner:** Mining Sector Lead

## Use Case Definition

**Actors:**
- Primary: Process Plant Manager, Metallurgical Team
- Supporting: Stockpile Operators, Quality Control, Mine Planning

**Trip Type:** OP_RUN (specialized)

**ODD (Operational Design Domain):**
- Geographic: Stockpile areas, reclaim feeders, crusher approaches
- Environmental: Dust control areas with visibility monitoring, temperature range -10°C to 55°C
- Time: 24/7 operations aligned with plant feed requirements
- Communications: Mine LTE/WiFi with local fallback
- Other: Material grade tracking and verification systems

**Trigger:**
Process plant feed requirements or grade blending targets

**Nominal Flow:**
1. Metallurgical team defines grade targets and blending requirements
2. System identifies optimal reclaim sequence from available stockpiles
3. Autonomous vehicles are dispatched with grade-specific assignments
4. Vehicles navigate to designated stockpile locations with precise positioning
5. Material sampling and verification occurs during loading process
6. Grade tracking system maintains real-time blend calculations
7. Vehicles deliver material to crusher or direct feed points with grade tagging
8. System continuously adjusts reclaim sequence based on real-time assay results
9. Plant performance metrics are monitored for correlation with feed quality
10. Comprehensive material movement and grade tracking data is maintained

**Variants / Edge Cases:**
- Grade deviation detection: Corrective blending adjustments
- Stockpile depletion: Dynamic replanning with available materials
- Sampling system failure: Fallback to conservative grade estimates
- Weather impact on material properties: Moisture compensation
- Plant feed rate changes: Dynamic adjustment of delivery sequence

## Performance & Integration

**KPIs:**
- **P0 (Must have):**
  - Grade variance: ≤ 5% from target specifications
  - Misrouting rate: ≤ 0.5% of material movements
  - Plant throughput stability: ≥ 95% of target
  - Recovery impact: ≥ 2% improvement in mineral recovery
- **P1 (Should have):**
  - Rehandling reduction: -25% vs. conventional methods
  - Energy efficiency: +15% per ton processed
  - Blend optimization: ≥ 90% of theoretical optimal blend
  - Stockpile inventory accuracy: ≥ 98%

**Dependencies:**
- **Services:**
  - `dispatch`: Grade-aware vehicle assignment
  - `routing`: Stockpile-specific navigation
  - `analytics`: Grade tracking and blend optimization
  - `map-service`: Stockpile modeling and volumetrics
  - `adapters/wms`: Integration with material management systems
- **Rules:**
  - `rules/odd/mining/stockpile_rules.yaml`: Stockpile operation parameters
  - `rules/policy/mining/grade_control.yaml`: Material handling and verification
  - `rules/policy/mining/sampling.yaml`: Quality assurance protocols
- **External Systems:**
  - Laboratory Information Management System: Assay results and quality control
  - Plant Control System: Feed rate and performance metrics
  - Mine Planning System: Material classification and stockpile allocation

**Risks & Mitigations:**
- **Grade misclassification:**
  - Impact: High
  - Mitigation: Multiple verification points, real-time sampling, RFID/barcode tracking
- **Stockpile model inaccuracy:**
  - Impact: Medium
  - Mitigation: Regular drone surveys, reconciliation with extracted volumes, conservative margin
- **Sampling bias:**
  - Impact: High
  - Mitigation: Multiple sampling points, statistical validation, regular calibration
- **Material property changes (moisture, density):**
  - Impact: Medium
  - Mitigation: Real-time monitoring, adaptive handling parameters, weather compensation

## Validation & Deployment

**Acceptance / Test Hooks:**
- **Simulation Scenarios:**
  - `mining/stockpile/blend_optimization.json`: Complex multi-grade blending
  - `mining/stockpile/quality_deviation.json`: Handling of off-spec material
  - `mining/stockpile/inventory_management.json`: Stockpile depletion and rotation
- **Logs/Telemetry:**
  - Grade metrics: target vs. actual, blend consistency, sampling frequency
  - Material movement: volumes, locations, timestamps, chain of custody
  - Plant performance: throughput, recovery, energy consumption correlation
- **Gates:**
  - Grade control accuracy validation with laboratory verification
  - Plant performance improvement ≥ 2% in controlled tests
  - Stockpile inventory reconciliation ≤ 2% variance

**Rollout Plan:**
- **Phase 1:** Single material type with basic grade tracking
- **Phase 2:** Multiple grades with blend optimization
- **Phase 3:** Full integration with plant control systems and real-time adjustment

## Additional Information

**Related Use Cases:**
- M1: Pit-to-Crusher Autonomous Haul
- M2: Overburden Removal Cycle Coordination
- M12: Lab Sample Courier

**References:**
- Metallurgical Process Control Standards
- Stockpile Management Best Practices
- Material Sampling and Analysis Protocols

**Notes:**
This use case represents a sophisticated application of autonomous material handling with direct impact on processing efficiency and mineral recovery. Success here demonstrates the system's ability to integrate with complex process control requirements while maintaining material quality and traceability.

## Cross-Functional Considerations

**Product Management:**
- Clear definition of grade specifications and tolerances
- Measurable impact on plant performance metrics
- Integration with existing material management workflows

**Design:**
- Intuitive visualization of blend targets and actual performance
- Clear status indicators for material quality and sampling status
- Accessibility for operators in control room and field environments

**Engineering:**
- Robust integration with sampling systems and laboratory interfaces
- Real-time grade tracking with minimal latency
- Redundant verification mechanisms for critical material movements

**Data:**
- Comprehensive material genealogy from mine to mill
- Statistical validation of sampling adequacy
- Long-term storage of quality data for process optimization

**QA:**
- Validation of grade accuracy across diverse material types
- Performance testing under varying plant demand scenarios
- Verification of material traceability and chain of custody

**Security:**
- Protection of proprietary blend recipes and processing parameters
- Access controls for grade adjustment and override functions
- Audit trails for all quality-impacting decisions

**Operations:**
- Clear procedures for handling off-spec material
- Training for quality control personnel on system interaction
- Maintenance protocols for sampling equipment and sensors

**Metallurgy:**
- Integration with metallurgical accounting systems
- Feedback loops for recovery optimization
- Reconciliation with downstream process performance
