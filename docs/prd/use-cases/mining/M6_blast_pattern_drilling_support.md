# M6 — Blast Pattern Drilling Support

## Basic Information

**ID:** M6  
**Name:** Blast Pattern Drilling Support  
**Sector:** Mining  
**Status:** Approved  
**Version:** 1.0  
**Last Updated:** 2025-09-14  
**Owner:** Mining Sector Lead

## Use Case Definition

**Actors:**
- Primary: Drill & Blast Engineer, Drill Operators
- Supporting: Mine Planning, Geology Team, Safety Department

**Trip Type:** DRILL_SUPPORT_RUN

**ODD (Operational Design Domain):**
- Geographic: Active mining benches, drill pattern areas
- Environmental: Dust control areas, temperature range -10°C to 55°C, vibration exposure
- Time: 24/7 operations aligned with drilling schedules
- Communications: Mine LTE/WiFi with local fallback
- Other: Operation in proximity to drilling equipment and uneven terrain

**Trigger:**
Scheduled drilling operations or blast pattern implementation

**Nominal Flow:**
1. Drill & Blast Engineer uploads blast pattern design to the system
2. System generates support mission plan with precise material requirements
3. Autonomous vehicles are loaded with drill consumables (rods, bits, water, fuel)
4. Vehicles navigate to active drilling locations with terrain-aware routing
5. System coordinates with drill rigs to prioritize support delivery based on consumption rates
6. Vehicles position precisely for efficient transfer of consumables to drill rigs
7. System tracks consumption and coordinates replacement cycles
8. Real-time monitoring of drilling progress influences support scheduling
9. Upon pattern completion, system coordinates final material collection
10. Mission concludes with consumption reporting and next pattern preparation

**Variants / Edge Cases:**
- Drill plan modification: Dynamic replanning of support logistics
- Consumable shortage: Priority-based allocation and expedited resupply
- Drill rig breakdown: Resource reallocation and schedule adjustment
- Bench access restrictions: Alternative routing and staging areas
- Weather impact: Modified delivery schedule and material protection

## Performance & Integration

**KPIs:**
- **P0 (Must have):**
  - Drill utilization improvement: +15% through reduced wait time
  - Consumable delivery accuracy: ≥98% on-time delivery
  - Drill pattern completion: -10% cycle time vs. manual support
  - Zero safety incidents with drill rig interactions
- **P1 (Should have):**
  - Fuel efficiency: +12% vs. conventional support
  - Consumable waste reduction: -20% through precise delivery
  - Support vehicle utilization: +25% through optimized scheduling
  - Material inventory accuracy: ≥99%

**Dependencies:**
- **Services:**
  - `dispatch`: Support mission planning and scheduling
  - `routing`: Bench-aware navigation with terrain consideration
  - `fleet-health`: Vehicle monitoring in high-dust environment
  - `analytics`: Consumption tracking and prediction
  - `adapters/drill-management`: Integration with drill control systems
- **Rules:**
  - `rules/odd/mining/drill_support.yaml`: Drill support operation parameters
  - `rules/policy/mining/consumable_management.yaml`: Material handling and prioritization
  - `rules/policy/mining/drill_proximity.yaml`: Safe operating distances and procedures
  - `rules/policy/mining/bench_access.yaml`: Terrain and access constraints
- **External Systems:**
  - Drill & Blast Design System: Pattern specifications and progress tracking
  - Drill Control System: Operational status and consumable requirements
  - Inventory Management System: Consumable tracking and replenishment

**Risks & Mitigations:**
- **Drill plan changes:**
  - Impact: Medium
  - Mitigation: Real-time synchronization with blast design system, dynamic replanning capability
- **Terrain instability near drill sites:**
  - Impact: High
  - Mitigation: Recent survey data integration, conservative approach paths, stability monitoring
- **Dust interference with sensors:**
  - Impact: Medium
  - Mitigation: Enhanced sensor cleaning systems, redundant sensing modalities, dust-aware navigation
- **Drill rig proximity operations:**
  - Impact: High
  - Mitigation: Precise positioning systems, coordinated approach protocols, operator confirmation

## Validation & Deployment

**Acceptance / Test Hooks:**
- **Simulation Scenarios:**
  - `mining/drill_support/consumable_delivery.json`: Precise positioning and transfer
  - `mining/drill_support/pattern_coordination.json`: Multi-rig support optimization
  - `mining/drill_support/terrain_navigation.json`: Bench access and positioning
- **Logs/Telemetry:**
  - Support metrics: delivery timing, positioning accuracy, transfer efficiency
  - Consumption metrics: usage rates, inventory levels, waste reduction
  - Coordination metrics: drill wait time, pattern completion rate, replanning frequency
- **Gates:**
  - Drill utilization improvement validation with operational data
  - Safety protocol verification with zero incidents
  - Consumable management efficiency with inventory reconciliation

**Rollout Plan:**
- **Phase 1:** Single drill support with basic consumable delivery
- **Phase 2:** Multiple drill coordination with consumption prediction
- **Phase 3:** Full pattern optimization with integrated blast design

## Additional Information

**Related Use Cases:**
- M1: Pit-to-Crusher Haul Loop
- M2: Overburden Removal Cycle
- M5: Haul Road Condition Patrol

**References:**
- Drill & Blast Operations Manual
- Consumable Management Best Practices
- Bench Safety Standards

**Notes:**
This use case represents a specialized support operation that directly impacts mining productivity through improved drill utilization. Success here demonstrates the system's ability to coordinate with existing mining equipment while operating in challenging terrain and dusty conditions.

## Cross-Functional Considerations

**Product Management:**
- Clear definition of drill support requirements and KPIs
- Measurable impact on pattern completion time and drill utilization
- Integration with existing drill & blast workflows

**Design:**
- Intuitive visualization of drill pattern progress and support status
- Clear coordination interfaces between support vehicles and drill operators
- Accessibility for field conditions and high-visibility requirements

**Engineering:**
- Precise positioning systems for consumable transfer
- Robust operation in high-dust and vibration environments
- Reliable terrain assessment for bench navigation

**Data:**
- Comprehensive consumption tracking and prediction
- Pattern progress monitoring and completion metrics
- Integration with blast design and drill control data

**QA:**
- Validation of support operations across varied bench conditions
- Testing of coordination workflows with multiple drill rigs
- Verification of safety protocols in proximity operations

**Security:**
- Protection of blast pattern information
- Secure communications with drill control systems
- Physical security measures for explosive-adjacent operations

**Operations:**
- Clear procedures for consumable loading and transfer
- Training for support coordination with drill operators
- Maintenance protocols for dust-exposed vehicles

**Safety:**
- Proximity operation safety rules and monitoring
- Bench stability assessment integration
- Emergency procedures for drill-related incidents

**Mining Engineering:**
- Integration with blast design systems
- Drill pattern optimization feedback
- Consumable requirement specification
