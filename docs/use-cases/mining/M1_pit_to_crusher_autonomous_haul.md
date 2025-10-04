# M1 — Pit-to-Crusher Autonomous Haul

## Basic Information

**ID:** M1  
**Name:** Pit-to-Crusher Autonomous Haul  
**Sector:** Mining  
**Status:** Approved  
**Version:** 1.0  
**Last Updated:** 2025-09-13  
**Owner:** Mining Sector Lead

## Use Case Definition

**Actors:**
- Primary: Dispatch, Haul Trucks
- Supporting: Excavator Operator (loader), Crusher Operator, Mine Operations Center

**Trip Type:** OP_RUN (loop)

**ODD (Operational Design Domain):**
- Geographic: Mine haul roads, fixed route networks
- Environmental: Dust ≤ threshold (visibility > 100m), slope ≤ Smax (typically 10-12%), temperature range -10°C to 55°C
- Time: 24/7 operations
- Communications: Mine LTE/WiFi with satellite backup
- Other: Heat derate curves loaded from vehicle profile

**Trigger:**
Loader queue readiness signal + truck SOC/fuel sufficiency check

**Nominal Flow:**
1. Dispatch assigns truck to loading slot based on queue optimization
2. Truck navigates to loader rendezvous point with precise positioning
3. Truck receives load and performs on-board weighing
4. System selects optimal route to crusher based on current conditions
5. Truck navigates to crusher dump point with traffic awareness
6. Dump operation is executed with crusher coordination
7. System logs production metrics and updates fleet analytics
8. Truck begins return loop to loader queue

**Variants / Edge Cases:**
- Dust occlusion: Sensor fusion adaptation and speed adjustment
- Blocked ramp: Dynamic rerouting or coordinated safe-stop
- Slippery grade after rain: Traction control adaptation
- Grader in lane: Slow/stop and wait or coordinate passing
- Crusher backup: Queue management at dump point

## Performance & Integration

**KPIs:**
- **P0 (Must have):**
  - Production increase: +8-12% tons/hour vs. manual operation
  - Assists per distance: ≤0.2/1k km
  - Fleet availability: ≥99.5%
  - Cost per ton reduction: -13% vs. manual operation
- **P1 (Should have):**
  - Fuel efficiency: +15% vs. manual operation
  - Tire life extension: +20%
  - Cycle time consistency: <5% variance
  - Dust-related slowdowns: -30% vs. manual operation

**Dependencies:**
- **Services:**
  - `routing`: Mine-specific path planning
  - `dispatch`: Queue optimization and assignment
  - `energy-manager`: Fuel/battery optimization
  - `fleet-health`: Vehicle condition monitoring
  - `predictive-maint`: Component health prediction
  - `weather-fusion`: Dust and visibility monitoring
- **Rules:**
  - `rules/odd/mining/haul_road_rules.yaml`: Road grade and width constraints
  - `rules/policy/mining/queue_management.yaml`: Loader and crusher queue policies
  - `rules/policy/safety/dust_adaptation.yaml`: Visibility-based speed adjustments
- **External Systems:**
  - Mine Production System: Tonnage tracking and shift targets
  - Fleet Management System: Integration with existing mine systems
  - Weighbridge System: Load verification

**Risks & Mitigations:**
- **Heat derating impact on production:**
  - Impact: High
  - Mitigation: Heat-aware dispatch pacing, thermal management, night shift bias
- **Dust impact on perception:**
  - Impact: High
  - Mitigation: Multi-sensor fusion, air-knife control systems, dust-aware routing
- **Mixed fleet interactions:**
  - Impact: Medium
  - Mitigation: V2V communications, conservative spacing policies, human vehicle awareness
- **Slope safety:**
  - Impact: High
  - Mitigation: Grade-specific speed profiles, enhanced brake monitoring, runaway ramps

## Validation & Deployment

**Acceptance / Test Hooks:**
- **Simulation Scenarios:**
  - `mining/haul/low_visibility_steep_grade.json`: Dust + maximum grade combination
  - `mining/haul/crusher_congestion.json`: Multiple trucks at dump point
  - `mining/haul/mixed_traffic_interaction.json`: Light vehicles and heavy equipment
- **Logs/Telemetry:**
  - Production metrics: tons/hour, cycle times
  - Vehicle health: brake temperatures, engine load, tire pressure
  - Environmental conditions: dust levels, temperature, precipitation
- **Gates:**
  - Throughput model validation against baseline
  - Safety incident rate = 0 in simulation
  - Heat management effectiveness in >45°C conditions

**Rollout Plan:**
- **Phase 1:** Single truck on dedicated haul segment during day shift
- **Phase 2:** Expand to 3-5 trucks on full pit-to-crusher loop
- **Phase 3:** Full fleet integration with 24/7 operations

## Additional Information

**Related Use Cases:**
- M2: Overburden Removal Cycle Coordination
- M3: Autonomous Refuel/Recharge Sweeper
- M4: Stockpile Reclaim & Grade Control

**References:**
- Mine Haul Road Design Standards
- Dust Management Plan
- Heat Management Protocol for Heavy Equipment

**Notes:**
This use case represents the core production workflow for mining operations and directly impacts the primary KPI of tons/hour. Success here demonstrates immediate ROI and builds credibility for expansion to other mining use cases.
