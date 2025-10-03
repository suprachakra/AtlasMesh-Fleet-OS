# M2 — Overburden Removal Cycle Coordination

## Basic Information

**ID:** M2  
**Name:** Overburden Removal Cycle Coordination  
**Sector:** Mining  
**Status:** Approved  
**Version:** 1.0  
**Last Updated:** 2025-09-13  
**Owner:** Mining Sector Lead

## Use Case Definition

**Actors:**
- Primary: Mine Planning Engineer, Dispatch Supervisor
- Supporting: Excavator Operators, Haul Truck Fleet, Maintenance Team

**Trip Type:** OP_RUN (continuous loop)

**ODD (Operational Design Domain):**
- Geographic: Overburden removal areas, waste dumps, mining benches
- Environmental: Dust levels ≤ threshold (visibility > 100m), temperature range -10°C to 55°C
- Time: 24/7 operations with shift handover protocols
- Communications: Mine LTE/WiFi with redundant coverage
- Other: Dynamic queueing at multiple excavator faces

**Trigger:**
Mine plan execution with multiple excavator faces active simultaneously

**Nominal Flow:**
1. Planning system publishes excavator locations and production targets
2. Dispatch optimizes truck assignments across multiple excavators based on priority
3. Dynamic queueing algorithm balances wait times to minimize excavator idle time
4. Trucks navigate to assigned excavator with collision avoidance and traffic management
5. Loading sequence optimized for excavator cycle time and truck positioning
6. Waste material classified and routed to appropriate dump location
7. Dump operation executed with precise placement requirements
8. System updates production metrics and adjusts assignments in real-time
9. Continuous rebalancing to maintain optimal excavator utilization

**Variants / Edge Cases:**
- Excavator relocation: Dynamic fleet reassignment
- Dump area congestion: Queue management with priority rules
- Excavator breakdown: Rapid rebalancing to other active faces
- Grade/material change: Routing adjustment to appropriate dump location
- Shift change: Handover protocol with minimal production impact

## Performance & Integration

**KPIs:**
- **P0 (Must have):**
  - Excavator idle time: -25% vs. manual operation
  - Truck bunching index: ≤ 0.15 (measure of fleet distribution)
  - Production target achievement: ≥ 95%
  - Assists per distance: ≤ 0.25/1k km
- **P1 (Should have):**
  - Fuel efficiency: +10% vs. manual operation
  - Queue time variance: ≤ 15% across excavators
  - Cycle time consistency: ≤ 10% variance
  - Dump placement accuracy: ≥ 98% within target zone

**Dependencies:**
- **Services:**
  - `dispatch`: Multi-excavator assignment optimization
  - `rebalancer`: Dynamic fleet distribution
  - `routing`: Waste dump routing with material classification
  - `analytics`: Production tracking and optimization
  - `fleet-health`: Equipment condition monitoring
- **Rules:**
  - `rules/odd/mining/multi_excavator_rules.yaml`: Excavator approach and loading rules
  - `rules/policy/mining/queue_management.yaml`: Queue priority and fairness policies
  - `rules/policy/mining/dump_placement.yaml`: Material placement requirements
- **External Systems:**
  - Mine Planning System: Production targets and excavator locations
  - Fleet Management System: Equipment tracking and status
  - Material Classification System: Waste rock routing

**Risks & Mitigations:**
- **Radio dead zones affecting communications:**
  - Impact: High
  - Mitigation: Store-and-forward protocols, local decision making, mesh network
- **Excavator productivity imbalance:**
  - Impact: Medium
  - Mitigation: Dynamic rebalancing algorithm, production-based assignment weighting
- **Material misclassification:**
  - Impact: Medium
  - Mitigation: Multiple validation points, operator verification, dump location verification
- **Traffic congestion at choke points:**
  - Impact: Medium
  - Mitigation: Traffic flow modeling, passing zone protocols, priority rules

## Validation & Deployment

**Acceptance / Test Hooks:**
- **Simulation Scenarios:**
  - `mining/overburden/multi_excavator_balance.json`: Balancing across 3+ excavators
  - `mining/overburden/excavator_relocation.json`: Dynamic response to excavator moves
  - `mining/overburden/dump_congestion.json`: Management of dump area traffic
- **Logs/Telemetry:**
  - Production metrics: tons moved, cycle times, queue times
  - Balance metrics: excavator utilization, truck distribution
  - Material handling: classification accuracy, placement precision
- **Gates:**
  - Digital twin validation with historical production data
  - Excavator utilization improvement ≥ 20% in simulation
  - Field validation with incremental fleet size

**Rollout Plan:**
- **Phase 1:** Single excavator face with 3-5 trucks
- **Phase 2:** Two excavator faces with dynamic assignment
- **Phase 3:** Full multi-excavator operation with complete fleet

## Additional Information

**Related Use Cases:**
- M1: Pit-to-Crusher Autonomous Haul
- M24: Waste Dump Sequencing
- M9: Autonomous Loader-Truck Sync

**References:**
- Mine Production Optimization Handbook
- Overburden Management Best Practices
- Fleet Balancing Algorithm Documentation

**Notes:**
This use case focuses on optimizing the coordination between multiple loading units and haul trucks, which is critical for maximizing mine productivity. Success here demonstrates the system's ability to handle complex multi-asset scheduling problems.
