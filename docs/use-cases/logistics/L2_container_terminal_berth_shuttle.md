# L2 — Container Terminal Berth Shuttle (Quay ↔ Yard)

## Basic Information

**ID:** L2  
**Name:** Container Terminal Berth Shuttle (Quay ↔ Yard)  
**Sector:** Logistics  
**Status:** Approved  
**Version:** 1.0  
**Last Updated:** 2025-09-13  
**Owner:** Logistics Sector Lead

## Use Case Definition

**Actors:**
- Primary: Terminal Operations Manager, Vessel Operations Coordinator
- Supporting: Ship-to-Shore (STS) Crane Operators, RTG/RMG Operators, Gate Operations

**Trip Type:** OP_RUN (continuous synchronized loops)

**ODD (Operational Design Domain):**
- Geographic: Container terminal quay and yard areas with designated lanes
- Environmental: All weather conditions with appropriate adjustments, wind ≤ 60 km/h for loaded operations
- Time: 24/7 operations aligned with vessel berthing windows
- Communications: Terminal WiFi/LTE with high reliability requirements
- Other: Synchronized operations with STS cranes and yard equipment

**Trigger:**
Vessel arrival and berth allocation in Terminal Operating System (TOS)

**Nominal Flow:**
1. TOS publishes vessel work plan with container moves and priorities
2. System calculates optimal shuttle assignments based on crane productivity targets
3. Autonomous terminal tractors are dispatched to STS crane queues with anti-bunching algorithms
4. Tractors position precisely under STS cranes with vision-guided alignment
5. After container loading, tractors navigate to assigned yard blocks with traffic management
6. RTG/RMG coordination for precise positioning and container handoff
7. System continuously rebalances fleet based on real-time crane productivity
8. Exception handling for crane delays, equipment failures, or container issues
9. Performance metrics tracked against vessel productivity targets

**Variants / Edge Cases:**
- Crane delay: Dynamic queue management and rebalancing
- Hot hatch prioritization: Expedited container handling for priority moves
- Equipment failure: Rapid reassignment and recovery
- Yard congestion: Alternative stacking locations with TOS update
- Weather degradation: Adjusted speeds and safety parameters

## Performance & Integration

**KPIs:**
- **P0 (Must have):**
  - Berth productivity: Moves per hour ≥ 25 per crane
  - Crane idle time: -15-25% vs. manual operation
  - Vessel turnaround time: -10% vs. baseline
  - Safety incidents: 0
- **P1 (Should have):**
  - Fuel/energy efficiency: +15% vs. manual operation
  - Equipment utilization: +20% vs. baseline
  - Yard congestion: -30% vs. baseline
  - Tractor cycle time consistency: ≤ 10% variance

**Dependencies:**
- **Services:**
  - `adapters/tos`: Integration with Terminal Operating System
  - `dispatch`: Synchronized tractor assignment
  - `rebalancer`: Dynamic fleet distribution for crane productivity
  - `routing`: Terminal-specific path planning with traffic management
  - `v2x-service`: Equipment coordination and positioning
- **Rules:**
  - `rules/odd/logistics/terminal_rules.yaml`: Terminal-specific operational rules
  - `rules/policy/logistics/crane_coordination.yaml`: STS and yard crane interaction policies
  - `rules/policy/logistics/container_priority.yaml`: Container handling priorities
- **External Systems:**
  - Terminal Operating System: Work instructions and container tracking
  - Crane Management System: Productivity and status monitoring
  - Yard Management System: Container location and stacking rules

**Risks & Mitigations:**
- **Crane synchronization failures:**
  - Impact: High
  - Mitigation: Real-time position tracking, predictive positioning, fallback protocols
- **Terminal congestion:**
  - Impact: Medium
  - Mitigation: Dynamic routing, yard density monitoring, alternative path planning
- **TOS integration reliability:**
  - Impact: High
  - Mitigation: Transaction logging, error recovery, manual override capability
- **Equipment conflicts:**
  - Impact: High
  - Mitigation: V2X communication, equipment reservation system, priority rules

## Validation & Deployment

**Acceptance / Test Hooks:**
- **Simulation Scenarios:**
  - `logistics/terminal/vessel_operation.json`: Complete vessel operation simulation
  - `logistics/terminal/crane_synchronization.json`: STS crane coordination
  - `logistics/terminal/yard_congestion.json`: Peak yard traffic management
- **Logs/Telemetry:**
  - Productivity metrics: moves per hour, cycle times, equipment utilization
  - Synchronization metrics: crane wait times, positioning accuracy
  - Exception handling: recovery times, manual interventions
- **Gates:**
  - Digital twin validation with historical vessel operation data
  - Crane productivity improvement ≥ 15% in simulation
  - Field validation with incremental fleet size

**Rollout Plan:**
- **Phase 1:** Single berth operation during off-peak hours
- **Phase 2:** Multiple berth operations with limited vessel types
- **Phase 3:** Full terminal integration with all vessel types and peak operations

## Additional Information

**Related Use Cases:**
- L1: Yard Switcher: Dock ↔ Yard Slot Shuttling
- L9: Empty Container Repositioning
- L12: Port ↔ ICD Shuttle

**References:**
- Terminal Operations Best Practices
- Container Handling Equipment Standards
- Vessel Productivity Optimization Guide

**Notes:**
This use case represents a high-throughput, time-critical operation that directly impacts vessel turnaround times and terminal productivity. Success here demonstrates the system's ability to coordinate with multiple external systems and equipment types in a dynamic environment.
