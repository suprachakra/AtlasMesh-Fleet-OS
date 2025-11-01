# M7 — Shovel-Truck Coordination

## Basic Information

**ID:** M7  
**Name:** Shovel-Truck Coordination  
**Sector:** Mining  
**Status:** Approved  
**Version:** 1.0  
**Last Updated:** 2025-09-14  
**Owner:** Mining Sector Lead

## Use Case Definition

**Actors:**
- Primary: Shovel Operators, Dispatch Supervisor
- Supporting: Mine Planning, Production Management, Maintenance

**Trip Type:** LOAD_CYCLE_RUN

**ODD (Operational Design Domain):**
- Geographic: Loading areas, shovel operating zones, approach/departure corridors
- Environmental: Dust control areas, temperature range -10°C to 55°C
- Time: 24/7 operations aligned with shovel production
- Communications: Mine LTE/WiFi with local fallback
- Other: Close coordination with manual shovel operations

**Trigger:**
Shovel availability signal or dispatch assignment

**Nominal Flow:**
1. Dispatch system assigns haul trucks to shovels based on production targets and material types
2. System calculates optimal approach sequence and spacing for assigned trucks
3. Trucks navigate to shovel staging area with precise timing to minimize queue and shovel wait time
4. System coordinates with shovel operator through visual indicators and digital communication
5. Truck executes precise spotting maneuver based on shovel position and operator guidance
6. Loading operation is monitored with payload distribution and stability verification
7. Upon completion signal, truck executes departure maneuver with collision avoidance
8. System tracks cycle metrics and adjusts subsequent truck timing for continuous improvement
9. Production data is synchronized with mine planning and dispatch systems
10. Cycle repeats with dynamic adjustments based on shovel productivity and material characteristics

**Variants / Edge Cases:**
- Shovel repositioning: Queue management and approach replanning
- Double-sided loading: Enhanced coordination for alternating sides
- Material type change: Destination and route updates
- Spotting challenges: Operator-assisted positioning
- Shovel delay: Dynamic queue management and truck reassignment

## Performance & Integration

**KPIs:**
- **P0 (Must have):**
  - Shovel hang time reduction: -20% vs. manual operation
  - Spotting accuracy: ≥95% first-attempt success
  - Cycle time consistency: ≤10% variance
  - Zero collision incidents
- **P1 (Should have):**
  - Payload optimization: +5% average fill factor
  - Fuel efficiency: +10% through optimized spotting
  - Queue time reduction: -30% vs. baseline
  - Truck utilization improvement: +15% through reduced wait time

**Dependencies:**
- **Services:**
  - `dispatch`: Assignment and queue management
  - `routing`: Loading area navigation with precision approach
  - `fleet-health`: Vehicle monitoring during loading operations
  - `analytics`: Cycle time and productivity analysis
  - `adapters/shovel-interface`: Integration with shovel systems
- **Rules:**
  - `rules/odd/mining/loading_zone.yaml`: Loading operation parameters
  - `rules/policy/mining/spotting.yaml`: Positioning and coordination procedures
  - `rules/policy/mining/payload_management.yaml`: Load distribution and limits
  - `rules/policy/mining/shovel_interaction.yaml`: Communication and signaling protocols
- **External Systems:**
  - Mine Production System: Material tracking and production targets
  - Shovel Control System: Position, status, and operator inputs
  - Fleet Management System: Assignment and dispatch integration

**Risks & Mitigations:**
- **Spotting misalignment:**
  - Impact: High
  - Mitigation: Precision positioning systems, operator confirmation, gradual approach protocol
- **Communication failure with shovel:**
  - Impact: High
  - Mitigation: Multi-modal communication (digital, visual indicators), fallback protocols
- **Ground condition changes:**
  - Impact: Medium
  - Mitigation: Recent survey data integration, approach path assessment, operator input
- **Payload imbalance:**
  - Impact: Medium
  - Mitigation: Real-time load monitoring, distribution guidance, operator alerts

## Validation & Deployment

**Acceptance / Test Hooks:**
- **Simulation Scenarios:**
  - `mining/shovel_truck/spotting_precision.json`: Positioning accuracy and consistency
  - `mining/shovel_truck/queue_management.json`: Multiple truck coordination
  - `mining/shovel_truck/exception_handling.json`: Repositioning and communication failures
- **Logs/Telemetry:**
  - Cycle metrics: spotting time, hang time, departure time, total cycle time
  - Positioning metrics: approach accuracy, final position offset, correction attempts
  - Production metrics: payload, fill factor, material type compliance
- **Gates:**
  - Hang time reduction validation with operational data
  - Spotting accuracy verification with position tracking
  - Safety protocol compliance with zero incidents

**Rollout Plan:**
- **Phase 1:** Single shovel operation with limited truck count
- **Phase 2:** Multiple shovel coordination with queue optimization
- **Phase 3:** Full integration with production planning and material tracking

## Additional Information

**Related Use Cases:**
- M1: Pit-to-Crusher Haul Loop
- M2: Overburden Removal Cycle
- M4: Stockpile Reclaim & Grade Control

**References:**
- Shovel-Truck Productivity Standards
- Loading Best Practices Manual
- Mining Cycle Optimization Guide

**Notes:**
This use case represents a critical productivity driver in mining operations, requiring precise coordination between autonomous trucks and manually operated shovels. Success here demonstrates the system's ability to integrate with existing equipment while significantly improving cycle efficiency and reducing operator workload.

## Cross-Functional Considerations

**Product Management:**
- Clear definition of productivity metrics and targets
- Measurable impact on shovel utilization and cycle time
- Integration with existing dispatch and production workflows

**Design:**
- Intuitive operator interface for shovel operators
- Clear visual indicators for positioning and status
- Accessibility for field conditions and operator constraints

**Engineering:**
- Precision positioning systems for spotting maneuvers
- Robust communication protocols with shovel systems
- Reliable operation in high-dust loading environments

**Data:**
- Comprehensive cycle time analysis and optimization
- Payload tracking and distribution monitoring
- Integration with production planning and material tracking

**QA:**
- Validation of spotting accuracy across varied conditions
- Testing of queue management and coordination workflows
- Verification of safety protocols during close-proximity operations

**Security:**
- Protection of production data and dispatch algorithms
- Secure communications with shovel control systems
- Physical security measures for loading operations

**Operations:**
- Clear procedures for shovel-truck coordination
- Training for shovel operators on autonomous interaction
- Exception handling protocols for communication failures

**Safety:**
- Proximity operation safety rules and monitoring
- Loading area stability assessment
- Emergency procedures for loading-related incidents

**Mining Engineering:**
- Integration with production planning systems
- Material tracking and reconciliation
- Loading area design optimization
