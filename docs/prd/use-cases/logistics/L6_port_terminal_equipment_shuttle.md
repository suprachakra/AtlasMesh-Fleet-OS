# L6 — Port Terminal Equipment Shuttle

## Basic Information

**ID:** L6  
**Name:** Port Terminal Equipment Shuttle  
**Sector:** Logistics  
**Status:** Approved  
**Version:** 1.0  
**Last Updated:** 2025-09-14  
**Owner:** Logistics Sector Lead

## Use Case Definition

**Actors:**
- Primary: Terminal Operations Manager, Equipment Maintenance Team
- Supporting: Vessel Operations, Yard Planning, Safety Department

**Trip Type:** EQUIPMENT_SHUTTLE_RUN

**ODD (Operational Design Domain):**
- Geographic: Container terminal areas including maintenance facilities, storage yards, and berth areas
- Environmental: Maritime conditions, temperature range -10°C to 50°C, salt exposure
- Time: 24/7 operations aligned with vessel schedules and maintenance windows
- Communications: Terminal WiFi/LTE with local mesh fallback
- Other: Operation around oversized equipment and dynamic terminal traffic

**Trigger:**
Equipment maintenance schedule, vessel operations requirements, or equipment redeployment needs

**Nominal Flow:**
1. Terminal operations or maintenance system generates equipment movement request
2. System evaluates optimal routing and timing based on terminal congestion and priorities
3. Autonomous shuttle is dispatched to equipment location with precise arrival timing
4. System coordinates with equipment operators or maintenance team for handover
5. Specialized attachment mechanisms secure the equipment for safe transport
6. Shuttle navigates through terminal with equipment-specific routing considerations
7. At destination, system coordinates precise positioning for equipment deployment
8. Release procedures are executed with verification of stable placement
9. System updates equipment location in TOS and maintenance systems
10. Shuttle returns to staging area or proceeds to next assignment

**Variants / Edge Cases:**
- Emergency equipment redeployment: Priority routing and terminal-wide coordination
- Oversized or non-standard equipment: Special routing and attachment procedures
- Maintenance facility congestion: Dynamic scheduling and staging areas
- Weather impact on operations: Modified procedures for wind/rain conditions
- Multiple equipment coordination: Sequenced movements and yard optimization

## Performance & Integration

**KPIs:**
- **P0 (Must have):**
  - Equipment redeployment time: -40% vs. manual operations
  - Transport accuracy: 100% correct equipment to location
  - Terminal disruption: Zero impact on vessel operations
  - Safety incidents: 0
- **P1 (Should have):**
  - Fuel/energy efficiency: +20% vs. conventional transport
  - Equipment availability improvement: +15% through faster maintenance cycles
  - Location accuracy: ≤10cm final positioning
  - TOS synchronization: 100% accuracy

**Dependencies:**
- **Services:**
  - `dispatch`: Equipment movement planning and scheduling
  - `routing`: Terminal-aware navigation with congestion avoidance
  - `fleet-health`: Vehicle monitoring in maritime environment
  - `adapters/tos`: Terminal Operating System integration
  - `adapters/maintenance`: Maintenance management system integration
- **Rules:**
  - `rules/odd/logistics/terminal_operations.yaml`: Terminal operation parameters
  - `rules/policy/logistics/equipment_transport.yaml`: Equipment handling procedures
  - `rules/policy/logistics/vessel_priority.yaml`: Vessel operation priority rules
  - `rules/policy/logistics/maintenance_coordination.yaml`: Maintenance workflow integration
- **External Systems:**
  - Terminal Operating System: Equipment tracking and yard planning
  - Maintenance Management System: Work orders and equipment status
  - Vessel Planning System: Berth schedules and equipment requirements

**Risks & Mitigations:**
- **Equipment securing failure:**
  - Impact: Critical
  - Mitigation: Multi-point verification, sensor monitoring, operator confirmation, fail-safe mechanisms
- **Terminal congestion impact:**
  - Impact: High
  - Mitigation: Real-time congestion monitoring, dynamic routing, coordination with TOS
- **TOS synchronization errors:**
  - Impact: High
  - Mitigation: Robust API integration, confirmation protocols, reconciliation processes
- **Maritime environment challenges:**
  - Impact: Medium
  - Mitigation: Ruggedized components, corrosion protection, environmental monitoring

## Validation & Deployment

**Acceptance / Test Hooks:**
- **Simulation Scenarios:**
  - `logistics/terminal/equipment_transport.json`: Various equipment types and conditions
  - `logistics/terminal/congestion_navigation.json`: Peak terminal traffic handling
  - `logistics/terminal/maintenance_coordination.json`: Integration with maintenance workflows
- **Logs/Telemetry:**
  - Transport metrics: movement time, route efficiency, positioning accuracy
  - Equipment metrics: securing verification, stability monitoring, handover confirmation
  - Integration metrics: TOS synchronization, maintenance system updates, location accuracy
- **Gates:**
  - Equipment transport safety validation with zero incidents
  - TOS integration verification with 100% location accuracy
  - Terminal efficiency improvement with measurable KPI deltas

**Rollout Plan:**
- **Phase 1:** Single equipment type in limited terminal area
- **Phase 2:** Multiple equipment types with maintenance facility integration
- **Phase 3:** Full terminal operations with vessel coordination

## Additional Information

**Related Use Cases:**
- L2: Container Terminal Berth Shuttle
- L4: Middle-Mile Hub-to-Hub Corridor
- L7: Terminal Yard Optimization

**References:**
- Terminal Equipment Operations Manual
- Port Maintenance Best Practices
- Terminal Traffic Management Guidelines

**Notes:**
This use case represents a specialized logistics operation within maritime terminals, requiring coordination between various terminal systems and precise handling of valuable equipment. Success here demonstrates the system's ability to integrate with terminal operations while improving equipment utilization and maintenance efficiency.

## Cross-Functional Considerations

**Product Management:**
- Clear definition of terminal efficiency metrics
- Measurable impact on equipment availability and utilization
- Integration with existing terminal operations workflows

**Design:**
- Intuitive interfaces for terminal operators and maintenance teams
- Clear status visualization for equipment movements
- Accessibility for maritime terminal conditions

**Engineering:**
- Robust equipment securing mechanisms with verification
- Precise positioning systems for equipment placement
- Reliable operation in maritime environments

**Data:**
- Comprehensive equipment tracking and history
- Terminal congestion analysis and prediction
- Integration with TOS and maintenance data systems

**QA:**
- Validation of equipment transport safety across varied conditions
- Testing of terminal navigation with congestion scenarios
- Verification of TOS integration accuracy

**Security:**
- Protection of terminal operations data
- Secure communications with TOS and maintenance systems
- Physical security measures for equipment transport

**Operations:**
- Clear procedures for equipment handover and securing
- Training for terminal staff on autonomous shuttle interaction
- Exception handling protocols for non-standard situations

**Safety:**
- Equipment securing verification protocols
- Terminal traffic safety rules
- Emergency procedures for equipment-related incidents

**Maritime Compliance:**
- Adherence to port authority regulations
- Documentation of equipment movements
- Compliance with maritime safety standards
