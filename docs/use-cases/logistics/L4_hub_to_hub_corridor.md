# L4 — Middle-Mile Hub-to-Hub (Private Corridor)

## Basic Information

**ID:** L4  
**Name:** Middle-Mile Hub-to-Hub (Private Corridor)  
**Sector:** Logistics  
**Status:** Approved  
**Version:** 1.0  
**Last Updated:** 2025-09-13  
**Owner:** Logistics Sector Lead

## Use Case Definition

**Actors:**
- Primary: Logistics Network Manager, Hub Operations Teams
- Supporting: Fleet Maintenance, Regulatory Compliance, Security Teams

**Trip Type:** CORRIDOR_RUN

**ODD (Operational Design Domain):**
- Geographic: Dedicated or managed lanes between logistics hubs, minimal public road interaction
- Environmental: All weather conditions with appropriate adjustments, temperature range -10°C to 55°C
- Time: 24/7 operations with time-of-day specific protocols (day/night/peak)
- Communications: Corridor-wide connectivity with redundant coverage
- Other: Drop-and-hook operations at origin and destination

**Trigger:**
Scheduled departures or dynamic demand-based dispatch

**Nominal Flow:**
1. Origin hub prepares trailer/container with validated documentation
2. System schedules optimal departure based on destination capacity and network flow
3. Autonomous tractor is dispatched for trailer pickup with precise yard navigation
4. Tractor performs automated coupling with multi-factor verification
5. Vehicle conducts pre-trip inspection with sensor-based validation
6. Corridor entry protocol with access control and regulatory compliance checks
7. Autonomous transit with continuous monitoring and predictive ETA updates
8. Destination approach protocol with arrival notification and dock assignment
9. Precise positioning at designated drop location with automated uncoupling
10. System updates load status and prepares vehicle for next assignment

**Variants / Edge Cases:**
- Weather degradation: Adaptive speed profiles and following distances
- Corridor maintenance: Dynamic lane management and traffic control
- Vehicle health alerts: Predictive intervention and service routing
- Security incidents: Escalation and response protocols
- Hub congestion: Dynamic scheduling and queuing management

## Performance & Integration

**KPIs:**
- **P0 (Must have):**
  - On-time performance: ≥ 95% within schedule window
  - Empty miles: ≤ 3% of total distance
  - Cost per mile: -15% vs. human-driven operation
  - Safety incidents: 0
- **P1 (Should have):**
  - Fuel/energy efficiency: +20% vs. human-driven operation
  - Asset utilization: ≥ 22 hours/day average
  - Hub dwell time: ≤ 30 minutes per stop
  - ETA accuracy: ≥ 95% within 10 minutes

**Dependencies:**
- **Services:**
  - `dispatch`: Network flow optimization and scheduling
  - `routing`: Corridor-specific navigation with traffic awareness
  - `policy-engine`: Regulatory compliance and operational rules
  - `energy-manager`: Optimal energy management and refueling/recharging
  - `fleet-health`: Predictive maintenance and health monitoring
- **Rules:**
  - `rules/odd/logistics/corridor_rules.yaml`: Corridor-specific operational parameters
  - `rules/policy/logistics/drop_hook.yaml`: Coupling/uncoupling procedures
  - `rules/policy/logistics/hub_interaction.yaml`: Hub entry/exit protocols
  - `rules/policy/security/cargo_integrity.yaml`: Load security and verification
- **External Systems:**
  - Transportation Management System: Load planning and scheduling
  - Yard Management System: Dock assignments and trailer tracking
  - Electronic Logging Device (ELD): Regulatory compliance and hours of service

**Risks & Mitigations:**
- **Public road interface points:**
  - Impact: High
  - Mitigation: Controlled access points, traffic management systems, reduced speed zones
- **Hub congestion impact on network:**
  - Impact: Medium
  - Mitigation: Real-time capacity monitoring, dynamic scheduling, overflow areas
- **Weather impact on corridor availability:**
  - Impact: Medium
  - Mitigation: Weather-specific protocols, alternative routing, predictive scheduling
- **System-wide disruption (power, comms):**
  - Impact: High
  - Mitigation: Degraded mode operations, local decision making, safe harbor locations

## Validation & Deployment

**Acceptance / Test Hooks:**
- **Simulation Scenarios:**
  - `logistics/corridor/network_flow_optimization.json`: Multi-hub network simulation
  - `logistics/corridor/weather_degradation.json`: Operations in adverse conditions
  - `logistics/corridor/hub_congestion.json`: Peak demand handling
- **Logs/Telemetry:**
  - Network metrics: on-time performance, asset utilization, empty miles
  - Efficiency metrics: fuel/energy consumption, dwell times, transit times
  - Safety metrics: following distances, speed compliance, intervention events
- **Gates:**
  - Network simulation validation with historical demand patterns
  - Regulatory compliance verification with zero violations
  - Field validation with incremental corridor segments

**Rollout Plan:**
- **Phase 1:** Single corridor segment during limited hours
- **Phase 2:** Extended hours with multiple corridor segments
- **Phase 3:** Full network integration with dynamic scheduling

## Additional Information

**Related Use Cases:**
- L1: Yard Switcher: Dock ↔ Yard Slot Shuttling
- L9: Empty Container Repositioning
- L12: Port ↔ ICD Shuttle

**References:**
- Middle-Mile Logistics Optimization Guide
- Autonomous Corridor Design Standards
- Drop-and-Hook Operations Manual

**Notes:**
This use case represents a high-efficiency application of autonomous technology in controlled environments with significant economic impact. Success here demonstrates the system's ability to operate reliably over extended distances while integrating seamlessly with hub operations at both ends.

## Cross-Functional Considerations

**Product Management:**
- Clear definition of corridor operational parameters
- Measurable economic impact metrics
- Integration with existing logistics workflows and systems

**Design:**
- Intuitive visualization of network flow and asset status
- Clear communication of regulatory compliance status
- Accessibility for diverse operator environments (mobile, desktop, control center)

**Engineering:**
- Robust corridor-wide communications infrastructure
- Seamless handoffs between autonomous and manual operations
- Redundant safety systems for extended-distance operations

**Data:**
- Comprehensive trip logging for regulatory compliance
- Network optimization algorithms with continuous improvement
- Long-term storage of operational data for pattern analysis

**QA:**
- Validation of performance across diverse weather conditions
- Testing of hub interaction procedures and exception handling
- Verification of regulatory compliance in all operational modes

**Security:**
- Protection of cargo manifests and routing information
- Access controls for corridor entry and exit points
- Physical security measures for autonomous vehicles and cargo

**Operations:**
- Clear procedures for handling exceptions and disruptions
- Training for hub personnel on autonomous vehicle interaction
- Maintenance protocols for corridor infrastructure and vehicles

**Regulatory:**
- Compliance with hours of service and electronic logging requirements
- Documentation of safety case for regulatory approval
- Regular auditing and reporting procedures

**Financial:**
- Clear cost modeling for ROI calculations
- Asset utilization tracking and optimization
- Maintenance cost tracking and predictive budgeting
