# M3 — Autonomous Refuel/Recharge Sweeper

## Basic Information

**ID:** M3  
**Name:** Autonomous Refuel/Recharge Sweeper  
**Sector:** Mining  
**Status:** Approved  
**Version:** 1.0  
**Last Updated:** 2025-09-13  
**Owner:** Mining Sector Lead

## Use Case Definition

**Actors:**
- Primary: Fleet Maintenance Manager, Energy Management Team
- Supporting: Equipment Operators, Production Supervisors

**Trip Type:** SERVICE_RUN

**ODD (Operational Design Domain):**
- Geographic: Active mining areas, haul roads, service areas
- Environmental: All weather conditions with appropriate adjustments, temperature range -10°C to 55°C
- Time: 24/7 operations with priority during production peaks
- Communications: Mine LTE/WiFi with local fallback
- Other: Hazardous materials handling protocols for fuel operations

**Trigger:**
Equipment energy level thresholds or predictive service scheduling

**Nominal Flow:**
1. Fleet health system identifies vehicles approaching refuel/recharge thresholds
2. Energy manager optimizes service sequence to minimize production impact
3. Service vehicle is dispatched with optimized route to target equipment
4. Target equipment is notified of upcoming service and positioning requirements
5. Service vehicle navigates to equipment location with hazmat safety protocols
6. Precise positioning for refueling/recharging connection with vision guidance
7. Automated refueling/recharging process with continuous monitoring
8. Service completion verification and documentation
9. System updates fleet health records and schedules next service interval
10. Service vehicle proceeds to next equipment in optimized sequence

**Variants / Edge Cases:**
- Emergency low fuel/charge: Priority service protocol
- Service vehicle capacity constraints: Optimization for multiple service runs
- Equipment relocation during service window: Dynamic rerouting
- Hazardous conditions detection: Safety protocol activation
- Connection failures: Retry procedures and escalation

## Performance & Integration

**KPIs:**
- **P0 (Must have):**
  - Production minutes saved: +15-25% vs. fixed refueling
  - Service completion rate: ≥ 98% of scheduled services
  - Equipment downtime for refueling: -40% vs. baseline
  - Safety incidents: 0
- **P1 (Should have):**
  - Fuel/energy efficiency: +10% through optimized scheduling
  - Service vehicle utilization: ≥ 85%
  - Queue time at service locations: -50% vs. fixed locations
  - Predictive accuracy: ≥ 90% for service timing needs

**Dependencies:**
- **Services:**
  - `energy-manager`: Consumption monitoring and service scheduling
  - `fleet-health`: Equipment status and maintenance integration
  - `routing`: Service-optimized path planning
  - `predictive-maint`: Service window forecasting
  - `dispatch`: Service vehicle assignment
- **Rules:**
  - `rules/odd/mining/service_vehicle_rules.yaml`: Service vehicle operation parameters
  - `rules/policy/mining/refueling_safety.yaml`: Hazardous materials handling
  - `rules/policy/mining/service_priority.yaml`: Equipment service prioritization
- **External Systems:**
  - Fleet Management System: Equipment tracking and status
  - Maintenance Management System: Service records and scheduling
  - Fuel/Energy Management System: Consumption tracking and inventory

**Risks & Mitigations:**
- **Fuel spill/hazardous material incident:**
  - Impact: High
  - Mitigation: Automated safety interlocks, pressure monitoring, spill containment systems
- **Production impact from poorly timed service:**
  - Impact: Medium
  - Mitigation: Production-aware scheduling, just-in-time service, coordination with dispatch
- **Connection failures:**
  - Impact: Medium
  - Mitigation: Multiple connection attempts, manual override capability, backup service options
- **Fire risk during refueling:**
  - Impact: High
  - Mitigation: Continuous monitoring, automatic shutoff, fire suppression systems

## Validation & Deployment

**Acceptance / Test Hooks:**
- **Simulation Scenarios:**
  - `mining/service/refuel_sequence_optimization.json`: Multi-equipment service planning
  - `mining/service/connection_procedures.json`: Docking and connection reliability
  - `mining/service/emergency_low_fuel.json`: Priority service response
- **Logs/Telemetry:**
  - Service metrics: completion times, success rates, connection attempts
  - Optimization metrics: production impact, equipment utilization, service vehicle efficiency
  - Safety metrics: pressure readings, connection integrity, environmental monitoring
- **Gates:**
  - Safety protocol validation with 100% success rate
  - Production impact reduction ≥ 15% in simulation
  - Field validation with incremental service scope

**Rollout Plan:**
- **Phase 1:** Single service vehicle with limited equipment types
- **Phase 2:** Multiple service vehicles with coordinated scheduling
- **Phase 3:** Full fleet integration with predictive service optimization

## Additional Information

**Related Use Cases:**
- M1: Pit-to-Crusher Autonomous Haul
- M10: Autonomous Greaser/Lube Route
- M22: Storm Pre-Stage Assets

**References:**
- Mobile Refueling Safety Standards
- Energy Optimization in Mining Operations
- Hazardous Materials Handling Procedures

**Notes:**
This use case represents a critical support function that directly impacts production efficiency by minimizing equipment downtime for essential services. Success here demonstrates the system's ability to handle hazardous materials safely while optimizing complex scheduling constraints.
