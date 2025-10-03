# R3 — Airport Pickup / Drop with Staging

## Basic Information

**ID:** R3  
**Name:** Airport Pickup / Drop with Staging  
**Sector:** Ride-hail  
**Status:** Approved  
**Version:** 1.0  
**Last Updated:** 2025-09-13  
**Owner:** Ride-hail Sector Lead

## Use Case Definition

**Actors:**
- Primary: Riders, Autonomous Vehicle
- Supporting: Airport Operations, Law Enforcement, Fleet Operations

**Trip Type:** AIRPORT_RUN

**ODD (Operational Design Domain):**
- Geographic: Airport terminals, designated pickup/dropoff areas, staging lots
- Environmental: Standard weather thresholds with airport-specific adjustments
- Time: 24/7 operations aligned with flight schedules
- Communications: Airport WiFi/LTE with high reliability requirements
- Other: Strict compliance with airport authority regulations and procedures

**Trigger:**
Rider request from/to airport or predictive staging based on flight schedules

**Nominal Flow:**
1. System monitors flight arrival data and predicts demand patterns
2. Vehicles are pre-positioned in staging lots based on demand forecasts
3. For dropoffs, vehicle navigates to appropriate terminal with level-specific routing
4. Vehicle positions at designated dropoff point with precise curb management
5. For pickups, system assigns optimal vehicle from staging lot upon request
6. Rider receives detailed pickup instructions with terminal-specific guidance
7. Vehicle navigates from staging lot to pickup point with priority access
8. Precise positioning at designated pickup point with rider identification features
9. Vehicle manages luggage expectations and capacity constraints
10. System tracks airport-specific metrics and compliance with regulations

**Variants / Edge Cases:**
- Terminal congestion: Dynamic curb assignment and rider communication
- Flight delays: Staging adjustments and rider notification
- Law enforcement interaction: Protocol for credential verification
- Special access needs: Accessibility accommodations at specific terminals
- Peak demand surge: Queue management and expectation setting

## Performance & Integration

**KPIs:**
- **P0 (Must have):**
  - Curb dwell time: ≤ 3 minutes average (per airport regulations)
  - Rider wait time: P95 ≤ 10 minutes from request to pickup
  - Staging lot utilization: ≥ 85% during peak periods
  - Compliance incidents: 0
- **P1 (Should have):**
  - Pickup cancellations: ≤ 5%
  - Rider-vehicle match time: ≤ 45 seconds average
  - Prediction accuracy: ≥ 85% for demand forecasting
  - Rider satisfaction: CSAT ≥ 4.7/5.0 for airport trips

**Dependencies:**
- **Services:**
  - `dispatch`: Airport-specific vehicle assignment
  - `routing`: Terminal-specific navigation
  - `rebalancer`: Staging lot optimization
  - `v2x-service`: Airport infrastructure integration
  - `rider-ux`: Airport-specific rider communications
- **Rules:**
  - `rules/odd/ride-hail/airport_rules.yaml`: Airport-specific operational parameters
  - `rules/policy/ride-hail/curb_management.yaml`: Terminal curb policies
  - `rules/policy/ride-hail/staging_lot.yaml`: Staging lot behavior and exit sequencing
  - `rules/policy/ride-hail/law_enforcement.yaml`: Airport authority interaction protocols
- **External Systems:**
  - Flight Information System: Arrival/departure data
  - Airport Access Control: Vehicle credentials and access management
  - Terminal Management System: Curb allocation and congestion monitoring

**Risks & Mitigations:**
- **Terminal congestion:**
  - Impact: High
  - Mitigation: Real-time curb monitoring, dynamic allocation, alternative pickup points
- **Regulatory compliance:**
  - Impact: High
  - Mitigation: Airport-specific rule packs, credential management, audit logging
- **Rider-vehicle matching challenges:**
  - Impact: Medium
  - Mitigation: Enhanced identification features, terminal-specific instructions, location precision
- **Demand surge management:**
  - Impact: Medium
  - Mitigation: Predictive staging, dynamic pricing, queue transparency

## Validation & Deployment

**Acceptance / Test Hooks:**
- **Simulation Scenarios:**
  - `ride-hail/airport/terminal_operations.json`: Terminal-specific pickup/dropoff
  - `ride-hail/airport/staging_lot_management.json`: Staging optimization
  - `ride-hail/airport/peak_demand_handling.json`: Surge capacity management
- **Logs/Telemetry:**
  - Compliance metrics: dwell times, access credentials, regulatory adherence
  - Efficiency metrics: staging utilization, pickup times, match accuracy
  - Experience metrics: wait times, cancellations, rider feedback
- **Gates:**
  - Airport authority approval of operational plan
  - Compliance verification with zero violations in simulation
  - Field validation with incremental volume scaling

**Rollout Plan:**
- **Phase 1:** Limited operations at single terminal during off-peak hours
- **Phase 2:** Multi-terminal operations with dedicated staging areas
- **Phase 3:** Full airport integration with demand-based dynamic staging

## Additional Information

**Related Use Cases:**
- R1: Standard Point-to-Point Ride
- R4: Accessibility Ride (Wheelchair-Accessible)
- R20: Law-Enforcement Interaction

**References:**
- Airport Ground Transportation Regulations
- Curb Management Best Practices
- Passenger Pickup/Dropoff Optimization Guide

**Notes:**
This use case represents a highly regulated operation with strict compliance requirements and complex coordination challenges. Success here demonstrates the system's ability to operate in controlled environments with multiple stakeholders while maintaining high service levels during variable demand patterns.
