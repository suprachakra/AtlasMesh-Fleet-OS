# R6 — Scheduled Commuter Service

## Basic Information

**ID:** R6  
**Name:** Scheduled Commuter Service  
**Sector:** Ride-hail  
**Status:** Approved  
**Version:** 1.0  
**Last Updated:** 2025-09-14  
**Owner:** Ride-hail Sector Lead

## Use Case Definition

**Actors:**
- Primary: Commuters, Fleet Operations Manager
- Supporting: Corporate Mobility Partners, Transit Agencies, City Authorities

**Trip Type:** SCHEDULED_RUN

**ODD (Operational Design Domain):**
- Geographic: Fixed corridors between residential areas and business districts
- Environmental: Urban/suburban conditions, standard weather thresholds
- Time: Peak commute hours (typically 06:00-10:00 and 16:00-20:00)
- Communications: Urban LTE/5G with high reliability requirements
- Other: Integration with transit schedules and corporate mobility programs

**Trigger:**
Scheduled service times or subscription-based recurring bookings

**Nominal Flow:**
1. System generates optimized commuter routes based on subscription data and demand patterns
2. Vehicles are pre-positioned at strategic pickup points before service windows
3. Riders receive notifications with precise pickup timing and vehicle information
4. Vehicles follow predetermined routes with minimal deviation, making scheduled stops
5. Riders are authenticated via app or corporate ID with streamlined boarding
6. System monitors passenger load and adjusts subsequent vehicle spacing if needed
7. Real-time ETA updates are provided to waiting riders at downstream stops
8. At destination, riders disembark at designated drop points optimized for business districts
9. System captures commute metrics and satisfaction data for service optimization
10. Vehicles are repositioned for reverse commute or other service types based on demand

**Variants / Edge Cases:**
- Weather delays: Dynamic schedule adjustment with proactive notifications
- Capacity constraints: Overflow vehicle dispatch with priority routing
- Transit connection timing: Adaptive scheduling to maintain transit transfers
- Corporate event impact: Special service scaling for large workplace events
- Traffic incidents: Route deviation with minimal impact on schedule

## Performance & Integration

**KPIs:**
- **P0 (Must have):**
  - Schedule adherence: ≥92% on-time arrivals (±3 minutes)
  - Rider satisfaction: CSAT ≥4.7/5.0 for commuter service
  - Capacity utilization: ≥85% average seat fill
  - Corporate program retention: ≥95% quarterly renewal
- **P1 (Should have):**
  - Energy efficiency: +25% vs. on-demand service
  - Transit connection success: ≥98% of planned transfers
  - Rider acquisition cost: -40% vs. standard marketing
  - Peak demand coverage: ≥99% of subscription requests fulfilled

**Dependencies:**
- **Services:**
  - `dispatch`: Schedule optimization and vehicle assignment
  - `routing`: Fixed-route management with minimal deviation
  - `rider-ux`: Subscription and scheduled ride experience
  - `analytics`: Commute pattern analysis and optimization
  - `policy-engine`: Corporate program rules and pricing
- **Rules:**
  - `rules/odd/ride-hail/scheduled_service.yaml`: Scheduled service parameters
  - `rules/policy/ride-hail/subscription.yaml`: Subscription and recurring ride rules
  - `rules/policy/ride-hail/corporate_programs.yaml`: Corporate mobility program integration
  - `rules/policy/ride-hail/transit_connection.yaml`: Transit synchronization policies
- **External Systems:**
  - Corporate Mobility Platforms: Employee subscriptions and billing
  - Transit Information Systems: Schedule data and real-time updates
  - Traffic Management Systems: Congestion and incident data

**Risks & Mitigations:**
- **Schedule disruption:**
  - Impact: High
  - Mitigation: Buffer time in schedules, dynamic adjustment, proactive rider communication
- **Demand pattern shifts:**
  - Impact: Medium
  - Mitigation: Regular pattern analysis, subscription flexibility, capacity adjustment
- **Transit schedule changes:**
  - Impact: Medium
  - Mitigation: Real-time transit data integration, adaptive scheduling, buffer times
- **Corporate program changes:**
  - Impact: High
  - Mitigation: Program versioning, transition periods, account management communication

## Validation & Deployment

**Acceptance / Test Hooks:**
- **Simulation Scenarios:**
  - `ride-hail/commuter/schedule_adherence.json`: Various traffic and demand conditions
  - `ride-hail/commuter/capacity_optimization.json`: Load balancing and utilization
  - `ride-hail/commuter/transit_connection.json`: Transfer timing and reliability
- **Logs/Telemetry:**
  - Schedule metrics: on-time performance, deviation patterns, recovery effectiveness
  - Rider metrics: wait time, transit connections made, subscription utilization
  - Service metrics: seat utilization, energy efficiency, cost per rider-mile
- **Gates:**
  - Schedule adherence validation with real-world operations
  - Rider satisfaction verification through targeted surveys
  - Corporate program ROI demonstration with usage analytics

**Rollout Plan:**
- **Phase 1:** Single corridor with limited subscription base
- **Phase 2:** Multiple corridors with corporate program integration
- **Phase 3:** Full network with transit connection guarantees

## Additional Information

**Related Use Cases:**
- R1: Standard Point-to-Point Ride
- R2: Shared Ride / Pool
- R7: Special Event Transport

**References:**
- Commuter Service Optimization Guide
- Corporate Mobility Program Standards
- Transit Integration Best Practices

**Notes:**
This use case represents a scheduled service model that complements on-demand operations, providing predictable capacity utilization and recurring revenue streams. Success here demonstrates the system's ability to operate reliable scheduled services while maintaining flexibility for changing conditions and integration with other mobility systems.

## Cross-Functional Considerations

**Product Management:**
- Clear definition of service reliability metrics
- Measurable impact on rider acquisition and retention
- Integration with corporate mobility programs and transit

**Design:**
- Intuitive subscription management for riders
- Clear schedule visualization and updates
- Accessibility for commuters with varied needs

**Engineering:**
- Reliable schedule optimization algorithms
- Precise vehicle positioning for pickup windows
- Robust integration with transit data feeds

**Data:**
- Comprehensive commute pattern analysis
- Schedule adherence tracking and optimization
- Corporate program utilization analytics

**QA:**
- Validation of schedule reliability across varied conditions
- Testing of rider communication during disruptions
- Verification of capacity management and overflow handling

**Security:**
- Protection of corporate program and subscription data
- Secure rider authentication methods
- Privacy controls for commute pattern information

**Operations:**
- Clear procedures for schedule disruption management
- Training for operations staff on fixed-route service
- Exception handling protocols for capacity constraints

**Business Development:**
- Corporate program structure and pricing
- Transit agency partnerships
- Service level agreements and performance reporting

**City Relations:**
- Coordination with transit agencies
- Integration with mobility hubs
- Data sharing for transportation planning
