# R17 — Autonomous Carpooling Service

## Basic Information

**ID:** R17  
**Name:** Autonomous Carpooling Service  
**Sector:** Ride-hail  
**Status:** Approved  
**Version:** 1.0  
**Last Updated:** 2025-09-14  
**Owner:** Ride-hail Sector Lead

## Use Case Definition

**Actors:**
- Primary: Commuters, Carpooling Groups, Fleet Operations
- Supporting: Employers, Transportation Authorities, Community Organizations

**Trip Type:** CARPOOLING_RUN

**ODD (Operational Design Domain):**
- Geographic: Residential areas, business districts, commuter corridors
- Environmental: All weather conditions, temperature range -20°C to 45°C
- Time: Primarily commuting hours with scheduled service windows
- Communications: Urban and suburban network with reliable coverage
- Other: Operation with specialized group coordination and passenger management systems

**Trigger:**
Scheduled carpooling service, recurring commute pattern, or employer-sponsored program

**Nominal Flow:**
1. System receives carpooling mission with group details and schedule requirements
2. Route optimization with pickup sequence and timing coordination
3. Vehicle is configured for appropriate capacity and passenger comfort
4. Route planning incorporates efficient pickup sequence and traffic conditions
5. Navigation to initial pickup with precise timing and positioning
6. Passenger verification with group membership confirmation
7. Multi-stop pickup with optimized sequencing and wait time management
8. Group-aware journey with appropriate social dynamics and comfort settings
9. Destination arrival with organized dropoff sequence
10. Service documentation with commute metrics and sustainability impact

**Variants / Edge Cases:**
- Passenger no-shows: Wait time protocols and dynamic replanning
- Route disruptions: Alternative path selection and ETA updates
- Group composition changes: Flexible seating arrangements and pickup adjustments
- Schedule variations: Regular vs. occasional commuter accommodation
- Special needs: Accessibility requirements and group-specific accommodations

## Performance & Integration

**KPIs:**
- **P0 (Must have):**
  - Schedule adherence: ≥95% on-time performance for regular commutes
  - Group satisfaction: ≥4.5/5 rating for carpooling experience
  - Occupancy rate: ≥3.5 passengers per vehicle average
  - Cost efficiency: ≥40% savings vs. individual transport
- **P1 (Should have):**
  - Sustainability impact: Measurable reduction in emissions and congestion
  - Group stability: Consistent participation and retention
  - Route optimization: Minimal detour time for individual passengers
  - Integration effectiveness: Seamless coordination with employer programs

**Dependencies:**
- **Services:**
  - `carpooling-service`: Group coordination and scheduling
  - `routing`: Multi-pickup path optimization
  - `passenger-matching`: Group formation and compatibility
  - `policy-engine`: Carpooling protocols and program rules
  - `sustainability-tracking`: Environmental impact measurement
- **Rules:**
  - `rules/odd/ride-hail/carpooling.yaml`: Service operation parameters
  - `rules/policy/ride-hail/group_transport.yaml`: Multi-passenger protocols
  - `rules/policy/ride-hail/commuter_programs.yaml`: Employer integration
  - `rules/policy/mobility/shared_transport.yaml`: Incentive structures
- **External Systems:**
  - Employer Commuter Programs: Participation and subsidies
  - Transportation Authority Systems: HOV lane access and incentives
  - Community Platforms: Group formation and social coordination

**Risks & Mitigations:**
- **Schedule coordination challenges:**
  - Impact: High
  - Mitigation: Flexible timing windows, priority passenger designation, wait time policies, advance notification
- **Group dynamics issues:**
  - Impact: Medium
  - Mitigation: Compatibility matching, group guidelines, feedback systems, conflict resolution protocols
- **Varying commute patterns:**
  - Impact: Medium
  - Mitigation: Adaptive scheduling, occasional rider accommodation, alternative group matching, flexible routing
- **Program sustainability:**
  - Impact: High
  - Mitigation: Employer subsidies, public incentives, cost optimization, demonstrated value metrics

## Validation & Deployment

**Acceptance / Test Hooks:**
- **Simulation Scenarios:**
  - `ride-hail/carpooling/standard_commute.json`: Regular group commuting
  - `ride-hail/carpooling/variable_schedule.json`: Flexible timing management
  - `ride-hail/carpooling/group_formation.json`: Compatibility and matching
- **Logs/Telemetry:**
  - Service metrics: schedule adherence, occupancy rates, route efficiency
  - Passenger metrics: wait times, journey satisfaction, group stability
  - Sustainability metrics: emissions reduction, congestion impact, cost efficiency
- **Gates:**
  - Commuter group acceptance of service quality
  - Employer validation of program effectiveness
  - Transportation authority verification of congestion impact

**Rollout Plan:**
- **Phase 1:** Basic carpooling with established commuter groups
- **Phase 2:** Enhanced capability with dynamic group formation and flexible scheduling
- **Phase 3:** Full integration with employer programs and transportation incentives

## Additional Information

**Related Use Cases:**
- R3: Scheduled Commuter Service
- R4: Dynamic Pooled Rides
- R14: Autonomous Multi-Modal Connection

**References:**
- Carpooling Program Guidelines
- Commuter Benefit Regulations
- Group Transportation Best Practices

**Notes:**
This use case addresses the sustainable transportation need for efficient commuter carpooling, which provides significant environmental and cost benefits. Success here demonstrates the system's ability to effectively coordinate group transportation while maintaining passenger satisfaction and schedule reliability. The autonomous approach enables consistent service quality with optimized routing while providing the flexibility needed to accommodate varying commute patterns and group dynamics.

## Cross-Functional Considerations

**Product Management:**
- Clear definition of carpooling service requirements
- Measurable impact on commuter satisfaction
- Integration with existing mobility programs

**Design:**
- Intuitive group coordination interfaces
- Clear schedule and timing information
- Accessibility for diverse commuters

**Engineering:**
- Multi-stop route optimization
- Group management systems
- Schedule coordination algorithms

**Data:**
- Commute pattern analytics
- Group compatibility modeling
- Sustainability impact measurement

**QA:**
- Schedule reliability testing
- Group experience validation
- System integration verification

**Security:**
- Passenger verification protocols
- Group membership management
- Personal information protection

**Operations:**
- Clear procedures for group coordination
- Training for commuter management
- Performance monitoring protocols

**Partnerships:**
- Employer program integration
- Transportation authority coordination
- Community organization engagement

**Sustainability:**
- Environmental impact measurement
- Congestion reduction tracking
- Efficiency optimization strategies
