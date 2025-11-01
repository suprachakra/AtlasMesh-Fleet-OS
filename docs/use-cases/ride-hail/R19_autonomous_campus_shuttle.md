# R19 — Autonomous Campus Shuttle

## Basic Information

**ID:** R19  
**Name:** Autonomous Campus Shuttle  
**Sector:** Ride-hail  
**Status:** Approved  
**Version:** 1.0  
**Last Updated:** 2025-09-14  
**Owner:** Ride-hail Sector Lead

## Use Case Definition

**Actors:**
- Primary: Campus Users, Facility Management
- Supporting: Security Operations, Event Coordinators, Accessibility Services

**Trip Type:** CAMPUS_SHUTTLE_RUN

**ODD (Operational Design Domain):**
- Geographic: Corporate campuses, university grounds, medical complexes, large facilities
- Environmental: All weather conditions with campus-specific adaptations, temperature range -20°C to 45°C
- Time: Extended operating hours aligned with campus activities
- Communications: Campus network with comprehensive coverage
- Other: Operation with specialized campus navigation and access management systems

**Trigger:**
Scheduled route service, on-demand shuttle request, or special event support

**Nominal Flow:**
1. System receives campus shuttle mission with service details and requirements
2. Service mode determination (fixed route, on-demand, or event support)
3. Vehicle is configured for appropriate capacity and campus-specific branding
4. Route planning incorporates campus layout and access restrictions
5. Navigation along designated route with precise stop positioning
6. Campus user verification with appropriate access credentials
7. Efficient boarding with capacity management and accessibility support
8. Campus-aware journey with appropriate speed and pathway protocols
9. Stop announcements with campus information and connection details
10. Service documentation with ridership metrics and campus utilization insights

**Variants / Edge Cases:**
- Special events: Increased capacity planning and temporary route adjustments
- Campus restrictions: Authorized area management and access control
- After-hours service: Security protocols and limited stop operations
- Accessibility needs: Specialized boarding assistance and route modifications
- Weather impacts: Campus-specific adaptations and covered route prioritization

## Performance & Integration

**KPIs:**
- **P0 (Must have):**
  - Schedule adherence: ≥98% on-time performance for fixed routes
  - User satisfaction: ≥4.5/5 rating for shuttle experience
  - Campus coverage: 100% of designated service areas
  - Safety record: Zero incidents within campus environment
- **P1 (Should have):**
  - Capacity optimization: Efficient ridership management
  - Wait time: ≤5 minutes for on-demand service
  - Energy efficiency: Optimized consumption for campus operations
  - Integration effectiveness: Seamless coordination with campus systems

**Dependencies:**
- **Services:**
  - `campus-shuttle-service`: Route management and scheduling
  - `routing`: Campus-aware path planning
  - `access-management`: Credential verification and authorization
  - `policy-engine`: Campus protocols and operation rules
  - `information-delivery`: Campus announcements and wayfinding
- **Rules:**
  - `rules/odd/ride-hail/campus_operations.yaml`: Shuttle operation parameters
  - `rules/policy/ride-hail/fixed_routes.yaml`: Route service protocols
  - `rules/policy/ride-hail/campus_access.yaml`: Authorization requirements
  - `rules/policy/safety/pedestrian_areas.yaml`: Shared space operations
- **External Systems:**
  - Campus Management System: Facility information and access control
  - Event Management System: Special event coordination
  - Security System: Access verification and monitoring

**Risks & Mitigations:**
- **Pedestrian interactions:**
  - Impact: High
  - Mitigation: Low speed operation, enhanced pedestrian detection, clear pathways, audible alerts
- **Campus access restrictions:**
  - Impact: Medium
  - Mitigation: Real-time access data, authorization verification, alternative routing, security coordination
- **Capacity constraints:**
  - Impact: Medium
  - Mitigation: Dynamic vehicle allocation, capacity forecasting, peak planning, supplemental service
- **Weather challenges:**
  - Impact: Medium
  - Mitigation: Campus-specific adaptations, covered route options, all-weather capability, service adjustments

## Validation & Deployment

**Acceptance / Test Hooks:**
- **Simulation Scenarios:**
  - `ride-hail/campus/fixed_route.json`: Scheduled service operations
  - `ride-hail/campus/on_demand.json`: Request-based service
  - `ride-hail/campus/special_event.json`: High-capacity operations
- **Logs/Telemetry:**
  - Service metrics: schedule adherence, ridership volumes, route efficiency
  - User metrics: wait times, journey satisfaction, accessibility utilization
  - Operational metrics: energy consumption, vehicle utilization, campus integration
- **Gates:**
  - Campus management acceptance of service quality
  - User experience validation through satisfaction surveys
  - Security verification of access control integration

**Rollout Plan:**
- **Phase 1:** Basic fixed-route service on primary campus pathways
- **Phase 2:** Enhanced capability with on-demand service and event support
- **Phase 3:** Full campus integration with comprehensive service options

## Additional Information

**Related Use Cases:**
- R3: Scheduled Commuter Service
- R5: High-Volume Event Transport
- R13: Autonomous Passenger Assistance

**References:**
- Campus Mobility Guidelines
- Shuttle Service Standards
- Pedestrian Zone Operations Protocols

**Notes:**
This use case addresses the specialized transportation needs within campus environments, which require careful integration with existing facilities and activities. Success here demonstrates the system's ability to provide efficient mobility services while respecting campus protocols and enhancing the user experience. The autonomous approach enables consistent service quality with optimal routing while adapting to the dynamic nature of campus operations and events.

## Cross-Functional Considerations

**Product Management:**
- Clear definition of campus service requirements
- Measurable impact on campus mobility
- Integration with existing facility systems

**Design:**
- Campus-appropriate vehicle styling
- Clear route and stop information
- Accessibility for all campus users

**Engineering:**
- Pedestrian-safe operation systems
- Campus pathway navigation
- Access control integration

**Data:**
- Ridership pattern analytics
- Campus utilization insights
- Service optimization models

**QA:**
- Pedestrian interaction testing
- Campus pathway validation
- Access control verification

**Security:**
- Campus credential integration
- Authorized area management
- After-hours operation protocols

**Operations:**
- Clear procedures for campus coordination
- Training for campus-specific requirements
- Performance monitoring protocols

**Facility Management:**
- Campus route integration
- Stop location optimization
- Infrastructure coordination

**Accessibility Services:**
- Boarding assistance protocols
- Route accessibility verification
- Special needs accommodation
