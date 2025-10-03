# R4 — Accessibility Ride (Wheelchair-Accessible)

## Basic Information

**ID:** R4  
**Name:** Accessibility Ride (Wheelchair-Accessible)  
**Sector:** Ride-hail  
**Status:** Approved  
**Version:** 1.0  
**Last Updated:** 2025-09-13  
**Owner:** Ride-hail Sector Lead

## Use Case Definition

**Actors:**
- Primary: Riders with Accessibility Needs, Autonomous WAV (Wheelchair Accessible Vehicle)
- Supporting: Accessibility Support Specialists, Fleet Operations, Regulatory Compliance

**Trip Type:** ACCESSIBILITY_RUN

**ODD (Operational Design Domain):**
- Geographic: Licensed districts with accessibility-verified pickup/dropoff points
- Environmental: Standard weather thresholds with additional accessibility considerations
- Time: 24/7 operations with priority during peak demand
- Communications: Urban LTE/5G with local fallback
- Other: Specialized vehicle configuration with accessibility features

**Trigger:**
Rider request for wheelchair-accessible vehicle or other accessibility accommodations

**Nominal Flow:**
1. Rider requests accessible vehicle with specific accommodation needs
2. System assigns appropriate WAV with verified equipment status
3. Vehicle navigates to pickup point with accessibility-optimized positioning
4. Vehicle deploys ramp/lift with sensor monitoring for safe operation
5. Rider boards with automated securement system verification
6. System confirms proper wheelchair securement with multi-point validation
7. Vehicle navigates to destination with specialized ride comfort parameters
8. At destination, system selects optimal dropoff point for accessibility
9. Vehicle deploys ramp/lift and monitors safe exit
10. System captures accessibility-specific feedback and service quality metrics

**Variants / Edge Cases:**
- Securement challenges: Additional verification steps and support escalation
- Inaccessible pickup/dropoff: Alternative location suggestions with minimal distance
- Equipment malfunction: Immediate support escalation and recovery procedures
- Companion riders: Accommodation of both wheelchair and standard seating
- Service animals: Detection and appropriate space allocation

## Performance & Integration

**KPIs:**
- **P0 (Must have):**
  - ADA/PRM fulfillment rate: ≥ 98% of requests
  - Securement error rate: 0
  - Wait time parity: ≤ 20% longer than standard rides
  - Rider satisfaction: CSAT ≥ 4.7/5.0
- **P1 (Should have):**
  - Accessibility vehicle availability: ≥ 95% of service hours
  - Securement time: ≤ 3 minutes average
  - Pickup point accessibility score: ≥ 90% optimal
  - Support escalation rate: ≤ 5% of rides

**Dependencies:**
- **Services:**
  - `dispatch`: Accessibility-aware vehicle assignment
  - `routing`: Accessibility-optimized navigation and pickup/dropoff selection
  - `rider-ux`: Specialized accessibility interface and communication
  - `fleet-health`: Accessibility equipment monitoring and validation
  - `policy-engine`: Accessibility compliance and service rules
- **Rules:**
  - `rules/odd/ride-hail/accessibility_rules.yaml`: Accessibility-specific operational parameters
  - `rules/policy/ride-hail/securement.yaml`: Wheelchair securement procedures and verification
  - `rules/policy/ride-hail/service_animals.yaml`: Service animal accommodation policies
  - `rules/policy/ride-hail/companion_riders.yaml`: Companion accommodation policies
- **External Systems:**
  - Accessibility Infrastructure Database: Curb cuts, accessible entrances, etc.
  - Regulatory Compliance System: ADA and local accessibility requirements
  - Accessibility Support System: Specialized support resources

**Risks & Mitigations:**
- **Securement failure:**
  - Impact: Critical
  - Mitigation: Multi-point verification system, visual confirmation, sensor monitoring, automatic braking profile adjustment
- **Inaccessible environment:**
  - Impact: High
  - Mitigation: Pre-validated pickup/dropoff points, alternative suggestions, real-time accessibility mapping
- **Equipment reliability:**
  - Impact: High
  - Mitigation: Redundant systems, pre-trip verification, regular maintenance certification
- **Rider communication challenges:**
  - Impact: Medium
  - Mitigation: Multiple communication channels, specialized training, clear visual indicators

## Validation & Deployment

**Acceptance / Test Hooks:**
- **Simulation Scenarios:**
  - `ride-hail/accessibility/securement_verification.json`: Various wheelchair types and positions
  - `ride-hail/accessibility/ramp_deployment.json`: Different curb heights and obstacles
  - `ride-hail/accessibility/companion_accommodation.json`: Mixed passenger configurations
- **Logs/Telemetry:**
  - Securement metrics: attachment points, tension verification, stability
  - Equipment metrics: ramp/lift operation, door clearance, interior space
  - Experience metrics: ride smoothness, wait times, total trip duration
- **Gates:**
  - ADA compliance verification with zero violations
  - Securement system validation with 100% success rate
  - Field validation with diverse wheelchair types and mobility devices

**Rollout Plan:**
- **Phase 1:** Limited WAV fleet with specialized support team
- **Phase 2:** Expanded fleet with integrated support protocols
- **Phase 3:** Full service parity with standard ride-hail operations

## Additional Information

**Related Use Cases:**
- R1: Standard Point-to-Point Ride
- R3: Airport Pickup / Drop with Staging
- R16: Hospital Patient Rides

**References:**
- ADA Transportation Requirements
- Wheelchair Securement Best Practices
- Accessibility Service Standards

**Notes:**
This use case represents a critical equity and compliance application that ensures transportation access for all users. Success here demonstrates the system's ability to provide specialized service while maintaining safety, dignity, and efficiency for riders with accessibility needs.

## Cross-Functional Considerations

**Product Management:**
- Clear definition of accessibility service standards
- Measurable equity and inclusion metrics
- Integration with broader transportation accessibility initiatives

**Design:**
- Universal design principles throughout user experience
- Multiple communication modalities (visual, audio, haptic)
- Extensive user testing with diverse accessibility needs

**Engineering:**
- Robust securement systems with redundant verification
- Precision control for ramp/lift deployment
- Specialized ride comfort parameters for medical considerations

**Data:**
- Privacy protections for health-related information
- Accessibility mapping data collection and validation
- Service quality metrics with accessibility-specific dimensions

**QA:**
- Testing with actual mobility devices and users
- Validation across diverse environmental conditions
- Verification of regulatory compliance in all operational modes

**Security:**
- Protection of health-related rider information
- Secure handling of accessibility preferences and needs
- Physical security measures for specialized equipment

**Operations:**
- Specialized maintenance protocols for accessibility equipment
- Training for support personnel on accessibility needs
- Clear escalation procedures for accessibility-related issues

**Regulatory:**
- Comprehensive compliance with ADA and local requirements
- Documentation of accessibility accommodations and service levels
- Regular auditing and reporting procedures

**Community Relations:**
- Engagement with disability advocacy organizations
- Transparent communication about accessibility features and limitations
- Feedback mechanisms for continuous improvement

**Medical/Health:**
- Consideration of diverse mobility needs and medical requirements
- Accommodation of service animals and medical equipment
- Emergency protocols for medical situations
