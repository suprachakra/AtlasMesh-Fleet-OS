# R9 — Medical Transport (Non-Emergency)

## Basic Information

**ID:** R9  
**Name:** Medical Transport (Non-Emergency)  
**Sector:** Ride-hail  
**Status:** Approved  
**Version:** 1.0  
**Last Updated:** 2025-09-14  
**Owner:** Ride-hail Sector Lead

## Use Case Definition

**Actors:**
- Primary: Patients, Healthcare Providers
- Supporting: Caregivers, Healthcare Facility Staff, Fleet Operations

**Trip Type:** MEDICAL_RUN

**ODD (Operational Design Domain):**
- Geographic: Healthcare facilities, residential areas, standard operational zones
- Environmental: Standard weather thresholds with enhanced comfort considerations
- Time: Aligned with healthcare appointment schedules
- Communications: Urban LTE/5G with high reliability requirements
- Other: Enhanced accessibility and comfort features

**Trigger:**
Healthcare appointment scheduling, discharge transportation, or recurring medical visits

**Nominal Flow:**
1. Healthcare provider or patient schedules medical transport through dedicated interface
2. System assigns appropriate vehicle with required accessibility features and comfort settings
3. Pre-trip verification confirms special requirements (mobility assistance, companion, etc.)
4. Vehicle navigates to pickup location with precise arrival timing
5. Patient boarding is facilitated with appropriate assistance features and verification
6. Vehicle interior adjusts to patient comfort preferences (temperature, lighting, etc.)
7. System provides smooth, comfort-optimized routing to healthcare facility
8. Upon arrival, system selects optimal drop-off point for facility access
9. Healthcare facility is notified of patient arrival with relevant information
10. Return trip is coordinated with appointment completion or scheduled in advance

**Variants / Edge Cases:**
- Mobility assistance requirements: Enhanced boarding protocols
- Companion accompaniment: Space and seating accommodations
- Medical equipment transport: Secure storage and handling
- Appointment delays: Waiting or rescheduling coordination
- Patient condition changes: Comfort adjustments or escalation protocols

## Performance & Integration

**KPIs:**
- **P0 (Must have):**
  - On-time performance: ≥97% for medical appointments
  - Accessibility compliance: 100% of required accommodations provided
  - Ride comfort: ≥4.8/5.0 patient satisfaction
  - Healthcare integration: ≥95% successful facility notifications
- **P1 (Should have):**
  - No-show reduction: -40% vs. traditional medical transport
  - Appointment adherence: +25% improvement in patient compliance
  - Return trip coordination: ≤15 minutes average wait time
  - Cost efficiency: -30% vs. specialized medical transport services

**Dependencies:**
- **Services:**
  - `dispatch`: Medical priority scheduling and vehicle assignment
  - `routing`: Comfort-optimized navigation with accessibility considerations
  - `rider-ux`: Patient-focused experience and communication
  - `adapters/healthcare`: Healthcare system integration
  - `policy-engine`: Medical transport protocols and privacy rules
- **Rules:**
  - `rules/odd/ride-hail/medical_transport.yaml`: Medical transport parameters
  - `rules/policy/ride-hail/accessibility.yaml`: Accommodation requirements and verification
  - `rules/policy/ride-hail/companion_riders.yaml`: Caregiver and companion policies
  - `rules/policy/privacy/healthcare.yaml`: Medical information handling requirements
- **External Systems:**
  - Healthcare Appointment Systems: Scheduling and facility integration
  - Patient Management Systems: Special requirements and preferences
  - Health Insurance Systems: Coverage verification and billing

**Risks & Mitigations:**
- **Patient mobility challenges:**
  - Impact: High
  - Mitigation: Comprehensive accessibility features, clear assistance protocols, training
- **Medical information privacy:**
  - Impact: High
  - Mitigation: HIPAA-compliant data handling, minimal necessary information, secure channels
- **Appointment timing precision:**
  - Impact: High
  - Mitigation: Buffer planning, priority routing, healthcare coordination
- **Patient comfort requirements:**
  - Impact: Medium
  - Mitigation: Customizable environment, smooth driving profile, comfort verification

## Validation & Deployment

**Acceptance / Test Hooks:**
- **Simulation Scenarios:**
  - `ride-hail/medical/accessibility_scenarios.json`: Various mobility assistance needs
  - `ride-hail/medical/appointment_coordination.json`: Timing and scheduling challenges
  - `ride-hail/medical/comfort_optimization.json`: Ride quality for sensitive conditions
- **Logs/Telemetry:**
  - Timing metrics: pickup punctuality, appointment arrival, return coordination
  - Accessibility metrics: feature utilization, assistance effectiveness, compliance
  - Experience metrics: comfort ratings, ride smoothness, environment customization
- **Gates:**
  - Accessibility compliance validation with ADA standards
  - Healthcare integration verification with facility partners
  - Patient experience assessment with representative user testing

**Rollout Plan:**
- **Phase 1:** Basic medical transport with accessibility features
- **Phase 2:** Healthcare facility integration and appointment coordination
- **Phase 3:** Full patient experience optimization and insurance integration

## Additional Information

**Related Use Cases:**
- R1: Standard Point-to-Point Ride
- R4: Accessibility Ride
- R6: Scheduled Commuter Service

**References:**
- Non-Emergency Medical Transportation Standards
- Healthcare Accessibility Guidelines
- Patient Privacy Requirements

**Notes:**
This use case represents a specialized service that addresses significant healthcare access challenges while providing a more cost-effective alternative to traditional medical transport. Success here demonstrates the system's ability to meet specific patient needs while integrating with healthcare systems and maintaining privacy compliance.

## Cross-Functional Considerations

**Product Management:**
- Clear definition of medical transport service levels
- Measurable impact on healthcare access and appointment adherence
- Integration with healthcare provider workflows

**Design:**
- Patient-centered experience design
- Clear communication for healthcare coordination
- Accessibility for diverse medical needs

**Engineering:**
- Enhanced ride comfort algorithms
- Reliable healthcare system integration
- Secure handling of protected health information

**Data:**
- HIPAA-compliant data management
- Minimal necessary information principles
- Secure storage with appropriate retention

**QA:**
- Validation of accessibility features with diverse needs
- Testing of healthcare integration workflows
- Verification of comfort optimization effectiveness

**Security:**
- Protection of health-related information
- Secure facility notification mechanisms
- Privacy controls for sensitive conditions

**Operations:**
- Clear procedures for patient assistance
- Training for operations staff on medical needs
- Escalation protocols for patient concerns

**Healthcare Relations:**
- Facility partnership development
- Provider education on service capabilities
- Integration with patient transportation programs

**Legal/Compliance:**
- HIPAA and healthcare privacy compliance
- Accessibility regulation adherence
- Insurance and billing requirements

**Community Impact:**
- Healthcare access improvement metrics
- Appointment adherence tracking
- Patient outcome correlation studies
