# D5 — Autonomous Medevac Shuttle (Non-Clinical)

## Basic Information

**ID:** D5  
**Name:** Autonomous Medevac Shuttle (Non-Clinical)  
**Sector:** Defense  
**Status:** Approved  
**Version:** 1.0  
**Last Updated:** 2025-09-13  
**Owner:** Defense Sector Lead

## Use Case Definition

**Actors:**
- Primary: Military Medical Command, Field Medics
- Supporting: Forward Operating Units, Medical Facilities, Escort Security (optional)

**Trip Type:** MEDICAL_RUN

**ODD (Operational Design Domain):**
- Geographic: Established routes between casualty collection points and medical facilities
- Environmental: All weather conditions with appropriate adjustments, temperature control for patient transport
- Time: 24/7 operations with day/night capabilities
- Communications: Secure comms with medical priority protocols, store-and-forward capability
- Other: Enhanced ride comfort parameters, specialized medical configuration

**Trigger:**
Medical evacuation request from field units or scheduled medical transport

**Nominal Flow:**
1. Medical command issues evacuation request with priority level and patient details
2. System selects appropriate vehicle with verified medical configuration
3. Vehicle navigates to casualty collection point with optimal route selection
4. Upon arrival, system verifies location and prepares for patient loading
5. Medical personnel load stabilized casualties with stretcher accommodation
6. System verifies secure patient positioning with multi-point validation
7. Vehicle navigates to medical facility with specialized comfort parameters (minimized jerk, acceleration control)
8. Continuous monitoring of patient compartment environment (temperature, stability)
9. Upon arrival at medical facility, system coordinates handoff with receiving staff
10. Vehicle undergoes sanitization protocol before returning to service

**Variants / Edge Cases:**
- Multiple casualty transport: Priority-based configuration and routing
- Route blockage: Dynamic rerouting with medical urgency consideration
- Environmental extremes: Enhanced climate control for patient protection
- Security threats: Escort coordination and threat-aware routing
- Communications failure: Autonomous completion with offline navigation

## Performance & Integration

**KPIs:**
- **P0 (Must have):**
  - Response time: -25% vs. conventional transport
  - Patient compartment stability: Jerk/acceleration within medical limits
  - Route risk score: ≤ 0.2 (scale 0-1)
  - Temperature control accuracy: ±1°C from target
- **P1 (Should have):**
  - Mission completion rate: ≥ 99.5%
  - Sanitization verification: 100% compliance
  - Fuel/energy efficiency: +15% vs. conventional transport
  - Patient monitoring data integrity: 100%

**Dependencies:**
- **Services:**
  - `dispatch`: Medical priority-based assignment
  - `routing`: Medical comfort-optimized navigation
  - `policy-engine`: Medical transport protocols and regulations
  - `fleet-health`: Medical configuration verification
  - `environment-control`: Patient compartment climate management
- **Rules:**
  - `rules/odd/defense/medical_transport.yaml`: Medical transport operational parameters
  - `rules/policy/defense/patient_safety.yaml`: Patient security and comfort requirements
  - `rules/policy/defense/medical_priority.yaml`: Evacuation priority protocols
  - `rules/policy/security/medical_transport.yaml`: Security considerations for medical vehicles
- **External Systems:**
  - Medical Command System: Evacuation requests and patient tracking
  - Field Medical System: Patient status and requirements
  - Medical Facility System: Receiving capacity and preparation

**Risks & Mitigations:**
- **Patient condition deterioration during transport:**
  - Impact: Critical
  - Mitigation: Stable ride characteristics, environmental controls, monitoring systems (non-clinical)
- **Route delays impacting medical outcomes:**
  - Impact: High
  - Mitigation: Priority routing, alternative path planning, continuous ETA updates
- **Medical equipment interference:**
  - Impact: High
  - Mitigation: EMI-shielded compartments, certified equipment compatibility, power isolation
- **Sanitization failures between transports:**
  - Impact: High
  - Mitigation: Automated cleaning verification, contamination detection, maintenance tracking

## Validation & Deployment

**Acceptance / Test Hooks:**
- **Simulation Scenarios:**
  - `defense/medevac/patient_stability.json`: Ride comfort and stability testing
  - `defense/medevac/multiple_casualty.json`: Multiple patient configuration handling
  - `defense/medevac/environmental_control.json`: Climate management in extreme conditions
- **Logs/Telemetry:**
  - Patient compartment metrics: temperature, humidity, vibration, acceleration
  - Route metrics: comfort score, time to destination, route stability
  - Vehicle configuration: medical equipment status, sanitization verification
- **Gates:**
  - Medical command approval of transport protocols
  - Ride comfort validation within medical specifications
  - Field validation with simulated patient transport

**Rollout Plan:**
- **Phase 1:** Non-emergency medical transport on established routes
- **Phase 2:** Urgent medical transport with medical escort
- **Phase 3:** Full evacuation capabilities with reduced escort requirements

## Additional Information

**Related Use Cases:**
- D1: FOB Resupply Convoy
- D2: Last-Mile Critical Drop
- D22: Firefighting Support

**References:**
- Military Medical Evacuation Standards
- Patient Transport Comfort Requirements
- Medical Vehicle Sanitization Protocols

**Notes:**
This use case focuses on the non-clinical transport of stabilized casualties, not providing medical care during transit. The system is designed to reduce response times and improve transport comfort while ensuring patients arrive at medical facilities in stable condition. The clear limitation to non-clinical transport maintains appropriate medical responsibility while leveraging autonomy for logistics efficiency.

## Cross-Functional Considerations

**Product Management:**
- Clear definition of medical transport scope (non-clinical)
- Measurable impact on medical response capabilities
- Integration with existing medical evacuation workflows

**Design:**
- Intuitive status communication for medical personnel
- Clear loading/unloading guidance and verification
- Accessibility for field conditions and medical equipment

**Engineering:**
- Enhanced suspension and motion control for patient comfort
- Medical-grade environmental control systems
- EMI shielding for medical device compatibility

**Data:**
- Privacy protections for patient information
- Secure handling of medical transport records
- Sanitization verification and maintenance tracking

**QA:**
- Validation of ride comfort across diverse terrain
- Testing of patient compartment environmental controls
- Verification of sanitization effectiveness

**Security:**
- Protection of medical mission information
- Physical security measures for medical transport
- Compliance with medical data handling requirements

**Medical:**
- Consultation with medical professionals on requirements
- Validation of transport parameters for patient safety
- Clear documentation of non-clinical limitations

**Operations:**
- Specialized maintenance protocols for medical configuration
- Training for medical personnel on vehicle interaction
- Clear procedures for emergency situations

**Regulatory:**
- Compliance with military medical transport regulations
- Documentation of safety case for medical applications
- Regular certification and inspection procedures
