# R1 — Standard Point-to-Point Ride

## Basic Information

**ID:** R1  
**Name:** Standard Point-to-Point Ride  
**Sector:** Ride-hail  
**Status:** Approved  
**Version:** 1.0  
**Last Updated:** 2025-09-13  
**Owner:** Ride-hail Sector Lead

## Use Case Definition

**Actors:**
- Primary: Rider, Autonomous Vehicle
- Supporting: Rider Support, Fleet Operations

**Trip Type:** OP_RUN

**ODD (Operational Design Domain):**
- Geographic: Licensed districts with geofence boundaries
- Environmental: Weather thresholds enforced (visibility >100m, no flooding, wind <80km/h), temperature range -10°C to 55°C
- Time: 24/7 operations with time-of-day specific routing profiles
- Communications: Urban LTE/5G with local fallback
- Other: Work-zone tele-assist allowed, rider interaction protocols

**Trigger:**
App request within ODD boundaries and vehicle availability

**Nominal Flow:**
1. System dispatches vehicle with accurate ETA to rider
2. Vehicle selects optimal pickup curb location based on safety and convenience
3. Rider verification process confirms correct passenger
4. Vehicle executes trip with real-time routing and obstacle avoidance
5. Vehicle selects optimal drop-off location based on destination and safety
6. System prompts rider for CSAT rating and feedback
7. Vehicle prepares for next assignment (cleaning check, repositioning)
8. System logs trip metrics and updates fleet analytics

**Variants / Edge Cases:**
- Blocked curb at pickup: Alternative spot selection with rider notification
- Police/fire activity: Rerouting with rider notification
- Construction/detour: Work-zone navigation with possible tele-assist
- Rider no-show: Wait policy with timeout and cancellation procedure
- Extreme heat: Cabin pre-cooling and energy management

## Performance & Integration

**KPIs:**
- **P0 (Must have):**
  - Wait time: P95 ≤ 7 minutes
  - Customer satisfaction: CSAT ≥ 4.8/5.0
  - Cancellation rate: ≤ target (typically 5%)
  - Assists per distance: ≤ 0.5/1k km
- **P1 (Should have):**
  - Trip completion rate: ≥ 98%
  - On-time pickup: ≥ 95% within 2 min of ETA
  - Energy efficiency: +10% vs. human drivers
  - Idle time between trips: < 10 minutes average

**Dependencies:**
- **Services:**
  - `dispatch`: Rider matching and vehicle assignment
  - `routing`: Urban-specific path planning
  - `policy-engine`: Rider interaction and safety rules
  - `fleet-response`: Q&A tele-assist for edge cases
  - `alerts-incident`: Rider support escalation
- **Rules:**
  - `rules/odd/ride-hail/urban_rules.yaml`: Urban driving parameters
  - `rules/policy/ride-hail/pickup_dropoff.yaml`: Curb selection policies
  - `rules/policy/ride-hail/rider_interaction.yaml`: Rider communication protocols
  - `rules/policy/safety/work_zone.yaml`: Construction area handling
- **External Systems:**
  - Rider App: Booking, tracking, and payment
  - Payment Processing: Fare calculation and processing
  - Emergency Services API: Incident reporting

**Risks & Mitigations:**
- **Ambiguous pickup/dropoff locations:**
  - Impact: Medium
  - Mitigation: Precise geolocation, rider confirmation, alternative spot suggestions
- **Construction zones and road changes:**
  - Impact: High
  - Mitigation: Tele-assist Q&A, recent map layers, work zone detection
- **Rider misconduct or emergency:**
  - Impact: Medium
  - Mitigation: In-cabin monitoring, emergency protocols, support escalation
- **Public relations incidents:**
  - Impact: High
  - Mitigation: Incident response templates, media protocols, transparent communication

## Validation & Deployment

**Acceptance / Test Hooks:**
- **Simulation Scenarios:**
  - `ride-hail/standard/work_zone_navigation.json`: Construction area handling
  - `ride-hail/standard/blocked_pickup_spot.json`: Alternative pickup location
  - `ride-hail/standard/night_glare_conditions.json`: Low-light and glare handling
- **Logs/Telemetry:**
  - Rider experience metrics: wait times, trip duration, route efficiency
  - Safety metrics: proximity events, hard braking/acceleration, assists
  - Comfort metrics: jerk, acceleration, temperature
- **Gates:**
  - Rider experience metrics within 10% of targets
  - Safety scenario pass rate 100%
  - Assist rate below threshold in simulation

**Rollout Plan:**
- **Phase 1:** Limited hours operation in low-complexity district
- **Phase 2:** Expanded hours and district coverage with full rider support
- **Phase 3:** Full 24/7 operations across all approved districts

## Additional Information

**Related Use Cases:**
- R2: Shared Ride / Pool
- R3: Airport Pickup / Drop with Staging
- R4: Accessibility Ride

**References:**
- Ride-hail Regulatory Requirements
- Rider Experience Guidelines
- Incident Response Playbook

**Notes:**
This use case represents the core service offering for ride-hail operations and directly impacts customer satisfaction and adoption. Success here builds the foundation for more complex ride types and service expansion.
