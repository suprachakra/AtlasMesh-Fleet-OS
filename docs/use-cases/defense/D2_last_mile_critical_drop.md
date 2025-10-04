# D2 — Last-Mile Critical Drop (Contested)

## Basic Information

**ID:** D2  
**Name:** Last-Mile Critical Drop (Contested)  
**Sector:** Defense  
**Status:** Approved  
**Version:** 1.0  
**Last Updated:** 2025-09-13  
**Owner:** Defense Sector Lead

## Use Case Definition

**Actors:**
- Primary: Border Guards, Special Operations Forces (SOF) Logistics
- Supporting: Mission Commander, Dispatch, UGV

**Trip Type:** OP_RUN (solo vehicle)

**ODD (Operational Design Domain):**
- Geographic: Last-mile supply routes, unpaved terrain, remote outposts
- Environmental: Desert conditions, visibility ≥ 100m, temperature range -10°C to 55°C
- Time: 24/7 operations with night vision capability
- Communications: Minimal comms footprint, store-and-forward, SAT fallback
- Other: Stealth mode capabilities, randomized path selection

**Trigger:**
Critical inventory threshold alert or scheduled resupply window

**Nominal Flow:**
1. Mission Commander creates secure supply manifest with criticality level
2. System plans optimal path with randomization factors for security
3. Solo UGV loads critical supplies in tamper-evident containers
4. UGV executes mission with minimal RF emissions profile
5. Upon arrival at cache point, UGV authenticates location and conditions
6. Contactless handoff with digital receipt attestation (QR + biometric)
7. System generates chain-of-custody proof with cryptographic verification
8. UGV returns via different route with mission telemetry

**Variants / Edge Cases:**
- Ambush detection: Sensor anomaly triggers safe protocol
- Route blockage: Dynamic rerouting with risk assessment
- Authentication failure: Secure hold protocol with escalation
- Comms blackout: Autonomous completion with offline verification
- Night operations: IR/thermal mode with adjusted movement profile

## Performance & Integration

**KPIs:**
- **P0 (Must have):**
  - Drop success rate: ≥ 99%
  - Chain-of-custody verification: 100% complete
  - Safe-stop rate: ≤ 1 per 10 missions
  - Custody proof delivery: within 60s of handoff
- **P1 (Should have):**
  - Route unpredictability index: ≥ 80%
  - Mission time variance: ≤ 15% from estimate
  - Energy efficiency: +20% vs. manned delivery
  - RF emissions: ≤ 10% of standard operations

**Dependencies:**
- **Services:**
  - `trip-service`: Mission planning and execution
  - `policy-engine`: Security and ROE enforcement
  - `map-service`: Offline maps with terrain analysis
  - `v2x-service`: Minimal, encrypted communications
  - `ota-service`: Secure attestation and verification
- **Rules:**
  - `rules/odd/defense/contested_terrain.yaml`: Terrain handling policies
  - `rules/policy/security/emissions_control.yaml`: RF emissions management
  - `rules/policy/security/chain_of_custody.yaml`: Supply integrity verification
- **External Systems:**
  - Defense logistics system: Supply manifest integration
  - Cryptographic verification system: Chain-of-custody validation

**Risks & Mitigations:**
- **Ambush / hostile action:**
  - Impact: High
  - Mitigation: Randomized path selection, minimal RF emissions, stealth movement profiles
- **Supply integrity compromise:**
  - Impact: High
  - Mitigation: Tamper-evident containers, continuous monitoring, cryptographic seals
- **Authentication failure:**
  - Impact: Medium
  - Mitigation: Multi-factor authentication, fallback protocols, operator escalation
- **Vehicle loss:**
  - Impact: High
  - Mitigation: Secure data wipe, self-neutralization capabilities, minimal sensitive data storage

## Validation & Deployment

**Acceptance / Test Hooks:**
- **Simulation Scenarios:**
  - `defense/critical_drop/contested_terrain.json`: Navigation through difficult terrain
  - `defense/critical_drop/ambush_detection.json`: Threat detection and response
  - `defense/critical_drop/authentication_failure.json`: Authentication failure handling
- **Logs/Telemetry:**
  - Mission integrity metrics: route adherence, emissions profile
  - Authentication and handoff verification logs
  - Threat detection and response metrics
- **Gates:**
  - Simulation pass rate ≥ 95% across all defense critical drop scenarios
  - Security audit pass with zero critical findings
  - Field validation with incremental autonomy level

**Rollout Plan:**
- **Phase 1:** Single vehicle on known routes with human escort
- **Phase 2:** Expand to multiple routes with reduced escort presence
- **Phase 3:** Full autonomous operation with remote monitoring only

## Additional Information

**Related Use Cases:**
- D1: FOB Resupply Convoy
- D8: Secure Data Courier
- D16: Decoy Convoy for Deception

**References:**
- Defense Logistics Manual v3.2
- Last-Mile Supply Chain SOP
- Contested Environment Operations Guide

**Notes:**
This use case represents a high-security, mission-critical application that demonstrates the system's ability to operate in contested environments with minimal support infrastructure. Success here will establish credibility for sensitive defense applications.
