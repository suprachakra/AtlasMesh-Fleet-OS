# D1 — Autonomous FOB Resupply Convoy

## Basic Information

**ID:** D1  
**Name:** Autonomous FOB Resupply Convoy  
**Sector:** Defense  
**Status:** Approved  
**Version:** 1.0  
**Last Updated:** 2025-09-13  
**Owner:** Defense Sector Lead

## Use Case Definition

**Actors:**
- Primary: Mission Commander, Dispatcher
- Supporting: UGVs, Escort Sensors, Base Security

**Trip Type:** OP_RUN (multi-vehicle)

**ODD (Operational Design Domain):**
- Geographic: Desert corridors, established supply routes
- Environmental: Visibility ≥ Vmin (200m), wind ≤ Wmax (80 km/h), temperature range -10°C to 55°C
- Time: 24/7 operations
- Communications: GNSS degraded acceptable, SAT fallback required, mesh networking
- Other: Convoy spacing policy enforcement

**Trigger:**
Supply level at FOB < policy threshold (typically 72h remaining) or scheduled resupply window

**Nominal Flow:**
1. Mission Commander creates multi-stop route with supply manifest
2. System encrypts manifest and assigns security classification
3. Convoy forms with platoon spacing policy applied
4. Execution begins with continuous EW monitoring and mesh comms
5. System automatically implements safe-harbor protocol if comms loss exceeds threshold
6. Delivery at FOB with digital receipt attestation
7. Backhaul of return cargo (if applicable)
8. Mission completion with after-action report generation

**Variants / Edge Cases:**
- GPS jamming: System falls back to visual-inertial odometry and pre-loaded maps
- Sand drifts: Detection and rerouting or safe-stop if impassable
- Blocked waypoint: Dynamic replanning with Mission Commander approval
- Escort handover: Protocol for transferring control to human escort at sensitive zones
- Night operations: Thermal/IR mode with adjusted spacing and speed profiles

## Performance & Integration

**KPIs:**
- **P0 (Must have):**
  - Mission completion rate: ≥ 98% in ODD
  - Assists per distance: ≤ 0.3/1k km
  - Q&A tele-assist turnaround: P95 < 2s
  - Tamper/security events: 0
- **P1 (Should have):**
  - Time to delivery: -25% vs. manned convoys
  - Exposure hours reduction: -80% (human exposure reduction)
  - Fuel efficiency: +10% vs. manned convoys
  - Comms resilience: 99.5% uptime across mission

**Dependencies:**
- **Services:**
  - `dispatch`: Multi-vehicle coordination
  - `routing`: Desert-aware path planning
  - `rebalancer`: Convoy formation management
  - `v2x-service`: Vehicle-to-vehicle communications
  - `weather-fusion`: Real-time environmental monitoring
  - `policy-engine`: Security and ROE enforcement
  - `map-service`: Offline maps with provenance
  - `alerts-incident`: Security and safety alerting
- **Rules:**
  - `rules/odd/defense/convoy_rules.yaml`: Spacing and formation policies
  - `rules/policy/security/comms_degraded.yaml`: Communications fallback procedures
  - `rules/policy/safety/collision_avoidance.yaml`: Convoy-specific safety parameters
- **External Systems:**
  - Defense ERP: Supply manifest integration
  - Security classification system: Data handling requirements

**Risks & Mitigations:**
- **Electronic warfare / jamming:**
  - Impact: High
  - Mitigation: Multi-sensor localization, pre-loaded maps, mesh networking
- **Convoy break / vehicle separation:**
  - Impact: High
  - Mitigation: Auto regroup protocol, safe-harbor points, contingency planning
- **Communications failure:**
  - Impact: Medium
  - Mitigation: Store-and-forward protocols, autonomous decision-making within ROE
- **Hostile action:**
  - Impact: High
  - Mitigation: Route randomization, security escort protocols, tamper-evident systems

## Validation & Deployment

**Acceptance / Test Hooks:**
- **Simulation Scenarios:**
  - `defense/convoy/jamming_with_drift.json`: GNSS denial + sand drift obstacles
  - `defense/convoy/comms_degraded_reroute.json`: Communications failure with rerouting
  - `defense/convoy/blocked_road_safe_harbor.json`: Major obstacle with safe-harbor activation
- **Logs/Telemetry:**
  - Convoy integrity metrics: spacing, formation adherence
  - Communications quality and fallback activation
  - Security events and attestation logs
- **Gates:**
  - Simulation pass rate ≥ 95% across all defense convoy scenarios
  - Security audit pass with zero critical findings
  - Field validation with incremental autonomy level

**Rollout Plan:**
- **Phase 1:** 2-vehicle convoy on secure base roads with human escort
- **Phase 2:** Expand to 5-vehicle convoy on established supply routes
- **Phase 3:** Scale to 10+ vehicles, integrate with ERP for auto-manifests

## Additional Information

**Related Use Cases:**
- D2: Last-Mile Ammo/Water Drop
- D8: Secure Data Courier
- D10: Emergency Extraction Escort

**References:**
- Defense Sector Requirements Document v2.3
- Convoy Operations SOP Manual
- Electronic Warfare Resilience Framework

**Notes:**
This use case represents a high-value, high-risk application that demonstrates the system's resilience to harsh conditions and communications challenges. Success here will establish credibility across all sectors.
