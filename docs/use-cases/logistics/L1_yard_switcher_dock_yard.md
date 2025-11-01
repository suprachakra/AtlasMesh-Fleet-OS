# L1 — Yard Switcher: Dock ↔ Yard Slot Shuttling

## Basic Information

**ID:** L1  
**Name:** Yard Switcher: Dock ↔ Yard Slot Shuttling  
**Sector:** Logistics  
**Status:** Approved  
**Version:** 1.0  
**Last Updated:** 2025-09-13  
**Owner:** Logistics Sector Lead

## Use Case Definition

**Actors:**
- Primary: Yard Master, Autonomous Yard Tractor
- Supporting: Gate Operations, WMS/TOS Systems, Dock Personnel

**Trip Type:** OP_RUN (continuous)

**ODD (Operational Design Domain):**
- Geographic: Private yard roads, terminal areas
- Environmental: All weather conditions with appropriate speed adjustments, temperature range -10°C to 55°C
- Time: 24/7 operations
- Communications: Yard WiFi/LTE with local fallback
- Other: Mixed traffic with human-operated vehicles and pedestrians, V2I at gates

**Trigger:**
WMS wave release signal or TOS crane window allocation

**Nominal Flow:**
1. System assigns move task based on WMS/TOS priorities
2. Yard tractor navigates to trailer/container location
3. Hitch verification process confirms correct trailer/container
4. Gate clearance obtained via V2I communication
5. Tractor navigates to assigned dock/slot with pedestrian awareness
6. Trailer/container placement with precision positioning
7. System confirms placement and updates WMS/TOS
8. Tractor proceeds to next assignment in queue

**Variants / Edge Cases:**
- Mis-spotted trailer: Detection and exception handling
- Pedestrian crossing path: Detection, yield, and resume
- Blocked dock: Wait policy with timeout and reassignment
- Yard congestion: Dynamic path planning and priority management
- Gate system failure: Fallback to manual verification

## Performance & Integration

**KPIs:**
- **P0 (Must have):**
  - Gate-to-dock time: P95 ≤ target (typically 15-20 minutes)
  - Miss-spot rate: <0.5%
  - Safety incidents: 0
  - Throughput increase: +15-25% vs. manual operation
- **P1 (Should have):**
  - Fuel/energy efficiency: +10-15% vs. manual operation
  - Yard asset utilization: +20-30%
  - System integration accuracy: >99.9% with WMS/TOS
  - Dwell time reduction: -30%

**Dependencies:**
- **Services:**
  - `adapters/wms`: Integration with Warehouse Management Systems
  - `adapters/tos`: Integration with Terminal Operating Systems
  - `dispatch`: Yard move optimization
  - `routing`: Yard-specific path planning
  - `policy-engine`: Safety and operational rules
  - `v2x-service`: Gate and infrastructure communication
- **Rules:**
  - `rules/odd/logistics/yard_rules.yaml`: Yard-specific speed and safety rules
  - `rules/policy/logistics/dock_approach.yaml`: Dock approach and positioning
  - `rules/policy/safety/pedestrian_priority.yaml`: Pedestrian interaction policies
- **External Systems:**
  - Warehouse Management System: Move requests and confirmations
  - Terminal Operating System: Container tracking and assignments
  - Gate Management System: Access control and verification

**Risks & Mitigations:**
- **Pedestrian safety in mixed traffic:**
  - Impact: High
  - Mitigation: Low-speed operations, pedestrian detection, audible warnings, designated walkways
- **RF dead zones affecting communications:**
  - Impact: Medium
  - Mitigation: Store-and-forward protocols, local decision making, graceful degradation
- **Trailer coupling failures:**
  - Impact: Medium
  - Mitigation: Multi-sensor verification, automated retry with limits, exception alerts
- **WMS/TOS integration reliability:**
  - Impact: High
  - Mitigation: Heartbeat monitoring, transaction logging, manual override capability

## Validation & Deployment

**Acceptance / Test Hooks:**
- **Simulation Scenarios:**
  - `logistics/yard/standard_dock_approach.json`: Normal operations with various trailers
  - `logistics/yard/pedestrian_interaction.json`: Various pedestrian crossing scenarios
  - `logistics/yard/congestion_management.json`: High-volume yard operations
- **Logs/Telemetry:**
  - Move completion metrics: time, accuracy, exceptions
  - Safety metrics: proximity events, yields, stops
  - Integration metrics: system handshakes, confirmation times
- **Gates:**
  - WMS/TOS contract test suite passing
  - Safety scenario pass rate 100%
  - Performance metrics within 10% of targets in simulation

**Rollout Plan:**
- **Phase 1:** Single yard tractor on limited routes during day shift
- **Phase 2:** Expand to 3-5 tractors on full yard operations
- **Phase 3:** Full fleet integration with WMS/TOS and 24/7 operations

## Additional Information

**Related Use Cases:**
- L2: Container Terminal Berth Shuttle
- L3: Cross-Dock Wave Fulfillment
- L9: Port Empty Repositioning Optimization

**References:**
- Yard Safety Standards
- WMS/TOS Integration Specifications
- Terminal Operations SOP

**Notes:**
This use case represents the foundation for logistics automation with immediate ROI through labor savings and throughput improvements. Success here enables expansion to more complex logistics scenarios and builds confidence with terminal operators.
