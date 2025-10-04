# R2 — Shared Ride / Pool

## Basic Information

**ID:** R2  
**Name:** Shared Ride / Pool  
**Sector:** Ride-hail  
**Status:** Approved  
**Version:** 1.0  
**Last Updated:** 2025-09-13  
**Owner:** Ride-hail Sector Lead

## Use Case Definition

**Actors:**
- Primary: Multiple Riders, Autonomous Vehicle
- Supporting: Ride-hail Platform, Fleet Operations, Rider Support

**Trip Type:** POOL_RUN

**ODD (Operational Design Domain):**
- Geographic: Licensed districts with geofence boundaries, optimized for shared routes
- Environmental: Standard weather thresholds (visibility >100m, no flooding, wind <80km/h)
- Time: Peak hours focus (commute windows, event dispersal)
- Communications: Urban LTE/5G with local fallback
- Other: Multi-stop routing with fairness constraints

**Trigger:**
Multiple rider requests with compatible origin-destination pairs and time windows

**Nominal Flow:**
1. System identifies compatible ride requests based on route overlap and time windows
2. Dispatch optimizes multi-pickup sequence considering fairness, ETA, and capacity
3. Vehicle is assigned with optimized route and pickup/dropoff sequence
4. Riders receive individualized ETAs and trip details with privacy protections
5. Vehicle executes first pickup with standard verification process
6. Navigation continues to subsequent pickups with dynamic route adjustments
7. Vehicle manages cabin space allocation and luggage constraints
8. Dropoffs are executed in optimal sequence based on route efficiency and fairness
9. System calculates individual fare splits based on distance, time, and detour factors
10. Riders provide individual ratings with specific shared-ride metrics

**Variants / Edge Cases:**
- Rider no-show: Skip logic with timeout and continuation protocol
- Capacity constraints: Luggage or wheelchair space management
- Route deviation requests: Policy for accommodating mid-trip changes
- Rider conflict: Conflict resolution protocol and support escalation
- Last-minute cancellation: Reoptimization for remaining riders

## Performance & Integration

**KPIs:**
- **P0 (Must have):**
  - Seat fill rate: ≥ 2.2 riders per trip average
  - Detour time: P95 ≤ 15 minutes from direct route
  - Rider satisfaction: CSAT ≥ 4.5/5.0 (vs. 4.8 for solo rides)
  - Matching success rate: ≥ 80% of pool-eligible requests
- **P1 (Should have):**
  - Rider wait time: P95 ≤ 10 minutes (vs. 7 for solo rides)
  - Fairness index: ≥ 0.9 (measure of equitable detour distribution)
  - Vehicle utilization: +30% vs. solo rides
  - Energy efficiency: +25% passenger-km per kWh

**Dependencies:**
- **Services:**
  - `dispatch`: Multi-rider matching and sequencing
  - `routing`: Multi-stop optimization with fairness constraints
  - `rebalancer`: Pool-optimized positioning
  - `policy-engine`: Rider interaction and fairness rules
  - `rider-ux`: Pool-specific rider communications
- **Rules:**
  - `rules/odd/ride-hail/pool_rules.yaml`: Pool-specific operational parameters
  - `rules/policy/ride-hail/multi_pickup.yaml`: Pickup/dropoff sequencing policies
  - `rules/policy/ride-hail/fairness.yaml`: Detour and wait time fairness constraints
  - `rules/policy/ride-hail/privacy.yaml`: Inter-rider privacy protections
- **External Systems:**
  - Rider App: Pool-specific UI/UX elements
  - Payment Processing: Split fare calculation
  - Rating System: Pool-specific feedback categories

**Risks & Mitigations:**
- **Rider dissatisfaction with detours:**
  - Impact: High
  - Mitigation: Clear expectations setting, fairness algorithms, transparent ETA updates
- **Privacy concerns between riders:**
  - Impact: Medium
  - Mitigation: Limited information sharing, destination anonymization, privacy settings
- **Capacity constraints:**
  - Impact: Medium
  - Mitigation: Pre-trip luggage/space declaration, vehicle capacity matching
- **Rider conflicts:**
  - Impact: Medium
  - Mitigation: Cabin monitoring, support escalation protocols, conflict resolution guidelines

## Validation & Deployment

**Acceptance / Test Hooks:**
- **Simulation Scenarios:**
  - `ride-hail/pool/multi_pickup_sequence.json`: Various pickup/dropoff combinations
  - `ride-hail/pool/fairness_optimization.json`: Detour distribution testing
  - `ride-hail/pool/capacity_constraints.json`: Luggage and space management
- **Logs/Telemetry:**
  - Matching metrics: compatible requests, successful matches, rejection reasons
  - Efficiency metrics: seat utilization, detour times, route efficiency
  - Experience metrics: wait times, ride times, fairness distribution
- **Gates:**
  - Simulation pass rate ≥ 95% across all pool scenarios
  - Fairness algorithm validation with diverse trip patterns
  - Field validation with staged multi-rider trips

**Rollout Plan:**
- **Phase 1:** Limited hours operation on high-demand corridors
- **Phase 2:** Expanded hours and district coverage with full rider support
- **Phase 3:** Dynamic pricing integration and capacity optimization

## Additional Information

**Related Use Cases:**
- R1: Standard Point-to-Point Ride
- R9: Micro-transit Feeder
- R11: Subscription Commute

**References:**
- Ride Pooling Optimization Research
- Rider Experience Guidelines for Shared Mobility
- Fairness Algorithms in Transportation

**Notes:**
This use case represents a complex optimization challenge balancing efficiency, fairness, and rider experience. Success here demonstrates the system's ability to manage multi-party interactions while maintaining high service levels and vehicle utilization.
