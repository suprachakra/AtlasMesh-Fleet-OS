# D26 — Rugged Terrain Mobility & Traversability

## Basic Information

**ID:** D26  
**Name:** Rugged Terrain Mobility & Traversability  
**Sector:** Defense  
**Status:** Draft  
**Version:** 1.0  
**Last Updated:** 2025-09-27  
**Owner:** Defense Sector Lead

## Use Case Definition

**Actors**
- **Primary:** Mission Commander, Convoy Lead, UGV/Autonomy Operator
- **Supporting:** Engineer (Sapper) Unit, Recovery Team, TOC (Tactical Operations Center)

**Intent**
Enable autonomous (or supervised autonomous) movement of logistics and patrol vehicles over **rugged, off-road terrain** (wadis, rock fields, ruts, steps, side-slopes, dunes) while keeping **mission tempo**, **crew safety**, and **vehicle integrity** within limits.

## Operating Conditions

**Environment**
- Terrain: hardpan, rock gardens (loose/embedded), soft sand, gravel, washouts, small boulder fields
- Grades: up to **±15%** sustained; **steps/ledges ≤ 0.3 m** (vehicle dependent)
- Side-slopes: up to **12–15°** (vehicle dependent)
- Weather: heat **50–60°C**, dust/sand ingestion; occasional flash-flood erosion
- Visibility: daylight/night; NVG compatible

**Comms/GNSS**
- **Intermittent LTE/5G/mesh/SAT**; GNSS degraded/denied pockets common
- **Edge-first autonomy** with **store-and-forward** telemetry

**Vehicles**
- Wheeled 4×4/6×6 MRAP/light trucks; tracked UGVs
- Constraints vary by **vehicle profile**: curb weight, **ground clearance**, **approach/departure/breakover**, **tire type/pressure control**, **CoG height**

## Preconditions & Triggers

**Preconditions**
- Site/route survey loaded (DEM/hazard layers if available)
- Vehicle profile + ODD policy loaded (grade, side-slope, step limits)
- Recovery kit and **safe-stop** zones planned
- ROE and convoy spacing rules configured

**Triggers**
- Mission plan includes off-road segments or degraded roads
- Dynamic reroute required due to obstacle, washout, or threat
- Tele-assist Q&A requested for ambiguous traversability

## Success Metrics & Constraints

**Mission Outcomes**
- **Mission completion ≥ 98%** within ODD
- **Assists ≤ 0.5 / 1,000 km** on off-road segments
- **Stuck events ≤ 1 / 10,000 km**; **self-recovery success ≥ 80%**
- **Rollover/near-rollover = 0**; side-slope exceedances auto-abort

**Mobility KPIs**
- Traversability model **precision/recall ≥ 0.9/0.85**
- Speed adherence: **avg speed ≥ 85%** of planned off-road pace
- **Slip ratio** within vehicle-specific limits (alert if exceeded for >5 s)
- **Ride severity** (ISO 2631 proxy) stayed within safe band

**Constraints**
- Hard limits enforced from vehicle profile (grade/step/side-slope/clearance)
- Thermal/derating curves respected; dust occlusion → degrade/slow/clean

## Failure Modes & Mitigations

| Failure Mode | Detection | Mitigation |
|---|---|---|
| Wheel slip / sink in soft sand | Slip ratio ↑, acceleration mismatch, depth camera | Reduce tire pressure (if CTIS), lower speed/gear, alternate line, safe-stop |
| High-centering / belly | Ground clearance model vs. DEM/hazard | Reroute; build up ramp (engineer assist); reverse path with guidance |
| Rollover risk (side-slope) | IMU roll-rate/angle, prediction vs. threshold | Immediate speed reduction, widen track line; **auto-abort** above limit |
| Rock strike | Spectral vibration spike, under-carriage proximity | Reduce speed; path offset; trigger inspection on next stop |
| Occlusion (dust) | Lidar return drop; camera contrast loss | Switch to radar/IMU-dominant fusion; slow/hold; lens air-knife/cleaning |
| GNSS loss | Position confidence drop | Switch to SLAM/local odometry; convoy leader-follower; breadcrumb beacons |
| Comms outage | RTT/loss thresholds | Edge autonomy budget (≥45 min); store-and-forward; safe-stop waypoints |

## Data & Integration

- Inputs: **DEM**/elevation tiles, slope/aspect rasters, prior traversability maps, on-vehicle perception (lidar/radar/camera), IMU, wheel-torque/odometry, CTIS status
- Products: **Traversability cost map**, predicted line, expected speed/ride severity, residual risk score
- Interfaces: Mission planner/route engine, Policy/ODD guard, Tele-assist Q&A, Maintenance (impact events)

## Security & Compliance

- STANAG/DoD data handling overlays; air-gapped option
- Evidence hooks: decision traces, policy versions, sensor provenance
- Safety: **hard interlocks** on roll/grade/step; operator confirmation for borderline segments

## Validation & Deployment

**Acceptance / Test Hooks**
- **Simulation scenarios**: rock-garden crossing, side-slope traverse, soft-sand climb, step up/down, washout bypass, GNSS-denied convoy leader-follower
- **Logs/Telemetry**: roll angle/rate, slip ratio, impact events, occlusion %, assist calls, predicted vs. actual speed
- **Gates**:
  - Traversability model P/R ≥ 0.9/0.85 on sector dataset
  - Rollover risk never breached in sim/field trials
  - Assist rate and stuck events under thresholds for 30-day trial

