# AtlasMesh Fleet OS - Use Case Taxonomy

This document provides a comprehensive taxonomy of use cases for the AtlasMesh Fleet OS across all supported sectors. It ensures we have consistent categorization and don't miss critical use case families.

## Mission Families

* **Move things** (people, cargo, fuel/water, tools)
* **Sense & inspect** (patrol, survey, asset inspection, mapping)
* **Maintain & service** (repair, refuel/recharge, swap, cleaning)
* **Control & coordinate** (convoy, platoon, berth/dock/stand control)
* **Protect & respond** (safety, recovery, evacuation, emergency runs)

## Trip Types (UI Left Rail)

* `OP_RUN` (normal operations) - Standard operational trips
* `RELEASE_RUN` (fleet launch window) - Initial deployment or new feature release
* `MAPPING_RUN` - Dedicated map data collection
* `CALIB_RUN` - Sensor calibration and system tuning
* `MAINT_RUN` - Maintenance-related trips
* `TEST_RUN` (A/B, sandbox routes) - Experimental or validation runs
* `EMERGENCY_RUN` - Emergency response operations
* `TRAINING_RUN` (operator drills) - Training and simulation
* `BACKHAUL_RUN` (return logistics) - Return trips after primary mission
* `RELOCATION_RUN` (repositioning) - Asset repositioning

## Trip Status Flow

* `PLANNED` → Initial trip creation, not yet ready for dispatch
* `QUEUED` → Ready for dispatch, awaiting assignment
* `DISPATCHED` → Assigned to vehicle, ready to start
* `ON_TRIP` → Active trip in progress
* `HOLD_SAFE` → Temporary safe stop (awaiting resolution)
* `COMPLETED` → Successfully completed trip
* `ABORTED` → Manually terminated trip
* `FAILED` → Trip failed to complete (with reason taxonomy)

## Sector-Specific Trip Types

### Defense

* `OP_RUN`
* `CONVOY_RUN`
* `EMERGENCY_RUN`
* `MAINT_RUN`
* `RELOCATION_RUN`
* `DATA_COURIER_RUN`
* `PATROL_RUN`

### Mining

* `OP_RUN` (loop)
* `SERVICE_RUN` (refuel/lube)
* `MAINT_RUN`
* `MAPPING_RUN`
* `CALIB_RUN`
* `EMERGENCY_RUN`

### Logistics

* `OP_RUN`
* `GATE_RUN`
* `BERTH_SHUTTLE`
* `CROSS_DOCK_RUN`
* `HAZMAT_RUN`
* `RELOCATION_RUN`
* `BACKHAUL_RUN`

### Ride-Hail

* `OP_RUN`
* `STAGING_RUN`
* `AIRPORT_RUN`
* `ACCESSIBILITY_RUN`
* `POOL_RUN`
* `TEST_RUN` (work-zones)

## Use Case Template

All use cases follow this standardized template:

```markdown
# ID — Name

**Actors:** Primary and supporting actors involved
**Trip Type:** Maps to UI (Release/Operation/Mapping/Calibration/Maintenance/Emergency/Test)
**ODD:** Geographic/road/weather/communications constraints
**Trigger:** Event/state/SLA that initiates the use case
**Nominal Flow:** 5-8 steps of the primary flow
**Variants / Edge Cases:** Key variations and edge conditions
**KPIs:** P0/P1 KPIs with thresholds
**Dependencies:** Services and rules required
**Risks & Mitigations:** Key risks and designed-in mitigations
**Acceptance / Test hooks:** Simulation scenarios, logs, gates
```

## Integration with Product Development

Each use case in this repository:

1. Maps to at least one **P0 KPI** in `data/contracts/kpis.yaml`
2. Has corresponding wireframes for key UI components
3. Links to service dependencies with contract tests
4. Includes telemetry schema updates
5. Has CI twin-gates for simulation validation

## Directory Structure

```
docs/prd/use-cases/
├── 00_taxonomy.md                  # This file
├── _TEMPLATE.md                    # Template for new use cases
├── defense/
│   ├── D1_autonomous_fob_resupply_convoy.md
│   ├── D2_last_mile_ammo_water_drop.md
│   └── ...
├── mining/
│   ├── M1_pit_to_crusher_autonomous_haul.md
│   ├── M2_overburden_removal_cycle.md
│   └── ...
├── logistics/
│   ├── L1_yard_switcher_dock_yard.md
│   ├── L2_container_terminal_berth_shuttle.md
│   └── ...
└── ride-hail/
    ├── R1_standard_point_to_point_ride.md
    ├── R2_shared_ride_pool.md
    └── ...
```
