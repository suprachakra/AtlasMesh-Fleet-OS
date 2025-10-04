## A) Personas (canonical table)
| ID | Role | Sector | Goals | Pain points | Top tasks | Key decisions | Primary tools | KPI interests (MET.*) |
|---|---|---|---|---|---|---|---|---|
| PERS.DEF.MISSION_CMD | Mission Commander | DEF | Mission completion, force protection | GNSS denial, ROE dynamics | Approve missions, monitor risk, authorize assists | Route selection, abort/continue, ROE exceptions | Control Center, Intel feeds | MET.SAFETY.MISSION_COMPLETION, MET.SAFETY.ASSIST_RATE |
| PERS.DEF.DISPATCH | Defense Dispatcher | DEF | Orchestrate convoys, SLA | Assist latency, comms blackouts | Assign vehicles, triage incidents | Assign/hold, re-route | Control Center | MET.OPS.DISPATCH_LAT_P95, MET.RELIAB.AVAIL_ODD |
| PERS.DEF.AVIATION_OPS | Airfield Ops Lead | DEF | Zero FOD, runway uptime | Debris detection, window conflicts | Plan sweeps, sector closures | Sweep frequency, closures | Control Center, NOTAM | MET.SAFETY.FOD_EVENTS, MET.RELIAB.AVAIL_ODD |
| PERS.DEF.SECURITY | Force Protection Lead | DEF | Perimeter security, fast response | Blind spots, false alarms | Patrol plans, alarm triage | Dispatch/engage/stand-down | Control Center, VMS | MET.SAFETY.RESPONSE_TTA, MET.SAFETY.FALSE_ALARM_RATE |
| PERS.MIN.DISPATCH | Mine Dispatch | MIN | Tons/hour, safety | Haul road degradation, dust | Assign cycles, slot shovels | Lane priorities, slotting | Control Center, TMS | MET.PROD.TONS_PER_HOUR, MET.SAFETY.ASSIST_RATE |
| PERS.MIN.MAINT_SUP | Maintenance Supervisor | MIN | Uptime, PdM efficacy | Surprise failures, parts delays | Plan service, triage faults | Pull/return to service | Fleet Health, CMMS | MET.RELIAB.AVAIL_ODD, MET.COST.MAINT_$ |
| PERS.MIN.GEO_ENG | Geotechnical Engineer | MIN | Slope safety | Sensor coverage, false positives | Monitor slopes, set thresholds | Alarm bands, exclusions | Geo Dashboards | MET.SAFETY.SLOPE_ALERTS, MET.SAFETY.FALSE_ALARM_RATE |
| PERS.LOG.YARD_SUP | Yard Supervisor | LOG | Gate-to-dock SLAs | Queue opacity, crane idle | Slot assignments, exceptions | Hold/release lanes, re-sequence | Control Center, TOS/WMS | MET.OPS.GATE_TO_DOCK_P95, MET.COST.EMPTY_MILES |
| PERS.LOG.TERMINAL_OPS | Terminal Ops Manager | LOG | Berth productivity | Window conflicts, TEU bottlenecks | Plan shuttles, align berth ops | Crew windows, equipment alloc | Control Center, TOS | MET.PROD.MOVES_PER_HOUR, MET.RELIAB.AVAIL_ODD |
| PERS.LOG.SECURITY | Yard Security Lead | LOG | Incidents↓, coverage↑ | Blind zones, patrol gaps | Patrol routes, incident response | Patrol frequency, escalation | Control Center, VMS | MET.SAFETY.INCIDENT_RATE, MET.SAFETY.RESPONSE_TTA |
| PERS.RH.CITY_MGR | City Program Manager | RH | ETA reliability, compliance | Work zones, public trust | Permit mgmt, incident review | Geofence edits, surge policy | Control Center | MET.EXP.ETA_P95, MET.GOV.AUDIT_BUNDLE_COMPLETE |
| PERS.RH.SUPPORT | Rider Support Lead | RH | CSAT, zero harm | Ambiguity on incidents | Assist scripts, post-incident comms | Refund/credit, escalations | Support Console | MET.EXP.CSAT, MET.SAFETY.ZERO_HARM |

> Extend sector-specific personas in `./personas/*.md`; keep this table as the index of cross-sector roles.

---

## B) Scenarios (index by sector)

### B1) Defense scenarios (D1–D26)
| ID | Title | Given | When | Then (outcome) | Outcome metrics (MET.*) | Links (FR/NFR) | Doc |
|---|---|---|---|---|---|---|---|
| SCN.DEF.D1_CONVOY_RESUPPLY | Autonomous FOB Resupply Convoy | Multi-vehicle convoy in mixed terrain | Dispatch plans route w/ spacing & ROE | Convoy completes without ROE breach | MET.SAFETY.MISSION_COMPLETION, MET.SAFETY.ASSIST_RATE | FR-DEF-001; NFR-DEF-SEC-001 | ../use-cases/defense/D1_autonomous_fob_resupply_convoy.md |
| SCN.DEF.D2_LAST_MILE_DROP | Last-Mile Critical Drop | GNSS degraded urban edge | System executes drop zone | Delivery within window; evidence logged | MET.OPS.TRIP_SUCCESS_RATE, MET.GOV.AUDIT_BUNDLE_COMPLETE | FR-DEF-002; NFR-DEF-GNSS-001 | ../use-cases/defense/D2_last_mile_critical_drop.md |
| SCN.DEF.D3_PERIMETER_PATROL | Base Perimeter Patrol | Perimeter waypoints w/ blind spots | Patrol cycles + anomaly triage | P95 response within SLA | MET.SAFETY.RESPONSE_TTA, MET.SAFETY.FALSE_ALARM_RATE | FR-DEF-003; NFR-DEF-VMS-001 | ../use-cases/defense/D3_base_perimeter_patrol.md |
| SCN.DEF.D4_ROUTE_CLEARANCE | Route Clearance Recon | Suspected IED corridor | Recon runs clearance pattern | Corridor clear/blocked w/ evidence | MET.SAFETY.FALSE_NEG_RATE, MET.GOV.AUDIT_BUNDLE_COMPLETE | FR-DEF-004; NFR-DEF-SAFETY-001 | ../use-cases/defense/D4_route_clearance_recon.md |
| SCN.DEF.D5_MEDEVAC_SHUTTLE | Autonomous MEDEVAC Shuttle | Casualty evac request | Shuttle plans protected corridor | Transfer time within medical SLA | MET.SAFETY.ZERO_HARM, MET.OPS.TRANSFER_TTA | FR-DEF-005; NFR-DEF-PRIV-001 | ../use-cases/defense/D5_autonomous_medevac_shuttle.md |
| SCN.DEF.D6_BORDER_CORRIDOR | Border Corridor Logistics | Long corridor; jamming risk | Fleet executes corridor plan | ≥98% segment completion in ODD | MET.SAFETY.MISSION_COMPLETION, MET.RELIAB.ODD_CONFORMANCE | FR-DEF-006; NFR-DEF-COMMS-001 | ../use-cases/defense/D6_border_corridor_logistics.md |
| SCN.DEF.D7_ENG_SUPPORT_HAUL | Engineering Support Haul | Heavy materials to forward site | Assign heavy-haul w/ grade limits | No over-grade events; on-time arrival | MET.SAFETY.GRADE_VIOLATIONS, MET.OPS.TRIP_SUCCESS_RATE | FR-DEF-007; NFR-DEF-VEH-001 | ../use-cases/defense/D7_engineering_support_haul.md |
| SCN.DEF.D8_FORCE_PROTECTION | Force Protection Perimeter | Elevated threat level | System raises patrol frequency | Incidents ↓; coverage maintained | MET.SAFETY.INCIDENT_RATE, MET.RELIAB.AVAIL_ODD | FR-DEF-008; NFR-DEF-ALERTS-001 | ../use-cases/defense/D8_force_protection_perimeter.md |
| SCN.DEF.D9_COUNTER_UAS | Counter-UAS Operations | UAS incursion detected | Units reposition to intercept | Intercept corridor secured | MET.SAFETY.RESPONSE_TTA | FR-DEF-009; NFR-DEF-INT-001 | ../use-cases/defense/D9_counter_uas_operations.md |
| SCN.DEF.D10_TACT_RECON | Tactical Reconnaissance | Dynamic recon tasking | Vehicle performs low-profile route | Coverage complete; low detectability | MET.OPS.COVERAGE_RATE | FR-DEF-010; NFR-DEF-EMI-001 | ../use-cases/defense/D10_tactical_reconnaissance.md |
| SCN.DEF.D11_SECURE_COURIER | Secure Data Courier | Classified media transfer | Courier runs attested route | Chain-of-custody intact | MET.GOV.CHAIN_OF_CUSTODY | FR-DEF-011; NFR-DEF-CRYPTO-001 | ../use-cases/defense/D11_secure_data_courier.md |
| SCN.DEF.D12_MEDEVAC_AUTON | Autonomous Medical Evacuation | Multiple pickup points | System sequences pickups | Medical SLA met across chain | MET.OPS.TRANSFER_TTA | FR-DEF-012; NFR-DEF-HIPAA-001 | ../use-cases/defense/D12_autonomous_medical_evacuation.md |
| SCN.DEF.D13_FWD_REFUEL | Autonomous Forward Refueling | Forward assets low on fuel | Refueler plans protected path | Refuel window hit; no ROE breach | MET.OPS.TRIP_SUCCESS_RATE | FR-DEF-013; NFR-DEF-HAZ-001 | ../use-cases/defense/D13_autonomous_forward_refueling.md |
| SCN.DEF.D14_COMMS_RELAY | Autonomous Comms Relay | Coverage gap in valley | Relay vehicle holds station | Coverage restored to SLA | MET.OPS.COMMS_UPTIME | FR-DEF-014; NFR-DEF-RF-001 | ../use-cases/defense/D14_autonomous_communications_relay.md |
| SCN.DEF.D15_COUNTER_IED | Counter-IED Route Clearance | IED risk along MSR | Clearance sweep executes | Corridor state w/ evidence | MET.SAFETY.FALSE_NEG_RATE | FR-DEF-015; NFR-DEF-SENSOR-001 | ../use-cases/defense/D15_counter_ied_route_clearance.md |
| SCN.DEF.D16_EW_SUPPORT | Electronic Warfare Support | Jamming present | GNSS fallback engages | Mission continuation w/ SLAM | MET.RELIAB.ODD_CONFORMANCE | FR-DEF-016; NFR-DEF-GNSS-002 | ../use-cases/defense/D16_autonomous_electronic_warfare_support.md |
| SCN.DEF.D17_DISTRO_CENTER | Logistics Distribution Center | High-tempo in/out | Yard orchestrates autonomously | SLA throughput achieved | MET.PROD.MOVES_PER_HOUR | FR-DEF-017; NFR-DEF-TOS-001 | ../use-cases/defense/D17_autonomous_logistics_distribution_center.md |
| SCN.DEF.D18_RUNWAY_FOD | Runway Debris Monitoring | Runway needs sweep | Vehicle runs FOD pattern | Zero missed FOD events | MET.SAFETY.FOD_EVENTS | FR-DEF-018; NFR-DEF-AERO-001 | ../use-cases/defense/D18_autonomous_runway_debris_monitoring.md |
| SCN.DEF.D19_AC_MAINT_SUP | Aircraft Maintenance Support | Parts/tools shuttle | Sequence service bays | Turns completed on time | MET.OPS.TURN_TIME | FR-DEF-019; NFR-DEF-AERO-002 | ../use-cases/defense/D19_aircraft_maintenance_support.md |
| SCN.DEF.D20_TACT_POWER | Tactical Power Distribution | Field power demand spike | Power carts reposition | Outage time minimized | MET.RELIAB.POWER_UPTIME | FR-DEF-020; NFR-DEF-ENERGY-001 | ../use-cases/defense/D20_autonomous_tactical_power_distribution.md |
| SCN.DEF.D21_WATER_PURIF | Tactical Water Purification | Water need at FOB | Unit relocates/operates | Potable output within SLA | MET.OPS.OUTPUT_RATE | FR-DEF-021; NFR-DEF-UTILS-001 | ../use-cases/defense/D21_autonomous_tactical_water_purification.md |
| SCN.DEF.D22_FIELD_REPAIR | Field Repair Support | Asset failure in field | Repair shuttle dispatched | MTTR reduced vs baseline | MET.OPS.MTTR | FR-DEF-022; NFR-DEF-CMMS-001 | ../use-cases/defense/D22_autonomous_field_repair_support.md |
| SCN.DEF.D23_PERIM_RESP | Perimeter Security Response | Alarm from sensor | Nearest unit intercepts | P95 response < SLA | MET.SAFETY.RESPONSE_TTA | FR-DEF-023; NFR-DEF-VMS-002 | ../use-cases/defense/D23_autonomous_perimeter_security_response.md |
| SCN.DEF.D24_LE_INTERACT | Law Enforcement Interaction | Mixed civil-mil corridor | Execute LE protocol | Zero escalation incidents | MET.SAFETY.ZERO_HARM | FR-DEF-024; NFR-DEF-PROTOCOL-001 | ../use-cases/defense/D24_law_enforcement_interaction_protocol.md |
| SCN.DEF.D25_CIV_TRAFFIC | Civilian Traffic Integration | Civilian vehicles present | Yield/communicate appropriately | Smooth merges; no incidents | MET.SAFETY.INCIDENT_RATE | FR-DEF-025; NFR-DEF-HMI-001 | ../use-cases/defense/D25_civilian_traffic_integration.md |
| SCN.DEF.D26_RUGGED_TERRAIN | Rugged Terrain Mobility Support | Loose sand/ruts/rock; steep grades | Apply terrain policy & traction control | No traction/grade violations | MET.SAFETY.GRADE_VIOLATIONS, MET.RELIAB.AVAIL_ODD | FR-DEF-026; NFR-DEF-TERRAIN-001 | ../use-cases/defense/D26_rugged_terrain_mobility_support.md |

### B2) Logistics scenarios (L1–L20)
| ID | Title | Given | When | Then (outcome) | Outcome metrics (MET.*) | Links (FR/NFR) | Doc |
|---|---|---|---|---|---|---|---|
| SCN.LOG.L1_YARD_SWITCHER | Yard Switcher: Dock & Yard | Peak arrivals | Auto-assign dock slots | P95 gate-to-dock ≤ target | MET.OPS.GATE_TO_DOCK_P95, MET.COST.EMPTY_MILES | FR-LOG-001; NFR-LOG-TOS-001 | ../use-cases/logistics/L1_yard_switcher_dock_yard.md |
| SCN.LOG.L2_BERTH_SHUTTLE | Container Terminal Berth Shuttle | Berth window active | Plan berth↔yard shuttles | Moves/hour ≥ plan | MET.PROD.MOVES_PER_HOUR | FR-LOG-002; NFR-LOG-CRANE-001 | ../use-cases/logistics/L2_container_terminal_berth_shuttle.md |
| SCN.LOG.L3_CROSS_DOCK_WAVE | Cross-Dock Wave Fulfillment | Wave drop | Sequence docks | Wave completes on time | MET.OPS.WAVE_TTA | FR-LOG-003; NFR-LOG-WMS-001 | ../use-cases/logistics/L3_cross_dock_wave_fulfillment.md |
| SCN.LOG.L4_HUB2HUB | Hub-to-Hub Corridor | Long corridor & tolls | Optimize route/timing | On-time arrival; cost ↓ | MET.OPS.OTP, MET.COST.$_PER_MOVE | FR-LOG-004; NFR-LOG-COMMS-001 | ../use-cases/logistics/L4_hub_to_hub_corridor.md |
| SCN.LOG.L5_COLD_CHAIN_SHUTTLE | Cold Chain Shuttle | Temp-sensitive load | Plan shuttle | Temp compliance maintained | MET.QUAL.TEMP_EXCURSIONS | FR-LOG-005; NFR-LOG-COLD-001 | ../use-cases/logistics/L5_cold_chain_shuttle.md |
| SCN.LOG.L6_TERMINAL_EQUIP | Terminal Equipment Shuttle | RTG/STS support | Feed equipment JIT | Idle time ↓ vs baseline | MET.PROD.EQUIP_IDLE_TIME | FR-LOG-006; NFR-LOG-SYNC-001 | ../use-cases/logistics/L6_port_terminal_equipment_shuttle.md |
| SCN.LOG.L7_YARD_OPTIM | Terminal Yard Optimization | Yard congestion | Rebalance lanes | Queue variance ↓ | MET.OPS.QUEUE_VAR, MET.OPS.THROUGHPUT | FR-LOG-007; NFR-LOG-ALGO-001 | ../use-cases/logistics/L7_terminal_yard_optimization.md |
| SCN.LOG.L8_EMPTY_REPO | Empty Container Repositioning | Mismatch of empties | Plan reposition moves | Empty reposition cost ↓ | MET.COST.EMPTY_MILES | FR-LOG-008; NFR-LOG-NETWORK-001 | ../use-cases/logistics/L8_empty_container_repositioning.md |
| SCN.LOG.L9_RAIL_CONN | Intermodal Rail Connection | Train ETA confirmed | Sync yard↔rail handoff | Missed handoffs → 0 | MET.OPS.MISSED_HANDOFFS | FR-LOG-009; NFR-LOG-ETA-001 | ../use-cases/logistics/L9_intermodal_rail_connection.md |
| SCN.LOG.L10_CUSTOMS | Cross-Border Customs Transit | Border queue & checks | Follow customs overlay | Clearance time within SLA | MET.OPS.CLEARANCE_TIME | FR-LOG-010; NFR-LOG-COMPLIANCE-001 | ../use-cases/logistics/L10_cross_border_customs_transit.md |
| SCN.LOG.L11_HAZMAT | Hazardous Materials Transport | Hazmat constraints | Use hazmat policy routes | No policy violations | MET.SAFETY.HAZMAT_VIOL, MET.OPS.OTP | FR-LOG-011; NFR-LOG-HAZ-001 | ../use-cases/logistics/L11_hazardous_materials_transport.md |
| SCN.LOG.L12_XDOCK_TRANSFER | Autonomous Cross-Dock Transfer | Pallet moves between docks | Auto-schedule transfers | Transfers complete within window | MET.OPS.TRANSFER_TTA | FR-LOG-012; NFR-LOG-WMS-002 | ../use-cases/logistics/L12_autonomous_cross_dock_transfer.md |
| SCN.LOG.L13_REEFER_TRANSPORT | Autonomous Refrigerated Transport | Reefers outbound | Schedule loads & routes | No temp excursions | MET.QUAL.TEMP_EXCURSIONS | FR-LOG-013; NFR-LOG-COLD-002 | ../use-cases/logistics/L13_autonomous_refrigerated_transport.md |
| SCN.LOG.L14_RETURNS | Autonomous Returns Processing | High returns volume | Auto-route to stations | Backlog cleared within SLA | MET.OPS.RETURNS_BACKLOG_TTA | FR-LOG-014; NFR-LOG-RMA-001 | ../use-cases/logistics/L14_autonomous_returns_processing.md |
| SCN.LOG.L15_CYCLE_COUNT | Autonomous Inventory Cycle Count | Inventory window | Run scan/count routes | Accuracy ≥ target | MET.QUAL.COUNT_ACCURACY | FR-LOG-015; NFR-LOG-SCAN-001 | ../use-cases/logistics/L15_autonomous_inventory_cycle_count.md |
| SCN.LOG.L16_ORDER_FULFILL | Autonomous Order Fulfillment | Wave assigned | Orchestrate pick-pack-move | Orders fulfilled on time | MET.OPS.OTP | FR-LOG-016; NFR-LOG-WMS-003 | ../use-cases/logistics/L16_autonomous_order_fulfillment.md |
| SCN.LOG.L17A_PARCEL_SORT | Autonomous Parcel Sorting | Peak parcels | Auto-sort sequence | Sort accuracy/throughput hit | MET.PROD.SORT_TPH, MET.QUAL.SORT_ACCURACY | FR-LOG-017A; NFR-LOG-SORT-001 | ../use-cases/logistics/L17_autonomous_parcel_sorting.md |
| SCN.LOG.L17B_REVERSE_LOG | Autonomous Reverse Logistics | Returns consolidation | Plan reverse lanes | Fill rate ↑; cost/return ↓ | MET.COST.$_PER_RETURN, MET.OPS.FILL_RATE | FR-LOG-017B; NFR-LOG-RMA-002 | ../use-cases/logistics/L17_autonomous_reverse_logistics.md |
| SCN.LOG.L18A_COLD_MON | Autonomous Cold Chain Monitoring | Reefer fleet in yard | Monitor + alert | Excursions detected early | MET.QUAL.TEMP_EXCURSIONS, MET.SAFETY.ALERT_LAT_P95 | FR-LOG-018A; NFR-LOG-IOT-001 | ../use-cases/logistics/L18_autonomous_cold_chain_monitoring.md |
| SCN.LOG.L18B_DOCK_SCHED | Autonomous Dock Scheduling | Over-subscribed docks | Optimize schedule | Dwell time ↓; on-time ↑ | MET.OPS.DWELL_TIME, MET.OPS.OTP | FR-LOG-018B; NFR-LOG-SCHED-001 | ../use-cases/logistics/L18_autonomous_dock_scheduling.md |
| SCN.LOG.L19A_FAC_MAINT | Autonomous Facility Maintenance | Preventive patrol needed | Patrol & log issues | MTBF ↑; WOs raised | MET.RELIAB.MTBF, MET.OPS.WO_TTA | FR-LOG-019A; NFR-LOG-CMMS-001 | ../use-cases/logistics/L19_autonomous_facility_maintenance.md |
| SCN.LOG.L19B_FREIGHT_CONS | Autonomous Freight Consolidation | LTL→FTL plan | Auto-stage consolidation | Trips ↓; fill rate ↑ | MET.COST.TRIPS_PER_TON, MET.OPS.FILL_RATE | FR-LOG-019B; NFR-LOG-WMS-004 | ../use-cases/logistics/L19_autonomous_freight_consolidation.md |
| SCN.LOG.L20_YARD_SECURITY | Autonomous Yard Security Patrol | Night shift | Patrol + anomaly alerts | Incident rate ↓ | MET.SAFETY.INCIDENT_RATE | FR-LOG-020; NFR-LOG-VMS-003 | ../use-cases/logistics/L20_autonomous_yard_security_patrol.md |

### B3) Mining scenarios (M1–M25)
| ID | Title | Given | When | Then (outcome) | Outcome metrics (MET.*) | Links (FR/NFR) | Doc |
|---|---|---|---|---|---|---|---|
| SCN.MIN.M1_PIT2CRUSH | Pit-to-Crusher Haul | Fixed-graph haul | Assign cycles | Tons/hour ≥ plan | MET.PROD.TONS_PER_HOUR, MET.SAFETY.ASSIST_RATE | FR-MIN-001; NFR-MIN-GRADE-001 | ../use-cases/mining/M1_pit_to_crusher_autonomous_haul.md |
| SCN.MIN.M2_OVERBURDEN | Overburden Removal Cycle | Blast window set | Orchestrate removal | Cycle time ≤ target | MET.PROD.CYCLE_TIME | FR-MIN-002; NFR-MIN-SAFETY-001 | ../use-cases/mining/M2_overburden_removal_cycle.md |
| SCN.MIN.M3_SERVICE_RUNS | Refuel/Recharge & Sweeper | Low SOC & dust | Schedule service runs | Availability maintained | MET.RELIAB.AVAIL_ODD | FR-MIN-003; NFR-MIN-ENERGY-001 | ../use-cases/mining/M3_autonomous_refuel_recharge_sweeper.md |
| SCN.MIN.M4_STOCKPILE | Stockpile Reclaim & Grade Control | Reclaim order | Execute reclaim | Grade variance ≤ target | MET.QUAL.GRADE_VAR | FR-MIN-004; NFR-MIN-SENSOR-001 | ../use-cases/mining/M4_stockpile_reclaim_grade_control.md |
| SCN.MIN.M5_ROAD_PATROL | Haul Road Condition Patrol | Road degradation | Patrol + score | Alerts & reroutes issued | MET.SAFETY.ROAD_ALERTS, MET.OPS.REROUTE_TTA | FR-MIN-005; NFR-MIN-ML-001 | ../use-cases/mining/M5_haul_road_condition_patrol.md |
| SCN.MIN.M6_DRILL_SUPPORT | Blast Pattern Drilling Support | Pattern plan | Stage materials & mark | Drilling starts on time | MET.OPS.START_TTA | FR-MIN-006; NFR-MIN-EVID-001 | ../use-cases/mining/M6_blast_pattern_drilling_support.md |
| SCN.MIN.M7_SHOVEL_TRUCK | Shovel–Truck Coordination | Several shovels active | Slot trucks to shovels | Queue time ↓; throughput ↑ | MET.PROD.QUEUE_TIME, MET.PROD.TONS_PER_HOUR | FR-MIN-007; NFR-MIN-RADIO-001 | ../use-cases/mining/M7_shovel_truck_coordination.md |
| SCN.MIN.M8_TAILINGS_INSPECT | Tailings Dam Inspection | Daily inspection | Patrol & capture | Compliance evidence ready | MET.GOV.AUDIT_BUNDLE_COMPLETE | FR-MIN-008; NFR-MIN-ENV-001 | ../use-cases/mining/M8_tailings_dam_inspection.md |
| SCN.MIN.M9_ENV_SWEEP | Environmental Monitoring Sweep | Air/noise reqs | Sweep & log | Variance within limits | MET.QUAL.ENV_VAR | FR-MIN-009; NFR-MIN-SENSOR-002 | ../use-cases/mining/M9_environmental_monitoring_sweep.md |
| SCN.MIN.M10_TAILINGS_MON | Tailings Facility Monitoring | Geotech sensors live | Monitor & alert | Response within SLA | MET.SAFETY.RESPONSE_TTA | FR-MIN-010; NFR-MIN-IOT-001 | ../use-cases/mining/M10_tailings_facility_monitoring.md |
| SCN.MIN.M11_EXP_DELIVERY | Explosive Delivery | Blast plan | Deliver explosives | Delivery within window | MET.OPS.TRIP_SUCCESS_RATE | FR-MIN-011; NFR-MIN-HAZ-001 | ../use-cases/mining/M11_autonomous_explosive_delivery.md |
| SCN.MIN.M12_GRADE_SAMPLING | Grade Control Sampling | Sampling plan | Execute sampling route | Samples logged & traced | MET.GOV.CHAIN_OF_CUSTODY | FR-MIN-012; NFR-MIN-LAB-001 | ../use-cases/mining/M12_autonomous_grade_control_sampling.md |
| SCN.MIN.M13_DEWATER | Dewatering Pump Management | Rising water | Reposition pumps | Water level within band | MET.OPS.WATER_LEVEL_BAND | FR-MIN-013; NFR-MIN-UTILS-001 | ../use-cases/mining/M13_autonomous_dewatering_pump_management.md |
| SCN.MIN.M14_CORE_TRANSPORT | Drill Core Transport | Core ready at rig | Shuttle to core shed | Chain-of-custody intact | MET.GOV.CHAIN_OF_CUSTODY | FR-MIN-014; NFR-MIN-HANDLING-001 | ../use-cases/mining/M14_autonomous_drill_core_transport.md |
| SCN.MIN.M15_SHIFT_SHUTTLE | Shift Change Shuttle | Shift window | Shuttle crews | On-time crew arrivals | MET.OPS.OTP | FR-MIN-015; NFR-MIN-HSE-001 | ../use-cases/mining/M15_autonomous_shift_change_shuttle.md |
| SCN.MIN.M16_DUST_SUPPRESS | Dust Suppression | Dust index ↑ | Spray routes run | Visibility within band | MET.SAFETY.VISIBILITY_BAND | FR-MIN-016; NFR-MIN-WATER-001 | ../use-cases/mining/M16_autonomous_dust_suppression.md |
| SCN.MIN.M17_SLOPE_MON | Slope Stability Monitoring | Rainfall period | High-freq scans | Early warnings issued | MET.SAFETY.SLOPE_ALERTS | FR-MIN-017; NFR-MIN-GEOSCAN-001 | ../use-cases/mining/M17_autonomous_slope_stability_monitoring.md |
| SCN.MIN.M18_EXP_TRANSPORT | Explosive Transport | Magazine→pattern | Secure transit | No violations; on-time | MET.SAFETY.HAZMAT_VIOL, MET.OPS.OTP | FR-MIN-018; NFR-MIN-HAZ-002 | ../use-cases/mining/M18_autonomous_explosive_transport.md |
| SCN.MIN.M19_RESCUE_SUPPORT | Mine Rescue Support | Incident declared | Deliver gear; set beacons | MTTR reduced | MET.OPS.MTTR | FR-MIN-019; NFR-MIN-EMERG-001 | ../use-cases/mining/M19_autonomous_mine_rescue_support.md |
| SCN.MIN.M20_RECLAMATION | Reclamation Monitoring | Post-closure sweep | Patrol & log | Evidence package ready | MET.GOV.AUDIT_BUNDLE_COMPLETE | FR-MIN-020; NFR-MIN-ENV-002 | ../use-cases/mining/M20_autonomous_reclamation_monitoring.md |
| SCN.MIN.M21_FUEL_LUBE | Autonomous Fuel & Lube Service | Fleet needs service | Schedule on-route service | Availability preserved | MET.RELIAB.AVAIL_ODD, MET.COST.MAINT_$ | FR-MIN-021; NFR-MIN-SAFETY-002 | ../use-cases/mining/M21_autonomous_fuel_and_lube_service.md |
| SCN.MIN.M22_TIRE_MON | Autonomous Tire Monitoring Service | High wear risk | Inspect & flag | Unplanned stops ↓ | MET.RELIAB.MTBF, MET.SAFETY.INCIDENT_RATE | FR-MIN-022; NFR-MIN-IOT-002 | ../use-cases/mining/M22_autonomous_tire_monitoring_service.md |
| SCN.MIN.M23_CONVEYOR_INSPECT | Autonomous Conveyor Inspection | Conveyor run window | Patrol belts & rollers | Faults detected early | MET.RELIAB.MTBF, MET.OPS.WO_TTA | FR-MIN-023; NFR-MIN-CMMS-002 | ../use-cases/mining/M23_autonomous_conveyor_inspection.md |
| SCN.MIN.M24_VENT_MON | Autonomous Ventilation Monitoring | Underground vent ops | Survey airflow/quality | Air quality within band | MET.QUAL.AIR_Q_BAND, MET.SAFETY.ALERT_LAT_P95 | FR-MIN-024; NFR-MIN-ENV-003 | ../use-cases/mining/M24_autonomous_ventilation_monitoring.md |
| SCN.MIN.M25_UG_MAPPING | Autonomous Underground Mapping | New headings | Map & localize | Coverage ≥ target; updates pushed | MET.OPS.COVERAGE_RATE, MET.GOV.AUDIT_BUNDLE_COMPLETE | FR-MIN-025; NFR-MIN-MAP-001 | ../use-cases/mining/M25_autonomous_underground_mapping.md |

### B4) Ride-hail scenarios (R1–R20)
| ID | Title | Given | When | Then (outcome) | Outcome metrics (MET.*) | Links (FR/NFR) | Doc |
|---|---|---|---|---|---|---|---|
| SCN.RH.R1_STD_RIDE | Standard Point-to-Point Ride | Rider requests trip | Vehicle dispatched | OTP; smooth pickup/drop | MET.EXP.ETA_P95, MET.OPS.OTP | FR-RH-001; NFR-RH-HMI-001 | ../use-cases/ride-hail/R1_standard_point_to_point_ride.md |
| SCN.RH.R2_POOL | Shared Ride Pool | Multiple riders nearby | Pooling enabled | High fill rate; minimal detour | MET.OPS.FILL_RATE, MET.EXP.DETOUR_P95 | FR-RH-002; NFR-RH-HMI-002 | ../use-cases/ride-hail/R2_shared_ride_pool.md |
| SCN.RH.R3_AIRPORT | Airport Pickup/Drop Staging | Airport geofence | Follow staging protocol | Compliance; wait ≤ SLA | MET.EXP.ETA_P95, MET.GOV.AUDIT_BUNDLE_COMPLETE | FR-RH-003; NFR-RH-AIR-001 | ../use-cases/ride-hail/R3_airport_pickup_drop_staging.md |
| SCN.RH.R4_ACCESS | Accessibility Ride | PRM request | Accessibility flow engaged | Completion ≥ 98% | MET.EXP.ACCESS_FULFILL, MET.SAFETY.ZERO_HARM | FR-RH-004; NFR-RH-ADA-001 | ../use-cases/ride-hail/R4_accessibility_ride.md |
| SCN.RH.R5_NIGHT_ESCORT | Night Safety Escort | Night hours | Safety overlay applied | Incident rate ↓; CSAT ↑ | MET.SAFETY.INCIDENT_RATE, MET.EXP.CSAT | FR-RH-005; NFR-RH-PRIV-001 | ../use-cases/ride-hail/R5_night_safety_escort.md |
| SCN.RH.R6_COMMUTER | Scheduled Commuter Service | Repeating schedule | Pre-dispatch | OTP ≥ target; no-shows ↓ | MET.OPS.OTP, MET.EXP.NO_SHOW_RATE | FR-RH-006; NFR-RH-SCHED-001 | ../use-cases/ride-hail/R6_scheduled_commuter_service.md |
| SCN.RH.R7_EVENT | Special Event Transport | Event geofence | Surge + staging policy | Throughput ↑; wait ≤ target | MET.OPS.THROUGHPUT, MET.EXP.ETA_P95 | FR-RH-007; NFR-RH-SURGE-001 | ../use-cases/ride-hail/R7_special_event_transport.md |
| SCN.RH.R8_LE_INTERACT | Law Enforcement Interaction | LE stop | Protocol executed | Zero escalation incidents | MET.SAFETY.ZERO_HARM | FR-RH-008; NFR-RH-PROTOCOL-001 | ../use-cases/ride-hail/R8_law_enforcement_interaction.md |
| SCN.RH.R9_NEMT | Non-Emergency Medical Transport | Medical appointment | NEMT overlay | On-time arrival; comfort | MET.OPS.OTP, MET.EXP.CSAT | FR-RH-009; NFR-RH-PRIV-002 | ../use-cases/ride-hail/R9_medical_transport_non_emergency.md |
| SCN.RH.R10_MM_CONNECT | Multi-Modal Transit Connection | Transit leg | Sync pickup/drop | Missed connections ↓ | MET.OPS.MISSED_HANDOFFS | FR-RH-010; NFR-RH-ETA-001 | ../use-cases/ride-hail/R10_multi_modal_transit_connection.md |
| SCN.RH.R11_EVAC | Emergency Evacuation Transport | Evac order | Priority routing | Evac completed safely | MET.SAFETY.ZERO_HARM, MET.OPS.COVERAGE_RATE | FR-RH-011; NFR-RH-EMERG-001 | ../use-cases/ride-hail/R11_emergency_evacuation_transport.md |
| SCN.RH.R12_AIRPORT_QUEUE | Airport Queue Management | Queue congestion | Queue algorithm runs | Wait variance ↓ | MET.OPS.QUEUE_VAR, MET.EXP.ETA_P95 | FR-RH-012; NFR-RH-AIR-002 | ../use-cases/ride-hail/R12_autonomous_airport_queue_management.md |
| SCN.RH.R13_PASSENGER_ASSIST | Autonomous Passenger Assistance | Rider needs help | Assist workflow triggers | Resolution within SLA | MET.SAFETY.ASSIST_LAT_P95, MET.EXP.CSAT | FR-RH-013; NFR-RH-HMI-003 | ../use-cases/ride-hail/R13_autonomous_passenger_assistance.md |
| SCN.RH.R14_MM_CONNECT2 | Autonomous Multi-Modal Connection | Timed transfer | Smart staging | Connection reliability ↑ | MET.OPS.MISSED_HANDOFFS | FR-RH-014; NFR-RH-ETA-002 | ../use-cases/ride-hail/R14_autonomous_multi_modal_connection.md |
| SCN.RH.R15_PKG | Autonomous Package Delivery | Parcel request | Combine with rides | $/delivery ↓; OTP ↑ | MET.COST.$_PER_DELIVERY, MET.OPS.OTP | FR-RH-015; NFR-RH-HAZ-001 | ../use-cases/ride-hail/R15_autonomous_package_delivery.md |
| SCN.RH.R16_TOUR | Autonomous Tourism Experience | Tourist route | Scenic overlay | CSAT ≥ target | MET.EXP.CSAT | FR-RH-016; NFR-RH-HMI-004 | ../use-cases/ride-hail/R16_autonomous_tourism_experience.md |
| SCN.RH.R17_CARPOOL | Autonomous Carpooling Service | Cohort defined | Pre-match carpools | Fill rate ↑; detour ≤ SLA | MET.OPS.FILL_RATE, MET.EXP.DETOUR_P95 | FR-RH-017; NFR-RH-SCHED-002 | ../use-cases/ride-hail/R17_autonomous_carpooling_service.md |
| SCN.RH.R18_SUBSCRIPTION | Autonomous Subscription Service | Subscriber plan | Reserved capacity applied | SLA adherence; churn ↓ | MET.EXP.SLA_ADHERENCE, MET.BIZ.CHURN_RATE | FR-RH-018; NFR-RH-BILL-001 | ../use-cases/ride-hail/R18_autonomous_subscription_service.md |
| SCN.RH.R19_CAMPUS | Autonomous Campus Shuttle | Campus geofence | Loop service runs | Headway variance ≤ target | MET.OPS.HEADWAY_VAR | FR-RH-019; NFR-RH-GEOF-001 | ../use-cases/ride-hail/R19_autonomous_campus_shuttle.md |
| SCN.RH.R20_NIGHT_SAFE | Autonomous Night Safety Service | Night hours | Safety overlay | Incident rate ↓ | MET.SAFETY.INCIDENT_RATE | FR-RH-020; NFR-RH-PRIV-003 | ../use-cases/ride-hail/R20_autonomous_night_safety_service.md |

---

## C) Coverage matrix (persona × scenario groups)
> Rows = personas; columns = scenario groups. “●” = frequent interaction/ownership.

| Persona \ Scenario group | DEF (convoy/protection) | DEF (airfield/infra) | MIN (production) | MIN (safety/env) | LOG (yard/terminal) | LOG (corridor/cross-border) | RH (district ops) |
|---|:--:|:--:|:--:|:--:|:--:|:--:|:--:|
| PERS.DEF.MISSION_CMD | ● | ● |  |  |  |  |  |
| PERS.DEF.DISPATCH | ● | ● |  |  |  |  |  |
| PERS.DEF.AVIATION_OPS |  | ● |  |  |  |  |  |
| PERS.DEF.SECURITY | ● | ● |  |  |  |  |  |
| PERS.MIN.DISPATCH |  |  | ● | ● |  |  |  |
| PERS.MIN.MAINT_SUP |  |  | ● |  |  |  |  |
| PERS.MIN.GEO_ENG |  |  |  | ● |  |  |  |
| PERS.LOG.YARD_SUP |  |  |  |  | ● |  |  |
| PERS.LOG.TERMINAL_OPS |  |  |  |  | ● | ● |  |
| PERS.LOG.SECURITY |  |  |  |  | ● |  |  |
| PERS.RH.CITY_MGR |  |  |  |  |  |  | ● |
| PERS.RH.SUPPORT |  |  |  |  |  |  | ● |

---

## D) Conventions & ID schema
- **Scenario IDs**: `SCN.<SECTOR>.<SHORT_CODE>` where `<SECTOR>` ∈ {DEF, MIN, LOG, RH}.  
- **Defense IDs**: D1…D26; **Mining**: M1…M25; **Logistics**: L1…L20 (split variants with A/B); **Ride-hail**: R1…R20.  
- **Paths**: `../use-cases/<sector>/<file>.md` (ensure files exist).  
- **Each scenario must include**: ≥1 FR, ≥1 NFR, ≥1 MET.* reference.  
- **Additions**: extend in sector folders; update this index and coverage matrix.

---