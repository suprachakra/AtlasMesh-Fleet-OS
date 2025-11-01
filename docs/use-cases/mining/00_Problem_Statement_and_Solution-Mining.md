# 00_Problem_Statement_and_Solution_MINING.md

> **Sector:** Mining
> **Scope of this page:** State the problem we're solving in mining and how AtlasMesh Fleet Management System solves it. Details for each use case live in `docs/use-cases/mining/` (M1–M25).

---

## Problem Statement (Mining)

Open-pit and underground mines run on thin margins and tight SLAs. Existing vehicle fleets are manual and inefficient, but retrofitting them with autonomy is blocked by:

* **Fleet Management Gaps.** Manual vehicle dispatch, no real-time fleet monitoring, inefficient haul cycle management, and lack of unified fleet operations across mining equipment.
* **Integration Complexity.** Mining FMS (Wenco/Hexagon), PLCs, SCADA, mine planning systems are siloed; integrations take months; existing fleet management systems don't integrate with customer systems.
* **Operational Inefficiency.** Manual vehicle operations, no predictive maintenance, energy waste, and compliance gaps that require manual intervention and evidence collection.
* **Environmental Challenges.** 50–60 °C heat, dust occlusion, night glare, rugged terrain, and changing geometry that require specialized fleet management capabilities.
* **Safety & Compliance.** Manual safety monitoring, ventilation/gas alerts, slope movement watch, blast zone rules, and audit trails that need automated fleet management integration.

**Business impact:** 
- **Fleet Utilization**: 60% average vehicle utilization
- **Manual Operations**: 5-8 operators per shift for vehicle management
- **Integration Costs**: $1M+ per mine for system integration
- **Energy Waste**: 25-35% energy waste due to inefficient routing
- **Maintenance Costs**: 50% higher maintenance costs due to reactive approach

---

## Solution (Mining overlay on AtlasMesh Fleet Management System)

AtlasMesh provides a **vehicle-agnostic Fleet Management System** with a **mining overlay**: rules/ODD, dispatch, routing, assist Q&A, evidence automation, and integrations. No forks; all mine specifics live in configs and policy packs.

### What we deliver (mining-relevant capabilities)

* **Haul cycle orchestration:** shovel–truck–crusher dispatch, anti-bunching, queue time control, and re-route on road degradation.
* **Terrain-aware autonomy:** grade/surface classification, speed derating, berm detection, water-hazard avoidance, adaptive path smoothing.
* **Heat/dust resilience:** dust-aware perception configs, thermal budgets, heat-aware shift scheduling, energy/fuel optimization.
* **Safety & compliance automation:** ventilation/gas alerts, slope movement watch, blast zone rules, audit bundles per release; underground **ventilation monitoring** and **10 mm mapping** specs are first-class test hooks in CI  .
* **Predictive maintenance & tire health:** sensor fusion for RUL, tire pressure/temperature trends, auto-ticketing to maintenance.
* **Integrations:** FMS, mine planning (designs/blocks), SCADA/PLC for pumps/fans, weighbridges, access control.
* **Assist Q&A (no tele-drive):** scripted triage for edge cases; assist budgets enforced per ODD.

### What “good” looks like (targets for Yr-1 unless noted)

* **Throughput:** **+8–12%** tons/hour on steady-state pits.
* **Availability (ODD):** **≥99.3%** rolling 30-day.
* **Assist rate:** **≤0.5 / 1,000 km** (Yr-1), **≤0.3** (Yr-2).
* **Queueing:** crusher/shovel wait **−15–25%**.
* **Downtime:** **−20–30%** maintenance cost via PdM.
* **Underground:** ventilation alert **≤2 min** detect→notify; mapping precision **≤10 mm**, 100% critical coverage  .

---

## Use-Case Inventory (Mining M1–M25)

> One-liners and primary success KPI per use case. Full specs live beside each `M*.md`.

### **Fleet Management Operations**
| ID      | Name                                  | Problem (one-liner)                       | Primary KPI / Outcome                               |
| ------- | ------------------------------------- | ----------------------------------------- | --------------------------------------------------- |
| **M21** | Fleet Monitoring & Analytics          | No real-time fleet visibility             | Fleet utilization **+15%**, downtime **−30%**       |
| **M22** | Fleet Dispatch & Routing              | Manual dispatch inefficiency              | Dispatch time **−40%**, fuel efficiency **+20%**    |
| **M23** | Fleet Maintenance Management          | Reactive maintenance costs                | Maintenance cost **−25%**, uptime **+10%**          |
| **M24** | Fleet Energy Management               | Energy waste in operations                | Energy efficiency **+25%**, fuel cost **−20%**      |
| **M25** | Fleet Performance Analytics           | No operational insights                   | Decision speed **+50%**, optimization **+15%**       |

### **Mining FMS Integration Workflows**
| ID      | Name                                  | Problem (one-liner)                       | Primary KPI / Outcome                               |
| ------- | ------------------------------------- | ----------------------------------------- | --------------------------------------------------- |
| **M26** | Wenco FMS Integration                 | Siloed fleet management systems           | Integration time **−80%**, data accuracy **+95%**   |
| **M27** | MineStar Integration                  | Manual data handoffs                      | Data sync **+99%**, processing time **−60%**        |
| **M28** | Modular Mining Integration            | Fragmented operations                     | Operational efficiency **+20%**, errors **−90%**     |
| **M29** | SCADA/PLC Integration                 | Manual equipment monitoring               | Response time **−70%**, automation **+85%**         |

### **Core Mining Operations**
| ID      | Name                                  | Problem (one-liner)                       | Primary KPI / Outcome                               |
| ------- | ------------------------------------- | ----------------------------------------- | --------------------------------------------------- |
| **M1**  | Pit-to-Crusher Autonomous Haul        | Haul cycle volatility & queue spikes      | Tons/hour **+8–12%**, Queue P95 **−20%**            |
| **M2**  | Overburden Removal Cycle              | Cycle time drift from path degradation    | Cycle time variance **−15%**                        |
| **M3**  | Autonomous Refuel/Recharge/Sweeper    | Idle time for refuel/cleanup              | Refuel dwell **−25%**, sweep SLA **≥95%**           |
| **M4**  | Stockpile Reclaim & Grade Control     | Over/under-reclaim, grade errors          | Grade conformance **≥98%**                          |
| **M5**  | Haul Road Condition Patrol            | Rutting/washboarding increases assists    | ODD breaches auto-flagged, assists **−30%**         |
| **M6**  | Blast Pattern Drilling Support        | Misalignments cause rework/safety risk    | Hole position error **≤X cm**                       |
| **M7**  | Shovel–Truck Coordination             | Bunching and idle at shovels              | Bunching events **−30%**, shovel idle **−20%**      |
| **M8**  | Tailings Dam Inspection               | Missed anomalies increase risk            | Inspection coverage **100%**, alert MTTA **≤2 min** |
| **M9**  | Environmental Monitoring Sweep        | Dust/noise exceedances missed             | Exceedance detect→notify **≤2 min**                 |
| **M10** | Tailings Facility Monitoring          | Stability metrics unmanaged               | Risk threshold breaches **0 missed**                |
| **M11** | Autonomous Explosive Delivery         | Human exposure & delays                   | On-time delivery **≥98%**, exposure **↓**           |
| **M12** | Grade Control Sampling                | Sampling inefficiency                     | Sample cycle time **−20%**                          |
| **M13** | Dewatering Pump Management            | Flood risk from pump failures             | Uptime **≥99.5%**, response **≤5 min**              |
| **M14** | Drill Core Transport                  | Core chain-of-custody delays              | Turnaround **−30%**, loss **0**                     |
| **M15** | Shift-Change Shuttle                  | Exposure and lost minutes                 | Shuttle punctuality **≥98%**                        |
| **M16** | Dust Suppression                      | Reactive spraying wastes water            | PM exceedance minutes **−40%**, water **−20%**      |
| **M17** | Slope Stability Monitoring            | Late detection of movement                | Movement detect latency **≤5 min**                  |
| **M18** | Explosive Transport                   | Routing & custody risk                    | Compliance incidents **0**, ETA P95 **≤target**     |
| **M19** | Mine Rescue Support                   | Slow gear staging in emergencies          | Time-to-staged resources **−30%**                   |
| **M20** | Reclamation Monitoring                | Sparse audits miss regressions            | Visit frequency **+2×**, gaps **0**                 |
| **M21** | Fuel & Lube Service                   | Unsynchronized service windows            | Service conflicts **−50%**, idle **−20%**           |
| **M22** | Tire Monitoring Service               | Tire failures drive downtime              | Unplanned tire downtime **−30%**                    |
| **M23** | Conveyor Inspection                   | Missed faults cause line stops            | Fault lead-time **+48–72 h**                        |
| **M24** | **Autonomous Ventilation Monitoring** | Manual rounds miss gas/airflow excursions | Alert **≤2 min**, critical coverage **100%**        |
| **M25** | **Autonomous Underground Mapping**    | Infrequent surveys, unsafe exposure       | **≤10 mm** precision, coverage **100%**             |

---

## Solution

**How we solve the above, in practice**

1. **Plan & Dispatch.** Multi-objective haul routing, queue control, fan/pump tasks, patrols.
2. **ODD & Policy.** Mine-specific ODD (grades, berms, water), blast zones, ventilation thresholds; enforced in policy engine.
3. **Perception & Terrain.** Dust-aware configs, terrain classification, hazard avoidance; graceful degradation with assist budgets.
4. **Underground packs.** Mesh-comms tolerant missions, **ventilation** programs, **survey-grade mapping** with QA hooks built into CI (scenarios, coverage checks)  .
5. **Data & PdM.** Telemetry contracts, tire/asset models, drift sentry; auto-tickets to maintenance.
6. **Evidence-as-code.** Every release ships audit bundles for safety/environmental compliance; mine change logs captured.

**Done =** targets above are met for 90-day windows with green audit bundles and zero critical safety incidents.

---

**Notes**

* This page is intentionally narrow: **Problem → Solution** for mining. Architecture, UI, and broader strategy live elsewhere.
* All mine-specific logic is policy/config; **no single-tenant forks**.

**Internal references**
Ventilation monitoring spec (M24)  • Underground mapping spec (M25) 

---
