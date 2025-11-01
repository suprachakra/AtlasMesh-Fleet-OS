# 00_Problem_Statement_and_Solution_LOGISTICS.md

> **Purpose.** Define the logistics sector problem we are solving and the AtlasMesh Fleet Management System solution that unblocks yard, terminal, cross-dock, and corridor automation across **ports, warehouses, DCs, cold-chain, and intermodal**—without vendor lock-in. This page summarizes pains → use-case coverage → solution architecture → KPIs → rollout & gates.

---

## 1) The Problem (Logistics)

Modern logistics networks are **fragmented, time-sensitive, and system-heavy**. Existing vehicle fleets are manual and inefficient, but retrofitting them with autonomy is blocked by:

1) **Fleet Management Gaps.** Manual vehicle dispatch, no real-time fleet monitoring, inefficient routing, and lack of unified fleet operations across yard/terminal/DC operations.

2) **Integration Complexity.** WMS/TMS/TOS are siloed; integrations take months; existing fleet management systems don't integrate with customer systems; vendor lock-in prevents flexibility.

3) **Operational Inefficiency.** Manual vehicle operations, no predictive maintenance, energy waste, and compliance gaps that require manual intervention and evidence collection.

**Observed impact (baseline across sites):**
- **Fleet Utilization**: 60% average vehicle utilization
- **Manual Operations**: 3-5 operators per shift for vehicle management
- **Integration Costs**: $500K+ per facility for system integration
- **Energy Waste**: 20-30% energy waste due to inefficient routing
- **Maintenance Costs**: 40% higher maintenance costs due to reactive approach

---

## 2) Use-Case Universe (L1–L20) — grouped

**A. Fleet Management Operations**
- **L21 Fleet Monitoring**: Real-time vehicle tracking, status monitoring, and performance analytics
- **L22 Fleet Dispatch**: AI-powered task assignment and vehicle routing optimization
- **L23 Fleet Maintenance**: Predictive maintenance and automated scheduling
- **L24 Fleet Energy Management**: Smart charging and energy optimization
- **L25 Fleet Analytics**: Performance insights and operational optimization

**B. WMS Integration Workflows**
- **L26 WMS Order Integration**: Real-time order processing and vehicle assignment
- **L27 Inventory Management**: Automated inventory tracking and replenishment
- **L28 Dock Management**: Automated dock assignment and scheduling
- **L29 Warehouse Operations**: Cross-dock, sorting, and fulfillment support

**C. Yard & Terminal Moves**
- L1 Yard switcher (dock/yard tractor), L2 Berth shuttle, L6 Terminal equipment shuttle, L7 Yard optimization, L8 Empty repositioning, L18 Dock scheduling, L20 Yard security patrol

**D. Facility & Cross-Dock**
- L3 Cross-dock wave, L12 Cross-dock transfer, L16 Order fulfillment support, L17 Parcel sorting, L17 Reverse logistics (returns), L19 Facility maintenance

**E. Intermodal & Corridor**
- L4 Hub-to-hub corridor, L9 Rail connection, L10 Cross-border customs transit

**F. Temperature-Controlled**
- L5 Cold-chain shuttle, L13 Refrigerated transport, L18 Cold-chain monitoring

**G. Regulated & Risk**
- L11 HazMat transport, L20 Yard security patrol (intrusion, tamper, incident evidence)

**H. Orchestration & Planning**
- L7 Yard optimization, L18 Dock scheduling, **L19 Freight consolidation** (cube/weight optimization, manifesting)

---

## 3) Why current approaches fail

- **Single-purpose autonomy**: point solutions (yard only, or robot tugs) don’t generalize across facilities, vehicles, or climate bands.
- **Tele-drive dependence**: requires pristine comms; collapses under private LTE/Wi-Fi dead zones; unsafe for mixed traffic.
- **Integration hell**: custom code per WMS/TMS/TOS; brittle APIs; no contract tests; slow to certify.
- **Policy on paper**: HazMat, customs, and food safety rules live in SOPs, not in software; evidence prep is manual.
- **No offline posture**: cloud hiccups stall operations instead of degrading gracefully.

---

## 4) AtlasMesh Solution (Logistics)

**Agnostic by design** (vehicle, sensor, map, weather, cloud) with **policy-as-code** and **adapter marketplace** so one codebase spans yards, terminals, DCs, corridors, and border zones.

### 4.1 Core capabilities (logistics pack)

- **Yard-aware planning & routing**: lane priorities, door/berth windows, crane/door sync, safety zones, forklift interaction.
- **Dock & wave orchestration**: real-time door assignment, wave timing, door readiness gating, carrier ETA reconciliation.
- **Intermodal handoff**: precise crane/straddle carrier rendezvous, rail slot timing, TOC/TOS handshakes.
- **Corridor autonomy (hub↔hub)**: geofenced routes, weigh-station & customs overlays, convoy policies where allowed.
- **Cold-chain governance**: temperature bands enforced in policy; mission auto-halts or re-routes on excursion risk.
- **HazMat policies**: segregation, placard verification, route restrictions, emergency response overlays.
- **Freight consolidation & load planning**: cube/weight optimization, compatibility checks, **manifest automation** (L19). :contentReference[oaicite:3]{index=3}
- **Security patrol & incident evidence**: multi-modal sensing, anomaly detection, 2-min alerting, tamper detection (L20). :contentReference[oaicite:4]{index=4}
- **Tele-assist Q&A (no tele-drive)** with assist budgets and auditable traces.
- **Offline-first**: store-and-forward logs, edge-resident safety behaviors for comms loss.

### 4.2 Integration & data

- **Adapters (certified):** WMS (inventory, door status), TMS (loads/ETAs), TOS (crane windows, stack plan), YMS, LMS.
- **Contracts:** event schema for `door_ready`, `crane_window`, `wave_release`, `temp_band`, `hazmat_class`, `rail_slot`, `customs_clear`.
- **Evidence ledger:** per mission bundle (route plan, policy evaluations, sensor excerpts, manifests, handoff proofs).

---

## 5) ODD & Operating Constraints (Logistics)

- **Sites:** warehouse/DC yards, marine terminals, intermodal ramps, cross-dock and cold rooms, private corridors.
- **Traffic:** mixed (humans, forklifts, hostlers, cranes); speed caps; right-of-way policies.
- **Comms:** private LTE/Wi-Fi with dead-zone tolerance; satellite optional.
- **Environment:** −30–50 °C, rain/dust; day/night; reflective yards; condensation in cold rooms.
- **Safety guardrails:** geofences, approach cones at docks/berths, forklift proximity rules, e-stop zones.

---

## 6) Use-case clusters → solution mapping

| Cluster | Representative L# | What breaks today | AtlasMesh capability | P0 KPIs |
|---|---|---|---|---|
| **Yard & Dock** | L1, L7, L18 | Miss-spots, long dwell, manual door juggling | Yard graph + door/berth scheduler; YMS/WMS adapters; safety zones | **On-time yard moves ≥95%**, **dock dwell P95 ≤30 min**, **miss-spot ≈0** |
| **Berth & Intermodal** | L2, L6, L9 | Idle crane, mistimed handoffs | Crane window alignment; TOS handshakes; mm-level approach | **Crane idle −15%**, **handoff success ≥99.5%** |
| **Cross-dock & Waves** | L3, L12, L16, L17 | Wave slippage; rehandles | Wave orchestration; slotting; load ID validation | **Wave adherence ≥95%**, **rehandles −20%** |
| **Cold-chain** | L5, L13, L18 | Temp excursions | Temp-band policy; pre-cool checks; thermal-aware routes | **Excursions = 0**, **rejects −30%** |
| **Regulated & Risk** | L11, L10, L20 | HazMat/customs lapses, security blind spots | HazMat segregation; customs doc gates; security patrol & evidence | **Compliance findings = 0**; **threat detection ≥95%**, **alert ≤2 min** :contentReference[oaicite:5]{index=5} |
| **Consolidation & Planning** | **L19** | Under-filled loads; doc errors | Cube/weight optimization; compatibility rules; manifesting | **Cube ≥90%**, **0 damage**, **100% manifest accuracy** :contentReference[oaicite:6]{index=6} |

---

## 7) KPIs (sector-level)

- **On-time yard moves ≥ 95%** (gate→dock→gate)
- **Dock/berth dwell P95 ≤ 30 min**
- **Crane/door idle −15–25%**
- **Miss-spot / rehandle ≈ 0**
- **Cost per move −10–15%**
- **Cold-chain excursions = 0**
- **HazMat & customs violations = 0**
- **Security detection ≥ 95%**, **alert ≤ 2 min** (L20) :contentReference[oaicite:7]{index=7}
- **Freight consolidation**: **cube ≥ 90%**, **100% manifest accuracy** (L19) :contentReference[oaicite:8]{index=8}

> All KPIs are bound to machine-readable formulas in `data/contracts/kpis.yaml`; releases gate if red.

---

## 8) Risks & mitigations (logistics)

- **Mixed-traffic incidents** → forklift & pedestrian detection, speed zoning, audible/visual intent cues, safe-stop cones.
- **API drift (WMS/TMS/TOS)** → contract tests, adapter version pinning, fallbacks/shims.
- **Comms dead zones** → offline-first routing; on-edge policy evaluation; deferred uploads.
- **Cold-room condensation/sensor fog** → sensor hoods, thermal profiles, perception fallbacks.
- **Security tamper** → encrypted comms, tamper events, patrol cross-checks (L20). :contentReference[oaicite:9]{index=9}

---

## 9) Rollout & quality gates

**Phase 1 — Pilot yard/DC (4–8 weeks).**  
Scope L1/L7/L18. Gate on: **on-time moves ≥90%**, **dwell P95 ≤35 min**, **zero critical safety**.

**Phase 2 — Terminal & intermodal (8–12 weeks).**  
Add L2/L6/L9 + HazMat (L11) + cold-chain (L5/L13/L18). Gate on: **crane idle −10%**, **excursions 0**, **HazMat violations 0**.

**Phase 3 — Network & planning (12–20 weeks).**  
Add corridor (L4/L10), **freight consolidation (L19)**, yard security (L20). Gate on: **cube ≥90%**, **alert ≤2 min**, **cost/move −10–15%**. 

---

## 10) Acceptance & evidence (per site)

- **Scenario pass** (sim + field) for yard moves, dock approaches, crane handoff, forklift proximity, HazMat segregation, cold-chain route.
- **System of record parity**: WMS/TMS/TOS/YMS events match mission logs within ±2 s.
- **Compliance bundle**: manifests, policy decisions, sensor snippets, security evidence (where applicable) per release and per mission (L19/L20). 

---

## 11) Appendices (use-case index)

- **Yard/Terminal:** L1, L2, L6, L7, L8, L18, **L20** :contentReference[oaicite:12]{index=12}  
- **Facility/Cross-dock:** L3, L12, L16, L17 (parcel), L17 (reverse), L19 (facility maintenance)  
- **Intermodal/Corridor:** L4, L9, L10  
- **Cold-chain:** L5, L13, L18 (monitoring)  
- **Planning/Consolidation:** **L19 (freight consolidation)** :contentReference[oaicite:13]{index=13}  
- **Regulated & Security:** L11, **L20** :contentReference[oaicite:14]{index=14}

> Logistics is one of four sector problem statements. The same **agnostic Fleet OS** powers all sectors via overlays (policy + adapters). No forks.
