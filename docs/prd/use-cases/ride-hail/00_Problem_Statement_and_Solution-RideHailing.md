# 00_Problem_Statement_and_Solution_RIDEHAILING.md

**Purpose**
Define the problem AtlasMesh solves for autonomous ride-hailing across city, airport, campus, and special-program contexts; state the success outcomes; and describe the solution at a level that lets engineering, ops, safety, and GTM converge on “what we’re building and why.”

---

## 1) Problem Statement (what’s broken today)

Ride-hailing at L4 autonomy fails to meet **trust, reliability, and compliance** bars once you leave a narrow “happy path.” Across R1–R20, the recurring problems are:

* **Trust & safety at the edges.** Night trips, vulnerable riders, and law-enforcement interactions require deterministic, auditable flows—not ad-hoc operator judgment (R5, R8, R11, R20). In practice: riders don’t feel safe; operators lack verifiable proof of safe conduct.
* **Venue/curb complexity.** Airports and event districts impose queueing, staging, and curb-dwell rules that change by hour and by authority (R3, R7, R12); violations cause fines, delays, and shut-offs.
* **Accessibility, assistance & NEMT.** ADA/PRM, assisted boarding, and non-emergency medical transport add time-bound protocols and handoffs that generic AV stacks don’t encode (R4, R9, R13).
* **Multi-modal reality.** First/last-mile connections (rail/bus/ferry), campus/internal shuttles, and subscription routes must work as a system, not as isolated rides (R6, R10, R14, R16, R18, R19).
* **Demand volatility.** Peaks (events, weather, shift changes) break ETA promises without proactive rebalancing and policy-aware pooling (R2, R6, R7, R17).
* **Operational proof.** Regulators, venue authorities, and community partners need **evidence** of conformance (routes, dwell, ID checks, emergency handling)—not slideware.

**Net effect:** Missed ETAs, higher cancellations, rider anxiety at night, accessibility failures, airport sanctions, brittle integrations, and blocked scale.

---

## 2) Outcomes that define success (12–18 months)

**Rider & service**

* **P95 wait time ≤ 7 min**, **ETA error ≤ 90 s**, **ride completion ≥ 98%**, **CSAT ≥ 4.8/5**.
* **Night safety layer**: **0 security incidents**, **≤ 3 min** average response to safety requests, **100% safety-optimized route adherence** (service-level KPI). 
* **Accessibility completion ≥ 98%** with verified assistance steps.

**Venue conformance**

* **Airport/curb compliance ≥ 99%** (staging, dwell, bay assignment); **queue adherence ≥ 99%** at controlled venues (airport/campus).
* **Campus shuttle**: **≥ 98% on-time performance** on fixed routes; **100% designated area coverage** (program KPI). 

**Safety & operations**

* **Assists ≤ 0.5 / 1,000 km** trending toward **≤ 0.3**; **critical incidents = 0**; **policy violations = 0** with audit trails.
* **Authority integrations** (security/community/airport) active and verified in logs.

---

## 3) Solution (what we’re building)

AtlasMesh delivers a **Ride-hailing Service Layer** on top of our Fleet OS that encodes venue rules, safety protocols, accessibility flows, and multi-modal handoffs as **policy-driven, auditable behaviors**—not custom forks.

### 3.1 Core solution levers → problems they solve

* **Safety-first mission planning**
  Routes constrained by “safe corridors” at night; automatic avoidance of flagged areas; continuous security monitoring; one-tap escalation to authorities with evidence package. (Maps, routing, and policy rules compose a **Night Safety** service profile). 

* **Venue logic packs (airport, campus, events)**
  Queue join/leave, bay assignment, dwell timers, and signage/PA hints baked into missions; acceptance tests mirror venue SOPs (e.g., **campus fixed-route/on-demand/event** modes with schedule adherence KPIs). 

* **Accessibility & assistance flows**
  Pre-boarding checks, ramp/PRM timing budgets, caregiver handoff confirmation, and post-ride verification; incidents auto-explainable from logs.

* **Multi-modal connectors**
  Timetable/GTFS ingestion; “hold for connection” policy; curb-to-platform wayfinding; campus/internal circulators coordinated with public transit.

* **Demand-aware dispatch, pooling & rebalancing**
  Forecasts demand spikes (events/shift changes); policy-constrained pooling (no detours that violate accessibility/curfew rules); proactive vehicle staging.

* **Evidence-as-code**
  Every ride compiles a verifiable bundle: route choice rationale, ID checks, venue compliance (dwell/queue), accessibility steps, and emergency handling.

* **Tele-assist Q&A (no tele-drive)**
  Scripted triage trees per service profile (night safety, airport curb, accessibility); strict budgets; all operator inputs recorded for audit.

### 3.2 Service profiles (pre-configured, policy-driven)

* **Standard & Shared Rides** (R1, R2, R6, R7, R17)
* **Airport & Special Venues** (R3, R7, R12)
* **Accessibility & NEMT** (R4, R9, R13)
* **Night Safety & Community Programs** (R5, R11, R20) 
* **Multi-modal & Campus** (R10, R14, R19) 
* **Subscriptions/Tourism/Packages** (R15, R16, R18)

Each profile ships with: ODD contract, policy rules, UI hints, acceptance scenarios, and KPI gates.

---

## 4) Acceptance & test gates (how we know it works)

* **Scenario suites by profile** (OpenSCENARIO + venue simulators):

  * Night: routine safety ops, security concern, vulnerable passenger. 
  * Campus: fixed route, on-demand, special event surge. 
  * Airport: queue join/leave, bay reassignment, curb dwell.
  * Accessibility/NEMT: assisted boarding, caregiver handoff, missed-step remediation.
* **Go/No-Go gates:** All P0 KPIs met for the profile (wait/ETA, incidents=0, venue compliance, accessibility completion); red/yellow gates block rollout until green.

---

## 5) In scope / Out of scope

**In scope:** L4 geofenced ride-hailing missions; night safety; airport/campus/event venues; accessibility/NEMT; multi-modal connectors; tele-assist Q&A; evidence bundles.

**Out of scope:** Remote driving/tele-operation; uncontrolled public-road beta without permits; lethal or crowd-control functionality; ad-hoc per-tenant code forks (all variance via policy/config).

---

## 6) Use-case coverage (R1–R20)

* **Core rides & pooling:** R1, R2, R6, R7, R17
* **Airports & queues:** R3, R12
* **Accessibility & assistance/NEMT:** R4, R9, R13
* **Night safety & emergency:** R5, R11, R20 
* **Multi-modal & campus:** R10, R14, R19 
* **Programs & add-ons:** R15 (package), R16 (tourism), R18 (subscription)

---

## 7) Open decisions (tracked)

* Night “safe-corridor” data sources (municipal feeds vs. private security partners).
* Airport queue protocol variations across authorities (baseline pack vs. per-airport overlays).
* ADA/PRM evidence granularity (minimal vs. rich media) and retention windows.

---

**Bottom line**
The problem is **trustworthy, compliant, and seamless autonomy across the messy real world of ride-hailing**. The solution is a **policy-driven service layer** with venue-specific packs, safety/assistance protocols, multi-modal intelligence, and **evidence-as-code**—so we can prove we did the right thing, every ride, everywhere we’re permitted to operate.
