# Market Insights & Jobs-To-Be-Done (JTBD)

## 0) Purpose & guardrails (baked in)

This document maps the market we serve, what buyers hire us to accomplish, and what harsh ODDs demand—so product, GTM, and engineering ship the right things first.
**Guardrails:**

* Use **Metric IDs (MET.*)** only; formulas/targets live in the Metrics Canon.
* Reference **risk IDs (RR-*)** only; mitigations live in the Risk Register.
* Market figures labeled **Directional** provide context; **Authoritative** planning uses our Finance model.

---

## 1) Market layers, provenance & sensitivity

| Layer                  | Definition                                                                                         |   2024 size |                    2034 outlook | Provenance                               | Notes                                       |
| ---------------------- | -------------------------------------------------------------------------------------------------- | ----------: | ------------------------------: | ---------------------------------------- | ------------------------------------------- |
| Global AV TAM          | All AV use cases & regions (consumer + enterprise)                                                 | **$207.4B** |        **$4,450B @ 36.3% CAGR** | **Directional** (multi-report synthesis) | Context only                                |
| Focused Enterprise TAM | Regulated, industrial & government ODDs we serve (Defense, Mining, Logistics, geofenced Ride-hail) |  **$12.5B** | Expands with permits & adapters | **Authoritative** (internal roll-up)     | Basis for sector plans                      |
| SAM                    | Priority regions with workable permits & buyer pull                                                |  **$3.73B** |       ↑ with jurisdiction packs | **Authoritative** (Finance)              | Source of truth in finance model            |
| 3-yr SOM               | Realistic capture (lighthouse → replication)                                                       | **$131.3M** |               Sensitivity-bound | **Authoritative** (Finance)              | Bounded by install rate & evidence velocity |

**Sensitivity knobs (owned assumptions)**

| Assumption                         |        Low |             Base |         High | Owner             | Review cadence |
| ---------------------------------- | ---------: | ---------------: | -----------: | ----------------- | -------------- |
| Permit velocity (priority regions) | 1 city/qtr | **2 cities/qtr** | 3 cities/qtr | Compliance        | 90 days        |
| Adapter coverage (WMS/TOS/ERP)     |        60% |          **75%** |          90% | Integrations      | 60 days        |
| Evidence bundle pass rate          |        85% |          **95%** |          99% | Safety/Compliance | Per release    |
| Install throughput (veh/week)      |         20 |           **35** |           50 | Operations        | 60 days        |

> If any **Base** drops to **Low** for >1 review, re-cast SOM in Finance and reflect changes in OKRs.

---

## 2) Segmentation—application, geography, autonomy

### 2.1 Application (directional 2024 snapshot)

| Segment                   | Share / Size            |           Growth signal | Demand drivers                      | Headwinds                                         |
| ------------------------- | ----------------------- | ----------------------: | ----------------------------------- | ------------------------------------------------- |
| Transportation / Robotaxi | **~82.5%**, **$189.9B** |                    Fast | 24/7 urban mobility; TNC ops        | Regulatory fragmentation; mixed-traffic incidents |
| Defense                   | **~17.5%**, **$17.5B**  | **Fastest (~45% CAGR)** | Force protection; contested ops     | Export controls; ROE evidence                     |
| Freight & Logistics       | **$31.5B** (42% CAGR)   |                    High | Driver shortage; yard/corridor SLAs | Adapter sprawl; customs friction                  |
| Industrial / Mining       | **$17.2B** (41% CAGR)   |                    High | Safety; tons/hour                   | Heat/dust; heavy-class hardware                   |

### 2.2 Geography

| Region        |         2024 share        | Trajectory                       | Strategic implication                            |
| ------------- | :-----------------------: | -------------------------------- | ------------------------------------------------ |
| North America |          **~40%**         | Mature pilots; high scrutiny     | Lead with transparency & evidence-as-code        |
| APAC          | Fastest (**~39.9% CAGR**) | China/SEA momentum               | Partner-first; data residency; local compliance  |
| EU/UK         |          Moderate         | Safety/ethics-first              | SOTIF rigor; explainability                      |
| GCC/MENA      |         Strategic         | Permit agility; industrial focus | Heat/dust ODD; defense/ports/mining entry points |

### 2.3 Autonomy levels (revenue now vs growth)

| Level      | Revenue weight now | Growth to 2034 | Notes                                       |
| ---------- | ------------------ | -------------- | ------------------------------------------- |
| L1–L2 ADAS | High               | Stable         | Consumer-heavy; not our offer               |
| L3         | Emerging           | Moderate       | Bridge to L4 corridors                      |
| **L4–L5**  | Low today          | **Fastest**    | Our core: geofenced & industrial ODDs first |

---

## 3) Adoption & readiness scorecard (1–5; higher = nearer-term fit)

| Sector                    | ODD controllability | Regulatory maturity | Integration complexity | ROI clarity | Readiness | Primary blockers                               |
| ------------------------- | :-----------------: | :-----------------: | :--------------------: | :---------: | :-------: | ---------------------------------------------- |
| **Mining**                |          5          |          4          |            3           |      5      |  **4.25** | Dust/visibility; haul-road quality; heavy SKUs |
| **Logistics & Ports**     |          4          |          4          |            3           |      4      |  **3.75** | WMS/TOS fragmentation; customs                 |
| **Defense**               |          3          |          3          |            3           |      4      |  **3.25** | GNSS denial; ROE; classified ops               |
| **Ride-hail (geofenced)** |          3          |          3          |            4           |      3      |  **3.25** | Public trust; work zones; incident comms       |

---

## 4) Buying centers, timelines & budget envelopes

| Segment                     | Primary buyers                       | Timeline     | Budget envelope | What wins                                             | Likely objections                |
| --------------------------- | ------------------------------------ | ------------ | --------------- | ----------------------------------------------------- | -------------------------------- |
| Ride-hail / Urban transport | City mobility authority; TNC program | **18–36 mo** | **$50M–$500M**  | Safety case; explainability; community engagement     | Liability; protest risk          |
| Freight & Logistics         | VP Ops; Terminal/Yard Ops; 3PL/DC    | **12–24 mo** | **$10M–$100M**  | Throughput; adapters; weeks-to-deploy                 | Change mgmt; cold-chain; customs |
| Defense                     | MOD/Forces procurement; base/air ops | **24–60 mo** | **$100M–$1B+**  | ROE-compliant autonomy; GNSS-denied ops; audit trails | Export controls; cyber assurance |
| Industrial / Mining         | COO; Mine Ops; HSE; Maintenance      | **12–18 mo** | **$5M–$50M**    | Tons/hour ↑; cost/ton ↓; zero-harm                    | Heat/dust reliability; retrofits |
| Accessibility mobility      | Insurers; healthcare orgs; families  | **3–6 mo**   | Programmatic    | Reliability; dignity; ease of use                     | Trust; handoff model             |

---

## 5) Cross-sector JTBD → sector overlays → MET.* (IDs only)

### 5.1 Core JTBD (functional • emotional • social)

| JTBD                            | Functional job                                 | Emotional job                        | Social job                           | Metrics (IDs)                                          | Product implications                                      |
| ------------------------------- | ---------------------------------------------- | ------------------------------------ | ------------------------------------ | ------------------------------------------------------ | --------------------------------------------------------- |
| **Assess ODD & readiness**      | Decide corridor/site viability fast            | Confidence we won’t run a dead pilot | Signal diligence to regulators/board | MET.SAFETY.READINESS_SCORE, MET.RELIAB.ODD_CONFORMANCE | ODD scoring; terrain/heat/dust models; bootstrap policies |
| **Plan & dispatch**             | Assign & route with ODD/energy/work rules      | Reduce SLA anxiety                   | Prove ops excellence                 | MET.OPS.DISPATCH_LAT_P95, MET.COST.EMPTY_MILES         | Multi-objective routing; tariff-aware energy              |
| **Operate through disruptions** | Maintain service under weather/GNSS/Comms loss | Calm under stress                    | Show resilience publicly             | MET.SAFETY.ASSIST_RATE, MET.RELIAB.AVAIL_ODD           | Offline-first; SLAM fallback; occlusion modes; assist Q&A |
| **Assure compliance**           | Produce audit-ready bundles                    | No audit dread                       | “Safe hands” vendor signal           | MET.GOV.AUDIT_BUNDLE_COMPLETE, MET.GOV.FINDINGS_P1     | Evidence-as-code; jurisdiction packs; change log          |
| **Optimize throughput & cost**  | Close loop data→policy→result                  | Control KPIs                         | Win budget battles                   | MET.PROD.MOVES_PER_HOUR, MET.COST.$_PER_MOVE           | Yard/corridor optimizers; demand/rebalance                |
| **Protect privacy & security**  | Zero-trust; minimization; residency            | Sleep-at-night security              | Meet board/regulator bar             | MET.SEC.INCIDENTS_P1, MET.GOV.PRIVACY_FINDINGS         | mTLS/OIDC; SBOM/signing; residency controls               |

### 5.2 Sector overlays (what each weights most)

| Sector    | JTBD emphasis (top-3)                                               | Primary MET.*                                                                     |
| --------- | ------------------------------------------------------------------- | --------------------------------------------------------------------------------- |
| Defense   | Operate through disruptions; Assure compliance; Assess ODD          | MET.RELIAB.ODD_CONFORMANCE, MET.GOV.AUDIT_BUNDLE_COMPLETE, MET.SAFETY.ASSIST_RATE |
| Mining    | Optimize throughput & cost; Operate through disruptions; Assess ODD | MET.PROD.TONS_PER_HOUR, MET.COST.$_PER_MOVE, MET.RELIAB.AVAIL_ODD                 |
| Logistics | Plan & dispatch; Optimize throughput & cost; Assure compliance      | MET.OPS.GATE_TO_DOCK_P95, MET.OPS.THROUGHPUT, MET.GOV.AUDIT_BUNDLE_COMPLETE       |
| Ride-hail | Plan & dispatch; Protect privacy & security; Assure compliance      | MET.EXP.ETA_P95, MET.SEC.INCIDENTS_P1, MET.GOV.AUDIT_BUNDLE_COMPLETE              |

---

## 6) ODD realities (what we must survive) → required capabilities

| Dimension         | Constraint (directional)                   | Impact                     | Required capability                                | Risk link  |
| ----------------- | ------------------------------------------ | -------------------------- | -------------------------------------------------- | ---------- |
| Weather           | Heavy rain/snow → **~70%** perception loss | Missed detections; assists | Weather modes; sensor fusion; conservative driving | RR-TECH-01 |
| Temperature       | **−40 °C to +85 °C**                       | Derating; failures         | Thermal design; health sentry; derating policies   | RR-TECH-02 |
| Lighting          | Night/glare/poor contrast                  | Range/accuracy loss        | HDR cams; high-beam LiDAR; lighting-aware policy   | RR-ODD-03  |
| Terrain & surface | Loose substrate, ruts, rocks, grades       | Traction & articulation    | Terrain/traction packs; slope-vs-speed             | RR-ODD-04  |
| Maps & infra      | Work zones; unmapped roads                 | Localization error         | SLAM fallback; map confidence scoring              | RR-TECH-05 |
| Comms/GNSS        | Dead zones; jamming/spoofing               | Remote features loss       | Offline-first; mesh relays; GNSS-denied SLAM       | RR-OPS-06  |
| Energy            | Tariff volatility; charge windows          | Cost/uptime                | Tariff-aware scheduling; PdM on energy/cooling     | RR-BIZ-07  |

---

## 7) Competition—named & category (limits → our edge)

### 7.1 Named (directional)

| Player                   | Positioning        | Strengths               | Structural limits in our focus            | Our stance                                    |
| ------------------------ | ------------------ | ----------------------- | ----------------------------------------- | --------------------------------------------- |
| Waymo                    | Urban robotaxi     | Transparency; ops scale | Mixed-traffic incidents → regulatory drag | Evidence-as-code + controllable ODDs          |
| Tesla                    | Consumer ADAS      | Manufacturing scale     | Not L4 in regulated ODD                   | Industrial/regulatory lane, not consumer ADAS |
| Cruise                   | Urban robotaxi     | GM integration          | Safety suspensions                        | Rugged ODD + industrial throughput            |
| Aurora                   | Freight corridors  | OEM partnerships        | Long approvals; integration time          | Adapter marketplace; corridor packs           |
| WeRide / Pony.AI / Baidu | China-led          | Night ops; scale        | Geopolitical/export risk                  | SEA partnerships; residency compliance        |
| SteerAI                  | Defense/industrial | GPS-denied; UAE backing | Narrow scope                              | Cross-sector reuse via overlays               |

### 7.2 Category

| Category                      | What they optimize | Structural limit           | Implication                          |
| ----------------------------- | ------------------ | -------------------------- | ------------------------------------ |
| Single-sector AV stacks       | Deep vertical fit  | Poor portability           | Overlay model & reuse across sectors |
| OEM captive stacks            | Tight HW/SW        | Vendor lock-in             | Agnostic adapters; exit options      |
| Tele-operation (remote drive) | Coverage stop-gap  | Regulatory/safety friction | **Assist Q&A**, not remote drive     |
| Cloud-only dispatch           | Scale in good nets | Breaks offline             | Offline-first + edge arbitration     |
| Sim/tool vendors              | Validation depth   | Ops/evidence absent        | Twin-gates + evidence-as-code story  |

---

## 8) Research synthesis → where we win first (and why)

* **Mining (M1–M25)**: fixed-graph ops; throughput tied to haul-road & dust → **terrain/traction policies** + **PdM** win.
* **Logistics (L1–L20)**: yard/rail/customs orchestration; adapter fragmentation → **contract-tested adapters** + **queue/berth optimizers**.
* **Defense (D1–D26)**: convoy/corridor under GNSS denial; ROE evidence → **GNSS-denied SLAM**, **jurisdiction/ROE packs**.
* **Ride-hail (R1–R20)**: ETA reliability, accessibility, airport/event overlays, incident transparency → **assist explainers**, **LE protocols**, **airport queue mgmt**.

**Green-flags:** controllable ODD; adapter list matches marketplace; funded owner; permit path clear.
**Red-flags (no-go):** remote-drive requirement; bespoke single-site hardware; no auditability.

---

## 9) Product implications (traceable to metrics, OKRs, and risks)

| Insight → Implication                | What we ship                                     | Metrics (IDs)                                        | OKR link       | Risk link  |
| ------------------------------------ | ------------------------------------------------ | ---------------------------------------------------- | -------------- | ---------- |
| Weather/lighting degrade performance | Weather/lighting modes; occlusion handling       | MET.SAFETY.SAFE_STOP_RATE, MET.RELIAB.AVAIL_ODD      | A1.KR2/KR3     | RR-TECH-01 |
| Rugged terrain drives assists        | Terrain/traction policy packs; slope-aware speed | MET.SAFETY.GRADE_VIOLATIONS, MET.SAFETY.ASSIST_RATE  | A1.KR3         | RR-ODD-04  |
| Adapter sprawl blocks time-to-value  | Certified adapters + contract tests              | MET.OPS.TIME_TO_VALUE, MET.GOV.AUDIT_BUNDLE_COMPLETE | A2.KR3         | RR-OPS-08  |
| Compliance friction slows GTM        | Evidence bundles; jurisdiction packs             | MET.GOV.FINDINGS_P1=0                                | A1.KR4, A2.KR2 | RR-COMP-02 |
| Energy cost & uptime matter          | Tariff-aware scheduling; PdM (energy/cooling)    | MET.COST.$_PER_MOVE, MET.RELIAB.AVAIL_ODD            | A2.KR1         | RR-BIZ-07  |

**Claims discipline & acceptance gates:**

* No KPI claim without a Metrics-Canon ID and live dashboard source.
* No market figure without provenance label (Directional vs Authoritative).
* No implication row ships without ≥1 MET.* and ≥1 OKR ref.

---

## 10) Provenance and refresh triggers

* **Provenance classes:** *Authoritative* (Finance model; signed customer datasets; regulator filings). *Directional* (analyst/press/vendor). Directional informs context only.
* **Auto-refresh:** sensitivity knobs in §1 have owners & cadences; a lint job flags expired assumptions and blocks merges until refreshed.

---

## 11) Cross-doc references (for depth, not duplication)

* Personas & scenarios index (IDs D*, M*, L*, R*).
* Metrics Canon (metric formulas/targets; SoT).
* OKR Index & OKR-Glue (how MET.* roll to KRs).
* Risk Register (owners/controls/tests for RR-*).
* Adapter Matrix; Jurisdiction Packs.

---

### Appendix — Ambiguity traps & our countermeasures

* **TAM inflation:** always label provenance; planning uses Finance SoT.
* **Metric drift:** IDs only here; formulas/targets live elsewhere.
* **Scope creep:** stick to inclusions/exclusions; link out for details.
* **Compliance claims:** no “approved” language without `audit_bundle_score=100%`.

---