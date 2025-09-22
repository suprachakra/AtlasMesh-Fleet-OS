# Executive Summary

AtlasMesh Fleet OS is the agnostic operating system for Level-4 autonomous fleets in defense, mining, logistics, and ride-hail. We retrofit mixed vehicles and orchestrate missions in 50–60 °C heat, dust, GNSS degradation, and intermittent comms, while automating evidence for regulators. One platform, policy overlays—weeks to deploy, evidence-ready by default, and no vendor lock-in.

**North star.** Within 12 months of first deployment, any customer can:

* activate **≥90% of their compatible fleet** via retrofit kit + Fleet OS,
* operate **≥98.5% mission uptime** in ODD, **≤0.3 assists/1,000 km**,
* demonstrate **positive ROI < 18 months** (sector-specific), while meeting safety, regulatory, and cyber requirements.

**What we ship.**

1. **AV Kit (hardware/software)** that retrofits diverse vehicles (UGVs, haul trucks, yard tractors, vans/robotaxis) with **ROS2-based Edge Stack**.
2. **AtlasMesh Fleet OS** (control center, policy engine, routing/dispatch, energy, telemetry, OTA, **CARLA/Gazebo simulation gates**).
3. **Data & Safety stack** (provenance-aware geospatial store with **Lanelet2/OpenDRIVE support**, scenario bank, KPI registry, safety case evidence automation).

**Why we win.** One codebase, policy-driven overlays (sector/vehicle/city/tenant), **Hybrid Decision Framework** (behavior trees + rules + learned priors), and **twin-gated CI/CD** eliminate hidden integration work, reduce time-to-revenue, and scale safely.

---

### 2) Strategic context & market definition (ME focus)

**Sectors & ODD realities (Middle East).**

* **Defense.** Cross-border desert patrols, GPS-denied corridors, convoy logistics, C2 security, EW resilience.
* **Mining.** Open-pit/haul in dust & heat, fixed corridors, fuel/energy constraints, 24/7 production SLAs.
* **Logistics & Supply Chain.** Ports/terminals, warehouses, last-mile in gated areas, intermodal yards; V2I opportunities.
* **Ride-hailing.** Geofenced districts with clear municipal permits; rider UX, accessibility, and incident handling.

**Environment constraints:** **50–60 °C ambient**, sand/dust ingestion, limited shade, shock/vibration, **intermittent LTE/5G**, private LTE, SAT failover, sporadic road markings, sudden sand drifts, nighttime heat radiance.

**Buyers & users (archetypes).**

* Economic buyers: defense procurement, mining COO, logistics VP Ops, mobility authority, port operator.
* Technical: CIO/CTO, CISO, Head of Autonomy, Depot/Terminal Ops, Safety/Compliance.
* End users: dispatchers, rider support, depot techs, field operators, analysts.

---

### 3) Problem landscape (by sector) → measurable outcomes

| Sector           | Today's Problems (ME)                                                                   | Impacts                                                 | Measurable Outcome Targets (12–18 mo)                                                                                                                       |
| ---------------- | --------------------------------------------------------------------------------------- | ------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **Defense**      | High risk logistics; GPS jamming; convoy coordination; data classification; harsh comms | Mission aborts; exposure of personnel; "dark" corridors | **≥98% mission completion** in ODD; **≤1 safe-stop/10k km**; **sub-2s** tele-assist roundtrip within policy budget; **crypto posture attested per release** |
| **Mining**       | Heat derating; dust occlusion; unplanned downtime; fuel inefficiency                    | Throughput loss; safety incidents                       | **+8–12%** tons/hour; **≤0.2 assists/1k km**; **≥99.5% fleet availability**; **20–30% PdM-driven maintenance cost reduction**                               |
| **Logistics**    | Yard congestion; manual dispatch; WMS/TOS fragmentation; variable demand                | Missed SLAs; idle assets                                | **95%+ on-time** yard moves; **≤3%** empty miles; **10–15%** energy cost reduction; **<90s** gate turnaround variance                                       |
| **Ride-hailing** | Rider trust; complex urban works; ADA; regulator scrutiny                               | Cancellations; PR risk                                  | **P95 wait ≤7 min**, **CSAT ≥4.8/5**, **zero harm** safety posture, **≤0.5 assists/1k km**, compliant incident workflows                                    |

> **Design rule:** Every problem has a KPI, baseline, and alert threshold registered in `data/contracts/kpis.yaml` and enforced by `alerts-incident`. No KPI → feature cannot ship.

---

### 4) Value proposition & differentiation

**Agnostic by design.**

* **Vehicle-agnostic.** Modular I/O (CAN/J1939/LIN/FlexRay, drive-by-wire adapters), sensor abstraction, thermal-rated compute, **attested OTA**.
* **Platform-agnostic.** Cloud-neutral; deploy on customer cloud or ours; k8s-native; mTLS/OIDC; **policy-as-code**.
* **Sector-agnostic.** **Rules/overlays** for ODD, regulatory, dispatch, HMI; no forks.

**Extreme-weather advantage.**

* Heat-aware routing & charge/fuel scheduling, dust-robust perception configs, **weather fusion** with **freshness vs. credibility** controls.
* **Comms-tolerant autonomy** (store-and-forward, opportunistic sync, SAT backup), **graceful degradation** to ODD-safe behaviors.

**Operational truth.** **Self-auditing**: every deployment auto-produces safety & compliance evidence; **red/amber gates** block releases that regress safety/CSAT/assist KPIs or violate ODD.

---

### 5) Scope, non-goals, boundaries

**In-scope.** L4 geofenced autonomy; retrofit kits; Fleet OS; tele-assist **Q&A** (no joystick drive); safety case; ops tooling; integrations (WMS/TOS/ERP/V2X); rider/mission UX.

**Out-of-scope (non-goals).**

* Selling complete vehicles; **Level-5** everywhere; lethal functionality; driver-takeover via remote driving; uncontrolled public-road beta.
* **Single-tenant forks** (all tenant variance via configs/policy only).
* Unsupported climates without thermal spec conformance.

**Boundary Conditions**:
- **Vehicle Classes**: Support vehicles ≤26,000 kg GVWR; wheelbase 2.5-8.5m; max 3 articulation points
- **ODD Limits**: Max operational speed 80 km/h; visibility ≥10m; max grade 15%; temperature -40°C to +65°C
- **Assist Budget**: ≤2/1,000 km; max 45-min offline operation; max 5% variant budget
- **Regulatory**: No deployment without jurisdiction pack; no operation without safety case
- **Economic**: No custom development without ROI validation; no sector expansion without proven economics

**Decision Framework**: Any scope expansion requires:
1. Strategic alignment assessment
2. Variant budget impact analysis
3. Safety case extension validation
4. Economic model validation
5. Executive approval with documented rationale

**Enforcement.** ADR: `ADR/00xx-non-goals.md` + toggles in `rules/odd/*` and `rules/policy/*`. Sales playbooks reject out-of-scope RFPs.

---

### 6) ODD (Operational Design Domain) definition

**Axes.** Geography (tiles), road classes, weather (heat/dust/wind/visibility), time of day, traffic density, comms bands, **GNSS integrity**, legal constraints.

**ODD contract.** Machine-readable in `rules/odd/`, referenced by dispatch/routing; **ODD guard** halts or re-plans when sensors, weather, or comms breach thresholds (with tele-assist budget).

**Examples.**

* **Defense corridor:** paved/unpaved, GNSS degraded allowed with SLAM confidence ≥X, SAT fallback required, convoy platooning ≤Y gap.
* **Mine haul:** fixed route graph, dust visibility ≥ Vmin, slope ≤ Smax, heat derate curve loaded from vehicle profile.
* **Ride-hail district:** municipal permit zone; work-zone cones → consult tele-assist; rider pickup/ADA flows.

---

### 7) Users & stakeholders (personas)

* **Dispatcher (Ops).** Needs SLA-tight assignment, incident triage.
* **Rider Support (RH).** Needs live trip context, safe-stop flows, UX scripts.
* **Depot/Mine Tech.** Needs health, firmware staging, PdM tickets, SIM/telemetry status.
* **Mission Commander (Def).** Needs convoy status, ROE-compliant controls, audit trail.
* **Port/Yard Supervisor (Log).** Needs yard map, queue control, crane sync.
* **Compliance/Safety.** Needs safety case evidence, audit bundles.
* **CISO.** Needs SBOMs, attestation, secrets posture, incident runbooks.

**RACI** in `docs/strategy/04_Product_and_Marketplace_Strategy.md`. Objection library in Marketing Plan.

---

### 8) Top-line success metrics & OKR linkage

**North-star metric families** (registered & alert-backed):

* **Safety:** zero harm; assists/1k km; safe-stops/10k km; disengagement taxonomy.
* **Reliability:** fleet availability; mission completion; ODD conformance rate.
* **Economic:** cost/ton-km (mining/logistics); cost/ride (ride-hail); ROI months to payback.
* **Operational:** P95 dispatch latency; on-time arrival; charger queue time; energy $/km.
* **Experience (ride-hail):** CSAT, cancellation rate, pickup ETA P95, accessibility completion.

**OKR examples (Yr-1).**

* **O:** Demonstrate sector-agnostic scale in ME.

  * **KR1:** Deploy ≥3 sectors, ≥2 countries, ≥250 vehicles under one Fleet OS.
  * **KR2:** Achieve **≤0.3 assists/1k km** across all sectors' ODDs for 90-day period.
  * **KR3:** Positive ROI (<18 months) evidenced for first 2 customers per sector.

---

### 9) Problem statements → metrics

### Defense

* **P-D1:** Convoys fail in GNSS-denied segments.
  **Metric:** `% GNSS-denied km with maintained localization ≥ X confidence`.
* **P-D2:** Tele-assist delayed by SAT latency.
  **Metric:** `P95 Q&A turnaround < 2 s` within policy budget.
* **P-D3:** Data provenance gaps erode trust.
  **Metric:** `% map updates with signed provenance & review window ≤ 48 h`.

### Mining

* **P-M1:** Heat derating reduces throughput.
  **Metric:** `tons/hour vs ambient temp curve slope ≤ −ε`.
* **P-M2:** Dust occlusion spikes assists.
  **Metric:** `assists/1k km in visibility < Vmin` (down 30% vs baseline).
* **P-M3:** Unplanned downtime.
  **Metric:** `PdM precision/recall ≥ 0.8/0.8` on top 10 failure modes.

### Logistics/Supply chain

* **P-L1:** Yard congestion from unmanaged queues.
  **Metric:** `gate-to-dock time P95 ≤ target`.
* **P-L2:** Fragmented IT (WMS/TOS).
  **Metric:** `manual re-entry tasks/day → 0`.
* **P-L3:** Energy cost volatility.
  **Metric:** `$ per move reduced 10–15% via tariff-aware scheduling`.

### Ride-hailing

* **P-R1:** Rider trust during anomalies.
  **Metric:** `CSAT on assisted events ≥ 4.6`.
* **P-R2:** Work-zone ambiguity.
  **Metric:** `construction-related assists reduced 40% q/q`.
* **P-R3:** Accessibility gaps.
  **Metric:** `ADA/PRM fulfillment ≥ 98%` in ODD.

---

### 10) Design principles & hard trade-offs (with policies)

1. **Provenance vs. Freshness.**
   **Policy:** score incoming geo/weather data on `credibility × freshness`; workflows prefer **credible** unless freshness surpasses threshold *and* risk score < policy limit. All decisions logged.

2. **Autonomy vs. Assist.**
   **Policy:** per-sector assist budget (time & frequency). Exceeding budget → **auto rollback** last build + red gate.

3. **Edge vs. Cloud.**
   **Policy:** safety-critical logic on edge; cloud only for planning/analytics. Cloud outage must **not** reduce minimal safe behavior.

4. **Retrofit vs. Purpose-built.**
   **Policy:** default retrofit; allow purpose-built only when per-km TCO proves ≥15% improvement at scale and ADR approves.

5. **Security vs. Operability.**
   **Policy:** all comms mTLS + PKI; OTA signed and staged; no exceptions. Air-gapped workflows available for defense tenants.

6. **Uniformity vs. Customization.**
   **Policy:** customer specifics expressed **exclusively** in `configs/` + `rules/` overlays; forks forbidden by CI.

---

### 11) Ethics & guardrails

* **Do-no-harm:** geofences and ROE encoded; no lethal payload control.
* **Privacy-by-default:** least data; retention limits; PII masking on edge; audit trails.
* **Transparency:** incident explainers; rider/mission log redaction rules; **model cards** for deployed ML.
* **Fair access:** accessibility & language/RTL support; price fairness policies in ride-hail.

Compliance hooks live in `docs/safety/` and `compliance/`.

---

### 12) Regulatory posture (adaptive)

* **Safety case automation:** evidence bundles **auto-generated** per release (`compliance/audit-bundles/`).
* **Jurisdiction overlays:** local speed, lane rules, AV permits in `rules/regulatory/*`.
* **Change control:** any rule change → RFC + ADR + sim gate pass required.
* **Law-enforcement protocols:** standardized interactions; safe-stop zones; contact trees.

---

## 12) Data strategy

* **Geospatial database** with **immutable provenance**, multi-resolution tiles, conflict resolution (source credibility graph), freshness SLAs.
* **Telemetry contract**: versioned schemas (Avro/Proto), backward-compat tests.
* **Model lifecycle:** precision/recall tracked; population drift sentry; **shadow/canary** serving; automatic rollback on red metrics.
* **Weather fusion:** 1P/2P/3P blend; **gap-fill** with on-vehicle sensors; confidence tags consumed by routing.
* **Labeling:** active-learning loop; **scenario miner** creates test assets for sim; label QA with gold-set audits.

---

### 13) Engineering architecture

* **Services**: policy engine, dispatch, routing, rebalancing, energy, fleet-health, predictive-maint, map, weather, v2x, OTA, alerts.
* **Edge**: **ROS2-based** vehicle agent with **containerized nodes**, tele-assist client (Q&A only), diagnostics agent (snapshots, SBOM).
* **Decision Framework**: **Hybrid approach** with behavior trees, rule-based safety arbitration, and learned priors; explainable decisions.
* **Reliability:** idempotent APIs, backpressure, circuit breakers; **offline-first** modes with 45-minute autonomy.
* **Security:** SBOMs per release, image signing, mTLS, secret rotation; **zero-trust** network with ISO 21434 compliance.
* **Observability:** golden signals per service; SLOs enforced; incident playbooks pre-wired; ROS2 node monitoring.

---

### 14) UX/HMI

* **Control Center.** Left rail (trip types, statuses), live map, fleet timeline, quick filters (ODD breaches, assists), **RTL & Arabic** support, WCAG 2.2 AA.
* **Vehicle detail.** VIN, auto/manual, today's L4 hours & km, heartbeat, MPI, event feed.
* **Add trip.** Trip type, vehicle ID, schedule, driver/owner (if any), duration, advanced: static map ver., ODD, assist budget, experimental routes, operational city, VPN, road-graph ver.
* **Garage PC.** Bay/slot status, disk/firmware staging, SIM status, work logs.
* **Rider/Mission UX.** Clear comms; explainers for assists; privacy & consent.

**Design acceptance:** every flow has **empty/edge/error** states; red routes scripted.

---

### 15) QA & Safety

* **Twin-Gated CI/CD:** **CARLA/Gazebo simulation** scenarios must pass sector/ODD minimums; regression diffs gated.
* **Scenario Bank:** Comprehensive OpenSCENARIO-based test scenarios across sectors, vehicle types, and environmental conditions.
* **E2E tests:** create→dispatch→route→complete across overlays; soak tests in heat/dust sims.
* **Assist analysis:** taxonomy + root-cause pipeline with weekly auto-report; red threshold creates blocking ticket.
* **HARA/STPA:** hazards documented; mitigations prove measurable risk reduction.
* **Field validation:** pilot scorecard with **resource-independent** checks (automated logs/audits).

---

### 16) Risks, mitigations, contingencies, fail-fast

| Risk | Likelihood/Impact | Impact Metrics | Mitigation (designed-in) | Contingency | Tripwire (auto) | Mitigation Timeline |
| --- | --- | --- | --- | --- | --- | --- |
| Sensor occlusion (dust) | M/H | Perception confidence <80%; false positives >2% | Sensor redundancy; dust-aware fusion; wiper/air-knife control | Reroute to low-dust corridors; safe-stop | Occlusion rate > policy → route avoid; if persistent → halt | Immediate detection; <30s rerouting; <90s safe-stop |
| Heat derating | M/H | Compute utilization >90%; thermal throttling >5% | Thermal modeling; heat-aware dispatch & charge | Night-shift bias; staged cool-downs | SOC/thermal breach → de-rate speed, pull to shade | <60s detection; <5min rerouting to shade |
| Comms outage | H/M | Packet loss >10%; latency >500ms | Offline-first; SAT fallback; store-and-forward | Delay non-critical uploads; safe-stop areas | Cloud RTT > X → edge-only mode; alerts | <10s detection; <45min offline operation |
| Regulatory change | M/M | Compliance gap >0; permit at risk | Rule overlays; evidence automation | Freeze zone; engage authority | Policy mismatch at build time blocks deploy | <24h policy update; <7d full compliance |
| Security breach attempt | L/H | IDS alerts; anomalous access patterns | mTLS, attestation, signed OTA; IDS | Rotate keys; isolate tenant; forensic bundle | SIEM alert + auto-quarantine workload | <5min detection; <30min containment |
| Assist over-use | M/M | Assist rate >2/1,000km; assist budget >80% | Assist budget & coaching; scenario mining | Feature rollback; extra sim coverage | Budget exceeded → red gate + rollback | <1h detection; <24h rollback if needed |

---

### 17) Assumptions (versioned) & dependencies

* **L4 only** in geofenced ODDs with tele-assist Q&A allowed within policy.
* **Vehicle interfaces** available (DBW or kit adapters).
* **Legal permits** for each pilot region secured.
* **Connectivity**: at least one of Private LTE/5G, Wi-Fi, SAT present (RPO/RTO defined).
* **Thermal envelope**: kit spec supports sustained 55 °C ambient with peak management.

> Stored in `docs/strategy/assumptions.yaml` with **staleness check CI**. Each assumption links to ≥1 ADR.

---

### 18) Validation plan

| Function | Validation Responsibility | Validation Method | Success Criteria | Failure Response |
| --- | --- | --- | --- | --- |
| **Product Mgmt** | KPI coverage audit; ROI calculator validation | Gap analysis; sensitivity testing | 100% features with KPIs; ROI model accuracy ±10% | Feature freeze until KPI binding; model recalibration |
| **Design** | Usability tests; cognitive load assessment | User studies; heuristic evaluation | SUS ≥80; cognitive load ≤4/7; error recovery ≤3 steps | UX remediation sprint; design pattern library update |
| **Brand/Marketing** | Value messaging alignment; crisis preparedness | Message testing; crisis simulation | Message comprehension ≥90%; crisis response time ≤30 min | Messaging refinement; crisis playbook update |
| **Engineering** | Performance budgets; fault injection; rollback testing | Automated testing; chaos engineering | Latency/resource targets met; 100% graceful degradation | Performance optimization sprint; architecture review |
| **Data** | Lineage verification; drift detection; model card completeness | Automated validation; peer review | 100% data with lineage; drift detection latency ≤24h | Data quality task force; model retraining |
| **QA** | Scenario coverage; assist root-cause analysis | Coverage analysis; incident investigation | ≥95% scenario coverage; MTTR ≤24h for critical issues | Scenario expansion sprint; root cause analysis process improvement |
| **Security** | Threat modeling; penetration testing; SBOM verification | STRIDE/LINDDUN analysis; red team exercises | Zero P1 findings; SBOM completeness 100% | Security remediation sprint; dependency review |
| **Ops** | Runbook verification; alert coverage; SLA validation | Disaster recovery drills; alert testing | 100% critical paths with runbooks; alert accuracy ≥95% | Runbook enhancement sprint; monitoring coverage expansion |
| **Legal/Compliance** | Safety case review; permit verification; audit readiness | Documentation review; mock audits | Zero compliance gaps; audit readiness score ≥90% | Compliance remediation sprint; documentation enhancement |

**Governance**: Monthly Program Review with automated metrics dashboard. Any **red** metric auto-blocks next release train until resolved with verified fix and root cause analysis.

---

### 19) Success criteria & phase gates

1. **Alpha (internal):** All core services up; sim gates green; ODD for first sector encoded; no forking.
2. **Pilot-A (closed site):** ≥20 vehicles; **≥97% uptime**, assists ≤0.7/1k km; safety evidence bundle pass.
3. **Pilot-B (customer):** ≥50 vehicles; integrations live; **ROI trajectory** proves positive.
4. **Prod-1:** ≥100 vehicles; **≤0.3 assists/1k km**; OKRs trending green; audits passed.
5. **Scale:** Multi-tenant; 3+ sectors; 2+ countries; shared release train stable.

Each gate enforced by CI "twin-gates" job + Program Board sign-off.

---

### 20) Glossary (excerpt)

* **ODD:** Operational Design Domain.
* **Assist:** Human tele-assist Q&A input incorporated by autonomy; **no remote driving**.
* **Tele-assist budget:** Policy-set cap on assist frequency/duration per km/time.
* **Provenance:** Source identity, trust score, and change history for data (maps, weather, labels).
* **PdM:** Predictive maintenance; RUL: remaining useful life.
* **Safe-stop:** System-initiated halt in validated safe area.
* **ADR:** Architecture Decision Record (non-code policy).
* **ROS2:** Robot Operating System 2; framework for on-vehicle software.
* **Behavior Tree:** Hierarchical structure for decision-making logic.
* **Twin-Gated CI/CD:** Pipeline requiring simulation scenario validation.
* **Lanelet2/OpenDRIVE:** Standard HD map formats supported by the system.

Full glossary: `docs/strategy/glossary.md`

---

### Bottom line
It is **deliberately engineered to leave no gaps**: every promise in this Vision & Problem Statement is backed by a measurable KPI, a policy, an ODD rule, or an automated gate. If it's not encoded, it doesn't ship. This ensures strategic clarity, technical integrity, safety, and scalability across **defense, mining, logistics, and ride-hailing**—under the harsh realities of the Middle East and beyond.







