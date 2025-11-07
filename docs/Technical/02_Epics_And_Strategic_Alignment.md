<div align="center">

# 🎯 Epics and Strategic Alignment

**AtlasMesh is a Fleet Management System — not an autonomy stack.** We orchestrate mixed fleets, decisions, evidence, and operations across vendors, vehicles, sensors, maps and clouds.

</div>

---

## 📋 Table of Contents

<div align="center">

| 🎯 **[Epic-to-Problem Mapping](#1-epic-to-problem-mapping)** | 📊 **[Epic Overview](#2-epic-overview--strategic-mapping)** | 🚀 **[Strategic Alignment](#3-strategic-alignment)** | 📈 **[Success Metrics](#4-success-metrics)** |
|:---:|:---:|:---:|:---:|
| **Problem-Solution Mapping** | **Epic Details & Impact** | **OKR Alignment** | **Measurement Framework** |

</div>

---

## 🎯 **1) Epic-to-Problem Mapping**

<div align="center">

| 🌪️ **Environmental Brittleness** | 🔗 **Integration Hell** | 📋 **Compliance Nightmare** | 🔒 **Vendor Lock-in** | ⚡ **Operational Inefficiency** |
|:---:|:---:|:---:|:---:|:---:|
| **Primary Epics** | **Primary Epics** | **Primary Epics** | **Primary Epics** | **Primary Epics** |
| E3 Dispatch & Dynamic Ops | E1 Vehicle-Agnostic Control | E2 Policy & Compliance | E1 Vehicle-Agnostic | E3 Dispatch & Dynamic Ops |
| E7 Reliability & Degradation | E5 Sector Packs & Marketplace | E6 Evidence & Safety Case | E5 Sector Packs | E4 Tele-Assist & Control Center |
| **Secondary Epics** | **Secondary Epics** | **Secondary Epics** | **Secondary Epics** | **Secondary Epics** |
| E12 Map/Weather/Comms Agnosticism | E12 Interop/Portability | E10 Privacy/Security/Zero-Trust | E12 Map/Weather/Cloud Agnosticism | E11 Cost/Carbon & Analytics |
| | | | | E8 Data & Digital Twin |
| **Customer Impact** | **Customer Impact** | **Customer Impact** | **Customer Impact** | **Customer Impact** |
| ↑ Uptime, safe continuity under dust/heat/GNSS-denied | ↓ Onboarding time & cost; multi-OEM flexibility | Audit-ready by default; lower risk & prep cost | Procurement leverage; avoid single-source risk | ↓ Assists, ↑ throughput, measurable TCO reduction |

</div>

## ✅ **1.1) Epic Definition of Ready**

Before any Epic enters development, it must meet these criteria:

<div align="center">

| 📊 **Evidence Pack Requirements** | 💰 **Variant Budget Estimates** | 🎯 **SLI Targets Defined** | 🚀 **Rollout/Rollback Plans** |
|:---:|:---:|:---:|:---:|
| 5-7 contextual interviews (ops/safety/maintenance) OR | Code delta ≤5% per dimension | Performance SLIs with thresholds | Feature flags configured |
| Telemetry analysis (30-day slice) OR | Test delta ≤25% per dimension | Safety SLIs with gates | Canary/pilot plan defined |
| Sim/twin scenario impact analysis | Variant budget CI check passed | Reliability SLIs with SLOs | Rollback procedure tested |
| Policy & compliance review completed | CCB approval obtained (if budget exceeded) | Business SLIs with baselines | Kill-switch functionality validated |

</div>

---

## 📊 **2) Epic Overview & Strategic Mapping**

> Abbreviations: OKRs O-1..O-5 as in PRD; FR/NFR refer to PRD requirement IDs. Metrics are labeled **MET.***

| Epic ID | Epic Name                               | Description                                              | Strategic Objectives | Primary Sectors         | Pri | Business Impact Metrics                                   | Verification Methodology                           | Failure Consequences                                |
| ------- | --------------------------------------- | -------------------------------------------------------- | -------------------- | ----------------------- | :-: | --------------------------------------------------------- | -------------------------------------------------- | --------------------------------------------------- |
| **E1**  | Vehicle-Agnostic Control & ODD Gate     | Unified HAL, profile load ≤30s, ODD preflight, safe-stop | O-2, O-1             | CORE, MIN, DEF, LGX, RH |  P0 | **MET.TTV.SENSOR_TIME_DAYS**, **MET.SAF.ASSIST_RATE**     | FR-001/002/003/004 tests; twin gates; field pilots | Onboarding delays; safety blocks; OEM lock-in risk  |
| **E2**  | Policy & Compliance Automation          | Real-time policy eval + versioning + immutable audits    | O-4, O-1             | ALL                     |  P0 | **MET.REG.AUDIT_PASS%**, **MET.PERF.POLICY_P99**          | FR-009/010/011; NFR-Comp-01/02; audit drills       | Audit findings; halted missions; regulator scrutiny |
| **E3**  | Dispatch, Routing & Dynamic Ops         | Trip FSM, multi-objective routing, fast re-route         | O-3, O-1             | MIN, LGX, RH, MT, DEF   |  P0 | **MET.PERF.ROUTE_P95**, **MET.OPS.THRUPUT%**              | FR-040/041/006/012/014; A/B on live sites          | Missed SLAs; throughput loss; cost overrun          |
| **E4**  | Tele-Assist & Control Center UX         | Triage, SA bundle ≤15s, role-safe assist                 | O-1, O-5             | ALL                     |  P0 | **MET.SAF.ASSIST_RATE**, **MET.UX.SUS**                   | FR-080/081/041; NFR-U-01; UX tests                 | Incident risk; operator fatigue; churn              |
| **E5**  | Sector Packs & Marketplace Integrations | RH/LGX/MIN/DEF presets; ERP/WMS/TOS/map adapters         | O-2, O-3             | RH, LGX, MIN, DEF, MT   |  P1 | **MET.TTV.SITE_GOLIVE_DAYS**, **MET.INTEG.MAP_TIME_DAYS** | FR-037/038/063; contract tests                     | Slow time-to-value; SIs blocked                     |
| **E6**  | Evidence, Safety Case & SLA Reporting   | Continuous evidence, release bundles, SLA exports        | O-4, O-3             | ALL                     |  P0 | **MET.REG.AUDIT_PASS%**, **MET.BIZ.SLA_ATTAIN%**          | FR-029/030/031/047; NFR-Comp-01/02                 | Release blocks; SLA disputes/credits                |
| **E7**  | Reliability, SRE & Degradation Modes    | Degraded ops (network/sensor/compute), failover          | O-1, O-3             | ALL                     |  P0 | **MET.REL.UPTIME%**, **MET.REL.MTTR**                     | NFR-R-01/03/04; FR-043/042/058                     | Downtime; unsafe halts; penalties                   |
| **E8**  | Data Platform & Digital Twin            | Event bus, telemetry, twin gates, scenario packs         | O-1, O-3             | ALL                     |  P1 | **MET.OBS.COVERAGE%**, **MET.SIM.GATE_PASS%**             | FR-036/058/019/062; CI gates                       | Regressions reach prod; blind spots                 |
| **E9**  | OTA & Release Engineering               | Signed, staged OTA; ring rollout; rollback               | O-2, O-1             | ALL                     |  P0 | **MET.DEV.DEPLOY_DUR_MIN**, **MET.SEC.OTA_INTEGRITY%**    | FR-028; NFR-Sec-03; DRills                         | Fleet brick risk; security exposure                 |
| **E10** | Privacy, Security & Zero-Trust          | mTLS, RBAC/ABAC, residency, brand segmentation           | O-4, O-1             | ALL                     |  P0 | **MET.SEC.CVE_DAYS**, **MET.PRIV.VIOLATIONS=0**           | FR-030/033/052/064; NFR-Sec/Priv                   | Breaches; regulator fines; halted ops               |
| **E11** | Cost/Carbon & Analytics                 | Trip/site cost & CO₂ ledgers; optimization               | O-3                  | ALL                     |  P2 | **MET.BIZ.TCO_DELTA%**, **MET.BIZ.CO2EQ_DELTA%**          | FR-040; data QA                                    | ROI unclear; buyer objections                       |
| **E12** | Map/Weather/Comms Agnosticism           | Multi-map, weather fusion, comms failover                | O-1, O-2             | ALL                     |  P0 | **MET.PERF.REROUTE_S**, **MET.NET.FAILOVER_S**            | FR-015/016/017/018; NFR-Port-02/03                 | Fragile ops; single-vendor brittleness              |
| **E16** | Multi-Fleet Coordination                | Cross-fleet resource sharing, fleet federation, coordination protocols | O-1, O-3             | ALL                     |  P1 | **MET.FLEET.COORDINATION_EFFICIENCY%**, **MET.FLEET.RESOURCE_UTILIZATION%** | FR-076; NFR-P-01; fleet coordination tests         | Resource conflicts; fleet isolation failures         |
| **E17** | Mission Management & Orchestration      | Mission templates, dependencies, scheduling, analytics  | O-1, O-3             | ALL                     |  P1 | **MET.MISSION.SUCCESS_RATE%**, **MET.MISSION.EXECUTION_TIME** | FR-077/078; mission orchestration tests            | Mission failures; dependency deadlocks              |
| **E18** | Fleet Optimization & Intelligence       | Multi-objective optimization, dynamic rebalancing, cost optimization | O-3                   | ALL                     |  P1 | **MET.FLEET.OPTIMIZATION_GAIN%**, **MET.FLEET.COST_REDUCTION%** | FR-079; optimization algorithm tests                | Optimization loops; cost overruns                   |
| **E19** | Fleet Analytics & Health Management     | Health scoring, efficiency metrics, predictive analytics | O-1, O-3             | ALL                     |  P1 | **MET.FLEET.HEALTH_SCORE**, **MET.FLEET.PREDICTIVE_ACCURACY%** | FR-080; analytics validation tests                 | False positives; prediction failures                |
| **E20** | Fleet Resource Management               | Resource allocation, optimization, monitoring, planning  | O-3                   | ALL                     |  P1 | **MET.FLEET.RESOURCE_EFFICIENCY%**, **MET.FLEET.ALLOCATION_TIME** | FR-081; resource management tests                   | Resource starvation; allocation conflicts            |
| **E21** | Fleet Performance Management            | Performance monitoring, optimization, reporting, benchmarking | O-1, O-3             | ALL                     |  P1 | **MET.FLEET.PERFORMANCE_SCORE**, **MET.FLEET.BENCHMARK_RANKING** | FR-082; performance validation tests                | Performance degradation; benchmark failures          |

---

## 🗺️ **3) Now / Next / Later Roadmap (with gates & risks)**

**Now (execution window 0–2 quarters)**

* **E1, E2, E3, E4, E7, E9, E12** (Foundational).
* **Dependency gates:**

  * G-1: **NFR-P-02** (Policy P99 ≤10ms) green before E2 → E3 full-enable.
  * G-2: **NFR-R-04** (Geo failover <60s) green before any E9 ring>25%.
  * G-3: **FR-001/002** conformance ≥95% OEM variants before first multi-OEM site.

**Next (2–4 quarters)**

* **E5, E6, E8** (scale, compliance at speed).
* **Gates:**

  * G-4: **FR-036** twin gates ≥95% deterministic; **MET.SIM.GATE_PASS% ≥ 98%**.
  * G-5: **NFR-Comp-01** 100% evidence completeness for two consecutive releases.

**Later (4+ quarters)**

* **E11** (advanced optimization & finance overlays), **E16-E21** (fleet management capabilities), extended sector packs, CITY add-on expansions.
* **Gates:**

  * G-6: **NFR-DQ-01/02** stable for 2 quarters; data lineage complete.
  * G-7: **FR-076-082** fleet management capabilities validated across all sectors.

**Risk links:** RR-001 (OEM API drift) → E1/E5; RR-003 (perf limits) → E3/E7/E12; RR-005 (evidence overhead) → E6. Mitigations per PRD §10.

---

## 📈 **4) Metrics Canon (MET.*) & Crosswalk**

| Metric ID                      | Definition / SLI                   | OKR     | FR/NFR Sources               |
| ------------------------------ | ---------------------------------- | ------- | ---------------------------- |
| **MET.SAF.ASSIST_RATE**        | assists / 1000 km (12-mo rolling)  | O-1     | NFR-S-01; FR-020/041/080/081 |
| **MET.SAF.CRITICAL_QTR**       | critical incidents per quarter     | O-1     | NFR-S-02; FR-004             |
| **MET.PERF.ROUTE_P95**         | P95 route calc time (s)            | O-3     | NFR-P-03; FR-040/041         |
| **MET.PERF.POLICY_P99**        | P99 policy eval latency (ms)       | O-4     | NFR-P-02; FR-009/010         |
| **MET.REL.UPTIME%**            | Control Center uptime (monthly)    | O-3     | NFR-R-01                     |
| **MET.REL.MTTR**               | Mean time to recovery (critical)   | O-3     | NFR-R-03                     |
| **MET.TTV.SITE_GOLIVE_DAYS**   | Days from contract → go-live       | O-2     | E5 aggregate; FR-037/038     |
| **MET.INTEG.SENSOR_TIME_DAYS** | Days to integrate new sensor       | O-2     | NFR-Port-01; E1              |
| **MET.INTEG.MAP_TIME_DAYS**    | Days to integrate new map provider | O-2     | NFR-Port-02; E12             |
| **MET.UX.SUS**                 | System Usability Scale score       | O-5     | NFR-U-01; E4                 |
| **MET.SEC.CVE_DAYS**           | Days to remediate critical CVEs    | O-4     | NFR-Sec-02; E10              |
| **MET.BIZ.SLA_ATTAIN%**        | % of contracted SLAs achieved      | O-3     | FR-047; E6                   |
| **MET.BIZ.TCO_DELTA%**         | Site TCO reduction vs. baseline    | O-3     | E3/E11                       |
| **MET.BIZ.CO2EQ_DELTA%**       | CO₂e reduction vs. baseline        | O-3     | FR-040; E11                  |
| **MET.SEC.OTA_INTEGRITY%**     | % OTA with attested integrity      | O-2/O-1 | NFR-Sec-03; E9               |
| **MET.SIM.GATE_PASS%**         | % scenarios passing twin gates     | O-1/O-3 | FR-036/062; E8               |
| **MET.FLEET.COORDINATION_EFFICIENCY%** | Cross-fleet coordination efficiency | O-1/O-3 | FR-076; E16                  |
| **MET.FLEET.RESOURCE_UTILIZATION%** | Fleet resource utilization rate    | O-3     | FR-081; E20                  |
| **MET.MISSION.SUCCESS_RATE%**  | Mission success rate percentage    | O-1/O-3 | FR-077/078; E17              |
| **MET.MISSION.EXECUTION_TIME** | Average mission execution time     | O-3     | FR-077/078; E17              |
| **MET.FLEET.OPTIMIZATION_GAIN%** | Fleet optimization improvement     | O-3     | FR-079; E18                  |
| **MET.FLEET.COST_REDUCTION%**  | Fleet cost reduction percentage    | O-3     | FR-079; E18                  |
| **MET.FLEET.HEALTH_SCORE**    | Fleet health score (0-100)         | O-1/O-3 | FR-080; E19                  |
| **MET.FLEET.PREDICTIVE_ACCURACY%** | Fleet predictive accuracy        | O-1/O-3 | FR-080; E19                  |
| **MET.FLEET.RESOURCE_EFFICIENCY%** | Fleet resource efficiency        | O-3     | FR-081; E20                  |
| **MET.FLEET.ALLOCATION_TIME** | Fleet resource allocation time     | O-3     | FR-081; E20                  |
| **MET.FLEET.PERFORMANCE_SCORE** | Fleet performance score (0-100)  | O-1/O-3 | FR-082; E21                  |
| **MET.FLEET.BENCHMARK_RANKING** | Fleet benchmark ranking          | O-1/O-3 | FR-082; E21                  |

---

## 🌍 **5) Geographic & Partner Phasing (E5/E6 alignment)**

| Phase                 | Regions & Archetype Sites                      | Priority Epics        | Partner Motifs                                        | Exit Criteria                                                    |
| --------------------- | ---------------------------------------------- | --------------------- | ----------------------------------------------------- | ---------------------------------------------------------------- |
| **ME-1 (Launch)**     | UAE/KSA mining, defense logistics yards, ports | E1/E2/E3/E4/E6/E7/E12 | Gov & critical infrastructure operators; regional SIs | **MET.TTV.SITE_GOLIVE_DAYS ≤ 90**, **MET.SAF.ASSIST_RATE ≤ 0.5** |
| **APAC-2 (Scale)**    | SG/JP ports & RH pilots; AU mining             | E5/E8/E9              | OEM alliances, telcos, port authorities               | Two multi-OEM sites live; **MET.PERF.ROUTE_P95 ≤ 5s**            |
| **GLOBAL-3 (Expand)** | EU/US logistics, micro-transit & city add-ons  | E11 + CITY overlays   | City DOTs, curb APIs, major 3PLs                      | **MET.BIZ.SLA_ATTAIN% ≥ 98%** for 2 consecutive quarters         |

> **Evidence readiness (E6)** follows phase cadence: audit-ready bundles are required to progress between phases.

---

## 💰 **6) Financial Tie-Ins (per Epic) — pointer to `07_Business_Model_and_Financials.md`**

| Epic    | Primary Revenue Levers                                                          | Secondary Levers / Cost Impact                    | Notes                                          |
| ------- | ------------------------------------------------------------------------------- | ------------------------------------------------- | ---------------------------------------------- |
| **E1**  | **Per-vehicle Fleet Management System subscription**; Professional services for OEM onboarding | ↓ Integration COGS; ↓ CAC via repeatable adapters | Directly shortens sales→value timeline         |
| **E2**  | **Compliance module add-on (per vehicle/site)**                                 | ↓ Audit prep cost; fewer penalties                | Drives stickiness, upsell to regulated sectors |
| **E3**  | **Optimization add-on (per vehicle/site)**                                      | ↓ Ops cost (fuel/idle/time) → ROI stories         | Underpins throughput SLAs for LGX/MIN          |
| **E4**  | **Operator console seats**                                                      | ↑ NRR via UX outcomes; ↓ support cost             | Seat-based + usage tiers                       |
| **E5**  | **Marketplace adapters (annual license)**; **Sector pack SKUs**                 | SI revenue share; faster expansion                | Aligns with regional partner plays             |
| **E6**  | **SLA reporting & evidence bundles**                                            | ↓ Legal risk; premium compliance tier             | Required for enterprise & public sector        |
| **E7**  | **Premium reliability tier** (enhanced SLOs)                                    | ↓ Downtime credits                                | Ties to contract SLAs                          |
| **E8**  | **Digital twin & analytics licenses**                                           | ↓ Regression risk (fewer rollbacks)               | Scenario packs by sector/brand                 |
| **E9**  | Included in core; **Enterprise release mgmt add-on**                            | ↓ Deployment toil; ↓ incident cost                | De-risks large fleet updates                   |
| **E10** | **Security/residency add-ons**                                                  | ↓ Breach risk; compliance unlocks                 | Mandatory in defense/public                    |
| **E11** | **Cost & carbon reporting add-on**                                              | Supports ESG bids; carbon credits narratives      | Adds CFO-friendly value                        |
| **E12** | Included; **Map/weather provider switching service**                            | ↓ Vendor costs; procurement leverage              | Locks in "agnostic" value prop                 |
| **E16** | **Multi-fleet coordination add-on (per fleet)**                                 | ↓ Resource waste; ↑ utilization efficiency        | Enables enterprise multi-fleet operations      |
| **E17** | **Mission management platform (per mission)**                                   | ↓ Mission planning time; ↑ success rates          | Mission-critical operations value               |
| **E18** | **Fleet optimization engine (per vehicle/site)**                                 | ↓ Operating costs; ↑ efficiency                   | ROI-driven optimization value                   |
| **E19** | **Fleet analytics & health monitoring (per fleet)**                              | ↓ Downtime; ↑ predictive maintenance             | Proactive fleet management value                 |
| **E20** | **Fleet resource management (per resource unit)**                               | ↓ Resource waste; ↑ allocation efficiency        | Resource optimization value                     |
| **E21** | **Fleet performance management (per fleet)**                                     | ↓ Performance issues; ↑ benchmarking              | Performance optimization value                   |

---

## 🔗 **7) Traceability View (Strategy → Epics → PRD)**

* **Safety (O-1):** E1/E2/E3/E4/E6/E7/E8/E9/E10/E12/E16/E17/E19/E21 → FR-003/004/009/011/020/041/042/043/060/062/076/077/080/082; NFR-S-01/02/03.
* **Time-to-Value (O-2):** E1/E5/E9/E12 → FR-001/002/015/051/063; NFR-Port-01/02/03.
* **Scale & Cost (O-3):** E3/E7/E8/E11/E16/E17/E18/E19/E20/E21 → FR-012/025/026/039/040/058/076/077/078/079/080/081/082; NFR-Sc-01/02, NFR-P-06.
* **Regulatory (O-4):** E2/E6/E10 → FR-009/011/029/033/064; NFR-Comp-01/02/03, NFR-Priv-01/03.
* **UX (O-5):** E4 + supporting epics → FR-020/021/022/031/032/048/049; NFR-U-01.

---

## ✅ **8) Verification & Exit Gates per Epic**

| Epic    | Must-Pass Gates (extract)                                                             |
| ------- | ------------------------------------------------------------------------------------- |
| **E1**  | FR-001/002 conformance ≥95% across targeted OEMs; **MET.INTEG.SENSOR_TIME_DAYS ≤ 14** |
| **E2**  | **MET.PERF.POLICY_P99 ≤ 10ms** under 1k eval/s; audit drill “green” ×2                |
| **E3**  | **MET.PERF.ROUTE_P95 ≤ 5s**; re-route ≤30s in ≥95% events                             |
| **E4**  | SA bundle ≤15s P95; **MET.UX.SUS ≥ 80** across roles                                  |
| **E5**  | Map/provider integration ≤21 days; ≥3 certified adapters live                         |
| **E6**  | Evidence bundle completeness 100% for two consecutive releases                        |
| **E7**  | **NFR-R-01 ≥ 99.5%**, **NFR-R-04 < 60s** in drills; degraded modes validated          |
| **E8**  | **MET.SIM.GATE_PASS% ≥ 98%** across sector scenario packs                             |
| **E9**  | Ring rollout with auto-rollback verified; **MET.SEC.OTA_INTEGRITY% = 100%**           |
| **E10** | **MET.SEC.CVE_DAYS** (crit ≤14) no breaches; residency assertions 100%                |
| **E11** | Demonstrated **MET.BIZ.TCO_DELTA% ≥ 12%** on at least two sites                       |
| **E12** | Failover ≤2s P95; weather fusion confidence gating live                               |
| **E16** | **MET.FLEET.COORDINATION_EFFICIENCY% ≥ 85%**; cross-fleet resource sharing validated |
| **E17** | **MET.MISSION.SUCCESS_RATE% ≥ 95%**; mission dependency resolution validated        |
| **E18** | **MET.FLEET.OPTIMIZATION_GAIN% ≥ 15%**; multi-objective optimization validated       |
| **E19** | **MET.FLEET.HEALTH_SCORE ≥ 80**; predictive accuracy ≥90% validated                 |
| **E20** | **MET.FLEET.RESOURCE_EFFICIENCY% ≥ 90%**; resource allocation validated             |
| **E21** | **MET.FLEET.PERFORMANCE_SCORE ≥ 85**; benchmark ranking validated                    |

---

## 👥 **9) Ownership & RASCI (abbrev.)**

* **E1** Platform Lead (Edge/BE) — R; QA/SRE — A; PM — S; OEM Partners — C; Security — I
* **E2** Policy Lead — R; Compliance — A; BE — S; Legal/Reg — C; SRE — I
* **E3** Routing Lead — R; BE/Maps — A; Ops — C
* **E4** UX Lead — R; FE — A; Safety/Training — C
* **E5** PM (Sector) — R; Partner Eng — A; SIs — C
* **E6** Compliance — R; DevOps — A; Data — S
* **E7** SRE — R; Platform — A; Safety — C
* **E8** Data/ML — R; QA — A; PM — C
* **E9** DevOps — R; Security — A
* **E10** Security — R; Privacy — A
* **E11** Data/PM — R; CFO — A
* **E12** Maps/Comms — R; SRE — A
* **E16** Fleet Lead — R; BE — A; PM — S; Security — C; SRE — I
* **E17** Mission Lead — R; BE — A; PM — S; UX — C; QA — I
* **E18** Optimization Lead — R; ML — A; BE — S; Data — C; QA — I
* **E19** Analytics Lead — R; Data — A; ML — S; BE — C; QA — I
* **E20** Resource Lead — R; BE — A; PM — S; Data — C; SRE — I
* **E21** Performance Lead — R; SRE — A; Data — S; BE — C; QA — I

---

## 📚 **10) Pointers to PRD & Specs (keep this doc lean)**

* Functional/NFR detail, APIs, and test specs: **see PRD v3.0** (sections 3, 4, 7, 11).
* Financials & comps: **see `07_Business_Model_and_Financials.md`** (for ARR levers, margins).
* Risks, assumptions, open questions: **see PRD §10** (RR-* references used above).

---

### Appendix A — Quick Index (Epic ↔ FR/NFR anchors)

* **E1:** FR-001/002/003/004/005; NFR-Port-01/02/03
* **E2:** FR-009/010/011; NFR-Comp-01/02/03, NFR-P-02
* **E3:** FR-006/012/040/041/014; NFR-P-03
* **E4:** FR-020/021/022/023/041/080/081; NFR-U-01
* **E5:** FR-037/038/051/063; NFR-I-01/-I-04
* **E6:** FR-029/030/031/047; NFR-Comp-*
* **E7:** FR-042/043/058; NFR-R-01/-R-03/-R-04
* **E8:** FR-019/036/062/058; NFR-Ob-01/02/03, NFR-Sc-01/02
* **E9:** FR-028; NFR-Sec-03, NFR-M-02
* **E10:** FR-030/033/052/064; NFR-Sec-01/02, NFR-Priv-01/03
* **E11:** FR-040; NFR-DQ-01/02
* **E12:** FR-015/016/017/018; NFR-Port-02/03
* **E16:** FR-076; NFR-P-01
* **E17:** FR-077/078; NFR-U-01
* **E18:** FR-079; NFR-P-03
* **E19:** FR-080; NFR-Ob-01/02/03
* **E20:** FR-081; NFR-Sc-01/02
* **E21:** FR-082; NFR-R-01/03

---

## 🏗️ **6) Epic-to-Service Mapping**

### **Service Coverage by Epic**

<div align="center">

| **Epic** | **Epic Name** | **Primary Services** | **Supporting Services** | **Total Services** |
|:---:|:---:|:---:|:---:|:---:|
| **E1** | Vehicle-Agnostic Control & ODD Gate | vehicle-agent, vehicle-hal, sensor-drivers | vehicle-interface, sensor-data-collector, safety-monitor | **6** |
| **E2** | Policy & Compliance Automation | policy-engine, compliance-automation | auditability, evidence-engine, security | **5** |
| **E3** | Dispatch, Routing & Dynamic Ops | dispatch-service, routing-service, fleet-manager | mission-management, hd-map-service, weather-fusion | **6** |
| **E4** | Tele-Assist & Control Center UX | control-center-ui, remote-operations | vehicle-gateway, comms-orchestration | **4** |
| **E5** | Sector Packs & Marketplace Integrations | wms-adapters, defense-adapters, mining-adapters, ride-hail-adapters | sector-overlays, erp-integration, external-integrations | **7** |
| **E6** | Evidence, Safety Case & SLA Reporting | evidence-engine, auditability, compliance-automation | data-lineage, security, conformance-testing | **6** |
| **E7** | Reliability, SRE & Degradation Modes | degraded-modes, chaos-engineering, monitoring-alerting | observability, oncall-runbooks, testing-framework | **6** |
| **E8** | Data Platform & Digital Twin | digital-twin, data-fusion, telemetry-ingestion | data-lineage, ml-pipeline, simulation | **6** |
| **E9** | OTA & Release Engineering | ota-manager, release-engineering | security, testing-framework, conformance-testing | **5** |
| **E10** | Privacy, Security & Zero-Trust | zero-trust-iam, security, key-secret-management | auth-service, security-hardening, purpose-binding-residency | **6** |
| **E11** | Cost/Carbon & Analytics | cost-optimization, fleet-analytics, energy-management | data-fusion, ml-pipeline, analytics-service | **6** |
| **E12** | Map/Weather/Comms Agnosticism | hd-map-service, weather-fusion, comms-orchestration | map-data-contract, sensor-pack-registry, platform-adapters | **6** |
| **E16** | Multi-Fleet Coordination | fleet-coordination, fleet-manager | fleet-optimization, fleet-resources, tenant-entitlements | **5** |
| **E17** | Mission Management & Orchestration | mission-management, mission-planning | path-planning, fleet-manager, dispatch-service | **5** |
| **E18** | Fleet Optimization & Intelligence | fleet-optimization, fleet-manager | fleet-analytics, ml-pipeline, cost-optimization | **5** |
| **E19** | Fleet Analytics & Health Management | fleet-analytics, analytics-service, predictive-maintenance | data-fusion, ml-pipeline, fleet-performance | **6** |
| **E20** | Fleet Resource Management | fleet-resources, fleet-manager | fleet-optimization, tenant-entitlements, resource-allocation | **5** |
| **E21** | Fleet Performance Management | fleet-performance, fleet-manager | fleet-analytics, performance-monitoring, benchmarking | **5** |

</div>

### **Coverage Summary**
- **Total Services**: 72
- **Total Epics**: 21
- **Average Services per Epic**: 5.7
- **Coverage**: 100% of services mapped to at least one epic
- **Overlap**: Services support multiple epics for comprehensive functionality

---
