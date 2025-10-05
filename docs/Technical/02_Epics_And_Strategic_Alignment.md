## 02_Epics_and_Strategic_Alignment.md

**Positioning:** *AtlasMesh is a Fleet OS — not an autonomy stack.* We orchestrate mixed fleets, decisions, evidence, and operations across vendors, vehicles, sensors, maps, and clouds.

---

### 1) Epic-to-Problem Mapping

| Core Problem                           | Primary Epic(s)                                                               | Secondary Epic(s)                                   | Customer Impact                                                  |
| -------------------------------------- | ----------------------------------------------------------------------------- | --------------------------------------------------- | ---------------------------------------------------------------- |
| Environmental **Brittleness**          | **E3 Dispatch & Dynamic Ops**, **E7 Reliability & Degradation**               | E12 Map/Weather/Comms Agnosticism                   | ↑ Uptime, safe continuity under dust/heat/GNSS-denied conditions |
| **Integration Hell** (6+ months / OEM) | **E1 Vehicle-Agnostic Control & ODD Gate**, **E5 Sector Packs & Marketplace** | E12 Interop/Portability                             | ↓ Onboarding time & cost; multi-OEM flexibility                  |
| **Compliance Nightmare**               | **E2 Policy & Compliance Automation**, **E6 Evidence & Safety Case**          | E10 Privacy/Security/Zero-Trust                     | Audit-ready by default; lower risk & prep cost                   |
| **Vendor Lock-in**                     | **E1 Vehicle-Agnostic**, **E5 Sector Packs**                                  | E12 Map/Weather/Cloud Agnosticism                   | Procurement leverage; avoid single-source risk                   |
| Operational **Inefficiency**           | **E3 Dispatch & Dynamic Ops**, **E4 Tele-Assist & Control Center UX**         | E11 Cost/Carbon & Analytics, E8 Data & Digital Twin | ↓ Assists, ↑ throughput, measurable TCO reduction                |

---

### 2) Epic Overview & Strategic Mapping

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

---

### 3) Now / Next / Later Roadmap (with gates & risks)

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

* **E11** (advanced optimization & finance overlays), extended sector packs, CITY add-on expansions.
* **Gates:**

  * G-6: **NFR-DQ-01/02** stable for 2 quarters; data lineage complete.

**Risk links:** RR-001 (OEM API drift) → E1/E5; RR-003 (perf limits) → E3/E7/E12; RR-005 (evidence overhead) → E6. Mitigations per PRD §10.

---

### 4) Metrics Canon (MET.*) & Crosswalk

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

---

### 5) Geographic & Partner Phasing (E5/E6 alignment)

| Phase                 | Regions & Archetype Sites                      | Priority Epics        | Partner Motifs                                        | Exit Criteria                                                    |
| --------------------- | ---------------------------------------------- | --------------------- | ----------------------------------------------------- | ---------------------------------------------------------------- |
| **ME-1 (Launch)**     | UAE/KSA mining, defense logistics yards, ports | E1/E2/E3/E4/E6/E7/E12 | Gov & critical infrastructure operators; regional SIs | **MET.TTV.SITE_GOLIVE_DAYS ≤ 90**, **MET.SAF.ASSIST_RATE ≤ 0.5** |
| **APAC-2 (Scale)**    | SG/JP ports & RH pilots; AU mining             | E5/E8/E9              | OEM alliances, telcos, port authorities               | Two multi-OEM sites live; **MET.PERF.ROUTE_P95 ≤ 5s**            |
| **GLOBAL-3 (Expand)** | EU/US logistics, micro-transit & city add-ons  | E11 + CITY overlays   | City DOTs, curb APIs, major 3PLs                      | **MET.BIZ.SLA_ATTAIN% ≥ 98%** for 2 consecutive quarters         |

> **Evidence readiness (E6)** follows phase cadence: audit-ready bundles are required to progress between phases.

---

### 6) Financial Tie-Ins (per Epic) — pointer to `07_Business_Model_and_Financials.md`

| Epic    | Primary Revenue Levers                                                          | Secondary Levers / Cost Impact                    | Notes                                          |
| ------- | ------------------------------------------------------------------------------- | ------------------------------------------------- | ---------------------------------------------- |
| **E1**  | **Per-vehicle Fleet OS subscription**; Professional services for OEM onboarding | ↓ Integration COGS; ↓ CAC via repeatable adapters | Directly shortens sales→value timeline         |
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
| **E12** | Included; **Map/weather provider switching service**                            | ↓ Vendor costs; procurement leverage              | Locks in “agnostic” value prop                 |

---

### 7) Traceability View (Strategy → Epics → PRD)

* **Safety (O-1):** E1/E2/E3/E4/E6/E7/E8/E9/E10/E12 → FR-003/004/009/011/020/041/042/043/060/062; NFR-S-01/02/03.
* **Time-to-Value (O-2):** E1/E5/E9/E12 → FR-001/002/015/051/063; NFR-Port-01/02/03.
* **Scale & Cost (O-3):** E3/E7/E8/E11 → FR-012/025/026/039/040/058; NFR-Sc-01/02, NFR-P-06.
* **Regulatory (O-4):** E2/E6/E10 → FR-009/011/029/033/064; NFR-Comp-01/02/03, NFR-Priv-01/03.
* **UX (O-5):** E4 + supporting epics → FR-020/021/022/031/032/048/049; NFR-U-01.

---

### 8) Verification & Exit Gates per Epic

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

---

### 9) Ownership & RASCI (abbrev.)

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

---

### 10) Pointers to PRD & Specs (keep this doc lean)

* Functional/NFR detail, APIs, and test specs: **see PRD v3.0** (sections 3, 4, 7, 11).
* Financials & comps: **see `07_Business_Model_and_Financials.md`** (for ARR levers, margins).
* Risks, assumptions, open questions: **see PRD §10** (RR-* references used above).

---

#### Appendix A — Quick Index (Epic ↔ FR/NFR anchors)

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

---

