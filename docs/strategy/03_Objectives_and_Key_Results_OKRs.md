<div align="center">

## 🎯 Objectives and Key Results (OKRs)

**Strategic Objectives and Measurable Key Results**

</div>

---

### 📋 Table of Contents

<div align="center">

| 🎯 **[Company Objectives](#a-company-objectives-commit--stretch)** | 🛡️ **[O1: Safety & Operations](#o1--prove-repeatable-l4-operations-in-harsh--controlled-odds)** | 💰 **[O2: Commercial Proof](#o2--demonstrate-roi-and-scalability-commercial-proof)** | 🌐 **[O3: Agnosticism](#o3--prove-were-credibly-agnostic-vehicle-sensor-map-cloud)** |
|:---:|:---:|:---:|:---:|
| **Strategic Objectives Overview** | **Safety & Reliability** | **Commercial Success** | **Platform Agnosticism** |

| 📋 **[O4: Regulatory](#o4--evidence--compliance-automation)** | 👤 **[O5: User Experience](#o5--operator-experience-excellence)** | 📊 **[Success Metrics](#b-success-metrics--measurement)** | 📚 **[References](#c-references--related-docs)** |
|:---:|:---:|:---:|:---:|
| **Compliance & Evidence** | **User Experience** | **Measurement Framework** | **Supporting Documentation** |

</div>

---

This file defines **what we commit to this period** and **how we score it**. All metric math/SQL lives only in the Metrics Canon.

---

### 🎯 **A) Company Objectives (commit → stretch)**

#### O1 — Prove repeatable L4 operations in harsh & controlled ODDs

| KR ID       | Key Result                                                      | Target (this period)                                                          | Measurement (SoT)                         | Linked Metrics (IDs)                                                                                 | Hard Gate                 |
| ----------- | --------------------------------------------------------------- | ----------------------------------------------------------------------------- | ----------------------------------------- | ---------------------------------------------------------------------------------------------------- | ------------------------- |
| **KR.O1.1** | 3 sector pilots sustain **production SLAs 60 consecutive days** | **3 sites** (1× Mining, 1× Logistics, 1× Defense)                             | Site dashboards; rolling 60-day validator | MET.RELIAB.AVAIL_ODD · MET.SAFETY.ASSIST_RATE · MET.SAFETY.ZERO_HARM · MET.GOV.AUDIT_BUNDLE_COMPLETE | Block if any S2/S3 safety |
| **KR.O1.2** | Availability **in ODD**                                         | **≥99.3%** (R30D)                                                             | `vw_ops_availability_daily`               | MET.RELIAB.AVAIL_ODD                                                                                 | Block <99.0%              |
| **KR.O1.3** | Assist & harm                                                   | **≤0.5 / 1k km** (SLA), **ZERO** critical harm                                | `vw_safety_events_daily`                  | MET.SAFETY.ASSIST_RATE · MET.SAFETY.ZERO_HARM                                                        | Block on critical         |
| **KR.O1.4** | Audit-ready releases & permits                                  | **≥6** releases at **100%** bundle completeness; **2** jurisdiction sign-offs | CI evidence scorer; regulator records     | MET.GOV.AUDIT_BUNDLE_COMPLETE · MET.GOV.AUDIT_FINDINGS_P1                                            | Block if <100%            |

**Owners:** VP Safety (A), Eng (R), Compliance (R), Product (R)

---

#### O2 — Demonstrate ROI and scalability (commercial proof)

| KR ID       | Key Result                               | Target (this period)                                                                         | Measurement (SoT)          | Linked Metrics (IDs)                                               |
| ----------- | ---------------------------------------- | -------------------------------------------------------------------------------------------- | -------------------------- | ------------------------------------------------------------------ |
| **KR.O2.1** | Sector KPI deltas achieved at live sites | Mining: **cost/ton −10%** • Logistics: **gate→dock P95 ≤ 30m** • Ride-hail: **ETA P95 ≤ 7m** | Canon views per sector     | MET.COST.COST_PER_TON · MET.OPS.GATE_TO_DOCK_P95 · MET.EXP.ETA_P95 |
| **KR.O2.2** | Commercial traction                      | **$XXM TCV** across **6 logos** (≥2/sector), **≥55% SW GM**                                  | CRM + Finance              | (Business KPI)                                                     |
| **KR.O2.3** | Install & cutover velocity               | **≤8h/vehicle** install; **≤14 days** site cutover                                           | Program tracker / runbooks | (Ops counters; see Ops OKRs)                                       |

**Owners:** CRO (A), VP Ops (R), PMO (R)

---

#### O3 — Prove we’re credibly agnostic (vehicle, sensor, map, cloud)

| KR ID       | Key Result                | Target (this period)                                                    | Evidence (SoT)                      |
| ----------- | ------------------------- | ----------------------------------------------------------------------- | ----------------------------------- |
| **KR.O3.1** | Vehicle classes certified | **5** (haul truck, yard tractor, 4×4 UGV, forklift, van)                | Adapter/policy certification logs   |
| **KR.O3.2** | Deploy-mode parity        | **3** modes (GCP/Azure/on-prem) with identical APIs                     | CI env matrix; conformance suite    |
| **KR.O3.3** | Sensor/map diversity      | **2×** lidar, **2×** camera stacks, **2×** GNSS/INS, **2×** map sources | HW/Data BOM; conformance tests      |
| **KR.O3.4** | Rules as code             | **100%** rules tested; **<24h** safe change                             | Policy test suite; change lead-time |

**Owners:** CTO (A), Platform Eng (R), Integrations (R)

---

### B) Workstream OKRs (feed O1–O3)

#### Product Management

| KR ID       | Key Result                 | Target                                             | Evidence         |
| ----------- | -------------------------- | -------------------------------------------------- | ---------------- |
| **KR.PM.1** | Sector **policy packs**    | 4 packs live; regulatory change absorbed **<48h**  | Policy repo CI   |
| **KR.PM.2** | Use-case spec coverage     | **100%** scenario rows → specs with sim/twin gates | Docs CI          |
| **KR.PM.3** | Ops NPS (site supervisors) | **≥50**                                            | Quarterly survey |

#### Design & UX

| KR ID       | Key Result              | Target                                               | Linked Metrics                         |
| ----------- | ----------------------- | ---------------------------------------------------- | -------------------------------------- |
| **KR.DS.1** | Accessibility           | **WCAG 2.2 AA** incl. RTL/Arabic                     | —                                      |
| **KR.DS.2** | Incident triage speed   | **≤60s P95** (usability tests)                       | (Field proxy) MET.OPS.DISPATCH_LAT_P95 |
| **KR.DS.3** | LE/First-Responder flow | Validated by **2** agencies; **0** miscomm incidents | MET.SAFETY.ZERO_HARM (proxy)           |

#### Engineering (Platform & Edge)

| KR ID        | Key Result        | Target                                         | Evidence        |
| ------------ | ----------------- | ---------------------------------------------- | --------------- |
| **KR.ENG.1** | Control-loop perf | **P95 ≤ 50 ms** on-vehicle; ETA error ≤ **8%** | Perf telemetry  |
| **KR.ENG.2** | OTA reliability   | Blue/green + attestation; rollback **<5 min**  | Release metrics |
| **KR.ENG.3** | CI twin gates     | Scenario pass **≥95%** for merge               | CI dashboard    |

#### Data / ML

| KR ID         | Key Result             | Target                                                | Linked Metrics                                         |
| ------------- | ---------------------- | ----------------------------------------------------- | ------------------------------------------------------ |
| **KR.DATA.1** | Provenance & loss      | **100%** vehicles scored; data loss **<0.1%**         | MET.DATA.PROVENANCE_SCORE (↑) · MET.DATA.LOSS_RATE (↓) |
| **KR.DATA.2** | PdM impact             | Unscheduled downtime **−20%**                         | MET.RELIAB.MTBF (↑) · MET.RELIAB.AVAIL_ODD (↑)         |
| **KR.DATA.3** | Demand/dispatch uplift | Seat-fill **+10%** (RH) or loader idle **−25%** (MIN) | MET.EXP.ETA_P95 (↓) · MET.PROD.TONS_PER_HOUR (↑)       |

#### QA & Safety

| KR ID       | Key Result          | Target                                                      | Linked Metrics                |
| ----------- | ------------------- | ----------------------------------------------------------- | ----------------------------- |
| **KR.QA.1** | Critical defects    | **0** P1 in prod; Sev-1 MTTR **<1h**                        | MET.OPS.MTTR                  |
| **KR.QA.2** | Evidence automation | **100%** HARA/STPA & bundles per release                    | MET.GOV.AUDIT_BUNDLE_COMPLETE |
| **KR.QA.3** | Drills              | EW/jam, sensor loss, LE stop, HAZMAT • fixes **<2 sprints** | Drill log                     |

#### Security

| KR ID        | Key Result  | Target                                                  | Linked Metrics                                     |
| ------------ | ----------- | ------------------------------------------------------- | -------------------------------------------------- |
| **KR.SEC.1** | mTLS + SBOM | **100%** releases signed; **no critical CVEs >15 days** | MET.SEC.SBOM_SIGNED · MET.SEC.P1_FINDINGS_OPEN (0) |
| **KR.SEC.2** | Red-team    | **2×/year**; **no P1 >30 days**                         | MET.SEC.INCIDENTS_P1 (0)                           |

#### Operations

| KR ID        | Key Result     | Target                                             | Evidence / Metrics   |
| ------------ | -------------- | -------------------------------------------------- | -------------------- |
| **KR.OPS.1** | Site readiness | Playbooks **100%**; avg readiness **≤10 biz days** | Runbook tracker      |
| **KR.OPS.2** | Spares & RMA   | Turnaround **<72h**                                | Ops tracker          |
| **KR.OPS.3** | On-call SLO    | Page-ack **P95 <5 min**                            | MET.OPS.MTTR (proxy) |

#### GTM & Partnerships

| KR ID        | Key Result          | Target                                                | Evidence        |
| ------------ | ------------------- | ----------------------------------------------------- | --------------- |
| **KR.GTM.1** | Logos & references  | **6** logos; **3** public case studies with KPI diffs | Case study pack |
| **KR.GTM.2** | Integrators enabled | **3** partners; time-to-first-POC **<45 days**        | Partner portal  |
| **KR.GTM.3** | TCO tools           | Sector TCO live; win rate **≥30%**                    | CRM             |

---

### C) Interdependencies & guardrails

| Objective / KR      | Depends On                   | Enables                  | Risk If Missed           | Catch-Up Plan                           |
| ------------------- | ---------------------------- | ------------------------ | ------------------------ | --------------------------------------- |
| **O1** Safety/Avail | ENG perf, QA gates, Security | **O2** ROI, **O3** Scale | Deployment halt          | Feature freeze; safety sprint; 3P audit |
| **O2** ROI/Scale    | O1 proven                    | Growth, margin           | Revenue delay; churn     | Deployment SWAT; phased rollout         |
| **O3** Agnostic     | Platform maturity            | Multi-sector reuse       | Vendor lock-in; BOM risk | Adapter blitz; BOM alternates           |

**Non-negotiable:** any safety/compliance metric **red** → pause non-remediation KRs until green.

---

### D) Scoring, reviews & evidence

* **Scoring:** 0.0–1.0; **0.7–0.8 = success**, 1.0 = stretch.
* **Reviews:** Monthly exec; bi-weekly program; weekly CCB/Safety for gated KRs.
* **Evidence per KR:** dashboard slice or SoT query + owner sign-off + artifact links (PRs, test runs, bundle hash).
* **Change control:** targets can change only via PMO change log; Canon IDs/formulas stay immutable.

---

### E) Crosswalk to Metrics Canon

(See the Canon file below for definitions/formulas.)

| KR        | Canon IDs                                                                                            |
| --------- | ---------------------------------------------------------------------------------------------------- |
| KR.O1.1   | MET.RELIAB.AVAIL_ODD · MET.SAFETY.ASSIST_RATE · MET.SAFETY.ZERO_HARM · MET.GOV.AUDIT_BUNDLE_COMPLETE |
| KR.O1.2   | MET.RELIAB.AVAIL_ODD                                                                                 |
| KR.O1.3   | MET.SAFETY.ASSIST_RATE · MET.SAFETY.ZERO_HARM                                                        |
| KR.O1.4   | MET.GOV.AUDIT_BUNDLE_COMPLETE · MET.GOV.AUDIT_FINDINGS_P1                                            |
| KR.O2.1   | MET.COST.COST_PER_TON · MET.OPS.GATE_TO_DOCK_P95 · MET.EXP.ETA_P95                                   |
| KR.DATA.1 | MET.DATA.PROVENANCE_SCORE · MET.DATA.LOSS_RATE                                                       |
| KR.QA.1   | MET.OPS.MTTR                                                                                         |
| KR.SEC.*  | MET.SEC.SBOM_SIGNED · MET.SEC.P1_FINDINGS_OPEN · MET.SEC.INCIDENTS_P1                                |

---

#### Appendix A — KR ↔ Canon Traceability (with SoT & Owners)

> Keep this near the end, after your OKR tables.

| KR                              | What it proves               | Canon IDs (no math here)                                                                             | Primary SoT view(s)                                                               | Accountable owner             |
| ------------------------------- | ---------------------------- | ---------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------- | ----------------------------- |
| **KR.O1.1** (3 pilots, 60d)     | Repeatable L4 ops            | MET.RELIAB.AVAIL_ODD · MET.SAFETY.ASSIST_RATE · MET.SAFETY.ZERO_HARM · MET.GOV.AUDIT_BUNDLE_COMPLETE | `vw_ops_availability_daily`, `vw_safety_events_daily`, `vw_governance_kpis_daily` | VP Safety (A), Site Leads (R) |
| **KR.O1.2** (Avail ≥99.3%)      | Reliability in ODD           | MET.RELIAB.AVAIL_ODD                                                                                 | `vw_ops_availability_daily`                                                       | Ops Lead                      |
| **KR.O1.3** (Assist/harm)       | Safe autonomy load           | MET.SAFETY.ASSIST_RATE · MET.SAFETY.ZERO_HARM                                                        | `vw_safety_events_daily`                                                          | Safety Lead                   |
| **KR.O1.4** (Bundles/permits)   | Evidence & compliance        | MET.GOV.AUDIT_BUNDLE_COMPLETE · MET.GOV.AUDIT_FINDINGS_P1                                            | `vw_governance_kpis_daily`                                                        | Compliance Lead               |
| **KR.O2.1** (ROI deltas)        | Sector value                 | MET.COST.COST_PER_TON · MET.OPS.GATE_TO_DOCK_P95 · MET.EXP.ETA_P95                                   | `vw_cost_kpis_daily`, `vw_ops_flow_kpis_daily`, `vw_experience_kpis_daily`        | PM + Ops                      |
| **KR.O2.3** (Install/cutover)   | TTV reality                  | (Ops counters)                                                                                       | Program tracker                                                                   | VP Ops                        |
| **KR.DATA.1** (Prov/loss)       | Data trust                   | MET.DATA.PROVENANCE_SCORE · MET.DATA.LOSS_RATE                                                       | `vw_data_lineage_daily`                                                           | Head of Data                  |
| **KR.QA.1** (0 P1, MTTR <1h)    | Defect escape control        | MET.OPS.MTTR                                                                                         | `vw_ops_flow_kpis_daily`                                                          | QA Lead / SRE                 |
| **KR.SEC.1–2** (SBOM, red-team) | Security hygiene & incidents | MET.SEC.SBOM_SIGNED · MET.SEC.P1_FINDINGS_OPEN · MET.SEC.INCIDENTS_P1                                | `vw_security_kpis_daily`                                                          | Security Lead                 |

---

#### Appendix B — KR Evidence Pack (close-out schema)

> Standardizes what “proof” looks like. Auditors love this.

**Checklist**

* Data slice for each Canon ID (with site/sector filters + window)
* SoT provenance (view name, **view version**, **query hash**, run timestamp)
* Controls: passing tests (dbt), twin/scenario runs, drill AARs
* Traceability: PRs/commits for code/policy/config
* Owner sign-off (A/R)

**Manifest (JSON)**

```json
{
  "kr_id": "KR.O1.1",
  "period": "FY1-H1",
  "metrics": [
    {"id":"MET.RELIAB.AVAIL_ODD","window":"R60D","filters":{"site":"Mine-Alpha"},"value":0.995,"sot_view":"vw_ops_availability_daily","view_version":"v3","query_sha256":"..."},
    {"id":"MET.SAFETY.ASSIST_RATE","window":"R60D","filters":{"site":"Mine-Alpha"},"value":0.28,"sot_view":"vw_safety_events_daily","view_version":"v5","query_sha256":"..."},
    {"id":"MET.SAFETY.ZERO_HARM","window":"R60D","filters":{"site":"Mine-Alpha"},"value":0,"sot_view":"vw_safety_events_daily","view_version":"v5","query_sha256":"..."},
    {"id":"MET.GOV.AUDIT_BUNDLE_COMPLETE","window":"release:2025.06.15","value":1.0,"sot_view":"vw_governance_kpis_daily","view_version":"v2","artifact_hash":"sha256:..."}
  ],
  "controls":{"tests":["dbt:test/2025-06-16#1425"],"twin":["ci:twin/merge-4812#passed"],"drills":["drill:EW-jam-2025Q2#AAR-ok"]},
  "provenance":{"generated_at_utc":"2025-06-16T14:32:11Z","generator_version":"evidence-pack@1.2.0"},
  "signoff":{"accountable":["VP Safety"],"responsible":["Site Lead"],"date":"2025-06-16"}
}
```

---

#### Appendix C — KR Scoring Rules (0.0–1.0)

* **Threshold:** hit target = **0.7–0.8** (success), hit stretch = **1.0**, linear taper to **0.0** at fail band.
* **Directional:** normalize delta vs baseline; clamp [0,1]; baseline declared in evidence pack.
* **Binary:** 1.0 or 0.0 (e.g., bundle completeness).
* **Composite:** weighted mean of sub-metrics (weights stated in the KR).

**Objective score** = weighted mean of its KR scores. **No score** if manifest is incomplete or fails provenance checks.

---

#### Appendix D — Gates & Stop-Conditions (enforced)

* **Build/Merge:** block if **twin scenario pass <95%** (KR.ENG.3).
* **Release:** block if **MET.GOV.AUDIT_BUNDLE_COMPLETE < 1.0**; block if **MET.SEC.P1_FINDINGS_OPEN > 0** or critical CVEs >15d.
* **Operate:** pause non-remediation work if any **safety/compliance Canon metric is red**; auto-escalate if **MET.RELIAB.AVAIL_ODD < 99.0% (R7D)** or **MET.SAFETY.ASSIST_RATE > 0.5/1k km (R7D)**.

> Gates are **references** to Canon bands; no formulas here.

---

#### Appendix E — Alerting & Review Cadence

| Family | Red threshold source | Who’s paged                | Channel         | Cadence                     |
| ------ | -------------------- | -------------------------- | --------------- | --------------------------- |
| SAFETY | Canon bands          | On-call Safety + Site Lead | #safety-ops     | Daily + weekly Safety Board |
| RELIAB | Canon bands          | SRE + Ops Lead             | #sre-fleet      | Daily standup               |
| OPS    | Canon bands          | Product Ops                | #control-center | Weekly ops review           |
| GOV    | Canon bands          | Compliance                 | #compliance     | Per-release + monthly       |
| SEC    | Canon bands          | Security on-call           | #sec-incidents  | Weekly + incident AAR       |
| DATA   | Canon bands          | Data Eng on-call           | #data-pipelines | Daily + fortnightly         |

---

#### Appendix F — Reviewer Mini-Checklist

* [ ] Canon IDs exist (not deprecated)
* [ ] Evidence manifest valid (view versions + query hashes present)
* [ ] Windows/filters match KR scope
* [ ] Controls/tests attached and passing
* [ ] A/R sign-offs included
* [ ] Score per declared KR type
* [ ] No red Canon metrics for the KR window

### OKR → Epic → FR/NFR Traceability Flow

#### Formal Traceability Model

**Level 1: Strategic Objectives (O-1 to O-5)**
- **O-1 Safety**: Prove repeatable L4 operations in harsh & controlled ODDs
- **O-2 Time-to-Value**: Demonstrate ROI and scalability (commercial proof)  
- **O-3 Agnosticism**: Prove we're credibly agnostic (vehicle, sensor, map, cloud)
- **O-4 Regulatory**: Evidence & compliance automation
- **O-5 UX**: Operator experience excellence

**Level 2: Key Results (KR.O1.1 to KR.GTM.3)**
- Each KR maps to 1-3 Canon metrics
- Each KR has evidence pack requirements
- Each KR has success criteria and gates

**Level 3: Epics (E-1 to E-12)**
- Each Epic maps to 1-2 Strategic Objectives
- Each Epic has DoR checklist (evidence pack, variant budget, SLI targets)
- Each Epic has verification methodology and exit gates

**Level 4: Functional Requirements (FR-001 to FR-075)**
- Each FR maps to 1 Epic and 1-2 OKRs
- Each FR has acceptance criteria, telemetry, and traceability
- Each FR has department ownership and priority

**Level 5: Non-Functional Requirements (NFR-P-01 to NFR-Gov-01)**
- Each NFR maps to 1-2 Epics and OKRs
- Each NFR has SLI/SLA definitions and alert policies
- Each NFR has measurement methodology and gates

#### Verification Methodology

**Monthly Traceability Audit**:
- [ ] 100% KRs have Canon metric mappings
- [ ] 100% Epics have DoR evidence packs
- [ ] 100% FRs have OKR linkages
- [ ] 100% NFRs have SLI instrumentation
- [ ] 100% features have traceability chains

**Quarterly Outcome Quality (OQ) Review**:
- [ ] Compare SLIs vs targets for shipped features
- [ ] Validate evidence pack completeness
- [ ] Assess traceability chain integrity
- [ ] Feed learnings back into playbooks

---

