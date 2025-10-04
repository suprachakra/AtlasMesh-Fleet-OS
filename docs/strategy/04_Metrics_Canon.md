# 04_Metrics_Canon.md

This is the **single source of truth** for metric IDs, definitions, units, windows, SoT views, owners, alert bands, and default SLO guidance. OKR targets live in the OKR file.

## 0) Taxonomy & conventions

* **ID pattern:** `MET.<FAMILY>.<NAME>` (upper snake). Families: **SAFETY, RELIAB, OPS, PROD, COST, EXP, GOV, QUAL, SEC, DATA**.
* **Window default:** rolling **30-day** unless otherwise specified.
* **Units:** distance **km**; time **seconds** (minutes only when stated). Money in site currency; label currency in dashboards.
* **SoT:** dbt views under `analytics/warehouse/views/*` (versioned).
* **Dimensions:** all metrics must be sliceable by **sector, site, vehicle_class, ODD_segment, software_release**.
* **Immutability:** IDs are permanent; if logic changes incompatibly, create a new ID or versioned view.
* **Maturity:** each metric carries **M1–M3** maturity (M1 = provisional; M3 = audited & stable).

---

## 1) Source-of-Truth map

| Family | Primary SoT view (dbt)       | Notes                                        |
| ------ | ---------------------------- | -------------------------------------------- |
| SAFETY | `vw_safety_events_daily`     | Incident taxonomy; safe-stops; assists       |
| RELIAB | `vw_ops_availability_daily`  | Availability by ODD & cause                  |
| OPS    | `vw_ops_flow_kpis_daily`     | Dispatch latency, dwell, handoffs, clearance |
| PROD   | `vw_productivity_kpis_daily` | Tons/hr, moves/hr, sort tph                  |
| COST   | `vw_cost_kpis_daily`         | $/move, cost/ton, empty miles                |
| EXP    | `vw_experience_kpis_daily`   | ETA P95, CSAT                                |
| GOV    | `vw_governance_kpis_daily`   | Bundle completeness, findings                |
| QUAL   | `vw_quality_kpis_daily`      | Temp excursions, count accuracy              |
| SEC    | `vw_security_kpis_daily`     | SBOM signing, findings, incidents            |
| DATA   | `vw_data_lineage_daily`      | Provenance score, loss rate, freshness       |

---

## 2) Canonical metrics (grouped)

> **Default SLO bands** are guidance; period targets are set in OKRs.
> Columns: **ID | Name | Definition | Formula (sketch) | Window | SoT | Owner | Default SLO | Alerts | Maturity**

### 2.1 SAFETY

| ID                                | Name                      | Definition                             | Formula                   | Window | SoT    | Owner            | Default SLO | Alerts    | M  |
| --------------------------------- | ------------------------- | -------------------------------------- | ------------------------- | ------ | ------ | ---------------- | ----------- | --------- | -- |
| **MET.SAFETY.MISSION_COMPLETION** | Mission completion rate   | % missions completed within ODD & ROE  | `completed/started`       | 30d    | SAFETY | Safety Lead      | ≥98%        | Red <97%  | M3 |
| **MET.SAFETY.ZERO_HARM**          | Critical harm rate        | Critical harm incidents per 10k km     | `crit_incidents/(km/10k)` | 30d    | SAFETY | Safety           | =0          | Red >0    | M3 |
| **MET.SAFETY.ASSIST_RATE**        | Assists per 1k km         | Human Q&A assists per 1k km            | `assists/(km/1000)`       | 30d    | SAFETY | Safety           | ≤0.5        | Red >0.5  | M3 |
| **MET.SAFETY.RESPONSE_TTA_P95**   | Response time P95         | Seconds to first qualified response    | `P95(t_resp - t_alarm)`   | 30d    | SAFETY | Ops/Sec          | ≤120s       | Red >180s | M2 |
| **MET.SAFETY.GRADE_VIOLATIONS**   | Grade/traction violations | Count of grade > policy or slip events | `count(policy_violation)` | 30d    | SAFETY | Safety           | =0          | Red >0    | M2 |
| **MET.SAFETY.HAZMAT_VIOL**        | Hazmat policy violations  | Violations per 10k km (hazmat trips)   | `viol/(haz_km/10k)`       | 30d    | SAFETY | Logistics Safety | =0          | Red >0    | M2 |

### 2.2 RELIABILITY

| ID                             | Name                       | Definition                                   | Formula                     | Window | SoT    | Owner           | Default SLO | Alerts           | M  |
| ------------------------------ | -------------------------- | -------------------------------------------- | --------------------------- | ------ | ------ | --------------- | ----------- | ---------------- | -- |
| **MET.RELIAB.AVAIL_ODD**       | Availability in ODD        | % time vehicle mission-capable in ODD        | `capable_hours/odd_hours`   | 30d    | RELIAB | Ops             | ≥99.3%      | Red <99.0%       | M3 |
| **MET.RELIAB.ODD_CONFORMANCE** | ODD conformance rate       | % km within encoded ODD                      | `odd_conf_km/total_km`      | 30d    | RELIAB | Safety          | ≥99.5%      | Red <99.0%       | M2 |
| **MET.RELIAB.MTBF**            | Mean time between failures | Median hours between service-stopping faults | `median(diff(fault_times))` | 90d    | RELIAB | Reliability Eng | ↑trend      | Red 10%↓ vs base | M2 |
| **MET.RELIAB.POWER_UPTIME**    | Power uptime               | % time critical power is available           | `power_ok/total`            | 30d    | RELIAB | Ops             | ≥99.9%      | Red <99.7%       | M2 |

### 2.3 OPERATIONS

| ID                           | Name                     | Definition                 | Formula                     | Window | SoT | Owner       | Default SLO | Alerts           | M  |
| ---------------------------- | ------------------------ | -------------------------- | --------------------------- | ------ | --- | ----------- | ----------- | ---------------- | -- |
| **MET.OPS.DISPATCH_LAT_P95** | Dispatch latency P95     | Seconds request→assigned   | `P95(t_assign - t_request)` | 30d    | OPS | Product Ops | ≤60s        | Red >90s         | M2 |
| **MET.OPS.OTP**              | On-time performance      | % trips meeting ETA/SLA    | `on_time/total`             | 30d    | OPS | Ops         | ≥95%        | Red <92%         | M2 |
| **MET.OPS.GATE_TO_DOCK_P95** | Gate→dock time P95 (min) | Minutes gate-in→first dock | `P95(t_dock - t_gate)`      | 30d    | OPS | Logistics   | ≤30m        | Red >40m         | M2 |
| **MET.OPS.DWELL_TIME**       | Dwell time avg           | Avg minutes idling         | `avg(dwell_mins)`           | 30d    | OPS | Logistics   | ↓trend      | Red +10% vs base | M2 |
| **MET.OPS.MTTR**             | Mean time to recover     | Minutes to Sev-1 recovery  | `avg(t_recover - t_start)`  | 30d    | OPS | SRE         | ≤60m        | Red >90m         | M2 |
| **MET.OPS.CLEARANCE_TIME**   | Border clearance time    | Minutes per crossing       | `avg(t_clear - t_arrive)`   | 30d    | OPS | Logistics   | ≤45m        | Red >60m         | M1 |

### 2.4 PRODUCTION / THROUGHPUT

| ID                          | Name            | Definition                    | Formula                | Window | SoT  | Owner     | Default SLO | Alerts          | M  |
| --------------------------- | --------------- | ----------------------------- | ---------------------- | ------ | ---- | --------- | ----------- | --------------- | -- |
| **MET.PROD.TONS_PER_HOUR**  | Tons/hour       | Material moved per hour       | `sum(tons)/sum(hours)` | 30d    | PROD | Mining    | Site plan   | Red 5%↓ vs plan | M2 |
| **MET.PROD.MOVES_PER_HOUR** | Moves/hour      | Yard/container moves per hour | `moves/hours`          | 30d    | PROD | Ports     | Site plan   | Red 5%↓ vs plan | M2 |
| **MET.PROD.SORT_TPH**       | Sort throughput | Parcels sorted per hour       | `sorted/hours`         | 30d    | PROD | Logistics | Site plan   | Red 5%↓ vs plan | M1 |

### 2.5 COST

| ID                         | Name          | Definition                        | Formula                                 | Window | SoT  | Owner          | Default SLO | Alerts          | M  |
| -------------------------- | ------------- | --------------------------------- | --------------------------------------- | ------ | ---- | -------------- | ----------- | --------------- | -- |
| **MET.COST.$_PER_MOVE**    | Cost per move | Direct op cost per completed move | `(fuel+energy+tires+maint+labor)/moves` | 30d    | COST | FinOps         | ↓trend      | Red 5%↑ vs base | M2 |
| **MET.COST.COST_PER_TON**  | Cost per ton  | Direct + alloc per ton hauled     | `total_op_cost/tons`                    | 30d    | COST | Mining Finance | ↓trend      | Red 5%↑ vs base | M2 |
| **MET.COST.EMPTY_MILES**   | Empty miles % | % km without payload              | `empty_km/total_km`                     | 30d    | COST | Logistics      | ≤3%         | Red >5%         | M2 |
| **MET.COST.TRIPS_PER_TON** | Trips per ton | Trips / ton (lower better)        | `trips/tons`                            | 30d    | COST | Logistics      | ↓trend      | Red 5%↑ vs base | M1 |

### 2.6 EXPERIENCE

| ID                  | Name                  | Definition          | Formula                     | Window | SoT | Owner  | Default SLO | Alerts   | M  |
| ------------------- | --------------------- | ------------------- | --------------------------- | ------ | --- | ------ | ----------- | -------- | -- |
| **MET.EXP.ETA_P95** | Wait time P95 (min)   | 95th pct rider wait | `P95(t_pickup - t_request)` | 30d    | EXP | RH Ops | ≤7m         | Red >9m  | M2 |
| **MET.EXP.CSAT**    | Customer satisfaction | Avg rating (1–5)    | `avg(rating)`               | 30d    | EXP | RH Ops | ≥4.8        | Red <4.6 | M1 |

### 2.7 GOVERNANCE / COMPLIANCE

| ID                                | Name                       | Definition                    | Formula            | Window      | SoT | Owner      | Default SLO | Alerts     | M  |
| --------------------------------- | -------------------------- | ----------------------------- | ------------------ | ----------- | --- | ---------- | ----------- | ---------- | -- |
| **MET.GOV.AUDIT_BUNDLE_COMPLETE** | Bundle completeness        | % required artifacts present  | `present/required` | per release | GOV | Compliance | **100%**    | Red <100%  | M3 |
| **MET.GOV.AUDIT_FINDINGS_P1**     | P1 audit findings open     | Count of P1 audit issues open | `count(p1_open)`   | 30d         | GOV | Compliance | 0           | Red >0     | M3 |
| **MET.GOV.CHAIN_OF_CUSTODY**      | Chain-of-custody integrity | % transfers with signed CoC   | `signed/transfers` | 30d         | GOV | Compliance | ≥99.9%      | Red <99.5% | M2 |

### 2.8 QUALITY (sector-specific)

| ID                           | Name                       | Definition                 | Formula                   | Window | SoT  | Owner     | Default SLO | Alerts     | M  |
| ---------------------------- | -------------------------- | -------------------------- | ------------------------- | ------ | ---- | --------- | ----------- | ---------- | -- |
| **MET.QUAL.TEMP_EXCURSIONS** | Cold-chain temp excursions | # out-of-band per 1k hours | `excursions/(hours/1000)` | 30d    | QUAL | Logistics | =0          | Red >0     | M2 |
| **MET.QUAL.COUNT_ACCURACY**  | Cycle count accuracy       | % accurate counts          | `accurate/total`          | 30d    | QUAL | Logistics | ≥99.5%      | Red <99.0% | M1 |
| **MET.QUAL.GRADE_VAR**       | Ore grade variance         | Variance vs target spec    | `var(grade-target)`       | 30d    | QUAL | Mining    | ≤ spec      | Red > spec | M1 |

### 2.9 SECURITY

| ID                           | Name                    | Definition                          | Formula                | Window      | SoT | Owner    | Default SLO | Alerts    | M  |
| ---------------------------- | ----------------------- | ----------------------------------- | ---------------------- | ----------- | --- | -------- | ----------- | --------- | -- |
| **MET.SEC.P1_FINDINGS_OPEN** | Open P1 vulns           | # P1 security findings open         | `count(p1_open)`       | 30d         | SEC | Security | 0           | Red >0    | M3 |
| **MET.SEC.SBOM_SIGNED**      | SBOM completeness       | % releases with signed SBOM         | `signed_sbom/releases` | per release | SEC | Security | 100%        | Red <100% | M3 |
| **MET.SEC.INCIDENTS_P1**     | Security incidents (P1) | Count of P1 prod security incidents | `count(incidents_p1)`  | 30d         | SEC | Security | 0           | Red >0    | M2 |

### 2.10 DATA (new family)

| ID                            | Name                  | Definition                                                | Formula                                       | Window | SoT  | Owner    | Default SLO | Alerts    | M  |
| ----------------------------- | --------------------- | --------------------------------------------------------- | --------------------------------------------- | ------ | ---- | -------- | ----------- | --------- | -- |
| **MET.DATA.PROVENANCE_SCORE** | Data provenance score | Weighted score of signed lineage, completeness, freshness | `w1*lineage + w2*completeness + w3*freshness` | 30d    | DATA | Data Eng | ≥0.95       | Red <0.9  | M2 |
| **MET.DATA.LOSS_RATE**        | Telemetry loss rate   | % expected records missing/late                           | `missing_expected/expected_total`             | 30d    | DATA | Data Eng | ≤0.1%       | Red >0.2% | M2 |

---

## 3) Validation & alerting

* **Unit tests:** dbt tests for non-nulls, ranges, referential integrity; CI blocks if fail.
* **Recalc parity:** parity checks vs historical for breaking deltas; create `vN` view on incompat changes.
* **Alerting:** Red thresholds raise PagerDuty to metric owner + Slack channel `#kpi-alerts`.
* **Privacy:** Canon excludes direct PII; customer dashboards use pseudonymized IDs.

---

## 4) Crosswalk to OKRs

(Kept minimal; the OKR file holds the KR list. This table lets reviewers verify coverage.)

| KR (see OKRs) | Canon IDs                                                                                            |
| ------------- | ---------------------------------------------------------------------------------------------------- |
| KR.O1.1       | MET.RELIAB.AVAIL_ODD · MET.SAFETY.ASSIST_RATE · MET.SAFETY.ZERO_HARM · MET.GOV.AUDIT_BUNDLE_COMPLETE |
| KR.O1.2       | MET.RELIAB.AVAIL_ODD                                                                                 |
| KR.O1.3       | MET.SAFETY.ASSIST_RATE · MET.SAFETY.ZERO_HARM                                                        |
| KR.O1.4       | MET.GOV.AUDIT_BUNDLE_COMPLETE · MET.GOV.AUDIT_FINDINGS_P1                                            |
| KR.O2.1       | MET.COST.COST_PER_TON · MET.OPS.GATE_TO_DOCK_P95 · MET.EXP.ETA_P95                                   |
| KR.DATA.1     | MET.DATA.PROVENANCE_SCORE · MET.DATA.LOSS_RATE                                                       |
| KR.QA.1       | MET.OPS.MTTR                                                                                         |
| KR.SEC.*      | MET.SEC.SBOM_SIGNED · MET.SEC.P1_FINDINGS_OPEN · MET.SEC.INCIDENTS_P1                                |

---

## 5) Versioning & deprecations

* **Change policy:** never change a metric’s meaning under the same ID. Create a new ID or versioned view.
* **Changelog:** append deprecations/renames at the end of this file with migration notes.

---