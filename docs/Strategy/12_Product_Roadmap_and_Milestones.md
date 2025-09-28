# 08_Product_Roadmap_ & Milestones

## Executive Summary

**18-Month Strategic Roadmap (2-week sprints):** MVP → Pilots → Scale. AtlasMesh Fleet OS is delivered with **zero-compromise safety** and a **No-Loophole Assurance Layer (NLAL)** embedded into every sprint: twin-gated CI/CD, policy shadow-evals, contract tests, chaos & red-team drills, audit-evidence gates, and automatic rollbacks. Each milestone is wired to OKRs, FR/NFR IDs, and sector packs.

**Phases**

* **Months 1–6: MVP & Foundation** — Core platform, safety framework, vehicle abstraction, policy engine, twin-gated CI/CD, control center & assist v1.
* **Months 7–12: Pilots & Validation** — Lighthouse customers, enterprise integrations, PdM, weather-aware routing, multi-site ops.
* **Months 13–18: Scale & Expansion** — 100+ vehicles production, advanced AI/ML, internationalization, market leadership.


# Capacity & Hiring Plan (Months 1–18)

**Assumptions:** 2-week sprints; blended velocity grows with ramp & learning; 20% engineering slack baked in for unplanned work and incidents.

## 0. Velocity Model & Critical Path

* **Velocity formula:** `Team SP/Sprint = #Devs × 7 SP × focus_factor`, where focus_factor starts 0.65 (M1–2), 0.8 (M3–6), 0.9 (M7–12), 0.95 (M13–18).
* **System critical path (hard gates):** HAL & ODD (E-01/E-02) → Twin Gates (E-03/E-12/E-36) → Evidence/Compliance (E-04) → Dispatch/Routing (E-05) → Control Center/Assist (E-09/E-13) → Deployment Automation (E-14) → Enterprise Integration (E-15).
* **Slack policy:** ≥15% sprint capacity reserved for defects/drift; cannot be traded without DAB approval.

## 1. Capacity by Month (FTE & Velocity)

| Month | Total FTE | Eng (Core/Edge/BE/FE) | ML/Perception | SRE/Sec | PM/Design | QA | Est. Velocity (SP/sprint) |
| ----- | --------: | --------------------: | ------------: | ------: | --------: | -: | ------------------------: |
| 1     |        24 |          14 (4/4/4/2) |             3 |     2/1 |       2/1 |  3 |                       140 |
| 2     |        28 |          17 (5/5/5/2) |             3 |     2/2 |       3/1 |  4 |                       170 |
| 3     |        36 |          22 (6/7/7/2) |             4 |     3/2 |       3/2 |  6 |                       220 |
| 4     |        42 |          26 (7/8/9/2) |             5 |     3/3 |       4/2 |  7 |                       260 |
| 5     |        48 |        30 (8/10/10/2) |             6 |     4/3 |       4/3 |  8 |                       300 |
| 6     |        54 |        34 (9/11/12/2) |             6 |     4/4 |       5/3 |  9 |                       330 |
| 7–9   |        62 |       38 (10/12/14/2) |             8 |     5/4 |       6/3 | 11 |                       380 |
| 10–12 |        70 |       43 (11/14/16/2) |             9 |     6/4 |       7/4 | 12 |                       420 |
| 13–15 |        82 |       50 (12/16/20/2) |            11 |     7/5 |       8/5 | 14 |                       480 |
| 16–18 |        96 |       60 (14/20/24/2) |            12 |     8/6 |       9/6 | 16 |                       540 |

**Hiring milestones:** key leaders by M1 (Platform, Safety, DevOps), M2 (Routing, Policy), M3 (Perception), M4 (UX), M5 (SRE), M6 (Data), M7 (Security), M9 (Deploy/Field).

**Load shaping:** If velocity < plan by >10% for two sprints, trigger **G-X.Throughput** (see Gates) → scope trade, add FTE, or slide dates via DAB.

---

# FinOps Guardrails & Budgets

**Principles:** Transparent COGS, predictable OpEx/CapEx, ARR-linked spend. FinOps gates block expansions that violate guardrails.

## 1. Budget Guardrails (Targets)

* **Cloud COGS ≤ 22% of ARR** (run-rate), trending ≤ 18% by Month 18.
* **Gross Margin ≥ 60%** by Month 12; ≥ 65% by Month 18.
* **Telemetry & storage unit costs:**

  * Ingest ≤ **$0.35/GB** (all-in), Hot storage ≤ **$0.02/GB-month**, Warm ≤ **$0.005/GB-month**, Cold ≤ **$0.0015/GB-month**.
  * Per-vehicle compute (edge GPU) amortized ≤ **$210/mo** at pilot, ≤ **$150/mo** at scale.
* **SRE headcount ratio:** 1:12 services (M1–6) → 1:18 (M7–12) → 1:24 (M13–18).
* **Contracts:** No provider concentration >40% of COGS category (cloud, maps, comms, sensors).

## 2. Phase Budgets (CapEx/OpEx Bands)

| Phase                | CapEx (pilot kits, testbed) | OpEx/mo by Phase | Notes                                                           |
| -------------------- | --------------------------: | ---------------: | --------------------------------------------------------------- |
| Months 1–6 (MVP)     |                   $1.8–2.4M |        $450–600k | Twin infra, 2 test vehicles/class × 4 classes, staging clusters |
| Months 7–12 (Pilots) |                   $2.0–2.8M |        $700–900k | 3 lighthouse sites, data retention growth, SRE ramp             |
| Months 13–18 (Scale) |                   $2.5–3.5M |        $1.1–1.5M | 10+ customers live, analytics & DR full footprint               |

**Pricing/Payback levers:**

* Target **CAC payback ≤ 12 months** by Month 12; **≤ 9 months** by Month 18.
* **Unit economics:** Platform fee + per-vehicle + sector add-ons. Ensure **ARR/vehicle ≥ 4 × total monthly COGS/vehicle**.

**FinOps Gates (new):**

* **G-F1:** COGS >22% ARR for 2 consecutive months → freeze nonessential scale-outs.
* **G-F2:** Cloud/storage cost/GB exceeds guardrail → enforce data retention downtiers; invoke compression/purging plan (ties to FR-050).
* **G-F3:** CAC payback >12 mo rolling → halt GTM spend growth until conversion fixes shipped.

---
### Assurance Layer (AL) — Always-On Controls

1. **Twin-Gated CI/CD (E-03, FR-036/062):** PRs fail if sim KRs regress (assist rate, safe-stop, policy).
2. **Evidence Gate (E-04, FR-029, NFR-Comp-01/02):** Releases blocked unless evidence bundles are 100% complete & signed.
3. **Policy Shadow-Eval + Diff (E-02, FR-009/011, NFR-P-02):** Every routing/dispatch change runs in shadow; drift >1% blocks.
4. **Contract Tests (FR-038/051/063, NFR-I-01/I-04):** OEM/ERP/map adapters must be 100% green across version matrix.
5. **Chaos/Resilience (NFR-R-03/R-04, E-11):** Fault-injection & DR drills pass before raising traffic.
6. **Security & OTA Integrity (FR-028, NFR-Sec-01/02/03/05):** mTLS everywhere, CVE SLAs, signed/attested updates only.
7. **Privacy & Residency (FR-033/064, NFR-Priv-01/03):** Purpose binding & region locks enforced by policy engine.
8. **Observability Coverage (NFR-Ob-01/03):** 100% services instrumented; missing exporters block merge.
9. **UX Safety & A11y (FR-032, NFR-U-01):** WCAG 2.2 AA and SUS gates; SUS <80 triggers redesign sprint.
10. **FinOps Guardrails:** Cost/carbon SLIs per deploy (FR-040); overruns >10% trigger optimization gate.
11. **GTM/Comms Readiness:** SLA reporting (FR-047) and release notes (FR-045) auto-generated; incomplete → block.
12. **On-Call Readiness:** Runbooks & paging tested; alert noise >2% (NFR-Ob-02) triggers tuning before go-live.

> **Epic Tags:**
> E-01 Vehicle Abstraction · E-02 Policy Engine · E-03 Twin-Gated CI/CD · E-04 Evidence Automation · E-05 Dispatch & Routing · E-06 Energy Mgmt · E-07 Predictive Maintenance · E-08 Map Fusion · E-09 Tele-Assist · E-10 Weather Fusion · E-11 Offline-First · E-12 Multi-Sensor Fusion · E-13 Fleet Control Center · E-14 Deployment Automation · E-15 Enterprise Integration

---

## 18-Month Deliverables & Milestones

| Timeframe (Sprints / Month) | Phase & Objectives                                                       | Key Activities / Tasks                                                                                                                                                       | Integrated Epics                   | Teams Involved                                                  | Dependencies                                            | Sprint Goals & Success Metrics                                                                                                                          | Deliverables                                                                       | SAFe Milestones                                                                                             | Risks & Mitigations                                                            | Outcomes                                            |
| --------------------------- | ------------------------------------------------------------------------ | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ---------------------------------- | --------------------------------------------------------------- | ------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------ | --------------------------------------------------- |
| **S1–S2 (Month 1)**         | **MVP Foundation** — Core platform, safety scaffolding, dev infra        | S1: ROS2 edge bring-up; HAL skeleton; CI/CD baseline; safety case template. S2: Profile repo+validator; policy engine skeleton; fusion pipeline stub; evidence event schema. | E-01, E-02, E-03, E-12, E-04       | Platform, Safety, DevOps, ROS2, Systems, QA, Sec, Docs          | HW kits; ROS2 Humble; test vehicles; vendor SDKs; cloud | **Tech:** 15 ROS2 nodes; 2 profiles; test cov ≥80%. **Perf:** loop ≤100ms (NFR-P-01); CI ≤30m. **OKR:** O-2/O-1.                                        | Edge v0.1; HAL; CI/CD; safety template; monitoring seed.                           | **M1:** Core platform up; quality gates active. **Hard gate:** NFR-Ob-01 exporter check.                    | **High:** ROS2 complexity → expert assist. **Med:** HW delays → backup vendor. | Infra proven; HAL validated; velocity baseline set. |
| **S3–S4 (Month 2)**         | **Safety & Evidence** — Safety case v1, evidence pipeline, profile certs | S3: Collector+signing; scenario bank 50; profile conformance. S4: Auto safety tests; compliance job; fusion v0.1; perf baseline.                                             | E-04, E-03, E-12, E-01             | Platform, Safety, Test, Compliance                              | Vehicle specs; ISO inputs; sim licenses; track access   | **Safety:** safety case v1 auto-built; coverage ≥90%; scenario pass ≥95%; fault-modes ≥20. **Tech:** 2 classes certified; loop ≤80ms. **OKR:** O-1/O-4. | Safety case v1; evidence pipeline; HARA prelim; dashboard v0.1; UGV+Haul profiles. | **M2:** Safety foundation; evidence jobs green. **Hard gate:** FR-029/NFR-Comp-01=100%.                     | **High:** Long gen time → pipeline split & cache.                              | Evidence 100%; 2 classes certified; cov ≥85%.       |
| **S5–S6 (Month 3)**         | **Multi-Vehicle & Sector** — More classes; sector policies; routing base | S5: Yard Tractor + Passenger profiles; sector policy packs; dispatch svc; map fusion arch. S6: Multi-vehicle testing; routing v1; sector UI overlays; pilot prep.            | E-01, E-02, E-05, E-13, E-08       | Platform, Vehicle, Routing, UI, Policy; Sector Pods MIN/DEF/LGX | Added specs; sector rules; map providers; pilot LOIs    | **Vehicle:** 4 classes; hot-swap <30s (FR-005). **Routing:** v1 Pareto (FR-012). **OKR:** O-2/O-3.                                                      | 4 profiles; convoy basics; fleet dashboard seed; policy packs (MIN/DEF/LGX).       | **M3:** Multi-vehicle ready; pilots initiated. **Gate:** FR-001/002 ≥95% conformance (G-3).                 | **High:** Pilot slippage → alternate customers pre-contracted.                 | 10 vehicles tested; routing v1 validated.           |
| **S7–S8 (Month 4)**         | **Advanced Perception** — Fusion & weather resilience                    | S7: 3+ modality fusion; confidence scoring; env monitors. S8: Redundancy; weather-aware routing; degraded modes.                                                             | E-12, E-10, E-05, E-11, E-04       | Perception, Env/Weather, Safety, Perf                           | Sensor HW; weather APIs; env labs                       | **Perception:** confidence ≥90%; fail detect ≤100ms. **Env:** 3 sources fused; 50°C+ validated; 100+ field hrs. **OKR:** O-1/O-3.                       | Fusion v1; health monitors; weather fusion v1; harsh-env tuning.                   | **M4:** Perception operational; env resilience proven. **Gate:** NFR-P-01 at P95 ≤50ms in heat chamber sim. | **High:** Sensor drift → auto-cal & quarantine.                                | Fusion & weather claims substantiated.              |
| **S9–S10 (Month 5)**        | **Ops & Assist** — Control center; tele-assist; offline-first            | S9: Control center v1; assist workflow; store-and-forward. S10: UX polish; assist triage; sync engine; energy seed.                                                          | E-13, E-09, E-11, E-06             | Ops, UI, Backend, Assist, Energy                                | UX specs; operator SMEs; comms profiles                 | **Ops:** SUS ≥80 (NFR-U-01); RTT ≤2s (NFR-P-04); ≥10 assists/hr. **Offline:** ≥45m; resync ≤5m; 90% features. **OKR:** O-5/O-3.                         | Control center v1; assist v1; offline capability.                                  | **M5:** Ops platform live; assist SLA. **Gate:** FR-041/080/081 latency SLIs green.                         | **High:** SUS<70 → redesign sprint committed.                                  | Operator adoption; assist SLA achieved.             |
| **S11–S12 (Month 6)**       | **MVP & Pilots Launch** — 3 lighthouse                                   | S11: Site prep; deploy; train; perf monitors. S12: Optimize; validate KPIs; MVP cert.                                                                                        | E-14, E-13, E-15 (+ all for field) | Deploy, CS, Field, All Eng                                      | Site readiness; permits; gear                           | **Deploy:** ≤90 days TTV (O-2). **Perf:** avail ≥99.0%; assist ≤0.5/1k km (O-1); CSAT ≥4.5.                                                             | Mining/Defense/Logistics pilots; baseline reports.                                 | **M6:** MVP complete. **Gate:** FR-036/062 ≥98% pass (G-4).                                                 | **High:** Site delays → surge field team; prefab configs.                      | 3 pilots operational; references secured.           |
| **S13–S14 (Month 7)**       | **Pilot Optimization** — PdM + enterprise                                | S13: PdM rollout; ERP/WMS connectors; perf tuning. S14: PdM validation; E2E tests; analytics; expansion prep.                                                                | E-07, E-15, E-05, E-06             | ML, Integrations, Perf                                          | Hist maint data; ERP/WMS access                         | **PdM:** ≥85% accuracy; −20% downtime. **Integrations:** ≤4w/system; data ≥95% acc. **OKR:** O-3/O-2.                                                   | PdM v1; health; sched; adapters; sync jobs.                                        | **M7:** PdM live; enterprise integrated. **Gate:** FR-038/051 contracts 100%.                               | **High:** PdM <80% → retrain & rules hybrid.                                   | 20% downtime reduction; 4-week integration proven.  |
| **S15–S16 (Month 8)**       | **Advanced Features** — Weather + routing + multi-site                   | S15: Fuse weather→routing; advanced algos; multi-site. S16: Cross-site mgmt; analytics; 6 customers live.                                                                    | E-10, E-05, E-08, E-13             | Weather, Routing, Analytics, Coordination, CS                   | Weather providers; cross-site network                   | **Weather:** +15% routing efficiency; ≥90% prediction. **Scale:** 6+ sites; ≤100ms cross-site; avail ≥99.5%. **OKR:** O-3/O-1.                          | Weather-aware routing; multi-site mgmt; analytics v1.                              | **M8:** Weather operational; multi-site proven. **Gate:** NFR-P-06 bus P99 <100ms.                          | **Med:** Multi-site flakiness → shard & back-pressure.                         | 6 customers; 99.5%+ site avail.                     |
| **S17–S18 (Month 9)**       | **Performance Optimization** — Energy & CX                               | S17: Perf sweeps; Energy v2; CX uplift; reliability. S18: Perf & energy validation; feedback loop.                                                                           | E-06 (+ cross-epic perf)           | Perf, Energy, UX/CX                                             | Bench infra; feedback loops                             | **Perf:** loop ≤50ms; +25% throughput; +20% resource eff. **Energy:** −15% energy; +10% charge eff. **OKR:** O-3/O-5.                                   | Optimized control; Energy v2; CX polish; support tooling.                          | **M9:** Perf targets exceeded. **Gate:** NFR-P-01/Sc-02 green.                                              | **Med:** Targets miss → tiger team & hotfix.                                   | 50ms achieved; −15% energy; CSAT ≥4.8.              |
| **S19–S20 (Month 10)**      | **Reliability & Resilience** — Fault tolerance & DR                      | S19: Reliability patterns; chaos; offline 60m; DR drills. S20: Resilience validation; recovery procedures; BCP.                                                              | E-11 (+ SRE)                       | Reliability, SRE, Infra, Chaos                                  | Fault-inj lab; DR runbooks                              | **Reliability:** MTTR ≤1h; avail ≥99.8%; recovery ≤5m. **Offline:** ≥60m; 95% funcs; data 100%. **OKR:** O-1/O-3.                                       | Fault tolerance; extended offline; DR/BCP; chaos framework.                        | **M10:** Reliability targets exceeded. **Gate:** NFR-R-04 failover <60s.                                    | **Med:** Offline <55m → store/forward tuning.                                  | MTTR ≤1h; 99.8%+ avail; 60m offline.                |
| **S21–S22 (Month 11)**      | **Scale Preparation** — 50+ vehicles; obs & analytics                    | S21: Scale infra; 100% obs; perf@scale tests. S22: Validation; dashboards; customer prep.                                                                                    | E-03, E-13, E-14                   | Scale, Infra, Observability, Analytics, CS                      | Capacity reservations; agents                           | **Scale:** 50+ vehicles; no perf loss. **Obs:** 100% services traced/logged (NFR-Ob-01/03). **OKR:** O-3.                                               | Scalable cloud; dashboards; benchmarks.                                            | **M11:** Scale infra ready. **Gate:** alert noise ≤2% (NFR-Ob-02).                                          | **High:** Perf drop → autoscale & shard.                                       | 50+ vehicles stable; predictive insights live.      |
| **S23–S24 (Month 12)**      | **Pilot Completion & Scale Launch** — 10 customers                       | S23: Pilot sign-offs; production launch; GTM push. S24: Scale optimization; Phase-2 plan.                                                                                    | All core + GTM                     | Org-wide, Market                                                | Customer readiness; revenue ops                         | **Scale:** 10 customers; 100+ vehicles; $10M ARR run-rate. **Market:** 5% share; CSAT ≥4.8. **OKR:** O-3/O-5.                                           | 100+ deployments; ref program; revenue mileposts.                                  | **M12:** Production scale achieved. **Gate:** FR-047 SLA report packs signed.                               | **High:** Launch slippage → deployment SWAT playbook.                          | Market foothold; revenue targets met.               |
| **S25–S26 (Month 13)**      | **Scale Optimization** — Ops efficiency; intl prep                       | S25: Ops automation; CX uplift; intl prep. S26: Efficiency validation; first intl launch.                                                                                    | Cross-functional                   | Ops Opt, CX, Intl, R&D                                          | Ops metrics; intl legal/reg                             | **Ops:** +25% efficiency; −20% cost/unit; 90% automation. **Customer:** CSAT ≥4.9; retention ≥98%. **OKR:** O-3/O-5.                                    | Automation playbooks; intl GTM plan; partner MoUs.                                 | **M13:** Ops optimized; intl go-live. **Gate:** residency & export (FR-064; Export policy).                 | **High:** Reg delays → legal task force escalation.                            | Efficiency +25%; intl entry green.                  |
| **S27–S28 (Month 14)**      | **Advanced AI/ML** — Next-gen models & differentiation                   | S27: Advanced AI features; model upgrades; differentiators. S28: ML validation; inference optimization; patents.                                                             | E-07 (+ ML platform)               | AI/ML, Data Sci, Innovation                                     | GPU budget; data pipelines                              | **ML:** ≥95% accuracy; ≤20ms inference; ≥80% feature adoption. **OKR:** O-3/O-5.                                                                        | Next-gen models; real-time optimizers; IP filings.                                 | **M14:** Advanced AI deployed. **Gate:** bias/drift monitors green; model cards archived.                   | **Med:** Model <90% → retrain + rules fallback.                                | Durable differentiation; IP portfolio up.           |
| **S29–S30 (Month 15)**      | **Market Expansion** — Accounts & revenue growth                         | S29: Expansion campaigns; revenue optimization; LTV uplift. S30: Target validation; leadership reinforcement.                                                                | GTM + Product                      | Sales, CS, Marketing, BizDev                                    | Pipeline; budgets; macro                                | **Market:** 25 customers; +150% revenue; 8% share. **Customer:** LTV +40%; retention ≥99%. **OKR:** O-3.                                                | 25 customers live; revenue kits; exec briefings.                                   | **M15:** Expansion > plan. **Gate:** churn <1%; NRR >120%.                                                  | **Med:** Growth <120% → pricing/packaging reset.                               | 25 customers; 150% growth; 8% share.                |
| **S31–S32 (Month 16)**      | **Platform Excellence** — Top-tier perf & reliability                    | S31: Platform hardening; top-1% perf; ops excellence. S32: Benchmarks; awards; customer councils.                                                                            | Cross-platform                     | Excellence Squad, Exec                                          | Benchmarks; industry bodies                             | **Platform:** Top 1% perf; 99.9% reliability; SUS ≥4.95/5. **OKR:** O-3/O-5.                                                                            | Optimized stacks; excellence reports; awards packages.                             | **M16:** Excellence recognized. **Gate:** external benchmark audit pass.                                    | **Low:** Recognition lag → PR campaign.                                        | Benchmarks passed; recognition landed.              |
| **S33–S34 (Month 17)**      | **Innovation Leadership** — Breakthroughs & partnerships                 | S33: Breakthroughs; partner strategy; patent scale-up. S34: Validate; 3-yr tech roadmap; partnership exec.                                                                   | R&D + Corp Dev                     | CTO, Innovation, Legal, Partnerships                            | Partner MoUs; research ties                             | **Innovation:** ≥3 breakthroughs; ≥10 patents; ≥5 partnerships. **OKR:** O-3.                                                                           | Breakthrough tech; patent portfolio; 3-yr roadmap.                                 | **M17:** Innovation leadership secured. **Gate:** partner security & data-sharing controls audited.         | **Med:** Dev slips >4w → re-staff path.                                        | Future moat strengthened; roadmap locked.           |
| **S35–S36 (Month 18)**      | **Market Leadership Consolidation** — Financials & scale                 | S35: Consolidate leadership; ARR; 50+ customers. S36: Validate; celebrate; next phase plan.                                                                                  | Org-wide                           | Entire Org, Exec                                                | Market conditions; finance                              | **Market:** #1 in harsh environments; 12% share. **Financial:** $35M ARR; positive margin. **OKR:** O-3/O-5.                                            | Leadership assets; investor pack; Phase-2 OKRs.                                    | **M18:** Leadership consolidated. **Gate:** third-party audit of SLAs & safety outcomes.                    | **Low:** Competitive pressure → accelerate innovation & GTM.                   | Leadership secured; sustainable growth model.       |

---

## Success Validation Framework (18-Month Targets)

| Category              | Target                                         | Validation                                                   | Confidence |
| --------------------- | ---------------------------------------------- | ------------------------------------------------------------ | ---------- |
| Customer Success      | **50+ customers**, **CSAT ≥4.95/5**            | Post-deployment surveys, NPS, reference program              | 95%        |
| Technical Performance | **99.9% availability**, **≤50ms control loop** | Synthetic checks, field telemetry, chaos drills              | 98%        |
| Safety & Compliance   | **0 critical incidents**, **100% audit pass**  | Safety board reviews, automated evidence, third-party audits | 99%        |
| Financial Performance | **$35M ARR**, **≥60% gross margin**            | Finance systems, signed MSAs/SOWs                            | 92%        |
| Market Position       | **12% share**, **#1 in harsh environments**    | Analyst reports, awards, customer councils                   | 88%        |

### Embedded Gates & Dependencies (applies across all sprints)

* **G-1 Policy Perf:** NFR-P-02 P99 ≤10ms before scaling E-02.
* **G-3 Multi-OEM:** FR-001/002 ≥95% conformance before first multi-OEM site.
* **G-4 Twin:** FR-036/062 ≥98% pass before scale pilots.
* **G-5 Evidence:** FR-029/NFR-Comp-01 at 100% for 2 consecutive releases before regulated go-lives.
* **Auto-Resolution:** FR-053 ⇢ FR-009 + FR-058; FR-054 ⇢ FR-038; FR-060 ⇢ FR-027 + FR-012; FR-062 ⇢ FR-036; FR-052 ⇢ FR-030; FR-012 ⇢ FR-015/016/017.

### Tie-backs to Epics & OKRs

* **O-1 Safety:** G-S1, G-M1/M2 hard-block releases; ML fallback & degraded modes wired to **E-3/E-4/E-9**.
* **O-2 Time-to-Value:** Capacity model + G-X keep dates realistic; vendor swap procedure protects schedule (**E-1/E-2/E-14/E-15**).
* **O-3 Cost & Scale:** FinOps guardrails & COGS caps; telemetry/storage unit economics → **E-8/E-11/E-13**.
* **O-4 Regulatory:** Intl pack gates **G-C1–C4**; evidence automation **E-4**.
* **O-5 UX:** SUS ≥80 as DAB criterion; i18n/a11y gates **FR-031/032**.


---

# Vendor/OEM Dependency Plan

## 1. Dual-Sourcing & SLAs

* **Maps:** HERE/Mapbox/OSM-Lanelet; SLA ≥99.9%, notice ≥180 days for breaking changes; delta QA harness (FR-015/016).
* **Comms:** Primary telco + satellite fallback; **failover ≤ 2s** (NFR-Port-03).
* **Sensors:** Radar/LiDAR/Camera have at least two qualified vendors each; **lead times** documented (8–12 weeks typical); lab validation matrix per vendor.
* **Edge compute:** Orin AGX + x86 fallback; A/B image parity, secure boot attestation.

## 2. Change Control & Escrow

* **MSA:** 6-month change notice, versioned APIs, monthly reliability report, penalty credits.
* **IP/Keys:** Escrow for SDKs/firmware; rolling key rotation; SBOM feed required (FR-028/NFR-Sec-03).

## 3. Swap Procedure (72-hour objective)

1. Trigger: SLA breach >2× in 30 days or critical API change without notice.
2. Activate adapter shim; run contract tests CI (FR-038).
3. Canary 5% fleet; validate SLOs; full cutover within 72h if green.
4. Backfill evidence & audit (FR-029/011).

---

# International Compliance Pack & Rollout Sequencing

## 1. Regional Checklists (blocking)

* **US:** FMVSS/NHTSA guidance, MSHA (mining), SOC2/ISO 27001, CCPA. Artifacts: safety case, cybersecurity plan, data maps.
* **EU:** 26262/21448, **R155/R156**, GDPR DPIA, **Data residency** per member state.
* **GCC (e.g., KSA/UAE):** Data localization, telecom licensing, security approval for defense sites; Arabic UI (FR-031/032).
* **India:** CERT-In incident timelines, data localization where applicable, OEM integration rules.
* **Australia:** Privacy Act, workplace H&S, mining regs.

## 2. Rollout Gates 

* **G-C1:** Residency policy tests green (FR-064/NFR-Priv-03).
* **G-C2:** R155/R156 auditor attestation uploaded for OTA & CSMS.
* **G-C3:** Language/i18n + accessibility audits passed (FR-031/032).
* **G-C4:** Export control check (ITAR/EAR) for tenant/site signed.

---

# GTM Scorecard by Phase (targets are blocking)

| Phase                | Pipeline ($) | Win-rate | CAC Payback |   NRR | References | Notes                                 |
| -------------------- | -----------: | -------: | ----------: | ----: | ---------: | ------------------------------------- |
| Months 1–6 (MVP)     |       $8–10M |   20–25% |      ≤15 mo | 105%+ |          2 | 3 lighthouse logos; pricing learnings |
| Months 7–12 (Pilots) |      $20–25M |   28–35% |      ≤12 mo | 115%+ |          6 | Expansion pilots; case studies        |
| Months 13–18 (Scale) |      $45–55M |   35–45% |       ≤9 mo | 125%+ |         12 | Multi-site rollouts; co-marketing     |

**GTM Gates (new):**

* **G-G1:** Win-rate < 25% for 2 months → pause outbound scale; product fixes prioritized.
* **G-G2:** NRR <110% rolling → trigger adoption/expansion taskforce.
* **G-G3:** Reference count misses by phase → block PR/paid campaigns.

---

# Security Calendar & Resilience Drills

| Cadence     | Activity                        | Owner      | SLO/Gate                                    |
| ----------- | ------------------------------- | ---------- | ------------------------------------------- |
| Monthly     | CVE review & patch burn-down    | Sec Lead   | **NFR-Sec-02**; zero overdue criticals      |
| Monthly     | SBOM diff & OTA attestation     | Sec + OTA  | **NFR-Sec-03**; block on attestation fail   |
| Monthly     | Chaos injection (faults, links) | SRE        | MTTR ≤ 60m (NFR-R-03), RTO < 60s (NFR-R-04) |
| Quarterly   | Red team + social engineering   | Sec        | P1 findings = 0 to ship; action plan ≤ 10d  |
| Quarterly   | DR failover game-day            | SRE        | Measured RTO < 60s, data loss = 0           |
| Per Release | Compliance artifact pack        | Compliance | **NFR-Comp-01/02/03** all green             |
| Bi-Weekly   | Access review & least privilege | Sec/IT     | 100% anomalies resolved in 5d               |

**New Gate:** **G-S1 Security Greenlight** (all above events within SLA) required to ship to production tenants.

---

# ML Risk Management Plan

## 1. Model Governance

* **Model Cards** for each ML asset; **risk class** (safety-critical vs advisory).
* **Retrain cadence:** monthly (advisory), quarterly (safety-critical) or on **drift**.
* **Data contracts:** schema + quality checks; PSI < **0.2** or KS p-value ≥ **0.05**; else **G-M1 Drift Gate** triggers.

## 2. Safety & Fallbacks

* **HIL window:** If model confidence < **0.85** or uncertainty > **2σ**, route to Tele-Assist (FR-020/041) within 10s.
* **Degraded mode:** Cap speed by 30%, expand buffers; require policy approval for continued ops (FR-043).
* **Explainability:** Log feature attributions for safety-critical decisions; stored in audit (FR-011/029).

## 3. Monitoring SLIs

* **Precision/Recall ≥ 0.9/0.9** on safety suites; **latency ≤ 20ms**/frame on edge.
* **Drift alerts** TTD < 24h; **false assist rate** contribution < 0.05/1000 km (ties NFR-S-01).

**Gates:**

* **G-M1:** Drift beyond thresholds → block promotion; retrain required.
* **G-M2:** Safety suite regression >2% → release blocked; hotfix path only.
* **G-X.Throughput:** Velocity shortfall >10% for two sprints → replan capacity/scope.
* **G-F1/F2/F3:** FinOps gates (COGS/ARR, $/GB, CAC payback).
* **G-C1–C4:** International compliance gates.
* **G-G1–G3:** GTM performance gates.
* **G-S1:** Security greenlight gate.
* **G-M1/M2:** ML drift/safety regression gates.

---

# Decision Governance & Go/No-Go (DAB)

## 1. Deployment Approval Board (DAB)

* **Members:** SVP Eng (Chair), Safety Lead, Security Lead, SVP Product, Compliance Lead, SRE Lead, Finance (observer).
* **Authority:** **Stop-the-line** power; only DAB can override hard gates.

## 2. Go/No-Go Checklist (must be all green)

* **Product:** Epic acceptance met; SUS ≥ 80 (NFR-U-01).
* **Safety:** Assists ≤ 0.5/1k km trend; 0 critical incidents.
* **Security:** G-S1 green; no P1s open.
* **ML:** G-M1/M2 green; explainability logs present.
* **FinOps:** G-F1/F2/F3 green; GM ≥ phase target.
* **Compliance:** Regional pack satisfied (G-C1..C4).
* **SRE:** Availability & MTTR within SLO; DR drill passed ≤ last 90 days.
* **GTM:** Scorecard met; references on track.

## 3. Escalation Paths

* **P0:** SVP Eng + Safety Lead immediate; DAB convenes ≤ 2h.
* **P1:** Area owner fixes ≤ 48h; DAB weekly review.
* **P2:** Backlog with risk acceptance by SVP Product.

---