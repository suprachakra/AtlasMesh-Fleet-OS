# AtlasMesh Fleet OS — Objectives and Key Results (OKRs)

**Period:** FY-1 (two halves: H1, H2)
**Scoring:** 0.0–1.0; **stretch** set where 0.7–0.8 is success.
**Guardrails:** Safety, compliance, and ethics **override** revenue.

## A) Company-Level OKRs (AtlasMesh Fleet OS)

### **Objective A1 — Prove repeatable L4 operations in harsh & controlled ODDs.**

| Key Result | Target | Measurement Methodology | Data Source | Validation Frequency | Owner | Dependencies |
| --- | --- | --- | --- | --- | --- | --- |
| **KR1:** Ship **3 sector pilots** to production SLAs | 1× Mining, 1× Port/Yard, 1× Defense; ≥95% KPIs met for 60 days | Sector-specific KPI dashboard with daily aggregation; 60-day rolling window | Telemetry system; customer acceptance docs | Daily tracking; Weekly review; 60-day validation | Sector Leads | Vehicle readiness; site preparation; regulatory approval |
| **KR2:** Achieve **fleet availability** | ≥99.3% (rolling 30-day) | (Operational hours - Downtime) / Operational hours; categorized by cause | Vehicle heartbeat; maintenance logs; incident records | Real-time monitoring; Daily aggregation; Weekly review | Operations Lead | Hardware reliability; software stability; maintenance processes |
| **KR3:** **Assists & safety** | ≤0.5 / 1,000 km; zero critical incidents | Assist count / distance traveled; incident severity classification | Assist logs; safety event records; distance tracking | Real-time monitoring; Daily review; Weekly trend analysis | Safety Lead | Perception quality; policy coverage; scenario testing |
| **KR4:** Deliver **audit-ready safety case bundles** | ≥6 releases; 2 jurisdiction sign-offs | Bundle completeness score; regulatory submission tracking | CI/CD pipeline; compliance system; regulatory correspondence | Per-release verification; Monthly regulatory review | Compliance Lead | Evidence collection; regulatory relationships; documentation quality |

### **Objective A2 — Demonstrate ROI and scalability (commercial proof).**

* **KR1:** Show **measured KPI deltas**: Mining cost/ton −10%+, Port cycle −20%+, Ride-hail wait P95 ≤7m.
* **KR2:** Close **\$XXM TCV** across 6 logos (≥2 per sector), **gross margin ≥55%** on software.
* **KR3:** **Install time ≤8 hours**/vehicle for target SKUs; **site cutover ≤14 days**.

### **Objective A3 — Be credibly agnostic (vehicle, sensor, map, cloud).**

* **KR1:** Certify **5 vehicle classes** (haul truck, yard tractor, 4×4 UGV, forklift, van).
* **KR2:** Run in **3 cloud/on-prem modes** with identical APIs (GCP/Azure/on-prem).
* **KR3:** Support **2 lidar vendors, 2 camera stacks, 2 GNSS/INS** with hot-swap configs.
* **KR4:** Policy & rules as code with **100% tests**; change safely in <24h.

## B) Workstream OKRs

### **Product Management**

* **O:** Align roadmap to sector KPIs and avoid "forking hell."

  * **KR1:** Publish & maintain **policy packs** for Defense/Mine/Logistics/Ride-hail with **<2-day lead time** for regulatory changes.
  * **KR2:** Convert **100% of use-case rows** to full Markdown specs with acceptance criteria & sim gates.
  * **KR3:** Win **NPS ≥50** from ops supervisors in all live sites.

### **Design & UX**

* **O:** Make the control center and LE/rider HMIs **obvious and trust-building**.

  * **KR1:** **WCAG 2.2 AA** compliance; Arabic/RTL support live.
  * **KR2:** **Time-to-decision** for incident triage **≤60s P95** in usability tests.
  * **KR3:** **LE/First-Responder flow** validated by 2 agencies; measurable **miscommunication incidents = 0**.

### **Engineering (Platform & Edge)**

* **O:** Ship a hardened platform with predictable performance.

| Key Result | Target | Measurement | Dependencies | Risks | Mitigation | Owner |
| --- | --- | --- | --- | --- | --- | --- |
| **KR1:** Control-loop latency | **P95 ≤ 50 ms** on-vehicle; routing ETA error ≤ 8% | Instrumented telemetry; performance testing | Hardware capabilities; sensor processing; network latency | Hardware constraints; algorithm complexity; environmental factors | Performance profiling; optimization sprints; degradation modes | Platform Lead |
| **KR2:** OTA reliability | **Blue/green OTA** with attestation; failure rollback < 5 minutes | Deployment metrics; rollback testing; attestation verification | Security infrastructure; network connectivity; storage capacity | Network unreliability; verification failures; storage constraints | Staged rollout; fallback mechanisms; delta updates | OTA Lead |
| **KR3:** CI twin gates | Scenario pass rate **≥95%** for merge | CI metrics; coverage analysis; scenario execution time | Simulation infrastructure; scenario library; test automation | Infrastructure scaling; scenario gaps; false positives/negatives | Infrastructure investment; scenario mining; test refinement | DevOps Lead |

**Interdependencies**:
- **Provides to**: Product (feature velocity), QA (quality baseline), Operations (deployment reliability)
- **Consumes from**: Data (models, datasets), QA (test scenarios), Security (requirements)

**Risk Factors**:
- **Critical Path Items**: Control-loop performance, OTA reliability
- **External Dependencies**: Hardware supply chain, cloud infrastructure
- **Technical Debt Concerns**: Test coverage, performance optimization

**Quarterly Milestones**:
1. Q1: Control-loop optimization complete; OTA attestation implemented
2. Q2: Twin gate coverage expanded to 90% of critical paths
3. Q3: Performance telemetry dashboard with automated alerting
4. Q4: Full matrix testing across all supported configurations

### **Data/ML**

* **O:** Data drives safer autonomy and lower cost, with provenance.

  * **KR1:** Instrument **100%** of vehicles with provenance & freshness scoring; **data loss < 0.1%**.
  * **KR2:** PdM (predictive maintenance) models reduce **unscheduled downtime −20%**; document precision/recall & gap closure plan.
  * **KR3:** Demand/dispatch model improves **seat-fill% +10%** (ride-hail) or **loader idle −25%** (mining).

### **QA & Safety**

* **O:** No regressions slip; safety is continuously evidenced.

  * **KR1:** **0** open **critical** defects in production; **MTTR < 1h** for Sev-1.
  * **KR2:** **HARA/STPA** artifacts generated per release; **evidence bundle** produced and archived automatically.
  * **KR3:** Drill **quarterly**: EW/jam, sensor loss, LE stop, HAZMAT spill; after-action fixes merged < 2 sprints.

### **Security**

* **O:** Make compromise economically infeasible.

  * **KR1:** mTLS everywhere; **SBOM signed** per release; **no critical CVEs > 15 days**.
  * **KR2:** Red-team 2×/year; **no P1 findings remain > 30 days**.
  * **KR3:** **Secure OTA** with device attestation; keys rotated quarterly.

### **GTM & Partnerships**

* **O:** Win credible lighthouse sites and references.

  * **KR1:** 6 logos; **3 referenceable** case studies with KPI diffs and quotes.
  * **KR2:** **3 integrator partners** (ports, mining, defense primes) enabled; **time-to-first-POC < 45 days**.
  * **KR3:** **TCO calculators** per sector live; win rate ≥30% qualified deals.

### **Operations**

* **O:** Stand up repeatable deployments.

  * **KR1:** Site playbooks (ports, mines, bases) **100%** published; average **site readiness ≤ 10 business days**.
  * **KR2:** **Spares & RMA** process with **<72h** turnaround.
  * **KR3:** **24/7 on-call** SLO: **page-ack P95 < 5 min**.

### **Compliance & Ethics**

* **O:** Proactive compliance and transparent ethics.

  * **KR1:** **Policy guardrails**: no weaponization, privacy-by-default, data retention limits; audits pass **100%**.
  * **KR2:** **Regulator onboarding kits** delivered for 2 countries; cycle time to permit **≤ 60 days**.
  * **KR3:** Public **transparency report** published H2.

## C) Sector OKRs (lighthouse-specific)

### **Mining (Lighthouse #1)**

* **KR:** Tons/hr **+10%**, cost/ton **−12%**, availability **≥99.5%**, water-dust compliance **100%**.

### **Port/Yard (Lighthouse #2)**

* **KR:** Dock/yard cycle **−25%**, miss-spot **≈0**, gate dwell **−15%**, incidents **0**.

### **Defense (Lighthouse #3)**

* **KR:** Mission completion **≥98%**, exposure-hours **−80%**, assists **≤0.3/1k km**, audit bundle accepted.

### **Ride-Hail (Pilot Corridor)**

* **KR:** Wait P95 **≤7 min**, CSAT **≥4.8/5**, assist **≤0.5/1k km**, zero critical incidents.

## C.1) OKR Interdependency Matrix

| OKR ID | Depends On | Enables | Critical Path | Risk if Missed | Catch-Up Strategy |
| --- | --- | --- | --- | --- | --- |
| **O-1 (Safety)** | Engineering capability (O-3) | Regulatory trust (O-4) | **Yes** | Deployment halt; safety incidents | Feature freeze; safety sprint; external audit |
| **O-2 (Time-to-Value)** | Deployment automation (Ops OKR) | Cost & Scale (O-3) | **Yes** | Revenue delay; customer dissatisfaction | Deployment SWAT team; phased approach |
| **O-3 (Cost & Scale)** | Core platform stability (Eng OKR) | Business viability; expansion | No | Margin pressure; limited market | Cost optimization sprint; feature prioritization |
| **O-4 (Regulatory)** | Safety & evidence (O-1, O-4) | Market access; customer trust | **Yes** | Market exclusion; deployment blocks | Compliance task force; jurisdiction prioritization |
| **O-5 (UX)** | Control center (Ops OKR) | Adoption; referenceability | No | Training costs; operator resistance | UX sprint; training enhancement; feedback loops |

**Critical Path Sequence**:
1. Core safety capabilities (O-1)
2. Evidence generation & compliance (O-4)
3. Deployment automation (O-2)
4. Scaling & optimization (O-3, O-5)

This sequence ensures that foundational safety and compliance are established before scaling, with deployment efficiency as the bridge between capability and scale.

## D) KPI Dictionary (unambiguous)

| KPI | Definition | Calculation Method | Data Source | Baseline | Target | Owner | Reporting Frequency |
| --- | --- | --- | --- | --- | --- | --- | --- |
| **Assist rate** | Frequency of human assistance required during autonomous operation | (# Q\&A assist events) / (1,000 km), excludes cosmetic operator UI clicks | Assist platform logs; vehicle telemetry | 2.0/1,000 km | ≤0.5/1,000 km | Safety Lead | Daily |
| **Availability** | Percentage of time vehicles are operational and mission-capable | (vehicle mission-capable hours) / (scheduled hours) | Vehicle status logs; maintenance system; scheduling system | 97.5% | ≥99.3% | Operations Lead | Daily |
| **Dock cycle time** | Total time for container handling from gate entry to exit | gate-in → gate-out per container move | TOS integration; gate systems; vehicle telemetry | 45 minutes | ≤30 minutes | Logistics Lead | Daily |
| **Cost/ton** | Total operational cost per ton of material moved | (fuel+energy+tires+maintenance+labor) / moved tons | Fleet management system; ERP integration; production system | Varies by site | -10-15% vs. baseline | Mining Lead | Weekly |
| **Wait P95** | 95th percentile of passenger waiting time | 95th percentile time rider waits from request to pickup | Ride request system; vehicle dispatch logs | 12 minutes | ≤7 minutes | Ride-hail Lead | Daily |
| **Evidence bundle completeness** | Percentage of required compliance artifacts present in release bundle | Automated scoring based on required artifacts | CI/CD pipeline; compliance system | 85% | 100% | Compliance Lead | Per release |

## E) Cadence, Reviews, and Source of Truth

* **Monthly exec review**: A-level OKRs; sector dashboards; risk register.
* **Bi-weekly workstream reviews**: product, eng, data, ops, safety.
* **Dashboards**: `ui/control-center-web` → "KPIs" tab (live), and `docs/architecture/diagrams/okrs.pbix` (snapshot).
* **Change control**: Any KR change requires **ADR** update (`ADR/00xx-okr-adjustment.md`) with rationale, data, and stakeholder sign-offs.

## F) Implementation Plan & Governance

| Quarter | Key Activities                                                                                  | Ownership                   | Reviews & Cadence                               |
| ------- | ----------------------------------------------------------------------------------------------- | --------------------------- | ----------------------------------------------- |
| Q1      | Repo & docs scaffold; safety CI; sector catalogs; initial overlays; EW profiles; adapters scope | PM, Eng, Safety, Data       | Bi-weekly product council; monthly safety board |
| Q2      | Sector pilots (defense/mining); WMS/TOS adapters; tele-assist SLAs; PdM v1; compliance packs    | PM+Program, Ops, Compliance | Fortnightly exec review; stakeholder demos      |
| Q3      | Ride-hail ops room; energy optimizer; ODD expansion kit; incident loop < 14 days                | PM, Data/ML, SRE            | Quarterly roadmap reset                         |
| Q4      | Scale & harden; audit and certifications; partner co-sells; global readiness                    | GTM, Compliance, Finance    | Quarterly board package                         |

## G) Ethical & Compliance Integration

### Principles & Implementation

| Principle | Implementation Mechanism | Verification Method | Success Criteria | Owner | Review Cadence |
| --- | --- | --- | --- | --- | --- |
| **Safety-first** | Safety case generation; twin gates; assist budgets | Scenario coverage analysis; incident review | Zero safety incidents; 100% scenario coverage | Safety Lead | Weekly metrics; Monthly review |
| **Transparency** | Explainable decisions; audit trails; customer dashboards | UI/UX testing; audit completeness | User comprehension ≥90%; audit trail completeness | Product Lead | Bi-weekly UX testing; Monthly audit |
| **Human Dignity** | Operator workload management; accessibility compliance | Cognitive load assessment; WCAG audit | Cognitive load ≤4/7; WCAG 2.2 AA compliance | Design Lead | Monthly assessment; Quarterly audit |
| **Minimal Data** | Data minimization policies; retention controls; anonymization | Privacy impact assessment; data audit | PII reduction ≥30%; 100% compliance with retention | Data Lead | Monthly data review; Quarterly audit |
| **Explainability** | Decision logging; confidence scoring; visualization | User testing; decision trace validation | Explanation satisfaction ≥85%; trace completeness | ML Lead | Bi-weekly testing; Monthly validation |

### Controls & Enforcement

| Control | Implementation | Testing Method | Failure Response | Continuous Improvement |
| --- | --- | --- | --- | --- |
| **Policy-as-Code** | Rule engine with formal verification; CI enforcement | Static analysis; runtime verification | Build rejection; runtime alerts | Pattern extraction from incidents |
| **Safety Case Evidence** | Automated collection pipeline; completeness scoring | Gap analysis; third-party review | Release block; remediation sprint | Scenario expansion from field data |
| **Privacy-by-Design** | Data flow analysis; PII detection; minimization rules | Privacy scan; penetration testing | Data flow correction; access restriction | Regular privacy threat modeling |
| **Explainability Hooks** | Decision point instrumentation; confidence metrics | Traceability testing; user validation | Additional instrumentation; UX enhancement | User feedback incorporation |
| **Red Team Drills** | Scheduled adversarial testing; scenario simulation | Drill execution; response assessment | Process refinement; training enhancement | Scenario library expansion |

### Governance Structure

- **Ethics Review Board**: Quarterly review of principles, incidents, and emerging issues
- **Compliance Council**: Monthly review of regulatory landscape and compliance status
- **Safety Committee**: Weekly review of safety metrics, incidents, and near-misses
- **Data Governance Board**: Bi-weekly review of data practices and privacy controls

## H) Pre-Mortem (Failure Scenarios)

| ID | Scenario | Trigger Condition | Early Warning Indicators | Impact | Mitigation / Plan | Recovery Time Objective | Owner | Cross-Functional Dependencies |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| PM-01 | Heat wave + dust storm | Ambient ≥ 52°C, PM10/PM2.5 spike | Weather forecast 72h ahead; PM10 trend >20% daily increase; thermal warnings >5% of fleet | Vehicle availability drop; assist rate spike; mission aborts | EW profile; lens cleaning cadence; occlusion-aware planning; degrade-to-safe | <4h to normal operations; <30min to safe state | EW Eng | Weather data quality (Data); thermal design (Hardware); operational procedures (Ops) |
| PM-02 | GNSS denial | Jamming/spoofing zone | GNSS signal quality degradation; position confidence drop; multiple vehicles affected | Localization errors; route deviations; mission delays | Map/SLAM fallback; convoy leader-follower; ODD clamp; logging forensics | <10min to degraded operations; <2h to alternative navigation | Routing | Map quality (Data); sensor fusion (Perception); mission planning (Product) |
| PM-03 | Telecom outage | Multi-hour carrier loss | Packet loss increase; latency spikes; regional outage alerts | Tele-assist unavailable; data synchronization delays; coordination challenges | Store-and-forward; local autonomy budgets; mesh relays; BCP comms | <1min to local autonomy; <24h to full synchronization | SRE | Edge computing (Engineering); offline protocols (Product); customer communication (Support) |
| PM-04 | OTA regression | Faulty rollout | Test coverage gaps; simulation anomalies; canary metrics deviation | Fleet instability; feature regression; customer impact | Staged canary; attestation; auto rollback; A/B guardrails | <15min to detect; <30min to rollback; <4h to root cause | Release Mgr | CI/CD pipeline (DevOps); monitoring (SRE); incident response (Support) |
| PM-05 | Adapter drift | WMS/TOS API change | Integration test failures; increased error rates; version mismatch alerts | Broken handoffs; operational disruption; data inconsistency | Contract tests; adapter versioning; fallback CSV/API shim | <4h to adapter fix; <24h to full resolution | Integrations | Customer systems (External); API design (Engineering); data validation (QA) |
| PM-06 | Compliance breach | New rule unmodeled | Regulatory announcements; compliance scan alerts; jurisdiction changes | Deployment halt; regulatory scrutiny; reputation damage | Jurisdiction packs in CI; fast patch path; legal watch | <24h to compliance assessment; <72h to remediation plan | Compliance | Legal monitoring (Legal); policy engine (Engineering); documentation (Product) |
| PM-07 | Assist overload | Edge cases surge | Assist queue growth; resolution time increase; pattern emergence | SLA breach; operator fatigue; customer dissatisfaction | Case clustering; classifier routing; scripted macros; staffing surge plan | <1h to triage adjustment; <24h to pattern mitigation | Ops | Assist platform (Engineering); operator training (HR); scenario development (QA) |
| PM-08 | Model drift | Data shift | Confidence score trends; prediction error increase; OOD detection | Safety/econ degrade; performance inconsistency; assist increase | Drift sentry; shadow retrain; canary; rollback | <4h to detection; <24h to mitigation; <7d to retraining | Data/ML | Data pipeline (Data); model deployment (Engineering); validation (QA) |
| PM-09 | Sensor supply delay | Vendor EOL | Lead time increases; inventory alerts; vendor communications | Hardware shortage; deployment delays; maintenance backlog | Multi-vendor BOM; abstraction; buffer stock | <30d to alternative sourcing; <90d to design adaptation | Hardware | Supply chain (Procurement); sensor abstraction (Engineering); customer communication (Sales) |
| PM-10 | Misuse/insider | Privileged abuse | Unusual access patterns; policy violations; authentication anomalies | Safety/data risk; compliance violations; trust damage | RBAC/ABAC; just-in-time access; audit trails; alerts | <5min to detection; <30min to containment; <4h to investigation | Security | Access control (IT); monitoring (SRE); incident response (Security) |

**Early Detection System**:
- Daily automated scanning for early warning indicators
- Weekly cross-functional risk review
- Monthly simulation of top 3 risk scenarios
- Quarterly full-scale disaster recovery drill

**Recovery Readiness**:
- Documented playbooks for each failure scenario
- Pre-approved emergency response procedures
- Defined escalation paths and decision authority
- Regular tabletop exercises for response teams

## Final guardrail

Any KR pursuit that pressures **Safety, Compliance, or Ethics** fails by definition. "**No heroics**" culture: pause, explain, fix, then resume.
