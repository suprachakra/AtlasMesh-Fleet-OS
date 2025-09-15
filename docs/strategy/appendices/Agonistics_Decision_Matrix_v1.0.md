# AtlasMesh Fleet OS — Agnostics Final Decision Matrix (v1.0)

> **Applies repo-wide.** Variability is capped by the **Variant Budget**: ≤ **5% core code delta** and ≤ **25% test-time delta** per release caused by any single tenet. Exceed → **CCB review** → de-scope or carve-out.

---

## 1) Vehicle-Agnostic

| Category                    | Detail                                                                                                                                                                                                                                                                                              |
| --------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **Decision**                | **Qualified YES (class-level, model-bounded)**                                                                                                                                                                                                                                                      |
| **In-Scope (v1)**           | **Vehicle classes**: Light industrial (ATV/UTV), Medium (van/pickup), Heavy (mine haul, terminal tractor), Defense UGV/8×8. **Drive-by-wire** available; **redundant braking**; DBW latency budget ≤ **20 ms** link, ≤ **80 ms** end-to-end actuation.                                              |
| **Out-of-Scope (v1)**       | No rail/air/marine control loops; no non-deterministic actuators; no vehicles without certified DBW; no bespoke one-off prototypes without safety artifacts.                                                                                                                                        |
| **Assumptions**             | Fleet owners can provide CAN/J1939/FlexRay specs; brake test tracks are available; per-vehicle mass/inertia data can be measured; firmware versioning and rollback are supported.                                                                                                                   |
| **Mandatory Guardrails**    | **Vehicle Profile** per model (mass, wheelbase, CG, brake curves, max jerk/lat accel, actuator latencies). **Controller & planner parameters** are profile-scoped (config only). **Safety Bundle** (HARA/FMEA deltas, stopping distance certs, HiL traces). **No core forks** to support a vehicle. |
| **KPIs / SLOs**             | Availability ≥ **99.0%** across ≥ **3** classes; **assist rate ≤ 2 / 1,000 km**; controlled stop distance within **±5%** of profile spec; **telemetry gap < 0.5%** per 24 h.                                                                                                                        |
| **Evidence / Validation**   | **HiL** per profile; **closed-course brake/stability** tests; **PR e2e sim gates** using vehicle dynamics surrogates; **on-road shadow** results; certification artifacts attached to release tag.                                                                                                  |
| **Top Risks → Mitigations** | Dynamics mismatch → **Profile conformance test** on loadout changes. DbW faults → **watchdog + degraded modes** (speed cap, limp-home). Supply variability → **profile templates** + adapter tests. Ops overload → **garage tooling** auto-detects buses & DIDs.                                    |
| **Cost/Complexity**         | Initial +30–40% to build profiles/HiL; run +10–15% QA matrix.                                                                                                                                                                                                                                       |
| **Dependencies**            | `edge/vehicle-agent`, `services/gateway-vehicle`, `services/fleet-health`, `services/ota-manager`.                                                                                                                                                                                                  |
| **Implementation Enablers** | `/configs/vehicles/*` (YAML), `/rules/odd/vehicle_limits.rego`, scenario bank: `/sim/scenario-bank/vehicle/*`.                                                                                                                                                                                      |
| **Change Control**          | New vehicle → **CCB** approval with Safety Bundle and performance deltas; deprecations with **two releases** notice.                                                                                                                                                                                |
| **Kill-Switch**             | ≥ **2** critical incidents attributable to profile gaps in 2 quarters, or **>25%** schedule slip due to vehicle variance → pause new profiles; revert to Tier-A only.                                                                                                                               |
| **Expand Criteria**         | Two consecutive releases meeting KPIs + **TCO ↓ ≥15%** vs single-vehicle baseline → add one new model/class.                                                                                                                                                                                        |
| **Safety/Regulatory**       | ISO 26262 work products per profile; SOTIF safety case deltas; UNECE R155/156 for cybersecurity/OTA.                                                                                                                                                                                                |
| **Data/ML**                 | Per-profile **model card** for control parametrization; dynamics features versioned; drift sentry on actuation latency.                                                                                                                                                                             |
| **Design/UX**               | Operator UIs surface **profile-aware** limits (speed, grade, payload); consistent terminology via **design tokens**.                                                                                                                                                                                |
| **QA/Test**                 | **Matrix**: class×payload×grade×μ; **golden braking** suites; **fault injection** (sensor/actuator) on HiL.                                                                                                                                                                                         |

---

## 2) Platform-Agnostic (Cloud/On-Prem)

| Category                | Detail                                                                                                                                              |
| ----------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------- |
| **Decision**            | **YES (Kubernetes-first, provider-neutral)**                                                                                                        |
| **In-Scope (v1)**       | K8s (EKS/GKE/AKS/k3s), CSI/CNI standard; Postgres; Kafka/NATS; S3-compatible object store; OpenTelemetry; Hashicorp Vault or cloud KMS via adapter. |
| **Out-of-Scope (v1)**   | No hard dependency on proprietary PaaS (e.g., BigQuery-only features); no serverless runtime coupling; no cloud-specific IAM baked into app code.   |
| **Assumptions**         | Customer can host k8s or consume managed k8s; outbound egress allowed for updates (or staged via depot).                                            |
| **Guardrails**          | **Helm/Kustomize** overlays per provider; **infra conformance tests**; **SBOM signing**; single **deployment interface** (`deploy/terraform`).      |
| **KPIs / SLOs**         | Green CI on **cloud+on-prem** matrix per release; **cold start ≤ 60 min**; **P99 RPC latency** budget maintained across providers.                  |
| **Evidence**            | CI artifact: multi-provider integration tests; load tests w/ SLO attestation; IaC drift detection logs.                                             |
| **Risks → Mitigations** | Feature gaps → **adapter** layer + graceful degrade. Cost creep → **FinOps dashboards** per env. Ops sprawl → **runbooks** per provider.            |
| **Cost/Complexity**     | +20% infra work initially; +10% CI runtime for matrix jobs.                                                                                         |
| **Dependencies**        | `deploy/terraform`, `deploy/helm`, `security/sbom`, `ci/pipelines`.                                                                                 |
| **Change Control**      | New provider requires **conformance suite pass**; two-release deprecation window.                                                                   |
| **Kill-Switch**         | If provider blocks SLOs or increases TCO by **>25%**, we downgrade to Tier-B support.                                                               |
| **Expand Criteria**     | Two clients on new provider with SLOs met → promote to Tier-A.                                                                                      |
| **Safety/Reg**          | R155/156 OTA compliance preserved on all infra; evidence portability guaranteed.                                                                    |
| **Data/ML**             | Feature store supports **pluggable** storage (S3/GCS/MinIO) via contracts.                                                                          |
| **Design/UX**           | Operator UX unaffected; only deployment targets differ.                                                                                             |
| **QA/Test**             | **Smoke + load** on each provider; **failover drills** scripted.                                                                                    |

---

## 3) Sector-Agnostic

| Category                | Detail                                                                                                                                  |
| ----------------------- | --------------------------------------------------------------------------------------------------------------------------------------- |
| **Decision**            | **YES (shared backbone + sector overlays)**                                                                                             |
| **In-Scope (v1)**       | Defense, Mining, Logistics/Ports, Ride-hail. Differences expressed via **policies, KPIs, UI tokens**, not core forks.                   |
| **Out-of-Scope (v1)**   | Rail, crewed aviation, maritime autonomy (future siblings).                                                                             |
| **Assumptions**         | Sector SMEs available; regulator expectations documented; site access for acceptance tests.                                             |
| **Guardrails**          | **Policy engine** gates constraints; **jurisdiction packs**; **design tokens** for language/workflows; **evidence mappers** per sector. |
| **KPIs / SLOs**         | ≥ **90%** code shared; sector UI code ≤ **5%**; sector SLAs met in **≥2** sectors simultaneously each release.                          |
| **Evidence**            | Sector acceptance scenarios in `/sim/scenario-bank/sector/*`; policy unit tests; evidence bundles mapped to sector frameworks.          |
| **Risks → Mitigations** | UX confusion → sector-specific **workspace modes**. Compliance drift → **policy versioning** + audits.                                  |
| **Cost/Complexity**     | +10–15% overlays; saved by shared backbone.                                                                                             |
| **Dependencies**        | `rules/policy`, `configs/sectors/*`, `ui/design-system`.                                                                                |
| **Change Control**      | New sector needs **overlay RFC** + 12 critical scenarios; CCB sign-off.                                                                 |
| **Kill-Switch**         | If sector overlay causes > **10%** regression in other sectors' SLOs for 2 releases → freeze overlay expansion.                         |
| **Expand Criteria**     | Achieve target KPIs in pilot site + zero critical incidents → move to Tier-A support.                                                   |
| **Safety/Reg**          | Sector-specific standards linked (e.g., GMG mining, defense ROE).                                                                       |
| **Data/ML**             | Sector tags on data; model endpoints enable **policy features** as inputs.                                                              |
| **Design/UX**           | Role-based navigation per sector; glossary swaps via tokens.                                                                            |
| **QA/Test**             | Suite per sector; **canary** sites per release.                                                                                         |

---

## 4) Sensor-Agnostic

| Category                | Detail                                                                                                                                                                 |
| ----------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **Decision**            | **Conditional YES (class-agnostic; certified "Sensor Packs")**                                                                                                         |
| **In-Scope (v1)**       | LiDAR, radar, camera, IMU, GNSS; two **certified packs**: **Rugged-A** (64-beam LiDAR + radars + thermal + HDR cams), **Urban-B** (solid-state LiDAR + cams + radar).  |
| **Out-of-Scope (v1)**   | Unproven modalities (event cams, mmWave imaging) in production; mixing sensors outside defined packs.                                                                  |
| **Assumptions**         | Time-sync hardware (PTP/GNSS), calibration targets on site, vendor SDKs available under stable licenses.                                                               |
| **Guardrails**          | **Sensor HAL**, **time sync daemon**, **calibration SOP**; pack-bound **perception models**; **latency/throughput** budgets; **fail-silent** policy for flaky sensors. |
| **KPIs / SLOs**         | Perception latency p95 ≤ **60 ms** (pack-specific); precision/recall ≥ pack targets; false-stop ≤ **1/2,000 km**; **pack swap** without app change.                    |
| **Evidence**            | Pack-specific datasets; confusion matrices; night/fog/dust suites; golden replays.                                                                                     |
| **Risks → Mitigations** | Vendor EOL → second-source within pack. Calibration drift → **scheduled recal** + live drift detection.                                                                |
| **Cost/Complexity**     | +25–35% initial; +15% test matrix.                                                                                                                                     |
| **Dependencies**        | `adapters/sensors/*`, `ml/models/perception/*`, `data/quality/drift`.                                                                                                  |
| **Change Control**      | New pack needs model card, SOPs, KPI passes; CCB.                                                                                                                      |
| **Kill-Switch**         | 2+ perception-attributable incidents per quarter → pack frozen.                                                                                                        |
| **Expand Criteria**     | Three months KPI adherence across two sites → promote.                                                                                                                 |
| **Safety/Reg**          | SOTIF hazards catalog per pack.                                                                                                                                        |
| **Data/ML**             | Model per pack; **feature schema versioning**; online drift alerts.                                                                                                    |
| **Design/UX**           | Ops UI shows pack health and calibration state.                                                                                                                        |
| **QA/Test**             | Environmental chambers, dust/fog tests; sensor fault injection.                                                                                                        |

---

## 5) Map-Source-Agnostic

| Category                | Detail                                                                                                                                                                     |
| ----------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| **Decision**            | **YES (semantic contract + provenance; credibility > freshness)**                                                                                                                          |
| **In-Scope (v1)**       | HERE/ESRI/OSM/custom site maps; **local site maps** for mines/ports; **SLAM degrade** mode with restricted capabilities.                                                                   |
| **Out-of-Scope (v1)**   | Providers without lane-level semantics where lane-level is required; using stale maps for safety-critical routing.                                                                         |
| **Assumptions**         | Providers supply change-feeds/diffs; we can deploy limited human verification when conflicts arise.                                                                                        |
| **Guardrails**          | **Map Data Contract** (lanes, rules, turn restrictions, elevations); **provenance signatures**; **conflict resolver** with human-in-the-loop escalation; versioned **RoadGraph per trip**. |
| **KPIs / SLOs**         | Route correctness ≥ **99.x%** on golden corridors; ETA median error within target band; **map conflict MTTResolve ≤ 24h** site/≤ 48h city.                                                 |
| **Evidence**            | Contract tests per provider; replay routes after updates; provenance logs attached to trips.                                                                                               |
| **Risks → Mitigations** | Staleness → **credibility overrides**; auto-blacklist segments. Semantic gaps → **augmentation layers**.                                                                                   |
| **Cost/Complexity**     | +15% fusion & ops; human verification budgeted.                                                                                                                                            |
| **Dependencies**        | `services/map-service`, `services/routing`, `rules/policy`.                                                                                                                                |
| **Change Control**      | Provider onboarding with contract pass + trial corridor.                                                                                                                                   |
| **Kill-Switch**         | Repeated contract breaches → provider disabled.                                                                                                                                            |
| **Expand Criteria**     | Six months with <0.1% routing regressions → lift restrictions.                                                                                                                             |
| **Safety/Reg**          | Evidenced map versions in safety case.                                                                                                                                                     |
| **Data/ML**             | Map features as inputs; provenance tracked as features.                                                                                                                                    |
| **Design/UX**           | Ops view flags **credibility/freshness**; conflict banners.                                                                                                                                |
| **QA/Test**             | Map diffs trigger **auto-replay** test battery.                                                                                                                                            |

---

## 6) Weather-Source-Agnostic

| Category                | Detail                                                                                                                              |
| ----------------------- | ----------------------------------------------------------------------------------------------------------------------------------- |
| **Decision**            | **YES (fusion with confidence & freshness budgets)**                                                                                |
| **In-Scope (v1)**       | **1P** local sensors, **2P** site feeds, **3P** met APIs; confidence-weighted fusion; weather-aware routing/assist policies.        |
| **Out-of-Scope (v1)**   | Blind majority voting; using stale feeds beyond **freshness TTL**; operating in weather states outside ODD.                         |
| **Assumptions**         | Sensor siting supports quality signals; APIs with SLAs; sites allow periodic recalibration.                                         |
| **Guardrails**          | **Confidence model**, **freshness timeouts**, **fallback ladders**; operator override with audit; **policy thresholds** per sector. |
| **KPIs / SLOs**         | Weather-related incidents trend **↓ QoQ**; route hindsight shows **positive net utility**; fusion output availability ≥ **99.5%**.  |
| **Evidence**            | Backtests against ground truth; post-incident attribution; fusion model card.                                                       |
| **Risks → Mitigations** | Noisy sensors → **robust stats** + outlier filters. Feed outages → **cached nowcasts** + redundancy.                                |
| **Cost/Complexity**     | +10–15% compute; site sensor CAPEX.                                                                                                 |
| **Dependencies**        | `services/weather-fusion`, `rules/policy`, `services/routing`.                                                                      |
| **Change Control**      | New provider requires calibration report + 30-day shadow.                                                                           |
| **Kill-Switch**         | Fusion negative utility across 2 releases → pin to most credible single feed.                                                       |
| **Expand Criteria**     | Two seasons w/ KPI gains → expand coverage.                                                                                         |
| **Safety/Reg**          | Weather gates in safety argument; ODD boundaries enforced.                                                                          |
| **Data/ML**             | Weather features stored with lineage; model retrains seasonally.                                                                    |
| **Design/UX**           | Operator view shows **confidence bands** and policy effects.                                                                        |
| **QA/Test**             | Synthetic weather scenarios; sensor sabotage tests.                                                                                 |

---

## 7) Comms-Agnostic (LTE/5G, Wi-Fi, SATCOM, Mesh, V2X)

| Category                | Detail                                                                                                                                                   |
| ----------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **Decision**            | **YES (multi-path orchestration + offline-first)**                                                                                                       |
| **In-Scope (v1)**       | LTE/5G, Wi-Fi, SATCOM, mesh; **V2X** for local broadcast; **store-and-forward** for evidence; **WAN-independent** motion control.                        |
| **Out-of-Scope (v1)**   | Remote joystick driving over WAN; dependence on single carrier; unauthenticated V2X.                                                                     |
| **Assumptions**         | SATCOM costs acceptable for niche; PKI infra available; vehicles have edge storage for **≥45 min**.                                                      |
| **Guardrails**          | **Policy budgets** (latency, jitter, cost); **mTLS** everywhere; V2X with PKI and misbehavior detection; **graceful degradation** (assist → hints only). |
| **KPIs / SLOs**         | Assist RTT p50 **<30 s**, p95 **<90 s**; handover success ≥ **99%**; **offline continuity ≥45 min**; evidence upload **T+2 h**.                          |
| **Evidence**            | Handover drills; link failover chaos tests; upload completion logs.                                                                                      |
| **Risks → Mitigations** | Coverage gaps → SATCOM/mesh fallback; cost spikes → **policy cost caps**; security → strict cert rotation, IDS.                                          |
| **Cost/Complexity**     | SATCOM OPEX (controlled by policies); +10% engineering for orchestration.                                                                                |
| **Dependencies**        | `edge/vehicle-agent`, `services/gateway-vehicle`, `services/alerts-incident`.                                                                            |
| **Change Control**      | New carrier or mesh tech requires field trials + security review.                                                                                        |
| **Kill-Switch**         | If assists routinely exceed RTT p95 or outages > SLA → disable costly path and re-tune policies.                                                         |
| **Expand Criteria**     | 3-month stability + cost within budget → broader rollout.                                                                                                |
| **Safety/Reg**          | WAN-independent safety; evidence retention policies met.                                                                                                 |
| **Data/ML**             | Bandwidth-aware telemetry sampling; loss-robust encodings.                                                                                               |
| **Design/UX**           | Operator UI shows comms path, cost meter, and confidence.                                                                                                |
| **QA/Test**             | RF-shield tests; moving-cell vehicle trials; V2X spoofing tests.                                                                                         |

---

## How to Use This Matrix in the Repo

* Add as `docs/Strategy/appendices/Agonistics_Decision_Matrix_v1.0.md`.
* For each tenet, create an **ADR** (`ADR/00xx-<tenet>.md`) linking: scope, guardrails, KPIs, and **the exact CI/twin gates** that enforce them.
* Wire KPIs to gates:

  * **Sim/Twin**: `/sim/ci-gates/*` must include vehicle×sector×pack×weather matrices.
  * **Contracts**: `/data/contracts/*`, `/rules/*` tests must pass for **every adapter**.
  * **Release Evidence**: `/compliance/safety-case/*` bundles auto-attached to release tags.
* Enforce **Variant Budget** via CI policy: merge blocked if delta in matrix cost > thresholds.

---

### Why this is "no-loopholes"

* Every "YES" is conditional with **scope fences, hard KPIs, and kill-switches**.
* Every variability point maps to **specific repo paths** and **automated gates**.
* We've anticipated **safety, data, design, QA, ops, cost, and compliance** per tenet.
