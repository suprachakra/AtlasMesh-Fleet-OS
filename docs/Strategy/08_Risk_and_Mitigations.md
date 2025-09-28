## 0) How to use
- **Plan**: Every program must select applicable risks from `risk/catalog.yaml` + sector overlay and bind owners.
- **Run**: KRIs stream into Control Center → *Risk* tab. Any **red** tripwire auto-triggers the mapped playbook and, if marked *blocking*, halts releases/operations.
- **Prove**: Evidence pointers (SoT) in the catalog are checked in CI/CD; missing artifacts block merges.

---

## 1) Lifecycle phases & gates (where risks bite)

| Phase | What we do | Primary risk families | Gate (hard stop) |
|---|---|---|---|
| **Concept** | Define ODD, hazards, ethics guardrails | Scope creep, legal/liability, ethics | Missing HARA/STPA → **block** |
| **Pilot** | 20–50 vehicles, single site | Sensor/weather brittleness, dispatcher overload, GNSS denial | KRIs red > 24h OR assist > SLA → **block** |
| **Production-1** | 50–150 vehicles, 1–2 sites | OTA regressions, adapter drift, model drift | Twin-gates < 95% pass OR SBOM missing → **block** |
| **Scale** | 150+ vehicles, multi-tenant | Telemetry gaps, security posture, regulatory fragmentation | Audit bundle < 100% OR mTLS gaps → **block** |

> Gates are enforced via CI + runtime sentinels; see `risk/catalog.yaml:*.blocking`.

---

## 2) Risk taxonomy (canonical)

| Category | Examples (catalog IDs) | Typical KRIs (metric IDs) |
|---|---|---|
| **Cybersecurity** | R-001 Remote Fleet Disabling, R-002 Supply Chain Compromise, R-003 Cloud DDoS | `MET.SEC.DETECT_TTA`, `MET.SEC.ATTEST_FAIL_RATE`, `MET.SEC.CVE_AGE` |
| **Technical/System** | R-010 Sensor Degradation (dust/heat), R-011 GNSS Denial, R-012 Connectivity Loss | `MET.PERCEP.CONF_P50`, `MET.RELIAB.ODD_CONFORMANCE`, `MET.NET.LAT_P95` |
| **Operational/Dispatch** | R-020 Dispatcher Overload, R-021 Assist Overuse, R-022 Platoon Monitoring Gaps | `MET.UX.TRIAGE_TTD_P95`, `MET.SAFETY.ASSIST_RATE`, `MET.SAFETY.ASSIST_RTT_P95` |
| **Safety/Regulatory** | R-030 Mixed-Traffic Collisions, R-031 Emergency Response Conflict, R-032 Compliance Drift | `MET.SAFETY.INCIDENT_RATE`, `MET.GOV.AUDIT_BUNDLE_COMPLETE` |
| **ML/Data** | R-040 Model Drift, R-041 Adversarial Inputs, R-042 Data Provenance Gaps | `MET.ML.DRIFT_SCORE`, `MET.ML.OOD_RATE`, `MET.DATA.LOSS_RATE` |
| **Integration/Adapters** | R-050 Adapter/API Drift, R-051 Map/Weather Feed Staleness | `MET.INT.CONTRACT_TEST_PASS%`, `MET.MAP.FRESHNESS_HRS` |
| **Business/Financial** | R-060 Downtime Cost, R-061 Insurance/Liability Gap | `MET.RELIAB.AVAIL_ODD`, `MET.COST.$_PER_DOWNTIME_HR` |
| **Privacy/Data Protection** | R-070 GDPR/PII Breach, R-071 Cross-border Data Flow | `MET.PRIV.PII_EVENTS`, `MET.PRIV.DPA_COVERAGE%` |

---

## 3) Cross-sector KRI targets (base; sector overrides apply)

| KRI (Metrics Canon ID) | Target | Window | Source of Truth |
|---|---:|---|---|
| `MET.RELIAB.AVAIL_ODD` | ≥ **99.3%** | rolling 30d | ops.availability_dashboard |
| `MET.SAFETY.ASSIST_RATE` | ≤ **0.3 / 1,000 km** | rolling 30d | assist.logs_per_km |
| `MET.SAFETY.ZERO_HARM` | **0 critical** incidents | quarterly | safety.event_registry |
| `MET.GOV.AUDIT_BUNDLE_COMPLETE` | **100%** | per release | ci.evidence_bundle_score |
| `MET.SEC.CVE_AGE` | **≤ 15 days** critical CVEs | rolling | vuln.mgmt_report |
| `MET.ML.DRIFT_SCORE` | **≤ θ** (canon threshold) | 24h | ml.drift_sentry |
| `MET.INT.CONTRACT_TEST_PASS%` | **100%** on pinned versions | per build | integrations.contract_tests |

> Sector overlays tweak these (see §6 and `risk/overlays/*`).

---

## 4) Top risks (detail) — condensed tables

### 4.1 Cybersecurity (sample)

| ID | Risk | Tripwire (red) | Prevent/Detect/Correct | Fallback | Evidence (SoT) | Owner |
|---|---|---|---|---|---|---|
| **R-001** | Remote Fleet Disabling (mass “stolen”/kill) | `MET.SEC.DETECT_TTA > 5m` **OR** `MET.SEC.ATTEST_FAIL_RATE > 0.5%/15m` | **Prevent:** mTLS, HSM keys, ABAC/JIT, signed intents • **Detect:** SIEM rules, UEBA • **Correct:** SOAR auto-isolate, key rotate | Graduated safe-stop zones; unaffected tenants continue; staged un-isolation post-attestation | SIEM alerts, OTA attestation logs, SBOM | **CISO** |
| **R-002** | Supply Chain Compromise (malicious lib/fw) | Critical CVE age > 15d OR unsigned artifact detected | **Prevent:** SBOM, Sigstore, vendor attestation • **Detect:** SCA, runtime attesters • **Correct:** hotfix & forced update | Freeze affected component set; last-green rollback | SBOM, SCA report, attester proofs | **SVP Eng** |
| **R-003** | Cloud DDoS/tenant isolation failure | `MET.NET.RTT_P95 > 500ms` for 10m | **Prevent:** WAF + rate-limits • **Detect:** RTT/5xx monitors • **Correct:** autoscale, traffic scrubbing | Offline-first mode up to 60m | SRE dashboards, incident log | **SRE Lead** |

### 4.2 Technical/System (sample)

| ID | Risk | Tripwire (red) | Prevent/Detect/Correct | Fallback | Evidence | Owner |
|---|---|---|---|---|---|---|
| **R-010** | Sensor Degradation (dust/heat) | `MET.PERCEP.CONF_P50 < 0.70` **AND** `MET.SAFETY.VISIBILITY_BAND < Vmin` 60s | **Prevent:** sensor cleaning cadence, dust-aware fusion • **Detect:** occlusion rate • **Correct:** re-route lower dust corridors | Degrade mode: speed caps, safe-stop bay | Perception confidence, weather feed provenance | **Perception Lead** |
| **R-011** | GNSS Denial/Jamming | `MET.RELIAB.ODD_CONFORMANCE < 98%` in GNSS-low tiles | SLAM fallback, convoy leader-follower, RF map | ODD clamp; forced convoy | ODD guard traces, SLAM confidence | **Routing Lead** |
| **R-012** | Connectivity Loss | Loss >10 min in corridor | Store-and-forward; mesh relay; SAT backup | Local autonomy ≤ 60m | Net health logs | **Edge Lead** |

### 4.3 Operational/Dispatch (sample)

| ID | Risk | Tripwire (red) | Prevent/Detect/Correct | Fallback | Evidence | Owner |
|---|---|---|---|---|---|---|
| **R-020** | Dispatcher Overload | `MET.UX.TRIAGE_TTD_P95 > 60s` or assist queue > N 10m | Macros, clustering, autoscale assist pods | Intake clamp; route low-risk queues | Assist logs, paging history | **Ops Lead** |
| **R-021** | Assist Overuse | `MET.SAFETY.ASSIST_RATE > 0.5/1k km` 24h | Scenario mining, policy coverage expansion | Build rollback; train scripts | Weekly assist RCA | **Safety Lead** |

### 4.4 ML/Data (sample)

| ID | Risk | Tripwire (red) | Prevent/Detect/Correct | Fallback | Evidence | Owner |
|---|---|---|---|---|---|---|
| **R-040** | Model Drift | `MET.ML.DRIFT_SCORE > θ` or `MET.ML.OOD_RATE > x%` 30m | Shadow eval, canary, data QC | Roll back to last green; raise assist budget | Model card, drift sentry | **Head of Data** |
| **R-041** | Adversarial Inputs | Spike in misclassifications on guarded assets | Adv. test in CI; input sanitization | Policy tighten; human review | CI reports | **ML Sec** |

(Full list in `../risk/catalog.yaml`.)

---

## 5) Acceptance & enforcement (no-loopholes)
- **Blocking flags**: Any risk with `blocking: true` in catalog halts release or operations when red. Only the CRO (or delegate) can grant one-time waivers, logged in `risk/waivers/*.md`.
- **Traceability**: Each risk maps to ≥1 **scenario (SCN.*)**, ≥1 **FR/NFR**, and ≥1 **metric (MET.*)**.
- **Evidence**: Each control points to a verifiable SoT (table, log, build artifact). CI fails if missing.

---

## 6) Sector overlays (what changes per sector)

| Sector | Key threshold tweaks | Extra controls |
|---|---|---|
| **Defense** | `MET.RELIAB.ODD_CONFORMANCE ≥ 98%` in GNSS-low tiles; `MET.SAFETY.RESPONSE_TTA` stricter | LE protocol drill monthly; air-gap workflows; SAT failover mandatory |
| **Mining** | Add `MET.SAFETY.GRADE_VIOLATIONS = 0`; higher heat derate thresholds | Haul-road scoring; slope alarms; dust suppression integration |
| **Logistics** | `MET.INT.CONTRACT_TEST_PASS% = 100%` on WMS/TOS pins; `MET.QUAL.TEMP_EXCURSIONS = 0` | Adapter canary; cold-chain sensor self-test |
| **Ride-hail** | `MET.EXP.ETA_P95 ≤ 7m`, work-zone assists ↓ 40% q/q | Incident explainers; accessibility drills; LE interaction scripts |

See `../risk/overlays/*.yaml` for machine-readable details.

---

## 7) Pre-mortems & drills
- **Monthly tabletop**: top-3 risks by exposure (see `../risk/drills.md`).
- **Quarterly live drill**: EW/jam, LE stop, HAZMAT spill, OTA rollback.
- **Missed drill** → **yellow**; repeat miss → **red** for the owning team.

---

## 8) Appendices
- **Catalog (enforced)**: `../risk/catalog.yaml`
- **Overlays**: `../risk/overlays/{def,min,log,rh}.yaml`
- **Drills & playbooks**: `../risk/drills.md`
- **Lint rules**: `../lint/risk-rules.yml`

