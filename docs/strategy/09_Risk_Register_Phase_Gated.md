## Purpose

The single source of truth for **concrete risks** and **phase gates**. Each row: **signals → controls → tests → metrics → owner → evidence**. Scoring uses **S×L×D = RPN** with **GREEN/YELLOW/RED** gates identical to Governance §2.

### Scoring & Gates

* **S**everity, **L**ikelihood, **D**etectability (1–10 each) → **RPN (1–1000)**
* **GREEN** ≤ 80 | **YELLOW** 81–150 (ship only with passing tests) | **RED** >150 (block)

### How to read a row

* **Controls:** *Prevent / Detect / Correct* (P/D/C) are explicit.
* **Tests:** What must pass **before** the phase moves forward.
* **Metrics:** Only **IDs** from Metrics Canon (no formulas here).
* **Evidence:** Path(s) inside the release/site evidence bundle.

---

## §2 DESIGN (before coding starts / architecture frozen)

| ID         | Risk (what could go wrong)                  | Signals (top 3)                                                          | Controls (P/D/C)                                                                                               | Tests / Gate                                                         | Metrics (IDs)                           | Owner       |  S |  L |  D | **RPN** |   Gate  | Evidence                            |
| ---------- | ------------------------------------------- | ------------------------------------------------------------------------ | -------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------- | --------------------------------------- | ----------- | -: | -: | -: | ------: | :-----: | ----------------------------------- |
| **RR-D01** | ODD boundaries unclear → unsafe assumptions | Conflicting terrain limits; missing weather bands; persona/scenario gaps | **P:** ODD spec template; rugged-terrain policy catalogue • **D:** ODD lint in CI • **C:** Clamp-to-safe rules | **Gate:** ODD spec complete + lint=pass + sign-offs (Safety/Product) | MET.RELIAB.ODD_CONFORMANCE              | Product     |  8 |  5 |  4 |     160 | **RED** | `/evidence/design/odd.md`           |
| **RR-D02** | Safety requirements incomplete (SOTIF gaps) | Unmapped edge cases; reliance on ML only                                 | **P:** HARA/SOTIF library • **D:** Hazard coverage report • **C:** Degrade-to-safe behaviors                   | HARA coverage ≥ 95% critical paths                                   | MET.SAFETY.SAFE_STOP_RATE               | Safety      |  9 |  4 |  4 |     144 | **YEL** | `/evidence/safety/hara.pdf`         |
| **RR-D03** | Data/PII design weak (privacy breach later) | No DPIA; unclear retention; mixed residency                              | **P:** DPIA, data maps • **D:** Privacy lint • **C:** Tokenization/anonymization                               | DPIA approved; residency declared                                    | MET.PRIV.DPIA_STATUS, MET.PRIV.DATA_MIN | Data        |  7 |  4 |  5 |     140 | **YEL** | `/evidence/privacy/dpia.md`         |
| **RR-D04** | Supplier single-source (sensor/EOL)         | EOL notices; MOQ shocks                                                  | **P:** Dual-vendor BOM • **D:** Supply watch • **C:** Alt pack                                                 | Dual-vendor in BOM; swap tested                                      | MET.SUPPLY.DUAL_VENDOR_COVER            | Procurement |  7 |  4 |  5 |     140 | **YEL** | `/evidence/supply/bom.xlsx`         |
| **RR-D05** | Single-point failure in control compute     | No redundancy path; thermal margin < spec                                | **P:** Redundant compute path • **D:** Fault injection CI • **C:** Safe halt                                   | Fault injection pass rate ≥95%                                       | MET.SAFETY.FAILSAFE_PASS_RATE           | Eng         | 10 |  3 |  6 |     180 | **RED** | `/evidence/eng/fault_injection.log` |
| **RR-D06** | Map/SLAM plan brittle in GNSS-denial        | SLAM unproven; map source monoculture                                    | **P:** Multi-source maps • **D:** GNSS-off sim pack • **C:** Corridor clamp                                    | GNSS-off scenario pack pass                                          | MET.RELIAB.ODD_CONFORMANCE              | Perception  |  8 |  4 |  4 |     128 | **YEL** | `/evidence/maps/sim_gnss_off/`      |
| **RR-D07** | LE/first-responder HMI ambiguous            | No protocol states; missing UX scripts                                   | **P:** Protocolized HMI • **D:** Cognitive walkthroughs • **C:** On-vehicle quick card                         | LE walkthroughs pass; scripts signed                                 | MET.SAFETY.RESPONSE_TTA                 | Design      |  7 |  3 |  5 |     105 | **YEL** | `/evidence/ux/le_hmi.pdf`           |

**Design Exit Gate:** All **RED→≤YEL** and all **YEL tests passing**. Safety + Product sign-off.

---

## §3 PRE-DEPLOY (site readiness / permits / integrations)

| ID         | Risk                            | Signals                          | Controls (P/D/C)                                                                        | Tests / Gate                     | Metrics                       | Owner        |  S |  L |  D | **RPN** |   Gate  | Evidence                            |
| ---------- | ------------------------------- | -------------------------------- | --------------------------------------------------------------------------------------- | -------------------------------- | ----------------------------- | ------------ | -: | -: | -: | ------: | :-----: | ----------------------------------- |
| **RR-P01** | Permit/regs gap                 | Pending approvals; new circular  | **P:** Jurisdiction pack • **D:** Legal watch • **C:** Policy hotfix path               | Permits on file; pack=pass in CI | MET.GOV.AUDIT_BUNDLE_COMPLETE | Compliance   |  9 |  3 |  4 |     108 | **YEL** | `/evidence/compliance/permits/`     |
| **RR-P02** | Adapter/API drift (WMS/TOS/ERP) | Contract test fail; error spikes | **P:** Contract tests • **D:** Canary proxy • **C:** CSV/API shim                       | Contract tests green             | MET.OPS.TIME_TO_VALUE         | Integrations |  8 |  4 |  4 |     128 | **YEL** | `/evidence/integrations/contracts/` |
| **RR-P3**  | Comms coverage holes            | Latency > SLA; packet loss       | **P:** Multi-path (LTE/SAT/Mesh) • **D:** Drive-test heatmap • **C:** Store-and-forward | Coverage heatmap ≥ target        | MET.OPS.COMMS_UPTIME          | SRE          |  7 |  4 |  5 |     140 | **YEL** | `/evidence/network/heatmaps/`       |
| **RR-P4**  | Ops not trained on new release  | Training gaps; SOP unsigned      | **P:** Role-based training • **D:** Readiness quiz • **C:** On-call shadow              | Readiness ≥ 90% pass             | MET.OPS.TRAINING_PCT_PASS     | Ops          |  6 |  4 |  5 |     120 | **YEL** | `/evidence/ops/training/`           |
| **RR-P5**  | Incident response not drilled   | No tabletop; unclear comms       | **P:** Drill scripts • **D:** Dry-run • **C:** Pager duty                               | Drill pass; comms tree tested    | MET.SAFETY.RESPONSE_TTA       | Safety/Ops   |  8 |  3 |  5 |     120 | **YEL** | `/evidence/drills/ir_tabletop.md`   |

**Pre-Deploy Gate:** All **YEL tests passing**; no **RED**.

---

## §4 GO-LIVE (canary → staged rollout)

| ID         | Risk                               | Signals                             | Controls (P/D/C)                                                       | Tests / Gate                             | Metrics                      | Owner        |  S |  L |  D | **RPN** |   Gate  | Evidence                      |
| ---------- | ---------------------------------- | ----------------------------------- | ---------------------------------------------------------------------- | ---------------------------------------- | ---------------------------- | ------------ | -: | -: | -: | ------: | :-----: | ----------------------------- |
| **RR-G01** | Canary shows safety regressions    | Assist spike; safe-stops ↑          | **P:** Twin-gates CI • **D:** Canary metrics • **C:** Auto freeze      | Canary window clean (48–72h)             | MET.SAFETY.ASSIST_RATE       | Release      |  9 |  3 |  4 |     108 | **YEL** | `/evidence/release/canary/`   |
| **RR-G02** | OTA rollback fails                 | Attest mismatch; rollback time >30m | **P:** Signed OTA/PKI • **D:** Attestation • **C:** One-click rollback | Rollback TTA < 30m (dry-run)             | MET.OPS.RELEASE_ROLLBACK_TTA | Release      |  8 |  4 |  4 |     128 | **YEL** | `/evidence/ota/attest/`       |
| **RR-G03** | LE/first-responder handoffs fail   | 911/112 test issues                 | **P:** Protocol pack • **D:** Joint test • **C:** Hotline bridge       | LE test pass; hotline SLA                | MET.SAFETY.RESPONSE_TTA      | Safety/Comms |  8 |  3 |  5 |     120 | **YEL** | `/evidence/ops/le_test.md`    |
| **RR-G04** | Public comms misfires (brand risk) | Confusion; media misreport          | **P:** Comms playbook • **D:** Dry-run Q&A • **C:** War-room           | Playbook sign-off; spokespersons trained | MET.BRAND.SENTIMENT_IDX      | Comms        |  6 |  3 |  6 |     108 | **YEL** | `/evidence/comms/launch_kit/` |

**Go-Live Gate:** All **YEL tests passing** over canary window; no **RED**.

---

## §5 OPERATE (steady-state)

| ID         | Risk                           | Signals                     | Controls (P/D/C)                                                              | Tests / Ongoing Checks           | Metrics                               | Owner        |  S |  L |  D | **RPN** |   Gate  | Evidence                        |
| ---------- | ------------------------------ | --------------------------- | ----------------------------------------------------------------------------- | -------------------------------- | ------------------------------------- | ------------ | -: | -: | -: | ------: | :-----: | ------------------------------- |
| **RR-O01** | Model/data drift → safety deg  | Confidence drift; error ↑   | **P:** Drift sentry • **D:** Shadow eval • **C:** Rollback model              | Weekly drift report; shadow pass | MET.SAFETY.ASSIST_RATE                | Data/ML      |  8 |  4 |  4 |     128 | **YEL** | `/evidence/ml/drift/`           |
| **RR-O02** | Sensor degradation (dust/heat) | Occlusion; thermal throttle | **P:** Auto cleaning; derate • **D:** Health checks • **C:** Pull for service | PdM alerts < band; MTBF ↑        | MET.RELIAB.AVAIL_ODD                  | Eng/Ops      |  7 |  4 |  5 |     140 | **YEL** | `/evidence/ops/pdm/`            |
| **RR-O03** | Assist overload during surge   | Queue growth; P95 ↑         | **P:** Assist budget; scripts • **D:** Queue SLOs • **C:** Surge staffing     | P95 assist latency ≤ SLA         | MET.SAFETY.ASSIST_LAT_P95             | Ops          |  7 |  3 |  5 |     105 | **YEL** | `/evidence/ops/assist_sla/`     |
| **RR-O04** | Comms outage (region)          | Packet loss; carrier alert  | **P:** Multi-path • **D:** SLOs • **C:** Store-forward                        | Local autonomy continuity        | MET.OPS.COMMS_UPTIME                  | SRE          |  8 |  3 |  5 |     120 | **YEL** | `/evidence/network/slo/`        |
| **RR-O05** | Privacy breach (PII misuse)    | DLP hits; unusual access    | **P:** Minimization/RBAC • **D:** DLP/SIEM • **C:** Lockdown                  | DLP false-pos ≤ band; no P1s     | MET.SEC.INCIDENTS_P1                  | Security     |  9 |  2 |  6 |     108 | **YEL** | `/evidence/security/dlp/`       |
| **RR-O06** | SLA breach (availability/ETA)  | AVAIL dip; ETA p95 ↑        | **P:** Redundancy; rebalancing • **D:** SLA dashboards • **C:** Remediation   | AVAIL ≥ target; ETA in band      | MET.RELIAB.AVAIL_ODD, MET.EXP.ETA_P95 | Ops          |  7 |  3 |  4 |      84 | **GRN** | `/evidence/ops/sla/`            |
| **RR-O07** | Public incident / PR crisis    | Viral clip; media spike     | **P:** Transparency • **D:** Media monitor • **C:** Incident comms            | 60-min response; facts bundle    | MET.GOV.AUDIT_BUNDLE_COMPLETE         | Comms/Safety |  8 |  2 |  6 |      96 | **YEL** | `/evidence/comms/incident_kit/` |

**Operate Guardrails (continuous):** If any **Operate** row flips to **RED**, auto-freeze updates and convene Safety + CCB within **24h**.

---

## §6 UPDATE (OTA / policy / dependency changes)

| ID         | Risk                              | Signals                    | Controls (P/D/C)                                                        | Tests / Gate                  | Metrics                       | Owner      |  S |  L |  D | **RPN** |   Gate  | Evidence                          |
| ---------- | --------------------------------- | -------------------------- | ----------------------------------------------------------------------- | ----------------------------- | ----------------------------- | ---------- | -: | -: | -: | ------: | :-----: | --------------------------------- |
| **RR-U01** | Unsigned/compromised OTA          | Attest fail; SBOM mismatch | **P:** PKI/mTLS; SBOM • **D:** Sig verify • **C:** Block install        | Attestation pass; SBOM diff=0 | MET.SEC.INCIDENTS_P1          | Security   |  9 |  2 |  5 |      90 | **YEL** | `/evidence/ota/attest/`           |
| **RR-U02** | Rollback time > 30m               | Canary fail; rollback slow | **P:** A/B slots • **D:** Rollback drills • **C:** One-click rollback   | Rollback TTA < 30m            | MET.OPS.RELEASE_ROLLBACK_TTA  | Release    |  8 |  3 |  4 |      96 | **YEL** | `/evidence/ota/rollback_drill.md` |
| **RR-U03** | Policy/regulatory change untested | New rule; pack outdated    | **P:** Jurisdiction packs in CI • **D:** Pack lint • **C:** Hotfix path | Pack=pass; docs updated       | MET.GOV.AUDIT_BUNDLE_COMPLETE | Compliance |  8 |  3 |  5 |     120 | **YEL** | `/evidence/compliance/packs/`     |
| **RR-U04** | Dependency upgrade regression     | Lib/CUDA/driver bump       | **P:** Matrix tests • **D:** ABI checks • **C:** Pin + revert           | Matrix green across SKUs      | MET.OPS.TIME_TO_VALUE         | Eng/DevOps |  7 |  3 |  5 |     105 | **YEL** | `/evidence/ci/matrix/`            |

**Update Gate:** All **YEL tests passing**; no **RED**.

---

## Residual Risk Acceptance (strict)

* Only the **Safety Review Board** can accept residual risk, after:

  1. Evidence bundle linked in the row,
  2. Mitigation time-box assigned,
  3. Metrics & monitors configured,
  4. Stakeholders sign (Safety **A**, Compliance **A**, Product **C**, Eng **C**, Legal **C**).

## Traceability

* Every row may reference scenarios (`SCN.*`) and personas (`PERS.*`) when relevant.
* Every metric reference uses **IDs from Metrics Canon** (no formulas here).

---

### Operational Notes

* If a risk moves **RED → YEL → GREEN**, **do not delete** the row; keep history for audit.
* Add drill outcomes (pass/fail + date) to the **Evidence** column.

---