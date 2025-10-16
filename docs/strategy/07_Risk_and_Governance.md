## Purpose

Set the **rules of the game** for risk: who decides, how we score, how changes ship, and how we prove control. This file is **policy & oversight** only. The **live risk rows & phase gates live in** `./02_Risk_Register_Phase_Gated.md`.

## Single Sources of Truth (SoT)

| Topic                              | Where it lives                                        | What this file references                 |
| ---------------------------------- | ----------------------------------------------------- | ----------------------------------------- |
| Risk rows & go/no-go gates         | `./02_Risk_Register_Phase_Gated.md`                   | Section 6 (snapshot only)                 |
| KPI formulas/targets               | `../03_Metrics_Canon.md`                              | Metric IDs (e.g., MET.SAFETY.ASSIST_RATE) |
| Problem/Solution & sector overlays | `../00_Problem_Statement_and_Solution_ALL_SECTORS.md` | Links only                                |
| Personas/Scenarios                 | `../02_Personas_and_Scenarios.md`                     | Scenario IDs & personas only              |
| Compliance evidence                | Release bundles in CI                                 | References and checks only                |

## 1) Risk Taxonomy

| Category              | Examples                                                              | Primary Owners            |
| --------------------- | --------------------------------------------------------------------- | ------------------------- |
| **SAFETY / TECH**     | Perception/plan/control faults, ODD mismatch, model drift, SOTIF gaps | Safety, Eng               |
| **OPS / DISPATCH**    | Assist overload, site readiness, comms loss, incident response        | Ops                       |
| **SECURITY / DATA**   | Breach, OTA compromise, PII misuse, residency                         | Security, Data            |
| **REG / LEGAL**       | Permits, jurisdiction changes, export controls                        | Compliance, Legal         |
| **FIN / BRAND**       | SLA breach penalties, downtime cost, public incident                  | Finance, Comms            |
| **SUPPLY / PARTNERS** | Sensor EOL, vendor API drift, integrator failure                      | Procurement, Integrations |

## 2) Scoring & Gates (unified with the Register)

* **FMEA-style**: Severity (**S**) × Likelihood (**L**) × Detectability (**D**) = **RPN** (1–1000).
* **Gates**:

  * **GREEN** ≤ 80 (ship allowed)
  * **YELLOW** 81–150 (ship **only** with passing tests + time-boxed mitigation)
  * **RED** >150 (block release/site entry)
* Every risk row must include **phase(s)**, **signals**, **controls** (prevent / detect / correct), **tests**, **metric IDs**, **owner**, and **evidence path**.

## 3) Governance Bodies & Cadence

| Body                           | Members                                | Cadence          | Scope                           | Outputs                               |
| ------------------------------ | -------------------------------------- | ---------------- | ------------------------------- | ------------------------------------- |
| **Safety Review Board**        | Safety, Eng, QA, Legal                 | Monthly + ad-hoc | Safety case, residual risk      | Safety case approval; risk acceptance |
| **Change Control Board (CCB)** | Eng, Safety, QA, Ops                   | Weekly           | Release approvals, change risk  | Go/No-Go; rollback plans              |
| **Product Council**            | Product, Eng, Safety, Data, Compliance | Bi-weekly        | Roadmap, cross-risk trade-offs  | Prioritized plan; risk budget         |
| **Security Council**           | Security, Eng, Legal                   | Monthly          | Threats, vulns, OTA/PKI         | Remediation plan; attestations        |
| **Exec Risk Review**           | C-suite, VPs                           | Quarterly        | Appetite, top-10 RPN, transfers | Adjusted appetite; escalations        |

### Decision Rights (RACI, condensed)

| Decision                                                                 | Exec | Product Council | CCB   | Safety Board | Security | Ops     |
| ------------------------------------------------------------------------ | ---- | --------------- | ----- | ------------ | -------- | ------- |
| Product roadmap                                                          | A    | R               | C     | C            | C        | C       |
| Release approval                                                         | I    | A               | **R** | C            | C        | C       |
| Safety case                                                              | I    | I               | C     | **A/R**      | I        | C       |
| Security policy/OTA keys                                                 | I    | I               | C     | I            | **A/R**  | C       |
| Operational procedures                                                   | I    | C               | C     | C            | C        | **A/R** |
| *Legend: A = Accountable, R = Responsible, C = Consulted, I = Informed.* |      |                 |       |              |          |         |

## 4) Release & Change Governance (hard-wired to phase gates)

* **Design →** must pass **Register §2 (Design)**: hazards identified; architecture mitigations in place; test plans approved.
* **Pre-Deploy →** must pass **Register §3 (Pre-Deploy)**: site readiness, permits, adapter contract tests, drills.
* **Go-Live →** must pass **Register §4 (Go-Live)**: canary, OTA rollback, LE/first-responder coordination, public comms
* **Operate →** monitored under **Register §5 (Operate)**: AVAIL ODD band, assist budget, privacy controls.
* **Update →** must pass **Register §6 (Update)**: signed OTA, attestation, rollback <30m, policy deltas tested.

## 5) Compliance Framework (evidence, not prose)

| Domain            | Standard                     | What we store as evidence                     | Where                         |
| ----------------- | ---------------------------- | --------------------------------------------- | ----------------------------- |
| Functional safety | ISO 26262, ISO 21448 (SOTIF) | HARA, FMEA, V&V traces, safety case           | CI bundle `/evidence/safety/` |
| Cybersecurity     | ISO/SAE 21434, UNECE R155    | Threat model, SBOM, pentest, PKI attestations | `/evidence/security/`         |
| Software updates  | UNECE R156                   | OTA SOPs, canary, rollback proofs             | `/evidence/ota/`              |
| Data protection   | GDPR/CCPA + regionals        | DPIA, data maps, retention proofs             | `/evidence/privacy/`          |
| Export/regulatory | ITAR/EAR + local permits     | Classifications, permits, audits              | `/evidence/compliance/`       |

> Metric references use IDs from **`../03_Metrics_Canon.md`** (e.g., `MET.SAFETY.ASSIST_RATE`). No formulas or targets here.

## 6) Live Risk Snapshot (read-only)

> **Authoritative source:** `./02_Risk_Register_Phase_Gated.md`. Refresh this table from the register; **do not edit risk content here**.

| ID                 | Phase   | Risk                                      | Gate   | Owner   | RPN | Last test  |
| ------------------ | ------- | ----------------------------------------- | ------ | ------- | --: | ---------- |
| RR-G02             | Go-Live | OTA rollback fails during canary          | YELLOW | Release | 128 | 2025-09-22 |
| RR-O03             | Operate | Assist overload during weather surge      | GREEN  | Ops     |  72 | 2025-09-24 |
| RR-D05             | Design  | Single-point compute path in control loop | RED    | Eng     | 192 | 2025-09-19 |
| *(…top 10 by RPN)* |         |                                           |        |         |     |            |

## 7) Continuous Improvement & Metrics

We track governance health with **metric IDs only** (SoT: Metrics Canon):

* `MET.SAFETY.ASSIST_RATE`, `MET.RELIAB.AVAIL_ODD`, `MET.SEC.INCIDENTS_P1`, `MET.GOV.AUDIT_BUNDLE_COMPLETE`, `MET.OPS.TIME_TO_VALUE`, `MET.OPS.RELEASE_ROLLBACK_TTA`.


---
