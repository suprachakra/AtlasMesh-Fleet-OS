## 1) Cadence
- **Monthly tabletop**: top-3 exposure risks from `risk/catalog.yaml` (auto-selected).
- **Quarterly live drills** (rotate):
  - **Q1**: EW/GNSS denial (R-011) – defense overlay
  - **Q2**: OTA rollback on canary regressions (R-002)
  - **Q3**: LE stop & incident comms (R-030) – ride-hail/log overlays
  - **Q4**: HAZMAT spill & yard lockdown (R-050) – logistics overlay

Missed drill = **yellow**; repeat miss = **red** for owning team.

## 2) Playbook template
**Name:** `<risk id> – <short title>`  
**Objective:** Validate detection, response, fallback, and evidence capture within SLAs.  
**Scope:** Sites/tenants involved.  
**Injects:** Timed events (e.g., GNSS spoof; adapter schema change).  
**Success criteria:** Tripwire triggers; fallback engaged; MTTR ≤ target; evidence bundle complete.  
**Artifacts:** Logs/telemetry snapshots, timeline, RCA, fixes/PRs.

## 3) Example playbooks

### R-011 — GNSS denial (defense)
- **Inject:** RF jammer sim for 7 minutes across two corridor tiles.
- **Expect:** ODD guard alarm < 60s; SLAM fallback; convoy gap policy enforced.
- **Fallback:** ODD clamp to convoy-only; mission continues.
- **SLA:** Response ≤ 120s; `ODD_CONFORMANCE ≥ 98%`.
- **Evidence:** odd/guard logs, slam/confidence traces, commander acknowledgment.

### R-002 — OTA regression (all sectors)
- **Inject:** Canary build introduces planner regression.
- **Expect:** Twin-gate fail; staged rollback < 30m; blue/green attest OK.
- **SLA:** Detection < 15m; rollback < 30m.
- **Evidence:** CI/CD logs, attestation proofs, diff report.

### R-050 — Adapter/API drift (logistics)
- **Inject:** WMS adds required field without version bump.
- **Expect:** Contract test catches; adapter canary isolates; shim applied < 4h.
- **Evidence:** Contract test record, error-rate graph, hotfix PR.

