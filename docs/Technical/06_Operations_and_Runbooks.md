<div align="center">

# 🚀 AtlasMesh Fleet OS — Operations and Runbooks

**Comprehensive Operations Guide for On-Call Engineers and SREs**

</div>

---

## 📋 Table of Contents

<div align="center">

| 🚨 **[On-Call Overview](#1-oncall-overview)** | 🌍 **[Environments](#2-environments)** | 📊 **[SLOs and Alerts](#3-slos-and-alerts)** | 🔧 **[Runbooks](#4-runbooks)** |
|:---:|:---:|:---:|:---:|
| **Pager Rotation & Escalation** | **Environment Management** | **Service Level Objectives** | **Operational Procedures** |

| 📈 **[Monitoring](#5-monitoring)** | 🚨 **[Incident Response](#6-incident-response)** | 📚 **[References](#7-references--related-docs)** |
|:---:|:---:|:---:|
| **Observability & Dashboards** | **Incident Management Process** | **Supporting Documentation** |

</div>

---

## 🚨 **1) On‑Call Overview**

<div align="center">

| 🔄 **Primary Pager Rotation** | 🚨 **Escalation Matrix** | 📊 **Key Dashboards** | 💬 **Chat Channels** | ⏰ **Change Freeze Windows** |
|:---:|:---:|:---:|:---:|:---:|
| Backend → Edge → Simulation → Security | Sev-1 to Incident Commander; Sev-2 to Workstream Leads | Fleet Health, API Latency, Map Service, OTA, Comms, Simulation CI | #oncall-backend, #oncall-edge, #oncall-security, #incident-room | Customer local peak hours unless emergency fix |

</div>

## 2) Environments

| Env | Purpose | Data | Controls |
| --- | --- | --- | --- |
| Dev | Developer testing | Synthetic | Low blast radius, feature flags |
| Staging | Pre-prod validation | Sanitized | Full gates, shadow traffic |
| Prod (Cloud) | Fleet management | Operational | SLOs, change control, evidence |
| Prod (Edge) | On-vehicle | Operational | Secure boot, mTLS, OTA controls |

## 3) SLOs and Alerts

| Service | SLI | SLO | Alert | Page |
| --- | --- | --- | --- | --- |
| API | P95 latency | ≤ 300 ms | 10m burn > 2x | Backend |
| Assist | RTT p95 | ≤ 90 s | 15m > 90 s | Backend |
| Map | Conflict MTTResolve | ≤ 24 h site/48 h city | Breach forecast | Mapping |
| Vehicle | Availability | ≥ 99.0% | 15m < target | Edge |
| OTA | Rollout success | ≥ 99.5% | Any wave < 98% | Edge |
| CI Twin | Suite duration | ≤ 4 h | +20% over baseline | Simulation |
| Security | Critical vulns open | 0 > 14 days | Any breach | Security |

## 4) Standard Operating Procedures

### 4.1 Change Management (Prod)

1. Create change record with risk, rollback plan, owner
2. Verify green in Staging (tests + twin gates)
3. Announce window in #oncall channels
4. Execute canary → 25% → 100%; monitor SLOs
5. Attach evidence to release tag

### 4.2 Evidence Attachment

- Generate safety/compliance bundle and attach to the Git tag
- Include: SBOM, test matrices, twin results, map provenance, OTA attestations

## 5) Incident Response

### 5.1 Severity and Roles

| Sev | Impact | Examples | Roles |
| --- | --- | --- | --- |
| Sev‑1 | Safety/major outage | Widespread vehicle stop, data breach | IC, Backend, Edge, Security |
| Sev‑2 | Degraded service | Regional comms blackout, map issue | IC, Owning team |
| Sev‑3 | Minor | Single site latency spike | Owning team |

### 5.2 Timeline Expectations

- Ack: ≤ 5 min (Sev‑1), ≤ 15 min (Sev‑2)
- Mitigate: ≤ 30 min (Sev‑1), ≤ 2 h (Sev‑2)
- Comms: Status updates every 15 min (Sev‑1), 30 min (Sev‑2)

## 6) Runbooks (Critical Scenarios)

### 6.1 P95 API latency > SLO

1. Check dashboard: API latency, error rate, saturation
2. Identify hot endpoints via tracing
3. Mitigate: scale API and dependencies; enable read caches
4. Roll back last change if deployed within 60 min
5. Post‑incident: add load tests; update budgets

Commands:
```bash
kubectl -n atlasmesh top pods | sort -k3 -rh | head -10
kubectl -n atlasmesh rollout undo deploy/fleet-api
```

### 6.2 Vehicle offline or degraded (Edge)

1. Confirm last heartbeat in Fleet Health
2. Check comms path: LTE/5G, Wi‑Fi, SATCOM; verify mTLS cert validity
3. If offline: switch to store‑and‑forward; confirm local autonomy active
4. Retrieve on-vehicle logs when link restored; verify ROS2 node health
5. If persistent: safe stop and schedule maintenance

Checklist:
- ROS2 critical nodes running; CPU < 70%, memory < 80%
- Local map/version consistent with trip
- Assist requests triaged within policy latency

### 6.3 Comms blackout across a site

1. Identify scope via link health dashboard
2. Enforce offline‑first: reduce telemetry, prioritize safety events
3. Activate mesh or SATCOM fallback within policy cost caps
4. Communicate to site ops; freeze risky missions
5. After recovery: reconcile data, check evidence uploads

### 6.4 Map conflict or bad update detected

1. Autotriggered: conflict detector raised severity
2. Pin affected area to last‑known‑good version
3. Route through safe corridors only; enable conservative planning
4. Start conflict resolution workflow: verify sources, HIL replay
5. Close when validation passes; attach provenance record

### 6.5 Simulation CI twin gates failing

1. Inspect failing scenarios; categorize by capability/sector
2. Verify simulator health and runner pool capacity
3. Re‑run narrowed suite; bisect most recent changes
4. Block merge; assign owners per capability
5. After fix: expand to full matrix; record regression case

### 6.6 OTA rollout issues (edge)

1. Halt further waves; keep canaries only
2. Verify signature and integrity; check secure boot logs
3. Roll back via dual partition to N‑1
4. Collect logs; open RCA item and attach to release
5. Resume rollout with smaller waves after fix

### 6.7 GNSS denial or spoofing suspected

1. Alert comes from localization anomaly detection
2. Switch to SLAM/VIO + inertial; cap speed; restrict area
3. Trigger safe harbor protocol; notify ops
4. Mark affected time windows for evidence
5. Validate post‑event; recalibrate if needed

### 6.8 Security incident (credential leakage, intrusion)

1. Declare incident (Sev‑1 if safety impact)
2. Rotate impacted credentials; revoke tokens, rotate mTLS certs
3. Isolate affected workloads; enable higher logging level
4. Forensics: snapshot disks, export audit logs
5. Coordinate notifications; begin post‑mortem within 24 h

### 6.9 Degradation mode activation (graceful)

1. Detect trigger: sensor health, compute pressure, policy uncertainty
2. Enter predefined degradation level; enforce speed/feature caps
3. Surface operator banners; log variant and rationale
4. Attempt recovery criteria; step‑wise restoration
5. Verify via tests; record RTO/RPO metrics

## 7) Operational Checklists

### 7.1 Pre‑release

- All CI twin suites green; performance within ±10% of baseline
- Evidence bundle attached; SBOM signed
- Rollback plan validated; canary dry‑run passed

### 7.2 Site bring‑up

- Comms paths validated; cost caps enforced
- Map snapshot loaded and validated; provenance recorded
- Vehicle profiles loaded; brake/stability tests completed

### 7.3 Daily health

- Fleet availability ≥ SLO; assists within targets
- No untriaged P1/P2 alerts; evidence uploads < 2 h delay

## 8) Playbooks (Quick Blocks)

> Incident: P95 latency > SLO
>
> - Check dashboard: API Latency
> - Scale or rollback
> - Escalate: @oncall-backend

> Incident: Evidence upload backlog
>
> - Check object store latency and quotas
> - Throttle non-critical telemetry exports
> - Escalate: @oncall-backend

## 9) Tooling and Access

| Tool | Purpose | Notes |
| --- | --- | --- |
| kubectl + contexts | Cluster ops | Read-only context for observers |
| Grafana | Dashboards | On-call folders pinned |
| Loki/ELK | Logs | Retention ≥ 30 days |
| Tracing | APM | Critical paths instrumented |
| On-vehicle shell | Diagnostics | Break-glass with approvals |

## 10) Post‑Incident Review

1. Timeline, root cause, contributing factors
2. What detection missed; alert improvements
3. Preventative actions; owners and deadlines
4. Update runbooks and tests; link to ADRs if architectural

## 11) Fleet Management Protocols

### 11.1 Multi-Fleet Coordination Protocols

#### Fleet Emergency Protocols
1. **Cross-Fleet Resource Sharing**
   - Monitor resource utilization across all fleets
   - Enable automatic resource reallocation during emergencies
   - Maintain fleet isolation boundaries
   - Escalate to Fleet Lead if coordination fails

2. **Fleet Federation Management**
   - Verify multi-tenant isolation is maintained
   - Monitor cross-fleet communication channels
   - Ensure data sovereignty compliance
   - Alert on unauthorized cross-fleet access

#### Fleet Communication Protocols
1. **Inter-Fleet Communication**
   - Monitor V2V and V2I communication health
   - Verify message integrity and authentication
   - Handle communication failures gracefully
   - Maintain communication logs for audit

2. **Fleet Coordination Standards**
   - Enforce coordination protocols across all fleets
   - Monitor protocol compliance
   - Handle protocol violations
   - Update protocols based on operational learnings

### 11.2 Mission Management Protocols

#### Mission Safety Protocols
1. **Mission Execution Safety**
   - Verify mission dependencies are resolved
   - Monitor mission execution progress
   - Handle mission failures and rollbacks
   - Maintain mission audit trails

2. **Mission Coordination**
   - Ensure mission templates are valid
   - Monitor mission scheduling and execution
   - Handle mission conflicts and priorities
   - Coordinate mission resources

### 11.3 Fleet Optimization Protocols

#### Fleet Performance Monitoring
1. **Optimization Engine Health**
   - Monitor optimization algorithm performance
   - Verify optimization results are valid
   - Handle optimization failures
   - Maintain optimization audit trails

2. **Fleet Rebalancing**
   - Monitor fleet utilization across all resources
   - Execute dynamic rebalancing when needed
   - Handle rebalancing conflicts
   - Maintain rebalancing logs

### 11.4 Fleet Analytics Protocols

#### Fleet Health Monitoring
1. **Health Score Calculation**
   - Monitor fleet health metrics continuously
   - Calculate health scores using validated algorithms
   - Alert on health score degradation
   - Maintain health score history

2. **Predictive Analytics**
   - Monitor predictive model performance
   - Verify prediction accuracy
   - Handle prediction failures
   - Update models based on new data

### 11.5 Fleet Resource Management Protocols

#### Resource Allocation
1. **Resource Optimization**
   - Monitor resource allocation efficiency
   - Execute resource optimization algorithms
   - Handle resource conflicts
   - Maintain resource allocation logs

2. **Resource Planning**
   - Monitor resource utilization trends
   - Plan resource requirements
   - Handle resource shortages
   - Maintain resource planning records

### 11.6 Fleet Performance Management Protocols

#### Performance Monitoring
1. **Performance Metrics Collection**
   - Monitor fleet performance metrics
   - Calculate performance scores
   - Alert on performance degradation
   - Maintain performance history

2. **Performance Benchmarking**
   - Compare fleet performance against benchmarks
   - Identify performance improvement opportunities
   - Handle benchmark failures
   - Maintain benchmark records

## 12) References

- Requirements: `docs/Technical/03_Requirements_FRs_NFRs.md`
- Security: `docs/Technical/05_Security_and_Compliance.md`
- Agnostics Matrix: `docs/strategy/appendices/Agonistics_Decision_Matrix_v1.0.md`
- ADRs index: `docs/ADR/README.md`



| Tool | Purpose | Notes |
| --- | --- | --- |
| kubectl + contexts | Cluster ops | Read-only context for observers |
| Grafana | Dashboards | On-call folders pinned |
| Loki/ELK | Logs | Retention ≥ 30 days |
| Tracing | APM | Critical paths instrumented |
| On-vehicle shell | Diagnostics | Break-glass with approvals |

## 10) Post‑Incident Review

1. Timeline, root cause, contributing factors
2. What detection missed; alert improvements
3. Preventative actions; owners and deadlines
4. Update runbooks and tests; link to ADRs if architectural

## 11) References

- Requirements: `docs/Technical/03_Requirements_FRs_NFRs.md`
- Security: `docs/Technical/05_Security_and_Compliance.md`
- Agnostics Matrix: `docs/strategy/appendices/Agonistics_Decision_Matrix_v1.0.md`
- ADRs index: `docs/ADR/README.md`


