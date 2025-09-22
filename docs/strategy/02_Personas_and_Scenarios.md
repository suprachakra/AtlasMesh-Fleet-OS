# 02_Personas_and_Scenarios.md

# Personas & Scenarios

## Personas (decision-first, KPI-tied)
| Persona | Sector(s) | Core JTBD | Top Decisions (Daily) | Success Metrics They Care About | Critical Tools & Handoffs |
|---|---|---|---|---|---|
| **Mission Commander** | Defense | Complete missions within ROE under GNSS/comms stress | Route approvals; convoy spacing; assist escalation | Mission completion %, safe-stops/10k km, assist response time | Ops Room; ROE policy panel; incident playbooks |
| **Pit Dispatcher** | Mining | Maximize tons/hour safely | Assignments; queue balancing; reroutes for derating | Tons/hr, availability %, cost/ton | Control Center; PdM tickets; loader/hauler sync |
| **Yard/Port Supervisor** | Logistics | Hit OTIF & reduce crane idle | Gate sequencing; lane closures; crane windows | Gate→dock P95, crane idle, mis-spots | TOS adapter; yard map; exception budget |
| **Ops Dispatcher** | Ride-hail | Keep ETA low & CSAT high | Rebalancing; incident triage; accessibility flows | ETA P95, CSAT, cancel rate | Dispatch console; rider support tools |
| **Depot/Mine Technician** | All | Keep vehicles mission-capable | Firmware staging; diagnostics; part swaps | MTTR, RMA SLA, % staged successfully | Garage PC; OTA; SBOM |
| **Safety/Compliance Officer** | All | Approve releases & audits | Evidence review; policy changes; permit renewals | Audit completeness %, approval cycle time | Evidence bundles; jurisdiction packs |
| **CISO/SecOps** | All | Maintain cyber posture | Key mgmt; OTA attestation; incident response | Critical CVEs ≤15 days; zero P1 findings | CSMS; SIEM; OTA attest |

## Scenarios (Given / When / Then, with outcome metrics)
> Links reference FR/NFR IDs in your requirements repo; adjust IDs as needed.

| ID | Scenario (G/W/T) | Outcome Metric(s) | Links |
|---|---|---|---|
| **S-DEF-01** | **Given** a GNSS-degraded corridor and EW alert, **when** a convoy enters and lead-vehicle confidence drops below X, **then** switch to SLAM+leader-follower and notify commander. | Mission completion ≥98%; ≤1 safe-stop/10k km | FR-NAV-07; NFR-RES-02 |
| **S-DEF-02** | **Given** SAT latency spikes, **when** tele-assist is requested, **then** enforce policy budget and return guidance ≤2 s P95. | Assist P95 ≤2 s; assists ≤0.5/1k km | FR-ASSIST-03; NFR-SLA-01 |
| **S-MIN-01** | **Given** ambient ≥50 °C, **when** compute thermal hits threshold, **then** derate route & speed and schedule cool-down. | Availability ≥99.5%; tons/hr slope stabilized | FR-ENER-02; NFR-THERM-01 |
| **S-MIN-02** | **Given** dust occlusion, **when** perception confidence <80%, **then** engage multi-sensor fusion + wiper/air-knife and use degraded-mode pathing. | ≤0.3 assists/1k km during dust events | FR-PER-05; NFR-ENV-02 |
| **S-LOG-01** | **Given** gate congestion, **when** predicted queue > threshold, **then** resequence tractors and reserve crane windows. | Gate→dock P95 −20%; mis-spots ≈0 | FR-YARD-04; NFR-LAT-01 |
| **S-LOG-02** | **Given** a TOS API version change, **when** contract tests fail, **then** pin adapter version & auto-fallback to shim. | Zero aborted moves; MTTF-adapter drift >90 days | FR-ADPT-06; NFR-CONTRACT-01 |
| **S-RH-01** | **Given** surge in district A, **when** ETA P95 > target, **then** rebalance vehicles and throttle low-value trips. | ETA P95 ≤7 min; CSAT ≥4.8 | FR-DISP-03; NFR-QOS-01 |
| **S-ALL-SEC-01** | **Given** OTA rollout, **when** attestation fails, **then** auto-rollback and quarantine workload. | Rollback <30 min; 0 P1 security incidents | FR-OTA-05; NFR-SEC-02 |
| **S-AUD-01** | **Given** a release train, **when** safety-bundle completeness <100%, **then** block release. | Audit completeness 100% | FR-EVID-01; NFR-GATE-01 |
