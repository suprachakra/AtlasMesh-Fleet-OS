# 03_OKRs_and_Metrics_Canon.md

## Company OKRs (FY-1)

**O1 — Prove repeatable L4 operations in harsh, regulated ODDs.**  
- KR1: 3 pilots to production SLAs (mine, port, defense) with ≥95% KPIs green for 60 days.  
- KR2: Availability ≥**99.0%** (30-day rolling).  
- KR3: Assist rate ≤**0.5 / 1,000 km**; zero critical incidents.  
- KR4: **6 releases** with **100%** evidence bundles; **2** regulator sign-offs.

**O2 — Demonstrate ROI and deployment velocity.**  
- KR1: Mining cost/ton −10%+, port cycle −20%+, ride-hail ETA P95 ≤7 min.  
- KR2: Site cutover ≤**14 days**; install ≤**8 h** per vehicle.  
- KR3: Close **$XXM TCV** across ≥6 logos; software gross margin ≥**55%**.

**O3 — Be credibly agnostic (vehicles, sensors, maps, clouds).**  
- KR1: **5** vehicle classes certified (haul, yard tractor, 4×4 UGV, forklift, van).  
- KR2: Support **2** lidar, **2** camera stacks, **2** GNSS/INS with hot-swap configs.  
- KR3: Run on **GCP/Azure/on-prem** with identical APIs; policy changes safe <**24 h**.

## Workstream OKRs (roll-ups)

- **Engineering/Platform** — P95 control-loop ≤**50 ms**; OTA blue/green with attestation; twin-gate pass rate ≥**95%**.  
- **Data/ML** — Data loss <**0.1%**; PdM cuts unscheduled downtime **−20%**; demand model +**10%** seat-fill / **−25%** loader idle.  
- **QA & Safety** — 0 open P1 in prod; STPA/HARA per release; quarterly drills (EW, sensor loss, LE stop).  
- **Security** — mTLS everywhere; **0** critical CVEs >**15 days**; 2× red-team/year with **0** P1 open >**30 days**.  
- **Ops** — Site readiness ≤**10 business days**; RMA <**72 h**; on-call ack P95 <**5 min**.  
- **GTM/Partnerships** — **6** logos; **3** referenceable case studies; **3** integrators enabled; time-to-first-POC <**45 days**.  

## Metrics Canon (formulas, SLI/SLO, ownership)

| Metric | Formula | SLI/SLO Target | Source of Truth | Owner | Cadence |
|---|---|---|---|---|---|
| **Availability** | Mission-capable hrs / Scheduled hrs | **SLO ≥99.0%** | Fleet telemetry | Ops | Daily |
| **Assist rate** | # assists / 1,000 km | **SLO ≤0.5** (Yr-1) | Assist logs + odometer | Safety | Daily |
| **Mission completion** | Completed missions / Dispatched missions | SLI (track ↑) | Ops DB | Sector Ops | Daily |
| **Gate→dock P95** | 95th pct dwell (gate-in→dock) | **SLO −20% vs baseline** | TOS + telemetry | Logistics | Daily |
| **Tons/hour** | Total moved tons / Operating hour | **SLO +10% vs baseline** | Production + telemetry | Mining | Daily |
| **ETA P95** | 95th pct rider wait (request→pickup) | **SLO ≤7 min** | Dispatch DB | Ride-hail | Daily |
| **CSAT** | Avg post-trip rating (1–5) | **SLO ≥4.8** | App feedback | Ride-hail | Weekly |
| **PdM lift** | (Unscheduled downtime_baseline − current) / baseline | SLI (track ↑) | Maintenance system | Data/ML | Monthly |
| **Evidence completeness** | Present artifacts / Required artifacts | **SLO 100%** | CI pipeline | Compliance | Per release |
| **Deployment time** | Contract-signed → First mission | **SLO ≤8 weeks** | Program tracker | PMO | Weekly |
| **Install time** | Vehicle start → Ready for mission | **SLO ≤8 h** | Field ops logs | Ops | Weekly |
| **Adapter health** | Contract-test pass rate | **SLO 100% pre-deploy** | CI + adapter tests | Integrations | Per build |
| **Security posture** | # critical CVEs open >15 days | **SLO 0** | Vuln mgmt | Security | Weekly |
| **Rollback time** | Detection → Stable rollback | **SLO <30 min** | CI/CD + alerts | Release Eng | Per incident |
| **Cost/ton** | (Fuel+Energy+Tires+Maint+Labor)/Tons | SLI (−10–15% vs baseline) | ERP + telemetry | Mining | Monthly |
| **Energy $/km** | Energy cost / km | SLI (track ↓) | Telematics + tariffs | Ops/Data | Weekly |
| **ODD conformance** | Time in-ODD / Total runtime | **SLO ≥98%** | ODD guard logs | Safety | Daily |
| **Audit cycle time** | Release date → Regulator acceptance | SLI (track ↓) | Compliance tracker | Compliance | Monthly |

> **Notes**  
> • SLI = service-level indicator (we track trend); SLO = target we commit to.  
> • All metrics must exist in `data/contracts/kpis.yaml`; CI blocks any unmapped metric references.
