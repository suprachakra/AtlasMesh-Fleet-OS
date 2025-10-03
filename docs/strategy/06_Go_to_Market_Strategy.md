# AtlasMesh Fleet OS — Go-to-Market Strategy

**Scope:** Sales/partner motions, **OEM blueprint**, **transport-authority blueprint**, pricing **execution**, KPIs, governance. *(Product/market truths live in 04.)*

### 0) Targets (TAM/SAM/SOM summary)

TAM: Global autonomous fleet mgmt ~$45B by 2030 • Primary SAM (L4/retrofit/regulated) ~$7B • SOM: Y1 $15M → Y3 $150M → Y5 $500M.

### 1) Phased market entry (ME → APAC → EU/US)

* **Phase 1 (0–6m)**: 1 defense base, 1 open-pit mine, 1 port, 1 city ops room. *Exit:* ≥3 LOIs; metrics ≥90%.
* **Phase 2 (6–18m)**: Replicate across GCC; adapter marketplace v1 (≥10 certified). *Exit:* ≥12 paying sites; evidence pass ≥95%.
* **Phase 3 (18–36m)**: SI/reseller program; multi-region; Evidence Cloud read-only for regulators. *Exit:* <1% critical policy violations/qtr.

### 2) OEM partnership blueprint (realistic)

**We integrate FMS; we don’t touch the AV brain.**

| Stage |                        6–8w |                                                3–6m |                                        6–12m |                                                        12–24m |                  Ongoing |
| ----- | --------------------------: | --------------------------------------------------: | -------------------------------------------: | ------------------------------------------------------------: | -----------------------: |
| Name  |                     Explore |                                  Evaluate (lab/rig) |                      Pilot-of-Record (fleet) |                                      Production Program (SOP) |                Scale-out |
| Gates | Joint Tech Plan; lab access | ≥95% contract-tests pass; 21434 threat model signed | R30D: AVAIL ≥98.5%; bundle ≥95%; drills done | 60D all north-stars green; OTA signing + PPAP; warranty terms | ≥2 regions live; NPS ≥50 |

**Engineering checkpoints:** E/E (CAN/LIN/ETH); OTA signing & **rollback <5m**; line-end/EOL tests; field diag + shared RCA; UN R155/156 campaign path; versioned data contracts + golden playback.

**Commercial terms:** Program **NRE** + **per-vehicle royalty** + **support annuity**; foreground IP by contribution; co-marketing only after **bundle 100%** + 30D green.

**Risks→Mitigations:** integration drift→conformance matrix; warranty ping-pong→trace escrow; SOP slip→golden sample bench & PPAP burndown; lock-in→dual-vendor BOM, perf-parity tests.

### 3) Transport authority blueprint (Jumeirah-1 → city; Punggol → island)

| Phase | P0                      | P1                                          | P2                                                  | P3                                          | P4                                      | P5                          |
| ----- | ----------------------- | ------------------------------------------- | --------------------------------------------------- | ------------------------------------------- | --------------------------------------- | --------------------------- |
| Scope | Orientation             | Closed-course                               | **Micro-ODD** VIP                                   | **Community (free 8–12 wks)**               | **Paid corridor**                       | Citywide                    |
| Users | Reviewers               | Ops staff                                   | VIP cohort                                          | Public (free)                               | Paying riders                           | Paying riders               |
| Key   | Safety case, comms plan | Local drills (LE stop, sensor loss, EW/jam) | Soft launch; ops stability                          | Awareness + load test; accessibility audits | Fares; complaint mgmt                   | Multi-modal, priority lanes |
| Exit  | Reviewers assigned      | All test IDs pass                           | R30D: AVAIL ≥98.5%; assists ≤0.5/1k km; bundle ≥95% | Safety deltas stable; complaint SLA         | 60D north-stars green; LE validation ×2 | Renewal gates pre-agreed    |

**Auto-brakes:** assist spikes; incident clusters; weather deltas; complaint surge ⇒ freeze, publish update, drill, retry after green.

### 4) Sales motions & enablement

* **Enterprise direct** (100+ vehicles) • **SI/channel** (20–100).
* **Process:** inbound/outbound/partner → BANT + tech fit → discovery → solution/ROI → PoC/Pilot → production.
* **Assets:** Salesforce; ROI calculators; demo env; safety pre-read; regulator playbooks; adapter marketplace catalog.

### 5) Pricing execution (quote waterfall)

1. Count **MVEs** → choose **Sector Overlay**(s).
2. Add **Evidence+Gov** (regulated) + **Residency** (if required).
3. SLA tier (Std / Premium 99.95% + 24/7 SRE).
4. Apply fences (term/volume/multi-SKU/reference).
5. Add **contract frame** (Pilot Pack / Production / OEM / Authority).
6. Output priced BOM + **change-control annex**.
   **Indexation:** cloud index + CPI. **Currency:** local; residency uplift as needed.

### 6) Marketing & brand (evidence-led)

* **Position:** “The world’s most agnostic autonomous **Fleet OS**.”
* **Cadence:** corridor scorecards; monthly safety/evidence posts; regulator-approved case studies; LE/first-responder demos.
* **Channels:** events (ADIPEC/MINExpo/SeaTrade), SG/ME authority workshops, technical whitepapers (offline resilience, evidence CI).

### 7) Customer success & ops governance

* **Onboarding:** deployment → training → acceptance tests (tied to safety bundles).
* **Run:** weekly Change Control Board; monthly regulator review; quarterly drills (EW/jam/sensor loss/LE stop/HAZMAT).
* **Rollback:** signed blue/green OTA; **<5m** target; SemVer + 90-day deprecations.

### 8) KPIs (always-on)

* **Ops/Safety:** assists/1k km; incidents; triage ≤60s P95; availability.
* **Biz:** pilot→prod conversion; ARR; TCV by wave; win-rate.
* **Eng:** CI twin pass rate; perf P95; adapter fit.
* **Security/Data:** SBOM signing; P1 open days; provenance ≥0.95; loss ≤0.1%.
* **Trust:** complaint SLA; sentiment; report cadence.

---