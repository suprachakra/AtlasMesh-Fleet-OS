## 1) Business Model (what we monetize, how cash moves)

**We monetize the Fleet OS only.** No ride/freight ops revenue. No autonomous driving “brain.”

**Revenue streams**

1. **Subscriptions (ARR):** **MVE/month** for FMS-Core + overlays (Health+Predict, Evidence+Gov, Residency, Sector & Jurisdiction packs, Premium SLA).
2. **OEM Programs:** **NRE** (milestone-billed, SOP-aligned) + **per-vehicle royalty** (VIN activation or SOP month) + program support annuity.
3. **Authority / Corridor Licenses:** Annual corridor licenses tied to corridor MVEs; Evidence+Gov included; Residency uplift where required.
4. **Bounded Services:** Implementation, adapter certification, training (fixed-fee, margin-capped).
5. **Optional hardware (partner pass-through):** When customers insist. **Excluded from ARR/NRR**; reported separately; margin target 0–10%.

**Primary value metric:** **Managed Vehicle-Equivalent (MVE)** per month (normalizes vehicle class, AV/human mix, duty cycle).
**Discount fences:** only **term, volume, multi-SKU, reference rights**. **No** price erosion on **Evidence+Gov** below floor.

---

## 2) Pricing (numbers here; feature prose lives in 04)

**List price bands (USD; local mirrors via FX indexation)**

| SKU / Overlay         | List (per MVE / mo) |
| --------------------- | ------------------: |
| **FMS-Core**          |             600–850 |
| **Health+Predict**    |            +150–220 |
| **Evidence+Gov**      |            +250–350 |
| **Residency**         |            +300–450 |
| **Sector overlay**    |             +90–140 |
| **Jurisdiction pack** |              +30–60 |
| **Premium SLA**       |             +80–120 |

**OEM economics**

| Component                      | Typical                      |
| ------------------------------ | ---------------------------- |
| **NRE (per program)**          | $1.0–3.0M (milestone-billed) |
| **Royalty (per vehicle / yr)** | $600–1,200                   |
| **Program support (annual)**   | $150–400k                    |

**Authority / corridor**

| Component                 | Typical                      |
| ------------------------- | ---------------------------- |
| Corridor license (annual) | $250–750k                    |
| Residency uplift          | +15–25%                      |
| Public portal/reporting   | $50–150k (one-time + maint.) |

**Hardware (optional, partner pass-through):** quoted separately; **not counted** in ARR/NRR; GM goal 0–10%; no inventory exposure.

---

## 3) Revenue recognition & controls (ASC606, no surprises)

* **Subscriptions/overlays/SLA:** ratable from **Production Cutover Date** (signed acceptance).
* **OEM NRE:** %-of-completion by **objective milestones** (lab integration, PPAP set pass, EOL validation).
* **OEM royalty:** monthly on VIN activation/SOP; usage-independent.
* **Authority corridor:** annual, ratable; public portal/reporting upon delivery.
* **Pilot success credits:** booked as **contra-revenue** on first production invoice.
* **Deal desk** enforces fences, floors, indexation; any exception pre-approved.

---

## 4) Unit economics (steady-state targets)

**COGS per MVE**

| Element                | FMS-Core | Health | Evidence | Residency |
| ---------------------- | -------: | -----: | -------: | --------: |
| Cloud compute+storage  |       60 |     25 |       35 |        40 |
| Telemetry/messaging    |       20 |      5 |        5 |         7 |
| Maps/weather/licensing |       18 |      0 |        6 |         2 |
| Security/attest/SBOM   |        8 |      4 |        6 |         6 |
| SRE on-call/tooling    |       12 |      6 |        8 |        10 |
| **COGS subtotal**      |  **118** | **40** |   **60** |    **65** |

**GM (mid-band prices):** FMS-Core ~80–86%; Health ~73–79%; Evidence ~74–79%; Residency ~74–78%.
**Services:** margin cap 20–35%.
**Cloud guardrail:** renegotiate or pass-through if **cloud/egress per MVE** > +12% QoQ.

---

## 5) Funnels & cohorts (Pilot→Prod; VIP→Free→Paid)

| Segment           | Pilot→Prod | VIP→Free | Free→Paid | 12-mo Expansion (MVEs) |
| ----------------- | ---------: | -------: | --------: | ---------------------: |
| Defense           |     65–75% |      n/a |       n/a |                +25–35% |
| Mining            |     55–65% |      n/a |       n/a |                +20–30% |
| Ports & Logistics |     50–60% |   60–70% |    45–55% |                +25–35% |
| Ride-hail/Urban   |     45–55% |   70–80% |    35–45% |                +30–40% |

**GRR ≥ 92%**, **NRR 120–135%** via overlays (Evidence/Residency/Jurisdiction).
**Auto-brake:** any **safety/security red** shifts corridor bookings out 1–2 quarters automatically.

---

## 6) GTM capacity model (linked to 05)

| Metric                        |       Y1 |        Y2 |        Y3 |
| ----------------------------- | -------: | --------: | --------: |
| AEs (ramped quota $)          | 6 (2.2M) | 12 (2.5M) | 20 (2.8M) |
| Pipeline coverage             |     3.0× |      2.8× |      2.6× |
| Partner-sourced bookings      |      20% |       30% |       35% |
| OEM programs in flight        |      2–3 |       4–6 |       6–8 |
| Active corridors / city (avg) |        2 |         5 |         8 |

---

## 7) Financials (5-yr, Base case)

**Assumptions:** Avg price/MVE $880 blended; attach: Health 55%, Evidence 70%, Residency 35%, Sector 80%, Jurisdiction 60%. MVEs: 0 → 1,200 (Y1e) → 6,500 (Y3e) → 14,000 (Y5e). OEM: 2 (Y1), 5 (Y3), 9 (Y5). Corridors: 3 (Y1), 12 (Y3), 25 (Y5).

**Revenue ($M)**

|                       |       Y1 |       Y2 |        Y3 |        Y4 |        Y5 |
| --------------------- | -------: | -------: | --------: | --------: | --------: |
| Subscriptions         |      8.6 |     26.5 |      74.2 |     138.0 |     225.0 |
| Overlays              |      5.1 |     16.2 |      47.8 |      93.5 |     156.3 |
| OEM NRE               |      3.0 |      5.0 |       7.5 |       8.0 |       8.5 |
| OEM Royalties         |      0.6 |      2.0 |       5.5 |      10.5 |      17.5 |
| Authority / Corridors |      2.2 |      6.0 |      12.5 |      20.0 |      30.0 |
| Services              |      1.4 |      2.5 |       4.0 |       5.0 |       6.0 |
| **Total**             | **20.9** | **58.2** | **151.5** | **275.0** | **443.3** |

**Gross Profit / Opex / EBITDA ($M)**

|                  |       Y1 |       Y2 |        Y3 |        Y4 |        Y5 |
| ---------------- | -------: | -------: | --------: | --------: | --------: |
| COGS             |      4.3 |     12.3 |      32.6 |      57.9 |      92.8 |
| **Gross Profit** | **16.6** | **45.9** | **118.9** | **217.1** | **350.5** |
| R&D              |      9.0 |     13.0 |      18.0 |      23.0 |      28.0 |
| S&M              |      8.5 |     14.5 |      25.0 |      34.0 |      45.0 |
| G&A              |      4.0 |      5.5 |       7.5 |       9.5 |      12.0 |
| **EBITDA**       | **-4.9** | **12.9** |  **68.4** | **150.6** | **265.5** |

**Notes:** Blended GM ~79%; EBITDA positive **Y2**; OCF positive **Y3** (with corridor pre-bills).

**Side-by-side (why we changed mix)**

| Mix assumption     |     Y1 Rev |    Y1 GM | Risk                                     |
| ------------------ | ---------: | -------: | ---------------------------------------- |
| Old (30% hardware) |    $10–12M |  ~60–65% | Inventory, COGS volatility, cash tied up |
| New (asset-light)  | **$20.9M** | **~79%** | Lower rev “optics,” higher quality ARR   |

---

## 8) Scenarios & sensitivities

| Driver            | Conservative | Upside              | Y3 EBITDA impact |
| ----------------- | ------------ | ------------------- | ---------------: |
| Avg price/MVE     | -10%         | +8%                 |    -$16M / +$13M |
| Evidence attach   | -15 pp       | +10 pp              |      -$9M / +$6M |
| Corridor velocity | -30%         | +25%                |     -$10M / +$8M |
| OEM SOP slippage  | +2 Q         | 0 slippage, +1 prog |      -$6M / +$7M |
| Cloud/MVE         | +15%         | -10%                |      -$5M / +$3M |

---

## 9) Capital plan & hiring gates

* **Use of funds (24m):** R&D 40% • GTM 35% • Certs/Drills 10% • WC/contingency 15%.
* **Gates:**

  * Gate A: ARR ≥ $12M **and** safety/security greens → grow AEs 6→10.
  * Gate B: ARR ≥ $35M, GRR ≥ 92% → CS/Support 1:70→1:60 MVE.
  * Gate C: OEM royalties RR ≥ $3M → add HW/validation pod.

**Funding plan (aligned to gates):**

* **Now–Q4’25:** existing runway + **corridor pre-bills** + OEM **NRE**.
* **Series B (Q4’25 target):** **$35M** to accelerate APAC + OEM programs; trigger = **$25M ARR**, 3 authority corridors live, 2 OEM Pilots-of-Record.
* **Use of B:** 40% product, 30% GTM, 20% ops, 10% WC.

---

## 10) Financial risks → mitigations

| Risk                 | Impact             | Mitigation (baked in)                                              |
| -------------------- | ------------------ | ------------------------------------------------------------------ |
| Discount creep       | GM compression     | Deal desk fences; Evidence floors; exec approvals                  |
| Cloud/egress spikes  | COGS ↑             | Dual-cloud price books; indexation pass-through; storage lifecycle |
| OEM SOP slippage     | NRE/royalty timing | Milestone billing; buffer factor 1.3× programs                     |
| Corridor freezes     | Bookings deferral  | Auto-brake; reserve corridors; public trust buffer                 |
| Logo concentration   | Volatility         | Cap any logo <20% ARR; 60/40 sector mix                            |
| FX                   | Rev variability    | Local currency contracts + FX indexation                           |
| Collections (public) | Cash lag           | Annual pre-bills; DSO ladder; milestone-tied invoices              |
| Regulatory shifts    | Launch delays      | Jurisdiction Packs; shadow modes; sector diversification           |

---

## 11) KPIs & governance (single source of truth)

* **ARR / NRR / GRR**, MVE expansion, attach rates by overlay.
* **Per-MVE COGS**, cloud per MVE, SRE toil, SBOM signing rate.
* **Rev-rec checklists** (by stream) must pass before close.
* **Auto-pause:** safety/security **red** updates the bookings sheet and halts corridor-linked claims.

---

## 12) Appendices

* Price card (by region, with fences & floors)
* Contract economics (OEM, Authority, Production)
* Cohort ledger schema (Bookings→Billings→Rev→Cash)
* Driver dictionary (owners, ranges, sources)
* Hardware policy (partner pass-through, no inventory, ARR exclusion)

---