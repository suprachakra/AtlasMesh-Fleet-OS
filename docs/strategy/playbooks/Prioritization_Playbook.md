# Prioritization Playbook

**Purpose**: Standard framework for prioritizing product work  
**Owner**: PM CoP  
**Last Updated**: 2025-10-04

---

## Prioritization Framework

### Scoring Formula

```
Priority Score = RICE × Safety Multiplier × Variant Cost Modifier × Confidence
```

Where:
- **RICE** = (Reach × Impact × Confidence) / Effort
- **Safety Multiplier** = 1.0 (none), 1.5 (low), 2.0 (medium), 3.0 (high), 5.0 (critical)
- **Variant Cost Modifier** = 1.0 - (estimated_variant_delta / 100)
- **Confidence** = 0.5 (low), 0.7 (medium), 0.9 (high), 1.0 (validated)

---

## RICE Components

### Reach
**How many users/vehicles/sites affected per quarter?**
- **Massive** (10): All vehicles/users across all sectors
- **High** (7): Majority of vehicles/users in 2+ sectors
- **Medium** (4): Specific sector or vehicle class
- **Low** (1): Single site or niche use case

### Impact
**How much does this move the needle on OKRs?**
- **Massive** (3): Directly moves multiple KRs by ≥20%
- **High** (2): Moves 1-2 KRs by ≥10%
- **Medium** (1): Moves KRs by 5-10%
- **Low** (0.5): Indirect impact or <5% movement
- **Minimal** (0.25): Nice-to-have, no clear KR impact

### Confidence
**How confident are we in Reach/Impact estimates?**
- **High** (100%): Strong evidence from telemetry, interviews, and sim
- **Medium** (80%): Some evidence, reasonable assumptions
- **Low** (50%): Mostly assumptions, limited evidence

### Effort
**Person-months of work (all teams)**
- Include: Design, Eng, QA, Data, Safety reviews
- Consider: Complexity, risk, dependencies, learning curve

---

## Safety Multiplier

| Safety Impact | Multiplier | Criteria |
|---------------|------------|----------|
| **Critical** | 5.0x | Directly affects autonomous vehicle control, emergency systems |
| **High** | 3.0x | Affects policy evaluation, ODD boundaries, safety monitoring |
| **Medium** | 2.0x | Affects telemetry, evidence generation, compliance |
| **Low** | 1.5x | Affects UI, reporting, analytics (non-safety-critical) |
| **None** | 1.0x | Pure internal tools, documentation, optimizations |

---

## Variant Cost Modifier

Penalizes features that increase variant budget (code delta):

| Estimated Variant Delta | Modifier | Notes |
|------------------------|----------|-------|
| **0-1%** | 1.00x | Ideal: No variant cost |
| **1-2%** | 0.98x | Acceptable: Minor customization |
| **2-3%** | 0.95x | Warning: Consider refactoring |
| **3-5%** | 0.90x | High: Requires CCB justification |
| **>5%** | 0.80x | Violation: Must be exceptional case |

**Encourages**: Config-driven overlays, policy-based customization, adapter patterns

---

## Queue Management

### Queue States

1. **Discovery**: Evidence pack in progress
2. **Ready**: DoR complete, waiting for capacity
3. **Blocked**: Dependency, safety review, or vendor blocker
4. **In-Dev**: Actively being developed
5. **Beta**: In pilot/canary rollout
6. **GA**: Generally available in production
7. **Sustain**: Maintenance mode, no active development

### Queue Limits (WIP)

- **Discovery**: Max 10 items (time-boxed to 3 weeks each)
- **Ready**: Max 20 items (prioritized backlog)
- **In-Dev**: Capacity-based (team velocity)
- **Beta**: Max 5 concurrent pilots

---

## Prioritization Process

### Weekly Backlog Prioritization (45 minutes)

**Pre-work**:
- All intake forms reviewed
- RICE scores calculated
- Safety multipliers assigned
- Variant cost estimates validated

**Meeting agenda**:
1. **New Intakes** (15 min): Review, assign to Discovery or reject
2. **Ready Queue** (20 min): Re-rank based on scores, strategic shifts
3. **Blocked Items** (10 min): Unblock or defer

**Output**: Prioritized backlog with top 10 items clearly ranked

### Scoring Examples

**Example 1: Emergency Stop Improvement**
- Reach: 10 (all vehicles)
- Impact: 3 (massive safety impact)
- Confidence: 100% (validated in sim)
- Effort: 6 person-months
- RICE: (10 × 3 × 1.0) / 6 = 5.0
- Safety Multiplier: 5.0x (critical)
- Variant Cost: 0.98x (1% delta)
- **Priority Score**: 5.0 × 5.0 × 0.98 = **24.5** → **P0**

**Example 2: Dashboard Color Theme**
- Reach: 7 (all users)
- Impact: 0.25 (minimal KR movement)
- Confidence: 80%
- Effort: 1 person-month
- RICE: (7 × 0.25 × 0.8) / 1 = 1.4
- Safety Multiplier: 1.0x (none)
- Variant Cost: 1.0x (0% delta)
- **Priority Score**: 1.4 × 1.0 × 1.0 = **1.4** → **P2/P3**

---

## Priority Tiers

| Tier | Score Range | Description | SLA |
|------|-------------|-------------|-----|
| **P0** | ≥15 | Safety-critical, regulatory blockers, major outages | Start within 1 sprint |
| **P1** | 8-14.9 | High business impact, strategic bets, customer commitments | Start within 1 quarter |
| **P2** | 3-7.9 | Important improvements, technical debt, optimizations | Start within 2 quarters |
| **P3** | <3 | Nice-to-haves, future considerations | Backlog, revisit quarterly |

---

## Special Considerations

### Fast-Track Criteria (Skip Normal Queue)

Items that bypass prioritization:
1. **P0 Safety Incidents**: Immediate response required
2. **Regulatory Mandates**: Legal compliance deadlines
3. **Critical Security Vulnerabilities**: CVE with active exploits
4. **Customer SLA Breaches**: Contractual obligations at risk

**Process**: Still require evidence and DoR, but jump to front of queue

### Deferred/Rejected Reasons

Common reasons to defer or reject:
- **Insufficient Evidence**: Not enough user validation
- **No Strategic Alignment**: Doesn't move OKRs
- **Variant Budget Violation**: Delta >5% without justification
- **Technical Infeasibility**: Blocked by fundamental constraints
- **Resource Constraints**: No capacity within reasonable timeframe

---

## Capacity Planning

### Quarterly Capacity Allocation

**Strategic Mix** (target percentages):
- 60% Strategic bets (OKR-driving features)
- 20% Quality & performance (SLO improvements, tech debt)
- 10% Technical debt & refactoring
- 10% Discovery & learning

**Safety Buffer**: 15% sprint capacity reserved for unplanned work (incidents, defects, urgent fixes)

### Team Velocity Tracking
- Measure velocity in story points per sprint
- Track trend over time (should stabilize after 3-4 sprints)
- Adjust capacity planning based on realized velocity

---

## Conflict Resolution

### When Priorities Conflict

1. **Safety First**: Safety-critical work always takes precedence
2. **OKR Alignment**: Feature that moves more/higher OKRs wins
3. **Customer Commitments**: Contractual obligations honored
4. **Steering Group Decision**: VP/Steering resolves conflicts in monthly council

### Escalation Path
1. PM triads attempt to resolve
2. CoP Steward mediates
3. Steering Group decides (monthly meeting)
4. Executive Sponsor makes final call (rare)

---

## Anti-Patterns to Avoid

1. **HiPPO (Highest Paid Person's Opinion)**: Always require evidence
2. **Squeaky Wheel**: Don't prioritize just because someone complains loudly
3. **Shiny Object Syndrome**: New ideas must prove value vs. existing priorities
4. **Sunk Cost Fallacy**: Kill features that aren't working (30-day OQ reviews)
5. **Scope Creep**: Stick to MVP definition; enhancements go to backlog

---

## Success Metrics

- **Prioritization Accuracy**: ≥80% of P0/P1 items deliver expected outcomes
- **Queue Health**: <10% of Ready items blocked for >1 month
- **Re-prioritization Churn**: <20% of items change priority tier per quarter
- **Strategic Alignment**: ≥90% of effort goes to OKR-moving work

---

**This playbook ensures prioritization is objective, evidence-based, and aligned with strategic objectives while respecting safety, compliance, and variant budget constraints.**

