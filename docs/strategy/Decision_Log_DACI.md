# AtlasMesh Fleet OS - Product Decision Log (DACI)

**Purpose**: Single source of truth for all cross-team product decisions  
**Decision Model**: DACI (Driver-Approver-Contributors-Informed)  
**Last Updated**: 2025-10-04

---

## Decision Log Format

Each decision entry follows this structure:

```yaml
decision_id: DEC-YYYY-MM-###
date: YYYY-MM-DD
title: "[Brief decision title]"
status: proposed | approved | rejected | superseded
decision_model:
  driver: "[Name, Role]"
  approver: "[Name, Role]"
  contributors: ["[Name, Role]", ...]
  informed: ["[Team/Group]", ...]
context: |
  [Problem statement and background]
options_considered:
  - option: "[Option A]"
    pros: ["[Pro 1]", "[Pro 2]"]
    cons: ["[Con 1]", "[Con 2]"]
  - option: "[Option B]"
    pros: ["[Pro 1]"]
    cons: ["[Con 1]"]
decision: "[Chosen option and rationale]"
implications:
  technical: ["[Implication 1]"]
  organizational: ["[Implication 2]"]
  financial: ["[Implication 3]"]
traceability:
  okrs: ["[O-#]"]
  epics: ["[E-##]"]
  adrs: ["[ADR-####]"]
next_steps: ["[Action 1]", "[Action 2]"]
review_date: "YYYY-MM-DD"  # When to revisit
```

---

## Active Decisions (2025)

### DEC-2025-10-001: Qualified Agnosticism Framework Adoption
**Date**: 2025-10-02  
**Status**: ✅ Approved

**DACI**:
- **Driver**: Platform Architecture Team
- **Approver**: CTO, VP Engineering, Safety Lead
- **Contributors**: All engineering teams, QA, Product
- **Informed**: All departments

**Context**:
Original "universal agnosticism" vision faced five constraining realities (physics, safety certification, ODD boundaries, sensors, regulations). Need engineering-grounded approach.

**Options Considered**:
1. **Universal Agnosticism** - Rejected (ignores fundamental constraints)
2. **No Agnosticism (Per-Customer Forks)** - Rejected (doesn't scale)
3. **Qualified Agnosticism** - ✅ Selected (bounded, contract-driven, measurable)

**Decision**:
Adopt qualified agnosticism with:
- Vehicle-agnostic (≤5% code delta, class/model-bounded)
- Sector-agnostic (≥90% code reuse via policy overlays)
- Platform-agnostic (100% conformance via adapters)
- Automated variant budget enforcement

**Implications**:
- Technical: 7 new services, 14 new docs, 3 ADRs
- Organizational: PM CoP framework required for governance
- Financial: +40% initial dev, -50% time-to-market for new classes

**Traceability**:
- OKRs: O-2 (Agnostic Architecture Leadership)
- Epics: E-01 (Vehicle), E-02 (Policy), E-03 (Platform)
- ADRs: ADR-0011, ADR-0012, ADR-0013

**Next Steps**:
- Implement Vehicle HAL, Variant Budget, Conformance Testing services
- Execute 90-180 day programmatic proof points
- Establish PM CoP operating cadence

**Review Date**: 2026-04-01 (6-month retrospective)

---

### DEC-2025-10-002: PM Community of Practice Framework
**Date**: 2025-10-04  
**Status**: ✅ Approved

**DACI**:
- **Driver**: VP Product
- **Approver**: CEO, CTO, VP Engineering
- **Contributors**: Product team, Safety, Design, Engineering, QA, CS/GTM
- **Informed**: All employees

**Context**:
Qualified agnosticism requires cross-functional coordination, evidence-based decisions, and variant budget discipline. Need formalized PM framework.

**Options Considered**:
1. **Informal coordination** - Rejected (doesn't scale, inconsistent)
2. **Heavyweight PMO** - Rejected (process bloat, slows velocity)
3. **Lightweight PM CoP** - ✅ Selected (structured but agile)

**Decision**:
Establish PM Community of Practice with:
- Weekly/bi-weekly/monthly/quarterly cadence
- Evidence-first decision making
- DACI model for cross-team decisions
- DoR/DoD enforcement via CI
- Variant budget governance

**Implications**:
- Technical: CI guards for DoR/DoD, traceability, variant budget
- Organizational: New cadences, role definitions, cross-functional reviews
- Financial: ~8 hours/month per PM, tooling costs minimal

**Traceability**:
- OKRs: All (PM CoP supports all objectives)
- Strategic Imperative: Agnostic Architecture Leadership

**Next Steps**:
- Augment existing docs (vision, requirements, roadmap)
- Create 9 templates/playbooks
- Establish cadences starting Week 1
- Update README with PM framework overview

**Review Date**: 2026-01-04 (Quarterly review)

---

## Decision Template (Copy for New Decisions)

```yaml
decision_id: DEC-YYYY-MM-###
date: YYYY-MM-DD
title: ""
status: proposed
decision_model:
  driver: ""
  approver: ""
  contributors: []
  informed: []
context: |
  
options_considered:
  - option: ""
    pros: []
    cons: []
decision: ""
implications:
  technical: []
  organizational: []
  financial: []
traceability:
  okrs: []
  epics: []
  adrs: []
next_steps: []
review_date: ""
```

---

## Decision Process

### 1. Proposal
- Driver creates decision entry with status "proposed"
- Circulates to Contributors for feedback (48h minimum)
- Updates based on feedback

### 2. Review
- Presents to Approver(s) in relevant meeting (Weekly/Monthly)
- Approver may: Approve, Request Changes, Reject
- Decision documented with rationale

### 3. Communication
- Decision log updated with final status
- Informed stakeholders notified
- Next steps assigned with owners

### 4. Execution
- Driver ensures next steps are tracked
- Progress reviewed in subsequent meetings

### 5. Review & Retrospective
- Decisions reviewed on scheduled review_date
- Outcomes assessed against expectations
- Learnings captured for future decisions

---

## Decision History Archive

[Older decisions moved here after review_date passes]

---

**All product decisions are documented in this log to ensure transparency, traceability, and organizational learning.**

