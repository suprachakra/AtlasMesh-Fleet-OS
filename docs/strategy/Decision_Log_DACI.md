## Product Decision Log (DACI)

**Purpose**: Single source of truth for all cross-team product decisions  
**Decision Model**: DACI (Driver-Approver-Contributors-Informed)  
**Last Updated**: 2025-10-04

---

### Decision Format

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
review_date: "YYYY-MM-DD"
```

---

### Active Decisions (2025)

| Decision ID | Date | Status | Driver | Approver | Context | Decision | Review Date |
|-------------|------|--------|--------|----------|---------|----------|-------------|
| **DEC-2025-10-001** | 2025-10-02 | ✅ Approved | Platform Architecture Team | CTO, VP Engineering, Safety Lead | Original "universal agnosticism" vision faced five constraining realities. Need engineering-grounded approach. | Adopt qualified agnosticism with vehicle-agnostic (≤5% code delta), sector-agnostic (≥90% code reuse), platform-agnostic (100% conformance), automated variant budget enforcement | 2026-04-01 |
| **DEC-2025-10-002** | 2025-10-04 | ✅ Approved | VP Product | CEO, CTO, VP Engineering | Qualified agnosticism requires cross-functional coordination, evidence-based decisions, and variant budget discipline. Need formalized PM framework. | Establish PM Community of Practice with weekly/bi-weekly/monthly/quarterly cadence, evidence-first decision making, DACI model, DoR/DoD enforcement via CI, variant budget governance | 2026-01-04 |

#### Decision Details

| Decision ID | DACI | Options Considered | Implications | Traceability | Next Steps |
|-------------|------|-------------------|--------------|--------------|------------|
| **DEC-2025-10-001** | **Driver**: Platform Architecture Team<br/>**Approver**: CTO, VP Engineering, Safety Lead<br/>**Contributors**: All engineering teams, QA, Product<br/>**Informed**: All departments | 1. **Universal Agnosticism** - Rejected (ignores fundamental constraints)<br/>2. **No Agnosticism (Per-Customer Forks)** - Rejected (doesn't scale)<br/>3. **Qualified Agnosticism** - ✅ Selected (bounded, contract-driven, measurable) | **Technical**: 7 new services, 14 new docs, 3 ADRs<br/>**Organizational**: PM CoP framework required for governance<br/>**Financial**: +40% initial dev, -50% time-to-market for new classes | **OKRs**: O-2 (Agnostic Architecture Leadership)<br/>**Epics**: E-01 (Vehicle), E-02 (Policy), E-03 (Platform)<br/>**ADRs**: ADR-0011, ADR-0012, ADR-0013 | - Implement Vehicle HAL, Variant Budget, Conformance Testing services<br/>- Execute 90-180 day programmatic proof points<br/>- Establish PM CoP operating cadence |
| **DEC-2025-10-002** | **Driver**: VP Product<br/>**Approver**: CEO, CTO, VP Engineering<br/>**Contributors**: Product team, Safety, Design, Engineering, QA, CS/GTM<br/>**Informed**: All employees | 1. **Informal coordination** - Rejected (doesn't scale, inconsistent)<br/>2. **Heavyweight PMO** - Rejected (process bloat, slows velocity)<br/>3. **Lightweight PM CoP** - ✅ Selected (structured but agile) | **Technical**: CI guards for DoR/DoD, traceability, variant budget<br/>**Organizational**: New cadences, role definitions, cross-functional reviews<br/>**Financial**: ~8 hours/month per PM, tooling costs minimal | **OKRs**: All (PM CoP supports all objectives)<br/>**Strategic Imperative**: Agnostic Architecture Leadership | - Augment existing docs (vision, requirements, roadmap)<br/>- Create 9 templates/playbooks<br/>- Establish cadences starting Week 1<br/>- Update README with PM framework overview |

---

### Decision Template (Copy for New Decisions)

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

### Decision Process

##### 1. Proposal
- Driver creates decision entry with status "proposed"
- Circulates to Contributors for feedback (48h minimum)
- Updates based on feedback

##### 2. Review
- Presents to Approver(s) in relevant meeting (Weekly/Monthly)
- Approver may: Approve, Request Changes, Reject
- Decision documented with rationale

##### 3. Communication
- Decision log updated with final status
- Informed stakeholders notified
- Next steps assigned with owners

##### 4. Execution
- Driver ensures next steps are tracked
- Progress reviewed in subsequent meetings

##### 5. Review & Retrospective
- Decisions reviewed on scheduled review_date
- Outcomes assessed against expectations
- Learnings captured for future decisions

---

### Decision History Archive

[Older decisions moved here after review_date passes]

---

**All product decisions are documented in this log to ensure transparency, traceability, and organizational learning.**
