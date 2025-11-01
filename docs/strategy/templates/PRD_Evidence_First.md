# Evidence-First PRD Template

**Epic/Feature**: [Epic ID] - [Feature Name]  
**Owner**: [PM Name]  
**Date**: [YYYY-MM-DD]  
**Status**: [Draft/Review/Approved]  
**Version**: [1.0]

---

## 1. Evidence Pack (Required)

### 1.1 User Research
- [ ] **5-7 contextual interviews** with target users (ops/safety/maintenance)
- [ ] **Interview summary** with key insights and quotes
- [ ] **User personas** and Jobs-To-Be-Done (JTBD) identified
- [ ] **Pain points** validated with quantitative data

### 1.2 Telemetry Analysis
- [ ] **30-day telemetry slice** analyzed for baseline metrics
- [ ] **Current state metrics** documented with trends
- [ ] **Gap analysis** showing improvement opportunity
- [ ] **Data quality** assessment completed

### 1.3 Simulation/Twin Analysis
- [ ] **Scenario impact analysis** using sim/twin environment
- [ ] **Performance impact** quantified
- [ ] **Safety impact** assessed
- [ ] **Edge cases** identified and documented

### 1.4 Policy & Compliance Review
- [ ] **Regulatory requirements** reviewed
- [ ] **Safety standards** compliance checked
- [ ] **Data privacy** implications assessed
- [ ] **Export control** considerations reviewed (if applicable)

---

## 2. Problem Statement

### 2.1 Current State
**What is the current situation?**
- [Describe current state with specific metrics]

### 2.2 Pain Points
**What problems are we solving?**
- [List specific problems with evidence]

### 2.3 Success Criteria
**How will we know we've succeeded?**
- [Define measurable success criteria]

---

## 3. Solution Design

### 3.1 Solution Overview
**What are we building?**
- [High-level solution description]

### 3.2 User Stories
**Who will use this and how?**
- [User stories with acceptance criteria]

### 3.3 Technical Approach
**How will we build it?**
- [Technical approach and architecture]

---

## 4. Requirements

### 4.1 Functional Requirements
| ID | Description | Acceptance Criteria | Priority | Owner |
|----|-------------|-------------------|----------|-------|
| FR-XXX | [Requirement] | [Given/When/Then] | P0/P1/P2/P3 | [Dept] |

### 4.2 Non-Functional Requirements
| ID | Attribute | Requirement | SLI/SLA | Owner |
|----|-----------|-------------|---------|-------|
| NFR-XXX | [Performance/Security/etc] | [Target] | [Measurement] | [Dept] |

---

## 5. Traceability

### 5.1 OKR Linkage
**Which OKRs does this support?**
- [List OKR IDs and how this feature supports them]

### 5.2 Epic Assignment
**Which Epic does this belong to?**
- [Epic ID and description]

### 5.3 Cross-References
**Related FRs/NFRs:**
- [List related requirement IDs]

---

## 6. Variant Budget

### 6.1 Code Delta Estimate
- **Vehicle dimension**: X% delta
- **Sector dimension**: X% delta  
- **Platform dimension**: X% delta
- **Total estimated**: X% (≤5% limit)

### 6.2 Test Delta Estimate
- **Vehicle dimension**: X% delta
- **Sector dimension**: X% delta
- **Platform dimension**: X% delta
- **Total estimated**: X% (≤25% limit)

### 6.3 Budget Justification
**Why is this variant budget acceptable?**
- [Justification for budget usage]

---

## 7. Rollout Plan

### 7.1 Feature Flags
- [ ] Feature flags configured
- [ ] Flag names and conditions defined
- [ ] Rollback triggers identified

### 7.2 Deployment Strategy
- [ ] Canary deployment plan
- [ ] Pilot sites identified
- [ ] Gradual rollout schedule

### 7.3 Rollback Plan
- [ ] Rollback triggers defined
- [ ] Rollback procedure documented
- [ ] Kill-switch functionality specified

---

## 8. Success Metrics

### 8.1 SLI Targets
| Metric | Current | Target | Measurement |
|--------|---------|--------|-------------|
| [Metric Name] | [Baseline] | [Target] | [How to measure] |

### 8.2 Business Impact
**Expected business outcomes:**
- [Quantified business impact]

### 8.3 User Impact
**Expected user experience improvements:**
- [User experience improvements]

---

## 9. Risk Assessment

### 9.1 Technical Risks
| Risk | Impact | Probability | Mitigation |
|------|--------|-------------|------------|
| [Risk] | High/Med/Low | High/Med/Low | [Mitigation] |

### 9.2 Business Risks
| Risk | Impact | Probability | Mitigation |
|------|--------|-------------|------------|
| [Risk] | High/Med/Low | High/Med/Low | [Mitigation] |

### 9.3 Safety Risks
| Risk | Impact | Probability | Mitigation |
|------|--------|-------------|------------|
| [Risk] | High/Med/Low | High/Med/Low | [Mitigation] |

---

## 10. Dependencies

### 10.1 Technical Dependencies
- [List technical dependencies]

### 10.2 Business Dependencies
- [List business dependencies]

### 10.3 External Dependencies
- [List external dependencies]

---

## 11. Approval Chain

### 11.1 Required Approvals
- [ ] **PM Approval**: [PM Name] - [Date]
- [ ] **Design Approval**: [Design Lead] - [Date]
- [ ] **Engineering Approval**: [Eng Lead] - [Date]
- [ ] **Safety Approval**: [Safety Lead] - [Date] (if P0/P1 safety tier)
- [ ] **Compliance Approval**: [Compliance Lead] - [Date] (if regulatory)

### 11.2 DACI Decision
- **Driver**: [Name]
- **Approver**: [Name]
- **Consulted**: [Names]
- **Informed**: [Names]

---

## 12. Definition of Ready Checklist

### 12.1 Evidence Pack
- [ ] User research completed
- [ ] Telemetry analysis completed
- [ ] Sim/twin analysis completed
- [ ] Policy review completed

### 12.2 Requirements
- [ ] FRs defined with acceptance criteria
- [ ] NFRs defined with SLI/SLA
- [ ] Traceability complete
- [ ] Dependencies mapped

### 12.3 Technical
- [ ] SLI targets defined
- [ ] Telemetry events specified
- [ ] Test hooks planned
- [ ] Variant budget estimated

### 12.4 Rollout
- [ ] Feature flags configured
- [ ] Rollout plan defined
- [ ] Rollback procedure documented
- [ ] Safety/compliance sign-off (if applicable)

### 12.5 Ownership
- [ ] Department owners assigned
- [ ] Priority tier confirmed
- [ ] Sector scope defined
- [ ] Triad sign-off (PM+Design+Eng)

---

## 13. Post-Launch Review

### 13.1 30-Day OQ Review
**Scheduled for**: [Date]
**Reviewer**: [Name]

### 13.2 Success Criteria Review
- [ ] SLI targets met
- [ ] Business impact achieved
- [ ] User feedback collected
- [ ] Lessons learned documented

---

## 14. Appendices

### 14.1 Evidence Artifacts
- [Links to evidence documents]

### 14.2 Technical Specifications
- [Links to technical specs]

### 14.3 User Research
- [Links to user research]

### 14.4 Compliance Documentation
- [Links to compliance docs]