# [Feature Name] - Product Requirements Document

**Document Type**: Evidence-First PRD  
**Status**: Draft | In Review | Approved | Delivered  
**Owner**: [PM Name]  
**Contributors**: [Design, Eng, Safety, Data]  
**Created**: YYYY-MM-DD  
**Last Updated**: YYYY-MM-DD  
**Target Release**: Q# YYYY

---

## 1. Problem & Evidence

### Problem Statement
[Clear, concise description of the problem being solved]

### Evidence Pack

**User Interviews**:
- [5-7 contextual interviews with ops/safety/maintenance personnel]
- Key insights: [Summarize main pain points]
- Interview notes: [Links to detailed notes]

**Telemetry Analysis**:
- Analysis period: [e.g., Last 30 days]
- Key findings: [Quantified pain points from data]
- Dashboard links: [Links to telemetry dashboards]

**Simulation Impact**:
- Scenarios tested: [List sim/twin scenarios]
- Impact analysis: [Expected improvement metrics]
- Simulation results: [Links to sim reports]

**Policy & Compliance Review**:
- Regulatory frameworks impacted: [ISO 26262, SOTIF, etc.]
- Policy changes required: [Yes/No, details]
- Compliance sign-off: [Safety Lead, Compliance Officer]

---

## 2. Users & Jobs-To-Be-Done (JTBD)

### Primary Users
| User Role | JTBD | Current Pain | Proposed Solution |
|-----------|------|--------------|-------------------|
| [Role] | [Job to be done] | [Current workaround/pain] | [How we solve it] |

### User Personas
[Link to persona documents or brief description]

---

## 3. Scope & ODD Impact

### In Scope
- [Feature capabilities included]

### Out of Scope
- [Explicitly excluded capabilities]

### ODD (Operational Design Domain) Impact
- **Sectors Affected**: [defense, mining, logistics, ride_hail]
- **Vehicle Classes**: [ClassA, ClassB, ClassC, etc.]
- **Platforms**: [azure_eks, aws_eks, on_prem_k3s]
- **Environmental Conditions**: [Weather, terrain, connectivity constraints]
- **ODD Changes**: [Any changes to operational boundaries]

---

## 4. Functional Requirements (FRs)

| FR ID | Description | Acceptance Criteria | Priority | Epic | OKR |
|-------|-------------|---------------------|----------|------|-----|
| FR-XXX | [Requirement] | Given [context] When [action] Then [outcome]; **Neg:** [failure case] | P0/P1/P2 | E-## | O-# |

---

## 5. Non-Functional Requirements (NFRs)

| NFR ID | Category | Requirement | Target | Measurement |
|--------|----------|-------------|--------|-------------|
| NFR-XXX | Performance | [Requirement] | [Target value] | [How measured] |
| NFR-XXX | Reliability | [Requirement] | [Target value] | [How measured] |
| NFR-XXX | Security | [Requirement] | [Target value] | [How measured] |
| NFR-XXX | Compliance | [Requirement] | [Target value] | [How measured] |

---

## 6. Policy Deltas

### Policy Changes Required
- [List policy rules that need to be added/modified/removed]

### Rego Policy Updates
```rego
# Example policy code changes
package atlasmesh.policy.[domain]

# [Policy rule description]
```

### Sector-Specific Overlays
| Sector | Policy Overlay | Justification |
|--------|----------------|---------------|
| [Sector] | [Policy changes] | [Why needed] |

---

## 7. Safety & Evidence Hooks

### Safety Impact Assessment
- **Safety Criticality**: [None | Low | Medium | High | Critical]
- **ASIL Level**: [ASIL-A | ASIL-B | ASIL-C | ASIL-D]
- **Hazard Analysis**: [Link to HARA/FMEA analysis]

### Evidence Requirements
- **ISO 26262**: [Required work products]
- **SOTIF**: [Scenario validation requirements]
- **UN R155/R156**: [Cybersecurity/OTA requirements]

### Safety Gates
- [ ] Hazard analysis completed
- [ ] Safety requirements derived
- [ ] Verification & validation plan approved
- [ ] Safety case updated

---

## 8. SLIs/SLAs & Dashboards

### Service Level Indicators (SLIs)

| SLI Name | Definition | Target | Alert Threshold | Dashboard |
|----------|------------|--------|-----------------|-----------|
| [Metric] | [How measured] | [Target value] | [When to alert] | [Grafana link] |

### Service Level Agreements (SLAs)
- [Customer-facing SLA commitments]

### Monitoring & Alerting
- **Dashboards**: [Links to Grafana dashboards]
- **Alerts**: [PagerDuty/Slack alert configurations]
- **Runbooks**: [Links to incident response runbooks]

---

## 9. Data & ML Needs

### Data Requirements
- **Data Sources**: [Where data comes from]
- **Data Volume**: [Expected data volume]
- **Data Retention**: [Retention policy]
- **Data Privacy**: [PII handling, purpose binding]

### ML Models (if applicable)
- **Model Type**: [Prediction, classification, optimization]
- **Model Card**: [Link to model card]
- **Training Data**: [Dataset description]
- **Drift Monitoring**: [How drift is detected]
- **Retraining Strategy**: [When/how model is updated]

### Feature Store Integration
- **Features Required**: [List of features]
- **Feature Lineage**: [Data provenance]

---

## 10. UX Flows & Design

### User Flows
[Describe key user flows or link to design artifacts]

### Wireframes/Mockups
[Links to Figma/design files]

### Design Tokens
- **Color Scheme**: [Sector-specific or default]
- **Typography**: [Font choices]
- **Components Used**: [Design system components]

### Accessibility Requirements
- [ ] WCAG 2.2 AA compliance validated
- [ ] Keyboard navigation support
- [ ] Screen reader compatibility
- [ ] RTL (Right-to-Left) support if needed
- [ ] Color contrast ratios meet standards

---

## 11. Rollout & Rollback

### Rollout Strategy
- **Phase 1**: [Canary - X% of fleet/users]
- **Phase 2**: [Pilot sites - Specific locations]
- **Phase 3**: [GA - Full rollout]

### Feature Flags
| Flag Name | Purpose | Default State | Rollout %  |
|-----------|---------|---------------|-----------|
| [flag_id] | [Purpose] | disabled/enabled | 0-100% |

### Kill Switch
- **Mechanism**: [How to emergency disable]
- **Trigger Conditions**: [When to activate]
- **Fallback Behavior**: [What happens when disabled]

### Rollback Procedure
1. [Step-by-step rollback process]
2. [Recovery time objective]
3. [Data consistency handling]

---

## 12. Variant Budget

### Estimated Impact
- **Vehicle-Agnostic Delta**: [X%] (Limit: ≤5%)
- **Sector-Agnostic Delta**: [X%] (Limit: ≤5%)
- **Platform-Agnostic Delta**: [X%] (Limit: ≤5%)
- **Test Complexity Delta**: [X%] (Limit: ≤25%)

### Justification
[Why this variant cost is necessary and acceptable]

### Mitigation Strategy
[How we'll minimize variant budget impact]

---

## 13. Risks & Mitigations

| Risk | Impact | Probability | Mitigation | Owner |
|------|--------|-------------|------------|-------|
| [Risk description] | High/Medium/Low | High/Medium/Low | [Mitigation strategy] | [Owner] |

---

## 14. Success Metrics & Evaluation Plan

### Success Criteria
| Metric | Baseline | Target | Timeline |
|--------|----------|--------|----------|
| [Metric] | [Current] | [Goal] | [When measured] |

### Evaluation Plan
- **30-Day OQ Review**: [Scheduled date]
- **Success Definition**: [What constitutes success]
- **Failure Criteria**: [What triggers rollback/iteration]

### Learning Objectives
- [Key questions to answer post-launch]

---

## 15. Traceability

### Strategic Linkage
```yaml
traceability:
  okrs: [O-2, O-4]              # Company objectives
  key_results: [KR.O2.1]        # Specific KRs
  epics: [E-05, E-10]           # Epic alignment
  frs: [FR-012, FR-045]         # Functional requirements
  nfrs: [NFR-P-02, NFR-Sc-01]   # Non-functional requirements
  tests:
    - bdd/scenarios/xxx.feature
    - sim/scenarios/yyy.yaml
    - perf/benchmarks/zzz.js
  slis:
    - MET.OPS.DISPATCH_LAT_P95
    - MET.SAFETY.ASSIST_RATE
  evidence:
    - compliance/iso26262/xxx.pdf
    - audit/policy_decisions/yyy.json
```

### Dependencies
- **Upstream**: [What must be delivered first]
- **Downstream**: [What depends on this]
- **Parallel**: [What's being developed concurrently]

---

## 16. Definition of Ready Checklist

**Evidence Pack**:
- [ ] 5-7 contextual interviews completed
- [ ] 30-day telemetry analysis performed
- [ ] Sim/twin scenario impact validated
- [ ] Policy & compliance review signed off

**Requirements**:
- [ ] Opportunity Canvas documented
- [ ] JTBD clearly identified
- [ ] Acceptance criteria written (incl. negative cases)
- [ ] Risks & mitigations documented

**Technical**:
- [ ] SLIs/SLAs defined with thresholds
- [ ] Telemetry events/fields specified
- [ ] Test strategy planned
- [ ] Dependencies mapped

**Traceability**:
- [ ] OKR linkage documented
- [ ] Epic assignment confirmed
- [ ] Cross-references complete

**Rollout**:
- [ ] Rollout plan defined
- [ ] Rollback procedure documented
- [ ] Safety/compliance sign-off (if needed)
- [ ] Variant budget estimate within limits

**Approval**:
- [ ] Triad sign-off (PM + Design + Eng)
- [ ] Safety sign-off (if policy-touching)
- [ ] CoP Steward approval

---

## 17. Approval & Sign-Off

| Role | Name | Date | Signature |
|------|------|------|-----------|
| **PM Owner** | | | |
| **Design Lead** | | | |
| **Engineering Lead** | | | |
| **Safety Lead** (if required) | | | |
| **CoP Steward** | | | |

**Approval Decision**: Approved | Needs Revision | Deferred | Rejected

**Comments**: [Approval comments, conditions, or requirements]

---

**This PRD follows the AtlasMesh evidence-first product management framework. All requirements are traceable to strategic objectives, backed by evidence, and designed with safety, compliance, and variant budget constraints in mind.**

