# Beta Program Playbook

**Purpose**: Guide for running safe, effective beta programs  
**Owner**: PM CoP  
**Last Updated**: 2025-10-04

---

## Beta Program Overview

A **beta program** is a controlled rollout of new features to a subset of customers/sites to validate functionality, performance, and business outcomes before general availability.

---

## Beta Site Selection Criteria

### Site Characteristics

**Ideal Beta Site**:
- Representative of target use case
- Engaged customer willing to provide feedback
- Operational stability (not dealing with other major issues)
- Technical capability (can handle instrumentation/telemetry)
- Contractual flexibility (willing to accept beta terms)

**Diversity Targets**:
- At least 2 different sectors (if multi-sector feature)
- At least 2 different vehicle classes (if vehicle-specific)
- Mix of deployment platforms (Azure + on-prem if applicable)
- Geographic diversity (different regulatory environments)

### Minimum Beta Cohort Size

| Feature Type | Minimum Sites | Minimum Vehicles | Duration |
|--------------|---------------|------------------|----------|
| **Safety-Critical** | 3 sites | 15 vehicles | 90 days |
| **Operational** | 2 sites | 10 vehicles | 60 days |
| **Enhancement** | 1 site | 5 vehicles | 30 days |

---

## Beta Program Phases

### Phase 1: Planning & Preparation (2-3 weeks)

**Activities**:
- [ ] Select beta sites based on criteria above
- [ ] Obtain customer agreement and sign beta terms
- [ ] Define success criteria and exit conditions
- [ ] Prepare beta-specific instrumentation
- [ ] Create beta monitoring dashboard
- [ ] Brief beta site teams on program goals

**Deliverables**:
- Beta program charter
- Site selection rationale
- Success criteria document
- Monitoring plan
- Communication plan

### Phase 2: Deployment & Ramp (1-2 weeks)

**Activities**:
- [ ] Deploy to beta sites with feature flags
- [ ] Validate instrumentation is working
- [ ] Monitor initial performance (first 24-72 hours intensively)
- [ ] Daily check-ins with beta sites
- [ ] Fast-response support channel established

**Success Gates**:
- No P0/P1 incidents in first 48 hours
- All instrumentation emitting as expected
- Beta site teams trained and comfortable

### Phase 3: Evaluation & Learning (4-12 weeks)

**Activities**:
- [ ] Weekly beta site check-ins
- [ ] SLI/SLO monitoring vs. targets
- [ ] Collect qualitative feedback
- [ ] Identify and fix issues
- [ ] Compare actual vs. expected outcomes
- [ ] Document learnings and improvements

**Data Collection**:
- Quantitative: SLIs, performance metrics, error rates
- Qualitative: User interviews, feedback surveys
- Operational: Support tickets, escalations, workarounds

### Phase 4: Decision & Next Steps (1 week)

**Decision Options**:
1. **Promote to GA**: Success criteria met, ready for full rollout
2. **Extend Beta**: Need more time/data, extend with specific goals
3. **Iterate**: Issues identified, pull back and fix
4. **Sunset**: Doesn't work, kill the feature

**Decision Criteria**: See "Success Criteria" section below

---

## Success Criteria

### Quantitative Criteria (Must Pass)

- [ ] **Safety**: Zero P0 safety incidents; assist rate ≤ target
- [ ] **Performance**: SLIs meet targets (p50/p95/p99)
- [ ] **Reliability**: Availability ≥ target (typically 99.5%+)
- [ ] **Business Outcomes**: OKR metrics show positive movement
- [ ] **Adoption**: ≥80% of users at beta sites actively using feature

### Qualitative Criteria (Should Pass)

- [ ] **User Satisfaction**: Positive feedback from beta site teams
- [ ] **Operational Fit**: Integrates smoothly into workflows
- [ ] **Support Load**: No excessive support burden
- [ ] **Training Burden**: Users can be trained effectively

### Red Flags (Automatic Pull-Back)

- **Any P0 safety incident** related to beta feature
- **Availability degradation** >1% at beta sites
- **Assist rate increase** >20% vs. baseline
- **Customer escalation** to executive level
- **Regulatory concern** raised by compliance team

---

## Evidence Capture

### Continuous Monitoring

**Dashboards** (check daily):
- Safety metrics (assists, incidents, near-misses)
- Performance metrics (latency, throughput, error rates)
- Business metrics (productivity, cost, customer satisfaction)
- System health (availability, resource utilization)

**Alerts** (respond immediately):
- P0/P1 incidents
- SLI threshold breaches
- Availability degradation
- Error rate spikes

### Feedback Collection

**Weekly surveys** (beta site teams):
- What's working well?
- What's not working?
- What's missing?
- Would you recommend this feature?

**Bi-weekly interviews** (selected users):
- Deep-dive on user experience
- Workflow integration assessment
- Pain points and workarounds
- Suggestions for improvement

**Continuous telemetry**:
- Feature usage patterns
- User journey completion rates
- Error/retry rates
- Abandonment points

---

## Exit & Rollback

### GA Promotion Criteria

**All must be true**:
- [ ] All quantitative success criteria met
- [ ] No red flags present
- [ ] Qualitative feedback predominantly positive
- [ ] Evidence bundle complete for regulatory compliance
- [ ] Support playbooks and runbooks updated
- [ ] Rollback procedure tested and verified

### Rollback Triggers

**Immediate rollback if**:
- P0 safety incident attributed to beta feature
- System availability drops below 99% at beta sites
- Customer demands removal

**Planned rollback if**:
- Success criteria not met after maximum beta duration
- Iteration required based on feedback
- Strategic priorities change

### Rollback Procedure

1. **Disable feature flags** at affected sites
2. **Verify rollback** (functional and data consistency)
3. **Monitor recovery** (24-48 hours)
4. **Communicate** to beta sites and stakeholders
5. **Document** reasons and learnings
6. **Plan** next steps (iterate, extend, sunset)

---

## Communication Plan

### Stakeholder Communication

**Beta Launch**:
- Announce to beta sites 1 week prior
- Provide feature overview, benefits, risks
- Set expectations on feedback and support

**Weekly Updates**:
- Status summary (green/yellow/red)
- Key metrics snapshot
- Issues encountered and resolutions
- Next steps

**Beta Completion**:
- Results summary (quantitative + qualitative)
- Decision rationale (GA/extend/iterate/sunset)
- Next steps and timeline

### Internal Communication

- **Engineering**: Daily standups during ramp, weekly thereafter
- **Support**: Weekly sync on issues and escalations
- **Leadership**: Bi-weekly status updates
- **PM CoP**: Beta results in monthly Craft Review

---

## Regulator Communication

### For Safety-Critical Features

**Before Beta**:
- [ ] Notify relevant regulatory authorities
- [ ] Provide beta program plan and safety analysis
- [ ] Obtain necessary approvals or exemptions

**During Beta**:
- [ ] Monthly progress reports to regulators
- [ ] Immediate notification of any safety incidents
- [ ] Provide requested data or evidence

**After Beta**:
- [ ] Final beta report with evidence bundle
- [ ] Address any regulatory feedback
- [ ] Obtain clearance for GA (if required)

---

## Beta Program Checklist

### Pre-Launch
- [ ] Beta sites selected and agreements signed
- [ ] Success criteria defined and baseline captured
- [ ] Monitoring dashboard configured
- [ ] Feature flags and kill-switch tested
- [ ] Rollback procedure documented and tested
- [ ] Support team trained
- [ ] Beta site teams trained
- [ ] Regulator notification (if safety-critical)

### During Beta
- [ ] Daily monitoring (first week)
- [ ] Weekly check-ins with beta sites
- [ ] Weekly metrics review
- [ ] Issue tracking and rapid response
- [ ] Continuous evidence collection
- [ ] Feedback synthesis

### Post-Beta
- [ ] 30-day OQ review completed
- [ ] Evidence bundle generated
- [ ] Decision documented (GA/extend/iterate/sunset)
- [ ] Learnings captured for playbook updates
- [ ] Beta sites thanked and updated on decision

---

## Success Metrics

- **Beta Completion Rate**: ≥90% of betas complete without major rollback
- **GA Promotion Rate**: ≥70% of successful betas promote to GA
- **Learning Velocity**: Average 3+ actionable insights per beta
- **Customer Satisfaction**: ≥80% of beta participants would recommend

---

**This playbook ensures beta programs are safe, evidence-based, and deliver valuable learning while minimizing risk to customers and operations.**

