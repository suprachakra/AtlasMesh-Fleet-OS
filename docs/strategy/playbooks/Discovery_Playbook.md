# Discovery Playbook

**Purpose**: Guide PMs through evidence-first discovery process  
**Owner**: PM CoP  
**Last Updated**: 2025-10-04

---

## Overview

**Discovery** is a time-boxed (1-3 weeks) process to validate whether an opportunity is worth pursuing. The output is an **Opportunity Canvas** and **Evidence Pack** that enables go/no-go decisions.

---

## Minimum Evidence Pack

Before writing a PRD, gather this minimum evidence:

### 1. User Research (5-7 Contextual Interviews)

**Who to interview**:
- Fleet Operators (day-to-day users)
- Safety Managers (safety/compliance owners)
- Maintenance Technicians (vehicle service personnel)
- Site Supervisors (operational leadership)
- End Users (passengers for ride-hail, cargo recipients for logistics)

**Interview structure** (45-60 minutes):
1. **Context**: Current role, typical day, responsibilities
2. **Pain Points**: Specific problems encountered, frequency, impact
3. **Workarounds**: How they solve it today, why it's suboptimal
4. **Success Criteria**: What would "good" look like
5. **Constraints**: Regulatory, operational, technical limitations

**Deliverable**: Interview notes template → `docs/strategy/templates/Customer_Interview_Guide.md`

### 2. Telemetry Analysis (30-Day Slice)

**Data to analyze**:
- Incident/assist co-occurrence patterns
- Performance bottlenecks (latency, errors, timeouts)
- User behavior patterns (abandonment, retries, error rates)
- System health metrics (availability, degradation events)

**Analysis questions**:
- How often does this problem occur?
- What's the business impact? (cost, safety, customer satisfaction)
- Which sectors/sites are most affected?
- What's the trend over time?

**Deliverable**: Telemetry dashboard with key insights and quantified pain points

### 3. Simulation Impact Analysis

**Scenarios to test**:
- Baseline: Current system behavior
- Hypothesis: Proposed solution impact
- Edge cases: Failure modes and degraded conditions

**Metrics to capture**:
- Safety impact (assist rate, incidents)
- Performance impact (latency, throughput)
- Reliability impact (availability, MTBF)

**Deliverable**: Sim/twin scenario results with before/after comparison

### 4. Policy & Compliance Review

**Questions to answer**:
- Does this touch autonomous behavior? (Yes → Safety sign-off required)
- Does this change policies? (Yes → Policy engine updates)
- Which compliance frameworks are affected? (ISO 26262, SOTIF, R155/R156)
- What evidence artifacts are required?

**Reviewers**:
- Safety Lead (for autonomy-impacting changes)
- Compliance Officer (for regulatory changes)
- Security Team (for data/access changes)

**Deliverable**: Signed compliance review with any constraints or requirements

---

## Discovery Process (1-3 Weeks)

### Week 1: Research & Data Collection
- **Day 1-2**: Define research questions, schedule interviews
- **Day 3-5**: Conduct 5-7 user interviews
- **Day 4-5**: Parallel telemetry analysis (30-day slice)

### Week 2: Analysis & Validation
- **Day 1-2**: Synthesize interview insights, identify patterns
- **Day 3**: Run simulation scenarios (baseline vs hypothesis)
- **Day 4**: Conduct policy & compliance review
- **Day 5**: Create Opportunity Canvas draft

### Week 3: Decision & Planning (if needed)
- **Day 1-2**: Refine Opportunity Canvas based on feedback
- **Day 3**: Present to PM triad (PM+Design+Eng) for validation
- **Day 4**: Present to CoP Discovery Review for go/no-go
- **Day 5**: Document decision, create PRD outline (if approved)

---

## Decision Types

### Build vs. Partner
**Build if**:
- Core differentiation or competitive advantage
- High strategic value (moves multiple OKRs)
- No suitable partners available
- Variant budget impact acceptable

**Partner if**:
- Commodity capability available in market
- Not core differentiation
- Partner has proven track record
- Faster time-to-market

### MVP Scope Definition
**Include in MVP**:
- Minimum capabilities to validate hypothesis
- End-to-end workflow for primary user
- Essential safety/compliance requirements
- SLI instrumentation for outcome measurement

**Defer to V1.1+**:
- Nice-to-have enhancements
- Secondary user workflows
- Performance optimizations (unless safety-critical)
- Additional sector customizations

### Sector Overlay vs. Core Feature
**Core Feature** (shared across all sectors):
- Universal user need (dispatch, routing, monitoring)
- No sector-specific business logic
- Contributes to ≥90% code reuse target

**Sector Overlay** (sector-specific):
- Sector-unique workflows or requirements
- Regulatory/compliance differences
- Sector-specific KPIs or terminology
- Implemented via policy engine + UI tokens

---

## Quality Checks

### Evidence Pack Completeness
- [ ] 5-7 interviews completed with diverse roles
- [ ] Telemetry analysis shows quantified pain points
- [ ] Simulation validates hypothesis and impact
- [ ] Policy/compliance review signed off

### Opportunity Canvas Quality
- [ ] Problem statement is clear and specific
- [ ] Evidence directly supports problem claims
- [ ] Business impact is quantified
- [ ] Scope is realistic and bounded
- [ ] Risks are identified with mitigations

### Go/No-Go Criteria
- [ ] Strategic alignment confirmed (OKRs/Epics)
- [ ] Evidence is compelling (not just opinions)
- [ ] Technical feasibility validated
- [ ] Variant budget within acceptable range
- [ ] Resources available for execution

---

## Common Pitfalls to Avoid

1. **Opinion-Based Decisions**: Always back claims with evidence
2. **Scope Creep**: Define MVP tightly; defer enhancements
3. **Skipping Safety Review**: Required for all autonomy-impacting changes
4. **Ignoring Variant Budget**: Estimate and validate budget impact early
5. **Insufficient Evidence**: Don't rush; gather quality evidence
6. **Analysis Paralysis**: Time-box discovery (1-3 weeks max)

---

## Discovery Review Meeting Format

**Pre-work (48h before)**:
- Opportunity Canvas shared
- Evidence pack attached
- Reviewers read and comment asynchronously

**Meeting (60 minutes)**:
- 10 min: PM presents opportunity and evidence
- 20 min: Q&A and clarifications
- 20 min: Discussion of alternatives and risks
- 10 min: Go/no-go decision and next steps

**Post-meeting**:
- Decision documented in DACI log
- If go: PRD development starts
- If no-go: Opportunity archived with learnings
- If needs-more-evidence: Discovery continues with specific questions

---

## Success Metrics

- **Evidence Quality**: ≥90% of approved opportunities have complete evidence packs
- **Decision Speed**: Average time from intake to decision ≤3 weeks
- **Outcome Accuracy**: ≥70% of approved opportunities achieve target outcomes
- **False Positives**: ≤10% of approved opportunities fail in execution

---

**This playbook ensures all product decisions are evidence-based, strategically aligned, and set up for measurable success.**

