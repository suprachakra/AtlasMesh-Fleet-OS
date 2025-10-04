# Deprecation Playbook

**Purpose**: Safe, customer-friendly deprecation of features, APIs, and configurations  
**Owner**: PM CoP  
**Last Updated**: 2025-10-04

---

## Deprecation Philosophy

**Deprecation is a product feature** with:
- Clear timelines and communication
- Migration paths and tooling
- Evidence of user adoption
- Support during transition
- Respectful customer treatment

---

## Deprecation Timeline

### Standard Deprecation Windows

| Component Type | Minimum Notice | Total Deprecation Period | Phases |
|----------------|----------------|-------------------------|--------|
| **Public APIs** | 12 months | 18 months | Announce → Migrate → Sunset |
| **Vehicle Profiles** | 12 months | 18 months | Announce → Certify Replacement → Sunset |
| **Sector Overlays** | 18 months | 24 months | Announce → Migrate → Support → Sunset |
| **Platform Adapters** | 6 months | 12 months | Announce → Migrate → Sunset |
| **UI Features** | 6 months | 12 months | Announce → Migrate → Sunset |
| **Internal Services** | 3 months | 6 months | Announce → Migrate → Sunset |

### Accelerated Deprecation

**Security/Safety reasons** (3-6 month fast-track):
- Critical security vulnerability with no patch
- Safety certification revoked
- Regulatory prohibition

**Process**: Same phases, compressed timeline, intensive customer support

---

## Deprecation Phases

### Phase 1: Announcement (T-12 to T-6 months)

**Activities**:
- [ ] Publish deprecation notice in release notes
- [ ] Update documentation with deprecation warnings
- [ ] Add deprecation warnings to affected APIs/UI
- [ ] Identify affected customers and notify directly
- [ ] Publish migration guide
- [ ] Set up support channels for migration questions

**Communications**:
- Release notes: "⚠️ DEPRECATED: [Feature] will be removed in [Version/Date]"
- Email to affected customers
- In-app warnings (banner, toast, console logs)
- Documentation updates

### Phase 2: Migration Period (T-6 to T-1 months)

**Activities**:
- [ ] Provide migration tooling/scripts
- [ ] Offer migration support (office hours, dedicated support)
- [ ] Track migration progress via telemetry
- [ ] Identify stragglers and provide proactive outreach
- [ ] Update examples and tutorials to use new approach
- [ ] Monitor support ticket volume and common issues

**Telemetry to Track**:
```yaml
deprecation.old_feature_usage:
  fields:
    - customer_id
    - usage_count
    - last_used_timestamp
  purpose: "Identify customers still using deprecated feature"
  
deprecation.migration_complete:
  fields:
    - customer_id
    - migrated_from
    - migrated_to
    - migration_date
  purpose: "Track successful migrations"
```

**Outreach Triggers**:
- T-60 days: Email reminder to all affected customers
- T-30 days: Personal outreach to customers with high usage
- T-14 days: Escalation to account managers for non-responsive customers
- T-7 days: Final warning with specific sunset date

### Phase 3: Support Period (T-1 to T+0 months)

**Activities**:
- [ ] Increase support capacity for migration issues
- [ ] Publish updated FAQs based on common questions
- [ ] Create migration success stories / case studies
- [ ] Monitor system health for migration-related issues
- [ ] Prepare sunset execution plan

**Customer Support**:
- Dedicated Slack channel or support queue
- Office hours with engineers
- 1:1 migration assistance for strategic customers
- Expedited issue resolution

### Phase 4: Sunset (T+0)

**Activities**:
- [ ] Remove deprecated feature from codebase
- [ ] Update documentation to remove references
- [ ] Archive deprecation-related assets
- [ ] Monitor for unexpected dependencies
- [ ] Verify no customer impact from removal

**Execution**:
1. Deploy sunset release during maintenance window
2. Monitor system health (24-48 hours intensive)
3. Respond to any reported issues
4. Confirm successful removal
5. Archive deprecation project

---

## Migration Guide Template

Every deprecation must include a migration guide:

### Migration Guide Structure

```markdown
# Migration Guide: [Old Feature] → [New Feature]

## Why We're Deprecating
[Explanation of reasons]

## What's Changing
[Side-by-side comparison]

## Migration Steps
1. [Step 1]
2. [Step 2]
3. [Step 3]

## Code Examples
### Before (Deprecated)
```[language]
[old code]
```

### After (New Approach)
```[language]
[new code]
```

## Migration Tools
[Scripts, automation, or tools available]

## FAQ
[Common questions and answers]

## Support
[How to get help]
```

---

## Telemetry & Monitoring

### Usage Tracking

**Implement telemetry**:
- Count of deprecated feature usage per customer
- Last usage timestamp
- Migration completion status
- Error rates during migration

**Dashboard**:
- Active users of deprecated feature (trend down)
- Migration completion rate (trend up)
- Support ticket volume
- Migration-related errors

### Success Criteria

- [ ] ≥95% of customers migrated before sunset
- [ ] <5% increase in support tickets during migration
- [ ] Zero P0/P1 incidents caused by deprecation
- [ ] Customer satisfaction maintained (NPS ≥ baseline)

---

## Communication Templates

### Deprecation Announcement Email

```
Subject: [Feature Name] Deprecation Notice - Action Required

Dear [Customer],

We're writing to inform you of an upcoming change to AtlasMesh Fleet OS.

WHAT'S CHANGING
[Feature Name] will be deprecated on [Date]. This feature is being replaced by 
[New Feature] which provides [benefits].

WHY WE'RE MAKING THIS CHANGE
[Brief explanation: better performance, improved safety, regulatory compliance]

TIMELINE
- TODAY: Deprecation announced
- [T-6 months]: Migration support begins
- [T-1 month]: Final migration window
- [T+0]: Feature removed

WHAT YOU NEED TO DO
[Specific migration steps or link to migration guide]

MIGRATION SUPPORT
We're here to help:
- Migration Guide: [link]
- Office Hours: [schedule]
- Support Channel: [Slack/email]
- Dedicated Engineer: [contact]

YOUR CURRENT USAGE
Based on telemetry, your fleet uses this feature:
- [Usage statistics specific to customer]

QUESTIONS?
Contact: [support email/Slack]

Thank you for your partnership,
AtlasMesh Product Team
```

### Sunset Reminder (T-30 days)

```
Subject: REMINDER: [Feature Name] Sunset in 30 Days

Dear [Customer],

This is a reminder that [Feature Name] will be removed on [Date] - just 30 days away.

MIGRATION STATUS
Based on our records:
[X] of your [Y] vehicles have migrated ✅
[Z] vehicles still using deprecated feature ⚠️

URGENT: NEXT STEPS
[Specific actions needed]

NEED HELP?
We're available to assist:
[Contact information and support channels]
```

---

## Support FAQ Template

**Q: Why is this feature being deprecated?**  
A: [Reason with customer benefit focus]

**Q: What happens if I don't migrate?**  
A: On [sunset date], [feature] will stop working. [Impact description]

**Q: Is there an equivalent feature?**  
A: Yes, [new feature] provides [benefits]. Migration guide: [link]

**Q: Who can help with migration?**  
A: [Support channels and contacts]

**Q: What if I have a unique use case?**  
A: Contact [PM/eng contact] to discuss alternatives.

**Q: Can I get an extension?**  
A: Extensions are evaluated case-by-case. Contact [account manager] with business justification.

---

## Regulator Communication

### For Safety/Compliance-Impacting Deprecations

**Before Announcement**:
- [ ] Notify relevant regulatory authorities
- [ ] Provide impact analysis and safety assessment
- [ ] Obtain necessary approvals

**During Migration**:
- [ ] Quarterly progress reports
- [ ] Safety monitoring results
- [ ] Compliance validation

**After Sunset**:
- [ ] Final report confirming safe deprecation
- [ ] Updated safety case and compliance artifacts

---

## Special Cases

### Emergency Deprecation (Security/Safety)

**Trigger**: Critical security vulnerability or safety issue

**Process**:
1. **Immediate** (Day 0): Disable feature via kill-switch
2. **24 hours**: Notify all customers with workaround
3. **1 week**: Provide migration path or fixed version
4. **30 days**: Complete migration with intensive support
5. **60 days**: Sunset (compressed timeline)

### Unpopular Feature Sunset

**If usage <5% of customers**:
- Shorter timeline acceptable (6 months)
- Less intensive support
- Focus resources on high-impact users

---

## Variant Budget Impact

### Code Removal Guidelines

**When deprecating**:
- Remove associated code, tests, and configurations
- Improve variant budget health (reduce delta)
- Celebrate variant cost reduction in monthly report

**Before removal**:
- Ensure no hidden dependencies
- Verify feature flags properly isolate code
- Test with feature disabled end-to-end

---

## Post-Sunset Review

### Retrospective (Within 2 weeks of sunset)

**Questions to answer**:
- Did we meet the timeline?
- How many customers successfully migrated?
- What issues arose during migration?
- How effective was our communication?
- What would we do differently?

**Metrics**:
- Migration completion rate
- Support ticket volume
- Customer satisfaction (survey)
- Incidents during deprecation
- Variant budget improvement

**Learnings**:
- Update this playbook with improvements
- Share learnings in PM CoP Craft Review

---

## Deprecation Checklist

**Planning**:
- [ ] Deprecation rationale documented
- [ ] Timeline established
- [ ] Migration path designed
- [ ] Success criteria defined
- [ ] Affected customers identified

**Execution**:
- [ ] Announcement published
- [ ] Migration guide created
- [ ] Telemetry instrumented
- [ ] Support channels established
- [ ] Regulator notifications (if required)

**Monitoring**:
- [ ] Usage tracking active
- [ ] Migration dashboard live
- [ ] Support tickets tracked
- [ ] Customer outreach scheduled

**Completion**:
- [ ] ≥95% customers migrated
- [ ] Support volume normalized
- [ ] Feature removed from codebase
- [ ] Documentation updated
- [ ] Post-sunset review completed

---

**This playbook ensures deprecations are executed professionally, safely, and with minimal customer disruption while reducing technical debt and variant budget.**

