# PM CoP Framework → AtlasMesh Existing Documentation Mapping

## Executive Summary

This document maps the proposed **Product Management Community of Practice (PM CoP)** framework to **existing AtlasMesh documentation**. The goal is to augment existing files rather than create redundant documents.

## ✅ **What We Already Have (No New Docs Needed)**

| PM CoP Component | Existing Document | Status | Notes |
|------------------|-------------------|--------|-------|
| **Vision & Strategy** | `docs/strategy/01_Executive_Summary_and_Vision.md` | ✅ Complete | Add PM CoP charter section |
| **OKRs & Objectives** | `docs/strategy/03_Objectives_and_Key_Results_OKRs.md` | ✅ Complete | Already has company/workstream OKRs |
| **Metrics Canon** | `docs/strategy/04_Metrics_Canon.md` | ✅ Complete | Comprehensive metric definitions with SoT |
| **Epics & Alignment** | `docs/Technical/02_Epics_And_Strategic_Alignment.md` | ✅ Complete | Epic-to-OKR traceability exists |
| **FRs & NFRs** | `docs/Technical/03_Requirements_FRs_NFRs.md` | ✅ Complete | Add DoR/DoD criteria |
| **Roadmap** | `docs/strategy/12_Product_Roadmap_and_Milestones.md` | ✅ Complete | Add PM CoP cadence |
| **Use Cases** | `docs/prd/use-cases/` | ✅ Complete | 98 use cases across 4 sectors |
| **Cross-Dept Checklist** | `docs/Technical/12_Cross_Department_Checklist.md` | ✅ Complete | Already has cross-functional alignment |
| **GTM Strategy** | `docs/strategy/06_Go_to_Market_Strategy.md` | ✅ Complete | Existing GTM framework |
| **Risk Management** | `docs/strategy/07_Risk_and_Governance.md` | ✅ Complete | Risk register and governance |

## 🔴 **Genuine Gaps (New Docs Required)**

| PM CoP Component | Missing | Required? | Proposed Location |
|------------------|---------|-----------|-------------------|
| **Intake Form Template** | ✅ Missing | **YES** | `docs/strategy/templates/Intake_Form.yaml` |
| **Evidence-First PRD Template** | ✅ Missing | **YES** | `docs/strategy/templates/PRD_Evidence_First.md` |
| **Opportunity Canvas Template** | ✅ Missing | **YES** | `docs/strategy/templates/Opportunity_Canvas.md` |
| **DACI Decision Log** | ✅ Missing | **YES** | `docs/strategy/Decision_Log_DACI.md` |
| **Discovery Playbook** | ✅ Missing | **YES** | `docs/strategy/playbooks/Discovery_Playbook.md` |
| **Prioritization Playbook** | ✅ Missing | **YES** | `docs/strategy/playbooks/Prioritization_Playbook.md` |
| **Beta Program Playbook** | ✅ Missing | **YES** | `docs/strategy/playbooks/Beta_Program_Playbook.md` |
| **Deprecation Playbook** | ✅ Missing | **YES** | `docs/strategy/playbooks/Deprecation_Playbook.md` |
| **30-Day OQ Review Template** | ✅ Missing | **YES** | `docs/strategy/templates/OQ_Review.md` |

## 📝 **Augmentation Plan (Update Existing Docs)**

### **1. docs/strategy/01_Executive_Summary_and_Vision.md**
**Add Section**: "Product Management Community of Practice"
- CoP Charter (1-page)
- Roles and governance structure
- Decision model (DACI)
- Ground rules (10 non-negotiables)

### **2. docs/Technical/03_Requirements_FRs_NFRs.md**
**Add Section**: "Definition of Ready (DoR) & Definition of Done (DoD)"
- DoR checklist for Epics/FRs/NFRs
- DoD checklist for releases
- Evidence-first criteria
- Traceability requirements

### **3. docs/strategy/12_Product_Roadmap_and_Milestones.md**
**Add Section**: "PM CoP Operating Cadence"
- Weekly: Intake & Prioritization
- Bi-weekly: Discovery Reviews
- Monthly: Roadmap & Risk Council, Craft Review
- Quarterly: Strategy/OKR alignment, Outcome Review

### **4. docs/Technical/02_Epics_And_Strategic_Alignment.md**
**Add Section**: "Epic Definition of Ready"
- Evidence pack requirements
- Variant budget estimates
- SLI targets
- Rollout/rollback plans

### **5. docs/strategy/03_Objectives_and_Key_Results_OKRs.md**
**Add Section**: "OKR → Epic → FR/NFR Traceability Flow"
- Formal traceability model
- Verification methodology
- Outcome Quality (OQ) review process

### **6. docs/strategy/04_Metrics_Canon.md**
**Add Section**: "PM CoP Craft Metrics"
- Evidence coverage %
- Traceability completeness
- Roadmap accuracy
- Variant budget compliance
- DoR/DoD adherence

## 🎯 **Implementation Strategy**

### **Phase 1: Augment Existing (No New Files)**
1. Update 6 existing strategy/technical docs with PM CoP sections
2. Add DoR/DoD criteria to requirements doc
3. Add cadence to roadmap doc
4. Add charter to vision doc

### **Phase 2: Create Only Missing Templates (9 New Files)**
1. Intake Form (YAML)
2. Evidence-First PRD Template (MD)
3. Opportunity Canvas (MD)
4. DACI Decision Log (MD)
5. Discovery Playbook (MD)
6. Prioritization Playbook (MD)
7. Beta Program Playbook (MD)
8. Deprecation Playbook (MD)
9. OQ Review Template (MD)

### **Phase 3: Update README**
Add "Product Management Framework" section with:
- One-liner on evidence-first approach
- Links to augmented existing docs
- Links to new templates
- Flow diagram (Intake → Discovery → Definition → Delivery → Evidence)

## 📊 **Proposed Directory Structure (Minimal New Files)**

```
docs/
├── strategy/
│   ├── 01_Executive_Summary_and_Vision.md (AUGMENT: Add PM CoP Charter)
│   ├── 03_Objectives_and_Key_Results_OKRs.md (AUGMENT: Add traceability flow)
│   ├── 04_Metrics_Canon.md (AUGMENT: Add PM craft metrics)
│   ├── 12_Product_Roadmap_and_Milestones.md (AUGMENT: Add PM cadence)
│   ├── Decision_Log_DACI.md (NEW: DACI decision log)
│   ├── playbooks/ (NEW FOLDER)
│   │   ├── Discovery_Playbook.md
│   │   ├── Prioritization_Playbook.md
│   │   ├── Beta_Program_Playbook.md
│   │   └── Deprecation_Playbook.md
│   └── templates/ (NEW FOLDER)
│       ├── Intake_Form.yaml
│       ├── PRD_Evidence_First.md
│       ├── Opportunity_Canvas.md
│       └── OQ_Review.md
└── Technical/
    ├── 02_Epics_And_Strategic_Alignment.md (AUGMENT: Add Epic DoR)
    └── 03_Requirements_FRs_NFRs.md (AUGMENT: Add DoR/DoD)
```

**Total New Files**: 9 (all templates/playbooks)
**Augmented Files**: 6 (existing strategy/technical docs)

---

**This approach maximizes reuse of existing documentation while adding only genuinely necessary templates and playbooks.**

