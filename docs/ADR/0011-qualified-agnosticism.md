# ADR-0011: Qualified Agnosticism Framework

| | |
|---|---|
| **Status** | Accepted |
| **Date** | 2025-10-02 |
| **Proposer** | Platform Architecture Team |
| **Approvers** | CTO, VP Engineering, Safety Lead |
| **Supersedes** | N/A |
| **Related ADRs** | ADR-0001, ADR-0002, ADR-0003, ADR-0004 |

## Context

The original vision for AtlasMesh Fleet OS included "vehicle-agnostic, sector-agnostic, and platform-agnostic" capabilities. However, early implementations revealed that universal agnosticism is constrained by five fundamental realities:

1. **Physics & Actuation**: Control loops must be re-parametrized per vehicle profile
2. **Safety Certification**: ISO 26262/SOTIF evidence is model-specific
3. **ODD Boundaries**: Scenarios differ across sectors and environments
4. **Data & Sensor Packs**: Perception depends on specific sensor constellations
5. **Regulation & Security**: Jurisdictional differences force policy-level divergence

We need an engineering-grounded approach that acknowledges these constraints while maximizing code reuse and operational flexibility.

## Decision

We adopt **"qualified agnosticism"** - a pragmatic approach that achieves agnosticism **within bounded constraints** through:

### 1. Agnostic By Contract
All abstraction boundaries are defined by explicit, testable contracts:
- **Vehicle HAL Contract**: Motion, sensor, diagnostic, safety interfaces
- **Sector Policy Contract**: Policy evaluation, overlay definitions, evidence mapping
- **Platform Adapter Contract**: Storage, messaging, security, compute interfaces

### 2. Variant Budget Enforcement
Automated tracking and enforcement of code/test delta limits:
- **Core code delta ≤5%** per tenet (vehicle/sector/platform)
- **Test-time delta ≤25%** per tenet
- **CI blocks releases** that exceed budgets
- **Change Control Board (CCB)** for exceptions

### 3. Bounded Feasibility
- **Vehicle-Agnostic**: Class/model-bounded with certified profiles (Feasibility: **Medium-High 70%**)
- **Sector-Agnostic**: Policy overlays targeting ≥90% code reuse (Feasibility: **High 85%**)
- **Platform-Agnostic**: K8s-first with provider adapters (Feasibility: **High 90%**)

### 4. Safety-First Certification
- Per-model ISO 26262/SOTIF validation
- Automated evidence generation with process reuse
- HiL + track testing per vehicle profile
- Jurisdiction-specific compliance mapping

## Guardrails

### Technical Constraints
- **No universal claims**: Bounded to realistic vehicle classes, sectors, platforms
- **Physics respected**: Vehicle profiles capture real dynamics and actuation constraints
- **Certification reality**: Model-specific evidence with automated generation
- **Measurable limits**: Variant budgets with automated CI/CD enforcement

### Enforcement Mechanisms
- **Pre-commit hooks**: Local variant budget validation
- **CI gates**: Build blocking when budgets exceeded
- **CCB workflow**: Formal exception handling process
- **Conformance testing**: Multi-dimensional validation before deployment

## KPIs/SLOs

### Technical Success Metrics
- **Fleet Availability**: ≥99.0% across ≥3 vehicle classes
- **Assist Rate**: ≤2/1,000 km across all sectors
- **Policy Runtime**: P99 ≤40ms for all policy evaluations
- **Code Reuse**: ≥90% shared code across sectors
- **Platform Conformance**: 100% across target platforms
- **Variant Budget Compliance**: Respected for 3 consecutive releases

### Business Impact Metrics
- **Time to Market**: -50% for new vehicle class onboarding
- **Development Cost**: -60% for new sector expansion
- **Operational Efficiency**: +30% improvement in fleet utilization
- **Compliance Cost**: -40% reduction in certification overhead

## Implementation

### Phase 1: Vehicle-Agnostic Foundation
- [x] Vehicle HAL service implementation
- [x] Vehicle profile system (Terminal Tractor, Mine Haul examples)
- [x] Safety monitoring and constraint enforcement
- [ ] 3-vehicle demonstration (UTV, Terminal Tractor, Mine Haul)

### Phase 2: Sector Overlays
- [x] Policy engine with Rego support
- [x] Sector overlay service framework
- [ ] Defense sector overlay (priority 1)
- [ ] Mining sector overlay (priority 2)
- [ ] Logistics sector overlay (priority 3)
- [ ] Ride-hail sector overlay (priority 4)

### Phase 3: Platform Abstraction
- [x] Platform adapter interfaces
- [x] Storage adapter skeleton (AWS/Azure/GCP)
- [ ] Messaging adapter implementation
- [ ] Security adapter implementation
- [ ] Conformance suite validation

### Phase 4: Enforcement & Validation
- [x] Variant budget service
- [x] Conformance testing framework
- [x] Evidence generation automation
- [ ] CCB workflow integration
- [ ] Continuous budget monitoring

## Consequences

### Positive
- **Realistic Approach**: Engineering-grounded rather than marketing-driven
- **Measurable Success**: Clear KPIs and programmatic proof points
- **Safety Maintained**: Certification requirements integrated throughout
- **Code Reuse**: Significant reduction in development and maintenance costs
- **Scalability**: Proven path for expanding to new vehicles, sectors, platforms

### Negative
- **Increased Complexity**: Multi-dimensional testing and validation overhead
- **Certification Cost**: Per-model safety certification remains necessary
- **Organizational Alignment**: Requires cross-department coordination and buy-in
- **Initial Investment**: +40% development effort over single-target system
- **Ongoing Overhead**: +15% per release for multi-dimensional validation

### Risks & Mitigations
| Risk | Impact | Mitigation |
|------|--------|------------|
| Certification bottleneck | High | Automated evidence generation |
| Variant budget creep | High | Automated CI/CD enforcement gates |
| Test matrix explosion | Medium | Prioritized test scenario selection |
| Platform lock-in | Medium | Contract-driven adapter interfaces |

## Alternatives Considered

### Alternative 1: Universal Agnosticism
**Decision**: Rejected
**Reason**: Ignores fundamental constraints of physics, safety certification, and regulatory requirements. Would result in unsafe or non-compliant system.

### Alternative 2: No Agnosticism (Per-Customer Forks)
**Decision**: Rejected
**Reason**: Does not scale; high maintenance overhead; difficult to maintain safety and compliance across forks.

### Alternative 3: Limited Agnosticism (Single Dimension)
**Decision**: Rejected
**Reason**: Leaves significant business value on the table; qualified agnosticism across all three dimensions provides maximum ROI.

## Compliance & Monitoring

### Regulatory Compliance
- **ISO 26262**: Per-model functional safety evidence (automated generation)
- **SOTIF**: Safety of intended functionality validation per vehicle class
- **UN R155**: Cybersecurity requirements compliance across all deployments
- **UN R156**: Software update security validation for OTA systems

### Monitoring Strategy
- **Variant Budget Dashboard**: Real-time code delta tracking per dimension
- **Conformance Dashboard**: Multi-dimensional test matrix status
- **Evidence Dashboard**: Compliance artifact generation and validation
- **Performance Monitoring**: SLO tracking across all qualified agnosticism metrics

---

**This ADR establishes qualified agnosticism as the foundational architectural principle of AtlasMesh Fleet OS, providing a realistic, engineering-grounded path to achieving meaningful vehicle, sector, and platform independence.**
