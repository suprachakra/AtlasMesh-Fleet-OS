# ADR-0003: Sector-Agnostic Architecture

* **Status:** Accepted
* **Date:** 2025-09-14
* **Proposer:** Product Architecture Team

## Context

AtlasMesh Fleet OS aims to serve multiple sectors (Defense, Mining, Logistics/Ports, Ride-hail) without maintaining separate codebases or forks for each sector. Each sector has unique requirements, terminology, workflows, and regulatory considerations that must be accommodated within a unified platform.

## Decision

We will implement a **sector-agnostic architecture** with the following key components:

1. **Shared Core Platform** - A unified codebase that:
   - Provides common autonomy, fleet management, and safety capabilities
   - Maintains sector-neutral terminology in core components
   - Exposes extension points for sector-specific behaviors

2. **Policy Engine** - A rules-based system that:
   - Defines sector-specific constraints and behaviors
   - Implements regulatory requirements as code
   - Provides audit trails for compliance verification
   - Enables runtime configuration without code changes

3. **Sector Overlays** - Configuration-driven customizations:
   - UI/UX adaptations (terminology, workflows, visualizations)
   - KPI definitions and dashboards
   - Integration points with sector-specific systems
   - Documentation and training materials

4. **Evidence Mapping** - Sector-specific compliance frameworks:
   - Maps generic evidence collection to sector standards
   - Generates appropriate documentation for audits
   - Validates compliance through automated checks

## Guardrails

1. **Code Sharing** - ≥90% code shared across sectors
2. **UI Customization** - Sector-specific UI code ≤5% of total
3. **No Forks** - Differences expressed via policies and configuration, not code forks
4. **Validation** - Sector acceptance scenarios must pass for all supported sectors

## KPIs/SLOs

1. ≥90% code shared across sectors
2. Sector-specific UI code ≤5% of total
3. Sector SLAs met in ≥2 sectors simultaneously each release

## Implementation Paths

1. `/rules/policy/*` - Policy definitions for sector-specific constraints
2. `/configs/sectors/*` - Sector configuration overlays
3. `/ui/design-system/*` - UI components with sector-specific tokens

## Consequences

### Positive

1. Reduced maintenance burden through shared codebase
2. Faster feature delivery across sectors
3. Cross-sector learning and optimization
4. Simplified release management

### Negative

1. Initial complexity in designing for multiple sectors (+10-15%)
2. Potential for compromise solutions that aren't optimal for any sector
3. Risk of terminology confusion across sectors
4. Increased testing matrix complexity

## Kill-Switch Criteria

If a sector overlay causes >10% regression in other sectors' SLOs for 2 consecutive releases, we will freeze overlay expansion for that sector.

## Alternatives Considered

1. **Sector-specific forks** - Rejected due to maintenance burden and feature lag
2. **Separate products per sector** - Rejected due to duplication and divergence
3. **Single sector focus initially** - Considered but rejected due to strategic requirements
