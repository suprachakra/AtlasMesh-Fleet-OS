# ADR-0005: Map-Source-Agnostic Architecture

* **Status:** Accepted
* **Date:** 2025-09-14
* **Proposer:** Mapping & Localization Team

## Context

AtlasMesh Fleet OS operates across diverse environments (urban, industrial, defense) with varying map availability, quality, and update frequency. Different regions and sectors may require different map providers or custom maps. We need a flexible approach that can utilize multiple map sources while maintaining safety and performance.

## Decision

We will implement a **map-source-agnostic architecture** with the following key components:

1. **Map Data Contract** - A formal specification that defines:
   - Required semantic elements (lanes, rules, turn restrictions, elevations)
   - Quality metrics and confidence indicators
   - Update mechanisms and versioning
   - Provenance tracking and validation

2. **Map Provider Adapters** - Standardized interfaces for:
   - Commercial providers (HERE, ESRI)
   - Open source (OSM)
   - Custom site maps (mines, ports, bases)
   - Converting between provider formats and our internal representation

3. **Map Fusion System** - A conflict resolution framework that:
   - Combines data from multiple sources with provenance tracking
   - Applies credibility weighting based on source and freshness
   - Resolves conflicts with clear policies
   - Provides human-in-the-loop escalation for critical conflicts

4. **Versioned Road Graph** - A system that:
   - Maintains immutable, versioned map snapshots
   - Associates each trip with a specific map version
   - Enables reproducibility and audit trails
   - Supports rollback in case of map quality issues

## Guardrails

1. **Provenance Tracking** - All map data must have source attribution and timestamps
2. **Credibility Over Freshness** - Prefer reliable data over newer but unverified updates
3. **Conflict Resolution** - Clear policies for handling contradictory map information
4. **Safety Boundaries** - Prevent operation in areas with insufficient map quality

## KPIs/SLOs

1. Route correctness ≥99.x% on golden corridors
2. ETA median error within target band for each sector
3. Map conflict MTTResolve ≤24h for site maps, ≤48h for city maps

## Implementation Paths

1. `/services/map-service/*` - Core map management and versioning
2. `/services/routing/*` - Route planning using map data
3. `/rules/policy/*` - Policies for map usage and conflict resolution

## Consequences

### Positive

1. Enables operation across regions with different map providers
2. Reduces dependency on any single map source
3. Allows for custom maps in specialized environments
4. Provides clear audit trails for safety cases

### Negative

1. Increased complexity in map management (+15%)
2. Need for human verification in conflict cases
3. Potential for inconsistent user experience across regions
4. Additional storage and processing requirements

## Kill-Switch Criteria

If repeated contract breaches occur for a specific provider, that provider will be disabled until issues are resolved.

## Alternatives Considered

1. **Single map provider** - Rejected due to coverage limitations and lock-in risk
2. **Custom maps only** - Rejected due to scaling limitations and maintenance burden
3. **Provider-specific integrations** - Rejected due to code duplication and maintenance
