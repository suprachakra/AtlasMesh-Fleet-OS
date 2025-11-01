# ADR-0006: Weather-Source-Agnostic Architecture

* **Status:** Accepted
* **Date:** 2025-09-14
* **Proposer:** Environmental Systems Team

## Context

AtlasMesh Fleet OS operates in diverse and often harsh environments where weather conditions directly impact operational safety and performance. Weather data availability, quality, and timeliness vary significantly across regions and operational domains. We need a flexible approach that can utilize multiple weather data sources while maintaining safety and performance.

## Decision

We will implement a **weather-source-agnostic architecture** with the following key components:

1. **Weather Data Fusion System** - A framework that:
   - Ingests data from multiple sources with different characteristics
   - Assigns confidence scores based on source reliability and freshness
   - Resolves conflicts using configurable policies
   - Provides a unified weather model for downstream consumers

2. **Multi-tier Weather Sources**:
   - **1P**: Local sensors on vehicles and infrastructure
   - **2P**: Site-specific weather stations and feeds
   - **3P**: Commercial meteorological APIs and services
   - Each with defined quality metrics and update frequencies

3. **Confidence Model** - A system that:
   - Evaluates data quality and reliability in real-time
   - Considers source history and environmental factors
   - Provides confidence intervals for weather parameters
   - Guides operational decisions based on uncertainty

4. **Weather-Aware Policies** - Rules that:
   - Define operational constraints based on weather conditions
   - Adapt routing and behavior to environmental factors
   - Implement sector-specific weather responses
   - Provide clear audit trails for weather-related decisions

## Guardrails

1. **Freshness Timeouts** - Maximum age for different types of weather data
2. **Fallback Ladders** - Defined progression when primary sources fail
3. **Confidence Thresholds** - Minimum confidence required for operational decisions
4. **Operator Override** - Mechanisms for human intervention with audit trails

## KPIs/SLOs

1. Weather-related incidents trend ↓ quarter-over-quarter
2. Route hindsight analysis shows positive net utility for weather-based decisions
3. Fusion output availability ≥99.5%

## Implementation Paths

1. `/services/weather-fusion/*` - Core weather data integration
2. `/rules/policy/*` - Weather-related operational policies
3. `/services/routing/*` - Weather-aware route planning

## Consequences

### Positive

1. Enables operation across regions with different weather data availability
2. Reduces dependency on any single weather source
3. Improves safety through better environmental awareness
4. Provides clear audit trails for weather-related decisions

### Negative

1. Increased complexity in weather data management (+10-15%)
2. Additional compute requirements for fusion and confidence modeling
3. Need for site-specific weather instrumentation (CAPEX)
4. Potential for conservative operation in uncertain conditions

## Kill-Switch Criteria

If fusion shows negative utility across 2 consecutive releases, we will pin to the most credible single feed until issues are resolved.

## Alternatives Considered

1. **Single weather provider** - Rejected due to coverage limitations and reliability concerns
2. **Vehicle sensors only** - Rejected due to limited forecasting ability and range
3. **Manual operator weather assessment** - Rejected due to scaling limitations and inconsistency
