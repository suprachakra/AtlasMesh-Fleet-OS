# ADR-0007: Communications-Agnostic Architecture

* **Status:** Accepted
* **Date:** 2025-09-14
* **Proposer:** Communications & Connectivity Team

## Context

AtlasMesh Fleet OS operates in environments with varying communications infrastructure, from well-connected urban areas to remote locations with limited connectivity. Different operational domains have different requirements for latency, bandwidth, reliability, and security. We need a flexible approach that can utilize multiple communication technologies while maintaining operational safety and performance.

## Decision

We will implement a **communications-agnostic architecture** with the following key components:

1. **Multi-Path Communications Manager** - A system that:
   - Orchestrates traffic across multiple communication channels
   - Implements intelligent path selection based on message priority, cost, and network conditions
   - Provides seamless handover between communication technologies
   - Manages quality of service across heterogeneous networks

2. **Supported Communication Technologies**:
   - Cellular (LTE/5G)
   - Wi-Fi (infrastructure and ad-hoc)
   - SATCOM (for remote operations)
   - Mesh networking (vehicle-to-vehicle)
   - V2X (for local safety-critical communications)

3. **Offline-First Design** - Core capabilities that:
   - Maintain safe operation without continuous connectivity
   - Store and forward telemetry and evidence data
   - Implement local decision-making with bounded autonomy
   - Gracefully degrade services based on connectivity status

4. **Communications Security Framework** - A system that:
   - Implements end-to-end encryption across all channels
   - Provides mutual authentication for all endpoints
   - Detects and mitigates communication-based attacks
   - Enforces data protection policies regardless of channel

## Guardrails

1. **Policy Budgets** - Defined limits for latency, jitter, and cost per message type
2. **Security Requirements** - mTLS for all communications, PKI for V2X
3. **Offline Operation** - Minimum 45-minute operation without WAN connectivity
4. **Graceful Degradation** - Clear service level reductions based on connectivity status

## KPIs/SLOs

1. Assist RTT p50 <30s, p95 <90s
2. Handover success rate ≥99%
3. Offline continuity ≥45 minutes
4. Evidence upload completion within T+2 hours

## Implementation Paths

1. `/edge/vehicle-agent/*` - On-vehicle communications management
2. `/services/gateway-vehicle/*` - Backend communications orchestration
3. `/services/alerts-incident/*` - Connectivity incident management

## Consequences

### Positive

1. Enables operation across regions with different communications infrastructure
2. Reduces dependency on any single communications provider
3. Improves resilience through communications diversity
4. Supports operation in connectivity-challenged environments

### Negative

1. Increased complexity in communications management (+10%)
2. Additional costs for redundant communications paths (especially SATCOM)
3. Potential for inconsistent user experience based on connectivity
4. More complex testing and validation scenarios

## Kill-Switch Criteria

If assists routinely exceed RTT p95 targets or outages exceed SLA thresholds, we will disable costly communication paths and re-tune policies.

## Alternatives Considered

1. **Single communications technology** - Rejected due to coverage limitations and reliability concerns
2. **Always-connected requirement** - Rejected due to operational constraints in remote areas
3. **Remote driving over WAN** - Rejected due to safety concerns with variable latency
