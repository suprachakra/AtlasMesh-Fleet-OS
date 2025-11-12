## Agnostic Matrix CI Gate

This document describes the CI gate that enforces the Agnostic Decision Matrix requirements across the codebase.

### Purpose

The Agnostic Matrix Gate ensures that all code changes adhere to the principles and guardrails defined in the [Agnostics Decision Matrix](../../strategy/appendices/Agonistics_Decision_Matrix_v1.0.md). It prevents changes that would violate the established boundaries for each agnostic tenet or exceed the overall Variant Budget.

### Gate Requirements

### Variant Budget Enforcement

For each PR, the gate calculates:
- **Core Code Delta**: Percentage of core code changed to support variability
- **Test-Time Delta**: Percentage increase in test execution time due to matrix testing

PRs are blocked if:
- Any single tenet exceeds 5% core code delta
- Any single tenet exceeds 25% test-time delta

### Per-Tenet Validation

#### 1. Vehicle-Agnostic

- Verifies that vehicle-specific code is confined to `/configs/vehicles/*`
- Ensures controller parameters are profile-scoped, not hardcoded
- Validates that core control algorithms don't contain vehicle-specific branches
- Checks that safety bundles are present for affected vehicle classes

#### 2. Platform-Agnostic

- Validates absence of cloud provider-specific API calls in application code
- Ensures infrastructure code uses the abstraction layers
- Verifies that new dependencies don't introduce provider lock-in
- Checks multi-provider CI matrix for the change

#### 3. Sector-Agnostic

- Ensures sector-specific logic is expressed through policies, not code forks
- Validates that UI customizations use the design token system
- Verifies that sector-specific terminology is properly abstracted
- Checks that changes pass acceptance tests for all supported sectors

#### 4. Sensor-Agnostic

- Verifies sensor-specific code is confined to the HAL layer
- Ensures perception models are pack-aware, not sensor-specific
- Validates that sensor fusion can handle degraded sensor conditions
- Checks calibration procedures for affected sensor packs

#### 5. Map-Source-Agnostic

- Ensures map provider-specific code is confined to adapters
- Validates that map data adheres to the contract specification
- Verifies provenance tracking for map updates
- Checks route correctness on golden corridors tests

#### 6. Weather-Source-Agnostic

- Ensures weather source-specific code is confined to adapters
- Validates fusion system handles multiple data sources correctly
- Verifies confidence model operation with test data
- Checks weather-aware policies for consistency

#### 7. Comms-Agnostic

- Ensures communication path-specific code is properly abstracted
- Validates offline operation capabilities
- Verifies security controls across communication channels
- Checks handover scenarios between communication technologies

## Implementation

The gate is implemented as a composite check that:

1. Analyzes code changes using static analysis
2. Executes matrix tests to measure performance impact
3. Validates configuration changes against schemas
4. Verifies policy consistency and coverage

## Usage

The gate runs automatically on all PRs and blocks merging if violations are detected. Developers receive detailed feedback on specific violations and guidance on remediation.

### Override Process

In exceptional cases where a violation is necessary:

1. Developer submits a CCB request with justification
2. CCB reviews impact and approves/rejects the exception
3. If approved, a one-time override token is issued
4. The override and justification are documented in the PR

## Metrics

The gate tracks:

- Violation attempts by type and frequency
- Override requests and approvals
- Trend of variant budget utilization over time
- Impact on build/test times

This data is used to refine the matrix constraints and identify areas where additional abstraction may be needed.
