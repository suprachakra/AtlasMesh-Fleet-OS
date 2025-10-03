# ADR-0013: Multi-Dimensional Conformance Testing

| | |
|---|---|
| **Status** | Accepted |
| **Date** | 2025-10-02 |
| **Proposer** | Quality Assurance Team |
| **Approvers** | CTO, VP Engineering, QA Lead |
| **Supersedes** | N/A |
| **Related ADRs** | ADR-0011 (Qualified Agnosticism) |

## Context

Qualified agnosticism introduces multiple dimensions of variation (vehicle × sector × platform), creating a test matrix with hundreds of possible combinations. Traditional testing approaches are insufficient because:

1. **Combinatorial Explosion**: 6 vehicle classes × 4 sectors × 3 platforms = 72 base combinations
2. **Certification Requirements**: Each vehicle model requires safety validation
3. **Conformance Validation**: Need to prove interfaces remain consistent
4. **Evidence Generation**: Regulatory compliance requires automated artifact creation

We need a testing framework that validates qualified agnosticism claims while managing test complexity and execution time.

## Decision

We implement a **multi-dimensional conformance testing framework** with:

### 1. Test Matrix Definition
```yaml
conformance_matrix:
  dimensions:
    vehicles:
      - ClassA_LightIndustrial
      - ClassB_HeavyDuty
      - ClassC_Mining
      - ClassD_Defense
      - ClassE_Passenger
      - ClassF_Transit
      
    sectors:
      - defense
      - mining
      - logistics
      - ride_hail
      
    platforms:
      - azure_eks
      - aws_eks
      - on_prem_k3s
      
  priority_matrix:
    critical:
      - {vehicle: ClassC_Mining, sector: mining, platform: azure_eks}
      - {vehicle: ClassD_Defense, sector: defense, platform: azure_eks}
    high:
      - {vehicle: ClassB_HeavyDuty, sector: logistics, platform: azure_eks}
      - {vehicle: ClassE_Passenger, sector: ride_hail, platform: azure_eks}
    medium:
      - All other combinations with azure_eks
    low:
      - Combinations with aws_eks and on_prem_k3s
```

### 2. Conformance Categories
- **Interface Conformance**: API contracts, data schemas, protocol compliance
- **Behavior Conformance**: Functional equivalence across dimensions
- **Performance Conformance**: SLO compliance (latency, throughput, availability)
- **Safety Conformance**: Certification requirements per vehicle model

### 3. Test Execution Strategy
- **Parallel Execution**: Run independent combinations in parallel
- **Smart Sampling**: Risk-based test selection for non-critical paths
- **Incremental Testing**: Only test affected dimensions on changes
- **Evidence Collection**: Automated artifact generation during test runs

### 4. Evidence Automation
- **Test Execution Logs**: Structured JSON with all test results
- **Performance Metrics**: Latency, throughput, resource utilization
- **Safety Validation**: Constraint verification, emergency response testing
- **Compliance Artifacts**: Regulatory-ready evidence bundles

## Guardrails

### Test Selection Criteria
- **Critical Path**: Always test safety-critical and high-revenue combinations
- **Risk-Based**: Prioritize high-impact, high-probability failure scenarios
- **Coverage Target**: >95% of critical paths, >80% of all combinations
- **Time Budget**: Complete conformance suite in <30 minutes

### Quality Gates
- **Pre-Merge**: Critical path conformance must pass
- **Pre-Deploy**: Full conformance suite must pass for production
- **Continuous**: Regression testing on every commit
- **Periodic**: Full matrix validation weekly

## KPIs/SLOs

| Metric | Target | Current |
|--------|--------|---------|
| **Test Execution Time** | <30 minutes | TBD |
| **Conformance Coverage** | >95% critical paths | TBD |
| **False Positive Rate** | <2% | TBD |
| **Evidence Generation Time** | <5 minutes | TBD |
| **Test Success Rate** | >98% on stable branches | TBD |

## Implementation

### Conformance Test Runner
```python
# testing/conformance/runner.py
import argparse
import json
from pathlib import Path

CONFORMANCE_MATRIX = {
    "vehicles": ["ClassA_LightIndustrial", "ClassB_HeavyDuty", "ClassC_Mining"],
    "sectors": ["defense", "mining", "logistics", "ride_hail"],
    "platforms": ["azure_eks", "aws_eks", "on_prem_k3s"],
}

def run_conformance(test_id, vehicle, sector, platform, dry_run=False):
    # Execute conformance test for specific combination
    # Returns test results and evidence artifacts
    pass

def generate_evidence(results):
    # Generate regulatory-compliant evidence bundle
    # Includes test logs, performance metrics, compliance artifacts
    pass

def main():
    parser = argparse.ArgumentParser(description="Run conformance test matrix")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--priority", choices=["critical", "high", "medium", "low"])
    args = parser.parse_args()
    
    # Execute test matrix based on priority
    # Generate and export evidence bundle
```

### Conformance Service API
- `POST /api/v1/conformance/test` - Execute conformance test suite
- `GET /api/v1/conformance/matrix` - Get test matrix status
- `GET /api/v1/conformance/evidence` - Get certification evidence
- `GET /api/v1/conformance/reports` - Get conformance reports

### Evidence Bundle Structure
```
evidence/
├── conformance_results.json       # Test execution results
├── performance_metrics.json       # Performance data
├── safety_validation.json         # Safety constraint verification
├── compliance_artifacts/          # Regulatory evidence
│   ├── iso26262/                  # ISO 26262 artifacts
│   ├── sotif/                     # SOTIF validation evidence
│   └── un_r155_r156/              # Cybersecurity compliance
└── metadata.json                  # Bundle metadata and signatures
```

## Consequences

### Positive
- **Automated Validation**: No manual cross-dimensional testing required
- **Early Detection**: Conformance failures caught before production
- **Evidence Generation**: Compliance artifacts automatically created
- **Risk Mitigation**: Critical paths always validated
- **Scalability**: Framework scales with new vehicles/sectors/platforms

### Negative
- **CI/CD Time**: Additional 15-30 minutes per build for full suite
- **Infrastructure Cost**: Parallel test execution requires compute resources
- **Test Maintenance**: Matrix grows with each new dimension
- **Complexity**: Developers must understand multi-dimensional validation

### Risks & Mitigations
| Risk | Impact | Mitigation |
|------|--------|------------|
| Test matrix explosion | Medium | Smart sampling and priority-based execution |
| Long CI/CD times | Medium | Parallel execution and incremental testing |
| False conformance failures | High | Robust test design and flake detection |
| Evidence generation failures | High | Validation and retry logic |

## Alternatives Considered

### Alternative 1: Manual Testing
**Decision**: Rejected
**Reason**: Does not scale with matrix size; inconsistent coverage; human error prone

### Alternative 2: Single-Dimension Testing
**Decision**: Rejected
**Reason**: Does not validate cross-dimensional interactions; misses integration failures

### Alternative 3: Production Testing Only
**Decision**: Rejected
**Reason**: Too risky; failures affect customers; expensive to remediate

## Compliance & Monitoring

### Regulatory Compliance
- **ISO 26262**: Automated functional safety evidence per vehicle model
- **SOTIF**: Safety of intended functionality validation artifacts
- **UN R155/R156**: Cybersecurity and OTA update compliance evidence

### Monitoring Strategy
- **Conformance Dashboard**: Real-time test matrix status visualization
- **Trend Analysis**: Historical conformance quality tracking
- **Alert System**: Notifications for conformance failures
- **Evidence Tracking**: Compliance artifact generation and validation status

---

**This ADR establishes multi-dimensional conformance testing as the validation framework for qualified agnosticism, ensuring system reliability and regulatory compliance across all supported dimensions.**

