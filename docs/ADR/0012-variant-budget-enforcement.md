# ADR-0012: Variant Budget Enforcement

| | |
|---|---|
| **Status** | Accepted |
| **Date** | 2025-10-02 |
| **Proposer** | Platform Architecture Team |
| **Approvers** | CTO, VP Engineering |
| **Supersedes** | N/A |
| **Related ADRs** | ADR-0011 (Qualified Agnosticism) |

## Context

Qualified agnosticism requires maintaining code reuse targets across vehicle, sector, and platform dimensions. Without automated enforcement, architectural drift is inevitable - developers will add vehicle-specific logic, sector-specific workflows, and platform-specific dependencies that fragment the codebase.

We need a system that:
1. **Automatically measures** code and test deltas per agnostic dimension
2. **Enforces hard limits** through CI/CD gates
3. **Provides visibility** into variant budget consumption
4. **Enables exceptions** through formal governance process

## Decision

We implement an **automated variant budget enforcement system** with the following components:

### 1. Budget Definitions
```yaml
variant_budgets:
  vehicle_agnostic:
    hard_limit: 5.0%      # Maximum code delta per vehicle class
    soft_limit: 3.0%      # Warning threshold
    trend_limit: 0.5%     # Maximum growth per release
    
  sector_agnostic:
    hard_limit: 5.0%      # Maximum code delta per sector overlay
    soft_limit: 3.0%      # Warning threshold
    trend_limit: 0.5%     # Maximum growth per release
    
  platform_agnostic:
    hard_limit: 5.0%      # Maximum code delta per platform adapter
    soft_limit: 3.0%      # Warning threshold
    trend_limit: 0.5%     # Maximum growth per release
    
  test_complexity:
    hard_limit: 25.0%     # Maximum test delta
    soft_limit: 20.0%     # Warning threshold
    trend_limit: 2.0%     # Maximum growth per release
```

### 2. Delta Analysis Service
- **Git-based analysis**: Compares current branch against baseline
- **Metrics collection**: Lines of code, cyclomatic complexity, function count
- **Exclusion rules**: Profiles and configs excluded from delta calculation
- **Trend tracking**: Historical budget consumption analysis

### 3. CI/CD Integration
- **Pre-commit hooks**: Local validation before commit
- **Pull request gates**: Budget check before merge approval
- **Build gates**: Block builds that exceed hard limits
- **Deployment gates**: Prevent releases with budget violations

### 4. Change Control Board (CCB) Workflow
- **Automatic triggers**: CCB review for budget violations
- **Formal process**: Exception request, impact assessment, approval
- **Remediation plans**: Mandatory refactoring plans for exceptions
- **Audit trail**: All exceptions logged and tracked

## Guardrails

### Budget Calculation
- **Baseline**: `main` branch or last release tag
- **Comparison**: Feature branch or current commit
- **Scope**: Per-dimension analysis (vehicle/sector/platform)
- **Exclusions**: Configuration files, test data, documentation

### Enforcement Levels
1. **Soft Limit (3%)**: Warning in PR comments, metrics logged
2. **Hard Limit (5%)**: Build blocked, CCB review required
3. **Trend Limit (0.5%/release)**: Long-term drift prevention

### Exception Process
1. Developer exceeds budget → Build blocked
2. Developer files CCB exception request
3. CCB reviews impact, alternatives, remediation plan
4. CCB approves with conditions OR rejects
5. Exception logged in budget dashboard

## KPIs/SLOs

| Metric | Target | Measurement |
|--------|--------|-------------|
| **Analysis Time** | <2 minutes | Time to complete delta analysis |
| **False Positive Rate** | <1% | Incorrect budget violations |
| **Budget Accuracy** | >99% | Correctness of delta calculation |
| **Enforcement Coverage** | 100% | All PRs checked before merge |

## Implementation

### Service Architecture
```
services/variant-budget/
├── cmd/main.go                    # Service entry point
├── internal/
│   ├── analyzer/                  # Delta analysis logic
│   │   ├── git.go                 # Git diff analysis
│   │   ├── metrics.go             # Code metrics calculation
│   │   └── exclusions.go          # Exclusion rule engine
│   ├── budget/                    # Budget tracking
│   │   ├── tracker.go             # Budget state management
│   │   └── history.go             # Historical trend analysis
│   ├── enforcer/                  # Policy enforcement
│   │   ├── gates.go               # CI/CD gate logic
│   │   └── ccb.go                 # CCB workflow integration
│   └── server/                    # HTTP/metrics endpoints
│       ├── routes.go              # API routes
│       └── metrics.go             # Prometheus metrics
├── configs/
│   └── budgets.yaml               # Budget policy definitions
└── Dockerfile
```

### API Endpoints
- `POST /api/v1/budget/analyze` - Analyze code delta for budget compliance
- `GET /api/v1/budget/status` - Get current budget status per dimension
- `GET /api/v1/budget/history` - Get historical budget trends
- `GET /api/v1/budget/violations` - Get budget violations and exceptions
- `POST /api/v1/budget/ccb/request` - Submit CCB exception request

### CI/CD Integration
```yaml
# .github/workflows/variant-budget.yml
name: Variant Budget Check
on: [pull_request]

jobs:
  budget-check:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3
        with:
          fetch-depth: 0  # Full history for delta analysis
          
      - name: Analyze Variant Budget
        run: |
          curl -X POST http://variant-budget:8093/api/v1/budget/analyze \
            -d '{"commit_sha": "${{ github.sha }}", "baseline": "main"}'
            
      - name: Check Budget Compliance
        run: |
          STATUS=$(curl http://variant-budget:8093/api/v1/budget/status)
          if echo "$STATUS" | jq -e '.violations | length > 0'; then
            echo "Budget violations detected - blocking merge"
            exit 1
          fi
```

## Consequences

### Positive
- **Automated Enforcement**: No manual code reviews for variant budget compliance
- **Early Detection**: Budget violations caught in CI before merge
- **Visibility**: Real-time dashboard shows budget consumption per dimension
- **Governance**: Formal CCB process for justified exceptions
- **Trend Analysis**: Historical data prevents long-term architectural drift

### Negative
- **CI/CD Overhead**: Additional 1-2 minutes per build for delta analysis
- **False Positives**: May require tuning of exclusion rules and metrics
- **Process Overhead**: CCB reviews add approval latency for exceptions
- **Learning Curve**: Teams must understand variant budget concepts

### Trade-offs
- **Strictness vs Velocity**: Hard limits may slow feature delivery initially
- **Automation vs Judgment**: Automated enforcement may miss context-specific cases
- **Global vs Local**: Single budget for all dimensions vs per-component budgets

## Alternatives Considered

### Alternative 1: Manual Code Reviews
**Decision**: Rejected
**Reason**: Does not scale; inconsistent enforcement; human error prone

### Alternative 2: Post-Release Analysis
**Decision**: Rejected
**Reason**: Too late to prevent technical debt accumulation; expensive to remediate

### Alternative 3: No Budget Limits
**Decision**: Rejected
**Reason**: Inevitable architectural drift; defeats qualified agnosticism goals

## Compliance & Monitoring

### Monitoring
- **Prometheus Metrics**: Budget consumption, violations, exceptions, trends
- **Grafana Dashboard**: Real-time budget status visualization
- **Alerting**: Slack/email notifications for soft limit breaches
- **Reporting**: Weekly/monthly budget compliance reports

### Audit Trail
- All budget analyses logged with commit SHA, timestamp, results
- All CCB exceptions logged with requester, approver, justification
- Historical budget data retained for trend analysis and audits

---

**This ADR establishes automated variant budget enforcement as a critical quality gate for maintaining qualified agnosticism across all dimensions of the AtlasMesh Fleet OS architecture.**

