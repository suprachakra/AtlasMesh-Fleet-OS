# ADR-0002: Platform-Agnostic Architecture

* **Status:** Accepted
* **Date:** 2025-09-14
* **Proposer:** Platform Engineering Team
* **Approvers:** CTO, SVP Engineering, SVP Operations
* **Consulted:** Cloud Infrastructure Team, Security Team, DevOps Team
* **Informed:** All Engineering, Product Management
* **Related Issues:** ATLAS-156, ATLAS-187, ATLAS-204
* **Supersedes:** None
* **Superseded by:** None

## Context

AtlasMesh Fleet OS must operate across multiple deployment environments, including various cloud providers and on-premises installations. Customers have diverse infrastructure requirements, security policies, and data residency needs that must be accommodated without compromising functionality or performance.

### Problem Statement

1. Customers have different cloud preferences and existing investments
2. Some sectors (especially defense) require on-premises deployment
3. Vendor lock-in to a single cloud provider creates business risk
4. Cloud-specific features can create inconsistent behavior across deployments
5. Operational complexity increases with multiple deployment targets

### Market & Business Drivers

1. Defense customers require air-gapped and on-premises solutions
2. Mining operations often have limited connectivity and local infrastructure
3. Enterprise customers typically have established cloud provider relationships
4. Regulatory requirements may dictate data residency in specific regions
5. Cost optimization requires flexibility to leverage different pricing models

## Decision

We will implement a **platform-agnostic architecture** with the following key components:

1. **Kubernetes-First Approach** - Standardize on Kubernetes as our primary orchestration platform:
   - Support major managed K8s offerings (EKS, GKE, AKS)
   - Support self-hosted K8s (k3s for edge deployments)
   - Use standard K8s primitives and avoid provider-specific extensions

2. **Infrastructure Abstraction Layer** - Create abstraction layers for core infrastructure services:
   - Database: PostgreSQL (standard or managed)
   - Messaging: Kafka/NATS with provider-agnostic clients
   - Object Storage: S3-compatible API (AWS S3, GCS with interop, MinIO)
   - Observability: OpenTelemetry with pluggable backends
   - Secrets: Hashicorp Vault or cloud KMS via adapter

3. **Deployment Framework** - Unified deployment approach:
   - Terraform modules with provider-specific implementations
   - Helm charts with provider-specific values
   - Kustomize overlays for environment-specific configurations

4. **Conformance Testing** - Provider-specific test suites that validate:
   - Feature compatibility
   - Performance characteristics
   - Security controls
   - Operational procedures

## Guardrails

| Guardrail | Description | Enforcement Mechanism | Violation Response |
| --- | --- | --- | --- |
| **No Proprietary Lock-in** | Avoid dependencies on cloud-specific PaaS offerings | Architecture reviews; dependency scanning; code audits | PR rejection; refactoring requirement; alternative implementation |
| **No Runtime Coupling** | No serverless functions or cloud-specific runtimes | CI validation; deployment templates; code analysis | Architecture review; implementation redesign; service abstraction |
| **No IAM Baking** | Authentication/authorization must be abstracted, not tied to cloud IAM | Security reviews; authentication framework validation | Security remediation; abstraction layer implementation; access model redesign |
| **Deployment Interface** | Single deployment interface (`deploy/terraform`) regardless of target | Deployment pipeline validation; infrastructure as code reviews | Infrastructure refactoring; deployment standardization; template updates |

## KPIs/SLOs

| KPI/SLO | Description | Target | Measurement Method | Reporting Frequency | Owner |
| --- | --- | --- | --- | --- | --- |
| **CI Matrix Success** | Test success across all supported platforms | Green CI on cloud+on-prem matrix per release | CI pipeline results; test coverage analysis | Per release | DevOps Lead |
| **Deployment Time** | Time to deploy a complete environment | Cold start ≤60 minutes | Deployment pipeline metrics; provisioning time tracking | Per deployment | Infrastructure Lead |
| **Performance Consistency** | Latency consistency across platforms | P99 RPC latency budget maintained across providers | Distributed tracing; performance benchmarks | Weekly | Performance Engineering Lead |
| **Feature Parity** | Consistency of features across platforms | 100% for core capabilities | Feature matrix validation; capability testing | Per release | Product Management |
| **Cost Efficiency** | Infrastructure cost relative to baseline | Within 15% of optimal provider | Cost tracking; resource utilization analysis | Monthly | FinOps Lead |

## Implementation Paths

| Component | Path | Description | Responsible Team | Dependencies |
| --- | --- | --- | --- | --- |
| **Infrastructure Code** | `/deploy/terraform/*` | Infrastructure as code with provider modules | Infrastructure | Terraform; provider APIs; resource models |
| **Kubernetes Resources** | `/deploy/helm/*` | Kubernetes manifests and charts | DevOps | Kubernetes; service definitions; configuration schema |
| **SBOM & Attestation** | `/security/sbom/*` | Software bill of materials and attestation | Security | Dependency analysis; signing tools; verification framework |
| **CI/CD Workflows** | `/ci/pipelines/*` | CI/CD workflows for multi-provider testing | DevOps | CI/CD system; test framework; deployment automation |
| **Conformance Tests** | `/tests/conformance/*` | Platform-agnostic validation test suite | QA | Test framework; service contracts; environment profiles |
| **Adapter Framework** | `/adapters/cloud/*` | Provider-specific adapters and abstractions | Platform | Service interfaces; provider SDKs; abstraction models |

### Integration Points

1. **Identity & Access Management** - Integration with different IAM systems
2. **Storage Services** - Abstraction over object storage, block storage, databases
3. **Network Services** - Load balancing, DNS, service discovery
4. **Monitoring & Logging** - Integration with observability platforms
5. **Secret Management** - Secure storage and retrieval of credentials

## Consequences

### Positive

| Consequence | Description | Beneficiaries | Value Proposition |
| --- | --- | --- | --- |
| **Deployment Flexibility** | Enables deployment in various environments | Customers, Sales | Broader market reach; customer satisfaction; competitive advantage |
| **Vendor Independence** | Avoids lock-in to specific cloud providers | Business, Procurement | Negotiation leverage; risk mitigation; strategic flexibility |
| **Data Sovereignty** | Supports data residency requirements | Compliance, Legal, Customers | Regulatory compliance; market access; customer trust |
| **Hybrid/Multi-Cloud** | Allows for mixed infrastructure strategies | Operations, Customers | Optimal resource allocation; resilience; cost optimization |
| **Operational Consistency** | Same tools and processes across environments | Operations, Engineering | Reduced complexity; knowledge transfer; operational efficiency |

### Negative

| Consequence | Description | Mitigation Strategy | Residual Risk |
| --- | --- | --- | --- |
| **Development Overhead** | Initial development overhead (+20%) for abstraction layers | Modular design; reusable components; phased implementation | Medium - Front-loaded cost with long-term benefits |
| **CI Runtime Increase** | Increased CI runtime (+10%) for matrix testing | Parallelization; selective testing; caching strategies | Low - Manageable with infrastructure optimization |
| **Feature Lag** | Potential delay in adopting new cloud capabilities | Feature prioritization; abstraction design; provider roadmaps | Medium - Some capabilities will be delayed or limited |
| **Optimization Limits** | May miss optimization opportunities specific to providers | Performance benchmarking; critical path analysis; selective optimization | Low - Core performance requirements still met |

## Kill-Switch Criteria

If a provider consistently blocks SLOs or increases TCO by >25%, we will downgrade it to Tier-B support (limited features, best-effort maintenance).

## Alternatives Considered

| Alternative | Description | Pros | Cons | Why Rejected |
| --- | --- | --- | --- | --- |
| **Cloud-native approach** | Fully leverage specific cloud provider capabilities | Optimal performance; Advanced features; Simplified development | Vendor lock-in; Limited portability; Customer constraints | Lock-in concerns; Customer requirements for flexibility; Market reach limitations |
| **Lowest common denominator** | Use only features available across all platforms | Maximum portability; Simplified testing; Consistent behavior | Feature limitations; Performance constraints; Competitive disadvantage | Feature limitations too restrictive; Competitive disadvantage; Customer expectations |
| **Multiple codebases per provider** | Maintain separate implementations for each platform | Optimal use of each platform; No compromise on features; Perfect fit | Extreme maintenance burden; Inconsistent behavior; Duplicated effort | Unsustainable maintenance burden; High risk of divergent behavior; Resource constraints |

## Compliance & Regulatory Considerations

1. **Data Residency**: Platform-specific configurations for regional compliance requirements
2. **Security Controls**: Consistent implementation of security controls across platforms
3. **Audit Trails**: Standardized logging and evidence collection regardless of platform
4. **Certification Boundaries**: Clear documentation of responsibility boundaries with cloud providers
5. **Disaster Recovery**: Platform-specific but capability-consistent recovery procedures

## Monitoring & Metrics

| Metric | Description | Target | Warning Threshold | Critical Threshold | Data Source |
| --- | --- | --- | --- | --- | --- |
| **Platform Parity** | Feature and performance consistency across platforms | 100% feature parity; ±10% performance | 95-99% features; ±15% performance | <95% features; >±20% performance | Conformance tests; benchmarks |
| **Deployment Success** | Successful deployments as percentage of attempts | ≥99% | 95-99% | <95% | Deployment pipeline metrics |
| **Adapter Coverage** | Percentage of services with cross-platform support | 100% | 90-99% | <90% | Code analysis; capability matrix |
| **Resource Utilization** | Efficiency of resource usage across platforms | Within 15% of optimal | 15-25% deviation | >25% deviation | Resource monitoring; cost analysis |
| **Operational Incidents** | Platform-specific incidents requiring intervention | <1 per month per platform | 1-3 per month | >3 per month | Incident management system |

## Implementation Milestones

1. **Q1**: Core infrastructure templates for primary cloud provider and Kubernetes
2. **Q2**: Adapter framework and first alternative cloud provider support
3. **Q3**: On-premises deployment capability and conformance test suite
4. **Q4**: Complete platform parity across all target environments

## References

1. Cloud Provider Evaluation Matrix (`docs/architecture/cloud-provider-evaluation.md`)
2. Platform Abstraction Design (`docs/design/platform-abstraction.md`)
3. Kubernetes Deployment Strategy (`docs/operations/kubernetes-deployment.md`)
4. Multi-Cloud Security Model (`docs/security/multi-cloud-security.md`)
