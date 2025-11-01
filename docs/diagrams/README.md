# AtlasMesh Fleet OS - Diagrams as Code

This directory contains all architectural diagrams for the AtlasMesh Fleet OS, generated from source-of-truth and enforced via CI/CD.

## 📋 Diagram Coverage Matrix

| View Type | Stakeholder | Source of Truth | Generator | Path | CI Gate |
|-----------|-------------|-----------------|-----------|------|---------|
| **C4 Context (C1)** | Execs/SVPs | `architecture/c4/*.dsl` | Structurizr DSL | `c4/context/` | Manual update on scope changes |
| **C4 Container (C2)** | Eng/SRE | DSL + k8s services | Structurizr + kube-import | `c4/container/` | New service ⇒ fail if not updated |
| **C4 Component (C3)** | Developers | Source code modules | Code analysis → PlantUML | `c4/component/` | Module graph change ⇒ re-gen |
| **Sequence Diagrams** | PM/QA | OpenTelemetry traces | trace2uml | `sequence/` | API/contract change ⇒ re-gen |
| **Data Flow (DFD)** | Data/Privacy | Kafka + OpenAPI | Custom → Mermaid | `dfd/` | New topic/PII ⇒ update required |
| **STRIDE Threat** | Security | DFD + trust boundaries | OWASP PyTM → PlantUML | `threat/` | Public endpoint ⇒ review |
| **UML Class** | Dev/QA | Proto/Avro/OpenAPI | Schema → PlantUML | `uml/` | Schema version ⇒ re-gen |
| **ERD** | Data/DBA | Live DB schema | SchemaSpy/Prisma | `erd/` | Migration ⇒ ERD re-gen |
| **ROS2 Node Graph** | Edge/AV | ros2 graph + launch | ros2graphviz → SVG | `ros2/` | New node/topic ⇒ update |
| **API Topology** | Integrators | OpenAPI specs | openapi-graph | `api/` | OpenAPI diff ⇒ rebuild |

## 🚀 Quick Start

```bash
# Generate all diagrams
make diagrams

# Generate only changed diagrams
make diagrams:changed

# Build docs site with diagrams
make docs

# Preview docs locally
make docs:serve
```

## 🎯 Safety-Critical Diagrams

These diagrams are **mandatory** and must be updated for any safety-related changes:

- `sequence/emergency_stop_*.puml` - Emergency stop command flow
- `sequence/vehicle_command_dispatch_*.puml` - Vehicle command validation and dispatch
- `sequence/autonomy_fallback_*.puml` - Autonomous system fallback procedures
- `threat/vehicle_communication_*.puml` - Vehicle communication threat model
- `dfd/command_validation_*.mmd` - Command validation data flow

## 📁 Directory Structure

```
docs/diagrams/
├── c4/
│   ├── context/           # C4 Level 1 - System context
│   ├── container/         # C4 Level 2 - Container view
│   └── component/         # C4 Level 3 - Component view
├── sequence/              # Sequence diagrams
│   ├── safety-critical/   # Emergency, fallback, critical flows
│   ├── core-flows/        # Normal operation flows
│   └── integration/       # External system interactions
├── dfd/                   # Data Flow Diagrams
│   ├── safety/            # Safety-critical data flows
│   ├── telemetry/         # Telemetry and monitoring
│   └── integration/       # External data flows
├── threat/                # STRIDE threat models
├── uml/                   # UML class diagrams
├── erd/                   # Entity Relationship Diagrams
├── ros2/                  # ROS2 node graphs
├── api/                   # API topology diagrams
├── runtime/               # Runtime topology (K8s, etc.)
└── style/                 # Style guides and templates
```

## 🔧 Tools Used

- **Structurizr DSL** - C4 architecture diagrams
- **PlantUML** - Sequence, class, component diagrams
- **Mermaid** - Data flow diagrams, flowcharts
- **Kroki** - Universal diagram renderer
- **MkDocs Material** - Documentation site
- **Custom generators** - From OpenAPI, traces, schemas

## 📊 Quality Gates

All diagrams must:
- ✅ Be generated from source-of-truth
- ✅ Include version, date, and source reference
- ✅ Follow AtlasMesh style guide
- ✅ Pass CI validation checks
- ✅ Be approved by domain owners (CODEOWNERS)

## 🚨 Safety Requirements

For safety-critical diagrams:
- **MANDATORY**: Must be updated within same PR as code changes
- **REVIEW**: Requires Safety + Security approval
- **TRACEABILITY**: Must link to safety requirements and test cases
- **VERSIONING**: All versions retained for audit trail
