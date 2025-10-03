# AtlasMesh Fleet OS - API Contracts & SDK System

This directory contains the complete API contract management system for AtlasMesh Fleet OS, including OpenAPI specifications, client SDK generation, contract testing, and version management.

## 📁 Directory Structure

```
api/
├── contracts/
│   ├── registry.yaml              # Central API contract registry
│   ├── v1/                        # Current API version contracts
│   │   ├── trip-service.yaml      # Trip Service OpenAPI spec
│   │   ├── fleet-manager.yaml     # Fleet Manager OpenAPI spec
│   │   ├── dispatch-service.yaml  # Dispatch Service OpenAPI spec
│   │   ├── routing-service.yaml   # Routing Service OpenAPI spec
│   │   ├── policy-engine.yaml     # Policy Engine OpenAPI spec
│   │   ├── auth-service.yaml      # Auth Service OpenAPI spec
│   │   └── api-gateway.yaml       # API Gateway OpenAPI spec
│   └── versions/                  # Historical API versions
│       └── {service}/
│           └── {version}/
├── schemas/                       # Shared schemas and data contracts
├── examples/                      # API usage examples
└── README.md                      # This file

tools/
├── sdk-generator/                 # SDK generation tools
│   ├── generate-go-sdk.js         # Go SDK generator
│   ├── generate-typescript-sdk.js # TypeScript SDK generator
│   ├── generate-python-sdk.js     # Python SDK generator
│   └── templates/                 # SDK templates
├── contract-testing/              # Contract validation tools
│   ├── contract-validator.js      # Contract testing framework
│   ├── tests/                     # Contract test cases
│   └── reports/                   # Validation reports
├── api-versioning/                # Version management tools
│   ├── version-manager.js         # API version manager
│   ├── migrations/                # Migration guides
│   └── notices/                   # Deprecation notices
└── package.json                   # Tool dependencies

sdks/                              # Generated client SDKs
├── go/                           # Go SDK
├── typescript/                   # TypeScript/JavaScript SDK
└── python/                       # Python SDK
```

## 🚀 Quick Start

### 1. Install Dependencies

```bash
# Install tool dependencies
cd tools && npm install

# Install OpenAPI generator (if not already installed)
npm install -g @openapitools/openapi-generator-cli
```

### 2. Validate API Contracts

```bash
# Validate against development environment
make api-contracts-validate

# Validate against staging
make api-contracts-validate-staging

# Validate against production
make api-contracts-validate-prod
```

### 3. Generate Client SDKs

```bash
# Generate all SDKs
make sdk-generate-all

# Or generate specific SDKs
make sdk-generate-go
make sdk-generate-typescript
make sdk-generate-python
```

### 4. Test Generated SDKs

```bash
make sdk-test
```

## 📋 API Contract Registry

The `contracts/registry.yaml` file serves as the central registry for all API contracts. It defines:

- **Service Metadata**: Name, description, version, owner team
- **Versioning Strategy**: Semantic versioning, deprecation policies
- **SDK Configuration**: Package names, repositories, documentation links
- **Contract Testing**: Consumer-provider relationships, test environments
- **Compatibility Matrix**: Service-to-service version compatibility

### Example Registry Entry

```yaml
services:
  trip-service:
    name: "Trip Service"
    description: "Trip lifecycle management service"
    base_url: "http://trip-service:8080"
    contract_path: "/api/contracts/v1/trip-service.yaml"
    current_version: "1.0.0"
    supported_versions: ["1.0.0"]
    owner_team: "operations"
    status: "stable"
```

## 🔄 API Versioning

### Version Management

AtlasMesh Fleet OS uses **semantic versioning** for APIs:

- **Major (X.0.0)**: Breaking changes
- **Minor (0.X.0)**: New features, backward compatible
- **Patch (0.0.X)**: Bug fixes, backward compatible

### Creating New Versions

```bash
# Interactive version creation
make api-version-create

# Or use the CLI directly
node tools/api-versioning/version-manager.js create trip-service minor "Added new trip status"
```

### Deprecating Versions

```bash
# Interactive deprecation
make api-version-deprecate

# Or use the CLI directly
node tools/api-versioning/version-manager.js deprecate trip-service 1.0.0 "Security vulnerability fixed in 1.1.0"
```

### Version Status

```bash
# View all version status
make api-version-status

# List versions for specific service
make api-version-list
```

## 🧪 Contract Testing

The contract testing framework validates:

1. **OpenAPI Specification Validity**
2. **Service Endpoint Availability**
3. **Request/Response Schema Compliance**
4. **Security Requirements**
5. **Cross-Service Compatibility**

### Running Contract Tests

```bash
# Test against development environment
node tools/contract-testing/contract-validator.js development

# Test against staging
node tools/contract-testing/contract-validator.js staging

# Test against production (read-only)
node tools/contract-testing/contract-validator.js production
```

### Contract Test Reports

Reports are generated in `tools/contract-testing/reports/`:

- **JSON Report**: Machine-readable test results
- **HTML Report**: Human-readable dashboard
- **Recommendations**: Actionable improvement suggestions

## 📦 SDK Generation

### Supported Languages

- **Go**: Type-safe client with context support
- **TypeScript**: Modern JavaScript/TypeScript client
- **Python**: Pythonic client with async support

### SDK Features

- **Type Safety**: Generated types from OpenAPI schemas
- **Authentication**: Built-in support for JWT and API key auth
- **Error Handling**: Structured error responses
- **Retry Logic**: Configurable retry policies
- **Logging**: Debug and trace logging
- **Documentation**: Auto-generated docs and examples

### Using Generated SDKs

#### Go SDK

```go
package main

import (
    "context"
    "github.com/atlasmesh/fleet-os-sdk-go"
    "github.com/atlasmesh/fleet-os-sdk-go/config"
)

func main() {
    client := atlasmesh.NewClientWithAPIKey("your-api-key",
        config.WithBaseURL("https://api.atlasmesh.com"),
    )
    
    ctx := context.Background()
    vehicles, err := client.FleetManager.ListVehicles(ctx, nil)
    if err != nil {
        panic(err)
    }
    
    fmt.Printf("Found %d vehicles\n", len(vehicles.Vehicles))
}
```

#### TypeScript SDK

```typescript
import { AtlasMeshClient } from '@atlasmesh/fleet-os-sdk';

const client = new AtlasMeshClient({
  apiKey: 'your-api-key',
  baseURL: 'https://api.atlasmesh.com'
});

const vehicles = await client.fleetManager.listVehicles();
console.log(`Found ${vehicles.vehicles.length} vehicles`);
```

#### Python SDK

```python
from atlasmesh_fleet_os import AtlasMeshClient

client = AtlasMeshClient(
    api_key='your-api-key',
    base_url='https://api.atlasmesh.com'
)

vehicles = await client.fleet_manager.list_vehicles()
print(f"Found {len(vehicles.vehicles)} vehicles")
```

## 🔒 Security & Authentication

### Authentication Methods

1. **JWT Bearer Tokens**: For user-based authentication
2. **API Keys**: For service-to-service communication
3. **mTLS**: For high-security environments

### Security Validation

Contract tests automatically validate:

- Security scheme definitions
- Protected endpoint coverage
- Authentication requirement enforcement
- HTTPS enforcement in production

## 📊 Monitoring & Analytics

### API Usage Metrics

- Request counts by service/version/method
- Response times (P50, P95, P99)
- Error rates by status code
- Version adoption rates

### SLOs (Service Level Objectives)

- **Contract Compliance**: 99.9% of requests match contract
- **Response Time**: P95 < 500ms
- **Availability**: 99.95% uptime
- **Compatibility**: 100% backward compatibility within major versions

## 🛠️ Development Workflow

### Adding a New Service

1. **Create OpenAPI Specification**
   ```bash
   cp api/contracts/v1/template.yaml api/contracts/v1/new-service.yaml
   # Edit the specification
   ```

2. **Register in Registry**
   ```yaml
   # Add to api/contracts/registry.yaml
   services:
     new-service:
       name: "New Service"
       # ... configuration
   ```

3. **Generate SDK**
   ```bash
   make sdk-generate-all
   ```

4. **Validate Contract**
   ```bash
   make api-contracts-validate
   ```

### Making Breaking Changes

1. **Create Major Version**
   ```bash
   make api-version-create
   # Select service: new-service
   # Select type: major
   # Description: Breaking change description
   ```

2. **Update Contract**
   ```bash
   # Edit the new version contract
   vim api/contracts/versions/new-service/2.0.0/contracts/new-service.yaml
   ```

3. **Generate Migration Guide**
   ```bash
   # Automatically generated during version creation
   # Review and enhance: tools/api-versioning/migrations/new-service/1.0.0-to-2.0.0.md
   ```

4. **Deprecate Old Version**
   ```bash
   make api-version-deprecate
   # Follow the timeline in registry.yaml
   ```

## 📚 Best Practices

### OpenAPI Specification

1. **Use Semantic Versioning**: Follow semver for API versions
2. **Comprehensive Documentation**: Include descriptions, examples, and error cases
3. **Consistent Naming**: Use consistent field names across services
4. **Proper HTTP Status Codes**: Use appropriate status codes for different scenarios
5. **Security First**: Define security requirements for all endpoints

### Contract Testing

1. **Test Early and Often**: Run contract tests in CI/CD
2. **Environment Parity**: Test against all environments
3. **Consumer-Driven**: Include consumer expectations in tests
4. **Backward Compatibility**: Validate compatibility between versions

### SDK Generation

1. **Keep Templates Updated**: Maintain SDK templates with best practices
2. **Version Synchronization**: Keep SDK versions in sync with API versions
3. **Documentation**: Generate comprehensive documentation with examples
4. **Error Handling**: Implement consistent error handling patterns

## 🚨 Troubleshooting

### Common Issues

#### Contract Validation Failures

```bash
# Check OpenAPI specification validity
npx swagger-parser validate api/contracts/v1/service-name.yaml

# Validate against live service
curl -f http://service-name:8080/health
```

#### SDK Generation Errors

```bash
# Check OpenAPI generator installation
npx openapi-generator-cli version

# Validate contract before generation
make api-contracts-validate
```

#### Version Compatibility Issues

```bash
# Check version status
make api-version-status

# Review compatibility matrix
cat api/contracts/registry.yaml | grep -A 10 compatibility
```

### Getting Help

- **Documentation**: https://docs.atlasmesh.com/api/
- **Support**: api-team@atlasmesh.com
- **Issues**: Create issue in the repository
- **Slack**: #api-support channel

## 📈 Roadmap

### Upcoming Features

- **GraphQL Support**: GraphQL schema generation and federation
- **gRPC Integration**: Protocol buffer definitions and client generation
- **API Gateway Integration**: Automatic route configuration
- **Performance Testing**: Load testing integration with contract validation
- **Chaos Engineering**: Contract resilience testing
- **Multi-Language Support**: Additional SDK languages (Rust, Java, C#)

### Version History

- **v1.0.0**: Initial API contract system
- **v1.1.0**: SDK generation framework
- **v1.2.0**: Contract testing automation
- **v1.3.0**: Version management system

---

*For more information, see the [AtlasMesh Fleet OS Documentation](https://docs.atlasmesh.com)*
