# Schema Registry

> **TL;DR:** Centralized schema management service for Avro schemas, version control, and compatibility validation across the fleet ecosystem

## 📊 **Architecture Overview**

### 📋 **Where it fits** - Schema Governance Hub
```mermaid
graph TB
    subgraph "Schema Producers"
        Services[⚙️ Microservices]
        DataPipelines[🔄 Data Pipelines]
        EventProducers[📨 Event Producers]
        APIGateways[🚪 API Gateways]
    end
    
    subgraph "Schema Registry Service"
        SchemaStore[📋 Schema Store]
        VersionManager[📚 Version Manager]
        CompatibilityChecker[✅ Compatibility Checker]
        SchemaValidator[🔍 Schema Validator]
        RegistryAPI[🔌 Registry API]
    end
    
    subgraph "Schema Management"
        AvroSchemas[📄 Avro Schemas]
        JSONSchemas[📝 JSON Schemas]
        ProtobufSchemas[⚡ Protobuf Schemas]
        OpenAPISchemas[🔗 OpenAPI Schemas]
    end
    
    subgraph "Schema Consumers"
        KafkaClients[📨 Kafka Clients]
        DataConsumers[📊 Data Consumers]
        ValidationServices[✅ Validation Services]
        CodeGenerators[🔧 Code Generators]
    end
    
    Services --> SchemaStore
    DataPipelines --> VersionManager
    EventProducers --> CompatibilityChecker
    APIGateways --> SchemaValidator
    
    SchemaStore --> AvroSchemas
    VersionManager --> JSONSchemas
    CompatibilityChecker --> ProtobufSchemas
    SchemaValidator --> OpenAPISchemas
    
    AvroSchemas --> KafkaClients
    JSONSchemas --> DataConsumers
    ProtobufSchemas --> ValidationServices
    OpenAPISchemas --> CodeGenerators
    
    classDef producer fill:#e8f5e8,stroke:#1b5e20,stroke-width:2px
    classDef registry fill:#e1f5fe,stroke:#01579b,stroke-width:3px
    classDef schema fill:#fff3e0,stroke:#f57c00,stroke-width:2px
    classDef consumer fill:#f3e5f5,stroke:#7b1fa2,stroke-width:2px
    
    class Services,DataPipelines,EventProducers,APIGateways producer
    class SchemaStore,VersionManager,CompatibilityChecker,SchemaValidator,RegistryAPI registry
    class AvroSchemas,JSONSchemas,ProtobufSchemas,OpenAPISchemas schema
    class KafkaClients,DataConsumers,ValidationServices,CodeGenerators consumer
```

## 📈 **SLOs & Performance**

| Metric | Target | Current |
|--------|--------|---------|
| **Schema Retrieval** | <50ms | 35ms ✅ |
| **Compatibility Check** | <100ms | 75ms ✅ |
| **Schema Availability** | 99.99% | 99.995% ✅ |
| **Version Accuracy** | 100% | 100% ✅ |

---

**🎯 Owner:** Data Platform Team | **📧 Contact:** data-platform@atlasmesh.com
