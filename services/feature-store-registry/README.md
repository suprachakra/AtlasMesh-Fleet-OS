# Feature Store Registry

> **TL;DR:** Centralized feature store and model registry for ML lifecycle management, drift detection, and model governance

## 📊 **Architecture Overview**

### 🏪 **Where it fits** - ML Feature & Model Hub
```mermaid
graph TB
    subgraph "Feature Sources"
        VehicleTelemetry[🚗 Vehicle Telemetry]
        OperationalData[⚙️ Operational Data]
        ExternalData[🌐 External Data]
        HistoricalFeatures[📚 Historical Features]
    end
    
    subgraph "Feature Store Registry Service"
        FeatureStore[🏪 Feature Store]
        ModelRegistry[📋 Model Registry]
        DriftDetector[📊 Drift Detector]
        VersionManager[📚 Version Manager]
        RegistryAPI[🔌 Registry API]
    end
    
    subgraph "ML Lifecycle"
        FeatureEngineering[🔧 Feature Engineering]
        ModelTraining[🎓 Model Training]
        ModelValidation[✅ Model Validation]
        ModelDeployment[🚀 Model Deployment]
    end
    
    subgraph "ML Consumers"
        PredictiveMaintenance[🔧 Predictive Maintenance]
        RouteOptimization[🗺️ Route Optimization]
        DemandForecasting[📈 Demand Forecasting]
        AnomalyDetection[🚨 Anomaly Detection]
    end
    
    VehicleTelemetry --> FeatureStore
    OperationalData --> ModelRegistry
    ExternalData --> DriftDetector
    HistoricalFeatures --> VersionManager
    
    FeatureStore --> FeatureEngineering
    ModelRegistry --> ModelTraining
    DriftDetector --> ModelValidation
    VersionManager --> ModelDeployment
    
    FeatureEngineering --> PredictiveMaintenance
    ModelTraining --> RouteOptimization
    ModelValidation --> DemandForecasting
    ModelDeployment --> AnomalyDetection
    
    classDef source fill:#e8f5e8,stroke:#1b5e20,stroke-width:2px
    classDef registry fill:#e1f5fe,stroke:#01579b,stroke-width:3px
    classDef lifecycle fill:#fff3e0,stroke:#f57c00,stroke-width:2px
    classDef consumer fill:#f3e5f5,stroke:#7b1fa2,stroke-width:2px
    
    class VehicleTelemetry,OperationalData,ExternalData,HistoricalFeatures source
    class FeatureStore,ModelRegistry,DriftDetector,VersionManager,RegistryAPI registry
    class FeatureEngineering,ModelTraining,ModelValidation,ModelDeployment lifecycle
    class PredictiveMaintenance,RouteOptimization,DemandForecasting,AnomalyDetection consumer
```

## 📈 **SLOs & Performance**

| Metric | Target | Current |
|--------|--------|---------|
| **Feature Serving** | <10ms | 7ms ✅ |
| **Model Registry** | <100ms | 75ms ✅ |
| **Drift Detection** | <1h | 45min ✅ |
| **Feature Freshness** | <5min | 3min ✅ |

---

**🎯 Owner:** ML Platform Team | **📧 Contact:** ml-platform@atlasmesh.com
