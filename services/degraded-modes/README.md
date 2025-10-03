# Degraded Modes

> **TL;DR:** Adaptive system behavior service managing graceful degradation under network, sensor, or compute constraints

## 📊 **Architecture Overview**

### ⚠️ **Where it fits** - Resilience Management Hub
```mermaid
graph TB
    subgraph "Constraint Detection"
        NetworkMonitor[🌐 Network Monitor]
        SensorMonitor[📡 Sensor Monitor]
        ComputeMonitor[💻 Compute Monitor]
        ResourceMonitor[📊 Resource Monitor]
    end
    
    subgraph "Degraded Modes Service"
        ConstraintAnalyzer[📊 Constraint Analyzer]
        ModeSelector[🎯 Mode Selector]
        BehaviorAdapter[🔄 Behavior Adapter]
        PerformanceOptimizer[⚡ Performance Optimizer]
        DegradedAPI[🔌 Degraded API]
    end
    
    subgraph "Degradation Strategies"
        ReducedFunctionality[📉 Reduced Functionality]
        OfflineMode[📴 Offline Mode]
        CachedOperations[💾 Cached Operations]
        FallbackBehaviors[🔄 Fallback Behaviors]
    end
    
    subgraph "Adaptive Behaviors"
        ServicePrioritization[🎯 Service Prioritization]
        ResourceReallocation[🔄 Resource Reallocation]
        QualityAdjustment[📊 Quality Adjustment]
        GracefulRecovery[✅ Graceful Recovery]
    end
    
    NetworkMonitor --> ConstraintAnalyzer
    SensorMonitor --> ModeSelector
    ComputeMonitor --> BehaviorAdapter
    ResourceMonitor --> PerformanceOptimizer
    
    ConstraintAnalyzer --> ReducedFunctionality
    ModeSelector --> OfflineMode
    BehaviorAdapter --> CachedOperations
    PerformanceOptimizer --> FallbackBehaviors
    
    ReducedFunctionality --> ServicePrioritization
    OfflineMode --> ResourceReallocation
    CachedOperations --> QualityAdjustment
    FallbackBehaviors --> GracefulRecovery
    
    classDef detection fill:#e8f5e8,stroke:#1b5e20,stroke-width:2px
    classDef degraded fill:#e1f5fe,stroke:#01579b,stroke-width:3px
    classDef strategy fill:#fff3e0,stroke:#f57c00,stroke-width:2px
    classDef behavior fill:#f3e5f5,stroke:#7b1fa2,stroke-width:2px
    
    class NetworkMonitor,SensorMonitor,ComputeMonitor,ResourceMonitor detection
    class ConstraintAnalyzer,ModeSelector,BehaviorAdapter,PerformanceOptimizer,DegradedAPI degraded
    class ReducedFunctionality,OfflineMode,CachedOperations,FallbackBehaviors strategy
    class ServicePrioritization,ResourceReallocation,QualityAdjustment,GracefulRecovery behavior
```

## 📈 **SLOs & Performance**

| Metric | Target | Current |
|--------|--------|---------|
| **Constraint Detection** | <30s | 22s ✅ |
| **Mode Transition Time** | <60s | 45s ✅ |
| **Service Availability** | >95% | 97% ✅ |
| **Recovery Success Rate** | >90% | 93% ✅ |

---

**🎯 Owner:** Resilience Engineering Team | **📧 Contact:** resilience@atlasmesh.com
