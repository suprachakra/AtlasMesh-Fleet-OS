# Map Data Contract

> **TL;DR:** Map data management service providing Lanelet2/OpenDRIVE integration, provenance tracking, and diff pipeline automation

## 📊 **Architecture Overview**

### 🗺️ **Where it fits** - Map Data Governance Hub
```mermaid
graph TB
    subgraph "Map Data Sources"
        Lanelet2Maps[🗺️ Lanelet2 Maps]
        OpenDRIVEMaps[🛣️ OpenDRIVE Maps]
        OSMData[🌍 OSM Data]
        ProprietaryMaps[🏢 Proprietary Maps]
    end
    
    subgraph "Map Data Contract Service"
        DataAdapter[🔄 Data Adapter]
        ProvenanceTracker[📋 Provenance Tracker]
        DiffProcessor[🔍 Diff Processor]
        VersionManager[📚 Version Manager]
        MapAPI[🗺️ Map API]
    end
    
    subgraph "Data Processing"
        FormatConverter[🔄 Format Converter]
        QualityValidator[✅ Quality Validator]
        ChangeDetector[🔍 Change Detector]
        ConflictResolver[⚖️ Conflict Resolver]
    end
    
    subgraph "Map Services"
        RoutingService[🗺️ Routing Service]
        LocalizationService[📍 Localization Service]
        PlanningService[🎯 Planning Service]
        SimulationService[🎮 Simulation Service]
    end
    
    Lanelet2Maps --> DataAdapter
    OpenDRIVEMaps --> ProvenanceTracker
    OSMData --> DiffProcessor
    ProprietaryMaps --> VersionManager
    
    DataAdapter --> FormatConverter
    ProvenanceTracker --> QualityValidator
    DiffProcessor --> ChangeDetector
    VersionManager --> ConflictResolver
    
    FormatConverter --> RoutingService
    QualityValidator --> LocalizationService
    ChangeDetector --> PlanningService
    ConflictResolver --> SimulationService
    
    classDef source fill:#e8f5e8,stroke:#1b5e20,stroke-width:2px
    classDef contract fill:#e1f5fe,stroke:#01579b,stroke-width:3px
    classDef processing fill:#fff3e0,stroke:#f57c00,stroke-width:2px
    classDef service fill:#f3e5f5,stroke:#7b1fa2,stroke-width:2px
    
    class Lanelet2Maps,OpenDRIVEMaps,OSMData,ProprietaryMaps source
    class DataAdapter,ProvenanceTracker,DiffProcessor,VersionManager,MapAPI contract
    class FormatConverter,QualityValidator,ChangeDetector,ConflictResolver processing
    class RoutingService,LocalizationService,PlanningService,SimulationService service
```

## 📈 **SLOs & Performance**

| Metric | Target | Current |
|--------|--------|---------|
| **Map Data Freshness** | <24h | 18h ✅ |
| **Format Conversion** | <5min | 3.5min ✅ |
| **Quality Validation** | >99% | 99.5% ✅ |
| **Diff Processing** | <10min | 7min ✅ |

---

**🎯 Owner:** Map Data Team | **📧 Contact:** map-data@atlasmesh.com
