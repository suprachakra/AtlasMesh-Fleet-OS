# Time Sync Calibration

> **TL;DR:** Precision time synchronization and sensor calibration service using PTP/GNSS for accurate fleet-wide timing and sensor alignment

## 📊 **Architecture Overview**

### ⏰ **Where it fits** - Precision Timing Hub
```mermaid
graph TB
    subgraph "Time Sources"
        GNSS[🛰️ GNSS Satellites]
        PTPMaster[⏰ PTP Master Clock]
        NTPServers[🌐 NTP Servers]
        AtomicClock[⚛️ Atomic Clock Reference]
    end
    
    subgraph "Time Sync Calibration Service"
        TimeOrchestrator[⏰ Time Orchestrator]
        CalibrationEngine[⚖️ Calibration Engine]
        SyncValidator[✅ Sync Validator]
        DriftCompensator[🔄 Drift Compensator]
        TimingAPI[🔌 Timing API]
    end
    
    subgraph "Calibration Targets"
        VehicleSensors[📡 Vehicle Sensors]
        CameraSystems[📷 Camera Systems]
        LiDARSystems[🔍 LiDAR Systems]
        IMUSystems[🧭 IMU Systems]
    end
    
    subgraph "Precision Services"
        SensorFusion[🔄 Sensor Fusion]
        LocalizationService[📍 Localization Service]
        PerceptionPipeline[👁️ Perception Pipeline]
        ControlSystems[🎮 Control Systems]
    end
    
    GNSS --> TimeOrchestrator
    PTPMaster --> CalibrationEngine
    NTPServers --> SyncValidator
    AtomicClock --> DriftCompensator
    
    TimeOrchestrator --> VehicleSensors
    CalibrationEngine --> CameraSystems
    SyncValidator --> LiDARSystems
    DriftCompensator --> IMUSystems
    
    VehicleSensors --> SensorFusion
    CameraSystems --> LocalizationService
    LiDARSystems --> PerceptionPipeline
    IMUSystems --> ControlSystems
    
    classDef source fill:#e8f5e8,stroke:#1b5e20,stroke-width:2px
    classDef timing fill:#e1f5fe,stroke:#01579b,stroke-width:3px
    classDef target fill:#fff3e0,stroke:#f57c00,stroke-width:2px
    classDef service fill:#f3e5f5,stroke:#7b1fa2,stroke-width:2px
    
    class GNSS,PTPMaster,NTPServers,AtomicClock source
    class TimeOrchestrator,CalibrationEngine,SyncValidator,DriftCompensator,TimingAPI timing
    class VehicleSensors,CameraSystems,LiDARSystems,IMUSystems target
    class SensorFusion,LocalizationService,PerceptionPipeline,ControlSystems service
```

## 📈 **SLOs & Performance**

| Metric | Target | Current |
|--------|--------|---------|
| **Time Sync Accuracy** | <1ms | 0.5ms ✅ |
| **Calibration Precision** | <0.1° | 0.05° ✅ |
| **Drift Compensation** | <10ppm | 5ppm ✅ |
| **Sync Availability** | >99.9% | 99.95% ✅ |

---

**🎯 Owner:** Precision Systems Team | **📧 Contact:** precision@atlasmesh.com
