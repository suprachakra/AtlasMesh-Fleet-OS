# Sensor Data Collector

> **TL;DR:** Multi-sensor data collection service aggregating vehicle sensors, environmental data, and IoT device telemetry

## 📊 **Architecture Overview**

### 📡 **Where it fits** - Sensor Data Hub
```mermaid
graph TB
    subgraph "Sensor Sources"
        VehicleSensors[🚗 Vehicle Sensors]
        EnvironmentalSensors[🌍 Environmental Sensors]
        IoTDevices[📱 IoT Devices]
        WeatherStations[🌤️ Weather Stations]
    end
    
    subgraph "Sensor Data Collector"
        DataAggregator[📊 Data Aggregator]
        SensorCalibration[⚖️ Sensor Calibration]
        QualityAssurance[✅ Quality Assurance]
        DataFusion[🔄 Data Fusion]
        CollectorAPI[📡 Collector API]
    end
    
    subgraph "Data Processing"
        RealTimeProcessing[⚡ Real-time Processing]
        BatchProcessing[📦 Batch Processing]
        AnomalyDetection[🚨 Anomaly Detection]
        DataCompression[🗜️ Data Compression]
    end
    
    subgraph "Storage & Distribution"
        SensorDatabase[(📊 Sensor Database)]
        TimeSeriesDB[(⏰ Time Series DB)]
        DataLake[(🏞️ Data Lake)]
        EventStream[📨 Event Stream]
    end
    
    VehicleSensors --> DataAggregator
    EnvironmentalSensors --> DataAggregator
    IoTDevices --> DataAggregator
    WeatherStations --> DataAggregator
    
    DataAggregator --> SensorCalibration
    SensorCalibration --> QualityAssurance
    QualityAssurance --> DataFusion
    DataFusion --> CollectorAPI
    
    CollectorAPI --> RealTimeProcessing
    CollectorAPI --> BatchProcessing
    CollectorAPI --> AnomalyDetection
    CollectorAPI --> DataCompression
    
    RealTimeProcessing --> SensorDatabase
    BatchProcessing --> TimeSeriesDB
    AnomalyDetection --> DataLake
    DataCompression --> EventStream
    
    classDef sensor fill:#e8f5e8,stroke:#1b5e20,stroke-width:2px
    classDef collector fill:#e1f5fe,stroke:#01579b,stroke-width:3px
    classDef processing fill:#fff3e0,stroke:#f57c00,stroke-width:2px
    classDef storage fill:#f3e5f5,stroke:#7b1fa2,stroke-width:2px
    
    class VehicleSensors,EnvironmentalSensors,IoTDevices,WeatherStations sensor
    class DataAggregator,SensorCalibration,QualityAssurance,DataFusion,CollectorAPI collector
    class RealTimeProcessing,BatchProcessing,AnomalyDetection,DataCompression processing
    class SensorDatabase,TimeSeriesDB,DataLake,EventStream storage
```

## 📈 **SLOs & Performance**

| Metric | Target | Current |
|--------|--------|---------|
| **Data Collection Rate** | 50K samples/s | 42K samples/s ✅ |
| **Sensor Accuracy** | >98% | 99.2% ✅ |
| **Data Completeness** | >99% | 99.5% ✅ |
| **Processing Latency** | <200ms | 150ms ✅ |

---

**🎯 Owner:** IoT Platform Team | **📧 Contact:** iot-platform@atlasmesh.com
