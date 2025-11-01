# Service Code Changes Plan

> **Comprehensive plan for implementing sector-specific configurations and adapters**

## 📋 **Overview**

This document outlines the service code changes needed to implement sector-specific configurations and adapters for AtlasMesh Fleet OS. The changes will enable the platform to support multiple sectors while maintaining a consistent core architecture.

## 🎯 **Phase 1: Core Services (Major Changes - Add Sector-Specific Configurations)**

### **A. Vehicle Agent Service**

#### **Current State**
- Basic vehicle control and navigation
- Generic sensor integration
- Standard localization and perception

#### **Required Changes**
- **Sector-Specific Localization**:
  - Logistics: GPS + IMU fusion (warehouse navigation)
  - Defense: LiDAR SLAM + IMU (GPS-denied environments)
  - Mining: GPS + IMU + wheel odometry (outdoor mines)
  - Ride-Hail: GPS + IMU + HD maps (urban navigation)

- **Sector-Specific Perception**:
  - Logistics: Pallet detection, dock recognition
  - Defense: Threat detection, formation awareness
  - Mining: Blast zone detection, equipment recognition
  - Ride-Hail: Pedestrian detection, traffic sign recognition

- **Sector-Specific Control**:
  - Logistics: Low-speed precision control
  - Defense: Formation control, convoy coordination
  - Mining: Heavy vehicle dynamics, slope handling
  - Ride-Hail: Smooth passenger experience, traffic compliance

#### **Implementation Plan**
1. **Create sector configuration system**
2. **Implement sector-specific modules**
3. **Add configuration loading mechanism**
4. **Update service initialization**
5. **Add sector-specific testing**

### **B. Fleet Manager Service**

#### **Current State**
- Basic fleet coordination
- Generic vehicle management
- Standard resource allocation

#### **Required Changes**
- **Sector-Specific Fleet Management**:
  - Logistics: Warehouse zones, shift management, charging coordination
  - Defense: UGV formations, mission readiness, security monitoring
  - Mining: Equipment tracking, production metrics, maintenance prediction
  - Ride-Hail: Robotaxi coordination, passenger capacity, cleanliness status

- **Sector-Specific Resource Management**:
  - Logistics: WMS integration, dock coordination, operator scheduling
  - Defense: Mission resources, ammunition tracking, security protocols
  - Mining: Equipment coordination, production optimization, safety compliance
  - Ride-Hail: Passenger matching, route optimization, service quality

#### **Implementation Plan**
1. **Create sector-specific fleet management modules**
2. **Implement sector configuration loading**
3. **Add sector-specific resource management**
4. **Update fleet coordination logic**
5. **Add sector-specific testing**

### **C. Dispatch Service**

#### **Current State**
- Basic trip assignment
- Generic routing logic
- Standard optimization

#### **Required Changes**
- **Sector-Specific Dispatch**:
  - Logistics: WMS task integration, multi-stop routing, priority handling
  - Defense: Mission assignment, formation control, threat-based prioritization
  - Mining: Haul cycle optimization, shovel-truck matching, production targets
  - Ride-Hail: Ride matching, dynamic pricing, passenger pooling

- **Sector-Specific Routing**:
  - Logistics: Warehouse-optimized routes, dock coordination
  - Defense: GPS-denied routing, formation-aware paths
  - Mining: Haul road optimization, blast zone avoidance
  - Ride-Hail: Urban routing, traffic compliance, passenger comfort

#### **Implementation Plan**
1. **Create sector-specific dispatch modules**
2. **Implement sector configuration loading**
3. **Add sector-specific routing logic**
4. **Update optimization algorithms**
5. **Add sector-specific testing**

### **D. Control Center UI Service**

#### **Current State**
- Basic vehicle monitoring
- Generic dashboard
- Standard operator interface

#### **Required Changes**
- **Sector-Specific UI Components**:
  - Logistics: Warehouse dashboard, task management, efficiency metrics
  - Defense: Tactical map, mission control, threat monitoring
  - Mining: Mine operations center, production dashboard, safety alerts
  - Ride-Hail: Operations center, passenger support, incident management

- **Sector-Specific Features**:
  - Logistics: WMS integration, dock status, operator scheduling
  - Defense: Mission planning, formation control, security monitoring
  - Mining: Production tracking, equipment status, safety compliance
  - Ride-Hail: Passenger tracking, ride quality, service metrics

#### **Implementation Plan**
1. **Create sector-specific UI components**
2. **Implement sector configuration loading**
3. **Add sector-specific features**
4. **Update UI routing and navigation**
5. **Add sector-specific testing**

## 🎯 **Phase 2: Sector-Specific Adapters (New Files)**

### **A. Logistics Adapters**

#### **WMS Adapters**
- **SAP EWM Adapter** (`services/wms-adapters/sap-ewm/`)
  - Real-time task synchronization
  - Inventory location mapping
  - Pick/place confirmations
  - Error handling and retry logic

- **Manhattan SCALE Adapter** (`services/wms-adapters/manhattan-scale/`)
  - Task management integration
  - Warehouse zone coordination
  - Equipment status synchronization
  - Performance monitoring

- **Oracle WMS Adapter** (`services/wms-adapters/oracle-wms/`)
  - Multi-warehouse support
  - Advanced inventory management
  - Custom workflow integration
  - Scalability optimization

#### **Logistics-Specific Services**
- **Warehouse Navigation Service** (`services/warehouse-navigation/`)
  - GPS-denied navigation
  - Pallet detection and handling
  - Dock coordination
  - Safety protocols

- **Cold Chain Management Service** (`services/cold-chain-management/`)
  - Temperature monitoring
  - Route optimization
  - Compliance tracking
  - Alert management

### **B. Defense Adapters**

#### **Military Command Adapters**
- **NATO Command Adapter** (`services/defense-adapters/nato-command/`)
  - Mission planning integration
  - Command and control protocols
  - Security compliance
  - Audit trail management

- **Encrypted Communications Adapter** (`services/defense-adapters/encrypted-comms/`)
  - Military-grade encryption
  - Secure key management
  - Message authentication
  - Threat detection

#### **Defense-Specific Services**
- **Formation Control Service** (`services/formation-control/`)
  - Multi-UGV coordination
  - Formation maintenance
  - Collision avoidance
  - Communication protocols

- **Mission Planning Service** (`services/mission-planning/`)
  - Waypoint planning
  - Route validation
  - Threat analysis
  - Mission execution

### **C. Mining Adapters**

#### **Mining FMS Adapters**
- **Wenco Adapter** (`services/mining-fms-adapters/wenco/`)
  - Equipment status synchronization
  - Production reporting
  - Location tracking
  - Payload data integration

- **MineStar Adapter** (`services/mining-fms-adapters/minestar/`)
  - Fleet management integration
  - Production optimization
  - Safety compliance
  - Performance monitoring

- **Modular Mining DISPATCH Adapter** (`services/mining-fms-adapters/modular-mining/`)
  - Dispatch optimization
  - Equipment coordination
  - Production planning
  - Cost optimization

#### **Mining-Specific Services**
- **Heavy Vehicle Control Service** (`services/heavy-vehicle-control/`)
  - 400-ton truck control
  - Slope handling
  - Load management
  - Safety protocols

- **Blast Zone Management Service** (`services/blast-zone-management/`)
  - Blast schedule integration
  - Zone detection and avoidance
  - Safety protocols
  - Emergency procedures

### **D. Ride-Hail Adapters**

#### **Passenger Service Adapters**
- **Payment System Adapter** (`services/ride-hail-adapters/payment-system/`)
  - Payment processing
  - Billing integration
  - Refund management
  - Fraud detection

- **Passenger App Adapter** (`services/ride-hail-adapters/passenger-app/`)
  - Ride booking
  - Real-time tracking
  - Communication
  - Feedback collection

#### **Ride-Hail-Specific Services**
- **Passenger Safety Service** (`services/passenger-safety/`)
  - Safety monitoring
  - Emergency response
  - Incident management
  - Compliance tracking

- **Urban Navigation Service** (`services/urban-navigation/`)
  - Traffic compliance
  - Route optimization
  - Passenger comfort
  - Accessibility support

## 🎯 **Implementation Timeline**

### **Phase 1: Core Services (Months 1-6)**
- **Month 1-2**: Vehicle Agent sector-specific configurations
- **Month 3-4**: Fleet Manager sector-specific configurations
- **Month 5-6**: Dispatch Service sector-specific configurations

### **Phase 2: Sector-Specific Adapters (Months 7-12)**
- **Month 7-8**: Logistics adapters (SAP EWM, Manhattan SCALE)
- **Month 9-10**: Mining adapters (Wenco, MineStar)
- **Month 11-12**: Defense adapters (NATO Command, Encrypted Comms)

### **Phase 3: Ride-Hail Adapters (Months 13-18)**
- **Month 13-15**: Ride-hail adapters (Payment, Passenger App)
- **Month 16-18**: Urban navigation and passenger safety services

## 🎯 **Technical Implementation Details**

### **A. Configuration System**

#### **Sector Configuration Structure**
```yaml
# configs/sectors/logistics/config.yaml
sector: logistics
services:
  vehicle-agent:
    localization:
      strategy: gps_imu_fusion
      accuracy: 10cm
    perception:
      pallet_detection: true
      dock_recognition: true
    control:
      max_speed: 3m/s
      precision: 5cm
  fleet-manager:
    warehouse_zones: true
    shift_management: true
    charging_coordination: true
  dispatch:
    wms_integration: true
    multi_stop_routing: true
    priority_handling: true
```

#### **Configuration Loading**
```go
// services/vehicle-agent/internal/config/sector_config.go
func LoadSectorConfig(sector string) (*SectorConfig, error) {
    configPath := fmt.Sprintf("configs/sectors/%s/config.yaml", sector)
    config, err := loadYAMLConfig(configPath)
    if err != nil {
        return nil, fmt.Errorf("failed to load sector config: %w", err)
    }
    return config, nil
}
```

### **B. Service Initialization**

#### **Sector-Aware Service Initialization**
```go
// services/vehicle-agent/cmd/main.go
func main() {
    sector := os.Getenv("ATLASMESH_SECTOR")
    if sector == "" {
        sector = "logistics" // default
    }
    
    config, err := LoadSectorConfig(sector)
    if err != nil {
        log.Fatal("Failed to load sector config:", err)
    }
    
    service := NewVehicleAgentService(config)
    service.Run()
}
```

### **C. Sector-Specific Modules**

#### **Localization Module**
```go
// services/vehicle-agent/internal/localization/module.go
type LocalizationModule struct {
    strategy LocalizationStrategy
    config   *LocalizationConfig
}

func (lm *LocalizationModule) Initialize(sector string, config *LocalizationConfig) error {
    switch sector {
    case "logistics":
        return lm.initializeLogistics(config)
    case "defense":
        return lm.initializeDefense(config)
    case "mining":
        return lm.initializeMining(config)
    case "ride_hail":
        return lm.initializeRideHail(config)
    default:
        return fmt.Errorf("unsupported sector: %s", sector)
    }
}
```

## 🎯 **Testing Strategy**

### **A. Unit Testing**
- **Sector-specific test cases**
- **Configuration loading tests**
- **Service initialization tests**
- **Module-specific tests**

### **B. Integration Testing**
- **Cross-service integration tests**
- **Sector-specific workflow tests**
- **Adapter integration tests**
- **End-to-end scenario tests**

### **C. Performance Testing**
- **Sector-specific performance benchmarks**
- **Configuration switching performance**
- **Adapter performance tests**
- **Scalability tests**

## 🎯 **Success Metrics**

### **A. Technical Metrics**
- **Configuration Loading Time**: <100ms
- **Service Initialization Time**: <5s
- **Adapter Response Time**: <500ms
- **Test Coverage**: >80%

### **B. Business Metrics**
- **Sector Support**: 4 sectors fully supported
- **Adapter Coverage**: 12+ adapters implemented
- **Customer Onboarding**: <2 weeks per sector
- **Deployment Success**: >95% success rate

## 🎯 **Risk Mitigation**

### **A. Technical Risks**
- **Configuration Complexity**: Use YAML-based configuration
- **Service Coupling**: Implement loose coupling with interfaces
- **Adapter Reliability**: Implement retry logic and circuit breakers
- **Performance Impact**: Use lazy loading and caching

### **B. Business Risks**
- **Sector Requirements**: Regular customer validation
- **Competitive Pressure**: Focus on differentiation
- **Resource Constraints**: Phased implementation approach
- **Market Changes**: Flexible architecture design

## 🎯 **Next Steps**

1. **Phase 1 Implementation**: Start with Vehicle Agent sector-specific configurations
2. **Adapter Development**: Begin with Logistics WMS adapters
3. **Testing Framework**: Implement comprehensive testing strategy
4. **Customer Validation**: Validate sector-specific requirements
5. **Deployment Planning**: Plan phased deployment strategy

This plan provides a comprehensive roadmap for implementing sector-specific configurations and adapters while maintaining the core platform's stability and scalability.
