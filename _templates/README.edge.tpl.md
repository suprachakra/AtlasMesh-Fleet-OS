# {{PACKAGE_NAME}} (Edge/ROS2)

> **TL;DR:** {{PACKAGE_DESCRIPTION}}

<!-- BEGIN:DIAGRAM-STRIP -->
<p align="center">
  <a href="{{ROS2_GRAPH_PATH}}">
    <img src="{{ROS2_GRAPH_PATH}}" alt="{{PACKAGE_NAME}} ROS2 node graph" width="32%" />
  </a>
  <a href="{{LIFECYCLE_SEQUENCE_PATH}}">
    <img src="{{LIFECYCLE_SEQUENCE_PATH}}" alt="{{PACKAGE_NAME}} lifecycle sequence" width="32%" />
  </a>
  <a href="{{EDGE_DFD_PATH}}">
    <img src="{{EDGE_DFD_PATH}}" alt="Edge-to-cloud data flow" width="32%" />
  </a>
</p>

**🤖 Edge Overview:** *ROS2 nodes* • *Lifecycle flow* • *Edge↔Cloud data*
<!-- END:DIAGRAM-STRIP -->

## 🔗 **ROS2 Contracts**

| Type | Topic/Service | Message Type | QoS |
|------|---------------|--------------|-----|
| **Publisher** | `{{PUB_TOPIC_1}}` | `{{PUB_MSG_TYPE_1}}` | `{{PUB_QOS_1}}` |
| **Subscriber** | `{{SUB_TOPIC_1}}` | `{{SUB_MSG_TYPE_1}}` | `{{SUB_QOS_1}}` |
| **Service** | `{{SERVICE_1}}` | `{{SERVICE_TYPE_1}}` | `{{SERVICE_QOS_1}}` |

## ⚡ **ROS2 Node Lifecycle**

```mermaid
sequenceDiagram
    autonumber
    participant Launch as Launch System
    participant Node as {{PACKAGE_NAME}}
    participant ROS2 as ROS2 Core
    participant HW as Hardware
    
    Launch->>Node: ros2 launch {{PACKAGE_NAME}}
    Note right of Launch: {{LAUNCH_DESCRIPTION}}
    
    Node->>ROS2: Initialize node
    Note right of Node: {{INIT_DESCRIPTION}}
    
    Node->>HW: Setup hardware interfaces
    Note right Node: {{HARDWARE_SETUP}}
    
    loop Every {{LOOP_FREQUENCY}}
        HW->>Node: Sensor data
        Node->>ROS2: Publish {{PRIMARY_TOPIC}}
        Note right of Node: {{PROCESSING_DESCRIPTION}}
    end
    
    Note over Launch,HW: {{RUNTIME_SUMMARY}}
```

## 🚀 **Quick Start**

### **Development (Local)**
```bash
# Build ROS2 package
colcon build --packages-select {{PACKAGE_NAME}}

# Source workspace
source install/setup.bash

# Launch node
ros2 launch {{PACKAGE_NAME}} {{LAUNCH_FILE}}

# Monitor topics
ros2 topic echo {{PRIMARY_TOPIC}}
```

### **Edge Deployment**
```bash
# Deploy to vehicle
make deploy.edge.{{PACKAGE_SLUG}} VEHICLE_ID={{VEHICLE_ID}}

# Check node status
ros2 node list | grep {{PACKAGE_NAME}}

# View logs
ros2 log view {{PACKAGE_NAME}}
```

## 📈 **Performance & Safety**

| Metric | Target | Current | Safety Impact |
|--------|--------|---------|---------------|
| **Loop Rate** | {{LOOP_RATE_TARGET}} | {{LOOP_RATE_CURRENT}} | {{SAFETY_IMPACT_RATE}} |
| **Latency** | {{LATENCY_TARGET}} | {{LATENCY_CURRENT}} | {{SAFETY_IMPACT_LATENCY}} |
| **CPU Usage** | {{CPU_TARGET}} | {{CPU_CURRENT}} | {{SAFETY_IMPACT_CPU}} |
| **Memory** | {{MEMORY_TARGET}} | {{MEMORY_CURRENT}} | {{SAFETY_IMPACT_MEMORY}} |

**🚨 Safety Classification:** {{SAFETY_CLASSIFICATION}} | **🛡️ Redundancy:** {{REDUNDANCY_LEVEL}}

## 🏗️ **ROS2 Architecture**

### **Node Dependencies**
```mermaid
graph TB
    subgraph "Input Nodes"
        {{INPUT_NODE_1}}[{{INPUT_NODE_1}}]
        {{INPUT_NODE_2}}[{{INPUT_NODE_2}}]
    end
    
    subgraph "This Package"
        {{MAIN_NODE}}[{{MAIN_NODE}}]
        {{HELPER_NODE}}[{{HELPER_NODE}}]
    end
    
    subgraph "Output Nodes"
        {{OUTPUT_NODE_1}}[{{OUTPUT_NODE_1}}]
        {{OUTPUT_NODE_2}}[{{OUTPUT_NODE_2}}]
    end
    
    {{INPUT_NODE_1}} -->|{{TOPIC_1}}| {{MAIN_NODE}}
    {{INPUT_NODE_2}} -->|{{TOPIC_2}}| {{MAIN_NODE}}
    {{MAIN_NODE}} -->|{{OUTPUT_TOPIC_1}}| {{OUTPUT_NODE_1}}
    {{MAIN_NODE}} -->|{{OUTPUT_TOPIC_2}}| {{OUTPUT_NODE_2}}
    {{MAIN_NODE}} <-->|{{INTERNAL_TOPIC}}| {{HELPER_NODE}}
    
    classDef thisPackage fill:#e1f5fe,stroke:#01579b,stroke-width:3px
    classDef inputNode fill:#f3e5f5,stroke:#4a148c,stroke-width:2px
    classDef outputNode fill:#e8f5e8,stroke:#1b5e20,stroke-width:2px
    
    class {{MAIN_NODE}},{{HELPER_NODE}} thisPackage
    class {{INPUT_NODE_1}},{{INPUT_NODE_2}} inputNode
    class {{OUTPUT_NODE_1}},{{OUTPUT_NODE_2}} outputNode
```

### **Hardware Interfaces**
- **{{HARDWARE_1}}** - {{HARDWARE_1_DESC}}
- **{{HARDWARE_2}}** - {{HARDWARE_2_DESC}}

## 🔧 **Configuration**

### **ROS2 Parameters**
| Parameter | Description | Default | Range |
|-----------|-------------|---------|-------|
| `{{PARAM_1}}` | {{PARAM_1_DESC}} | `{{PARAM_1_DEFAULT}}` | {{PARAM_1_RANGE}} |
| `{{PARAM_2}}` | {{PARAM_2_DESC}} | `{{PARAM_2_DEFAULT}}` | {{PARAM_2_RANGE}} |

### **Launch Arguments**
```xml
<!-- {{LAUNCH_FILE}} -->
<launch>
  <arg name="{{ARG_1}}" default="{{ARG_1_DEFAULT}}" description="{{ARG_1_DESC}}" />
  <arg name="{{ARG_2}}" default="{{ARG_2_DEFAULT}}" description="{{ARG_2_DESC}}" />
</launch>
```

## 🛡️ **Safety & Reliability**

### **Fault Detection**
- **Watchdog Timer:** {{WATCHDOG_TIMEOUT}}
- **Health Monitoring:** {{HEALTH_CHECK_FREQUENCY}}
- **Failsafe Behavior:** {{FAILSAFE_DESCRIPTION}}

### **Edge Security**
- **Node Authentication:** {{NODE_AUTH}}
- **Topic Encryption:** {{TOPIC_ENCRYPTION}}
- **Hardware Security:** {{HARDWARE_SECURITY}}

## 📊 **Monitoring**

### **ROS2 Diagnostics**
```bash
# Node health
ros2 run diagnostic_updater diagnostic_analyzer

# Topic statistics
ros2 topic bw {{PRIMARY_TOPIC}}
ros2 topic hz {{PRIMARY_TOPIC}}

# System resources
ros2 run resource_retriever system_monitor
```

### **Edge Telemetry**
- **Vehicle Gateway:** Publishes to `vehicle.telemetry.{{PACKAGE_NAME}}`
- **Metrics:** CPU, memory, network, hardware status
- **Alerts:** Automatic escalation for safety-critical failures

## 🧪 **Testing**

### **ROS2 Testing**
```bash
# Unit tests
colcon test --packages-select {{PACKAGE_NAME}}

# Integration tests with hardware-in-loop
make test.hil.{{PACKAGE_SLUG}}

# Simulation tests
ros2 launch {{PACKAGE_NAME}} simulation.launch.py
```

### **Safety Validation**
- **FMEA Analysis:** {{FMEA_DOCUMENT}}
- **Hazard Analysis:** {{HAZARD_ANALYSIS}}
- **Certification:** {{CERTIFICATION_STATUS}}

---

**📅 Last Updated:** {{LAST_UPDATED}} | **🤖 ROS2 Version:** {{ROS2_VERSION}}

**🎯 Package Owner:** {{PACKAGE_OWNER}} | **📧 Contact:** {{CONTACT_EMAIL}}
