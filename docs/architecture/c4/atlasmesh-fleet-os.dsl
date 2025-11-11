workspace "AtlasMesh Fleet OS" "Autonomous Vehicle Fleet Management Platform" {

    model {
        # External Actors
        fleetOperator = person "Fleet Operator" "Manages autonomous vehicle fleets and operations" "User"
        safetyOperator = person "Safety Operator" "Monitors safety-critical systems and handles emergencies" "Safety"
        vehicle = softwareSystem "Autonomous Vehicle" "L4/L5 autonomous vehicle with sensors and compute" "Vehicle"
        uaeGov = softwareSystem "UAE Government Systems" "Regulatory compliance and emergency services" "External"
        weatherService = softwareSystem "Weather Services" "Multi-source weather data providers" "External"
        mapProvider = softwareSystem "Map Data Providers" "HD maps and real-time traffic data" "External"
        
        # AtlasMesh Fleet OS - Main System
        atlasMeshFleetOS = softwareSystem "AtlasMesh Fleet OS" "Vehicle-agnostic, platform-agnostic, sector-agnostic L4 autonomous fleet operating system" {
            
            # Control Center (Web UI)
            controlCenter = container "Control Center" "React-based web application for fleet management and monitoring" "React 18 + TypeScript" "WebApp"
            
            # API Gateway
            apiGateway = container "API Gateway" "Central entry point for all API requests with authentication and routing" "Go + Gin" "API"
            
            # Core Fleet Services
            fleetManager = container "Fleet Manager" "Manages fleets, vehicles, and operational state" "Go + PostgreSQL" "Service"
            vehicleGateway = container "Vehicle Gateway" "Real-time bidirectional communication with vehicles" "Go + WebSocket + Kafka" "Service"
            policyEngine = container "Policy Engine" "Policy-as-code evaluation using OPA/Rego" "Go + OPA" "Service"
            
            # Authentication & Security
            authService = container "Auth Service" "JWT-based authentication and RBAC/ABAC authorization" "Go + Vault" "Service"
            zeroTrustIAM = container "Zero Trust IAM" "SPIFFE/SPIRE service identity and mTLS" "Go + SPIRE" "Service"
            keyManagement = container "Key Management" "Vault-based secrets and certificate management" "HashiCorp Vault" "Service"
            
            # Data & Analytics
            telemetryIngest = container "Telemetry Ingest" "High-throughput vehicle telemetry processing" "Go + Kafka + ClickHouse" "Service"
            analyticsEngine = container "Analytics Engine" "Real-time analytics and fleet insights" "Go + ClickHouse" "Service"
            dataLineage = container "Data Lineage" "Data provenance and lineage tracking" "Go + Neo4j" "Service"
            
            # AI/ML Services
            mlPipeline = container "ML Pipeline" "Machine learning model training and inference" "Python + MLflow" "Service"
            predictiveMaintenance = container "Predictive Maintenance" "RUL prediction and maintenance scheduling" "Python + TensorFlow" "Service"
            digitalTwin = container "Digital Twin" "Vehicle and fleet simulation for testing" "Python + CARLA" "Service"
            
            # Integration Services
            uaeIntegration = container "UAE Integration" "Integration with UAE government systems" "Go + REST" "Service"
            weatherFusion = container "Weather Fusion" "Multi-source weather data fusion with confidence scoring" "Go + HTTP" "Service"
            mapDataContract = container "Map Data Contract" "Lanelet2/OpenDRIVE map data management" "Go + PostGIS" "Service"
            
            # Infrastructure Services
            eventBus = container "Event Bus" "Kafka-based event streaming and messaging" "Apache Kafka" "Infrastructure"
            database = container "Database Cluster" "PostgreSQL cluster for persistent data" "PostgreSQL + TimescaleDB" "Database"
            cache = container "Cache Cluster" "Redis cluster for session and real-time data" "Redis Cluster" "Cache"
            monitoring = container "Monitoring Stack" "Prometheus + Grafana + Jaeger observability" "Prometheus + Grafana" "Infrastructure"
            
            # Edge Stack (On-Vehicle)
            vehicleAgent = container "Vehicle Agent" "On-vehicle ROS2-based control and monitoring" "ROS2 + C++" "Edge"
        }
        
        # Relationships - External
        fleetOperator -> controlCenter "Manages fleets and monitors operations" "HTTPS"
        safetyOperator -> controlCenter "Monitors safety systems and handles emergencies" "HTTPS"
        vehicle -> vehicleGateway "Sends telemetry and receives commands" "WebSocket/MQTT"
        vehicle -> vehicleAgent "Runs edge stack for autonomous operation" "ROS2"
        
        atlasMeshFleetOS -> uaeGov "Reports compliance data and emergency events" "HTTPS/API"
        weatherFusion -> weatherService "Fetches multi-source weather data" "HTTPS/API"
        mapDataContract -> mapProvider "Synchronizes HD map data" "HTTPS/API"
        
        # Relationships - Internal (Control Center)
        controlCenter -> apiGateway "API requests for fleet operations" "HTTPS/REST"
        
        # Relationships - API Gateway
        apiGateway -> authService "Authenticates requests and validates tokens" "gRPC"
        apiGateway -> fleetManager "Fleet and vehicle management operations" "gRPC"
        apiGateway -> vehicleGateway "Real-time vehicle communication" "gRPC"
        apiGateway -> policyEngine "Policy evaluation for decisions" "gRPC"
        apiGateway -> analyticsEngine "Analytics queries and reports" "gRPC"
        
        # Relationships - Core Services
        fleetManager -> database "Stores fleet, vehicle, and operational data" "SQL"
        fleetManager -> cache "Caches frequently accessed data" "Redis Protocol"
        fleetManager -> eventBus "Publishes fleet events" "Kafka"
        fleetManager -> policyEngine "Evaluates fleet policies" "gRPC"
        
        vehicleGateway -> eventBus "Publishes telemetry and command events" "Kafka"
        vehicleGateway -> cache "Caches vehicle state and connections" "Redis Protocol"
        vehicleGateway -> telemetryIngest "Forwards telemetry for processing" "Kafka"
        
        policyEngine -> database "Stores policies and audit logs" "SQL"
        policyEngine -> cache "Caches policy evaluation results" "Redis Protocol"
        
        # Relationships - Security
        authService -> keyManagement "Retrieves secrets and certificates" "Vault API"
        authService -> database "Stores user accounts and roles" "SQL"
        zeroTrustIAM -> keyManagement "Manages service certificates" "Vault API"
        
        # Relationships - Data & Analytics
        telemetryIngest -> database "Stores processed telemetry data" "SQL"
        telemetryIngest -> analyticsEngine "Forwards data for real-time analytics" "Kafka"
        analyticsEngine -> database "Queries telemetry and fleet data" "SQL"
        dataLineage -> database "Tracks data lineage and provenance" "SQL"
        
        # Relationships - AI/ML
        mlPipeline -> database "Accesses training data" "SQL"
        mlPipeline -> predictiveMaintenance "Provides trained models" "gRPC"
        predictiveMaintenance -> fleetManager "Schedules maintenance based on predictions" "gRPC"
        digitalTwin -> vehicleGateway "Simulates vehicle behavior" "gRPC"
        
        # Relationships - Integration
        uaeIntegration -> fleetManager "Synchronizes compliance data" "gRPC"
        weatherFusion -> telemetryIngest "Provides weather context for telemetry" "Kafka"
        mapDataContract -> vehicleGateway "Provides map updates to vehicles" "gRPC"
        
        # Relationships - Infrastructure
        monitoring -> database "Monitors database health" "SQL"
        monitoring -> eventBus "Monitors Kafka health" "JMX"
        monitoring -> cache "Monitors Redis health" "Redis Protocol"
        
        # Relationships - Edge
        vehicleAgent -> vehicleGateway "Communicates with fleet management" "WebSocket/MQTT"
        vehicleAgent -> vehicle "Controls vehicle systems" "CAN/Ethernet"
    }

    views {
        # System Context Diagram (C1)
        systemContext atlasMeshFleetOS "SystemContext" {
            include *
            autoLayout
            title "AtlasMesh Fleet OS - System Context"
            description "High-level view of the AtlasMesh Fleet OS and its interactions with external systems and users"
        }
        
        # Container Diagram (C2) - Full System
        container atlasMeshFleetOS "Containers" {
            include *
            autoLayout
            title "AtlasMesh Fleet OS - Container View"
            description "Container-level view showing all services and their interactions"
        }
        
        # Container Diagram (C2) - Core Services Only
        container atlasMeshFleetOS "CoreServices" {
            include controlCenter apiGateway fleetManager vehicleGateway policyEngine authService
            include database cache eventBus monitoring
            include fleetOperator safetyOperator vehicle
            autoLayout
            title "AtlasMesh Fleet OS - Core Services"
            description "Core services essential for basic fleet operations"
        }
        
        # Container Diagram (C2) - Safety Critical
        container atlasMeshFleetOS "SafetyCritical" {
            include vehicleGateway policyEngine authService zeroTrustIAM vehicleAgent
            include fleetOperator safetyOperator vehicle
            autoLayout
            title "AtlasMesh Fleet OS - Safety Critical Systems"
            description "Safety-critical components for autonomous vehicle operation"
        }
        
        # Container Diagram (C2) - Data Flow
        container atlasMeshFleetOS "DataFlow" {
            include vehicleGateway telemetryIngest analyticsEngine dataLineage
            include database eventBus cache
            include vehicle
            autoLayout
            title "AtlasMesh Fleet OS - Data Flow"
            description "Data processing and analytics pipeline"
        }
        
        styles {
            element "Person" {
                color #ffffff
                fontSize 22
                shape Person
            }
            element "User" {
                background #08427b
            }
            element "Safety" {
                background #d73027
            }
            element "Software System" {
                background #1168bd
                color #ffffff
            }
            element "Vehicle" {
                background #f39c12
                color #ffffff
            }
            element "External" {
                background #999999
                color #ffffff
            }
            element "Container" {
                background #438dd5
                color #ffffff
            }
            element "WebApp" {
                shape WebBrowser
                background #2ecc71
            }
            element "API" {
                background #e67e22
            }
            element "Service" {
                background #3498db
            }
            element "Database" {
                shape Cylinder
                background #27ae60
            }
            element "Cache" {
                shape Cylinder
                background #e74c3c
            }
            element "Infrastructure" {
                background #95a5a6
            }
            element "Edge" {
                background #9b59b6
            }
        }
    }
}
