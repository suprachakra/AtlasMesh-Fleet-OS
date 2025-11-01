# AtlasMesh Fleet OS - Production Run/Debug Playbook

## 🎯 Executive Summary

This playbook provides comprehensive operational procedures for running, debugging, and maintaining AtlasMesh Fleet OS across all environments. Designed for Ops, Engineering, and QA teams to ensure system reliability, performance, and rapid incident response.

**System Status**: ✅ Production-Ready Architecture | 48 Microservices | Multi-Environment Deployment

---

# 1) Deployment Topology — Parts & Units

## Cloud Infrastructure (Kubernetes - Helm/ArgoCD)

### Core Services Layer
```yaml
# API & Business Logic
- api-gateway           # Entry point, rate limiting, auth
- fleet-manager         # Fleet operations and dispatch
- vehicle-gateway       # Vehicle communication hub
- mission-management          # Trip lifecycle management
- dispatch-service      # Vehicle assignment logic
- routing-service       # Route calculation and optimization
- policy-engine         # Business rules and compliance (OPA/Rego)

# Specialized Services
- weather-fusion        # Multi-source weather aggregation
- predictive-maintenance # RUL prediction and work orders
- telemetry-ingestion      # Real-time data collection
- evidence-engine       # Safety case and audit trails
- ota-manager          # Over-the-air update orchestration
- map-data-contract    # Map versioning and validation
- alerts-incident      # Alert management and escalation

# Integration Services
- uae-government-integration  # ADTA, emergency services APIs
- business-systems-connector           # SAP, Oracle WMS integration
- arabic-localization        # RTL UI and cultural adaptations
```

### Data Plane
```yaml
# Primary Storage
- postgresql-cluster    # Transactional data (Aurora/CockroachDB)
- timescaledb          # Time-series telemetry data
- redis-cluster        # Caching and session management
- clickhouse-cluster   # Hot path analytics
- minio-cluster        # Object storage (S3-compatible)

# Message Bus
- kafka-cluster        # Event streaming and telemetry
- schema-registry      # Avro schema management
- nats-cluster         # Command/control messaging

# Search & Analytics
- elasticsearch        # Log aggregation and search
- neo4j               # Data lineage and relationships
```

### Observability Stack
```yaml
- otel-collector       # OpenTelemetry data collection
- prometheus          # Metrics collection and alerting
- grafana             # Dashboards and visualization
- jaeger              # Distributed tracing
- loki                # Log aggregation
- tempo               # Trace storage
- sentry              # Error tracking and performance
- pagerduty           # Incident management
```

### UI & Frontend
```yaml
- control-center-web   # React SPA for fleet operations
- mobile-app          # React Native field operations
- admin-web           # System administration interface
- cdn-assets          # Static asset delivery (CloudFront/CDN)
```

## Edge Infrastructure (Vehicle)

### Vehicle Agent (ROS2 Stack)
```yaml
# Perception & Localization
- perception-node      # Sensor fusion and object detection
- localization-node    # GNSS/IMU/visual positioning
- mapping-node         # SLAM and map updates

# Planning & Control
- planning-node        # Path planning and behavior trees
- control-node         # Vehicle control and actuation
- decision-node        # Safety arbitration and emergency stops
- sensor-hal          # Hardware abstraction layer

# Communication & Health
- comms-node          # Multi-path connectivity (LTE/5G/Wi-Fi/SAT)
- health-monitor      # System diagnostics and telemetry
- edge-logger         # Local evidence collection
- ota-client          # Update management
```

### Connectivity Layer
```yaml
- cellular-modem       # LTE/5G primary connectivity
- wifi-adapter        # Wi-Fi backup and depot connectivity
- satcom-terminal     # Satellite communication (remote areas)
- v2x-radio           # Vehicle-to-everything communication (PKI)
```

### Evidence & Storage
```yaml
- evidence-cache       # Ring buffer for video/sensor data
- local-storage       # SSD for critical data persistence
- secure-element      # TPM 2.0 for cryptographic operations
```

## Garage PC (Depot Infrastructure)

### Depot Controller
```yaml
- node-controller      # Depot-wide orchestration
- image-cache         # OTA image staging and validation
- disk-health         # Storage monitoring and maintenance
- canary-controller   # Gradual rollout management
- charging-optimizer  # Energy management and scheduling
```

## Simulation & Digital Twin

### Simulation Cluster
```yaml
- carla-cluster       # CARLA simulation instances
- gazebo-cluster      # Gazebo physics simulation
- scenario-runner     # Test scenario orchestration
- digital-twin-gates  # CI/CD validation gates
- record-replay       # Scenario capture and playback
```

---

# 2) Environments & Promotion Gates

| Environment | Purpose | Data Source | Promotion Gates | SLO Requirements |
|-------------|---------|-------------|-----------------|------------------|
| **Dev** | Local development | Faker + fixtures | Unit tests ≥95%, Contract tests pass | N/A |
| **Sim-Cloud** | CI + simulation | Synthetic + replays | Twin gates ≥95%, Performance budgets met | N/A |
| **Edge-in-Loop** | Hardware-in-loop testing | Sim + real sensors | Safety checks pass, Control loop p95 <50ms | N/A |
| **Staging** | Full stack integration | Masked prod + seeded | E2E slices green, Security scan pass, A11y WCAG 2.2 AA | 99.0% uptime |
| **Preprod/Canary** | Limited production traffic | Live (5-10% traffic) | SLOs stable ≥48h, Error budget intact | 99.5% uptime |
| **Production** | Full customer traffic | Live | Executive approval, Rollback plan armed | 99.9% uptime |

### Gate Criteria Details

#### Dev → Sim-Cloud
- All unit tests pass (≥95% coverage)
- Contract tests validate API compatibility
- Static analysis (SonarQube) quality gate
- Security scan (no critical vulnerabilities)

#### Sim-Cloud → Edge-in-Loop
- Digital twin scenarios pass (≥95% success rate)
- Performance budgets met (API p95 <300ms, UI TTI <3s)
- Chaos engineering tests pass
- Load testing validates capacity

#### Edge-in-Loop → Staging
- Hardware compatibility validated
- Safety system tests pass
- Control loop latency within limits
- Sensor fusion accuracy validated

#### Staging → Preprod
- End-to-end vertical slices pass
- Security penetration testing complete
- Accessibility compliance verified
- Performance benchmarks met

#### Preprod → Production
- SLOs maintained for 48+ hours
- Error budget consumption <50%
- Business stakeholder approval
- Incident response plan validated

---

# 3) How to Run Slices

## Local Development (Docker Compose/Tilt)

### Thin Vertical Slice
```bash
# Start core services
make dev-stack

# Services included:
# - api-gateway (port 8080)
# - fleet-manager (port 8081)
# - vehicle-gateway (port 8082)
# - policy-engine (port 8083)
# - telemetry-ingestion (port 8086)
# - postgresql (port 5432)
# - redis (port 6379)
# - control-center-ui (port 3000)

# Seed test data
make seed-fixtures trips=200 vehicles=100 fleets=5

# Start simulation feed
sim-feeder --profile yard-tractor --rate 10Hz --vehicles 10
```

### Full Local Stack
```bash
# Start all services
docker-compose -f docker-compose.yml -f docker-compose.dev.yml up -d

# Verify health
make health-check

# Access points:
# - Control Center: http://localhost:3000
# - API Gateway: http://localhost:8080
# - Grafana: http://localhost:3001
# - Jaeger: http://localhost:16686
```

## In-Cluster Development (Namespace per Branch)

### Deploy Feature Branch
```bash
# Create namespace
kubectl create namespace feat-jira-123

# Deploy services
helm upgrade --install fleet-os charts/fleet-os \
  --namespace feat-jira-123 \
  --set image.tag=feat-jira-123 \
  --set ingress.host=feat-jira-123.dev.atlasmesh.ae

# Port forwarding for debugging
telepresence connect
telepresence intercept fleet-manager --port 8081:8081 --namespace feat-jira-123
```

### Cleanup
```bash
# Remove feature environment
kubectl delete namespace feat-jira-123
```

## Edge Development Rig

### ROS2 Development
```bash
# List active nodes
ros2 node list

# Monitor vehicle state
ros2 topic echo /vehicle/state

# Replay recorded scenario
ros2 bag play scenarios/scenario_001.bag

# Debug specific node
ros2 run --prefix 'gdb -ex run --args' perception perception_node

# Container-based development
docker run -it --rm \
  -v $(pwd):/workspace \
  -v /dev:/dev \
  --privileged \
  atlasmesh/ros2-dev:latest \
  bash
```

### Vehicle Simulation
```bash
# Start CARLA server
carla-server --world-port=2000 --rpc-port=2001

# Launch vehicle simulation
ros2 launch vehicle_sim vehicle.launch.py \
  vehicle_id:=AV-001 \
  carla_host:=localhost \
  carla_port:=2000
```

---

# 4) Debug Playbooks by Layer

## A. UI / Control Center

### Common Issues
- **Stale map data**: WebSocket connection dropped
- **Slow table rendering**: Large dataset without pagination
- **Broken commands**: RBAC permissions or API errors

### Diagnostic Steps
```bash
# Check WebSocket connection
# Browser DevTools → Network → WS tab
# Verify heartbeat messages < 5s intervals

# Check API calls
# Network tab → look for x-correlation-id in headers
# Verify CORS headers present

# Feature flag validation
# Admin → Feature Flags → verify user cohort assignment

# Performance profiling
# React DevTools Profiler → identify slow components
# Lighthouse CI → check Core Web Vitals
```

### Quick Fixes
```bash
# Toggle problematic features off
curl -X POST http://api-gateway:8080/admin/feature-flags \
  -H "Authorization: Bearer $ADMIN_TOKEN" \
  -d '{"flag": "driverless.ui", "enabled": false}'

# Reduce map layers
# UI Settings → Map Layers → disable non-essential layers

# Clear browser cache
# Hard refresh (Ctrl+Shift+R) or incognito mode
```

## B. API / Microservices

### Golden Rules
- Every request must emit `trace_id`, `entity_id`, `tenant_id`
- Use structured JSON logging
- Include correlation IDs across service boundaries

### Diagnostic Steps
```bash
# Find logs by trace ID
kubectl logs deployment/fleet-manager | grep "trace_id:abc123"

# Distributed tracing
# Jaeger UI → search by trace_id → analyze span latency

# Temporary debug logging
curl -X POST http://fleet-manager:8080/debug/loglevel \
  -d '{"level": "DEBUG", "ttl": "10m"}'

# Contract testing
make conformance-test service=fleet-manager

# Health check
curl http://fleet-manager:8080/health
```

### Quick Fixes
```bash
# Rollback via ArgoCD
argocd app rollback fleet-manager --revision HEAD~1

# Scale up replicas
kubectl scale deployment fleet-manager --replicas=5

# Check configuration drift
kubectl diff -f manifests/fleet-manager.yaml
```

## C. Message Bus (Kafka/NATS)

### Monitoring
```bash
# Check consumer lag
kafka-consumer-groups.sh --bootstrap-server kafka:9092 \
  --group fleet-telemetry --describe

# Monitor topic throughput
kafkacat -C -b kafka:9092 -t vehicle.telemetry -o end -q -e

# NATS monitoring
nats sub trip.lifecycle
nats stream info TELEMETRY
```

### Troubleshooting
```bash
# Dead letter queue stats
kafka-topics.sh --bootstrap-server kafka:9092 \
  --topic vehicle.telemetry.dlq --describe

# Schema registry compatibility
curl http://schema-registry:8081/subjects/vehicle.telemetry-value/versions

# Reset consumer group (CAUTION)
kafka-consumer-groups.sh --bootstrap-server kafka:9092 \
  --group fleet-telemetry --reset-offsets --to-latest \
  --topic vehicle.telemetry --execute
```

### Quick Fixes
```bash
# Enable safe mode (reduced batch size)
kubectl patch deployment telemetry-consumer \
  -p '{"spec":{"template":{"spec":{"containers":[{"name":"consumer","env":[{"name":"SAFE_MODE","value":"true"}]}]}}}}'

# Scale consumer group
kubectl scale deployment telemetry-consumer --replicas=10
```

## D. Edge / ROS2

### System Health
```bash
# Verify node graph
rqt_graph

# Check topic frequencies
ros2 topic hz /control/commands  # Should be ≥20Hz
ros2 topic hz /perception/objects

# Monitor system resources
ros2 run ros2_tracing trace --output trace_$(date +%s)

# Check node health
ros2 node info /perception_node
```

### Debugging
```bash
# Record problematic scenario
ros2 bag record -a -o incident_$(date +%s)

# Replay for debugging
ros2 bag play incident_1234567890.bag

# Monitor control loop latency
ros2 topic echo /diagnostics | grep control_loop_latency

# Check safety system status
ros2 service call /safety/status safety_msgs/srv/GetStatus
```

### Quick Fixes
```bash
# Emergency stop
ros2 service call /emergency_stop std_srvs/srv/Trigger

# Fallback to manual control
ros2 param set /control_node fallback_mode true

# Restart problematic node
sudo systemctl restart ros2-perception

# Speed cap for degraded conditions
ros2 param set /planning_node max_speed 5.0  # 5 m/s
```

## E. Routing / Maps

### Diagnostics
```bash
# Route debugging
curl "http://routing-service:8080/route/diagnose?from=24.4539,54.3773&to=25.2048,55.2708&explain=true"

# Map version validation
curl http://map-service:8080/version/current

# Check road graph integrity
curl http://map-service:8080/validate/road-graph
```

### Map Updates
```bash
# Validate new map data
map-validator --input new_map.osm --output validation_report.json

# Deploy map update
kubectl apply -f manifests/map-update-job.yaml

# Rollback map version
curl -X POST http://map-service:8080/rollback \
  -d '{"version": "v2.1.0"}'
```

### Quick Fixes
```bash
# Blacklist problematic road segment
curl -X POST http://routing-service:8080/blacklist \
  -d '{"segment_id": "way_123456", "reason": "construction", "ttl": "24h"}'

# Force route recalculation
curl -X POST http://routing-service:8080/recalculate \
  -d '{"trip_id": "trip-123"}'
```

## F. Policy Engine (OPA/Rego)

### Policy Debugging
```bash
# Explain policy decision
curl "http://policy-engine:8080/policy/decide?explain=full" \
  -d '{"input": {"vehicle_id": "AV-001", "action": "dispatch"}}'

# Test policy rules
opa test policies/

# Validate policy syntax
opa fmt --diff policies/
```

### Policy Management
```bash
# Deploy new policy
curl -X PUT http://policy-engine:8080/policies/fleet-dispatch \
  -d @policies/fleet-dispatch.rego

# Rollback policy
curl -X DELETE http://policy-engine:8080/policies/fleet-dispatch/v2

# Emergency policy override
curl -X POST http://policy-engine:8080/override \
  -d '{"rule": "emergency_dispatch", "ttl": "1h", "reason": "incident_response"}'
```

## G. Weather Fusion

### Monitoring
```bash
# Check data source health
curl http://weather-fusion:8080/sources/health

# Fusion confidence metrics
curl http://weather-fusion:8080/confidence/current

# Data freshness check
curl http://weather-fusion:8080/freshness
```

### Troubleshooting
```bash
# Pin to specific weather source
curl -X POST http://weather-fusion:8080/pin \
  -d '{"source": "openweather", "duration": "2h"}'

# Clear stale cache
curl -X DELETE http://weather-fusion:8080/cache

# Validate fusion algorithm
curl http://weather-fusion:8080/validate/fusion
```

## H. Telemetry & Evidence

### Data Pipeline Health
```bash
# Check ingestion rates
curl http://telemetry-ingest:8080/metrics/ingestion

# Evidence queue status
curl http://evidence-engine:8080/queue/status

# Upload retry statistics
curl http://evidence-engine:8080/uploads/retries
```

### Evidence Management
```bash
# Generate evidence bundle
curl -X POST http://evidence-engine:8080/bundle \
  -d '{"incident_id": "inc-123", "time_range": "1h"}'

# Verify evidence integrity
curl http://evidence-engine:8080/verify/bundle/bundle-123

# Export for regulator
curl http://evidence-engine:8080/export/regulator \
  -d '{"bundle_id": "bundle-123", "format": "pdf"}'
```

## I. OTA / Garage

### Update Management
```bash
# Check update readiness
curl http://ota-manager:8080/readiness/AV-001

# Start canary rollout
curl -X POST http://ota-manager:8080/rollout \
  -d '{"version": "v2.1.0", "cohort": "canary", "percentage": 10}'

# Monitor rollout progress
curl http://ota-manager:8080/rollout/status/v2.1.0
```

### Garage Operations
```bash
# Bay status
curl http://garage-controller:8080/bays/status

# Image validation
curl http://garage-controller:8080/images/validate/v2.1.0

# Emergency rollback
curl -X POST http://ota-manager:8080/rollback \
  -d '{"version": "v2.0.9", "reason": "critical_bug"}'
```

## J. Networking/Comms

### Connectivity Monitoring
```bash
# Assist RTT dashboard
curl http://monitoring:8080/metrics/assist_rtt

# Connection quality metrics
curl http://comms-orchestration:8080/quality

# Handover statistics
curl http://comms-orchestration:8080/handovers/stats
```

### Network Troubleshooting
```bash
# Test connectivity paths
curl -X POST http://comms-orchestration:8080/test \
  -d '{"vehicle_id": "AV-001", "paths": ["lte", "wifi", "satcom"]}'

# Force path selection
curl -X POST http://comms-orchestration:8080/force-path \
  -d '{"vehicle_id": "AV-001", "path": "satcom"}'

# V2X misbehavior detection
curl http://v2x-manager:8080/misbehavior/detected
```

## K. Security & RBAC

### Access Control
```bash
# Check user permissions
curl http://auth-service:8080/permissions/user123

# Audit access attempts
curl http://auth-service:8080/audit/access?user=user123&timerange=1h

# Role assignment validation
curl http://auth-service:8080/roles/validate/fleet-operator
```

### Security Monitoring
```bash
# Failed authentication attempts
curl http://auth-service:8080/security/failed-auth

# Suspicious activity detection
curl http://security-monitor:8080/suspicious/activity

# Certificate validation
curl http://pki-manager:8080/certificates/validate
```

---

# 5) E2E "Vertical Slice" Validations

## Post-Deployment Validation Suite

### 1. Trip Happy Path
```bash
# Test: Create → Assign → Execute → Complete
curl -X POST http://api-gateway:8080/api/v1/trips \
  -d '{
    "pickup": {"lat": 24.4539, "lng": 54.3773},
    "destination": {"lat": 25.2048, "lng": 55.2708},
    "passenger_count": 2
  }'

# Verify: Trip lifecycle events in Kafka
# Verify: Route calculation completed
# Verify: Vehicle assignment successful
# Verify: Map trail recorded
# Verify: KPIs updated
```

### 2. Emergency Safe Stop
```bash
# Test: Emergency stop command
curl -X POST http://api-gateway:8080/api/v1/vehicles/AV-001/emergency-stop \
  -H "Authorization: Bearer $OPERATOR_TOKEN" \
  -d '{"reason": "obstacle_detected", "dual_auth": true}'

# Verify: Vehicle stopped within 5 seconds
# Verify: Incident record created
# Verify: Evidence spool captured
# Verify: Notifications sent
```

### 3. Remote Assist Flow
```bash
# Test: Trigger assist request
curl -X POST http://api-gateway:8080/api/v1/assist/request \
  -d '{"vehicle_id": "AV-001", "type": "navigation_hint"}'

# Verify: Operator notification < 10s
# Verify: Assist session established
# Verify: Resolution recorded
# Verify: RTT within SLA
```

### 4. Policy Enforcement
```bash
# Test: Policy gate activation
curl -X PUT http://policy-engine:8080/policies/speed-limit \
  -d '{"max_speed": 30, "zone": "school_zone"}'

# Verify: Route recalculation triggered
# Verify: Speed limits enforced
# Verify: Audit trail recorded
```

### 5. Evidence Export
```bash
# Test: Generate regulator package
curl -X POST http://evidence-engine:8080/export/regulator \
  -d '{"incident_id": "inc-123", "format": "compliance_bundle"}'

# Verify: SHA-256 manifest generated
# Verify: Access controls enforced
# Verify: Encryption applied
```

### 6. OTA Rollback Canary
```bash
# Test: Canary rollback
curl -X POST http://ota-manager:8080/rollback \
  -d '{"version": "v2.0.9", "cohort": "canary", "percentage": 10}'

# Verify: Health probes pass
# Verify: Error rates normal
# Verify: Gradual rollout or halt
```

### 7. Garage Image Staging
```bash
# Test: Stage new image
curl -X POST http://garage-controller:8080/stage \
  -d '{"image": "fleet-os:v2.1.0", "bays": ["bay-01", "bay-02"]}'

# Verify: Checksum validation
# Verify: Canary deployment
# Verify: SMART disk health
```

### 8. Map Update Validation
```bash
# Test: Publish geofence update
curl -X POST http://map-service:8080/geofences \
  -d '{"name": "construction_zone", "polygon": [...], "restrictions": ["no_entry"]}'

# Verify: Conflict detection
# Verify: Route recalculation
# Verify: Replay tests pass
```

### 9. Weather Degradation
```bash
# Test: Simulate weather source failure
curl -X POST http://chaos-engineering:8080/inject \
  -d '{"target": "weather-source-1", "failure": "network_timeout"}'

# Verify: Fusion confidence adjustment
# Verify: Route preferences updated
# Verify: Service continuity
```

### 10. Telemetry Load Test
```bash
# Test: Burst telemetry load
curl -X POST http://load-generator:8080/burst \
  -d '{"multiplier": 10, "duration": "5m", "target": "telemetry-ingest"}'

# Verify: Ingestion stays within p95 budget
# Verify: No lag growth > 2 minutes
# Verify: Auto-scaling triggers
```

---

# 6) Progressive Delivery & Rollback

## ArgoCD Rollouts Configuration

### Service Rollout Strategy
```yaml
apiVersion: argoproj.io/v1alpha1
kind: Rollout
metadata:
  name: fleet-manager
spec:
  replicas: 10
  strategy:
    canary:
      steps:
      - setWeight: 5    # 5% traffic
      - pause: {duration: 10m}
      - analysis:
          templates:
          - templateName: success-rate
          args:
          - name: service-name
            value: fleet-manager
      - setWeight: 25   # 25% traffic
      - pause: {duration: 10m}
      - analysis:
          templates:
          - templateName: latency-check
      - setWeight: 50   # 50% traffic
      - pause: {duration: 20m}
      - setWeight: 100  # Full rollout
      
  analysis:
    successCondition: result[0] >= 0.95
    failureLimit: 3
    interval: 2m
```

### Auto-Rollback Triggers
- Error rate > 1% for 5 consecutive minutes
- P95 latency > 500ms for 3 consecutive checks
- SLO violation (availability < 99.9%)
- Error budget burn rate > 10% per hour
- DLQ message spike > 1000 messages/minute

### Kill Switches
```bash
# Feature flag kill switch
curl -X POST http://feature-flags:8080/kill-switch \
  -d '{"feature": "autonomous_dispatch", "reason": "safety_concern"}'

# Policy override
curl -X POST http://policy-engine:8080/emergency-override \
  -d '{"rule": "manual_approval_required", "duration": "2h"}'

# Route blacklist
curl -X POST http://routing-service:8080/blacklist \
  -d '{"segments": ["way_123", "way_456"], "reason": "safety"}'

# Map rollback
curl -X POST http://map-service:8080/rollback \
  -d '{"version": "v2.0.1", "reason": "data_corruption"}'

# Provider demotion
curl -X POST http://weather-fusion:8080/demote \
  -d '{"provider": "weather_api_1", "reason": "accuracy_issues"}'
```

---

# 7) Observability & Dashboards

## Golden Signal Dashboards

### API Gateway Metrics
- **Latency**: P50, P95, P99 response times
- **Traffic**: Requests per second by endpoint
- **Errors**: 4xx/5xx error rates
- **Saturation**: Connection pool utilization

### Service-Specific SLIs
```yaml
Fleet Manager:
  - trip_assignment_success_rate: >95%
  - assignment_latency_p95: <2s
  - vehicle_utilization: >70%

Routing Service:
  - route_calculation_p95: <500ms
  - route_success_rate: >99%
  - map_cache_hit_rate: >90%

Policy Engine:
  - decision_latency_p99: <10ms
  - policy_cache_hit_rate: >95%
  - rule_evaluation_success: >99.9%

Vehicle Gateway:
  - command_success_rate: >99%
  - telemetry_ingestion_rate: 10k msgs/sec
  - connection_uptime: >99.5%
```

### Domain-Specific Dashboards

#### Fleet Operations
```yaml
Metrics:
  - active_vehicles: gauge
  - trips_per_hour: counter
  - average_trip_duration: histogram
  - fleet_utilization_percentage: gauge
  - emergency_stops_per_day: counter
  - assist_requests_per_hour: counter

Alerts:
  - fleet_utilization < 50%
  - emergency_stops > 5/day
  - assist_requests > 10/hour
```

#### Edge/Vehicle Health
```yaml
Metrics:
  - control_loop_latency_p95: <50ms
  - sensor_health_score: >90%
  - cpu_utilization: <80%
  - gpu_utilization: <90%
  - thermal_throttling_events: 0
  - network_handovers_per_hour: counter

Alerts:
  - control_loop_latency > 100ms
  - sensor_health < 80%
  - thermal_throttling detected
```

#### Evidence & Compliance
```yaml
Metrics:
  - evidence_queue_depth: gauge
  - evidence_upload_success_rate: >99%
  - evidence_export_time_p95: <30s
  - audit_trail_completeness: 100%
  - compliance_violations: 0

Alerts:
  - evidence_queue_depth > 1000
  - upload_success_rate < 95%
  - compliance_violation detected
```

### Log Hygiene Standards
```json
{
  "timestamp": "2025-10-01T09:30:00Z",
  "level": "INFO",
  "service": "fleet-manager",
  "trace_id": "abc123def456",
  "span_id": "789ghi012",
  "tenant_id": "tenant-001",
  "entity_id": "vehicle-AV-001",
  "user_id": "user-123",
  "message": "Trip assigned successfully",
  "duration_ms": 150,
  "status_code": 200,
  "endpoint": "/api/v1/trips/assign"
}
```

### Distributed Tracing
- W3C Trace Context propagation across all services
- Correlation IDs in all HTTP headers
- Span annotations for business events
- Error tracking with stack traces
- Performance profiling integration

---

# 8) Smoke Tests (10-Minute Checklist)

## Environment Validation Checklist

### UI/Frontend (2 minutes)
```bash
# Control Center loads
curl -I http://control-center.atlasmesh.ae
# Expected: 200 OK, TTI < 3s

# WebSocket connection established
# Browser DevTools → Network → WS tab
# Expected: Connection established, heartbeat messages

# Map renders with vehicle pins
# Visual verification: Map loads, vehicles visible
```

### API Health (3 minutes)
```bash
# API Gateway health
curl http://api-gateway:8080/health
# Expected: {"status": "healthy"}

# Core services health
for service in fleet-manager vehicle-gateway policy-engine routing-service; do
  curl http://$service:8080/health
done
# Expected: All return 200 OK

# Database connectivity
curl http://fleet-manager:8080/health/database
# Expected: {"database": "connected"}
```

### Business Logic (3 minutes)
```bash
# Create test trip
TRIP_ID=$(curl -X POST http://api-gateway:8080/api/v1/trips \
  -d '{"pickup": {"lat": 24.4539, "lng": 54.3773}, "destination": {"lat": 25.2048, "lng": 55.2708}}' \
  | jq -r '.trip_id')

# Verify trip appears on map
curl http://api-gateway:8080/api/v1/trips/$TRIP_ID
# Expected: Trip status "created" or "assigned"

# Test emergency stop (with proper auth)
curl -X POST http://api-gateway:8080/api/v1/vehicles/AV-001/emergency-stop \
  -H "Authorization: Bearer $TEST_TOKEN"
# Expected: 200 OK or 202 Accepted
```

### Policy & Compliance (1 minute)
```bash
# Policy engine responsive
curl http://policy-engine:8080/policies/health
# Expected: 200 OK

# Evidence export test
curl -X POST http://evidence-engine:8080/export/test \
  -d '{"format": "json", "time_range": "1m"}'
# Expected: Export bundle created
```

### OTA & Updates (1 minute)
```bash
# OTA manager health
curl http://ota-manager:8080/health
# Expected: 200 OK

# Garage controller status
curl http://garage-controller:8080/status
# Expected: All bays operational
```

---

# 9) Chaos Engineering & Performance Testing

## Chaos Testing Scenarios

### Network Chaos
```bash
# Inject 2% packet loss
kubectl apply -f chaos/network-loss.yaml

# Monitor impact on WebSocket connections
# Expected: Graceful degradation, reconnection logic

# Inject 200ms latency
kubectl apply -f chaos/network-latency.yaml

# Monitor API response times
# Expected: Circuit breakers activate, timeouts handled
```

### Service Chaos
```bash
# Kill random pods
kubectl delete pod -l app=fleet-manager --random

# Monitor service mesh behavior
# Expected: Traffic rerouted, no user impact

# Simulate database failover
kubectl patch postgresql primary --type='json' \
  -p='[{"op": "replace", "path": "/spec/replicas", "value": 0}]'

# Expected: Replica promotion, <30s write outage
```

### Load Testing
```bash
# API load test
k6 run --vus 100 --duration 5m scripts/api-load-test.js

# Expected: P95 latency < 500ms, error rate < 1%

# WebSocket load test
artillery run scripts/websocket-load-test.yml

# Expected: Connection stability, message delivery

# UI performance test
lighthouse-ci --upload.target=temporary-public-storage \
  --collect.url=http://control-center.atlasmesh.ae

# Expected: Performance score > 90, TTI < 3s
```

### Edge Chaos
```bash
# Simulate sensor failure
ros2 param set /perception_node sensor_lidar_enabled false

# Expected: Sensor fusion degradation, safe operation

# Network connectivity loss
sudo iptables -A OUTPUT -d api-gateway.atlasmesh.ae -j DROP

# Expected: Offline mode activation, data queuing
```

---

# 10) Incident Response & Postmortem

## Severity Classification

### Severity 1 (Critical)
- **Definition**: Safety-critical system failure, compliance violation, or complete service outage
- **Response Time**: 15 minutes
- **Escalation**: Immediate page to on-call engineer, incident commander activation
- **Examples**: Vehicle collision, emergency stop failure, data breach

### Severity 2 (High)
- **Definition**: Major feature outage affecting operations
- **Response Time**: 1 hour
- **Escalation**: On-call engineer within 30 minutes
- **Examples**: Fleet dispatch down, map service unavailable, assist system failure

### Severity 3 (Medium)
- **Definition**: Performance degradation or minor feature issues
- **Response Time**: 4 hours
- **Escalation**: Team lead within 2 hours
- **Examples**: Slow API responses, UI glitches, non-critical alerts

### Severity 4 (Low)
- **Definition**: Minor issues with workarounds available
- **Response Time**: 24 hours
- **Escalation**: Team lead within 8 hours
- **Examples**: Documentation errors, cosmetic UI issues

## Incident Response Procedure

### 1. Detection & Triage (0-15 minutes)
```bash
# Capture essential information
INCIDENT_ID="INC-$(date +%Y%m%d-%H%M%S)"
TRACE_ID="$(curl -s http://api-gateway:8080/trace/current | jq -r '.trace_id')"
LAST_DEPLOY="$(kubectl get deployment fleet-manager -o jsonpath='{.metadata.annotations.deployment\.kubernetes\.io/revision}')"

# Create incident channel
slack-cli create-channel "#incident-$INCIDENT_ID"

# Page on-call engineer
pagerduty trigger --service fleet-os --incident-key $INCIDENT_ID
```

### 2. Immediate Response (15-30 minutes)
```bash
# Assess impact
curl http://monitoring:8080/impact-assessment?incident=$INCIDENT_ID

# Implement immediate mitigation
# Option 1: Rollback
argocd app rollback fleet-manager --revision $LAST_GOOD_REVISION

# Option 2: Kill switch
curl -X POST http://feature-flags:8080/kill-switch \
  -d '{"feature": "problematic_feature", "reason": "incident_$INCIDENT_ID"}'

# Option 3: Traffic diversion
kubectl patch service fleet-manager \
  -p '{"spec":{"selector":{"version":"stable"}}}'
```

### 3. Communication (Ongoing)
```bash
# Status page update
curl -X POST http://status-page:8080/incidents \
  -d '{
    "id": "'$INCIDENT_ID'",
    "title": "Fleet Operations Degradation",
    "status": "investigating",
    "impact": "major"
  }'

# Stakeholder notification
slack-cli message "#fleet-ops" \
  "🚨 Incident $INCIDENT_ID: Investigating fleet dispatch issues. ETA for resolution: 30 minutes."
```

### 4. Root Cause Analysis (Post-mitigation)
```bash
# Collect evidence
kubectl logs deployment/fleet-manager --since=1h > logs_$INCIDENT_ID.txt
curl http://jaeger:16686/api/traces/$TRACE_ID > trace_$INCIDENT_ID.json
curl http://prometheus:9090/api/v1/query_range?query=fleet_manager_errors > metrics_$INCIDENT_ID.json

# Timeline reconstruction
incident-timeline generate --incident $INCIDENT_ID --output timeline.md
```

## Postmortem Template

```markdown
# Incident Postmortem: $INCIDENT_ID

## Summary
Brief description of the incident and its impact.

## Timeline
- **Detection**: 2025-10-01 09:15 UTC - Alert fired
- **Response**: 2025-10-01 09:18 UTC - On-call engineer paged
- **Mitigation**: 2025-10-01 09:25 UTC - Rollback initiated
- **Resolution**: 2025-10-01 09:30 UTC - Service restored

## Impact
- **Duration**: 15 minutes
- **Affected Users**: 1,250 fleet operators
- **Revenue Impact**: $5,000 estimated
- **Safety Impact**: None (fail-safe mechanisms worked)

## Root Cause
Detailed technical explanation of what went wrong.

## Contributing Factors
- Insufficient testing of edge case
- Missing monitoring for specific metric
- Deployment process gap

## Resolution
What was done to resolve the incident.

## Action Items
| Action | Owner | Due Date | Status |
|--------|-------|----------|--------|
| Add monitoring for X metric | @engineer1 | 2025-10-08 | Open |
| Improve test coverage | @engineer2 | 2025-10-15 | Open |
| Update deployment checklist | @sre1 | 2025-10-05 | Open |

## Lessons Learned
- What went well
- What could be improved
- Process changes needed
```

---

# 11) Common Failure Signatures & Quick Fixes

## Failure Pattern Recognition

### Map-Related Issues
**Signature**: Vehicles taking unexpected routes, "route not found" errors
```bash
# Diagnosis
curl http://routing-service:8080/route/diagnose?from=24.4539,54.3773&to=25.2048,55.2708

# Quick Fix
curl -X POST http://map-service:8080/rollback \
  -d '{"version": "v2.0.1", "reason": "routing_anomaly"}'

# Re-run validation
make test-replay-scenarios
```

### High Assist RTT
**Signature**: Operator response times > 90 seconds, connection timeouts
```bash
# Diagnosis
curl http://comms-orchestration:8080/quality/analysis

# Quick Fixes
# 1. Switch to backup carrier
curl -X POST http://comms-orchestration:8080/failover \
  -d '{"vehicle_id": "AV-001", "target_path": "backup_lte"}'

# 2. Reduce video bitrate
curl -X POST http://vehicle-gateway:8080/config/video \
  -d '{"bitrate": "1mbps", "quality": "medium"}'

# 3. Enable SATCOM for critical vehicles
curl -X POST http://comms-orchestration:8080/enable-satcom \
  -d '{"vehicle_ids": ["AV-001", "AV-002"]}'
```

### Evidence Backlog
**Signature**: Evidence queue depth > 1000, upload failures
```bash
# Diagnosis
curl http://evidence-engine:8080/queue/analysis

# Quick Fixes
# 1. Throttle non-critical telemetry
curl -X POST http://telemetry-ingest:8080/throttle \
  -d '{"categories": ["debug", "info"], "rate": "50%"}'

# 2. Increase upload bandwidth
curl -X POST http://garage-controller:8080/bandwidth \
  -d '{"upload_limit": "100mbps"}'

# 3. Compress evidence data
curl -X POST http://evidence-engine:8080/compression \
  -d '{"enabled": true, "level": "high"}'
```

### DLQ Message Spike
**Signature**: Dead letter queue messages > 1000/minute
```bash
# Diagnosis
kafka-consumer-groups.sh --bootstrap-server kafka:9092 \
  --group telemetry-processor --describe

# Quick Fixes
# 1. Pin consumer to previous schema
kubectl patch deployment telemetry-processor \
  -p '{"spec":{"template":{"spec":{"containers":[{"name":"processor","env":[{"name":"SCHEMA_VERSION","value":"v1.2.0"}]}]}}}}'

# 2. Hotfix producer schema
kubectl patch deployment telemetry-ingest \
  -p '{"spec":{"template":{"spec":{"containers":[{"name":"ingest","image":"telemetry-ingest:v1.2.1-hotfix"}]}}}}'

# 3. Replay DLQ messages after fix
kafka-console-producer.sh --bootstrap-server kafka:9092 \
  --topic vehicle.telemetry < dlq_messages.json
```

### UI Staleness
**Signature**: WebSocket disconnections, stale data on dashboard
```bash
# Diagnosis
curl http://api-gateway:8080/websocket/stats

# Quick Fixes
# 1. Increase proxy timeout
kubectl patch configmap nginx-config \
  -p '{"data":{"proxy_read_timeout":"300s"}}'

# 2. Enable WebSocket keepalive
kubectl patch deployment api-gateway \
  -p '{"spec":{"template":{"spec":{"containers":[{"name":"gateway","env":[{"name":"WS_KEEPALIVE","value":"30s"}]}]}}}}'

# 3. Add reconnection jitter
# Client-side fix in UI code
```

---

# 12) Release Train & Branching Strategy

## Git Workflow
```
main (production)
├── release/v2.1.0 (release candidate)
├── develop (integration)
├── feature/JIRA-123-new-routing (feature branch)
└── hotfix/JIRA-456-critical-fix (hotfix branch)
```

## Release Process

### 1. Feature Development
```bash
# Create feature branch
git checkout -b feature/JIRA-123-new-routing develop

# Development and testing
make test
make lint
make security-scan

# Create PR to develop
gh pr create --title "Add new routing algorithm" --base develop
```

### 2. Release Preparation
```bash
# Create release branch
git checkout -b release/v2.1.0 develop

# Update version numbers
make version-bump version=v2.1.0

# Generate release artifacts
make build-release
make generate-sbom
make security-scan
make compliance-check

# Tag release
git tag -a v2.1.0 -m "Release v2.1.0"
```

### 3. Deployment Pipeline
```yaml
# .github/workflows/release.yml
name: Release Pipeline
on:
  push:
    tags: ['v*']

jobs:
  build:
    runs-on: ubuntu-latest
    steps:
    - name: Build Images
      run: make docker-build
    
    - name: Security Scan
      run: make security-scan
    
    - name: Generate SBOM
      run: make generate-sbom
    
    - name: Push to Registry
      run: make docker-push

  deploy-staging:
    needs: build
    runs-on: ubuntu-latest
    steps:
    - name: Deploy to Staging
      run: |
        argocd app sync fleet-os-staging
        argocd app wait fleet-os-staging --health

  e2e-tests:
    needs: deploy-staging
    runs-on: ubuntu-latest
    steps:
    - name: Run E2E Tests
      run: make e2e-tests env=staging

  deploy-production:
    needs: e2e-tests
    runs-on: ubuntu-latest
    environment: production
    steps:
    - name: Deploy to Production
      run: |
        argocd app sync fleet-os-production
        argocd app wait fleet-os-production --health
```

### 4. Rollout Strategy
```bash
# Phase 1: Canary (5% traffic, 1 depot)
argocd app patch fleet-os-production --patch '
{
  "spec": {
    "source": {
      "helm": {
        "parameters": [
          {"name": "canary.enabled", "value": "true"},
          {"name": "canary.weight", "value": "5"}
        ]
      }
    }
  }
}'

# Monitor for 2 hours
sleep 7200

# Phase 2: Regional (25% traffic, 3 depots)
argocd app patch fleet-os-production --patch '
{
  "spec": {
    "source": {
      "helm": {
        "parameters": [
          {"name": "canary.weight", "value": "25"}
        ]
      }
    }
  }
}'

# Phase 3: Full rollout (100% traffic)
argocd app patch fleet-os-production --patch '
{
  "spec": {
    "source": {
      "helm": {
        "parameters": [
          {"name": "canary.enabled", "value": "false"}
        ]
      }
    }
  }
}'
```

---

# 13) Contact Information & Escalation

## On-Call Rotation
- **Primary**: Backend Team (API, Services, Data)
- **Secondary**: Edge Team (ROS2, Vehicle Systems)
- **Tertiary**: Security Team (Auth, Compliance, PKI)
- **Escalation**: Engineering Manager → CTO

## Emergency Contacts
- **Incident Commander**: +971-50-XXX-XXXX
- **Engineering Manager**: +971-50-XXX-XXXY  
- **Security Lead**: +971-50-XXX-XXXZ
- **CTO**: +971-50-XXX-XXXA

## Communication Channels
- **Slack**: #fleet-ops-alerts, #incident-response, #on-call
- **PagerDuty**: fleet-os-critical, fleet-os-high
- **Status Page**: https://status.atlasmesh.ae
- **Documentation**: https://docs.atlasmesh.ae

---

**This playbook ensures operational excellence for AtlasMesh Fleet OS across all environments, providing clear procedures for running, debugging, and maintaining the system at production scale.**

---

**Document Version**: 1.0.0  
**Last Updated**: October 2025  
**Next Review**: January 2026  
**Maintained By**: Platform Engineering Team
