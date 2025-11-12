## AtlasMesh Fleet OS - Deployment Guide

### Overview

This guide provides comprehensive instructions for deploying AtlasMesh Fleet OS across different environments (SIT, UAT, Production) with proper configuration management, monitoring, and security.

### Prerequisites

### System Requirements
- **Kubernetes Cluster**: v1.28+ with at least 32GB RAM, 16 CPU cores
- **Docker**: v24.0+ with BuildKit enabled
- **Helm**: v3.12+ for package management
- **Terraform**: v1.5+ for infrastructure provisioning
- **kubectl**: v1.28+ configured for cluster access

### Infrastructure Dependencies
- **PostgreSQL**: v15+ with TimescaleDB extension
- **Redis**: v7.0+ cluster setup
- **Kafka**: v3.5+ with Schema Registry
- **ClickHouse**: v23.8+ for analytics
- **MinIO**: v2023+ for object storage
- **Prometheus**: v2.45+ for metrics
- **Grafana**: v10.0+ for dashboards
- **Jaeger**: v1.47+ for tracing

## Environment Setup

### 1. SIT (System Integration Testing)
```bash
# Clone repository
git clone https://github.com/atlasmesh/fleet-os.git
cd fleet-os

# Set environment
export ENVIRONMENT=sit
export NAMESPACE=atlasmesh-sit

# Deploy infrastructure
cd deployment/terraform/sit
terraform init
terraform plan
terraform apply

# Deploy services
cd ../../kubernetes/sit
kubectl create namespace $NAMESPACE
helm install atlasmesh-fleet-os ./helm-chart -n $NAMESPACE -f values-sit.yaml
```

### 2. UAT (User Acceptance Testing)
```bash
# Set environment
export ENVIRONMENT=uat
export NAMESPACE=atlasmesh-uat

# Deploy with UAT configuration
cd deployment/kubernetes/uat
kubectl create namespace $NAMESPACE
helm install atlasmesh-fleet-os ./helm-chart -n $NAMESPACE -f values-uat.yaml

# Run smoke tests
kubectl run smoke-test --image=atlasmesh/smoke-test:latest -n $NAMESPACE
```

### 3. Production
```bash
# Set environment
export ENVIRONMENT=prod
export NAMESPACE=atlasmesh-prod

# Deploy with production configuration
cd deployment/kubernetes/prod
kubectl create namespace $NAMESPACE

# Apply security policies
kubectl apply -f security/network-policies.yaml -n $NAMESPACE
kubectl apply -f security/pod-security-policies.yaml -n $NAMESPACE

# Deploy with high availability
helm install atlasmesh-fleet-os ./helm-chart -n $NAMESPACE -f values-prod.yaml

# Verify deployment
kubectl get pods -n $NAMESPACE
kubectl get services -n $NAMESPACE
```

## Service Configuration

### Core Services
```yaml
# Fleet Manager
fleet-manager:
  replicas: 3
  resources:
    requests:
      cpu: 500m
      memory: 1Gi
    limits:
      cpu: 2000m
      memory: 4Gi
  env:
    - name: DATABASE_URL
      valueFrom:
        secretKeyRef:
          name: postgres-credentials
          key: url
    - name: REDIS_URL
      valueFrom:
        secretKeyRef:
          name: redis-credentials
          key: url

# Vehicle Gateway
vehicle-gateway:
  replicas: 5
  resources:
    requests:
      cpu: 1000m
      memory: 2Gi
    limits:
      cpu: 4000m
      memory: 8Gi
  autoscaling:
    enabled: true
    minReplicas: 5
    maxReplicas: 20
    targetCPUUtilizationPercentage: 70
```

### Database Configuration
```yaml
# PostgreSQL with TimescaleDB
postgresql:
  enabled: true
  auth:
    postgresPassword: ${POSTGRES_PASSWORD}
    database: atlasmesh_fleet
  primary:
    persistence:
      enabled: true
      size: 100Gi
      storageClass: fast-ssd
  metrics:
    enabled: true
    serviceMonitor:
      enabled: true
```

### Message Queue Configuration
```yaml
# Kafka Configuration
kafka:
  enabled: true
  replicaCount: 3
  persistence:
    enabled: true
    size: 50Gi
  zookeeper:
    enabled: true
    replicaCount: 3
  schemaRegistry:
    enabled: true
```

## Security Configuration

### TLS/SSL Setup
```bash
# Generate certificates
openssl req -x509 -nodes -days 365 -newkey rsa:2048 \
  -keyout tls.key -out tls.crt \
  -subj "/CN=atlasmesh-fleet-os.local"

# Create TLS secret
kubectl create secret tls atlasmesh-tls \
  --key tls.key --cert tls.crt -n $NAMESPACE
```

### RBAC Configuration
```yaml
apiVersion: rbac.authorization.k8s.io/v1
kind: Role
metadata:
  namespace: atlasmesh-prod
  name: fleet-operator
rules:
- apiGroups: [""]
  resources: ["pods", "services", "configmaps"]
  verbs: ["get", "list", "watch", "create", "update", "patch"]
- apiGroups: ["apps"]
  resources: ["deployments", "replicasets"]
  verbs: ["get", "list", "watch", "create", "update", "patch"]
```

### Network Policies
```yaml
apiVersion: networking.k8s.io/v1
kind: NetworkPolicy
metadata:
  name: fleet-network-policy
spec:
  podSelector:
    matchLabels:
      app: atlasmesh-fleet-os
  policyTypes:
  - Ingress
  - Egress
  ingress:
  - from:
    - podSelector:
        matchLabels:
          app: nginx-ingress
    ports:
    - protocol: TCP
      port: 8080
```

## Monitoring & Observability

### Prometheus Configuration
```yaml
# ServiceMonitor for fleet services
apiVersion: monitoring.coreos.com/v1
kind: ServiceMonitor
metadata:
  name: fleet-services
spec:
  selector:
    matchLabels:
      app: atlasmesh-fleet-os
  endpoints:
  - port: metrics
    interval: 30s
    path: /metrics
```

### Grafana Dashboards
```bash
# Import dashboards
kubectl create configmap fleet-dashboards \
  --from-file=monitoring/grafana/dashboards/ -n $NAMESPACE

# Configure data sources
kubectl apply -f monitoring/grafana/datasources.yaml -n $NAMESPACE
```

### Jaeger Tracing
```yaml
# Jaeger configuration
jaeger:
  enabled: true
  strategy: production
  collector:
    resources:
      requests:
        cpu: 500m
        memory: 1Gi
  query:
    resources:
      requests:
        cpu: 200m
        memory: 512Mi
```

## Backup & Recovery

### Database Backup
```bash
# Automated backup script
#!/bin/bash
BACKUP_DIR="/backups/postgres"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)

# Create backup
pg_dump -h $POSTGRES_HOST -U $POSTGRES_USER -d atlasmesh_fleet \
  | gzip > $BACKUP_DIR/backup_$TIMESTAMP.sql.gz

# Upload to S3
aws s3 cp $BACKUP_DIR/backup_$TIMESTAMP.sql.gz \
  s3://atlasmesh-backups/postgres/

# Cleanup old backups (keep 30 days)
find $BACKUP_DIR -name "*.sql.gz" -mtime +30 -delete
```

### Disaster Recovery
```bash
# Cross-region backup replication
aws s3 sync s3://atlasmesh-backups-primary/ \
  s3://atlasmesh-backups-dr/ --region us-west-2

# Database restoration
gunzip -c backup_20231201_120000.sql.gz | \
  psql -h $POSTGRES_HOST -U $POSTGRES_USER -d atlasmesh_fleet
```

## Performance Tuning

### Resource Optimization
```yaml
# HPA configuration
apiVersion: autoscaling/v2
kind: HorizontalPodAutoscaler
metadata:
  name: fleet-manager-hpa
spec:
  scaleTargetRef:
    apiVersion: apps/v1
    kind: Deployment
    name: fleet-manager
  minReplicas: 3
  maxReplicas: 10
  metrics:
  - type: Resource
    resource:
      name: cpu
      target:
        type: Utilization
        averageUtilization: 70
  - type: Resource
    resource:
      name: memory
      target:
        type: Utilization
        averageUtilization: 80
```

### Database Optimization
```sql
-- Performance tuning queries
-- Index optimization
CREATE INDEX CONCURRENTLY idx_vehicle_telemetry_timestamp 
ON vehicle_telemetry (timestamp DESC);

-- Partition management
SELECT create_hypertable('vehicle_telemetry', 'timestamp');
SELECT add_retention_policy('vehicle_telemetry', INTERVAL '90 days');

-- Connection pooling
ALTER SYSTEM SET max_connections = 200;
ALTER SYSTEM SET shared_buffers = '4GB';
ALTER SYSTEM SET effective_cache_size = '12GB';
```

## Troubleshooting

### Common Issues

#### 1. Pod Startup Failures
```bash
# Check pod status
kubectl describe pod <pod-name> -n $NAMESPACE

# Check logs
kubectl logs <pod-name> -n $NAMESPACE --previous

# Check events
kubectl get events -n $NAMESPACE --sort-by=.metadata.creationTimestamp
```

#### 2. Database Connection Issues
```bash
# Test database connectivity
kubectl run postgres-test --image=postgres:15 --rm -it -- \
  psql -h postgres-service -U postgres -d atlasmesh_fleet

# Check connection pool
kubectl exec -it <fleet-manager-pod> -- \
  curl http://localhost:8080/health/database
```

#### 3. Service Discovery Issues
```bash
# Check service endpoints
kubectl get endpoints -n $NAMESPACE

# Test service connectivity
kubectl run network-test --image=nicolaka/netshoot --rm -it -- \
  nslookup fleet-manager-service.atlasmesh-prod.svc.cluster.local
```

#### 4. Performance Issues
```bash
# Check resource usage
kubectl top pods -n $NAMESPACE
kubectl top nodes

# Check HPA status
kubectl get hpa -n $NAMESPACE

# Analyze slow queries
kubectl exec -it postgres-0 -- \
  psql -U postgres -d atlasmesh_fleet -c "
  SELECT query, calls, total_time, mean_time 
  FROM pg_stat_statements 
  ORDER BY total_time DESC LIMIT 10;"
```

## Health Checks

### Service Health Endpoints
```bash
# Fleet Manager
curl http://fleet-manager:8080/health

# Vehicle Gateway
curl http://vehicle-gateway:8080/health

# Policy Engine
curl http://policy-engine:8080/health

# Weather Fusion
curl http://weather-fusion:8080/health
```

### System Health Dashboard
```bash
# Access Grafana dashboard
kubectl port-forward svc/grafana 3000:80 -n monitoring

# Access Prometheus
kubectl port-forward svc/prometheus 9090:9090 -n monitoring

# Access Jaeger UI
kubectl port-forward svc/jaeger-query 16686:16686 -n monitoring
```

## Rollback Procedures

### Application Rollback
```bash
# Rollback to previous version
helm rollback atlasmesh-fleet-os -n $NAMESPACE

# Rollback specific deployment
kubectl rollout undo deployment/fleet-manager -n $NAMESPACE

# Check rollout status
kubectl rollout status deployment/fleet-manager -n $NAMESPACE
```

### Database Rollback
```bash
# Restore from backup
kubectl exec -it postgres-0 -- \
  pg_restore -U postgres -d atlasmesh_fleet /backups/backup_latest.sql

# Verify data integrity
kubectl exec -it postgres-0 -- \
  psql -U postgres -d atlasmesh_fleet -c "SELECT COUNT(*) FROM vehicles;"
```

## Maintenance Windows

### Scheduled Maintenance
```bash
# Drain nodes for maintenance
kubectl drain <node-name> --ignore-daemonsets --delete-emptydir-data

# Update system packages
sudo apt update && sudo apt upgrade -y

# Restart services
kubectl rollout restart deployment/fleet-manager -n $NAMESPACE

# Uncordon nodes
kubectl uncordon <node-name>
```

### Zero-Downtime Updates
```bash
# Blue-green deployment
helm upgrade atlasmesh-fleet-os ./helm-chart -n $NAMESPACE \
  --set image.tag=v2.0.0 --wait

# Canary deployment
kubectl patch deployment fleet-manager -n $NAMESPACE \
  -p '{"spec":{"template":{"spec":{"containers":[{"name":"fleet-manager","image":"atlasmesh/fleet-manager:v2.0.0"}]}}}}'
```

## Contact & Support

- **Operations Team**: ops@atlasmesh.ae
- **On-Call**: +971-50-XXX-XXXX
- **Slack**: #atlasmesh-ops
- **Documentation**: https://docs.atlasmesh.ae
- **Status Page**: https://status.atlasmesh.ae

## Appendix

### Environment Variables Reference
```bash
# Database
DATABASE_URL=postgresql://user:pass@host:5432/db
REDIS_URL=redis://host:6379/0

# Kafka
KAFKA_BROKERS=kafka-1:9092,kafka-2:9092,kafka-3:9092
SCHEMA_REGISTRY_URL=http://schema-registry:8081

# Monitoring
PROMETHEUS_URL=http://prometheus:9090
JAEGER_ENDPOINT=http://jaeger-collector:14268/api/traces

# Security
JWT_SECRET=your-jwt-secret
ENCRYPTION_KEY=your-encryption-key
```

### Port Reference
```
Fleet Manager: 8080
Vehicle Gateway: 8081
Policy Engine: 8082
Weather Fusion: 8083
Predictive Maintenance: 8084
Garage Tools: 8085
Telemetry Ingest: 8086
Cost Optimization: 8087
```
