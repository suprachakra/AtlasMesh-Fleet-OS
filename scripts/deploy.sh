#!/bin/bash

# AtlasMesh Fleet OS Deployment Script
# 
# This script handles deployment across different environments:
# - Development (local)
# - SIT (System Integration Testing)
# - UAT (User Acceptance Testing)
# - Production
#
# Usage: ./scripts/deploy.sh [environment] [options]
# Example: ./scripts/deploy.sh development --build --logs

set -euo pipefail

# Script configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
LOG_FILE="$PROJECT_ROOT/logs/deploy-$(date +%Y%m%d-%H%M%S).log"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Logging functions
log_info() {
    echo -e "${BLUE}[INFO]${NC} $1" | tee -a "$LOG_FILE"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1" | tee -a "$LOG_FILE"
}

log_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1" | tee -a "$LOG_FILE"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1" | tee -a "$LOG_FILE"
}

# Create logs directory
mkdir -p "$PROJECT_ROOT/logs"

# Default values
ENVIRONMENT="development"
BUILD_IMAGES=false
SHOW_LOGS=false
CLEAN_VOLUMES=false
HEALTH_CHECK=true
SKIP_TESTS=false

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        development|sit|uat|production)
            ENVIRONMENT="$1"
            shift
            ;;
        --build)
            BUILD_IMAGES=true
            shift
            ;;
        --logs)
            SHOW_LOGS=true
            shift
            ;;
        --clean)
            CLEAN_VOLUMES=true
            shift
            ;;
        --no-health-check)
            HEALTH_CHECK=false
            shift
            ;;
        --skip-tests)
            SKIP_TESTS=true
            shift
            ;;
        --help)
            echo "AtlasMesh Fleet OS Deployment Script"
            echo ""
            echo "Usage: $0 [environment] [options]"
            echo ""
            echo "Environments:"
            echo "  development    Local development environment (default)"
            echo "  sit           System Integration Testing environment"
            echo "  uat           User Acceptance Testing environment"
            echo "  production    Production environment"
            echo ""
            echo "Options:"
            echo "  --build       Build Docker images before deployment"
            echo "  --logs        Show container logs after deployment"
            echo "  --clean       Clean volumes before deployment"
            echo "  --no-health-check  Skip health checks"
            echo "  --skip-tests  Skip running tests"
            echo "  --help        Show this help message"
            exit 0
            ;;
        *)
            log_error "Unknown option: $1"
            exit 1
            ;;
    esac
done

log_info "Starting AtlasMesh Fleet OS deployment for environment: $ENVIRONMENT"

# Validate environment
case $ENVIRONMENT in
    development)
        COMPOSE_FILE="docker-compose.yml"
        ENV_FILE=".env.development"
        ;;
    sit)
        COMPOSE_FILE="environments/sit/docker-compose.sit.yml"
        ENV_FILE="environments/sit/.env.sit"
        ;;
    uat)
        COMPOSE_FILE="environments/uat/docker-compose.uat.yml"
        ENV_FILE="environments/uat/.env.uat"
        ;;
    production)
        COMPOSE_FILE="environments/production/docker-compose.prod.yml"
        ENV_FILE="environments/production/.env.production"
        ;;
    *)
        log_error "Invalid environment: $ENVIRONMENT"
        exit 1
        ;;
esac

# Check prerequisites
check_prerequisites() {
    log_info "Checking prerequisites..."
    
    # Check Docker
    if ! command -v docker &> /dev/null; then
        log_error "Docker is not installed or not in PATH"
        exit 1
    fi
    
    # Check Docker Compose
    if ! command -v docker-compose &> /dev/null && ! docker compose version &> /dev/null; then
        log_error "Docker Compose is not installed or not in PATH"
        exit 1
    fi
    
    # Check if Docker daemon is running
    if ! docker info &> /dev/null; then
        log_error "Docker daemon is not running"
        exit 1
    fi
    
    # Check compose file exists
    if [[ ! -f "$PROJECT_ROOT/$COMPOSE_FILE" ]]; then
        log_error "Compose file not found: $COMPOSE_FILE"
        exit 1
    fi
    
    # Check environment file exists
    if [[ ! -f "$PROJECT_ROOT/$ENV_FILE" ]]; then
        log_warning "Environment file not found: $ENV_FILE"
        log_info "Creating default environment file..."
        create_default_env_file
    fi
    
    log_success "Prerequisites check passed"
}

# Create default environment file
create_default_env_file() {
    case $ENVIRONMENT in
        development)
            cat > "$PROJECT_ROOT/$ENV_FILE" << EOF
# AtlasMesh Fleet OS - Development Environment Variables
POSTGRES_PASSWORD=atlasmesh_dev_password
REDIS_PASSWORD=atlasmesh_dev_redis
CLICKHOUSE_PASSWORD=atlasmesh_dev_clickhouse
MINIO_ROOT_USER=atlasmesh
MINIO_ROOT_PASSWORD=atlasmesh_dev_minio
GRAFANA_ADMIN_PASSWORD=admin
VAULT_ROOT_TOKEN=dev-only-token
JAEGER_ENDPOINT=http://jaeger:14268/api/traces
LOG_LEVEL=debug
ENVIRONMENT=development
EOF
            ;;
        sit)
            cat > "$PROJECT_ROOT/$ENV_FILE" << EOF
# AtlasMesh Fleet OS - SIT Environment Variables
POSTGRES_PASSWORD_SIT=\${POSTGRES_PASSWORD_SIT}
REDIS_PASSWORD_SIT=\${REDIS_PASSWORD_SIT}
CLICKHOUSE_PASSWORD_SIT=\${CLICKHOUSE_PASSWORD_SIT}
MINIO_ROOT_USER_SIT=atlasmesh
MINIO_ROOT_PASSWORD_SIT=\${MINIO_ROOT_PASSWORD_SIT}
GRAFANA_ADMIN_PASSWORD_SIT=\${GRAFANA_ADMIN_PASSWORD_SIT}
LOG_LEVEL=info
ENVIRONMENT=sit
EOF
            ;;
        uat)
            cat > "$PROJECT_ROOT/$ENV_FILE" << EOF
# AtlasMesh Fleet OS - UAT Environment Variables
POSTGRES_PASSWORD_UAT=\${POSTGRES_PASSWORD_UAT}
POSTGRES_REPLICATION_PASSWORD_UAT=\${POSTGRES_REPLICATION_PASSWORD_UAT}
REDIS_PASSWORD_UAT=\${REDIS_PASSWORD_UAT}
GRAFANA_ADMIN_PASSWORD_UAT=\${GRAFANA_ADMIN_PASSWORD_UAT}
SMTP_HOST=\${SMTP_HOST}
SMTP_USER=\${SMTP_USER}
SMTP_PASSWORD=\${SMTP_PASSWORD}
SENTRY_DSN_UAT=\${SENTRY_DSN_UAT}
ANALYTICS_ID_UAT=\${ANALYTICS_ID_UAT}
LOG_LEVEL=warn
ENVIRONMENT=uat
EOF
            ;;
        production)
            cat > "$PROJECT_ROOT/$ENV_FILE" << EOF
# AtlasMesh Fleet OS - Production Environment Variables
# NOTE: All sensitive values should be set via Docker secrets or external secret management
POSTGRES_PASSWORD_PROD=\${POSTGRES_PASSWORD_PROD}
REDIS_PASSWORD_PROD=\${REDIS_PASSWORD_PROD}
S3_BACKUP_BUCKET=\${S3_BACKUP_BUCKET}
S3_ACCESS_KEY=\${S3_ACCESS_KEY}
S3_SECRET_KEY=\${S3_SECRET_KEY}
SMTP_HOST=\${SMTP_HOST}
SMTP_USER=\${SMTP_USER}
VERSION=\${VERSION:-latest}
LOG_LEVEL=error
ENVIRONMENT=production
EOF
            ;;
    esac
    log_info "Created default environment file: $ENV_FILE"
}

# Clean up function
cleanup() {
    if [[ "$CLEAN_VOLUMES" == "true" ]]; then
        log_info "Cleaning up volumes..."
        docker-compose -f "$PROJECT_ROOT/$COMPOSE_FILE" --env-file "$PROJECT_ROOT/$ENV_FILE" down -v
        log_success "Volumes cleaned"
    fi
}

# Build images if requested
build_images() {
    if [[ "$BUILD_IMAGES" == "true" ]]; then
        log_info "Building Docker images..."
        docker-compose -f "$PROJECT_ROOT/$COMPOSE_FILE" --env-file "$PROJECT_ROOT/$ENV_FILE" build --parallel
        log_success "Images built successfully"
    fi
}

# Deploy services
deploy_services() {
    log_info "Deploying services..."
    
    # Start core infrastructure first
    log_info "Starting core infrastructure..."
    docker-compose -f "$PROJECT_ROOT/$COMPOSE_FILE" --env-file "$PROJECT_ROOT/$ENV_FILE" up -d postgres redis zookeeper kafka
    
    # Wait for infrastructure to be ready
    log_info "Waiting for infrastructure to be ready..."
    sleep 30
    
    # Start remaining services
    log_info "Starting application services..."
    docker-compose -f "$PROJECT_ROOT/$COMPOSE_FILE" --env-file "$PROJECT_ROOT/$ENV_FILE" up -d
    
    log_success "Services deployed successfully"
}

# Health check function
perform_health_checks() {
    if [[ "$HEALTH_CHECK" == "false" ]]; then
        return 0
    fi
    
    log_info "Performing health checks..."
    
    local max_attempts=30
    local attempt=1
    local services=("api-gateway" "policy-engine" "mission-management" "dispatch-service" "routing-service" "fleet-manager")
    
    for service in "${services[@]}"; do
        log_info "Checking health of $service..."
        
        while [[ $attempt -le $max_attempts ]]; do
            if docker-compose -f "$PROJECT_ROOT/$COMPOSE_FILE" --env-file "$PROJECT_ROOT/$ENV_FILE" exec -T "$service" curl -f http://localhost:8080/health &> /dev/null; then
                log_success "$service is healthy"
                break
            fi
            
            if [[ $attempt -eq $max_attempts ]]; then
                log_error "$service failed health check after $max_attempts attempts"
                return 1
            fi
            
            log_info "Attempt $attempt/$max_attempts: $service not ready, waiting..."
            sleep 10
            ((attempt++))
        done
        
        attempt=1
    done
    
    log_success "All services passed health checks"
}

# Run tests
run_tests() {
    if [[ "$SKIP_TESTS" == "true" ]]; then
        log_info "Skipping tests as requested"
        return 0
    fi
    
    log_info "Running integration tests..."
    
    case $ENVIRONMENT in
        development|sit)
            # Run integration tests
            if docker-compose -f "$PROJECT_ROOT/$COMPOSE_FILE" --env-file "$PROJECT_ROOT/$ENV_FILE" run --rm test-runner npm run test:integration; then
                log_success "Integration tests passed"
            else
                log_error "Integration tests failed"
                return 1
            fi
            ;;
        uat)
            # Run UAT tests
            if docker-compose -f "$PROJECT_ROOT/$COMPOSE_FILE" --env-file "$PROJECT_ROOT/$ENV_FILE" run --rm selenium-chrome npm run test:e2e; then
                log_success "E2E tests passed"
            else
                log_error "E2E tests failed"
                return 1
            fi
            ;;
        production)
            # Run smoke tests only
            log_info "Running smoke tests for production..."
            if curl -f "https://api.atlasmesh.ai/v2/health" &> /dev/null; then
                log_success "Production smoke tests passed"
            else
                log_error "Production smoke tests failed"
                return 1
            fi
            ;;
    esac
}

# Show logs if requested
show_logs() {
    if [[ "$SHOW_LOGS" == "true" ]]; then
        log_info "Showing container logs..."
        docker-compose -f "$PROJECT_ROOT/$COMPOSE_FILE" --env-file "$PROJECT_ROOT/$ENV_FILE" logs -f
    fi
}

# Generate deployment report
generate_report() {
    local report_file="$PROJECT_ROOT/logs/deployment-report-$ENVIRONMENT-$(date +%Y%m%d-%H%M%S).json"
    
    log_info "Generating deployment report..."
    
    cat > "$report_file" << EOF
{
  "deployment": {
    "environment": "$ENVIRONMENT",
    "timestamp": "$(date -u +%Y-%m-%dT%H:%M:%SZ)",
    "compose_file": "$COMPOSE_FILE",
    "env_file": "$ENV_FILE",
    "build_images": $BUILD_IMAGES,
    "clean_volumes": $CLEAN_VOLUMES,
    "health_check": $HEALTH_CHECK,
    "skip_tests": $SKIP_TESTS
  },
  "services": {
EOF
    
    # Get service status
    local first=true
    while IFS= read -r line; do
        if [[ "$line" =~ ^[a-zA-Z] ]]; then
            local service_name=$(echo "$line" | awk '{print $1}')
            local service_status=$(echo "$line" | awk '{print $2}')
            
            if [[ "$first" == "true" ]]; then
                first=false
            else
                echo "," >> "$report_file"
            fi
            
            echo "    \"$service_name\": \"$service_status\"" >> "$report_file"
        fi
    done < <(docker-compose -f "$PROJECT_ROOT/$COMPOSE_FILE" --env-file "$PROJECT_ROOT/$ENV_FILE" ps --format "table {{.Service}}\t{{.State}}" | tail -n +2)
    
    cat >> "$report_file" << EOF
  },
  "system_info": {
    "docker_version": "$(docker --version)",
    "compose_version": "$(docker-compose --version 2>/dev/null || docker compose version)",
    "host_os": "$(uname -s)",
    "host_arch": "$(uname -m)"
  }
}
EOF
    
    log_success "Deployment report generated: $report_file"
}

# Main deployment flow
main() {
    log_info "AtlasMesh Fleet OS Deployment Started"
    log_info "Environment: $ENVIRONMENT"
    log_info "Compose file: $COMPOSE_FILE"
    log_info "Environment file: $ENV_FILE"
    
    # Execute deployment steps
    check_prerequisites
    cleanup
    build_images
    deploy_services
    perform_health_checks
    run_tests
    generate_report
    
    log_success "Deployment completed successfully!"
    log_info "Access the system:"
    
    case $ENVIRONMENT in
        development)
            log_info "  - API Gateway: http://localhost:8080"
            log_info "  - Control Center: http://localhost:3000"
            log_info "  - Grafana: http://localhost:3000 (admin/admin)"
            log_info "  - Prometheus: http://localhost:9090"
            ;;
        sit)
            log_info "  - API Gateway: http://localhost:8081"
            log_info "  - Grafana: http://localhost:3001"
            ;;
        uat)
            log_info "  - Load Balancer: http://localhost (port 80/443)"
            log_info "  - Control Center: http://localhost:3003"
            ;;
        production)
            log_info "  - Production endpoints as configured"
            ;;
    esac
    
    show_logs
}

# Trap for cleanup on script exit
trap 'log_info "Deployment script interrupted"' INT TERM

# Run main function
main "$@"
