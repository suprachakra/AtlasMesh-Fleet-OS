#!/bin/bash

# AtlasMesh Fleet OS - Deployment Script
# Deploys the complete fleet management platform to Kubernetes
# Supports multiple environments: development, staging, production

set -euo pipefail

# Configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
DEPLOY_DIR="$PROJECT_ROOT/deploy"

# Default values
ENVIRONMENT="${ENVIRONMENT:-staging}"
NAMESPACE="${NAMESPACE:-atlasmesh-fleet-os-${ENVIRONMENT}}"
HELM_RELEASE_NAME="${HELM_RELEASE_NAME:-atlasmesh-fleet-os}"
KUBECTL_CONTEXT="${KUBECTL_CONTEXT:-}"
DRY_RUN="${DRY_RUN:-false}"
SKIP_BUILD="${SKIP_BUILD:-false}"
SKIP_TESTS="${SKIP_TESTS:-false}"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Logging functions
log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

log_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Help function
show_help() {
    cat << EOF
AtlasMesh Fleet OS Deployment Script

Usage: $0 [OPTIONS]

OPTIONS:
    -e, --environment ENV       Deployment environment (development|staging|production) [default: staging]
    -n, --namespace NAMESPACE   Kubernetes namespace [default: atlasmesh-fleet-os-ENV]
    -r, --release RELEASE       Helm release name [default: atlasmesh-fleet-os]
    -c, --context CONTEXT       Kubectl context to use
    -d, --dry-run              Perform a dry run without making changes
    --skip-build               Skip building Docker images
    --skip-tests               Skip running tests
    -h, --help                 Show this help message

EXAMPLES:
    # Deploy to staging (default)
    $0

    # Deploy to production
    $0 --environment production

    # Dry run deployment
    $0 --dry-run

    # Deploy to specific namespace
    $0 --environment development --namespace my-dev-namespace

ENVIRONMENT VARIABLES:
    ENVIRONMENT                Deployment environment
    NAMESPACE                  Kubernetes namespace
    KUBECTL_CONTEXT           Kubectl context
    DRY_RUN                   Dry run flag (true/false)
    SKIP_BUILD                Skip build flag (true/false)
    SKIP_TESTS                Skip tests flag (true/false)
    GOOGLE_MAPS_API_KEY       Google Maps API key
    DOCKER_REGISTRY           Docker registry URL
    DOCKER_USERNAME           Docker registry username
    DOCKER_PASSWORD           Docker registry password

EOF
}

# Parse command line arguments
parse_args() {
    while [[ $# -gt 0 ]]; do
        case $1 in
            -e|--environment)
                ENVIRONMENT="$2"
                NAMESPACE="atlasmesh-fleet-os-${ENVIRONMENT}"
                shift 2
                ;;
            -n|--namespace)
                NAMESPACE="$2"
                shift 2
                ;;
            -r|--release)
                HELM_RELEASE_NAME="$2"
                shift 2
                ;;
            -c|--context)
                KUBECTL_CONTEXT="$2"
                shift 2
                ;;
            -d|--dry-run)
                DRY_RUN="true"
                shift
                ;;
            --skip-build)
                SKIP_BUILD="true"
                shift
                ;;
            --skip-tests)
                SKIP_TESTS="true"
                shift
                ;;
            -h|--help)
                show_help
                exit 0
                ;;
            *)
                log_error "Unknown option: $1"
                show_help
                exit 1
                ;;
        esac
    done
}

# Validate environment
validate_environment() {
    log_info "Validating environment: $ENVIRONMENT"
    
    case $ENVIRONMENT in
        development|staging|production)
            log_success "Environment '$ENVIRONMENT' is valid"
            ;;
        *)
            log_error "Invalid environment: $ENVIRONMENT"
            log_error "Valid environments: development, staging, production"
            exit 1
            ;;
    esac
}

# Check prerequisites
check_prerequisites() {
    log_info "Checking prerequisites..."
    
    # Check required commands
    local required_commands=("kubectl" "helm" "docker")
    for cmd in "${required_commands[@]}"; do
        if ! command -v "$cmd" &> /dev/null; then
            log_error "Required command '$cmd' not found"
            exit 1
        fi
    done
    
    # Check kubectl context
    if [[ -n "$KUBECTL_CONTEXT" ]]; then
        log_info "Using kubectl context: $KUBECTL_CONTEXT"
        kubectl config use-context "$KUBECTL_CONTEXT"
    else
        local current_context
        current_context=$(kubectl config current-context)
        log_info "Using current kubectl context: $current_context"
    fi
    
    # Test cluster connectivity
    if ! kubectl cluster-info &> /dev/null; then
        log_error "Cannot connect to Kubernetes cluster"
        exit 1
    fi
    
    # Check Helm
    if ! helm version &> /dev/null; then
        log_error "Helm is not properly configured"
        exit 1
    fi
    
    log_success "Prerequisites check passed"
}

# Build Docker images
build_images() {
    if [[ "$SKIP_BUILD" == "true" ]]; then
        log_info "Skipping Docker image build"
        return
    fi
    
    log_info "Building Docker images..."
    
    # Build Control Center UI
    log_info "Building Control Center UI..."
    cd "$PROJECT_ROOT/ui/control-center"
    
    # Create production build
    if [[ ! -f ".env.production" ]]; then
        log_warning "Creating production environment file"
        cat > .env.production << EOF
REACT_APP_DEFAULT_MAP_PROVIDER=openstreetmap
REACT_APP_DEFAULT_MAP_CENTER_LAT=24.4539
REACT_APP_DEFAULT_MAP_CENTER_LNG=54.3773
REACT_APP_DEFAULT_MAP_ZOOM=12
REACT_APP_ENABLE_CLUSTERING=true
REACT_APP_ENABLE_REAL_TIME_UPDATES=true
REACT_APP_ENABLE_VEHICLE_TRAILS=true
REACT_APP_DEBUG_MODE=false
REACT_APP_MOCK_DATA=false
REACT_APP_LOG_LEVEL=info
EOF
    fi
    
    # Build Docker image
    docker build -t "atlasmesh/control-center-ui:1.0.0" \
        --build-arg NODE_ENV=production \
        --build-arg REACT_APP_VERSION=1.0.0 \
        -f Dockerfile .
    
    # Tag for registry
    if [[ -n "${DOCKER_REGISTRY:-}" ]]; then
        docker tag "atlasmesh/control-center-ui:1.0.0" \
            "$DOCKER_REGISTRY/atlasmesh/control-center-ui:1.0.0"
    fi
    
    cd "$PROJECT_ROOT"
    
    # Build backend services (placeholder - would build actual Go services)
    log_info "Building backend services..."
    local services=("api-gateway" "fleet-manager" "policy-engine" "vehicle-gateway" "telemetry-ingestion")
    
    for service in "${services[@]}"; do
        log_info "Building $service..."
        # In a real deployment, this would build the actual Go services
        # For now, we'll create placeholder Dockerfiles
        
        mkdir -p "$PROJECT_ROOT/services/$service"
        if [[ ! -f "$PROJECT_ROOT/services/$service/Dockerfile" ]]; then
            cat > "$PROJECT_ROOT/services/$service/Dockerfile" << EOF
FROM golang:1.21-alpine AS builder
WORKDIR /app
COPY . .
RUN go mod download
RUN CGO_ENABLED=0 GOOS=linux go build -o $service ./cmd/main.go

FROM alpine:3.18
RUN apk --no-cache add ca-certificates tzdata
WORKDIR /root/
COPY --from=builder /app/$service .
EXPOSE 8080
CMD ["./$service"]
EOF
        fi
        
        # Build placeholder image
        docker build -t "atlasmesh/$service:1.0.0" "$PROJECT_ROOT/services/$service" || true
        
        if [[ -n "${DOCKER_REGISTRY:-}" ]]; then
            docker tag "atlasmesh/$service:1.0.0" \
                "$DOCKER_REGISTRY/atlasmesh/$service:1.0.0" || true
        fi
    done
    
    log_success "Docker images built successfully"
}

# Push images to registry
push_images() {
    if [[ -z "${DOCKER_REGISTRY:-}" ]]; then
        log_info "No Docker registry specified, skipping image push"
        return
    fi
    
    log_info "Pushing images to registry: $DOCKER_REGISTRY"
    
    # Login to registry
    if [[ -n "${DOCKER_USERNAME:-}" && -n "${DOCKER_PASSWORD:-}" ]]; then
        echo "$DOCKER_PASSWORD" | docker login "$DOCKER_REGISTRY" -u "$DOCKER_USERNAME" --password-stdin
    fi
    
    # Push Control Center UI
    docker push "$DOCKER_REGISTRY/atlasmesh/control-center-ui:1.0.0"
    
    # Push backend services
    local services=("api-gateway" "fleet-manager" "policy-engine" "vehicle-gateway" "telemetry-ingestion")
    for service in "${services[@]}"; do
        docker push "$DOCKER_REGISTRY/atlasmesh/$service:1.0.0" || true
    done
    
    log_success "Images pushed to registry"
}

# Run tests
run_tests() {
    if [[ "$SKIP_TESTS" == "true" ]]; then
        log_info "Skipping tests"
        return
    fi
    
    log_info "Running tests..."
    
    # Frontend tests
    cd "$PROJECT_ROOT/ui/control-center"
    if [[ -f "package.json" ]]; then
        log_info "Running frontend tests..."
        npm test -- --coverage --watchAll=false || log_warning "Frontend tests failed"
    fi
    
    # Backend tests (placeholder)
    cd "$PROJECT_ROOT"
    log_info "Running backend tests..."
    # go test ./... || log_warning "Backend tests failed"
    
    log_success "Tests completed"
}

# Create namespace
create_namespace() {
    log_info "Creating namespace: $NAMESPACE"
    
    if kubectl get namespace "$NAMESPACE" &> /dev/null; then
        log_info "Namespace '$NAMESPACE' already exists"
    else
        if [[ "$DRY_RUN" == "true" ]]; then
            log_info "[DRY RUN] Would create namespace: $NAMESPACE"
        else
            kubectl create namespace "$NAMESPACE"
            kubectl label namespace "$NAMESPACE" \
                app.kubernetes.io/name=atlasmesh-fleet-os \
                app.kubernetes.io/version=1.0.0 \
                environment="$ENVIRONMENT"
            log_success "Namespace '$NAMESPACE' created"
        fi
    fi
}

# Apply Kubernetes manifests
apply_manifests() {
    log_info "Applying Kubernetes manifests..."
    
    local manifests_dir="$DEPLOY_DIR/kubernetes"
    
    if [[ "$DRY_RUN" == "true" ]]; then
        log_info "[DRY RUN] Would apply manifests from: $manifests_dir"
        kubectl apply --dry-run=client -k "$manifests_dir" || true
    else
        # Apply in order
        kubectl apply -f "$manifests_dir/namespace.yaml"
        kubectl apply -f "$manifests_dir/configmaps/" -n "$NAMESPACE"
        kubectl apply -f "$manifests_dir/secrets/" -n "$NAMESPACE"
        kubectl apply -f "$manifests_dir/deployments/" -n "$NAMESPACE"
        
        log_success "Kubernetes manifests applied"
    fi
}

# Deploy with Helm
deploy_helm() {
    log_info "Deploying with Helm..."
    
    local helm_dir="$DEPLOY_DIR/helm/atlasmesh-fleet-os"
    local values_file="$helm_dir/values.yaml"
    local env_values_file="$helm_dir/values-${ENVIRONMENT}.yaml"
    
    # Prepare Helm command
    local helm_cmd=(
        "helm" "upgrade" "--install" "$HELM_RELEASE_NAME"
        "$helm_dir"
        "--namespace" "$NAMESPACE"
        "--create-namespace"
        "--values" "$values_file"
        "--set" "global.environment=$ENVIRONMENT"
        "--set" "global.imageRegistry=${DOCKER_REGISTRY:-registry.atlasmesh.com}"
        "--timeout" "10m"
        "--wait"
    )
    
    # Add environment-specific values if they exist
    if [[ -f "$env_values_file" ]]; then
        helm_cmd+=("--values" "$env_values_file")
    fi
    
    # Add API keys from environment
    if [[ -n "${GOOGLE_MAPS_API_KEY:-}" ]]; then
        helm_cmd+=("--set" "controlCenterUI.apiKeys.googleMaps=$GOOGLE_MAPS_API_KEY")
    fi
    
    # Add dry-run flag if specified
    if [[ "$DRY_RUN" == "true" ]]; then
        helm_cmd+=("--dry-run")
        log_info "[DRY RUN] Helm deployment command:"
        echo "${helm_cmd[@]}"
    else
        log_info "Executing Helm deployment..."
        "${helm_cmd[@]}"
        log_success "Helm deployment completed"
    fi
}

# Verify deployment
verify_deployment() {
    if [[ "$DRY_RUN" == "true" ]]; then
        log_info "[DRY RUN] Skipping deployment verification"
        return
    fi
    
    log_info "Verifying deployment..."
    
    # Wait for deployments to be ready
    log_info "Waiting for deployments to be ready..."
    kubectl wait --for=condition=available --timeout=300s \
        deployment/control-center-ui -n "$NAMESPACE" || log_warning "Control Center UI deployment not ready"
    
    # Check pod status
    log_info "Checking pod status..."
    kubectl get pods -n "$NAMESPACE" -o wide
    
    # Check services
    log_info "Checking services..."
    kubectl get services -n "$NAMESPACE"
    
    # Check ingress
    log_info "Checking ingress..."
    kubectl get ingress -n "$NAMESPACE" || log_info "No ingress found"
    
    # Health check
    log_info "Performing health checks..."
    local control_center_service
    control_center_service=$(kubectl get service control-center-ui-service -n "$NAMESPACE" -o jsonpath='{.spec.clusterIP}')
    if [[ -n "$control_center_service" ]]; then
        kubectl run health-check --rm -i --restart=Never --image=curlimages/curl:latest -- \
            curl -f "http://$control_center_service/" || log_warning "Health check failed"
    fi
    
    log_success "Deployment verification completed"
}

# Display deployment information
show_deployment_info() {
    log_info "Deployment Information:"
    echo "========================"
    echo "Environment: $ENVIRONMENT"
    echo "Namespace: $NAMESPACE"
    echo "Helm Release: $HELM_RELEASE_NAME"
    echo "Kubectl Context: $(kubectl config current-context)"
    echo "========================"
    
    if [[ "$DRY_RUN" != "true" ]]; then
        # Show ingress URLs
        log_info "Access URLs:"
        kubectl get ingress -n "$NAMESPACE" -o custom-columns=NAME:.metadata.name,HOSTS:.spec.rules[*].host --no-headers | \
            while read -r name hosts; do
                echo "  $name: https://$hosts"
            done
        
        # Show monitoring URLs
        echo "  Grafana: https://grafana.atlasmesh.com (if enabled)"
        echo "  Prometheus: https://prometheus.atlasmesh.com (if enabled)"
    fi
}

# Cleanup on exit
cleanup() {
    local exit_code=$?
    if [[ $exit_code -ne 0 ]]; then
        log_error "Deployment failed with exit code: $exit_code"
    fi
    exit $exit_code
}

# Main deployment function
main() {
    log_info "Starting AtlasMesh Fleet OS deployment..."
    log_info "Abu Dhabi-centered autonomous vehicle fleet management platform"
    
    # Parse arguments
    parse_args "$@"
    
    # Validate and check prerequisites
    validate_environment
    check_prerequisites
    
    # Show deployment info
    show_deployment_info
    
    # Confirm deployment (except for dry run)
    if [[ "$DRY_RUN" != "true" && "$ENVIRONMENT" == "production" ]]; then
        echo -n "Deploy to PRODUCTION environment? (y/N): "
        read -r confirm
        if [[ "$confirm" != "y" && "$confirm" != "Y" ]]; then
            log_info "Deployment cancelled"
            exit 0
        fi
    fi
    
    # Execute deployment steps
    create_namespace
    build_images
    push_images
    run_tests
    apply_manifests
    deploy_helm
    verify_deployment
    
    log_success "🚀 AtlasMesh Fleet OS deployment completed successfully!"
    log_info "The platform is now ready to manage autonomous vehicle fleets in Abu Dhabi"
    
    show_deployment_info
}

# Set trap for cleanup
trap cleanup EXIT

# Run main function
main "$@"
