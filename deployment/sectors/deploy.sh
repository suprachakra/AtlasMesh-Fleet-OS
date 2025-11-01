#!/bin/bash

# AtlasMesh Fleet OS - Sector-Specific Deployment Script
# This script deploys AtlasMesh Fleet OS for a specific sector

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Default values
SECTOR=""
ENVIRONMENT="production"
NAMESPACE=""
DRY_RUN=false
VERBOSE=false

# Function to print colored output
print_status() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Function to show usage
show_usage() {
    cat << EOF
Usage: $0 [OPTIONS]

Deploy AtlasMesh Fleet OS for a specific sector

OPTIONS:
    -s, --sector SECTOR        Sector to deploy (logistics, defense, mining, ride_hail)
    -e, --environment ENV      Environment (production, staging, development) [default: production]
    -n, --namespace NAMESPACE  Kubernetes namespace [default: atlasmesh-{sector}]
    -d, --dry-run             Perform a dry run without applying changes
    -v, --verbose             Enable verbose output
    -h, --help                Show this help message

EXAMPLES:
    $0 -s logistics
    $0 -s defense -e staging
    $0 -s mining -n custom-mining-namespace
    $0 -s ride_hail -d

EOF
}

# Function to validate sector
validate_sector() {
    case $SECTOR in
        logistics|defense|mining|ride_hail)
            print_success "Valid sector: $SECTOR"
            ;;
        *)
            print_error "Invalid sector: $SECTOR"
            print_error "Valid sectors: logistics, defense, mining, ride_hail"
            exit 1
            ;;
    esac
}

# Function to set namespace
set_namespace() {
    if [ -z "$NAMESPACE" ]; then
        NAMESPACE="atlasmesh-$SECTOR"
    fi
    print_status "Using namespace: $NAMESPACE"
}

# Function to check prerequisites
check_prerequisites() {
    print_status "Checking prerequisites..."
    
    # Check if kubectl is installed
    if ! command -v kubectl &> /dev/null; then
        print_error "kubectl is not installed or not in PATH"
        exit 1
    fi
    
    # Check if kubectl can connect to cluster
    if ! kubectl cluster-info &> /dev/null; then
        print_error "Cannot connect to Kubernetes cluster"
        exit 1
    fi
    
    # Check if sector directory exists
    if [ ! -d "deployments/sectors/$SECTOR" ]; then
        print_error "Sector deployment directory not found: deployments/sectors/$SECTOR"
        exit 1
    fi
    
    print_success "Prerequisites check passed"
}

# Function to deploy namespace
deploy_namespace() {
    print_status "Deploying namespace..."
    
    local namespace_file="deployments/sectors/$SECTOR/k8s/namespace.yaml"
    if [ ! -f "$namespace_file" ]; then
        print_error "Namespace file not found: $namespace_file"
        exit 1
    fi
    
    if [ "$DRY_RUN" = true ]; then
        print_status "Dry run: Would deploy namespace from $namespace_file"
        kubectl apply --dry-run=client -f "$namespace_file"
    else
        kubectl apply -f "$namespace_file"
        print_success "Namespace deployed successfully"
    fi
}

# Function to deploy secrets
deploy_secrets() {
    print_status "Deploying secrets..."
    
    local secrets_file="deployments/sectors/$SECTOR/k8s/secrets.yaml"
    if [ ! -f "$secrets_file" ]; then
        print_error "Secrets file not found: $secrets_file"
        exit 1
    fi
    
    if [ "$DRY_RUN" = true ]; then
        print_status "Dry run: Would deploy secrets from $secrets_file"
        kubectl apply --dry-run=client -f "$secrets_file"
    else
        kubectl apply -f "$secrets_file"
        print_success "Secrets deployed successfully"
    fi
}

# Function to deploy configmap
deploy_configmap() {
    print_status "Deploying configmap..."
    
    local configmap_file="deployments/sectors/$SECTOR/k8s/configmap.yaml"
    if [ ! -f "$configmap_file" ]; then
        print_error "Configmap file not found: $configmap_file"
        exit 1
    fi
    
    if [ "$DRY_RUN" = true ]; then
        print_status "Dry run: Would deploy configmap from $configmap_file"
        kubectl apply --dry-run=client -f "$configmap_file"
    else
        kubectl apply -f "$configmap_file"
        print_success "Configmap deployed successfully"
    fi
}

# Function to deploy services
deploy_services() {
    print_status "Deploying services..."
    
    local services_file="deployments/sectors/$SECTOR/k8s/services.yaml"
    if [ ! -f "$services_file" ]; then
        print_error "Services file not found: $services_file"
        exit 1
    fi
    
    if [ "$DRY_RUN" = true ]; then
        print_status "Dry run: Would deploy services from $services_file"
        kubectl apply --dry-run=client -f "$services_file"
    else
        kubectl apply -f "$services_file"
        print_success "Services deployed successfully"
    fi
}

# Function to deploy deployments
deploy_deployments() {
    print_status "Deploying deployments..."
    
    local deployments_file="deployments/sectors/$SECTOR/k8s/deployments.yaml"
    if [ ! -f "$deployments_file" ]; then
        print_error "Deployments file not found: $deployments_file"
        exit 1
    fi
    
    if [ "$DRY_RUN" = true ]; then
        print_status "Dry run: Would deploy deployments from $deployments_file"
        kubectl apply --dry-run=client -f "$deployments_file"
    else
        kubectl apply -f "$deployments_file"
        print_success "Deployments deployed successfully"
    fi
}

# Function to wait for deployments
wait_for_deployments() {
    if [ "$DRY_RUN" = true ]; then
        print_status "Dry run: Skipping deployment wait"
        return
    fi
    
    print_status "Waiting for deployments to be ready..."
    
    # Wait for all deployments in the namespace to be ready
    kubectl wait --for=condition=available --timeout=300s deployment --all -n "$NAMESPACE" || {
        print_warning "Some deployments may not be ready yet"
        print_status "Checking deployment status..."
        kubectl get deployments -n "$NAMESPACE"
    }
    
    print_success "Deployments are ready"
}

# Function to show deployment status
show_status() {
    print_status "Deployment status:"
    
    echo ""
    echo "Namespace: $NAMESPACE"
    echo "Sector: $SECTOR"
    echo "Environment: $ENVIRONMENT"
    echo ""
    
    if [ "$DRY_RUN" = true ]; then
        print_status "Dry run completed - no changes were applied"
        return
    fi
    
    # Show pods
    print_status "Pods:"
    kubectl get pods -n "$NAMESPACE" || true
    
    echo ""
    
    # Show services
    print_status "Services:"
    kubectl get services -n "$NAMESPACE" || true
    
    echo ""
    
    # Show deployments
    print_status "Deployments:"
    kubectl get deployments -n "$NAMESPACE" || true
}

# Function to cleanup on error
cleanup() {
    print_error "Deployment failed. Cleaning up..."
    # Add cleanup logic here if needed
    exit 1
}

# Set trap for cleanup
trap cleanup ERR

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -s|--sector)
            SECTOR="$2"
            shift 2
            ;;
        -e|--environment)
            ENVIRONMENT="$2"
            shift 2
            ;;
        -n|--namespace)
            NAMESPACE="$2"
            shift 2
            ;;
        -d|--dry-run)
            DRY_RUN=true
            shift
            ;;
        -v|--verbose)
            VERBOSE=true
            shift
            ;;
        -h|--help)
            show_usage
            exit 0
            ;;
        *)
            print_error "Unknown option: $1"
            show_usage
            exit 1
            ;;
    esac
done

# Check if sector is provided
if [ -z "$SECTOR" ]; then
    print_error "Sector is required"
    show_usage
    exit 1
fi

# Main deployment process
main() {
    print_status "Starting AtlasMesh Fleet OS deployment for sector: $SECTOR"
    
    # Validate inputs
    validate_sector
    set_namespace
    
    # Check prerequisites
    check_prerequisites
    
    # Deploy components in order
    deploy_namespace
    deploy_secrets
    deploy_configmap
    deploy_services
    deploy_deployments
    
    # Wait for deployments to be ready
    wait_for_deployments
    
    # Show final status
    show_status
    
    print_success "AtlasMesh Fleet OS deployment completed successfully for sector: $SECTOR"
}

# Run main function
main

