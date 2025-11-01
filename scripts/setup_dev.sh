#!/bin/bash

# AtlasMesh Fleet OS - Development Environment Setup
# Sets up complete development environment with all dependencies

set -euo pipefail

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Configuration
PROJECT_NAME="AtlasMesh Fleet OS"
MIN_NODE_VERSION="18.0.0"
MIN_GO_VERSION="1.21.0"
MIN_PYTHON_VERSION="3.10.0"
MIN_DOCKER_VERSION="20.10.0"

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

# Version comparison function
version_compare() {
    if [[ $1 == $2 ]]; then
        return 0
    fi
    local IFS=.
    local i ver1=($1) ver2=($2)
    # fill empty fields in ver1 with zeros
    for ((i=${#ver1[@]}; i<${#ver2[@]}; i++)); do
        ver1[i]=0
    done
    for ((i=0; i<${#ver1[@]}; i++)); do
        if [[ -z ${ver2[i]} ]]; then
            # fill empty fields in ver2 with zeros
            ver2[i]=0
        fi
        if ((10#${ver1[i]} > 10#${ver2[i]})); then
            return 1
        fi
        if ((10#${ver1[i]} < 10#${ver2[i]})); then
            return 2
        fi
    done
    return 0
}

# Check if command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Check system requirements
check_system_requirements() {
    log_info "Checking system requirements..."
    
    # Check OS
    if [[ "$OSTYPE" == "linux-gnu"* ]]; then
        OS="linux"
        log_info "Detected Linux system"
    elif [[ "$OSTYPE" == "darwin"* ]]; then
        OS="macos"
        log_info "Detected macOS system"
    elif [[ "$OSTYPE" == "msys" ]] || [[ "$OSTYPE" == "cygwin" ]]; then
        OS="windows"
        log_info "Detected Windows system"
    else
        log_error "Unsupported operating system: $OSTYPE"
        exit 1
    fi
    
    # Check available memory (minimum 8GB recommended)
    if command_exists free; then
        MEMORY_GB=$(free -g | awk '/^Mem:/{print $2}')
        if [[ $MEMORY_GB -lt 8 ]]; then
            log_warning "System has ${MEMORY_GB}GB RAM. 8GB+ recommended for development."
        else
            log_success "System has ${MEMORY_GB}GB RAM"
        fi
    fi
    
    # Check available disk space (minimum 20GB recommended)
    DISK_SPACE_GB=$(df -BG . | awk 'NR==2 {print $4}' | sed 's/G//')
    if [[ $DISK_SPACE_GB -lt 20 ]]; then
        log_warning "Available disk space: ${DISK_SPACE_GB}GB. 20GB+ recommended."
    else
        log_success "Available disk space: ${DISK_SPACE_GB}GB"
    fi
}

# Install Node.js
install_nodejs() {
    log_info "Checking Node.js installation..."
    
    if command_exists node; then
        NODE_VERSION=$(node --version | sed 's/v//')
        version_compare $NODE_VERSION $MIN_NODE_VERSION
        case $? in
            0|1) log_success "Node.js $NODE_VERSION is installed and meets requirements" ;;
            2) 
                log_warning "Node.js $NODE_VERSION is installed but version $MIN_NODE_VERSION+ is required"
                log_info "Please update Node.js to continue"
                return 1
                ;;
        esac
    else
        log_warning "Node.js not found. Please install Node.js $MIN_NODE_VERSION or later"
        log_info "Visit: https://nodejs.org/"
        return 1
    fi
    
    # Check npm
    if command_exists npm; then
        NPM_VERSION=$(npm --version)
        log_success "npm $NPM_VERSION is installed"
    else
        log_error "npm not found. Please install npm"
        return 1
    fi
    
    # Install global packages
    log_info "Installing global npm packages..."
    npm install -g @nx/cli@latest
    npm install -g @axe-core/cli
    npm install -g typescript
    
    log_success "Node.js environment ready"
}

# Install Go
install_go() {
    log_info "Checking Go installation..."
    
    if command_exists go; then
        GO_VERSION=$(go version | awk '{print $3}' | sed 's/go//')
        version_compare $GO_VERSION $MIN_GO_VERSION
        case $? in
            0|1) log_success "Go $GO_VERSION is installed and meets requirements" ;;
            2) 
                log_warning "Go $GO_VERSION is installed but version $MIN_GO_VERSION+ is required"
                log_info "Please update Go to continue"
                return 1
                ;;
        esac
    else
        log_warning "Go not found. Please install Go $MIN_GO_VERSION or later"
        log_info "Visit: https://golang.org/dl/"
        return 1
    fi
    
    # Set up Go environment
    if [[ -z "${GOPATH:-}" ]]; then
        export GOPATH=$HOME/go
        log_info "GOPATH set to $GOPATH"
    fi
    
    # Install Go tools
    log_info "Installing Go development tools..."
    go install golang.org/x/tools/cmd/goimports@latest
    go install github.com/golangci/golangci-lint/cmd/golangci-lint@latest
    go install github.com/securecodewarrior/sast-scan@latest
    
    log_success "Go environment ready"
}

# Install Python
install_python() {
    log_info "Checking Python installation..."
    
    if command_exists python3; then
        PYTHON_VERSION=$(python3 --version | awk '{print $2}')
        version_compare $PYTHON_VERSION $MIN_PYTHON_VERSION
        case $? in
            0|1) log_success "Python $PYTHON_VERSION is installed and meets requirements" ;;
            2) 
                log_warning "Python $PYTHON_VERSION is installed but version $MIN_PYTHON_VERSION+ is required"
                log_info "Please update Python to continue"
                return 1
                ;;
        esac
    else
        log_warning "Python 3 not found. Please install Python $MIN_PYTHON_VERSION or later"
        log_info "Visit: https://python.org/downloads/"
        return 1
    fi
    
    # Check pip
    if command_exists pip3; then
        PIP_VERSION=$(pip3 --version | awk '{print $2}')
        log_success "pip $PIP_VERSION is installed"
    else
        log_error "pip3 not found. Please install pip"
        return 1
    fi
    
    # Create virtual environment
    if [[ ! -d ".venv" ]]; then
        log_info "Creating Python virtual environment..."
        python3 -m venv .venv
    fi
    
    # Activate virtual environment
    source .venv/bin/activate
    
    # Install Python development tools
    log_info "Installing Python development tools..."
    pip install --upgrade pip
    pip install black flake8 mypy pytest pytest-cov
    pip install pre-commit
    
    log_success "Python environment ready"
}

# Install Docker
install_docker() {
    log_info "Checking Docker installation..."
    
    if command_exists docker; then
        DOCKER_VERSION=$(docker --version | awk '{print $3}' | sed 's/,//')
        version_compare $DOCKER_VERSION $MIN_DOCKER_VERSION
        case $? in
            0|1) log_success "Docker $DOCKER_VERSION is installed and meets requirements" ;;
            2) 
                log_warning "Docker $DOCKER_VERSION is installed but version $MIN_DOCKER_VERSION+ is required"
                log_info "Please update Docker to continue"
                return 1
                ;;
        esac
        
        # Check if Docker daemon is running
        if docker info >/dev/null 2>&1; then
            log_success "Docker daemon is running"
        else
            log_warning "Docker daemon is not running. Please start Docker"
            return 1
        fi
    else
        log_warning "Docker not found. Please install Docker $MIN_DOCKER_VERSION or later"
        log_info "Visit: https://docs.docker.com/get-docker/"
        return 1
    fi
    
    # Check Docker Compose
    if command_exists docker-compose || docker compose version >/dev/null 2>&1; then
        log_success "Docker Compose is available"
    else
        log_warning "Docker Compose not found. Please install Docker Compose"
        return 1
    fi
}

# Install additional tools
install_additional_tools() {
    log_info "Installing additional development tools..."
    
    # Install kubectl if not present
    if ! command_exists kubectl; then
        log_info "Installing kubectl..."
        case $OS in
            "linux")
                curl -LO "https://dl.k8s.io/release/$(curl -L -s https://dl.k8s.io/release/stable.txt)/bin/linux/amd64/kubectl"
                chmod +x kubectl
                sudo mv kubectl /usr/local/bin/
                ;;
            "macos")
                if command_exists brew; then
                    brew install kubectl
                else
                    log_warning "Homebrew not found. Please install kubectl manually"
                fi
                ;;
            "windows")
                log_info "Please install kubectl manually from https://kubernetes.io/docs/tasks/tools/install-kubectl-windows/"
                ;;
        esac
    else
        log_success "kubectl is already installed"
    fi
    
    # Install helm if not present
    if ! command_exists helm; then
        log_info "Installing Helm..."
        case $OS in
            "linux"|"macos")
                curl https://raw.githubusercontent.com/helm/helm/main/scripts/get-helm-3 | bash
                ;;
            "windows")
                log_info "Please install Helm manually from https://helm.sh/docs/intro/install/"
                ;;
        esac
    else
        log_success "Helm is already installed"
    fi
    
    # Install jq for JSON processing
    if ! command_exists jq; then
        log_info "Installing jq..."
        case $OS in
            "linux")
                sudo apt-get update && sudo apt-get install -y jq
                ;;
            "macos")
                if command_exists brew; then
                    brew install jq
                else
                    log_warning "Homebrew not found. Please install jq manually"
                fi
                ;;
            "windows")
                log_info "Please install jq manually from https://stedolan.github.io/jq/download/"
                ;;
        esac
    else
        log_success "jq is already installed"
    fi
}

# Set up project dependencies
setup_project_dependencies() {
    log_info "Setting up project dependencies..."
    
    # Install npm dependencies
    if [[ -f "package.json" ]]; then
        log_info "Installing npm dependencies..."
        npm ci
        log_success "npm dependencies installed"
    fi
    
    # Install Go dependencies
    log_info "Installing Go dependencies..."
    cd services
    find . -name "go.mod" -execdir go mod download \;
    cd ..
    log_success "Go dependencies installed"
    
    # Install Python dependencies
    if [[ -f "requirements.txt" ]]; then
        log_info "Installing Python dependencies..."
        source .venv/bin/activate
        pip install -r requirements.txt
        log_success "Python dependencies installed"
    fi
}

# Set up development tools
setup_development_tools() {
    log_info "Setting up development tools..."
    
    # Set up pre-commit hooks
    if command_exists pre-commit; then
        log_info "Installing pre-commit hooks..."
        pre-commit install
        log_success "Pre-commit hooks installed"
    fi
    
    # Create necessary directories
    log_info "Creating project directories..."
    mkdir -p ci/reports
    mkdir -p ci/performance-budgets
    mkdir -p ci/accessibility-reports
    mkdir -p ci/contract-tests
    mkdir -p compliance/evidence/bundles
    mkdir -p compliance/evidence/artifacts
    mkdir -p security/sbom
    mkdir -p logs
    
    # Set up environment files
    if [[ ! -f ".env" ]]; then
        log_info "Creating environment file..."
        cp configs/env/.env.example .env
        log_info "Please edit .env file with your configuration"
    fi
    
    # Make scripts executable
    log_info "Making scripts executable..."
    find scripts -name "*.sh" -exec chmod +x {} \;
    chmod +x tools/*.js
    
    log_success "Development tools configured"
}

# Validate setup
validate_setup() {
    log_info "Validating development environment setup..."
    
    local validation_failed=false
    
    # Test Node.js setup
    if ! npm --version >/dev/null 2>&1; then
        log_error "npm validation failed"
        validation_failed=true
    fi
    
    # Test Go setup
    if ! go version >/dev/null 2>&1; then
        log_error "Go validation failed"
        validation_failed=true
    fi
    
    # Test Python setup
    if ! python3 --version >/dev/null 2>&1; then
        log_error "Python validation failed"
        validation_failed=true
    fi
    
    # Test Docker setup
    if ! docker --version >/dev/null 2>&1; then
        log_error "Docker validation failed"
        validation_failed=true
    fi
    
    # Test project structure
    local required_dirs=("services" "ui" "adapters" "configs" "docs")
    for dir in "${required_dirs[@]}"; do
        if [[ ! -d "$dir" ]]; then
            log_error "Required directory missing: $dir"
            validation_failed=true
        fi
    done
    
    if [[ "$validation_failed" == "true" ]]; then
        log_error "Development environment validation failed"
        return 1
    fi
    
    log_success "Development environment validation passed"
}

# Run a quick test
run_quick_test() {
    log_info "Running quick validation test..."
    
    # Test variant budget analyzer
    if node tools/variant-budget-analyzer.js >/dev/null 2>&1; then
        log_success "Variant budget analyzer test passed"
    else
        log_warning "Variant budget analyzer test failed (this is expected on first run)"
    fi
    
    # Test evidence bundle generator
    if node tools/evidence-bundle-generator.js >/dev/null 2>&1; then
        log_success "Evidence bundle generator test passed"
    else
        log_warning "Evidence bundle generator test failed"
    fi
    
    log_success "Quick validation completed"
}

# Main setup function
main() {
    echo -e "${BLUE}"
    echo "=============================================="
    echo "  $PROJECT_NAME Development Setup"
    echo "=============================================="
    echo -e "${NC}"
    
    # Check if running in project root
    if [[ ! -f "package.json" ]] || [[ ! -d "services" ]]; then
        log_error "Please run this script from the project root directory"
        exit 1
    fi
    
    # Run setup steps
    check_system_requirements
    
    if ! install_nodejs; then
        log_error "Node.js setup failed. Please install Node.js and try again."
        exit 1
    fi
    
    if ! install_go; then
        log_error "Go setup failed. Please install Go and try again."
        exit 1
    fi
    
    if ! install_python; then
        log_error "Python setup failed. Please install Python and try again."
        exit 1
    fi
    
    if ! install_docker; then
        log_error "Docker setup failed. Please install Docker and try again."
        exit 1
    fi
    
    install_additional_tools
    setup_project_dependencies
    setup_development_tools
    
    if ! validate_setup; then
        log_error "Setup validation failed. Please check the errors above."
        exit 1
    fi
    
    run_quick_test
    
    echo -e "${GREEN}"
    echo "=============================================="
    echo "  Setup Complete!"
    echo "=============================================="
    echo -e "${NC}"
    
    log_success "Development environment is ready!"
    echo ""
    log_info "Next steps:"
    echo "  1. Edit .env file with your configuration"
    echo "  2. Run 'make dev' to start the development environment"
    echo "  3. Visit http://localhost:3000 for the Control Center"
    echo "  4. Run 'make help' to see all available commands"
    echo ""
    log_info "Documentation: https://docs.atlasmesh.io"
    log_info "Support: https://github.com/atlasmesh/fleet-os/issues"
}

# Run main function
main "$@"
