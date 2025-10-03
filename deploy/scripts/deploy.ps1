# AtlasMesh Fleet OS - PowerShell Deployment Script
# Deploys the complete fleet management platform to Kubernetes
# Supports multiple environments: development, staging, production

param(
    [Parameter(Mandatory=$false)]
    [ValidateSet("development", "staging", "production")]
    [string]$Environment = "staging",
    
    [Parameter(Mandatory=$false)]
    [string]$Namespace = "",
    
    [Parameter(Mandatory=$false)]
    [string]$HelmReleaseName = "atlasmesh-fleet-os",
    
    [Parameter(Mandatory=$false)]
    [string]$KubectlContext = "",
    
    [Parameter(Mandatory=$false)]
    [switch]$DryRun = $false,
    
    [Parameter(Mandatory=$false)]
    [switch]$SkipBuild = $false,
    
    [Parameter(Mandatory=$false)]
    [switch]$SkipTests = $false,
    
    [Parameter(Mandatory=$false)]
    [switch]$Help = $false
)

# Set default namespace if not provided
if ([string]::IsNullOrEmpty($Namespace)) {
    $Namespace = "atlasmesh-fleet-os-$Environment"
}

# Configuration
$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Definition
$ProjectRoot = Split-Path -Parent (Split-Path -Parent $ScriptDir)
$DeployDir = Join-Path $ProjectRoot "deploy"

# Colors for output (PowerShell compatible)
function Write-Info {
    param([string]$Message)
    Write-Host "[INFO] $Message" -ForegroundColor Blue
}

function Write-Success {
    param([string]$Message)
    Write-Host "[SUCCESS] $Message" -ForegroundColor Green
}

function Write-Warning {
    param([string]$Message)
    Write-Host "[WARNING] $Message" -ForegroundColor Yellow
}

function Write-Error {
    param([string]$Message)
    Write-Host "[ERROR] $Message" -ForegroundColor Red
}

# Help function
function Show-Help {
    @"
AtlasMesh Fleet OS Deployment Script (PowerShell)

Usage: .\deploy.ps1 [OPTIONS]

OPTIONS:
    -Environment ENV         Deployment environment (development|staging|production) [default: staging]
    -Namespace NAMESPACE     Kubernetes namespace [default: atlasmesh-fleet-os-ENV]
    -HelmReleaseName RELEASE Helm release name [default: atlasmesh-fleet-os]
    -KubectlContext CONTEXT  Kubectl context to use
    -DryRun                  Perform a dry run without making changes
    -SkipBuild              Skip building Docker images
    -SkipTests              Skip running tests
    -Help                   Show this help message

EXAMPLES:
    # Deploy to staging (default)
    .\deploy.ps1

    # Deploy to production
    .\deploy.ps1 -Environment production

    # Dry run deployment
    .\deploy.ps1 -DryRun

    # Deploy to specific namespace
    .\deploy.ps1 -Environment development -Namespace my-dev-namespace

ENVIRONMENT VARIABLES:
    GOOGLE_MAPS_API_KEY       Google Maps API key
    DOCKER_REGISTRY           Docker registry URL
    DOCKER_USERNAME           Docker registry username
    DOCKER_PASSWORD           Docker registry password
"@
}

# Check prerequisites
function Test-Prerequisites {
    Write-Info "Checking prerequisites..."
    
    # Check required commands
    $requiredCommands = @("kubectl", "helm", "docker")
    foreach ($cmd in $requiredCommands) {
        if (!(Get-Command $cmd -ErrorAction SilentlyContinue)) {
            Write-Error "Required command '$cmd' not found"
            exit 1
        }
    }
    
    # Check kubectl context
    if (![string]::IsNullOrEmpty($KubectlContext)) {
        Write-Info "Using kubectl context: $KubectlContext"
        kubectl config use-context $KubectlContext
    } else {
        $currentContext = kubectl config current-context
        Write-Info "Using current kubectl context: $currentContext"
    }
    
    # Test cluster connectivity
    $clusterInfo = kubectl cluster-info 2>$null
    if ($LASTEXITCODE -ne 0) {
        Write-Error "Cannot connect to Kubernetes cluster"
        exit 1
    }
    
    # Check Helm
    $helmVersion = helm version 2>$null
    if ($LASTEXITCODE -ne 0) {
        Write-Error "Helm is not properly configured"
        exit 1
    }
    
    Write-Success "Prerequisites check passed"
}

# Build Docker images
function Build-Images {
    if ($SkipBuild) {
        Write-Info "Skipping Docker image build"
        return
    }
    
    Write-Info "Building Docker images..."
    
    # Build Control Center UI
    Write-Info "Building Control Center UI..."
    $uiPath = Join-Path $ProjectRoot "ui\control-center"
    Push-Location $uiPath
    
    try {
        # Create production environment file
        $envFile = ".env.production"
        if (!(Test-Path $envFile)) {
            Write-Warning "Creating production environment file"
            @"
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
"@ | Out-File -FilePath $envFile -Encoding UTF8
        }
        
        # Build Docker image
        docker build -t "atlasmesh/control-center-ui:1.0.0" --build-arg NODE_ENV=production --build-arg REACT_APP_VERSION=1.0.0 -f Dockerfile .
        
        # Tag for registry
        if ($env:DOCKER_REGISTRY) {
            docker tag "atlasmesh/control-center-ui:1.0.0" "$($env:DOCKER_REGISTRY)/atlasmesh/control-center-ui:1.0.0"
        }
    }
    finally {
        Pop-Location
    }
    
    Write-Success "Docker images built successfully"
}

# Create namespace
function New-KubernetesNamespace {
    Write-Info "Creating namespace: $Namespace"
    
    $existingNamespace = kubectl get namespace $Namespace 2>$null
    if ($LASTEXITCODE -eq 0) {
        Write-Info "Namespace '$Namespace' already exists"
    } else {
        if ($DryRun) {
            Write-Info "[DRY RUN] Would create namespace: $Namespace"
        } else {
            kubectl create namespace $Namespace
            kubectl label namespace $Namespace app.kubernetes.io/name=atlasmesh-fleet-os app.kubernetes.io/version=1.0.0 environment=$Environment
            Write-Success "Namespace '$Namespace' created"
        }
    }
}

# Deploy with Helm
function Deploy-Helm {
    Write-Info "Deploying with Helm..."
    
    $helmDir = Join-Path $DeployDir "helm\atlasmesh-fleet-os"
    $valuesFile = Join-Path $helmDir "values.yaml"
    
    # Prepare Helm command arguments
    $helmArgs = @(
        "upgrade", "--install", $HelmReleaseName,
        $helmDir,
        "--namespace", $Namespace,
        "--create-namespace",
        "--values", $valuesFile,
        "--set", "global.environment=$Environment",
        "--timeout", "10m",
        "--wait"
    )
    
    # Add registry if specified
    if ($env:DOCKER_REGISTRY) {
        $helmArgs += "--set"
        $helmArgs += "global.imageRegistry=$($env:DOCKER_REGISTRY)"
    }
    
    # Add API keys from environment
    if ($env:GOOGLE_MAPS_API_KEY) {
        $helmArgs += "--set"
        $helmArgs += "controlCenterUI.apiKeys.googleMaps=$($env:GOOGLE_MAPS_API_KEY)"
    }
    
    # Add dry-run flag if specified
    if ($DryRun) {
        $helmArgs += "--dry-run"
        Write-Info "[DRY RUN] Helm deployment command: helm $($helmArgs -join ' ')"
    } else {
        Write-Info "Executing Helm deployment..."
        & helm $helmArgs
        if ($LASTEXITCODE -eq 0) {
            Write-Success "Helm deployment completed"
        } else {
            Write-Error "Helm deployment failed"
            exit 1
        }
    }
}

# Verify deployment
function Test-Deployment {
    if ($DryRun) {
        Write-Info "[DRY RUN] Skipping deployment verification"
        return
    }
    
    Write-Info "Verifying deployment..."
    
    # Wait for deployments to be ready
    Write-Info "Waiting for deployments to be ready..."
    kubectl wait --for=condition=available --timeout=300s deployment/control-center-ui -n $Namespace
    if ($LASTEXITCODE -ne 0) {
        Write-Warning "Control Center UI deployment not ready"
    }
    
    # Check pod status
    Write-Info "Checking pod status..."
    kubectl get pods -n $Namespace -o wide
    
    # Check services
    Write-Info "Checking services..."
    kubectl get services -n $Namespace
    
    Write-Success "Deployment verification completed"
}

# Show deployment information
function Show-DeploymentInfo {
    Write-Info "Deployment Information:"
    Write-Host "========================"
    Write-Host "Environment: $Environment"
    Write-Host "Namespace: $Namespace"
    Write-Host "Helm Release: $HelmReleaseName"
    $currentContext = kubectl config current-context
    Write-Host "Kubectl Context: $currentContext"
    Write-Host "========================"
    
    if (!$DryRun) {
        Write-Info "Access URLs:"
        Write-Host "  Control Center: https://app.atlasmesh.com"
        Write-Host "  Grafana: https://grafana.atlasmesh.com (if enabled)"
        Write-Host "  Prometheus: https://prometheus.atlasmesh.com (if enabled)"
    }
}

# Main deployment function
function Main {
    if ($Help) {
        Show-Help
        return
    }
    
    Write-Info "Starting AtlasMesh Fleet OS deployment..."
    Write-Info "Abu Dhabi-centered autonomous vehicle fleet management platform"
    
    # Validate environment
    if ($Environment -notin @("development", "staging", "production")) {
        Write-Error "Invalid environment: $Environment"
        Write-Error "Valid environments: development, staging, production"
        exit 1
    }
    
    Write-Success "Environment '$Environment' is valid"
    
    # Check prerequisites
    Test-Prerequisites
    
    # Show deployment info
    Show-DeploymentInfo
    
    # Confirm production deployment
    if (!$DryRun -and $Environment -eq "production") {
        $confirm = Read-Host "Deploy to PRODUCTION environment? (y/N)"
        if ($confirm -ne "y" -and $confirm -ne "Y") {
            Write-Info "Deployment cancelled"
            exit 0
        }
    }
    
    try {
        # Execute deployment steps
        New-KubernetesNamespace
        Build-Images
        Deploy-Helm
        Test-Deployment
        
        Write-Success "🚀 AtlasMesh Fleet OS deployment completed successfully!"
        Write-Info "The platform is now ready to manage autonomous vehicle fleets in Abu Dhabi"
        
        Show-DeploymentInfo
    }
    catch {
        Write-Error "Deployment failed: $_"
        exit 1
    }
}

# Run main function
Main
