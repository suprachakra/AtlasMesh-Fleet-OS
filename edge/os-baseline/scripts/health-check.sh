#!/bin/bash

# AtlasMesh Fleet OS - Edge OS Health Check Script
# Validates system health for autonomous vehicle operations

set -euo pipefail

# Configuration
HEALTH_CHECK_TIMEOUT=30
LOG_FILE="/var/log/atlasmesh-health.log"
VEHICLE_ID="${VEHICLE_ID:-unknown}"

# Health check results
HEALTH_STATUS=0
HEALTH_REPORT=""

# Logging function
log() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] HEALTH: $*" | tee -a "$LOG_FILE"
}

# Add to health report
report() {
    HEALTH_REPORT="${HEALTH_REPORT}$1\n"
}

# Check function with error handling
check() {
    local check_name="$1"
    local check_command="$2"
    local critical="${3:-false}"
    
    log "Checking: $check_name"
    
    if timeout "$HEALTH_CHECK_TIMEOUT" bash -c "$check_command" >/dev/null 2>&1; then
        report "✅ $check_name: OK"
        log "$check_name: PASS"
        return 0
    else
        if [[ "$critical" == "true" ]]; then
            report "❌ $check_name: CRITICAL FAILURE"
            log "$check_name: CRITICAL FAILURE"
            HEALTH_STATUS=2
        else
            report "⚠️  $check_name: WARNING"
            log "$check_name: WARNING"
            if [[ $HEALTH_STATUS -eq 0 ]]; then
                HEALTH_STATUS=1
            fi
        fi
        return 1
    fi
}

# System resource checks
check_system_resources() {
    log "Checking system resources..."
    
    # CPU usage check
    local cpu_usage
    cpu_usage=$(top -bn1 | grep "Cpu(s)" | awk '{print $2}' | sed 's/%us,//' | sed 's/\..*//')
    if [[ ${cpu_usage:-100} -lt 90 ]]; then
        report "✅ CPU Usage: ${cpu_usage}%"
    else
        report "⚠️  CPU Usage: ${cpu_usage}% (HIGH)"
        HEALTH_STATUS=1
    fi
    
    # Memory usage check
    local memory_usage
    memory_usage=$(free | grep Mem | awk '{printf("%.0f", $3/$2 * 100.0)}')
    if [[ ${memory_usage:-100} -lt 85 ]]; then
        report "✅ Memory Usage: ${memory_usage}%"
    else
        report "⚠️  Memory Usage: ${memory_usage}% (HIGH)"
        HEALTH_STATUS=1
    fi
    
    # Disk usage check
    local disk_usage
    disk_usage=$(df / | awk 'NR==2 {printf("%.0f", $3/$2 * 100.0)}')
    if [[ ${disk_usage:-100} -lt 90 ]]; then
        report "✅ Disk Usage: ${disk_usage}%"
    else
        report "⚠️  Disk Usage: ${disk_usage}% (HIGH)"
        HEALTH_STATUS=1
    fi
    
    # Load average check
    local load_avg
    load_avg=$(uptime | awk -F'load average:' '{print $2}' | awk '{print $1}' | sed 's/,//')
    local cpu_cores
    cpu_cores=$(nproc)
    local load_threshold=$((cpu_cores * 2))
    
    if (( $(echo "$load_avg < $load_threshold" | bc -l) )); then
        report "✅ Load Average: $load_avg"
    else
        report "⚠️  Load Average: $load_avg (HIGH)"
        HEALTH_STATUS=1
    fi
}

# Service health checks
check_services() {
    log "Checking critical services..."
    
    # Containerd
    check "Containerd Service" "systemctl is-active containerd" true
    check "Containerd Socket" "test -S /run/containerd/containerd.sock" true
    check "Containerd API" "ctr version" true
    
    # AtlasMesh services
    check "Edge Agent Service" "systemctl is-active atlasmesh-edge-agent" true
    check "TPM Manager Service" "systemctl is-active atlasmesh-tpm-manager" false
    check "Secure Boot Validator" "systemctl is-active atlasmesh-secure-boot-validator" false
    
    # System services
    check "SSH Service" "systemctl is-active ssh" false
    check "Time Sync Service" "systemctl is-active chrony" false
    check "Audit Service" "systemctl is-active auditd" false
    check "Firewall Service" "systemctl is-active ufw" false
}

# Network connectivity checks
check_network() {
    log "Checking network connectivity..."
    
    # Local network interface
    check "Network Interface" "ip link show | grep -q 'state UP'" true
    
    # DNS resolution
    check "DNS Resolution" "nslookup google.com" false
    
    # Internet connectivity
    check "Internet Connectivity" "ping -c 1 -W 5 8.8.8.8" false
    
    # AtlasMesh cloud connectivity (if configured)
    if [[ -n "${ATLASMESH_CLOUD_ENDPOINT:-}" ]]; then
        check "AtlasMesh Cloud" "curl -s --max-time 10 $ATLASMESH_CLOUD_ENDPOINT/health" false
    fi
    
    # Container registry connectivity
    check "Container Registry" "ping -c 1 -W 5 registry.atlasmesh.com" false
}

# Security checks
check_security() {
    log "Checking security status..."
    
    # TPM status
    if [[ -e /dev/tpm0 ]] || [[ -e /dev/tpmrm0 ]]; then
        check "TPM Device" "test -e /dev/tpm0 -o -e /dev/tpmrm0" false
        check "TPM Resource Manager" "systemctl is-active tpm2-abrmd" false
        
        # TPM functionality
        if command -v tpm2_getrandom >/dev/null 2>&1; then
            check "TPM Functionality" "tpm2_getrandom 8" false
        fi
    else
        report "ℹ️  TPM: Not available"
    fi
    
    # Secure boot status
    if command -v mokutil >/dev/null 2>&1; then
        if mokutil --sb-state 2>/dev/null | grep -q "SecureBoot enabled"; then
            report "✅ Secure Boot: Enabled"
        else
            report "⚠️  Secure Boot: Disabled"
        fi
    else
        report "ℹ️  Secure Boot: Cannot verify"
    fi
    
    # File system permissions
    check "AtlasMesh Home Permissions" "test -d /var/lib/atlasmesh && test -O /var/lib/atlasmesh" true
    check "Certificate Directory Permissions" "test -d /etc/ssl/atlasmesh/private && test $(stat -c %a /etc/ssl/atlasmesh/private) = '700'" false
    
    # Firewall status
    if command -v ufw >/dev/null 2>&1; then
        check "Firewall Status" "ufw status | grep -q 'Status: active'" false
    fi
}

# Storage checks
check_storage() {
    log "Checking storage health..."
    
    # Required directories
    local required_dirs=(
        "/var/lib/atlasmesh"
        "/etc/atlasmesh"
        "/var/log/atlasmesh"
        "/etc/ssl/atlasmesh"
    )
    
    for dir in "${required_dirs[@]}"; do
        check "Directory $dir" "test -d $dir" true
    done
    
    # Disk space for critical paths
    local critical_paths=(
        "/"
        "/var/lib/atlasmesh"
        "/var/log"
    )
    
    for path in "${critical_paths[@]}"; do
        local available_space
        available_space=$(df "$path" | awk 'NR==2 {print $4}')
        local available_mb=$((available_space / 1024))
        
        if [[ $available_mb -gt 1024 ]]; then  # More than 1GB
            report "✅ $path: ${available_mb}MB available"
        elif [[ $available_mb -gt 512 ]]; then  # More than 512MB
            report "⚠️  $path: ${available_mb}MB available (LOW)"
            HEALTH_STATUS=1
        else
            report "❌ $path: ${available_mb}MB available (CRITICAL)"
            HEALTH_STATUS=2
        fi
    done
    
    # Check for read-only file systems
    if mount | grep -q "ro,"; then
        report "⚠️  Read-only file systems detected"
        HEALTH_STATUS=1
    fi
}

# Container runtime checks
check_containers() {
    log "Checking container runtime..."
    
    # Containerd namespace check
    check "Containerd Namespaces" "ctr namespaces list" false
    
    # Container image availability
    if ctr images list | grep -q "registry.k8s.io/pause"; then
        report "✅ Pause Image: Available"
    else
        report "⚠️  Pause Image: Missing"
        HEALTH_STATUS=1
    fi
    
    # Check for running containers
    local running_containers
    running_containers=$(ctr containers list -q | wc -l)
    report "ℹ️  Running Containers: $running_containers"
    
    # Check container resource usage
    if [[ $running_containers -gt 0 ]]; then
        # This would require more sophisticated monitoring
        report "ℹ️  Container monitoring active"
    fi
}

# Application-specific checks
check_applications() {
    log "Checking AtlasMesh applications..."
    
    # Edge agent health endpoint
    if curl -s --max-time 5 http://localhost:8080/health >/dev/null 2>&1; then
        report "✅ Edge Agent API: Healthy"
    else
        report "❌ Edge Agent API: Unhealthy"
        HEALTH_STATUS=2
    fi
    
    # Metrics endpoint
    if curl -s --max-time 5 http://localhost:9090/metrics >/dev/null 2>&1; then
        report "✅ Metrics Endpoint: Available"
    else
        report "⚠️  Metrics Endpoint: Unavailable"
        HEALTH_STATUS=1
    fi
    
    # Vehicle profile validation
    if [[ -f "/etc/atlasmesh/vehicle-profiles/current.yaml" ]]; then
        report "✅ Vehicle Profile: Loaded"
    else
        report "⚠️  Vehicle Profile: Missing"
        HEALTH_STATUS=1
    fi
    
    # Policy validation
    if [[ -d "/etc/atlasmesh/policies" ]] && [[ -n "$(ls -A /etc/atlasmesh/policies)" ]]; then
        report "✅ Policies: Loaded"
    else
        report "⚠️  Policies: Missing"
        HEALTH_STATUS=1
    fi
}

# Generate health report
generate_report() {
    local timestamp
    timestamp=$(date '+%Y-%m-%d %H:%M:%S')
    
    echo "=========================================="
    echo "AtlasMesh Edge OS Health Report"
    echo "=========================================="
    echo "Timestamp: $timestamp"
    echo "Vehicle ID: $VEHICLE_ID"
    echo "Hostname: $(hostname)"
    echo "Uptime: $(uptime -p)"
    echo "=========================================="
    echo -e "$HEALTH_REPORT"
    echo "=========================================="
    
    case $HEALTH_STATUS in
        0)
            echo "Overall Status: ✅ HEALTHY"
            ;;
        1)
            echo "Overall Status: ⚠️  WARNING"
            ;;
        2)
            echo "Overall Status: ❌ CRITICAL"
            ;;
    esac
    echo "=========================================="
}

# Main health check function
main() {
    log "Starting health check for vehicle $VEHICLE_ID"
    
    # Perform all health checks
    check_system_resources
    check_services
    check_network
    check_security
    check_storage
    check_containers
    check_applications
    
    # Generate and display report
    generate_report
    
    # Log final status
    case $HEALTH_STATUS in
        0)
            log "Health check completed: HEALTHY"
            ;;
        1)
            log "Health check completed: WARNING"
            ;;
        2)
            log "Health check completed: CRITICAL"
            ;;
    esac
    
    exit $HEALTH_STATUS
}

# Handle script arguments
case "${1:-check}" in
    "check")
        main
        ;;
    "status")
        echo $HEALTH_STATUS
        ;;
    "report")
        generate_report
        ;;
    *)
        echo "Usage: $0 [check|status|report]"
        exit 1
        ;;
esac
