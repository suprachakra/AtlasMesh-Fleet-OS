#!/bin/bash

# AtlasMesh Fleet OS - Edge OS Startup Script
# Initializes hardened Ubuntu LTS edge system for autonomous vehicle operations

set -euo pipefail

# Configuration
ATLASMESH_HOME="/var/lib/atlasmesh"
ATLASMESH_CONFIG="/etc/atlasmesh"
LOG_FILE="/var/log/atlasmesh-startup.log"
VEHICLE_ID="${VEHICLE_ID:-unknown}"
SECTOR="${SECTOR:-general}"
ENVIRONMENT="${ENVIRONMENT:-production}"

# Logging function
log() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] $*" | tee -a "$LOG_FILE"
}

# Error handling
error_exit() {
    log "ERROR: $1"
    exit 1
}

# Signal handlers
cleanup() {
    log "Received shutdown signal, cleaning up..."
    # Graceful shutdown of services
    systemctl stop atlasmesh-edge-agent || true
    systemctl stop atlasmesh-tpm-manager || true
    exit 0
}

trap cleanup SIGTERM SIGINT

# Main startup function
main() {
    log "Starting AtlasMesh Edge OS initialization..."
    log "Vehicle ID: $VEHICLE_ID"
    log "Sector: $SECTOR"
    log "Environment: $ENVIRONMENT"

    # Validate system requirements
    validate_system

    # Initialize TPM
    initialize_tpm

    # Validate secure boot
    validate_secure_boot

    # Configure networking
    configure_networking

    # Initialize container runtime
    initialize_containerd

    # Configure security
    configure_security

    # Initialize monitoring
    initialize_monitoring

    # Start AtlasMesh services
    start_atlasmesh_services

    # Health check
    perform_health_check

    log "AtlasMesh Edge OS initialization completed successfully"

    # Keep the container running
    wait_for_shutdown
}

# Validate system requirements
validate_system() {
    log "Validating system requirements..."

    # Check if running as root
    if [[ $EUID -ne 0 ]]; then
        error_exit "This script must be run as root"
    fi

    # Check Ubuntu version
    if ! grep -q "Ubuntu 22.04" /etc/os-release; then
        log "WARNING: Not running on Ubuntu 22.04 LTS"
    fi

    # Check required directories
    mkdir -p "$ATLASMESH_HOME"/{data,logs,config,certs}
    mkdir -p "$ATLASMESH_CONFIG"/{vehicle-profiles,policies,certificates}
    mkdir -p /var/log/atlasmesh

    # Set proper permissions
    chown -R atlasmesh:atlasmesh "$ATLASMESH_HOME"
    chown -R atlasmesh:atlasmesh "$ATLASMESH_CONFIG"
    chmod 700 "$ATLASMESH_HOME"/certs
    chmod 700 /etc/ssl/atlasmesh/private

    # Check disk space
    local available_space
    available_space=$(df / | awk 'NR==2 {print $4}')
    if [[ $available_space -lt 1048576 ]]; then  # 1GB in KB
        log "WARNING: Low disk space available: ${available_space}KB"
    fi

    # Check memory
    local available_memory
    available_memory=$(free -m | awk 'NR==2 {print $7}')
    if [[ $available_memory -lt 512 ]]; then
        log "WARNING: Low memory available: ${available_memory}MB"
    fi

    log "System validation completed"
}

# Initialize TPM
initialize_tpm() {
    log "Initializing TPM 2.0..."

    # Check if TPM device exists
    if [[ ! -e /dev/tpm0 ]] && [[ ! -e /dev/tpmrm0 ]]; then
        log "WARNING: TPM device not found, running without TPM support"
        return 0
    fi

    # Start TPM resource manager
    if ! systemctl is-active --quiet tpm2-abrmd; then
        systemctl start tpm2-abrmd || log "WARNING: Failed to start TPM resource manager"
    fi

    # Initialize TPM if needed
    if command -v tpm2_startup >/dev/null 2>&1; then
        tpm2_startup -c || log "TPM already initialized or startup failed"
        
        # Generate or load attestation key
        if [[ ! -f "$ATLASMESH_HOME/certs/tpm-ak.pub" ]]; then
            log "Generating TPM attestation key..."
            tpm2_createak -C 0x81010001 -G rsa -g sha256 -s rsassa \
                -u "$ATLASMESH_HOME/certs/tpm-ak.pub" \
                -r "$ATLASMESH_HOME/certs/tpm-ak.priv" || \
                log "WARNING: Failed to create TPM attestation key"
        fi
    fi

    log "TPM initialization completed"
}

# Validate secure boot
validate_secure_boot() {
    log "Validating secure boot status..."

    # Check if secure boot is enabled
    if command -v mokutil >/dev/null 2>&1; then
        if mokutil --sb-state | grep -q "SecureBoot enabled"; then
            log "Secure boot is enabled"
            
            # Validate boot chain integrity
            if [[ -f /sys/kernel/security/tpm0/binary_bios_measurements ]]; then
                log "TPM PCR measurements available for boot validation"
            fi
        else
            log "WARNING: Secure boot is not enabled"
        fi
    else
        log "WARNING: mokutil not available, cannot verify secure boot status"
    fi

    # Check kernel lockdown mode
    if [[ -f /sys/kernel/security/lockdown ]]; then
        local lockdown_mode
        lockdown_mode=$(cat /sys/kernel/security/lockdown)
        log "Kernel lockdown mode: $lockdown_mode"
    fi

    log "Secure boot validation completed"
}

# Configure networking
configure_networking() {
    log "Configuring networking..."

    # Configure firewall
    if command -v ufw >/dev/null 2>&1; then
        ufw --force reset
        ufw default deny incoming
        ufw default allow outgoing
        
        # Allow SSH (if enabled)
        if [[ "${ENABLE_SSH:-false}" == "true" ]]; then
            ufw allow 22/tcp
        fi
        
        # Allow AtlasMesh services
        ufw allow 8080/tcp  # Edge agent API
        ufw allow 9090/tcp  # Metrics
        ufw allow 8443/tcp  # Secure edge API
        
        # Allow vehicle communication ports
        ufw allow 11311/tcp # ROS master
        ufw allow 1883/tcp  # MQTT
        
        ufw --force enable
        log "Firewall configured and enabled"
    fi

    # Configure time synchronization
    if systemctl is-enabled --quiet chrony; then
        systemctl start chrony
        log "Time synchronization started"
    fi

    # Configure DNS
    if [[ -f /etc/systemd/resolved.conf ]]; then
        systemctl restart systemd-resolved
        log "DNS resolver restarted"
    fi

    log "Networking configuration completed"
}

# Initialize container runtime
initialize_containerd() {
    log "Initializing containerd..."

    # Start containerd service
    systemctl start containerd
    systemctl enable containerd

    # Wait for containerd to be ready
    local retries=0
    while ! ctr version >/dev/null 2>&1 && [[ $retries -lt 30 ]]; do
        log "Waiting for containerd to be ready... (attempt $((retries + 1)))"
        sleep 2
        ((retries++))
    done

    if [[ $retries -eq 30 ]]; then
        error_exit "Containerd failed to start within timeout"
    fi

    # Pull essential images
    log "Pulling essential container images..."
    
    # Pull pause image
    ctr images pull registry.k8s.io/pause:3.9 || log "WARNING: Failed to pull pause image"
    
    # Pull AtlasMesh base images (if registry is accessible)
    if ping -c 1 registry.atlasmesh.com >/dev/null 2>&1; then
        ctr images pull registry.atlasmesh.com/atlasmesh/vehicle-agent:latest || \
            log "WARNING: Failed to pull vehicle agent image"
    fi

    log "Containerd initialization completed"
}

# Configure security
configure_security() {
    log "Configuring security..."

    # Apply sysctl security settings
    sysctl -p /etc/sysctl.d/99-atlasmesh-security.conf

    # Start audit daemon
    if systemctl is-enabled --quiet auditd; then
        systemctl start auditd
        log "Audit daemon started"
    fi

    # Start fail2ban
    if systemctl is-enabled --quiet fail2ban; then
        systemctl start fail2ban
        log "Fail2ban started"
    fi

    # Configure AppArmor profiles (if available)
    if command -v aa-status >/dev/null 2>&1; then
        aa-status || log "AppArmor not active"
    fi

    # Set up certificate directories with proper permissions
    chmod 755 /etc/ssl/atlasmesh
    chmod 755 /etc/ssl/atlasmesh/ca
    chmod 755 /etc/ssl/atlasmesh/certs
    chmod 700 /etc/ssl/atlasmesh/private

    log "Security configuration completed"
}

# Initialize monitoring
initialize_monitoring() {
    log "Initializing monitoring..."

    # Configure rsyslog
    if systemctl is-enabled --quiet rsyslog; then
        systemctl restart rsyslog
        log "Rsyslog restarted with AtlasMesh configuration"
    fi

    # Set up log rotation
    if command -v logrotate >/dev/null 2>&1; then
        logrotate -f /etc/logrotate.d/atlasmesh || log "WARNING: Log rotation test failed"
    fi

    # Create monitoring directories
    mkdir -p /var/lib/atlasmesh/metrics
    chown atlasmesh:atlasmesh /var/lib/atlasmesh/metrics

    log "Monitoring initialization completed"
}

# Start AtlasMesh services
start_atlasmesh_services() {
    log "Starting AtlasMesh services..."

    # Start TPM manager
    if systemctl is-enabled --quiet atlasmesh-tpm-manager; then
        systemctl start atlasmesh-tpm-manager
        log "TPM manager started"
    fi

    # Start secure boot validator
    if systemctl is-enabled --quiet atlasmesh-secure-boot-validator; then
        systemctl start atlasmesh-secure-boot-validator
        log "Secure boot validator started"
    fi

    # Start edge agent
    if systemctl is-enabled --quiet atlasmesh-edge-agent; then
        systemctl start atlasmesh-edge-agent
        log "Edge agent started"
    fi

    # Wait for services to be ready
    sleep 5

    log "AtlasMesh services startup completed"
}

# Perform health check
perform_health_check() {
    log "Performing health check..."

    local health_status=0

    # Check system resources
    local cpu_usage
    cpu_usage=$(top -bn1 | grep "Cpu(s)" | awk '{print $2}' | sed 's/%us,//')
    log "CPU usage: ${cpu_usage}%"

    local memory_usage
    memory_usage=$(free | grep Mem | awk '{printf("%.1f", $3/$2 * 100.0)}')
    log "Memory usage: ${memory_usage}%"

    local disk_usage
    disk_usage=$(df / | awk 'NR==2 {printf("%.1f", $3/$2 * 100.0)}')
    log "Disk usage: ${disk_usage}%"

    # Check critical services
    local services=("containerd" "atlasmesh-edge-agent")
    for service in "${services[@]}"; do
        if systemctl is-active --quiet "$service"; then
            log "Service $service is running"
        else
            log "ERROR: Service $service is not running"
            health_status=1
        fi
    done

    # Check network connectivity
    if ping -c 1 8.8.8.8 >/dev/null 2>&1; then
        log "Network connectivity: OK"
    else
        log "WARNING: No external network connectivity"
    fi

    # Check TPM status
    if [[ -e /dev/tpm0 ]] || [[ -e /dev/tpmrm0 ]]; then
        if systemctl is-active --quiet tpm2-abrmd; then
            log "TPM status: OK"
        else
            log "WARNING: TPM resource manager not running"
        fi
    fi

    if [[ $health_status -eq 0 ]]; then
        log "Health check passed"
    else
        log "Health check failed with errors"
    fi

    return $health_status
}

# Wait for shutdown signal
wait_for_shutdown() {
    log "AtlasMesh Edge OS is ready and running"
    log "Vehicle ID: $VEHICLE_ID"
    log "Sector: $SECTOR"
    log "Services status:"
    systemctl --no-pager status atlasmesh-edge-agent || true
    
    # Create ready file
    touch /var/lib/atlasmesh/ready
    
    # Wait for signals
    while true; do
        sleep 30
        
        # Periodic health check
        if ! /usr/local/bin/health-check.sh >/dev/null 2>&1; then
            log "WARNING: Health check failed"
        fi
    done
}

# Execute main function
main "$@"
