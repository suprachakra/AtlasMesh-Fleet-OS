#!/bin/sh
# AtlasMesh Control Center Health Check Script
# Performs comprehensive health checks for the web application

set -e

# Configuration
HEALTH_URL="http://localhost:3000/health"
TIMEOUT=10
MAX_RETRIES=3
RETRY_DELAY=2

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Logging function
log() {
    echo "$(date '+%Y-%m-%d %H:%M:%S') [HEALTH] $1"
}

# Error function
error() {
    echo "${RED}$(date '+%Y-%m-%d %H:%M:%S') [ERROR] $1${NC}" >&2
}

# Success function
success() {
    echo "${GREEN}$(date '+%Y-%m-%d %H:%M:%S') [SUCCESS] $1${NC}"
}

# Warning function
warning() {
    echo "${YELLOW}$(date '+%Y-%m-%d %H:%M:%S') [WARNING] $1${NC}"
}

# Check if nginx is running
check_nginx_process() {
    log "Checking nginx process..."
    
    if pgrep nginx > /dev/null 2>&1; then
        success "Nginx process is running"
        return 0
    else
        error "Nginx process is not running"
        return 1
    fi
}

# Check if port 3000 is listening
check_port() {
    log "Checking if port 3000 is listening..."
    
    if netstat -ln 2>/dev/null | grep -q ":3000 "; then
        success "Port 3000 is listening"
        return 0
    else
        error "Port 3000 is not listening"
        return 1
    fi
}

# Check HTTP health endpoint
check_http_health() {
    log "Checking HTTP health endpoint..."
    
    local retry_count=0
    
    while [ $retry_count -lt $MAX_RETRIES ]; do
        if wget --quiet --timeout=$TIMEOUT --tries=1 --spider "$HEALTH_URL" 2>/dev/null; then
            success "HTTP health endpoint is responding"
            return 0
        else
            retry_count=$((retry_count + 1))
            if [ $retry_count -lt $MAX_RETRIES ]; then
                warning "HTTP health check failed, retrying in ${RETRY_DELAY}s (attempt $retry_count/$MAX_RETRIES)"
                sleep $RETRY_DELAY
            fi
        fi
    done
    
    error "HTTP health endpoint is not responding after $MAX_RETRIES attempts"
    return 1
}

# Check if main application files exist
check_application_files() {
    log "Checking application files..."
    
    local required_files="/usr/share/nginx/html/index.html"
    local missing_files=""
    
    for file in $required_files; do
        if [ ! -f "$file" ]; then
            missing_files="$missing_files $file"
        fi
    done
    
    if [ -n "$missing_files" ]; then
        error "Missing required files:$missing_files"
        return 1
    else
        success "All required application files are present"
        return 0
    fi
}

# Check disk space
check_disk_space() {
    log "Checking disk space..."
    
    local usage=$(df /usr/share/nginx/html | awk 'NR==2 {print $5}' | sed 's/%//')
    local threshold=90
    
    if [ "$usage" -gt "$threshold" ]; then
        error "Disk usage is ${usage}%, exceeding threshold of ${threshold}%"
        return 1
    else
        success "Disk usage is ${usage}%, within acceptable limits"
        return 0
    fi
}

# Check memory usage
check_memory() {
    log "Checking memory usage..."
    
    local mem_info=$(cat /proc/meminfo)
    local mem_total=$(echo "$mem_info" | grep MemTotal | awk '{print $2}')
    local mem_available=$(echo "$mem_info" | grep MemAvailable | awk '{print $2}')
    
    if [ "$mem_total" -gt 0 ] && [ "$mem_available" -gt 0 ]; then
        local mem_usage=$((100 - (mem_available * 100 / mem_total)))
        local threshold=90
        
        if [ "$mem_usage" -gt "$threshold" ]; then
            warning "Memory usage is ${mem_usage}%, approaching threshold of ${threshold}%"
        else
            success "Memory usage is ${mem_usage}%, within acceptable limits"
        fi
        return 0
    else
        warning "Could not determine memory usage"
        return 0
    fi
}

# Check nginx configuration
check_nginx_config() {
    log "Checking nginx configuration..."
    
    if nginx -t 2>/dev/null; then
        success "Nginx configuration is valid"
        return 0
    else
        error "Nginx configuration is invalid"
        return 1
    fi
}

# Check if static assets are accessible
check_static_assets() {
    log "Checking static assets..."
    
    local test_url="http://localhost:3000/index.html"
    
    if wget --quiet --timeout=$TIMEOUT --tries=1 --spider "$test_url" 2>/dev/null; then
        success "Static assets are accessible"
        return 0
    else
        error "Static assets are not accessible"
        return 1
    fi
}

# Main health check function
main() {
    log "Starting health check for AtlasMesh Control Center..."
    
    local checks_passed=0
    local total_checks=7
    
    # Run all health checks
    check_nginx_process && checks_passed=$((checks_passed + 1))
    check_port && checks_passed=$((checks_passed + 1))
    check_http_health && checks_passed=$((checks_passed + 1))
    check_application_files && checks_passed=$((checks_passed + 1))
    check_disk_space && checks_passed=$((checks_passed + 1))
    check_memory && checks_passed=$((checks_passed + 1))
    check_nginx_config && checks_passed=$((checks_passed + 1))
    
    # Optional check (doesn't affect overall health)
    check_static_assets || true
    
    # Determine overall health status
    if [ $checks_passed -eq $total_checks ]; then
        success "Health check passed ($checks_passed/$total_checks checks)"
        exit 0
    else
        error "Health check failed ($checks_passed/$total_checks checks passed)"
        exit 1
    fi
}

# Handle signals
trap 'error "Health check interrupted"; exit 1' INT TERM

# Run main function
main "$@"
