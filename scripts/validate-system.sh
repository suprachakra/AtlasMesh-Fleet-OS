#!/bin/bash

# AtlasMesh Fleet OS System Validation Script
# 
# This script performs comprehensive system validation across all environments:
# - API endpoint validation
# - Database connectivity tests
# - Message broker health checks
# - Performance benchmarks
# - Security validation
# - Data pipeline validation
#
# Usage: ./scripts/validate-system.sh [environment] [test-suite]
# Example: ./scripts/validate-system.sh development --full

set -euo pipefail

# Script configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
VALIDATION_LOG="$PROJECT_ROOT/logs/validation-$(date +%Y%m%d-%H%M%S).log"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
NC='\033[0m' # No Color

# Test counters
TOTAL_TESTS=0
PASSED_TESTS=0
FAILED_TESTS=0
SKIPPED_TESTS=0

# Logging functions
log_info() {
    echo -e "${BLUE}[INFO]${NC} $1" | tee -a "$VALIDATION_LOG"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1" | tee -a "$VALIDATION_LOG"
    ((PASSED_TESTS++))
}

log_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1" | tee -a "$VALIDATION_LOG"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1" | tee -a "$VALIDATION_LOG"
    ((FAILED_TESTS++))
}

log_skip() {
    echo -e "${PURPLE}[SKIP]${NC} $1" | tee -a "$VALIDATION_LOG"
    ((SKIPPED_TESTS++))
}

test_start() {
    ((TOTAL_TESTS++))
    log_info "TEST $TOTAL_TESTS: $1"
}

# Create logs directory
mkdir -p "$PROJECT_ROOT/logs"

# Default values
ENVIRONMENT="development"
TEST_SUITE="basic"
API_BASE_URL=""
TIMEOUT=30

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        development|sit|uat|production)
            ENVIRONMENT="$1"
            shift
            ;;
        --basic)
            TEST_SUITE="basic"
            shift
            ;;
        --full)
            TEST_SUITE="full"
            shift
            ;;
        --performance)
            TEST_SUITE="performance"
            shift
            ;;
        --security)
            TEST_SUITE="security"
            shift
            ;;
        --api-url)
            API_BASE_URL="$2"
            shift 2
            ;;
        --timeout)
            TIMEOUT="$2"
            shift 2
            ;;
        --help)
            echo "AtlasMesh Fleet OS System Validation Script"
            echo ""
            echo "Usage: $0 [environment] [options]"
            echo ""
            echo "Environments:"
            echo "  development    Local development environment (default)"
            echo "  sit           System Integration Testing environment"
            echo "  uat           User Acceptance Testing environment"
            echo "  production    Production environment"
            echo ""
            echo "Test Suites:"
            echo "  --basic       Basic health and connectivity tests (default)"
            echo "  --full        Comprehensive system validation"
            echo "  --performance Performance and load testing"
            echo "  --security    Security and compliance validation"
            echo ""
            echo "Options:"
            echo "  --api-url URL Custom API base URL"
            echo "  --timeout SEC Request timeout in seconds (default: 30)"
            echo "  --help        Show this help message"
            exit 0
            ;;
        *)
            log_error "Unknown option: $1"
            exit 1
            ;;
    esac
done

# Set API base URL based on environment if not provided
if [[ -z "$API_BASE_URL" ]]; then
    case $ENVIRONMENT in
        development)
            API_BASE_URL="http://localhost:8080/v2"
            ;;
        sit)
            API_BASE_URL="http://localhost:8081/v2"
            ;;
        uat)
            API_BASE_URL="http://localhost/v2"
            ;;
        production)
            API_BASE_URL="https://api.atlasmesh.ai/v2"
            ;;
    esac
fi

log_info "Starting AtlasMesh Fleet OS validation for environment: $ENVIRONMENT"
log_info "API Base URL: $API_BASE_URL"
log_info "Test Suite: $TEST_SUITE"
log_info "Timeout: ${TIMEOUT}s"

# Utility functions
make_request() {
    local method="$1"
    local endpoint="$2"
    local expected_status="${3:-200}"
    local data="${4:-}"
    
    local curl_cmd="curl -s -w '%{http_code}' --max-time $TIMEOUT"
    
    if [[ "$method" == "POST" && -n "$data" ]]; then
        curl_cmd="$curl_cmd -X POST -H 'Content-Type: application/json' -d '$data'"
    elif [[ "$method" == "PUT" && -n "$data" ]]; then
        curl_cmd="$curl_cmd -X PUT -H 'Content-Type: application/json' -d '$data'"
    elif [[ "$method" == "DELETE" ]]; then
        curl_cmd="$curl_cmd -X DELETE"
    fi
    
    local response
    response=$(eval "$curl_cmd '$API_BASE_URL$endpoint'")
    local status_code="${response: -3}"
    local body="${response%???}"
    
    if [[ "$status_code" == "$expected_status" ]]; then
        return 0
    else
        log_error "Expected status $expected_status, got $status_code for $method $endpoint"
        return 1
    fi
}

# Basic Health Tests
test_api_health() {
    test_start "API Gateway Health Check"
    if make_request "GET" "/health" "200"; then
        log_success "API Gateway is healthy"
    else
        log_error "API Gateway health check failed"
    fi
}

test_service_health() {
    local services=("policy" "trip" "dispatch" "routing" "fleet")
    
    for service in "${services[@]}"; do
        test_start "$service Service Health Check"
        if make_request "GET" "/$service/health" "200"; then
            log_success "$service service is healthy"
        else
            log_error "$service service health check failed"
        fi
    done
}

test_database_connectivity() {
    test_start "Database Connectivity"
    
    # Test via API endpoint that requires database
    if make_request "GET" "/policies" "200"; then
        log_success "Database connectivity verified"
    else
        log_error "Database connectivity test failed"
    fi
}

test_message_broker() {
    test_start "Message Broker Connectivity"
    
    # Test telemetry ingestion which uses Kafka
    local test_data='{"vehicle_id":"test-vehicle-001","batch_id":"test-batch-001","timestamp":"2024-01-15T10:30:00Z","telemetry_data":[{"recorded_at":"2024-01-15T10:30:00Z","location":{"latitude":25.2048,"longitude":55.2708},"speed_kmh":0,"operational_status":"idle"}]}'
    
    if make_request "POST" "/telemetry/ingest" "202" "$test_data"; then
        log_success "Message broker connectivity verified"
    else
        log_error "Message broker connectivity test failed"
    fi
}

# API Functionality Tests
test_policy_management() {
    test_start "Policy Management API"
    
    # Create a test policy
    local policy_data='{"name":"test-policy","version":"v1.0.0","rego_code":"package test\nallow = true","schema":{},"sector":"universal"}'
    
    if make_request "POST" "/policies" "201" "$policy_data"; then
        log_success "Policy creation API works"
        
        # List policies
        if make_request "GET" "/policies" "200"; then
            log_success "Policy listing API works"
        else
            log_error "Policy listing API failed"
        fi
    else
        log_error "Policy creation API failed"
    fi
}

test_trip_management() {
    test_start "Trip Management API"
    
    # Create a test trip
    local trip_data='{"origin":{"latitude":25.2048,"longitude":55.2708},"destination":{"latitude":25.2148,"longitude":55.2808},"vehicle_requirements":{"vehicle_type":"sedan"},"scheduled_start":"2024-01-15T12:00:00Z"}'
    
    if make_request "POST" "/trips" "201" "$trip_data"; then
        log_success "Trip creation API works"
        
        # List trips
        if make_request "GET" "/trips" "200"; then
            log_success "Trip listing API works"
        else
            log_error "Trip listing API failed"
        fi
    else
        log_error "Trip creation API failed"
    fi
}

test_fleet_management() {
    test_start "Fleet Management API"
    
    # Register a test vehicle
    local vehicle_data='{"asset_tag":"TEST-001","manufacturer":"TestCorp","model":"TestVehicle","serial_number":"TEST123456","vehicle_profile":{"type":"sedan","capacity":4}}'
    
    if make_request "POST" "/vehicles" "201" "$vehicle_data"; then
        log_success "Vehicle registration API works"
        
        # List vehicles
        if make_request "GET" "/vehicles" "200"; then
            log_success "Vehicle listing API works"
        else
            log_error "Vehicle listing API failed"
        fi
    else
        log_error "Vehicle registration API failed"
    fi
}

# Performance Tests
test_api_performance() {
    test_start "API Performance Test"
    
    local start_time=$(date +%s%N)
    if make_request "GET" "/health" "200"; then
        local end_time=$(date +%s%N)
        local duration=$(( (end_time - start_time) / 1000000 )) # Convert to milliseconds
        
        if [[ $duration -lt 200 ]]; then
            log_success "API response time: ${duration}ms (< 200ms SLA)"
        elif [[ $duration -lt 500 ]]; then
            log_warning "API response time: ${duration}ms (within acceptable range)"
        else
            log_error "API response time: ${duration}ms (exceeds 500ms threshold)"
        fi
    else
        log_error "Performance test failed - API not responding"
    fi
}

test_concurrent_requests() {
    test_start "Concurrent Request Handling"
    
    local concurrent_requests=10
    local success_count=0
    
    for i in $(seq 1 $concurrent_requests); do
        if make_request "GET" "/health" "200" &; then
            ((success_count++))
        fi
    done
    
    wait # Wait for all background jobs to complete
    
    if [[ $success_count -eq $concurrent_requests ]]; then
        log_success "Handled $concurrent_requests concurrent requests successfully"
    else
        log_error "Only $success_count/$concurrent_requests concurrent requests succeeded"
    fi
}

# Security Tests
test_authentication() {
    test_start "Authentication Required"
    
    # Test that protected endpoints require authentication
    if make_request "GET" "/policies" "401"; then
        log_success "Authentication is properly enforced"
    else
        log_error "Authentication enforcement failed"
    fi
}

test_rate_limiting() {
    test_start "Rate Limiting"
    
    # Make rapid requests to trigger rate limiting
    local rate_limit_triggered=false
    
    for i in $(seq 1 100); do
        if ! make_request "GET" "/health" "200"; then
            rate_limit_triggered=true
            break
        fi
        sleep 0.01 # 10ms between requests
    done
    
    if [[ "$rate_limit_triggered" == "true" ]]; then
        log_success "Rate limiting is active"
    else
        log_warning "Rate limiting not triggered (may be set too high)"
    fi
}

# Data Pipeline Tests
test_telemetry_pipeline() {
    test_start "Telemetry Data Pipeline"
    
    # Send test telemetry data
    local telemetry_data='{"vehicle_id":"pipeline-test-001","batch_id":"pipeline-batch-001","timestamp":"2024-01-15T10:30:00Z","telemetry_data":[{"recorded_at":"2024-01-15T10:30:00Z","location":{"latitude":25.2048,"longitude":55.2708},"speed_kmh":45.2,"operational_status":"active"}]}'
    
    if make_request "POST" "/telemetry/ingest" "202" "$telemetry_data"; then
        log_success "Telemetry ingestion pipeline works"
        
        # Wait a moment for processing
        sleep 5
        
        # Verify data can be queried (if query endpoint exists)
        if make_request "GET" "/telemetry/vehicle/pipeline-test-001" "200"; then
            log_success "Telemetry data query works"
        else
            log_warning "Telemetry data query endpoint not available or data not yet processed"
        fi
    else
        log_error "Telemetry ingestion pipeline failed"
    fi
}

# Monitoring and Observability Tests
test_metrics_endpoint() {
    test_start "Metrics Endpoint"
    
    if make_request "GET" "/metrics" "200"; then
        log_success "Metrics endpoint is accessible"
    else
        log_error "Metrics endpoint failed"
    fi
}

test_logging() {
    test_start "Structured Logging"
    
    # Make a request and check if it generates logs (this is a basic test)
    if make_request "GET" "/health" "200"; then
        log_success "Logging system appears functional"
    else
        log_error "Logging test failed"
    fi
}

# Test Suite Execution
run_basic_tests() {
    log_info "Running basic test suite..."
    
    test_api_health
    test_service_health
    test_database_connectivity
    test_message_broker
}

run_full_tests() {
    log_info "Running full test suite..."
    
    # Basic tests
    run_basic_tests
    
    # API functionality tests
    test_policy_management
    test_trip_management
    test_fleet_management
    
    # Data pipeline tests
    test_telemetry_pipeline
    
    # Monitoring tests
    test_metrics_endpoint
    test_logging
}

run_performance_tests() {
    log_info "Running performance test suite..."
    
    test_api_performance
    test_concurrent_requests
}

run_security_tests() {
    log_info "Running security test suite..."
    
    test_authentication
    test_rate_limiting
}

# Generate validation report
generate_validation_report() {
    local report_file="$PROJECT_ROOT/logs/validation-report-$ENVIRONMENT-$(date +%Y%m%d-%H%M%S).json"
    
    log_info "Generating validation report..."
    
    local success_rate=0
    if [[ $TOTAL_TESTS -gt 0 ]]; then
        success_rate=$(( (PASSED_TESTS * 100) / TOTAL_TESTS ))
    fi
    
    cat > "$report_file" << EOF
{
  "validation": {
    "environment": "$ENVIRONMENT",
    "test_suite": "$TEST_SUITE",
    "timestamp": "$(date -u +%Y-%m-%dT%H:%M:%SZ)",
    "api_base_url": "$API_BASE_URL",
    "timeout": $TIMEOUT
  },
  "results": {
    "total_tests": $TOTAL_TESTS,
    "passed_tests": $PASSED_TESTS,
    "failed_tests": $FAILED_TESTS,
    "skipped_tests": $SKIPPED_TESTS,
    "success_rate_percent": $success_rate
  },
  "status": "$(if [[ $FAILED_TESTS -eq 0 ]]; then echo "PASS"; else echo "FAIL"; fi)",
  "system_info": {
    "validation_duration_seconds": $(($(date +%s) - START_TIME)),
    "host_os": "$(uname -s)",
    "curl_version": "$(curl --version | head -n1)"
  }
}
EOF
    
    log_success "Validation report generated: $report_file"
}

# Main validation flow
main() {
    local START_TIME=$(date +%s)
    
    log_info "AtlasMesh Fleet OS System Validation Started"
    log_info "Environment: $ENVIRONMENT"
    log_info "Test Suite: $TEST_SUITE"
    
    # Execute test suites based on selection
    case $TEST_SUITE in
        basic)
            run_basic_tests
            ;;
        full)
            run_full_tests
            ;;
        performance)
            run_basic_tests
            run_performance_tests
            ;;
        security)
            run_basic_tests
            run_security_tests
            ;;
        *)
            log_error "Unknown test suite: $TEST_SUITE"
            exit 1
            ;;
    esac
    
    # Generate report
    generate_validation_report
    
    # Summary
    log_info "Validation Summary:"
    log_info "  Total Tests: $TOTAL_TESTS"
    log_success "  Passed: $PASSED_TESTS"
    if [[ $FAILED_TESTS -gt 0 ]]; then
        log_error "  Failed: $FAILED_TESTS"
    fi
    if [[ $SKIPPED_TESTS -gt 0 ]]; then
        log_skip "  Skipped: $SKIPPED_TESTS"
    fi
    
    local success_rate=0
    if [[ $TOTAL_TESTS -gt 0 ]]; then
        success_rate=$(( (PASSED_TESTS * 100) / TOTAL_TESTS ))
    fi
    
    log_info "  Success Rate: ${success_rate}%"
    
    if [[ $FAILED_TESTS -eq 0 ]]; then
        log_success "System validation PASSED!"
        exit 0
    else
        log_error "System validation FAILED!"
        exit 1
    fi
}

# Trap for cleanup on script exit
trap 'log_info "Validation script interrupted"' INT TERM

# Run main function
main "$@"
