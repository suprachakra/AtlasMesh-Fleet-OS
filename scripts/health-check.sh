#!/bin/bash
# AtlasMesh Fleet OS - System Health Check Script

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Configuration
SERVICES=(
    "fleet-manager:8080"
    "policy-engine:8081"
    "telemetry-ingestion:8082"
    "weather-fusion:8083"
    "observability:8084"
    "security:8085"
)

DATABASES=(
    "postgres:5432"
    "redis:6379"
    "kafka:9092"
)

echo -e "${BLUE}AtlasMesh Fleet OS - System Health Check${NC}"
echo "========================================"

# Function to check HTTP service health
check_http_service() {
    local service_name=$1
    local service_url=$2
    
    echo -n "Checking $service_name... "
    
    if curl -s -f "$service_url/health" > /dev/null 2>&1; then
        echo -e "${GREEN}✓ Healthy${NC}"
        return 0
    else
        echo -e "${RED}✗ Unhealthy${NC}"
        return 1
    fi
}

# Function to check database connectivity
check_database() {
    local db_name=$1
    local db_host=$2
    local db_port=$3
    
    echo -n "Checking $db_name connectivity... "
    
    case $db_name in
        "postgres")
            if pg_isready -h "$db_host" -p "$db_port" > /dev/null 2>&1; then
                echo -e "${GREEN}✓ Connected${NC}"
                return 0
            fi
            ;;
        "redis")
            if redis-cli -h "$db_host" -p "$db_port" ping > /dev/null 2>&1; then
                echo -e "${GREEN}✓ Connected${NC}"
                return 0
            fi
            ;;
        "kafka")
            if nc -z "$db_host" "$db_port" > /dev/null 2>&1; then
                echo -e "${GREEN}✓ Connected${NC}"
                return 0
            fi
            ;;
    esac
    
    echo -e "${RED}✗ Disconnected${NC}"
    return 1
}

# Check services
echo -e "\n${YELLOW}Service Health Checks:${NC}"
service_failures=0

for service_config in "${SERVICES[@]}"; do
    IFS=':' read -r service_name service_port <<< "$service_config"
    service_url="http://localhost:$service_port"
    
    if ! check_http_service "$service_name" "$service_url"; then
        ((service_failures++))
    fi
done

# Check databases
echo -e "\n${YELLOW}Database Connectivity Checks:${NC}"
db_failures=0

for db_config in "${DATABASES[@]}"; do
    IFS=':' read -r db_name db_port <<< "$db_config"
    
    if ! check_database "$db_name" "localhost" "$db_port"; then
        ((db_failures++))
    fi
done

# Check system resources
echo -e "\n${YELLOW}System Resource Checks:${NC}"

# Check disk space
echo -n "Checking disk space... "
disk_usage=$(df / | awk 'NR==2 {print $5}' | sed 's/%//')
if [ "$disk_usage" -lt 80 ]; then
    echo -e "${GREEN}✓ OK (${disk_usage}% used)${NC}"
else
    echo -e "${YELLOW}⚠ Warning (${disk_usage}% used)${NC}"
fi

# Check memory usage
echo -n "Checking memory usage... "
memory_usage=$(free | awk 'NR==2{printf "%.0f", $3*100/$2}')
if [ "$memory_usage" -lt 80 ]; then
    echo -e "${GREEN}✓ OK (${memory_usage}% used)${NC}"
else
    echo -e "${YELLOW}⚠ Warning (${memory_usage}% used)${NC}"
fi

# Check CPU load
echo -n "Checking CPU load... "
cpu_load=$(uptime | awk -F'load average:' '{print $2}' | awk '{print $1}' | sed 's/,//')
cpu_cores=$(nproc)
cpu_threshold=$(echo "$cpu_cores * 0.8" | bc)

if (( $(echo "$cpu_load < $cpu_threshold" | bc -l) )); then
    echo -e "${GREEN}✓ OK (load: $cpu_load)${NC}"
else
    echo -e "${YELLOW}⚠ Warning (load: $cpu_load)${NC}"
fi

# Summary
echo -e "\n${YELLOW}Health Check Summary:${NC}"
total_failures=$((service_failures + db_failures))

if [ $total_failures -eq 0 ]; then
    echo -e "${GREEN}✓ All systems healthy${NC}"
    exit 0
else
    echo -e "${RED}✗ $total_failures issues detected${NC}"
    echo "  - Service failures: $service_failures"
    echo "  - Database failures: $db_failures"
    exit 1
fi
