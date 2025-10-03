#!/bin/bash
# AtlasMesh Kafka Topics Initialization Script
# Creates all required Kafka topics for telemetry ingestion and analytics

set -e

# Configuration
KAFKA_BROKER=${KAFKA_BROKER:-"localhost:29092"}
REPLICATION_FACTOR=${REPLICATION_FACTOR:-1}
TIMEOUT=${TIMEOUT:-30}

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

# Wait for Kafka to be ready
wait_for_kafka() {
    log_info "Waiting for Kafka to be ready at $KAFKA_BROKER..."
    
    local retries=0
    local max_retries=30
    
    while [ $retries -lt $max_retries ]; do
        if kafka-broker-api-versions --bootstrap-server "$KAFKA_BROKER" >/dev/null 2>&1; then
            log_success "Kafka is ready!"
            return 0
        fi
        
        retries=$((retries + 1))
        log_info "Attempt $retries/$max_retries - Kafka not ready yet, waiting 5 seconds..."
        sleep 5
    done
    
    log_error "Kafka failed to become ready after $max_retries attempts"
    return 1
}

# Create a single topic
create_topic() {
    local topic_name=$1
    local partitions=$2
    local retention_ms=$3
    local segment_ms=$4
    local compression_type=$5
    local max_message_bytes=$6
    local description=$7
    
    log_info "Creating topic: $topic_name"
    
    # Check if topic already exists
    if kafka-topics --bootstrap-server "$KAFKA_BROKER" --list | grep -q "^${topic_name}$"; then
        log_warning "Topic $topic_name already exists, skipping creation"
        return 0
    fi
    
    # Create the topic
    kafka-topics --bootstrap-server "$KAFKA_BROKER" \
        --create \
        --topic "$topic_name" \
        --partitions "$partitions" \
        --replication-factor "$REPLICATION_FACTOR" \
        --config retention.ms="$retention_ms" \
        --config segment.ms="$segment_ms" \
        --config compression.type="$compression_type" \
        --config max.message.bytes="$max_message_bytes" \
        --config min.insync.replicas=1
    
    if [ $? -eq 0 ]; then
        log_success "Created topic: $topic_name ($description)"
    else
        log_error "Failed to create topic: $topic_name"
        return 1
    fi
}

# Create all topics
create_all_topics() {
    log_info "Creating AtlasMesh Kafka topics..."
    
    # Vehicle Telemetry Topics
    create_topic "vehicle-telemetry" 12 604800000 86400000 "snappy" 1048576 "High-frequency vehicle telemetry data"
    create_topic "vehicle-telemetry-aggregated" 6 2592000000 86400000 "lz4" 2097152 "Aggregated vehicle telemetry for analytics"
    
    # Vehicle Command Topics
    create_topic "vehicle-commands" 8 2592000000 86400000 "snappy" 524288 "Commands sent to vehicles"
    create_topic "vehicle-command-responses" 8 2592000000 86400000 "snappy" 524288 "Command execution responses from vehicles"
    
    # Alert Topics
    create_topic "vehicle-alerts" 4 7776000000 86400000 "snappy" 262144 "Vehicle alerts and notifications"
    create_topic "critical-alerts" 2 31536000000 86400000 "snappy" 262144 "Critical alerts requiring immediate attention"
    
    # Trip and Route Topics
    create_topic "trip-events" 6 7776000000 86400000 "snappy" 524288 "Trip lifecycle events"
    create_topic "route-updates" 4 2592000000 86400000 "snappy" 1048576 "Dynamic route updates and optimizations"
    
    # System and Infrastructure Topics
    create_topic "system-metrics" 4 2592000000 86400000 "lz4" 262144 "System performance and health metrics"
    create_topic "audit-logs" 3 31536000000 86400000 "gzip" 524288 "Audit trail for compliance and security"
    
    # Dead Letter Queue Topics
    create_topic "dead-letter-queue" 2 7776000000 86400000 "gzip" 2097152 "Failed messages for investigation and replay"
    create_topic "schema-validation-errors" 1 2592000000 86400000 "gzip" 1048576 "Schema validation failures"
    
    # Analytics and ML Topics
    create_topic "ml-features" 4 7776000000 86400000 "lz4" 1048576 "Feature vectors for machine learning"
    create_topic "predictions" 2 2592000000 86400000 "snappy" 262144 "ML model predictions and recommendations"
    
    # Real-time Streaming Topics
    create_topic "realtime-telemetry" 8 3600000 300000 "snappy" 524288 "Real-time telemetry for dashboard streaming"
    create_topic "realtime-alerts" 4 7200000 600000 "snappy" 262144 "Real-time alerts for immediate notification"
    
    log_success "All topics created successfully!"
}

# List all topics
list_topics() {
    log_info "Listing all Kafka topics:"
    kafka-topics --bootstrap-server "$KAFKA_BROKER" --list | sort
}

# Show topic details
show_topic_details() {
    local topic_name=$1
    
    if [ -z "$topic_name" ]; then
        log_error "Topic name is required"
        return 1
    fi
    
    log_info "Topic details for: $topic_name"
    kafka-topics --bootstrap-server "$KAFKA_BROKER" --describe --topic "$topic_name"
}

# Delete a topic (for development/testing)
delete_topic() {
    local topic_name=$1
    
    if [ -z "$topic_name" ]; then
        log_error "Topic name is required"
        return 1
    fi
    
    log_warning "Deleting topic: $topic_name"
    read -p "Are you sure? This action cannot be undone. (y/N): " -n 1 -r
    echo
    
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        kafka-topics --bootstrap-server "$KAFKA_BROKER" --delete --topic "$topic_name"
        log_success "Topic $topic_name deleted"
    else
        log_info "Topic deletion cancelled"
    fi
}

# Create consumer groups
create_consumer_groups() {
    log_info "Consumer groups will be created automatically when consumers connect"
    log_info "Planned consumer groups:"
    echo "  - telemetry-ingestion-service"
    echo "  - analytics-processor"
    echo "  - command-processor"
    echo "  - alert-processor"
    echo "  - realtime-dashboard"
    echo "  - audit-processor"
}

# Main function
main() {
    case "${1:-create}" in
        "create")
            wait_for_kafka
            create_all_topics
            create_consumer_groups
            list_topics
            ;;
        "list")
            wait_for_kafka
            list_topics
            ;;
        "describe")
            wait_for_kafka
            show_topic_details "$2"
            ;;
        "delete")
            wait_for_kafka
            delete_topic "$2"
            ;;
        "help"|"-h"|"--help")
            echo "Usage: $0 [command] [options]"
            echo ""
            echo "Commands:"
            echo "  create          Create all AtlasMesh Kafka topics (default)"
            echo "  list            List all existing topics"
            echo "  describe <topic> Show details for a specific topic"
            echo "  delete <topic>   Delete a specific topic (interactive)"
            echo "  help            Show this help message"
            echo ""
            echo "Environment Variables:"
            echo "  KAFKA_BROKER         Kafka broker address (default: localhost:29092)"
            echo "  REPLICATION_FACTOR   Topic replication factor (default: 1)"
            echo "  TIMEOUT              Connection timeout in seconds (default: 30)"
            echo ""
            echo "Examples:"
            echo "  $0 create                           # Create all topics"
            echo "  $0 list                             # List all topics"
            echo "  $0 describe vehicle-telemetry       # Show topic details"
            echo "  KAFKA_BROKER=kafka:9092 $0 create   # Use different broker"
            ;;
        *)
            log_error "Unknown command: $1"
            echo "Use '$0 help' for usage information"
            exit 1
            ;;
    esac
}

# Handle script interruption
trap 'log_warning "Script interrupted"; exit 130' INT TERM

# Run main function
main "$@"
