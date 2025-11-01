#!/bin/bash
# AtlasMesh Fleet OS - PostgreSQL Backup Script
# Automated full and incremental backups with S3 upload

set -euo pipefail

# Configuration
POSTGRES_HOST=${POSTGRES_HOST:-postgres}
POSTGRES_PORT=${POSTGRES_PORT:-5432}
POSTGRES_DB=${POSTGRES_DB:-atlasmesh_fleet_os}
POSTGRES_USER=${POSTGRES_USER:-postgres}
BACKUP_DIR="/backups/postgres"
S3_BUCKET=${S3_BUCKET:-atlasmesh-backups}
RETENTION_DAYS=${BACKUP_RETENTION_DAYS:-30}
BACKUP_TYPE=${1:-full}

# Create backup directory
mkdir -p "$BACKUP_DIR"

# Generate timestamp
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
DATE=$(date +"%Y%m%d")

# Logging function
log() {
    echo "[$(date +'%Y-%m-%d %H:%M:%S')] $1" | tee -a "$BACKUP_DIR/backup.log"
}

# Error handling
error_exit() {
    log "ERROR: $1"
    exit 1
}

# Check dependencies
check_dependencies() {
    command -v pg_dump >/dev/null 2>&1 || error_exit "pg_dump not found"
    command -v aws >/dev/null 2>&1 || error_exit "AWS CLI not found"
    command -v gzip >/dev/null 2>&1 || error_exit "gzip not found"
}

# Test database connection
test_connection() {
    log "Testing database connection..."
    PGPASSWORD="$PGPASSWORD" pg_isready -h "$POSTGRES_HOST" -p "$POSTGRES_PORT" -U "$POSTGRES_USER" -d "$POSTGRES_DB" || error_exit "Cannot connect to database"
    log "Database connection successful"
}

# Perform full backup
full_backup() {
    log "Starting full backup..."
    
    local backup_file="$BACKUP_DIR/full_backup_${TIMESTAMP}.sql"
    local compressed_file="${backup_file}.gz"
    
    # Create full backup
    PGPASSWORD="$PGPASSWORD" pg_dump \
        -h "$POSTGRES_HOST" \
        -p "$POSTGRES_PORT" \
        -U "$POSTGRES_USER" \
        -d "$POSTGRES_DB" \
        --verbose \
        --no-password \
        --format=custom \
        --compress=9 \
        --file="$backup_file" || error_exit "Full backup failed"
    
    # Compress backup
    gzip "$backup_file" || error_exit "Compression failed"
    
    # Calculate checksum
    local checksum=$(sha256sum "$compressed_file" | cut -d' ' -f1)
    echo "$checksum" > "${compressed_file}.sha256"
    
    # Get backup size
    local size=$(du -h "$compressed_file" | cut -f1)
    
    log "Full backup completed: $compressed_file (Size: $size, Checksum: $checksum)"
    
    # Upload to S3
    upload_to_s3 "$compressed_file" "postgres/full/"
    upload_to_s3 "${compressed_file}.sha256" "postgres/full/"
    
    # Create backup metadata
    create_backup_metadata "$compressed_file" "full" "$checksum" "$size"
}

# Perform incremental backup (WAL archiving)
incremental_backup() {
    log "Starting incremental backup (WAL archiving)..."
    
    local wal_dir="$BACKUP_DIR/wal/$DATE"
    mkdir -p "$wal_dir"
    
    # Get current WAL file
    local current_wal=$(PGPASSWORD="$PGPASSWORD" psql -h "$POSTGRES_HOST" -p "$POSTGRES_PORT" -U "$POSTGRES_USER" -d "$POSTGRES_DB" -t -c "SELECT pg_walfile_name(pg_current_wal_lsn());" | tr -d ' ')
    
    if [ -n "$current_wal" ]; then
        log "Current WAL file: $current_wal"
        
        # Force WAL switch to archive current segment
        PGPASSWORD="$PGPASSWORD" psql -h "$POSTGRES_HOST" -p "$POSTGRES_PORT" -U "$POSTGRES_USER" -d "$POSTGRES_DB" -c "SELECT pg_switch_wal();" >/dev/null
        
        # Wait a moment for archiving
        sleep 5
        
        # Archive WAL files to S3
        if [ -d "/var/lib/postgresql/data/pg_wal" ]; then
            find /var/lib/postgresql/data/pg_wal -name "*.ready" -type f | while read -r ready_file; do
                wal_file=$(basename "$ready_file" .ready)
                if [ -f "/var/lib/postgresql/data/pg_wal/$wal_file" ]; then
                    cp "/var/lib/postgresql/data/pg_wal/$wal_file" "$wal_dir/"
                    gzip "$wal_dir/$wal_file"
                    upload_to_s3 "$wal_dir/${wal_file}.gz" "postgres/wal/$DATE/"
                    log "Archived WAL file: $wal_file"
                fi
            done
        fi
    fi
    
    log "Incremental backup completed"
}

# Upload file to S3
upload_to_s3() {
    local file_path="$1"
    local s3_prefix="$2"
    local s3_key="${s3_prefix}$(basename "$file_path")"
    
    log "Uploading to S3: s3://$S3_BUCKET/$s3_key"
    
    aws s3 cp "$file_path" "s3://$S3_BUCKET/$s3_key" \
        --storage-class STANDARD_IA \
        --metadata "backup-date=$DATE,backup-type=$BACKUP_TYPE,database=$POSTGRES_DB" || error_exit "S3 upload failed"
    
    log "Upload completed: s3://$S3_BUCKET/$s3_key"
}

# Create backup metadata
create_backup_metadata() {
    local backup_file="$1"
    local backup_type="$2"
    local checksum="$3"
    local size="$4"
    
    local metadata_file="$BACKUP_DIR/metadata_${TIMESTAMP}.json"
    
    cat > "$metadata_file" << EOF
{
    "backup_id": "postgres_${backup_type}_${TIMESTAMP}",
    "backup_type": "$backup_type",
    "database": "$POSTGRES_DB",
    "timestamp": "$TIMESTAMP",
    "date": "$DATE",
    "file_path": "$backup_file",
    "file_size": "$size",
    "checksum": "$checksum",
    "postgres_version": "$(PGPASSWORD="$PGPASSWORD" psql -h "$POSTGRES_HOST" -p "$POSTGRES_PORT" -U "$POSTGRES_USER" -d "$POSTGRES_DB" -t -c "SELECT version();" | head -1 | tr -d ' ')",
    "backup_start": "$(date -Iseconds)",
    "retention_days": $RETENTION_DAYS,
    "s3_bucket": "$S3_BUCKET",
    "s3_prefix": "postgres/$backup_type/"
}
EOF
    
    upload_to_s3 "$metadata_file" "postgres/metadata/"
    log "Backup metadata created: $metadata_file"
}

# Clean up old backups
cleanup_old_backups() {
    log "Cleaning up backups older than $RETENTION_DAYS days..."
    
    # Local cleanup
    find "$BACKUP_DIR" -type f -mtime +$RETENTION_DAYS -delete 2>/dev/null || true
    
    # S3 cleanup (using lifecycle policies is recommended for production)
    local cutoff_date=$(date -d "$RETENTION_DAYS days ago" +%Y%m%d)
    
    aws s3 ls "s3://$S3_BUCKET/postgres/" --recursive | while read -r line; do
        local file_date=$(echo "$line" | awk '{print $1}' | tr -d '-')
        local file_path=$(echo "$line" | awk '{print $4}')
        
        if [[ "$file_date" < "$cutoff_date" ]]; then
            aws s3 rm "s3://$S3_BUCKET/$file_path" || log "Failed to delete s3://$S3_BUCKET/$file_path"
        fi
    done
    
    log "Cleanup completed"
}

# Verify backup integrity
verify_backup() {
    local backup_file="$1"
    
    if [ -f "${backup_file}.sha256" ]; then
        log "Verifying backup integrity..."
        if sha256sum -c "${backup_file}.sha256"; then
            log "Backup integrity verified"
        else
            error_exit "Backup integrity check failed"
        fi
    fi
}

# Send notification
send_notification() {
    local status="$1"
    local message="$2"
    
    if [ -n "${SLACK_WEBHOOK_URL:-}" ]; then
        curl -X POST -H 'Content-type: application/json' \
            --data "{\"text\":\"🗄️ AtlasMesh Backup: $status - $message\"}" \
            "$SLACK_WEBHOOK_URL" 2>/dev/null || true
    fi
    
    if [ -n "${EMAIL_SMTP_HOST:-}" ] && [ -n "${ALERT_EMAIL:-}" ]; then
        # Email notification would be implemented here
        log "Email notification: $status - $message"
    fi
}

# Main execution
main() {
    log "Starting PostgreSQL backup process (Type: $BACKUP_TYPE)"
    
    check_dependencies
    test_connection
    
    case "$BACKUP_TYPE" in
        "full")
            full_backup
            ;;
        "incremental")
            incremental_backup
            ;;
        *)
            error_exit "Invalid backup type: $BACKUP_TYPE. Use 'full' or 'incremental'"
            ;;
    esac
    
    cleanup_old_backups
    
    log "Backup process completed successfully"
    send_notification "SUCCESS" "PostgreSQL $BACKUP_TYPE backup completed for $POSTGRES_DB"
}

# Error handling
trap 'error_exit "Backup process interrupted"' INT TERM
trap 'send_notification "FAILED" "PostgreSQL backup failed for $POSTGRES_DB"' ERR

# Run main function
main "$@"
