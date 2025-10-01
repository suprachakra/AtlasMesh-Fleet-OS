-- AtlasMesh Telemetry Database Schema
-- ClickHouse tables for high-performance telemetry storage and analytics

-- Create database if not exists
CREATE DATABASE IF NOT EXISTS atlasmesh_telemetry;

-- Use the telemetry database
USE atlasmesh_telemetry;

-- Vehicle Telemetry Table (Hot Path - Real-time analytics)
CREATE TABLE IF NOT EXISTS vehicle_telemetry (
    -- Header information
    timestamp DateTime64(3) CODEC(Delta, ZSTD),
    vehicle_id String CODEC(ZSTD),
    sequence_number UInt64 CODEC(Delta, ZSTD),
    schema_version String CODEC(ZSTD),
    source String CODEC(ZSTD),
    
    -- Position data
    latitude Float64 CODEC(ZSTD),
    longitude Float64 CODEC(ZSTD),
    altitude Nullable(Float64) CODEC(ZSTD),
    heading Nullable(Float64) CODEC(ZSTD),
    position_accuracy Nullable(Float64) CODEC(ZSTD),
    gnss_fix_type Enum8('NO_FIX' = 0, 'GPS' = 1, 'DGPS' = 2, 'RTK_FLOAT' = 3, 'RTK_FIXED' = 4) CODEC(ZSTD),
    
    -- Motion data
    speed Float64 CODEC(ZSTD),
    acceleration_x Float64 CODEC(ZSTD),
    acceleration_y Float64 CODEC(ZSTD),
    acceleration_z Float64 CODEC(ZSTD),
    angular_velocity_roll Float64 CODEC(ZSTD),
    angular_velocity_pitch Float64 CODEC(ZSTD),
    angular_velocity_yaw Float64 CODEC(ZSTD),
    wheel_speed_fl Float64 CODEC(ZSTD),
    wheel_speed_fr Float64 CODEC(ZSTD),
    wheel_speed_rl Float64 CODEC(ZSTD),
    wheel_speed_rr Float64 CODEC(ZSTD),
    
    -- Vehicle state
    operational_mode Enum8('MANUAL' = 0, 'ASSISTED' = 1, 'AUTONOMOUS' = 2, 'EMERGENCY' = 3, 'MAINTENANCE' = 4) CODEC(ZSTD),
    system_status Enum8('IDLE' = 0, 'READY' = 1, 'ACTIVE' = 2, 'ERROR' = 3, 'MAINTENANCE' = 4) CODEC(ZSTD),
    emergency_stop_active Bool CODEC(ZSTD),
    autonomous_mode_active Bool CODEC(ZSTD),
    manual_override_active Bool CODEC(ZSTD),
    
    -- Powertrain data
    battery_voltage Nullable(Float64) CODEC(ZSTD),
    battery_current Nullable(Float64) CODEC(ZSTD),
    battery_soc Nullable(Float64) CODEC(ZSTD),
    battery_temperature Nullable(Float64) CODEC(ZSTD),
    battery_health Nullable(Float64) CODEC(ZSTD),
    engine_rpm Nullable(Float64) CODEC(ZSTD),
    engine_temperature Nullable(Float64) CODEC(ZSTD),
    fuel_level Nullable(Float64) CODEC(ZSTD),
    fuel_consumption_rate Nullable(Float64) CODEC(ZSTD),
    brake_pressure Float64 CODEC(ZSTD),
    throttle_position Float64 CODEC(ZSTD),
    steering_angle Float64 CODEC(ZSTD),
    
    -- Environmental data
    ambient_temperature Nullable(Float64) CODEC(ZSTD),
    humidity Nullable(Float64) CODEC(ZSTD),
    barometric_pressure Nullable(Float64) CODEC(ZSTD),
    wind_speed Nullable(Float64) CODEC(ZSTD),
    wind_direction Nullable(Float64) CODEC(ZSTD),
    visibility Nullable(Float64) CODEC(ZSTD),
    
    -- Performance metrics
    control_loop_frequency Float64 CODEC(ZSTD),
    sensor_fusion_latency Float64 CODEC(ZSTD),
    policy_evaluation_time Float64 CODEC(ZSTD),
    communication_latency Float64 CODEC(ZSTD),
    cpu_usage Float64 CODEC(ZSTD),
    memory_usage Float64 CODEC(ZSTD),
    disk_usage Float64 CODEC(ZSTD),
    
    -- Trip context
    trip_id Nullable(String) CODEC(ZSTD),
    route_id Nullable(String) CODEC(ZSTD),
    waypoint_index Nullable(Int32) CODEC(ZSTD),
    distance_to_waypoint Nullable(Float64) CODEC(ZSTD),
    estimated_arrival Nullable(DateTime64(3)) CODEC(ZSTD),
    
    -- Metadata
    sector String CODEC(ZSTD),
    vehicle_type String CODEC(ZSTD),
    firmware_version String CODEC(ZSTD),
    software_version String CODEC(ZSTD),
    profile_version String CODEC(ZSTD),
    
    -- Ingestion metadata
    ingestion_timestamp DateTime64(3) DEFAULT now64() CODEC(Delta, ZSTD),
    partition_date Date MATERIALIZED toDate(timestamp) CODEC(ZSTD)
)
ENGINE = MergeTree()
PARTITION BY partition_date
ORDER BY (vehicle_id, timestamp)
TTL timestamp + INTERVAL 7 DAY  -- Hot data retention: 7 days
SETTINGS index_granularity = 8192;

-- Sensor Health Table
CREATE TABLE IF NOT EXISTS sensor_health (
    timestamp DateTime64(3) CODEC(Delta, ZSTD),
    vehicle_id String CODEC(ZSTD),
    sensor_id String CODEC(ZSTD),
    sensor_type Enum8('LIDAR' = 0, 'CAMERA' = 1, 'RADAR' = 2, 'IMU' = 3, 'GNSS' = 4, 'OTHER' = 5) CODEC(ZSTD),
    status Enum8('OK' = 0, 'WARNING' = 1, 'ERROR' = 2, 'OFFLINE' = 3) CODEC(ZSTD),
    last_update DateTime64(3) CODEC(ZSTD),
    error_message Nullable(String) CODEC(ZSTD),
    
    -- Ingestion metadata
    ingestion_timestamp DateTime64(3) DEFAULT now64() CODEC(Delta, ZSTD),
    partition_date Date MATERIALIZED toDate(timestamp) CODEC(ZSTD)
)
ENGINE = MergeTree()
PARTITION BY partition_date
ORDER BY (vehicle_id, sensor_id, timestamp)
TTL timestamp + INTERVAL 30 DAY
SETTINGS index_granularity = 8192;

-- Vehicle Alerts Table
CREATE TABLE IF NOT EXISTS vehicle_alerts (
    timestamp DateTime64(3) CODEC(Delta, ZSTD),
    vehicle_id String CODEC(ZSTD),
    alert_id String CODEC(ZSTD),
    severity Enum8('INFO' = 0, 'WARNING' = 1, 'ERROR' = 2, 'CRITICAL' = 3) CODEC(ZSTD),
    category String CODEC(ZSTD),
    message String CODEC(ZSTD),
    acknowledged Bool DEFAULT false CODEC(ZSTD),
    acknowledged_by Nullable(String) CODEC(ZSTD),
    acknowledged_at Nullable(DateTime64(3)) CODEC(ZSTD),
    resolved Bool DEFAULT false CODEC(ZSTD),
    resolved_by Nullable(String) CODEC(ZSTD),
    resolved_at Nullable(DateTime64(3)) CODEC(ZSTD),
    
    -- Geospatial context
    latitude Nullable(Float64) CODEC(ZSTD),
    longitude Nullable(Float64) CODEC(ZSTD),
    
    -- Ingestion metadata
    ingestion_timestamp DateTime64(3) DEFAULT now64() CODEC(Delta, ZSTD),
    partition_date Date MATERIALIZED toDate(timestamp) CODEC(ZSTD)
)
ENGINE = MergeTree()
PARTITION BY partition_date
ORDER BY (vehicle_id, severity, timestamp)
TTL timestamp + INTERVAL 90 DAY  -- Alert retention: 90 days
SETTINGS index_granularity = 8192;

-- Vehicle Commands Table (for audit trail)
CREATE TABLE IF NOT EXISTS vehicle_commands (
    timestamp DateTime64(3) CODEC(Delta, ZSTD),
    command_id String CODEC(ZSTD),
    vehicle_id String CODEC(ZSTD),
    issuer_id String CODEC(ZSTD),
    command_type Enum8('MOTION' = 0, 'SAFETY' = 1, 'SYSTEM' = 2, 'TRIP' = 3, 'OTA' = 4, 'DIAGNOSTIC' = 5) CODEC(ZSTD),
    priority Enum8('LOW' = 0, 'NORMAL' = 1, 'HIGH' = 2, 'CRITICAL' = 3, 'EMERGENCY' = 4) CODEC(ZSTD),
    command_payload String CODEC(ZSTD),  -- JSON payload
    execution_status Enum8('PENDING' = 0, 'EXECUTING' = 1, 'COMPLETED' = 2, 'FAILED' = 3, 'TIMEOUT' = 4) CODEC(ZSTD),
    execution_start_time Nullable(DateTime64(3)) CODEC(ZSTD),
    execution_end_time Nullable(DateTime64(3)) CODEC(ZSTD),
    execution_duration_ms Nullable(UInt32) CODEC(ZSTD),
    error_message Nullable(String) CODEC(ZSTD),
    
    -- Ingestion metadata
    ingestion_timestamp DateTime64(3) DEFAULT now64() CODEC(Delta, ZSTD),
    partition_date Date MATERIALIZED toDate(timestamp) CODEC(ZSTD)
)
ENGINE = MergeTree()
PARTITION BY partition_date
ORDER BY (vehicle_id, timestamp)
TTL timestamp + INTERVAL 365 DAY  -- Command audit retention: 1 year
SETTINGS index_granularity = 8192;

-- Fleet Summary Materialized View (for dashboard performance)
CREATE MATERIALIZED VIEW IF NOT EXISTS fleet_summary_mv
ENGINE = SummingMergeTree()
PARTITION BY toYYYYMM(timestamp)
ORDER BY (sector, vehicle_type, toStartOfHour(timestamp))
AS SELECT
    toStartOfHour(timestamp) as timestamp,
    sector,
    vehicle_type,
    operational_mode,
    system_status,
    count() as telemetry_count,
    uniq(vehicle_id) as unique_vehicles,
    avg(speed) as avg_speed,
    max(speed) as max_speed,
    avg(battery_soc) as avg_battery_soc,
    avg(fuel_level) as avg_fuel_level,
    sum(emergency_stop_active) as emergency_stops,
    avg(control_loop_frequency) as avg_control_frequency,
    avg(communication_latency) as avg_comm_latency
FROM vehicle_telemetry
GROUP BY 
    toStartOfHour(timestamp),
    sector,
    vehicle_type,
    operational_mode,
    system_status;

-- Vehicle Performance Materialized View
CREATE MATERIALIZED VIEW IF NOT EXISTS vehicle_performance_mv
ENGINE = SummingMergeTree()
PARTITION BY toYYYYMM(timestamp)
ORDER BY (vehicle_id, toStartOfMinute(timestamp))
AS SELECT
    toStartOfMinute(timestamp) as timestamp,
    vehicle_id,
    sector,
    vehicle_type,
    count() as sample_count,
    avg(speed) as avg_speed,
    max(speed) as max_speed,
    avg(acceleration_x) as avg_acceleration,
    max(abs(acceleration_x)) as max_acceleration,
    avg(battery_soc) as avg_battery_soc,
    min(battery_soc) as min_battery_soc,
    avg(fuel_level) as avg_fuel_level,
    min(fuel_level) as min_fuel_level,
    avg(control_loop_frequency) as avg_control_frequency,
    min(control_loop_frequency) as min_control_frequency,
    avg(sensor_fusion_latency) as avg_sensor_latency,
    max(sensor_fusion_latency) as max_sensor_latency,
    avg(policy_evaluation_time) as avg_policy_time,
    max(policy_evaluation_time) as max_policy_time,
    avg(communication_latency) as avg_comm_latency,
    max(communication_latency) as max_comm_latency,
    avg(cpu_usage) as avg_cpu_usage,
    max(cpu_usage) as max_cpu_usage,
    avg(memory_usage) as avg_memory_usage,
    max(memory_usage) as max_memory_usage
FROM vehicle_telemetry
GROUP BY 
    toStartOfMinute(timestamp),
    vehicle_id,
    sector,
    vehicle_type;

-- Geospatial Analytics Table (for location-based queries)
CREATE TABLE IF NOT EXISTS vehicle_locations (
    timestamp DateTime64(3) CODEC(Delta, ZSTD),
    vehicle_id String CODEC(ZSTD),
    latitude Float64 CODEC(ZSTD),
    longitude Float64 CODEC(ZSTD),
    speed Float64 CODEC(ZSTD),
    heading Nullable(Float64) CODEC(ZSTD),
    operational_mode Enum8('MANUAL' = 0, 'ASSISTED' = 1, 'AUTONOMOUS' = 2, 'EMERGENCY' = 3, 'MAINTENANCE' = 4) CODEC(ZSTD),
    trip_id Nullable(String) CODEC(ZSTD),
    
    -- Geohash for spatial indexing
    geohash String MATERIALIZED geohashEncode(longitude, latitude, 12) CODEC(ZSTD),
    
    -- Ingestion metadata
    ingestion_timestamp DateTime64(3) DEFAULT now64() CODEC(Delta, ZSTD),
    partition_date Date MATERIALIZED toDate(timestamp) CODEC(ZSTD)
)
ENGINE = MergeTree()
PARTITION BY partition_date
ORDER BY (geohash, timestamp)
TTL timestamp + INTERVAL 30 DAY  -- Location data retention: 30 days
SETTINGS index_granularity = 8192;

-- Create indexes for better query performance
-- Skipping index for vehicle_id (for better filtering)
ALTER TABLE vehicle_telemetry ADD INDEX idx_vehicle_id vehicle_id TYPE bloom_filter GRANULARITY 1;
ALTER TABLE vehicle_telemetry ADD INDEX idx_trip_id trip_id TYPE bloom_filter GRANULARITY 1;
ALTER TABLE vehicle_telemetry ADD INDEX idx_sector sector TYPE set(100) GRANULARITY 1;

-- Projection for time-series queries
ALTER TABLE vehicle_telemetry ADD PROJECTION proj_time_series (
    SELECT 
        vehicle_id,
        timestamp,
        speed,
        battery_soc,
        fuel_level,
        operational_mode,
        system_status
    ORDER BY timestamp, vehicle_id
);

-- Create a dictionary for vehicle metadata (for joins)
CREATE DICTIONARY IF NOT EXISTS vehicle_metadata_dict (
    vehicle_id String,
    vehicle_type String,
    sector String,
    profile_version String,
    last_updated DateTime64(3)
)
PRIMARY KEY vehicle_id
SOURCE(CLICKHOUSE(
    HOST 'localhost'
    PORT 9000
    USER 'default'
    PASSWORD ''
    DB 'atlasmesh_telemetry'
    TABLE 'vehicle_metadata'
))
LIFETIME(MIN 300 MAX 600)  -- Cache for 5-10 minutes
LAYOUT(HASHED());

-- Vehicle metadata table (updated by fleet manager)
CREATE TABLE IF NOT EXISTS vehicle_metadata (
    vehicle_id String,
    vehicle_type String,
    sector String,
    profile_version String,
    manufacturer String,
    model String,
    year UInt16,
    registration_date Date,
    last_maintenance_date Nullable(Date),
    next_maintenance_date Nullable(Date),
    status Enum8('ACTIVE' = 0, 'INACTIVE' = 1, 'MAINTENANCE' = 2, 'RETIRED' = 3),
    last_updated DateTime64(3) DEFAULT now64()
)
ENGINE = ReplacingMergeTree(last_updated)
ORDER BY vehicle_id
SETTINGS index_granularity = 8192;

-- Data quality monitoring table
CREATE TABLE IF NOT EXISTS data_quality_metrics (
    timestamp DateTime64(3) DEFAULT now64(),
    metric_name String,
    metric_value Float64,
    vehicle_id Nullable(String),
    sector Nullable(String),
    tags Map(String, String),
    
    partition_date Date MATERIALIZED toDate(timestamp)
)
ENGINE = MergeTree()
PARTITION BY partition_date
ORDER BY (metric_name, timestamp)
TTL timestamp + INTERVAL 90 DAY
SETTINGS index_granularity = 8192;
