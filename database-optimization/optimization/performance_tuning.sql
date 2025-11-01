-- AtlasMesh Fleet OS - Database Performance Optimization
-- Query tuning, advanced indexing, partitioning strategies, connection pooling

-- =====================================================
-- ADVANCED INDEXING STRATEGIES
-- =====================================================

-- Fleet Management Indexes
CREATE INDEX CONCURRENTLY IF NOT EXISTS idx_fleets_status_created 
ON fleets (status, created_at DESC) 
WHERE status IN ('active', 'maintenance');

CREATE INDEX CONCURRENTLY IF NOT EXISTS idx_fleets_location_gist 
ON fleets USING GIST (location) 
WHERE location IS NOT NULL;

-- Vehicle Indexes with Partial Indexing
CREATE INDEX CONCURRENTLY IF NOT EXISTS idx_vehicles_active_status 
ON vehicles (fleet_id, status, last_seen DESC) 
WHERE status = 'active';

CREATE INDEX CONCURRENTLY IF NOT EXISTS idx_vehicles_maintenance_due 
ON vehicles (next_maintenance_date) 
WHERE next_maintenance_date <= CURRENT_DATE + INTERVAL '30 days';

CREATE INDEX CONCURRENTLY IF NOT EXISTS idx_vehicles_location_time 
ON vehicles USING GIST (current_location, last_seen);

-- Composite Index for Vehicle Telemetry (Hot Path)
CREATE INDEX CONCURRENTLY IF NOT EXISTS idx_vehicle_telemetry_hot 
ON vehicle_telemetry (vehicle_id, timestamp DESC, metric_type) 
WHERE timestamp >= CURRENT_TIMESTAMP - INTERVAL '24 hours';

-- Trip Management Indexes
CREATE INDEX CONCURRENTLY IF NOT EXISTS idx_trips_active_driver 
ON trips (driver_id, status, created_at DESC) 
WHERE status IN ('assigned', 'in_progress', 'started');

CREATE INDEX CONCURRENTLY IF NOT EXISTS idx_trips_route_optimization 
ON trips (pickup_location, dropoff_location, scheduled_time) 
WHERE status IN ('assigned', 'scheduled');

-- Driver Performance Indexes
CREATE INDEX CONCURRENTLY IF NOT EXISTS idx_drivers_performance 
ON drivers (status, rating DESC, total_trips DESC) 
WHERE status = 'active';

-- Maintenance Records Indexes
CREATE INDEX CONCURRENTLY IF NOT EXISTS idx_maintenance_due_date 
ON maintenance_records (vehicle_id, scheduled_date) 
WHERE status IN ('scheduled', 'in_progress');

CREATE INDEX CONCURRENTLY IF NOT EXISTS idx_maintenance_cost_analysis 
ON maintenance_records (maintenance_type, completed_date, cost) 
WHERE completed_date IS NOT NULL;

-- Route Optimization Indexes
CREATE INDEX CONCURRENTLY IF NOT EXISTS idx_routes_geospatial 
ON routes USING GIST (start_point, end_point);

CREATE INDEX CONCURRENTLY IF NOT EXISTS idx_routes_performance 
ON routes (estimated_duration, actual_duration, distance) 
WHERE actual_duration IS NOT NULL;

-- Sensor Data Indexes (Time-series optimization)
CREATE INDEX CONCURRENTLY IF NOT EXISTS idx_sensor_data_time_series 
ON sensor_data (sensor_id, timestamp DESC, value) 
WHERE timestamp >= CURRENT_TIMESTAMP - INTERVAL '7 days';

-- Incident Management Indexes
CREATE INDEX CONCURRENTLY IF NOT EXISTS idx_incidents_active_severity 
ON incidents (status, severity, created_at DESC) 
WHERE status IN ('open', 'investigating', 'in_progress');

-- =====================================================
-- TABLE PARTITIONING STRATEGIES
-- =====================================================

-- Partition vehicle_telemetry by time (monthly partitions)
-- First, create the parent table if not exists
CREATE TABLE IF NOT EXISTS vehicle_telemetry_partitioned (
    id BIGSERIAL,
    vehicle_id UUID NOT NULL,
    timestamp TIMESTAMP WITH TIME ZONE NOT NULL,
    metric_type VARCHAR(50) NOT NULL,
    value JSONB NOT NULL,
    location POINT,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP
) PARTITION BY RANGE (timestamp);

-- Create monthly partitions for the current year and next year
DO $$
DECLARE
    start_date DATE;
    end_date DATE;
    partition_name TEXT;
BEGIN
    -- Create partitions for current year
    FOR i IN 1..12 LOOP
        start_date := DATE_TRUNC('month', DATE '2024-01-01' + (i-1) * INTERVAL '1 month');
        end_date := start_date + INTERVAL '1 month';
        partition_name := 'vehicle_telemetry_y2024m' || LPAD(i::TEXT, 2, '0');
        
        EXECUTE format('CREATE TABLE IF NOT EXISTS %I PARTITION OF vehicle_telemetry_partitioned 
                       FOR VALUES FROM (%L) TO (%L)', 
                       partition_name, start_date, end_date);
        
        -- Add indexes to each partition
        EXECUTE format('CREATE INDEX IF NOT EXISTS %I ON %I (vehicle_id, timestamp DESC)', 
                       partition_name || '_vehicle_time_idx', partition_name);
        EXECUTE format('CREATE INDEX IF NOT EXISTS %I ON %I (metric_type, timestamp DESC)', 
                       partition_name || '_metric_time_idx', partition_name);
    END LOOP;
    
    -- Create partitions for next year
    FOR i IN 1..12 LOOP
        start_date := DATE_TRUNC('month', DATE '2025-01-01' + (i-1) * INTERVAL '1 month');
        end_date := start_date + INTERVAL '1 month';
        partition_name := 'vehicle_telemetry_y2025m' || LPAD(i::TEXT, 2, '0');
        
        EXECUTE format('CREATE TABLE IF NOT EXISTS %I PARTITION OF vehicle_telemetry_partitioned 
                       FOR VALUES FROM (%L) TO (%L)', 
                       partition_name, start_date, end_date);
        
        EXECUTE format('CREATE INDEX IF NOT EXISTS %I ON %I (vehicle_id, timestamp DESC)', 
                       partition_name || '_vehicle_time_idx', partition_name);
        EXECUTE format('CREATE INDEX IF NOT EXISTS %I ON %I (metric_type, timestamp DESC)', 
                       partition_name || '_metric_time_idx', partition_name);
    END LOOP;
END $$;

-- Partition trip_history by date
CREATE TABLE IF NOT EXISTS trip_history_partitioned (
    id BIGSERIAL,
    trip_id UUID NOT NULL,
    vehicle_id UUID NOT NULL,
    driver_id UUID,
    start_time TIMESTAMP WITH TIME ZONE NOT NULL,
    end_time TIMESTAMP WITH TIME ZONE,
    pickup_location POINT NOT NULL,
    dropoff_location POINT NOT NULL,
    distance_km DECIMAL(10,2),
    duration_minutes INTEGER,
    fare DECIMAL(10,2),
    status VARCHAR(20) NOT NULL,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP
) PARTITION BY RANGE (start_time);

-- Create quarterly partitions for trip history
DO $$
DECLARE
    start_date DATE;
    end_date DATE;
    partition_name TEXT;
    quarter INTEGER;
BEGIN
    -- Create partitions for 2024
    FOR quarter IN 1..4 LOOP
        start_date := DATE_TRUNC('quarter', DATE '2024-01-01' + (quarter-1) * INTERVAL '3 months');
        end_date := start_date + INTERVAL '3 months';
        partition_name := 'trip_history_2024_q' || quarter;
        
        EXECUTE format('CREATE TABLE IF NOT EXISTS %I PARTITION OF trip_history_partitioned 
                       FOR VALUES FROM (%L) TO (%L)', 
                       partition_name, start_date, end_date);
        
        EXECUTE format('CREATE INDEX IF NOT EXISTS %I ON %I (vehicle_id, start_time DESC)', 
                       partition_name || '_vehicle_idx', partition_name);
        EXECUTE format('CREATE INDEX IF NOT EXISTS %I ON %I (driver_id, start_time DESC)', 
                       partition_name || '_driver_idx', partition_name);
    END LOOP;
    
    -- Create partitions for 2025
    FOR quarter IN 1..4 LOOP
        start_date := DATE_TRUNC('quarter', DATE '2025-01-01' + (quarter-1) * INTERVAL '3 months');
        end_date := start_date + INTERVAL '3 months';
        partition_name := 'trip_history_2025_q' || quarter;
        
        EXECUTE format('CREATE TABLE IF NOT EXISTS %I PARTITION OF trip_history_partitioned 
                       FOR VALUES FROM (%L) TO (%L)', 
                       partition_name, start_date, end_date);
        
        EXECUTE format('CREATE INDEX IF NOT EXISTS %I ON %I (vehicle_id, start_time DESC)', 
                       partition_name || '_vehicle_idx', partition_name);
        EXECUTE format('CREATE INDEX IF NOT EXISTS %I ON %I (driver_id, start_time DESC)', 
                       partition_name || '_driver_idx', partition_name);
    END LOOP;
END $$;

-- =====================================================
-- MATERIALIZED VIEWS FOR PERFORMANCE
-- =====================================================

-- Fleet Performance Summary (Refreshed every 15 minutes)
CREATE MATERIALIZED VIEW IF NOT EXISTS mv_fleet_performance_summary AS
SELECT 
    f.fleet_id,
    f.name as fleet_name,
    COUNT(v.vehicle_id) as total_vehicles,
    COUNT(CASE WHEN v.status = 'active' THEN 1 END) as active_vehicles,
    COUNT(CASE WHEN v.status = 'maintenance' THEN 1 END) as maintenance_vehicles,
    AVG(v.fuel_level) as avg_fuel_level,
    AVG(v.battery_level) as avg_battery_level,
    COUNT(CASE WHEN v.last_seen >= CURRENT_TIMESTAMP - INTERVAL '1 hour' THEN 1 END) as online_vehicles,
    CURRENT_TIMESTAMP as last_updated
FROM fleets f
LEFT JOIN vehicles v ON f.fleet_id = v.fleet_id
WHERE f.status = 'active'
GROUP BY f.fleet_id, f.name;

CREATE UNIQUE INDEX IF NOT EXISTS mv_fleet_performance_summary_pkey 
ON mv_fleet_performance_summary (fleet_id);

-- Vehicle Health Metrics (Refreshed every 30 minutes)
CREATE MATERIALIZED VIEW IF NOT EXISTS mv_vehicle_health_metrics AS
SELECT 
    v.vehicle_id,
    v.fleet_id,
    v.license_plate,
    v.status,
    v.fuel_level,
    v.battery_level,
    v.mileage,
    EXTRACT(DAYS FROM (CURRENT_DATE - v.last_maintenance_date)) as days_since_maintenance,
    EXTRACT(DAYS FROM (v.next_maintenance_date - CURRENT_DATE)) as days_until_maintenance,
    CASE 
        WHEN v.fuel_level < 20 OR v.battery_level < 20 THEN 'critical'
        WHEN v.fuel_level < 40 OR v.battery_level < 40 THEN 'warning'
        ELSE 'good'
    END as health_status,
    -- Calculate health score (0-100)
    LEAST(100, GREATEST(0, 
        (COALESCE(v.fuel_level, 100) + COALESCE(v.battery_level, 100)) / 2 *
        CASE 
            WHEN EXTRACT(DAYS FROM (v.next_maintenance_date - CURRENT_DATE)) < 0 THEN 0.5
            WHEN EXTRACT(DAYS FROM (v.next_maintenance_date - CURRENT_DATE)) < 7 THEN 0.8
            ELSE 1.0
        END
    )) as health_score,
    CURRENT_TIMESTAMP as last_updated
FROM vehicles v
WHERE v.status != 'decommissioned';

CREATE UNIQUE INDEX IF NOT EXISTS mv_vehicle_health_metrics_pkey 
ON mv_vehicle_health_metrics (vehicle_id);

-- Driver Performance Metrics (Refreshed daily)
CREATE MATERIALIZED VIEW IF NOT EXISTS mv_driver_performance_metrics AS
SELECT 
    d.driver_id,
    d.employee_id,
    d.first_name,
    d.last_name,
    d.status,
    COUNT(t.trip_id) as total_trips_30d,
    AVG(t.rating) as avg_rating_30d,
    SUM(t.distance_km) as total_distance_30d,
    SUM(t.duration_minutes) as total_duration_30d,
    COUNT(CASE WHEN t.status = 'completed' THEN 1 END) as completed_trips_30d,
    COUNT(CASE WHEN t.status = 'cancelled' THEN 1 END) as cancelled_trips_30d,
    ROUND(
        COUNT(CASE WHEN t.status = 'completed' THEN 1 END)::DECIMAL / 
        NULLIF(COUNT(t.trip_id), 0) * 100, 2
    ) as completion_rate_30d,
    AVG(CASE WHEN t.status = 'completed' THEN t.duration_minutes END) as avg_trip_duration,
    CURRENT_TIMESTAMP as last_updated
FROM drivers d
LEFT JOIN trips t ON d.driver_id = t.driver_id 
    AND t.created_at >= CURRENT_DATE - INTERVAL '30 days'
WHERE d.status IN ('active', 'inactive')
GROUP BY d.driver_id, d.employee_id, d.first_name, d.last_name, d.status;

CREATE UNIQUE INDEX IF NOT EXISTS mv_driver_performance_metrics_pkey 
ON mv_driver_performance_metrics (driver_id);

-- Route Efficiency Analysis (Refreshed every 2 hours)
CREATE MATERIALIZED VIEW IF NOT EXISTS mv_route_efficiency_analysis AS
SELECT 
    r.route_id,
    r.name as route_name,
    r.start_location,
    r.end_location,
    r.distance_km,
    r.estimated_duration_minutes,
    COUNT(t.trip_id) as trip_count_7d,
    AVG(t.actual_duration_minutes) as avg_actual_duration,
    AVG(t.distance_km) as avg_actual_distance,
    ROUND(
        (r.estimated_duration_minutes::DECIMAL / NULLIF(AVG(t.actual_duration_minutes), 0)) * 100, 2
    ) as duration_accuracy_pct,
    ROUND(
        (r.distance_km::DECIMAL / NULLIF(AVG(t.distance_km), 0)) * 100, 2
    ) as distance_accuracy_pct,
    CASE 
        WHEN AVG(t.actual_duration_minutes) <= r.estimated_duration_minutes * 1.1 THEN 'excellent'
        WHEN AVG(t.actual_duration_minutes) <= r.estimated_duration_minutes * 1.2 THEN 'good'
        WHEN AVG(t.actual_duration_minutes) <= r.estimated_duration_minutes * 1.3 THEN 'fair'
        ELSE 'poor'
    END as efficiency_rating,
    CURRENT_TIMESTAMP as last_updated
FROM routes r
LEFT JOIN trips t ON r.route_id = t.route_id 
    AND t.completed_at >= CURRENT_DATE - INTERVAL '7 days'
    AND t.status = 'completed'
GROUP BY r.route_id, r.name, r.start_location, r.end_location, r.distance_km, r.estimated_duration_minutes;

CREATE UNIQUE INDEX IF NOT EXISTS mv_route_efficiency_analysis_pkey 
ON mv_route_efficiency_analysis (route_id);

-- =====================================================
-- QUERY OPTIMIZATION FUNCTIONS
-- =====================================================

-- Function to get vehicle telemetry with optimized query
CREATE OR REPLACE FUNCTION get_vehicle_telemetry_optimized(
    p_vehicle_id UUID,
    p_metric_type VARCHAR DEFAULT NULL,
    p_hours_back INTEGER DEFAULT 24,
    p_limit INTEGER DEFAULT 1000
)
RETURNS TABLE (
    timestamp TIMESTAMP WITH TIME ZONE,
    metric_type VARCHAR,
    value JSONB,
    location POINT
) AS $$
BEGIN
    RETURN QUERY
    SELECT 
        vt.timestamp,
        vt.metric_type,
        vt.value,
        vt.location
    FROM vehicle_telemetry_partitioned vt
    WHERE vt.vehicle_id = p_vehicle_id
        AND vt.timestamp >= CURRENT_TIMESTAMP - (p_hours_back || ' hours')::INTERVAL
        AND (p_metric_type IS NULL OR vt.metric_type = p_metric_type)
    ORDER BY vt.timestamp DESC
    LIMIT p_limit;
END;
$$ LANGUAGE plpgsql STABLE;

-- Function to get fleet summary with caching
CREATE OR REPLACE FUNCTION get_fleet_summary_cached(p_fleet_id UUID DEFAULT NULL)
RETURNS TABLE (
    fleet_id UUID,
    fleet_name VARCHAR,
    total_vehicles BIGINT,
    active_vehicles BIGINT,
    maintenance_vehicles BIGINT,
    avg_fuel_level DECIMAL,
    avg_battery_level DECIMAL,
    online_vehicles BIGINT,
    last_updated TIMESTAMP WITH TIME ZONE
) AS $$
BEGIN
    -- Refresh materialized view if data is older than 15 minutes
    IF (SELECT MAX(last_updated) FROM mv_fleet_performance_summary) < CURRENT_TIMESTAMP - INTERVAL '15 minutes' THEN
        REFRESH MATERIALIZED VIEW CONCURRENTLY mv_fleet_performance_summary;
    END IF;
    
    RETURN QUERY
    SELECT 
        fps.fleet_id,
        fps.fleet_name,
        fps.total_vehicles,
        fps.active_vehicles,
        fps.maintenance_vehicles,
        fps.avg_fuel_level,
        fps.avg_battery_level,
        fps.online_vehicles,
        fps.last_updated
    FROM mv_fleet_performance_summary fps
    WHERE p_fleet_id IS NULL OR fps.fleet_id = p_fleet_id
    ORDER BY fps.fleet_name;
END;
$$ LANGUAGE plpgsql STABLE;

-- Function for efficient trip search with geospatial optimization
CREATE OR REPLACE FUNCTION search_trips_optimized(
    p_pickup_lat DECIMAL DEFAULT NULL,
    p_pickup_lng DECIMAL DEFAULT NULL,
    p_radius_km DECIMAL DEFAULT 5,
    p_date_from DATE DEFAULT NULL,
    p_date_to DATE DEFAULT NULL,
    p_status VARCHAR DEFAULT NULL,
    p_limit INTEGER DEFAULT 100
)
RETURNS TABLE (
    trip_id UUID,
    vehicle_id UUID,
    driver_id UUID,
    pickup_location POINT,
    dropoff_location POINT,
    start_time TIMESTAMP WITH TIME ZONE,
    end_time TIMESTAMP WITH TIME ZONE,
    status VARCHAR,
    distance_km DECIMAL,
    duration_minutes INTEGER
) AS $$
DECLARE
    pickup_point POINT;
BEGIN
    -- Create point for geospatial search if coordinates provided
    IF p_pickup_lat IS NOT NULL AND p_pickup_lng IS NOT NULL THEN
        pickup_point := POINT(p_pickup_lng, p_pickup_lat);
    END IF;
    
    RETURN QUERY
    SELECT 
        t.trip_id,
        t.vehicle_id,
        t.driver_id,
        t.pickup_location,
        t.dropoff_location,
        t.start_time,
        t.end_time,
        t.status,
        t.distance_km,
        t.duration_minutes
    FROM trips t
    WHERE (p_date_from IS NULL OR DATE(t.start_time) >= p_date_from)
        AND (p_date_to IS NULL OR DATE(t.start_time) <= p_date_to)
        AND (p_status IS NULL OR t.status = p_status)
        AND (pickup_point IS NULL OR 
             ST_DWithin(
                 ST_GeogFromText('POINT(' || t.pickup_location[0] || ' ' || t.pickup_location[1] || ')'),
                 ST_GeogFromText('POINT(' || pickup_point[0] || ' ' || pickup_point[1] || ')'),
                 p_radius_km * 1000
             ))
    ORDER BY t.start_time DESC
    LIMIT p_limit;
END;
$$ LANGUAGE plpgsql STABLE;

-- =====================================================
-- AUTOMATED MAINTENANCE PROCEDURES
-- =====================================================

-- Procedure to refresh all materialized views
CREATE OR REPLACE FUNCTION refresh_performance_views()
RETURNS VOID AS $$
BEGIN
    -- Refresh fleet performance summary
    REFRESH MATERIALIZED VIEW CONCURRENTLY mv_fleet_performance_summary;
    
    -- Refresh vehicle health metrics
    REFRESH MATERIALIZED VIEW CONCURRENTLY mv_vehicle_health_metrics;
    
    -- Refresh driver performance metrics (less frequent)
    IF EXTRACT(HOUR FROM CURRENT_TIME) IN (2, 14) THEN
        REFRESH MATERIALIZED VIEW CONCURRENTLY mv_driver_performance_metrics;
    END IF;
    
    -- Refresh route efficiency analysis
    IF EXTRACT(HOUR FROM CURRENT_TIME) % 2 = 0 THEN
        REFRESH MATERIALIZED VIEW CONCURRENTLY mv_route_efficiency_analysis;
    END IF;
    
    RAISE NOTICE 'Performance views refreshed at %', CURRENT_TIMESTAMP;
END;
$$ LANGUAGE plpgsql;

-- Procedure to clean up old partitions
CREATE OR REPLACE FUNCTION cleanup_old_partitions()
RETURNS VOID AS $$
DECLARE
    partition_record RECORD;
    cutoff_date DATE;
BEGIN
    -- Keep data for 2 years, drop older partitions
    cutoff_date := CURRENT_DATE - INTERVAL '2 years';
    
    -- Find and drop old telemetry partitions
    FOR partition_record IN 
        SELECT schemaname, tablename 
        FROM pg_tables 
        WHERE tablename LIKE 'vehicle_telemetry_y%'
            AND tablename < 'vehicle_telemetry_y' || EXTRACT(YEAR FROM cutoff_date)
    LOOP
        EXECUTE 'DROP TABLE IF EXISTS ' || partition_record.schemaname || '.' || partition_record.tablename;
        RAISE NOTICE 'Dropped old partition: %', partition_record.tablename;
    END LOOP;
    
    -- Find and drop old trip history partitions
    FOR partition_record IN 
        SELECT schemaname, tablename 
        FROM pg_tables 
        WHERE tablename LIKE 'trip_history_%'
            AND tablename < 'trip_history_' || EXTRACT(YEAR FROM cutoff_date)
    LOOP
        EXECUTE 'DROP TABLE IF EXISTS ' || partition_record.schemaname || '.' || partition_record.tablename;
        RAISE NOTICE 'Dropped old partition: %', partition_record.tablename;
    END LOOP;
END;
$$ LANGUAGE plpgsql;

-- =====================================================
-- PERFORMANCE MONITORING VIEWS
-- =====================================================

-- View to monitor slow queries
CREATE OR REPLACE VIEW v_slow_queries AS
SELECT 
    query,
    calls,
    total_time,
    mean_time,
    max_time,
    stddev_time,
    rows,
    100.0 * shared_blks_hit / nullif(shared_blks_hit + shared_blks_read, 0) AS hit_percent
FROM pg_stat_statements 
WHERE calls > 100
ORDER BY mean_time DESC;

-- View to monitor table sizes and bloat
CREATE OR REPLACE VIEW v_table_sizes AS
SELECT 
    schemaname,
    tablename,
    pg_size_pretty(pg_total_relation_size(schemaname||'.'||tablename)) as size,
    pg_size_pretty(pg_relation_size(schemaname||'.'||tablename)) as table_size,
    pg_size_pretty(pg_total_relation_size(schemaname||'.'||tablename) - pg_relation_size(schemaname||'.'||tablename)) as index_size,
    n_tup_ins + n_tup_upd + n_tup_del as total_writes,
    n_tup_hot_upd,
    n_dead_tup,
    last_vacuum,
    last_autovacuum,
    last_analyze,
    last_autoanalyze
FROM pg_stat_user_tables 
ORDER BY pg_total_relation_size(schemaname||'.'||tablename) DESC;

-- View to monitor index usage
CREATE OR REPLACE VIEW v_index_usage AS
SELECT 
    schemaname,
    tablename,
    indexname,
    idx_tup_read,
    idx_tup_fetch,
    idx_scan,
    CASE 
        WHEN idx_scan = 0 THEN 'Never used'
        WHEN idx_scan < 100 THEN 'Rarely used'
        ELSE 'Frequently used'
    END as usage_status,
    pg_size_pretty(pg_relation_size(indexrelid)) as index_size
FROM pg_stat_user_indexes
ORDER BY idx_scan DESC;

-- =====================================================
-- CONFIGURATION RECOMMENDATIONS
-- =====================================================

-- Create a function to provide configuration recommendations
CREATE OR REPLACE FUNCTION get_performance_recommendations()
RETURNS TABLE (
    category VARCHAR,
    recommendation TEXT,
    current_value TEXT,
    suggested_value TEXT,
    impact VARCHAR
) AS $$
BEGIN
    RETURN QUERY
    SELECT 
        'Memory'::VARCHAR as category,
        'shared_buffers should be 25% of RAM'::TEXT as recommendation,
        current_setting('shared_buffers') as current_value,
        '2GB'::TEXT as suggested_value,
        'High'::VARCHAR as impact
    UNION ALL
    SELECT 
        'Memory'::VARCHAR,
        'effective_cache_size should be 75% of RAM'::TEXT,
        current_setting('effective_cache_size'),
        '6GB'::TEXT,
        'Medium'::VARCHAR
    UNION ALL
    SELECT 
        'Connections'::VARCHAR,
        'max_connections for fleet workload'::TEXT,
        current_setting('max_connections'),
        '200'::TEXT,
        'Medium'::VARCHAR
    UNION ALL
    SELECT 
        'WAL'::VARCHAR,
        'wal_buffers for write-heavy workload'::TEXT,
        current_setting('wal_buffers'),
        '16MB'::TEXT,
        'Medium'::VARCHAR
    UNION ALL
    SELECT 
        'Checkpoints'::VARCHAR,
        'checkpoint_completion_target'::TEXT,
        current_setting('checkpoint_completion_target'),
        '0.9'::TEXT,
        'Low'::VARCHAR;
END;
$$ LANGUAGE plpgsql STABLE;

-- =====================================================
-- AUTOMATED STATISTICS COLLECTION
-- =====================================================

-- Function to collect performance statistics
CREATE OR REPLACE FUNCTION collect_performance_stats()
RETURNS VOID AS $$
BEGIN
    -- Create performance stats table if not exists
    CREATE TABLE IF NOT EXISTS performance_stats (
        id SERIAL PRIMARY KEY,
        collected_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
        metric_name VARCHAR(100) NOT NULL,
        metric_value DECIMAL,
        metric_unit VARCHAR(20),
        additional_info JSONB
    );
    
    -- Collect database size
    INSERT INTO performance_stats (metric_name, metric_value, metric_unit, additional_info)
    SELECT 
        'database_size_mb',
        pg_database_size(current_database()) / 1024.0 / 1024.0,
        'MB',
        jsonb_build_object('database', current_database());
    
    -- Collect connection count
    INSERT INTO performance_stats (metric_name, metric_value, metric_unit)
    SELECT 
        'active_connections',
        COUNT(*),
        'count'
    FROM pg_stat_activity 
    WHERE state = 'active';
    
    -- Collect cache hit ratio
    INSERT INTO performance_stats (metric_name, metric_value, metric_unit)
    SELECT 
        'cache_hit_ratio',
        100.0 * sum(blks_hit) / nullif(sum(blks_hit + blks_read), 0),
        'percent'
    FROM pg_stat_database;
    
    -- Collect average query time
    INSERT INTO performance_stats (metric_name, metric_value, metric_unit)
    SELECT 
        'avg_query_time_ms',
        AVG(mean_time),
        'ms'
    FROM pg_stat_statements
    WHERE calls > 10;
    
    RAISE NOTICE 'Performance statistics collected at %', CURRENT_TIMESTAMP;
END;
$$ LANGUAGE plpgsql;

-- =====================================================
-- SCHEDULED MAINTENANCE JOBS
-- =====================================================

-- Note: These would typically be scheduled using pg_cron or external scheduler

-- Schedule to refresh materialized views every 15 minutes
-- SELECT cron.schedule('refresh-views', '*/15 * * * *', 'SELECT refresh_performance_views();');

-- Schedule to collect performance stats every hour
-- SELECT cron.schedule('collect-stats', '0 * * * *', 'SELECT collect_performance_stats();');

-- Schedule to cleanup old partitions weekly
-- SELECT cron.schedule('cleanup-partitions', '0 2 * * 0', 'SELECT cleanup_old_partitions();');

-- Schedule to analyze tables daily
-- SELECT cron.schedule('analyze-tables', '0 3 * * *', 'ANALYZE;');

-- =====================================================
-- PERFORMANCE TESTING QUERIES
-- =====================================================

-- Test query performance for common operations
CREATE OR REPLACE FUNCTION test_query_performance()
RETURNS TABLE (
    test_name VARCHAR,
    execution_time_ms DECIMAL,
    rows_returned BIGINT
) AS $$
DECLARE
    start_time TIMESTAMP;
    end_time TIMESTAMP;
    row_count BIGINT;
BEGIN
    -- Test 1: Fleet summary query
    start_time := clock_timestamp();
    SELECT COUNT(*) INTO row_count FROM get_fleet_summary_cached();
    end_time := clock_timestamp();
    
    RETURN QUERY SELECT 
        'Fleet Summary Query'::VARCHAR,
        EXTRACT(MILLISECONDS FROM (end_time - start_time))::DECIMAL,
        row_count;
    
    -- Test 2: Vehicle telemetry query
    start_time := clock_timestamp();
    SELECT COUNT(*) INTO row_count 
    FROM get_vehicle_telemetry_optimized(
        (SELECT vehicle_id FROM vehicles LIMIT 1),
        NULL, 24, 1000
    );
    end_time := clock_timestamp();
    
    RETURN QUERY SELECT 
        'Vehicle Telemetry Query'::VARCHAR,
        EXTRACT(MILLISECONDS FROM (end_time - start_time))::DECIMAL,
        row_count;
    
    -- Test 3: Trip search query
    start_time := clock_timestamp();
    SELECT COUNT(*) INTO row_count 
    FROM search_trips_optimized(25.2048, 55.2708, 5, CURRENT_DATE - 7, CURRENT_DATE);
    end_time := clock_timestamp();
    
    RETURN QUERY SELECT 
        'Trip Search Query'::VARCHAR,
        EXTRACT(MILLISECONDS FROM (end_time - start_time))::DECIMAL,
        row_count;
END;
$$ LANGUAGE plpgsql;

-- Final optimization: Update table statistics
ANALYZE;
