-- AtlasMesh Fleet OS - Fleet Performance Data Mart
-- Optimized for fleet analytics and business intelligence

-- Create Fleet Performance Data Mart
CREATE DATABASE IF NOT EXISTS atlasmesh_analytics;

USE atlasmesh_analytics;

-- =====================================================
-- FLEET PERFORMANCE MART TABLES
-- =====================================================

-- Fleet Performance Summary (Hourly Aggregation)
CREATE TABLE IF NOT EXISTS fleet_performance_hourly
(
    date_hour DateTime,
    fleet_id String,
    fleet_name String,
    
    -- Vehicle Metrics
    total_vehicles UInt32,
    active_vehicles UInt32,
    idle_vehicles UInt32,
    maintenance_vehicles UInt32,
    offline_vehicles UInt32,
    
    -- Utilization Metrics
    utilization_rate Float64,
    avg_trip_duration_minutes Float64,
    avg_distance_per_trip_km Float64,
    total_trips UInt32,
    completed_trips UInt32,
    cancelled_trips UInt32,
    
    -- Performance Metrics
    avg_speed_kmh Float64,
    fuel_efficiency_km_per_liter Float64,
    energy_efficiency_km_per_kwh Float64,
    avg_fuel_level_percent Float64,
    avg_battery_level_percent Float64,
    
    -- Financial Metrics
    total_revenue Decimal(15,2),
    total_cost Decimal(15,2),
    profit_margin Float64,
    cost_per_km Decimal(10,4),
    revenue_per_km Decimal(10,4),
    
    -- Quality Metrics
    avg_customer_rating Float64,
    on_time_performance Float64,
    safety_score Float64,
    maintenance_score Float64,
    
    -- Environmental Metrics
    co2_emissions_kg Float64,
    fuel_consumption_liters Float64,
    energy_consumption_kwh Float64,
    
    -- Operational Metrics
    avg_wait_time_minutes Float64,
    avg_pickup_time_minutes Float64,
    route_deviation_percent Float64,
    
    created_at DateTime DEFAULT now(),
    updated_at DateTime DEFAULT now()
)
ENGINE = ReplacingMergeTree(updated_at)
PARTITION BY toYYYYMM(date_hour)
ORDER BY (fleet_id, date_hour)
TTL date_hour + INTERVAL 2 YEAR;

-- Fleet Performance Summary (Daily Aggregation)
CREATE TABLE IF NOT EXISTS fleet_performance_daily
(
    date Date,
    fleet_id String,
    fleet_name String,
    
    -- Aggregated from hourly data
    avg_utilization_rate Float64,
    max_utilization_rate Float64,
    min_utilization_rate Float64,
    
    total_trips UInt32,
    total_distance_km Float64,
    total_duration_minutes UInt64,
    
    avg_performance_score Float64,
    total_revenue Decimal(15,2),
    total_cost Decimal(15,2),
    
    -- Daily specific metrics
    peak_hour_utilization Float64,
    off_peak_utilization Float64,
    weekend_adjustment Float64,
    
    created_at DateTime DEFAULT now(),
    updated_at DateTime DEFAULT now()
)
ENGINE = ReplacingMergeTree(updated_at)
PARTITION BY toYYYYMM(date)
ORDER BY (fleet_id, date)
TTL date + INTERVAL 5 YEAR;

-- Vehicle Performance Summary
CREATE TABLE IF NOT EXISTS vehicle_performance_daily
(
    date Date,
    vehicle_id String,
    fleet_id String,
    vehicle_type String,
    license_plate String,
    
    -- Usage Metrics
    total_trips UInt32,
    total_distance_km Float64,
    total_duration_minutes UInt64,
    utilization_hours Float64,
    idle_time_minutes UInt64,
    
    -- Performance Metrics
    avg_speed_kmh Float64,
    max_speed_kmh Float64,
    fuel_efficiency Float64,
    energy_efficiency Float64,
    
    -- Health Metrics
    health_score Float64,
    maintenance_alerts UInt32,
    fault_codes Array(String),
    
    -- Financial Metrics
    revenue_generated Decimal(10,2),
    operating_cost Decimal(10,2),
    maintenance_cost Decimal(10,2),
    
    -- Quality Metrics
    customer_rating Float64,
    on_time_trips UInt32,
    late_trips UInt32,
    cancelled_trips UInt32,
    
    created_at DateTime DEFAULT now(),
    updated_at DateTime DEFAULT now()
)
ENGINE = ReplacingMergeTree(updated_at)
PARTITION BY toYYYYMM(date)
ORDER BY (vehicle_id, date)
TTL date + INTERVAL 3 YEAR;

-- Driver Performance Summary
CREATE TABLE IF NOT EXISTS driver_performance_daily
(
    date Date,
    driver_id String,
    fleet_id String,
    employee_id String,
    
    -- Trip Metrics
    total_trips UInt32,
    completed_trips UInt32,
    cancelled_trips UInt32,
    total_distance_km Float64,
    total_duration_minutes UInt64,
    
    -- Performance Metrics
    avg_rating Float64,
    safety_score Float64,
    efficiency_score Float64,
    punctuality_score Float64,
    
    -- Behavioral Metrics
    harsh_braking_events UInt32,
    harsh_acceleration_events UInt32,
    speeding_events UInt32,
    idle_time_minutes UInt64,
    
    -- Financial Metrics
    earnings Decimal(10,2),
    bonus Decimal(10,2),
    penalties Decimal(10,2),
    
    -- Working Hours
    working_hours Float64,
    break_time_minutes UInt64,
    overtime_hours Float64,
    
    created_at DateTime DEFAULT now(),
    updated_at DateTime DEFAULT now()
)
ENGINE = ReplacingMergeTree(updated_at)
PARTITION BY toYYYYMM(date)
ORDER BY (driver_id, date)
TTL date + INTERVAL 3 YEAR;

-- Route Performance Summary
CREATE TABLE IF NOT EXISTS route_performance_daily
(
    date Date,
    route_id String,
    route_name String,
    start_location String,
    end_location String,
    
    -- Usage Metrics
    total_trips UInt32,
    avg_duration_minutes Float64,
    avg_distance_km Float64,
    
    -- Performance Metrics
    on_time_percentage Float64,
    avg_delay_minutes Float64,
    completion_rate Float64,
    
    -- Traffic Metrics
    avg_traffic_density Float64,
    peak_hour_delay Float64,
    weather_impact_score Float64,
    
    -- Cost Metrics
    avg_fuel_cost Decimal(8,2),
    avg_maintenance_cost Decimal(8,2),
    cost_per_km Decimal(6,4),
    
    created_at DateTime DEFAULT now(),
    updated_at DateTime DEFAULT now()
)
ENGINE = ReplacingMergeTree(updated_at)
PARTITION BY toYYYYMM(date)
ORDER BY (route_id, date)
TTL date + INTERVAL 2 YEAR;

-- =====================================================
-- MATERIALIZED VIEWS FOR REAL-TIME AGGREGATION
-- =====================================================

-- Real-time Fleet Performance View
CREATE MATERIALIZED VIEW IF NOT EXISTS mv_fleet_performance_realtime
TO fleet_performance_hourly
AS SELECT
    toStartOfHour(timestamp) as date_hour,
    fleet_id,
    any(fleet_name) as fleet_name,
    
    -- Vehicle counts
    uniqExact(vehicle_id) as total_vehicles,
    countIf(status = 'active') as active_vehicles,
    countIf(status = 'idle') as idle_vehicles,
    countIf(status = 'maintenance') as maintenance_vehicles,
    countIf(status = 'offline') as offline_vehicles,
    
    -- Utilization
    avg(utilization_rate) as utilization_rate,
    avg(trip_duration_minutes) as avg_trip_duration_minutes,
    avg(distance_km) as avg_distance_per_trip_km,
    count() as total_trips,
    countIf(trip_status = 'completed') as completed_trips,
    countIf(trip_status = 'cancelled') as cancelled_trips,
    
    -- Performance
    avg(speed_kmh) as avg_speed_kmh,
    avg(fuel_efficiency) as fuel_efficiency_km_per_liter,
    avg(energy_efficiency) as energy_efficiency_km_per_kwh,
    avg(fuel_level_percent) as avg_fuel_level_percent,
    avg(battery_level_percent) as avg_battery_level_percent,
    
    -- Financial
    sum(revenue) as total_revenue,
    sum(cost) as total_cost,
    (sum(revenue) - sum(cost)) / sum(revenue) * 100 as profit_margin,
    sum(cost) / sum(distance_km) as cost_per_km,
    sum(revenue) / sum(distance_km) as revenue_per_km,
    
    -- Quality
    avg(customer_rating) as avg_customer_rating,
    countIf(on_time = 1) / count() * 100 as on_time_performance,
    avg(safety_score) as safety_score,
    avg(maintenance_score) as maintenance_score,
    
    -- Environmental
    sum(co2_emissions_kg) as co2_emissions_kg,
    sum(fuel_consumption_liters) as fuel_consumption_liters,
    sum(energy_consumption_kwh) as energy_consumption_kwh,
    
    -- Operational
    avg(wait_time_minutes) as avg_wait_time_minutes,
    avg(pickup_time_minutes) as avg_pickup_time_minutes,
    avg(route_deviation_percent) as route_deviation_percent,
    
    now() as created_at,
    now() as updated_at
FROM vehicle_telemetry_stream
WHERE timestamp >= now() - INTERVAL 1 HOUR
GROUP BY date_hour, fleet_id;

-- =====================================================
-- OLAP CUBE DEFINITIONS
-- =====================================================

-- Fleet Performance Cube (Multi-dimensional analysis)
CREATE TABLE IF NOT EXISTS fleet_performance_cube
(
    -- Dimensions
    date_dimension Date,
    time_dimension DateTime,
    fleet_dimension String,
    vehicle_type_dimension String,
    location_dimension String,
    weather_dimension String,
    
    -- Measures
    trip_count UInt32,
    distance_sum Float64,
    duration_sum UInt64,
    revenue_sum Decimal(15,2),
    cost_sum Decimal(15,2),
    fuel_consumption_sum Float64,
    co2_emissions_sum Float64,
    
    -- Calculated Measures
    utilization_rate Float64,
    efficiency_score Float64,
    profitability_ratio Float64,
    customer_satisfaction Float64,
    
    created_at DateTime DEFAULT now()
)
ENGINE = SummingMergeTree()
PARTITION BY toYYYYMM(date_dimension)
ORDER BY (fleet_dimension, vehicle_type_dimension, location_dimension, date_dimension)
TTL date_dimension + INTERVAL 3 YEAR;

-- =====================================================
-- BUSINESS INTELLIGENCE VIEWS
-- =====================================================

-- Executive Dashboard View
CREATE VIEW IF NOT EXISTS v_executive_dashboard AS
SELECT
    fleet_id,
    fleet_name,
    today() as report_date,
    
    -- KPIs
    total_vehicles,
    active_vehicles,
    utilization_rate,
    total_revenue,
    profit_margin,
    avg_customer_rating,
    
    -- Trends (compared to previous day)
    utilization_rate - lag(utilization_rate, 1) OVER (PARTITION BY fleet_id ORDER BY date) as utilization_trend,
    total_revenue - lag(total_revenue, 1) OVER (PARTITION BY fleet_id ORDER BY date) as revenue_trend,
    avg_customer_rating - lag(avg_customer_rating, 1) OVER (PARTITION BY fleet_id ORDER BY date) as rating_trend,
    
    -- Performance Indicators
    CASE 
        WHEN utilization_rate >= 80 THEN 'Excellent'
        WHEN utilization_rate >= 60 THEN 'Good'
        WHEN utilization_rate >= 40 THEN 'Fair'
        ELSE 'Poor'
    END as utilization_status,
    
    CASE 
        WHEN profit_margin >= 20 THEN 'Excellent'
        WHEN profit_margin >= 10 THEN 'Good'
        WHEN profit_margin >= 5 THEN 'Fair'
        ELSE 'Poor'
    END as profitability_status
    
FROM fleet_performance_daily
WHERE date >= today() - 30
ORDER BY fleet_id, date DESC;

-- Operational Dashboard View
CREATE VIEW IF NOT EXISTS v_operational_dashboard AS
SELECT
    fleet_id,
    fleet_name,
    date,
    
    -- Operational Metrics
    total_trips,
    completed_trips,
    cancelled_trips,
    completion_rate,
    on_time_performance,
    
    -- Vehicle Status
    active_vehicles,
    maintenance_vehicles,
    offline_vehicles,
    
    -- Efficiency Metrics
    avg_trip_duration_minutes,
    avg_distance_per_trip_km,
    fuel_efficiency_km_per_liter,
    
    -- Alerts
    CASE WHEN maintenance_vehicles / total_vehicles > 0.1 THEN 'High Maintenance Alert' ELSE 'Normal' END as maintenance_alert,
    CASE WHEN completion_rate < 0.9 THEN 'Low Completion Rate Alert' ELSE 'Normal' END as completion_alert,
    CASE WHEN on_time_performance < 0.8 THEN 'Punctuality Alert' ELSE 'Normal' END as punctuality_alert
    
FROM fleet_performance_daily
WHERE date >= today() - 7
ORDER BY fleet_id, date DESC;

-- Financial Dashboard View
CREATE VIEW IF NOT EXISTS v_financial_dashboard AS
SELECT
    fleet_id,
    fleet_name,
    toYYYYMM(date) as month,
    
    -- Revenue Metrics
    sum(total_revenue) as monthly_revenue,
    sum(total_cost) as monthly_cost,
    sum(total_revenue - total_cost) as monthly_profit,
    avg(profit_margin) as avg_profit_margin,
    
    -- Cost Breakdown
    sum(fuel_consumption_liters * 1.5) as fuel_cost, -- Assuming 1.5 AED per liter
    sum(maintenance_cost) as maintenance_cost,
    sum(driver_cost) as driver_cost,
    sum(insurance_cost) as insurance_cost,
    
    -- Efficiency Metrics
    sum(total_revenue) / sum(total_distance_km) as revenue_per_km,
    sum(total_cost) / sum(total_distance_km) as cost_per_km,
    sum(total_revenue) / sum(total_trips) as revenue_per_trip,
    
    -- Growth Metrics
    (sum(total_revenue) - lag(sum(total_revenue), 1) OVER (PARTITION BY fleet_id ORDER BY month)) / lag(sum(total_revenue), 1) OVER (PARTITION BY fleet_id ORDER BY month) * 100 as revenue_growth_percent
    
FROM fleet_performance_daily
WHERE date >= today() - 365
GROUP BY fleet_id, fleet_name, month
ORDER BY fleet_id, month DESC;

-- =====================================================
-- DATA QUALITY CHECKS
-- =====================================================

-- Data Quality Monitoring View
CREATE VIEW IF NOT EXISTS v_data_quality_checks AS
SELECT
    'fleet_performance_hourly' as table_name,
    count() as total_records,
    countIf(total_vehicles = 0) as zero_vehicle_records,
    countIf(utilization_rate < 0 OR utilization_rate > 100) as invalid_utilization_records,
    countIf(total_revenue < 0) as negative_revenue_records,
    countIf(date_hour > now()) as future_date_records,
    max(updated_at) as last_update,
    now() as check_timestamp
FROM fleet_performance_hourly
WHERE date_hour >= today() - 1

UNION ALL

SELECT
    'vehicle_performance_daily' as table_name,
    count() as total_records,
    countIf(total_trips = 0 AND total_distance_km > 0) as inconsistent_trip_records,
    countIf(health_score < 0 OR health_score > 100) as invalid_health_scores,
    countIf(avg_speed_kmh > 200) as unrealistic_speed_records,
    countIf(date > today()) as future_date_records,
    max(updated_at) as last_update,
    now() as check_timestamp
FROM vehicle_performance_daily
WHERE date >= today() - 1;

-- =====================================================
-- INDEXES FOR PERFORMANCE
-- =====================================================

-- Additional indexes for common query patterns
-- Note: ClickHouse uses different indexing approach, these are for reference

-- Index for fleet performance queries by date range
-- CREATE INDEX IF NOT EXISTS idx_fleet_perf_date_fleet ON fleet_performance_daily (date, fleet_id);

-- Index for vehicle performance queries
-- CREATE INDEX IF NOT EXISTS idx_vehicle_perf_vehicle_date ON vehicle_performance_daily (vehicle_id, date);

-- Index for driver performance queries
-- CREATE INDEX IF NOT EXISTS idx_driver_perf_driver_date ON driver_performance_daily (driver_id, date);
