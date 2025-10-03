-- AtlasMesh Fleet Manager - Stored Procedures and Functions
-- Migration 002: Business logic functions for fleet analytics and operations
-- Abu Dhabi autonomous vehicle fleet management

-- Set search path
SET search_path TO fleet_core, public;

-- ============================================================================
-- FLEET ANALYTICS FUNCTIONS
-- ============================================================================

-- Calculate fleet utilization metrics
CREATE OR REPLACE FUNCTION calculate_fleet_utilization(
    p_fleet_id UUID,
    p_start_date TIMESTAMP WITH TIME ZONE DEFAULT NOW() - INTERVAL '24 hours',
    p_end_date TIMESTAMP WITH TIME ZONE DEFAULT NOW()
)
RETURNS TABLE (
    fleet_id UUID,
    total_vehicles INTEGER,
    active_vehicles INTEGER,
    utilization_percentage DECIMAL(5,2),
    avg_trip_duration_minutes DECIMAL(8,2),
    total_distance_km DECIMAL(10,2),
    avg_speed_kmh DECIMAL(6,2),
    efficiency_score DECIMAL(3,2)
) AS $$
BEGIN
    RETURN QUERY
    WITH fleet_stats AS (
        SELECT 
            v.fleet_id,
            COUNT(v.vehicle_id) as total_vehicles,
            COUNT(CASE WHEN v.operational_status IN ('driving_av', 'driving_manual') THEN 1 END) as active_vehicles,
            AVG(CASE WHEN v.operational_status IN ('driving_av', 'driving_manual') THEN 1.0 ELSE 0.0 END) * 100 as utilization_pct,
            AVG(v.current_speed) as avg_speed,
            SUM(v.odometer_km) as total_distance
        FROM vehicles v
        WHERE v.fleet_id = p_fleet_id
        AND v.last_seen BETWEEN p_start_date AND p_end_date
        GROUP BY v.fleet_id
    ),
    trip_stats AS (
        SELECT 
            AVG(EXTRACT(EPOCH FROM (completed_at - started_at))/60) as avg_duration_minutes
        FROM trips t
        WHERE t.fleet_id = p_fleet_id
        AND t.started_at BETWEEN p_start_date AND p_end_date
        AND t.status = 'completed'
    )
    SELECT 
        fs.fleet_id,
        fs.total_vehicles::INTEGER,
        fs.active_vehicles::INTEGER,
        fs.utilization_pct::DECIMAL(5,2),
        COALESCE(ts.avg_duration_minutes, 0)::DECIMAL(8,2),
        COALESCE(fs.total_distance, 0)::DECIMAL(10,2),
        COALESCE(fs.avg_speed, 0)::DECIMAL(6,2),
        CASE 
            WHEN fs.utilization_pct > 80 AND fs.avg_speed > 30 THEN 1.0
            WHEN fs.utilization_pct > 60 AND fs.avg_speed > 20 THEN 0.8
            WHEN fs.utilization_pct > 40 THEN 0.6
            ELSE 0.4
        END::DECIMAL(3,2) as efficiency_score
    FROM fleet_stats fs
    CROSS JOIN trip_stats ts;
END;
$$ LANGUAGE plpgsql;

-- Calculate vehicle health score with Abu Dhabi environmental factors
CREATE OR REPLACE FUNCTION calculate_vehicle_health_score(
    p_vehicle_id UUID,
    p_include_environmental BOOLEAN DEFAULT TRUE
)
RETURNS TABLE (
    vehicle_id UUID,
    health_score DECIMAL(3,2),
    battery_health DECIMAL(3,2),
    mechanical_health DECIMAL(3,2),
    sensor_health DECIMAL(3,2),
    environmental_impact DECIMAL(3,2),
    maintenance_urgency VARCHAR(20),
    recommendations TEXT[]
) AS $$
DECLARE
    v_battery_level DECIMAL(5,2);
    v_odometer_km DECIMAL(10,2);
    v_engine_hours DECIMAL(10,2);
    v_last_maintenance DATE;
    v_sensor_status JSONB;
    v_current_temp DECIMAL(5,2);
    v_location GEOMETRY;
    
    battery_score DECIMAL(3,2);
    mechanical_score DECIMAL(3,2);
    sensor_score DECIMAL(3,2);
    environmental_score DECIMAL(3,2);
    overall_score DECIMAL(3,2);
    urgency VARCHAR(20);
    recs TEXT[];
BEGIN
    -- Get vehicle data
    SELECT 
        battery_level, odometer_km, engine_hours, last_maintenance_date,
        sensor_configuration, current_location
    INTO 
        v_battery_level, v_odometer_km, v_engine_hours, v_last_maintenance,
        v_sensor_status, v_location
    FROM vehicles 
    WHERE vehicle_id = p_vehicle_id;
    
    IF NOT FOUND THEN
        RAISE EXCEPTION 'Vehicle not found: %', p_vehicle_id;
    END IF;
    
    -- Calculate battery health (considering Abu Dhabi heat impact)
    battery_score := CASE 
        WHEN v_battery_level >= 80 THEN 1.0
        WHEN v_battery_level >= 60 THEN 0.8
        WHEN v_battery_level >= 40 THEN 0.6
        WHEN v_battery_level >= 20 THEN 0.4
        ELSE 0.2
    END;
    
    -- Adjust for extreme heat (Abu Dhabi summer impact)
    IF p_include_environmental THEN
        -- Simulate temperature impact (in production, get from weather service)
        v_current_temp := 45.0; -- Typical Abu Dhabi summer temperature
        IF v_current_temp > 45 THEN
            battery_score := battery_score * 0.9; -- 10% reduction in extreme heat
        ELSIF v_current_temp > 40 THEN
            battery_score := battery_score * 0.95; -- 5% reduction in high heat
        END IF;
    END IF;
    
    -- Calculate mechanical health
    mechanical_score := CASE 
        WHEN v_odometer_km < 50000 THEN 1.0
        WHEN v_odometer_km < 100000 THEN 0.9
        WHEN v_odometer_km < 200000 THEN 0.8
        WHEN v_odometer_km < 300000 THEN 0.6
        ELSE 0.4
    END;
    
    -- Maintenance overdue penalty
    IF v_last_maintenance IS NOT NULL AND v_last_maintenance < CURRENT_DATE - INTERVAL '90 days' THEN
        mechanical_score := mechanical_score * 0.7;
        recs := array_append(recs, 'Maintenance overdue - schedule immediately');
    ELSIF v_last_maintenance IS NOT NULL AND v_last_maintenance < CURRENT_DATE - INTERVAL '60 days' THEN
        mechanical_score := mechanical_score * 0.85;
        recs := array_append(recs, 'Maintenance due soon');
    END IF;
    
    -- Calculate sensor health (simplified)
    sensor_score := 0.9; -- Default good sensor health
    
    -- Environmental impact score (Abu Dhabi specific)
    environmental_score := 1.0;
    IF p_include_environmental THEN
        -- Dust and sand impact
        environmental_score := 0.85; -- Typical reduction due to desert conditions
        recs := array_append(recs, 'Regular cleaning required due to desert environment');
        
        -- Add sandstorm impact if applicable
        -- In production, this would check weather alerts
        IF EXTRACT(MONTH FROM CURRENT_DATE) IN (3, 4, 5) THEN -- Sandstorm season
            environmental_score := environmental_score * 0.9;
            recs := array_append(recs, 'Increased maintenance during sandstorm season');
        END IF;
    END IF;
    
    -- Calculate overall health score
    overall_score := (battery_score * 0.3 + mechanical_score * 0.4 + sensor_score * 0.2 + environmental_score * 0.1);
    
    -- Determine maintenance urgency
    urgency := CASE 
        WHEN overall_score >= 0.8 THEN 'low'
        WHEN overall_score >= 0.6 THEN 'medium'
        WHEN overall_score >= 0.4 THEN 'high'
        ELSE 'critical'
    END;
    
    -- Add specific recommendations
    IF battery_score < 0.6 THEN
        recs := array_append(recs, 'Battery health declining - consider replacement');
    END IF;
    
    IF mechanical_score < 0.7 THEN
        recs := array_append(recs, 'Mechanical systems require attention');
    END IF;
    
    IF v_odometer_km > 250000 THEN
        recs := array_append(recs, 'High mileage vehicle - increase inspection frequency');
    END IF;
    
    -- Return results
    RETURN QUERY SELECT 
        p_vehicle_id,
        overall_score,
        battery_score,
        mechanical_score,
        sensor_score,
        environmental_score,
        urgency,
        COALESCE(recs, ARRAY[]::TEXT[]);
END;
$$ LANGUAGE plpgsql;

-- ============================================================================
-- ROUTE OPTIMIZATION FUNCTIONS
-- ============================================================================

-- Calculate optimal route considering Abu Dhabi traffic patterns
CREATE OR REPLACE FUNCTION calculate_optimal_route(
    p_start_lat DECIMAL(10,8),
    p_start_lng DECIMAL(11,8),
    p_end_lat DECIMAL(10,8),
    p_end_lng DECIMAL(11,8),
    p_vehicle_type VARCHAR(50) DEFAULT 'passenger',
    p_time_of_day INTEGER DEFAULT EXTRACT(HOUR FROM NOW()),
    p_avoid_sandstorms BOOLEAN DEFAULT TRUE
)
RETURNS TABLE (
    route_id UUID,
    estimated_duration_minutes INTEGER,
    estimated_distance_km DECIMAL(8,2),
    fuel_consumption_liters DECIMAL(6,2),
    route_efficiency_score DECIMAL(3,2),
    traffic_impact_factor DECIMAL(3,2),
    weather_impact_factor DECIMAL(3,2),
    recommended_departure_time TIMESTAMP WITH TIME ZONE,
    waypoints JSONB,
    warnings TEXT[]
) AS $$
DECLARE
    base_distance_km DECIMAL(8,2);
    base_duration_minutes INTEGER;
    traffic_factor DECIMAL(3,2);
    weather_factor DECIMAL(3,2);
    efficiency_score DECIMAL(3,2);
    fuel_consumption DECIMAL(6,2);
    route_warnings TEXT[];
    optimal_departure TIMESTAMP WITH TIME ZONE;
    route_waypoints JSONB;
BEGIN
    -- Calculate base distance (simplified Haversine formula)
    base_distance_km := 6371 * acos(
        cos(radians(p_start_lat)) * cos(radians(p_end_lat)) * 
        cos(radians(p_end_lng) - radians(p_start_lng)) + 
        sin(radians(p_start_lat)) * sin(radians(p_end_lat))
    );
    
    -- Base duration (assuming average speed of 50 km/h in Abu Dhabi)
    base_duration_minutes := (base_distance_km / 50.0 * 60)::INTEGER;
    
    -- Calculate traffic impact factor based on Abu Dhabi traffic patterns
    traffic_factor := CASE 
        WHEN p_time_of_day BETWEEN 7 AND 9 THEN 1.5  -- Morning rush
        WHEN p_time_of_day BETWEEN 17 AND 19 THEN 1.6 -- Evening rush (worse)
        WHEN p_time_of_day BETWEEN 12 AND 14 THEN 1.2 -- Lunch time
        WHEN p_time_of_day BETWEEN 22 AND 6 THEN 0.8  -- Night time
        ELSE 1.0 -- Normal traffic
    END;
    
    -- Adjust for Friday (weekend in UAE)
    IF EXTRACT(DOW FROM NOW()) = 5 THEN -- Friday
        traffic_factor := traffic_factor * 0.7; -- Less traffic on Friday
    END IF;
    
    -- Calculate weather impact factor (Abu Dhabi specific)
    weather_factor := 1.0;
    
    -- Sandstorm impact (simplified - in production, check weather API)
    IF p_avoid_sandstorms AND EXTRACT(MONTH FROM NOW()) IN (3, 4, 5) THEN
        weather_factor := 1.3; -- 30% longer during sandstorm season
        route_warnings := array_append(route_warnings, 'Sandstorm season - expect delays');
    END IF;
    
    -- Extreme heat impact (summer months)
    IF EXTRACT(MONTH FROM NOW()) BETWEEN 6 AND 9 THEN
        weather_factor := weather_factor * 1.1; -- 10% longer due to heat
        route_warnings := array_append(route_warnings, 'Extreme heat - ensure vehicle cooling');
    END IF;
    
    -- Calculate final duration with all factors
    base_duration_minutes := (base_duration_minutes * traffic_factor * weather_factor)::INTEGER;
    
    -- Calculate fuel consumption based on vehicle type and conditions
    fuel_consumption := CASE p_vehicle_type
        WHEN 'passenger' THEN base_distance_km * 0.08 -- 8L/100km
        WHEN 'cargo' THEN base_distance_km * 0.12     -- 12L/100km
        WHEN 'bus' THEN base_distance_km * 0.25       -- 25L/100km
        ELSE base_distance_km * 0.10                  -- Default 10L/100km
    END;
    
    -- Adjust fuel consumption for traffic and weather
    fuel_consumption := fuel_consumption * (traffic_factor * 0.5 + 0.5) * (weather_factor * 0.3 + 0.7);
    
    -- Calculate efficiency score
    efficiency_score := CASE 
        WHEN traffic_factor <= 1.0 AND weather_factor <= 1.0 THEN 1.0
        WHEN traffic_factor <= 1.2 AND weather_factor <= 1.1 THEN 0.8
        WHEN traffic_factor <= 1.5 AND weather_factor <= 1.3 THEN 0.6
        ELSE 0.4
    END;
    
    -- Calculate optimal departure time (avoid rush hours)
    optimal_departure := CASE 
        WHEN p_time_of_day BETWEEN 7 AND 9 THEN 
            CURRENT_TIMESTAMP + INTERVAL '2 hours' -- Wait until after morning rush
        WHEN p_time_of_day BETWEEN 16 AND 19 THEN 
            CURRENT_TIMESTAMP + INTERVAL '3 hours' -- Wait until after evening rush
        ELSE CURRENT_TIMESTAMP + INTERVAL '15 minutes' -- Leave soon
    END;
    
    -- Generate waypoints (simplified - in production, use actual routing service)
    route_waypoints := jsonb_build_array(
        jsonb_build_object('lat', p_start_lat, 'lng', p_start_lng, 'type', 'start'),
        jsonb_build_object('lat', (p_start_lat + p_end_lat)/2, 'lng', (p_start_lng + p_end_lng)/2, 'type', 'waypoint'),
        jsonb_build_object('lat', p_end_lat, 'lng', p_end_lng, 'type', 'end')
    );
    
    -- Add route-specific warnings
    IF base_distance_km > 100 THEN
        route_warnings := array_append(route_warnings, 'Long distance route - plan for rest stops');
    END IF;
    
    IF traffic_factor > 1.4 THEN
        route_warnings := array_append(route_warnings, 'Heavy traffic expected - consider alternative departure time');
    END IF;
    
    -- Return results
    RETURN QUERY SELECT 
        gen_random_uuid() as route_id,
        base_duration_minutes,
        base_distance_km,
        fuel_consumption,
        efficiency_score,
        traffic_factor,
        weather_factor,
        optimal_departure,
        route_waypoints,
        COALESCE(route_warnings, ARRAY[]::TEXT[]);
END;
$$ LANGUAGE plpgsql;

-- ============================================================================
-- OPERATIONAL ANALYTICS FUNCTIONS
-- ============================================================================

-- Generate fleet performance report
CREATE OR REPLACE FUNCTION generate_fleet_performance_report(
    p_fleet_id UUID,
    p_report_period VARCHAR(20) DEFAULT 'daily', -- daily, weekly, monthly
    p_start_date TIMESTAMP WITH TIME ZONE DEFAULT NOW() - INTERVAL '24 hours'
)
RETURNS TABLE (
    report_id UUID,
    fleet_id UUID,
    report_period VARCHAR(20),
    report_date TIMESTAMP WITH TIME ZONE,
    total_vehicles INTEGER,
    active_vehicles INTEGER,
    utilization_rate DECIMAL(5,2),
    total_trips INTEGER,
    completed_trips INTEGER,
    cancelled_trips INTEGER,
    total_distance_km DECIMAL(10,2),
    total_fuel_consumed_liters DECIMAL(8,2),
    avg_trip_duration_minutes DECIMAL(8,2),
    safety_incidents INTEGER,
    maintenance_alerts INTEGER,
    efficiency_score DECIMAL(3,2),
    cost_per_km DECIMAL(6,2),
    carbon_emissions_kg DECIMAL(8,2),
    performance_trends JSONB,
    recommendations TEXT[]
) AS $$
DECLARE
    report_start_date TIMESTAMP WITH TIME ZONE;
    report_end_date TIMESTAMP WITH TIME ZONE;
    v_total_vehicles INTEGER;
    v_active_vehicles INTEGER;
    v_total_trips INTEGER;
    v_completed_trips INTEGER;
    v_cancelled_trips INTEGER;
    v_total_distance DECIMAL(10,2);
    v_total_fuel DECIMAL(8,2);
    v_avg_duration DECIMAL(8,2);
    v_safety_incidents INTEGER;
    v_maintenance_alerts INTEGER;
    v_efficiency DECIMAL(3,2);
    v_cost_per_km DECIMAL(6,2);
    v_carbon_emissions DECIMAL(8,2);
    v_trends JSONB;
    v_recommendations TEXT[];
BEGIN
    -- Calculate report period dates
    report_start_date := p_start_date;
    report_end_date := CASE p_report_period
        WHEN 'daily' THEN p_start_date + INTERVAL '1 day'
        WHEN 'weekly' THEN p_start_date + INTERVAL '1 week'
        WHEN 'monthly' THEN p_start_date + INTERVAL '1 month'
        ELSE p_start_date + INTERVAL '1 day'
    END;
    
    -- Get vehicle counts
    SELECT 
        COUNT(*),
        COUNT(CASE WHEN operational_status IN ('driving_av', 'driving_manual', 'idle') THEN 1 END)
    INTO v_total_vehicles, v_active_vehicles
    FROM vehicles 
    WHERE fleet_id = p_fleet_id;
    
    -- Get trip statistics
    SELECT 
        COUNT(*),
        COUNT(CASE WHEN status = 'completed' THEN 1 END),
        COUNT(CASE WHEN status = 'cancelled' THEN 1 END),
        COALESCE(SUM(distance_km), 0),
        COALESCE(AVG(EXTRACT(EPOCH FROM (completed_at - started_at))/60), 0)
    INTO v_total_trips, v_completed_trips, v_cancelled_trips, v_total_distance, v_avg_duration
    FROM trips 
    WHERE fleet_id = p_fleet_id 
    AND started_at BETWEEN report_start_date AND report_end_date;
    
    -- Calculate fuel consumption (estimated)
    v_total_fuel := v_total_distance * 0.08; -- 8L/100km average
    
    -- Get safety incidents
    SELECT COUNT(*)
    INTO v_safety_incidents
    FROM vehicle_alerts va
    JOIN vehicles v ON va.vehicle_id = v.vehicle_id
    WHERE v.fleet_id = p_fleet_id
    AND va.category = 'safety'
    AND va.occurred_at BETWEEN report_start_date AND report_end_date;
    
    -- Get maintenance alerts
    SELECT COUNT(*)
    INTO v_maintenance_alerts
    FROM vehicle_alerts va
    JOIN vehicles v ON va.vehicle_id = v.vehicle_id
    WHERE v.fleet_id = p_fleet_id
    AND va.category = 'maintenance'
    AND va.occurred_at BETWEEN report_start_date AND report_end_date;
    
    -- Calculate efficiency score
    v_efficiency := CASE 
        WHEN v_total_vehicles > 0 THEN
            (v_active_vehicles::DECIMAL / v_total_vehicles * 0.4 +
             CASE WHEN v_total_trips > 0 THEN v_completed_trips::DECIMAL / v_total_trips ELSE 0 END * 0.4 +
             CASE WHEN v_safety_incidents = 0 THEN 1.0 ELSE GREATEST(0.0, 1.0 - v_safety_incidents * 0.1) END * 0.2)
        ELSE 0
    END;
    
    -- Calculate cost per km (simplified)
    v_cost_per_km := CASE 
        WHEN v_total_distance > 0 THEN
            (v_total_fuel * 1.5 + v_total_vehicles * 50) / v_total_distance -- Fuel + maintenance costs
        ELSE 0
    END;
    
    -- Calculate carbon emissions (kg CO2)
    v_carbon_emissions := v_total_fuel * 2.31; -- 2.31 kg CO2 per liter of fuel
    
    -- Generate performance trends
    v_trends := jsonb_build_object(
        'utilization_trend', CASE WHEN v_total_vehicles > 0 THEN v_active_vehicles::DECIMAL / v_total_vehicles * 100 ELSE 0 END,
        'completion_rate', CASE WHEN v_total_trips > 0 THEN v_completed_trips::DECIMAL / v_total_trips * 100 ELSE 0 END,
        'safety_score', CASE WHEN v_safety_incidents = 0 THEN 100 ELSE GREATEST(0, 100 - v_safety_incidents * 10) END,
        'fuel_efficiency', CASE WHEN v_total_distance > 0 THEN v_total_fuel / v_total_distance * 100 ELSE 0 END
    );
    
    -- Generate recommendations
    v_recommendations := ARRAY[]::TEXT[];
    
    IF v_active_vehicles::DECIMAL / NULLIF(v_total_vehicles, 0) < 0.7 THEN
        v_recommendations := array_append(v_recommendations, 'Low vehicle utilization - consider fleet optimization');
    END IF;
    
    IF v_safety_incidents > 0 THEN
        v_recommendations := array_append(v_recommendations, 'Safety incidents detected - review safety protocols');
    END IF;
    
    IF v_maintenance_alerts > v_total_vehicles * 0.2 THEN
        v_recommendations := array_append(v_recommendations, 'High maintenance alerts - schedule preventive maintenance');
    END IF;
    
    IF v_total_fuel / NULLIF(v_total_distance, 0) > 0.10 THEN
        v_recommendations := array_append(v_recommendations, 'High fuel consumption - review driving patterns and routes');
    END IF;
    
    -- Abu Dhabi specific recommendations
    IF EXTRACT(MONTH FROM NOW()) BETWEEN 6 AND 9 THEN
        v_recommendations := array_append(v_recommendations, 'Summer operations - ensure adequate cooling and hydration');
    END IF;
    
    IF EXTRACT(MONTH FROM NOW()) IN (3, 4, 5) THEN
        v_recommendations := array_append(v_recommendations, 'Sandstorm season - increase vehicle cleaning frequency');
    END IF;
    
    -- Return results
    RETURN QUERY SELECT 
        gen_random_uuid() as report_id,
        p_fleet_id,
        p_report_period,
        NOW(),
        v_total_vehicles,
        v_active_vehicles,
        CASE WHEN v_total_vehicles > 0 THEN v_active_vehicles::DECIMAL / v_total_vehicles * 100 ELSE 0 END,
        v_total_trips,
        v_completed_trips,
        v_cancelled_trips,
        v_total_distance,
        v_total_fuel,
        v_avg_duration,
        v_safety_incidents,
        v_maintenance_alerts,
        v_efficiency,
        v_cost_per_km,
        v_carbon_emissions,
        v_trends,
        v_recommendations;
END;
$$ LANGUAGE plpgsql;

-- ============================================================================
-- PREDICTIVE MAINTENANCE FUNCTIONS
-- ============================================================================

-- Predict maintenance needs based on vehicle data and Abu Dhabi conditions
CREATE OR REPLACE FUNCTION predict_maintenance_needs(
    p_vehicle_id UUID DEFAULT NULL,
    p_fleet_id UUID DEFAULT NULL,
    p_prediction_horizon_days INTEGER DEFAULT 30
)
RETURNS TABLE (
    vehicle_id UUID,
    asset_tag VARCHAR(100),
    maintenance_type VARCHAR(100),
    predicted_date DATE,
    confidence_score DECIMAL(3,2),
    urgency_level VARCHAR(20),
    estimated_cost DECIMAL(10,2),
    recommended_actions TEXT[],
    environmental_factors JSONB
) AS $$
DECLARE
    v_record RECORD;
    maintenance_date DATE;
    confidence DECIMAL(3,2);
    urgency VARCHAR(20);
    cost DECIMAL(10,2);
    actions TEXT[];
    env_factors JSONB;
BEGIN
    -- Loop through vehicles
    FOR v_record IN 
        SELECT v.vehicle_id, v.asset_tag, v.odometer_km, v.engine_hours, 
               v.last_maintenance_date, v.battery_level, v.health_score,
               v.manufacturer, v.model, v.year
        FROM vehicles v
        WHERE (p_vehicle_id IS NULL OR v.vehicle_id = p_vehicle_id)
        AND (p_fleet_id IS NULL OR v.fleet_id = p_fleet_id)
        AND v.operational_status != 'decommissioned'
    LOOP
        -- Predict battery maintenance (critical in Abu Dhabi heat)
        IF v_record.battery_level < 70 OR 
           (v_record.last_maintenance_date IS NOT NULL AND 
            v_record.last_maintenance_date < CURRENT_DATE - INTERVAL '60 days') THEN
            
            maintenance_date := CURRENT_DATE + 
                CASE 
                    WHEN v_record.battery_level < 50 THEN INTERVAL '7 days'
                    WHEN v_record.battery_level < 60 THEN INTERVAL '14 days'
                    ELSE INTERVAL '21 days'
                END;
            
            confidence := CASE 
                WHEN v_record.battery_level < 50 THEN 0.95
                WHEN v_record.battery_level < 60 THEN 0.85
                ELSE 0.75
            END;
            
            urgency := CASE 
                WHEN v_record.battery_level < 50 THEN 'critical'
                WHEN v_record.battery_level < 60 THEN 'high'
                ELSE 'medium'
            END;
            
            cost := CASE 
                WHEN urgency = 'critical' THEN 5000.00
                WHEN urgency = 'high' THEN 3000.00
                ELSE 1500.00
            END;
            
            actions := ARRAY[
                'Battery health assessment',
                'Cooling system inspection',
                'Charging system calibration'
            ];
            
            -- Abu Dhabi environmental factors
            env_factors := jsonb_build_object(
                'heat_impact', 'high',
                'dust_exposure', 'high',
                'recommended_season', 'winter_months',
                'cooling_priority', true
            );
            
            RETURN QUERY SELECT 
                v_record.vehicle_id,
                v_record.asset_tag,
                'battery_maintenance'::VARCHAR(100),
                maintenance_date,
                confidence,
                urgency,
                cost,
                actions,
                env_factors;
        END IF;
        
        -- Predict general maintenance based on mileage
        IF v_record.odometer_km > 0 THEN
            -- Calculate next service based on mileage (every 10,000 km)
            IF v_record.odometer_km % 10000 > 8000 OR 
               (v_record.last_maintenance_date IS NOT NULL AND 
                v_record.last_maintenance_date < CURRENT_DATE - INTERVAL '90 days') THEN
                
                maintenance_date := CURRENT_DATE + INTERVAL '14 days';
                confidence := 0.80;
                urgency := 'medium';
                cost := 2000.00;
                
                actions := ARRAY[
                    'Oil and filter change',
                    'Brake system inspection',
                    'Air filter replacement (desert conditions)',
                    'Cooling system check'
                ];
                
                env_factors := jsonb_build_object(
                    'dust_impact', 'high',
                    'filter_replacement_frequency', 'increased',
                    'cooling_system_priority', true
                );
                
                RETURN QUERY SELECT 
                    v_record.vehicle_id,
                    v_record.asset_tag,
                    'general_maintenance'::VARCHAR(100),
                    maintenance_date,
                    confidence,
                    urgency,
                    cost,
                    actions,
                    env_factors;
            END IF;
        END IF;
        
        -- Predict tire maintenance (important in desert conditions)
        IF v_record.odometer_km > 40000 THEN
            maintenance_date := CURRENT_DATE + INTERVAL '21 days';
            confidence := 0.70;
            urgency := 'medium';
            cost := 1200.00;
            
            actions := ARRAY[
                'Tire pressure check',
                'Tread depth measurement',
                'Wheel alignment',
                'Sand damage inspection'
            ];
            
            env_factors := jsonb_build_object(
                'sand_abrasion', 'high',
                'heat_expansion', 'significant',
                'pressure_monitoring', 'critical'
            );
            
            RETURN QUERY SELECT 
                v_record.vehicle_id,
                v_record.asset_tag,
                'tire_maintenance'::VARCHAR(100),
                maintenance_date,
                confidence,
                urgency,
                cost,
                actions,
                env_factors;
        END IF;
        
        -- Predict air conditioning maintenance (critical in Abu Dhabi)
        IF EXTRACT(MONTH FROM CURRENT_DATE) IN (4, 5) THEN -- Pre-summer maintenance
            maintenance_date := CURRENT_DATE + INTERVAL '10 days';
            confidence := 0.90;
            urgency := 'high';
            cost := 800.00;
            
            actions := ARRAY[
                'AC system inspection',
                'Refrigerant level check',
                'Cabin filter replacement',
                'Condenser cleaning'
            ];
            
            env_factors := jsonb_build_object(
                'summer_preparation', true,
                'temperature_extremes', 'expected',
                'ac_load', 'maximum'
            );
            
            RETURN QUERY SELECT 
                v_record.vehicle_id,
                v_record.asset_tag,
                'ac_maintenance'::VARCHAR(100),
                maintenance_date,
                confidence,
                urgency,
                cost,
                actions,
                env_factors;
        END IF;
    END LOOP;
END;
$$ LANGUAGE plpgsql;

-- ============================================================================
-- COMPLIANCE AND AUDIT FUNCTIONS
-- ============================================================================

-- Generate UAE compliance report
CREATE OR REPLACE FUNCTION generate_uae_compliance_report(
    p_organization_id UUID,
    p_report_date DATE DEFAULT CURRENT_DATE
)
RETURNS TABLE (
    report_id UUID,
    organization_id UUID,
    report_date DATE,
    compliance_score DECIMAL(5,2),
    total_vehicles INTEGER,
    compliant_vehicles INTEGER,
    violations_count INTEGER,
    critical_violations INTEGER,
    regulatory_requirements JSONB,
    compliance_status VARCHAR(20),
    recommendations TEXT[],
    next_audit_date DATE
) AS $$
DECLARE
    v_total_vehicles INTEGER;
    v_compliant_vehicles INTEGER;
    v_violations INTEGER;
    v_critical_violations INTEGER;
    v_compliance_score DECIMAL(5,2);
    v_status VARCHAR(20);
    v_recommendations TEXT[];
    v_requirements JSONB;
BEGIN
    -- Get vehicle counts
    SELECT COUNT(*)
    INTO v_total_vehicles
    FROM vehicles v
    JOIN fleets f ON v.fleet_id = f.fleet_id
    WHERE f.organization_id = p_organization_id;
    
    -- Count compliant vehicles (simplified criteria)
    SELECT COUNT(*)
    INTO v_compliant_vehicles
    FROM vehicles v
    JOIN fleets f ON v.fleet_id = f.fleet_id
    WHERE f.organization_id = p_organization_id
    AND v.autonomy_level IN ('L0', 'L1', 'L2', 'L3', 'L4') -- L5 not approved in UAE
    AND v.last_maintenance_date >= CURRENT_DATE - INTERVAL '90 days'
    AND v.health_score >= 0.7;
    
    -- Count violations
    SELECT 
        COUNT(*),
        COUNT(CASE WHEN severity IN ('critical', 'high') THEN 1 END)
    INTO v_violations, v_critical_violations
    FROM vehicle_alerts va
    JOIN vehicles v ON va.vehicle_id = v.vehicle_id
    JOIN fleets f ON v.fleet_id = f.fleet_id
    WHERE f.organization_id = p_organization_id
    AND va.category = 'compliance'
    AND va.occurred_at >= p_report_date - INTERVAL '30 days'
    AND va.status = 'active';
    
    -- Calculate compliance score
    v_compliance_score := CASE 
        WHEN v_total_vehicles = 0 THEN 0
        ELSE (v_compliant_vehicles::DECIMAL / v_total_vehicles * 70 +
              CASE WHEN v_violations = 0 THEN 30 ELSE GREATEST(0, 30 - v_violations * 5) END)
    END;
    
    -- Determine status
    v_status := CASE 
        WHEN v_compliance_score >= 95 THEN 'excellent'
        WHEN v_compliance_score >= 85 THEN 'good'
        WHEN v_compliance_score >= 70 THEN 'acceptable'
        ELSE 'needs_improvement'
    END;
    
    -- Generate recommendations
    v_recommendations := ARRAY[]::TEXT[];
    
    IF v_critical_violations > 0 THEN
        v_recommendations := array_append(v_recommendations, 
            'Address critical compliance violations immediately');
    END IF;
    
    IF v_compliant_vehicles::DECIMAL / NULLIF(v_total_vehicles, 0) < 0.9 THEN
        v_recommendations := array_append(v_recommendations, 
            'Increase vehicle maintenance frequency to meet UAE standards');
    END IF;
    
    IF v_violations > v_total_vehicles * 0.1 THEN
        v_recommendations := array_append(v_recommendations, 
            'Review operational procedures to reduce compliance violations');
    END IF;
    
    -- UAE regulatory requirements
    v_requirements := jsonb_build_object(
        'uae_av_regulations', jsonb_build_object(
            'autonomy_levels_approved', ARRAY['L0', 'L1', 'L2', 'L3', 'L4'],
            'safety_driver_required', true,
            'insurance_mandatory', true,
            'regular_inspections', true
        ),
        'adta_guidelines', jsonb_build_object(
            'data_residency', 'uae_only',
            'privacy_compliance', 'gdpr_equivalent',
            'cybersecurity_standards', 'iso_27001'
        ),
        'environmental_standards', jsonb_build_object(
            'emissions_limits', 'euro_6',
            'noise_limits', '70_db',
            'waste_management', 'mandatory'
        )
    );
    
    -- Return results
    RETURN QUERY SELECT 
        gen_random_uuid() as report_id,
        p_organization_id,
        p_report_date,
        v_compliance_score,
        v_total_vehicles,
        v_compliant_vehicles,
        v_violations,
        v_critical_violations,
        v_requirements,
        v_status,
        v_recommendations,
        p_report_date + INTERVAL '90 days' as next_audit_date;
END;
$$ LANGUAGE plpgsql;

-- ============================================================================
-- UTILITY FUNCTIONS
-- ============================================================================

-- Update vehicle health scores for all vehicles
CREATE OR REPLACE FUNCTION update_all_vehicle_health_scores()
RETURNS INTEGER AS $$
DECLARE
    v_count INTEGER := 0;
    v_vehicle RECORD;
    v_health_score DECIMAL(3,2);
BEGIN
    FOR v_vehicle IN SELECT vehicle_id FROM vehicles WHERE operational_status != 'decommissioned' LOOP
        SELECT health_score 
        INTO v_health_score
        FROM calculate_vehicle_health_score(v_vehicle.vehicle_id, true);
        
        UPDATE vehicles 
        SET health_score = v_health_score, updated_at = NOW()
        WHERE vehicle_id = v_vehicle.vehicle_id;
        
        v_count := v_count + 1;
    END LOOP;
    
    RETURN v_count;
END;
$$ LANGUAGE plpgsql;

-- Create indexes for performance
CREATE INDEX IF NOT EXISTS idx_vehicles_health_score ON vehicles(health_score DESC);
CREATE INDEX IF NOT EXISTS idx_vehicles_last_maintenance ON vehicles(last_maintenance_date);
CREATE INDEX IF NOT EXISTS idx_vehicle_alerts_category_severity ON vehicle_alerts(category, severity);
CREATE INDEX IF NOT EXISTS idx_trips_fleet_started_at ON trips(fleet_id, started_at);

-- Grant permissions
GRANT EXECUTE ON ALL FUNCTIONS IN SCHEMA fleet_core TO fleet_manager_service;
GRANT EXECUTE ON ALL FUNCTIONS IN SCHEMA fleet_core TO policy_engine_service;

-- Comments
COMMENT ON FUNCTION calculate_fleet_utilization IS 'Calculate comprehensive fleet utilization metrics with Abu Dhabi operational considerations';
COMMENT ON FUNCTION calculate_vehicle_health_score IS 'Calculate vehicle health score including Abu Dhabi environmental impact factors';
COMMENT ON FUNCTION calculate_optimal_route IS 'Calculate optimal routes considering Abu Dhabi traffic patterns and weather conditions';
COMMENT ON FUNCTION generate_fleet_performance_report IS 'Generate comprehensive fleet performance reports with UAE-specific metrics';
COMMENT ON FUNCTION predict_maintenance_needs IS 'Predict maintenance needs based on vehicle data and Abu Dhabi environmental conditions';
COMMENT ON FUNCTION generate_uae_compliance_report IS 'Generate UAE regulatory compliance reports for autonomous vehicle operations';
