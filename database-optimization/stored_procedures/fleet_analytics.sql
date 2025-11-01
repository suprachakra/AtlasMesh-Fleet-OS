-- AtlasMesh Fleet Manager - Core Analytics Stored Procedures
-- Fleet analytics, vehicle health scoring, route optimization, and predictive maintenance
-- Abu Dhabi Autonomous Vehicle Operations

-- Enable required extensions
CREATE EXTENSION IF NOT EXISTS "plpgsql";
CREATE EXTENSION IF NOT EXISTS "btree_gin";

-- Set search path
SET search_path TO fleet_core, public;

-- =====================================================
-- VEHICLE HEALTH SCORING PROCEDURES
-- =====================================================

-- Calculate comprehensive vehicle health score
CREATE OR REPLACE FUNCTION calculate_vehicle_health_score(
    p_vehicle_id UUID,
    p_assessment_time TIMESTAMP WITH TIME ZONE DEFAULT NOW()
) RETURNS JSONB AS $$
DECLARE
    v_vehicle_data RECORD;
    v_health_score NUMERIC := 100.0;
    v_health_breakdown JSONB := '{}';
    v_alerts TEXT[] := ARRAY[]::TEXT[];
    v_recommendations TEXT[] := ARRAY[]::TEXT[];
BEGIN
    -- Get vehicle data
    SELECT v.*, 
           EXTRACT(EPOCH FROM (p_assessment_time - v.last_seen)) / 60 as minutes_since_seen
    INTO v_vehicle_data
    FROM vehicles v 
    WHERE v.vehicle_id = p_vehicle_id;
    
    IF NOT FOUND THEN
        RAISE EXCEPTION 'Vehicle not found: %', p_vehicle_id;
    END IF;
    
    -- Battery/Fuel Health Assessment (30% weight)
    DECLARE
        v_power_score NUMERIC := 100.0;
        v_power_status TEXT := 'optimal';
    BEGIN
        IF v_vehicle_data.battery_level IS NOT NULL AND v_vehicle_data.battery_level > 0 THEN
            CASE 
                WHEN v_vehicle_data.battery_level < 15 THEN
                    v_power_score := 20.0;
                    v_power_status := 'critical';
                    v_alerts := array_append(v_alerts, 'Critical battery level');
                    v_recommendations := array_append(v_recommendations, 'Schedule immediate charging');
                WHEN v_vehicle_data.battery_level < 30 THEN
                    v_power_score := 60.0;
                    v_power_status := 'low';
                    v_alerts := array_append(v_alerts, 'Low battery level');
                    v_recommendations := array_append(v_recommendations, 'Plan charging session');
                WHEN v_vehicle_data.battery_level < 50 THEN
                    v_power_score := 80.0;
                    v_power_status := 'moderate';
                ELSE
                    v_power_score := 100.0;
                    v_power_status := 'optimal';
            END CASE;
        END IF;
        
        v_health_breakdown := jsonb_set(v_health_breakdown, '{power_system}', 
            jsonb_build_object(
                'score', v_power_score,
                'status', v_power_status,
                'battery_level', v_vehicle_data.battery_level,
                'fuel_level', v_vehicle_data.fuel_level
            )
        );
        
        v_health_score := v_health_score - (30.0 * (100.0 - v_power_score) / 100.0);
    END;
    
    -- Connectivity Health Assessment (25% weight)
    DECLARE
        v_connectivity_score NUMERIC := 100.0;
        v_connectivity_status TEXT := 'excellent';
    BEGIN
        CASE 
            WHEN v_vehicle_data.minutes_since_seen > 10 THEN
                v_connectivity_score := 0.0;
                v_connectivity_status := 'offline';
                v_alerts := array_append(v_alerts, 'Vehicle offline - no communication');
            WHEN v_vehicle_data.minutes_since_seen > 5 THEN
                v_connectivity_score := 30.0;
                v_connectivity_status := 'poor';
                v_alerts := array_append(v_alerts, 'Poor connectivity detected');
            WHEN v_vehicle_data.minutes_since_seen > 2 THEN
                v_connectivity_score := 70.0;
                v_connectivity_status := 'degraded';
            ELSE
                v_connectivity_score := 100.0;
                v_connectivity_status := 'excellent';
        END CASE;
        
        v_health_breakdown := jsonb_set(v_health_breakdown, '{connectivity}', 
            jsonb_build_object(
                'score', v_connectivity_score,
                'status', v_connectivity_status,
                'last_seen', v_vehicle_data.last_seen,
                'minutes_since_seen', v_vehicle_data.minutes_since_seen
            )
        );
        
        v_health_score := v_health_score - (25.0 * (100.0 - v_connectivity_score) / 100.0);
    END;
    
    -- Operational Status Assessment (25% weight)
    DECLARE
        v_operational_score NUMERIC := 100.0;
        v_operational_status TEXT := 'optimal';
    BEGIN
        CASE v_vehicle_data.operational_status
            WHEN 'active' THEN
                v_operational_score := 100.0;
                v_operational_status := 'optimal';
            WHEN 'idle' THEN
                v_operational_score := 90.0;
                v_operational_status := 'good';
            WHEN 'maintenance' THEN
                v_operational_score := 50.0;
                v_operational_status := 'maintenance';
                v_alerts := array_append(v_alerts, 'Vehicle in maintenance mode');
            WHEN 'offline' THEN
                v_operational_score := 0.0;
                v_operational_status := 'offline';
                v_alerts := array_append(v_alerts, 'Vehicle offline');
            ELSE
                v_operational_score := 75.0;
                v_operational_status := 'unknown';
        END CASE;
        
        -- Autonomy status impact
        CASE v_vehicle_data.autonomy_status
            WHEN 'fault' THEN
                v_operational_score := v_operational_score * 0.3;
                v_alerts := array_append(v_alerts, 'Autonomy system fault detected');
            WHEN 'degraded' THEN
                v_operational_score := v_operational_score * 0.7;
                v_alerts := array_append(v_alerts, 'Degraded autonomy performance');
            ELSE
                NULL;
        END CASE;
        
        v_health_breakdown := jsonb_set(v_health_breakdown, '{operational}', 
            jsonb_build_object(
                'score', v_operational_score,
                'status', v_operational_status,
                'operational_status', v_vehicle_data.operational_status,
                'autonomy_status', v_vehicle_data.autonomy_status
            )
        );
        
        v_health_score := v_health_score - (25.0 * (100.0 - v_operational_score) / 100.0);
    END;
    
    -- Ensure score doesn't go below 0
    v_health_score := GREATEST(0, v_health_score);
    
    -- Determine overall health status
    DECLARE
        v_overall_status TEXT;
    BEGIN
        CASE 
            WHEN v_health_score >= 90 THEN v_overall_status := 'excellent';
            WHEN v_health_score >= 75 THEN v_overall_status := 'good';
            WHEN v_health_score >= 60 THEN v_overall_status := 'fair';
            WHEN v_health_score >= 40 THEN v_overall_status := 'poor';
            ELSE v_overall_status := 'critical';
        END CASE;
        
        v_health_breakdown := jsonb_set(v_health_breakdown, '{overall}', 
            jsonb_build_object(
                'score', ROUND(v_health_score, 1),
                'status', v_overall_status,
                'assessment_time', p_assessment_time
            )
        );
    END;
    
    -- Add alerts and recommendations
    v_health_breakdown := jsonb_set(v_health_breakdown, '{alerts}', to_jsonb(v_alerts));
    v_health_breakdown := jsonb_set(v_health_breakdown, '{recommendations}', to_jsonb(v_recommendations));
    
    -- Update vehicle health score in database
    UPDATE vehicles 
    SET health_score = ROUND(v_health_score, 1),
        updated_at = p_assessment_time
    WHERE vehicle_id = p_vehicle_id;
    
    RETURN v_health_breakdown;
END;
$$ LANGUAGE plpgsql;

-- =====================================================
-- FLEET ANALYTICS PROCEDURES
-- =====================================================

-- Get comprehensive fleet utilization metrics
CREATE OR REPLACE FUNCTION get_fleet_utilization_analytics(
    p_fleet_id UUID DEFAULT NULL,
    p_organization_id UUID DEFAULT NULL,
    p_start_time TIMESTAMP WITH TIME ZONE DEFAULT NOW() - INTERVAL '24 hours',
    p_end_time TIMESTAMP WITH TIME ZONE DEFAULT NOW()
) RETURNS JSONB AS $$
DECLARE
    v_result JSONB := '{}';
    v_total_vehicles INTEGER := 0;
    v_active_vehicles INTEGER := 0;
    v_dispatched_vehicles INTEGER := 0;
    v_maintenance_vehicles INTEGER := 0;
    v_offline_vehicles INTEGER := 0;
    v_avg_health_score NUMERIC := 0;
    v_total_distance NUMERIC := 0;
    v_utilization_rate NUMERIC := 0;
    v_availability_rate NUMERIC := 0;
BEGIN
    -- Get current fleet statistics
    SELECT 
        COUNT(*) as total,
        COUNT(*) FILTER (WHERE operational_status = 'active') as active,
        COUNT(*) FILTER (WHERE operational_status = 'dispatched') as dispatched,
        COUNT(*) FILTER (WHERE operational_status = 'maintenance') as maintenance,
        COUNT(*) FILTER (WHERE operational_status = 'offline') as offline,
        COALESCE(AVG(health_score), 0) as avg_health,
        COALESCE(SUM(odometer_km), 0) as total_distance
    INTO v_total_vehicles, v_active_vehicles, v_dispatched_vehicles, 
         v_maintenance_vehicles, v_offline_vehicles, v_avg_health_score, v_total_distance
    FROM vehicles 
    WHERE (p_fleet_id IS NULL OR fleet_id = p_fleet_id)
    AND (p_organization_id IS NULL OR organization_id = p_organization_id);
    
    -- Calculate utilization metrics
    IF v_total_vehicles > 0 THEN
        v_utilization_rate := ROUND((v_active_vehicles + v_dispatched_vehicles) * 100.0 / v_total_vehicles, 2);
        v_availability_rate := ROUND((v_total_vehicles - v_offline_vehicles - v_maintenance_vehicles) * 100.0 / v_total_vehicles, 2);
    END IF;
    
    -- Build result JSON
    v_result := jsonb_build_object(
        'fleet_id', p_fleet_id,
        'organization_id', p_organization_id,
        'analysis_period', jsonb_build_object(
            'start_time', p_start_time,
            'end_time', p_end_time,
            'duration_hours', EXTRACT(EPOCH FROM (p_end_time - p_start_time)) / 3600
        ),
        'fleet_overview', jsonb_build_object(
            'total_vehicles', v_total_vehicles,
            'active_vehicles', v_active_vehicles,
            'dispatched_vehicles', v_dispatched_vehicles,
            'maintenance_vehicles', v_maintenance_vehicles,
            'offline_vehicles', v_offline_vehicles
        ),
        'performance_metrics', jsonb_build_object(
            'utilization_rate', v_utilization_rate,
            'availability_rate', v_availability_rate,
            'avg_health_score', ROUND(v_avg_health_score, 1),
            'operational_efficiency', ROUND((v_utilization_rate + v_availability_rate) / 2, 2)
        ),
        'operational_metrics', jsonb_build_object(
            'total_distance_km', ROUND(v_total_distance, 1),
            'avg_distance_per_vehicle', CASE WHEN v_total_vehicles > 0 THEN ROUND(v_total_distance / v_total_vehicles, 1) ELSE 0 END,
            'maintenance_rate', CASE WHEN v_total_vehicles > 0 THEN ROUND(v_maintenance_vehicles * 100.0 / v_total_vehicles, 2) ELSE 0 END,
            'connectivity_rate', CASE WHEN v_total_vehicles > 0 THEN ROUND((v_total_vehicles - v_offline_vehicles) * 100.0 / v_total_vehicles, 2) ELSE 0 END
        ),
        'generated_at', NOW()
    );
    
    RETURN v_result;
END;
$$ LANGUAGE plpgsql;

-- =====================================================
-- ROUTE OPTIMIZATION PROCEDURES
-- =====================================================

-- Calculate optimal route considering multiple constraints
CREATE OR REPLACE FUNCTION calculate_optimal_route(
    p_start_lat DOUBLE PRECISION,
    p_start_lon DOUBLE PRECISION,
    p_end_lat DOUBLE PRECISION,
    p_end_lon DOUBLE PRECISION,
    p_vehicle_id UUID DEFAULT NULL,
    p_optimization_type TEXT DEFAULT 'balanced',
    p_constraints JSONB DEFAULT '{}'::JSONB
) RETURNS JSONB AS $$
DECLARE
    v_result JSONB := '{}';
    v_vehicle_data RECORD;
    v_distance_km NUMERIC;
    v_estimated_duration_minutes NUMERIC;
    v_fuel_consumption NUMERIC;
    v_route_score NUMERIC;
    v_waypoints JSONB := '[]';
    v_constraints_applied TEXT[] := ARRAY[]::TEXT[];
BEGIN
    -- Validate coordinates
    IF p_start_lat < -90 OR p_start_lat > 90 OR p_end_lat < -90 OR p_end_lat > 90 THEN
        RAISE EXCEPTION 'Invalid latitude values';
    END IF;
    
    -- Get vehicle data if specified
    IF p_vehicle_id IS NOT NULL THEN
        SELECT * INTO v_vehicle_data FROM vehicles WHERE vehicle_id = p_vehicle_id;
        IF NOT FOUND THEN
            RAISE EXCEPTION 'Vehicle not found: %', p_vehicle_id;
        END IF;
    END IF;
    
    -- Calculate great circle distance (simplified)
    v_distance_km := ROUND(
        6371 * acos(
            cos(radians(p_start_lat)) * cos(radians(p_end_lat)) * 
            cos(radians(p_end_lon) - radians(p_start_lon)) + 
            sin(radians(p_start_lat)) * sin(radians(p_end_lat))
        ), 2
    );
    
    -- Base time estimation (assuming average speed of 50 km/h in urban areas)
    v_estimated_duration_minutes := ROUND(v_distance_km * 60 / 50, 0);
    
    -- Apply optimization adjustments
    CASE p_optimization_type
        WHEN 'fastest' THEN
            v_estimated_duration_minutes := v_estimated_duration_minutes * 0.8;
            v_constraints_applied := array_append(v_constraints_applied, 'highway_preference');
        WHEN 'shortest' THEN
            v_estimated_duration_minutes := v_estimated_duration_minutes * 1.1;
            v_constraints_applied := array_append(v_constraints_applied, 'direct_route');
        WHEN 'fuel_efficient' THEN
            v_estimated_duration_minutes := v_estimated_duration_minutes * 1.15;
            v_distance_km := v_distance_km * 0.95;
            v_constraints_applied := array_append(v_constraints_applied, 'fuel_optimization');
        WHEN 'balanced' THEN
            v_constraints_applied := array_append(v_constraints_applied, 'balanced_optimization');
        ELSE
            RAISE EXCEPTION 'Invalid optimization type: %', p_optimization_type;
    END CASE;
    
    -- Apply Abu Dhabi specific constraints
    IF (p_constraints->>'abu_dhabi_mode')::BOOLEAN = TRUE THEN
        v_estimated_duration_minutes := v_estimated_duration_minutes * 1.05;
        v_constraints_applied := array_append(v_constraints_applied, 'abu_dhabi_adaptations');
        
        -- Check for dust storm conditions
        IF (p_constraints->>'dust_storm_risk')::TEXT = 'high' THEN
            v_estimated_duration_minutes := v_estimated_duration_minutes * 1.3;
            v_constraints_applied := array_append(v_constraints_applied, 'dust_storm_routing');
        END IF;
    END IF;
    
    -- Calculate fuel consumption estimate
    IF p_vehicle_id IS NOT NULL AND v_vehicle_data.vehicle_profile->>'fuel_efficiency' IS NOT NULL THEN
        v_fuel_consumption := v_distance_km / (v_vehicle_data.vehicle_profile->>'fuel_efficiency')::NUMERIC;
    ELSE
        v_fuel_consumption := v_distance_km * 0.08; -- Default 8L/100km
    END IF;
    
    -- Generate simplified waypoints
    v_waypoints := jsonb_build_array(
        jsonb_build_object('lat', p_start_lat, 'lon', p_start_lon, 'type', 'start'),
        jsonb_build_object('lat', p_end_lat, 'lon', p_end_lon, 'type', 'end')
    );
    
    -- Calculate route quality score
    v_route_score := ROUND(
        100 - 
        (LEAST(v_estimated_duration_minutes / 60, 5) * 10) -
        (LEAST(v_distance_km / 50, 5) * 5) -
        (CASE WHEN array_length(v_constraints_applied, 1) > 3 THEN 15 ELSE 0 END),
        1
    );
    
    -- Build comprehensive result
    v_result := jsonb_build_object(
        'route_id', gen_random_uuid(),
        'optimization_type', p_optimization_type,
        'start_location', jsonb_build_object('lat', p_start_lat, 'lon', p_start_lon),
        'end_location', jsonb_build_object('lat', p_end_lat, 'lon', p_end_lon),
        'vehicle_id', p_vehicle_id,
        'route_metrics', jsonb_build_object(
            'distance_km', v_distance_km,
            'estimated_duration_minutes', v_estimated_duration_minutes,
            'estimated_fuel_consumption_liters', ROUND(v_fuel_consumption, 2),
            'route_quality_score', v_route_score
        ),
        'waypoints', v_waypoints,
        'constraints_applied', array_to_json(v_constraints_applied),
        'estimated_cost', jsonb_build_object(
            'fuel_cost_aed', ROUND(v_fuel_consumption * 2.5, 2),
            'time_cost_minutes', v_estimated_duration_minutes,
            'total_score', v_route_score
        ),
        'generated_at', NOW(),
        'valid_until', NOW() + INTERVAL '30 minutes'
    );
    
    RETURN v_result;
END;
$$ LANGUAGE plpgsql;

-- =====================================================
-- INDEXES FOR PERFORMANCE OPTIMIZATION
-- =====================================================

-- Indexes for vehicle health scoring
CREATE INDEX IF NOT EXISTS idx_vehicles_health_scoring 
ON vehicles (health_score, operational_status, autonomy_status, last_seen);

-- Indexes for fleet analytics
CREATE INDEX IF NOT EXISTS idx_vehicles_fleet_analytics 
ON vehicles (fleet_id, organization_id, operational_status, created_at);

-- Indexes for maintenance predictions
CREATE INDEX IF NOT EXISTS idx_vehicles_maintenance 
ON vehicles (odometer_km, health_score, battery_level, fuel_level);
