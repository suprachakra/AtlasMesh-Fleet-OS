-- AtlasMesh Fleet Manager - Predictive Maintenance & Cost Analysis
-- Predictive maintenance algorithms and comprehensive cost calculations
-- Abu Dhabi Autonomous Vehicle Operations

-- Set search path
SET search_path TO fleet_core, public;

-- =====================================================
-- PREDICTIVE MAINTENANCE PROCEDURES
-- =====================================================

-- Predict maintenance needs using advanced heuristics
CREATE OR REPLACE FUNCTION predict_maintenance_needs(
    p_vehicle_id UUID DEFAULT NULL,
    p_fleet_id UUID DEFAULT NULL,
    p_prediction_horizon_days INTEGER DEFAULT 30
) RETURNS JSONB AS $$
DECLARE
    v_result JSONB := '{}';
    v_predictions JSONB := '[]';
    v_vehicle_cursor CURSOR FOR 
        SELECT * FROM vehicles 
        WHERE (p_vehicle_id IS NULL OR vehicle_id = p_vehicle_id)
        AND (p_fleet_id IS NULL OR fleet_id = p_fleet_id)
        AND operational_status != 'offline';
    v_vehicle_record RECORD;
BEGIN
    FOR v_vehicle_record IN v_vehicle_cursor LOOP
        DECLARE
            v_vehicle_prediction JSONB := '{}';
            v_maintenance_items JSONB := '[]';
            v_risk_score NUMERIC := 0;
            v_priority TEXT := 'low';
        BEGIN
            -- Mileage-based predictions
            IF v_vehicle_record.odometer_km > 0 THEN
                DECLARE
                    v_km_since_service NUMERIC := v_vehicle_record.odometer_km % 5000;
                    v_daily_km NUMERIC := 100; -- Estimated daily usage
                    v_days_to_service NUMERIC;
                BEGIN
                    v_days_to_service := (5000 - v_km_since_service) / v_daily_km;
                    
                    IF v_days_to_service <= p_prediction_horizon_days THEN
                        v_maintenance_items := v_maintenance_items || jsonb_build_object(
                            'type', 'scheduled_service',
                            'description', 'Regular scheduled maintenance',
                            'predicted_date', (NOW() + (v_days_to_service || ' days')::INTERVAL)::DATE,
                            'confidence', 0.9,
                            'estimated_cost_aed', 1500,
                            'estimated_duration_hours', 4,
                            'urgency', CASE 
                                WHEN v_days_to_service <= 7 THEN 'high'
                                WHEN v_days_to_service <= 14 THEN 'medium'
                                ELSE 'low'
                            END
                        );
                        v_risk_score := v_risk_score + 30;
                    END IF;
                END;
            END IF;
            
            -- Health score based predictions
            IF v_vehicle_record.health_score < 75 THEN
                v_maintenance_items := v_maintenance_items || jsonb_build_object(
                    'type', 'health_inspection',
                    'description', 'Vehicle health inspection due to low health score',
                    'predicted_date', (NOW() + INTERVAL '7 days')::DATE,
                    'confidence', 0.8,
                    'estimated_cost_aed', 500,
                    'estimated_duration_hours', 2,
                    'urgency', CASE 
                        WHEN v_vehicle_record.health_score < 40 THEN 'critical'
                        WHEN v_vehicle_record.health_score < 60 THEN 'high'
                        ELSE 'medium'
                    END
                );
                v_risk_score := v_risk_score + (80 - v_vehicle_record.health_score);
            END IF;
            
            -- Battery system predictions (Abu Dhabi heat considerations)
            IF v_vehicle_record.battery_level IS NOT NULL AND v_vehicle_record.battery_level > 0 THEN
                DECLARE
                    v_battery_degradation_rate NUMERIC := 0.1; -- 10% per year in Abu Dhabi heat
                    v_battery_age_years NUMERIC := EXTRACT(EPOCH FROM (NOW() - v_vehicle_record.created_at)) / (365.25 * 24 * 3600);
                    v_predicted_battery_health NUMERIC := 100 - (v_battery_age_years * v_battery_degradation_rate * 100);
                BEGIN
                    IF v_predicted_battery_health < 70 THEN
                        v_maintenance_items := v_maintenance_items || jsonb_build_object(
                            'type', 'battery_replacement',
                            'description', 'Battery replacement recommended due to degradation',
                            'predicted_date', (NOW() + INTERVAL '14 days')::DATE,
                            'confidence', 0.75,
                            'estimated_cost_aed', 8000,
                            'estimated_duration_hours', 6,
                            'urgency', CASE 
                                WHEN v_predicted_battery_health < 50 THEN 'critical'
                                WHEN v_predicted_battery_health < 60 THEN 'high'
                                ELSE 'medium'
                            END,
                            'abu_dhabi_factor', 'High temperature accelerates battery degradation'
                        );
                        v_risk_score := v_risk_score + (70 - v_predicted_battery_health);
                    END IF;
                END;
            END IF;
            
            -- Connectivity issues prediction
            IF EXTRACT(EPOCH FROM (NOW() - v_vehicle_record.last_seen)) / 60 > 5 THEN
                DECLARE
                    v_connectivity_issue_severity TEXT;
                    v_minutes_offline NUMERIC := EXTRACT(EPOCH FROM (NOW() - v_vehicle_record.last_seen)) / 60;
                BEGIN
                    IF v_minutes_offline > 60 THEN
                        v_connectivity_issue_severity := 'critical';
                    ELSIF v_minutes_offline > 30 THEN
                        v_connectivity_issue_severity := 'high';
                    ELSE
                        v_connectivity_issue_severity := 'medium';
                    END IF;
                    
                    v_maintenance_items := v_maintenance_items || jsonb_build_object(
                        'type', 'communication_check',
                        'description', 'Communication system inspection needed',
                        'predicted_date', (NOW() + INTERVAL '3 days')::DATE,
                        'confidence', 0.85,
                        'estimated_cost_aed', 300,
                        'estimated_duration_hours', 1,
                        'urgency', v_connectivity_issue_severity,
                        'details', jsonb_build_object(
                            'minutes_offline', v_minutes_offline,
                            'last_communication', v_vehicle_record.last_seen
                        )
                    );
                    v_risk_score := v_risk_score + LEAST(v_minutes_offline * 0.5, 50);
                END;
            END IF;
            
            -- Abu Dhabi specific maintenance predictions
            DECLARE
                v_abu_dhabi_season TEXT;
                v_current_month INTEGER := EXTRACT(MONTH FROM NOW());
            BEGIN
                -- Determine season for maintenance scheduling
                IF v_current_month IN (6, 7, 8, 9) THEN
                    v_abu_dhabi_season := 'summer_extreme';
                ELSIF v_current_month IN (10, 11, 12, 1, 2) THEN
                    v_abu_dhabi_season := 'winter_mild';
                ELSE
                    v_abu_dhabi_season := 'transition';
                END IF;
                
                -- Summer extreme heat maintenance
                IF v_abu_dhabi_season = 'summer_extreme' THEN
                    v_maintenance_items := v_maintenance_items || jsonb_build_object(
                        'type', 'cooling_system_check',
                        'description', 'Enhanced cooling system inspection for extreme summer heat',
                        'predicted_date', (NOW() + INTERVAL '14 days')::DATE,
                        'confidence', 0.9,
                        'estimated_cost_aed', 400,
                        'estimated_duration_hours', 2,
                        'urgency', 'high',
                        'abu_dhabi_factor', 'Summer temperatures exceed 45°C, critical for vehicle cooling'
                    );
                    v_risk_score := v_risk_score + 20;
                END IF;
                
                -- Dust and sand filtration maintenance
                v_maintenance_items := v_maintenance_items || jsonb_build_object(
                    'type', 'dust_protection_service',
                    'description', 'Air filter and dust protection system maintenance',
                    'predicted_date', (NOW() + INTERVAL '21 days')::DATE,
                    'confidence', 0.95,
                    'estimated_cost_aed', 250,
                    'estimated_duration_hours', 1,
                    'urgency', 'medium',
                    'abu_dhabi_factor', 'Desert environment requires frequent filter maintenance'
                );
                v_risk_score := v_risk_score + 15;
            END;
            
            -- Determine overall priority based on risk score
            IF v_risk_score >= 100 THEN
                v_priority := 'critical';
            ELSIF v_risk_score >= 70 THEN
                v_priority := 'high';
            ELSIF v_risk_score >= 40 THEN
                v_priority := 'medium';
            ELSE
                v_priority := 'low';
            END IF;
            
            -- Build vehicle prediction
            v_vehicle_prediction := jsonb_build_object(
                'vehicle_id', v_vehicle_record.vehicle_id,
                'asset_tag', v_vehicle_record.asset_tag,
                'current_health_score', v_vehicle_record.health_score,
                'risk_score', ROUND(v_risk_score, 1),
                'priority', v_priority,
                'predicted_maintenance', v_maintenance_items,
                'total_estimated_cost_aed', (
                    SELECT SUM((item->>'estimated_cost_aed')::NUMERIC)
                    FROM jsonb_array_elements(v_maintenance_items) item
                ),
                'total_estimated_hours', (
                    SELECT SUM((item->>'estimated_duration_hours')::NUMERIC)
                    FROM jsonb_array_elements(v_maintenance_items) item
                ),
                'next_maintenance_date', (
                    SELECT MIN((item->>'predicted_date')::DATE)
                    FROM jsonb_array_elements(v_maintenance_items) item
                ),
                'abu_dhabi_specific_items', (
                    SELECT COUNT(*)
                    FROM jsonb_array_elements(v_maintenance_items) item
                    WHERE item ? 'abu_dhabi_factor'
                )
            );
            
            v_predictions := v_predictions || v_vehicle_prediction;
        END;
    END LOOP;
    
    -- Build comprehensive summary result
    v_result := jsonb_build_object(
        'prediction_horizon_days', p_prediction_horizon_days,
        'total_vehicles_analyzed', jsonb_array_length(v_predictions),
        'predictions', v_predictions,
        'summary', jsonb_build_object(
            'critical_priority', (
                SELECT COUNT(*) FROM jsonb_array_elements(v_predictions) pred 
                WHERE pred->>'priority' = 'critical'
            ),
            'high_priority', (
                SELECT COUNT(*) FROM jsonb_array_elements(v_predictions) pred 
                WHERE pred->>'priority' = 'high'
            ),
            'medium_priority', (
                SELECT COUNT(*) FROM jsonb_array_elements(v_predictions) pred 
                WHERE pred->>'priority' = 'medium'
            ),
            'total_estimated_cost_aed', (
                SELECT SUM((pred->>'total_estimated_cost_aed')::NUMERIC)
                FROM jsonb_array_elements(v_predictions) pred
                WHERE pred->>'total_estimated_cost_aed' IS NOT NULL
            ),
            'total_estimated_hours', (
                SELECT SUM((pred->>'total_estimated_hours')::NUMERIC)
                FROM jsonb_array_elements(v_predictions) pred
                WHERE pred->>'total_estimated_hours' IS NOT NULL
            ),
            'abu_dhabi_adaptations', (
                SELECT SUM((pred->>'abu_dhabi_specific_items')::INTEGER)
                FROM jsonb_array_elements(v_predictions) pred
            )
        ),
        'recommendations', jsonb_build_array(
            'Schedule critical priority maintenance immediately',
            'Plan maintenance during cooler months when possible',
            'Increase dust protection maintenance frequency',
            'Monitor battery health closely during summer months'
        ),
        'generated_at', NOW()
    );
    
    RETURN v_result;
END;
$$ LANGUAGE plpgsql;

-- =====================================================
-- COST CALCULATION PROCEDURES
-- =====================================================

-- Calculate comprehensive fleet operational costs
CREATE OR REPLACE FUNCTION calculate_fleet_operational_costs(
    p_fleet_id UUID DEFAULT NULL,
    p_organization_id UUID DEFAULT NULL,
    p_start_date DATE DEFAULT CURRENT_DATE - INTERVAL '30 days',
    p_end_date DATE DEFAULT CURRENT_DATE
) RETURNS JSONB AS $$
DECLARE
    v_result JSONB := '{}';
    v_fuel_costs NUMERIC := 0;
    v_maintenance_costs NUMERIC := 0;
    v_operational_costs NUMERIC := 0;
    v_insurance_costs NUMERIC := 0;
    v_depreciation_costs NUMERIC := 0;
    v_total_distance NUMERIC := 0;
    v_total_vehicles INTEGER := 0;
    v_cost_per_km NUMERIC := 0;
    v_cost_per_vehicle_per_day NUMERIC := 0;
    v_days_in_period INTEGER;
    v_abu_dhabi_factors JSONB := '{}';
BEGIN
    v_days_in_period := p_end_date - p_start_date + 1;
    
    -- Get fleet vehicle count and total distance
    SELECT COUNT(*), COALESCE(SUM(odometer_km), 0)
    INTO v_total_vehicles, v_total_distance
    FROM vehicles v
    WHERE (p_fleet_id IS NULL OR v.fleet_id = p_fleet_id)
    AND (p_organization_id IS NULL OR v.organization_id = p_organization_id);
    
    -- Calculate fuel costs (Abu Dhabi specific pricing)
    -- Average consumption: 8L/100km for mixed fleet
    -- Abu Dhabi fuel price: ~2.5 AED per liter
    v_fuel_costs := v_total_distance * 0.08 * 2.5;
    
    -- Calculate maintenance costs (enhanced for Abu Dhabi conditions)
    -- Base maintenance: 15 AED per vehicle per day
    -- Abu Dhabi premium: +30% for desert conditions
    v_maintenance_costs := v_total_vehicles * v_days_in_period * 15 * 1.3;
    
    -- Calculate operational costs
    -- Driver costs, utilities, depot costs: 25 AED per vehicle per day
    v_operational_costs := v_total_vehicles * v_days_in_period * 25;
    
    -- Calculate insurance costs (UAE specific)
    -- Comprehensive insurance: ~12 AED per vehicle per day
    v_insurance_costs := v_total_vehicles * v_days_in_period * 12;
    
    -- Calculate depreciation costs
    -- Average vehicle value: 150,000 AED, 5-year depreciation
    -- Accelerated depreciation in desert: +20%
    v_depreciation_costs := v_total_vehicles * v_days_in_period * (150000 / (5 * 365)) * 1.2;
    
    -- Abu Dhabi specific cost factors
    v_abu_dhabi_factors := jsonb_build_object(
        'desert_maintenance_premium', 0.30,
        'fuel_price_aed_per_liter', 2.5,
        'insurance_premium_uae', 0.15,
        'accelerated_depreciation', 0.20,
        'cooling_system_costs', v_total_vehicles * v_days_in_period * 3,
        'dust_protection_costs', v_total_vehicles * v_days_in_period * 2
    );
    
    -- Add Abu Dhabi specific costs
    v_operational_costs := v_operational_costs + 
                          (v_abu_dhabi_factors->>'cooling_system_costs')::NUMERIC +
                          (v_abu_dhabi_factors->>'dust_protection_costs')::NUMERIC;
    
    -- Calculate per-unit costs
    IF v_total_distance > 0 THEN
        v_cost_per_km := (v_fuel_costs + v_maintenance_costs + v_operational_costs + 
                         v_insurance_costs + v_depreciation_costs) / v_total_distance;
    END IF;
    
    IF v_total_vehicles > 0 AND v_days_in_period > 0 THEN
        v_cost_per_vehicle_per_day := (v_fuel_costs + v_maintenance_costs + v_operational_costs + 
                                      v_insurance_costs + v_depreciation_costs) / 
                                     (v_total_vehicles * v_days_in_period);
    END IF;
    
    -- Build comprehensive result
    v_result := jsonb_build_object(
        'fleet_id', p_fleet_id,
        'organization_id', p_organization_id,
        'analysis_period', jsonb_build_object(
            'start_date', p_start_date,
            'end_date', p_end_date,
            'days', v_days_in_period
        ),
        'fleet_metrics', jsonb_build_object(
            'total_vehicles', v_total_vehicles,
            'total_distance_km', ROUND(v_total_distance, 1),
            'avg_distance_per_vehicle', CASE WHEN v_total_vehicles > 0 THEN ROUND(v_total_distance / v_total_vehicles, 1) ELSE 0 END,
            'avg_daily_distance_per_vehicle', CASE WHEN v_total_vehicles > 0 AND v_days_in_period > 0 THEN ROUND(v_total_distance / (v_total_vehicles * v_days_in_period), 1) ELSE 0 END
        ),
        'cost_breakdown', jsonb_build_object(
            'fuel_costs_aed', ROUND(v_fuel_costs, 2),
            'maintenance_costs_aed', ROUND(v_maintenance_costs, 2),
            'operational_costs_aed', ROUND(v_operational_costs, 2),
            'insurance_costs_aed', ROUND(v_insurance_costs, 2),
            'depreciation_costs_aed', ROUND(v_depreciation_costs, 2),
            'total_costs_aed', ROUND(v_fuel_costs + v_maintenance_costs + v_operational_costs + v_insurance_costs + v_depreciation_costs, 2)
        ),
        'cost_percentages', jsonb_build_object(
            'fuel_percentage', CASE WHEN (v_fuel_costs + v_maintenance_costs + v_operational_costs + v_insurance_costs + v_depreciation_costs) > 0 
                THEN ROUND(v_fuel_costs * 100.0 / (v_fuel_costs + v_maintenance_costs + v_operational_costs + v_insurance_costs + v_depreciation_costs), 1) 
                ELSE 0 END,
            'maintenance_percentage', CASE WHEN (v_fuel_costs + v_maintenance_costs + v_operational_costs + v_insurance_costs + v_depreciation_costs) > 0 
                THEN ROUND(v_maintenance_costs * 100.0 / (v_fuel_costs + v_maintenance_costs + v_operational_costs + v_insurance_costs + v_depreciation_costs), 1) 
                ELSE 0 END,
            'operational_percentage', CASE WHEN (v_fuel_costs + v_maintenance_costs + v_operational_costs + v_insurance_costs + v_depreciation_costs) > 0 
                THEN ROUND(v_operational_costs * 100.0 / (v_fuel_costs + v_maintenance_costs + v_operational_costs + v_insurance_costs + v_depreciation_costs), 1) 
                ELSE 0 END
        ),
        'cost_metrics', jsonb_build_object(
            'cost_per_km_aed', ROUND(v_cost_per_km, 3),
            'cost_per_vehicle_per_day_aed', ROUND(v_cost_per_vehicle_per_day, 2),
            'cost_per_vehicle_per_month_aed', ROUND(v_cost_per_vehicle_per_day * 30, 2),
            'cost_per_1000km_aed', ROUND(v_cost_per_km * 1000, 2)
        ),
        'projections', jsonb_build_object(
            'monthly_cost_aed', ROUND((v_fuel_costs + v_maintenance_costs + v_operational_costs + v_insurance_costs + v_depreciation_costs) * 30.0 / v_days_in_period, 2),
            'yearly_cost_aed', ROUND((v_fuel_costs + v_maintenance_costs + v_operational_costs + v_insurance_costs + v_depreciation_costs) * 365.0 / v_days_in_period, 2),
            'cost_per_vehicle_yearly_aed', CASE WHEN v_total_vehicles > 0 
                THEN ROUND((v_fuel_costs + v_maintenance_costs + v_operational_costs + v_insurance_costs + v_depreciation_costs) * 365.0 / (v_days_in_period * v_total_vehicles), 2) 
                ELSE 0 END
        ),
        'abu_dhabi_factors', v_abu_dhabi_factors,
        'benchmarks', jsonb_build_object(
            'industry_average_cost_per_km_aed', 2.5,
            'performance_vs_industry', CASE WHEN v_cost_per_km > 0 
                THEN ROUND(((v_cost_per_km - 2.5) / 2.5) * 100, 1) 
                ELSE 0 END,
            'efficiency_rating', CASE 
                WHEN v_cost_per_km <= 2.0 THEN 'excellent'
                WHEN v_cost_per_km <= 2.5 THEN 'good'
                WHEN v_cost_per_km <= 3.0 THEN 'average'
                WHEN v_cost_per_km <= 3.5 THEN 'below_average'
                ELSE 'poor'
            END
        ),
        'optimization_recommendations', jsonb_build_array(
            'Consider route optimization to reduce fuel consumption',
            'Implement predictive maintenance to reduce breakdown costs',
            'Schedule maintenance during cooler months to reduce labor costs',
            'Monitor tire pressure regularly in desert conditions',
            'Use covered parking to reduce cooling costs'
        ),
        'generated_at', NOW()
    );
    
    RETURN v_result;
END;
$$ LANGUAGE plpgsql;

-- Generate maintenance cost projections
CREATE OR REPLACE FUNCTION project_maintenance_costs(
    p_fleet_id UUID DEFAULT NULL,
    p_months_ahead INTEGER DEFAULT 12
) RETURNS JSONB AS $$
DECLARE
    v_result JSONB := '{}';
    v_monthly_projections JSONB := '[]';
    v_total_projected_cost NUMERIC := 0;
    v_month_counter INTEGER := 1;
BEGIN
    WHILE v_month_counter <= p_months_ahead LOOP
        DECLARE
            v_month_date DATE := (CURRENT_DATE + (v_month_counter || ' months')::INTERVAL)::DATE;
            v_month_name TEXT := TO_CHAR(v_month_date, 'Month YYYY');
            v_seasonal_factor NUMERIC := 1.0;
            v_base_monthly_cost NUMERIC := 0;
            v_predicted_maintenance JSONB;
        BEGIN
            -- Get base monthly cost
            SELECT (result->>'monthly_cost_aed')::NUMERIC
            INTO v_base_monthly_cost
            FROM (
                SELECT calculate_fleet_operational_costs(p_fleet_id, NULL, CURRENT_DATE - 30, CURRENT_DATE) as result
            ) t;
            
            -- Apply seasonal factors for Abu Dhabi
            CASE EXTRACT(MONTH FROM v_month_date)
                WHEN 6, 7, 8, 9 THEN v_seasonal_factor := 1.4; -- Summer extreme heat
                WHEN 12, 1, 2 THEN v_seasonal_factor := 0.9;    -- Winter mild
                ELSE v_seasonal_factor := 1.1;                  -- Transition periods
            END CASE;
            
            -- Get predicted maintenance for this period
            SELECT predict_maintenance_needs(NULL, p_fleet_id, 30)
            INTO v_predicted_maintenance;
            
            DECLARE
                v_monthly_cost NUMERIC := v_base_monthly_cost * v_seasonal_factor;
                v_maintenance_items_count INTEGER := (v_predicted_maintenance->'summary'->>'critical_priority')::INTEGER + 
                                                     (v_predicted_maintenance->'summary'->>'high_priority')::INTEGER;
            BEGIN
                v_monthly_projections := v_monthly_projections || jsonb_build_object(
                    'month', v_month_name,
                    'month_number', v_month_counter,
                    'projected_cost_aed', ROUND(v_monthly_cost, 2),
                    'seasonal_factor', v_seasonal_factor,
                    'predicted_maintenance_items', v_maintenance_items_count,
                    'cost_category', CASE 
                        WHEN v_seasonal_factor > 1.3 THEN 'high_season'
                        WHEN v_seasonal_factor < 1.0 THEN 'low_season'
                        ELSE 'normal_season'
                    END
                );
                
                v_total_projected_cost := v_total_projected_cost + v_monthly_cost;
            END;
        END;
        
        v_month_counter := v_month_counter + 1;
    END LOOP;
    
    -- Build result
    v_result := jsonb_build_object(
        'fleet_id', p_fleet_id,
        'projection_months', p_months_ahead,
        'total_projected_cost_aed', ROUND(v_total_projected_cost, 2),
        'average_monthly_cost_aed', ROUND(v_total_projected_cost / p_months_ahead, 2),
        'monthly_projections', v_monthly_projections,
        'cost_insights', jsonb_build_object(
            'highest_cost_months', jsonb_build_array('July', 'August', 'September'),
            'lowest_cost_months', jsonb_build_array('December', 'January', 'February'),
            'summer_premium_percentage', 40,
            'winter_discount_percentage', 10
        ),
        'budget_recommendations', jsonb_build_array(
            'Allocate 40% more budget for summer months (June-September)',
            'Schedule major maintenance during winter months for cost savings',
            'Maintain emergency fund for unexpected breakdowns during extreme heat',
            'Consider preventive cooling system upgrades before summer'
        ),
        'generated_at', NOW()
    );
    
    RETURN v_result;
END;
$$ LANGUAGE plpgsql;
