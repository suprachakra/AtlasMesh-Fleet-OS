-- Mission Management Database Schema
-- Mission templates, orchestration, and execution

-- Create mission_management schema
CREATE SCHEMA IF NOT EXISTS mission_management;

-- Mission Templates
CREATE TABLE mission_management.mission_templates (
    template_id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    name VARCHAR(255) NOT NULL,
    description TEXT,
    parameters JSONB NOT NULL DEFAULT '{}',
    dependencies JSONB DEFAULT '[]',
    safety_requirements JSONB NOT NULL DEFAULT '{}',
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);

-- Mission Executions
CREATE TABLE mission_management.missions (
    mission_id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    template_id UUID REFERENCES mission_management.mission_templates(template_id),
    fleet_id UUID NOT NULL,
    vehicle_id UUID NOT NULL,
    parameters JSONB NOT NULL DEFAULT '{}',
    status VARCHAR(50) NOT NULL DEFAULT 'pending',
    started_at TIMESTAMP WITH TIME ZONE,
    completed_at TIMESTAMP WITH TIME ZONE,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);

-- Mission Dependencies
CREATE TABLE mission_management.mission_dependencies (
    mission_id UUID REFERENCES mission_management.missions(mission_id) ON DELETE CASCADE,
    dependency_mission_id UUID REFERENCES mission_management.missions(mission_id) ON DELETE CASCADE,
    dependency_type VARCHAR(50) NOT NULL,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    PRIMARY KEY (mission_id, dependency_mission_id)
);

-- Mission Execution Logs
CREATE TABLE mission_management.mission_logs (
    log_id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    mission_id UUID REFERENCES mission_management.missions(mission_id) ON DELETE CASCADE,
    log_level VARCHAR(20) NOT NULL,
    message TEXT NOT NULL,
    metadata JSONB DEFAULT '{}',
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);

-- Mission Performance Metrics
CREATE TABLE mission_management.mission_metrics (
    mission_id UUID REFERENCES mission_management.missions(mission_id) ON DELETE CASCADE,
    metric_type VARCHAR(100) NOT NULL,
    metric_value DECIMAL(10,4) NOT NULL,
    metric_unit VARCHAR(50) NOT NULL,
    recorded_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);

-- Mission Safety Events
CREATE TABLE mission_management.safety_events (
    event_id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    mission_id UUID REFERENCES mission_management.missions(mission_id) ON DELETE CASCADE,
    event_type VARCHAR(100) NOT NULL,
    severity VARCHAR(20) NOT NULL,
    description TEXT NOT NULL,
    resolution_status VARCHAR(50) DEFAULT 'open',
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    resolved_at TIMESTAMP WITH TIME ZONE
);

-- Trip Management Tables
CREATE TABLE mission_management.trips (
    trip_id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    mission_id UUID REFERENCES mission_management.missions(mission_id),
    fleet_id UUID NOT NULL,
    vehicle_id UUID NOT NULL,
    customer_id UUID,
    pickup_location JSONB NOT NULL,
    destination_location JSONB NOT NULL,
    route_data JSONB,
    status VARCHAR(50) NOT NULL DEFAULT 'pending',
    passenger_count INTEGER DEFAULT 1,
    trip_type VARCHAR(50) NOT NULL DEFAULT 'on_demand',
    estimated_duration INTEGER, -- in seconds
    estimated_distance DECIMAL(10,2), -- in kilometers
    started_at TIMESTAMP WITH TIME ZONE,
    completed_at TIMESTAMP WITH TIME ZONE,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);

-- Trip Routes
CREATE TABLE mission_management.trip_routes (
    route_id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    trip_id UUID REFERENCES mission_management.trips(trip_id) ON DELETE CASCADE,
    route_data JSONB NOT NULL,
    optimization_score DECIMAL(5,2),
    traffic_conditions JSONB,
    weather_conditions JSONB,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);

-- Trip Tracking
CREATE TABLE mission_management.trip_tracking (
    tracking_id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    trip_id UUID REFERENCES mission_management.trips(trip_id) ON DELETE CASCADE,
    location JSONB NOT NULL,
    speed DECIMAL(5,2),
    heading DECIMAL(5,2),
    accuracy DECIMAL(5,2),
    timestamp TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);

-- Trip Analytics
CREATE TABLE mission_management.trip_analytics (
    trip_id UUID REFERENCES mission_management.trips(trip_id) ON DELETE CASCADE,
    metric_type VARCHAR(100) NOT NULL,
    metric_value DECIMAL(10,4) NOT NULL,
    metric_unit VARCHAR(50) NOT NULL,
    recorded_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);

-- Trip Logs
CREATE TABLE mission_management.trip_logs (
    log_id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    trip_id UUID REFERENCES mission_management.trips(trip_id) ON DELETE CASCADE,
    log_level VARCHAR(20) NOT NULL,
    message TEXT NOT NULL,
    metadata JSONB DEFAULT '{}',
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);

-- Indexes for performance
CREATE INDEX idx_mission_templates_name ON mission_management.mission_templates(name);
CREATE INDEX idx_missions_fleet_id ON mission_management.missions(fleet_id);
CREATE INDEX idx_missions_vehicle_id ON mission_management.missions(vehicle_id);
CREATE INDEX idx_missions_status ON mission_management.missions(status);
CREATE INDEX idx_missions_created_at ON mission_management.missions(created_at);
CREATE INDEX idx_mission_dependencies_mission_id ON mission_management.mission_dependencies(mission_id);
CREATE INDEX idx_mission_logs_mission_id ON mission_management.mission_logs(mission_id);
CREATE INDEX idx_mission_logs_created_at ON mission_management.mission_logs(created_at);
CREATE INDEX idx_mission_metrics_mission_id ON mission_management.mission_metrics(mission_id);
CREATE INDEX idx_safety_events_mission_id ON mission_management.safety_events(mission_id);
CREATE INDEX idx_safety_events_severity ON mission_management.safety_events(severity);

-- Trip Management Indexes
CREATE INDEX idx_trips_fleet_id ON mission_management.trips(fleet_id);
CREATE INDEX idx_trips_vehicle_id ON mission_management.trips(vehicle_id);
CREATE INDEX idx_trips_customer_id ON mission_management.trips(customer_id);
CREATE INDEX idx_trips_status ON mission_management.trips(status);
CREATE INDEX idx_trips_trip_type ON mission_management.trips(trip_type);
CREATE INDEX idx_trips_created_at ON mission_management.trips(created_at);
CREATE INDEX idx_trip_routes_trip_id ON mission_management.trip_routes(trip_id);
CREATE INDEX idx_trip_tracking_trip_id ON mission_management.trip_tracking(trip_id);
CREATE INDEX idx_trip_tracking_timestamp ON mission_management.trip_tracking(timestamp);
CREATE INDEX idx_trip_analytics_trip_id ON mission_management.trip_analytics(trip_id);
CREATE INDEX idx_trip_logs_trip_id ON mission_management.trip_logs(trip_id);
CREATE INDEX idx_trip_logs_created_at ON mission_management.trip_logs(created_at);

-- Row Level Security
ALTER TABLE mission_management.mission_templates ENABLE ROW LEVEL SECURITY;
ALTER TABLE mission_management.missions ENABLE ROW LEVEL SECURITY;
ALTER TABLE mission_management.mission_dependencies ENABLE ROW LEVEL SECURITY;
ALTER TABLE mission_management.mission_logs ENABLE ROW LEVEL SECURITY;
ALTER TABLE mission_management.mission_metrics ENABLE ROW LEVEL SECURITY;
ALTER TABLE mission_management.safety_events ENABLE ROW LEVEL SECURITY;
ALTER TABLE mission_management.trips ENABLE ROW LEVEL SECURITY;
ALTER TABLE mission_management.trip_routes ENABLE ROW LEVEL SECURITY;
ALTER TABLE mission_management.trip_tracking ENABLE ROW LEVEL SECURITY;
ALTER TABLE mission_management.trip_analytics ENABLE ROW LEVEL SECURITY;
ALTER TABLE mission_management.trip_logs ENABLE ROW LEVEL SECURITY;

-- Audit triggers
CREATE OR REPLACE FUNCTION mission_management.audit_trigger()
RETURNS TRIGGER AS $$
BEGIN
    NEW.updated_at = NOW();
    RETURN NEW;
END;
$$ LANGUAGE plpgsql;

CREATE TRIGGER mission_templates_audit
    BEFORE UPDATE ON mission_management.mission_templates
    FOR EACH ROW
    EXECUTE FUNCTION mission_management.audit_trigger();

CREATE TRIGGER missions_audit
    BEFORE UPDATE ON mission_management.missions
    FOR EACH ROW
    EXECUTE FUNCTION mission_management.audit_trigger();

CREATE TRIGGER trips_audit
    BEFORE UPDATE ON mission_management.trips
    FOR EACH ROW
    EXECUTE FUNCTION mission_management.audit_trigger();
