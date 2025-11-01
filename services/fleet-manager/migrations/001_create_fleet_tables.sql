-- AtlasMesh Fleet Manager Database Schema
-- Migration 001: Create fleet, vehicles, and related tables
--
-- MIGRATION: Initial schema creation for fleet management system
-- BLAST RADIUS: Creates new schemas and tables - no existing data impact
-- ROLLBACK: Use migration 001_down.sql to drop all created objects
-- STATE DRIFT: Idempotent - safe to run multiple times
--
-- INTEGRATION CONTRACTS:
-- - Fleet Manager service reads/writes to fleet_core schema
-- - Telemetry Ingest service writes to fleet_telemetry schema  
-- - Audit service logs to fleet_audit schema
-- - PostGIS extension required for geospatial vehicle tracking
--
-- PERFORMANCE: Indexes created for common query patterns
-- SECURITY: Row-level security policies for multi-tenant isolation
-- COMPLIANCE: Audit trails for all data modifications

-- Enable required PostgreSQL extensions
-- PERF: UUID generation for primary keys
CREATE EXTENSION IF NOT EXISTS "uuid-ossp";
-- INTEGRATION: PostGIS for geospatial vehicle location tracking
CREATE EXTENSION IF NOT EXISTS "postgis";  
-- SECURITY: Cryptographic functions for sensitive data
CREATE EXTENSION IF NOT EXISTS "pgcrypto";

-- Create logical schemas for data organization
-- ARCHITECTURE: Separate schemas for different data domains
CREATE SCHEMA IF NOT EXISTS fleet_core;      -- Core fleet/vehicle entities
CREATE SCHEMA IF NOT EXISTS fleet_telemetry; -- Real-time telemetry data
CREATE SCHEMA IF NOT EXISTS fleet_audit;     -- Audit and compliance logs

-- Set default schema search path for this migration
SET search_path TO fleet_core, public;

-- Organizations table (multi-tenancy root entity)
-- SECURITY: Multi-tenant isolation - all other tables reference organization_id
-- COMPLIANCE: Tracks compliance frameworks and data residency requirements
CREATE TABLE organizations (
    -- Primary identifier - UUID for security and scalability
    organization_id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    name VARCHAR(255) NOT NULL,
    
    -- BUSINESS LOGIC: Sector determines available features and compliance requirements
    -- Abu Dhabi supports all sectors for autonomous vehicle operations
    sector VARCHAR(100) NOT NULL CHECK (sector IN ('ridehail', 'logistics', 'mining', 'defense', 'agriculture', 'construction', 'emergency', 'public_transport')),
    
    -- Configuration and entitlements (JSON for flexibility)
    -- ARCHITECTURE: Policy-as-code configuration stored as JSONB for performance
    configuration JSONB DEFAULT '{}',           -- Feature flags, limits, settings
    entitlements JSONB DEFAULT '{}',            -- What features org can access
    subscription_tier VARCHAR(50) DEFAULT 'basic' CHECK (subscription_tier IN ('basic', 'professional', 'enterprise', 'custom')),
    
    -- Contact information for billing and support
    primary_contact_name VARCHAR(255),
    primary_contact_email VARCHAR(255),
    primary_contact_phone VARCHAR(50),
    
    -- JSONB for flexible address structure (supports international formats)
    address JSONB,
    
    -- Status and lifecycle management
    -- BUSINESS LOGIC: Only 'active' orgs can operate vehicles
    status VARCHAR(50) DEFAULT 'active' CHECK (status IN ('active', 'suspended', 'terminated')),
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),  -- Audit trail
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),  -- Last modification
    
    -- Compliance and governance requirements
    -- COMPLIANCE: Array of applicable frameworks (ISO, UAE regulations, etc.)
    compliance_frameworks TEXT[] DEFAULT '{}',
    -- COMPLIANCE: Data residency rules for PII and telemetry
    data_residency_requirements JSONB DEFAULT '{}',
    
    -- Extensible metadata for future requirements
    metadata JSONB DEFAULT '{}'
);

-- Fleets table
CREATE TABLE fleets (
    fleet_id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    organization_id UUID NOT NULL REFERENCES organizations(organization_id) ON DELETE CASCADE,
    
    -- Fleet details
    name VARCHAR(255) NOT NULL,
    description TEXT,
    fleet_type VARCHAR(100) NOT NULL CHECK (fleet_type IN ('passenger', 'cargo', 'mixed', 'specialized')),
    
    -- Operational configuration
    configuration JSONB DEFAULT '{}',
    operational_hours JSONB, -- Operating schedule
    service_area GEOMETRY(MULTIPOLYGON, 4326), -- Geographic service area
    
    -- Capacity and limits
    max_vehicles INTEGER,
    max_concurrent_trips INTEGER,
    
    -- Status and lifecycle
    status VARCHAR(50) DEFAULT 'active' CHECK (status IN ('active', 'inactive', 'maintenance', 'decommissioned')),
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    created_by UUID NOT NULL,
    
    -- Metadata
    tags TEXT[] DEFAULT '{}',
    metadata JSONB DEFAULT '{}'
);

-- Vehicles table
CREATE TABLE vehicles (
    vehicle_id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    fleet_id UUID NOT NULL REFERENCES fleets(fleet_id) ON DELETE CASCADE,
    organization_id UUID NOT NULL REFERENCES organizations(organization_id) ON DELETE CASCADE,
    
    -- Vehicle identification
    asset_tag VARCHAR(100) NOT NULL,
    vin VARCHAR(17),
    license_plate VARCHAR(20),
    
    -- Vehicle specifications
    manufacturer VARCHAR(100) NOT NULL,
    model VARCHAR(100) NOT NULL,
    year INTEGER CHECK (year >= 1900 AND year <= EXTRACT(YEAR FROM NOW()) + 5),
    serial_number VARCHAR(100),
    
    -- Vehicle profile and capabilities
    vehicle_profile JSONB NOT NULL DEFAULT '{}',
    capabilities JSONB DEFAULT '{}',
    sensor_configuration JSONB DEFAULT '{}',
    
    -- Autonomy configuration
    autonomy_level VARCHAR(10) NOT NULL DEFAULT 'L0' CHECK (autonomy_level IN ('L0', 'L1', 'L2', 'L3', 'L4', 'L5')),
    autonomy_capabilities JSONB DEFAULT '{}',
    odd_configuration JSONB DEFAULT '{}', -- Operational Design Domain
    
    -- Current state
    operational_status VARCHAR(50) NOT NULL DEFAULT 'offline' CHECK (operational_status IN (
        'offline', 'idle', 'driving_av', 'driving_manual', 'remote_assist', 
        'charging', 'maintenance', 'safed', 'emergency', 'decommissioned'
    )),
    autonomy_status VARCHAR(50) DEFAULT 'offline' CHECK (autonomy_status IN (
        'offline', 'initializing', 'active', 'degraded', 'fallback', 'manual_override', 'emergency_stop'
    )),
    
    -- Location and movement
    current_location GEOMETRY(POINT, 4326),
    current_heading DECIMAL(5,2) CHECK (current_heading >= 0 AND current_heading < 360),
    current_speed DECIMAL(6,2) CHECK (current_speed >= 0), -- km/h
    
    -- Power and health
    battery_level DECIMAL(5,2) CHECK (battery_level >= 0 AND battery_level <= 100),
    fuel_level DECIMAL(5,2) CHECK (fuel_level >= 0 AND fuel_level <= 100),
    health_score DECIMAL(3,2) CHECK (health_score >= 0 AND health_score <= 1),
    
    -- Odometer and usage
    odometer_km DECIMAL(10,2) DEFAULT 0 CHECK (odometer_km >= 0),
    engine_hours DECIMAL(10,2) DEFAULT 0 CHECK (engine_hours >= 0),
    
    -- Current assignments
    current_trip_id UUID,
    assigned_depot_id UUID,
    assigned_operator_id UUID,
    
    -- Maintenance
    last_maintenance_date DATE,
    next_maintenance_due DATE,
    maintenance_schedule JSONB DEFAULT '{}',
    
    -- Software and firmware
    software_version VARCHAR(50),
    firmware_version VARCHAR(50),
    last_ota_update TIMESTAMP WITH TIME ZONE,
    
    -- Timestamps
    last_seen TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    commissioned_at TIMESTAMP WITH TIME ZONE,
    decommissioned_at TIMESTAMP WITH TIME ZONE,
    
    -- Metadata
    tags TEXT[] DEFAULT '{}',
    metadata JSONB DEFAULT '{}',
    
    -- Constraints
    UNIQUE(organization_id, asset_tag),
    UNIQUE(vin) WHERE vin IS NOT NULL,
    CHECK (decommissioned_at IS NULL OR decommissioned_at >= commissioned_at)
);

-- Depots table
CREATE TABLE depots (
    depot_id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    organization_id UUID NOT NULL REFERENCES organizations(organization_id) ON DELETE CASCADE,
    fleet_id UUID REFERENCES fleets(fleet_id) ON DELETE SET NULL,
    
    -- Depot details
    name VARCHAR(255) NOT NULL,
    depot_type VARCHAR(100) NOT NULL CHECK (depot_type IN ('maintenance', 'charging', 'parking', 'mixed')),
    
    -- Location
    location GEOMETRY(POINT, 4326) NOT NULL,
    address JSONB,
    geofence GEOMETRY(POLYGON, 4326),
    
    -- Capacity
    vehicle_capacity INTEGER NOT NULL CHECK (vehicle_capacity > 0),
    charging_stations INTEGER DEFAULT 0,
    maintenance_bays INTEGER DEFAULT 0,
    
    -- Operational details
    operational_hours JSONB,
    services_offered TEXT[] DEFAULT '{}',
    
    -- Status
    status VARCHAR(50) DEFAULT 'active' CHECK (status IN ('active', 'inactive', 'maintenance', 'closed')),
    
    -- Timestamps
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    
    -- Metadata
    metadata JSONB DEFAULT '{}'
);

-- Vehicle alerts table
CREATE TABLE vehicle_alerts (
    alert_id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    vehicle_id UUID NOT NULL REFERENCES vehicles(vehicle_id) ON DELETE CASCADE,
    
    -- Alert details
    alert_type VARCHAR(100) NOT NULL,
    severity VARCHAR(50) NOT NULL CHECK (severity IN ('info', 'low', 'medium', 'high', 'critical')),
    category VARCHAR(100) NOT NULL CHECK (category IN ('safety', 'technical', 'operational', 'maintenance', 'security', 'compliance')),
    
    -- Alert content
    title VARCHAR(255) NOT NULL,
    message TEXT NOT NULL,
    details JSONB DEFAULT '{}',
    
    -- Context
    location GEOMETRY(POINT, 4326),
    trip_id UUID,
    
    -- Status and resolution
    status VARCHAR(50) DEFAULT 'active' CHECK (status IN ('active', 'acknowledged', 'resolved', 'dismissed')),
    acknowledged_by UUID,
    acknowledged_at TIMESTAMP WITH TIME ZONE,
    resolved_by UUID,
    resolved_at TIMESTAMP WITH TIME ZONE,
    resolution_notes TEXT,
    
    -- Timestamps
    occurred_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    
    -- Correlation and tracing
    correlation_id UUID,
    source_system VARCHAR(100),
    
    -- Metadata
    metadata JSONB DEFAULT '{}'
);

-- Vehicle commands table (for audit trail)
CREATE TABLE vehicle_commands (
    command_id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    vehicle_id UUID NOT NULL REFERENCES vehicles(vehicle_id) ON DELETE CASCADE,
    
    -- Command details
    command_type VARCHAR(100) NOT NULL,
    command_payload JSONB NOT NULL DEFAULT '{}',
    priority VARCHAR(50) DEFAULT 'normal' CHECK (priority IN ('low', 'normal', 'high', 'critical', 'emergency')),
    
    -- Execution context
    issued_by UUID NOT NULL,
    reason TEXT,
    reason_category VARCHAR(100),
    
    -- Status and results
    status VARCHAR(50) DEFAULT 'pending' CHECK (status IN ('pending', 'sent', 'acknowledged', 'executing', 'completed', 'failed', 'timeout', 'cancelled')),
    result JSONB,
    error_message TEXT,
    
    -- Timing
    issued_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    sent_at TIMESTAMP WITH TIME ZONE,
    acknowledged_at TIMESTAMP WITH TIME ZONE,
    completed_at TIMESTAMP WITH TIME ZONE,
    timeout_at TIMESTAMP WITH TIME ZONE,
    
    -- Correlation
    correlation_id UUID,
    request_id UUID,
    
    -- Metadata
    metadata JSONB DEFAULT '{}'
);

-- Move to telemetry schema
SET search_path TO fleet_telemetry, public;

-- Vehicle telemetry snapshots (periodic state snapshots)
CREATE TABLE vehicle_telemetry_snapshots (
    snapshot_id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    vehicle_id UUID NOT NULL,
    
    -- Timestamp
    snapshot_time TIMESTAMP WITH TIME ZONE NOT NULL,
    
    -- Location and movement
    location GEOMETRY(POINT, 4326),
    heading DECIMAL(5,2),
    speed DECIMAL(6,2),
    acceleration JSONB, -- x, y, z components
    
    -- Vehicle state
    operational_status VARCHAR(50),
    autonomy_status VARCHAR(50),
    autonomy_confidence DECIMAL(3,2),
    odd_compliance BOOLEAN,
    
    -- Power and health
    battery_level DECIMAL(5,2),
    fuel_level DECIMAL(5,2),
    health_score DECIMAL(3,2),
    
    -- Sensor health
    sensor_status JSONB DEFAULT '{}',
    
    -- Environmental context
    weather_conditions JSONB,
    traffic_conditions JSONB,
    
    -- Trip context
    current_trip_id UUID,
    trip_progress DECIMAL(5,2),
    
    -- Raw telemetry data (compressed)
    raw_telemetry JSONB,
    
    -- Metadata
    data_quality_score DECIMAL(3,2),
    metadata JSONB DEFAULT '{}'
);

-- Partition the telemetry table by time
SELECT create_hypertable('fleet_telemetry.vehicle_telemetry_snapshots', 'snapshot_time', if_not_exists => TRUE);

-- Move to audit schema
SET search_path TO fleet_audit, public;

-- Fleet audit log
CREATE TABLE fleet_audit_log (
    audit_id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    
    -- Entity information
    entity_type VARCHAR(50) NOT NULL CHECK (entity_type IN ('organization', 'fleet', 'vehicle', 'depot')),
    entity_id UUID NOT NULL,
    
    -- Change details
    operation VARCHAR(50) NOT NULL CHECK (operation IN ('CREATE', 'UPDATE', 'DELETE', 'ACTIVATE', 'DEACTIVATE', 'COMMISSION', 'DECOMMISSION')),
    old_values JSONB,
    new_values JSONB,
    changed_fields TEXT[],
    
    -- Actor information
    changed_by UUID NOT NULL,
    change_reason TEXT,
    change_category VARCHAR(100),
    
    -- Timestamps
    changed_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    
    -- Request context
    correlation_id UUID,
    request_id UUID,
    user_agent TEXT,
    ip_address INET,
    
    -- Cryptographic integrity
    signature TEXT,
    hash_chain_previous TEXT,
    
    -- Metadata
    metadata JSONB DEFAULT '{}'
);

-- Indexes

-- Organizations indexes
CREATE INDEX idx_organizations_sector ON fleet_core.organizations(sector);
CREATE INDEX idx_organizations_status ON fleet_core.organizations(status);
CREATE INDEX idx_organizations_created_at ON fleet_core.organizations(created_at DESC);

-- Fleets indexes
CREATE INDEX idx_fleets_organization_id ON fleet_core.fleets(organization_id);
CREATE INDEX idx_fleets_status ON fleet_core.fleets(status);
CREATE INDEX idx_fleets_fleet_type ON fleet_core.fleets(fleet_type);
CREATE INDEX idx_fleets_service_area ON fleet_core.fleets USING GIST(service_area);
CREATE INDEX idx_fleets_tags ON fleet_core.fleets USING GIN(tags);

-- Vehicles indexes
CREATE INDEX idx_vehicles_fleet_id ON fleet_core.vehicles(fleet_id);
CREATE INDEX idx_vehicles_organization_id ON fleet_core.vehicles(organization_id);
CREATE INDEX idx_vehicles_asset_tag ON fleet_core.vehicles(asset_tag);
CREATE INDEX idx_vehicles_operational_status ON fleet_core.vehicles(operational_status);
CREATE INDEX idx_vehicles_autonomy_level ON fleet_core.vehicles(autonomy_level);
CREATE INDEX idx_vehicles_autonomy_status ON fleet_core.vehicles(autonomy_status);
CREATE INDEX idx_vehicles_current_location ON fleet_core.vehicles USING GIST(current_location);
CREATE INDEX idx_vehicles_last_seen ON fleet_core.vehicles(last_seen DESC);
CREATE INDEX idx_vehicles_current_trip_id ON fleet_core.vehicles(current_trip_id) WHERE current_trip_id IS NOT NULL;
CREATE INDEX idx_vehicles_battery_level ON fleet_core.vehicles(battery_level);
CREATE INDEX idx_vehicles_health_score ON fleet_core.vehicles(health_score);
CREATE INDEX idx_vehicles_tags ON fleet_core.vehicles USING GIN(tags);
CREATE INDEX idx_vehicles_manufacturer_model ON fleet_core.vehicles(manufacturer, model);

-- Depots indexes
CREATE INDEX idx_depots_organization_id ON fleet_core.depots(organization_id);
CREATE INDEX idx_depots_fleet_id ON fleet_core.depots(fleet_id);
CREATE INDEX idx_depots_location ON fleet_core.depots USING GIST(location);
CREATE INDEX idx_depots_geofence ON fleet_core.depots USING GIST(geofence);
CREATE INDEX idx_depots_depot_type ON fleet_core.depots(depot_type);
CREATE INDEX idx_depots_status ON fleet_core.depots(status);

-- Vehicle alerts indexes
CREATE INDEX idx_vehicle_alerts_vehicle_id ON fleet_core.vehicle_alerts(vehicle_id);
CREATE INDEX idx_vehicle_alerts_severity ON fleet_core.vehicle_alerts(severity);
CREATE INDEX idx_vehicle_alerts_category ON fleet_core.vehicle_alerts(category);
CREATE INDEX idx_vehicle_alerts_status ON fleet_core.vehicle_alerts(status);
CREATE INDEX idx_vehicle_alerts_occurred_at ON fleet_core.vehicle_alerts(occurred_at DESC);
CREATE INDEX idx_vehicle_alerts_correlation_id ON fleet_core.vehicle_alerts(correlation_id);
CREATE INDEX idx_vehicle_alerts_trip_id ON fleet_core.vehicle_alerts(trip_id) WHERE trip_id IS NOT NULL;

-- Vehicle commands indexes
CREATE INDEX idx_vehicle_commands_vehicle_id ON fleet_core.vehicle_commands(vehicle_id);
CREATE INDEX idx_vehicle_commands_status ON fleet_core.vehicle_commands(status);
CREATE INDEX idx_vehicle_commands_priority ON fleet_core.vehicle_commands(priority);
CREATE INDEX idx_vehicle_commands_issued_at ON fleet_core.vehicle_commands(issued_at DESC);
CREATE INDEX idx_vehicle_commands_correlation_id ON fleet_core.vehicle_commands(correlation_id);

-- Telemetry indexes
CREATE INDEX idx_vehicle_telemetry_vehicle_id ON fleet_telemetry.vehicle_telemetry_snapshots(vehicle_id);
CREATE INDEX idx_vehicle_telemetry_location ON fleet_telemetry.vehicle_telemetry_snapshots USING GIST(location);
CREATE INDEX idx_vehicle_telemetry_trip_id ON fleet_telemetry.vehicle_telemetry_snapshots(current_trip_id) WHERE current_trip_id IS NOT NULL;

-- Audit indexes
CREATE INDEX idx_fleet_audit_entity ON fleet_audit.fleet_audit_log(entity_type, entity_id);
CREATE INDEX idx_fleet_audit_operation ON fleet_audit.fleet_audit_log(operation);
CREATE INDEX idx_fleet_audit_changed_by ON fleet_audit.fleet_audit_log(changed_by);
CREATE INDEX idx_fleet_audit_changed_at ON fleet_audit.fleet_audit_log(changed_at DESC);
CREATE INDEX idx_fleet_audit_correlation_id ON fleet_audit.fleet_audit_log(correlation_id);

-- Functions and triggers

-- Update timestamp function
CREATE OR REPLACE FUNCTION fleet_core.update_updated_at_column()
RETURNS TRIGGER AS $$
BEGIN
    NEW.updated_at = NOW();
    RETURN NEW;
END;
$$ language 'plpgsql';

-- Apply update timestamp triggers
CREATE TRIGGER update_organizations_updated_at BEFORE UPDATE ON fleet_core.organizations FOR EACH ROW EXECUTE FUNCTION fleet_core.update_updated_at_column();
CREATE TRIGGER update_fleets_updated_at BEFORE UPDATE ON fleet_core.fleets FOR EACH ROW EXECUTE FUNCTION fleet_core.update_updated_at_column();
CREATE TRIGGER update_vehicles_updated_at BEFORE UPDATE ON fleet_core.vehicles FOR EACH ROW EXECUTE FUNCTION fleet_core.update_updated_at_column();
CREATE TRIGGER update_depots_updated_at BEFORE UPDATE ON fleet_core.depots FOR EACH ROW EXECUTE FUNCTION fleet_core.update_updated_at_column();
CREATE TRIGGER update_vehicle_alerts_updated_at BEFORE UPDATE ON fleet_core.vehicle_alerts FOR EACH ROW EXECUTE FUNCTION fleet_core.update_updated_at_column();

-- Vehicle last_seen update function
CREATE OR REPLACE FUNCTION fleet_core.update_vehicle_last_seen()
RETURNS TRIGGER AS $$
BEGIN
    -- Update last_seen when location or status changes
    IF (OLD.current_location IS DISTINCT FROM NEW.current_location) OR 
       (OLD.operational_status IS DISTINCT FROM NEW.operational_status) OR
       (OLD.autonomy_status IS DISTINCT FROM NEW.autonomy_status) THEN
        NEW.last_seen = NOW();
    END IF;
    RETURN NEW;
END;
$$ language 'plpgsql';

CREATE TRIGGER update_vehicle_last_seen BEFORE UPDATE ON fleet_core.vehicles FOR EACH ROW EXECUTE FUNCTION fleet_core.update_vehicle_last_seen();

-- Audit logging function
CREATE OR REPLACE FUNCTION fleet_audit.create_audit_entry()
RETURNS TRIGGER AS $$
DECLARE
    operation_type TEXT;
    old_data JSONB;
    new_data JSONB;
    entity_type_val TEXT;
BEGIN
    -- Determine entity type from table name
    entity_type_val = CASE TG_TABLE_NAME
        WHEN 'organizations' THEN 'organization'
        WHEN 'fleets' THEN 'fleet'
        WHEN 'vehicles' THEN 'vehicle'
        WHEN 'depots' THEN 'depot'
        ELSE TG_TABLE_NAME
    END;
    
    -- Determine operation type
    IF TG_OP = 'INSERT' THEN
        operation_type = 'CREATE';
        old_data = NULL;
        new_data = to_jsonb(NEW);
    ELSIF TG_OP = 'UPDATE' THEN
        operation_type = 'UPDATE';
        old_data = to_jsonb(OLD);
        new_data = to_jsonb(NEW);
    ELSIF TG_OP = 'DELETE' THEN
        operation_type = 'DELETE';
        old_data = to_jsonb(OLD);
        new_data = NULL;
    END IF;
    
    -- Insert audit record
    INSERT INTO fleet_audit.fleet_audit_log (
        entity_type,
        entity_id,
        operation,
        old_values,
        new_values,
        changed_by,
        correlation_id
    ) VALUES (
        entity_type_val,
        CASE entity_type_val
            WHEN 'organization' THEN COALESCE(NEW.organization_id, OLD.organization_id)
            WHEN 'fleet' THEN COALESCE(NEW.fleet_id, OLD.fleet_id)
            WHEN 'vehicle' THEN COALESCE(NEW.vehicle_id, OLD.vehicle_id)
            WHEN 'depot' THEN COALESCE(NEW.depot_id, OLD.depot_id)
        END,
        operation_type,
        old_data,
        new_data,
        COALESCE(
            NEW.updated_by, NEW.created_by, 
            OLD.updated_by, OLD.created_by,
            gen_random_uuid()::text::uuid
        ),
        gen_random_uuid()
    );
    
    RETURN COALESCE(NEW, OLD);
END;
$$ language 'plpgsql';

-- Apply audit triggers
CREATE TRIGGER fleet_audit_organizations AFTER INSERT OR UPDATE OR DELETE ON fleet_core.organizations FOR EACH ROW EXECUTE FUNCTION fleet_audit.create_audit_entry();
CREATE TRIGGER fleet_audit_fleets AFTER INSERT OR UPDATE OR DELETE ON fleet_core.fleets FOR EACH ROW EXECUTE FUNCTION fleet_audit.create_audit_entry();
CREATE TRIGGER fleet_audit_vehicles AFTER INSERT OR UPDATE OR DELETE ON fleet_core.vehicles FOR EACH ROW EXECUTE FUNCTION fleet_audit.create_audit_entry();
CREATE TRIGGER fleet_audit_depots AFTER INSERT OR UPDATE OR DELETE ON fleet_core.depots FOR EACH ROW EXECUTE FUNCTION fleet_audit.create_audit_entry();

-- Views

-- Active vehicles view
CREATE VIEW fleet_core.active_vehicles AS
SELECT * FROM fleet_core.vehicles 
WHERE operational_status NOT IN ('decommissioned', 'offline')
AND last_seen >= NOW() - INTERVAL '1 hour';

-- Fleet summary view
CREATE VIEW fleet_core.fleet_summary AS
SELECT 
    f.fleet_id,
    f.name,
    f.fleet_type,
    f.status,
    COUNT(v.vehicle_id) as total_vehicles,
    COUNT(CASE WHEN v.operational_status = 'driving_av' THEN 1 END) as active_vehicles,
    COUNT(CASE WHEN v.operational_status = 'idle' THEN 1 END) as idle_vehicles,
    COUNT(CASE WHEN v.operational_status = 'maintenance' THEN 1 END) as maintenance_vehicles,
    COUNT(CASE WHEN v.operational_status = 'charging' THEN 1 END) as charging_vehicles,
    AVG(v.battery_level) as avg_battery_level,
    AVG(v.health_score) as avg_health_score
FROM fleet_core.fleets f
LEFT JOIN fleet_core.vehicles v ON f.fleet_id = v.fleet_id
GROUP BY f.fleet_id, f.name, f.fleet_type, f.status;

-- Critical alerts view
CREATE VIEW fleet_core.critical_alerts AS
SELECT 
    va.*,
    v.asset_tag,
    v.fleet_id
FROM fleet_core.vehicle_alerts va
JOIN fleet_core.vehicles v ON va.vehicle_id = v.vehicle_id
WHERE va.severity IN ('critical', 'high')
AND va.status = 'active'
ORDER BY va.occurred_at DESC;

-- Grant permissions
GRANT USAGE ON SCHEMA fleet_core TO fleet_manager_service;
GRANT USAGE ON SCHEMA fleet_telemetry TO fleet_manager_service;
GRANT USAGE ON SCHEMA fleet_audit TO fleet_manager_service;

GRANT SELECT, INSERT, UPDATE, DELETE ON ALL TABLES IN SCHEMA fleet_core TO fleet_manager_service;
GRANT SELECT, INSERT ON ALL TABLES IN SCHEMA fleet_telemetry TO fleet_manager_service;
GRANT SELECT, INSERT ON ALL TABLES IN SCHEMA fleet_audit TO fleet_manager_service;

GRANT USAGE, SELECT ON ALL SEQUENCES IN SCHEMA fleet_core TO fleet_manager_service;
GRANT USAGE, SELECT ON ALL SEQUENCES IN SCHEMA fleet_telemetry TO fleet_manager_service;
GRANT USAGE, SELECT ON ALL SEQUENCES IN SCHEMA fleet_audit TO fleet_manager_service;

-- Create service user
DO $$
BEGIN
    IF NOT EXISTS (SELECT 1 FROM pg_user WHERE usename = 'fleet_manager_service') THEN
        CREATE USER fleet_manager_service WITH PASSWORD 'secure_password_change_me';
    END IF;
END
$$;

-- Comments
COMMENT ON TABLE fleet_core.organizations IS 'Multi-tenant organizations with sector-specific configurations';
COMMENT ON TABLE fleet_core.fleets IS 'Fleet definitions with operational parameters and service areas';
COMMENT ON TABLE fleet_core.vehicles IS 'Core vehicle registry with real-time state and capabilities';
COMMENT ON TABLE fleet_core.depots IS 'Physical depot locations with capacity and service information';
COMMENT ON TABLE fleet_core.vehicle_alerts IS 'Real-time vehicle alerts and notifications';
COMMENT ON TABLE fleet_core.vehicle_commands IS 'Audit trail of commands sent to vehicles';
COMMENT ON TABLE fleet_telemetry.vehicle_telemetry_snapshots IS 'Time-series vehicle telemetry snapshots';
COMMENT ON TABLE fleet_audit.fleet_audit_log IS 'Cryptographically signed audit log of fleet changes';
