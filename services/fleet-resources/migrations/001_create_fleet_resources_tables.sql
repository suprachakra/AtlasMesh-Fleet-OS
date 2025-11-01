-- Fleet Resource Management Database Schema
-- Resource allocation, optimization, monitoring, and planning

-- Create fleet_resources schema
CREATE SCHEMA IF NOT EXISTS fleet_resources;

-- Fleet Resource Pools
CREATE TABLE fleet_resources.resource_pools (
    pool_id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    fleet_id UUID NOT NULL,
    resource_type VARCHAR(100) NOT NULL,
    total_capacity DECIMAL(10,4) NOT NULL,
    available_capacity DECIMAL(10,4) NOT NULL,
    reserved_capacity DECIMAL(10,4) NOT NULL DEFAULT 0,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);

-- Resource Allocations
CREATE TABLE fleet_resources.resource_allocations (
    allocation_id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    pool_id UUID REFERENCES fleet_resources.resource_pools(pool_id) ON DELETE CASCADE,
    vehicle_id UUID NOT NULL,
    allocation_type VARCHAR(100) NOT NULL,
    allocated_amount DECIMAL(10,4) NOT NULL,
    status VARCHAR(50) NOT NULL DEFAULT 'active',
    allocated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    released_at TIMESTAMP WITH TIME ZONE
);

-- Resource Requests
CREATE TABLE fleet_resources.resource_requests (
    request_id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    fleet_id UUID NOT NULL,
    vehicle_id UUID NOT NULL,
    resource_type VARCHAR(100) NOT NULL,
    requested_amount DECIMAL(10,4) NOT NULL,
    priority INTEGER NOT NULL DEFAULT 0,
    status VARCHAR(50) NOT NULL DEFAULT 'pending',
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    fulfilled_at TIMESTAMP WITH TIME ZONE
);

-- Resource Utilization
CREATE TABLE fleet_resources.resource_utilization (
    utilization_id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    pool_id UUID REFERENCES fleet_resources.resource_pools(pool_id) ON DELETE CASCADE,
    utilization_percentage DECIMAL(5,2) NOT NULL,
    peak_utilization DECIMAL(5,2) NOT NULL,
    average_utilization DECIMAL(5,2) NOT NULL,
    calculated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);

-- Resource Planning
CREATE TABLE fleet_resources.resource_planning (
    plan_id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    fleet_id UUID NOT NULL,
    planning_horizon INTEGER NOT NULL,
    planned_allocations JSONB NOT NULL DEFAULT '{}',
    status VARCHAR(50) NOT NULL DEFAULT 'draft',
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);

-- Resource Conflicts
CREATE TABLE fleet_resources.resource_conflicts (
    conflict_id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    pool_id UUID REFERENCES fleet_resources.resource_pools(pool_id) ON DELETE CASCADE,
    conflict_type VARCHAR(100) NOT NULL,
    description TEXT NOT NULL,
    resolution_status VARCHAR(50) NOT NULL DEFAULT 'open',
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    resolved_at TIMESTAMP WITH TIME ZONE
);

-- Indexes for performance
CREATE INDEX idx_resource_pools_fleet_id ON fleet_resources.resource_pools(fleet_id);
CREATE INDEX idx_resource_pools_resource_type ON fleet_resources.resource_pools(resource_type);
CREATE INDEX idx_resource_allocations_pool_id ON fleet_resources.resource_allocations(pool_id);
CREATE INDEX idx_resource_allocations_vehicle_id ON fleet_resources.resource_allocations(vehicle_id);
CREATE INDEX idx_resource_allocations_status ON fleet_resources.resource_allocations(status);
CREATE INDEX idx_resource_requests_fleet_id ON fleet_resources.resource_requests(fleet_id);
CREATE INDEX idx_resource_requests_status ON fleet_resources.resource_requests(status);
CREATE INDEX idx_resource_requests_priority ON fleet_resources.resource_requests(priority);
CREATE INDEX idx_resource_utilization_pool_id ON fleet_resources.resource_utilization(pool_id);
CREATE INDEX idx_resource_planning_fleet_id ON fleet_resources.resource_planning(fleet_id);
CREATE INDEX idx_resource_conflicts_pool_id ON fleet_resources.resource_conflicts(pool_id);
CREATE INDEX idx_resource_conflicts_resolution_status ON fleet_resources.resource_conflicts(resolution_status);

-- Row Level Security
ALTER TABLE fleet_resources.resource_pools ENABLE ROW LEVEL SECURITY;
ALTER TABLE fleet_resources.resource_allocations ENABLE ROW LEVEL SECURITY;
ALTER TABLE fleet_resources.resource_requests ENABLE ROW LEVEL SECURITY;
ALTER TABLE fleet_resources.resource_utilization ENABLE ROW LEVEL SECURITY;
ALTER TABLE fleet_resources.resource_planning ENABLE ROW LEVEL SECURITY;
ALTER TABLE fleet_resources.resource_conflicts ENABLE ROW LEVEL SECURITY;

-- Audit triggers
CREATE OR REPLACE FUNCTION fleet_resources.audit_trigger()
RETURNS TRIGGER AS $$
BEGIN
    NEW.updated_at = NOW();
    RETURN NEW;
END;
$$ LANGUAGE plpgsql;

CREATE TRIGGER resource_pools_audit
    BEFORE UPDATE ON fleet_resources.resource_pools
    FOR EACH ROW
    EXECUTE FUNCTION fleet_resources.audit_trigger();

CREATE TRIGGER resource_planning_audit
    BEFORE UPDATE ON fleet_resources.resource_planning
    FOR EACH ROW
    EXECUTE FUNCTION fleet_resources.audit_trigger();
