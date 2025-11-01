-- Fleet Coordination Database Schema
-- Multi-fleet coordination and federation management

-- Create fleet_coordination schema
CREATE SCHEMA IF NOT EXISTS fleet_coordination;

-- Fleet Federations
CREATE TABLE fleet_coordination.fleet_federations (
    federation_id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    name VARCHAR(255) NOT NULL,
    description TEXT,
    status VARCHAR(50) NOT NULL DEFAULT 'active',
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);

-- Fleet Federation Members
CREATE TABLE fleet_coordination.fleet_federation_members (
    federation_id UUID REFERENCES fleet_coordination.fleet_federations(federation_id) ON DELETE CASCADE,
    fleet_id UUID NOT NULL,
    isolation_level VARCHAR(50) NOT NULL DEFAULT 'high',
    resource_sharing BOOLEAN DEFAULT FALSE,
    communication_enabled BOOLEAN DEFAULT TRUE,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    PRIMARY KEY (federation_id, fleet_id)
);

-- Cross-Fleet Resource Sharing
CREATE TABLE fleet_coordination.resource_sharing (
    sharing_id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    source_fleet_id UUID NOT NULL,
    target_fleet_id UUID NOT NULL,
    resource_type VARCHAR(100) NOT NULL,
    resource_id UUID NOT NULL,
    duration INTEGER NOT NULL,
    priority VARCHAR(50) NOT NULL,
    status VARCHAR(50) NOT NULL DEFAULT 'pending',
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);

-- Fleet Coordination Metrics
CREATE TABLE fleet_coordination.coordination_metrics (
    federation_id UUID REFERENCES fleet_coordination.fleet_federations(federation_id) ON DELETE CASCADE,
    metric_type VARCHAR(100) NOT NULL,
    metric_value DECIMAL(10,4) NOT NULL,
    metric_unit VARCHAR(50) NOT NULL,
    calculated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);

-- Fleet Communication Logs
CREATE TABLE fleet_coordination.communication_logs (
    log_id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    source_fleet_id UUID NOT NULL,
    target_fleet_id UUID NOT NULL,
    message_type VARCHAR(100) NOT NULL,
    message_content JSONB NOT NULL,
    status VARCHAR(50) NOT NULL,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);

-- Indexes for performance
CREATE INDEX idx_fleet_federations_status ON fleet_coordination.fleet_federations(status);
CREATE INDEX idx_fleet_federation_members_fleet_id ON fleet_coordination.fleet_federation_members(fleet_id);
CREATE INDEX idx_resource_sharing_status ON fleet_coordination.resource_sharing(status);
CREATE INDEX idx_resource_sharing_created_at ON fleet_coordination.resource_sharing(created_at);
CREATE INDEX idx_coordination_metrics_federation_id ON fleet_coordination.coordination_metrics(federation_id);
CREATE INDEX idx_communication_logs_created_at ON fleet_coordination.communication_logs(created_at);

-- Row Level Security
ALTER TABLE fleet_coordination.fleet_federations ENABLE ROW LEVEL SECURITY;
ALTER TABLE fleet_coordination.fleet_federation_members ENABLE ROW LEVEL SECURITY;
ALTER TABLE fleet_coordination.resource_sharing ENABLE ROW LEVEL SECURITY;
ALTER TABLE fleet_coordination.coordination_metrics ENABLE ROW LEVEL SECURITY;
ALTER TABLE fleet_coordination.communication_logs ENABLE ROW LEVEL SECURITY;

-- Audit triggers
CREATE OR REPLACE FUNCTION fleet_coordination.audit_trigger()
RETURNS TRIGGER AS $$
BEGIN
    NEW.updated_at = NOW();
    RETURN NEW;
END;
$$ LANGUAGE plpgsql;

CREATE TRIGGER fleet_federations_audit
    BEFORE UPDATE ON fleet_coordination.fleet_federations
    FOR EACH ROW
    EXECUTE FUNCTION fleet_coordination.audit_trigger();

CREATE TRIGGER resource_sharing_audit
    BEFORE UPDATE ON fleet_coordination.resource_sharing
    FOR EACH ROW
    EXECUTE FUNCTION fleet_coordination.audit_trigger();
