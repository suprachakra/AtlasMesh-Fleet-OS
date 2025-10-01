-- AtlasMesh Policy Engine Database Schema
-- Migration 001: Create policies and audit tables

-- Enable UUID extension
CREATE EXTENSION IF NOT EXISTS "uuid-ossp";
CREATE EXTENSION IF NOT EXISTS "pgcrypto";

-- Create schemas
CREATE SCHEMA IF NOT EXISTS policy_engine;
CREATE SCHEMA IF NOT EXISTS audit;

-- Set search path
SET search_path TO policy_engine, public;

-- Policies table
CREATE TABLE policies (
    policy_id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    name VARCHAR(255) NOT NULL,
    description TEXT,
    version INTEGER NOT NULL DEFAULT 1,
    status VARCHAR(50) NOT NULL DEFAULT 'draft' CHECK (status IN ('draft', 'active', 'deprecated', 'archived')),
    
    -- Policy content
    policy_type VARCHAR(100) NOT NULL CHECK (policy_type IN (
        'safety', 'routing', 'speed_limit', 'geofence', 'weather', 
        'time_restriction', 'vehicle_capability', 'passenger_safety',
        'cargo_handling', 'emergency_response', 'maintenance', 'compliance'
    )),
    policy_content JSONB NOT NULL,
    rego_code TEXT, -- OPA Rego policy code
    
    -- Scope and applicability
    scope VARCHAR(100) NOT NULL DEFAULT 'global' CHECK (scope IN ('global', 'fleet', 'vehicle', 'route', 'zone')),
    scope_ids TEXT[], -- Array of IDs this policy applies to
    
    -- Conditions
    conditions JSONB DEFAULT '{}', -- When this policy applies
    priority INTEGER NOT NULL DEFAULT 100, -- Higher number = higher priority
    
    -- Metadata
    created_by UUID NOT NULL,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    updated_by UUID,
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    
    -- Validation and testing
    validation_rules JSONB DEFAULT '{}',
    test_cases JSONB DEFAULT '[]',
    
    -- Compliance and governance
    compliance_frameworks TEXT[] DEFAULT '{}', -- GDPR, CCPA, etc.
    approval_required BOOLEAN DEFAULT false,
    approved_by UUID,
    approved_at TIMESTAMP WITH TIME ZONE,
    
    -- Versioning
    parent_policy_id UUID REFERENCES policies(policy_id),
    is_latest_version BOOLEAN DEFAULT true,
    
    -- Tags and categorization
    tags TEXT[] DEFAULT '{}',
    metadata JSONB DEFAULT '{}'
);

-- Indexes for policies
CREATE INDEX idx_policies_status ON policies(status);
CREATE INDEX idx_policies_type ON policies(policy_type);
CREATE INDEX idx_policies_scope ON policies(scope);
CREATE INDEX idx_policies_priority ON policies(priority DESC);
CREATE INDEX idx_policies_created_at ON policies(created_at DESC);
CREATE INDEX idx_policies_scope_ids ON policies USING GIN(scope_ids);
CREATE INDEX idx_policies_tags ON policies USING GIN(tags);
CREATE INDEX idx_policies_content ON policies USING GIN(policy_content);
CREATE INDEX idx_policies_conditions ON policies USING GIN(conditions);
CREATE INDEX idx_policies_latest_version ON policies(is_latest_version) WHERE is_latest_version = true;

-- Policy evaluations table (for audit and performance tracking)
CREATE TABLE policy_evaluations (
    evaluation_id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    policy_id UUID NOT NULL REFERENCES policies(policy_id),
    
    -- Evaluation context
    vehicle_id UUID,
    trip_id UUID,
    fleet_id UUID,
    user_id UUID,
    
    -- Input data
    input_data JSONB NOT NULL,
    context_data JSONB DEFAULT '{}',
    
    -- Evaluation results
    result JSONB NOT NULL, -- The policy decision
    decision VARCHAR(50) NOT NULL CHECK (decision IN ('allow', 'deny', 'warn', 'modify')),
    confidence_score DECIMAL(3,2) CHECK (confidence_score >= 0 AND confidence_score <= 1),
    
    -- Performance metrics
    evaluation_time_ms INTEGER NOT NULL,
    cache_hit BOOLEAN DEFAULT false,
    
    -- Timestamps
    evaluated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    
    -- Correlation
    correlation_id UUID, -- For tracing across services
    request_id UUID,
    
    -- Metadata
    metadata JSONB DEFAULT '{}'
);

-- Indexes for policy evaluations
CREATE INDEX idx_policy_evaluations_policy_id ON policy_evaluations(policy_id);
CREATE INDEX idx_policy_evaluations_vehicle_id ON policy_evaluations(vehicle_id);
CREATE INDEX idx_policy_evaluations_trip_id ON policy_evaluations(trip_id);
CREATE INDEX idx_policy_evaluations_decision ON policy_evaluations(decision);
CREATE INDEX idx_policy_evaluations_evaluated_at ON policy_evaluations(evaluated_at DESC);
CREATE INDEX idx_policy_evaluations_correlation_id ON policy_evaluations(correlation_id);
CREATE INDEX idx_policy_evaluations_performance ON policy_evaluations(evaluation_time_ms);

-- Policy violations table
CREATE TABLE policy_violations (
    violation_id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    policy_id UUID NOT NULL REFERENCES policies(policy_id),
    evaluation_id UUID REFERENCES policy_evaluations(evaluation_id),
    
    -- Violation details
    violation_type VARCHAR(100) NOT NULL,
    severity VARCHAR(50) NOT NULL CHECK (severity IN ('low', 'medium', 'high', 'critical')),
    description TEXT NOT NULL,
    
    -- Context
    vehicle_id UUID,
    trip_id UUID,
    fleet_id UUID,
    location JSONB, -- Geographic location if applicable
    
    -- Resolution
    status VARCHAR(50) NOT NULL DEFAULT 'open' CHECK (status IN ('open', 'acknowledged', 'resolved', 'false_positive')),
    resolved_by UUID,
    resolved_at TIMESTAMP WITH TIME ZONE,
    resolution_notes TEXT,
    
    -- Timestamps
    occurred_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    detected_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    
    -- Correlation
    correlation_id UUID,
    
    -- Metadata
    metadata JSONB DEFAULT '{}'
);

-- Indexes for policy violations
CREATE INDEX idx_policy_violations_policy_id ON policy_violations(policy_id);
CREATE INDEX idx_policy_violations_severity ON policy_violations(severity);
CREATE INDEX idx_policy_violations_status ON policy_violations(status);
CREATE INDEX idx_policy_violations_vehicle_id ON policy_violations(vehicle_id);
CREATE INDEX idx_policy_violations_occurred_at ON policy_violations(occurred_at DESC);
CREATE INDEX idx_policy_violations_correlation_id ON policy_violations(correlation_id);

-- Move to audit schema for audit tables
SET search_path TO audit, public;

-- Audit log table for policy changes
CREATE TABLE policy_audit_log (
    audit_id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    policy_id UUID NOT NULL,
    
    -- Change details
    operation VARCHAR(50) NOT NULL CHECK (operation IN ('CREATE', 'UPDATE', 'DELETE', 'ACTIVATE', 'DEACTIVATE')),
    old_values JSONB,
    new_values JSONB,
    changed_fields TEXT[],
    
    -- Actor information
    changed_by UUID NOT NULL,
    change_reason TEXT,
    
    -- Timestamps
    changed_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    
    -- Request context
    correlation_id UUID,
    request_id UUID,
    user_agent TEXT,
    ip_address INET,
    
    -- Cryptographic integrity
    signature TEXT, -- Digital signature of the audit record
    hash_chain_previous TEXT, -- Previous audit record hash for integrity chain
    
    -- Metadata
    metadata JSONB DEFAULT '{}'
);

-- Indexes for audit log
CREATE INDEX idx_policy_audit_log_policy_id ON policy_audit_log(policy_id);
CREATE INDEX idx_policy_audit_log_operation ON policy_audit_log(operation);
CREATE INDEX idx_policy_audit_log_changed_by ON policy_audit_log(changed_by);
CREATE INDEX idx_policy_audit_log_changed_at ON policy_audit_log(changed_at DESC);
CREATE INDEX idx_policy_audit_log_correlation_id ON policy_audit_log(correlation_id);

-- Decision audit log for compliance
CREATE TABLE decision_audit_log (
    decision_id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    evaluation_id UUID REFERENCES policy_engine.policy_evaluations(evaluation_id),
    
    -- Decision context
    decision_type VARCHAR(100) NOT NULL,
    decision_maker VARCHAR(100) NOT NULL, -- 'system', 'operator', 'ai_model'
    decision_result JSONB NOT NULL,
    
    -- Input and reasoning
    input_data JSONB NOT NULL,
    reasoning TEXT,
    confidence_level DECIMAL(3,2),
    
    -- Actors
    system_user_id UUID,
    human_user_id UUID,
    
    -- Timestamps
    decided_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    
    -- Correlation and tracing
    correlation_id UUID NOT NULL,
    trace_id UUID,
    span_id UUID,
    
    -- Cryptographic integrity
    signature TEXT NOT NULL, -- Cryptographically signed decision
    hash_previous TEXT, -- Hash chain for tamper detection
    
    -- Compliance
    retention_until TIMESTAMP WITH TIME ZONE, -- When this record can be deleted
    legal_hold BOOLEAN DEFAULT false, -- Prevent deletion for legal reasons
    
    -- Metadata
    metadata JSONB DEFAULT '{}'
);

-- Indexes for decision audit log
CREATE INDEX idx_decision_audit_log_evaluation_id ON decision_audit_log(evaluation_id);
CREATE INDEX idx_decision_audit_log_decision_type ON decision_audit_log(decision_type);
CREATE INDEX idx_decision_audit_log_decided_at ON decision_audit_log(decided_at DESC);
CREATE INDEX idx_decision_audit_log_correlation_id ON decision_audit_log(correlation_id);
CREATE INDEX idx_decision_audit_log_retention ON decision_audit_log(retention_until) WHERE retention_until IS NOT NULL;
CREATE INDEX idx_decision_audit_log_legal_hold ON decision_audit_log(legal_hold) WHERE legal_hold = true;

-- Functions and triggers

-- Function to update the updated_at timestamp
CREATE OR REPLACE FUNCTION policy_engine.update_updated_at_column()
RETURNS TRIGGER AS $$
BEGIN
    NEW.updated_at = NOW();
    RETURN NEW;
END;
$$ language 'plpgsql';

-- Trigger to automatically update updated_at
CREATE TRIGGER update_policies_updated_at 
    BEFORE UPDATE ON policy_engine.policies 
    FOR EACH ROW 
    EXECUTE FUNCTION policy_engine.update_updated_at_column();

-- Function to maintain policy versioning
CREATE OR REPLACE FUNCTION policy_engine.create_policy_version()
RETURNS TRIGGER AS $$
BEGIN
    -- Mark previous versions as not latest
    UPDATE policy_engine.policies 
    SET is_latest_version = false 
    WHERE name = NEW.name 
    AND policy_id != NEW.policy_id;
    
    -- Ensure new version is marked as latest
    NEW.is_latest_version = true;
    
    RETURN NEW;
END;
$$ language 'plpgsql';

-- Trigger for policy versioning
CREATE TRIGGER maintain_policy_versions
    BEFORE INSERT ON policy_engine.policies
    FOR EACH ROW
    EXECUTE FUNCTION policy_engine.create_policy_version();

-- Function to create audit log entries
CREATE OR REPLACE FUNCTION audit.create_policy_audit_entry()
RETURNS TRIGGER AS $$
DECLARE
    operation_type TEXT;
    old_data JSONB;
    new_data JSONB;
BEGIN
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
    INSERT INTO audit.policy_audit_log (
        policy_id,
        operation,
        old_values,
        new_values,
        changed_by,
        correlation_id
    ) VALUES (
        COALESCE(NEW.policy_id, OLD.policy_id),
        operation_type,
        old_data,
        new_data,
        COALESCE(NEW.updated_by, NEW.created_by, OLD.updated_by, OLD.created_by),
        gen_random_uuid() -- Generate correlation ID if not provided
    );
    
    RETURN COALESCE(NEW, OLD);
END;
$$ language 'plpgsql';

-- Trigger for automatic audit logging
CREATE TRIGGER policy_audit_trigger
    AFTER INSERT OR UPDATE OR DELETE ON policy_engine.policies
    FOR EACH ROW
    EXECUTE FUNCTION audit.create_policy_audit_entry();

-- Views for common queries

-- Active policies view
CREATE VIEW policy_engine.active_policies AS
SELECT * FROM policy_engine.policies 
WHERE status = 'active' 
AND is_latest_version = true;

-- Policy performance metrics view
CREATE VIEW policy_engine.policy_performance AS
SELECT 
    p.policy_id,
    p.name,
    p.policy_type,
    COUNT(pe.evaluation_id) as total_evaluations,
    AVG(pe.evaluation_time_ms) as avg_evaluation_time_ms,
    COUNT(CASE WHEN pe.cache_hit THEN 1 END) as cache_hits,
    COUNT(CASE WHEN pe.decision = 'deny' THEN 1 END) as denials,
    COUNT(CASE WHEN pe.decision = 'allow' THEN 1 END) as approvals,
    AVG(pe.confidence_score) as avg_confidence
FROM policy_engine.policies p
LEFT JOIN policy_engine.policy_evaluations pe ON p.policy_id = pe.policy_id
WHERE p.status = 'active'
GROUP BY p.policy_id, p.name, p.policy_type;

-- Recent violations view
CREATE VIEW policy_engine.recent_violations AS
SELECT 
    pv.*,
    p.name as policy_name,
    p.policy_type
FROM policy_engine.policy_violations pv
JOIN policy_engine.policies p ON pv.policy_id = p.policy_id
WHERE pv.occurred_at >= NOW() - INTERVAL '24 hours'
ORDER BY pv.occurred_at DESC;

-- Grant permissions
GRANT USAGE ON SCHEMA policy_engine TO policy_engine_service;
GRANT USAGE ON SCHEMA audit TO policy_engine_service;

GRANT SELECT, INSERT, UPDATE, DELETE ON ALL TABLES IN SCHEMA policy_engine TO policy_engine_service;
GRANT SELECT, INSERT ON ALL TABLES IN SCHEMA audit TO policy_engine_service;

GRANT USAGE, SELECT ON ALL SEQUENCES IN SCHEMA policy_engine TO policy_engine_service;
GRANT USAGE, SELECT ON ALL SEQUENCES IN SCHEMA audit TO policy_engine_service;

-- Create service user if it doesn't exist
DO $$
BEGIN
    IF NOT EXISTS (SELECT 1 FROM pg_user WHERE usename = 'policy_engine_service') THEN
        CREATE USER policy_engine_service WITH PASSWORD 'secure_password_change_me';
    END IF;
END
$$;

-- Comments for documentation
COMMENT ON TABLE policy_engine.policies IS 'Core policies table storing OPA/Rego policies and metadata';
COMMENT ON TABLE policy_engine.policy_evaluations IS 'Audit trail of policy evaluations for performance and compliance';
COMMENT ON TABLE policy_engine.policy_violations IS 'Record of policy violations for monitoring and response';
COMMENT ON TABLE audit.policy_audit_log IS 'Cryptographically signed audit log of policy changes';
COMMENT ON TABLE audit.decision_audit_log IS 'Cryptographically signed audit log of automated decisions';

COMMENT ON COLUMN policy_engine.policies.rego_code IS 'OPA Rego policy code for evaluation engine';
COMMENT ON COLUMN policy_engine.policies.scope_ids IS 'Array of entity IDs this policy applies to (fleet_ids, vehicle_ids, etc.)';
COMMENT ON COLUMN policy_engine.policies.priority IS 'Policy priority for conflict resolution (higher number = higher priority)';
COMMENT ON COLUMN audit.decision_audit_log.signature IS 'Cryptographic signature ensuring decision integrity and non-repudiation';
