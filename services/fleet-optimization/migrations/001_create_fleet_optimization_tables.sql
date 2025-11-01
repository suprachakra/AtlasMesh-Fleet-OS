-- Fleet Optimization Database Schema
-- Multi-objective optimization and fleet rebalancing

-- Create fleet_optimization schema
CREATE SCHEMA IF NOT EXISTS fleet_optimization;

-- Fleet Optimization Runs
CREATE TABLE fleet_optimization.optimization_runs (
    optimization_id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    fleet_id UUID NOT NULL,
    optimization_type VARCHAR(100) NOT NULL,
    objectives JSONB NOT NULL DEFAULT '{}',
    constraints JSONB NOT NULL DEFAULT '{}',
    status VARCHAR(50) NOT NULL DEFAULT 'pending',
    results JSONB DEFAULT NULL,
    started_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    completed_at TIMESTAMP WITH TIME ZONE,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);

-- Fleet Rebalancing
CREATE TABLE fleet_optimization.fleet_rebalancing (
    rebalancing_id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    fleet_id UUID NOT NULL,
    rebalancing_type VARCHAR(100) NOT NULL,
    source_zone VARCHAR(100),
    target_zone VARCHAR(100),
    vehicle_count INTEGER NOT NULL,
    status VARCHAR(50) NOT NULL DEFAULT 'pending',
    started_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    completed_at TIMESTAMP WITH TIME ZONE,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);

-- Optimization Results
CREATE TABLE fleet_optimization.optimization_results (
    result_id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    optimization_id UUID REFERENCES fleet_optimization.optimization_runs(optimization_id) ON DELETE CASCADE,
    result_type VARCHAR(100) NOT NULL,
    result_value DECIMAL(10,4) NOT NULL,
    result_unit VARCHAR(50) NOT NULL,
    confidence_score DECIMAL(5,4) DEFAULT 0.0,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);

-- Optimization Recommendations
CREATE TABLE fleet_optimization.optimization_recommendations (
    recommendation_id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    optimization_id UUID REFERENCES fleet_optimization.optimization_runs(optimization_id) ON DELETE CASCADE,
    recommendation_type VARCHAR(100) NOT NULL,
    description TEXT NOT NULL,
    impact_level VARCHAR(20) NOT NULL,
    implementation_time VARCHAR(50) NOT NULL,
    priority INTEGER DEFAULT 0,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);

-- Optimization Metrics
CREATE TABLE fleet_optimization.optimization_metrics (
    metric_id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    fleet_id UUID NOT NULL,
    metric_type VARCHAR(100) NOT NULL,
    metric_value DECIMAL(10,4) NOT NULL,
    metric_unit VARCHAR(50) NOT NULL,
    calculated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);

-- Indexes for performance
CREATE INDEX idx_optimization_runs_fleet_id ON fleet_optimization.optimization_runs(fleet_id);
CREATE INDEX idx_optimization_runs_status ON fleet_optimization.optimization_runs(status);
CREATE INDEX idx_optimization_runs_started_at ON fleet_optimization.optimization_runs(started_at);
CREATE INDEX idx_fleet_rebalancing_fleet_id ON fleet_optimization.fleet_rebalancing(fleet_id);
CREATE INDEX idx_fleet_rebalancing_status ON fleet_optimization.fleet_rebalancing(status);
CREATE INDEX idx_optimization_results_optimization_id ON fleet_optimization.optimization_results(optimization_id);
CREATE INDEX idx_optimization_recommendations_optimization_id ON fleet_optimization.optimization_recommendations(optimization_id);
CREATE INDEX idx_optimization_metrics_fleet_id ON fleet_optimization.optimization_metrics(fleet_id);
CREATE INDEX idx_optimization_metrics_calculated_at ON fleet_optimization.optimization_metrics(calculated_at);

-- Row Level Security
ALTER TABLE fleet_optimization.optimization_runs ENABLE ROW LEVEL SECURITY;
ALTER TABLE fleet_optimization.fleet_rebalancing ENABLE ROW LEVEL SECURITY;
ALTER TABLE fleet_optimization.optimization_results ENABLE ROW LEVEL SECURITY;
ALTER TABLE fleet_optimization.optimization_recommendations ENABLE ROW LEVEL SECURITY;
ALTER TABLE fleet_optimization.optimization_metrics ENABLE ROW LEVEL SECURITY;
