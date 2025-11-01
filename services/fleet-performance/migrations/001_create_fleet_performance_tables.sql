-- Fleet Performance Management Database Schema
-- Performance monitoring, optimization, reporting, and benchmarking

-- Create fleet_performance schema
CREATE SCHEMA IF NOT EXISTS fleet_performance;

-- Fleet Performance Scores
CREATE TABLE fleet_performance.fleet_performance_scores (
    score_id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    fleet_id UUID NOT NULL,
    performance_score DECIMAL(5,2) NOT NULL,
    components JSONB NOT NULL DEFAULT '{}',
    benchmark_ranking INTEGER,
    calculated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);

-- Fleet Performance Metrics
CREATE TABLE fleet_performance.fleet_performance_metrics (
    metric_id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    fleet_id UUID NOT NULL,
    metric_type VARCHAR(100) NOT NULL,
    metric_value DECIMAL(10,4) NOT NULL,
    metric_unit VARCHAR(50) NOT NULL,
    benchmark_value DECIMAL(10,4),
    performance_ratio DECIMAL(5,4),
    recorded_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);

-- Fleet Benchmarks
CREATE TABLE fleet_performance.fleet_benchmarks (
    benchmark_id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    benchmark_name VARCHAR(255) NOT NULL,
    benchmark_type VARCHAR(100) NOT NULL,
    benchmark_value DECIMAL(10,4) NOT NULL,
    benchmark_unit VARCHAR(50) NOT NULL,
    industry_standard BOOLEAN DEFAULT FALSE,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);

-- Fleet Performance Reports
CREATE TABLE fleet_performance.fleet_performance_reports (
    report_id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    fleet_id UUID NOT NULL,
    report_type VARCHAR(100) NOT NULL,
    report_period VARCHAR(50) NOT NULL,
    report_data JSONB NOT NULL DEFAULT '{}',
    generated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);

-- Fleet Performance Alerts
CREATE TABLE fleet_performance.fleet_performance_alerts (
    alert_id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    fleet_id UUID NOT NULL,
    alert_type VARCHAR(100) NOT NULL,
    alert_level VARCHAR(20) NOT NULL,
    description TEXT NOT NULL,
    threshold_value DECIMAL(10,4) NOT NULL,
    actual_value DECIMAL(10,4) NOT NULL,
    status VARCHAR(50) NOT NULL DEFAULT 'open',
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    resolved_at TIMESTAMP WITH TIME ZONE
);

-- Fleet Performance Trends
CREATE TABLE fleet_performance.fleet_performance_trends (
    trend_id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    fleet_id UUID NOT NULL,
    trend_type VARCHAR(100) NOT NULL,
    trend_direction VARCHAR(20) NOT NULL,
    trend_value DECIMAL(10,4) NOT NULL,
    trend_period VARCHAR(50) NOT NULL,
    calculated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);

-- Indexes for performance
CREATE INDEX idx_fleet_performance_scores_fleet_id ON fleet_performance.fleet_performance_scores(fleet_id);
CREATE INDEX idx_fleet_performance_scores_calculated_at ON fleet_performance.fleet_performance_scores(calculated_at);
CREATE INDEX idx_fleet_performance_metrics_fleet_id ON fleet_performance.fleet_performance_metrics(fleet_id);
CREATE INDEX idx_fleet_performance_metrics_metric_type ON fleet_performance.fleet_performance_metrics(metric_type);
CREATE INDEX idx_fleet_benchmarks_benchmark_type ON fleet_performance.fleet_benchmarks(benchmark_type);
CREATE INDEX idx_fleet_performance_reports_fleet_id ON fleet_performance.fleet_performance_reports(fleet_id);
CREATE INDEX idx_fleet_performance_reports_report_type ON fleet_performance.fleet_performance_reports(report_type);
CREATE INDEX idx_fleet_performance_alerts_fleet_id ON fleet_performance.fleet_performance_alerts(fleet_id);
CREATE INDEX idx_fleet_performance_alerts_status ON fleet_performance.fleet_performance_alerts(status);
CREATE INDEX idx_fleet_performance_trends_fleet_id ON fleet_performance.fleet_performance_trends(fleet_id);

-- Row Level Security
ALTER TABLE fleet_performance.fleet_performance_scores ENABLE ROW LEVEL SECURITY;
ALTER TABLE fleet_performance.fleet_performance_metrics ENABLE ROW LEVEL SECURITY;
ALTER TABLE fleet_performance.fleet_benchmarks ENABLE ROW LEVEL SECURITY;
ALTER TABLE fleet_performance.fleet_performance_reports ENABLE ROW LEVEL SECURITY;
ALTER TABLE fleet_performance.fleet_performance_alerts ENABLE ROW LEVEL SECURITY;
ALTER TABLE fleet_performance.fleet_performance_trends ENABLE ROW LEVEL SECURITY;
