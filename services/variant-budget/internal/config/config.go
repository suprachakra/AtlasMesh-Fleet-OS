package config

import (
"fmt"
"time"

"github.com/spf13/viper"
)

type Config struct {
Port             int              `mapstructure:"port"`
Version          string           `mapstructure:"version"`
MonitoringInterval time.Duration  `mapstructure:"monitoring_interval"`
AnalyzerConfig   AnalyzerConfig   `mapstructure:"analyzer"`
BudgetConfig     BudgetConfig     `mapstructure:"budget"`
EnforcementConfig EnforcementConfig `mapstructure:"enforcement"`
}

type AnalyzerConfig struct {
RepoPath      string   `mapstructure:"repo_path"`
BaselineBranch string  `mapstructure:"baseline_branch"`
DiffTargets   []string `mapstructure:"diff_targets"`
}

type BudgetConfig struct {
VehicleBudgetLimit  float64 `mapstructure:"vehicle_budget_limit"`
SectorBudgetLimit   float64 `mapstructure:"sector_budget_limit"`
PlatformBudgetLimit float64 `mapstructure:"platform_budget_limit"`
TestBudgetLimit     float64 `mapstructure:"test_budget_limit"`
}

type EnforcementConfig struct {
Enabled         bool          `mapstructure:"enabled"`
CCBWebhookURL   string        `mapstructure:"ccb_webhook_url"`
MetricsEnabled  bool          `mapstructure:"metrics_enabled"`
AlertThresholds map[string]float64 `mapstructure:"alert_thresholds"`
}

func Load() (*Config, error) {
v := viper.New()
v.SetConfigName("variant-budget")
v.SetConfigType("yaml")
v.AddConfigPath(".")
v.AddConfigPath("./configs")
v.AddConfigPath("./services/variant-budget" )

v.SetDefault("port", 8093)
v.SetDefault("version", "dev")
v.SetDefault("monitoring_interval", "5m")
v.SetDefault("budget.vehicle_budget_limit", 5.0)
v.SetDefault("budget.sector_budget_limit", 5.0)
v.SetDefault("budget.platform_budget_limit", 5.0)
v.SetDefault("budget.test_budget_limit", 25.0)

if err := v.ReadInConfig(); err != nil {
return nil, fmt.Errorf("failed to read config: %w", err)
}

type rawConfig Config
var cfg rawConfig
if err := v.Unmarshal(&cfg); err != nil {
return nil, fmt.Errorf("failed to unmarshal config: %w", err)
}

return (*Config)(&cfg), nil
}
