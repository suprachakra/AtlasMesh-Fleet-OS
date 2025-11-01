package config

import (
	"fmt"
	"os"
	"path/filepath"

	"gopkg.in/yaml.v3"
)

// SectorType defines the type of sector
type SectorType string

const (
	SectorLogistics SectorType = "logistics"
	SectorMining    SectorType = "mining"
	SectorDefense   SectorType = "defense"
	SectorRideHail  SectorType = "ride_hail"
	SectorCore      SectorType = "core" // For base configurations
)

// SectorConfig holds sector-specific configurations
type SectorConfig struct {
	Sector   SectorType             `yaml:"sector"`
	Services map[string]ServiceConfig `yaml:"services"`
}

// ServiceConfig holds configuration for a specific service within a sector
type ServiceConfig struct {
	Dispatch     *DispatchConfig     `yaml:"dispatch,omitempty"`
	Routing      *RoutingConfig      `yaml:"routing,omitempty"`
	Optimization *OptimizationConfig `yaml:"optimization,omitempty"`
	// Add other service-specific configurations here
}

// DispatchConfig holds dispatch specific configuration
type DispatchConfig struct {
	MaxConcurrentTasks  int                    `yaml:"max_concurrent_tasks"`
	TaskPrioritization  *TaskPrioritizationConfig `yaml:"task_prioritization,omitempty"`
	LoadBalancing       *LoadBalancingConfig      `yaml:"load_balancing,omitempty"`
	QueueManagement     *QueueManagementConfig    `yaml:"queue_management,omitempty"`
	Parameters          map[string]interface{}    `yaml:"parameters,omitempty"`
}

// TaskPrioritizationConfig holds task prioritization configuration
type TaskPrioritizationConfig struct {
	Enabled             bool    `yaml:"enabled"`
	PriorityLevels      int     `yaml:"priority_levels"`
	EmergencyOverride   bool    `yaml:"emergency_override"`
	DynamicPricing      bool    `yaml:"dynamic_pricing"`
	Parameters          map[string]interface{} `yaml:"parameters,omitempty"`
}

// LoadBalancingConfig holds load balancing configuration
type LoadBalancingConfig struct {
	Enabled             bool    `yaml:"enabled"`
	Strategy            string  `yaml:"strategy"`
	MaxLoadPerVehicle   int     `yaml:"max_load_per_vehicle"`
	CapacityManagement  bool    `yaml:"capacity_management"`
	Parameters          map[string]interface{} `yaml:"parameters,omitempty"`
}

// QueueManagementConfig holds queue management configuration
type QueueManagementConfig struct {
	Enabled             bool    `yaml:"enabled"`
	MaxQueueSize        int     `yaml:"max_queue_size"`
	QueueTimeoutMinutes int     `yaml:"queue_timeout_minutes"`
	FairScheduling      bool    `yaml:"fair_scheduling"`
	Parameters          map[string]interface{} `yaml:"parameters,omitempty"`
}

// RoutingConfig holds routing specific configuration
type RoutingConfig struct {
	Algorithm           string                    `yaml:"algorithm"`
	ConsiderTraffic     bool                      `yaml:"consider_traffic"`
	ConsiderWeather     bool                      `yaml:"consider_weather"`
	RealTimeUpdates     bool                      `yaml:"real_time_updates"`
	MultiStopRouting    *MultiStopRoutingConfig   `yaml:"multi_stop_routing,omitempty"`
	DynamicRouting      *DynamicRoutingConfig     `yaml:"dynamic_routing,omitempty"`
	Parameters          map[string]interface{}    `yaml:"parameters,omitempty"`
}

// MultiStopRoutingConfig holds multi-stop routing configuration
type MultiStopRoutingConfig struct {
	Enabled             bool    `yaml:"enabled"`
	MaxStopsPerRoute    int     `yaml:"max_stops_per_route"`
	OptimizationLevel   string  `yaml:"optimization_level"`
	Parameters          map[string]interface{} `yaml:"parameters,omitempty"`
}

// DynamicRoutingConfig holds dynamic routing configuration
type DynamicRoutingConfig struct {
	Enabled             bool    `yaml:"enabled"`
	UpdateFrequency     int     `yaml:"update_frequency_seconds"`
	RerouteThreshold    float64 `yaml:"reroute_threshold"`
	Parameters          map[string]interface{} `yaml:"parameters,omitempty"`
}

// OptimizationConfig holds optimization specific configuration
type OptimizationConfig struct {
	Algorithm           string                    `yaml:"algorithm"`
	OptimizationGoals   []string                  `yaml:"optimization_goals"`
	Constraints         *ConstraintsConfig        `yaml:"constraints,omitempty"`
	PerformanceTuning   *PerformanceTuningConfig  `yaml:"performance_tuning,omitempty"`
	Parameters          map[string]interface{}    `yaml:"parameters,omitempty"`
}

// ConstraintsConfig holds optimization constraints configuration
type ConstraintsConfig struct {
	TimeWindows         bool    `yaml:"time_windows"`
	CapacityLimits      bool    `yaml:"capacity_limits"`
	VehicleCompatibility bool   `yaml:"vehicle_compatibility"`
	Parameters          map[string]interface{} `yaml:"parameters,omitempty"`
}

// PerformanceTuningConfig holds performance tuning configuration
type PerformanceTuningConfig struct {
	MaxOptimizationTime int     `yaml:"max_optimization_time_seconds"`
	ParallelProcessing  bool    `yaml:"parallel_processing"`
	CacheResults        bool    `yaml:"cache_results"`
	Parameters          map[string]interface{} `yaml:"parameters,omitempty"`
}

// LoadSectorConfig loads the sector-specific configuration from a YAML file.
func LoadSectorConfig(sectorType SectorType) (*SectorConfig, error) {
	configPath := filepath.Join("config", "sectors", fmt.Sprintf("%s.yaml", string(sectorType)))
	if _, err := os.Stat(configPath); os.IsNotExist(err) {
		return nil, fmt.Errorf("sector configuration file not found: %s", configPath)
	}

	data, err := os.ReadFile(configPath)
	if err != nil {
		return nil, fmt.Errorf("failed to read sector configuration file %s: %w", configPath, err)
	}

	var cfg SectorConfig
	if err := yaml.Unmarshal(data, &cfg); err != nil {
		return nil, fmt.Errorf("failed to unmarshal sector configuration file %s: %w", configPath, err)
	}
	cfg.Sector = sectorType // Ensure the sector type is correctly set

	// Load base service configurations and merge
	baseConfigPath := filepath.Join("config", "services", "base.yaml")
	if _, err := os.Stat(baseConfigPath); !os.IsNotExist(err) {
		baseData, err := os.ReadFile(baseConfigPath)
		if err != nil {
			return nil, fmt.Errorf("failed to read base service configuration file %s: %w", baseConfigPath, err)
		}
		var baseCfg SectorConfig
		if err := yaml.Unmarshal(baseData, &baseCfg); err != nil {
			return nil, fmt.Errorf("failed to unmarshal base service configuration file %s: %w", baseConfigPath, err)
		}
		// Merge base config into sector config (sector-specific overrides base)
		for serviceName, baseServiceCfg := range baseCfg.Services {
			if _, ok := cfg.Services[serviceName]; !ok {
				cfg.Services[serviceName] = baseServiceCfg
			} else {
				// Merge individual fields if they exist in base but not in sector
				if baseServiceCfg.Dispatch != nil && cfg.Services[serviceName].Dispatch == nil {
					cfg.Services[serviceName].Dispatch = baseServiceCfg.Dispatch
				}
				if baseServiceCfg.Routing != nil && cfg.Services[serviceName].Routing == nil {
					cfg.Services[serviceName].Routing = baseServiceCfg.Routing
				}
				if baseServiceCfg.Optimization != nil && cfg.Services[serviceName].Optimization == nil {
					cfg.Services[serviceName].Optimization = baseServiceCfg.Optimization
				}
			}
		}
	}

	// Load service-specific configuration for the current sector
	sectorServiceConfigPath := filepath.Join("config", "services", fmt.Sprintf("%s.yaml", string(sectorType)))
	if _, err := os.Stat(sectorServiceConfigPath); !os.IsNotExist(err) {
		sectorServiceData, err := os.ReadFile(sectorServiceConfigPath)
		if err != nil {
			return nil, fmt.Errorf("failed to read sector-specific service configuration file %s: %w", sectorServiceConfigPath, err)
		}
		var sectorServiceCfg SectorConfig
		if err := yaml.Unmarshal(sectorServiceData, &sectorServiceCfg); err != nil {
			return nil, fmt.Errorf("failed to unmarshal sector-specific service configuration file %s: %w", sectorServiceConfigPath, err)
		}
		// Merge sector-specific service config (overrides base and general sector config)
		for serviceName, serviceCfg := range sectorServiceCfg.Services {
			if _, ok := cfg.Services[serviceName]; !ok {
				cfg.Services[serviceName] = serviceCfg
			} else {
				// Deep merge for existing services
				currentServiceCfg := cfg.Services[serviceName]
				if serviceCfg.Dispatch != nil {
					currentServiceCfg.Dispatch = serviceCfg.Dispatch
				}
				if serviceCfg.Routing != nil {
					currentServiceCfg.Routing = serviceCfg.Routing
				}
				if serviceCfg.Optimization != nil {
					currentServiceCfg.Optimization = serviceCfg.Optimization
				}
				cfg.Services[serviceName] = currentServiceCfg
			}
		}
	}

	return &cfg, nil
}

// GetServiceConfig retrieves the configuration for a specific service.
func (sc *SectorConfig) GetServiceConfig(serviceName string) (*ServiceConfig, error) {
	if svcConfig, ok := sc.Services[serviceName]; ok {
		return &svcConfig, nil
	}
	return nil, fmt.Errorf("service configuration not found for %s in sector %s", serviceName, sc.Sector)
}

// ValidateSectorConfig performs validation on the loaded sector configuration.
func (sc *SectorConfig) ValidateSectorConfig() error {
	if sc.Sector == "" {
		return fmt.Errorf("sector type cannot be empty")
	}
	// Add more comprehensive validation rules here based on expected configurations
	return nil
}
