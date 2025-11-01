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
	FleetManager *FleetManagerConfig `yaml:"fleet_manager,omitempty"`
	Dispatch     *DispatchConfig     `yaml:"dispatch,omitempty"`
	Analytics    *AnalyticsConfig    `yaml:"analytics,omitempty"`
	Energy       *EnergyConfig       `yaml:"energy,omitempty"`
	Safety       *SafetyConfig       `yaml:"safety,omitempty"`
	// Add other service-specific configurations here
}

// FleetManagerConfig holds fleet manager specific configuration
type FleetManagerConfig struct {
	MaxFleetSize        int                    `yaml:"max_fleet_size"`
	VehicleTypes        []string               `yaml:"vehicle_types"`
	BatteryManagement   *BatteryManagementConfig `yaml:"battery_management,omitempty"`
	ShiftScheduling     *ShiftSchedulingConfig   `yaml:"shift_scheduling,omitempty"`
	ChargingStations    *ChargingStationConfig   `yaml:"charging_stations,omitempty"`
	MissionReadiness    *MissionReadinessConfig  `yaml:"mission_readiness,omitempty"`
	Environmental       *EnvironmentalConfig     `yaml:"environmental,omitempty"`
	Parameters          map[string]interface{}   `yaml:"parameters,omitempty"`
}

// BatteryManagementConfig holds battery management configuration
type BatteryManagementConfig struct {
	Enabled             bool    `yaml:"enabled"`
	MinSOCForTask       int     `yaml:"min_soc_for_task"`
	MaxChargeTimeHours  int     `yaml:"max_charge_time_hours"`
	ChargingStrategy    string  `yaml:"charging_strategy"`
	BatteryHealthCheck  bool    `yaml:"battery_health_check"`
	Parameters          map[string]interface{} `yaml:"parameters,omitempty"`
}

// ShiftSchedulingConfig holds shift scheduling configuration
type ShiftSchedulingConfig struct {
	Enabled             bool    `yaml:"enabled"`
	ShiftDurationHours  int     `yaml:"shift_duration_hours"`
	BreakTimeMinutes    int     `yaml:"break_time_minutes"`
	MaxConsecutiveShifts int    `yaml:"max_consecutive_shifts"`
	Parameters          map[string]interface{} `yaml:"parameters,omitempty"`
}

// ChargingStationConfig holds charging station configuration
type ChargingStationConfig struct {
	Enabled             bool    `yaml:"enabled"`
	StationCount        int     `yaml:"station_count"`
	ChargingPowerKW     int     `yaml:"charging_power_kw"`
	QueueManagement     bool    `yaml:"queue_management"`
	Parameters          map[string]interface{} `yaml:"parameters,omitempty"`
}

// MissionReadinessConfig holds mission readiness configuration
type MissionReadinessConfig struct {
	Enabled             bool    `yaml:"enabled"`
	ReadinessChecks     []string `yaml:"readiness_checks"`
	MinFuelForMission   int     `yaml:"min_fuel_for_mission"`
	AmmunitionTracking  bool    `yaml:"ammunition_tracking"`
	Parameters          map[string]interface{} `yaml:"parameters,omitempty"`
}

// EnvironmentalConfig holds environmental configuration
type EnvironmentalConfig struct {
	Enabled             bool    `yaml:"enabled"`
	MaxTempC            int     `yaml:"max_temp_c"`
	MinVisibilityM      int     `yaml:"min_visibility_m"`
	MaxWindSpeedMPS     int     `yaml:"max_wind_speed_mps"`
	SandstormDetection  bool    `yaml:"sandstorm_detection"`
	Parameters          map[string]interface{} `yaml:"parameters,omitempty"`
}

// DispatchConfig holds dispatch specific configuration
type DispatchConfig struct {
	MaxConcurrentTasks  int                    `yaml:"max_concurrent_tasks"`
	TaskPrioritization  *TaskPrioritizationConfig `yaml:"task_prioritization,omitempty"`
	RouteOptimization   *RouteOptimizationConfig  `yaml:"route_optimization,omitempty"`
	LoadBalancing       *LoadBalancingConfig      `yaml:"load_balancing,omitempty"`
	Parameters          map[string]interface{}    `yaml:"parameters,omitempty"`
}

// TaskPrioritizationConfig holds task prioritization configuration
type TaskPrioritizationConfig struct {
	Enabled             bool    `yaml:"enabled"`
	PriorityLevels      int     `yaml:"priority_levels"`
	EmergencyOverride   bool    `yaml:"emergency_override"`
	Parameters          map[string]interface{} `yaml:"parameters,omitempty"`
}

// RouteOptimizationConfig holds route optimization configuration
type RouteOptimizationConfig struct {
	Enabled             bool    `yaml:"enabled"`
	Algorithm           string  `yaml:"algorithm"`
	ConsiderTraffic     bool    `yaml:"consider_traffic"`
	ConsiderWeather     bool    `yaml:"consider_weather"`
	Parameters          map[string]interface{} `yaml:"parameters,omitempty"`
}

// LoadBalancingConfig holds load balancing configuration
type LoadBalancingConfig struct {
	Enabled             bool    `yaml:"enabled"`
	Strategy            string  `yaml:"strategy"`
	MaxLoadPerVehicle   int     `yaml:"max_load_per_vehicle"`
	Parameters          map[string]interface{} `yaml:"parameters,omitempty"`
}

// AnalyticsConfig holds analytics specific configuration
type AnalyticsConfig struct {
	Enabled             bool    `yaml:"enabled"`
	MetricsCollection   *MetricsCollectionConfig `yaml:"metrics_collection,omitempty"`
	Reporting           *ReportingConfig         `yaml:"reporting,omitempty"`
	Parameters          map[string]interface{}   `yaml:"parameters,omitempty"`
}

// MetricsCollectionConfig holds metrics collection configuration
type MetricsCollectionConfig struct {
	Enabled             bool    `yaml:"enabled"`
	CollectionInterval  int     `yaml:"collection_interval_seconds"`
	RetentionDays       int     `yaml:"retention_days"`
	Parameters          map[string]interface{} `yaml:"parameters,omitempty"`
}

// ReportingConfig holds reporting configuration
type ReportingConfig struct {
	Enabled             bool    `yaml:"enabled"`
	ReportTypes         []string `yaml:"report_types"`
	ScheduleInterval    int     `yaml:"schedule_interval_hours"`
	Parameters          map[string]interface{} `yaml:"parameters,omitempty"`
}

// EnergyConfig holds energy specific configuration
type EnergyConfig struct {
	Enabled             bool    `yaml:"enabled"`
	SmartCharging       *SmartChargingConfig     `yaml:"smart_charging,omitempty"`
	V2G                 *V2GConfig               `yaml:"v2g,omitempty"`
	EnergyAwareRouting  *EnergyAwareRoutingConfig `yaml:"energy_aware_routing,omitempty"`
	Parameters          map[string]interface{}   `yaml:"parameters,omitempty"`
}

// SmartChargingConfig holds smart charging configuration
type SmartChargingConfig struct {
	Enabled             bool    `yaml:"enabled"`
	LoadBalancing       bool    `yaml:"load_balancing"`
	TimeOfUsePricing    bool    `yaml:"time_of_use_pricing"`
	Parameters          map[string]interface{} `yaml:"parameters,omitempty"`
}

// V2GConfig holds V2G configuration
type V2GConfig struct {
	Enabled             bool    `yaml:"enabled"`
	GridStabilization   bool    `yaml:"grid_stabilization"`
	PeakShaving         bool    `yaml:"peak_shaving"`
	Parameters          map[string]interface{} `yaml:"parameters,omitempty"`
}

// EnergyAwareRoutingConfig holds energy aware routing configuration
type EnergyAwareRoutingConfig struct {
	Enabled             bool    `yaml:"enabled"`
	ConsiderBatterySOC  bool    `yaml:"consider_battery_soc"`
	ChargingStops       bool    `yaml:"charging_stops"`
	Parameters          map[string]interface{} `yaml:"parameters,omitempty"`
}

// SafetyConfig holds safety specific configuration
type SafetyConfig struct {
	Enabled             bool    `yaml:"enabled"`
	EmergencyStop       *EmergencyStopConfig     `yaml:"emergency_stop,omitempty"`
	CollisionAvoidance  *CollisionAvoidanceConfig `yaml:"collision_avoidance,omitempty"`
	Geofencing          *GeofencingConfig         `yaml:"geofencing,omitempty"`
	Parameters          map[string]interface{}   `yaml:"parameters,omitempty"`
}

// EmergencyStopConfig holds emergency stop configuration
type EmergencyStopConfig struct {
	Enabled             bool    `yaml:"enabled"`
	ResponseTimeMS      int     `yaml:"response_time_ms"`
	OverrideCapability  bool    `yaml:"override_capability"`
	Parameters          map[string]interface{} `yaml:"parameters,omitempty"`
}

// CollisionAvoidanceConfig holds collision avoidance configuration
type CollisionAvoidanceConfig struct {
	Enabled             bool    `yaml:"enabled"`
	MinSafeDistance     float64 `yaml:"min_safe_distance_meters"`
	PredictionHorizon   int     `yaml:"prediction_horizon_seconds"`
	Parameters          map[string]interface{} `yaml:"parameters,omitempty"`
}

// GeofencingConfig holds geofencing configuration
type GeofencingConfig struct {
	Enabled             bool    `yaml:"enabled"`
	RestrictedZones     []string `yaml:"restricted_zones"`
	SpeedLimits         bool    `yaml:"speed_limits"`
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
				if baseServiceCfg.FleetManager != nil && cfg.Services[serviceName].FleetManager == nil {
					cfg.Services[serviceName].FleetManager = baseServiceCfg.FleetManager
				}
				if baseServiceCfg.Dispatch != nil && cfg.Services[serviceName].Dispatch == nil {
					cfg.Services[serviceName].Dispatch = baseServiceCfg.Dispatch
				}
				if baseServiceCfg.Analytics != nil && cfg.Services[serviceName].Analytics == nil {
					cfg.Services[serviceName].Analytics = baseServiceCfg.Analytics
				}
				if baseServiceCfg.Energy != nil && cfg.Services[serviceName].Energy == nil {
					cfg.Services[serviceName].Energy = baseServiceCfg.Energy
				}
				if baseServiceCfg.Safety != nil && cfg.Services[serviceName].Safety == nil {
					cfg.Services[serviceName].Safety = baseServiceCfg.Safety
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
				if serviceCfg.FleetManager != nil {
					currentServiceCfg.FleetManager = serviceCfg.FleetManager
				}
				if serviceCfg.Dispatch != nil {
					currentServiceCfg.Dispatch = serviceCfg.Dispatch
				}
				if serviceCfg.Analytics != nil {
					currentServiceCfg.Analytics = serviceCfg.Analytics
				}
				if serviceCfg.Energy != nil {
					currentServiceCfg.Energy = serviceCfg.Energy
				}
				if serviceCfg.Safety != nil {
					currentServiceCfg.Safety = serviceCfg.Safety
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
