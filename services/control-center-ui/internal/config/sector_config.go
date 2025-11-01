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
	Dashboard   *DashboardConfig   `yaml:"dashboard,omitempty"`
	WebSocket   *WebSocketConfig   `yaml:"websocket,omitempty"`
	// Add other service-specific configurations here
}

// DashboardConfig holds dashboard specific configuration
type DashboardConfig struct {
	Layout          *LayoutConfig          `yaml:"layout,omitempty"`
	Widgets         *WidgetsConfig         `yaml:"widgets,omitempty"`
	RealTimeUpdates *RealTimeUpdatesConfig `yaml:"real_time_updates,omitempty"`
	Themes          *ThemesConfig          `yaml:"themes,omitempty"`
	Parameters      map[string]interface{} `yaml:"parameters,omitempty"`
}

// LayoutConfig holds dashboard layout configuration
type LayoutConfig struct {
	GridSize        int                    `yaml:"grid_size"`
	MaxColumns      int                    `yaml:"max_columns"`
	MaxRows         int                    `yaml:"max_rows"`
	Responsive      bool                   `yaml:"responsive"`
	MobileOptimized bool                   `yaml:"mobile_optimized"`
	Parameters      map[string]interface{} `yaml:"parameters,omitempty"`
}

// WidgetsConfig holds dashboard widgets configuration
type WidgetsConfig struct {
	FleetOverview    *WidgetConfig `yaml:"fleet_overview,omitempty"`
	VehicleStatus    *WidgetConfig `yaml:"vehicle_status,omitempty"`
	TaskQueue        *WidgetConfig `yaml:"task_queue,omitempty"`
	Performance      *WidgetConfig `yaml:"performance,omitempty"`
	Alerts           *WidgetConfig `yaml:"alerts,omitempty"`
	Maps             *WidgetConfig `yaml:"maps,omitempty"`
	Analytics        *WidgetConfig `yaml:"analytics,omitempty"`
	Parameters       map[string]interface{} `yaml:"parameters,omitempty"`
}

// WidgetConfig holds individual widget configuration
type WidgetConfig struct {
	Enabled         bool                   `yaml:"enabled"`
	Size            string                 `yaml:"size"`
	Position        *PositionConfig        `yaml:"position,omitempty"`
	RefreshInterval int                    `yaml:"refresh_interval_seconds"`
	Parameters      map[string]interface{} `yaml:"parameters,omitempty"`
}

// PositionConfig holds widget position configuration
type PositionConfig struct {
	X      int `yaml:"x"`
	Y      int `yaml:"y"`
	Width  int `yaml:"width"`
	Height int `yaml:"height"`
}

// RealTimeUpdatesConfig holds real-time updates configuration
type RealTimeUpdatesConfig struct {
	Enabled         bool                   `yaml:"enabled"`
	UpdateInterval  int                    `yaml:"update_interval_seconds"`
	WebSocketURL    string                 `yaml:"websocket_url"`
	AutoReconnect   bool                   `yaml:"auto_reconnect"`
	Parameters      map[string]interface{} `yaml:"parameters,omitempty"`
}

// ThemesConfig holds theme configuration
type ThemesConfig struct {
	DefaultTheme    string                 `yaml:"default_theme"`
	AvailableThemes []string               `yaml:"available_themes"`
	DarkMode        bool                   `yaml:"dark_mode"`
	Parameters      map[string]interface{} `yaml:"parameters,omitempty"`
}

// WebSocketConfig holds WebSocket specific configuration
type WebSocketConfig struct {
	Enabled         bool                   `yaml:"enabled"`
	Port            int                    `yaml:"port"`
	Path            string                 `yaml:"path"`
	MaxConnections  int                    `yaml:"max_connections"`
	PingInterval    int                    `yaml:"ping_interval_seconds"`
	PongTimeout     int                    `yaml:"pong_timeout_seconds"`
	Parameters      map[string]interface{} `yaml:"parameters,omitempty"`
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
				if baseServiceCfg.Dashboard != nil && cfg.Services[serviceName].Dashboard == nil {
					cfg.Services[serviceName].Dashboard = baseServiceCfg.Dashboard
				}
				if baseServiceCfg.WebSocket != nil && cfg.Services[serviceName].WebSocket == nil {
					cfg.Services[serviceName].WebSocket = baseServiceCfg.WebSocket
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
				if serviceCfg.Dashboard != nil {
					currentServiceCfg.Dashboard = serviceCfg.Dashboard
				}
				if serviceCfg.WebSocket != nil {
					currentServiceCfg.WebSocket = serviceCfg.WebSocket
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
