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
	SectorCore      SectorType = "core"
)

// SectorConfig represents the complete sector configuration
type SectorConfig struct {
	Sector       string                 `yaml:"sector"`
	Description  string                 `yaml:"description"`
	Services     map[string]interface{} `yaml:"services"`
	Parameters   map[string]interface{} `yaml:"parameters"`
}

// LoadSectorConfig loads the sector-specific configuration from a YAML file.
func LoadSectorConfig(sectorType SectorType) (*SectorConfig, error) {
	// Load sector configuration
	sectorConfigPath := filepath.Join("configs", "sectors", string(sectorType), "config.yaml")
	if _, err := os.Stat(sectorConfigPath); os.IsNotExist(err) {
		return nil, fmt.Errorf("sector configuration file not found: %s", sectorConfigPath)
	}

	sectorData, err := os.ReadFile(sectorConfigPath)
	if err != nil {
		return nil, fmt.Errorf("failed to read sector configuration file %s: %w", sectorConfigPath, err)
	}

	var sectorCfg SectorConfig
	if err := yaml.Unmarshal(sectorData, &sectorCfg); err != nil {
		return nil, fmt.Errorf("failed to unmarshal sector configuration file %s: %w", sectorConfigPath, err)
	}

	// Load base service configurations and merge
	baseConfigPath := filepath.Join("configs", "services", "base.yaml")
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
		sectorCfg = mergeConfigs(baseCfg, sectorCfg)
	}

	// Load sector-specific service configuration and merge
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
		sectorCfg = mergeConfigs(sectorCfg, sectorServiceCfg)
	}

	return &sectorCfg, nil
}

// mergeConfigs merges two sector configurations, with the second taking precedence
func mergeConfigs(base, override SectorConfig) SectorConfig {
	result := base

	// Override sector and description
	if override.Sector != "" {
		result.Sector = override.Sector
	}
	if override.Description != "" {
		result.Description = override.Description
	}

	// Merge services
	if result.Services == nil {
		result.Services = make(map[string]interface{})
	}
	for serviceName, serviceConfig := range override.Services {
		result.Services[serviceName] = serviceConfig
	}

	// Merge parameters
	if result.Parameters == nil {
		result.Parameters = make(map[string]interface{})
	}
	for paramName, paramValue := range override.Parameters {
		result.Parameters[paramName] = paramValue
	}

	return result
}

// GetServiceConfig retrieves the configuration for a specific service.
func (sc *SectorConfig) GetServiceConfig(serviceName string) (map[string]interface{}, error) {
	if serviceConfig, ok := sc.Services[serviceName]; ok {
		if configMap, ok := serviceConfig.(map[string]interface{}); ok {
			return configMap, nil
		}
		return nil, fmt.Errorf("service configuration for %s is not a valid map", serviceName)
	}
	return nil, fmt.Errorf("service configuration not found for %s in sector %s", serviceName, sc.Sector)
}

// GetParameter retrieves a parameter value by key.
func (sc *SectorConfig) GetParameter(key string) (interface{}, bool) {
	if sc.Parameters == nil {
		return nil, false
	}
	value, exists := sc.Parameters[key]
	return value, exists
}

// GetStringParameter retrieves a string parameter value by key.
func (sc *SectorConfig) GetStringParameter(key string) (string, bool) {
	value, exists := sc.GetParameter(key)
	if !exists {
		return "", false
	}
	if strValue, ok := value.(string); ok {
		return strValue, true
	}
	return "", false
}

// GetIntParameter retrieves an integer parameter value by key.
func (sc *SectorConfig) GetIntParameter(key string) (int, bool) {
	value, exists := sc.GetParameter(key)
	if !exists {
		return 0, false
	}
	if intValue, ok := value.(int); ok {
		return intValue, true
	}
	return 0, false
}

// GetBoolParameter retrieves a boolean parameter value by key.
func (sc *SectorConfig) GetBoolParameter(key string) (bool, bool) {
	value, exists := sc.GetParameter(key)
	if !exists {
		return false, false
	}
	if boolValue, ok := value.(bool); ok {
		return boolValue, true
	}
	return false, false
}

// ValidateSectorConfig performs validation on the loaded sector configuration.
func (sc *SectorConfig) ValidateSectorConfig() error {
	if sc.Sector == "" {
		return fmt.Errorf("sector type cannot be empty")
	}
	
	// Validate required services
	requiredServices := []string{"vehicle_agent", "fleet_manager", "dispatch_service", "control_center_ui"}
	for _, serviceName := range requiredServices {
		if _, exists := sc.Services[serviceName]; !exists {
			return fmt.Errorf("required service %s not found in sector configuration", serviceName)
		}
	}
	
	// Add more comprehensive validation rules here based on expected configurations
	return nil
}

// GetAvailableSectors returns a list of available sector types
func GetAvailableSectors() []SectorType {
	return []SectorType{
		SectorLogistics,
		SectorMining,
		SectorDefense,
		SectorRideHail,
		SectorCore,
	}
}

// IsValidSector checks if a sector type is valid
func IsValidSector(sectorType SectorType) bool {
	availableSectors := GetAvailableSectors()
	for _, sector := range availableSectors {
		if sector == sectorType {
			return true
		}
	}
	return false
}
