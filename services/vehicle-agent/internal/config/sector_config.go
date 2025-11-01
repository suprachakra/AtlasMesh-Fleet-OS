package config

import (
	"fmt"
	"os"
	"path/filepath"
	"gopkg.in/yaml.v2"
)

// SectorType represents the different sectors supported
type SectorType string

const (
	SectorLogistics SectorType = "logistics"
	SectorDefense   SectorType = "defense"
	SectorMining    SectorType = "mining"
	SectorRideHail  SectorType = "ride_hail"
)

// SectorConfig represents the complete sector-specific configuration
type SectorConfig struct {
	Sector  SectorType           `yaml:"sector"`
	Services map[string]ServiceConfig `yaml:"services"`
}

// ServiceConfig represents configuration for a specific service
type ServiceConfig struct {
	Localization *LocalizationConfig `yaml:"localization,omitempty"`
	Perception   *PerceptionConfig   `yaml:"perception,omitempty"`
	Control      *ControlConfig      `yaml:"control,omitempty"`
	Safety       *SafetyConfig       `yaml:"safety,omitempty"`
}

// LocalizationConfig represents sector-specific localization configuration
type LocalizationConfig struct {
	Strategy     string                 `yaml:"strategy"`
	Accuracy     string                 `yaml:"accuracy"`
	Parameters   map[string]interface{} `yaml:"parameters"`
	Fallback     *FallbackConfig        `yaml:"fallback,omitempty"`
}

// PerceptionConfig represents sector-specific perception configuration
type PerceptionConfig struct {
	PalletDetection    bool                   `yaml:"pallet_detection,omitempty"`
	DockRecognition    bool                   `yaml:"dock_recognition,omitempty"`
	ThreatDetection    bool                   `yaml:"threat_detection,omitempty"`
	FormationAwareness bool                   `yaml:"formation_awareness,omitempty"`
	BlastZoneDetection bool                   `yaml:"blast_zone_detection,omitempty"`
	EquipmentRecognition bool                 `yaml:"equipment_recognition,omitempty"`
	PedestrianDetection bool                  `yaml:"pedestrian_detection,omitempty"`
	TrafficSignRecognition bool               `yaml:"traffic_sign_recognition,omitempty"`
	Parameters         map[string]interface{} `yaml:"parameters"`
}

// ControlConfig represents sector-specific control configuration
type ControlConfig struct {
	MaxSpeed           string                 `yaml:"max_speed"`
	Precision          string                 `yaml:"precision"`
	FormationControl   bool                   `yaml:"formation_control,omitempty"`
	ConvoyCoordination bool                   `yaml:"convoy_coordination,omitempty"`
	HeavyVehicleDynamics bool                `yaml:"heavy_vehicle_dynamics,omitempty"`
	SlopeHandling      bool                   `yaml:"slope_handling,omitempty"`
	SmoothRide         bool                   `yaml:"smooth_ride,omitempty"`
	TrafficCompliance  bool                   `yaml:"traffic_compliance,omitempty"`
	Parameters         map[string]interface{} `yaml:"parameters"`
}

// SafetyConfig represents sector-specific safety configuration
type SafetyConfig struct {
	EmergencyStop      bool                   `yaml:"emergency_stop"`
	CollisionAvoidance bool                   `yaml:"collision_avoidance"`
	FormationSafety    bool                   `yaml:"formation_safety,omitempty"`
	MissionSafety      bool                   `yaml:"mission_safety,omitempty"`
	MiningSafety       bool                   `yaml:"mining_safety,omitempty"`
	BlastZoneSafety    bool                   `yaml:"blast_zone_safety,omitempty"`
	PassengerSafety    bool                   `yaml:"passenger_safety,omitempty"`
	TrafficSafety      bool                   `yaml:"traffic_safety,omitempty"`
	Parameters         map[string]interface{} `yaml:"parameters"`
}

// FallbackConfig represents fallback configuration for localization
type FallbackConfig struct {
	Strategy string                 `yaml:"strategy"`
	Timeout  int                    `yaml:"timeout"`
	Parameters map[string]interface{} `yaml:"parameters"`
}

// LoadSectorConfig loads sector-specific configuration from YAML file
func LoadSectorConfig(sector SectorType) (*SectorConfig, error) {
	configPath := filepath.Join("config", "sectors", string(sector)+".yaml")
	
	// Check if file exists
	if _, err := os.Stat(configPath); os.IsNotExist(err) {
		return nil, fmt.Errorf("sector configuration file not found: %s", configPath)
	}
	
	// Read file
	data, err := os.ReadFile(configPath)
	if err != nil {
		return nil, fmt.Errorf("failed to read sector config file: %w", err)
	}
	
	// Parse YAML
	var config SectorConfig
	if err := yaml.Unmarshal(data, &config); err != nil {
		return nil, fmt.Errorf("failed to parse sector config YAML: %w", err)
	}
	
	// Validate sector type
	if config.Sector != sector {
		return nil, fmt.Errorf("sector mismatch: expected %s, got %s", sector, config.Sector)
	}
	
	return &config, nil
}

// GetServiceConfig returns configuration for a specific service
func (sc *SectorConfig) GetServiceConfig(serviceName string) (*ServiceConfig, error) {
	serviceConfig, exists := sc.Services[serviceName]
	if !exists {
		return nil, fmt.Errorf("service configuration not found: %s", serviceName)
	}
	return &serviceConfig, nil
}

// GetLocalizationConfig returns localization configuration for a service
func (sc *SectorConfig) GetLocalizationConfig(serviceName string) (*LocalizationConfig, error) {
	serviceConfig, err := sc.GetServiceConfig(serviceName)
	if err != nil {
		return nil, err
	}
	
	if serviceConfig.Localization == nil {
		return nil, fmt.Errorf("localization configuration not found for service: %s", serviceName)
	}
	
	return serviceConfig.Localization, nil
}

// GetPerceptionConfig returns perception configuration for a service
func (sc *SectorConfig) GetPerceptionConfig(serviceName string) (*PerceptionConfig, error) {
	serviceConfig, err := sc.GetServiceConfig(serviceName)
	if err != nil {
		return nil, err
	}
	
	if serviceConfig.Perception == nil {
		return nil, fmt.Errorf("perception configuration not found for service: %s", serviceName)
	}
	
	return serviceConfig.Perception, nil
}

// GetControlConfig returns control configuration for a service
func (sc *SectorConfig) GetControlConfig(serviceName string) (*ControlConfig, error) {
	serviceConfig, err := sc.GetServiceConfig(serviceName)
	if err != nil {
		return nil, err
	}
	
	if serviceConfig.Control == nil {
		return nil, fmt.Errorf("control configuration not found for service: %s", serviceName)
	}
	
	return serviceConfig.Control, nil
}

// GetSafetyConfig returns safety configuration for a service
func (sc *SectorConfig) GetSafetyConfig(serviceName string) (*SafetyConfig, error) {
	serviceConfig, err := sc.GetServiceConfig(serviceName)
	if err != nil {
		return nil, err
	}
	
	if serviceConfig.Safety == nil {
		return nil, fmt.Errorf("safety configuration not found for service: %s", serviceName)
	}
	
	return serviceConfig.Safety, nil
}

// ValidateSectorConfig validates the sector configuration
func (sc *SectorConfig) ValidateSectorConfig() error {
	if sc.Sector == "" {
		return fmt.Errorf("sector type is required")
	}
	
	// Validate sector type
	validSectors := []SectorType{SectorLogistics, SectorDefense, SectorMining, SectorRideHail}
	isValid := false
	for _, validSector := range validSectors {
		if sc.Sector == validSector {
			isValid = true
			break
		}
	}
	if !isValid {
		return fmt.Errorf("invalid sector type: %s", sc.Sector)
	}
	
	// Validate services
	if len(sc.Services) == 0 {
		return fmt.Errorf("at least one service configuration is required")
	}
	
	// Validate each service configuration
	for serviceName, serviceConfig := range sc.Services {
		if err := validateServiceConfig(serviceName, &serviceConfig); err != nil {
			return fmt.Errorf("invalid configuration for service %s: %w", serviceName, err)
		}
	}
	
	return nil
}

// validateServiceConfig validates a service configuration
func validateServiceConfig(serviceName string, config *ServiceConfig) error {
	// Validate localization config
	if config.Localization != nil {
		if config.Localization.Strategy == "" {
			return fmt.Errorf("localization strategy is required")
		}
		if config.Localization.Accuracy == "" {
			return fmt.Errorf("localization accuracy is required")
		}
	}
	
	// Validate perception config
	if config.Perception != nil {
		// Perception validation is sector-specific
		// This will be implemented based on sector requirements
	}
	
	// Validate control config
	if config.Control != nil {
		if config.Control.MaxSpeed == "" {
			return fmt.Errorf("control max speed is required")
		}
		if config.Control.Precision == "" {
			return fmt.Errorf("control precision is required")
		}
	}
	
	// Validate safety config
	if config.Safety != nil {
		if !config.Safety.EmergencyStop {
			return fmt.Errorf("emergency stop is required for safety")
		}
		if !config.Safety.CollisionAvoidance {
			return fmt.Errorf("collision avoidance is required for safety")
		}
	}
	
	return nil
}

// GetDefaultSectorConfig returns default configuration for a sector
func GetDefaultSectorConfig(sector SectorType) *SectorConfig {
	switch sector {
	case SectorLogistics:
		return getLogisticsDefaultConfig()
	case SectorDefense:
		return getDefenseDefaultConfig()
	case SectorMining:
		return getMiningDefaultConfig()
	case SectorRideHail:
		return getRideHailDefaultConfig()
	default:
		return getLogisticsDefaultConfig() // Default to logistics
	}
}

// getLogisticsDefaultConfig returns default logistics configuration
func getLogisticsDefaultConfig() *SectorConfig {
	return &SectorConfig{
		Sector: SectorLogistics,
		Services: map[string]ServiceConfig{
			"vehicle-agent": {
				Localization: &LocalizationConfig{
					Strategy: "gps_imu_fusion",
					Accuracy: "10cm",
					Parameters: map[string]interface{}{
						"gps_timeout": 5000,
						"imu_rate":    100,
					},
				},
				Perception: &PerceptionConfig{
					PalletDetection:    true,
					DockRecognition:    true,
					Parameters: map[string]interface{}{
						"detection_confidence": 0.95,
						"recognition_timeout":  2000,
					},
				},
				Control: &ControlConfig{
					MaxSpeed:  "3m/s",
					Precision: "5cm",
					Parameters: map[string]interface{}{
						"acceleration_limit": 2.0,
						"deceleration_limit": 3.0,
					},
				},
				Safety: &SafetyConfig{
					EmergencyStop:      true,
					CollisionAvoidance: true,
					Parameters: map[string]interface{}{
						"emergency_stop_time": 100,
						"collision_distance":  1.0,
					},
				},
			},
		},
	}
}

// getDefenseDefaultConfig returns default defense configuration
func getDefenseDefaultConfig() *SectorConfig {
	return &SectorConfig{
		Sector: SectorDefense,
		Services: map[string]ServiceConfig{
			"vehicle-agent": {
				Localization: &LocalizationConfig{
					Strategy: "lidar_slam_imu",
					Accuracy: "1m",
					Parameters: map[string]interface{}{
						"slam_iterations": 100,
						"loop_closure_threshold": 0.1,
					},
					Fallback: &FallbackConfig{
						Strategy: "dead_reckoning",
						Timeout:  10000,
					},
				},
				Perception: &PerceptionConfig{
					ThreatDetection:    true,
					FormationAwareness: true,
					Parameters: map[string]interface{}{
						"threat_confidence": 0.99,
						"formation_tolerance": 0.5,
					},
				},
				Control: &ControlConfig{
					MaxSpeed:           "5m/s",
					Precision:          "1m",
					FormationControl:   true,
					ConvoyCoordination: true,
					Parameters: map[string]interface{}{
						"formation_spacing": 2.0,
						"convoy_gap":        5.0,
					},
				},
				Safety: &SafetyConfig{
					EmergencyStop:      true,
					CollisionAvoidance: true,
					FormationSafety:    true,
					MissionSafety:      true,
					Parameters: map[string]interface{}{
						"emergency_stop_time": 50,
						"formation_safety_distance": 1.5,
					},
				},
			},
		},
	}
}

// getMiningDefaultConfig returns default mining configuration
func getMiningDefaultConfig() *SectorConfig {
	return &SectorConfig{
		Sector: SectorMining,
		Services: map[string]ServiceConfig{
			"vehicle-agent": {
				Localization: &LocalizationConfig{
					Strategy: "gps_imu_wheel_odometry",
					Accuracy: "50cm",
					Parameters: map[string]interface{}{
						"gps_timeout": 10000,
						"wheel_odometry_rate": 50,
					},
				},
				Perception: &PerceptionConfig{
					BlastZoneDetection:  true,
					EquipmentRecognition: true,
					Parameters: map[string]interface{}{
						"blast_zone_confidence": 1.0,
						"equipment_recognition_timeout": 5000,
					},
				},
				Control: &ControlConfig{
					MaxSpeed:             "15m/s",
					Precision:            "1m",
					HeavyVehicleDynamics: true,
					SlopeHandling:        true,
					Parameters: map[string]interface{}{
						"max_grade": 15.0,
						"load_factor": 1.2,
					},
				},
				Safety: &SafetyConfig{
					EmergencyStop:      true,
					CollisionAvoidance: true,
					MiningSafety:       true,
					BlastZoneSafety:    true,
					Parameters: map[string]interface{}{
						"emergency_stop_time": 200,
						"blast_zone_distance": 100.0,
					},
				},
			},
		},
	}
}

// getRideHailDefaultConfig returns default ride-hail configuration
func getRideHailDefaultConfig() *SectorConfig {
	return &SectorConfig{
		Sector: SectorRideHail,
		Services: map[string]ServiceConfig{
			"vehicle-agent": {
				Localization: &LocalizationConfig{
					Strategy: "gps_imu_hd_maps",
					Accuracy: "30cm",
					Parameters: map[string]interface{}{
						"hd_map_resolution": 0.1,
						"lane_accuracy": 0.2,
					},
				},
				Perception: &PerceptionConfig{
					PedestrianDetection:    true,
					TrafficSignRecognition: true,
					Parameters: map[string]interface{}{
						"pedestrian_confidence": 0.98,
						"traffic_sign_confidence": 0.95,
					},
				},
				Control: &ControlConfig{
					MaxSpeed:          "15m/s",
					Precision:         "30cm",
					SmoothRide:        true,
					TrafficCompliance: true,
					Parameters: map[string]interface{}{
						"max_acceleration": 2.0,
						"comfort_factor": 0.8,
					},
				},
				Safety: &SafetyConfig{
					EmergencyStop:      true,
					CollisionAvoidance: true,
					PassengerSafety:    true,
					TrafficSafety:      true,
					Parameters: map[string]interface{}{
						"emergency_stop_time": 100,
						"passenger_comfort_threshold": 0.5,
					},
				},
			},
		},
	}
}
