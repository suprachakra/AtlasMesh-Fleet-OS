package sector_tests

import (
	"testing"
	"os"
	"atlasmesh/config"
)

// TestDefenseSectorConfiguration tests the defense sector configuration
func TestDefenseSectorConfiguration(t *testing.T) {
	// Set environment variable for defense sector
	os.Setenv("ATLASMESH_SECTOR", "defense")
	defer os.Unsetenv("ATLASMESH_SECTOR")

	// Load defense sector configuration
	sectorConfig, err := config.LoadSectorConfig(config.SectorDefense)
	if err != nil {
		t.Fatalf("Failed to load defense sector configuration: %v", err)
	}

	// Validate sector configuration
	if err := sectorConfig.ValidateSectorConfig(); err != nil {
		t.Fatalf("Invalid defense sector configuration: %v", err)
	}

	// Test sector-specific parameters
	t.Run("DefenseParameters", func(t *testing.T) {
		// Test operational zones
		operationalZones, exists := sectorConfig.GetParameter("operational_zones")
		if !exists {
			t.Error("operational_zones parameter not found")
		}
		if zones, ok := operationalZones.([]string); !ok || len(zones) == 0 {
			t.Error("operational_zones should be a non-empty string array")
		}

		// Test outdoor operations
		outdoorOps, exists := sectorConfig.GetBoolParameter("outdoor_operations")
		if !exists || !outdoorOps {
			t.Error("outdoor_operations should be true for defense sector")
		}

		// Test encrypted communications
		encryptedComms, exists := sectorConfig.GetBoolParameter("encrypted_communications")
		if !exists || !encryptedComms {
			t.Error("encrypted_communications should be true for defense sector")
		}

		// Test security clearance
		securityClearance, exists := sectorConfig.GetStringParameter("security_clearance")
		if !exists || securityClearance != "classified" {
			t.Error("security_clearance should be 'classified' for defense sector")
		}

		// Test threat assessment
		threatAssessment, exists := sectorConfig.GetBoolParameter("threat_assessment")
		if !exists || !threatAssessment {
			t.Error("threat_assessment should be true for defense sector")
		}
	})

	// Test service configurations
	t.Run("ServiceConfigurations", func(t *testing.T) {
		services := []string{"vehicle_agent", "fleet_manager", "dispatch_service", "control_center_ui"}
		for _, serviceName := range services {
			serviceConfig, err := sectorConfig.GetServiceConfig(serviceName)
			if err != nil {
				t.Errorf("Failed to get %s service configuration: %v", serviceName, err)
			}
			if serviceConfig == nil {
				t.Errorf("%s service configuration is nil", serviceName)
			}
		}
	})

	// Test vehicle agent specific configuration
	t.Run("VehicleAgentConfiguration", func(t *testing.T) {
		vehicleAgentConfig, err := sectorConfig.GetServiceConfig("vehicle_agent")
		if err != nil {
			t.Fatalf("Failed to get vehicle_agent configuration: %v", err)
		}

		// Test perception configuration
		if perception, ok := vehicleAgentConfig["perception"].(map[string]interface{}); ok {
			if outdoorNav, exists := perception["outdoor_navigation"]; !exists || outdoorNav != true {
				t.Error("outdoor_navigation should be true for defense vehicle agent")
			}
			if gpsDenied, exists := perception["gps_denied_navigation"]; !exists || gpsDenied != true {
				t.Error("gps_denied_navigation should be true for defense vehicle agent")
			}
			if threatDet, exists := perception["threat_detection"]; !exists || threatDet != true {
				t.Error("threat_detection should be true for defense vehicle agent")
			}
		} else {
			t.Error("perception configuration not found or invalid")
		}

		// Test control configuration
		if control, ok := vehicleAgentConfig["control"].(map[string]interface{}); ok {
			if highSpeed, exists := control["high_speed_control"]; !exists || highSpeed != true {
				t.Error("high_speed_control should be true for defense vehicle agent")
			}
			if tacticalManeuvering, exists := control["tactical_maneuvering"]; !exists || tacticalManeuvering != true {
				t.Error("tactical_maneuvering should be true for defense vehicle agent")
			}
		} else {
			t.Error("control configuration not found or invalid")
		}
	})

	// Test fleet manager specific configuration
	t.Run("FleetManagerConfiguration", func(t *testing.T) {
		fleetManagerConfig, err := sectorConfig.GetServiceConfig("fleet_manager")
		if err != nil {
			t.Fatalf("Failed to get fleet_manager configuration: %v", err)
		}

		// Test max fleet size
		if maxFleetSize, exists := fleetManagerConfig["max_fleet_size"]; !exists || maxFleetSize != 20 {
			t.Error("max_fleet_size should be 20 for defense fleet manager")
		}

		// Test vehicle types
		if vehicleTypes, exists := fleetManagerConfig["vehicle_types"]; exists {
			if types, ok := vehicleTypes.([]string); ok {
				expectedTypes := []string{"ugv", "unmanned_ground_vehicle", "tactical_vehicle", "reconnaissance_vehicle"}
				if len(types) != len(expectedTypes) {
					t.Errorf("Expected %d vehicle types, got %d", len(expectedTypes), len(types))
				}
			} else {
				t.Error("vehicle_types should be a string array")
			}
		} else {
			t.Error("vehicle_types not found in fleet manager configuration")
		}

		// Test mission readiness
		if missionReadiness, exists := fleetManagerConfig["mission_readiness"]; exists {
			if readiness, ok := missionReadiness.(map[string]interface{}); ok {
				if enabled, exists := readiness["enabled"]; !exists || enabled != true {
					t.Error("mission_readiness should be enabled for defense fleet manager")
				}
			}
		} else {
			t.Error("mission_readiness not found in defense fleet manager configuration")
		}
	})
}

// TestDefenseSectorSecurity tests the security features of defense sector
func TestDefenseSectorSecurity(t *testing.T) {
	// Set environment variable for defense sector
	os.Setenv("ATLASMESH_SECTOR", "defense")
	defer os.Unsetenv("ATLASMESH_SECTOR")

	// Load defense sector configuration
	sectorConfig, err := config.LoadSectorConfig(config.SectorDefense)
	if err != nil {
		t.Fatalf("Failed to load defense sector configuration: %v", err)
	}

	// Test security-related parameters
	t.Run("SecurityParameters", func(t *testing.T) {
		// Test encrypted communications
		encryptedComms, exists := sectorConfig.GetBoolParameter("encrypted_communications")
		if !exists || !encryptedComms {
			t.Error("encrypted_communications should be true for defense sector")
		}

		// Test security clearance
		securityClearance, exists := sectorConfig.GetStringParameter("security_clearance")
		if !exists || securityClearance != "classified" {
			t.Error("security_clearance should be 'classified' for defense sector")
		}

		// Test threat assessment
		threatAssessment, exists := sectorConfig.GetBoolParameter("threat_assessment")
		if !exists || !threatAssessment {
			t.Error("threat_assessment should be true for defense sector")
		}
	})

	// Test service-specific security settings
	t.Run("ServiceSecuritySettings", func(t *testing.T) {
		// Test vehicle agent security settings
		vehicleAgentConfig, err := sectorConfig.GetServiceConfig("vehicle_agent")
		if err != nil {
			t.Fatalf("Failed to get vehicle_agent configuration: %v", err)
		}

		if localization, ok := vehicleAgentConfig["localization"].(map[string]interface{}); ok {
			if encryptedComms, exists := localization["encrypted_communications"]; !exists || encryptedComms != true {
				t.Error("localization encrypted_communications should be true for defense vehicle agent")
			}
		}

		// Test control center UI security settings
		controlCenterConfig, err := sectorConfig.GetServiceConfig("control_center_ui")
		if err != nil {
			t.Fatalf("Failed to get control_center_ui configuration: %v", err)
		}

		if dashboard, ok := controlCenterConfig["dashboard"].(map[string]interface{}); ok {
			if layout, exists := dashboard["layout"].(map[string]interface{}); exists {
				if securityClearance, exists := layout["security_clearance"]; !exists || securityClearance != "classified" {
					t.Error("dashboard security_clearance should be 'classified' for defense control center")
				}
			}
		}
	})
}

// TestDefenseSectorTacticalFeatures tests the tactical features of defense sector
func TestDefenseSectorTacticalFeatures(t *testing.T) {
	// Set environment variable for defense sector
	os.Setenv("ATLASMESH_SECTOR", "defense")
	defer os.Unsetenv("ATLASMESH_SECTOR")

	// Load defense sector configuration
	sectorConfig, err := config.LoadSectorConfig(config.SectorDefense)
	if err != nil {
		t.Fatalf("Failed to load defense sector configuration: %v", err)
	}

	// Test tactical-specific parameters
	t.Run("TacticalParameters", func(t *testing.T) {
		// Test tactical operations
		tacticalOps, exists := sectorConfig.GetBoolParameter("tactical_operations")
		if !exists || !tacticalOps {
			t.Error("tactical_operations should be true for defense sector")
		}

		// Test mission-based operations
		missionOps, exists := sectorConfig.GetBoolParameter("mission_based_operations")
		if !exists || !missionOps {
			t.Error("mission_based_operations should be true for defense sector")
		}

		// Test GPS-denied navigation
		gpsDenied, exists := sectorConfig.GetBoolParameter("gps_denied_navigation")
		if !exists || !gpsDenied {
			t.Error("gps_denied_navigation should be true for defense sector")
		}

		// Test formation control
		formationControl, exists := sectorConfig.GetBoolParameter("formation_control")
		if !exists || !formationControl {
			t.Error("formation_control should be true for defense sector")
		}
	})

	// Test dispatch service tactical settings
	t.Run("DispatchTacticalSettings", func(t *testing.T) {
		dispatchConfig, err := sectorConfig.GetServiceConfig("dispatch_service")
		if err != nil {
			t.Fatalf("Failed to get dispatch_service configuration: %v", err)
		}

		// Test task prioritization
		if taskPrioritization, exists := dispatchConfig["task_prioritization"]; exists {
			if prioritization, ok := taskPrioritization.(map[string]interface{}); ok {
				if threatLevelMapping, exists := prioritization["threat_level_mapping"]; !exists || threatLevelMapping != true {
					t.Error("task_prioritization threat_level_mapping should be true for defense dispatch")
				}
			}
		}

		// Test routing settings
		if routing, exists := dispatchConfig["routing"]; exists {
			if route, ok := routing.(map[string]interface{}); ok {
				if threatAvoidance, exists := route["threat_avoidance"]; !exists || threatAvoidance != true {
					t.Error("routing threat_avoidance should be true for defense dispatch")
				}
				if stealthMode, exists := route["stealth_mode"]; !exists || stealthMode != true {
					t.Error("routing stealth_mode should be true for defense dispatch")
				}
			}
		}
	})
}

