package sector_tests

import (
	"testing"
	"os"
	"atlasmesh/config"
)

// TestMiningSectorConfiguration tests the mining sector configuration
func TestMiningSectorConfiguration(t *testing.T) {
	// Set environment variable for mining sector
	os.Setenv("ATLASMESH_SECTOR", "mining")
	defer os.Unsetenv("ATLASMESH_SECTOR")

	// Load mining sector configuration
	sectorConfig, err := config.LoadSectorConfig(config.SectorMining)
	if err != nil {
		t.Fatalf("Failed to load mining sector configuration: %v", err)
	}

	// Validate sector configuration
	if err := sectorConfig.ValidateSectorConfig(); err != nil {
		t.Fatalf("Invalid mining sector configuration: %v", err)
	}

	// Test sector-specific parameters
	t.Run("MiningParameters", func(t *testing.T) {
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
			t.Error("outdoor_operations should be true for mining sector")
		}

		// Test production focused
		productionFocused, exists := sectorConfig.GetBoolParameter("production_focused")
		if !exists || !productionFocused {
			t.Error("production_focused should be true for mining sector")
		}

		// Test heavy equipment
		heavyEquipment, exists := sectorConfig.GetBoolParameter("heavy_equipment")
		if !exists || !heavyEquipment {
			t.Error("heavy_equipment should be true for mining sector")
		}

		// Test blast zone safety
		blastZoneSafety, exists := sectorConfig.GetBoolParameter("blast_zone_safety")
		if !exists || !blastZoneSafety {
			t.Error("blast_zone_safety should be true for mining sector")
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
				t.Error("outdoor_navigation should be true for mining vehicle agent")
			}
			if heavyVehicleDynamics, exists := perception["heavy_vehicle_dynamics"]; !exists || heavyVehicleDynamics != true {
				t.Error("heavy_vehicle_dynamics should be true for mining vehicle agent")
			}
			if blastZoneDet, exists := perception["blast_zone_detection"]; !exists || blastZoneDet != true {
				t.Error("blast_zone_detection should be true for mining vehicle agent")
			}
		} else {
			t.Error("perception configuration not found or invalid")
		}

		// Test control configuration
		if control, ok := vehicleAgentConfig["control"].(map[string]interface{}); ok {
			if heavyVehicleControl, exists := control["heavy_vehicle_control"]; !exists || heavyVehicleControl != true {
				t.Error("heavy_vehicle_control should be true for mining vehicle agent")
			}
			if gradeAdaptation, exists := control["grade_adaptation"]; !exists || gradeAdaptation != true {
				t.Error("grade_adaptation should be true for mining vehicle agent")
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
		if maxFleetSize, exists := fleetManagerConfig["max_fleet_size"]; !exists || maxFleetSize != 100 {
			t.Error("max_fleet_size should be 100 for mining fleet manager")
		}

		// Test vehicle types
		if vehicleTypes, exists := fleetManagerConfig["vehicle_types"]; exists {
			if types, ok := vehicleTypes.([]string); ok {
				expectedTypes := []string{"haul_truck", "bulldozer", "excavator", "loader", "drill_rig"}
				if len(types) != len(expectedTypes) {
					t.Errorf("Expected %d vehicle types, got %d", len(expectedTypes), len(types))
				}
			} else {
				t.Error("vehicle_types should be a string array")
			}
		} else {
			t.Error("vehicle_types not found in fleet manager configuration")
		}

		// Test shift scheduling
		if shiftScheduling, exists := fleetManagerConfig["shift_scheduling"]; exists {
			if scheduling, ok := shiftScheduling.(map[string]interface{}); ok {
				if enabled, exists := scheduling["enabled"]; !exists || enabled != true {
					t.Error("shift_scheduling should be enabled for mining fleet manager")
				}
				if shiftDuration, exists := scheduling["shift_duration_hours"]; !exists || shiftDuration != 12 {
					t.Error("shift_duration_hours should be 12 for mining fleet manager")
				}
			}
		} else {
			t.Error("shift_scheduling not found in mining fleet manager configuration")
		}
	})
}

// TestMiningSectorProduction tests the production features of mining sector
func TestMiningSectorProduction(t *testing.T) {
	// Set environment variable for mining sector
	os.Setenv("ATLASMESH_SECTOR", "mining")
	defer os.Unsetenv("ATLASMESH_SECTOR")

	// Load mining sector configuration
	sectorConfig, err := config.LoadSectorConfig(config.SectorMining)
	if err != nil {
		t.Fatalf("Failed to load mining sector configuration: %v", err)
	}

	// Test production-related parameters
	t.Run("ProductionParameters", func(t *testing.T) {
		// Test production focused
		productionFocused, exists := sectorConfig.GetBoolParameter("production_focused")
		if !exists || !productionFocused {
			t.Error("production_focused should be true for mining sector")
		}

		// Test production targets
		productionTargets, exists := sectorConfig.GetBoolParameter("production_targets")
		if !exists || !productionTargets {
			t.Error("production_targets should be true for mining sector")
		}

		// Test haul cycle optimization
		haulCycleOpt, exists := sectorConfig.GetBoolParameter("haul_cycle_optimization")
		if !exists || !haulCycleOpt {
			t.Error("haul_cycle_optimization should be true for mining sector")
		}
	})

	// Test dispatch service production settings
	t.Run("DispatchProductionSettings", func(t *testing.T) {
		dispatchConfig, err := sectorConfig.GetServiceConfig("dispatch_service")
		if err != nil {
			t.Fatalf("Failed to get dispatch_service configuration: %v", err)
		}

		// Test task prioritization
		if taskPrioritization, exists := dispatchConfig["task_prioritization"]; exists {
			if prioritization, ok := taskPrioritization.(map[string]interface{}); ok {
				if productionPriority, exists := prioritization["production_priority"]; !exists || productionPriority != true {
					t.Error("task_prioritization production_priority should be true for mining dispatch")
				}
			}
		}

		// Test load balancing
		if loadBalancing, exists := dispatchConfig["load_balancing"]; exists {
			if balancing, ok := loadBalancing.(map[string]interface{}); ok {
				if strategy, exists := balancing["strategy"]; !exists || strategy != "production_optimized" {
					t.Error("load_balancing strategy should be 'production_optimized' for mining dispatch")
				}
				if truckShovelMatching, exists := balancing["truck_shovel_matching"]; !exists || truckShovelMatching != true {
					t.Error("load_balancing truck_shovel_matching should be true for mining dispatch")
				}
			}
		}

		// Test routing settings
		if routing, exists := dispatchConfig["routing"]; exists {
			if route, ok := routing.(map[string]interface{}); ok {
				if haulCycleOpt, exists := route["haul_cycle_optimization"]; !exists || haulCycleOpt != true {
					t.Error("routing haul_cycle_optimization should be true for mining dispatch")
				}
			}
		}
	})
}

// TestMiningSectorSafety tests the safety features of mining sector
func TestMiningSectorSafety(t *testing.T) {
	// Set environment variable for mining sector
	os.Setenv("ATLASMESH_SECTOR", "mining")
	defer os.Unsetenv("ATLASMESH_SECTOR")

	// Load mining sector configuration
	sectorConfig, err := config.LoadSectorConfig(config.SectorMining)
	if err != nil {
		t.Fatalf("Failed to load mining sector configuration: %v", err)
	}

	// Test safety-related parameters
	t.Run("SafetyParameters", func(t *testing.T) {
		// Test blast zone safety
		blastZoneSafety, exists := sectorConfig.GetBoolParameter("blast_zone_safety")
		if !exists || !blastZoneSafety {
			t.Error("blast_zone_safety should be true for mining sector")
		}

		// Test heavy equipment
		heavyEquipment, exists := sectorConfig.GetBoolParameter("heavy_equipment")
		if !exists || !heavyEquipment {
			t.Error("heavy_equipment should be true for mining sector")
		}
	})

	// Test vehicle agent safety settings
	t.Run("VehicleAgentSafetySettings", func(t *testing.T) {
		vehicleAgentConfig, err := sectorConfig.GetServiceConfig("vehicle_agent")
		if err != nil {
			t.Fatalf("Failed to get vehicle_agent configuration: %v", err)
		}

		// Test safety configuration
		if safety, ok := vehicleAgentConfig["safety"].(map[string]interface{}); ok {
			if miningSafety, exists := safety["mining_safety"]; !exists || miningSafety != true {
				t.Error("mining_safety should be true for mining vehicle agent")
			}
			if blastZoneSafety, exists := safety["blast_zone_safety"]; !exists || blastZoneSafety != true {
				t.Error("blast_zone_safety should be true for mining vehicle agent")
			}
			if largeVehicleDet, exists := safety["large_vehicle_detection"]; !exists || largeVehicleDet != true {
				t.Error("large_vehicle_detection should be true for mining vehicle agent")
			}
		} else {
			t.Error("safety configuration not found or invalid")
		}
	})
}

