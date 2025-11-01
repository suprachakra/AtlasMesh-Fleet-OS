package sector_tests

import (
	"testing"
	"os"
	"atlasmesh/config"
)

// TestLogisticsSectorConfiguration tests the logistics sector configuration
func TestLogisticsSectorConfiguration(t *testing.T) {
	// Set environment variable for logistics sector
	os.Setenv("ATLASMESH_SECTOR", "logistics")
	defer os.Unsetenv("ATLASMESH_SECTOR")

	// Load logistics sector configuration
	sectorConfig, err := config.LoadSectorConfig(config.SectorLogistics)
	if err != nil {
		t.Fatalf("Failed to load logistics sector configuration: %v", err)
	}

	// Validate sector configuration
	if err := sectorConfig.ValidateSectorConfig(); err != nil {
		t.Fatalf("Invalid logistics sector configuration: %v", err)
	}

	// Test sector-specific parameters
	t.Run("LogisticsParameters", func(t *testing.T) {
		// Test warehouse zones
		warehouseZones, exists := sectorConfig.GetParameter("warehouse_zones")
		if !exists {
			t.Error("warehouse_zones parameter not found")
		}
		if zones, ok := warehouseZones.([]string); !ok || len(zones) == 0 {
			t.Error("warehouse_zones should be a non-empty string array")
		}

		// Test indoor operations
		indoorOps, exists := sectorConfig.GetBoolParameter("indoor_operations")
		if !exists || !indoorOps {
			t.Error("indoor_operations should be true for logistics sector")
		}

		// Test precision required
		precision, exists := sectorConfig.GetStringParameter("precision_required")
		if !exists || precision != "high" {
			t.Error("precision_required should be 'high' for logistics sector")
		}

		// Test max vehicle speed
		maxSpeed, exists := sectorConfig.GetIntParameter("max_vehicle_speed")
		if !exists || maxSpeed != 3 {
			t.Error("max_vehicle_speed should be 3 for logistics sector")
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
			if indoorNav, exists := perception["indoor_navigation"]; !exists || indoorNav != true {
				t.Error("indoor_navigation should be true for logistics vehicle agent")
			}
			if palletDet, exists := perception["pallet_detection"]; !exists || palletDet != true {
				t.Error("pallet_detection should be true for logistics vehicle agent")
			}
		} else {
			t.Error("perception configuration not found or invalid")
		}

		// Test control configuration
		if control, ok := vehicleAgentConfig["control"].(map[string]interface{}); ok {
			if lowSpeed, exists := control["low_speed_control"]; !exists || lowSpeed != true {
				t.Error("low_speed_control should be true for logistics vehicle agent")
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
		if maxFleetSize, exists := fleetManagerConfig["max_fleet_size"]; !exists || maxFleetSize != 50 {
			t.Error("max_fleet_size should be 50 for logistics fleet manager")
		}

		// Test vehicle types
		if vehicleTypes, exists := fleetManagerConfig["vehicle_types"]; exists {
			if types, ok := vehicleTypes.([]string); ok {
				expectedTypes := []string{"forklift", "tugger", "pallet_truck", "reach_truck"}
				if len(types) != len(expectedTypes) {
					t.Errorf("Expected %d vehicle types, got %d", len(expectedTypes), len(types))
				}
			} else {
				t.Error("vehicle_types should be a string array")
			}
		} else {
			t.Error("vehicle_types not found in fleet manager configuration")
		}
	})
}

// TestLogisticsSectorIntegration tests the integration between logistics services
func TestLogisticsSectorIntegration(t *testing.T) {
	// Set environment variable for logistics sector
	os.Setenv("ATLASMESH_SECTOR", "logistics")
	defer os.Unsetenv("ATLASMESH_SECTOR")

	// Load logistics sector configuration
	sectorConfig, err := config.LoadSectorConfig(config.SectorLogistics)
	if err != nil {
		t.Fatalf("Failed to load logistics sector configuration: %v", err)
	}

	// Test configuration consistency across services
	t.Run("ConfigurationConsistency", func(t *testing.T) {
		// Test that all services have consistent precision requirements
		services := []string{"vehicle_agent", "fleet_manager", "dispatch_service", "control_center_ui"}
		for _, serviceName := range services {
			serviceConfig, err := sectorConfig.GetServiceConfig(serviceName)
			if err != nil {
				t.Errorf("Failed to get %s service configuration: %v", serviceName, err)
				continue
			}

			// Check if service has precision-related configuration
			if precision, exists := serviceConfig["precision_required"]; exists {
				if precision != "high" {
					t.Errorf("%s service precision_required should be 'high' for logistics sector", serviceName)
				}
			}
		}
	})

	// Test warehouse-specific features
	t.Run("WarehouseFeatures", func(t *testing.T) {
		// Test that warehouse zones are properly configured
		warehouseZones, exists := sectorConfig.GetParameter("warehouse_zones")
		if !exists {
			t.Error("warehouse_zones parameter not found")
		}

		if zones, ok := warehouseZones.([]string); ok {
			expectedZones := []string{"A", "B", "C", "D"}
			if len(zones) != len(expectedZones) {
				t.Errorf("Expected %d warehouse zones, got %d", len(expectedZones), len(zones))
			}
		} else {
			t.Error("warehouse_zones should be a string array")
		}

		// Test that indoor operations are enabled
		indoorOps, exists := sectorConfig.GetBoolParameter("indoor_operations")
		if !exists || !indoorOps {
			t.Error("indoor_operations should be true for logistics sector")
		}
	})
}

// TestLogisticsSectorPerformance tests the performance characteristics of logistics sector
func TestLogisticsSectorPerformance(t *testing.T) {
	// Set environment variable for logistics sector
	os.Setenv("ATLASMESH_SECTOR", "logistics")
	defer os.Unsetenv("ATLASMESH_SECTOR")

	// Load logistics sector configuration
	sectorConfig, err := config.LoadSectorConfig(config.SectorLogistics)
	if err != nil {
		t.Fatalf("Failed to load logistics sector configuration: %v", err)
	}

	// Test performance-related parameters
	t.Run("PerformanceParameters", func(t *testing.T) {
		// Test real-time tracking is enabled
		realTimeTracking, exists := sectorConfig.GetBoolParameter("real_time_tracking")
		if !exists || !realTimeTracking {
			t.Error("real_time_tracking should be true for logistics sector")
		}

		// Test battery management is enabled
		batteryMgmt, exists := sectorConfig.GetBoolParameter("battery_management")
		if !exists || !batteryMgmt {
			t.Error("battery_management should be true for logistics sector")
		}

		// Test shift-based operations are enabled
		shiftOps, exists := sectorConfig.GetBoolParameter("shift_based_operations")
		if !exists || !shiftOps {
			t.Error("shift_based_operations should be true for logistics sector")
		}
	})

	// Test service-specific performance settings
	t.Run("ServicePerformanceSettings", func(t *testing.T) {
		// Test vehicle agent performance settings
		vehicleAgentConfig, err := sectorConfig.GetServiceConfig("vehicle_agent")
		if err != nil {
			t.Fatalf("Failed to get vehicle_agent configuration: %v", err)
		}

		if perception, ok := vehicleAgentConfig["perception"].(map[string]interface{}); ok {
			if updateFreq, exists := perception["update_frequency_hz"]; !exists || updateFreq != 10 {
				t.Error("perception update_frequency_hz should be 10 for logistics sector")
			}
		}

		// Test dispatch service performance settings
		dispatchConfig, err := sectorConfig.GetServiceConfig("dispatch_service")
		if err != nil {
			t.Fatalf("Failed to get dispatch_service configuration: %v", err)
		}

		if maxTasks, exists := dispatchConfig["max_concurrent_tasks"]; !exists || maxTasks != 100 {
			t.Error("max_concurrent_tasks should be 100 for logistics dispatch service")
		}
	})
}

