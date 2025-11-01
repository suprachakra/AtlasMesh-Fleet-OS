package sector_tests

import (
	"testing"
	"os"
	"atlasmesh/config"
)

// TestRideHailSectorConfiguration tests the ride-hail sector configuration
func TestRideHailSectorConfiguration(t *testing.T) {
	// Set environment variable for ride-hail sector
	os.Setenv("ATLASMESH_SECTOR", "ride_hail")
	defer os.Unsetenv("ATLASMESH_SECTOR")

	// Load ride-hail sector configuration
	sectorConfig, err := config.LoadSectorConfig(config.SectorRideHail)
	if err != nil {
		t.Fatalf("Failed to load ride-hail sector configuration: %v", err)
	}

	// Validate sector configuration
	if err := sectorConfig.ValidateSectorConfig(); err != nil {
		t.Fatalf("Invalid ride-hail sector configuration: %v", err)
	}

	// Test sector-specific parameters
	t.Run("RideHailParameters", func(t *testing.T) {
		// Test operational zones
		operationalZones, exists := sectorConfig.GetParameter("operational_zones")
		if !exists {
			t.Error("operational_zones parameter not found")
		}
		if zones, ok := operationalZones.([]string); !ok || len(zones) == 0 {
			t.Error("operational_zones should be a non-empty string array")
		}

		// Test passenger transport
		passengerTransport, exists := sectorConfig.GetBoolParameter("passenger_transport")
		if !exists || !passengerTransport {
			t.Error("passenger_transport should be true for ride-hail sector")
		}

		// Test urban operations
		urbanOps, exists := sectorConfig.GetBoolParameter("urban_operations")
		if !exists || !urbanOps {
			t.Error("urban_operations should be true for ride-hail sector")
		}

		// Test real-time dispatch
		realTimeDispatch, exists := sectorConfig.GetBoolParameter("real_time_dispatch")
		if !exists || !realTimeDispatch {
			t.Error("real_time_dispatch should be true for ride-hail sector")
		}

		// Test passenger safety
		passengerSafety, exists := sectorConfig.GetBoolParameter("passenger_safety")
		if !exists || !passengerSafety {
			t.Error("passenger_safety should be true for ride-hail sector")
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
			if urbanNav, exists := perception["urban_navigation"]; !exists || urbanNav != true {
				t.Error("urban_navigation should be true for ride-hail vehicle agent")
			}
			if pedestrianDet, exists := perception["pedestrian_detection"]; !exists || pedestrianDet != true {
				t.Error("pedestrian_detection should be true for ride-hail vehicle agent")
			}
			if trafficLawCompliance, exists := perception["traffic_law_compliance"]; !exists || trafficLawCompliance != true {
				t.Error("traffic_law_compliance should be true for ride-hail vehicle agent")
			}
		} else {
			t.Error("perception configuration not found or invalid")
		}

		// Test control configuration
		if control, ok := vehicleAgentConfig["control"].(map[string]interface{}); ok {
			if smoothAcceleration, exists := control["smooth_acceleration"]; !exists || smoothAcceleration != true {
				t.Error("smooth_acceleration should be true for ride-hail vehicle agent")
			}
			if passengerComfort, exists := control["passenger_comfort"]; !exists || passengerComfort != true {
				t.Error("passenger_comfort should be true for ride-hail vehicle agent")
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
		if maxFleetSize, exists := fleetManagerConfig["max_fleet_size"]; !exists || maxFleetSize != 200 {
			t.Error("max_fleet_size should be 200 for ride-hail fleet manager")
		}

		// Test vehicle types
		if vehicleTypes, exists := fleetManagerConfig["vehicle_types"]; exists {
			if types, ok := vehicleTypes.([]string); ok {
				expectedTypes := []string{"robotaxi", "shuttle", "autonomous_vehicle", "passenger_vehicle"}
				if len(types) != len(expectedTypes) {
					t.Errorf("Expected %d vehicle types, got %d", len(expectedTypes), len(types))
				}
			} else {
				t.Error("vehicle_types should be a string array")
			}
		} else {
			t.Error("vehicle_types not found in fleet manager configuration")
		}

		// Test passenger capacity tracking
		if passengerCapacity, exists := fleetManagerConfig["passenger_capacity_tracking"]; exists {
			if capacity, ok := passengerCapacity.(map[string]interface{}); ok {
				if enabled, exists := capacity["enabled"]; !exists || enabled != true {
					t.Error("passenger_capacity_tracking should be enabled for ride-hail fleet manager")
				}
			}
		} else {
			t.Error("passenger_capacity_tracking not found in ride-hail fleet manager configuration")
		}
	})
}

// TestRideHailSectorPassengerExperience tests the passenger experience features of ride-hail sector
func TestRideHailSectorPassengerExperience(t *testing.T) {
	// Set environment variable for ride-hail sector
	os.Setenv("ATLASMESH_SECTOR", "ride_hail")
	defer os.Unsetenv("ATLASMESH_SECTOR")

	// Load ride-hail sector configuration
	sectorConfig, err := config.LoadSectorConfig(config.SectorRideHail)
	if err != nil {
		t.Fatalf("Failed to load ride-hail sector configuration: %v", err)
	}

	// Test passenger experience parameters
	t.Run("PassengerExperienceParameters", func(t *testing.T) {
		// Test passenger transport
		passengerTransport, exists := sectorConfig.GetBoolParameter("passenger_transport")
		if !exists || !passengerTransport {
			t.Error("passenger_transport should be true for ride-hail sector")
		}

		// Test passenger safety
		passengerSafety, exists := sectorConfig.GetBoolParameter("passenger_safety")
		if !exists || !passengerSafety {
			t.Error("passenger_safety should be true for ride-hail sector")
		}

		// Test real-time dispatch
		realTimeDispatch, exists := sectorConfig.GetBoolParameter("real_time_dispatch")
		if !exists || !realTimeDispatch {
			t.Error("real_time_dispatch should be true for ride-hail sector")
		}
	})

	// Test dispatch service passenger settings
	t.Run("DispatchPassengerSettings", func(t *testing.T) {
		dispatchConfig, err := sectorConfig.GetServiceConfig("dispatch_service")
		if err != nil {
			t.Fatalf("Failed to get dispatch_service configuration: %v", err)
		}

		// Test ride matching
		if rideMatching, exists := dispatchConfig["ride_matching"]; exists {
			if matching, ok := rideMatching.(map[string]interface{}); ok {
				if enabled, exists := matching["enabled"]; !exists || enabled != true {
					t.Error("ride_matching should be enabled for ride-hail dispatch")
				}
				if waitTimeMin, exists := matching["wait_time_minimization"]; !exists || waitTimeMin != true {
					t.Error("ride_matching wait_time_minimization should be true for ride-hail dispatch")
				}
			}
		}

		// Test dynamic pricing
		if dynamicPricing, exists := dispatchConfig["dynamic_pricing"]; exists {
			if pricing, ok := dynamicPricing.(map[string]interface{}); ok {
				if enabled, exists := pricing["enabled"]; !exists || enabled != true {
					t.Error("dynamic_pricing should be enabled for ride-hail dispatch")
				}
				if surgePricing, exists := pricing["surge_pricing"]; !exists || surgePricing != true {
					t.Error("dynamic_pricing surge_pricing should be true for ride-hail dispatch")
				}
			}
		}

		// Test routing settings
		if routing, exists := dispatchConfig["routing"]; exists {
			if route, ok := routing.(map[string]interface{}); ok {
				if passengerPickup, exists := route["passenger_pickup_optimization"]; !exists || passengerPickup != true {
					t.Error("routing passenger_pickup_optimization should be true for ride-hail dispatch")
				}
				if multiPassenger, exists := route["multi_passenger_pooling"]; !exists || multiPassenger != true {
					t.Error("routing multi_passenger_pooling should be true for ride-hail dispatch")
				}
			}
		}
	})
}

// TestRideHailSectorUrbanOperations tests the urban operations features of ride-hail sector
func TestRideHailSectorUrbanOperations(t *testing.T) {
	// Set environment variable for ride-hail sector
	os.Setenv("ATLASMESH_SECTOR", "ride_hail")
	defer os.Unsetenv("ATLASMESH_SECTOR")

	// Load ride-hail sector configuration
	sectorConfig, err := config.LoadSectorConfig(config.SectorRideHail)
	if err != nil {
		t.Fatalf("Failed to load ride-hail sector configuration: %v", err)
	}

	// Test urban operations parameters
	t.Run("UrbanOperationsParameters", func(t *testing.T) {
		// Test urban operations
		urbanOps, exists := sectorConfig.GetBoolParameter("urban_operations")
		if !exists || !urbanOps {
			t.Error("urban_operations should be true for ride-hail sector")
		}

		// Test traffic law compliance
		trafficLawCompliance, exists := sectorConfig.GetBoolParameter("traffic_law_compliance")
		if !exists || !trafficLawCompliance {
			t.Error("traffic_law_compliance should be true for ride-hail sector")
		}
	})

	// Test vehicle agent urban settings
	t.Run("VehicleAgentUrbanSettings", func(t *testing.T) {
		vehicleAgentConfig, err := sectorConfig.GetServiceConfig("vehicle_agent")
		if err != nil {
			t.Fatalf("Failed to get vehicle_agent configuration: %v", err)
		}

		// Test perception urban settings
		if perception, ok := vehicleAgentConfig["perception"].(map[string]interface{}); ok {
			if urbanNav, exists := perception["urban_navigation"]; !exists || urbanNav != true {
				t.Error("perception urban_navigation should be true for ride-hail vehicle agent")
			}
			if pedestrianDet, exists := perception["pedestrian_detection"]; !exists || pedestrianDet != true {
				t.Error("perception pedestrian_detection should be true for ride-hail vehicle agent")
			}
			if trafficLawCompliance, exists := perception["traffic_law_compliance"]; !exists || trafficLawCompliance != true {
				t.Error("perception traffic_law_compliance should be true for ride-hail vehicle agent")
			}
		}

		// Test control urban settings
		if control, ok := vehicleAgentConfig["control"].(map[string]interface{}); ok {
			if smoothAcceleration, exists := control["smooth_acceleration"]; !exists || smoothAcceleration != true {
				t.Error("control smooth_acceleration should be true for ride-hail vehicle agent")
			}
			if passengerComfort, exists := control["passenger_comfort"]; !exists || passengerComfort != true {
				t.Error("control passenger_comfort should be true for ride-hail vehicle agent")
			}
		}
	})
}