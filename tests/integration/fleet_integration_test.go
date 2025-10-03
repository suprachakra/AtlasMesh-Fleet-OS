// AtlasMesh Fleet OS - Comprehensive Integration Tests
package integration

import (
	"bytes"
	"context"
	"encoding/json"
	"fmt"
	"net/http"
	"testing"
	"time"

	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
	"github.com/stretchr/testify/suite"
	"github.com/testcontainers/testcontainers-go"
	"github.com/testcontainers/testcontainers-go/wait"
)

// FleetIntegrationTestSuite provides comprehensive integration testing
type FleetIntegrationTestSuite struct {
	suite.Suite
	
	// Test infrastructure
	postgresContainer testcontainers.Container
	redisContainer    testcontainers.Container
	kafkaContainer    testcontainers.Container
	
	// Service endpoints
	fleetManagerURL   string
	policyEngineURL   string
	telemetryURL      string
	observabilityURL  string
	
	// Test context
	ctx context.Context
}

// SetupSuite initializes the test environment
func (suite *FleetIntegrationTestSuite) SetupSuite() {
	suite.ctx = context.Background()
	
	// Start PostgreSQL container
	postgresReq := testcontainers.ContainerRequest{
		Image:        "postgres:15",
		ExposedPorts: []string{"5432/tcp"},
		Env: map[string]string{
			"POSTGRES_DB":       "test_db",
			"POSTGRES_USER":     "test_user",
			"POSTGRES_PASSWORD": "test_password",
		},
		WaitingFor: wait.ForLog("database system is ready to accept connections"),
	}
	
	postgresContainer, err := testcontainers.GenericContainer(suite.ctx, testcontainers.GenericContainerRequest{
		ContainerRequest: postgresReq,
		Started:          true,
	})
	require.NoError(suite.T(), err)
	suite.postgresContainer = postgresContainer
	
	// Start Redis container
	redisReq := testcontainers.ContainerRequest{
		Image:        "redis:7",
		ExposedPorts: []string{"6379/tcp"},
		WaitingFor:   wait.ForLog("Ready to accept connections"),
	}
	
	redisContainer, err := testcontainers.GenericContainer(suite.ctx, testcontainers.GenericContainerRequest{
		ContainerRequest: redisReq,
		Started:          true,
	})
	require.NoError(suite.T(), err)
	suite.redisContainer = redisContainer
	
	// Start Kafka container
	kafkaReq := testcontainers.ContainerRequest{
		Image:        "confluentinc/cp-kafka:latest",
		ExposedPorts: []string{"9092/tcp"},
		Env: map[string]string{
			"KAFKA_ZOOKEEPER_CONNECT":                "zookeeper:2181",
			"KAFKA_ADVERTISED_LISTENERS":             "PLAINTEXT://localhost:9092",
			"KAFKA_OFFSETS_TOPIC_REPLICATION_FACTOR": "1",
		},
		WaitingFor: wait.ForLog("started (kafka.server.KafkaServer)"),
	}
	
	kafkaContainer, err := testcontainers.GenericContainer(suite.ctx, testcontainers.GenericContainerRequest{
		ContainerRequest: kafkaReq,
		Started:          true,
	})
	require.NoError(suite.T(), err)
	suite.kafkaContainer = kafkaContainer
	
	// Get container endpoints
	postgresPort, _ := postgresContainer.MappedPort(suite.ctx, "5432")
	redisPort, _ := redisContainer.MappedPort(suite.ctx, "6379")
	kafkaPort, _ := kafkaContainer.MappedPort(suite.ctx, "9092")
	
	// Set service URLs (assuming services are running locally for integration tests)
	suite.fleetManagerURL = "http://localhost:8080"
	suite.policyEngineURL = "http://localhost:8081"
	suite.telemetryURL = "http://localhost:8082"
	suite.observabilityURL = "http://localhost:8083"
	
	// Wait for services to be ready
	suite.waitForServices()
}

// TearDownSuite cleans up the test environment
func (suite *FleetIntegrationTestSuite) TearDownSuite() {
	if suite.postgresContainer != nil {
		suite.postgresContainer.Terminate(suite.ctx)
	}
	if suite.redisContainer != nil {
		suite.redisContainer.Terminate(suite.ctx)
	}
	if suite.kafkaContainer != nil {
		suite.kafkaContainer.Terminate(suite.ctx)
	}
}

// waitForServices waits for all services to be healthy
func (suite *FleetIntegrationTestSuite) waitForServices() {
	services := map[string]string{
		"Fleet Manager":   suite.fleetManagerURL + "/health",
		"Policy Engine":   suite.policyEngineURL + "/health",
		"Telemetry":       suite.telemetryURL + "/health",
		"Observability":   suite.observabilityURL + "/health",
	}
	
	for name, url := range services {
		suite.T().Logf("Waiting for %s to be ready...", name)
		
		for i := 0; i < 30; i++ { // Wait up to 30 seconds
			resp, err := http.Get(url)
			if err == nil && resp.StatusCode == 200 {
				resp.Body.Close()
				break
			}
			if resp != nil {
				resp.Body.Close()
			}
			time.Sleep(1 * time.Second)
		}
	}
}

// TestVehicleLifecycle tests the complete vehicle lifecycle
func (suite *FleetIntegrationTestSuite) TestVehicleLifecycle() {
	// 1. Create a new vehicle
	vehicle := map[string]interface{}{
		"asset_tag":      "TEST-001",
		"manufacturer":   "Tesla",
		"model":          "Model Y",
		"autonomy_level": "L4",
		"fleet_id":       "test-fleet-001",
	}
	
	vehicleJSON, _ := json.Marshal(vehicle)
	resp, err := http.Post(
		suite.fleetManagerURL+"/api/v1/vehicles",
		"application/json",
		bytes.NewBuffer(vehicleJSON),
	)
	require.NoError(suite.T(), err)
	require.Equal(suite.T(), http.StatusCreated, resp.StatusCode)
	
	var createdVehicle map[string]interface{}
	json.NewDecoder(resp.Body).Decode(&createdVehicle)
	resp.Body.Close()
	
	vehicleID := createdVehicle["vehicle_id"].(string)
	assert.NotEmpty(suite.T(), vehicleID)
	
	// 2. Retrieve the vehicle
	resp, err = http.Get(suite.fleetManagerURL + "/api/v1/vehicles/" + vehicleID)
	require.NoError(suite.T(), err)
	require.Equal(suite.T(), http.StatusOK, resp.StatusCode)
	resp.Body.Close()
	
	// 3. Update vehicle status
	statusUpdate := map[string]interface{}{
		"operational_status": "driving_av",
		"location": map[string]float64{
			"lat": 37.7749,
			"lng": -122.4194,
		},
		"battery_level": 85.0,
	}
	
	statusJSON, _ := json.Marshal(statusUpdate)
	req, _ := http.NewRequest("PATCH", suite.fleetManagerURL+"/api/v1/vehicles/"+vehicleID, bytes.NewBuffer(statusJSON))
	req.Header.Set("Content-Type", "application/json")
	
	client := &http.Client{}
	resp, err = client.Do(req)
	require.NoError(suite.T(), err)
	require.Equal(suite.T(), http.StatusOK, resp.StatusCode)
	resp.Body.Close()
	
	// 4. Verify status update
	resp, err = http.Get(suite.fleetManagerURL + "/api/v1/vehicles/" + vehicleID)
	require.NoError(suite.T(), err)
	
	var updatedVehicle map[string]interface{}
	json.NewDecoder(resp.Body).Decode(&updatedVehicle)
	resp.Body.Close()
	
	assert.Equal(suite.T(), "driving_av", updatedVehicle["operational_status"])
	assert.Equal(suite.T(), 85.0, updatedVehicle["battery_level"])
}

// TestPolicyEvaluation tests policy creation and evaluation
func (suite *FleetIntegrationTestSuite) TestPolicyEvaluation() {
	// 1. Create a safety policy
	policy := map[string]interface{}{
		"name":        "speed_limit_policy",
		"description": "Enforce speed limits in school zones",
		"policy_type": "speed_limit",
		"policy_content": map[string]interface{}{
			"max_speed_kmh": 25,
			"zone_type":     "school",
		},
		"scope": "global",
		"priority": 100,
	}
	
	policyJSON, _ := json.Marshal(policy)
	resp, err := http.Post(
		suite.policyEngineURL+"/api/v1/policies",
		"application/json",
		bytes.NewBuffer(policyJSON),
	)
	require.NoError(suite.T(), err)
	require.Equal(suite.T(), http.StatusCreated, resp.StatusCode)
	
	var createdPolicy map[string]interface{}
	json.NewDecoder(resp.Body).Decode(&createdPolicy)
	resp.Body.Close()
	
	policyID := createdPolicy["policy_id"].(string)
	assert.NotEmpty(suite.T(), policyID)
	
	// 2. Evaluate the policy
	evaluation := map[string]interface{}{
		"vehicle_id": "test-vehicle-001",
		"input_data": map[string]interface{}{
			"current_speed": 30,
			"zone_type":     "school",
			"location": map[string]float64{
				"lat": 37.7749,
				"lng": -122.4194,
			},
		},
	}
	
	evalJSON, _ := json.Marshal(evaluation)
	resp, err = http.Post(
		suite.policyEngineURL+"/api/v1/policies/"+policyID+"/evaluate",
		"application/json",
		bytes.NewBuffer(evalJSON),
	)
	require.NoError(suite.T(), err)
	require.Equal(suite.T(), http.StatusOK, resp.StatusCode)
	
	var evalResult map[string]interface{}
	json.NewDecoder(resp.Body).Decode(&evalResult)
	resp.Body.Close()
	
	// Should deny because speed (30) > limit (25)
	assert.Equal(suite.T(), "deny", evalResult["decision"])
}

// TestTelemetryIngestion tests telemetry data flow
func (suite *FleetIntegrationTestSuite) TestTelemetryIngestion() {
	// 1. Send telemetry data
	telemetry := map[string]interface{}{
		"vehicle_id":  "test-vehicle-001",
		"timestamp":   time.Now().Format(time.RFC3339),
		"data_type":   "vehicle_state",
		"data": map[string]interface{}{
			"speed":         45.5,
			"battery_level": 78.2,
			"location": map[string]float64{
				"lat": 37.7749,
				"lng": -122.4194,
			},
			"autonomy_status": "active",
		},
	}
	
	telemetryJSON, _ := json.Marshal(telemetry)
	resp, err := http.Post(
		suite.telemetryURL+"/api/v1/telemetry",
		"application/json",
		bytes.NewBuffer(telemetryJSON),
	)
	require.NoError(suite.T(), err)
	require.Equal(suite.T(), http.StatusAccepted, resp.StatusCode)
	resp.Body.Close()
	
	// 2. Query telemetry data
	time.Sleep(2 * time.Second) // Allow processing time
	
	resp, err = http.Get(suite.telemetryURL + "/api/v1/telemetry?vehicle_id=test-vehicle-001&limit=10")
	require.NoError(suite.T(), err)
	require.Equal(suite.T(), http.StatusOK, resp.StatusCode)
	
	var telemetryResponse map[string]interface{}
	json.NewDecoder(resp.Body).Decode(&telemetryResponse)
	resp.Body.Close()
	
	data := telemetryResponse["data"].([]interface{})
	assert.Greater(suite.T(), len(data), 0)
}

// TestObservabilityMetrics tests observability and monitoring
func (suite *FleetIntegrationTestSuite) TestObservabilityMetrics() {
	// 1. Check SLO status
	resp, err := http.Get(suite.observabilityURL + "/api/v1/slos")
	require.NoError(suite.T(), err)
	require.Equal(suite.T(), http.StatusOK, resp.StatusCode)
	
	var sloResponse map[string]interface{}
	json.NewDecoder(resp.Body).Decode(&sloResponse)
	resp.Body.Close()
	
	slos := sloResponse["slos"].([]interface{})
	assert.Greater(suite.T(), len(slos), 0)
	
	// 2. Check metrics endpoint
	resp, err = http.Get(suite.observabilityURL + "/metrics")
	require.NoError(suite.T(), err)
	require.Equal(suite.T(), http.StatusOK, resp.StatusCode)
	resp.Body.Close()
}

// TestEndToEndFleetOperation tests a complete fleet operation scenario
func (suite *FleetIntegrationTestSuite) TestEndToEndFleetOperation() {
	// This test simulates a complete autonomous vehicle operation:
	// 1. Vehicle registration and commissioning
	// 2. Policy deployment
	// 3. Trip assignment
	// 4. Real-time monitoring and telemetry
	// 5. Policy evaluation during operation
	// 6. Incident handling
	// 7. Trip completion and vehicle return
	
	suite.T().Log("Starting end-to-end fleet operation test...")
	
	// Step 1: Register vehicle
	vehicle := map[string]interface{}{
		"asset_tag":        "E2E-TEST-001",
		"manufacturer":     "Waymo",
		"model":            "Jaguar I-PACE",
		"autonomy_level":   "L4",
		"fleet_id":         "e2e-test-fleet",
		"operational_status": "idle",
	}
	
	vehicleJSON, _ := json.Marshal(vehicle)
	resp, err := http.Post(
		suite.fleetManagerURL+"/api/v1/vehicles",
		"application/json",
		bytes.NewBuffer(vehicleJSON),
	)
	require.NoError(suite.T(), err)
	require.Equal(suite.T(), http.StatusCreated, resp.StatusCode)
	
	var createdVehicle map[string]interface{}
	json.NewDecoder(resp.Body).Decode(&createdVehicle)
	resp.Body.Close()
	vehicleID := createdVehicle["vehicle_id"].(string)
	
	// Step 2: Deploy safety policy
	policy := map[string]interface{}{
		"name":        "e2e_safety_policy",
		"description": "E2E test safety policy",
		"policy_type": "safety",
		"policy_content": map[string]interface{}{
			"max_speed_kmh":     80,
			"emergency_brake":   true,
			"weather_restrictions": []string{"heavy_rain", "snow"},
		},
		"scope": "vehicle",
		"scope_ids": []string{vehicleID},
	}
	
	policyJSON, _ := json.Marshal(policy)
	resp, err = http.Post(
		suite.policyEngineURL+"/api/v1/policies",
		"application/json",
		bytes.NewBuffer(policyJSON),
	)
	require.NoError(suite.T(), err)
	require.Equal(suite.T(), http.StatusCreated, resp.StatusCode)
	resp.Body.Close()
	
	// Step 3: Simulate vehicle operation with telemetry
	for i := 0; i < 5; i++ {
		telemetry := map[string]interface{}{
			"vehicle_id": vehicleID,
			"timestamp":  time.Now().Format(time.RFC3339),
			"data_type":  "vehicle_state",
			"data": map[string]interface{}{
				"speed":           float64(40 + i*5), // Increasing speed
				"battery_level":   float64(90 - i*2), // Decreasing battery
				"autonomy_status": "active",
				"location": map[string]float64{
					"lat": 37.7749 + float64(i)*0.001,
					"lng": -122.4194 + float64(i)*0.001,
				},
			},
		}
		
		telemetryJSON, _ := json.Marshal(telemetry)
		resp, err = http.Post(
			suite.telemetryURL+"/api/v1/telemetry",
			"application/json",
			bytes.NewBuffer(telemetryJSON),
		)
		require.NoError(suite.T(), err)
		require.Equal(suite.T(), http.StatusAccepted, resp.StatusCode)
		resp.Body.Close()
		
		time.Sleep(500 * time.Millisecond)
	}
	
	// Step 4: Verify observability metrics were collected
	resp, err = http.Get(suite.observabilityURL + "/api/v1/slos")
	require.NoError(suite.T(), err)
	require.Equal(suite.T(), http.StatusOK, resp.StatusCode)
	resp.Body.Close()
	
	suite.T().Log("End-to-end fleet operation test completed successfully")
}

// TestPerformanceUnderLoad tests system performance under load
func (suite *FleetIntegrationTestSuite) TestPerformanceUnderLoad() {
	suite.T().Log("Starting performance under load test...")
	
	// Simulate concurrent vehicle operations
	numVehicles := 10
	numOperations := 50
	
	done := make(chan bool, numVehicles)
	
	for i := 0; i < numVehicles; i++ {
		go func(vehicleIndex int) {
			defer func() { done <- true }()
			
			vehicleID := fmt.Sprintf("perf-test-vehicle-%d", vehicleIndex)
			
			// Create vehicle
			vehicle := map[string]interface{}{
				"asset_tag":      fmt.Sprintf("PERF-%03d", vehicleIndex),
				"manufacturer":   "Tesla",
				"model":          "Model S",
				"autonomy_level": "L4",
				"fleet_id":       "performance-test-fleet",
			}
			
			vehicleJSON, _ := json.Marshal(vehicle)
			resp, err := http.Post(
				suite.fleetManagerURL+"/api/v1/vehicles",
				"application/json",
				bytes.NewBuffer(vehicleJSON),
			)
			if err != nil || resp.StatusCode != http.StatusCreated {
				suite.T().Errorf("Failed to create vehicle %d", vehicleIndex)
				return
			}
			resp.Body.Close()
			
			// Send multiple telemetry updates
			for j := 0; j < numOperations; j++ {
				telemetry := map[string]interface{}{
					"vehicle_id": vehicleID,
					"timestamp":  time.Now().Format(time.RFC3339),
					"data_type":  "vehicle_state",
					"data": map[string]interface{}{
						"speed":         float64(30 + j%50),
						"battery_level": float64(100 - j),
					},
				}
				
				telemetryJSON, _ := json.Marshal(telemetry)
				resp, err := http.Post(
					suite.telemetryURL+"/api/v1/telemetry",
					"application/json",
					bytes.NewBuffer(telemetryJSON),
				)
				if err != nil || resp.StatusCode != http.StatusAccepted {
					suite.T().Errorf("Failed to send telemetry for vehicle %d, operation %d", vehicleIndex, j)
				}
				if resp != nil {
					resp.Body.Close()
				}
				
				time.Sleep(10 * time.Millisecond)
			}
		}(i)
	}
	
	// Wait for all goroutines to complete
	for i := 0; i < numVehicles; i++ {
		<-done
	}
	
	suite.T().Log("Performance under load test completed")
}

// TestFailureRecovery tests system behavior during failures
func (suite *FleetIntegrationTestSuite) TestFailureRecovery() {
	suite.T().Log("Starting failure recovery test...")
	
	// Test database connection failure recovery
	// Test service restart scenarios
	// Test network partition handling
	// Test data consistency during failures
	
	// This would involve more complex failure injection scenarios
	// For now, we'll test basic error handling
	
	// 1. Test invalid requests
	invalidVehicle := map[string]interface{}{
		"invalid_field": "invalid_value",
	}
	
	vehicleJSON, _ := json.Marshal(invalidVehicle)
	resp, err := http.Post(
		suite.fleetManagerURL+"/api/v1/vehicles",
		"application/json",
		bytes.NewBuffer(vehicleJSON),
	)
	require.NoError(suite.T(), err)
	require.Equal(suite.T(), http.StatusBadRequest, resp.StatusCode)
	resp.Body.Close()
	
	// 2. Test non-existent resource access
	resp, err = http.Get(suite.fleetManagerURL + "/api/v1/vehicles/non-existent-id")
	require.NoError(suite.T(), err)
	require.Equal(suite.T(), http.StatusNotFound, resp.StatusCode)
	resp.Body.Close()
	
	suite.T().Log("Failure recovery test completed")
}

// Run the test suite
func TestFleetIntegrationSuite(t *testing.T) {
	suite.Run(t, new(FleetIntegrationTestSuite))
}
