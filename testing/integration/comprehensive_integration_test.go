package integration

import (
	"context"
	"encoding/json"
	"fmt"
	"log"
	"net/http"
	"os"
	"testing"
	"time"

	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
	"github.com/testcontainers/testcontainers-go"
	"github.com/testcontainers/testcontainers-go/modules/postgres"
	"github.com/testcontainers/testcontainers-go/modules/redis"
	"github.com/testcontainers/testcontainers-go/modules/kafka"
	"github.com/testcontainers/testcontainers-go/wait"
)

// Comprehensive Integration Test Suite for AtlasMesh Fleet OS
//
// Tests the complete system integration including:
// - Service-to-service communication
// - Database operations
// - Message queue processing
// - API endpoints
// - Security and authentication
// - Performance and load testing
// - Error handling and recovery

type IntegrationTestSuite struct {
	postgresContainer *postgres.PostgresContainer
	redisContainer    *redis.RedisContainer
	kafkaContainer    *kafka.KafkaContainer
	clickhouseContainer testcontainers.Container
	minioContainer    testcontainers.Container
	
	// Service URLs
	fleetManagerURL    string
	analyticsURL       string
	externalIntegrationsURL string
	mlPipelineURL      string
	vehicleGatewayURL  string
	
	// Test data
	testFleetID        string
	testVehicleID      string
	testOrganizationID string
}

func TestMain(m *testing.M) {
	// Setup test environment
	suite := &IntegrationTestSuite{}
	
	// Initialize containers
	if err := suite.setupContainers(); err != nil {
		log.Fatalf("Failed to setup containers: %v", err)
	}
	
	// Start services
	if err := suite.startServices(); err != nil {
		log.Fatalf("Failed to start services: %v", err)
	}
	
	// Run tests
	code := m.Run()
	
	// Cleanup
	suite.cleanup()
	
	os.Exit(code)
}

func (suite *IntegrationTestSuite) setupContainers() error {
	ctx := context.Background()
	
	// Setup PostgreSQL
	postgresContainer, err := postgres.RunContainer(ctx,
		testcontainers.WithImage("postgres:15"),
		postgres.WithDatabase("fleet_os_test"),
		postgres.WithUsername("testuser"),
		postgres.WithPassword("testpass"),
		postgres.WithInitScripts("../database/migrations/001_create_fleet_tables.sql"),
	)
	if err != nil {
		return fmt.Errorf("failed to start PostgreSQL: %v", err)
	}
	suite.postgresContainer = postgresContainer
	
	// Setup Redis
	redisContainer, err := redis.RunContainer(ctx,
		testcontainers.WithImage("redis:7-alpine"),
	)
	if err != nil {
		return fmt.Errorf("failed to start Redis: %v", err)
	}
	suite.redisContainer = redisContainer
	
	// Setup Kafka
	kafkaContainer, err := kafka.RunContainer(ctx,
		testcontainers.WithImage("confluentinc/cp-kafka:latest"),
	)
	if err != nil {
		return fmt.Errorf("failed to start Kafka: %v", err)
	}
	suite.kafkaContainer = kafkaContainer
	
	// Setup ClickHouse
	clickhouseReq := testcontainers.ContainerRequest{
		Image:        "clickhouse/clickhouse-server:latest",
		ExposedPorts: []string{"8123/tcp", "9000/tcp"},
		Env: map[string]string{
			"CLICKHOUSE_DB": "fleet_analytics",
		},
		WaitingFor: wait.ForLog("Ready for connections"),
	}
	clickhouseContainer, err := testcontainers.GenericContainer(ctx, testcontainers.GenericContainerRequest{
		ContainerRequest: clickhouseReq,
		Started:         true,
	})
	if err != nil {
		return fmt.Errorf("failed to start ClickHouse: %v", err)
	}
	suite.clickhouseContainer = clickhouseContainer
	
	// Setup MinIO
	minioReq := testcontainers.ContainerRequest{
		Image:        "minio/minio:latest",
		ExposedPorts: []string{"9000/tcp", "9001/tcp"},
		Cmd:          []string{"server", "/data", "--console-address", ":9001"},
		Env: map[string]string{
			"MINIO_ROOT_USER":     "minioadmin",
			"MINIO_ROOT_PASSWORD": "minioadmin",
		},
		WaitingFor: wait.ForLog("API: http://0.0.0.0:9000"),
	}
	minioContainer, err := testcontainers.GenericContainer(ctx, testcontainers.GenericContainerRequest{
		ContainerRequest: minioReq,
		Started:         true,
	})
	if err != nil {
		return fmt.Errorf("failed to start MinIO: %v", err)
	}
	suite.minioContainer = minioContainer
	
	return nil
}

func (suite *IntegrationTestSuite) startServices() error {
	// Get container URLs
	postgresURL, err := suite.postgresContainer.ConnectionString(context.Background())
	if err != nil {
		return fmt.Errorf("failed to get PostgreSQL URL: %v", err)
	}
	
	redisURL, err := suite.redisContainer.ConnectionString(context.Background())
	if err != nil {
		return fmt.Errorf("failed to get Redis URL: %v", err)
	}
	
	kafkaURL, err := suite.kafkaContainer.ConnectionString(context.Background())
	if err != nil {
		return fmt.Errorf("failed to get Kafka URL: %v", err)
	}
	
	// Set service URLs (in production, these would be actual service endpoints)
	suite.fleetManagerURL = "http://localhost:8080"
	suite.analyticsURL = "http://localhost:8081"
	suite.externalIntegrationsURL = "http://localhost:8082"
	suite.mlPipelineURL = "http://localhost:8083"
	suite.vehicleGatewayURL = "http://localhost:8084"
	
	// Initialize test data
	suite.testOrganizationID = "test-org-001"
	suite.testFleetID = "test-fleet-001"
	suite.testVehicleID = "test-vehicle-001"
	
	return nil
}

func (suite *IntegrationTestSuite) cleanup() {
	ctx := context.Background()
	
	if suite.postgresContainer != nil {
		suite.postgresContainer.Terminate(ctx)
	}
	if suite.redisContainer != nil {
		suite.redisContainer.Terminate(ctx)
	}
	if suite.kafkaContainer != nil {
		suite.kafkaContainer.Terminate(ctx)
	}
	if suite.clickhouseContainer != nil {
		suite.clickhouseContainer.Terminate(ctx)
	}
	if suite.minioContainer != nil {
		suite.minioContainer.Terminate(ctx)
	}
}

// Test Suite: Service Health Checks

func TestServiceHealthChecks(t *testing.T) {
	suite := &IntegrationTestSuite{}
	
	tests := []struct {
		name     string
		url      string
		expected int
	}{
		{"Fleet Manager Health", suite.fleetManagerURL + "/health", http.StatusOK},
		{"Analytics Service Health", suite.analyticsURL + "/health", http.StatusOK},
		{"External Integrations Health", suite.externalIntegrationsURL + "/health", http.StatusOK},
		{"ML Pipeline Health", suite.mlPipelineURL + "/health", http.StatusOK},
		{"Vehicle Gateway Health", suite.vehicleGatewayURL + "/health", http.StatusOK},
	}
	
	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			resp, err := http.Get(tt.url)
			require.NoError(t, err)
			defer resp.Body.Close()
			
			assert.Equal(t, tt.expected, resp.StatusCode)
		})
	}
}

// Test Suite: Fleet Manager Integration

func TestFleetManagerIntegration(t *testing.T) {
	suite := &IntegrationTestSuite{}
	
	t.Run("Create Organization", func(t *testing.T) {
		orgData := map[string]interface{}{
			"name":        "Test Organization",
			"description": "Test organization for integration testing",
			"contact_info": map[string]string{
				"email": "test@example.com",
				"phone": "+971501234567",
			},
		}
		
		resp, err := suite.makeRequest("POST", suite.fleetManagerURL+"/api/v1/organizations", orgData)
		require.NoError(t, err)
		defer resp.Body.Close()
		
		assert.Equal(t, http.StatusCreated, resp.StatusCode)
	})
	
	t.Run("Create Fleet", func(t *testing.T) {
		fleetData := map[string]interface{}{
			"organization_id": suite.testOrganizationID,
			"name":           "Test Fleet",
			"description":    "Test fleet for integration testing",
			"fleet_type":     "autonomous",
			"capacity":       100,
		}
		
		resp, err := suite.makeRequest("POST", suite.fleetManagerURL+"/api/v1/fleets", fleetData)
		require.NoError(t, err)
		defer resp.Body.Close()
		
		assert.Equal(t, http.StatusCreated, resp.StatusCode)
	})
	
	t.Run("Create Vehicle", func(t *testing.T) {
		vehicleData := map[string]interface{}{
			"fleet_id":         suite.testFleetID,
			"asset_tag":        "TEST-001",
			"vin":              "TESTVIN123456789",
			"license_plate":    "TEST-001",
			"manufacturer":     "Tesla",
			"model":            "Model 3",
			"year":             2023,
			"autonomy_level":   "L4",
			"vehicle_profile": map[string]interface{}{
				"battery_capacity": 75,
				"max_range":       400,
			},
			"capabilities": map[string]interface{}{
				"autonomous_driving": true,
				"remote_operation":   true,
			},
		}
		
		resp, err := suite.makeRequest("POST", suite.fleetManagerURL+"/api/v1/fleet/vehicles", vehicleData)
		require.NoError(t, err)
		defer resp.Body.Close()
		
		assert.Equal(t, http.StatusCreated, resp.StatusCode)
	})
	
	t.Run("Get Vehicle Status", func(t *testing.T) {
		resp, err := http.Get(suite.fleetManagerURL + "/api/v1/fleet/vehicles/" + suite.testVehicleID + "/status")
		require.NoError(t, err)
		defer resp.Body.Close()
		
		assert.Equal(t, http.StatusOK, resp.StatusCode)
	})
	
	t.Run("Update Vehicle Location", func(t *testing.T) {
		locationData := map[string]interface{}{
			"latitude":  25.2048,
			"longitude": 55.2708,
			"altitude":  10.5,
			"heading":   180.0,
			"speed":     45.0,
		}
		
		resp, err := suite.makeRequest("PUT", suite.fleetManagerURL+"/api/v1/fleet/vehicles/"+suite.testVehicleID+"/location", locationData)
		require.NoError(t, err)
		defer resp.Body.Close()
		
		assert.Equal(t, http.StatusOK, resp.StatusCode)
	})
	
	t.Run("Dispatch Vehicle", func(t *testing.T) {
		dispatchData := map[string]interface{}{
			"trip_id":      "trip-001",
			"origin":       map[string]float64{"latitude": 25.2048, "longitude": 55.2708},
			"destination":  map[string]float64{"latitude": 25.2582, "longitude": 55.3047},
			"passenger_id": "passenger-001",
			"priority":     "normal",
		}
		
		resp, err := suite.makeRequest("POST", suite.fleetManagerURL+"/api/v1/fleet/vehicles/"+suite.testVehicleID+"/dispatch", dispatchData)
		require.NoError(t, err)
		defer resp.Body.Close()
		
		assert.Equal(t, http.StatusOK, resp.StatusCode)
	})
}

// Test Suite: Analytics Service Integration

func TestAnalyticsServiceIntegration(t *testing.T) {
	suite := &IntegrationTestSuite{}
	
	t.Run("Get Fleet KPIs", func(t *testing.T) {
		resp, err := http.Get(suite.analyticsURL + "/api/v1/analytics/kpis?timeframe=24h")
		require.NoError(t, err)
		defer resp.Body.Close()
		
		assert.Equal(t, http.StatusOK, resp.StatusCode)
		
		var kpis map[string]interface{}
		err = json.NewDecoder(resp.Body).Decode(&kpis)
		require.NoError(t, err)
		
		assert.Contains(t, kpis, "utilization_rate")
		assert.Contains(t, kpis, "fleet_availability")
	})
	
	t.Run("Get Operational Metrics", func(t *testing.T) {
		resp, err := http.Get(suite.analyticsURL + "/api/v1/analytics/operational")
		require.NoError(t, err)
		defer resp.Body.Close()
		
		assert.Equal(t, http.StatusOK, resp.StatusCode)
	})
	
	t.Run("Get Financial Metrics", func(t *testing.T) {
		resp, err := http.Get(suite.analyticsURL + "/api/v1/analytics/financial")
		require.NoError(t, err)
		defer resp.Body.Close()
		
		assert.Equal(t, http.StatusOK, resp.StatusCode)
	})
	
	t.Run("Get Safety Metrics", func(t *testing.T) {
		resp, err := http.Get(suite.analyticsURL + "/api/v1/analytics/safety")
		require.NoError(t, err)
		defer resp.Body.Close()
		
		assert.Equal(t, http.StatusOK, resp.StatusCode)
	})
	
	t.Run("Execute Custom Query", func(t *testing.T) {
		queryData := map[string]interface{}{
			"query": "SELECT COUNT(*) as total_vehicles FROM fleet_core.vehicles",
		}
		
		resp, err := suite.makeRequest("POST", suite.analyticsURL+"/api/v1/analytics/query", queryData)
		require.NoError(t, err)
		defer resp.Body.Close()
		
		assert.Equal(t, http.StatusOK, resp.StatusCode)
	})
}

// Test Suite: External Integrations

func TestExternalIntegrations(t *testing.T) {
	suite := &IntegrationTestSuite{}
	
	t.Run("Get Vehicle Registration", func(t *testing.T) {
		resp, err := http.Get(suite.externalIntegrationsURL + "/api/v1/uae/vehicle-registration/" + suite.testVehicleID)
		require.NoError(t, err)
		defer resp.Body.Close()
		
		assert.Equal(t, http.StatusOK, resp.StatusCode)
	})
	
	t.Run("Check Compliance", func(t *testing.T) {
		resp, err := http.Get(suite.externalIntegrationsURL + "/api/v1/uae/compliance-check/" + suite.testVehicleID)
		require.NoError(t, err)
		defer resp.Body.Close()
		
		assert.Equal(t, http.StatusOK, resp.StatusCode)
	})
	
	t.Run("Get Current Weather", func(t *testing.T) {
		resp, err := http.Get(suite.externalIntegrationsURL + "/api/v1/weather/current/Dubai")
		require.NoError(t, err)
		defer resp.Body.Close()
		
		assert.Equal(t, http.StatusOK, resp.StatusCode)
	})
	
	t.Run("Get Route", func(t *testing.T) {
		routeData := map[string]interface{}{
			"origin":      "Dubai Marina",
			"destination": "Dubai Mall",
			"mode":        "driving",
		}
		
		resp, err := suite.makeRequest("POST", suite.externalIntegrationsURL+"/api/v1/maps/route", routeData)
		require.NoError(t, err)
		defer resp.Body.Close()
		
		assert.Equal(t, http.StatusOK, resp.StatusCode)
	})
	
	t.Run("Sync ERP Data", func(t *testing.T) {
		resp, err := suite.makeRequest("POST", suite.externalIntegrationsURL+"/api/v1/erp/sync", map[string]interface{}{})
		require.NoError(t, err)
		defer resp.Body.Close()
		
		assert.Equal(t, http.StatusOK, resp.StatusCode)
	})
}

// Test Suite: ML Pipeline Integration

func TestMLPipelineIntegration(t *testing.T) {
	suite := &IntegrationTestSuite{}
	
	t.Run("Get Models", func(t *testing.T) {
		resp, err := http.Get(suite.mlPipelineURL + "/api/v1/models")
		require.NoError(t, err)
		defer resp.Body.Close()
		
		assert.Equal(t, http.StatusOK, resp.StatusCode)
	})
	
	t.Run("Predict Maintenance", func(t *testing.T) {
		predictionData := map[string]interface{}{
			"vehicle_id": suite.testVehicleID,
			"data": map[string]interface{}{
				"battery_level":  85.5,
				"engine_temp":    75.2,
				"mileage":       15000,
				"age":           2,
				"usage_pattern": "normal",
			},
		}
		
		resp, err := suite.makeRequest("POST", suite.mlPipelineURL+"/api/v1/predict/maintenance", predictionData)
		require.NoError(t, err)
		defer resp.Body.Close()
		
		assert.Equal(t, http.StatusOK, resp.StatusCode)
	})
	
	t.Run("Optimize Route", func(t *testing.T) {
		routeData := map[string]interface{}{
			"origin": map[string]float64{
				"latitude":  25.2048,
				"longitude": 55.2708,
			},
			"destination": map[string]float64{
				"latitude":  25.2582,
				"longitude": 55.3047,
			},
			"constraints": map[string]interface{}{
				"max_duration": 30,
				"avoid_tolls":  true,
			},
		}
		
		resp, err := suite.makeRequest("POST", suite.mlPipelineURL+"/api/v1/predict/route", routeData)
		require.NoError(t, err)
		defer resp.Body.Close()
		
		assert.Equal(t, http.StatusOK, resp.StatusCode)
	})
	
	t.Run("Forecast Demand", func(t *testing.T) {
		forecastData := map[string]interface{}{
			"location_id": "location-001",
			"time_window": "next_hour",
			"factors": map[string]interface{}{
				"weather":       "clear",
				"time_of_day":   "morning",
				"events":        "none",
			},
		}
		
		resp, err := suite.makeRequest("POST", suite.mlPipelineURL+"/api/v1/predict/demand", forecastData)
		require.NoError(t, err)
		defer resp.Body.Close()
		
		assert.Equal(t, http.StatusOK, resp.StatusCode)
	})
}

// Test Suite: End-to-End Workflow

func TestEndToEndWorkflow(t *testing.T) {
	suite := &IntegrationTestSuite{}
	
	t.Run("Complete Fleet Operation Workflow", func(t *testing.T) {
		// 1. Create organization and fleet
		orgData := map[string]interface{}{
			"name":        "E2E Test Organization",
			"description": "End-to-end test organization",
		}
		
		orgResp, err := suite.makeRequest("POST", suite.fleetManagerURL+"/api/v1/organizations", orgData)
		require.NoError(t, err)
		require.Equal(t, http.StatusCreated, orgResp.StatusCode)
		orgResp.Body.Close()
		
		// 2. Create vehicle
		vehicleData := map[string]interface{}{
			"fleet_id":       suite.testFleetID,
			"asset_tag":      "E2E-001",
			"manufacturer":   "Tesla",
			"model":          "Model 3",
			"autonomy_level": "L4",
		}
		
		vehicleResp, err := suite.makeRequest("POST", suite.fleetManagerURL+"/api/v1/fleet/vehicles", vehicleData)
		require.NoError(t, err)
		require.Equal(t, http.StatusCreated, vehicleResp.StatusCode)
		vehicleResp.Body.Close()
		
		// 3. Update vehicle location
		locationData := map[string]interface{}{
			"latitude":  25.2048,
			"longitude": 55.2708,
			"speed":     45.0,
		}
		
		locationResp, err := suite.makeRequest("PUT", suite.fleetManagerURL+"/api/v1/fleet/vehicles/"+suite.testVehicleID+"/location", locationData)
		require.NoError(t, err)
		require.Equal(t, http.StatusOK, locationResp.StatusCode)
		locationResp.Body.Close()
		
		// 4. Dispatch vehicle
		dispatchData := map[string]interface{}{
			"trip_id":     "e2e-trip-001",
			"origin":      map[string]float64{"latitude": 25.2048, "longitude": 55.2708},
			"destination": map[string]float64{"latitude": 25.2582, "longitude": 55.3047},
		}
		
		dispatchResp, err := suite.makeRequest("POST", suite.fleetManagerURL+"/api/v1/fleet/vehicles/"+suite.testVehicleID+"/dispatch", dispatchData)
		require.NoError(t, err)
		require.Equal(t, http.StatusOK, dispatchResp.StatusCode)
		dispatchResp.Body.Close()
		
		// 5. Get analytics
		analyticsResp, err := http.Get(suite.analyticsURL + "/api/v1/analytics/kpis?timeframe=1h")
		require.NoError(t, err)
		require.Equal(t, http.StatusOK, analyticsResp.StatusCode)
		analyticsResp.Body.Close()
		
		// 6. Get ML predictions
		predictionData := map[string]interface{}{
			"vehicle_id": suite.testVehicleID,
			"data": map[string]interface{}{
				"battery_level": 85.5,
				"engine_temp":   75.2,
			},
		}
		
		predictionResp, err := suite.makeRequest("POST", suite.mlPipelineURL+"/api/v1/predict/maintenance", predictionData)
		require.NoError(t, err)
		require.Equal(t, http.StatusOK, predictionResp.StatusCode)
		predictionResp.Body.Close()
		
		// 7. Check external integrations
		weatherResp, err := http.Get(suite.externalIntegrationsURL + "/api/v1/weather/current/Dubai")
		require.NoError(t, err)
		require.Equal(t, http.StatusOK, weatherResp.StatusCode)
		weatherResp.Body.Close()
	})
}

// Test Suite: Performance and Load Testing

func TestPerformanceAndLoad(t *testing.T) {
	suite := &IntegrationTestSuite{}
	
	t.Run("Concurrent Vehicle Operations", func(t *testing.T) {
		concurrency := 10
		operations := 100
		
		results := make(chan error, concurrency)
		
		for i := 0; i < concurrency; i++ {
			go func(workerID int) {
				for j := 0; j < operations/concurrency; j++ {
					// Simulate vehicle location update
					locationData := map[string]interface{}{
						"latitude":  25.2048 + float64(j)*0.001,
						"longitude": 55.2708 + float64(j)*0.001,
						"speed":     45.0 + float64(j),
					}
					
					resp, err := suite.makeRequest("PUT", suite.fleetManagerURL+"/api/v1/fleet/vehicles/"+suite.testVehicleID+"/location", locationData)
					if err != nil {
						results <- err
						return
					}
					resp.Body.Close()
				}
				results <- nil
			}(i)
		}
		
		// Wait for all workers to complete
		for i := 0; i < concurrency; i++ {
			err := <-results
			require.NoError(t, err)
		}
	})
	
	t.Run("Analytics Query Performance", func(t *testing.T) {
		start := time.Now()
		
		resp, err := http.Get(suite.analyticsURL + "/api/v1/analytics/kpis?timeframe=24h")
		require.NoError(t, err)
		defer resp.Body.Close()
		
		duration := time.Since(start)
		assert.Less(t, duration, 5*time.Second, "Analytics query should complete within 5 seconds")
		assert.Equal(t, http.StatusOK, resp.StatusCode)
	})
}

// Test Suite: Security Testing

func TestSecurity(t *testing.T) {
	suite := &IntegrationTestSuite{}
	
	t.Run("Unauthorized Access", func(t *testing.T) {
		// Test without authentication token
		req, err := http.NewRequest("GET", suite.fleetManagerURL+"/api/v1/fleet/vehicles", nil)
		require.NoError(t, err)
		
		client := &http.Client{}
		resp, err := client.Do(req)
		require.NoError(t, err)
		defer resp.Body.Close()
		
		// Should return 401 Unauthorized
		assert.Equal(t, http.StatusUnauthorized, resp.StatusCode)
	})
	
	t.Run("Invalid Authentication Token", func(t *testing.T) {
		req, err := http.NewRequest("GET", suite.fleetManagerURL+"/api/v1/fleet/vehicles", nil)
		require.NoError(t, err)
		req.Header.Set("Authorization", "Bearer invalid-token")
		
		client := &http.Client{}
		resp, err := client.Do(req)
		require.NoError(t, err)
		defer resp.Body.Close()
		
		// Should return 401 Unauthorized
		assert.Equal(t, http.StatusUnauthorized, resp.StatusCode)
	})
	
	t.Run("SQL Injection Prevention", func(t *testing.T) {
		// Test with malicious input
		maliciousData := map[string]interface{}{
			"fleet_id":    "'; DROP TABLE vehicles; --",
			"asset_tag":   "'; DROP TABLE vehicles; --",
			"manufacturer": "Tesla",
			"model":       "Model 3",
		}
		
		resp, err := suite.makeRequest("POST", suite.fleetManagerURL+"/api/v1/fleet/vehicles", maliciousData)
		require.NoError(t, err)
		defer resp.Body.Close()
		
		// Should handle malicious input gracefully
		assert.NotEqual(t, http.StatusInternalServerError, resp.StatusCode)
	})
}

// Helper Methods

func (suite *IntegrationTestSuite) makeRequest(method, url string, data interface{}) (*http.Response, error) {
	jsonData, err := json.Marshal(data)
	if err != nil {
		return nil, err
	}
	
	req, err := http.NewRequest(method, url, bytes.NewBuffer(jsonData))
	if err != nil {
		return nil, err
	}
	
	req.Header.Set("Content-Type", "application/json")
	req.Header.Set("Authorization", "Bearer test-token")
	
	client := &http.Client{}
	return client.Do(req)
}

// Test Suite: Error Handling and Recovery

func TestErrorHandlingAndRecovery(t *testing.T) {
	suite := &IntegrationTestSuite{}
	
	t.Run("Invalid Vehicle ID", func(t *testing.T) {
		resp, err := http.Get(suite.fleetManagerURL + "/api/v1/fleet/vehicles/invalid-id/status")
		require.NoError(t, err)
		defer resp.Body.Close()
		
		assert.Equal(t, http.StatusNotFound, resp.StatusCode)
	})
	
	t.Run("Invalid JSON Payload", func(t *testing.T) {
		req, err := http.NewRequest("POST", suite.fleetManagerURL+"/api/v1/fleet/vehicles", strings.NewReader("invalid json"))
		require.NoError(t, err)
		req.Header.Set("Content-Type", "application/json")
		
		client := &http.Client{}
		resp, err := client.Do(req)
		require.NoError(t, err)
		defer resp.Body.Close()
		
		assert.Equal(t, http.StatusBadRequest, resp.StatusCode)
	})
	
	t.Run("Missing Required Fields", func(t *testing.T) {
		incompleteData := map[string]interface{}{
			"asset_tag": "TEST-002",
			// Missing required fields: fleet_id, manufacturer, model
		}
		
		resp, err := suite.makeRequest("POST", suite.fleetManagerURL+"/api/v1/fleet/vehicles", incompleteData)
		require.NoError(t, err)
		defer resp.Body.Close()
		
		assert.Equal(t, http.StatusBadRequest, resp.StatusCode)
	})
}

// Test Suite: Data Consistency

func TestDataConsistency(t *testing.T) {
	suite := &IntegrationTestSuite{}
	
	t.Run("Vehicle Creation and Retrieval", func(t *testing.T) {
		// Create vehicle
		vehicleData := map[string]interface{}{
			"fleet_id":     suite.testFleetID,
			"asset_tag":    "CONSISTENCY-001",
			"manufacturer": "BMW",
			"model":        "iX",
			"year":         2023,
		}
		
		createResp, err := suite.makeRequest("POST", suite.fleetManagerURL+"/api/v1/fleet/vehicles", vehicleData)
		require.NoError(t, err)
		require.Equal(t, http.StatusCreated, createResp.StatusCode)
		
		var createResult map[string]interface{}
		err = json.NewDecoder(createResp.Body).Decode(&createResult)
		require.NoError(t, err)
		createResp.Body.Close()
		
		vehicleID := createResult["vehicle_id"].(string)
		
		// Retrieve vehicle
		getResp, err := http.Get(suite.fleetManagerURL + "/api/v1/fleet/vehicles/" + vehicleID)
		require.NoError(t, err)
		defer getResp.Body.Close()
		
		assert.Equal(t, http.StatusOK, getResp.StatusCode)
		
		var getResult map[string]interface{}
		err = json.NewDecoder(getResp.Body).Decode(&getResult)
		require.NoError(t, err)
		
		assert.Equal(t, vehicleData["asset_tag"], getResult["asset_tag"])
		assert.Equal(t, vehicleData["manufacturer"], getResult["manufacturer"])
		assert.Equal(t, vehicleData["model"], getResult["model"])
	})
}
