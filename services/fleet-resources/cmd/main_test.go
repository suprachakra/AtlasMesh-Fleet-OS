package main

import (
	"bytes"
	"context"
	"encoding/json"
	"net/http"
	"net/http/httptest"
	"testing"
	"time"

	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/mock"
)

// Mock dependencies
type MockDatabase struct {
	mock.Mock
}

func (m *MockDatabase) QueryContext(ctx context.Context, query string, args ...interface{}) (interface{}, error) {
	mockArgs := m.Called(ctx, query, args)
	return mockArgs.Get(0), mockArgs.Error(1)
}

func (m *MockDatabase) ExecContext(ctx context.Context, query string, args ...interface{}) (interface{}, error) {
	mockArgs := m.Called(ctx, query, args)
	return mockArgs.Get(0), mockArgs.Error(1)
}

type MockRedisClient struct {
	mock.Mock
}

func (m *MockRedisClient) Set(ctx context.Context, key string, value interface{}, expiration time.Duration) error {
	args := m.Called(ctx, key, value, expiration)
	return args.Error(0)
}

func (m *MockRedisClient) Get(ctx context.Context, key string) (string, error) {
	args := m.Called(ctx, key)
	return args.String(0), args.Error(1)
}

func (m *MockRedisClient) Ping(ctx context.Context) error {
	args := m.Called(ctx)
	return args.Error(0)
}

type MockKafkaWriter struct {
	mock.Mock
}

func (m *MockKafkaWriter) WriteMessages(ctx context.Context, msgs ...interface{}) error {
	args := m.Called(ctx, msgs)
	return args.Error(0)
}

func (m *MockKafkaWriter) Close() error {
	args := m.Called()
	return args.Error(0)
}

func TestLoadConfig(t *testing.T) {
	config := loadConfig()
	
	assert.NotNil(t, config)
	assert.Equal(t, 8080, config.Port)
	assert.Equal(t, "info", config.LogLevel)
	assert.Equal(t, 1000, config.BufferSize)
}

func TestHealthCheck(t *testing.T) {
	// Create test service with mocks
	mockDB := &MockDatabase{}
	mockRedis := &MockRedisClient{}
	mockKafka := &MockKafkaWriter{}
	
	service := &ResourceManagementService{
		config:      loadConfig(),
		database:    mockDB,
		redis:       mockRedis,
		kafkaWriter: mockKafka,
	}
	
	// Mock Redis ping success
	mockRedis.On("Ping", mock.Anything).Return(nil)
	
	// Create test request
	req := httptest.NewRequest("GET", "/health", nil)
	w := httptest.NewRecorder()
	
	// Call health check
	service.healthCheck(w, req)
	
	// Assertions
	assert.Equal(t, http.StatusOK, w.Code)
	
	var response map[string]interface{}
	err := json.Unmarshal(w.Body.Bytes(), &response)
	assert.NoError(t, err)
	assert.Equal(t, "healthy", response["status"])
	
	mockRedis.AssertExpectations(t)
}

func TestListResources(t *testing.T) {
	// Create test service with mocks
	mockDB := &MockDatabase{}
	mockRedis := &MockRedisClient{}
	mockKafka := &MockKafkaWriter{}
	
	service := &ResourceManagementService{
		config:      loadConfig(),
		database:    mockDB,
		redis:       mockRedis,
		kafkaWriter: mockKafka,
	}
	
	// Mock database query success
	mockDB.On("QueryContext", mock.Anything, mock.Anything, mock.Anything).Return([]interface{}{}, nil)
	
	// Create test request
	req := httptest.NewRequest("GET", "/api/v1/resources", nil)
	w := httptest.NewRecorder()
	
	// Call list resources
	service.listResources(w, req)
	
	// Assertions
	assert.Equal(t, http.StatusOK, w.Code)
	
	var response map[string]interface{}
	err := json.Unmarshal(w.Body.Bytes(), &response)
	assert.NoError(t, err)
	assert.NotNil(t, response["resources"])
	
	mockDB.AssertExpectations(t)
}

func TestCreateResource(t *testing.T) {
	// Create test service with mocks
	mockDB := &MockDatabase{}
	mockRedis := &MockRedisClient{}
	mockKafka := &MockKafkaWriter{}
	
	service := &ResourceManagementService{
		config:      loadConfig(),
		database:    mockDB,
		redis:       mockRedis,
		kafkaWriter: mockKafka,
	}
	
	// Test resource data
	resource := map[string]interface{}{
		"name":        "Test Vehicle",
		"description": "Test vehicle for fleet operations",
		"type":        "vehicle",
		"fleet_id":    "fleet-001",
		"status":      "available",
		"capacity":    1000.0,
		"location": map[string]interface{}{
			"latitude":  24.4539,
			"longitude": 54.3773,
		},
		"metadata": map[string]interface{}{
			"make":  "Tesla",
			"model": "Model 3",
			"year":  2024,
		},
	}
	
	// Mock database exec success
	mockDB.On("ExecContext", mock.Anything, mock.Anything, mock.Anything).Return(nil, nil)
	
	// Mock Redis set success
	mockRedis.On("Set", mock.Anything, mock.Anything, mock.Anything, mock.Anything).Return(nil)
	
	// Mock Kafka write success
	mockKafka.On("WriteMessages", mock.Anything, mock.Anything).Return(nil)
	
	// Create test request
	body, _ := json.Marshal(resource)
	req := httptest.NewRequest("POST", "/api/v1/resources", bytes.NewReader(body))
	req.Header.Set("Content-Type", "application/json")
	w := httptest.NewRecorder()
	
	// Call create resource
	service.createResource(w, req)
	
	// Assertions
	assert.Equal(t, http.StatusCreated, w.Code)
	
	var response map[string]interface{}
	err := json.Unmarshal(w.Body.Bytes(), &response)
	assert.NoError(t, err)
	assert.Equal(t, "resource_created", response["status"])
	
	mockDB.AssertExpectations(t)
	mockRedis.AssertExpectations(t)
	mockKafka.AssertExpectations(t)
}

func TestCreateResourceInvalidJSON(t *testing.T) {
	service := &ResourceManagementService{
		config: loadConfig(),
	}
	
	// Create test request with invalid JSON
	req := httptest.NewRequest("POST", "/api/v1/resources", bytes.NewReader([]byte("invalid json")))
	req.Header.Set("Content-Type", "application/json")
	w := httptest.NewRecorder()
	
	// Call create resource
	service.createResource(w, req)
	
	// Assertions
	assert.Equal(t, http.StatusBadRequest, w.Code)
}

func TestGetResource(t *testing.T) {
	// Create test service with mocks
	mockDB := &MockDatabase{}
	mockRedis := &MockRedisClient{}
	mockKafka := &MockKafkaWriter{}
	
	service := &ResourceManagementService{
		config:      loadConfig(),
		database:    mockDB,
		redis:       mockRedis,
		kafkaWriter: mockKafka,
	}
	
	// Mock database query success
	mockDB.On("QueryContext", mock.Anything, mock.Anything, mock.Anything).Return([]interface{}{}, nil)
	
	// Create test request
	req := httptest.NewRequest("GET", "/api/v1/resources/test-resource-001", nil)
	w := httptest.NewRecorder()
	
	// Call get resource
	service.getResource(w, req)
	
	// Assertions
	assert.Equal(t, http.StatusOK, w.Code)
	
	var response map[string]interface{}
	err := json.Unmarshal(w.Body.Bytes(), &response)
	assert.NoError(t, err)
	assert.NotNil(t, response["resource"])
	
	mockDB.AssertExpectations(t)
}

func TestUpdateResource(t *testing.T) {
	// Create test service with mocks
	mockDB := &MockDatabase{}
	mockRedis := &MockRedisClient{}
	mockKafka := &MockKafkaWriter{}
	
	service := &ResourceManagementService{
		config:      loadConfig(),
		database:    mockDB,
		redis:       mockRedis,
		kafkaWriter: mockKafka,
	}
	
	// Test resource update data
	updateData := map[string]interface{}{
		"name":        "Updated Vehicle",
		"description": "Updated vehicle description",
		"status":      "in_use",
		"capacity":    1200.0,
	}
	
	// Mock database exec success
	mockDB.On("ExecContext", mock.Anything, mock.Anything, mock.Anything).Return(nil, nil)
	
	// Mock Redis set success
	mockRedis.On("Set", mock.Anything, mock.Anything, mock.Anything, mock.Anything).Return(nil)
	
	// Mock Kafka write success
	mockKafka.On("WriteMessages", mock.Anything, mock.Anything).Return(nil)
	
	// Create test request
	body, _ := json.Marshal(updateData)
	req := httptest.NewRequest("PUT", "/api/v1/resources/test-resource-001", bytes.NewReader(body))
	req.Header.Set("Content-Type", "application/json")
	w := httptest.NewRecorder()
	
	// Call update resource
	service.updateResource(w, req)
	
	// Assertions
	assert.Equal(t, http.StatusOK, w.Code)
	
	var response map[string]interface{}
	err := json.Unmarshal(w.Body.Bytes(), &response)
	assert.NoError(t, err)
	assert.Equal(t, "resource_updated", response["status"])
	
	mockDB.AssertExpectations(t)
	mockRedis.AssertExpectations(t)
	mockKafka.AssertExpectations(t)
}

func TestDeleteResource(t *testing.T) {
	// Create test service with mocks
	mockDB := &MockDatabase{}
	mockRedis := &MockRedisClient{}
	mockKafka := &MockKafkaWriter{}
	
	service := &ResourceManagementService{
		config:      loadConfig(),
		database:    mockDB,
		redis:       mockRedis,
		kafkaWriter: mockKafka,
	}
	
	// Mock database exec success
	mockDB.On("ExecContext", mock.Anything, mock.Anything, mock.Anything).Return(nil, nil)
	
	// Mock Redis delete success
	mockRedis.On("Del", mock.Anything, mock.Anything).Return(nil)
	
	// Mock Kafka write success
	mockKafka.On("WriteMessages", mock.Anything, mock.Anything).Return(nil)
	
	// Create test request
	req := httptest.NewRequest("DELETE", "/api/v1/resources/test-resource-001", nil)
	w := httptest.NewRecorder()
	
	// Call delete resource
	service.deleteResource(w, req)
	
	// Assertions
	assert.Equal(t, http.StatusOK, w.Code)
	
	var response map[string]interface{}
	err := json.Unmarshal(w.Body.Bytes(), &response)
	assert.NoError(t, err)
	assert.Equal(t, "resource_deleted", response["status"])
	
	mockDB.AssertExpectations(t)
	mockRedis.AssertExpectations(t)
	mockKafka.AssertExpectations(t)
}

func TestAllocateResource(t *testing.T) {
	// Create test service with mocks
	mockDB := &MockDatabase{}
	mockRedis := &MockRedisClient{}
	mockKafka := &MockKafkaWriter{}
	
	service := &ResourceManagementService{
		config:      loadConfig(),
		database:    mockDB,
		redis:       mockRedis,
		kafkaWriter: mockKafka,
	}
	
	// Test resource allocation data
	allocation := map[string]interface{}{
		"resource_id": "test-resource-001",
		"mission_id":  "test-mission-001",
		"allocated_by": "fleet-manager",
		"start_time":  time.Now().Format(time.RFC3339),
		"end_time":    time.Now().Add(2 * time.Hour).Format(time.RFC3339),
		"priority":    "high",
	}
	
	// Mock database exec success
	mockDB.On("ExecContext", mock.Anything, mock.Anything, mock.Anything).Return(nil, nil)
	
	// Mock Redis set success
	mockRedis.On("Set", mock.Anything, mock.Anything, mock.Anything, mock.Anything).Return(nil)
	
	// Mock Kafka write success
	mockKafka.On("WriteMessages", mock.Anything, mock.Anything).Return(nil)
	
	// Create test request
	body, _ := json.Marshal(allocation)
	req := httptest.NewRequest("POST", "/api/v1/resources/test-resource-001/allocate", bytes.NewReader(body))
	req.Header.Set("Content-Type", "application/json")
	w := httptest.NewRecorder()
	
	// Call allocate resource
	service.allocateResource(w, req)
	
	// Assertions
	assert.Equal(t, http.StatusOK, w.Code)
	
	var response map[string]interface{}
	err := json.Unmarshal(w.Body.Bytes(), &response)
	assert.NoError(t, err)
	assert.Equal(t, "resource_allocated", response["status"])
	
	mockDB.AssertExpectations(t)
	mockRedis.AssertExpectations(t)
	mockKafka.AssertExpectations(t)
}

func TestDeallocateResource(t *testing.T) {
	// Create test service with mocks
	mockDB := &MockDatabase{}
	mockRedis := &MockRedisClient{}
	mockKafka := &MockKafkaWriter{}
	
	service := &ResourceManagementService{
		config:      loadConfig(),
		database:    mockDB,
		redis:       mockRedis,
		kafkaWriter: mockKafka,
	}
	
	// Mock database exec success
	mockDB.On("ExecContext", mock.Anything, mock.Anything, mock.Anything).Return(nil, nil)
	
	// Mock Redis set success
	mockRedis.On("Set", mock.Anything, mock.Anything, mock.Anything, mock.Anything).Return(nil)
	
	// Mock Kafka write success
	mockKafka.On("WriteMessages", mock.Anything, mock.Anything).Return(nil)
	
	// Create test request
	req := httptest.NewRequest("POST", "/api/v1/resources/test-resource-001/deallocate", nil)
	w := httptest.NewRecorder()
	
	// Call deallocate resource
	service.deallocateResource(w, req)
	
	// Assertions
	assert.Equal(t, http.StatusOK, w.Code)
	
	var response map[string]interface{}
	err := json.Unmarshal(w.Body.Bytes(), &response)
	assert.NoError(t, err)
	assert.Equal(t, "resource_deallocated", response["status"])
	
	mockDB.AssertExpectations(t)
	mockRedis.AssertExpectations(t)
	mockKafka.AssertExpectations(t)
}

func TestGetResourceUtilization(t *testing.T) {
	// Create test service with mocks
	mockDB := &MockDatabase{}
	mockRedis := &MockRedisClient{}
	mockKafka := &MockKafkaWriter{}
	
	service := &ResourceManagementService{
		config:      loadConfig(),
		database:    mockDB,
		redis:       mockRedis,
		kafkaWriter: mockKafka,
	}
	
	// Mock database query success
	mockDB.On("QueryContext", mock.Anything, mock.Anything, mock.Anything).Return([]interface{}{}, nil)
	
	// Create test request
	req := httptest.NewRequest("GET", "/api/v1/resources/test-resource-001/utilization", nil)
	w := httptest.NewRecorder()
	
	// Call get resource utilization
	service.getResourceUtilization(w, req)
	
	// Assertions
	assert.Equal(t, http.StatusOK, w.Code)
	
	var response map[string]interface{}
	err := json.Unmarshal(w.Body.Bytes(), &response)
	assert.NoError(t, err)
	assert.NotNil(t, response["utilization"])
	
	mockDB.AssertExpectations(t)
}

func TestListResourceTypes(t *testing.T) {
	// Create test service with mocks
	mockDB := &MockDatabase{}
	mockRedis := &MockRedisClient{}
	mockKafka := &MockKafkaWriter{}
	
	service := &ResourceManagementService{
		config:      loadConfig(),
		database:    mockDB,
		redis:       mockRedis,
		kafkaWriter: mockKafka,
	}
	
	// Mock database query success
	mockDB.On("QueryContext", mock.Anything, mock.Anything, mock.Anything).Return([]interface{}{}, nil)
	
	// Create test request
	req := httptest.NewRequest("GET", "/api/v1/resource-types", nil)
	w := httptest.NewRecorder()
	
	// Call list resource types
	service.listResourceTypes(w, req)
	
	// Assertions
	assert.Equal(t, http.StatusOK, w.Code)
	
	var response map[string]interface{}
	err := json.Unmarshal(w.Body.Bytes(), &response)
	assert.NoError(t, err)
	assert.NotNil(t, response["resource_types"])
	
	mockDB.AssertExpectations(t)
}

func TestCreateResourceType(t *testing.T) {
	// Create test service with mocks
	mockDB := &MockDatabase{}
	mockRedis := &MockRedisClient{}
	mockKafka := &MockKafkaWriter{}
	
	service := &ResourceManagementService{
		config:      loadConfig(),
		database:    mockDB,
		redis:       mockRedis,
		kafkaWriter: mockKafka,
	}
	
	// Test resource type data
	resourceType := map[string]interface{}{
		"name":        "Autonomous Vehicle",
		"description": "Autonomous vehicle resource type",
		"category":    "vehicle",
		"attributes": []map[string]interface{}{
			{
				"name":        "capacity",
				"type":        "number",
				"required":    true,
				"unit":        "kg",
			},
			{
				"name":        "autonomy_level",
				"type":        "string",
				"required":    true,
				"values":      []string{"level_1", "level_2", "level_3", "level_4", "level_5"},
			},
		},
	}
	
	// Mock database exec success
	mockDB.On("ExecContext", mock.Anything, mock.Anything, mock.Anything).Return(nil, nil)
	
	// Mock Redis set success
	mockRedis.On("Set", mock.Anything, mock.Anything, mock.Anything, mock.Anything).Return(nil)
	
	// Mock Kafka write success
	mockKafka.On("WriteMessages", mock.Anything, mock.Anything).Return(nil)
	
	// Create test request
	body, _ := json.Marshal(resourceType)
	req := httptest.NewRequest("POST", "/api/v1/resource-types", bytes.NewReader(body))
	req.Header.Set("Content-Type", "application/json")
	w := httptest.NewRecorder()
	
	// Call create resource type
	service.createResourceType(w, req)
	
	// Assertions
	assert.Equal(t, http.StatusCreated, w.Code)
	
	var response map[string]interface{}
	err := json.Unmarshal(w.Body.Bytes(), &response)
	assert.NoError(t, err)
	assert.Equal(t, "resource_type_created", response["status"])
	
	mockDB.AssertExpectations(t)
	mockRedis.AssertExpectations(t)
	mockKafka.AssertExpectations(t)
}

func TestGetResourceType(t *testing.T) {
	// Create test service with mocks
	mockDB := &MockDatabase{}
	mockRedis := &MockRedisClient{}
	mockKafka := &MockKafkaWriter{}
	
	service := &ResourceManagementService{
		config:      loadConfig(),
		database:    mockDB,
		redis:       mockRedis,
		kafkaWriter: mockKafka,
	}
	
	// Mock database query success
	mockDB.On("QueryContext", mock.Anything, mock.Anything, mock.Anything).Return([]interface{}{}, nil)
	
	// Create test request
	req := httptest.NewRequest("GET", "/api/v1/resource-types/test-type-001", nil)
	w := httptest.NewRecorder()
	
	// Call get resource type
	service.getResourceType(w, req)
	
	// Assertions
	assert.Equal(t, http.StatusOK, w.Code)
	
	var response map[string]interface{}
	err := json.Unmarshal(w.Body.Bytes(), &response)
	assert.NoError(t, err)
	assert.NotNil(t, response["resource_type"])
	
	mockDB.AssertExpectations(t)
}

func TestUpdateResourceType(t *testing.T) {
	// Create test service with mocks
	mockDB := &MockDatabase{}
	mockRedis := &MockRedisClient{}
	mockKafka := &MockKafkaWriter{}
	
	service := &ResourceManagementService{
		config:      loadConfig(),
		database:    mockDB,
		redis:       mockRedis,
		kafkaWriter: mockKafka,
	}
	
	// Test resource type update data
	updateData := map[string]interface{}{
		"name":        "Updated Autonomous Vehicle",
		"description": "Updated autonomous vehicle resource type",
		"category":    "vehicle",
	}
	
	// Mock database exec success
	mockDB.On("ExecContext", mock.Anything, mock.Anything, mock.Anything).Return(nil, nil)
	
	// Mock Redis set success
	mockRedis.On("Set", mock.Anything, mock.Anything, mock.Anything, mock.Anything).Return(nil)
	
	// Mock Kafka write success
	mockKafka.On("WriteMessages", mock.Anything, mock.Anything).Return(nil)
	
	// Create test request
	body, _ := json.Marshal(updateData)
	req := httptest.NewRequest("PUT", "/api/v1/resource-types/test-type-001", bytes.NewReader(body))
	req.Header.Set("Content-Type", "application/json")
	w := httptest.NewRecorder()
	
	// Call update resource type
	service.updateResourceType(w, req)
	
	// Assertions
	assert.Equal(t, http.StatusOK, w.Code)
	
	var response map[string]interface{}
	err := json.Unmarshal(w.Body.Bytes(), &response)
	assert.NoError(t, err)
	assert.Equal(t, "resource_type_updated", response["status"])
	
	mockDB.AssertExpectations(t)
	mockRedis.AssertExpectations(t)
	mockKafka.AssertExpectations(t)
}

func TestDeleteResourceType(t *testing.T) {
	// Create test service with mocks
	mockDB := &MockDatabase{}
	mockRedis := &MockRedisClient{}
	mockKafka := &MockKafkaWriter{}
	
	service := &ResourceManagementService{
		config:      loadConfig(),
		database:    mockDB,
		redis:       mockRedis,
		kafkaWriter: mockKafka,
	}
	
	// Mock database exec success
	mockDB.On("ExecContext", mock.Anything, mock.Anything, mock.Anything).Return(nil, nil)
	
	// Mock Redis delete success
	mockRedis.On("Del", mock.Anything, mock.Anything).Return(nil)
	
	// Mock Kafka write success
	mockKafka.On("WriteMessages", mock.Anything, mock.Anything).Return(nil)
	
	// Create test request
	req := httptest.NewRequest("DELETE", "/api/v1/resource-types/test-type-001", nil)
	w := httptest.NewRecorder()
	
	// Call delete resource type
	service.deleteResourceType(w, req)
	
	// Assertions
	assert.Equal(t, http.StatusOK, w.Code)
	
	var response map[string]interface{}
	err := json.Unmarshal(w.Body.Bytes(), &response)
	assert.NoError(t, err)
	assert.Equal(t, "resource_type_deleted", response["status"])
	
	mockDB.AssertExpectations(t)
	mockRedis.AssertExpectations(t)
	mockKafka.AssertExpectations(t)
}

func TestValidateResourceInput(t *testing.T) {
	service := &ResourceManagementService{
		config: loadConfig(),
	}
	
	// Valid resource
	validResource := map[string]interface{}{
		"name":        "Test Vehicle",
		"description": "Test vehicle for fleet operations",
		"type":        "vehicle",
		"fleet_id":    "fleet-001",
		"status":      "available",
		"capacity":    1000.0,
		"location": map[string]interface{}{
			"latitude":  24.4539,
			"longitude": 54.3773,
		},
		"metadata": map[string]interface{}{
			"make":  "Tesla",
			"model": "Model 3",
			"year":  2024,
		},
	}
	
	err := service.validateResourceInput(validResource)
	assert.NoError(t, err)
	
	// Invalid resource - missing name
	invalidResource := map[string]interface{}{
		"description": "Test vehicle for fleet operations",
		"type":        "vehicle",
		"fleet_id":    "fleet-001",
		"status":      "available",
		"capacity":    1000.0,
	}
	
	err = service.validateResourceInput(invalidResource)
	assert.Error(t, err)
	assert.Contains(t, err.Error(), "name is required")
	
	// Invalid resource - invalid status
	invalidStatusResource := map[string]interface{}{
		"name":        "Test Vehicle",
		"description": "Test vehicle for fleet operations",
		"type":        "vehicle",
		"fleet_id":    "fleet-001",
		"status":      "invalid_status",
		"capacity":    1000.0,
	}
	
	err = service.validateResourceInput(invalidStatusResource)
	assert.Error(t, err)
	assert.Contains(t, err.Error(), "invalid status")
	
	// Invalid resource - negative capacity
	invalidCapacityResource := map[string]interface{}{
		"name":        "Test Vehicle",
		"description": "Test vehicle for fleet operations",
		"type":        "vehicle",
		"fleet_id":    "fleet-001",
		"status":      "available",
		"capacity":    -100.0,
	}
	
	err = service.validateResourceInput(invalidCapacityResource)
	assert.Error(t, err)
	assert.Contains(t, err.Error(), "capacity must be positive")
}

func TestValidateResourceTypeInput(t *testing.T) {
	service := &ResourceManagementService{
		config: loadConfig(),
	}
	
	// Valid resource type
	validResourceType := map[string]interface{}{
		"name":        "Autonomous Vehicle",
		"description": "Autonomous vehicle resource type",
		"category":    "vehicle",
		"attributes": []map[string]interface{}{
			{
				"name":     "capacity",
				"type":     "number",
				"required": true,
				"unit":     "kg",
			},
		},
	}
	
	err := service.validateResourceTypeInput(validResourceType)
	assert.NoError(t, err)
	
	// Invalid resource type - missing name
	invalidResourceType := map[string]interface{}{
		"description": "Autonomous vehicle resource type",
		"category":    "vehicle",
		"attributes": []map[string]interface{}{
			{
				"name":     "capacity",
				"type":     "number",
				"required": true,
				"unit":     "kg",
			},
		},
	}
	
	err = service.validateResourceTypeInput(invalidResourceType)
	assert.Error(t, err)
	assert.Contains(t, err.Error(), "name is required")
	
	// Invalid resource type - empty attributes
	invalidAttributesResourceType := map[string]interface{}{
		"name":        "Autonomous Vehicle",
		"description": "Autonomous vehicle resource type",
		"category":    "vehicle",
		"attributes":  []map[string]interface{}{},
	}
	
	err = service.validateResourceTypeInput(invalidAttributesResourceType)
	assert.Error(t, err)
	assert.Contains(t, err.Error(), "at least one attribute is required")
}

func TestValidateResourceAllocationInput(t *testing.T) {
	service := &ResourceManagementService{
		config: loadConfig(),
	}
	
	// Valid resource allocation
	validAllocation := map[string]interface{}{
		"resource_id": "test-resource-001",
		"mission_id":  "test-mission-001",
		"allocated_by": "fleet-manager",
		"start_time": time.Now().Format(time.RFC3339),
		"end_time":    time.Now().Add(2 * time.Hour).Format(time.RFC3339),
		"priority":    "high",
	}
	
	err := service.validateResourceAllocationInput(validAllocation)
	assert.NoError(t, err)
	
	// Invalid resource allocation - missing resource_id
	invalidAllocation := map[string]interface{}{
		"mission_id":  "test-mission-001",
		"allocated_by": "fleet-manager",
		"start_time": time.Now().Format(time.RFC3339),
		"end_time":    time.Now().Add(2 * time.Hour).Format(time.RFC3339),
		"priority":    "high",
	}
	
	err = service.validateResourceAllocationInput(invalidAllocation)
	assert.Error(t, err)
	assert.Contains(t, err.Error(), "resource_id is required")
	
	// Invalid resource allocation - invalid priority
	invalidPriorityAllocation := map[string]interface{}{
		"resource_id": "test-resource-001",
		"mission_id":  "test-mission-001",
		"allocated_by": "fleet-manager",
		"start_time": time.Now().Format(time.RFC3339),
		"end_time":    time.Now().Add(2 * time.Hour).Format(time.RFC3339),
		"priority":    "invalid_priority",
	}
	
	err = service.validateResourceAllocationInput(invalidPriorityAllocation)
	assert.Error(t, err)
	assert.Contains(t, err.Error(), "invalid priority")
	
	// Invalid resource allocation - start_time after end_time
	invalidTimeAllocation := map[string]interface{}{
		"resource_id": "test-resource-001",
		"mission_id":  "test-mission-001",
		"allocated_by": "fleet-manager",
		"start_time": time.Now().Add(2 * time.Hour).Format(time.RFC3339),
		"end_time":    time.Now().Format(time.RFC3339),
		"priority":    "high",
	}
	
	err = service.validateResourceAllocationInput(invalidTimeAllocation)
	assert.Error(t, err)
	assert.Contains(t, err.Error(), "start_time must be before end_time")
}

func TestGenerateUUID(t *testing.T) {
	service := &ResourceManagementService{
		config: loadConfig(),
	}
	
	// Test UUID generation
	uuid1 := service.generateUUID()
	uuid2 := service.generateUUID()
	
	assert.NotEmpty(t, uuid1)
	assert.NotEmpty(t, uuid2)
	assert.NotEqual(t, uuid1, uuid2)
	assert.Len(t, uuid1, 36) // UUID v4 format
	assert.Len(t, uuid2, 36) // UUID v4 format
}

func TestSetupRoutes(t *testing.T) {
	service := &ResourceManagementService{
		config: loadConfig(),
	}
	
	// Create test router
	router := service.setupRoutes()
	
	// Test that routes are registered
	assert.NotNil(t, router)
}

func TestLoadConfigWithEnvVars(t *testing.T) {
	// Test with environment variables
	// Note: In a real test, you would set environment variables
	config := loadConfig()
	
	assert.NotNil(t, config)
	assert.Equal(t, 8080, config.Port)
	assert.Equal(t, "info", config.LogLevel)
	assert.Equal(t, 1000, config.BufferSize)
	assert.Equal(t, 5*time.Minute, config.RedisTTL)
}

func TestGenerateCorrelationID(t *testing.T) {
	service := &ResourceManagementService{
		config: loadConfig(),
	}
	
	// Test correlation ID generation
	corrID1 := service.generateCorrelationID()
	corrID2 := service.generateCorrelationID()
	
	assert.NotEmpty(t, corrID1)
	assert.NotEmpty(t, corrID2)
	assert.NotEqual(t, corrID1, corrID2)
}

func TestGenerateMessageID(t *testing.T) {
	service := &ResourceManagementService{
		config: loadConfig(),
	}
	
	// Test message ID generation
	msgID1 := service.generateMessageID()
	msgID2 := service.generateMessageID()
	
	assert.NotEmpty(t, msgID1)
	assert.NotEmpty(t, msgID2)
	assert.NotEqual(t, msgID1, msgID2)
}

// Benchmark tests
func BenchmarkCreateResource(b *testing.B) {
	service := &ResourceManagementService{
		config: loadConfig(),
	}
	
	resource := map[string]interface{}{
		"name":        "Test Vehicle",
		"description": "Test vehicle for fleet operations",
		"type":        "vehicle",
		"fleet_id":    "fleet-001",
		"status":      "available",
		"capacity":    1000.0,
		"location": map[string]interface{}{
			"latitude":  24.4539,
			"longitude": 54.3773,
		},
		"metadata": map[string]interface{}{
			"make":  "Tesla",
			"model": "Model 3",
			"year":  2024,
		},
	}
	
	body, _ := json.Marshal(resource)
	
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		req := httptest.NewRequest("POST", "/api/v1/resources", bytes.NewReader(body))
		req.Header.Set("Content-Type", "application/json")
		w := httptest.NewRecorder()
		
		service.createResource(w, req)
	}
}

func BenchmarkListResources(b *testing.B) {
	service := &ResourceManagementService{
		config: loadConfig(),
	}
	
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		req := httptest.NewRequest("GET", "/api/v1/resources", nil)
		w := httptest.NewRecorder()
		
		service.listResources(w, req)
	}
}
