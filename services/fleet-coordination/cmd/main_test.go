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
	
	service := &FleetCoordinationService{
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

func TestListFederations(t *testing.T) {
	// Create test service with mocks
	mockDB := &MockDatabase{}
	mockRedis := &MockRedisClient{}
	mockKafka := &MockKafkaWriter{}
	
	service := &FleetCoordinationService{
		config:      loadConfig(),
		database:    mockDB,
		redis:       mockRedis,
		kafkaWriter: mockKafka,
	}
	
	// Mock database query success
	mockDB.On("QueryContext", mock.Anything, mock.Anything, mock.Anything).Return([]interface{}{}, nil)
	
	// Create test request
	req := httptest.NewRequest("GET", "/api/v1/federations", nil)
	w := httptest.NewRecorder()
	
	// Call list federations
	service.listFederations(w, req)
	
	// Assertions
	assert.Equal(t, http.StatusOK, w.Code)
	
	var response map[string]interface{}
	err := json.Unmarshal(w.Body.Bytes(), &response)
	assert.NoError(t, err)
	assert.NotNil(t, response["federations"])
	
	mockDB.AssertExpectations(t)
}

func TestCreateFederation(t *testing.T) {
	// Create test service with mocks
	mockDB := &MockDatabase{}
	mockRedis := &MockRedisClient{}
	mockKafka := &MockKafkaWriter{}
	
	service := &FleetCoordinationService{
		config:      loadConfig(),
		database:    mockDB,
		redis:       mockRedis,
		kafkaWriter: mockKafka,
	}
	
	// Test federation data
	federation := map[string]interface{}{
		"name":        "Test Federation",
		"description": "Test federation for coordination",
		"members":     []string{"fleet-001", "fleet-002"},
	}
	
	// Mock database exec success
	mockDB.On("ExecContext", mock.Anything, mock.Anything, mock.Anything).Return(nil, nil)
	
	// Mock Redis set success
	mockRedis.On("Set", mock.Anything, mock.Anything, mock.Anything, mock.Anything).Return(nil)
	
	// Mock Kafka write success
	mockKafka.On("WriteMessages", mock.Anything, mock.Anything).Return(nil)
	
	// Create test request
	body, _ := json.Marshal(federation)
	req := httptest.NewRequest("POST", "/api/v1/federations", bytes.NewReader(body))
	req.Header.Set("Content-Type", "application/json")
	w := httptest.NewRecorder()
	
	// Call create federation
	service.createFederation(w, req)
	
	// Assertions
	assert.Equal(t, http.StatusCreated, w.Code)
	
	var response map[string]interface{}
	err := json.Unmarshal(w.Body.Bytes(), &response)
	assert.NoError(t, err)
	assert.Equal(t, "federation_created", response["status"])
	
	mockDB.AssertExpectations(t)
	mockRedis.AssertExpectations(t)
	mockKafka.AssertExpectations(t)
}

func TestCreateFederationInvalidJSON(t *testing.T) {
	service := &FleetCoordinationService{
		config: loadConfig(),
	}
	
	// Create test request with invalid JSON
	req := httptest.NewRequest("POST", "/api/v1/federations", bytes.NewReader([]byte("invalid json")))
	req.Header.Set("Content-Type", "application/json")
	w := httptest.NewRecorder()
	
	// Call create federation
	service.createFederation(w, req)
	
	// Assertions
	assert.Equal(t, http.StatusBadRequest, w.Code)
}

func TestGetFederation(t *testing.T) {
	// Create test service with mocks
	mockDB := &MockDatabase{}
	mockRedis := &MockRedisClient{}
	mockKafka := &MockKafkaWriter{}
	
	service := &FleetCoordinationService{
		config:      loadConfig(),
		database:    mockDB,
		redis:       mockRedis,
		kafkaWriter: mockKafka,
	}
	
	// Mock database query success
	mockDB.On("QueryContext", mock.Anything, mock.Anything, mock.Anything).Return([]interface{}{}, nil)
	
	// Create test request
	req := httptest.NewRequest("GET", "/api/v1/federations/test-federation-001", nil)
	w := httptest.NewRecorder()
	
	// Call get federation
	service.getFederation(w, req)
	
	// Assertions
	assert.Equal(t, http.StatusOK, w.Code)
	
	var response map[string]interface{}
	err := json.Unmarshal(w.Body.Bytes(), &response)
	assert.NoError(t, err)
	assert.NotNil(t, response["federation"])
	
	mockDB.AssertExpectations(t)
}

func TestUpdateFederation(t *testing.T) {
	// Create test service with mocks
	mockDB := &MockDatabase{}
	mockRedis := &MockRedisClient{}
	mockKafka := &MockKafkaWriter{}
	
	service := &FleetCoordinationService{
		config:      loadConfig(),
		database:    mockDB,
		redis:       mockRedis,
		kafkaWriter: mockKafka,
	}
	
	// Test federation update data
	updateData := map[string]interface{}{
		"name":        "Updated Federation",
		"description": "Updated federation description",
	}
	
	// Mock database exec success
	mockDB.On("ExecContext", mock.Anything, mock.Anything, mock.Anything).Return(nil, nil)
	
	// Mock Redis set success
	mockRedis.On("Set", mock.Anything, mock.Anything, mock.Anything, mock.Anything).Return(nil)
	
	// Mock Kafka write success
	mockKafka.On("WriteMessages", mock.Anything, mock.Anything).Return(nil)
	
	// Create test request
	body, _ := json.Marshal(updateData)
	req := httptest.NewRequest("PUT", "/api/v1/federations/test-federation-001", bytes.NewReader(body))
	req.Header.Set("Content-Type", "application/json")
	w := httptest.NewRecorder()
	
	// Call update federation
	service.updateFederation(w, req)
	
	// Assertions
	assert.Equal(t, http.StatusOK, w.Code)
	
	var response map[string]interface{}
	err := json.Unmarshal(w.Body.Bytes(), &response)
	assert.NoError(t, err)
	assert.Equal(t, "federation_updated", response["status"])
	
	mockDB.AssertExpectations(t)
	mockRedis.AssertExpectations(t)
	mockKafka.AssertExpectations(t)
}

func TestDeleteFederation(t *testing.T) {
	// Create test service with mocks
	mockDB := &MockDatabase{}
	mockRedis := &MockRedisClient{}
	mockKafka := &MockKafkaWriter{}
	
	service := &FleetCoordinationService{
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
	req := httptest.NewRequest("DELETE", "/api/v1/federations/test-federation-001", nil)
	w := httptest.NewRecorder()
	
	// Call delete federation
	service.deleteFederation(w, req)
	
	// Assertions
	assert.Equal(t, http.StatusOK, w.Code)
	
	var response map[string]interface{}
	err := json.Unmarshal(w.Body.Bytes(), &response)
	assert.NoError(t, err)
	assert.Equal(t, "federation_deleted", response["status"])
	
	mockDB.AssertExpectations(t)
	mockRedis.AssertExpectations(t)
	mockKafka.AssertExpectations(t)
}

func TestListResourceSharing(t *testing.T) {
	// Create test service with mocks
	mockDB := &MockDatabase{}
	mockRedis := &MockRedisClient{}
	mockKafka := &MockKafkaWriter{}
	
	service := &FleetCoordinationService{
		config:      loadConfig(),
		database:    mockDB,
		redis:       mockRedis,
		kafkaWriter: mockKafka,
	}
	
	// Mock database query success
	mockDB.On("QueryContext", mock.Anything, mock.Anything, mock.Anything).Return([]interface{}{}, nil)
	
	// Create test request
	req := httptest.NewRequest("GET", "/api/v1/resource-sharing", nil)
	w := httptest.NewRecorder()
	
	// Call list resource sharing
	service.listResourceSharing(w, req)
	
	// Assertions
	assert.Equal(t, http.StatusOK, w.Code)
	
	var response map[string]interface{}
	err := json.Unmarshal(w.Body.Bytes(), &response)
	assert.NoError(t, err)
	assert.NotNil(t, response["resource_sharing"])
	
	mockDB.AssertExpectations(t)
}

func TestCreateResourceSharing(t *testing.T) {
	// Create test service with mocks
	mockDB := &MockDatabase{}
	mockRedis := &MockRedisClient{}
	mockKafka := &MockKafkaWriter{}
	
	service := &FleetCoordinationService{
		config:      loadConfig(),
		database:    mockDB,
		redis:       mockRedis,
		kafkaWriter: mockKafka,
	}
	
	// Test resource sharing data
	resourceSharing := map[string]interface{}{
		"federation_id": "test-federation-001",
		"resource_type": "vehicle",
		"resource_id":   "vehicle-001",
		"shared_by":     "fleet-001",
		"shared_with":   "fleet-002",
		"permissions":   []string{"read", "execute"},
	}
	
	// Mock database exec success
	mockDB.On("ExecContext", mock.Anything, mock.Anything, mock.Anything).Return(nil, nil)
	
	// Mock Redis set success
	mockRedis.On("Set", mock.Anything, mock.Anything, mock.Anything, mock.Anything).Return(nil)
	
	// Mock Kafka write success
	mockKafka.On("WriteMessages", mock.Anything, mock.Anything).Return(nil)
	
	// Create test request
	body, _ := json.Marshal(resourceSharing)
	req := httptest.NewRequest("POST", "/api/v1/resource-sharing", bytes.NewReader(body))
	req.Header.Set("Content-Type", "application/json")
	w := httptest.NewRecorder()
	
	// Call create resource sharing
	service.createResourceSharing(w, req)
	
	// Assertions
	assert.Equal(t, http.StatusCreated, w.Code)
	
	var response map[string]interface{}
	err := json.Unmarshal(w.Body.Bytes(), &response)
	assert.NoError(t, err)
	assert.Equal(t, "resource_sharing_created", response["status"])
	
	mockDB.AssertExpectations(t)
	mockRedis.AssertExpectations(t)
	mockKafka.AssertExpectations(t)
}

func TestGetCoordinationMetrics(t *testing.T) {
	// Create test service with mocks
	mockDB := &MockDatabase{}
	mockRedis := &MockRedisClient{}
	mockKafka := &MockKafkaWriter{}
	
	service := &FleetCoordinationService{
		config:      loadConfig(),
		database:    mockDB,
		redis:       mockRedis,
		kafkaWriter: mockKafka,
	}
	
	// Mock database query success
	mockDB.On("QueryContext", mock.Anything, mock.Anything, mock.Anything).Return([]interface{}{}, nil)
	
	// Create test request
	req := httptest.NewRequest("GET", "/api/v1/coordination/metrics", nil)
	w := httptest.NewRecorder()
	
	// Call get coordination metrics
	service.getCoordinationMetrics(w, req)
	
	// Assertions
	assert.Equal(t, http.StatusOK, w.Code)
	
	var response map[string]interface{}
	err := json.Unmarshal(w.Body.Bytes(), &response)
	assert.NoError(t, err)
	assert.NotNil(t, response["metrics"])
	
	mockDB.AssertExpectations(t)
}

func TestValidateFederationInput(t *testing.T) {
	service := &FleetCoordinationService{
		config: loadConfig(),
	}
	
	// Valid federation
	validFederation := map[string]interface{}{
		"name":        "Test Federation",
		"description": "Test federation for coordination",
		"members":     []string{"fleet-001", "fleet-002"},
	}
	
	err := service.validateFederationInput(validFederation)
	assert.NoError(t, err)
	
	// Invalid federation - missing name
	invalidFederation := map[string]interface{}{
		"description": "Test federation for coordination",
		"members":     []string{"fleet-001", "fleet-002"},
	}
	
	err = service.validateFederationInput(invalidFederation)
	assert.Error(t, err)
	assert.Contains(t, err.Error(), "name is required")
	
	// Invalid federation - empty members
	invalidMembersFederation := map[string]interface{}{
		"name":        "Test Federation",
		"description": "Test federation for coordination",
		"members":     []string{},
	}
	
	err = service.validateFederationInput(invalidMembersFederation)
	assert.Error(t, err)
	assert.Contains(t, err.Error(), "at least one member is required")
}

func TestValidateResourceSharingInput(t *testing.T) {
	service := &FleetCoordinationService{
		config: loadConfig(),
	}
	
	// Valid resource sharing
	validResourceSharing := map[string]interface{}{
		"federation_id": "test-federation-001",
		"resource_type": "vehicle",
		"resource_id":   "vehicle-001",
		"shared_by":     "fleet-001",
		"shared_with":   "fleet-002",
		"permissions":   []string{"read", "execute"},
	}
	
	err := service.validateResourceSharingInput(validResourceSharing)
	assert.NoError(t, err)
	
	// Invalid resource sharing - missing federation_id
	invalidResourceSharing := map[string]interface{}{
		"resource_type": "vehicle",
		"resource_id":   "vehicle-001",
		"shared_by":     "fleet-001",
		"shared_with":   "fleet-002",
		"permissions":   []string{"read", "execute"},
	}
	
	err = service.validateResourceSharingInput(invalidResourceSharing)
	assert.Error(t, err)
	assert.Contains(t, err.Error(), "federation_id is required")
	
	// Invalid resource sharing - empty permissions
	invalidPermissionsResourceSharing := map[string]interface{}{
		"federation_id": "test-federation-001",
		"resource_type": "vehicle",
		"resource_id":   "vehicle-001",
		"shared_by":     "fleet-001",
		"shared_with":   "fleet-002",
		"permissions":   []string{},
	}
	
	err = service.validateResourceSharingInput(invalidPermissionsResourceSharing)
	assert.Error(t, err)
	assert.Contains(t, err.Error(), "at least one permission is required")
}

func TestGenerateUUID(t *testing.T) {
	service := &FleetCoordinationService{
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
	service := &FleetCoordinationService{
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
	service := &FleetCoordinationService{
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
	service := &FleetCoordinationService{
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
func BenchmarkCreateFederation(b *testing.B) {
	service := &FleetCoordinationService{
		config: loadConfig(),
	}
	
	federation := map[string]interface{}{
		"name":        "Test Federation",
		"description": "Test federation for coordination",
		"members":     []string{"fleet-001", "fleet-002"},
	}
	
	body, _ := json.Marshal(federation)
	
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		req := httptest.NewRequest("POST", "/api/v1/federations", bytes.NewReader(body))
		req.Header.Set("Content-Type", "application/json")
		w := httptest.NewRecorder()
		
		service.createFederation(w, req)
	}
}

func BenchmarkListFederations(b *testing.B) {
	service := &FleetCoordinationService{
		config: loadConfig(),
	}
	
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		req := httptest.NewRequest("GET", "/api/v1/federations", nil)
		w := httptest.NewRecorder()
		
		service.listFederations(w, req)
	}
}
