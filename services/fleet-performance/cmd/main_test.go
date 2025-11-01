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
	
	service := &PerformanceManagementService{
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

func TestListPerformanceMetrics(t *testing.T) {
	// Create test service with mocks
	mockDB := &MockDatabase{}
	mockRedis := &MockRedisClient{}
	mockKafka := &MockKafkaWriter{}
	
	service := &PerformanceManagementService{
		config:      loadConfig(),
		database:    mockDB,
		redis:       mockRedis,
		kafkaWriter: mockKafka,
	}
	
	// Mock database query success
	mockDB.On("QueryContext", mock.Anything, mock.Anything, mock.Anything).Return([]interface{}{}, nil)
	
	// Create test request
	req := httptest.NewRequest("GET", "/api/v1/performance-metrics", nil)
	w := httptest.NewRecorder()
	
	// Call list performance metrics
	service.listPerformanceMetrics(w, req)
	
	// Assertions
	assert.Equal(t, http.StatusOK, w.Code)
	
	var response map[string]interface{}
	err := json.Unmarshal(w.Body.Bytes(), &response)
	assert.NoError(t, err)
	assert.NotNil(t, response["performance_metrics"])
	
	mockDB.AssertExpectations(t)
}

func TestGetPerformanceMetric(t *testing.T) {
	// Create test service with mocks
	mockDB := &MockDatabase{}
	mockRedis := &MockRedisClient{}
	mockKafka := &MockKafkaWriter{}
	
	service := &PerformanceManagementService{
		config:      loadConfig(),
		database:    mockDB,
		redis:       mockRedis,
		kafkaWriter: mockKafka,
	}
	
	// Mock database query success
	mockDB.On("QueryContext", mock.Anything, mock.Anything, mock.Anything).Return([]interface{}{}, nil)
	
	// Create test request
	req := httptest.NewRequest("GET", "/api/v1/performance-metrics/test-metric-001", nil)
	w := httptest.NewRecorder()
	
	// Call get performance metric
	service.getPerformanceMetric(w, req)
	
	// Assertions
	assert.Equal(t, http.StatusOK, w.Code)
	
	var response map[string]interface{}
	err := json.Unmarshal(w.Body.Bytes(), &response)
	assert.NoError(t, err)
	assert.NotNil(t, response["performance_metric"])
	
	mockDB.AssertExpectations(t)
}

func TestCreatePerformanceMetric(t *testing.T) {
	// Create test service with mocks
	mockDB := &MockDatabase{}
	mockRedis := &MockRedisClient{}
	mockKafka := &MockKafkaWriter{}
	
	service := &PerformanceManagementService{
		config:      loadConfig(),
		database:    mockDB,
		redis:       mockRedis,
		kafkaWriter: mockKafka,
	}
	
	// Test performance metric data
	metric := map[string]interface{}{
		"name":        "Fleet Utilization",
		"description": "Percentage of fleet vehicles in active use",
		"type":        "utilization",
		"fleet_id":    "fleet-001",
		"value":       85.5,
		"unit":        "percentage",
		"threshold": map[string]interface{}{
			"warning":  70.0,
			"critical": 90.0,
		},
		"tags": []string{"fleet", "utilization", "performance"},
	}
	
	// Mock database exec success
	mockDB.On("ExecContext", mock.Anything, mock.Anything, mock.Anything).Return(nil, nil)
	
	// Mock Redis set success
	mockRedis.On("Set", mock.Anything, mock.Anything, mock.Anything, mock.Anything).Return(nil)
	
	// Mock Kafka write success
	mockKafka.On("WriteMessages", mock.Anything, mock.Anything).Return(nil)
	
	// Create test request
	body, _ := json.Marshal(metric)
	req := httptest.NewRequest("POST", "/api/v1/performance-metrics", bytes.NewReader(body))
	req.Header.Set("Content-Type", "application/json")
	w := httptest.NewRecorder()
	
	// Call create performance metric
	service.createPerformanceMetric(w, req)
	
	// Assertions
	assert.Equal(t, http.StatusCreated, w.Code)
	
	var response map[string]interface{}
	err := json.Unmarshal(w.Body.Bytes(), &response)
	assert.NoError(t, err)
	assert.Equal(t, "performance_metric_created", response["status"])
	
	mockDB.AssertExpectations(t)
	mockRedis.AssertExpectations(t)
	mockKafka.AssertExpectations(t)
}

func TestCreatePerformanceMetricInvalidJSON(t *testing.T) {
	service := &PerformanceManagementService{
		config: loadConfig(),
	}
	
	// Create test request with invalid JSON
	req := httptest.NewRequest("POST", "/api/v1/performance-metrics", bytes.NewReader([]byte("invalid json")))
	req.Header.Set("Content-Type", "application/json")
	w := httptest.NewRecorder()
	
	// Call create performance metric
	service.createPerformanceMetric(w, req)
	
	// Assertions
	assert.Equal(t, http.StatusBadRequest, w.Code)
}

func TestUpdatePerformanceMetric(t *testing.T) {
	// Create test service with mocks
	mockDB := &MockDatabase{}
	mockRedis := &MockRedisClient{}
	mockKafka := &MockKafkaWriter{}
	
	service := &PerformanceManagementService{
		config:      loadConfig(),
		database:    mockDB,
		redis:       mockRedis,
		kafkaWriter: mockKafka,
	}
	
	// Test performance metric update data
	updateData := map[string]interface{}{
		"name":        "Updated Fleet Utilization",
		"description": "Updated percentage of fleet vehicles in active use",
		"value":       90.0,
		"threshold": map[string]interface{}{
			"warning":  75.0,
			"critical": 95.0,
		},
	}
	
	// Mock database exec success
	mockDB.On("ExecContext", mock.Anything, mock.Anything, mock.Anything).Return(nil, nil)
	
	// Mock Redis set success
	mockRedis.On("Set", mock.Anything, mock.Anything, mock.Anything, mock.Anything).Return(nil)
	
	// Mock Kafka write success
	mockKafka.On("WriteMessages", mock.Anything, mock.Anything).Return(nil)
	
	// Create test request
	body, _ := json.Marshal(updateData)
	req := httptest.NewRequest("PUT", "/api/v1/performance-metrics/test-metric-001", bytes.NewReader(body))
	req.Header.Set("Content-Type", "application/json")
	w := httptest.NewRecorder()
	
	// Call update performance metric
	service.updatePerformanceMetric(w, req)
	
	// Assertions
	assert.Equal(t, http.StatusOK, w.Code)
	
	var response map[string]interface{}
	err := json.Unmarshal(w.Body.Bytes(), &response)
	assert.NoError(t, err)
	assert.Equal(t, "performance_metric_updated", response["status"])
	
	mockDB.AssertExpectations(t)
	mockRedis.AssertExpectations(t)
	mockKafka.AssertExpectations(t)
}

func TestDeletePerformanceMetric(t *testing.T) {
	// Create test service with mocks
	mockDB := &MockDatabase{}
	mockRedis := &MockRedisClient{}
	mockKafka := &MockKafkaWriter{}
	
	service := &PerformanceManagementService{
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
	req := httptest.NewRequest("DELETE", "/api/v1/performance-metrics/test-metric-001", nil)
	w := httptest.NewRecorder()
	
	// Call delete performance metric
	service.deletePerformanceMetric(w, req)
	
	// Assertions
	assert.Equal(t, http.StatusOK, w.Code)
	
	var response map[string]interface{}
	err := json.Unmarshal(w.Body.Bytes(), &response)
	assert.NoError(t, err)
	assert.Equal(t, "performance_metric_deleted", response["status"])
	
	mockDB.AssertExpectations(t)
	mockRedis.AssertExpectations(t)
	mockKafka.AssertExpectations(t)
}

func TestListPerformanceBenchmarks(t *testing.T) {
	// Create test service with mocks
	mockDB := &MockDatabase{}
	mockRedis := &MockRedisClient{}
	mockKafka := &MockKafkaWriter{}
	
	service := &PerformanceManagementService{
		config:      loadConfig(),
		database:    mockDB,
		redis:       mockRedis,
		kafkaWriter: mockKafka,
	}
	
	// Mock database query success
	mockDB.On("QueryContext", mock.Anything, mock.Anything, mock.Anything).Return([]interface{}{}, nil)
	
	// Create test request
	req := httptest.NewRequest("GET", "/api/v1/performance-benchmarks", nil)
	w := httptest.NewRecorder()
	
	// Call list performance benchmarks
	service.listPerformanceBenchmarks(w, req)
	
	// Assertions
	assert.Equal(t, http.StatusOK, w.Code)
	
	var response map[string]interface{}
	err := json.Unmarshal(w.Body.Bytes(), &response)
	assert.NoError(t, err)
	assert.NotNil(t, response["performance_benchmarks"])
	
	mockDB.AssertExpectations(t)
}

func TestCreatePerformanceBenchmark(t *testing.T) {
	// Create test service with mocks
	mockDB := &MockDatabase{}
	mockRedis := &MockRedisClient{}
	mockKafka := &MockKafkaWriter{}
	
	service := &PerformanceManagementService{
		config:      loadConfig(),
		database:    mockDB,
		redis:       mockRedis,
		kafkaWriter: mockKafka,
	}
	
	// Test performance benchmark data
	benchmark := map[string]interface{}{
		"name":        "Industry Standard Fleet Utilization",
		"description": "Industry standard for fleet utilization performance",
		"metric_type": "utilization",
		"industry":     "logistics",
		"value":       80.0,
		"unit":        "percentage",
		"source":      "industry_report_2024",
		"tags":        []string{"industry", "standard", "utilization"},
	}
	
	// Mock database exec success
	mockDB.On("ExecContext", mock.Anything, mock.Anything, mock.Anything).Return(nil, nil)
	
	// Mock Redis set success
	mockRedis.On("Set", mock.Anything, mock.Anything, mock.Anything, mock.Anything).Return(nil)
	
	// Mock Kafka write success
	mockKafka.On("WriteMessages", mock.Anything, mock.Anything).Return(nil)
	
	// Create test request
	body, _ := json.Marshal(benchmark)
	req := httptest.NewRequest("POST", "/api/v1/performance-benchmarks", bytes.NewReader(body))
	req.Header.Set("Content-Type", "application/json")
	w := httptest.NewRecorder()
	
	// Call create performance benchmark
	service.createPerformanceBenchmark(w, req)
	
	// Assertions
	assert.Equal(t, http.StatusCreated, w.Code)
	
	var response map[string]interface{}
	err := json.Unmarshal(w.Body.Bytes(), &response)
	assert.NoError(t, err)
	assert.Equal(t, "performance_benchmark_created", response["status"])
	
	mockDB.AssertExpectations(t)
	mockRedis.AssertExpectations(t)
	mockKafka.AssertExpectations(t)
}

func TestGetPerformanceBenchmark(t *testing.T) {
	// Create test service with mocks
	mockDB := &MockDatabase{}
	mockRedis := &MockRedisClient{}
	mockKafka := &MockKafkaWriter{}
	
	service := &PerformanceManagementService{
		config:      loadConfig(),
		database:    mockDB,
		redis:       mockRedis,
		kafkaWriter: mockKafka,
	}
	
	// Mock database query success
	mockDB.On("QueryContext", mock.Anything, mock.Anything, mock.Anything).Return([]interface{}{}, nil)
	
	// Create test request
	req := httptest.NewRequest("GET", "/api/v1/performance-benchmarks/test-benchmark-001", nil)
	w := httptest.NewRecorder()
	
	// Call get performance benchmark
	service.getPerformanceBenchmark(w, req)
	
	// Assertions
	assert.Equal(t, http.StatusOK, w.Code)
	
	var response map[string]interface{}
	err := json.Unmarshal(w.Body.Bytes(), &response)
	assert.NoError(t, err)
	assert.NotNil(t, response["performance_benchmark"])
	
	mockDB.AssertExpectations(t)
}

func TestUpdatePerformanceBenchmark(t *testing.T) {
	// Create test service with mocks
	mockDB := &MockDatabase{}
	mockRedis := &MockRedisClient{}
	mockKafka := &MockKafkaWriter{}
	
	service := &PerformanceManagementService{
		config:      loadConfig(),
		database:    mockDB,
		redis:       mockRedis,
		kafkaWriter: mockKafka,
	}
	
	// Test performance benchmark update data
	updateData := map[string]interface{}{
		"name":        "Updated Industry Standard Fleet Utilization",
		"description": "Updated industry standard for fleet utilization performance",
		"value":       85.0,
		"source":      "industry_report_2024_updated",
	}
	
	// Mock database exec success
	mockDB.On("ExecContext", mock.Anything, mock.Anything, mock.Anything).Return(nil, nil)
	
	// Mock Redis set success
	mockRedis.On("Set", mock.Anything, mock.Anything, mock.Anything, mock.Anything).Return(nil)
	
	// Mock Kafka write success
	mockKafka.On("WriteMessages", mock.Anything, mock.Anything).Return(nil)
	
	// Create test request
	body, _ := json.Marshal(updateData)
	req := httptest.NewRequest("PUT", "/api/v1/performance-benchmarks/test-benchmark-001", bytes.NewReader(body))
	req.Header.Set("Content-Type", "application/json")
	w := httptest.NewRecorder()
	
	// Call update performance benchmark
	service.updatePerformanceBenchmark(w, req)
	
	// Assertions
	assert.Equal(t, http.StatusOK, w.Code)
	
	var response map[string]interface{}
	err := json.Unmarshal(w.Body.Bytes(), &response)
	assert.NoError(t, err)
	assert.Equal(t, "performance_benchmark_updated", response["status"])
	
	mockDB.AssertExpectations(t)
	mockRedis.AssertExpectations(t)
	mockKafka.AssertExpectations(t)
}

func TestDeletePerformanceBenchmark(t *testing.T) {
	// Create test service with mocks
	mockDB := &MockDatabase{}
	mockRedis := &MockRedisClient{}
	mockKafka := &MockKafkaWriter{}
	
	service := &PerformanceManagementService{
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
	req := httptest.NewRequest("DELETE", "/api/v1/performance-benchmarks/test-benchmark-001", nil)
	w := httptest.NewRecorder()
	
	// Call delete performance benchmark
	service.deletePerformanceBenchmark(w, req)
	
	// Assertions
	assert.Equal(t, http.StatusOK, w.Code)
	
	var response map[string]interface{}
	err := json.Unmarshal(w.Body.Bytes(), &response)
	assert.NoError(t, err)
	assert.Equal(t, "performance_benchmark_deleted", response["status"])
	
	mockDB.AssertExpectations(t)
	mockRedis.AssertExpectations(t)
	mockKafka.AssertExpectations(t)
}

func TestListPerformanceAlerts(t *testing.T) {
	// Create test service with mocks
	mockDB := &MockDatabase{}
	mockRedis := &MockRedisClient{}
	mockKafka := &MockKafkaWriter{}
	
	service := &PerformanceManagementService{
		config:      loadConfig(),
		database:    mockDB,
		redis:       mockRedis,
		kafkaWriter: mockKafka,
	}
	
	// Mock database query success
	mockDB.On("QueryContext", mock.Anything, mock.Anything, mock.Anything).Return([]interface{}{}, nil)
	
	// Create test request
	req := httptest.NewRequest("GET", "/api/v1/performance-alerts", nil)
	w := httptest.NewRecorder()
	
	// Call list performance alerts
	service.listPerformanceAlerts(w, req)
	
	// Assertions
	assert.Equal(t, http.StatusOK, w.Code)
	
	var response map[string]interface{}
	err := json.Unmarshal(w.Body.Bytes(), &response)
	assert.NoError(t, err)
	assert.NotNil(t, response["performance_alerts"])
	
	mockDB.AssertExpectations(t)
}

func TestCreatePerformanceAlert(t *testing.T) {
	// Create test service with mocks
	mockDB := &MockDatabase{}
	mockRedis := &MockRedisClient{}
	mockKafka := &MockKafkaWriter{}
	
	service := &PerformanceManagementService{
		config:      loadConfig(),
		database:    mockDB,
		redis:       mockRedis,
		kafkaWriter: mockKafka,
	}
	
	// Test performance alert data
	alert := map[string]interface{}{
		"name":        "High Fleet Utilization Alert",
		"description": "Alert when fleet utilization exceeds 90%",
		"metric_id":   "test-metric-001",
		"condition":   "greater_than",
		"threshold":   90.0,
		"severity":    "warning",
		"enabled":     true,
		"recipients":  []string{"fleet-manager@example.com", "operations@example.com"},
	}
	
	// Mock database exec success
	mockDB.On("ExecContext", mock.Anything, mock.Anything, mock.Anything).Return(nil, nil)
	
	// Mock Redis set success
	mockRedis.On("Set", mock.Anything, mock.Anything, mock.Anything, mock.Anything).Return(nil)
	
	// Mock Kafka write success
	mockKafka.On("WriteMessages", mock.Anything, mock.Anything).Return(nil)
	
	// Create test request
	body, _ := json.Marshal(alert)
	req := httptest.NewRequest("POST", "/api/v1/performance-alerts", bytes.NewReader(body))
	req.Header.Set("Content-Type", "application/json")
	w := httptest.NewRecorder()
	
	// Call create performance alert
	service.createPerformanceAlert(w, req)
	
	// Assertions
	assert.Equal(t, http.StatusCreated, w.Code)
	
	var response map[string]interface{}
	err := json.Unmarshal(w.Body.Bytes(), &response)
	assert.NoError(t, err)
	assert.Equal(t, "performance_alert_created", response["status"])
	
	mockDB.AssertExpectations(t)
	mockRedis.AssertExpectations(t)
	mockKafka.AssertExpectations(t)
}

func TestGetPerformanceAlert(t *testing.T) {
	// Create test service with mocks
	mockDB := &MockDatabase{}
	mockRedis := &MockRedisClient{}
	mockKafka := &MockKafkaWriter{}
	
	service := &PerformanceManagementService{
		config:      loadConfig(),
		database:    mockDB,
		redis:       mockRedis,
		kafkaWriter: mockKafka,
	}
	
	// Mock database query success
	mockDB.On("QueryContext", mock.Anything, mock.Anything, mock.Anything).Return([]interface{}{}, nil)
	
	// Create test request
	req := httptest.NewRequest("GET", "/api/v1/performance-alerts/test-alert-001", nil)
	w := httptest.NewRecorder()
	
	// Call get performance alert
	service.getPerformanceAlert(w, req)
	
	// Assertions
	assert.Equal(t, http.StatusOK, w.Code)
	
	var response map[string]interface{}
	err := json.Unmarshal(w.Body.Bytes(), &response)
	assert.NoError(t, err)
	assert.NotNil(t, response["performance_alert"])
	
	mockDB.AssertExpectations(t)
}

func TestUpdatePerformanceAlert(t *testing.T) {
	// Create test service with mocks
	mockDB := &MockDatabase{}
	mockRedis := &MockRedisClient{}
	mockKafka := &MockKafkaWriter{}
	
	service := &PerformanceManagementService{
		config:      loadConfig(),
		database:    mockDB,
		redis:       mockRedis,
		kafkaWriter: mockKafka,
	}
	
	// Test performance alert update data
	updateData := map[string]interface{}{
		"name":        "Updated High Fleet Utilization Alert",
		"description": "Updated alert when fleet utilization exceeds 95%",
		"threshold":   95.0,
		"severity":    "critical",
		"enabled":     true,
	}
	
	// Mock database exec success
	mockDB.On("ExecContext", mock.Anything, mock.Anything, mock.Anything).Return(nil, nil)
	
	// Mock Redis set success
	mockRedis.On("Set", mock.Anything, mock.Anything, mock.Anything, mock.Anything).Return(nil)
	
	// Mock Kafka write success
	mockKafka.On("WriteMessages", mock.Anything, mock.Anything).Return(nil)
	
	// Create test request
	body, _ := json.Marshal(updateData)
	req := httptest.NewRequest("PUT", "/api/v1/performance-alerts/test-alert-001", bytes.NewReader(body))
	req.Header.Set("Content-Type", "application/json")
	w := httptest.NewRecorder()
	
	// Call update performance alert
	service.updatePerformanceAlert(w, req)
	
	// Assertions
	assert.Equal(t, http.StatusOK, w.Code)
	
	var response map[string]interface{}
	err := json.Unmarshal(w.Body.Bytes(), &response)
	assert.NoError(t, err)
	assert.Equal(t, "performance_alert_updated", response["status"])
	
	mockDB.AssertExpectations(t)
	mockRedis.AssertExpectations(t)
	mockKafka.AssertExpectations(t)
}

func TestDeletePerformanceAlert(t *testing.T) {
	// Create test service with mocks
	mockDB := &MockDatabase{}
	mockRedis := &MockRedisClient{}
	mockKafka := &MockKafkaWriter{}
	
	service := &PerformanceManagementService{
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
	req := httptest.NewRequest("DELETE", "/api/v1/performance-alerts/test-alert-001", nil)
	w := httptest.NewRecorder()
	
	// Call delete performance alert
	service.deletePerformanceAlert(w, req)
	
	// Assertions
	assert.Equal(t, http.StatusOK, w.Code)
	
	var response map[string]interface{}
	err := json.Unmarshal(w.Body.Bytes(), &response)
	assert.NoError(t, err)
	assert.Equal(t, "performance_alert_deleted", response["status"])
	
	mockDB.AssertExpectations(t)
	mockRedis.AssertExpectations(t)
	mockKafka.AssertExpectations(t)
}

func TestGeneratePerformanceReport(t *testing.T) {
	// Create test service with mocks
	mockDB := &MockDatabase{}
	mockRedis := &MockRedisClient{}
	mockKafka := &MockKafkaWriter{}
	
	service := &PerformanceManagementService{
		config:      loadConfig(),
		database:    mockDB,
		redis:       mockRedis,
		kafkaWriter: mockKafka,
	}
	
	// Test performance report request data
	reportRequest := map[string]interface{}{
		"report_type": "fleet_performance",
		"fleet_id":    "fleet-001",
		"start_date":  "2024-01-01",
		"end_date":    "2024-01-31",
		"format":      "pdf",
		"sections":    []string{"metrics", "benchmarks", "alerts", "trends"},
	}
	
	// Mock database query success
	mockDB.On("QueryContext", mock.Anything, mock.Anything, mock.Anything).Return([]interface{}{}, nil)
	
	// Create test request
	body, _ := json.Marshal(reportRequest)
	req := httptest.NewRequest("POST", "/api/v1/performance-reports", bytes.NewReader(body))
	req.Header.Set("Content-Type", "application/json")
	w := httptest.NewRecorder()
	
	// Call generate performance report
	service.generatePerformanceReport(w, req)
	
	// Assertions
	assert.Equal(t, http.StatusOK, w.Code)
	
	var response map[string]interface{}
	err := json.Unmarshal(w.Body.Bytes(), &response)
	assert.NoError(t, err)
	assert.Equal(t, "performance_report_generated", response["status"])
	assert.NotNil(t, response["report_id"])
	
	mockDB.AssertExpectations(t)
}

func TestGeneratePerformanceReportInvalidJSON(t *testing.T) {
	service := &PerformanceManagementService{
		config: loadConfig(),
	}
	
	// Create test request with invalid JSON
	req := httptest.NewRequest("POST", "/api/v1/performance-reports", bytes.NewReader([]byte("invalid json")))
	req.Header.Set("Content-Type", "application/json")
	w := httptest.NewRecorder()
	
	// Call generate performance report
	service.generatePerformanceReport(w, req)
	
	// Assertions
	assert.Equal(t, http.StatusBadRequest, w.Code)
}

func TestValidatePerformanceMetricInput(t *testing.T) {
	service := &PerformanceManagementService{
		config: loadConfig(),
	}
	
	// Valid performance metric
	validMetric := map[string]interface{}{
		"name":        "Fleet Utilization",
		"description": "Percentage of fleet vehicles in active use",
		"type":        "utilization",
		"fleet_id":    "fleet-001",
		"value":       85.5,
		"unit":        "percentage",
		"threshold": map[string]interface{}{
			"warning":  70.0,
			"critical": 90.0,
		},
		"tags": []string{"fleet", "utilization", "performance"},
	}
	
	err := service.validatePerformanceMetricInput(validMetric)
	assert.NoError(t, err)
	
	// Invalid performance metric - missing name
	invalidMetric := map[string]interface{}{
		"description": "Percentage of fleet vehicles in active use",
		"type":        "utilization",
		"fleet_id":    "fleet-001",
		"value":       85.5,
		"unit":        "percentage",
		"threshold": map[string]interface{}{
			"warning":  70.0,
			"critical": 90.0,
		},
		"tags": []string{"fleet", "utilization", "performance"},
	}
	
	err = service.validatePerformanceMetricInput(invalidMetric)
	assert.Error(t, err)
	assert.Contains(t, err.Error(), "name is required")
	
	// Invalid performance metric - invalid type
	invalidTypeMetric := map[string]interface{}{
		"name":        "Fleet Utilization",
		"description": "Percentage of fleet vehicles in active use",
		"type":        "invalid_type",
		"fleet_id":    "fleet-001",
		"value":       85.5,
		"unit":        "percentage",
		"threshold": map[string]interface{}{
			"warning":  70.0,
			"critical": 90.0,
		},
		"tags": []string{"fleet", "utilization", "performance"},
	}
	
	err = service.validatePerformanceMetricInput(invalidTypeMetric)
	assert.Error(t, err)
	assert.Contains(t, err.Error(), "invalid type")
	
	// Invalid performance metric - negative value
	invalidValueMetric := map[string]interface{}{
		"name":        "Fleet Utilization",
		"description": "Percentage of fleet vehicles in active use",
		"type":        "utilization",
		"fleet_id":    "fleet-001",
		"value":       -10.0,
		"unit":        "percentage",
		"threshold": map[string]interface{}{
			"warning":  70.0,
			"critical": 90.0,
		},
		"tags": []string{"fleet", "utilization", "performance"},
	}
	
	err = service.validatePerformanceMetricInput(invalidValueMetric)
	assert.Error(t, err)
	assert.Contains(t, err.Error(), "value must be non-negative")
}

func TestValidatePerformanceBenchmarkInput(t *testing.T) {
	service := &PerformanceManagementService{
		config: loadConfig(),
	}
	
	// Valid performance benchmark
	validBenchmark := map[string]interface{}{
		"name":        "Industry Standard Fleet Utilization",
		"description": "Industry standard for fleet utilization performance",
		"metric_type": "utilization",
		"industry":     "logistics",
		"value":       80.0,
		"unit":        "percentage",
		"source":      "industry_report_2024",
		"tags":        []string{"industry", "standard", "utilization"},
	}
	
	err := service.validatePerformanceBenchmarkInput(validBenchmark)
	assert.NoError(t, err)
	
	// Invalid performance benchmark - missing name
	invalidBenchmark := map[string]interface{}{
		"description": "Industry standard for fleet utilization performance",
		"metric_type": "utilization",
		"industry":     "logistics",
		"value":       80.0,
		"unit":        "percentage",
		"source":      "industry_report_2024",
		"tags":        []string{"industry", "standard", "utilization"},
	}
	
	err = service.validatePerformanceBenchmarkInput(invalidBenchmark)
	assert.Error(t, err)
	assert.Contains(t, err.Error(), "name is required")
	
	// Invalid performance benchmark - negative value
	invalidValueBenchmark := map[string]interface{}{
		"name":        "Industry Standard Fleet Utilization",
		"description": "Industry standard for fleet utilization performance",
		"metric_type": "utilization",
		"industry":     "logistics",
		"value":       -10.0,
		"unit":        "percentage",
		"source":      "industry_report_2024",
		"tags":        []string{"industry", "standard", "utilization"},
	}
	
	err = service.validatePerformanceBenchmarkInput(invalidValueBenchmark)
	assert.Error(t, err)
	assert.Contains(t, err.Error(), "value must be non-negative")
}

func TestValidatePerformanceAlertInput(t *testing.T) {
	service := &PerformanceManagementService{
		config: loadConfig(),
	}
	
	// Valid performance alert
	validAlert := map[string]interface{}{
		"name":        "High Fleet Utilization Alert",
		"description": "Alert when fleet utilization exceeds 90%",
		"metric_id":   "test-metric-001",
		"condition":   "greater_than",
		"threshold":   90.0,
		"severity":    "warning",
		"enabled":     true,
		"recipients":  []string{"fleet-manager@example.com", "operations@example.com"},
	}
	
	err := service.validatePerformanceAlertInput(validAlert)
	assert.NoError(t, err)
	
	// Invalid performance alert - missing name
	invalidAlert := map[string]interface{}{
		"description": "Alert when fleet utilization exceeds 90%",
		"metric_id":   "test-metric-001",
		"condition":   "greater_than",
		"threshold":   90.0,
		"severity":    "warning",
		"enabled":     true,
		"recipients":  []string{"fleet-manager@example.com", "operations@example.com"},
	}
	
	err = service.validatePerformanceAlertInput(invalidAlert)
	assert.Error(t, err)
	assert.Contains(t, err.Error(), "name is required")
	
	// Invalid performance alert - invalid condition
	invalidConditionAlert := map[string]interface{}{
		"name":        "High Fleet Utilization Alert",
		"description": "Alert when fleet utilization exceeds 90%",
		"metric_id":   "test-metric-001",
		"condition":   "invalid_condition",
		"threshold":   90.0,
		"severity":    "warning",
		"enabled":     true,
		"recipients":  []string{"fleet-manager@example.com", "operations@example.com"},
	}
	
	err = service.validatePerformanceAlertInput(invalidConditionAlert)
	assert.Error(t, err)
	assert.Contains(t, err.Error(), "invalid condition")
	
	// Invalid performance alert - invalid severity
	invalidSeverityAlert := map[string]interface{}{
		"name":        "High Fleet Utilization Alert",
		"description": "Alert when fleet utilization exceeds 90%",
		"metric_id":   "test-metric-001",
		"condition":   "greater_than",
		"threshold":   90.0,
		"severity":    "invalid_severity",
		"enabled":     true,
		"recipients":  []string{"fleet-manager@example.com", "operations@example.com"},
	}
	
	err = service.validatePerformanceAlertInput(invalidSeverityAlert)
	assert.Error(t, err)
	assert.Contains(t, err.Error(), "invalid severity")
	
	// Invalid performance alert - empty recipients
	invalidRecipientsAlert := map[string]interface{}{
		"name":        "High Fleet Utilization Alert",
		"description": "Alert when fleet utilization exceeds 90%",
		"metric_id":   "test-metric-001",
		"condition":   "greater_than",
		"threshold":   90.0,
		"severity":    "warning",
		"enabled":     true,
		"recipients":  []string{},
	}
	
	err = service.validatePerformanceAlertInput(invalidRecipientsAlert)
	assert.Error(t, err)
	assert.Contains(t, err.Error(), "at least one recipient is required")
}

func TestGenerateUUID(t *testing.T) {
	service := &PerformanceManagementService{
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
	service := &PerformanceManagementService{
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
	service := &PerformanceManagementService{
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
	service := &PerformanceManagementService{
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
func BenchmarkCreatePerformanceMetric(b *testing.B) {
	service := &PerformanceManagementService{
		config: loadConfig(),
	}
	
	metric := map[string]interface{}{
		"name":        "Fleet Utilization",
		"description": "Percentage of fleet vehicles in active use",
		"type":        "utilization",
		"fleet_id":    "fleet-001",
		"value":       85.5,
		"unit":        "percentage",
		"threshold": map[string]interface{}{
			"warning":  70.0,
			"critical": 90.0,
		},
		"tags": []string{"fleet", "utilization", "performance"},
	}
	
	body, _ := json.Marshal(metric)
	
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		req := httptest.NewRequest("POST", "/api/v1/performance-metrics", bytes.NewReader(body))
		req.Header.Set("Content-Type", "application/json")
		w := httptest.NewRecorder()
		
		service.createPerformanceMetric(w, req)
	}
}

func BenchmarkListPerformanceMetrics(b *testing.B) {
	service := &PerformanceManagementService{
		config: loadConfig(),
	}
	
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		req := httptest.NewRequest("GET", "/api/v1/performance-metrics", nil)
		w := httptest.NewRecorder()
		
		service.listPerformanceMetrics(w, req)
	}
}
