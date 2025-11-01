package main

import (
	"bytes"
	"encoding/json"
	"fmt"
	"log"
	"net/http"
	"sync"
	"time"
)

// AtlasMesh Fleet OS Load Testing Suite
//
// Comprehensive load testing for all services including:
// - Fleet Manager API load testing
// - Analytics Service performance testing
// - External Integrations stress testing
// - ML Pipeline concurrent processing
// - Vehicle Gateway WebSocket connections
// - Database performance under load
// - Message queue throughput testing

type LoadTestConfig struct {
	BaseURL                string
	ConcurrentUsers        int
	TestDuration          time.Duration
	RequestRate           int // requests per second
	EnableMetrics         bool
	EnableDetailedLogging bool
}

type LoadTestResult struct {
	TestName           string
	TotalRequests      int
	SuccessfulRequests int
	FailedRequests     int
	AverageResponseTime time.Duration
	MinResponseTime    time.Duration
	MaxResponseTime    time.Duration
	RequestsPerSecond  float64
	ErrorRate          float64
	StartTime          time.Time
	EndTime            time.Duration
}

type LoadTestSuite struct {
	config  LoadTestConfig
	results []LoadTestResult
	client  *http.Client
}

func main() {
	config := LoadTestConfig{
		BaseURL:                "http://localhost:8080",
		ConcurrentUsers:        100,
		TestDuration:           5 * time.Minute,
		RequestRate:           1000,
		EnableMetrics:         true,
		EnableDetailedLogging: true,
	}

	suite := &LoadTestSuite{
		config: config,
		client: &http.Client{
			Timeout: 30 * time.Second,
		},
	}

	log.Println("🚀 Starting AtlasMesh Fleet OS Load Testing Suite...")

	// Run all load tests
	suite.runFleetManagerLoadTest()
	suite.runAnalyticsLoadTest()
	suite.runExternalIntegrationsLoadTest()
	suite.runMLPipelineLoadTest()
	suite.runVehicleGatewayLoadTest()
	suite.runDatabaseLoadTest()
	suite.runMessageQueueLoadTest()
	suite.runEndToEndLoadTest()

	// Generate report
	suite.generateReport()
}

func (suite *LoadTestSuite) runFleetManagerLoadTest() {
	log.Println("📊 Running Fleet Manager Load Test...")

	startTime := time.Now()
	var wg sync.WaitGroup
	var mu sync.Mutex
	results := LoadTestResult{
		TestName:  "Fleet Manager API",
		StartTime: startTime,
	}

	// Test endpoints
	endpoints := []string{
		"/api/v1/fleet/vehicles",
		"/api/v1/fleet/vehicles/status",
		"/api/v1/fleet/utilization",
		"/api/v1/fleet/operational-metrics",
	}

	// Concurrent request simulation
	for i := 0; i < suite.config.ConcurrentUsers; i++ {
		wg.Add(1)
		go func(workerID int) {
			defer wg.Done()
			
			ticker := time.NewTicker(time.Duration(1000/suite.config.RequestRate) * time.Millisecond)
			defer ticker.Stop()
			
			timeout := time.After(suite.config.TestDuration)
			
			for {
				select {
				case <-ticker.C:
					endpoint := endpoints[workerID%len(endpoints)]
					start := time.Now()
					
					resp, err := suite.client.Get(suite.config.BaseURL + endpoint)
					responseTime := time.Since(start)
					
					mu.Lock()
					results.TotalRequests++
					if err != nil || resp.StatusCode >= 400 {
						results.FailedRequests++
					} else {
						results.SuccessfulRequests++
					}
					
					if responseTime < results.MinResponseTime || results.MinResponseTime == 0 {
						results.MinResponseTime = responseTime
					}
					if responseTime > results.MaxResponseTime {
						results.MaxResponseTime = responseTime
					}
					mu.Unlock()
					
					if resp != nil {
						resp.Body.Close()
					}
					
				case <-timeout:
					return
				}
			}
		}(i)
	}

	wg.Wait()
	results.EndTime = time.Since(startTime)
	results.AverageResponseTime = results.EndTime / time.Duration(results.TotalRequests)
	results.RequestsPerSecond = float64(results.TotalRequests) / results.EndTime.Seconds()
	results.ErrorRate = float64(results.FailedRequests) / float64(results.TotalRequests) * 100

	suite.results = append(suite.results, results)
	log.Printf("✅ Fleet Manager Load Test completed: %d requests, %.2f RPS, %.2f%% error rate",
		results.TotalRequests, results.RequestsPerSecond, results.ErrorRate)
}

func (suite *LoadTestSuite) runAnalyticsLoadTest() {
	log.Println("📈 Running Analytics Service Load Test...")

	startTime := time.Now()
	var wg sync.WaitGroup
	var mu sync.Mutex
	results := LoadTestResult{
		TestName:  "Analytics Service",
		StartTime: startTime,
	}

	// Analytics endpoints
	endpoints := []string{
		"/api/v1/analytics/kpis?timeframe=24h",
		"/api/v1/analytics/operational",
		"/api/v1/analytics/financial",
		"/api/v1/analytics/safety",
		"/api/v1/analytics/realtime",
	}

	for i := 0; i < suite.config.ConcurrentUsers; i++ {
		wg.Add(1)
		go func(workerID int) {
			defer wg.Done()
			
			ticker := time.NewTicker(time.Duration(1000/suite.config.RequestRate) * time.Millisecond)
			defer ticker.Stop()
			
			timeout := time.After(suite.config.TestDuration)
			
			for {
				select {
				case <-ticker.C:
					endpoint := endpoints[workerID%len(endpoints)]
					start := time.Now()
					
					resp, err := suite.client.Get(suite.config.BaseURL + endpoint)
					responseTime := time.Since(start)
					
					mu.Lock()
					results.TotalRequests++
					if err != nil || resp.StatusCode >= 400 {
						results.FailedRequests++
					} else {
						results.SuccessfulRequests++
					}
					
					if responseTime < results.MinResponseTime || results.MinResponseTime == 0 {
						results.MinResponseTime = responseTime
					}
					if responseTime > results.MaxResponseTime {
						results.MaxResponseTime = responseTime
					}
					mu.Unlock()
					
					if resp != nil {
						resp.Body.Close()
					}
					
				case <-timeout:
					return
				}
			}
		}(i)
	}

	wg.Wait()
	results.EndTime = time.Since(startTime)
	results.AverageResponseTime = results.EndTime / time.Duration(results.TotalRequests)
	results.RequestsPerSecond = float64(results.TotalRequests) / results.EndTime.Seconds()
	results.ErrorRate = float64(results.FailedRequests) / float64(results.TotalRequests) * 100

	suite.results = append(suite.results, results)
	log.Printf("✅ Analytics Load Test completed: %d requests, %.2f RPS, %.2f%% error rate",
		results.TotalRequests, results.RequestsPerSecond, results.ErrorRate)
}

func (suite *LoadTestSuite) runExternalIntegrationsLoadTest() {
	log.Println("🌐 Running External Integrations Load Test...")

	startTime := time.Now()
	var wg sync.WaitGroup
	var mu sync.Mutex
	results := LoadTestResult{
		TestName:  "External Integrations",
		StartTime: startTime,
	}

	// External integration endpoints
	endpoints := []string{
		"/api/v1/weather/current/Dubai",
		"/api/v1/weather/forecast/Dubai",
		"/api/v1/maps/geocode/Dubai%20Marina",
		"/api/v1/erp/orders",
		"/api/v1/wms/warehouses",
	}

	for i := 0; i < suite.config.ConcurrentUsers; i++ {
		wg.Add(1)
		go func(workerID int) {
			defer wg.Done()
			
			ticker := time.NewTicker(time.Duration(1000/suite.config.RequestRate) * time.Millisecond)
			defer ticker.Stop()
			
			timeout := time.After(suite.config.TestDuration)
			
			for {
				select {
				case <-ticker.C:
					endpoint := endpoints[workerID%len(endpoints)]
					start := time.Now()
					
					resp, err := suite.client.Get(suite.config.BaseURL + endpoint)
					responseTime := time.Since(start)
					
					mu.Lock()
					results.TotalRequests++
					if err != nil || resp.StatusCode >= 400 {
						results.FailedRequests++
					} else {
						results.SuccessfulRequests++
					}
					
					if responseTime < results.MinResponseTime || results.MinResponseTime == 0 {
						results.MinResponseTime = responseTime
					}
					if responseTime > results.MaxResponseTime {
						results.MaxResponseTime = responseTime
					}
					mu.Unlock()
					
					if resp != nil {
						resp.Body.Close()
					}
					
				case <-timeout:
					return
				}
			}
		}(i)
	}

	wg.Wait()
	results.EndTime = time.Since(startTime)
	results.AverageResponseTime = results.EndTime / time.Duration(results.TotalRequests)
	results.RequestsPerSecond = float64(results.TotalRequests) / results.EndTime.Seconds()
	results.ErrorRate = float64(results.FailedRequests) / float64(results.TotalRequests) * 100

	suite.results = append(suite.results, results)
	log.Printf("✅ External Integrations Load Test completed: %d requests, %.2f RPS, %.2f%% error rate",
		results.TotalRequests, results.RequestsPerSecond, results.ErrorRate)
}

func (suite *LoadTestSuite) runMLPipelineLoadTest() {
	log.Println("🤖 Running ML Pipeline Load Test...")

	startTime := time.Now()
	var wg sync.WaitGroup
	var mu sync.Mutex
	results := LoadTestResult{
		TestName:  "ML Pipeline",
		StartTime: startTime,
	}

	// ML prediction endpoints
	predictionData := map[string]interface{}{
		"vehicle_id": "test-vehicle-001",
		"data": map[string]interface{}{
			"battery_level":  85.5,
			"engine_temp":    75.2,
			"mileage":       15000,
		},
	}

	routeData := map[string]interface{}{
		"origin": map[string]float64{
			"latitude":  25.2048,
			"longitude": 55.2708,
		},
		"destination": map[string]float64{
			"latitude":  25.2582,
			"longitude": 55.3047,
		},
	}

	for i := 0; i < suite.config.ConcurrentUsers; i++ {
		wg.Add(1)
		go func(workerID int) {
			defer wg.Done()
			
			ticker := time.NewTicker(time.Duration(1000/suite.config.RequestRate) * time.Millisecond)
			defer ticker.Stop()
			
			timeout := time.After(suite.config.TestDuration)
			
			for {
				select {
				case <-ticker.C:
					var endpoint string
					var data interface{}
					
					if workerID%2 == 0 {
						endpoint = "/api/v1/predict/maintenance"
						data = predictionData
					} else {
						endpoint = "/api/v1/predict/route"
						data = routeData
					}
					
					start := time.Now()
					
					jsonData, _ := json.Marshal(data)
					resp, err := suite.client.Post(suite.config.BaseURL+endpoint, "application/json", bytes.NewBuffer(jsonData))
					responseTime := time.Since(start)
					
					mu.Lock()
					results.TotalRequests++
					if err != nil || resp.StatusCode >= 400 {
						results.FailedRequests++
					} else {
						results.SuccessfulRequests++
					}
					
					if responseTime < results.MinResponseTime || results.MinResponseTime == 0 {
						results.MinResponseTime = responseTime
					}
					if responseTime > results.MaxResponseTime {
						results.MaxResponseTime = responseTime
					}
					mu.Unlock()
					
					if resp != nil {
						resp.Body.Close()
					}
					
				case <-timeout:
					return
				}
			}
		}(i)
	}

	wg.Wait()
	results.EndTime = time.Since(startTime)
	results.AverageResponseTime = results.EndTime / time.Duration(results.TotalRequests)
	results.RequestsPerSecond = float64(results.TotalRequests) / results.EndTime.Seconds()
	results.ErrorRate = float64(results.FailedRequests) / float64(results.TotalRequests) * 100

	suite.results = append(suite.results, results)
	log.Printf("✅ ML Pipeline Load Test completed: %d requests, %.2f RPS, %.2f%% error rate",
		results.TotalRequests, results.RequestsPerSecond, results.ErrorRate)
}

func (suite *LoadTestSuite) runVehicleGatewayLoadTest() {
	log.Println("🚗 Running Vehicle Gateway Load Test...")

	startTime := time.Now()
	var wg sync.WaitGroup
	var mu sync.Mutex
	results := LoadTestResult{
		TestName:  "Vehicle Gateway",
		StartTime: startTime,
	}

	// Vehicle Gateway endpoints
	endpoints := []string{
		"/api/v1/vehicles/status",
		"/api/v1/vehicles/health",
		"/api/v1/vehicles/commands",
	}

	for i := 0; i < suite.config.ConcurrentUsers; i++ {
		wg.Add(1)
		go func(workerID int) {
			defer wg.Done()
			
			ticker := time.NewTicker(time.Duration(1000/suite.config.RequestRate) * time.Millisecond)
			defer ticker.Stop()
			
			timeout := time.After(suite.config.TestDuration)
			
			for {
				select {
				case <-ticker.C:
					endpoint := endpoints[workerID%len(endpoints)]
					start := time.Now()
					
					resp, err := suite.client.Get(suite.config.BaseURL + endpoint)
					responseTime := time.Since(start)
					
					mu.Lock()
					results.TotalRequests++
					if err != nil || resp.StatusCode >= 400 {
						results.FailedRequests++
					} else {
						results.SuccessfulRequests++
					}
					
					if responseTime < results.MinResponseTime || results.MinResponseTime == 0 {
						results.MinResponseTime = responseTime
					}
					if responseTime > results.MaxResponseTime {
						results.MaxResponseTime = responseTime
					}
					mu.Unlock()
					
					if resp != nil {
						resp.Body.Close()
					}
					
				case <-timeout:
					return
				}
			}
		}(i)
	}

	wg.Wait()
	results.EndTime = time.Since(startTime)
	results.AverageResponseTime = results.EndTime / time.Duration(results.TotalRequests)
	results.RequestsPerSecond = float64(results.TotalRequests) / results.EndTime.Seconds()
	results.ErrorRate = float64(results.FailedRequests) / float64(results.TotalRequests) * 100

	suite.results = append(suite.results, results)
	log.Printf("✅ Vehicle Gateway Load Test completed: %d requests, %.2f RPS, %.2f%% error rate",
		results.TotalRequests, results.RequestsPerSecond, results.ErrorRate)
}

func (suite *LoadTestSuite) runDatabaseLoadTest() {
	log.Println("🗄️ Running Database Load Test...")

	startTime := time.Now()
	var wg sync.WaitGroup
	var mu sync.Mutex
	results := LoadTestResult{
		TestName:  "Database Operations",
		StartTime: startTime,
	}

	// Database-intensive operations
	vehicleData := map[string]interface{}{
		"fleet_id":     "load-test-fleet",
		"asset_tag":    "LOAD-TEST-001",
		"manufacturer": "Tesla",
		"model":        "Model 3",
		"year":         2023,
	}

	for i := 0; i < suite.config.ConcurrentUsers; i++ {
		wg.Add(1)
		go func(workerID int) {
			defer wg.Done()
			
			ticker := time.NewTicker(time.Duration(1000/suite.config.RequestRate) * time.Millisecond)
			defer ticker.Stop()
			
			timeout := time.After(suite.config.TestDuration)
			
			for {
				select {
				case <-ticker.C:
					// Alternate between create and read operations
					var endpoint string
					var method string
					var data interface{}
					
					if workerID%2 == 0 {
						// Create vehicle
						method = "POST"
						endpoint = "/api/v1/fleet/vehicles"
						vehicleData["asset_tag"] = fmt.Sprintf("LOAD-TEST-%03d", workerID)
						data = vehicleData
					} else {
						// Read vehicles
						method = "GET"
						endpoint = "/api/v1/fleet/vehicles"
						data = nil
					}
					
					start := time.Now()
					
					var resp *http.Response
					var err error
					
					if method == "POST" {
						jsonData, _ := json.Marshal(data)
						resp, err = suite.client.Post(suite.config.BaseURL+endpoint, "application/json", bytes.NewBuffer(jsonData))
					} else {
						resp, err = suite.client.Get(suite.config.BaseURL + endpoint)
					}
					
					responseTime := time.Since(start)
					
					mu.Lock()
					results.TotalRequests++
					if err != nil || resp.StatusCode >= 400 {
						results.FailedRequests++
					} else {
						results.SuccessfulRequests++
					}
					
					if responseTime < results.MinResponseTime || results.MinResponseTime == 0 {
						results.MinResponseTime = responseTime
					}
					if responseTime > results.MaxResponseTime {
						results.MaxResponseTime = responseTime
					}
					mu.Unlock()
					
					if resp != nil {
						resp.Body.Close()
					}
					
				case <-timeout:
					return
				}
			}
		}(i)
	}

	wg.Wait()
	results.EndTime = time.Since(startTime)
	results.AverageResponseTime = results.EndTime / time.Duration(results.TotalRequests)
	results.RequestsPerSecond = float64(results.TotalRequests) / results.EndTime.Seconds()
	results.ErrorRate = float64(results.FailedRequests) / float64(results.TotalRequests) * 100

	suite.results = append(suite.results, results)
	log.Printf("✅ Database Load Test completed: %d requests, %.2f RPS, %.2f%% error rate",
		results.TotalRequests, results.RequestsPerSecond, results.ErrorRate)
}

func (suite *LoadTestSuite) runMessageQueueLoadTest() {
	log.Println("📨 Running Message Queue Load Test...")

	startTime := time.Now()
	var wg sync.WaitGroup
	var mu sync.Mutex
	results := LoadTestResult{
		TestName:  "Message Queue",
		StartTime: startTime,
	}

	// Message queue operations (simulated via API endpoints that use Kafka)
	endpoints := []string{
		"/api/v1/analytics/realtime",
		"/api/v1/analytics/stream",
		"/api/v1/sync/status",
	}

	for i := 0; i < suite.config.ConcurrentUsers; i++ {
		wg.Add(1)
		go func(workerID int) {
			defer wg.Done()
			
			ticker := time.NewTicker(time.Duration(1000/suite.config.RequestRate) * time.Millisecond)
			defer ticker.Stop()
			
			timeout := time.After(suite.config.TestDuration)
			
			for {
				select {
				case <-ticker.C:
					endpoint := endpoints[workerID%len(endpoints)]
					start := time.Now()
					
					resp, err := suite.client.Get(suite.config.BaseURL + endpoint)
					responseTime := time.Since(start)
					
					mu.Lock()
					results.TotalRequests++
					if err != nil || resp.StatusCode >= 400 {
						results.FailedRequests++
					} else {
						results.SuccessfulRequests++
					}
					
					if responseTime < results.MinResponseTime || results.MinResponseTime == 0 {
						results.MinResponseTime = responseTime
					}
					if responseTime > results.MaxResponseTime {
						results.MaxResponseTime = responseTime
					}
					mu.Unlock()
					
					if resp != nil {
						resp.Body.Close()
					}
					
				case <-timeout:
					return
				}
			}
		}(i)
	}

	wg.Wait()
	results.EndTime = time.Since(startTime)
	results.AverageResponseTime = results.EndTime / time.Duration(results.TotalRequests)
	results.RequestsPerSecond = float64(results.TotalRequests) / results.EndTime.Seconds()
	results.ErrorRate = float64(results.FailedRequests) / float64(results.TotalRequests) * 100

	suite.results = append(suite.results, results)
	log.Printf("✅ Message Queue Load Test completed: %d requests, %.2f RPS, %.2f%% error rate",
		results.TotalRequests, results.RequestsPerSecond, results.ErrorRate)
}

func (suite *LoadTestSuite) runEndToEndLoadTest() {
	log.Println("🔄 Running End-to-End Load Test...")

	startTime := time.Now()
	var wg sync.WaitGroup
	var mu sync.Mutex
	results := LoadTestResult{
		TestName:  "End-to-End Workflow",
		StartTime: startTime,
	}

	// Complete workflow simulation
	for i := 0; i < suite.config.ConcurrentUsers; i++ {
		wg.Add(1)
		go func(workerID int) {
			defer wg.Done()
			
			ticker := time.NewTicker(time.Duration(1000/suite.config.RequestRate) * time.Millisecond)
			defer ticker.Stop()
			
			timeout := time.After(suite.config.TestDuration)
			
			for {
				select {
				case <-ticker.C:
					// Simulate complete workflow
					workflowSteps := []func() error{
						func() error { return suite.simulateVehicleCreation(workerID) },
						func() error { return suite.simulateLocationUpdate(workerID) },
						func() error { return suite.simulateDispatch(workerID) },
						func() error { return suite.simulateAnalytics(workerID) },
						func() error { return suite.simulateMLPrediction(workerID) },
					}
					
					start := time.Now()
					
					var err error
					for _, step := range workflowSteps {
						if stepErr := step(); stepErr != nil {
							err = stepErr
							break
						}
					}
					
					responseTime := time.Since(start)
					
					mu.Lock()
					results.TotalRequests++
					if err != nil {
						results.FailedRequests++
					} else {
						results.SuccessfulRequests++
					}
					
					if responseTime < results.MinResponseTime || results.MinResponseTime == 0 {
						results.MinResponseTime = responseTime
					}
					if responseTime > results.MaxResponseTime {
						results.MaxResponseTime = responseTime
					}
					mu.Unlock()
					
				case <-timeout:
					return
				}
			}
		}(i)
	}

	wg.Wait()
	results.EndTime = time.Since(startTime)
	results.AverageResponseTime = results.EndTime / time.Duration(results.TotalRequests)
	results.RequestsPerSecond = float64(results.TotalRequests) / results.EndTime.Seconds()
	results.ErrorRate = float64(results.FailedRequests) / float64(results.TotalRequests) * 100

	suite.results = append(suite.results, results)
	log.Printf("✅ End-to-End Load Test completed: %d requests, %.2f RPS, %.2f%% error rate",
		results.TotalRequests, results.RequestsPerSecond, results.ErrorRate)
}

// Workflow simulation methods

func (suite *LoadTestSuite) simulateVehicleCreation(workerID int) error {
	vehicleData := map[string]interface{}{
		"fleet_id":     "e2e-fleet",
		"asset_tag":    fmt.Sprintf("E2E-%03d", workerID),
		"manufacturer": "Tesla",
		"model":        "Model 3",
		"year":         2023,
	}
	
	jsonData, _ := json.Marshal(vehicleData)
	resp, err := suite.client.Post(suite.config.BaseURL+"/api/v1/fleet/vehicles", "application/json", bytes.NewBuffer(jsonData))
	if err != nil {
		return err
	}
	defer resp.Body.Close()
	
	if resp.StatusCode >= 400 {
		return fmt.Errorf("vehicle creation failed with status %d", resp.StatusCode)
	}
	
	return nil
}

func (suite *LoadTestSuite) simulateLocationUpdate(workerID int) error {
	locationData := map[string]interface{}{
		"latitude":  25.2048 + float64(workerID)*0.001,
		"longitude": 55.2708 + float64(workerID)*0.001,
		"speed":     45.0 + float64(workerID),
	}
	
	jsonData, _ := json.Marshal(locationData)
	vehicleID := fmt.Sprintf("vehicle-%03d", workerID)
	resp, err := suite.client.Post(suite.config.BaseURL+"/api/v1/fleet/vehicles/"+vehicleID+"/location", "application/json", bytes.NewBuffer(jsonData))
	if err != nil {
		return err
	}
	defer resp.Body.Close()
	
	if resp.StatusCode >= 400 {
		return fmt.Errorf("location update failed with status %d", resp.StatusCode)
	}
	
	return nil
}

func (suite *LoadTestSuite) simulateDispatch(workerID int) error {
	dispatchData := map[string]interface{}{
		"trip_id":     fmt.Sprintf("trip-%03d", workerID),
		"origin":      map[string]float64{"latitude": 25.2048, "longitude": 55.2708},
		"destination": map[string]float64{"latitude": 25.2582, "longitude": 55.3047},
	}
	
	jsonData, _ := json.Marshal(dispatchData)
	vehicleID := fmt.Sprintf("vehicle-%03d", workerID)
	resp, err := suite.client.Post(suite.config.BaseURL+"/api/v1/fleet/vehicles/"+vehicleID+"/dispatch", "application/json", bytes.NewBuffer(jsonData))
	if err != nil {
		return err
	}
	defer resp.Body.Close()
	
	if resp.StatusCode >= 400 {
		return fmt.Errorf("dispatch failed with status %d", resp.StatusCode)
	}
	
	return nil
}

func (suite *LoadTestSuite) simulateAnalytics(workerID int) error {
	resp, err := suite.client.Get(suite.config.BaseURL + "/api/v1/analytics/kpis?timeframe=1h")
	if err != nil {
		return err
	}
	defer resp.Body.Close()
	
	if resp.StatusCode >= 400 {
		return fmt.Errorf("analytics failed with status %d", resp.StatusCode)
	}
	
	return nil
}

func (suite *LoadTestSuite) simulateMLPrediction(workerID int) error {
	predictionData := map[string]interface{}{
		"vehicle_id": fmt.Sprintf("vehicle-%03d", workerID),
		"data": map[string]interface{}{
			"battery_level": 85.5,
			"engine_temp":   75.2,
		},
	}
	
	jsonData, _ := json.Marshal(predictionData)
	resp, err := suite.client.Post(suite.config.BaseURL+"/api/v1/predict/maintenance", "application/json", bytes.NewBuffer(jsonData))
	if err != nil {
		return err
	}
	defer resp.Body.Close()
	
	if resp.StatusCode >= 400 {
		return fmt.Errorf("ML prediction failed with status %d", resp.StatusCode)
	}
	
	return nil
}

func (suite *LoadTestSuite) generateReport() {
	log.Println("📊 Generating Load Test Report...")
	
	fmt.Println("\n" + "="*80)
	fmt.Println("ATLASMESH FLEET OS LOAD TESTING REPORT")
	fmt.Println("="*80)
	
	totalRequests := 0
	totalSuccessful := 0
	totalFailed := 0
	
	for _, result := range suite.results {
		fmt.Printf("\n%s\n", "-"*60)
		fmt.Printf("Test: %s\n", result.TestName)
		fmt.Printf("Duration: %v\n", result.EndTime)
		fmt.Printf("Total Requests: %d\n", result.TotalRequests)
		fmt.Printf("Successful: %d\n", result.SuccessfulRequests)
		fmt.Printf("Failed: %d\n", result.FailedRequests)
		fmt.Printf("Requests/Second: %.2f\n", result.RequestsPerSecond)
		fmt.Printf("Error Rate: %.2f%%\n", result.ErrorRate)
		fmt.Printf("Average Response Time: %v\n", result.AverageResponseTime)
		fmt.Printf("Min Response Time: %v\n", result.MinResponseTime)
		fmt.Printf("Max Response Time: %v\n", result.MaxResponseTime)
		
		totalRequests += result.TotalRequests
		totalSuccessful += result.SuccessfulRequests
		totalFailed += result.FailedRequests
	}
	
	fmt.Printf("\n%s\n", "="*60)
	fmt.Println("SUMMARY")
	fmt.Printf("Total Requests: %d\n", totalRequests)
	fmt.Printf("Total Successful: %d\n", totalSuccessful)
	fmt.Printf("Total Failed: %d\n", totalFailed)
	fmt.Printf("Overall Success Rate: %.2f%%\n", float64(totalSuccessful)/float64(totalRequests)*100)
	fmt.Printf("Overall Error Rate: %.2f%%\n", float64(totalFailed)/float64(totalRequests)*100)
	fmt.Println("="*60)
	
	// Performance recommendations
	fmt.Println("\nPERFORMANCE RECOMMENDATIONS:")
	
	for _, result := range suite.results {
		if result.ErrorRate > 5.0 {
			fmt.Printf("⚠️  %s: High error rate (%.2f%%) - investigate service stability\n", result.TestName, result.ErrorRate)
		}
		if result.AverageResponseTime > 2*time.Second {
			fmt.Printf("⚠️  %s: Slow response time (%v) - optimize performance\n", result.TestName, result.AverageResponseTime)
		}
		if result.RequestsPerSecond < 100 {
			fmt.Printf("⚠️  %s: Low throughput (%.2f RPS) - consider scaling\n", result.TestName, result.RequestsPerSecond)
		}
	}
	
	fmt.Println("\n✅ Load testing completed successfully!")
}
