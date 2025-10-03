package main

import (
	"context"
	"encoding/json"
	"fmt"
	"log"
	"sync"
	"time"
)

// StreamProcessor handles real-time stream processing of telemetry data
// Implements sliding window analytics, anomaly detection, and aggregations
type StreamProcessor struct {
	service         *TelemetryIngestService
	processingQueue chan VehicleTelemetry
	workers         []*StreamWorker
	windowSize      time.Duration
	dataWindow      map[string][]VehicleTelemetry
	windowMutex     sync.RWMutex
	aggregations    map[string]*AggregationState
	aggregationsMutex sync.RWMutex
	anomalyDetector *AnomalyDetector
	running         bool
	stopChan        chan bool
}

type StreamWorker struct {
	id      int
	service *TelemetryIngestService
	processor *StreamProcessor
	running bool
	stopChan chan bool
}

type AggregationState struct {
	VehicleID          string                 `json:"vehicle_id"`
	WindowStart        time.Time              `json:"window_start"`
	WindowEnd          time.Time              `json:"window_end"`
	MessageCount       int                    `json:"message_count"`
	AvgSpeed           float64                `json:"avg_speed"`
	MaxSpeed           float64                `json:"max_speed"`
	MinSpeed           float64                `json:"min_speed"`
	DistanceTraveled   float64                `json:"distance_traveled"`
	AvgBatteryLevel    float64                `json:"avg_battery_level"`
	BatteryDrain       float64                `json:"battery_drain"`
	LocationPoints     []Location             `json:"location_points"`
	AlertCount         int                    `json:"alert_count"`
	DataQualityAvg     float64                `json:"data_quality_avg"`
	AnomalyScore       float64                `json:"anomaly_score"`
	Metadata           map[string]interface{} `json:"metadata"`
}

type AnomalyDetector struct {
	processor        *StreamProcessor
	models           map[string]*AnomalyModel
	modelsMutex      sync.RWMutex
	thresholds       map[string]float64
	detectionWindow  time.Duration
}

type AnomalyModel struct {
	VehicleID        string    `json:"vehicle_id"`
	LastUpdate       time.Time `json:"last_update"`
	SpeedBaseline    float64   `json:"speed_baseline"`
	SpeedVariance    float64   `json:"speed_variance"`
	LocationBaseline Location  `json:"location_baseline"`
	BatteryBaseline  float64   `json:"battery_baseline"`
	SampleCount      int       `json:"sample_count"`
}

func NewStreamProcessor(service *TelemetryIngestService) *StreamProcessor {
	processor := &StreamProcessor{
		service:         service,
		processingQueue: make(chan VehicleTelemetry, 10000),
		windowSize:      5 * time.Minute,
		dataWindow:      make(map[string][]VehicleTelemetry),
		aggregations:    make(map[string]*AggregationState),
		running:         false,
		stopChan:        make(chan bool),
	}
	
	// Initialize anomaly detector
	processor.anomalyDetector = &AnomalyDetector{
		processor:       processor,
		models:          make(map[string]*AnomalyModel),
		thresholds:      service.config.AlertThresholds,
		detectionWindow: 10 * time.Minute,
	}
	
	// Create workers
	for i := 0; i < service.config.ProcessingWorkers; i++ {
		worker := &StreamWorker{
			id:        i,
			service:   service,
			processor: processor,
			running:   false,
			stopChan:  make(chan bool),
		}
		processor.workers = append(processor.workers, worker)
	}
	
	return processor
}

func (sp *StreamProcessor) Start() {
	if sp.running {
		return
	}
	
	sp.running = true
	log.Printf("🚀 Starting Stream Processor with %d workers", len(sp.workers))
	
	// Start workers
	for _, worker := range sp.workers {
		go worker.Start()
	}
	
	// Start window management
	go sp.manageWindows()
	
	// Start aggregation processing
	go sp.processAggregations()
	
	// Start anomaly detection
	go sp.anomalyDetector.Start()
	
	log.Println("✅ Stream Processor started")
}

func (sp *StreamProcessor) Stop() {
	if !sp.running {
		return
	}
	
	log.Println("🛑 Stopping Stream Processor...")
	sp.running = false
	
	// Stop workers
	for _, worker := range sp.workers {
		worker.Stop()
	}
	
	// Stop anomaly detector
	sp.anomalyDetector.Stop()
	
	// Signal main processor to stop
	sp.stopChan <- true
	
	log.Println("✅ Stream Processor stopped")
}

func (sw *StreamWorker) Start() {
	sw.running = true
	log.Printf("🔧 Stream Worker %d started", sw.id)
	
	for sw.running {
		select {
		case telemetry := <-sw.processor.processingQueue:
			sw.processTelemetry(telemetry)
		case <-sw.stopChan:
			sw.running = false
		}
	}
	
	log.Printf("🔧 Stream Worker %d stopped", sw.id)
}

func (sw *StreamWorker) Stop() {
	if sw.running {
		sw.running = false
		sw.stopChan <- true
	}
}

func (sw *StreamWorker) processTelemetry(telemetry VehicleTelemetry) {
	start := time.Now()
	defer func() {
		sw.service.metrics.ProcessingLatency.WithLabelValues(telemetry.VehicleID, "stream_processing").Observe(time.Since(start).Seconds())
		sw.service.metrics.MessagesProcessed.WithLabelValues(telemetry.VehicleID, "stream_processing", "success").Inc()
	}()
	
	// Add to sliding window
	sw.processor.addToWindow(telemetry)
	
	// Update anomaly detection model
	sw.processor.anomalyDetector.UpdateModel(telemetry)
	
	// Check for real-time anomalies
	if anomalyScore := sw.processor.anomalyDetector.DetectAnomaly(telemetry); anomalyScore > 0.7 {
		sw.handleAnomaly(telemetry, anomalyScore)
	}
	
	// Update real-time aggregations
	sw.processor.updateAggregations(telemetry)
	
	// Store processed telemetry for time-series analysis
	sw.storeTimeSeriesData(telemetry)
}

func (sp *StreamProcessor) addToWindow(telemetry VehicleTelemetry) {
	sp.windowMutex.Lock()
	defer sp.windowMutex.Unlock()
	
	vehicleID := telemetry.VehicleID
	
	// Initialize window for vehicle if not exists
	if _, exists := sp.dataWindow[vehicleID]; !exists {
		sp.dataWindow[vehicleID] = []VehicleTelemetry{}
	}
	
	// Add new telemetry
	sp.dataWindow[vehicleID] = append(sp.dataWindow[vehicleID], telemetry)
	
	// Remove old data outside window
	cutoff := time.Now().Add(-sp.windowSize)
	validData := []VehicleTelemetry{}
	
	for _, data := range sp.dataWindow[vehicleID] {
		if data.Timestamp.After(cutoff) {
			validData = append(validData, data)
		}
	}
	
	sp.dataWindow[vehicleID] = validData
}

func (sp *StreamProcessor) updateAggregations(telemetry VehicleTelemetry) {
	sp.aggregationsMutex.Lock()
	defer sp.aggregationsMutex.Unlock()
	
	vehicleID := telemetry.VehicleID
	now := time.Now()
	windowStart := now.Truncate(time.Minute) // 1-minute windows
	
	aggregationKey := fmt.Sprintf("%s:%d", vehicleID, windowStart.Unix())
	
	// Get or create aggregation state
	agg, exists := sp.aggregations[aggregationKey]
	if !exists {
		agg = &AggregationState{
			VehicleID:   vehicleID,
			WindowStart: windowStart,
			WindowEnd:   windowStart.Add(time.Minute),
			Metadata:    make(map[string]interface{}),
		}
		sp.aggregations[aggregationKey] = agg
	}
	
	// Update aggregation
	agg.MessageCount++
	
	// Speed aggregation
	if agg.MessageCount == 1 {
		agg.AvgSpeed = telemetry.Speed
		agg.MaxSpeed = telemetry.Speed
		agg.MinSpeed = telemetry.Speed
	} else {
		agg.AvgSpeed = ((agg.AvgSpeed * float64(agg.MessageCount-1)) + telemetry.Speed) / float64(agg.MessageCount)
		if telemetry.Speed > agg.MaxSpeed {
			agg.MaxSpeed = telemetry.Speed
		}
		if telemetry.Speed < agg.MinSpeed {
			agg.MinSpeed = telemetry.Speed
		}
	}
	
	// Battery level aggregation
	if telemetry.BatteryLevel > 0 {
		if agg.AvgBatteryLevel == 0 {
			agg.AvgBatteryLevel = telemetry.BatteryLevel
		} else {
			batteryCount := float64(agg.MessageCount)
			agg.AvgBatteryLevel = ((agg.AvgBatteryLevel * (batteryCount - 1)) + telemetry.BatteryLevel) / batteryCount
		}
	}
	
	// Location tracking
	agg.LocationPoints = append(agg.LocationPoints, telemetry.Location)
	
	// Calculate distance traveled
	if len(agg.LocationPoints) > 1 {
		lastPoint := agg.LocationPoints[len(agg.LocationPoints)-2]
		currentPoint := telemetry.Location
		distance := calculateDistance(lastPoint, currentPoint)
		agg.DistanceTraveled += distance
	}
	
	// Data quality aggregation
	if agg.DataQualityAvg == 0 {
		agg.DataQualityAvg = telemetry.QualityScore
	} else {
		qualityCount := float64(agg.MessageCount)
		agg.DataQualityAvg = ((agg.DataQualityAvg * (qualityCount - 1)) + telemetry.QualityScore) / qualityCount
	}
	
	// Alert count
	agg.AlertCount += len(telemetry.Alerts)
	
	// Abu Dhabi specific metadata
	if sp.service.config.AbuDhabiMode {
		if telemetry.SystemStatus["within_abu_dhabi"] == true {
			agg.Metadata["abu_dhabi_time_percentage"] = calculateAbuDhabiTimePercentage(agg)
		}
		if telemetry.SystemStatus["peak_heat_hours"] == true {
			agg.Metadata["peak_heat_exposure"] = true
		}
	}
}

func (sp *StreamProcessor) manageWindows() {
	ticker := time.NewTicker(1 * time.Minute)
	defer ticker.Stop()
	
	for sp.running {
		select {
		case <-ticker.C:
			sp.cleanupOldWindows()
		case <-sp.stopChan:
			return
		}
	}
}

func (sp *StreamProcessor) cleanupOldWindows() {
	sp.windowMutex.Lock()
	defer sp.windowMutex.Unlock()
	
	cutoff := time.Now().Add(-sp.windowSize * 2) // Keep double window size for safety
	
	for vehicleID, data := range sp.dataWindow {
		validData := []VehicleTelemetry{}
		for _, telemetry := range data {
			if telemetry.Timestamp.After(cutoff) {
				validData = append(validData, telemetry)
			}
		}
		sp.dataWindow[vehicleID] = validData
		
		// Remove empty windows
		if len(validData) == 0 {
			delete(sp.dataWindow, vehicleID)
		}
	}
}

func (sp *StreamProcessor) processAggregations() {
	ticker := time.NewTicker(1 * time.Minute)
	defer ticker.Stop()
	
	for sp.running {
		select {
		case <-ticker.C:
			sp.publishCompletedAggregations()
		case <-sp.stopChan:
			return
		}
	}
}

func (sp *StreamProcessor) publishCompletedAggregations() {
	sp.aggregationsMutex.Lock()
	defer sp.aggregationsMutex.Unlock()
	
	now := time.Now()
	completedAggregations := []string{}
	
	for key, agg := range sp.aggregations {
		if now.After(agg.WindowEnd.Add(30 * time.Second)) { // 30-second grace period
			// Publish aggregation
			sp.publishAggregation(agg)
			completedAggregations = append(completedAggregations, key)
		}
	}
	
	// Clean up completed aggregations
	for _, key := range completedAggregations {
		delete(sp.aggregations, key)
	}
}

func (sp *StreamProcessor) publishAggregation(agg *AggregationState) {
	// Store in Redis for real-time access
	aggJSON, _ := json.Marshal(agg)
	key := fmt.Sprintf("aggregation:%s:%d", agg.VehicleID, agg.WindowStart.Unix())
	sp.service.redis.Set(context.Background(), key, aggJSON, 1*time.Hour)
	
	// Publish to Kafka for further processing
	topic := "vehicle.aggregations"
	sp.service.kafkaProducer.Produce(&kafka.Message{
		TopicPartition: kafka.TopicPartition{Topic: &topic, Partition: kafka.PartitionAny},
		Key:            []byte(agg.VehicleID),
		Value:          aggJSON,
	}, nil)
	
	// Update metrics
	sp.service.metrics.MessagesProcessed.WithLabelValues(agg.VehicleID, "aggregation", "published").Inc()
	
	log.Printf("📊 Published aggregation for vehicle %s: %d messages, avg speed %.1f km/h, distance %.2f km",
		agg.VehicleID, agg.MessageCount, agg.AvgSpeed, agg.DistanceTraveled)
}

func (sw *StreamWorker) handleAnomaly(telemetry VehicleTelemetry, score float64) {
	alert := Alert{
		ID:        fmt.Sprintf("anomaly_%s_%d", telemetry.VehicleID, time.Now().UnixNano()),
		Type:      "anomaly_detection",
		Severity:  determineAnomalySeverity(score),
		Message:   fmt.Sprintf("Anomalous behavior detected (score: %.2f)", score),
		Timestamp: time.Now(),
		VehicleID: telemetry.VehicleID,
		Location:  telemetry.Location,
		Metadata: map[string]interface{}{
			"anomaly_score": score,
			"detection_method": "stream_processing",
			"telemetry_timestamp": telemetry.Timestamp,
		},
	}
	
	sw.service.handleAlert(alert)
}

func (sw *StreamWorker) storeTimeSeriesData(telemetry VehicleTelemetry) {
	// Store in time-series format for historical analysis
	// This would typically go to InfluxDB, TimescaleDB, or similar
	
	timeSeriesData := map[string]interface{}{
		"vehicle_id":     telemetry.VehicleID,
		"timestamp":      telemetry.Timestamp,
		"location":       telemetry.Location,
		"speed":          telemetry.Speed,
		"heading":        telemetry.Heading,
		"battery_level":  telemetry.BatteryLevel,
		"fuel_level":     telemetry.FuelLevel,
		"quality_score":  telemetry.QualityScore,
		"system_status":  telemetry.SystemStatus,
		"sensor_data":    telemetry.SensorData,
	}
	
	// Store in Redis with TTL for recent access
	tsJSON, _ := json.Marshal(timeSeriesData)
	key := fmt.Sprintf("timeseries:%s:%d", telemetry.VehicleID, telemetry.Timestamp.Unix())
	sw.service.redis.Set(context.Background(), key, tsJSON, 24*time.Hour)
	
	// Publish to Kafka for long-term storage
	topic := "vehicle.timeseries"
	sw.service.kafkaProducer.Produce(&kafka.Message{
		TopicPartition: kafka.TopicPartition{Topic: &topic, Partition: kafka.PartitionAny},
		Key:            []byte(telemetry.VehicleID),
		Value:          tsJSON,
	}, nil)
}

// Anomaly Detection Implementation
func (ad *AnomalyDetector) Start() {
	log.Println("🤖 Starting Anomaly Detector")
	
	// Periodically update models and clean up old ones
	go ad.modelMaintenance()
}

func (ad *AnomalyDetector) Stop() {
	log.Println("🤖 Stopping Anomaly Detector")
}

func (ad *AnomalyDetector) UpdateModel(telemetry VehicleTelemetry) {
	ad.modelsMutex.Lock()
	defer ad.modelsMutex.Unlock()
	
	vehicleID := telemetry.VehicleID
	model, exists := ad.models[vehicleID]
	
	if !exists {
		model = &AnomalyModel{
			VehicleID:        vehicleID,
			LastUpdate:       time.Now(),
			SpeedBaseline:    telemetry.Speed,
			LocationBaseline: telemetry.Location,
			BatteryBaseline:  telemetry.BatteryLevel,
			SampleCount:      1,
		}
		ad.models[vehicleID] = model
		return
	}
	
	// Update model with exponential moving average
	alpha := 0.1 // Learning rate
	model.SpeedBaseline = (1-alpha)*model.SpeedBaseline + alpha*telemetry.Speed
	
	if telemetry.BatteryLevel > 0 {
		model.BatteryBaseline = (1-alpha)*model.BatteryBaseline + alpha*telemetry.BatteryLevel
	}
	
	// Update variance (simplified)
	speedDiff := telemetry.Speed - model.SpeedBaseline
	model.SpeedVariance = (1-alpha)*model.SpeedVariance + alpha*(speedDiff*speedDiff)
	
	model.LastUpdate = time.Now()
	model.SampleCount++
}

func (ad *AnomalyDetector) DetectAnomaly(telemetry VehicleTelemetry) float64 {
	ad.modelsMutex.RLock()
	defer ad.modelsMutex.RUnlock()
	
	model, exists := ad.models[telemetry.VehicleID]
	if !exists || model.SampleCount < 10 {
		return 0.0 // Need more data to establish baseline
	}
	
	anomalyScore := 0.0
	
	// Speed anomaly detection
	if model.SpeedVariance > 0 {
		speedZScore := abs(telemetry.Speed-model.SpeedBaseline) / sqrt(model.SpeedVariance)
		if speedZScore > 2.0 {
			anomalyScore += 0.3
		}
		if speedZScore > 3.0 {
			anomalyScore += 0.2
		}
	}
	
	// Battery drain anomaly (sudden drops)
	if telemetry.BatteryLevel > 0 && model.BatteryBaseline > 0 {
		batteryDrop := model.BatteryBaseline - telemetry.BatteryLevel
		if batteryDrop > 10.0 { // More than 10% drop from baseline
			anomalyScore += 0.4
		}
	}
	
	// Location anomaly (significant deviation from expected area)
	locationDistance := calculateDistance(model.LocationBaseline, telemetry.Location)
	if locationDistance > 50.0 { // More than 50km from baseline
		anomalyScore += 0.3
	}
	
	// Abu Dhabi specific anomaly checks
	if ad.processor.service.config.AbuDhabiMode {
		anomalyScore += ad.detectAbuDhabiAnomalies(telemetry, model)
	}
	
	return min(anomalyScore, 1.0)
}

func (ad *AnomalyDetector) detectAbuDhabiAnomalies(telemetry VehicleTelemetry, model *AnomalyModel) float64 {
	score := 0.0
	
	// Check if vehicle is outside Abu Dhabi boundaries unexpectedly
	if !isWithinAbuDhabi(telemetry.Location) && isWithinAbuDhabi(model.LocationBaseline) {
		score += 0.2 // Vehicle left Abu Dhabi unexpectedly
	}
	
	// Check for operation during extreme heat hours without proper cooling
	hour := telemetry.Timestamp.In(time.FixedZone("GST", 4*3600)).Hour()
	if hour >= 12 && hour <= 16 { // Peak heat hours
		if coolingStatus, exists := telemetry.SystemStatus["cooling_system"]; exists {
			if coolingStatus != "active" {
				score += 0.3 // Operating in extreme heat without cooling
			}
		}
	}
	
	// Check for prayer time violations (if configured)
	if telemetry.SystemStatus["prayer_time_proximity"] == true {
		if telemetry.Speed > 0 { // Vehicle moving during prayer time
			score += 0.1 // Minor anomaly for cultural consideration
		}
	}
	
	return score
}

func (ad *AnomalyDetector) modelMaintenance() {
	ticker := time.NewTicker(1 * time.Hour)
	defer ticker.Stop()
	
	for {
		select {
		case <-ticker.C:
			ad.cleanupOldModels()
		}
	}
}

func (ad *AnomalyDetector) cleanupOldModels() {
	ad.modelsMutex.Lock()
	defer ad.modelsMutex.Unlock()
	
	cutoff := time.Now().Add(-24 * time.Hour)
	modelsToDelete := []string{}
	
	for vehicleID, model := range ad.models {
		if model.LastUpdate.Before(cutoff) {
			modelsToDelete = append(modelsToDelete, vehicleID)
		}
	}
	
	for _, vehicleID := range modelsToDelete {
		delete(ad.models, vehicleID)
		log.Printf("🤖 Cleaned up anomaly model for inactive vehicle: %s", vehicleID)
	}
}

// Utility functions
func calculateDistance(loc1, loc2 Location) float64 {
	// Simplified distance calculation using Haversine formula
	const R = 6371 // Earth's radius in kilometers
	
	lat1 := loc1.Latitude * math.Pi / 180
	lat2 := loc2.Latitude * math.Pi / 180
	deltaLat := (loc2.Latitude - loc1.Latitude) * math.Pi / 180
	deltaLon := (loc2.Longitude - loc1.Longitude) * math.Pi / 180
	
	a := math.Sin(deltaLat/2)*math.Sin(deltaLat/2) +
		math.Cos(lat1)*math.Cos(lat2)*math.Sin(deltaLon/2)*math.Sin(deltaLon/2)
	c := 2 * math.Atan2(math.Sqrt(a), math.Sqrt(1-a))
	
	return R * c
}

func calculateAbuDhabiTimePercentage(agg *AggregationState) float64 {
	// Calculate percentage of time spent within Abu Dhabi boundaries
	abuDhabiPoints := 0
	totalPoints := len(agg.LocationPoints)
	
	for _, location := range agg.LocationPoints {
		if isWithinAbuDhabi(location) {
			abuDhabiPoints++
		}
	}
	
	if totalPoints == 0 {
		return 0
	}
	
	return float64(abuDhabiPoints) / float64(totalPoints) * 100
}

func isWithinAbuDhabi(location Location) bool {
	// Simplified Abu Dhabi boundary check
	return location.Latitude >= 24.0 && location.Latitude <= 25.0 &&
		   location.Longitude >= 54.0 && location.Longitude <= 55.5
}

func determineAnomalySeverity(score float64) string {
	if score >= 0.8 {
		return "critical"
	} else if score >= 0.6 {
		return "high"
	} else if score >= 0.4 {
		return "medium"
	} else {
		return "low"
	}
}

func abs(x float64) float64 {
	if x < 0 {
		return -x
	}
	return x
}

func sqrt(x float64) float64 {
	return math.Sqrt(x)
}

func min(a, b float64) float64 {
	if a < b {
		return a
	}
	return b
}

// Import math package at the top
import "math"
