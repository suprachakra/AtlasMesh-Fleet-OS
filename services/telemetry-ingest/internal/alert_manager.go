package main

import (
	"context"
	"encoding/json"
	"fmt"
	"log"
	"math"
	"strings"
	"sync"
	"time"
)

// AlertManager handles real-time alert evaluation and management
// Implements configurable alert rules, escalation policies, and notification routing
type AlertManager struct {
	service         *TelemetryIngestService
	rules           []AlertRule
	rulesMutex      sync.RWMutex
	escalationRules map[string]EscalationRule
	notificationChannels map[string]NotificationChannel
	alertHistory    map[string][]Alert
	historyMutex    sync.RWMutex
	running         bool
	stopChan        chan bool
}

type EscalationRule struct {
	ID              string        `json:"id"`
	AlertType       string        `json:"alert_type"`
	InitialDelay    time.Duration `json:"initial_delay"`
	EscalationDelay time.Duration `json:"escalation_delay"`
	MaxEscalations  int           `json:"max_escalations"`
	Channels        []string      `json:"channels"`
	AbuDhabiSpecific bool         `json:"abu_dhabi_specific"`
}

type NotificationChannel struct {
	ID          string                 `json:"id"`
	Type        string                 `json:"type"` // email, sms, webhook, emergency_services
	Config      map[string]interface{} `json:"config"`
	Enabled     bool                   `json:"enabled"`
	RateLimit   time.Duration          `json:"rate_limit"`
	LastUsed    time.Time              `json:"last_used"`
}

type AlertEvaluation struct {
	Rule        AlertRule         `json:"rule"`
	Telemetry   VehicleTelemetry  `json:"telemetry"`
	Triggered   bool              `json:"triggered"`
	Value       float64           `json:"value"`
	Threshold   float64           `json:"threshold"`
	Context     map[string]interface{} `json:"context"`
}

func NewAlertManager(service *TelemetryIngestService) *AlertManager {
	am := &AlertManager{
		service:              service,
		rules:                []AlertRule{},
		escalationRules:      make(map[string]EscalationRule),
		notificationChannels: make(map[string]NotificationChannel),
		alertHistory:         make(map[string][]Alert),
		running:              false,
		stopChan:             make(chan bool),
	}
	
	// Initialize default alert rules
	am.initializeDefaultRules()
	
	// Initialize notification channels
	am.initializeNotificationChannels()
	
	// Initialize escalation rules
	am.initializeEscalationRules()
	
	return am
}

func (am *AlertManager) Start() {
	if am.running {
		return
	}
	
	am.running = true
	log.Println("🚨 Starting Alert Manager")
	
	// Start alert processing
	go am.processAlerts()
	
	// Start escalation monitoring
	go am.monitorEscalations()
	
	// Start alert history cleanup
	go am.cleanupAlertHistory()
	
	log.Println("✅ Alert Manager started")
}

func (am *AlertManager) Stop() {
	if !am.running {
		return
	}
	
	log.Println("🛑 Stopping Alert Manager...")
	am.running = false
	am.stopChan <- true
	log.Println("✅ Alert Manager stopped")
}

func (am *AlertManager) initializeDefaultRules() {
	defaultRules := []AlertRule{
		{
			ID:        "battery_low",
			Name:      "Low Battery Alert",
			Condition: "battery_level < threshold",
			Threshold: 20.0,
			Severity:  "high",
			Enabled:   true,
			Metadata: map[string]interface{}{
				"category": "power_management",
				"action":   "schedule_charging",
			},
		},
		{
			ID:        "battery_critical",
			Name:      "Critical Battery Alert",
			Condition: "battery_level < threshold",
			Threshold: 10.0,
			Severity:  "critical",
			Enabled:   true,
			Metadata: map[string]interface{}{
				"category": "power_management",
				"action":   "immediate_charging",
				"escalate": true,
			},
		},
		{
			ID:        "fuel_low",
			Name:      "Low Fuel Alert",
			Condition: "fuel_level < threshold",
			Threshold: 15.0,
			Severity:  "medium",
			Enabled:   true,
			Metadata: map[string]interface{}{
				"category": "fuel_management",
				"action":   "schedule_refueling",
			},
		},
		{
			ID:        "speed_limit_violation",
			Name:      "Speed Limit Violation",
			Condition: "speed > threshold",
			Threshold: 120.0,
			Severity:  "high",
			Enabled:   true,
			Metadata: map[string]interface{}{
				"category": "traffic_violation",
				"action":   "reduce_speed",
				"legal_implications": true,
			},
		},
		{
			ID:        "connectivity_lost",
			Name:      "Vehicle Connectivity Lost",
			Condition: "time_since_last_seen > threshold_minutes",
			Threshold: 5.0,
			Severity:  "high",
			Enabled:   true,
			Metadata: map[string]interface{}{
				"category": "connectivity",
				"action":   "investigate_connection",
			},
		},
		{
			ID:        "geofence_violation",
			Name:      "Geofence Boundary Violation",
			Condition: "outside_authorized_area",
			Threshold: 0.0,
			Severity:  "critical",
			Enabled:   true,
			Metadata: map[string]interface{}{
				"category": "security",
				"action":   "immediate_investigation",
				"escalate": true,
			},
		},
		{
			ID:        "temperature_high",
			Name:      "High Operating Temperature",
			Condition: "system_temperature > threshold",
			Threshold: 85.0,
			Severity:  "medium",
			Enabled:   true,
			Metadata: map[string]interface{}{
				"category": "thermal_management",
				"action":   "activate_cooling",
				"abu_dhabi_critical": true,
			},
		},
		{
			ID:        "emergency_stop",
			Name:      "Emergency Stop Activated",
			Condition: "emergency_stop_triggered",
			Threshold: 0.0,
			Severity:  "emergency",
			Enabled:   true,
			Metadata: map[string]interface{}{
				"category": "safety",
				"action":   "immediate_response",
				"escalate": true,
				"notify_authorities": true,
			},
		},
	}
	
	// Abu Dhabi specific rules
	if am.service.config.AbuDhabiMode {
		abuDhabiRules := []AlertRule{
			{
				ID:        "dust_storm_exposure",
				Name:      "Dust Storm Exposure Alert",
				Condition: "visibility < threshold AND dust_level > normal",
				Threshold: 1000.0, // meters
				Severity:  "high",
				Enabled:   true,
				Metadata: map[string]interface{}{
					"category": "environmental",
					"action":   "seek_shelter",
					"abu_dhabi_specific": true,
				},
			},
			{
				ID:        "extreme_heat_operation",
				Name:      "Extreme Heat Operation Warning",
				Condition: "ambient_temperature > threshold AND cooling_inactive",
				Threshold: 45.0, // Celsius
				Severity:  "high",
				Enabled:   true,
				Metadata: map[string]interface{}{
					"category": "environmental",
					"action":   "activate_cooling",
					"abu_dhabi_specific": true,
				},
			},
			{
				ID:        "prayer_time_proximity",
				Name:      "Prayer Time Consideration",
				Condition: "prayer_time_proximity AND high_speed",
				Threshold: 30.0, // km/h
				Severity:  "low",
				Enabled:   true,
				Metadata: map[string]interface{}{
					"category": "cultural",
					"action":   "reduce_speed",
					"abu_dhabi_specific": true,
				},
			},
			{
				ID:        "border_approach",
				Name:      "UAE Border Approach",
				Condition: "approaching_border",
				Threshold: 5.0, // km from border
				Severity:  "medium",
				Enabled:   true,
				Metadata: map[string]interface{}{
					"category": "regulatory",
					"action":   "prepare_documentation",
					"abu_dhabi_specific": true,
				},
			},
		}
		
		defaultRules = append(defaultRules, abuDhabiRules...)
	}
	
	am.rulesMutex.Lock()
	am.rules = defaultRules
	am.rulesMutex.Unlock()
	
	log.Printf("✅ Initialized %d alert rules (%d Abu Dhabi specific)", len(defaultRules), len(defaultRules)-8)
}

func (am *AlertManager) initializeNotificationChannels() {
	channels := map[string]NotificationChannel{
		"fleet_operations": {
			ID:      "fleet_operations",
			Type:    "webhook",
			Enabled: true,
			Config: map[string]interface{}{
				"url": "http://fleet-manager:8080/api/v1/alerts/webhook",
				"timeout": "30s",
			},
			RateLimit: 1 * time.Second,
		},
		"control_center": {
			ID:      "control_center",
			Type:    "websocket",
			Enabled: true,
			Config: map[string]interface{}{
				"broadcast": true,
			},
			RateLimit: 500 * time.Millisecond,
		},
		"maintenance_team": {
			ID:      "maintenance_team",
			Type:    "email",
			Enabled: true,
			Config: map[string]interface{}{
				"recipients": []string{"maintenance@atlasmesh.ae"},
				"smtp_server": "smtp.atlasmesh.ae",
			},
			RateLimit: 5 * time.Minute,
		},
		"emergency_services": {
			ID:      "emergency_services",
			Type:    "emergency_api",
			Enabled: am.service.config.AbuDhabiMode,
			Config: map[string]interface{}{
				"endpoint": "https://api.uae-emergency.gov.ae/v1/incidents",
				"auth_token": "UAE_EMERGENCY_API_TOKEN",
			},
			RateLimit: 30 * time.Second,
		},
		"adta_integration": {
			ID:      "adta_integration",
			Type:    "government_api",
			Enabled: am.service.config.AbuDhabiMode,
			Config: map[string]interface{}{
				"endpoint": "https://api.adta.gov.ae/v1/incidents",
				"auth_token": "ADTA_API_TOKEN",
			},
			RateLimit: 1 * time.Minute,
		},
	}
	
	am.notificationChannels = channels
	log.Printf("✅ Initialized %d notification channels", len(channels))
}

func (am *AlertManager) initializeEscalationRules() {
	escalationRules := map[string]EscalationRule{
		"emergency": {
			ID:              "emergency_escalation",
			AlertType:       "emergency",
			InitialDelay:    0 * time.Second,
			EscalationDelay: 30 * time.Second,
			MaxEscalations:  3,
			Channels:        []string{"emergency_services", "control_center", "fleet_operations"},
			AbuDhabiSpecific: am.service.config.AbuDhabiMode,
		},
		"critical": {
			ID:              "critical_escalation",
			AlertType:       "critical",
			InitialDelay:    1 * time.Minute,
			EscalationDelay: 5 * time.Minute,
			MaxEscalations:  2,
			Channels:        []string{"control_center", "fleet_operations", "maintenance_team"},
		},
		"high": {
			ID:              "high_escalation",
			AlertType:       "high",
			InitialDelay:    5 * time.Minute,
			EscalationDelay: 15 * time.Minute,
			MaxEscalations:  1,
			Channels:        []string{"fleet_operations", "maintenance_team"},
		},
	}
	
	am.escalationRules = escalationRules
	log.Printf("✅ Initialized %d escalation rules", len(escalationRules))
}

func (am *AlertManager) EvaluateAlerts(telemetry VehicleTelemetry) []Alert {
	am.rulesMutex.RLock()
	defer am.rulesMutex.RUnlock()
	
	alerts := []Alert{}
	
	for _, rule := range am.rules {
		if !rule.Enabled {
			continue
		}
		
		evaluation := am.evaluateRule(rule, telemetry)
		if evaluation.Triggered {
			alert := am.createAlert(evaluation)
			alerts = append(alerts, alert)
		}
	}
	
	return alerts
}

func (am *AlertManager) evaluateRule(rule AlertRule, telemetry VehicleTelemetry) AlertEvaluation {
	evaluation := AlertEvaluation{
		Rule:      rule,
		Telemetry: telemetry,
		Triggered: false,
		Threshold: rule.Threshold,
		Context:   make(map[string]interface{}),
	}
	
	switch rule.ID {
	case "battery_low", "battery_critical":
		if telemetry.BatteryLevel > 0 && telemetry.BatteryLevel < rule.Threshold {
			evaluation.Triggered = true
			evaluation.Value = telemetry.BatteryLevel
			evaluation.Context["remaining_range"] = estimateRemainingRange(telemetry.BatteryLevel, "battery")
		}
		
	case "fuel_low":
		if telemetry.FuelLevel > 0 && telemetry.FuelLevel < rule.Threshold {
			evaluation.Triggered = true
			evaluation.Value = telemetry.FuelLevel
			evaluation.Context["remaining_range"] = estimateRemainingRange(telemetry.FuelLevel, "fuel")
		}
		
	case "speed_limit_violation":
		if telemetry.Speed > rule.Threshold {
			evaluation.Triggered = true
			evaluation.Value = telemetry.Speed
			evaluation.Context["speed_over_limit"] = telemetry.Speed - rule.Threshold
			evaluation.Context["location"] = telemetry.Location
		}
		
	case "connectivity_lost":
		minutesSinceLastSeen := time.Since(telemetry.Timestamp).Minutes()
		if minutesSinceLastSeen > rule.Threshold {
			evaluation.Triggered = true
			evaluation.Value = minutesSinceLastSeen
			evaluation.Context["last_known_location"] = telemetry.Location
		}
		
	case "geofence_violation":
		if am.isOutsideAuthorizedArea(telemetry) {
			evaluation.Triggered = true
			evaluation.Value = 1.0
			evaluation.Context["unauthorized_location"] = telemetry.Location
			evaluation.Context["distance_from_boundary"] = am.calculateDistanceFromBoundary(telemetry.Location)
		}
		
	case "temperature_high":
		if temp := am.extractTemperature(telemetry); temp > rule.Threshold {
			evaluation.Triggered = true
			evaluation.Value = temp
			evaluation.Context["cooling_status"] = telemetry.SystemStatus["cooling_system"]
		}
		
	case "emergency_stop":
		if telemetry.SystemStatus["emergency_stop"] == true {
			evaluation.Triggered = true
			evaluation.Value = 1.0
			evaluation.Context["emergency_reason"] = telemetry.SystemStatus["emergency_reason"]
		}
		
	// Abu Dhabi specific evaluations
	case "dust_storm_exposure":
		if am.service.config.AbuDhabiMode {
			visibility := am.extractVisibility(telemetry)
			dustLevel := am.extractDustLevel(telemetry)
			if visibility < rule.Threshold && dustLevel > 0.5 {
				evaluation.Triggered = true
				evaluation.Value = visibility
				evaluation.Context["dust_level"] = dustLevel
				evaluation.Context["weather_advisory"] = "dust_storm"
			}
		}
		
	case "extreme_heat_operation":
		if am.service.config.AbuDhabiMode {
			ambientTemp := am.extractAmbientTemperature(telemetry)
			coolingActive := telemetry.SystemStatus["cooling_system"] == "active"
			if ambientTemp > rule.Threshold && !coolingActive {
				evaluation.Triggered = true
				evaluation.Value = ambientTemp
				evaluation.Context["cooling_required"] = true
			}
		}
		
	case "prayer_time_proximity":
		if am.service.config.AbuDhabiMode {
			if telemetry.SystemStatus["prayer_time_proximity"] == true && telemetry.Speed > rule.Threshold {
				evaluation.Triggered = true
				evaluation.Value = telemetry.Speed
				evaluation.Context["prayer_time"] = true
				evaluation.Context["cultural_consideration"] = true
			}
		}
		
	case "border_approach":
		if am.service.config.AbuDhabiMode {
			distanceToBorder := am.calculateDistanceToBorder(telemetry.Location)
			if distanceToBorder < rule.Threshold {
				evaluation.Triggered = true
				evaluation.Value = distanceToBorder
				evaluation.Context["border_crossing_prep"] = true
				evaluation.Context["documentation_required"] = true
			}
		}
	}
	
	return evaluation
}

func (am *AlertManager) createAlert(evaluation AlertEvaluation) Alert {
	alert := Alert{
		ID:        fmt.Sprintf("%s_%s_%d", evaluation.Rule.ID, evaluation.Telemetry.VehicleID, time.Now().UnixNano()),
		Type:      evaluation.Rule.ID,
		Severity:  evaluation.Rule.Severity,
		Message:   am.generateAlertMessage(evaluation),
		Timestamp: time.Now(),
		VehicleID: evaluation.Telemetry.VehicleID,
		Location:  evaluation.Telemetry.Location,
		Metadata: map[string]interface{}{
			"rule_id":     evaluation.Rule.ID,
			"rule_name":   evaluation.Rule.Name,
			"threshold":   evaluation.Threshold,
			"actual_value": evaluation.Value,
			"context":     evaluation.Context,
			"telemetry_timestamp": evaluation.Telemetry.Timestamp,
		},
		Acknowledged: false,
	}
	
	// Add Abu Dhabi specific metadata
	if am.service.config.AbuDhabiMode {
		alert.Metadata["region"] = "abu_dhabi"
		alert.Metadata["local_time"] = time.Now().In(time.FixedZone("GST", 4*3600))
		
		if evaluation.Rule.Metadata["abu_dhabi_specific"] == true {
			alert.Metadata["cultural_context"] = true
		}
	}
	
	return alert
}

func (am *AlertManager) generateAlertMessage(evaluation AlertEvaluation) string {
	rule := evaluation.Rule
	
	switch rule.ID {
	case "battery_low":
		return fmt.Sprintf("Battery level is low (%.1f%%) - charging recommended", evaluation.Value)
	case "battery_critical":
		return fmt.Sprintf("CRITICAL: Battery level is critically low (%.1f%%) - immediate charging required", evaluation.Value)
	case "fuel_low":
		return fmt.Sprintf("Fuel level is low (%.1f%%) - refueling recommended", evaluation.Value)
	case "speed_limit_violation":
		return fmt.Sprintf("Speed limit violation detected (%.1f km/h) - reduce speed immediately", evaluation.Value)
	case "connectivity_lost":
		return fmt.Sprintf("Vehicle connectivity lost for %.1f minutes - investigating connection", evaluation.Value)
	case "geofence_violation":
		return fmt.Sprintf("Vehicle has left authorized operational area - immediate investigation required")
	case "temperature_high":
		return fmt.Sprintf("High operating temperature detected (%.1f°C) - cooling system activation required", evaluation.Value)
	case "emergency_stop":
		return fmt.Sprintf("EMERGENCY: Emergency stop has been activated - immediate response required")
	case "dust_storm_exposure":
		return fmt.Sprintf("Vehicle exposed to dust storm conditions (visibility: %.0fm) - seek shelter", evaluation.Value)
	case "extreme_heat_operation":
		return fmt.Sprintf("Operating in extreme heat (%.1f°C) without cooling - activate cooling system", evaluation.Value)
	case "prayer_time_proximity":
		return fmt.Sprintf("High speed operation during prayer time proximity - consider reducing speed for cultural respect")
	case "border_approach":
		return fmt.Sprintf("Approaching UAE border (%.1fkm) - prepare documentation and compliance checks", evaluation.Value)
	default:
		return fmt.Sprintf("%s: Threshold exceeded (%.2f > %.2f)", rule.Name, evaluation.Value, evaluation.Threshold)
	}
}

func (am *AlertManager) processAlerts() {
	ticker := time.NewTicker(10 * time.Second)
	defer ticker.Stop()
	
	for am.running {
		select {
		case <-ticker.C:
			am.processEscalations()
		case <-am.stopChan:
			return
		}
	}
}

func (am *AlertManager) processEscalations() {
	// Check for alerts that need escalation
	ctx := context.Background()
	keys, err := am.service.redis.Keys(ctx, "alert:*").Result()
	if err != nil {
		return
	}
	
	for _, key := range keys {
		alertData, err := am.service.redis.Get(ctx, key).Result()
		if err != nil {
			continue
		}
		
		var alert Alert
		if err := json.Unmarshal([]byte(alertData), &alert); err != nil {
			continue
		}
		
		// Check if alert needs escalation
		if am.shouldEscalate(alert) {
			am.escalateAlert(alert)
		}
	}
}

func (am *AlertManager) shouldEscalate(alert Alert) bool {
	escalationRule, exists := am.escalationRules[alert.Severity]
	if !exists {
		return false
	}
	
	// Check if enough time has passed for escalation
	timeSinceAlert := time.Since(alert.Timestamp)
	return timeSinceAlert > escalationRule.InitialDelay && !alert.Acknowledged
}

func (am *AlertManager) escalateAlert(alert Alert) {
	escalationRule := am.escalationRules[alert.Severity]
	
	for _, channelID := range escalationRule.Channels {
		channel, exists := am.notificationChannels[channelID]
		if !exists || !channel.Enabled {
			continue
		}
		
		// Check rate limiting
		if time.Since(channel.LastUsed) < channel.RateLimit {
			continue
		}
		
		// Send notification
		am.sendNotification(channel, alert)
		
		// Update last used time
		channel.LastUsed = time.Now()
		am.notificationChannels[channelID] = channel
	}
	
	log.Printf("🚨 Escalated alert: %s (%s) for vehicle %s", alert.Type, alert.Severity, alert.VehicleID)
}

func (am *AlertManager) sendNotification(channel NotificationChannel, alert Alert) {
	switch channel.Type {
	case "webhook":
		am.sendWebhookNotification(channel, alert)
	case "websocket":
		am.sendWebSocketNotification(channel, alert)
	case "email":
		am.sendEmailNotification(channel, alert)
	case "emergency_api":
		am.sendEmergencyAPINotification(channel, alert)
	case "government_api":
		am.sendGovernmentAPINotification(channel, alert)
	default:
		log.Printf("❌ Unknown notification channel type: %s", channel.Type)
	}
}

func (am *AlertManager) sendWebhookNotification(channel NotificationChannel, alert Alert) {
	// Implementation for webhook notifications
	log.Printf("📡 Sending webhook notification for alert: %s", alert.ID)
}

func (am *AlertManager) sendWebSocketNotification(channel NotificationChannel, alert Alert) {
	// Broadcast to WebSocket clients
	am.service.broadcastToWebSocketClients("alert_escalation", alert)
	log.Printf("📡 Sent WebSocket notification for alert: %s", alert.ID)
}

func (am *AlertManager) sendEmailNotification(channel NotificationChannel, alert Alert) {
	// Implementation for email notifications
	log.Printf("📧 Sending email notification for alert: %s", alert.ID)
}

func (am *AlertManager) sendEmergencyAPINotification(channel NotificationChannel, alert Alert) {
	// Send to UAE Emergency Services API
	if am.service.config.AbuDhabiMode && alert.Severity == "emergency" {
		log.Printf("🚨 Notifying UAE Emergency Services for alert: %s", alert.ID)
		
		notification := map[string]interface{}{
			"incident_type": "autonomous_vehicle_emergency",
			"alert_id":     alert.ID,
			"vehicle_id":   alert.VehicleID,
			"location": map[string]float64{
				"latitude":  alert.Location.Latitude,
				"longitude": alert.Location.Longitude,
			},
			"severity":     alert.Severity,
			"description":  alert.Message,
			"timestamp":    alert.Timestamp,
			"operator":     "AtlasMesh Fleet Operations",
			"contact":      "+971-2-XXX-XXXX",
		}
		
		// Store for audit trail
		notificationJSON, _ := json.Marshal(notification)
		am.service.redis.Set(context.Background(), 
			fmt.Sprintf("emergency_notification:%s", alert.ID), 
			notificationJSON, 7*24*time.Hour)
	}
}

func (am *AlertManager) sendGovernmentAPINotification(channel NotificationChannel, alert Alert) {
	// Send to ADTA (Abu Dhabi Transport Authority)
	if am.service.config.AbuDhabiMode {
		log.Printf("📡 Notifying ADTA for alert: %s", alert.ID)
		
		report := map[string]interface{}{
			"report_type": "autonomous_vehicle_incident",
			"alert_id":    alert.ID,
			"vehicle_id":  alert.VehicleID,
			"location": map[string]float64{
				"latitude":  alert.Location.Latitude,
				"longitude": alert.Location.Longitude,
			},
			"incident_type": alert.Type,
			"severity":      alert.Severity,
			"description":   alert.Message,
			"timestamp":     alert.Timestamp,
			"operator":      "AtlasMesh",
			"license_number": "AV-LICENSE-2024-001",
			"compliance":    "UAE_AV_Regulations_2024",
		}
		
		// Store for compliance audit
		reportJSON, _ := json.Marshal(report)
		am.service.redis.Set(context.Background(), 
			fmt.Sprintf("adta_report:%s", alert.ID), 
			reportJSON, 30*24*time.Hour)
	}
}

func (am *AlertManager) monitorEscalations() {
	ticker := time.NewTicker(1 * time.Minute)
	defer ticker.Stop()
	
	for am.running {
		select {
		case <-ticker.C:
			am.checkEscalationTimeouts()
		case <-am.stopChan:
			return
		}
	}
}

func (am *AlertManager) checkEscalationTimeouts() {
	// Implementation for checking escalation timeouts
	// This would monitor active alerts and ensure proper escalation
}

func (am *AlertManager) cleanupAlertHistory() {
	ticker := time.NewTicker(1 * time.Hour)
	defer ticker.Stop()
	
	for am.running {
		select {
		case <-ticker.C:
			am.performHistoryCleanup()
		case <-am.stopChan:
			return
		}
	}
}

func (am *AlertManager) performHistoryCleanup() {
	am.historyMutex.Lock()
	defer am.historyMutex.Unlock()
	
	cutoff := time.Now().Add(-24 * time.Hour)
	
	for vehicleID, alerts := range am.alertHistory {
		validAlerts := []Alert{}
		for _, alert := range alerts {
			if alert.Timestamp.After(cutoff) {
				validAlerts = append(validAlerts, alert)
			}
		}
		
		if len(validAlerts) == 0 {
			delete(am.alertHistory, vehicleID)
		} else {
			am.alertHistory[vehicleID] = validAlerts
		}
	}
}

// Utility functions for alert evaluation
func (am *AlertManager) isOutsideAuthorizedArea(telemetry VehicleTelemetry) bool {
	// Simplified geofence check - in production, this would use proper geofencing
	// For Abu Dhabi, check if outside emirate boundaries
	if am.service.config.AbuDhabiMode {
		return !isWithinAbuDhabi(telemetry.Location)
	}
	
	// Default authorized area (simplified)
	return telemetry.Location.Latitude < 20.0 || telemetry.Location.Latitude > 30.0 ||
		   telemetry.Location.Longitude < 50.0 || telemetry.Location.Longitude > 60.0
}

func (am *AlertManager) calculateDistanceFromBoundary(location Location) float64 {
	// Simplified boundary distance calculation
	// In production, this would use proper geospatial calculations
	return 0.0
}

func (am *AlertManager) calculateDistanceToBorder(location Location) float64 {
	// Calculate distance to UAE borders
	// Simplified calculation - in production would use proper border data
	
	// UAE approximate boundaries
	uaeBounds := struct {
		North, South, East, West float64
	}{
		North: 26.0, South: 22.5, East: 56.5, West: 51.0,
	}
	
	distToNorth := math.Abs(location.Latitude - uaeBounds.North) * 111.0 // Rough km per degree
	distToSouth := math.Abs(location.Latitude - uaeBounds.South) * 111.0
	distToEast := math.Abs(location.Longitude - uaeBounds.East) * 111.0
	distToWest := math.Abs(location.Longitude - uaeBounds.West) * 111.0
	
	return math.Min(math.Min(distToNorth, distToSouth), math.Min(distToEast, distToWest))
}

func (am *AlertManager) extractTemperature(telemetry VehicleTelemetry) float64 {
	if temp, exists := telemetry.SystemStatus["temperature"]; exists {
		if tempFloat, ok := temp.(float64); ok {
			return tempFloat
		}
	}
	if temp, exists := telemetry.SensorData["temperature"]; exists {
		if tempFloat, ok := temp.(float64); ok {
			return tempFloat
		}
	}
	return 0.0
}

func (am *AlertManager) extractVisibility(telemetry VehicleTelemetry) float64 {
	if vis, exists := telemetry.SensorData["visibility"]; exists {
		if visFloat, ok := vis.(float64); ok {
			return visFloat
		}
	}
	return 10000.0 // Default good visibility
}

func (am *AlertManager) extractDustLevel(telemetry VehicleTelemetry) float64 {
	if dust, exists := telemetry.SensorData["dust_level"]; exists {
		if dustFloat, ok := dust.(float64); ok {
			return dustFloat
		}
	}
	return 0.0 // Default no dust
}

func (am *AlertManager) extractAmbientTemperature(telemetry VehicleTelemetry) float64 {
	if temp, exists := telemetry.SensorData["ambient_temperature"]; exists {
		if tempFloat, ok := temp.(float64); ok {
			return tempFloat
		}
	}
	return 25.0 // Default moderate temperature
}

func estimateRemainingRange(level float64, powerType string) float64 {
	// Simplified range estimation
	switch powerType {
	case "battery":
		return level * 5.0 // 5km per % battery (simplified)
	case "fuel":
		return level * 8.0 // 8km per % fuel (simplified)
	default:
		return 0.0
	}
}
