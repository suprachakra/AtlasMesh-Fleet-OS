package main

import (
	"context"
	"crypto"
	"crypto/rand"
	"crypto/rsa"
	"crypto/sha256"
	"crypto/x509"
	"encoding/json"
	"encoding/pem"
	"fmt"
	"log"
	"net/http"
	"os"
	"os/signal"
	"sync"
	"syscall"
	"time"

	"github.com/gorilla/mux"
	"github.com/prometheus/client_golang/prometheus"
	"github.com/prometheus/client_golang/prometheus/promhttp"
	"go.opentelemetry.io/otel"
	"go.opentelemetry.io/otel/attribute"
	"go.opentelemetry.io/otel/trace"
)

// AuditabilityService manages cryptographically signed decision logs and retention
type AuditabilityService struct {
	decisionLogger    *DecisionLogger
	auditTrail        *AuditTrail
	retentionManager  *RetentionManager
	cryptoManager     *CryptoManager
	integrityChecker  *IntegrityChecker
	complianceEngine  *ComplianceEngine
	metrics           *AuditMetrics
	tracer            trace.Tracer
	config            *Config
}

type Config struct {
	Port              int                   `json:"port"`
	MetricsPort       int                   `json:"metrics_port"`
	LoggerConfig      LoggerConfig          `json:"logger"`
	TrailConfig       TrailConfig           `json:"trail"`
	RetentionConfig   RetentionConfig       `json:"retention"`
	CryptoConfig      CryptoConfig          `json:"crypto"`
	IntegrityConfig   IntegrityConfig       `json:"integrity"`
	ComplianceConfig  ComplianceConfig      `json:"compliance"`
}

type LoggerConfig struct {
	Format            string                `json:"format"`
	Level             string                `json:"level"`
	Destinations      []LogDestination      `json:"destinations"`
	Buffering         BufferingConfig       `json:"buffering"`
	Signing           SigningConfig         `json:"signing"`
	Encryption        EncryptionConfig      `json:"encryption"`
	Compression       bool                  `json:"compression"`
	Structured        bool                  `json:"structured"`
}

type TrailConfig struct {
	Enabled           bool                  `json:"enabled"`
	Immutable         bool                  `json:"immutable"`
	Chaining          bool                  `json:"chaining"`
	Timestamping      TimestampConfig       `json:"timestamping"`
	Verification      VerificationConfig    `json:"verification"`
	Replication       ReplicationConfig     `json:"replication"`
}

type RetentionConfig struct {
	Policies          []RetentionPolicy     `json:"policies"`
	Automation        AutomationConfig      `json:"automation"`
	Archive           ArchiveConfig         `json:"archive"`
	Deletion          DeletionConfig        `json:"deletion"`
	Compliance        []ComplianceRule      `json:"compliance"`
	Monitoring        MonitoringConfig      `json:"monitoring"`
}

type CryptoConfig struct {
	SigningKey        KeyConfig             `json:"signing_key"`
	EncryptionKey     KeyConfig             `json:"encryption_key"`
	HashAlgorithm     string                `json:"hash_algorithm"`
	SignatureAlgorithm string               `json:"signature_algorithm"`
	KeyRotation       KeyRotationConfig     `json:"key_rotation"`
	HSM               HSMConfig             `json:"hsm"`
}

// Core types
type DecisionLog struct {
	ID                string                `json:"id"`
	Timestamp         time.Time             `json:"timestamp"`
	Service           string                `json:"service"`
	Operation         string                `json:"operation"`
	Decision          Decision              `json:"decision"`
	Context           DecisionContext       `json:"context"`
	Input             interface{}           `json:"input"`
	Output            interface{}           `json:"output"`
	Metadata          LogMetadata           `json:"metadata"`
	Trace             TraceInfo             `json:"trace"`
	Hash              string                `json:"hash"`
	Signature         string                `json:"signature"`
	PreviousHash      string                `json:"previous_hash,omitempty"`
	ChainPosition     int64                 `json:"chain_position"`
	Verified          bool                  `json:"verified"`
	RetentionPolicy   string                `json:"retention_policy"`
	ExpiresAt         *time.Time            `json:"expires_at,omitempty"`
}

type Decision struct {
	Type              string                `json:"type"`
	Result            string                `json:"result"`
	Confidence        float64               `json:"confidence"`
	Reasoning         []ReasoningStep       `json:"reasoning"`
	Policies          []PolicyReference     `json:"policies"`
	Rules             []RuleReference       `json:"rules"`
	Factors           []DecisionFactor      `json:"factors"`
	Alternatives      []Alternative         `json:"alternatives"`
	RiskAssessment    RiskAssessment        `json:"risk_assessment"`
	Compliance        ComplianceCheck       `json:"compliance"`
}

type AuditEvent struct {
	ID                string                `json:"id"`
	Type              string                `json:"type"`
	Category          string                `json:"category"`
	Severity          string                `json:"severity"`
	Timestamp         time.Time             `json:"timestamp"`
	Actor             Actor                 `json:"actor"`
	Action            string                `json:"action"`
	Resource          Resource              `json:"resource"`
	Result            string                `json:"result"`
	Details           EventDetails          `json:"details"`
	Context           AuditContext          `json:"context"`
	Tags              map[string]string     `json:"tags"`
	Hash              string                `json:"hash"`
	Signature         string                `json:"signature"`
	ChainHash         string                `json:"chain_hash"`
	Verified          bool                  `json:"verified"`
}

type RetentionRecord struct {
	ID                string                `json:"id"`
	ResourceType      string                `json:"resource_type"`
	ResourceID        string                `json:"resource_id"`
	Policy            string                `json:"policy"`
	CreatedAt         time.Time             `json:"created_at"`
	RetentionPeriod   time.Duration         `json:"retention_period"`
	ExpiresAt         time.Time             `json:"expires_at"`
	Status            string                `json:"status"`
	Actions           []RetentionAction     `json:"actions"`
	Metadata          RetentionMetadata     `json:"metadata"`
	LastChecked       time.Time             `json:"last_checked"`
	NextCheck         time.Time             `json:"next_check"`
}

type IntegrityReport struct {
	ID                string                `json:"id"`
	Type              string                `json:"type"`
	Scope             IntegrityScope        `json:"scope"`
	Period            TimePeriod            `json:"period"`
	Status            string                `json:"status"`
	Summary           IntegritySummary      `json:"summary"`
	Violations        []IntegrityViolation  `json:"violations"`
	Recommendations   []string              `json:"recommendations"`
	GeneratedAt       time.Time             `json:"generated_at"`
	GeneratedBy       string                `json:"generated_by"`
	Hash              string                `json:"hash"`
	Signature         string                `json:"signature"`
}

// Supporting types
type LogDestination struct {
	Type              string                `json:"type"`
	Endpoint          string                `json:"endpoint"`
	Format            string                `json:"format"`
	Credentials       map[string]string     `json:"credentials"`
	Filters           []LogFilter           `json:"filters"`
	Enabled           bool                  `json:"enabled"`
}

type BufferingConfig struct {
	Enabled           bool                  `json:"enabled"`
	Size              int                   `json:"size"`
	FlushInterval     time.Duration         `json:"flush_interval"`
	FlushOnShutdown   bool                  `json:"flush_on_shutdown"`
}

type SigningConfig struct {
	Enabled           bool                  `json:"enabled"`
	Algorithm         string                `json:"algorithm"`
	KeyID             string                `json:"key_id"`
	Timestamping      bool                  `json:"timestamping"`
	ChainSigning      bool                  `json:"chain_signing"`
}

type EncryptionConfig struct {
	Enabled           bool                  `json:"enabled"`
	Algorithm         string                `json:"algorithm"`
	KeyID             string                `json:"key_id"`
	FieldLevel        bool                  `json:"field_level"`
	SensitiveFields   []string              `json:"sensitive_fields"`
}

type TimestampConfig struct {
	Enabled           bool                  `json:"enabled"`
	Authority         string                `json:"authority"`
	Protocol          string                `json:"protocol"`
	Verification      bool                  `json:"verification"`
}

type VerificationConfig struct {
	Enabled           bool                  `json:"enabled"`
	Frequency         time.Duration         `json:"frequency"`
	Scope             string                `json:"scope"`
	Alerts            bool                  `json:"alerts"`
}

type ReplicationConfig struct {
	Enabled           bool                  `json:"enabled"`
	Targets           []ReplicationTarget   `json:"targets"`
	Consistency       string                `json:"consistency"`
	Conflict          string                `json:"conflict"`
}

type RetentionPolicy struct {
	ID                string                `json:"id"`
	Name              string                `json:"name"`
	Description       string                `json:"description"`
	ResourceTypes     []string              `json:"resource_types"`
	Conditions        []RetentionCondition  `json:"conditions"`
	Actions           []PolicyAction        `json:"actions"`
	Period            time.Duration         `json:"period"`
	Priority          int                   `json:"priority"`
	Enabled           bool                  `json:"enabled"`
}

type AutomationConfig struct {
	Enabled           bool                  `json:"enabled"`
	Schedule          string                `json:"schedule"`
	BatchSize         int                   `json:"batch_size"`
	Concurrency       int                   `json:"concurrency"`
	DryRun            bool                  `json:"dry_run"`
	Notifications     []string              `json:"notifications"`
}

type ArchiveConfig struct {
	Enabled           bool                  `json:"enabled"`
	Storage           StorageConfig         `json:"storage"`
	Compression       string                `json:"compression"`
	Encryption        bool                  `json:"encryption"`
	Indexing          bool                  `json:"indexing"`
	Verification      bool                  `json:"verification"`
}

type DeletionConfig struct {
	Enabled           bool                  `json:"enabled"`
	SecureDelete      bool                  `json:"secure_delete"`
	Verification      bool                  `json:"verification"`
	Logging           bool                  `json:"logging"`
	Approval          bool                  `json:"approval"`
}

type ComplianceRule struct {
	Framework         string                `json:"framework"`
	Requirement       string                `json:"requirement"`
	MinRetention      time.Duration         `json:"min_retention"`
	MaxRetention      time.Duration         `json:"max_retention"`
	Actions           []string              `json:"actions"`
}

type MonitoringConfig struct {
	Enabled           bool                  `json:"enabled"`
	Metrics           []string              `json:"metrics"`
	Alerts            []AlertRule           `json:"alerts"`
	Dashboard         bool                  `json:"dashboard"`
}

type KeyConfig struct {
	Type              string                `json:"type"`
	Size              int                   `json:"size"`
	Algorithm         string                `json:"algorithm"`
	Source            string                `json:"source"`
	ID                string                `json:"id"`
}

type KeyRotationConfig struct {
	Enabled           bool                  `json:"enabled"`
	Frequency         time.Duration         `json:"frequency"`
	Overlap           time.Duration         `json:"overlap"`
	Notification      bool                  `json:"notification"`
}

type HSMConfig struct {
	Enabled           bool                  `json:"enabled"`
	Provider          string                `json:"provider"`
	Endpoint          string                `json:"endpoint"`
	Credentials       map[string]string     `json:"credentials"`
}

type DecisionContext struct {
	RequestID         string                `json:"request_id"`
	SessionID         string                `json:"session_id"`
	UserID            string                `json:"user_id"`
	VehicleID         string                `json:"vehicle_id,omitempty"`
	TripID            string                `json:"trip_id,omitempty"`
	Environment       string                `json:"environment"`
	Location          Location              `json:"location,omitempty"`
	Timestamp         time.Time             `json:"timestamp"`
	Metadata          map[string]interface{} `json:"metadata"`
}

type LogMetadata struct {
	Version           string                `json:"version"`
	Schema            string                `json:"schema"`
	Source            string                `json:"source"`
	Environment       string                `json:"environment"`
	Correlation       CorrelationInfo       `json:"correlation"`
	Classification    string                `json:"classification"`
	Sensitivity       string                `json:"sensitivity"`
}

type TraceInfo struct {
	TraceID           string                `json:"trace_id"`
	SpanID            string                `json:"span_id"`
	ParentSpanID      string                `json:"parent_span_id,omitempty"`
	Baggage           map[string]string     `json:"baggage,omitempty"`
}

type ReasoningStep struct {
	Step              int                   `json:"step"`
	Description       string                `json:"description"`
	Input             interface{}           `json:"input"`
	Output            interface{}           `json:"output"`
	Rule              string                `json:"rule,omitempty"`
	Confidence        float64               `json:"confidence"`
}

type PolicyReference struct {
	ID                string                `json:"id"`
	Name              string                `json:"name"`
	Version           string                `json:"version"`
	Applied           bool                  `json:"applied"`
	Result            string                `json:"result"`
}

type RuleReference struct {
	ID                string                `json:"id"`
	Name              string                `json:"name"`
	Condition         string                `json:"condition"`
	Action            string                `json:"action"`
	Matched           bool                  `json:"matched"`
}

type DecisionFactor struct {
	Name              string                `json:"name"`
	Value             interface{}           `json:"value"`
	Weight            float64               `json:"weight"`
	Impact            string                `json:"impact"`
	Source            string                `json:"source"`
}

type Alternative struct {
	Option            string                `json:"option"`
	Score             float64               `json:"score"`
	Pros              []string              `json:"pros"`
	Cons              []string              `json:"cons"`
	Risk              string                `json:"risk"`
}

type RiskAssessment struct {
	Level             string                `json:"level"`
	Score             float64               `json:"score"`
	Factors           []RiskFactor          `json:"factors"`
	Mitigation        []string              `json:"mitigation"`
	Residual          float64               `json:"residual"`
}

type ComplianceCheck struct {
	Framework         string                `json:"framework"`
	Requirements      []string              `json:"requirements"`
	Status            string                `json:"status"`
	Violations        []string              `json:"violations"`
	Evidence          []string              `json:"evidence"`
}

type Actor struct {
	Type              string                `json:"type"`
	ID                string                `json:"id"`
	Name              string                `json:"name"`
	Role              string                `json:"role"`
	IP                string                `json:"ip,omitempty"`
	UserAgent         string                `json:"user_agent,omitempty"`
}

type Resource struct {
	Type              string                `json:"type"`
	ID                string                `json:"id"`
	Name              string                `json:"name"`
	Path              string                `json:"path,omitempty"`
	Attributes        map[string]interface{} `json:"attributes,omitempty"`
}

type EventDetails struct {
	Method            string                `json:"method,omitempty"`
	Parameters        map[string]interface{} `json:"parameters,omitempty"`
	Response          interface{}           `json:"response,omitempty"`
	Error             string                `json:"error,omitempty"`
	Duration          time.Duration         `json:"duration,omitempty"`
	Size              int64                 `json:"size,omitempty"`
}

type AuditContext struct {
	Environment       string                `json:"environment"`
	Application       string                `json:"application"`
	Version           string                `json:"version"`
	Instance          string                `json:"instance"`
	Correlation       CorrelationInfo       `json:"correlation"`
	Security          SecurityContext       `json:"security"`
}

type RetentionAction struct {
	Type              string                `json:"type"`
	Status            string                `json:"status"`
	Timestamp         time.Time             `json:"timestamp"`
	Details           map[string]interface{} `json:"details"`
	Error             string                `json:"error,omitempty"`
}

type RetentionMetadata struct {
	Classification    string                `json:"classification"`
	LegalHold         bool                  `json:"legal_hold"`
	Compliance        []string              `json:"compliance"`
	Owner             string                `json:"owner"`
	Custodian         string                `json:"custodian"`
	Tags              map[string]string     `json:"tags"`
}

// Service components
type DecisionLogger struct {
	config      *LoggerConfig
	logs        map[string]*DecisionLog
	buffer      LogBuffer
	signer      LogSigner
	encryptor   LogEncryptor
	destinations map[string]LogDestination
	metrics     *AuditMetrics
	mu          sync.RWMutex
}

type AuditTrail struct {
	config      *TrailConfig
	events      map[string]*AuditEvent
	chain       EventChain
	timestamper Timestamper
	verifier    TrailVerifier
	replicator  TrailReplicator
	metrics     *AuditMetrics
	mu          sync.RWMutex
}

type RetentionManager struct {
	config      *RetentionConfig
	policies    map[string]*RetentionPolicy
	records     map[string]*RetentionRecord
	scheduler   RetentionScheduler
	executor    RetentionExecutor
	archiver    DataArchiver
	metrics     *AuditMetrics
	mu          sync.RWMutex
}

type CryptoManager struct {
	config      *CryptoConfig
	signingKey  *rsa.PrivateKey
	encryptionKey *rsa.PrivateKey
	keyRotator  KeyRotator
	hsm         HSMProvider
	metrics     *AuditMetrics
	mu          sync.RWMutex
}

type IntegrityChecker struct {
	config      *IntegrityConfig
	reports     map[string]*IntegrityReport
	checker     IntegrityValidator
	monitor     IntegrityMonitor
	metrics     *AuditMetrics
	mu          sync.RWMutex
}

type ComplianceEngine struct {
	config      *ComplianceConfig
	rules       map[string]*ComplianceRule
	checker     ComplianceChecker
	reporter    ComplianceReporter
	metrics     *AuditMetrics
	mu          sync.RWMutex
}

// AuditMetrics contains Prometheus metrics
type AuditMetrics struct {
	DecisionLogsCreated     *prometheus.CounterVec
	AuditEventsGenerated    *prometheus.CounterVec
	SignatureVerifications  *prometheus.CounterVec
	RetentionActions        *prometheus.CounterVec
	IntegrityChecks         *prometheus.CounterVec
	LoggingLatency          *prometheus.HistogramVec
	ChainLength             *prometheus.GaugeVec
	RetentionBacklog        *prometheus.GaugeVec
	IntegrityScore          *prometheus.GaugeVec
}

func main() {
	config := loadConfig()
	metrics := initializeMetrics()
	tracer := otel.Tracer("auditability-service")
	
	// Initialize crypto manager first
	cryptoManager := &CryptoManager{
		config: &config.CryptoConfig,
		metrics: metrics,
	}
	if err := cryptoManager.Initialize(); err != nil {
		log.Fatalf("Failed to initialize crypto manager: %v", err)
	}
	
	// Initialize components
	decisionLogger := &DecisionLogger{
		config:       &config.LoggerConfig,
		logs:         make(map[string]*DecisionLog),
		buffer:       NewLogBuffer(config.LoggerConfig.Buffering),
		signer:       NewLogSigner(cryptoManager),
		encryptor:    NewLogEncryptor(cryptoManager),
		destinations: make(map[string]LogDestination),
		metrics:      metrics,
	}
	
	auditTrail := &AuditTrail{
		config:      &config.TrailConfig,
		events:      make(map[string]*AuditEvent),
		chain:       NewEventChain(),
		timestamper: NewTimestamper(config.TrailConfig.Timestamping),
		verifier:    NewTrailVerifier(),
		replicator:  NewTrailReplicator(config.TrailConfig.Replication),
		metrics:     metrics,
	}
	
	retentionManager := &RetentionManager{
		config:    &config.RetentionConfig,
		policies:  make(map[string]*RetentionPolicy),
		records:   make(map[string]*RetentionRecord),
		scheduler: NewRetentionScheduler(),
		executor:  NewRetentionExecutor(),
		archiver:  NewDataArchiver(config.RetentionConfig.Archive),
		metrics:   metrics,
	}
	
	integrityChecker := &IntegrityChecker{
		config:  &config.IntegrityConfig,
		reports: make(map[string]*IntegrityReport),
		checker: NewIntegrityValidator(),
		monitor: NewIntegrityMonitor(),
		metrics: metrics,
	}
	
	complianceEngine := &ComplianceEngine{
		config:   &config.ComplianceConfig,
		rules:    make(map[string]*ComplianceRule),
		checker:  NewComplianceChecker(),
		reporter: NewComplianceReporter(),
		metrics:  metrics,
	}
	
	service := &AuditabilityService{
		decisionLogger:   decisionLogger,
		auditTrail:       auditTrail,
		retentionManager: retentionManager,
		cryptoManager:    cryptoManager,
		integrityChecker: integrityChecker,
		complianceEngine: complianceEngine,
		metrics:          metrics,
		tracer:           tracer,
		config:           config,
	}
	
	// Setup HTTP server
	router := mux.NewRouter()
	service.setupRoutes(router)
	
	server := &http.Server{
		Addr:    fmt.Sprintf(":%d", config.Port),
		Handler: router,
	}
	
	// Start background services
	go service.startRetentionAutomation()
	go service.startIntegrityMonitoring()
	go service.startComplianceChecking()
	go service.startLogBufferFlushing()
	
	// Start server
	go func() {
		log.Printf("Starting Auditability service on port %d", config.Port)
		if err := server.ListenAndServe(); err != nil && err != http.ErrServerClosed {
			log.Fatalf("Server failed: %v", err)
		}
	}()
	
	// Graceful shutdown
	c := make(chan os.Signal, 1)
	signal.Notify(c, os.Interrupt, syscall.SIGTERM)
	<-c
	
	ctx, cancel := context.WithTimeout(context.Background(), 30*time.Second)
	defer cancel()
	server.Shutdown(ctx)
}

func (s *AuditabilityService) setupRoutes(router *mux.Router) {
	api := router.PathPrefix("/api/v1").Subrouter()
	
	// Decision logs
	api.HandleFunc("/decisions", s.listDecisionLogs).Methods("GET")
	api.HandleFunc("/decisions", s.logDecision).Methods("POST")
	api.HandleFunc("/decisions/{logId}", s.getDecisionLog).Methods("GET")
	api.HandleFunc("/decisions/{logId}/verify", s.verifyDecisionLog).Methods("POST")
	
	// Audit events
	api.HandleFunc("/events", s.listAuditEvents).Methods("GET")
	api.HandleFunc("/events", s.createAuditEvent).Methods("POST")
	api.HandleFunc("/events/{eventId}", s.getAuditEvent).Methods("GET")
	api.HandleFunc("/events/{eventId}/verify", s.verifyAuditEvent).Methods("POST")
	
	// Retention management
	api.HandleFunc("/retention/policies", s.listRetentionPolicies).Methods("GET")
	api.HandleFunc("/retention/records", s.listRetentionRecords).Methods("GET")
	api.HandleFunc("/retention/execute", s.executeRetention).Methods("POST")
	
	// Integrity checking
	api.HandleFunc("/integrity/check", s.runIntegrityCheck).Methods("POST")
	api.HandleFunc("/integrity/reports", s.listIntegrityReports).Methods("GET")
	api.HandleFunc("/integrity/reports/{reportId}", s.getIntegrityReport).Methods("GET")
	
	// Compliance
	api.HandleFunc("/compliance/check", s.checkCompliance).Methods("POST")
	api.HandleFunc("/compliance/rules", s.listComplianceRules).Methods("GET")
	
	// Health
	api.HandleFunc("/health", s.healthCheck).Methods("GET")
	router.Handle("/metrics", promhttp.Handler())
}

func (s *AuditabilityService) logDecision(w http.ResponseWriter, r *http.Request) {
	ctx, span := s.tracer.Start(r.Context(), "log_decision")
	defer span.End()
	
	var request struct {
		Service     string          `json:"service"`
		Operation   string          `json:"operation"`
		Decision    Decision        `json:"decision"`
		Context     DecisionContext `json:"context"`
		Input       interface{}     `json:"input"`
		Output      interface{}     `json:"output"`
	}
	
	if err := json.NewDecoder(r.Body).Decode(&request); err != nil {
		http.Error(w, "Invalid request", http.StatusBadRequest)
		return
	}
	
	start := time.Now()
	decisionLog, err := s.decisionLogger.LogDecision(ctx, &request)
	if err != nil {
		s.metrics.DecisionLogsCreated.WithLabelValues("failed", request.Service).Inc()
		span.RecordError(err)
		http.Error(w, fmt.Sprintf("Failed to log decision: %v", err), http.StatusInternalServerError)
		return
	}
	
	duration := time.Since(start)
	s.metrics.DecisionLogsCreated.WithLabelValues("success", request.Service).Inc()
	s.metrics.LoggingLatency.WithLabelValues("decision").Observe(duration.Seconds())
	
	span.SetAttributes(
		attribute.String("log_id", decisionLog.ID),
		attribute.String("service", request.Service),
		attribute.String("operation", request.Operation),
		attribute.String("decision_result", request.Decision.Result),
		attribute.Float64("logging_time", duration.Seconds()),
	)
	
	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(decisionLog)
}

func (s *AuditabilityService) createAuditEvent(w http.ResponseWriter, r *http.Request) {
	ctx, span := s.tracer.Start(r.Context(), "create_audit_event")
	defer span.End()
	
	var request struct {
		Type        string            `json:"type"`
		Category    string            `json:"category"`
		Severity    string            `json:"severity"`
		Actor       Actor             `json:"actor"`
		Action      string            `json:"action"`
		Resource    Resource          `json:"resource"`
		Result      string            `json:"result"`
		Details     EventDetails      `json:"details"`
		Context     AuditContext      `json:"context"`
		Tags        map[string]string `json:"tags"`
	}
	
	if err := json.NewDecoder(r.Body).Decode(&request); err != nil {
		http.Error(w, "Invalid request", http.StatusBadRequest)
		return
	}
	
	start := time.Now()
	auditEvent, err := s.auditTrail.CreateEvent(ctx, &request)
	if err != nil {
		s.metrics.AuditEventsGenerated.WithLabelValues("failed", request.Type).Inc()
		span.RecordError(err)
		http.Error(w, fmt.Sprintf("Failed to create audit event: %v", err), http.StatusInternalServerError)
		return
	}
	
	duration := time.Since(start)
	s.metrics.AuditEventsGenerated.WithLabelValues("success", request.Type).Inc()
	s.metrics.LoggingLatency.WithLabelValues("audit_event").Observe(duration.Seconds())
	
	span.SetAttributes(
		attribute.String("event_id", auditEvent.ID),
		attribute.String("event_type", request.Type),
		attribute.String("category", request.Category),
		attribute.String("severity", request.Severity),
		attribute.String("action", request.Action),
		attribute.Float64("creation_time", duration.Seconds()),
	)
	
	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(auditEvent)
}

func (s *AuditabilityService) runIntegrityCheck(w http.ResponseWriter, r *http.Request) {
	ctx, span := s.tracer.Start(r.Context(), "run_integrity_check")
	defer span.End()
	
	var request struct {
		Type    string        `json:"type"`
		Scope   IntegrityScope `json:"scope"`
		Period  TimePeriod    `json:"period,omitempty"`
	}
	
	if err := json.NewDecoder(r.Body).Decode(&request); err != nil {
		http.Error(w, "Invalid request", http.StatusBadRequest)
		return
	}
	
	start := time.Now()
	report, err := s.integrityChecker.RunCheck(ctx, &request)
	if err != nil {
		s.metrics.IntegrityChecks.WithLabelValues("failed", request.Type).Inc()
		span.RecordError(err)
		http.Error(w, fmt.Sprintf("Failed to run integrity check: %v", err), http.StatusInternalServerError)
		return
	}
	
	duration := time.Since(start)
	s.metrics.IntegrityChecks.WithLabelValues("success", request.Type).Inc()
	
	// Update integrity score
	s.metrics.IntegrityScore.WithLabelValues(request.Type, "overall").Set(report.Summary.OverallScore)
	
	span.SetAttributes(
		attribute.String("report_id", report.ID),
		attribute.String("check_type", request.Type),
		attribute.String("status", report.Status),
		attribute.Float64("integrity_score", report.Summary.OverallScore),
		attribute.Float64("check_time", duration.Seconds()),
	)
	
	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(report)
}

func (s *AuditabilityService) startRetentionAutomation() {
	if !s.config.RetentionConfig.Automation.Enabled {
		return
	}
	
	ticker := time.NewTicker(1 * time.Hour)
	defer ticker.Stop()
	
	for {
		select {
		case <-ticker.C:
			s.processRetentionPolicies()
		}
	}
}

func (s *AuditabilityService) startIntegrityMonitoring() {
	ticker := time.NewTicker(30 * time.Minute)
	defer ticker.Stop()
	
	for {
		select {
		case <-ticker.C:
			s.performIntegrityChecks()
		}
	}
}

func (s *AuditabilityService) startComplianceChecking() {
	ticker := time.NewTicker(2 * time.Hour)
	defer ticker.Stop()
	
	for {
		select {
		case <-ticker.C:
			s.checkComplianceRules()
		}
	}
}

func (s *AuditabilityService) startLogBufferFlushing() {
	ticker := time.NewTicker(s.config.LoggerConfig.Buffering.FlushInterval)
	defer ticker.Stop()
	
	for {
		select {
		case <-ticker.C:
			s.flushLogBuffers()
		}
	}
}

func (s *AuditabilityService) processRetentionPolicies() {
	log.Println("Processing retention policies...")
	
	for policyID, policy := range s.retentionManager.policies {
		if policy.Enabled {
			actions, err := s.retentionManager.ExecutePolicy(context.Background(), policyID)
			if err != nil {
				log.Printf("Failed to execute retention policy %s: %v", policyID, err)
				s.metrics.RetentionActions.WithLabelValues("failed", "policy_execution").Inc()
			} else {
				s.metrics.RetentionActions.WithLabelValues("success", "policy_execution").Add(float64(len(actions)))
				log.Printf("Executed retention policy %s, performed %d actions", policy.Name, len(actions))
			}
		}
	}
	
	// Update backlog metric
	backlog := s.retentionManager.GetBacklogCount()
	s.metrics.RetentionBacklog.WithLabelValues("total").Set(float64(backlog))
}

func (s *AuditabilityService) performIntegrityChecks() {
	log.Println("Performing integrity checks...")
	
	// Check decision log integrity
	report, err := s.integrityChecker.CheckDecisionLogs(context.Background())
	if err != nil {
		log.Printf("Failed to check decision log integrity: %v", err)
		s.metrics.IntegrityChecks.WithLabelValues("failed", "decision_logs").Inc()
	} else {
		s.metrics.IntegrityChecks.WithLabelValues("success", "decision_logs").Inc()
		s.metrics.IntegrityScore.WithLabelValues("decision_logs", "overall").Set(report.Summary.OverallScore)
		
		if len(report.Violations) > 0 {
			log.Printf("Found %d integrity violations in decision logs", len(report.Violations))
		}
	}
	
	// Check audit trail integrity
	report, err = s.integrityChecker.CheckAuditTrail(context.Background())
	if err != nil {
		log.Printf("Failed to check audit trail integrity: %v", err)
		s.metrics.IntegrityChecks.WithLabelValues("failed", "audit_trail").Inc()
	} else {
		s.metrics.IntegrityChecks.WithLabelValues("success", "audit_trail").Inc()
		s.metrics.IntegrityScore.WithLabelValues("audit_trail", "overall").Set(report.Summary.OverallScore)
		
		// Update chain length metric
		chainLength := s.auditTrail.GetChainLength()
		s.metrics.ChainLength.WithLabelValues("audit_trail").Set(float64(chainLength))
	}
}

func (s *AuditabilityService) checkComplianceRules() {
	log.Println("Checking compliance rules...")
	
	for ruleID, rule := range s.complianceEngine.rules {
		violations, err := s.complianceEngine.CheckRule(context.Background(), ruleID)
		if err != nil {
			log.Printf("Failed to check compliance rule %s: %v", ruleID, err)
		} else {
			if len(violations) > 0 {
				log.Printf("Found %d violations for compliance rule %s (%s)", len(violations), rule.Framework, rule.Requirement)
			}
		}
	}
}

func (s *AuditabilityService) flushLogBuffers() {
	if s.config.LoggerConfig.Buffering.Enabled {
		err := s.decisionLogger.FlushBuffer()
		if err != nil {
			log.Printf("Failed to flush decision log buffer: %v", err)
		}
		
		err = s.auditTrail.FlushBuffer()
		if err != nil {
			log.Printf("Failed to flush audit trail buffer: %v", err)
		}
	}
}

// Helper functions and placeholder implementations
func loadConfig() *Config {
	return &Config{
		Port:        8080,
		MetricsPort: 9090,
		LoggerConfig: LoggerConfig{
			Format:      "json",
			Level:       "info",
			Structured:  true,
			Compression: true,
			Buffering: BufferingConfig{
				Enabled:       true,
				Size:          1000,
				FlushInterval: 30 * time.Second,
			},
			Signing: SigningConfig{
				Enabled:      true,
				Algorithm:    "RSA-PSS",
				Timestamping: true,
				ChainSigning: true,
			},
			Encryption: EncryptionConfig{
				Enabled:     true,
				Algorithm:   "AES-256-GCM",
				FieldLevel:  true,
			},
		},
		TrailConfig: TrailConfig{
			Enabled:   true,
			Immutable: true,
			Chaining:  true,
			Timestamping: TimestampConfig{
				Enabled: true,
				Authority: "rfc3161",
			},
			Verification: VerificationConfig{
				Enabled:   true,
				Frequency: 1 * time.Hour,
				Alerts:    true,
			},
		},
		RetentionConfig: RetentionConfig{
			Automation: AutomationConfig{
				Enabled:     true,
				Schedule:    "0 2 * * *", // Daily at 2 AM
				BatchSize:   100,
				Concurrency: 5,
			},
			Archive: ArchiveConfig{
				Enabled:     true,
				Compression: "gzip",
				Encryption:  true,
				Indexing:    true,
			},
			Deletion: DeletionConfig{
				Enabled:      true,
				SecureDelete: true,
				Verification: true,
				Logging:      true,
			},
		},
		CryptoConfig: CryptoConfig{
			HashAlgorithm:      "SHA-256",
			SignatureAlgorithm: "RSA-PSS",
			SigningKey: KeyConfig{
				Type:      "RSA",
				Size:      2048,
				Algorithm: "RSA-PSS",
			},
			EncryptionKey: KeyConfig{
				Type:      "RSA",
				Size:      2048,
				Algorithm: "RSA-OAEP",
			},
			KeyRotation: KeyRotationConfig{
				Enabled:   true,
				Frequency: 90 * 24 * time.Hour, // 90 days
				Overlap:   7 * 24 * time.Hour,  // 7 days
			},
		},
	}
}

func initializeMetrics() *AuditMetrics {
	metrics := &AuditMetrics{
		DecisionLogsCreated: prometheus.NewCounterVec(
			prometheus.CounterOpts{Name: "atlasmesh_decision_logs_created_total", Help: "Total decision logs created"},
			[]string{"status", "service"},
		),
		AuditEventsGenerated: prometheus.NewCounterVec(
			prometheus.CounterOpts{Name: "atlasmesh_audit_events_generated_total", Help: "Total audit events generated"},
			[]string{"status", "type"},
		),
		SignatureVerifications: prometheus.NewCounterVec(
			prometheus.CounterOpts{Name: "atlasmesh_signature_verifications_total", Help: "Total signature verifications"},
			[]string{"status", "type"},
		),
		RetentionActions: prometheus.NewCounterVec(
			prometheus.CounterOpts{Name: "atlasmesh_retention_actions_total", Help: "Total retention actions"},
			[]string{"status", "action_type"},
		),
		IntegrityChecks: prometheus.NewCounterVec(
			prometheus.CounterOpts{Name: "atlasmesh_integrity_checks_total", Help: "Total integrity checks"},
			[]string{"status", "check_type"},
		),
		LoggingLatency: prometheus.NewHistogramVec(
			prometheus.HistogramOpts{Name: "atlasmesh_logging_latency_seconds", Help: "Logging latency"},
			[]string{"log_type"},
		),
		ChainLength: prometheus.NewGaugeVec(
			prometheus.GaugeOpts{Name: "atlasmesh_chain_length", Help: "Audit chain length"},
			[]string{"chain_type"},
		),
		RetentionBacklog: prometheus.NewGaugeVec(
			prometheus.GaugeOpts{Name: "atlasmesh_retention_backlog", Help: "Retention backlog count"},
			[]string{"type"},
		),
		IntegrityScore: prometheus.NewGaugeVec(
			prometheus.GaugeOpts{Name: "atlasmesh_integrity_score", Help: "Integrity score"},
			[]string{"resource_type", "category"},
		),
	}
	
	prometheus.MustRegister(
		metrics.DecisionLogsCreated, metrics.AuditEventsGenerated, metrics.SignatureVerifications,
		metrics.RetentionActions, metrics.IntegrityChecks, metrics.LoggingLatency,
		metrics.ChainLength, metrics.RetentionBacklog, metrics.IntegrityScore,
	)
	
	return metrics
}

func (s *AuditabilityService) healthCheck(w http.ResponseWriter, r *http.Request) {
	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(map[string]string{"status": "healthy"})
}

// Crypto manager implementation
func (cm *CryptoManager) Initialize() error {
	// Generate signing key
	signingKey, err := rsa.GenerateKey(rand.Reader, cm.config.SigningKey.Size)
	if err != nil {
		return fmt.Errorf("failed to generate signing key: %v", err)
	}
	cm.signingKey = signingKey
	
	// Generate encryption key
	encryptionKey, err := rsa.GenerateKey(rand.Reader, cm.config.EncryptionKey.Size)
	if err != nil {
		return fmt.Errorf("failed to generate encryption key: %v", err)
	}
	cm.encryptionKey = encryptionKey
	
	return nil
}

func (cm *CryptoManager) SignData(data []byte) (string, error) {
	hash := sha256.Sum256(data)
	signature, err := rsa.SignPSS(rand.Reader, cm.signingKey, crypto.SHA256, hash[:], nil)
	if err != nil {
		return "", err
	}
	return fmt.Sprintf("%x", signature), nil
}

func (cm *CryptoManager) VerifySignature(data []byte, signature string) error {
	hash := sha256.Sum256(data)
	sigBytes := make([]byte, len(signature)/2)
	for i := 0; i < len(sigBytes); i++ {
		fmt.Sscanf(signature[i*2:i*2+2], "%02x", &sigBytes[i])
	}
	return rsa.VerifyPSS(&cm.signingKey.PublicKey, crypto.SHA256, hash[:], sigBytes, nil)
}

// Placeholder types and implementations
type IntegrityConfig struct{}
type ComplianceConfig struct{}
type LogFilter struct{}
type ReplicationTarget struct{}
type RetentionCondition struct{}
type PolicyAction struct{}
type AlertRule struct{}
type Location struct{}
type CorrelationInfo struct{}
type SecurityContext struct{}
type RiskFactor struct{}
type TimePeriod struct{}
type IntegrityScope struct{}
type IntegritySummary struct{ OverallScore float64 }
type IntegrityViolation struct{}

type LogBuffer interface{}
type LogSigner interface{}
type LogEncryptor interface{}
type EventChain interface{}
type Timestamper interface{}
type TrailVerifier interface{}
type TrailReplicator interface{}
type RetentionScheduler interface{}
type RetentionExecutor interface{}
type DataArchiver interface{}
type KeyRotator interface{}
type HSMProvider interface{}
type IntegrityValidator interface{}
type IntegrityMonitor interface{}
type ComplianceChecker interface{}
type ComplianceReporter interface{}

func NewLogBuffer(config BufferingConfig) LogBuffer { return nil }
func NewLogSigner(cm *CryptoManager) LogSigner { return nil }
func NewLogEncryptor(cm *CryptoManager) LogEncryptor { return nil }
func NewEventChain() EventChain { return nil }
func NewTimestamper(config TimestampConfig) Timestamper { return nil }
func NewTrailVerifier() TrailVerifier { return nil }
func NewTrailReplicator(config ReplicationConfig) TrailReplicator { return nil }
func NewRetentionScheduler() RetentionScheduler { return nil }
func NewRetentionExecutor() RetentionExecutor { return nil }
func NewDataArchiver(config ArchiveConfig) DataArchiver { return nil }
func NewIntegrityValidator() IntegrityValidator { return nil }
func NewIntegrityMonitor() IntegrityMonitor { return nil }
func NewComplianceChecker() ComplianceChecker { return nil }
func NewComplianceReporter() ComplianceReporter { return nil }

// Placeholder method implementations
func (dl *DecisionLogger) LogDecision(ctx context.Context, req interface{}) (*DecisionLog, error) {
	decisionLog := &DecisionLog{
		ID:            fmt.Sprintf("decision_%d", time.Now().Unix()),
		Timestamp:     time.Now(),
		ChainPosition: int64(len(dl.logs) + 1),
		Verified:      false,
		Hash:          fmt.Sprintf("%x", sha256.Sum256([]byte(fmt.Sprintf("decision_%d", time.Now().Unix())))),
	}
	dl.logs[decisionLog.ID] = decisionLog
	return decisionLog, nil
}

func (dl *DecisionLogger) FlushBuffer() error { return nil }

func (at *AuditTrail) CreateEvent(ctx context.Context, req interface{}) (*AuditEvent, error) {
	auditEvent := &AuditEvent{
		ID:        fmt.Sprintf("event_%d", time.Now().Unix()),
		Timestamp: time.Now(),
		Verified:  false,
		Hash:      fmt.Sprintf("%x", sha256.Sum256([]byte(fmt.Sprintf("event_%d", time.Now().Unix())))),
	}
	at.events[auditEvent.ID] = auditEvent
	return auditEvent, nil
}

func (at *AuditTrail) FlushBuffer() error { return nil }
func (at *AuditTrail) GetChainLength() int { return len(at.events) }

func (rm *RetentionManager) ExecutePolicy(ctx context.Context, policyID string) ([]RetentionAction, error) {
	return []RetentionAction{
		{Type: "archive", Status: "completed", Timestamp: time.Now()},
		{Type: "delete", Status: "completed", Timestamp: time.Now()},
	}, nil
}

func (rm *RetentionManager) GetBacklogCount() int { return len(rm.records) }

func (ic *IntegrityChecker) RunCheck(ctx context.Context, req interface{}) (*IntegrityReport, error) {
	report := &IntegrityReport{
		ID:          fmt.Sprintf("integrity_%d", time.Now().Unix()),
		Status:      "completed",
		Summary:     IntegritySummary{OverallScore: 0.95},
		GeneratedAt: time.Now(),
	}
	ic.reports[report.ID] = report
	return report, nil
}

func (ic *IntegrityChecker) CheckDecisionLogs(ctx context.Context) (*IntegrityReport, error) {
	return &IntegrityReport{
		ID:      fmt.Sprintf("decision_integrity_%d", time.Now().Unix()),
		Status:  "completed",
		Summary: IntegritySummary{OverallScore: 0.98},
	}, nil
}

func (ic *IntegrityChecker) CheckAuditTrail(ctx context.Context) (*IntegrityReport, error) {
	return &IntegrityReport{
		ID:      fmt.Sprintf("trail_integrity_%d", time.Now().Unix()),
		Status:  "completed",
		Summary: IntegritySummary{OverallScore: 0.97},
	}, nil
}

func (ce *ComplianceEngine) CheckRule(ctx context.Context, ruleID string) ([]interface{}, error) {
	return []interface{}{}, nil // No violations
}

// Placeholder handlers
func (s *AuditabilityService) listDecisionLogs(w http.ResponseWriter, r *http.Request) {
	http.Error(w, "Not implemented", http.StatusNotImplemented)
}
func (s *AuditabilityService) getDecisionLog(w http.ResponseWriter, r *http.Request) {
	http.Error(w, "Not implemented", http.StatusNotImplemented)
}
func (s *AuditabilityService) verifyDecisionLog(w http.ResponseWriter, r *http.Request) {
	http.Error(w, "Not implemented", http.StatusNotImplemented)
}
func (s *AuditabilityService) listAuditEvents(w http.ResponseWriter, r *http.Request) {
	http.Error(w, "Not implemented", http.StatusNotImplemented)
}
func (s *AuditabilityService) getAuditEvent(w http.ResponseWriter, r *http.Request) {
	http.Error(w, "Not implemented", http.StatusNotImplemented)
}
func (s *AuditabilityService) verifyAuditEvent(w http.ResponseWriter, r *http.Request) {
	http.Error(w, "Not implemented", http.StatusNotImplemented)
}
func (s *AuditabilityService) listRetentionPolicies(w http.ResponseWriter, r *http.Request) {
	http.Error(w, "Not implemented", http.StatusNotImplemented)
}
func (s *AuditabilityService) listRetentionRecords(w http.ResponseWriter, r *http.Request) {
	http.Error(w, "Not implemented", http.StatusNotImplemented)
}
func (s *AuditabilityService) executeRetention(w http.ResponseWriter, r *http.Request) {
	http.Error(w, "Not implemented", http.StatusNotImplemented)
}
func (s *AuditabilityService) listIntegrityReports(w http.ResponseWriter, r *http.Request) {
	http.Error(w, "Not implemented", http.StatusNotImplemented)
}
func (s *AuditabilityService) getIntegrityReport(w http.ResponseWriter, r *http.Request) {
	http.Error(w, "Not implemented", http.StatusNotImplemented)
}
func (s *AuditabilityService) checkCompliance(w http.ResponseWriter, r *http.Request) {
	http.Error(w, "Not implemented", http.StatusNotImplemented)
}
func (s *AuditabilityService) listComplianceRules(w http.ResponseWriter, r *http.Request) {
	http.Error(w, "Not implemented", http.StatusNotImplemented)
}
