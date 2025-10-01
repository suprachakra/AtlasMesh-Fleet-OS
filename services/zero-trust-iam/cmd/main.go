package main

import (
	"context"
	"crypto/tls"
	"crypto/x509"
	"encoding/json"
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
	"google.golang.org/grpc"
	"google.golang.org/grpc/credentials"
)

// ZeroTrustIAMService manages identity, authentication, and authorization
type ZeroTrustIAMService struct {
	spiffeManager    *SPIFFEManager
	spireAgent       *SPIREAgent
	mtlsManager      *mTLSManager
	policyEngine     *PolicyEngine
	identityRegistry *IdentityRegistry
	auditLogger      *AuditLogger
	metrics          *IAMMetrics
	tracer           trace.Tracer
	config           *Config
}

// Config holds the service configuration
type Config struct {
	Port                    int                     `json:"port"`
	MetricsPort             int                     `json:"metrics_port"`
	SPIFFEConfig            SPIFFEConfig            `json:"spiffe"`
	SPIREConfig             SPIREConfig             `json:"spire"`
	mTLSConfig              mTLSConfig              `json:"mtls"`
	PolicyConfig            PolicyConfig            `json:"policy"`
	IdentityConfig          IdentityConfig          `json:"identity"`
	AuditConfig             AuditConfig             `json:"audit"`
	TrustDomains            []TrustDomain           `json:"trust_domains"`
	CertificateRotation     CertificateRotation     `json:"certificate_rotation"`
	SecurityPolicies        []SecurityPolicy        `json:"security_policies"`
}

// SPIFFEConfig configures SPIFFE identity framework
type SPIFFEConfig struct {
	TrustDomain         string        `json:"trust_domain"`
	SocketPath          string        `json:"socket_path"`
	BundleEndpoint      string        `json:"bundle_endpoint"`
	WorkloadAPITimeout  time.Duration `json:"workload_api_timeout"`
	SVIDRefreshInterval time.Duration `json:"svid_refresh_interval"`
	BundleRefreshInterval time.Duration `json:"bundle_refresh_interval"`
	FederatedTrustDomains []string    `json:"federated_trust_domains"`
}

// SPIREConfig configures SPIRE server and agent
type SPIREConfig struct {
	ServerAddress       string            `json:"server_address"`
	AgentSocketPath     string            `json:"agent_socket_path"`
	DataDir             string            `json:"data_dir"`
	LogLevel            string            `json:"log_level"`
	UpstreamAuthority   string            `json:"upstream_authority"`
	NodeAttestors       []NodeAttestor    `json:"node_attestors"`
	WorkloadAttestors   []WorkloadAttestor `json:"workload_attestors"`
	KeyManager          KeyManager        `json:"key_manager"`
	HealthCheckInterval time.Duration     `json:"health_check_interval"`
}

// mTLSConfig configures mutual TLS
type mTLSConfig struct {
	Enabled                bool          `json:"enabled"`
	CertificateAuthority   string        `json:"certificate_authority"`
	ClientCertRequired     bool          `json:"client_cert_required"`
	VerifyClientCert       bool          `json:"verify_client_cert"`
	CipherSuites           []string      `json:"cipher_suites"`
	MinTLSVersion          string        `json:"min_tls_version"`
	MaxTLSVersion          string        `json:"max_tls_version"`
	CertificateValidation  CertValidation `json:"certificate_validation"`
	RevocationChecking     bool          `json:"revocation_checking"`
	OCSPStapling          bool          `json:"ocsp_stapling"`
}

// PolicyConfig configures policy engine
type PolicyConfig struct {
	PolicyStore         string        `json:"policy_store"`
	PolicyFormat        string        `json:"policy_format"`
	EvaluationTimeout   time.Duration `json:"evaluation_timeout"`
	CacheEnabled        bool          `json:"cache_enabled"`
	CacheTTL            time.Duration `json:"cache_ttl"`
	PolicyUpdateInterval time.Duration `json:"policy_update_interval"`
	DefaultDeny         bool          `json:"default_deny"`
	AuditDecisions      bool          `json:"audit_decisions"`
}

// IdentityConfig configures identity management
type IdentityConfig struct {
	IdentityStore       string        `json:"identity_store"`
	TokenTTL            time.Duration `json:"token_ttl"`
	RefreshTokenTTL     time.Duration `json:"refresh_token_ttl"`
	MaxSessions         int           `json:"max_sessions"`
	SessionTimeout      time.Duration `json:"session_timeout"`
	MFARequired         bool          `json:"mfa_required"`
	PasswordPolicy      PasswordPolicy `json:"password_policy"`
	AccountLockout      AccountLockout `json:"account_lockout"`
}

// AuditConfig configures audit logging
type AuditConfig struct {
	Enabled             bool          `json:"enabled"`
	LogLevel            string        `json:"log_level"`
	LogFormat           string        `json:"log_format"`
	LogDestination      string        `json:"log_destination"`
	RetentionDays       int           `json:"retention_days"`
	EncryptionEnabled   bool          `json:"encryption_enabled"`
	SigningEnabled      bool          `json:"signing_enabled"`
	TamperDetection     bool          `json:"tamper_detection"`
}

// TrustDomain represents a SPIFFE trust domain
type TrustDomain struct {
	Name            string   `json:"name"`
	TrustDomainID   string   `json:"trust_domain_id"`
	BundleEndpoint  string   `json:"bundle_endpoint"`
	FederatedWith   []string `json:"federated_with"`
	Policies        []string `json:"policies"`
}

// CertificateRotation configures certificate rotation
type CertificateRotation struct {
	Enabled             bool          `json:"enabled"`
	RotationInterval    time.Duration `json:"rotation_interval"`
	PreRotationWarning  time.Duration `json:"pre_rotation_warning"`
	GracePeriod         time.Duration `json:"grace_period"`
	AutomaticRotation   bool          `json:"automatic_rotation"`
	NotificationChannels []string     `json:"notification_channels"`
}

// SecurityPolicy represents a security policy
type SecurityPolicy struct {
	ID              string            `json:"id"`
	Name            string            `json:"name"`
	Version         string            `json:"version"`
	Description     string            `json:"description"`
	Rules           []PolicyRule      `json:"rules"`
	Subjects        []Subject         `json:"subjects"`
	Resources       []Resource        `json:"resources"`
	Actions         []Action          `json:"actions"`
	Conditions      []Condition       `json:"conditions"`
	Effect          string            `json:"effect"`
	Priority        int               `json:"priority"`
	Metadata        map[string]string `json:"metadata"`
}

// Identity represents a service or user identity
type Identity struct {
	ID              string            `json:"id"`
	SPIFFEID        string            `json:"spiffe_id"`
	Type            string            `json:"type"`
	Name            string            `json:"name"`
	Attributes      map[string]string `json:"attributes"`
	Roles           []string          `json:"roles"`
	Permissions     []Permission      `json:"permissions"`
	CreatedAt       time.Time         `json:"created_at"`
	UpdatedAt       time.Time         `json:"updated_at"`
	ExpiresAt       time.Time         `json:"expires_at"`
	Status          string            `json:"status"`
	LastSeen        time.Time         `json:"last_seen"`
	TrustDomain     string            `json:"trust_domain"`
	Metadata        map[string]string `json:"metadata"`
}

// Certificate represents an X.509 certificate
type Certificate struct {
	ID              string            `json:"id"`
	SerialNumber    string            `json:"serial_number"`
	Subject         string            `json:"subject"`
	Issuer          string            `json:"issuer"`
	NotBefore       time.Time         `json:"not_before"`
	NotAfter        time.Time         `json:"not_after"`
	KeyUsage        []string          `json:"key_usage"`
	ExtKeyUsage     []string          `json:"ext_key_usage"`
	DNSNames        []string          `json:"dns_names"`
	IPAddresses     []string          `json:"ip_addresses"`
	URIs            []string          `json:"uris"`
	Status          string            `json:"status"`
	RevocationReason string           `json:"revocation_reason,omitempty"`
	RevokedAt       *time.Time        `json:"revoked_at,omitempty"`
	Metadata        map[string]string `json:"metadata"`
}

// AuthenticationRequest represents an authentication request
type AuthenticationRequest struct {
	Identity        string            `json:"identity"`
	Credentials     map[string]string `json:"credentials"`
	ClientCert      *x509.Certificate `json:"client_cert,omitempty"`
	RequestMetadata map[string]string `json:"request_metadata"`
	Timestamp       time.Time         `json:"timestamp"`
}

// AuthenticationResponse represents an authentication response
type AuthenticationResponse struct {
	Success         bool              `json:"success"`
	Identity        *Identity         `json:"identity,omitempty"`
	Token           string            `json:"token,omitempty"`
	RefreshToken    string            `json:"refresh_token,omitempty"`
	ExpiresAt       time.Time         `json:"expires_at,omitempty"`
	Permissions     []Permission      `json:"permissions,omitempty"`
	ErrorMessage    string            `json:"error_message,omitempty"`
	Metadata        map[string]string `json:"metadata,omitempty"`
}

// AuthorizationRequest represents an authorization request
type AuthorizationRequest struct {
	Subject         Subject           `json:"subject"`
	Resource        Resource          `json:"resource"`
	Action          Action            `json:"action"`
	Context         map[string]interface{} `json:"context"`
	RequestMetadata map[string]string `json:"request_metadata"`
	Timestamp       time.Time         `json:"timestamp"`
}

// AuthorizationResponse represents an authorization response
type AuthorizationResponse struct {
	Allowed         bool              `json:"allowed"`
	Reason          string            `json:"reason"`
	PolicyID        string            `json:"policy_id,omitempty"`
	Decision        string            `json:"decision"`
	Obligations     []Obligation      `json:"obligations,omitempty"`
	Advice          []Advice          `json:"advice,omitempty"`
	ErrorMessage    string            `json:"error_message,omitempty"`
	Metadata        map[string]string `json:"metadata,omitempty"`
}

// Supporting types
type NodeAttestor struct {
	Type   string            `json:"type"`
	Config map[string]string `json:"config"`
}

type WorkloadAttestor struct {
	Type   string            `json:"type"`
	Config map[string]string `json:"config"`
}

type KeyManager struct {
	Type   string            `json:"type"`
	Config map[string]string `json:"config"`
}

type CertValidation struct {
	VerifyChain     bool `json:"verify_chain"`
	VerifyHostname  bool `json:"verify_hostname"`
	AllowSelfSigned bool `json:"allow_self_signed"`
}

type PasswordPolicy struct {
	MinLength        int  `json:"min_length"`
	RequireUppercase bool `json:"require_uppercase"`
	RequireLowercase bool `json:"require_lowercase"`
	RequireNumbers   bool `json:"require_numbers"`
	RequireSymbols   bool `json:"require_symbols"`
	MaxAge           int  `json:"max_age"`
	HistoryCount     int  `json:"history_count"`
}

type AccountLockout struct {
	Enabled           bool          `json:"enabled"`
	MaxAttempts       int           `json:"max_attempts"`
	LockoutDuration   time.Duration `json:"lockout_duration"`
	ResetAfter        time.Duration `json:"reset_after"`
}

type PolicyRule struct {
	ID          string      `json:"id"`
	Description string      `json:"description"`
	Condition   string      `json:"condition"`
	Effect      string      `json:"effect"`
	Priority    int         `json:"priority"`
}

type Subject struct {
	Type       string            `json:"type"`
	ID         string            `json:"id"`
	Attributes map[string]string `json:"attributes"`
}

type Resource struct {
	Type       string            `json:"type"`
	ID         string            `json:"id"`
	Attributes map[string]string `json:"attributes"`
}

type Action struct {
	Type       string            `json:"type"`
	Name       string            `json:"name"`
	Attributes map[string]string `json:"attributes"`
}

type Condition struct {
	Field    string      `json:"field"`
	Operator string      `json:"operator"`
	Value    interface{} `json:"value"`
}

type Permission struct {
	Resource string   `json:"resource"`
	Actions  []string `json:"actions"`
	Effect   string   `json:"effect"`
}

type Obligation struct {
	Type       string            `json:"type"`
	Parameters map[string]string `json:"parameters"`
}

type Advice struct {
	Type       string            `json:"type"`
	Message    string            `json:"message"`
	Parameters map[string]string `json:"parameters"`
}

// Service components
type SPIFFEManager struct {
	config  *SPIFFEConfig
	client  SPIFFEClient
	metrics *IAMMetrics
	mu      sync.RWMutex
}

type SPIREAgent struct {
	config  *SPIREConfig
	client  SPIREClient
	metrics *IAMMetrics
	mu      sync.RWMutex
}

type mTLSManager struct {
	config      *mTLSConfig
	certManager *CertificateManager
	metrics     *IAMMetrics
	mu          sync.RWMutex
}

type PolicyEngine struct {
	config      *PolicyConfig
	policyStore PolicyStore
	evaluator   PolicyEvaluator
	metrics     *IAMMetrics
	mu          sync.RWMutex
}

type IdentityRegistry struct {
	config      *IdentityConfig
	identities  map[string]*Identity
	certificates map[string]*Certificate
	metrics     *IAMMetrics
	mu          sync.RWMutex
}

type AuditLogger struct {
	config  *AuditConfig
	logger  Logger
	signer  CryptoSigner
	metrics *IAMMetrics
	mu      sync.RWMutex
}

// IAMMetrics contains Prometheus metrics
type IAMMetrics struct {
	AuthenticationAttempts  *prometheus.CounterVec
	AuthenticationLatency   *prometheus.HistogramVec
	AuthorizationRequests   *prometheus.CounterVec
	AuthorizationLatency    *prometheus.HistogramVec
	CertificateRotations    *prometheus.CounterVec
	PolicyEvaluations       *prometheus.CounterVec
	IdentityOperations      *prometheus.CounterVec
	mTLSConnections         *prometheus.CounterVec
	AuditEvents             *prometheus.CounterVec
	SecurityViolations      *prometheus.CounterVec
}

func main() {
	// Load configuration
	config := loadConfig()
	
	// Initialize metrics
	metrics := initializeMetrics()
	
	// Initialize tracer
	tracer := otel.Tracer("zero-trust-iam-service")
	
	// Initialize SPIFFE manager
	spiffeManager := &SPIFFEManager{
		config:  &config.SPIFFEConfig,
		client:  NewSPIFFEClient(config.SPIFFEConfig),
		metrics: metrics,
	}
	
	// Initialize SPIRE agent
	spireAgent := &SPIREAgent{
		config:  &config.SPIREConfig,
		client:  NewSPIREClient(config.SPIREConfig),
		metrics: metrics,
	}
	
	// Initialize mTLS manager
	mtlsManager := &mTLSManager{
		config:      &config.mTLSConfig,
		certManager: NewCertificateManager(config.mTLSConfig),
		metrics:     metrics,
	}
	
	// Initialize policy engine
	policyEngine := &PolicyEngine{
		config:      &config.PolicyConfig,
		policyStore: NewPolicyStore(config.PolicyConfig),
		evaluator:   NewPolicyEvaluator(config.PolicyConfig),
		metrics:     metrics,
	}
	
	// Initialize identity registry
	identityRegistry := &IdentityRegistry{
		config:       &config.IdentityConfig,
		identities:   make(map[string]*Identity),
		certificates: make(map[string]*Certificate),
		metrics:      metrics,
	}
	
	// Initialize audit logger
	auditLogger := &AuditLogger{
		config:  &config.AuditConfig,
		logger:  NewAuditLogger(config.AuditConfig),
		signer:  NewCryptoSigner(config.AuditConfig),
		metrics: metrics,
	}
	
	// Create service instance
	service := &ZeroTrustIAMService{
		spiffeManager:    spiffeManager,
		spireAgent:       spireAgent,
		mtlsManager:      mtlsManager,
		policyEngine:     policyEngine,
		identityRegistry: identityRegistry,
		auditLogger:      auditLogger,
		metrics:          metrics,
		tracer:           tracer,
		config:           config,
	}
	
	// Start HTTP server with mTLS
	router := mux.NewRouter()
	service.setupRoutes(router)
	
	tlsConfig, err := service.createTLSConfig()
	if err != nil {
		log.Fatalf("Failed to create TLS config: %v", err)
	}
	
	server := &http.Server{
		Addr:         fmt.Sprintf(":%d", config.Port),
		Handler:      router,
		TLSConfig:    tlsConfig,
		ReadTimeout:  15 * time.Second,
		WriteTimeout: 15 * time.Second,
		IdleTimeout:  60 * time.Second,
	}
	
	// Start metrics server
	metricsRouter := mux.NewRouter()
	metricsRouter.Handle("/metrics", promhttp.Handler())
	metricsRouter.HandleFunc("/health", service.healthCheck)
	metricsRouter.HandleFunc("/ready", service.readinessCheck)
	
	metricsServer := &http.Server{
		Addr:    fmt.Sprintf(":%d", config.MetricsPort),
		Handler: metricsRouter,
	}
	
	// Start servers
	go func() {
		log.Printf("Starting Zero-Trust IAM service on port %d with mTLS", config.Port)
		if err := server.ListenAndServeTLS("", ""); err != nil && err != http.ErrServerClosed {
			log.Fatalf("Server failed to start: %v", err)
		}
	}()
	
	go func() {
		log.Printf("Starting metrics server on port %d", config.MetricsPort)
		if err := metricsServer.ListenAndServe(); err != nil && err != http.ErrServerClosed {
			log.Fatalf("Metrics server failed to start: %v", err)
		}
	}()
	
	// Start background services
	go service.startCertificateRotation()
	go service.startPolicySync()
	go service.startIdentitySync()
	go service.startAuditProcessor()
	go service.startSecurityMonitoring()
	
	// Wait for interrupt signal
	c := make(chan os.Signal, 1)
	signal.Notify(c, os.Interrupt, syscall.SIGTERM)
	<-c
	
	log.Println("Shutting down Zero-Trust IAM service...")
	
	// Graceful shutdown
	ctx, cancel := context.WithTimeout(context.Background(), 30*time.Second)
	defer cancel()
	
	server.Shutdown(ctx)
	metricsServer.Shutdown(ctx)
	
	log.Println("Zero-Trust IAM service stopped")
}

func (s *ZeroTrustIAMService) setupRoutes(router *mux.Router) {
	// API routes
	api := router.PathPrefix("/api/v1").Subrouter()
	
	// Authentication endpoints
	api.HandleFunc("/auth/authenticate", s.authenticate).Methods("POST")
	api.HandleFunc("/auth/refresh", s.refreshToken).Methods("POST")
	api.HandleFunc("/auth/logout", s.logout).Methods("POST")
	api.HandleFunc("/auth/validate", s.validateToken).Methods("POST")
	
	// Authorization endpoints
	api.HandleFunc("/authz/authorize", s.authorize).Methods("POST")
	api.HandleFunc("/authz/policies", s.listPolicies).Methods("GET")
	api.HandleFunc("/authz/policies", s.createPolicy).Methods("POST")
	api.HandleFunc("/authz/policies/{policyId}", s.getPolicy).Methods("GET")
	api.HandleFunc("/authz/policies/{policyId}", s.updatePolicy).Methods("PUT")
	api.HandleFunc("/authz/policies/{policyId}", s.deletePolicy).Methods("DELETE")
	
	// Identity management endpoints
	api.HandleFunc("/identities", s.listIdentities).Methods("GET")
	api.HandleFunc("/identities", s.createIdentity).Methods("POST")
	api.HandleFunc("/identities/{identityId}", s.getIdentity).Methods("GET")
	api.HandleFunc("/identities/{identityId}", s.updateIdentity).Methods("PUT")
	api.HandleFunc("/identities/{identityId}", s.deleteIdentity).Methods("DELETE")
	api.HandleFunc("/identities/{identityId}/certificates", s.getIdentityCertificates).Methods("GET")
	
	// Certificate management endpoints
	api.HandleFunc("/certificates", s.listCertificates).Methods("GET")
	api.HandleFunc("/certificates", s.issueCertificate).Methods("POST")
	api.HandleFunc("/certificates/{certId}", s.getCertificate).Methods("GET")
	api.HandleFunc("/certificates/{certId}/revoke", s.revokeCertificate).Methods("POST")
	api.HandleFunc("/certificates/{certId}/renew", s.renewCertificate).Methods("POST")
	
	// SPIFFE/SPIRE endpoints
	api.HandleFunc("/spiffe/svid", s.getSVID).Methods("GET")
	api.HandleFunc("/spiffe/bundle", s.getTrustBundle).Methods("GET")
	api.HandleFunc("/spiffe/workloads", s.listWorkloads).Methods("GET")
	api.HandleFunc("/spiffe/workloads/{workloadId}/attest", s.attestWorkload).Methods("POST")
	
	// Trust domain endpoints
	api.HandleFunc("/trust-domains", s.listTrustDomains).Methods("GET")
	api.HandleFunc("/trust-domains/{domainId}/federate", s.federateTrustDomain).Methods("POST")
	api.HandleFunc("/trust-domains/{domainId}/bundle", s.getTrustDomainBundle).Methods("GET")
	
	// Audit endpoints
	api.HandleFunc("/audit/events", s.getAuditEvents).Methods("GET")
	api.HandleFunc("/audit/search", s.searchAuditEvents).Methods("POST")
	api.HandleFunc("/audit/export", s.exportAuditEvents).Methods("POST")
	
	// Health and status
	api.HandleFunc("/health", s.healthCheck).Methods("GET")
	api.HandleFunc("/ready", s.readinessCheck).Methods("GET")
	api.HandleFunc("/status", s.getStatus).Methods("GET")
}

func (s *ZeroTrustIAMService) authenticate(w http.ResponseWriter, r *http.Request) {
	ctx, span := s.tracer.Start(r.Context(), "authenticate")
	defer span.End()
	
	var authReq AuthenticationRequest
	if err := json.NewDecoder(r.Body).Decode(&authReq); err != nil {
		http.Error(w, "Invalid authentication request", http.StatusBadRequest)
		return
	}
	
	start := time.Now()
	
	// Extract client certificate if present
	if r.TLS != nil && len(r.TLS.PeerCertificates) > 0 {
		authReq.ClientCert = r.TLS.PeerCertificates[0]
	}
	
	// Perform authentication
	response, err := s.performAuthentication(ctx, &authReq)
	if err != nil {
		s.metrics.AuthenticationAttempts.WithLabelValues("failure", "error").Inc()
		span.RecordError(err)
		http.Error(w, fmt.Sprintf("Authentication failed: %v", err), http.StatusUnauthorized)
		return
	}
	
	duration := time.Since(start)
	s.metrics.AuthenticationLatency.WithLabelValues("success").Observe(duration.Seconds())
	
	if response.Success {
		s.metrics.AuthenticationAttempts.WithLabelValues("success", "authenticated").Inc()
		
		// Log successful authentication
		s.auditLogger.LogEvent("authentication", "success", map[string]interface{}{
			"identity":    authReq.Identity,
			"client_ip":   r.RemoteAddr,
			"user_agent":  r.UserAgent(),
			"duration_ms": duration.Milliseconds(),
		})
	} else {
		s.metrics.AuthenticationAttempts.WithLabelValues("failure", "invalid_credentials").Inc()
		
		// Log failed authentication
		s.auditLogger.LogEvent("authentication", "failure", map[string]interface{}{
			"identity":     authReq.Identity,
			"client_ip":    r.RemoteAddr,
			"user_agent":   r.UserAgent(),
			"error":        response.ErrorMessage,
			"duration_ms":  duration.Milliseconds(),
		})
	}
	
	span.SetAttributes(
		attribute.String("identity", authReq.Identity),
		attribute.Bool("success", response.Success),
		attribute.Float64("duration_seconds", duration.Seconds()),
	)
	
	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(response)
}

func (s *ZeroTrustIAMService) authorize(w http.ResponseWriter, r *http.Request) {
	ctx, span := s.tracer.Start(r.Context(), "authorize")
	defer span.End()
	
	var authzReq AuthorizationRequest
	if err := json.NewDecoder(r.Body).Decode(&authzReq); err != nil {
		http.Error(w, "Invalid authorization request", http.StatusBadRequest)
		return
	}
	
	start := time.Now()
	
	// Perform authorization
	response, err := s.performAuthorization(ctx, &authzReq)
	if err != nil {
		s.metrics.AuthorizationRequests.WithLabelValues("error").Inc()
		span.RecordError(err)
		http.Error(w, fmt.Sprintf("Authorization failed: %v", err), http.StatusInternalServerError)
		return
	}
	
	duration := time.Since(start)
	s.metrics.AuthorizationLatency.WithLabelValues(response.Decision).Observe(duration.Seconds())
	
	if response.Allowed {
		s.metrics.AuthorizationRequests.WithLabelValues("allowed").Inc()
	} else {
		s.metrics.AuthorizationRequests.WithLabelValues("denied").Inc()
	}
	
	// Log authorization decision
	s.auditLogger.LogEvent("authorization", response.Decision, map[string]interface{}{
		"subject":     authzReq.Subject,
		"resource":    authzReq.Resource,
		"action":      authzReq.Action,
		"allowed":     response.Allowed,
		"policy_id":   response.PolicyID,
		"reason":      response.Reason,
		"duration_ms": duration.Milliseconds(),
	})
	
	span.SetAttributes(
		attribute.String("subject_id", authzReq.Subject.ID),
		attribute.String("resource_id", authzReq.Resource.ID),
		attribute.String("action_name", authzReq.Action.Name),
		attribute.Bool("allowed", response.Allowed),
		attribute.String("decision", response.Decision),
	)
	
	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(response)
}

func (s *ZeroTrustIAMService) createTLSConfig() (*tls.Config, error) {
	if !s.config.mTLSConfig.Enabled {
		return nil, nil
	}
	
	// Load CA certificate
	caCert, err := s.mtlsManager.GetCACertificate()
	if err != nil {
		return nil, fmt.Errorf("failed to load CA certificate: %v", err)
	}
	
	caCertPool := x509.NewCertPool()
	caCertPool.AddCert(caCert)
	
	// Load server certificate and key
	serverCert, err := s.mtlsManager.GetServerCertificate()
	if err != nil {
		return nil, fmt.Errorf("failed to load server certificate: %v", err)
	}
	
	tlsConfig := &tls.Config{
		Certificates: []tls.Certificate{*serverCert},
		ClientAuth:   tls.RequireAndVerifyClientCert,
		ClientCAs:    caCertPool,
		MinVersion:   tls.VersionTLS12,
		MaxVersion:   tls.VersionTLS13,
		CipherSuites: []uint16{
			tls.TLS_ECDHE_RSA_WITH_AES_256_GCM_SHA384,
			tls.TLS_ECDHE_RSA_WITH_CHACHA20_POLY1305,
			tls.TLS_ECDHE_ECDSA_WITH_AES_256_GCM_SHA384,
			tls.TLS_ECDHE_ECDSA_WITH_CHACHA20_POLY1305,
		},
	}
	
	if s.config.mTLSConfig.ClientCertRequired {
		tlsConfig.ClientAuth = tls.RequireAndVerifyClientCert
	}
	
	return tlsConfig, nil
}

func (s *ZeroTrustIAMService) performAuthentication(ctx context.Context, req *AuthenticationRequest) (*AuthenticationResponse, error) {
	// Implement authentication logic
	
	// 1. Validate client certificate if present
	if req.ClientCert != nil {
		if err := s.validateClientCertificate(req.ClientCert); err != nil {
			return &AuthenticationResponse{
				Success:      false,
				ErrorMessage: fmt.Sprintf("Invalid client certificate: %v", err),
			}, nil
		}
	}
	
	// 2. Look up identity
	identity, err := s.identityRegistry.GetIdentity(req.Identity)
	if err != nil {
		return &AuthenticationResponse{
			Success:      false,
			ErrorMessage: "Identity not found",
		}, nil
	}
	
	// 3. Validate credentials
	if !s.validateCredentials(identity, req.Credentials) {
		return &AuthenticationResponse{
			Success:      false,
			ErrorMessage: "Invalid credentials",
		}, nil
	}
	
	// 4. Generate tokens
	token, err := s.generateAccessToken(identity)
	if err != nil {
		return nil, fmt.Errorf("failed to generate access token: %v", err)
	}
	
	refreshToken, err := s.generateRefreshToken(identity)
	if err != nil {
		return nil, fmt.Errorf("failed to generate refresh token: %v", err)
	}
	
	return &AuthenticationResponse{
		Success:      true,
		Identity:     identity,
		Token:        token,
		RefreshToken: refreshToken,
		ExpiresAt:    time.Now().Add(s.config.IdentityConfig.TokenTTL),
		Permissions:  identity.Permissions,
	}, nil
}

func (s *ZeroTrustIAMService) performAuthorization(ctx context.Context, req *AuthorizationRequest) (*AuthorizationResponse, error) {
	// Evaluate policies using the policy engine
	decision, err := s.policyEngine.Evaluate(ctx, req)
	if err != nil {
		return nil, fmt.Errorf("policy evaluation failed: %v", err)
	}
	
	return decision, nil
}

func (s *ZeroTrustIAMService) startCertificateRotation() {
	if !s.config.CertificateRotation.Enabled {
		return
	}
	
	log.Println("Starting certificate rotation service...")
	
	ticker := time.NewTicker(s.config.CertificateRotation.RotationInterval)
	defer ticker.Stop()
	
	for {
		select {
		case <-ticker.C:
			s.rotateCertificates()
		}
	}
}

func (s *ZeroTrustIAMService) startPolicySync() {
	log.Println("Starting policy synchronization service...")
	
	ticker := time.NewTicker(s.config.PolicyConfig.PolicyUpdateInterval)
	defer ticker.Stop()
	
	for {
		select {
		case <-ticker.C:
			s.syncPolicies()
		}
	}
}

func (s *ZeroTrustIAMService) startIdentitySync() {
	log.Println("Starting identity synchronization service...")
	
	ticker := time.NewTicker(1 * time.Hour)
	defer ticker.Stop()
	
	for {
		select {
		case <-ticker.C:
			s.syncIdentities()
		}
	}
}

func (s *ZeroTrustIAMService) startAuditProcessor() {
	log.Println("Starting audit processor...")
	
	ticker := time.NewTicker(5 * time.Minute)
	defer ticker.Stop()
	
	for {
		select {
		case <-ticker.C:
			s.processAuditEvents()
		}
	}
}

func (s *ZeroTrustIAMService) startSecurityMonitoring() {
	log.Println("Starting security monitoring...")
	
	ticker := time.NewTicker(1 * time.Minute)
	defer ticker.Stop()
	
	for {
		select {
		case <-ticker.C:
			s.monitorSecurity()
		}
	}
}

func (s *ZeroTrustIAMService) rotateCertificates() {
	log.Println("Rotating certificates...")
	
	// Get certificates that need rotation
	certs := s.identityRegistry.GetExpiringCertificates(s.config.CertificateRotation.PreRotationWarning)
	
	for _, cert := range certs {
		if err := s.mtlsManager.RotateCertificate(cert); err != nil {
			log.Printf("Failed to rotate certificate %s: %v", cert.ID, err)
			s.metrics.CertificateRotations.WithLabelValues("failure").Inc()
		} else {
			log.Printf("Successfully rotated certificate %s", cert.ID)
			s.metrics.CertificateRotations.WithLabelValues("success").Inc()
		}
	}
}

func (s *ZeroTrustIAMService) syncPolicies() {
	log.Println("Synchronizing policies...")
	
	// Sync policies from external sources
	if err := s.policyEngine.SyncPolicies(); err != nil {
		log.Printf("Failed to sync policies: %v", err)
	}
}

func (s *ZeroTrustIAMService) syncIdentities() {
	log.Println("Synchronizing identities...")
	
	// Sync identities from external sources
	if err := s.identityRegistry.SyncIdentities(); err != nil {
		log.Printf("Failed to sync identities: %v", err)
	}
}

func (s *ZeroTrustIAMService) processAuditEvents() {
	// Process and archive audit events
	if err := s.auditLogger.ProcessEvents(); err != nil {
		log.Printf("Failed to process audit events: %v", err)
	}
}

func (s *ZeroTrustIAMService) monitorSecurity() {
	// Monitor for security violations and anomalies
	violations := s.detectSecurityViolations()
	
	for _, violation := range violations {
		s.metrics.SecurityViolations.WithLabelValues(violation.Type).Inc()
		s.auditLogger.LogEvent("security_violation", violation.Type, violation.Details)
		
		// Take automated response if configured
		s.respondToSecurityViolation(violation)
	}
}

// Helper functions and placeholder implementations
func loadConfig() *Config {
	return &Config{
		Port:        8443, // HTTPS port for mTLS
		MetricsPort: 9090,
		SPIFFEConfig: SPIFFEConfig{
			TrustDomain:           "atlasmesh.fleet.local",
			SocketPath:            "/tmp/spire-agent/public/api.sock",
			BundleEndpoint:        "https://spire-server:8443",
			WorkloadAPITimeout:    30 * time.Second,
			SVIDRefreshInterval:   5 * time.Minute,
			BundleRefreshInterval: 1 * time.Hour,
		},
		SPIREConfig: SPIREConfig{
			ServerAddress:       "spire-server:8081",
			AgentSocketPath:     "/tmp/spire-agent/public/api.sock",
			DataDir:             "/opt/spire/data",
			LogLevel:            "INFO",
			HealthCheckInterval: 30 * time.Second,
		},
		mTLSConfig: mTLSConfig{
			Enabled:            true,
			ClientCertRequired: true,
			VerifyClientCert:   true,
			MinTLSVersion:      "1.2",
			MaxTLSVersion:      "1.3",
			RevocationChecking: true,
			OCSPStapling:      true,
		},
		PolicyConfig: PolicyConfig{
			PolicyStore:          "opa",
			PolicyFormat:         "rego",
			EvaluationTimeout:    5 * time.Second,
			CacheEnabled:         true,
			CacheTTL:            15 * time.Minute,
			PolicyUpdateInterval: 1 * time.Minute,
			DefaultDeny:          true,
			AuditDecisions:       true,
		},
		IdentityConfig: IdentityConfig{
			IdentityStore:   "internal",
			TokenTTL:        1 * time.Hour,
			RefreshTokenTTL: 24 * time.Hour,
			MaxSessions:     5,
			SessionTimeout:  30 * time.Minute,
			MFARequired:     true,
		},
		AuditConfig: AuditConfig{
			Enabled:           true,
			LogLevel:          "INFO",
			LogFormat:         "json",
			LogDestination:    "file",
			RetentionDays:     365,
			EncryptionEnabled: true,
			SigningEnabled:    true,
			TamperDetection:   true,
		},
		CertificateRotation: CertificateRotation{
			Enabled:            true,
			RotationInterval:   24 * time.Hour,
			PreRotationWarning: 7 * 24 * time.Hour,
			GracePeriod:        1 * time.Hour,
			AutomaticRotation:  true,
		},
	}
}

func initializeMetrics() *IAMMetrics {
	metrics := &IAMMetrics{
		AuthenticationAttempts: prometheus.NewCounterVec(
			prometheus.CounterOpts{
				Name: "atlasmesh_iam_authentication_attempts_total",
				Help: "Total authentication attempts",
			},
			[]string{"result", "reason"},
		),
		AuthenticationLatency: prometheus.NewHistogramVec(
			prometheus.HistogramOpts{
				Name: "atlasmesh_iam_authentication_duration_seconds",
				Help: "Authentication duration",
			},
			[]string{"result"},
		),
		AuthorizationRequests: prometheus.NewCounterVec(
			prometheus.CounterOpts{
				Name: "atlasmesh_iam_authorization_requests_total",
				Help: "Total authorization requests",
			},
			[]string{"decision"},
		),
		AuthorizationLatency: prometheus.NewHistogramVec(
			prometheus.HistogramOpts{
				Name: "atlasmesh_iam_authorization_duration_seconds",
				Help: "Authorization duration",
			},
			[]string{"decision"},
		),
		CertificateRotations: prometheus.NewCounterVec(
			prometheus.CounterOpts{
				Name: "atlasmesh_iam_certificate_rotations_total",
				Help: "Certificate rotations",
			},
			[]string{"result"},
		),
		PolicyEvaluations: prometheus.NewCounterVec(
			prometheus.CounterOpts{
				Name: "atlasmesh_iam_policy_evaluations_total",
				Help: "Policy evaluations",
			},
			[]string{"policy_id", "decision"},
		),
		IdentityOperations: prometheus.NewCounterVec(
			prometheus.CounterOpts{
				Name: "atlasmesh_iam_identity_operations_total",
				Help: "Identity operations",
			},
			[]string{"operation", "result"},
		),
		mTLSConnections: prometheus.NewCounterVec(
			prometheus.CounterOpts{
				Name: "atlasmesh_iam_mtls_connections_total",
				Help: "mTLS connections",
			},
			[]string{"result"},
		),
		AuditEvents: prometheus.NewCounterVec(
			prometheus.CounterOpts{
				Name: "atlasmesh_iam_audit_events_total",
				Help: "Audit events",
			},
			[]string{"event_type", "result"},
		),
		SecurityViolations: prometheus.NewCounterVec(
			prometheus.CounterOpts{
				Name: "atlasmesh_iam_security_violations_total",
				Help: "Security violations",
			},
			[]string{"violation_type"},
		),
	}
	
	prometheus.MustRegister(
		metrics.AuthenticationAttempts,
		metrics.AuthenticationLatency,
		metrics.AuthorizationRequests,
		metrics.AuthorizationLatency,
		metrics.CertificateRotations,
		metrics.PolicyEvaluations,
		metrics.IdentityOperations,
		metrics.mTLSConnections,
		metrics.AuditEvents,
		metrics.SecurityViolations,
	)
	
	return metrics
}

func (s *ZeroTrustIAMService) healthCheck(w http.ResponseWriter, r *http.Request) {
	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(map[string]string{"status": "healthy"})
}

func (s *ZeroTrustIAMService) readinessCheck(w http.ResponseWriter, r *http.Request) {
	// Check if all components are ready
	ready := s.spiffeManager.IsReady() &&
		s.spireAgent.IsReady() &&
		s.policyEngine.IsReady() &&
		s.identityRegistry.IsReady()
	
	if !ready {
		http.Error(w, "Service not ready", http.StatusServiceUnavailable)
		return
	}
	
	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(map[string]string{"status": "ready"})
}

func (s *ZeroTrustIAMService) getStatus(w http.ResponseWriter, r *http.Request) {
	status := map[string]interface{}{
		"service":           "zero-trust-iam",
		"version":           "1.0.0",
		"spiffe_manager":    s.spiffeManager.GetStatus(),
		"spire_agent":       s.spireAgent.GetStatus(),
		"mtls_manager":      s.mtlsManager.GetStatus(),
		"policy_engine":     s.policyEngine.GetStatus(),
		"identity_registry": s.identityRegistry.GetStatus(),
		"audit_logger":      s.auditLogger.GetStatus(),
	}
	
	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(status)
}

// Placeholder implementations for interfaces and methods
type SPIFFEClient interface{ GetSVID() (*Certificate, error) }
type SPIREClient interface{ RegisterWorkload(workloadID string) error }
type CertificateManager struct{}
type PolicyStore interface{ GetPolicy(id string) (*SecurityPolicy, error) }
type PolicyEvaluator interface{ Evaluate(ctx context.Context, req *AuthorizationRequest) (*AuthorizationResponse, error) }
type Logger interface{ Log(level, message string, fields map[string]interface{}) }
type CryptoSigner interface{ Sign(data []byte) ([]byte, error) }

type SecurityViolation struct {
	Type    string
	Details map[string]interface{}
}

func NewSPIFFEClient(config SPIFFEConfig) SPIFFEClient { return nil }
func NewSPIREClient(config SPIREConfig) SPIREClient { return nil }
func NewCertificateManager(config mTLSConfig) *CertificateManager { return &CertificateManager{} }
func NewPolicyStore(config PolicyConfig) PolicyStore { return nil }
func NewPolicyEvaluator(config PolicyConfig) PolicyEvaluator { return nil }
func NewAuditLogger(config AuditConfig) Logger { return nil }
func NewCryptoSigner(config AuditConfig) CryptoSigner { return nil }

// Placeholder method implementations
func (s *ZeroTrustIAMService) validateClientCertificate(cert *x509.Certificate) error { return nil }
func (s *ZeroTrustIAMService) validateCredentials(identity *Identity, credentials map[string]string) bool { return true }
func (s *ZeroTrustIAMService) generateAccessToken(identity *Identity) (string, error) { return "access_token", nil }
func (s *ZeroTrustIAMService) generateRefreshToken(identity *Identity) (string, error) { return "refresh_token", nil }
func (s *ZeroTrustIAMService) detectSecurityViolations() []SecurityViolation { return []SecurityViolation{} }
func (s *ZeroTrustIAMService) respondToSecurityViolation(violation SecurityViolation) {}

func (sm *SPIFFEManager) IsReady() bool { return true }
func (sm *SPIFFEManager) GetStatus() map[string]interface{} { return map[string]interface{}{} }

func (sa *SPIREAgent) IsReady() bool { return true }
func (sa *SPIREAgent) GetStatus() map[string]interface{} { return map[string]interface{}{} }

func (mm *mTLSManager) GetCACertificate() (*x509.Certificate, error) { return nil, nil }
func (mm *mTLSManager) GetServerCertificate() (*tls.Certificate, error) { return nil, nil }
func (mm *mTLSManager) RotateCertificate(cert *Certificate) error { return nil }
func (mm *mTLSManager) GetStatus() map[string]interface{} { return map[string]interface{}{} }

func (pe *PolicyEngine) Evaluate(ctx context.Context, req *AuthorizationRequest) (*AuthorizationResponse, error) {
	return &AuthorizationResponse{
		Allowed:  true,
		Decision: "allow",
		Reason:   "policy_match",
	}, nil
}
func (pe *PolicyEngine) SyncPolicies() error { return nil }
func (pe *PolicyEngine) IsReady() bool { return true }
func (pe *PolicyEngine) GetStatus() map[string]interface{} { return map[string]interface{}{} }

func (ir *IdentityRegistry) GetIdentity(id string) (*Identity, error) {
	return &Identity{
		ID:       id,
		SPIFFEID: fmt.Sprintf("spiffe://atlasmesh.fleet.local/service/%s", id),
		Type:     "service",
		Name:     id,
		Status:   "active",
	}, nil
}
func (ir *IdentityRegistry) GetExpiringCertificates(warning time.Duration) []*Certificate { return []*Certificate{} }
func (ir *IdentityRegistry) SyncIdentities() error { return nil }
func (ir *IdentityRegistry) IsReady() bool { return true }
func (ir *IdentityRegistry) GetStatus() map[string]interface{} { return map[string]interface{}{} }

func (al *AuditLogger) LogEvent(eventType, result string, details map[string]interface{}) {}
func (al *AuditLogger) ProcessEvents() error { return nil }
func (al *AuditLogger) GetStatus() map[string]interface{} { return map[string]interface{}{} }

// Placeholder implementations for remaining handlers
func (s *ZeroTrustIAMService) refreshToken(w http.ResponseWriter, r *http.Request) {
	http.Error(w, "Not implemented", http.StatusNotImplemented)
}

func (s *ZeroTrustIAMService) logout(w http.ResponseWriter, r *http.Request) {
	http.Error(w, "Not implemented", http.StatusNotImplemented)
}

func (s *ZeroTrustIAMService) validateToken(w http.ResponseWriter, r *http.Request) {
	http.Error(w, "Not implemented", http.StatusNotImplemented)
}

func (s *ZeroTrustIAMService) listPolicies(w http.ResponseWriter, r *http.Request) {
	http.Error(w, "Not implemented", http.StatusNotImplemented)
}

func (s *ZeroTrustIAMService) createPolicy(w http.ResponseWriter, r *http.Request) {
	http.Error(w, "Not implemented", http.StatusNotImplemented)
}

func (s *ZeroTrustIAMService) getPolicy(w http.ResponseWriter, r *http.Request) {
	http.Error(w, "Not implemented", http.StatusNotImplemented)
}

func (s *ZeroTrustIAMService) updatePolicy(w http.ResponseWriter, r *http.Request) {
	http.Error(w, "Not implemented", http.StatusNotImplemented)
}

func (s *ZeroTrustIAMService) deletePolicy(w http.ResponseWriter, r *http.Request) {
	http.Error(w, "Not implemented", http.StatusNotImplemented)
}

func (s *ZeroTrustIAMService) listIdentities(w http.ResponseWriter, r *http.Request) {
	http.Error(w, "Not implemented", http.StatusNotImplemented)
}

func (s *ZeroTrustIAMService) createIdentity(w http.ResponseWriter, r *http.Request) {
	http.Error(w, "Not implemented", http.StatusNotImplemented)
}

func (s *ZeroTrustIAMService) getIdentity(w http.ResponseWriter, r *http.Request) {
	http.Error(w, "Not implemented", http.StatusNotImplemented)
}

func (s *ZeroTrustIAMService) updateIdentity(w http.ResponseWriter, r *http.Request) {
	http.Error(w, "Not implemented", http.StatusNotImplemented)
}

func (s *ZeroTrustIAMService) deleteIdentity(w http.ResponseWriter, r *http.Request) {
	http.Error(w, "Not implemented", http.StatusNotImplemented)
}

func (s *ZeroTrustIAMService) getIdentityCertificates(w http.ResponseWriter, r *http.Request) {
	http.Error(w, "Not implemented", http.StatusNotImplemented)
}

func (s *ZeroTrustIAMService) listCertificates(w http.ResponseWriter, r *http.Request) {
	http.Error(w, "Not implemented", http.StatusNotImplemented)
}

func (s *ZeroTrustIAMService) issueCertificate(w http.ResponseWriter, r *http.Request) {
	http.Error(w, "Not implemented", http.StatusNotImplemented)
}

func (s *ZeroTrustIAMService) getCertificate(w http.ResponseWriter, r *http.Request) {
	http.Error(w, "Not implemented", http.StatusNotImplemented)
}

func (s *ZeroTrustIAMService) revokeCertificate(w http.ResponseWriter, r *http.Request) {
	http.Error(w, "Not implemented", http.StatusNotImplemented)
}

func (s *ZeroTrustIAMService) renewCertificate(w http.ResponseWriter, r *http.Request) {
	http.Error(w, "Not implemented", http.StatusNotImplemented)
}

func (s *ZeroTrustIAMService) getSVID(w http.ResponseWriter, r *http.Request) {
	http.Error(w, "Not implemented", http.StatusNotImplemented)
}

func (s *ZeroTrustIAMService) getTrustBundle(w http.ResponseWriter, r *http.Request) {
	http.Error(w, "Not implemented", http.StatusNotImplemented)
}

func (s *ZeroTrustIAMService) listWorkloads(w http.ResponseWriter, r *http.Request) {
	http.Error(w, "Not implemented", http.StatusNotImplemented)
}

func (s *ZeroTrustIAMService) attestWorkload(w http.ResponseWriter, r *http.Request) {
	http.Error(w, "Not implemented", http.StatusNotImplemented)
}

func (s *ZeroTrustIAMService) listTrustDomains(w http.ResponseWriter, r *http.Request) {
	http.Error(w, "Not implemented", http.StatusNotImplemented)
}

func (s *ZeroTrustIAMService) federateTrustDomain(w http.ResponseWriter, r *http.Request) {
	http.Error(w, "Not implemented", http.StatusNotImplemented)
}

func (s *ZeroTrustIAMService) getTrustDomainBundle(w http.ResponseWriter, r *http.Request) {
	http.Error(w, "Not implemented", http.StatusNotImplemented)
}

func (s *ZeroTrustIAMService) getAuditEvents(w http.ResponseWriter, r *http.Request) {
	http.Error(w, "Not implemented", http.StatusNotImplemented)
}

func (s *ZeroTrustIAMService) searchAuditEvents(w http.ResponseWriter, r *http.Request) {
	http.Error(w, "Not implemented", http.StatusNotImplemented)
}

func (s *ZeroTrustIAMService) exportAuditEvents(w http.ResponseWriter, r *http.Request) {
	http.Error(w, "Not implemented", http.StatusNotImplemented)
}
