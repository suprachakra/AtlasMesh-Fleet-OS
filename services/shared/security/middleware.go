package security

import (
	"context"
	"crypto/tls"
	"encoding/json"
	"fmt"
	"net/http"
	"strings"
	"time"

	"github.com/gorilla/mux"
	"github.com/prometheus/client_golang/prometheus"
	"github.com/prometheus/client_golang/prometheus/promauto"
	"go.opentelemetry.io/otel"
	"go.opentelemetry.io/otel/trace"
)

// SecurityMiddleware provides comprehensive security controls for all services
type SecurityMiddleware struct {
	config     SecurityConfig
	metrics    *SecurityMetrics
	tracer     trace.Tracer
	rateLimiter *RateLimiter
	validator  *InputValidator
	auditor    *SecurityAuditor
}

// SecurityConfig holds security configuration
type SecurityConfig struct {
	// Authentication
	JWTSecret           string        `json:"jwt_secret"`
	JWTExpiry          time.Duration `json:"jwt_expiry"`
	APIKeySecret       string        `json:"api_key_secret"`
	
	// Authorization
	RBACEnabled        bool          `json:"rbac_enabled"`
	PolicyEngineURL    string        `json:"policy_engine_url"`
	
	// Rate Limiting
	RateLimitEnabled   bool          `json:"rate_limit_enabled"`
	RateLimitRPS       int           `json:"rate_limit_rps"`
	RateLimitBurst     int           `json:"rate_limit_burst"`
	
	// Input Validation
	MaxRequestSize     int64         `json:"max_request_size"`
	MaxHeaderSize      int           `json:"max_header_size"`
	AllowedOrigins     []string      `json:"allowed_origins"`
	
	// mTLS
	MTLSEnabled        bool          `json:"mtls_enabled"`
	ClientCAFile        string        `json:"client_ca_file"`
	
	// Encryption
	EncryptionKey      string        `json:"encryption_key"`
	
	// Audit
	AuditEnabled       bool          `json:"audit_enabled"`
	AuditLogLevel      string        `json:"audit_log_level"`
}

// SecurityMetrics tracks security-related metrics
type SecurityMetrics struct {
	RequestsTotal        *prometheus.CounterVec
	RequestDuration      *prometheus.HistogramVec
	RateLimitHits        *prometheus.CounterVec
	AuthFailures         *prometheus.CounterVec
	InputValidationFailures *prometheus.CounterVec
	SecurityViolations   *prometheus.CounterVec
	MTLSConnections      *prometheus.GaugeVec
}

// RateLimiter implements token bucket rate limiting
type RateLimiter struct {
	buckets map[string]*TokenBucket
}

// TokenBucket implements token bucket algorithm
type TokenBucket struct {
	capacity    int
	tokens      int
	lastRefill  time.Time
	refillRate  time.Duration
}

// InputValidator validates and sanitizes input
type InputValidator struct {
	schemas map[string]*JSONSchema
}

// SecurityAuditor logs security events
type SecurityAuditor struct {
	logger Logger
	events chan SecurityEvent
}

// SecurityEvent represents a security event
type SecurityEvent struct {
	Timestamp   time.Time `json:"timestamp"`
	EventType   string    `json:"event_type"`
	Severity    string    `json:"severity"`
	UserID      string    `json:"user_id"`
	IPAddress   string    `json:"ip_address"`
	UserAgent   string    `json:"user_agent"`
	Resource    string    `json:"resource"`
	Action      string    `json:"action"`
	Result      string    `json:"result"`
	Details     string    `json:"details"`
}

// Logger interface for security auditing
type Logger interface {
	Info(msg string, fields ...interface{})
	Warn(msg string, fields ...interface{})
	Error(msg string, fields ...interface{})
	Debug(msg string, fields ...interface{})
}

// JSONSchema represents a JSON schema for validation
type JSONSchema struct {
	Type       string                 `json:"type"`
	Properties map[string]interface{} `json:"properties"`
	Required   []string               `json:"required"`
}

// NewSecurityMiddleware creates a new security middleware instance
func NewSecurityMiddleware(config SecurityConfig) *SecurityMiddleware {
	metrics := &SecurityMetrics{
		RequestsTotal: promauto.NewCounterVec(
			prometheus.CounterOpts{
				Name: "security_requests_total",
				Help: "Total number of requests processed by security middleware",
			},
			[]string{"method", "endpoint", "status", "auth_type"},
		),
		RequestDuration: promauto.NewHistogramVec(
			prometheus.HistogramOpts{
				Name: "security_request_duration_seconds",
				Help: "Request duration in seconds",
				Buckets: []float64{0.001, 0.005, 0.01, 0.025, 0.05, 0.1, 0.25, 0.5, 1.0, 2.5, 5.0, 10.0},
			},
			[]string{"method", "endpoint", "auth_type"},
		),
		RateLimitHits: promauto.NewCounterVec(
			prometheus.CounterOpts{
				Name: "security_rate_limit_hits_total",
				Help: "Total number of rate limit hits",
			},
			[]string{"endpoint", "client_ip"},
		),
		AuthFailures: promauto.NewCounterVec(
			prometheus.CounterOpts{
				Name: "security_auth_failures_total",
				Help: "Total number of authentication failures",
			},
			[]string{"auth_type", "reason"},
		),
		InputValidationFailures: promauto.NewCounterVec(
			prometheus.CounterOpts{
				Name: "security_input_validation_failures_total",
				Help: "Total number of input validation failures",
			},
			[]string{"endpoint", "field", "reason"},
		),
		SecurityViolations: promauto.NewCounterVec(
			prometheus.CounterOpts{
				Name: "security_violations_total",
				Help: "Total number of security violations",
			},
			[]string{"violation_type", "severity"},
		),
		MTLSConnections: promauto.NewGaugeVec(
			prometheus.GaugeOpts{
				Name: "security_mtls_connections",
				Help: "Number of active mTLS connections",
			},
			[]string{"client_cert"},
		),
	}

	return &SecurityMiddleware{
		config:     config,
		metrics:    metrics,
		tracer:     otel.Tracer("security-middleware"),
		rateLimiter: NewRateLimiter(config.RateLimitRPS, config.RateLimitBurst),
		validator:  NewInputValidator(),
		auditor:    NewSecurityAuditor(),
	}
}

// NewRateLimiter creates a new rate limiter
func NewRateLimiter(rps, burst int) *RateLimiter {
	return &RateLimiter{
		buckets: make(map[string]*TokenBucket),
	}
}

// NewInputValidator creates a new input validator
func NewInputValidator() *InputValidator {
	return &InputValidator{
		schemas: make(map[string]*JSONSchema),
	}
}

// NewSecurityAuditor creates a new security auditor
func NewSecurityAuditor() *SecurityAuditor {
	return &SecurityAuditor{
		events: make(chan SecurityEvent, 1000),
	}
}

// SecurityMiddlewareFunc returns the security middleware function
func (sm *SecurityMiddleware) SecurityMiddlewareFunc() func(http.Handler) http.Handler {
	return func(next http.Handler) http.Handler {
		return http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
			ctx, span := sm.tracer.Start(r.Context(), "security-middleware")
			defer span.End()

			start := time.Now()
			
			// Add security headers
			sm.addSecurityHeaders(w)
			
			// Rate limiting
			if sm.config.RateLimitEnabled {
				if !sm.rateLimit(r) {
					sm.metrics.RateLimitHits.WithLabelValues(r.URL.Path, sm.getClientIP(r)).Inc()
					sm.auditEvent(SecurityEvent{
						Timestamp: time.Now(),
						EventType: "rate_limit_exceeded",
						Severity:  "medium",
						IPAddress: sm.getClientIP(r),
						UserAgent: r.UserAgent(),
						Resource:  r.URL.Path,
						Action:    r.Method,
						Result:    "blocked",
						Details:   "Rate limit exceeded",
					})
					http.Error(w, "Rate limit exceeded", http.StatusTooManyRequests)
					return
				}
			}
			
			// mTLS validation
			if sm.config.MTLSEnabled {
				if !sm.validateMTLS(r) {
					sm.metrics.SecurityViolations.WithLabelValues("mtls_validation_failed", "high").Inc()
					sm.auditEvent(SecurityEvent{
						Timestamp: time.Now(),
						EventType: "mtls_validation_failed",
						Severity:  "high",
						IPAddress: sm.getClientIP(r),
						UserAgent: r.UserAgent(),
						Resource:  r.URL.Path,
						Action:    r.Method,
						Result:    "blocked",
						Details:   "mTLS validation failed",
					})
					http.Error(w, "mTLS validation failed", http.StatusForbidden)
					return
				}
			}
			
			// Authentication
			authResult := sm.authenticate(r)
			if !authResult.Success {
				sm.metrics.AuthFailures.WithLabelValues(authResult.AuthType, authResult.Reason).Inc()
				sm.auditEvent(SecurityEvent{
					Timestamp: time.Now(),
					EventType: "authentication_failed",
					Severity:  "medium",
					IPAddress: sm.getClientIP(r),
					UserAgent: r.UserAgent(),
					Resource:  r.URL.Path,
					Action:    r.Method,
					Result:    "blocked",
					Details:   authResult.Reason,
				})
				http.Error(w, "Authentication failed", http.StatusUnauthorized)
				return
			}
			
			// Authorization
			if sm.config.RBACEnabled {
				if !sm.authorize(r, authResult) {
					sm.metrics.SecurityViolations.WithLabelValues("authorization_failed", "medium").Inc()
					sm.auditEvent(SecurityEvent{
						Timestamp: time.Now(),
						EventType: "authorization_failed",
						Severity:  "medium",
						UserID:    authResult.UserID,
						IPAddress: sm.getClientIP(r),
						UserAgent: r.UserAgent(),
						Resource:  r.URL.Path,
						Action:    r.Method,
						Result:    "blocked",
						Details:   "Insufficient permissions",
					})
					http.Error(w, "Insufficient permissions", http.StatusForbidden)
					return
				}
			}
			
			// Input validation
			if r.Method == "POST" || r.Method == "PUT" || r.Method == "PATCH" {
				if !sm.validateInput(r) {
					sm.metrics.InputValidationFailures.WithLabelValues(r.URL.Path, "request_body", "validation_failed").Inc()
					sm.auditEvent(SecurityEvent{
						Timestamp: time.Now(),
						EventType: "input_validation_failed",
						Severity:  "low",
						UserID:    authResult.UserID,
						IPAddress: sm.getClientIP(r),
						UserAgent: r.UserAgent(),
						Resource:  r.URL.Path,
						Action:    r.Method,
						Result:    "blocked",
						Details:   "Input validation failed",
					})
					http.Error(w, "Input validation failed", http.StatusBadRequest)
					return
				}
			}
			
			// Request size validation
			if r.ContentLength > sm.config.MaxRequestSize {
				sm.metrics.SecurityViolations.WithLabelValues("request_too_large", "medium").Inc()
				sm.auditEvent(SecurityEvent{
					Timestamp: time.Now(),
					EventType: "request_too_large",
					Severity:  "medium",
					UserID:    authResult.UserID,
					IPAddress: sm.getClientIP(r),
					UserAgent: r.UserAgent(),
					Resource:  r.URL.Path,
					Action:    r.Method,
					Result:    "blocked",
					Details:   fmt.Sprintf("Request size %d exceeds limit %d", r.ContentLength, sm.config.MaxRequestSize),
				})
				http.Error(w, "Request too large", http.StatusRequestEntityTooLarge)
				return
			}
			
			// Add authentication context
			ctx = context.WithValue(ctx, "auth", authResult)
			r = r.WithContext(ctx)
			
			// Process request
			next.ServeHTTP(w, r)
			
			// Record metrics
			duration := time.Since(start).Seconds()
			sm.metrics.RequestDuration.WithLabelValues(r.Method, r.URL.Path, authResult.AuthType).Observe(duration)
			sm.metrics.RequestsTotal.WithLabelValues(r.Method, r.URL.Path, "200", authResult.AuthType).Inc()
		})
	}
}

// addSecurityHeaders adds security headers to the response
func (sm *SecurityMiddleware) addSecurityHeaders(w http.ResponseWriter) {
	w.Header().Set("X-Content-Type-Options", "nosniff")
	w.Header().Set("X-Frame-Options", "DENY")
	w.Header().Set("X-XSS-Protection", "1; mode=block")
	w.Header().Set("Strict-Transport-Security", "max-age=31536000; includeSubDomains; preload")
	w.Header().Set("Referrer-Policy", "strict-origin-when-cross-origin")
	w.Header().Set("Content-Security-Policy", "default-src 'self'")
	w.Header().Set("Permissions-Policy", "geolocation=(), microphone=(), camera=()")
}

// rateLimit checks if the request should be rate limited
func (sm *SecurityMiddleware) rateLimit(r *http.Request) bool {
	clientIP := sm.getClientIP(r)
	return sm.rateLimiter.Allow(clientIP)
}

// validateMTLS validates mTLS connection
func (sm *SecurityMiddleware) validateMTLS(r *http.Request) bool {
	if r.TLS == nil {
		return false
	}
	
	// Check if client certificate is present
	if len(r.TLS.PeerCertificates) == 0 {
		return false
	}
	
	// Validate client certificate
	cert := r.TLS.PeerCertificates[0]
	if cert.NotAfter.Before(time.Now()) {
		return false
	}
	
	// Additional certificate validation can be added here
	return true
}

// AuthResult represents authentication result
type AuthResult struct {
	Success  bool   `json:"success"`
	AuthType string `json:"auth_type"`
	UserID   string `json:"user_id"`
	Roles    []string `json:"roles"`
	Reason   string `json:"reason"`
}

// authenticate validates authentication
func (sm *SecurityMiddleware) authenticate(r *http.Request) AuthResult {
	// Check JWT token
	if authHeader := r.Header.Get("Authorization"); authHeader != "" {
		if strings.HasPrefix(authHeader, "Bearer ") {
			token := strings.TrimPrefix(authHeader, "Bearer ")
			if sm.validateJWT(token) {
				return AuthResult{
					Success:  true,
					AuthType: "jwt",
					UserID:   sm.extractUserIDFromJWT(token),
					Roles:    sm.extractRolesFromJWT(token),
				}
			}
		}
	}
	
	// Check API key
	if apiKey := r.Header.Get("X-API-Key"); apiKey != "" {
		if sm.validateAPIKey(apiKey) {
			return AuthResult{
				Success:  true,
				AuthType: "api_key",
				UserID:   sm.extractUserIDFromAPIKey(apiKey),
				Roles:    []string{"api_user"},
			}
		}
	}
	
	// Check mTLS certificate
	if r.TLS != nil && len(r.TLS.PeerCertificates) > 0 {
		cert := r.TLS.PeerCertificates[0]
		if sm.validateClientCert(cert) {
			return AuthResult{
				Success:  true,
				AuthType: "mtls",
				UserID:   cert.Subject.CommonName,
				Roles:    []string{"cert_user"},
			}
		}
	}
	
	return AuthResult{
		Success:  false,
		AuthType: "none",
		Reason:   "No valid authentication found",
	}
}

// authorize validates authorization
func (sm *SecurityMiddleware) authorize(r *http.Request, auth AuthResult) bool {
	// Implement RBAC authorization logic here
	// This would typically involve checking permissions against a policy engine
	
	// For now, return true for authenticated users
	return auth.Success
}

// validateInput validates request input
func (sm *SecurityMiddleware) validateInput(r *http.Request) bool {
	// Implement input validation logic here
	// This would typically involve JSON schema validation
	
	// For now, return true
	return true
}

// validateJWT validates JWT token
func (sm *SecurityMiddleware) validateJWT(token string) bool {
	// Implement JWT validation logic here
	// This would typically involve signature verification and expiration checks
	
	// For now, return true for non-empty tokens
	return token != ""
}

// validateAPIKey validates API key
func (sm *SecurityMiddleware) validateAPIKey(apiKey string) bool {
	// Implement API key validation logic here
	// This would typically involve checking against a database or cache
	
	// For now, return true for non-empty keys
	return apiKey != ""
}

// validateClientCert validates client certificate
func (sm *SecurityMiddleware) validateClientCert(cert *tls.Certificate) bool {
	// Implement client certificate validation logic here
	// This would typically involve checking against a CA and validating the certificate chain
	
	// For now, return true for valid certificates
	return cert != nil
}

// extractUserIDFromJWT extracts user ID from JWT token
func (sm *SecurityMiddleware) extractUserIDFromJWT(token string) string {
	// Implement JWT parsing logic here
	// This would typically involve decoding the JWT and extracting the user ID
	
	// For now, return a placeholder
	return "user_from_jwt"
}

// extractRolesFromJWT extracts roles from JWT token
func (sm *SecurityMiddleware) extractRolesFromJWT(token string) []string {
	// Implement JWT parsing logic here
	// This would typically involve decoding the JWT and extracting the roles
	
	// For now, return a placeholder
	return []string{"user"}
}

// extractUserIDFromAPIKey extracts user ID from API key
func (sm *SecurityMiddleware) extractUserIDFromAPIKey(apiKey string) string {
	// Implement API key parsing logic here
	// This would typically involve looking up the API key in a database
	
	// For now, return a placeholder
	return "user_from_api_key"
}

// getClientIP extracts client IP from request
func (sm *SecurityMiddleware) getClientIP(r *http.Request) string {
	// Check for forwarded headers
	if xff := r.Header.Get("X-Forwarded-For"); xff != "" {
		return strings.Split(xff, ",")[0]
	}
	if xri := r.Header.Get("X-Real-IP"); xri != "" {
		return xri
	}
	
	// Fall back to remote address
	ip, _, err := net.SplitHostPort(r.RemoteAddr)
	if err != nil {
		return r.RemoteAddr
	}
	return ip
}

// auditEvent logs a security event
func (sm *SecurityMiddleware) auditEvent(event SecurityEvent) {
	if sm.config.AuditEnabled {
		select {
		case sm.auditor.events <- event:
		default:
			// Channel is full, drop event
		}
	}
}

// Allow checks if a request should be allowed based on rate limiting
func (rl *RateLimiter) Allow(clientIP string) bool {
	bucket, exists := rl.buckets[clientIP]
	if !exists {
		bucket = &TokenBucket{
			capacity:   10, // Default capacity
			tokens:     10,
			lastRefill: time.Now(),
			refillRate: time.Second,
		}
		rl.buckets[clientIP] = bucket
	}
	
	return bucket.consume()
}

// consume attempts to consume a token from the bucket
func (tb *TokenBucket) consume() bool {
	now := time.Now()
	
	// Refill tokens based on time elapsed
	elapsed := now.Sub(tb.lastRefill)
	tokensToAdd := int(elapsed / tb.refillRate)
	
	if tokensToAdd > 0 {
		tb.tokens = min(tb.capacity, tb.tokens+tokensToAdd)
		tb.lastRefill = now
	}
	
	// Check if we have tokens available
	if tb.tokens > 0 {
		tb.tokens--
		return true
	}
	
	return false
}

// min returns the minimum of two integers
func min(a, b int) int {
	if a < b {
		return a
	}
	return b
}

// StartAuditProcessor starts the security audit processor
func (sm *SecurityMiddleware) StartAuditProcessor() {
	go func() {
		for event := range sm.auditor.events {
			// Process security event
			sm.processSecurityEvent(event)
		}
	}()
}

// processSecurityEvent processes a security event
func (sm *SecurityMiddleware) processSecurityEvent(event SecurityEvent) {
	// Log the event
	eventJSON, _ := json.Marshal(event)
	sm.auditor.logger.Info("Security event", "event", string(eventJSON))
	
	// Update metrics
	sm.metrics.SecurityViolations.WithLabelValues(event.EventType, event.Severity).Inc()
	
	// Additional processing can be added here (e.g., alerting, blocking)
}

// Close closes the security middleware
func (sm *SecurityMiddleware) Close() {
	close(sm.auditor.events)
}
