package resilience

import (
	"context"
	"fmt"
	"math"
	"sync"
	"time"

	"github.com/prometheus/client_golang/prometheus"
	"github.com/prometheus/client_golang/prometheus/promauto"
	"go.opentelemetry.io/otel"
	"go.opentelemetry.io/otel/trace"
)

// ResiliencePatterns provides comprehensive resilience patterns for microservices
type ResiliencePatterns struct {
	config     ResilienceConfig
	metrics    *ResilienceMetrics
	tracer     trace.Tracer
	circuitBreakers map[string]*CircuitBreaker
	mutex      sync.RWMutex
}

// ResilienceConfig holds resilience configuration
type ResilienceConfig struct {
	// Circuit Breaker
	CircuitBreakerEnabled bool          `json:"circuit_breaker_enabled"`
	FailureThreshold      int           `json:"failure_threshold"`
	RecoveryTimeout       time.Duration `json:"recovery_timeout"`
	HalfOpenMaxCalls      int           `json:"half_open_max_calls"`
	
	// Retry
	RetryEnabled          bool          `json:"retry_enabled"`
	MaxRetries            int           `json:"max_retries"`
	RetryDelay            time.Duration `json:"retry_delay"`
	MaxRetryDelay         time.Duration `json:"max_retry_delay"`
	RetryMultiplier       float64       `json:"retry_multiplier"`
	
	// Timeout
	TimeoutEnabled        bool          `json:"timeout_enabled"`
	DefaultTimeout        time.Duration `json:"default_timeout"`
	MaxTimeout            time.Duration `json:"max_timeout"`
	
	// Bulkhead
	BulkheadEnabled       bool          `json:"bulkhead_enabled"`
	MaxConcurrency        int           `json:"max_concurrency"`
	MaxQueueSize          int           `json:"max_queue_size"`
	
	// Rate Limiting
	RateLimitEnabled      bool          `json:"rate_limit_enabled"`
	RateLimitRPS          int           `json:"rate_limit_rps"`
	RateLimitBurst        int           `json:"rate_limit_burst"`
	
	// Monitoring
	MetricsEnabled        bool          `json:"metrics_enabled"`
	LoggingEnabled        bool          `json:"logging_enabled"`
}

// ResilienceMetrics tracks resilience-related metrics
type ResilienceMetrics struct {
	CircuitBreakerState    *prometheus.GaugeVec
	CircuitBreakerFailures *prometheus.CounterVec
	RetryAttempts          *prometheus.CounterVec
	RetrySuccess           *prometheus.CounterVec
	Timeouts               *prometheus.CounterVec
	BulkheadRejections     *prometheus.CounterVec
	RateLimitHits          *prometheus.CounterVec
	ErrorCount             *prometheus.CounterVec
	SuccessCount           *prometheus.CounterVec
	Latency                *prometheus.HistogramVec
}

// CircuitBreaker implements the circuit breaker pattern
type CircuitBreaker struct {
	name            string
	state           CircuitBreakerState
	failureCount    int
	lastFailureTime time.Time
	config          CircuitBreakerConfig
	mutex           sync.RWMutex
	metrics         *ResilienceMetrics
}

// CircuitBreakerState represents the state of a circuit breaker
type CircuitBreakerState int

const (
	StateClosed CircuitBreakerState = iota
	StateOpen
	StateHalfOpen
)

// CircuitBreakerConfig holds circuit breaker configuration
type CircuitBreakerConfig struct {
	FailureThreshold int           `json:"failure_threshold"`
	RecoveryTimeout  time.Duration `json:"recovery_timeout"`
	HalfOpenMaxCalls int           `json:"half_open_max_calls"`
}

// RetryConfig holds retry configuration
type RetryConfig struct {
	MaxRetries      int           `json:"max_retries"`
	RetryDelay      time.Duration `json:"retry_delay"`
	MaxRetryDelay   time.Duration `json:"max_retry_delay"`
	RetryMultiplier float64       `json:"retry_multiplier"`
}

// TimeoutConfig holds timeout configuration
type TimeoutConfig struct {
	Timeout    time.Duration `json:"timeout"`
	MaxTimeout time.Duration `json:"max_timeout"`
}

// BulkheadConfig holds bulkhead configuration
type BulkheadConfig struct {
	MaxConcurrency int `json:"max_concurrency"`
	MaxQueueSize   int `json:"max_queue_size"`
}

// RateLimitConfig holds rate limit configuration
type RateLimitConfig struct {
	RPS   int `json:"rps"`
	Burst int `json:"burst"`
}

// NewResiliencePatterns creates a new resilience patterns instance
func NewResiliencePatterns(config ResilienceConfig) *ResiliencePatterns {
	metrics := &ResilienceMetrics{
		CircuitBreakerState: promauto.NewGaugeVec(
			prometheus.GaugeOpts{
				Name: "resilience_circuit_breaker_state",
				Help: "Circuit breaker state (0=closed, 1=open, 2=half-open)",
			},
			[]string{"name"},
		),
		CircuitBreakerFailures: promauto.NewCounterVec(
			prometheus.CounterOpts{
				Name: "resilience_circuit_breaker_failures_total",
				Help: "Total number of circuit breaker failures",
			},
			[]string{"name", "error_type"},
		),
		RetryAttempts: promauto.NewCounterVec(
			prometheus.CounterOpts{
				Name: "resilience_retry_attempts_total",
				Help: "Total number of retry attempts",
			},
			[]string{"name", "attempt"},
		),
		RetrySuccess: promauto.NewCounterVec(
			prometheus.CounterOpts{
				Name: "resilience_retry_success_total",
				Help: "Total number of successful retries",
			},
			[]string{"name"},
		),
		Timeouts: promauto.NewCounterVec(
			prometheus.CounterOpts{
				Name: "resilience_timeouts_total",
				Help: "Total number of timeouts",
			},
			[]string{"name", "timeout_type"},
		),
		BulkheadRejections: promauto.NewCounterVec(
			prometheus.CounterOpts{
				Name: "resilience_bulkhead_rejections_total",
				Help: "Total number of bulkhead rejections",
			},
			[]string{"name"},
		),
		RateLimitHits: promauto.NewCounterVec(
			prometheus.CounterOpts{
				Name: "resilience_rate_limit_hits_total",
				Help: "Total number of rate limit hits",
			},
			[]string{"name"},
		),
		ErrorCount: promauto.NewCounterVec(
			prometheus.CounterOpts{
				Name: "resilience_errors_total",
				Help: "Total number of errors",
			},
			[]string{"name", "error_type"},
		),
		SuccessCount: promauto.NewCounterVec(
			prometheus.CounterOpts{
				Name: "resilience_success_total",
				Help: "Total number of successful operations",
			},
			[]string{"name"},
		),
		Latency: promauto.NewHistogramVec(
			prometheus.HistogramOpts{
				Name: "resilience_latency_seconds",
				Help: "Operation latency in seconds",
				Buckets: []float64{0.001, 0.005, 0.01, 0.025, 0.05, 0.1, 0.25, 0.5, 1.0, 2.5, 5.0, 10.0},
			},
			[]string{"name", "operation"},
		),
	}
	
	return &ResiliencePatterns{
		config:           config,
		metrics:          metrics,
		tracer:           otel.Tracer("resilience-patterns"),
		circuitBreakers:  make(map[string]*CircuitBreaker),
	}
}

// CircuitBreakerPattern implements the circuit breaker pattern
func (rp *ResiliencePatterns) CircuitBreakerPattern(name string, config CircuitBreakerConfig) *CircuitBreaker {
	rp.mutex.Lock()
	defer rp.mutex.Unlock()
	
	if cb, exists := rp.circuitBreakers[name]; exists {
		return cb
	}
	
	cb := &CircuitBreaker{
		name:    name,
		state:   StateClosed,
		config:  config,
		metrics: rp.metrics,
	}
	
	rp.circuitBreakers[name] = cb
	return cb
}

// ExecuteWithCircuitBreaker executes a function with circuit breaker protection
func (cb *CircuitBreaker) ExecuteWithCircuitBreaker(ctx context.Context, fn func() error) error {
	cb.mutex.Lock()
	defer cb.mutex.Unlock()
	
	// Check if circuit breaker is open
	if cb.state == StateOpen {
		if time.Since(cb.lastFailureTime) > cb.config.RecoveryTimeout {
			cb.state = StateHalfOpen
			cb.metrics.CircuitBreakerState.WithLabelValues(cb.name).Set(float64(StateHalfOpen))
		} else {
			cb.metrics.CircuitBreakerFailures.WithLabelValues(cb.name, "circuit_open").Inc()
			return fmt.Errorf("circuit breaker is open")
		}
	}
	
	// Check if circuit breaker is half-open and max calls reached
	if cb.state == StateHalfOpen && cb.failureCount >= cb.config.HalfOpenMaxCalls {
		cb.metrics.CircuitBreakerFailures.WithLabelValues(cb.name, "half_open_max_calls").Inc()
		return fmt.Errorf("circuit breaker half-open max calls reached")
	}
	
	// Execute function
	err := fn()
	
	if err != nil {
		cb.failureCount++
		cb.lastFailureTime = time.Now()
		
		// Check if failure threshold is reached
		if cb.failureCount >= cb.config.FailureThreshold {
			cb.state = StateOpen
			cb.metrics.CircuitBreakerState.WithLabelValues(cb.name).Set(float64(StateOpen))
		}
		
		cb.metrics.CircuitBreakerFailures.WithLabelValues(cb.name, "execution_failed").Inc()
		return err
	}
	
	// Reset failure count on success
	cb.failureCount = 0
	cb.state = StateClosed
	cb.metrics.CircuitBreakerState.WithLabelValues(cb.name).Set(float64(StateClosed))
	
	return nil
}

// RetryPattern implements the retry pattern
func (rp *ResiliencePatterns) RetryPattern(name string, config RetryConfig) func(context.Context, func() error) error {
	return func(ctx context.Context, fn func() error) error {
		var lastErr error
		
		for attempt := 0; attempt <= config.MaxRetries; attempt++ {
			if attempt > 0 {
				// Calculate delay with exponential backoff
				delay := time.Duration(float64(config.RetryDelay) * math.Pow(config.RetryMultiplier, float64(attempt-1)))
				if delay > config.MaxRetryDelay {
					delay = config.MaxRetryDelay
				}
				
				// Wait before retry
				select {
				case <-ctx.Done():
					return ctx.Err()
				case <-time.After(delay):
				}
			}
			
			// Record retry attempt
			rp.metrics.RetryAttempts.WithLabelValues(name, fmt.Sprintf("%d", attempt)).Inc()
			
			// Execute function
			err := fn()
			if err == nil {
				if attempt > 0 {
					rp.metrics.RetrySuccess.WithLabelValues(name).Inc()
				}
				return nil
			}
			
			lastErr = err
		}
		
		return fmt.Errorf("retry failed after %d attempts: %w", config.MaxRetries, lastErr)
	}
}

// TimeoutPattern implements the timeout pattern
func (rp *ResiliencePatterns) TimeoutPattern(name string, config TimeoutConfig) func(context.Context, func() error) error {
	return func(ctx context.Context, fn func() error) error {
		timeout := config.Timeout
		if timeout > config.MaxTimeout {
			timeout = config.MaxTimeout
		}
		
		// Create context with timeout
		ctx, cancel := context.WithTimeout(ctx, timeout)
		defer cancel()
		
		// Execute function
		err := fn()
		if err != nil {
			if ctx.Err() == context.DeadlineExceeded {
				rp.metrics.Timeouts.WithLabelValues(name, "deadline_exceeded").Inc()
				return fmt.Errorf("operation timed out after %v", timeout)
			}
			return err
		}
		
		return nil
	}
}

// BulkheadPattern implements the bulkhead pattern
func (rp *ResiliencePatterns) BulkheadPattern(name string, config BulkheadConfig) func(context.Context, func() error) error {
	semaphore := make(chan struct{}, config.MaxConcurrency)
	queue := make(chan func() error, config.MaxQueueSize)
	
	// Start worker pool
	for i := 0; i < config.MaxConcurrency; i++ {
		go func() {
			for fn := range queue {
				func() {
					semaphore <- struct{}{}
					defer func() { <-semaphore }()
					fn()
				}()
			}
		}()
	}
	
	return func(ctx context.Context, fn func() error) error {
		select {
		case queue <- fn:
			return nil
		default:
			rp.metrics.BulkheadRejections.WithLabelValues(name).Inc()
			return fmt.Errorf("bulkhead queue is full")
		}
	}
}

// RateLimitPattern implements the rate limiting pattern
func (rp *ResiliencePatterns) RateLimitPattern(name string, config RateLimitConfig) func(context.Context, func() error) error {
	limiter := NewRateLimiter(config.RPS, config.Burst)
	
	return func(ctx context.Context, fn func() error) error {
		if !limiter.Allow() {
			rp.metrics.RateLimitHits.WithLabelValues(name).Inc()
			return fmt.Errorf("rate limit exceeded")
		}
		
		return fn()
	}
}

// CombinedPattern combines multiple resilience patterns
func (rp *ResiliencePatterns) CombinedPattern(name string, patterns ...func(context.Context, func() error) error) func(context.Context, func() error) error {
	return func(ctx context.Context, fn func() error) error {
		// Apply patterns in sequence
		for _, pattern := range patterns {
			if err := pattern(ctx, fn); err != nil {
				return err
			}
		}
		return nil
	}
}

// ExecuteWithResilience executes a function with comprehensive resilience patterns
func (rp *ResiliencePatterns) ExecuteWithResilience(ctx context.Context, name string, fn func() error) error {
	start := time.Now()
	defer func() {
		rp.metrics.Latency.WithLabelValues(name, "operation").Observe(time.Since(start).Seconds())
	}()
	
	// Get circuit breaker
	cb := rp.CircuitBreakerPattern(name, CircuitBreakerConfig{
		FailureThreshold: rp.config.FailureThreshold,
		RecoveryTimeout:  rp.config.RecoveryTimeout,
		HalfOpenMaxCalls: rp.config.HalfOpenMaxCalls,
	})
	
	// Execute with circuit breaker
	err := cb.ExecuteWithCircuitBreaker(ctx, fn)
	if err != nil {
		rp.metrics.ErrorCount.WithLabelValues(name, "execution_failed").Inc()
		return err
	}
	
	rp.metrics.SuccessCount.WithLabelValues(name).Inc()
	return nil
}

// RateLimiter implements token bucket rate limiting
type RateLimiter struct {
	tokens   chan struct{}
	refill   time.Duration
	lastRefill time.Time
	mutex    sync.Mutex
}

// NewRateLimiter creates a new rate limiter
func NewRateLimiter(rps, burst int) *RateLimiter {
	rl := &RateLimiter{
		tokens:   make(chan struct{}, burst),
		refill:   time.Second / time.Duration(rps),
		lastRefill: time.Now(),
	}
	
	// Fill initial tokens
	for i := 0; i < burst; i++ {
		rl.tokens <- struct{}{}
	}
	
	// Start refill goroutine
	go rl.refillTokens()
	
	return rl
}

// Allow checks if a request should be allowed
func (rl *RateLimiter) Allow() bool {
	select {
	case <-rl.tokens:
		return true
	default:
		return false
	}
}

// refillTokens refills tokens at the specified rate
func (rl *RateLimiter) refillTokens() {
	ticker := time.NewTicker(rl.refill)
	defer ticker.Stop()
	
	for range ticker.C {
		select {
		case rl.tokens <- struct{}{}:
		default:
			// Bucket is full
		}
	}
}

// HealthCheck performs a health check on resilience patterns
func (rp *ResiliencePatterns) HealthCheck() map[string]interface{} {
	health := map[string]interface{}{
		"status":    "healthy",
		"timestamp": time.Now(),
		"patterns":  make(map[string]interface{}),
	}
	
	rp.mutex.RLock()
	defer rp.mutex.RUnlock()
	
	for name, cb := range rp.circuitBreakers {
		cb.mutex.RLock()
		state := cb.state
		failureCount := cb.failureCount
		cb.mutex.RUnlock()
		
		health["patterns"].(map[string]interface{})[name] = map[string]interface{}{
			"state":         state,
			"failure_count": failureCount,
		}
	}
	
	return health
}

// GetMetrics returns current metrics
func (rp *ResiliencePatterns) GetMetrics() map[string]interface{} {
	return map[string]interface{}{
		"circuit_breakers": len(rp.circuitBreakers),
		"config":          rp.config,
		"timestamp":       time.Now(),
	}
}

// Close closes the resilience patterns
func (rp *ResiliencePatterns) Close() {
	// Cleanup resources
	rp.mutex.Lock()
	defer rp.mutex.Unlock()
	
	for _, cb := range rp.circuitBreakers {
		// Reset circuit breaker state
		cb.mutex.Lock()
		cb.state = StateClosed
		cb.failureCount = 0
		cb.mutex.Unlock()
	}
}
