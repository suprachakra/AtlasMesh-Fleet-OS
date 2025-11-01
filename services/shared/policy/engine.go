package policy

import (
	"context"
	"encoding/json"
	"fmt"
	"net/http"
	"time"

	"github.com/open-policy-agent/opa/rego"
	"github.com/open-policy-agent/opa/storage"
	"github.com/open-policy-agent/opa/storage/inmem"
	"github.com/prometheus/client_golang/prometheus"
	"github.com/prometheus/client_golang/prometheus/promauto"
	"go.opentelemetry.io/otel"
	"go.opentelemetry.io/otel/trace"
)

// PolicyEngine provides comprehensive policy evaluation using OPA/Rego
type PolicyEngine struct {
	config     PolicyConfig
	store      storage.Store
	metrics    *PolicyMetrics
	tracer     trace.Tracer
	policies   map[string]*Policy
	compiler   *PolicyCompiler
}

// PolicyConfig holds policy engine configuration
type PolicyConfig struct {
	// OPA Configuration
	OPAAddress     string        `json:"opa_address"`
	OPATimeout     time.Duration `json:"opa_timeout"`
	OPARetryCount  int           `json:"opa_retry_count"`
	OPARetryDelay  time.Duration `json:"opa_retry_delay"`
	
	// Policy Configuration
	PolicyPath     string        `json:"policy_path"`
	DataPath       string        `json:"data_path"`
	BundlePath     string        `json:"bundle_path"`
	
	// Caching
	CacheEnabled   bool          `json:"cache_enabled"`
	CacheTTL       time.Duration `json:"cache_ttl"`
	CacheSize      int           `json:"cache_size"`
	
	// Monitoring
	MetricsEnabled bool          `json:"metrics_enabled"`
	LoggingEnabled bool          `json:"logging_enabled"`
}

// PolicyMetrics tracks policy-related metrics
type PolicyMetrics struct {
	EvaluationsTotal    *prometheus.CounterVec
	EvaluationDuration  *prometheus.HistogramVec
	PolicyViolations    *prometheus.CounterVec
	CacheHits           *prometheus.CounterVec
	CacheMisses         *prometheus.CounterVec
	PolicyLoads         *prometheus.CounterVec
	PolicyErrors        *prometheus.CounterVec
}

// Policy represents a policy definition
type Policy struct {
	ID          string            `json:"id"`
	Name        string            `json:"name"`
	Description string            `json:"description"`
	Version     string            `json:"version"`
	Rules       []PolicyRule      `json:"rules"`
	Metadata    map[string]interface{} `json:"metadata"`
	CreatedAt   time.Time         `json:"created_at"`
	UpdatedAt   time.Time         `json:"updated_at"`
}

// PolicyRule represents a single policy rule
type PolicyRule struct {
	ID          string            `json:"id"`
	Name        string            `json:"name"`
	Description string            `json:"description"`
	Resource    string            `json:"resource"`
	Action      string            `json:"action"`
	Effect      string            `json:"effect"`
	Conditions  []PolicyCondition `json:"conditions"`
	Priority    int               `json:"priority"`
}

// PolicyCondition represents a policy condition
type PolicyCondition struct {
	Field    string      `json:"field"`
	Operator string      `json:"operator"`
	Value    interface{} `json:"value"`
}

// PolicyCompiler compiles and optimizes policies
type PolicyCompiler struct {
	compiler *rego.Compiler
	store    storage.Store
}

// PolicyEvaluation represents a policy evaluation request
type PolicyEvaluation struct {
	PolicyID   string                 `json:"policy_id"`
	Resource   string                 `json:"resource"`
	Action     string                 `json:"action"`
	Subject    string                 `json:"subject"`
	Context    map[string]interface{} `json:"context"`
	Input      map[string]interface{} `json:"input"`
	Timestamp  time.Time              `json:"timestamp"`
}

// PolicyResult represents a policy evaluation result
type PolicyResult struct {
	Allowed    bool                   `json:"allowed"`
	Reason     string                 `json:"reason"`
	Violations []PolicyViolation     `json:"violations"`
	Metadata   map[string]interface{} `json:"metadata"`
	Timestamp  time.Time              `json:"timestamp"`
}

// PolicyViolation represents a policy violation
type PolicyViolation struct {
	RuleID     string `json:"rule_id"`
	RuleName   string `json:"rule_name"`
	Resource   string `json:"resource"`
	Action     string `json:"action"`
	Reason     string `json:"reason"`
	Severity   string `json:"severity"`
	Timestamp  time.Time `json:"timestamp"`
}

// NewPolicyEngine creates a new policy engine
func NewPolicyEngine(config PolicyConfig) (*PolicyEngine, error) {
	// Initialize storage
	store := inmem.New()
	
	// Initialize metrics
	metrics := &PolicyMetrics{
		EvaluationsTotal: promauto.NewCounterVec(
			prometheus.CounterOpts{
				Name: "policy_evaluations_total",
				Help: "Total number of policy evaluations",
			},
			[]string{"policy_id", "resource", "action", "result"},
		),
		EvaluationDuration: promauto.NewHistogramVec(
			prometheus.HistogramOpts{
				Name: "policy_evaluation_duration_seconds",
				Help: "Policy evaluation duration in seconds",
				Buckets: []float64{0.001, 0.005, 0.01, 0.025, 0.05, 0.1, 0.25, 0.5, 1.0, 2.5, 5.0, 10.0},
			},
			[]string{"policy_id", "resource", "action"},
		),
		PolicyViolations: promauto.NewCounterVec(
			prometheus.CounterOpts{
				Name: "policy_violations_total",
				Help: "Total number of policy violations",
			},
			[]string{"policy_id", "rule_id", "severity"},
		),
		CacheHits: promauto.NewCounterVec(
			prometheus.CounterOpts{
				Name: "policy_cache_hits_total",
				Help: "Total number of policy cache hits",
			},
			[]string{"policy_id"},
		),
		CacheMisses: promauto.NewCounterVec(
			prometheus.CounterOpts{
				Name: "policy_cache_misses_total",
				Help: "Total number of policy cache misses",
			},
			[]string{"policy_id"},
		),
		PolicyLoads: promauto.NewCounterVec(
			prometheus.CounterOpts{
				Name: "policy_loads_total",
				Help: "Total number of policy loads",
			},
			[]string{"policy_id", "status"},
		),
		PolicyErrors: promauto.NewCounterVec(
			prometheus.CounterOpts{
				Name: "policy_errors_total",
				Help: "Total number of policy errors",
			},
			[]string{"error_type", "policy_id"},
		),
	}
	
	// Initialize compiler
	compiler := &PolicyCompiler{
		store: store,
	}
	
	return &PolicyEngine{
		config:   config,
		store:    store,
		metrics:  metrics,
		tracer:   otel.Tracer("policy-engine"),
		policies: make(map[string]*Policy),
		compiler: compiler,
	}, nil
}

// LoadPolicy loads a policy from file or URL
func (pe *PolicyEngine) LoadPolicy(ctx context.Context, policyID, source string) error {
	ctx, span := pe.tracer.Start(ctx, "policy-load")
	defer span.End()
	
	start := time.Now()
	defer func() {
		pe.metrics.PolicyLoads.WithLabelValues(policyID, "success").Inc()
	}()
	
	// Load policy content
	content, err := pe.loadPolicyContent(source)
	if err != nil {
		pe.metrics.PolicyLoads.WithLabelValues(policyID, "error").Inc()
		pe.metrics.PolicyErrors.WithLabelValues("load_failed", policyID).Inc()
		return fmt.Errorf("failed to load policy content: %w", err)
	}
	
	// Parse policy
	policy, err := pe.parsePolicy(policyID, content)
	if err != nil {
		pe.metrics.PolicyLoads.WithLabelValues(policyID, "error").Inc()
		pe.metrics.PolicyErrors.WithLabelValues("parse_failed", policyID).Inc()
		return fmt.Errorf("failed to parse policy: %w", err)
	}
	
	// Compile policy
	err = pe.compiler.CompilePolicy(policy)
	if err != nil {
		pe.metrics.PolicyLoads.WithLabelValues(policyID, "error").Inc()
		pe.metrics.PolicyErrors.WithLabelValues("compile_failed", policyID).Inc()
		return fmt.Errorf("failed to compile policy: %w", err)
	}
	
	// Store policy
	pe.policies[policyID] = policy
	
	// Update metrics
	pe.metrics.PolicyLoads.WithLabelValues(policyID, "success").Inc()
	
	return nil
}

// EvaluatePolicy evaluates a policy against a request
func (pe *PolicyEngine) EvaluatePolicy(ctx context.Context, evaluation PolicyEvaluation) (*PolicyResult, error) {
	ctx, span := pe.tracer.Start(ctx, "policy-evaluate")
	defer span.End()
	
	start := time.Now()
	defer func() {
		pe.metrics.EvaluationDuration.WithLabelValues(evaluation.PolicyID, evaluation.Resource, evaluation.Action).Observe(time.Since(start).Seconds())
	}()
	
	// Get policy
	policy, exists := pe.policies[evaluation.PolicyID]
	if !exists {
		pe.metrics.PolicyErrors.WithLabelValues("policy_not_found", evaluation.PolicyID).Inc()
		return nil, fmt.Errorf("policy not found: %s", evaluation.PolicyID)
	}
	
	// Check cache if enabled
	if pe.config.CacheEnabled {
		if cached := pe.getCachedResult(evaluation); cached != nil {
			pe.metrics.CacheHits.WithLabelValues(evaluation.PolicyID).Inc()
			return cached, nil
		}
		pe.metrics.CacheMisses.WithLabelValues(evaluation.PolicyID).Inc()
	}
	
	// Evaluate policy
	result, err := pe.evaluatePolicyRules(ctx, policy, evaluation)
	if err != nil {
		pe.metrics.PolicyErrors.WithLabelValues("evaluation_failed", evaluation.PolicyID).Inc()
		return nil, fmt.Errorf("policy evaluation failed: %w", err)
	}
	
	// Cache result if enabled
	if pe.config.CacheEnabled {
		pe.setCachedResult(evaluation, result)
	}
	
	// Update metrics
	if result.Allowed {
		pe.metrics.EvaluationsTotal.WithLabelValues(evaluation.PolicyID, evaluation.Resource, evaluation.Action, "allowed").Inc()
	} else {
		pe.metrics.EvaluationsTotal.WithLabelValues(evaluation.PolicyID, evaluation.Resource, evaluation.Action, "denied").Inc()
	}
	
	// Record violations
	for _, violation := range result.Violations {
		pe.metrics.PolicyViolations.WithLabelValues(evaluation.PolicyID, violation.RuleID, violation.Severity).Inc()
	}
	
	return result, nil
}

// evaluatePolicyRules evaluates all rules in a policy
func (pe *PolicyEngine) evaluatePolicyRules(ctx context.Context, policy *Policy, evaluation PolicyEvaluation) (*PolicyResult, error) {
	var violations []PolicyViolation
	allowed := true
	
	// Sort rules by priority
	rules := pe.sortRulesByPriority(policy.Rules)
	
	// Evaluate each rule
	for _, rule := range rules {
		// Check if rule applies to this resource and action
		if !pe.ruleApplies(rule, evaluation) {
			continue
		}
		
		// Evaluate rule conditions
		ruleResult, err := pe.evaluateRule(ctx, rule, evaluation)
		if err != nil {
			return nil, fmt.Errorf("failed to evaluate rule %s: %w", rule.ID, err)
		}
		
		// Check if rule is violated
		if !ruleResult.Allowed {
			violation := PolicyViolation{
				RuleID:    rule.ID,
				RuleName:  rule.Name,
				Resource:  evaluation.Resource,
				Action:    evaluation.Action,
				Reason:    ruleResult.Reason,
				Severity:  pe.getRuleSeverity(rule),
				Timestamp: time.Now(),
			}
			violations = append(violations, violation)
			
			// If rule has "deny" effect, mark as not allowed
			if rule.Effect == "deny" {
				allowed = false
			}
		}
	}
	
	// Determine final result
	if len(violations) > 0 && !allowed {
		return &PolicyResult{
			Allowed:    false,
			Reason:     "Policy violations detected",
			Violations: violations,
			Metadata:   map[string]interface{}{"policy_id": policy.ID, "policy_version": policy.Version},
			Timestamp:  time.Now(),
		}, nil
	}
	
	return &PolicyResult{
		Allowed:    true,
		Reason:     "Policy evaluation passed",
		Violations: violations,
		Metadata:   map[string]interface{}{"policy_id": policy.ID, "policy_version": policy.Version},
		Timestamp:  time.Now(),
	}, nil
}

// evaluateRule evaluates a single policy rule
func (pe *PolicyEngine) evaluateRule(ctx context.Context, rule PolicyRule, evaluation PolicyEvaluation) (*PolicyResult, error) {
	// Check all conditions
	for _, condition := range rule.Conditions {
		if !pe.evaluateCondition(condition, evaluation) {
			return &PolicyResult{
				Allowed: false,
				Reason:  fmt.Sprintf("Condition failed: %s %s %v", condition.Field, condition.Operator, condition.Value),
			}, nil
		}
	}
	
	return &PolicyResult{
		Allowed: true,
		Reason:  "All conditions satisfied",
	}, nil
}

// evaluateCondition evaluates a single condition
func (pe *PolicyEngine) evaluateCondition(condition PolicyCondition, evaluation PolicyEvaluation) bool {
	// Get field value from context or input
	fieldValue := pe.getFieldValue(condition.Field, evaluation)
	
	// Apply operator
	switch condition.Operator {
	case "equals":
		return fieldValue == condition.Value
	case "not_equals":
		return fieldValue != condition.Value
	case "contains":
		if str, ok := fieldValue.(string); ok {
			if val, ok := condition.Value.(string); ok {
				return contains(str, val)
			}
		}
		return false
	case "not_contains":
		if str, ok := fieldValue.(string); ok {
			if val, ok := condition.Value.(string); ok {
				return !contains(str, val)
			}
		}
		return false
	case "in":
		if arr, ok := condition.Value.([]interface{}); ok {
			for _, val := range arr {
				if fieldValue == val {
					return true
				}
			}
		}
		return false
	case "not_in":
		if arr, ok := condition.Value.([]interface{}); ok {
			for _, val := range arr {
				if fieldValue == val {
					return false
				}
			}
		}
		return true
	case "greater_than":
		if num, ok := fieldValue.(float64); ok {
			if val, ok := condition.Value.(float64); ok {
				return num > val
			}
		}
		return false
	case "less_than":
		if num, ok := fieldValue.(float64); ok {
			if val, ok := condition.Value.(float64); ok {
				return num < val
			}
		}
		return false
	case "greater_than_or_equal":
		if num, ok := fieldValue.(float64); ok {
			if val, ok := condition.Value.(float64); ok {
				return num >= val
			}
		}
		return false
	case "less_than_or_equal":
		if num, ok := fieldValue.(float64); ok {
			if val, ok := condition.Value.(float64); ok {
				return num <= val
			}
		}
		return false
	case "regex":
		if str, ok := fieldValue.(string); ok {
			if pattern, ok := condition.Value.(string); ok {
				return matchesRegex(str, pattern)
			}
		}
		return false
	case "exists":
		return fieldValue != nil
	case "not_exists":
		return fieldValue == nil
	default:
		return false
	}
}

// getFieldValue retrieves a field value from context or input
func (pe *PolicyEngine) getFieldValue(field string, evaluation PolicyEvaluation) interface{} {
	// Check context first
	if value, exists := evaluation.Context[field]; exists {
		return value
	}
	
	// Check input
	if value, exists := evaluation.Input[field]; exists {
		return value
	}
	
	// Check built-in fields
	switch field {
	case "subject":
		return evaluation.Subject
	case "resource":
		return evaluation.Resource
	case "action":
		return evaluation.Action
	case "timestamp":
		return evaluation.Timestamp
	default:
		return nil
	}
}

// ruleApplies checks if a rule applies to the given resource and action
func (pe *PolicyEngine) ruleApplies(rule PolicyRule, evaluation PolicyEvaluation) bool {
	// Check resource pattern
	if !pe.matchesPattern(rule.Resource, evaluation.Resource) {
		return false
	}
	
	// Check action pattern
	if !pe.matchesPattern(rule.Action, evaluation.Action) {
		return false
	}
	
	return true
}

// matchesPattern checks if a value matches a pattern
func (pe *PolicyEngine) matchesPattern(pattern, value string) bool {
	if pattern == "*" {
		return true
	}
	
	if pattern == value {
		return true
	}
	
	// Simple wildcard matching
	if contains(pattern, "*") {
		return matchesWildcard(pattern, value)
	}
	
	return false
}

// sortRulesByPriority sorts rules by priority (higher priority first)
func (pe *PolicyEngine) sortRulesByPriority(rules []PolicyRule) []PolicyRule {
	// Simple bubble sort for demonstration
	// In production, use a more efficient sorting algorithm
	sorted := make([]PolicyRule, len(rules))
	copy(sorted, rules)
	
	for i := 0; i < len(sorted); i++ {
		for j := i + 1; j < len(sorted); j++ {
			if sorted[i].Priority < sorted[j].Priority {
				sorted[i], sorted[j] = sorted[j], sorted[i]
			}
		}
	}
	
	return sorted
}

// getRuleSeverity returns the severity level for a rule
func (pe *PolicyEngine) getRuleSeverity(rule PolicyRule) string {
	if severity, exists := rule.Metadata["severity"]; exists {
		if sev, ok := severity.(string); ok {
			return sev
		}
	}
	return "medium"
}

// loadPolicyContent loads policy content from source
func (pe *PolicyEngine) loadPolicyContent(source string) ([]byte, error) {
	// Implementation would load from file, URL, or database
	// For now, return empty content
	return []byte{}, nil
}

// parsePolicy parses policy content
func (pe *PolicyEngine) parsePolicy(policyID string, content []byte) (*Policy, error) {
	// Implementation would parse JSON, YAML, or Rego content
	// For now, return a sample policy
	return &Policy{
		ID:          policyID,
		Name:        "Sample Policy",
		Description: "A sample policy for demonstration",
		Version:     "1.0.0",
		Rules:       []PolicyRule{},
		Metadata:    make(map[string]interface{}),
		CreatedAt:   time.Now(),
		UpdatedAt:   time.Now(),
	}, nil
}

// getCachedResult retrieves a cached evaluation result
func (pe *PolicyEngine) getCachedResult(evaluation PolicyEvaluation) *PolicyResult {
	// Implementation would check cache
	return nil
}

// setCachedResult stores a cached evaluation result
func (pe *PolicyEngine) setCachedResult(evaluation PolicyEvaluation, result *PolicyResult) {
	// Implementation would store in cache
}

// CompilePolicy compiles a policy for efficient evaluation
func (pc *PolicyCompiler) CompilePolicy(policy *Policy) error {
	// Implementation would compile policy using OPA
	return nil
}

// Helper functions
func contains(s, substr string) bool {
	return len(s) >= len(substr) && s[:len(substr)] == substr
}

func matchesWildcard(pattern, value string) bool {
	// Simple wildcard matching implementation
	// In production, use a proper regex engine
	return false
}

func matchesRegex(text, pattern string) bool {
	// Implementation would use regex matching
	return false
}

// Close closes the policy engine
func (pe *PolicyEngine) Close() {
	// Cleanup resources
}
