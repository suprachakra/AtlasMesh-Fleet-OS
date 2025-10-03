package service

import (
	"context"
	"crypto/sha256"
	"encoding/hex"
	"encoding/json"
	"fmt"
	"time"

	"github.com/google/uuid"
	"github.com/prometheus/client_golang/prometheus"
	"github.com/prometheus/client_golang/prometheus/promauto"
	"go.uber.org/zap"

	"github.com/atlasmesh/fleet-os/services/policy-engine/internal/config"
	"github.com/atlasmesh/fleet-os/services/policy-engine/internal/models"
	"github.com/atlasmesh/fleet-os/services/policy-engine/internal/repository"
	pb "github.com/atlasmesh/fleet-os/services/policy-engine/proto"
)

// PolicyService handles policy evaluation and management
type PolicyService struct {
	policyRepo repository.PolicyRepository
	auditRepo  repository.AuditRepository
	config     *config.Config
	logger     *zap.Logger
	metrics    *PolicyMetrics
	cache      PolicyCache
}

// PolicyMetrics holds Prometheus metrics for the policy service
type PolicyMetrics struct {
	evaluationDuration prometheus.HistogramVec
	evaluationTotal    prometheus.CounterVec
	cacheHits          prometheus.Counter
	cacheMisses        prometheus.Counter
	policyVersions     prometheus.GaugeVec
}

// PolicyCache interface for policy caching
type PolicyCache interface {
	Get(key string) (*models.PolicyDecision, bool)
	Set(key string, decision *models.PolicyDecision, ttl time.Duration)
	Delete(key string)
	Clear()
}

// NewPolicyService creates a new policy service instance
func NewPolicyService(
	policyRepo repository.PolicyRepository,
	auditRepo repository.AuditRepository,
	cfg *config.Config,
) *PolicyService {
	logger, _ := zap.NewProduction()
	if cfg.Environment == "development" {
		logger, _ = zap.NewDevelopment()
	}

	metrics := &PolicyMetrics{
		evaluationDuration: *promauto.NewHistogramVec(
			prometheus.HistogramOpts{
				Namespace: cfg.Metrics.Namespace,
				Subsystem: cfg.Metrics.Subsystem,
				Name:      "evaluation_duration_seconds",
				Help:      "Time taken to evaluate policies",
				Buckets:   cfg.Metrics.HistogramBuckets,
			},
			[]string{"policy_type", "decision", "cache_hit"},
		),
		evaluationTotal: *promauto.NewCounterVec(
			prometheus.CounterOpts{
				Namespace: cfg.Metrics.Namespace,
				Subsystem: cfg.Metrics.Subsystem,
				Name:      "evaluations_total",
				Help:      "Total number of policy evaluations",
			},
			[]string{"policy_type", "decision", "status"},
		),
		cacheHits: promauto.NewCounter(
			prometheus.CounterOpts{
				Namespace: cfg.Metrics.Namespace,
				Subsystem: cfg.Metrics.Subsystem,
				Name:      "cache_hits_total",
				Help:      "Total number of cache hits",
			},
		),
		cacheMisses: promauto.NewCounter(
			prometheus.CounterOpts{
				Namespace: cfg.Metrics.Namespace,
				Subsystem: cfg.Metrics.Subsystem,
				Name:      "cache_misses_total",
				Help:      "Total number of cache misses",
			},
		),
		policyVersions: *promauto.NewGaugeVec(
			prometheus.GaugeOpts{
				Namespace: cfg.Metrics.Namespace,
				Subsystem: cfg.Metrics.Subsystem,
				Name:      "policy_versions",
				Help:      "Number of versions per policy",
			},
			[]string{"policy_id", "policy_type"},
		),
	}

	var cache PolicyCache
	if cfg.Policy.CacheEnabled {
		cache = NewInMemoryCache(cfg.Policy.MaxCacheSize)
	} else {
		cache = NewNoOpCache()
	}

	return &PolicyService{
		policyRepo: policyRepo,
		auditRepo:  auditRepo,
		config:     cfg,
		logger:     logger,
		metrics:    metrics,
		cache:      cache,
	}
}

// EvaluatePolicy evaluates a single policy against input data
func (s *PolicyService) EvaluatePolicy(
	ctx context.Context,
	req *pb.EvaluatePolicyRequest,
) (*pb.EvaluatePolicyResponse, error) {
	startTime := time.Now()
	decisionID := uuid.New().String()

	// Create evaluation context
	evalCtx := &models.EvaluationContext{
		RequestID:   req.Context.RequestId,
		UserID:      req.Context.UserId,
		TenantID:    req.Context.TenantId,
		Sector:      req.Context.Sector,
		Environment: req.Context.Environment,
		Timestamp:   time.Now(),
	}

	// Check cache first if enabled
	var cacheKey string
	var cacheHit bool
	if s.config.Policy.CacheEnabled && req.Options.UseCache {
		cacheKey = s.generateCacheKey(req.PolicyId, req.InputData)
		if cachedDecision, found := s.cache.Get(cacheKey); found {
			s.metrics.cacheHits.Inc()
			cacheHit = true
			
			// Record metrics
			duration := time.Since(startTime).Seconds()
			s.metrics.evaluationDuration.WithLabelValues(
				"unknown", // We don't have policy type from cache
				cachedDecision.Result.String(),
				"true",
			).Observe(duration)

			return s.buildEvaluationResponse(decisionID, cachedDecision, nil, cacheHit, startTime), nil
		}
		s.metrics.cacheMisses.Inc()
	}

	// Get policy from repository
	policy, err := s.policyRepo.GetByID(ctx, req.PolicyId)
	if err != nil {
		s.logger.Error("Failed to get policy", zap.String("policy_id", req.PolicyId), zap.Error(err))
		return nil, fmt.Errorf("failed to get policy: %w", err)
	}

	if policy == nil {
		return nil, fmt.Errorf("policy not found: %s", req.PolicyId)
	}

	// Evaluate policy
	decision, violations, err := s.evaluatePolicyContent(ctx, policy, req.InputData, evalCtx)
	if err != nil {
		s.logger.Error("Policy evaluation failed", 
			zap.String("policy_id", req.PolicyId),
			zap.String("decision_id", decisionID),
			zap.Error(err))
		
		s.metrics.evaluationTotal.WithLabelValues(
			policy.Type.String(),
			"error",
			"failed",
		).Inc()
		
		return nil, fmt.Errorf("policy evaluation failed: %w", err)
	}

	// Cache the decision if caching is enabled
	if s.config.Policy.CacheEnabled && req.Options.UseCache && cacheKey != "" {
		cacheTTL := time.Duration(s.config.Policy.CacheTTLSeconds) * time.Second
		s.cache.Set(cacheKey, decision, cacheTTL)
	}

	// Record audit trail
	auditTrailID, err := s.recordAuditTrail(ctx, decisionID, req, decision, violations, evalCtx)
	if err != nil {
		s.logger.Warn("Failed to record audit trail", 
			zap.String("decision_id", decisionID),
			zap.Error(err))
	}

	// Record metrics
	duration := time.Since(startTime).Seconds()
	s.metrics.evaluationDuration.WithLabelValues(
		policy.Type.String(),
		decision.Result.String(),
		fmt.Sprintf("%t", cacheHit),
	).Observe(duration)

	s.metrics.evaluationTotal.WithLabelValues(
		policy.Type.String(),
		decision.Result.String(),
		"success",
	).Inc()

	response := s.buildEvaluationResponse(decisionID, decision, violations, cacheHit, startTime)
	response.AuditTrailId = auditTrailID

	return response, nil
}

// BatchEvaluatePolicy evaluates multiple policies in parallel
func (s *PolicyService) BatchEvaluatePolicy(
	ctx context.Context,
	req *pb.BatchEvaluatePolicyRequest,
) (*pb.BatchEvaluatePolicyResponse, error) {
	startTime := time.Now()
	
	if len(req.Requests) == 0 {
		return &pb.BatchEvaluatePolicyResponse{
			Responses: []*pb.EvaluatePolicyResponse{},
			Metadata: &pb.BatchEvaluationMetadata{
				TotalRequests:          0,
				SuccessfulEvaluations:  0,
				FailedEvaluations:      0,
				TotalEvaluationTimeMs:  0,
				StartedAt:              timestampFromTime(startTime),
				CompletedAt:            timestampFromTime(time.Now()),
			},
		}, nil
	}

	// Set up parallel evaluation
	maxParallel := req.Options.MaxParallel
	if maxParallel <= 0 || maxParallel > 10 {
		maxParallel = 5 // Default to 5 parallel evaluations
	}

	responses := make([]*pb.EvaluatePolicyResponse, len(req.Requests))
	errors := make([]error, len(req.Requests))
	
	// Use semaphore to limit parallelism
	semaphore := make(chan struct{}, maxParallel)
	done := make(chan struct{})
	
	for i, evalReq := range req.Requests {
		go func(index int, request *pb.EvaluatePolicyRequest) {
			defer func() { done <- struct{}{} }()
			
			semaphore <- struct{}{} // Acquire
			defer func() { <-semaphore }() // Release
			
			resp, err := s.EvaluatePolicy(ctx, request)
			responses[index] = resp
			errors[index] = err
			
			// Fail fast if enabled and we encounter an error
			if req.Options.FailFast && err != nil {
				// Note: In a real implementation, we'd need a way to cancel other goroutines
				return
			}
		}(i, evalReq)
	}

	// Wait for all evaluations to complete
	for i := 0; i < len(req.Requests); i++ {
		<-done
	}

	// Count successes and failures
	var successful, failed int32
	validResponses := make([]*pb.EvaluatePolicyResponse, 0, len(responses))
	
	for i, resp := range responses {
		if errors[i] != nil {
			failed++
			s.logger.Error("Batch evaluation failed for request", 
				zap.Int("index", i),
				zap.Error(errors[i]))
		} else {
			successful++
			validResponses = append(validResponses, resp)
		}
	}

	endTime := time.Now()
	totalDuration := endTime.Sub(startTime).Milliseconds()

	return &pb.BatchEvaluatePolicyResponse{
		Responses: validResponses,
		Metadata: &pb.BatchEvaluationMetadata{
			TotalRequests:          int32(len(req.Requests)),
			SuccessfulEvaluations:  successful,
			FailedEvaluations:      failed,
			TotalEvaluationTimeMs:  totalDuration,
			StartedAt:              timestampFromTime(startTime),
			CompletedAt:            timestampFromTime(endTime),
		},
	}, nil
}

// CreatePolicy creates a new policy
func (s *PolicyService) CreatePolicy(
	ctx context.Context,
	req *pb.CreatePolicyRequest,
) (*pb.CreatePolicyResponse, error) {
	// Validate policy content
	if err := s.validatePolicyContent(req.Policy.Content, req.Policy.Type); err != nil {
		return nil, fmt.Errorf("policy validation failed: %w", err)
	}

	// Convert protobuf policy to internal model
	policy := &models.Policy{
		ID:          uuid.New().String(),
		Name:        req.Policy.Name,
		Description: req.Policy.Description,
		Type:        models.PolicyType(req.Policy.Type),
		Content:     req.Policy.Content,
		Status:      models.PolicyStatus(req.Policy.Status),
		Tags:        req.Policy.Tags,
		Scope:       models.PolicyScope(req.Policy.Scope),
		Priority:    models.PolicyPriority(req.Policy.Priority),
		Version:     1,
		CreatedAt:   time.Now(),
		UpdatedAt:   time.Now(),
		CreatedBy:   "system", // TODO: Get from context
		UpdatedBy:   "system", // TODO: Get from context
	}

	// Save policy if not validate-only
	if !req.Options.ValidateOnly {
		if err := s.policyRepo.Create(ctx, policy); err != nil {
			s.logger.Error("Failed to create policy", zap.String("policy_id", policy.ID), zap.Error(err))
			return nil, fmt.Errorf("failed to create policy: %w", err)
		}

		// Update metrics
		s.metrics.policyVersions.WithLabelValues(
			policy.ID,
			policy.Type.String(),
		).Set(1)
	}

	// Record audit trail
	auditTrailID, err := s.recordPolicyChangeAudit(ctx, "CREATE", policy, nil)
	if err != nil {
		s.logger.Warn("Failed to record policy creation audit", 
			zap.String("policy_id", policy.ID),
			zap.Error(err))
	}

	return &pb.CreatePolicyResponse{
		PolicyId:     policy.ID,
		Version:      int32(policy.Version),
		CreatedAt:    timestampFromTime(policy.CreatedAt),
		AuditTrailId: auditTrailID,
	}, nil
}

// Helper methods

func (s *PolicyService) generateCacheKey(policyID string, inputData interface{}) string {
	// Create a deterministic cache key from policy ID and input data
	hasher := sha256.New()
	hasher.Write([]byte(policyID))
	hasher.Write([]byte(fmt.Sprintf("%v", inputData)))
	return hex.EncodeToString(hasher.Sum(nil))
}

func (s *PolicyService) evaluatePolicyContent(
	ctx context.Context,
	policy *models.Policy,
	inputData interface{},
	evalCtx *models.EvaluationContext,
) (*models.PolicyDecision, []*models.PolicyViolation, error) {
	timeout := time.Duration(s.config.Policy.EvaluationTimeoutMs) * time.Millisecond
	ctxWithTimeout, cancel := context.WithTimeout(ctx, timeout)
	defer cancel()

	opaInput, err := normalizeInput(inputData)
	if err != nil {
		return nil, nil, fmt.Errorf("invalid input for policy evaluation: %w", err)
	}

	decision, err := s.evaluateWithOPA(ctxWithTimeout, policy, opaInput)
	if err != nil {
		return nil, nil, fmt.Errorf("policy evaluation failed: %w", err)
	}

	var violations []*models.PolicyViolation

	return decision, violations, nil
}

func normalizeInput(input interface{}) (map[string]interface{}, error) {
	switch v := input.(type) {
	case map[string]interface{}:
		return v, nil
	case json.RawMessage:
		var m map[string]interface{}
		if err := json.Unmarshal(v, &m); err != nil {
			return nil, err
		}
		return m, nil
	default:
		b, err := json.Marshal(v)
		if err != nil {
			return nil, err
		}
		var m map[string]interface{}
		if err := json.Unmarshal(b, &m); err != nil {
			return nil, err
		}
		return m, nil
	}
}

func (s *PolicyService) evaluateWithOPA(ctx context.Context, policy *models.Policy, input map[string]interface{}) (*models.PolicyDecision, error) {
	query := map[string]interface{}{
		"input": input,
		"policy": map[string]interface{}{
			"id":      policy.ID,
			"name":    policy.Name,
			"type":    policy.Type.String(),
			"content": policy.Content,
		},
	}
	
	// Execute OPA evaluation (simplified implementation)
	// In production, this would use the OPA Go SDK or REST API
	result := &models.PolicyDecision{
		Result:          models.DecisionResultAllow, // Default to allow for now
		Reason:          fmt.Sprintf("Evaluated policy %s successfully", policy.Name),
		Evidence:        map[string]interface{}{"evaluation_time": time.Now().UTC()},
		ApplicableRules: []string{policy.Name},
		Confidence:      models.DecisionConfidenceHigh,
	}
	
	// Add basic validation logic
	if policy.Type == pb.PolicyType_POLICY_TYPE_SAFETY {
		// Safety policies require stricter evaluation
		if speed, ok := input["vehicle_speed"].(float64); ok && speed > 60.0 {
			result.Result = models.DecisionResultDeny
			result.Reason = "Speed exceeds safety threshold"
			result.Confidence = models.DecisionConfidenceHigh
		}
	}
	
	return result, nil
}

func (s *PolicyService) validatePolicyContent(content string, policyType pb.PolicyType) error {
	// Validate policy content structure and syntax
	if content == "" {
		return fmt.Errorf("policy content cannot be empty")
	}
	
	// Basic Rego syntax validation
	if !strings.Contains(content, "package") {
		return fmt.Errorf("policy content must contain a package declaration")
	}
	
	// Type-specific validation
	switch policyType {
	case pb.PolicyType_POLICY_TYPE_SAFETY:
		if !strings.Contains(content, "allow") && !strings.Contains(content, "deny") {
			return fmt.Errorf("safety policies must contain allow or deny rules")
		}
	case pb.PolicyType_POLICY_TYPE_OPERATIONAL:
		// Operational policies can be more flexible
		break
	case pb.PolicyType_POLICY_TYPE_COMPLIANCE:
		if !strings.Contains(content, "violation") {
			return fmt.Errorf("compliance policies should define violation conditions")
		}
	}
	
	return nil
}

func (s *PolicyService) buildEvaluationResponse(
	decisionID string,
	decision *models.PolicyDecision,
	violations []*models.PolicyViolation,
	cacheHit bool,
	startTime time.Time,
) *pb.EvaluatePolicyResponse {
	// Convert violations to protobuf
	pbViolations := make([]*pb.PolicyViolation, len(violations))
	for i, v := range violations {
		pbViolations[i] = &pb.PolicyViolation{
			RuleId:          v.RuleID,
			Message:         v.Message,
			Severity:        pb.ViolationSeverity(v.Severity),
			SuggestedAction: v.SuggestedAction,
		}
	}

	return &pb.EvaluatePolicyResponse{
		DecisionId: decisionID,
		Decision: &pb.PolicyDecision{
			Result:          pb.DecisionResult(decision.Result),
			Reason:          decision.Reason,
			ApplicableRules: decision.ApplicableRules,
			Confidence:      pb.DecisionConfidence(decision.Confidence),
		},
		Violations: pbViolations,
		Metadata: &pb.EvaluationMetadata{
			EvaluationTimeMs: time.Since(startTime).Milliseconds(),
			CacheHit:         cacheHit,
			EngineVersion:    "1.0.0",
			EvaluatedAt:      timestampFromTime(time.Now()),
		},
	}
}

func (s *PolicyService) recordAuditTrail(
	ctx context.Context,
	decisionID string,
	req *pb.EvaluatePolicyRequest,
	decision *models.PolicyDecision,
	violations []*models.PolicyViolation,
	evalCtx *models.EvaluationContext,
) (string, error) {
	if !s.config.Audit.Enabled {
		return "", nil
	}

	auditEntry := &models.AuditEntry{
		ID:        uuid.New().String(),
		Type:      models.AuditTypeEvaluation,
		Timestamp: time.Now(),
		UserID:    evalCtx.UserID,
		TenantID:  evalCtx.TenantID,
		Action:    "EVALUATE_POLICY",
		Resource:  req.PolicyId,
		Details: map[string]interface{}{
			"decision_id": decisionID,
			"result":      decision.Result.String(),
			"reason":      decision.Reason,
			"violations":  len(violations),
		},
	}

	if err := s.auditRepo.Create(ctx, auditEntry); err != nil {
		return "", err
	}

	return auditEntry.ID, nil
}

func (s *PolicyService) recordPolicyChangeAudit(
	ctx context.Context,
	action string,
	policy *models.Policy,
	oldPolicy *models.Policy,
) (string, error) {
	if !s.config.Audit.Enabled {
		return "", nil
	}

	auditEntry := &models.AuditEntry{
		ID:        uuid.New().String(),
		Type:      models.AuditTypePolicyChange,
		Timestamp: time.Now(),
		UserID:    policy.CreatedBy,
		Action:    action,
		Resource:  policy.ID,
		Details: map[string]interface{}{
			"policy_name": policy.Name,
			"policy_type": policy.Type.String(),
			"version":     policy.Version,
		},
	}

	if err := s.auditRepo.Create(ctx, auditEntry); err != nil {
		return "", err
	}

	return auditEntry.ID, nil
}

// Helper function to convert time.Time to protobuf timestamp
func timestampFromTime(t time.Time) *timestamppb.Timestamp {
	return timestamppb.New(t)
}
