package service

import (
	"context"
	"crypto/md5"
	"encoding/json"
	"fmt"
	"hash/fnv"
	"net/http"
	"strconv"
	"strings"
	"time"

	"github.com/gin-gonic/gin"
	"github.com/prometheus/client_golang/prometheus"
	"github.com/prometheus/client_golang/prometheus/promauto"
	"google.golang.org/grpc/codes"
	"google.golang.org/grpc/status"
	"google.golang.org/protobuf/types/known/structpb"
	"google.golang.org/protobuf/types/known/timestamppb"

	"github.com/atlasmesh/fleet-os/services/feature-flags/internal/config"
	"github.com/atlasmesh/fleet-os/services/feature-flags/internal/storage"
	pb "github.com/atlasmesh/fleet-os/services/feature-flags/proto"
)

// Prometheus metrics
var (
	flagEvaluations = promauto.NewCounterVec(
		prometheus.CounterOpts{
			Name: "feature_flags_evaluations_total",
			Help: "Total number of feature flag evaluations",
		},
		[]string{"flag_key", "variation", "reason"},
	)

	flagEvaluationDuration = promauto.NewHistogramVec(
		prometheus.HistogramOpts{
			Name:    "feature_flags_evaluation_duration_seconds",
			Help:    "Duration of feature flag evaluations",
			Buckets: prometheus.DefBuckets,
		},
		[]string{"flag_key"},
	)

	killSwitchActivations = promauto.NewCounterVec(
		prometheus.CounterOpts{
			Name: "feature_flags_kill_switch_activations_total",
			Help: "Total number of kill switch activations",
		},
		[]string{"flag_key", "reason"},
	)

	cohortAssignments = promauto.NewCounterVec(
		prometheus.CounterOpts{
			Name: "feature_flags_cohort_assignments_total",
			Help: "Total number of cohort assignments",
		},
		[]string{"cohort_id", "entity_type"},
	)
)

// FeatureFlagsService implements the feature flags service
type FeatureFlagsService struct {
	pb.UnimplementedFeatureFlagsServiceServer
	storage storage.Storage
	config  *config.Config
}

// NewFeatureFlagsService creates a new feature flags service
func NewFeatureFlagsService(storage storage.Storage, cfg *config.Config) *FeatureFlagsService {
	return &FeatureFlagsService{
		storage: storage,
		config:  cfg,
	}
}

// EvaluateFlag evaluates a single feature flag
func (s *FeatureFlagsService) EvaluateFlag(ctx context.Context, req *pb.EvaluateFlagRequest) (*pb.EvaluateFlagResponse, error) {
	start := time.Now()
	defer func() {
		flagEvaluationDuration.WithLabelValues(req.FlagKey).Observe(time.Since(start).Seconds())
	}()

	// Get flag from storage
	flag, err := s.storage.GetFlag(ctx, req.FlagKey)
	if err != nil {
		if err == storage.ErrFlagNotFound {
			// Return default value if flag not found
			flagEvaluations.WithLabelValues(req.FlagKey, "default", "flag_not_found").Inc()
			return &pb.EvaluateFlagResponse{
				Value:       req.DefaultValue,
				VariationKey: "default",
				Reason:      "flag_not_found",
				IsDefault:   true,
				EvaluatedAt: timestamppb.Now(),
			}, nil
		}
		return nil, status.Errorf(codes.Internal, "failed to get flag: %v", err)
	}

	// Check if flag is disabled
	if !flag.Enabled {
		flagEvaluations.WithLabelValues(req.FlagKey, "default", "flag_disabled").Inc()
		return &pb.EvaluateFlagResponse{
			Value:       req.DefaultValue,
			VariationKey: "default",
			Reason:      "flag_disabled",
			IsDefault:   true,
			EvaluatedAt: timestamppb.Now(),
		}, nil
	}

	// Check kill switch
	if flag.KillSwitch != nil && flag.KillSwitch.Enabled {
		variation := s.findVariation(flag, flag.KillSwitch.DefaultVariationKey)
		if variation != nil {
			flagEvaluations.WithLabelValues(req.FlagKey, variation.Key, "kill_switch").Inc()
			return &pb.EvaluateFlagResponse{
				Value:       variation.Value,
				VariationKey: variation.Key,
				Reason:      "kill_switch",
				IsDefault:   false,
				EvaluatedAt: timestamppb.Now(),
			}, nil
		}
	}

	// Evaluate rules
	for _, rule := range flag.Rules {
		if !rule.Enabled {
			continue
		}

		if s.evaluateRule(rule, req.Context) {
			variation := s.findVariation(flag, rule.VariationKey)
			if variation != nil {
				flagEvaluations.WithLabelValues(req.FlagKey, variation.Key, "rule_match").Inc()
				return &pb.EvaluateFlagResponse{
					Value:       variation.Value,
					VariationKey: variation.Key,
					Reason:      fmt.Sprintf("rule_match:%s", rule.Id),
					IsDefault:   false,
					EvaluatedAt: timestamppb.Now(),
				}, nil
			}
		}
	}

	// Evaluate rollout
	if flag.Rollout != nil {
		variation, reason := s.evaluateRollout(flag.Rollout, req.Context, flag.Variations)
		if variation != nil {
			flagEvaluations.WithLabelValues(req.FlagKey, variation.Key, reason).Inc()
			return &pb.EvaluateFlagResponse{
				Value:       variation.Value,
				VariationKey: variation.Key,
				Reason:      reason,
				IsDefault:   false,
				EvaluatedAt: timestamppb.Now(),
			}, nil
		}
	}

	// Return default value if no rules match
	flagEvaluations.WithLabelValues(req.FlagKey, "default", "no_match").Inc()
	return &pb.EvaluateFlagResponse{
		Value:       req.DefaultValue,
		VariationKey: "default",
		Reason:      "no_match",
		IsDefault:   true,
		EvaluatedAt: timestamppb.Now(),
	}, nil
}

// BatchEvaluateFlags evaluates multiple feature flags
func (s *FeatureFlagsService) BatchEvaluateFlags(ctx context.Context, req *pb.BatchEvaluateFlagsRequest) (*pb.BatchEvaluateFlagsResponse, error) {
	if len(req.FlagKeys) > s.config.FeatureFlags.MaxBatchSize {
		return nil, status.Errorf(codes.InvalidArgument, "batch size exceeds maximum of %d", s.config.FeatureFlags.MaxBatchSize)
	}

	evaluations := make(map[string]*pb.EvaluateFlagResponse)

	for _, flagKey := range req.FlagKeys {
		defaultValue := req.DefaultValues[flagKey]
		
		evalReq := &pb.EvaluateFlagRequest{
			FlagKey:      flagKey,
			Context:      req.Context,
			DefaultValue: defaultValue,
		}

		evalResp, err := s.EvaluateFlag(ctx, evalReq)
		if err != nil {
			// Continue with other flags even if one fails
			evaluations[flagKey] = &pb.EvaluateFlagResponse{
				Value:       defaultValue,
				VariationKey: "error",
				Reason:      fmt.Sprintf("evaluation_error: %v", err),
				IsDefault:   true,
				EvaluatedAt: timestamppb.Now(),
			}
		} else {
			evaluations[flagKey] = evalResp
		}
	}

	return &pb.BatchEvaluateFlagsResponse{
		Evaluations: evaluations,
	}, nil
}

// CreateFlag creates a new feature flag
func (s *FeatureFlagsService) CreateFlag(ctx context.Context, req *pb.CreateFlagRequest) (*pb.CreateFlagResponse, error) {
	// Validate flag
	if err := s.validateFlag(req.Flag); err != nil {
		return nil, status.Errorf(codes.InvalidArgument, "invalid flag: %v", err)
	}

	// Set timestamps
	now := timestamppb.Now()
	req.Flag.CreatedAt = now
	req.Flag.UpdatedAt = now

	// Store flag
	if err := s.storage.CreateFlag(ctx, req.Flag); err != nil {
		if err == storage.ErrFlagExists {
			return nil, status.Errorf(codes.AlreadyExists, "flag already exists: %s", req.Flag.Key)
		}
		return nil, status.Errorf(codes.Internal, "failed to create flag: %v", err)
	}

	return &pb.CreateFlagResponse{
		Flag: req.Flag,
	}, nil
}

// GetFlag retrieves a feature flag
func (s *FeatureFlagsService) GetFlag(ctx context.Context, req *pb.GetFlagRequest) (*pb.GetFlagResponse, error) {
	flag, err := s.storage.GetFlag(ctx, req.FlagKey)
	if err != nil {
		if err == storage.ErrFlagNotFound {
			return nil, status.Errorf(codes.NotFound, "flag not found: %s", req.FlagKey)
		}
		return nil, status.Errorf(codes.Internal, "failed to get flag: %v", err)
	}

	return &pb.GetFlagResponse{
		Flag: flag,
	}, nil
}

// UpdateFlag updates a feature flag
func (s *FeatureFlagsService) UpdateFlag(ctx context.Context, req *pb.UpdateFlagRequest) (*pb.UpdateFlagResponse, error) {
	// Validate flag
	if err := s.validateFlag(req.Flag); err != nil {
		return nil, status.Errorf(codes.InvalidArgument, "invalid flag: %v", err)
	}

	// Set update timestamp
	req.Flag.UpdatedAt = timestamppb.Now()

	// Update flag
	if err := s.storage.UpdateFlag(ctx, req.Flag); err != nil {
		if err == storage.ErrFlagNotFound {
			return nil, status.Errorf(codes.NotFound, "flag not found: %s", req.Flag.Key)
		}
		return nil, status.Errorf(codes.Internal, "failed to update flag: %v", err)
	}

	return &pb.UpdateFlagResponse{
		Flag: req.Flag,
	}, nil
}

// DeleteFlag deletes a feature flag
func (s *FeatureFlagsService) DeleteFlag(ctx context.Context, req *pb.DeleteFlagRequest) (*pb.DeleteFlagResponse, error) {
	if err := s.storage.DeleteFlag(ctx, req.FlagKey); err != nil {
		if err == storage.ErrFlagNotFound {
			return nil, status.Errorf(codes.NotFound, "flag not found: %s", req.FlagKey)
		}
		return nil, status.Errorf(codes.Internal, "failed to delete flag: %v", err)
	}

	return &pb.DeleteFlagResponse{
		Success: true,
	}, nil
}

// ListFlags lists feature flags
func (s *FeatureFlagsService) ListFlags(ctx context.Context, req *pb.ListFlagsRequest) (*pb.ListFlagsResponse, error) {
	flags, nextToken, totalCount, err := s.storage.ListFlags(ctx, req.PageSize, req.PageToken, req.Filter, req.OrderBy)
	if err != nil {
		return nil, status.Errorf(codes.Internal, "failed to list flags: %v", err)
	}

	return &pb.ListFlagsResponse{
		Flags:         flags,
		NextPageToken: nextToken,
		TotalCount:    totalCount,
	}, nil
}

// KillSwitch activates a kill switch for a flag
func (s *FeatureFlagsService) KillSwitch(ctx context.Context, req *pb.KillSwitchRequest) (*pb.KillSwitchResponse, error) {
	// Get existing flag
	flag, err := s.storage.GetFlag(ctx, req.FlagKey)
	if err != nil {
		if err == storage.ErrFlagNotFound {
			return nil, status.Errorf(codes.NotFound, "flag not found: %s", req.FlagKey)
		}
		return nil, status.Errorf(codes.Internal, "failed to get flag: %v", err)
	}

	// Activate kill switch
	now := timestamppb.Now()
	flag.KillSwitch = &pb.KillSwitch{
		Enabled:              true,
		Reason:               req.Reason,
		ActivatedAt:          now,
		DefaultVariationKey:  req.DefaultVariationKey,
	}
	flag.UpdatedAt = now

	// Update flag
	if err := s.storage.UpdateFlag(ctx, flag); err != nil {
		return nil, status.Errorf(codes.Internal, "failed to activate kill switch: %v", err)
	}

	// Record metrics
	killSwitchActivations.WithLabelValues(req.FlagKey, req.Reason).Inc()

	return &pb.KillSwitchResponse{
		Success:     true,
		ActivatedAt: now,
	}, nil
}

// ReviveFlag deactivates a kill switch for a flag
func (s *FeatureFlagsService) ReviveFlag(ctx context.Context, req *pb.ReviveFlagRequest) (*pb.ReviveFlagResponse, error) {
	// Get existing flag
	flag, err := s.storage.GetFlag(ctx, req.FlagKey)
	if err != nil {
		if err == storage.ErrFlagNotFound {
			return nil, status.Errorf(codes.NotFound, "flag not found: %s", req.FlagKey)
		}
		return nil, status.Errorf(codes.Internal, "failed to get flag: %v", err)
	}

	// Deactivate kill switch
	if flag.KillSwitch != nil {
		flag.KillSwitch.Enabled = false
	}
	flag.UpdatedAt = timestamppb.Now()

	// Update flag
	if err := s.storage.UpdateFlag(ctx, flag); err != nil {
		return nil, status.Errorf(codes.Internal, "failed to revive flag: %v", err)
	}

	return &pb.ReviveFlagResponse{
		Success:   true,
		RevivedAt: timestamppb.Now(),
	}, nil
}

// Helper methods

func (s *FeatureFlagsService) findVariation(flag *pb.FeatureFlag, variationKey string) *pb.Variation {
	for _, variation := range flag.Variations {
		if variation.Key == variationKey {
			return variation
		}
	}
	return nil
}

func (s *FeatureFlagsService) evaluateRule(rule *pb.Rule, context *pb.EvaluationContext) bool {
	for _, condition := range rule.Conditions {
		if !s.evaluateCondition(condition, context) {
			return false
		}
	}
	return true
}

func (s *FeatureFlagsService) evaluateCondition(condition *pb.Condition, context *pb.EvaluationContext) bool {
	var contextValue string
	
	// Get context value based on attribute
	switch condition.Attribute {
	case "entity_id":
		contextValue = context.EntityId
	case "entity_type":
		contextValue = context.EntityType.String()
	case "sector":
		contextValue = context.Sector
	case "vehicle_id":
		contextValue = context.VehicleId
	case "user_id":
		contextValue = context.UserId
	case "tenant_id":
		contextValue = context.TenantId
	case "environment":
		contextValue = context.Environment
	case "region":
		contextValue = context.Region
	default:
		// Check custom attributes
		if context.Attributes != nil {
			contextValue = context.Attributes[condition.Attribute]
		}
	}

	// Evaluate condition based on operator
	switch condition.Operator {
	case pb.Operator_OPERATOR_EQUALS:
		return len(condition.Values) > 0 && contextValue == condition.Values[0]
	case pb.Operator_OPERATOR_NOT_EQUALS:
		return len(condition.Values) > 0 && contextValue != condition.Values[0]
	case pb.Operator_OPERATOR_IN:
		return s.stringInSlice(contextValue, condition.Values)
	case pb.Operator_OPERATOR_NOT_IN:
		return !s.stringInSlice(contextValue, condition.Values)
	case pb.Operator_OPERATOR_CONTAINS:
		return len(condition.Values) > 0 && strings.Contains(contextValue, condition.Values[0])
	case pb.Operator_OPERATOR_NOT_CONTAINS:
		return len(condition.Values) > 0 && !strings.Contains(contextValue, condition.Values[0])
	case pb.Operator_OPERATOR_STARTS_WITH:
		return len(condition.Values) > 0 && strings.HasPrefix(contextValue, condition.Values[0])
	case pb.Operator_OPERATOR_ENDS_WITH:
		return len(condition.Values) > 0 && strings.HasSuffix(contextValue, condition.Values[0])
	case pb.Operator_OPERATOR_COHORT_MEMBER:
		return len(condition.Values) > 0 && s.stringInSlice(condition.Values[0], context.CohortIds)
	default:
		return false
	}
}

func (s *FeatureFlagsService) evaluateRollout(rollout *pb.Rollout, context *pb.EvaluationContext, variations []*pb.Variation) (*pb.Variation, string) {
	switch rollout.Type {
	case pb.RolloutType_ROLLOUT_TYPE_PERCENTAGE:
		return s.evaluatePercentageRollout(rollout, context, variations)
	case pb.RolloutType_ROLLOUT_TYPE_COHORT:
		return s.evaluateCohortRollout(rollout, context, variations)
	case pb.RolloutType_ROLLOUT_TYPE_HYBRID:
		// Try cohort rollout first, then percentage
		if variation, reason := s.evaluateCohortRollout(rollout, context, variations); variation != nil {
			return variation, reason
		}
		return s.evaluatePercentageRollout(rollout, context, variations)
	default:
		return nil, "unknown_rollout_type"
	}
}

func (s *FeatureFlagsService) evaluatePercentageRollout(rollout *pb.Rollout, context *pb.EvaluationContext, variations []*pb.Variation) (*pb.Variation, string) {
	// Get bucket value based on bucketing attribute
	bucketValue := s.getBucketValue(rollout.BucketBy, context)
	if bucketValue == "" {
		return nil, "no_bucket_value"
	}

	// Calculate hash percentage (0-100)
	hashPercentage := s.calculateHashPercentage(bucketValue)

	// Check if user falls within rollout percentage
	if hashPercentage <= rollout.Percentage*100 {
		variation := s.findVariation(&pb.FeatureFlag{Variations: variations}, rollout.VariationKey)
		if variation != nil {
			return variation, "percentage_rollout"
		}
	}

	return nil, "not_in_rollout"
}

func (s *FeatureFlagsService) evaluateCohortRollout(rollout *pb.Rollout, context *pb.EvaluationContext, variations []*pb.Variation) (*pb.Variation, string) {
	for _, cohortRollout := range rollout.CohortRollouts {
		// Check if entity is in cohort
		if s.stringInSlice(cohortRollout.CohortId, context.CohortIds) {
			// If percentage is 100%, return variation immediately
			if cohortRollout.Percentage >= 1.0 {
				variation := s.findVariation(&pb.FeatureFlag{Variations: variations}, cohortRollout.VariationKey)
				if variation != nil {
					return variation, fmt.Sprintf("cohort_rollout:%s", cohortRollout.CohortId)
				}
			}

			// Otherwise, use percentage-based rollout within cohort
			bucketValue := s.getBucketValue(rollout.BucketBy, context)
			if bucketValue != "" {
				hashPercentage := s.calculateHashPercentage(bucketValue + cohortRollout.CohortId)
				if hashPercentage <= cohortRollout.Percentage*100 {
					variation := s.findVariation(&pb.FeatureFlag{Variations: variations}, cohortRollout.VariationKey)
					if variation != nil {
						return variation, fmt.Sprintf("cohort_rollout:%s", cohortRollout.CohortId)
					}
				}
			}
		}
	}

	return nil, "not_in_cohort_rollout"
}

func (s *FeatureFlagsService) getBucketValue(bucketBy string, context *pb.EvaluationContext) string {
	switch bucketBy {
	case "entity_id":
		return context.EntityId
	case "vehicle_id":
		return context.VehicleId
	case "user_id":
		return context.UserId
	case "tenant_id":
		return context.TenantId
	default:
		if context.Attributes != nil {
			return context.Attributes[bucketBy]
		}
		return context.EntityId // Default to entity_id
	}
}

func (s *FeatureFlagsService) calculateHashPercentage(value string) float64 {
	hash := fnv.New32a()
	hash.Write([]byte(value))
	return float64(hash.Sum32()%10000) / 100.0 // 0-99.99
}

func (s *FeatureFlagsService) stringInSlice(str string, slice []string) bool {
	for _, s := range slice {
		if s == str {
			return true
		}
	}
	return false
}

func (s *FeatureFlagsService) validateFlag(flag *pb.FeatureFlag) error {
	if flag.Key == "" {
		return fmt.Errorf("flag key is required")
	}

	if flag.Name == "" {
		return fmt.Errorf("flag name is required")
	}

	if len(flag.Variations) == 0 {
		return fmt.Errorf("at least one variation is required")
	}

	// Validate variations have unique keys
	variationKeys := make(map[string]bool)
	for _, variation := range flag.Variations {
		if variation.Key == "" {
			return fmt.Errorf("variation key is required")
		}
		if variationKeys[variation.Key] {
			return fmt.Errorf("duplicate variation key: %s", variation.Key)
		}
		variationKeys[variation.Key] = true
	}

	// Validate rules reference valid variations
	for _, rule := range flag.Rules {
		if rule.VariationKey != "" && !variationKeys[rule.VariationKey] {
			return fmt.Errorf("rule references invalid variation: %s", rule.VariationKey)
		}
	}

	// Validate rollout references valid variations
	if flag.Rollout != nil {
		if flag.Rollout.VariationKey != "" && !variationKeys[flag.Rollout.VariationKey] {
			return fmt.Errorf("rollout references invalid variation: %s", flag.Rollout.VariationKey)
		}
		for _, cohortRollout := range flag.Rollout.CohortRollouts {
			if cohortRollout.VariationKey != "" && !variationKeys[cohortRollout.VariationKey] {
				return fmt.Errorf("cohort rollout references invalid variation: %s", cohortRollout.VariationKey)
			}
		}
	}

	return nil
}

// HealthCheck checks the health of the service
func (s *FeatureFlagsService) HealthCheck(ctx context.Context) error {
	return s.storage.HealthCheck(ctx)
}

// HTTP handlers (simplified - full implementation would include proper request/response handling)

func (s *FeatureFlagsService) EvaluateHTTP(c *gin.Context) {
	// Implementation for HTTP evaluation endpoint
	c.JSON(http.StatusOK, gin.H{"message": "HTTP evaluation endpoint"})
}

func (s *FeatureFlagsService) BatchEvaluateHTTP(c *gin.Context) {
	// Implementation for HTTP batch evaluation endpoint
	c.JSON(http.StatusOK, gin.H{"message": "HTTP batch evaluation endpoint"})
}

func (s *FeatureFlagsService) ListFlagsHTTP(c *gin.Context) {
	// Implementation for HTTP list flags endpoint
	c.JSON(http.StatusOK, gin.H{"message": "HTTP list flags endpoint"})
}

func (s *FeatureFlagsService) CreateFlagHTTP(c *gin.Context) {
	// Implementation for HTTP create flag endpoint
	c.JSON(http.StatusOK, gin.H{"message": "HTTP create flag endpoint"})
}

func (s *FeatureFlagsService) GetFlagHTTP(c *gin.Context) {
	// Implementation for HTTP get flag endpoint
	c.JSON(http.StatusOK, gin.H{"message": "HTTP get flag endpoint"})
}

func (s *FeatureFlagsService) UpdateFlagHTTP(c *gin.Context) {
	// Implementation for HTTP update flag endpoint
	c.JSON(http.StatusOK, gin.H{"message": "HTTP update flag endpoint"})
}

func (s *FeatureFlagsService) DeleteFlagHTTP(c *gin.Context) {
	// Implementation for HTTP delete flag endpoint
	c.JSON(http.StatusOK, gin.H{"message": "HTTP delete flag endpoint"})
}

func (s *FeatureFlagsService) ListCohortsHTTP(c *gin.Context) {
	// Implementation for HTTP list cohorts endpoint
	c.JSON(http.StatusOK, gin.H{"message": "HTTP list cohorts endpoint"})
}

func (s *FeatureFlagsService) CreateCohortHTTP(c *gin.Context) {
	// Implementation for HTTP create cohort endpoint
	c.JSON(http.StatusOK, gin.H{"message": "HTTP create cohort endpoint"})
}

func (s *FeatureFlagsService) GetCohortHTTP(c *gin.Context) {
	// Implementation for HTTP get cohort endpoint
	c.JSON(http.StatusOK, gin.H{"message": "HTTP get cohort endpoint"})
}

func (s *FeatureFlagsService) UpdateCohortHTTP(c *gin.Context) {
	// Implementation for HTTP update cohort endpoint
	c.JSON(http.StatusOK, gin.H{"message": "HTTP update cohort endpoint"})
}

func (s *FeatureFlagsService) DeleteCohortHTTP(c *gin.Context) {
	// Implementation for HTTP delete cohort endpoint
	c.JSON(http.StatusOK, gin.H{"message": "HTTP delete cohort endpoint"})
}

func (s *FeatureFlagsService) AssignEntityToCohortHTTP(c *gin.Context) {
	// Implementation for HTTP assign entity to cohort endpoint
	c.JSON(http.StatusOK, gin.H{"message": "HTTP assign entity to cohort endpoint"})
}

func (s *FeatureFlagsService) RemoveEntityFromCohortHTTP(c *gin.Context) {
	// Implementation for HTTP remove entity from cohort endpoint
	c.JSON(http.StatusOK, gin.H{"message": "HTTP remove entity from cohort endpoint"})
}

func (s *FeatureFlagsService) GetEntityFlagsHTTP(c *gin.Context) {
	// Implementation for HTTP get entity flags endpoint
	c.JSON(http.StatusOK, gin.H{"message": "HTTP get entity flags endpoint"})
}

func (s *FeatureFlagsService) GetFlagAnalyticsHTTP(c *gin.Context) {
	// Implementation for HTTP get flag analytics endpoint
	c.JSON(http.StatusOK, gin.H{"message": "HTTP get flag analytics endpoint"})
}

func (s *FeatureFlagsService) GetCohortAnalyticsHTTP(c *gin.Context) {
	// Implementation for HTTP get cohort analytics endpoint
	c.JSON(http.StatusOK, gin.H{"message": "HTTP get cohort analytics endpoint"})
}

func (s *FeatureFlagsService) KillSwitchHTTP(c *gin.Context) {
	// Implementation for HTTP kill switch endpoint
	c.JSON(http.StatusOK, gin.H{"message": "HTTP kill switch endpoint"})
}

func (s *FeatureFlagsService) ReviveFlagHTTP(c *gin.Context) {
	// Implementation for HTTP revive flag endpoint
	c.JSON(http.StatusOK, gin.H{"message": "HTTP revive flag endpoint"})
}

func (s *FeatureFlagsService) ClearCacheHTTP(c *gin.Context) {
	// Implementation for HTTP clear cache endpoint
	c.JSON(http.StatusOK, gin.H{"message": "HTTP clear cache endpoint"})
}

func (s *FeatureFlagsService) GetStatsHTTP(c *gin.Context) {
	// Implementation for HTTP get stats endpoint
	c.JSON(http.StatusOK, gin.H{"message": "HTTP get stats endpoint"})
}

func (s *FeatureFlagsService) ExportConfigHTTP(c *gin.Context) {
	// Implementation for HTTP export config endpoint
	c.JSON(http.StatusOK, gin.H{"message": "HTTP export config endpoint"})
}

func (s *FeatureFlagsService) ImportConfigHTTP(c *gin.Context) {
	// Implementation for HTTP import config endpoint
	c.JSON(http.StatusOK, gin.H{"message": "HTTP import config endpoint"})
}

// Cohort service methods (stubs - full implementation would be in separate cohort service)

func (s *FeatureFlagsService) CreateCohort(ctx context.Context, req *pb.CreateCohortRequest) (*pb.CreateCohortResponse, error) {
	return nil, status.Errorf(codes.Unimplemented, "method CreateCohort not implemented")
}

func (s *FeatureFlagsService) GetCohort(ctx context.Context, req *pb.GetCohortRequest) (*pb.GetCohortResponse, error) {
	return nil, status.Errorf(codes.Unimplemented, "method GetCohort not implemented")
}

func (s *FeatureFlagsService) UpdateCohort(ctx context.Context, req *pb.UpdateCohortRequest) (*pb.UpdateCohortResponse, error) {
	return nil, status.Errorf(codes.Unimplemented, "method UpdateCohort not implemented")
}

func (s *FeatureFlagsService) DeleteCohort(ctx context.Context, req *pb.DeleteCohortRequest) (*pb.DeleteCohortResponse, error) {
	return nil, status.Errorf(codes.Unimplemented, "method DeleteCohort not implemented")
}

func (s *FeatureFlagsService) ListCohorts(ctx context.Context, req *pb.ListCohortsRequest) (*pb.ListCohortsResponse, error) {
	return nil, status.Errorf(codes.Unimplemented, "method ListCohorts not implemented")
}

func (s *FeatureFlagsService) AssignEntityToCohort(ctx context.Context, req *pb.AssignEntityToCohortRequest) (*pb.AssignEntityToCohortResponse, error) {
	return nil, status.Errorf(codes.Unimplemented, "method AssignEntityToCohort not implemented")
}

func (s *FeatureFlagsService) RemoveEntityFromCohort(ctx context.Context, req *pb.RemoveEntityFromCohortRequest) (*pb.RemoveEntityFromCohortResponse, error) {
	return nil, status.Errorf(codes.Unimplemented, "method RemoveEntityFromCohort not implemented")
}

func (s *FeatureFlagsService) GetEntityCohorts(ctx context.Context, req *pb.GetEntityCohortsRequest) (*pb.GetEntityCohortsResponse, error) {
	return nil, status.Errorf(codes.Unimplemented, "method GetEntityCohorts not implemented")
}

func (s *FeatureFlagsService) GetFlagAnalytics(ctx context.Context, req *pb.GetFlagAnalyticsRequest) (*pb.GetFlagAnalyticsResponse, error) {
	return nil, status.Errorf(codes.Unimplemented, "method GetFlagAnalytics not implemented")
}

func (s *FeatureFlagsService) GetCohortAnalytics(ctx context.Context, req *pb.GetCohortAnalyticsRequest) (*pb.GetCohortAnalyticsResponse, error) {
	return nil, status.Errorf(codes.Unimplemented, "method GetCohortAnalytics not implemented")
}
