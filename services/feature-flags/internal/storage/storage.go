package storage

import (
	"context"
	"errors"

	pb "github.com/atlasmesh/fleet-os/services/feature-flags/proto"
)

// Common errors
var (
	ErrFlagNotFound    = errors.New("flag not found")
	ErrFlagExists      = errors.New("flag already exists")
	ErrCohortNotFound  = errors.New("cohort not found")
	ErrCohortExists    = errors.New("cohort already exists")
)

// Storage interface defines the storage operations for feature flags
type Storage interface {
	// Feature flag operations
	CreateFlag(ctx context.Context, flag *pb.FeatureFlag) error
	GetFlag(ctx context.Context, key string) (*pb.FeatureFlag, error)
	UpdateFlag(ctx context.Context, flag *pb.FeatureFlag) error
	DeleteFlag(ctx context.Context, key string) error
	ListFlags(ctx context.Context, pageSize int32, pageToken, filter, orderBy string) ([]*pb.FeatureFlag, string, int32, error)

	// Cohort operations
	CreateCohort(ctx context.Context, cohort *pb.Cohort) error
	GetCohort(ctx context.Context, id string) (*pb.Cohort, error)
	UpdateCohort(ctx context.Context, cohort *pb.Cohort) error
	DeleteCohort(ctx context.Context, id string) error
	ListCohorts(ctx context.Context, pageSize int32, pageToken, filter, orderBy string) ([]*pb.Cohort, string, int32, error)

	// Entity-cohort operations
	AssignEntityToCohort(ctx context.Context, entityID string, entityType pb.EntityType, cohortID string, attributes map[string]string) error
	RemoveEntityFromCohort(ctx context.Context, entityID string, entityType pb.EntityType, cohortID string) error
	GetEntityCohorts(ctx context.Context, entityID string, entityType pb.EntityType) ([]string, error)

	// Analytics operations
	RecordEvaluation(ctx context.Context, flagKey, variationKey, reason string, context *pb.EvaluationContext) error
	GetFlagAnalytics(ctx context.Context, flagKey string, startTime, endTime int64, granularity string) (*pb.FlagAnalytics, error)
	GetCohortAnalytics(ctx context.Context, cohortID string, startTime, endTime int64) (*pb.CohortAnalytics, error)

	// Health check
	HealthCheck(ctx context.Context) error

	// Close the storage connection
	Close() error
}
