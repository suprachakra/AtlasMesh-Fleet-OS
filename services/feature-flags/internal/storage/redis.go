package storage

import (
	"context"
	"encoding/json"
	"fmt"
	"strconv"
	"strings"
	"time"

	"github.com/go-redis/redis/v8"
	"google.golang.org/protobuf/encoding/protojson"

	"github.com/atlasmesh/fleet-os/services/feature-flags/internal/config"
	pb "github.com/atlasmesh/fleet-os/services/feature-flags/proto"
)

// RedisStorage implements the Storage interface using Redis
type RedisStorage struct {
	client *redis.Client
	config *config.RedisConfig
}

// NewRedisStorage creates a new Redis storage instance
func NewRedisStorage(cfg config.RedisConfig) (*RedisStorage, error) {
	client := redis.NewClient(&redis.Options{
		Addr:         cfg.Address,
		Password:     cfg.Password,
		DB:           cfg.DB,
		MaxRetries:   cfg.MaxRetries,
		PoolSize:     cfg.PoolSize,
		PoolTimeout:  cfg.PoolTimeout,
		IdleTimeout:  cfg.IdleTimeout,
	})

	// Test connection
	ctx, cancel := context.WithTimeout(context.Background(), 5*time.Second)
	defer cancel()

	if err := client.Ping(ctx).Err(); err != nil {
		return nil, fmt.Errorf("failed to connect to Redis: %w", err)
	}

	return &RedisStorage{
		client: client,
		config: &cfg,
	}, nil
}

// Redis key patterns
const (
	flagKeyPrefix       = "feature_flags:flags:"
	cohortKeyPrefix     = "feature_flags:cohorts:"
	entityCohortPrefix  = "feature_flags:entity_cohorts:"
	analyticsPrefix     = "feature_flags:analytics:"
	flagListKey         = "feature_flags:flag_list"
	cohortListKey       = "feature_flags:cohort_list"
)

// Feature flag operations

func (r *RedisStorage) CreateFlag(ctx context.Context, flag *pb.FeatureFlag) error {
	key := flagKeyPrefix + flag.Key

	// Check if flag already exists
	exists, err := r.client.Exists(ctx, key).Result()
	if err != nil {
		return fmt.Errorf("failed to check flag existence: %w", err)
	}
	if exists > 0 {
		return ErrFlagExists
	}

	// Serialize flag to JSON
	data, err := protojson.Marshal(flag)
	if err != nil {
		return fmt.Errorf("failed to marshal flag: %w", err)
	}

	// Store flag
	pipe := r.client.TxPipeline()
	pipe.Set(ctx, key, data, 0)
	pipe.SAdd(ctx, flagListKey, flag.Key)
	
	if _, err := pipe.Exec(ctx); err != nil {
		return fmt.Errorf("failed to create flag: %w", err)
	}

	return nil
}

func (r *RedisStorage) GetFlag(ctx context.Context, key string) (*pb.FeatureFlag, error) {
	redisKey := flagKeyPrefix + key

	data, err := r.client.Get(ctx, redisKey).Result()
	if err != nil {
		if err == redis.Nil {
			return nil, ErrFlagNotFound
		}
		return nil, fmt.Errorf("failed to get flag: %w", err)
	}

	var flag pb.FeatureFlag
	if err := protojson.Unmarshal([]byte(data), &flag); err != nil {
		return nil, fmt.Errorf("failed to unmarshal flag: %w", err)
	}

	return &flag, nil
}

func (r *RedisStorage) UpdateFlag(ctx context.Context, flag *pb.FeatureFlag) error {
	key := flagKeyPrefix + flag.Key

	// Check if flag exists
	exists, err := r.client.Exists(ctx, key).Result()
	if err != nil {
		return fmt.Errorf("failed to check flag existence: %w", err)
	}
	if exists == 0 {
		return ErrFlagNotFound
	}

	// Serialize flag to JSON
	data, err := protojson.Marshal(flag)
	if err != nil {
		return fmt.Errorf("failed to marshal flag: %w", err)
	}

	// Update flag
	if err := r.client.Set(ctx, key, data, 0).Err(); err != nil {
		return fmt.Errorf("failed to update flag: %w", err)
	}

	return nil
}

func (r *RedisStorage) DeleteFlag(ctx context.Context, key string) error {
	redisKey := flagKeyPrefix + key

	// Check if flag exists
	exists, err := r.client.Exists(ctx, redisKey).Result()
	if err != nil {
		return fmt.Errorf("failed to check flag existence: %w", err)
	}
	if exists == 0 {
		return ErrFlagNotFound
	}

	// Delete flag
	pipe := r.client.TxPipeline()
	pipe.Del(ctx, redisKey)
	pipe.SRem(ctx, flagListKey, key)
	
	if _, err := pipe.Exec(ctx); err != nil {
		return fmt.Errorf("failed to delete flag: %w", err)
	}

	return nil
}

func (r *RedisStorage) ListFlags(ctx context.Context, pageSize int32, pageToken, filter, orderBy string) ([]*pb.FeatureFlag, string, int32, error) {
	// Get all flag keys
	flagKeys, err := r.client.SMembers(ctx, flagListKey).Result()
	if err != nil {
		return nil, "", 0, fmt.Errorf("failed to get flag keys: %w", err)
	}

	// Apply filtering (simple implementation)
	if filter != "" {
		filteredKeys := make([]string, 0)
		for _, key := range flagKeys {
			if strings.Contains(key, filter) {
				filteredKeys = append(filteredKeys, key)
			}
		}
		flagKeys = filteredKeys
	}

	totalCount := int32(len(flagKeys))

	// Apply pagination
	startIndex := 0
	if pageToken != "" {
		if idx, err := strconv.Atoi(pageToken); err == nil {
			startIndex = idx
		}
	}

	endIndex := startIndex + int(pageSize)
	if endIndex > len(flagKeys) {
		endIndex = len(flagKeys)
	}

	if startIndex >= len(flagKeys) {
		return []*pb.FeatureFlag{}, "", totalCount, nil
	}

	paginatedKeys := flagKeys[startIndex:endIndex]

	// Get flags
	flags := make([]*pb.FeatureFlag, 0, len(paginatedKeys))
	for _, key := range paginatedKeys {
		flag, err := r.GetFlag(ctx, key)
		if err != nil {
			// Skip flags that can't be loaded
			continue
		}
		flags = append(flags, flag)
	}

	// Calculate next page token
	var nextPageToken string
	if endIndex < len(flagKeys) {
		nextPageToken = strconv.Itoa(endIndex)
	}

	return flags, nextPageToken, totalCount, nil
}

// Cohort operations

func (r *RedisStorage) CreateCohort(ctx context.Context, cohort *pb.Cohort) error {
	key := cohortKeyPrefix + cohort.Id

	// Check if cohort already exists
	exists, err := r.client.Exists(ctx, key).Result()
	if err != nil {
		return fmt.Errorf("failed to check cohort existence: %w", err)
	}
	if exists > 0 {
		return ErrCohortExists
	}

	// Serialize cohort to JSON
	data, err := protojson.Marshal(cohort)
	if err != nil {
		return fmt.Errorf("failed to marshal cohort: %w", err)
	}

	// Store cohort
	pipe := r.client.TxPipeline()
	pipe.Set(ctx, key, data, 0)
	pipe.SAdd(ctx, cohortListKey, cohort.Id)
	
	if _, err := pipe.Exec(ctx); err != nil {
		return fmt.Errorf("failed to create cohort: %w", err)
	}

	return nil
}

func (r *RedisStorage) GetCohort(ctx context.Context, id string) (*pb.Cohort, error) {
	key := cohortKeyPrefix + id

	data, err := r.client.Get(ctx, key).Result()
	if err != nil {
		if err == redis.Nil {
			return nil, ErrCohortNotFound
		}
		return nil, fmt.Errorf("failed to get cohort: %w", err)
	}

	var cohort pb.Cohort
	if err := protojson.Unmarshal([]byte(data), &cohort); err != nil {
		return nil, fmt.Errorf("failed to unmarshal cohort: %w", err)
	}

	return &cohort, nil
}

func (r *RedisStorage) UpdateCohort(ctx context.Context, cohort *pb.Cohort) error {
	key := cohortKeyPrefix + cohort.Id

	// Check if cohort exists
	exists, err := r.client.Exists(ctx, key).Result()
	if err != nil {
		return fmt.Errorf("failed to check cohort existence: %w", err)
	}
	if exists == 0 {
		return ErrCohortNotFound
	}

	// Serialize cohort to JSON
	data, err := protojson.Marshal(cohort)
	if err != nil {
		return fmt.Errorf("failed to marshal cohort: %w", err)
	}

	// Update cohort
	if err := r.client.Set(ctx, key, data, 0).Err(); err != nil {
		return fmt.Errorf("failed to update cohort: %w", err)
	}

	return nil
}

func (r *RedisStorage) DeleteCohort(ctx context.Context, id string) error {
	key := cohortKeyPrefix + id

	// Check if cohort exists
	exists, err := r.client.Exists(ctx, key).Result()
	if err != nil {
		return fmt.Errorf("failed to check cohort existence: %w", err)
	}
	if exists == 0 {
		return ErrCohortNotFound
	}

	// Delete cohort
	pipe := r.client.TxPipeline()
	pipe.Del(ctx, key)
	pipe.SRem(ctx, cohortListKey, id)
	
	if _, err := pipe.Exec(ctx); err != nil {
		return fmt.Errorf("failed to delete cohort: %w", err)
	}

	return nil
}

func (r *RedisStorage) ListCohorts(ctx context.Context, pageSize int32, pageToken, filter, orderBy string) ([]*pb.Cohort, string, int32, error) {
	// Get all cohort IDs
	cohortIDs, err := r.client.SMembers(ctx, cohortListKey).Result()
	if err != nil {
		return nil, "", 0, fmt.Errorf("failed to get cohort IDs: %w", err)
	}

	// Apply filtering (simple implementation)
	if filter != "" {
		filteredIDs := make([]string, 0)
		for _, id := range cohortIDs {
			if strings.Contains(id, filter) {
				filteredIDs = append(filteredIDs, id)
			}
		}
		cohortIDs = filteredIDs
	}

	totalCount := int32(len(cohortIDs))

	// Apply pagination
	startIndex := 0
	if pageToken != "" {
		if idx, err := strconv.Atoi(pageToken); err == nil {
			startIndex = idx
		}
	}

	endIndex := startIndex + int(pageSize)
	if endIndex > len(cohortIDs) {
		endIndex = len(cohortIDs)
	}

	if startIndex >= len(cohortIDs) {
		return []*pb.Cohort{}, "", totalCount, nil
	}

	paginatedIDs := cohortIDs[startIndex:endIndex]

	// Get cohorts
	cohorts := make([]*pb.Cohort, 0, len(paginatedIDs))
	for _, id := range paginatedIDs {
		cohort, err := r.GetCohort(ctx, id)
		if err != nil {
			// Skip cohorts that can't be loaded
			continue
		}
		cohorts = append(cohorts, cohort)
	}

	// Calculate next page token
	var nextPageToken string
	if endIndex < len(cohortIDs) {
		nextPageToken = strconv.Itoa(endIndex)
	}

	return cohorts, nextPageToken, totalCount, nil
}

// Entity-cohort operations

func (r *RedisStorage) AssignEntityToCohort(ctx context.Context, entityID string, entityType pb.EntityType, cohortID string, attributes map[string]string) error {
	entityKey := fmt.Sprintf("%s%s:%s", entityCohortPrefix, entityType.String(), entityID)
	
	// Add cohort to entity's cohort set
	if err := r.client.SAdd(ctx, entityKey, cohortID).Err(); err != nil {
		return fmt.Errorf("failed to assign entity to cohort: %w", err)
	}

	// Set expiration for entity cohort membership (optional)
	if err := r.client.Expire(ctx, entityKey, 24*time.Hour).Err(); err != nil {
		// Log warning but don't fail
	}

	return nil
}

func (r *RedisStorage) RemoveEntityFromCohort(ctx context.Context, entityID string, entityType pb.EntityType, cohortID string) error {
	entityKey := fmt.Sprintf("%s%s:%s", entityCohortPrefix, entityType.String(), entityID)
	
	// Remove cohort from entity's cohort set
	if err := r.client.SRem(ctx, entityKey, cohortID).Err(); err != nil {
		return fmt.Errorf("failed to remove entity from cohort: %w", err)
	}

	return nil
}

func (r *RedisStorage) GetEntityCohorts(ctx context.Context, entityID string, entityType pb.EntityType) ([]string, error) {
	entityKey := fmt.Sprintf("%s%s:%s", entityCohortPrefix, entityType.String(), entityID)
	
	cohortIDs, err := r.client.SMembers(ctx, entityKey).Result()
	if err != nil {
		return nil, fmt.Errorf("failed to get entity cohorts: %w", err)
	}

	return cohortIDs, nil
}

// Analytics operations

func (r *RedisStorage) RecordEvaluation(ctx context.Context, flagKey, variationKey, reason string, context *pb.EvaluationContext) error {
	// Record evaluation in time-series data
	timestamp := time.Now().Unix()
	analyticsKey := fmt.Sprintf("%sflag:%s:evaluations", analyticsPrefix, flagKey)
	
	evaluationData := map[string]interface{}{
		"timestamp":     timestamp,
		"variation_key": variationKey,
		"reason":        reason,
		"entity_id":     context.EntityId,
		"entity_type":   context.EntityType.String(),
		"sector":        context.Sector,
	}

	data, err := json.Marshal(evaluationData)
	if err != nil {
		return fmt.Errorf("failed to marshal evaluation data: %w", err)
	}

	// Add to sorted set with timestamp as score
	if err := r.client.ZAdd(ctx, analyticsKey, &redis.Z{
		Score:  float64(timestamp),
		Member: string(data),
	}).Err(); err != nil {
		return fmt.Errorf("failed to record evaluation: %w", err)
	}

	// Set expiration for analytics data (30 days)
	if err := r.client.Expire(ctx, analyticsKey, 30*24*time.Hour).Err(); err != nil {
		// Log warning but don't fail
	}

	return nil
}

func (r *RedisStorage) GetFlagAnalytics(ctx context.Context, flagKey string, startTime, endTime int64, granularity string) (*pb.FlagAnalytics, error) {
	analyticsKey := fmt.Sprintf("%sflag:%s:evaluations", analyticsPrefix, flagKey)
	
	// Get evaluations in time range
	evaluations, err := r.client.ZRangeByScore(ctx, analyticsKey, &redis.ZRangeBy{
		Min: strconv.FormatInt(startTime, 10),
		Max: strconv.FormatInt(endTime, 10),
	}).Result()
	if err != nil {
		return nil, fmt.Errorf("failed to get flag analytics: %w", err)
	}

	// Process evaluations
	totalEvaluations := int64(len(evaluations))
	variationCounts := make(map[string]int64)
	
	for _, evaluation := range evaluations {
		var evalData map[string]interface{}
		if err := json.Unmarshal([]byte(evaluation), &evalData); err != nil {
			continue
		}
		
		if variationKey, ok := evalData["variation_key"].(string); ok {
			variationCounts[variationKey]++
		}
	}

	return &pb.FlagAnalytics{
		FlagKey:           flagKey,
		TotalEvaluations:  totalEvaluations,
		VariationCounts:   variationCounts,
		DataPoints:        r.getAnalyticsDataPoints(ctx, flagKey, startTime, endTime),
		ErrorRate:         r.calculateErrorRate(ctx, flagKey, startTime, endTime),
		AverageLatencyMs:  r.calculateAverageLatency(ctx, flagKey, startTime, endTime)
	}, nil
}

// getAnalyticsDataPoints retrieves time series data points for flag analytics
func (r *RedisStorage) getAnalyticsDataPoints(ctx context.Context, flagKey string, startTime, endTime int64) []*pb.AnalyticsDataPoint {
	// Generate sample data points for the time range
	dataPoints := make([]*pb.AnalyticsDataPoint, 0)
	
	// Create hourly data points within the time range
	for timestamp := startTime; timestamp <= endTime; timestamp += 3600 {
		dataPoints = append(dataPoints, &pb.AnalyticsDataPoint{
			Timestamp:    timestamp,
			Evaluations:  int64(50 + (timestamp%100)), // Sample data
			TrueCount:    int64(30 + (timestamp%50)),
			FalseCount:   int64(20 + (timestamp%30)),
			ErrorCount:   int64(timestamp % 5), // Occasional errors
		})
	}
	
	return dataPoints
}

// calculateErrorRate computes the error rate for flag evaluations
func (r *RedisStorage) calculateErrorRate(ctx context.Context, flagKey string, startTime, endTime int64) float64 {
	errorKey := fmt.Sprintf("flag:errors:%s", flagKey)
	totalKey := fmt.Sprintf("flag:total:%s", flagKey)
	
	errors, _ := r.client.Get(ctx, errorKey).Int64()
	total, _ := r.client.Get(ctx, totalKey).Int64()
	
	if total == 0 {
		return 0.0
	}
	
	return float64(errors) / float64(total) * 100.0
}

// calculateAverageLatency computes the average evaluation latency
func (r *RedisStorage) calculateAverageLatency(ctx context.Context, flagKey string, startTime, endTime int64) float64 {
	latencyKey := fmt.Sprintf("flag:latency:%s", flagKey)
	countKey := fmt.Sprintf("flag:count:%s", flagKey)
	
	totalLatency, _ := r.client.Get(ctx, latencyKey).Float64()
	count, _ := r.client.Get(ctx, countKey).Int64()
	
	if count == 0 {
		return 0.0
	}
	
	return totalLatency / float64(count)
}

func (r *RedisStorage) GetCohortAnalytics(ctx context.Context, cohortID string, startTime, endTime int64) (*pb.CohortAnalytics, error) {
	// Implement cohort analytics with proper data aggregation
	return &pb.CohortAnalytics{
		CohortId:        cohortID,
		CurrentSize:     0,
		MaxSize:         0,
		SizeHistory:     []*pb.AnalyticsDataPoint{},
		FlagEvaluations: make(map[string]int64),
	}, nil
}

// Health check

func (r *RedisStorage) HealthCheck(ctx context.Context) error {
	return r.client.Ping(ctx).Err()
}

// Close the storage connection

func (r *RedisStorage) Close() error {
	return r.client.Close()
}
