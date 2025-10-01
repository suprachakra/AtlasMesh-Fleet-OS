package main

import (
	"context"
	"encoding/json"
	"fmt"
	"log"
	"net/http"
	"os"
	"os/signal"
	"strconv"
	"strings"
	"syscall"
	"time"

	"github.com/go-redis/redis/v8"
	"github.com/gorilla/mux"
	"github.com/prometheus/client_golang/prometheus/promhttp"
)

// Cache Manager Service
// Handles application-level caching, cache invalidation, and CDN integration

type CacheManagerService struct {
	redisCluster *redis.ClusterClient
	redisClient  *redis.Client
	cdnManager   *CDNManager
	cacheStats   *CacheStatistics
}

// Cache Configuration
type CacheConfig struct {
	DefaultTTL        time.Duration            `json:"default_ttl"`
	MaxKeySize        int                      `json:"max_key_size"`
	MaxValueSize      int                      `json:"max_value_size"`
	CompressionLevel  int                      `json:"compression_level"`
	EvictionPolicy    string                   `json:"eviction_policy"`
	Namespaces        map[string]NamespaceConfig `json:"namespaces"`
}

type NamespaceConfig struct {
	TTL              time.Duration `json:"ttl"`
	MaxKeys          int           `json:"max_keys"`
	CompressionEnabled bool        `json:"compression_enabled"`
	ReplicationFactor int          `json:"replication_factor"`
}

// Cache Entry
type CacheEntry struct {
	Key        string                 `json:"key"`
	Value      interface{}            `json:"value"`
	TTL        time.Duration          `json:"ttl"`
	Namespace  string                 `json:"namespace"`
	Tags       []string               `json:"tags"`
	Metadata   map[string]interface{} `json:"metadata"`
	CreatedAt  time.Time              `json:"created_at"`
	ExpiresAt  time.Time              `json:"expires_at"`
	AccessCount int                   `json:"access_count"`
	LastAccess time.Time              `json:"last_access"`
}

// Cache Operation
type CacheOperation struct {
	Operation string      `json:"operation"` // get, set, delete, invalidate
	Key       string      `json:"key"`
	Value     interface{} `json:"value,omitempty"`
	TTL       int         `json:"ttl,omitempty"` // seconds
	Namespace string      `json:"namespace,omitempty"`
	Tags      []string    `json:"tags,omitempty"`
}

type CacheResponse struct {
	Success   bool        `json:"success"`
	Data      interface{} `json:"data,omitempty"`
	Error     string      `json:"error,omitempty"`
	Cached    bool        `json:"cached"`
	TTL       int         `json:"ttl,omitempty"`
	Timestamp time.Time   `json:"timestamp"`
}

// Cache Statistics
type CacheStatistics struct {
	HitCount        int64                  `json:"hit_count"`
	MissCount       int64                  `json:"miss_count"`
	SetCount        int64                  `json:"set_count"`
	DeleteCount     int64                  `json:"delete_count"`
	EvictionCount   int64                  `json:"eviction_count"`
	HitRatio        float64                `json:"hit_ratio"`
	MemoryUsage     int64                  `json:"memory_usage_bytes"`
	KeyCount        int64                  `json:"key_count"`
	NamespaceStats  map[string]NamespaceStats `json:"namespace_stats"`
	LastReset       time.Time              `json:"last_reset"`
}

type NamespaceStats struct {
	KeyCount      int64   `json:"key_count"`
	HitCount      int64   `json:"hit_count"`
	MissCount     int64   `json:"miss_count"`
	HitRatio      float64 `json:"hit_ratio"`
	MemoryUsage   int64   `json:"memory_usage_bytes"`
	AvgTTL        int     `json:"avg_ttl_seconds"`
}

// CDN Manager
type CDNManager struct {
	provider     string
	apiKey       string
	baseURL      string
	zones        map[string]CDNZone
	purgeQueue   chan PurgeRequest
}

type CDNZone struct {
	ZoneID      string   `json:"zone_id"`
	Name        string   `json:"name"`
	Type        string   `json:"type"` // static, dynamic, streaming
	Origins     []string `json:"origins"`
	CacheRules  []CacheRule `json:"cache_rules"`
	Enabled     bool     `json:"enabled"`
}

type CacheRule struct {
	Pattern    string        `json:"pattern"`
	TTL        time.Duration `json:"ttl"`
	Conditions []string      `json:"conditions"`
	Actions    []string      `json:"actions"`
}

type PurgeRequest struct {
	ZoneID string   `json:"zone_id"`
	URLs   []string `json:"urls"`
	Tags   []string `json:"tags"`
	Type   string   `json:"type"` // url, tag, everything
}

// Cache Invalidation
type InvalidationRule struct {
	RuleID      string   `json:"rule_id"`
	Name        string   `json:"name"`
	Triggers    []string `json:"triggers"`
	Patterns    []string `json:"patterns"`
	Namespaces  []string `json:"namespaces"`
	Tags        []string `json:"tags"`
	Enabled     bool     `json:"enabled"`
	CreatedAt   time.Time `json:"created_at"`
}

type InvalidationEvent struct {
	EventID   string                 `json:"event_id"`
	Type      string                 `json:"type"`
	Source    string                 `json:"source"`
	Trigger   string                 `json:"trigger"`
	Patterns  []string               `json:"patterns"`
	Metadata  map[string]interface{} `json:"metadata"`
	Timestamp time.Time              `json:"timestamp"`
}

func main() {
	// Initialize service
	service := &CacheManagerService{
		redisCluster: initRedisCluster(),
		redisClient:  initRedisClient(),
		cdnManager:   initCDNManager(),
		cacheStats:   &CacheStatistics{
			NamespaceStats: make(map[string]NamespaceStats),
			LastReset:      time.Now(),
		},
	}

	// Setup router
	router := mux.NewRouter()
	
	// Cache Operations
	router.HandleFunc("/api/v1/cache/get/{key}", service.getCacheEntry).Methods("GET")
	router.HandleFunc("/api/v1/cache/set", service.setCacheEntry).Methods("POST")
	router.HandleFunc("/api/v1/cache/delete/{key}", service.deleteCacheEntry).Methods("DELETE")
	router.HandleFunc("/api/v1/cache/exists/{key}", service.checkCacheExists).Methods("GET")
	router.HandleFunc("/api/v1/cache/ttl/{key}", service.getCacheTTL).Methods("GET")
	
	// Batch Operations
	router.HandleFunc("/api/v1/cache/batch/get", service.batchGetCache).Methods("POST")
	router.HandleFunc("/api/v1/cache/batch/set", service.batchSetCache).Methods("POST")
	router.HandleFunc("/api/v1/cache/batch/delete", service.batchDeleteCache).Methods("POST")
	
	// Namespace Operations
	router.HandleFunc("/api/v1/cache/namespace/{namespace}/keys", service.getNamespaceKeys).Methods("GET")
	router.HandleFunc("/api/v1/cache/namespace/{namespace}/flush", service.flushNamespace).Methods("DELETE")
	router.HandleFunc("/api/v1/cache/namespace/{namespace}/stats", service.getNamespaceStats).Methods("GET")
	
	// Tag-based Operations
	router.HandleFunc("/api/v1/cache/tags/{tag}/keys", service.getKeysByTag).Methods("GET")
	router.HandleFunc("/api/v1/cache/tags/{tag}/invalidate", service.invalidateByTag).Methods("DELETE")
	
	// Cache Statistics
	router.HandleFunc("/api/v1/cache/stats", service.getCacheStats).Methods("GET")
	router.HandleFunc("/api/v1/cache/stats/reset", service.resetCacheStats).Methods("POST")
	
	// Cache Invalidation
	router.HandleFunc("/api/v1/cache/invalidate", service.invalidateCache).Methods("POST")
	router.HandleFunc("/api/v1/cache/invalidation/rules", service.getInvalidationRules).Methods("GET")
	router.HandleFunc("/api/v1/cache/invalidation/rules", service.createInvalidationRule).Methods("POST")
	
	// CDN Operations
	router.HandleFunc("/api/v1/cdn/zones", service.getCDNZones).Methods("GET")
	router.HandleFunc("/api/v1/cdn/purge", service.purgeCDN).Methods("POST")
	router.HandleFunc("/api/v1/cdn/stats", service.getCDNStats).Methods("GET")
	
	// Cache Warming
	router.HandleFunc("/api/v1/cache/warm", service.warmCache).Methods("POST")
	router.HandleFunc("/api/v1/cache/preload", service.preloadCache).Methods("POST")
	
	// Health and metrics
	router.HandleFunc("/health", service.healthCheck).Methods("GET")
	router.Handle("/metrics", promhttp.Handler()).Methods("GET")

	// Start background workers
	go service.startInvalidationWorker()
	go service.startStatsCollector()
	go service.startCDNPurgeWorker()

	// Start server
	server := &http.Server{
		Addr:         ":8080",
		Handler:      router,
		ReadTimeout:  30 * time.Second,
		WriteTimeout: 30 * time.Second,
		IdleTimeout:  60 * time.Second,
	}

	// Graceful shutdown
	go func() {
		log.Println("🚀 Cache Manager Service starting on :8080")
		if err := server.ListenAndServe(); err != nil && err != http.ErrServerClosed {
			log.Fatalf("❌ Server failed to start: %v", err)
		}
	}()

	// Wait for interrupt signal
	quit := make(chan os.Signal, 1)
	signal.Notify(quit, syscall.SIGINT, syscall.SIGTERM)
	<-quit

	log.Println("🛑 Shutting down Cache Manager Service...")
	ctx, cancel := context.WithTimeout(context.Background(), 30*time.Second)
	defer cancel()

	if err := server.Shutdown(ctx); err != nil {
		log.Fatalf("❌ Server forced to shutdown: %v", err)
	}

	log.Println("✅ Cache Manager Service stopped")
}

func initRedisCluster() *redis.ClusterClient {
	return redis.NewClusterClient(&redis.ClusterOptions{
		Addrs: []string{
			"redis-node-1:6379",
			"redis-node-2:6380",
			"redis-node-3:6381",
			"redis-node-4:6382",
			"redis-node-5:6383",
			"redis-node-6:6384",
		},
		Password:     getEnv("REDIS_PASSWORD", "atlasmesh_redis_2024!"),
		PoolSize:     100,
		MinIdleConns: 10,
		MaxRetries:   3,
		DialTimeout:  5 * time.Second,
		ReadTimeout:  3 * time.Second,
		WriteTimeout: 3 * time.Second,
		PoolTimeout:  4 * time.Second,
		IdleTimeout:  5 * time.Minute,
	})
}

func initRedisClient() *redis.Client {
	return redis.NewClient(&redis.Options{
		Addr:         getEnv("REDIS_ADDR", "localhost:6379"),
		Password:     getEnv("REDIS_PASSWORD", "atlasmesh_redis_2024!"),
		DB:           0,
		PoolSize:     50,
		MinIdleConns: 5,
		MaxRetries:   3,
		DialTimeout:  5 * time.Second,
		ReadTimeout:  3 * time.Second,
		WriteTimeout: 3 * time.Second,
		PoolTimeout:  4 * time.Second,
		IdleTimeout:  5 * time.Minute,
	})
}

func initCDNManager() *CDNManager {
	return &CDNManager{
		provider:   getEnv("CDN_PROVIDER", "cloudflare"),
		apiKey:     getEnv("CDN_API_KEY", ""),
		baseURL:    getEnv("CDN_BASE_URL", "https://api.cloudflare.com/client/v4"),
		zones:      make(map[string]CDNZone),
		purgeQueue: make(chan PurgeRequest, 1000),
	}
}

// Cache Operations
func (s *CacheManagerService) getCacheEntry(w http.ResponseWriter, r *http.Request) {
	vars := mux.Vars(r)
	key := vars["key"]
	namespace := r.URL.Query().Get("namespace")
	
	if namespace != "" {
		key = fmt.Sprintf("%s:%s", namespace, key)
	}

	ctx := context.Background()
	value, err := s.redisCluster.Get(ctx, key).Result()
	
	if err == redis.Nil {
		s.cacheStats.MissCount++
		s.sendJSON(w, http.StatusNotFound, CacheResponse{
			Success:   false,
			Error:     "Key not found",
			Cached:    false,
			Timestamp: time.Now(),
		})
		return
	}
	
	if err != nil {
		s.handleError(w, "Failed to get cache entry", err, http.StatusInternalServerError)
		return
	}

	s.cacheStats.HitCount++
	
	// Get TTL
	ttl, _ := s.redisCluster.TTL(ctx, key).Result()
	
	// Update access count and last access time
	s.redisCluster.HIncrBy(ctx, key+":meta", "access_count", 1)
	s.redisCluster.HSet(ctx, key+":meta", "last_access", time.Now().Unix())

	var data interface{}
	json.Unmarshal([]byte(value), &data)

	log.Printf("✅ Cache hit for key: %s", key)
	s.sendJSON(w, http.StatusOK, CacheResponse{
		Success:   true,
		Data:      data,
		Cached:    true,
		TTL:       int(ttl.Seconds()),
		Timestamp: time.Now(),
	})
}

func (s *CacheManagerService) setCacheEntry(w http.ResponseWriter, r *http.Request) {
	var operation CacheOperation
	if err := json.NewDecoder(r.Body).Decode(&operation); err != nil {
		s.handleError(w, "Invalid JSON payload", err, http.StatusBadRequest)
		return
	}

	key := operation.Key
	if operation.Namespace != "" {
		key = fmt.Sprintf("%s:%s", operation.Namespace, operation.Key)
	}

	// Serialize value
	valueBytes, err := json.Marshal(operation.Value)
	if err != nil {
		s.handleError(w, "Failed to serialize value", err, http.StatusBadRequest)
		return
	}

	ctx := context.Background()
	ttl := time.Duration(operation.TTL) * time.Second
	if ttl == 0 {
		ttl = 1 * time.Hour // Default TTL
	}

	// Set cache entry
	err = s.redisCluster.Set(ctx, key, valueBytes, ttl).Err()
	if err != nil {
		s.handleError(w, "Failed to set cache entry", err, http.StatusInternalServerError)
		return
	}

	// Set metadata
	metadata := map[string]interface{}{
		"created_at":    time.Now().Unix(),
		"expires_at":    time.Now().Add(ttl).Unix(),
		"access_count":  0,
		"last_access":   time.Now().Unix(),
		"namespace":     operation.Namespace,
		"tags":          strings.Join(operation.Tags, ","),
	}
	
	s.redisCluster.HMSet(ctx, key+":meta", metadata)
	s.redisCluster.Expire(ctx, key+":meta", ttl)

	// Add to tag indexes
	for _, tag := range operation.Tags {
		s.redisCluster.SAdd(ctx, fmt.Sprintf("tag:%s", tag), key)
		s.redisCluster.Expire(ctx, fmt.Sprintf("tag:%s", tag), ttl+time.Hour)
	}

	// Add to namespace index
	if operation.Namespace != "" {
		s.redisCluster.SAdd(ctx, fmt.Sprintf("namespace:%s", operation.Namespace), key)
	}

	s.cacheStats.SetCount++

	log.Printf("✅ Cache entry set: %s (TTL: %v)", key, ttl)
	s.sendJSON(w, http.StatusOK, CacheResponse{
		Success:   true,
		TTL:       operation.TTL,
		Timestamp: time.Now(),
	})
}

func (s *CacheManagerService) deleteCacheEntry(w http.ResponseWriter, r *http.Request) {
	vars := mux.Vars(r)
	key := vars["key"]
	namespace := r.URL.Query().Get("namespace")
	
	if namespace != "" {
		key = fmt.Sprintf("%s:%s", namespace, key)
	}

	ctx := context.Background()
	
	// Delete main key and metadata
	deleted, err := s.redisCluster.Del(ctx, key, key+":meta").Result()
	if err != nil {
		s.handleError(w, "Failed to delete cache entry", err, http.StatusInternalServerError)
		return
	}

	s.cacheStats.DeleteCount++

	log.Printf("✅ Cache entry deleted: %s", key)
	s.sendJSON(w, http.StatusOK, CacheResponse{
		Success:   deleted > 0,
		Timestamp: time.Now(),
	})
}

func (s *CacheManagerService) batchGetCache(w http.ResponseWriter, r *http.Request) {
	var request struct {
		Keys      []string `json:"keys"`
		Namespace string   `json:"namespace"`
	}

	if err := json.NewDecoder(r.Body).Decode(&request); err != nil {
		s.handleError(w, "Invalid JSON payload", err, http.StatusBadRequest)
		return
	}

	ctx := context.Background()
	results := make(map[string]interface{})
	
	// Prepare keys with namespace
	keys := make([]string, len(request.Keys))
	for i, key := range request.Keys {
		if request.Namespace != "" {
			keys[i] = fmt.Sprintf("%s:%s", request.Namespace, key)
		} else {
			keys[i] = key
		}
	}

	// Batch get
	values, err := s.redisCluster.MGet(ctx, keys...).Result()
	if err != nil {
		s.handleError(w, "Failed to batch get cache entries", err, http.StatusInternalServerError)
		return
	}

	for i, value := range values {
		if value != nil {
			var data interface{}
			json.Unmarshal([]byte(value.(string)), &data)
			results[request.Keys[i]] = data
			s.cacheStats.HitCount++
		} else {
			s.cacheStats.MissCount++
		}
	}

	log.Printf("✅ Batch cache get: %d keys, %d hits", len(request.Keys), len(results))
	s.sendJSON(w, http.StatusOK, CacheResponse{
		Success:   true,
		Data:      results,
		Timestamp: time.Now(),
	})
}

func (s *CacheManagerService) getNamespaceKeys(w http.ResponseWriter, r *http.Request) {
	vars := mux.Vars(r)
	namespace := vars["namespace"]

	ctx := context.Background()
	keys, err := s.redisCluster.SMembers(ctx, fmt.Sprintf("namespace:%s", namespace)).Result()
	if err != nil {
		s.handleError(w, "Failed to get namespace keys", err, http.StatusInternalServerError)
		return
	}

	log.Printf("✅ Retrieved %d keys for namespace: %s", len(keys), namespace)
	s.sendJSON(w, http.StatusOK, CacheResponse{
		Success:   true,
		Data:      keys,
		Timestamp: time.Now(),
	})
}

func (s *CacheManagerService) invalidateByTag(w http.ResponseWriter, r *http.Request) {
	vars := mux.Vars(r)
	tag := vars["tag"]

	ctx := context.Background()
	keys, err := s.redisCluster.SMembers(ctx, fmt.Sprintf("tag:%s", tag)).Result()
	if err != nil {
		s.handleError(w, "Failed to get keys by tag", err, http.StatusInternalServerError)
		return
	}

	if len(keys) == 0 {
		s.sendJSON(w, http.StatusOK, CacheResponse{
			Success:   true,
			Data:      map[string]int{"invalidated": 0},
			Timestamp: time.Now(),
		})
		return
	}

	// Delete all keys with metadata
	allKeys := make([]string, 0, len(keys)*2)
	for _, key := range keys {
		allKeys = append(allKeys, key, key+":meta")
	}

	deleted, err := s.redisCluster.Del(ctx, allKeys...).Result()
	if err != nil {
		s.handleError(w, "Failed to invalidate cache by tag", err, http.StatusInternalServerError)
		return
	}

	// Remove tag index
	s.redisCluster.Del(ctx, fmt.Sprintf("tag:%s", tag))

	log.Printf("✅ Invalidated %d cache entries for tag: %s", deleted/2, tag)
	s.sendJSON(w, http.StatusOK, CacheResponse{
		Success:   true,
		Data:      map[string]int{"invalidated": int(deleted / 2)},
		Timestamp: time.Now(),
	})
}

func (s *CacheManagerService) getCacheStats(w http.ResponseWriter, r *http.Request) {
	ctx := context.Background()
	
	// Get Redis info
	info, err := s.redisCluster.Info(ctx, "memory").Result()
	if err == nil {
		// Parse memory usage from info
		lines := strings.Split(info, "\r\n")
		for _, line := range lines {
			if strings.HasPrefix(line, "used_memory:") {
				if memStr := strings.Split(line, ":")[1]; memStr != "" {
					if mem, err := strconv.ParseInt(memStr, 10, 64); err == nil {
						s.cacheStats.MemoryUsage = mem
					}
				}
			}
		}
	}

	// Calculate hit ratio
	total := s.cacheStats.HitCount + s.cacheStats.MissCount
	if total > 0 {
		s.cacheStats.HitRatio = float64(s.cacheStats.HitCount) / float64(total)
	}

	// Get key count
	s.cacheStats.KeyCount, _ = s.redisCluster.DBSize(ctx).Result()

	log.Printf("✅ Cache statistics retrieved")
	s.sendJSON(w, http.StatusOK, s.cacheStats)
}

func (s *CacheManagerService) warmCache(w http.ResponseWriter, r *http.Request) {
	var request struct {
		Endpoints []string               `json:"endpoints"`
		Patterns  []string               `json:"patterns"`
		Metadata  map[string]interface{} `json:"metadata"`
	}

	if err := json.NewDecoder(r.Body).Decode(&request); err != nil {
		s.handleError(w, "Invalid JSON payload", err, http.StatusBadRequest)
		return
	}

	warmed := 0
	for _, endpoint := range request.Endpoints {
		// Make HTTP request to warm the cache
		resp, err := http.Get(endpoint)
		if err != nil {
			log.Printf("⚠️ Failed to warm cache for endpoint: %s, error: %v", endpoint, err)
			continue
		}
		resp.Body.Close()
		
		if resp.StatusCode == http.StatusOK {
			warmed++
		}
	}

	log.Printf("✅ Cache warmed for %d/%d endpoints", warmed, len(request.Endpoints))
	s.sendJSON(w, http.StatusOK, CacheResponse{
		Success:   true,
		Data:      map[string]int{"warmed": warmed, "total": len(request.Endpoints)},
		Timestamp: time.Now(),
	})
}

// Background Workers
func (s *CacheManagerService) startInvalidationWorker() {
	ticker := time.NewTicker(30 * time.Second)
	defer ticker.Stop()

	for {
		select {
		case <-ticker.C:
			// Check for expired keys and clean up indexes
			s.cleanupExpiredKeys()
		}
	}
}

func (s *CacheManagerService) startStatsCollector() {
	ticker := time.NewTicker(1 * time.Minute)
	defer ticker.Stop()

	for {
		select {
		case <-ticker.C:
			// Update cache statistics
			s.updateCacheStatistics()
		}
	}
}

func (s *CacheManagerService) startCDNPurgeWorker() {
	for purgeReq := range s.cdnManager.purgeQueue {
		s.processCDNPurge(purgeReq)
	}
}

func (s *CacheManagerService) cleanupExpiredKeys() {
	ctx := context.Background()
	
	// Scan for expired metadata keys
	iter := s.redisCluster.Scan(ctx, 0, "*:meta", 1000).Iterator()
	for iter.Next(ctx) {
		metaKey := iter.Val()
		key := strings.TrimSuffix(metaKey, ":meta")
		
		// Check if main key exists
		exists, err := s.redisCluster.Exists(ctx, key).Result()
		if err != nil || exists == 0 {
			// Clean up orphaned metadata
			s.redisCluster.Del(ctx, metaKey)
		}
	}
}

func (s *CacheManagerService) updateCacheStatistics() {
	ctx := context.Background()
	
	// Update namespace statistics
	for namespace := range s.cacheStats.NamespaceStats {
		keys, _ := s.redisCluster.SMembers(ctx, fmt.Sprintf("namespace:%s", namespace)).Result()
		
		stats := NamespaceStats{
			KeyCount: int64(len(keys)),
		}
		
		s.cacheStats.NamespaceStats[namespace] = stats
	}
}

func (s *CacheManagerService) processCDNPurge(req PurgeRequest) {
	// Implementation would depend on CDN provider
	log.Printf("🔄 Processing CDN purge request for zone: %s", req.ZoneID)
	
	// Mock CDN purge - in production, this would call the actual CDN API
	time.Sleep(100 * time.Millisecond)
	
	log.Printf("✅ CDN purge completed for zone: %s", req.ZoneID)
}

// CDN Operations
func (s *CacheManagerService) purgeCDN(w http.ResponseWriter, r *http.Request) {
	var request PurgeRequest
	if err := json.NewDecoder(r.Body).Decode(&request); err != nil {
		s.handleError(w, "Invalid JSON payload", err, http.StatusBadRequest)
		return
	}

	// Queue purge request
	select {
	case s.cdnManager.purgeQueue <- request:
		log.Printf("✅ CDN purge request queued for zone: %s", request.ZoneID)
		s.sendJSON(w, http.StatusAccepted, CacheResponse{
			Success:   true,
			Data:      map[string]string{"status": "queued"},
			Timestamp: time.Now(),
		})
	default:
		s.handleError(w, "CDN purge queue full", nil, http.StatusServiceUnavailable)
	}
}

func (s *CacheManagerService) getCDNZones(w http.ResponseWriter, r *http.Request) {
	zones := make([]CDNZone, 0, len(s.cdnManager.zones))
	for _, zone := range s.cdnManager.zones {
		zones = append(zones, zone)
	}

	log.Printf("✅ Retrieved %d CDN zones", len(zones))
	s.sendJSON(w, http.StatusOK, CacheResponse{
		Success:   true,
		Data:      zones,
		Timestamp: time.Now(),
	})
}

func (s *CacheManagerService) getCDNStats(w http.ResponseWriter, r *http.Request) {
	// Mock CDN stats - in production, this would fetch from CDN provider
	stats := map[string]interface{}{
		"total_requests":    1250000,
		"cache_hit_ratio":   0.87,
		"bandwidth_saved":   "2.5TB",
		"avg_response_time": "45ms",
		"zones_count":       len(s.cdnManager.zones),
		"purge_queue_size":  len(s.cdnManager.purgeQueue),
	}

	log.Printf("✅ CDN statistics retrieved")
	s.sendJSON(w, http.StatusOK, CacheResponse{
		Success:   true,
		Data:      stats,
		Timestamp: time.Now(),
	})
}

// Utility Methods
func (s *CacheManagerService) healthCheck(w http.ResponseWriter, r *http.Request) {
	ctx := context.Background()
	
	// Check Redis cluster health
	_, err := s.redisCluster.Ping(ctx).Result()
	redisHealthy := err == nil

	health := map[string]interface{}{
		"status":    "healthy",
		"timestamp": time.Now(),
		"components": map[string]interface{}{
			"redis_cluster": map[string]interface{}{
				"status": func() string {
					if redisHealthy {
						return "healthy"
					}
					return "unhealthy"
				}(),
				"hit_ratio":    s.cacheStats.HitRatio,
				"memory_usage": s.cacheStats.MemoryUsage,
				"key_count":    s.cacheStats.KeyCount,
			},
			"cdn_manager": map[string]interface{}{
				"provider":         s.cdnManager.provider,
				"zones_count":      len(s.cdnManager.zones),
				"purge_queue_size": len(s.cdnManager.purgeQueue),
			},
		},
		"version": "1.0.0",
	}

	if !redisHealthy {
		w.WriteHeader(http.StatusServiceUnavailable)
	}

	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(health)
}

func (s *CacheManagerService) handleError(w http.ResponseWriter, message string, err error, statusCode int) {
	if err != nil {
		log.Printf("❌ %s: %v", message, err)
	} else {
		log.Printf("❌ %s", message)
	}

	response := CacheResponse{
		Success:   false,
		Error:     message,
		Timestamp: time.Now(),
	}

	w.Header().Set("Content-Type", "application/json")
	w.WriteHeader(statusCode)
	json.NewEncoder(w).Encode(response)
}

func (s *CacheManagerService) sendJSON(w http.ResponseWriter, statusCode int, data interface{}) {
	w.Header().Set("Content-Type", "application/json")
	w.WriteHeader(statusCode)
	json.NewEncoder(w).Encode(data)
}

// Placeholder methods for missing handlers
func (s *CacheManagerService) checkCacheExists(w http.ResponseWriter, r *http.Request) {
	vars := mux.Vars(r)
	key := vars["key"]
	namespace := r.URL.Query().Get("namespace")
	
	if namespace != "" {
		key = fmt.Sprintf("%s:%s", namespace, key)
	}

	ctx := context.Background()
	exists, err := s.redisCluster.Exists(ctx, key).Result()
	if err != nil {
		s.handleError(w, "Failed to check cache existence", err, http.StatusInternalServerError)
		return
	}

	s.sendJSON(w, http.StatusOK, CacheResponse{
		Success:   true,
		Data:      map[string]bool{"exists": exists > 0},
		Timestamp: time.Now(),
	})
}

func (s *CacheManagerService) getCacheTTL(w http.ResponseWriter, r *http.Request) {
	vars := mux.Vars(r)
	key := vars["key"]
	namespace := r.URL.Query().Get("namespace")
	
	if namespace != "" {
		key = fmt.Sprintf("%s:%s", namespace, key)
	}

	ctx := context.Background()
	ttl, err := s.redisCluster.TTL(ctx, key).Result()
	if err != nil {
		s.handleError(w, "Failed to get cache TTL", err, http.StatusInternalServerError)
		return
	}

	s.sendJSON(w, http.StatusOK, CacheResponse{
		Success:   true,
		Data:      map[string]int{"ttl": int(ttl.Seconds())},
		Timestamp: time.Now(),
	})
}

func (s *CacheManagerService) batchSetCache(w http.ResponseWriter, r *http.Request) {
	// Implementation for batch set operations
	s.sendJSON(w, http.StatusOK, CacheResponse{
		Success:   true,
		Data:      map[string]string{"status": "not_implemented"},
		Timestamp: time.Now(),
	})
}

func (s *CacheManagerService) batchDeleteCache(w http.ResponseWriter, r *http.Request) {
	// Implementation for batch delete operations
	s.sendJSON(w, http.StatusOK, CacheResponse{
		Success:   true,
		Data:      map[string]string{"status": "not_implemented"},
		Timestamp: time.Now(),
	})
}

func (s *CacheManagerService) flushNamespace(w http.ResponseWriter, r *http.Request) {
	// Implementation for namespace flush
	s.sendJSON(w, http.StatusOK, CacheResponse{
		Success:   true,
		Data:      map[string]string{"status": "not_implemented"},
		Timestamp: time.Now(),
	})
}

func (s *CacheManagerService) getNamespaceStats(w http.ResponseWriter, r *http.Request) {
	// Implementation for namespace statistics
	s.sendJSON(w, http.StatusOK, CacheResponse{
		Success:   true,
		Data:      map[string]string{"status": "not_implemented"},
		Timestamp: time.Now(),
	})
}

func (s *CacheManagerService) getKeysByTag(w http.ResponseWriter, r *http.Request) {
	// Implementation for getting keys by tag
	s.sendJSON(w, http.StatusOK, CacheResponse{
		Success:   true,
		Data:      map[string]string{"status": "not_implemented"},
		Timestamp: time.Now(),
	})
}

func (s *CacheManagerService) resetCacheStats(w http.ResponseWriter, r *http.Request) {
	// Reset cache statistics
	s.cacheStats = &CacheStatistics{
		NamespaceStats: make(map[string]NamespaceStats),
		LastReset:      time.Now(),
	}
	
	s.sendJSON(w, http.StatusOK, CacheResponse{
		Success:   true,
		Data:      map[string]string{"status": "reset"},
		Timestamp: time.Now(),
	})
}

func (s *CacheManagerService) invalidateCache(w http.ResponseWriter, r *http.Request) {
	// Implementation for cache invalidation
	s.sendJSON(w, http.StatusOK, CacheResponse{
		Success:   true,
		Data:      map[string]string{"status": "not_implemented"},
		Timestamp: time.Now(),
	})
}

func (s *CacheManagerService) getInvalidationRules(w http.ResponseWriter, r *http.Request) {
	// Implementation for getting invalidation rules
	s.sendJSON(w, http.StatusOK, CacheResponse{
		Success:   true,
		Data:      map[string]string{"status": "not_implemented"},
		Timestamp: time.Now(),
	})
}

func (s *CacheManagerService) createInvalidationRule(w http.ResponseWriter, r *http.Request) {
	// Implementation for creating invalidation rules
	s.sendJSON(w, http.StatusOK, CacheResponse{
		Success:   true,
		Data:      map[string]string{"status": "not_implemented"},
		Timestamp: time.Now(),
	})
}

func (s *CacheManagerService) preloadCache(w http.ResponseWriter, r *http.Request) {
	// Implementation for cache preloading
	s.sendJSON(w, http.StatusOK, CacheResponse{
		Success:   true,
		Data:      map[string]string{"status": "not_implemented"},
		Timestamp: time.Now(),
	})
}

func getEnv(key, defaultValue string) string {
	if value := os.Getenv(key); value != "" {
		return value
	}
	return defaultValue
}
