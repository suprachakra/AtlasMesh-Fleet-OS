package main

import (
	"context"
	"encoding/json"
	"fmt"
	"log"
	"net/http"
	"os"
	"os/signal"
	"sync"
	"syscall"
	"time"

	"github.com/gin-gonic/gin"
	"github.com/gorilla/websocket"
	"go.uber.org/zap"
	"go.uber.org/zap/zapcore"

	"atlasmesh/hd-map-service/internal/config"
	"atlasmesh/hd-map-service/internal/database"
	"atlasmesh/hd-map-service/internal/logger"
	"atlasmesh/hd-map-service/internal/metrics"
	"atlasmesh/hd-map-service/internal/mapbuilder"
	"atlasmesh/hd-map-service/internal/mapserver"
	"atlasmesh/hd-map-service/internal/storage"
	"atlasmesh/hd-map-service/internal/validation"
)

// HDMapService represents the main HD map service
type HDMapService struct {
	config     *config.Config
	logger     *zap.Logger
	db         *database.Database
	storage    *storage.Storage
	metrics    *metrics.Metrics
	mapBuilder *mapbuilder.Engine
	mapServer  *mapserver.Engine
	validator  *validation.Validator
	router     *gin.Engine
	wsUpgrader websocket.Upgrader
	ctx        context.Context
	cancel     context.CancelFunc
	wg         sync.WaitGroup
}

// MapTile represents a map tile
type MapTile struct {
	Z        int                    `json:"z"`
	X        int                    `json:"x"`
	Y        int                    `json:"y"`
	Data     []byte                 `json:"data"`
	Metadata map[string]interface{} `json:"metadata"`
	Version  string                 `json:"version"`
	Created  time.Time              `json:"created"`
	Updated  time.Time              `json:"updated"`
}

// Lane represents a lane in the HD map
type Lane struct {
	ID           string                 `json:"id"`
	RoadID       string                 `json:"road_id"`
	LaneIndex    int                    `json:"lane_index"`
	Geometry     []Point                `json:"geometry"`
	Width        float64                `json:"width"`
	Length       float64                `json:"length"`
	Type         string                 `json:"type"` // DRIVING, TURNING, MERGING, EXIT
	Direction    string                 `json:"direction"` // FORWARD, BACKWARD, BIDIRECTIONAL
	SpeedLimit   float64                `json:"speed_limit_mps"`
	Restrictions []string               `json:"restrictions"`
	Connections  []LaneConnection       `json:"connections"`
	Markings     []LaneMarking          `json:"markings"`
	Properties   map[string]interface{} `json:"properties"`
	Created      time.Time              `json:"created"`
	Updated      time.Time              `json:"updated"`
}

type Point struct {
	Latitude  float64 `json:"latitude"`
	Longitude float64 `json:"longitude"`
	Altitude  float64 `json:"altitude"`
}

type LaneConnection struct {
	FromLaneID string  `json:"from_lane_id"`
	ToLaneID   string  `json:"to_lane_id"`
	Type       string  `json:"type"` // STRAIGHT, LEFT_TURN, RIGHT_TURN, U_TURN
	Confidence float64 `json:"confidence"`
}

type LaneMarking struct {
	Type     string  `json:"type"` // SOLID, DASHED, DOUBLE
	Color    string  `json:"color"` // WHITE, YELLOW
	Width    float64 `json:"width"`
	Position []Point `json:"position"`
}

// TrafficSign represents a traffic sign
type TrafficSign struct {
	ID          string                 `json:"id"`
	Type        string                 `json:"type"` // STOP, YIELD, SPEED_LIMIT, NO_ENTRY
	Position    Point                  `json:"position"`
	Heading     float64                `json:"heading_deg"`
	Height      float64                `json:"height_m"`
	Value       string                 `json:"value"`
	Regulations []string               `json:"regulations"`
	Geometry    []Point                `json:"geometry"`
	Properties  map[string]interface{} `json:"properties"`
	Created     time.Time              `json:"created"`
	Updated     time.Time              `json:"updated"`
}

// Road represents a road segment
type Road struct {
	ID          string                 `json:"id"`
	Name        string                 `json:"name"`
	Type        string                 `json:"type"` // HIGHWAY, ARTERIAL, COLLECTOR, LOCAL
	Geometry    []Point                `json:"geometry"`
	Length      float64                `json:"length_m"`
	Width       float64                `json:"width_m"`
	Lanes       []Lane                 `json:"lanes"`
	SpeedLimit  float64                `json:"speed_limit_mps"`
	Surface     string                 `json:"surface"` // ASPHALT, CONCRETE, GRAVEL
	Properties  map[string]interface{} `json:"properties"`
	Created     time.Time              `json:"created"`
	Updated     time.Time              `json:"updated"`
}

// Intersection represents an intersection
type Intersection struct {
	ID          string                 `json:"id"`
	Position    Point                  `json:"position"`
	Type        string                 `json:"type"` // SIGNALIZED, STOP_SIGN, YIELD, ROUNDABOUT
	Geometry    []Point                `json:"geometry"`
	Lanes       []Lane                 `json:"lanes"`
	Signals     []TrafficSignal        `json:"signals"`
	Properties  map[string]interface{} `json:"properties"`
	Created     time.Time              `json:"created"`
	Updated     time.Time              `json:"updated"`
}

type TrafficSignal struct {
	ID       string `json:"id"`
	Position Point  `json:"position"`
	Type     string `json:"type"` // TRAFFIC_LIGHT, STOP_SIGN, YIELD_SIGN
	State    string `json:"state"` // RED, YELLOW, GREEN, FLASHING
}

// MapRegion represents a map region
type MapRegion struct {
	ID          string    `json:"id"`
	Name        string    `json:"name"`
	Bounds      Bounds    `json:"bounds"`
	Center      Point     `json:"center"`
	Zoom        int       `json:"zoom"`
	Lanes       []Lane    `json:"lanes"`
	Roads       []Road    `json:"roads"`
	Intersections []Intersection `json:"intersections"`
	TrafficSigns []TrafficSign `json:"traffic_signs"`
	Version     string    `json:"version"`
	Created     time.Time `json:"created"`
	Updated     time.Time `json:"updated"`
}

type Bounds struct {
	North float64 `json:"north"`
	South float64 `json:"south"`
	East  float64 `json:"east"`
	West  float64 `json:"west"`
}

// GeospatialQuery represents a geospatial query
type GeospatialQuery struct {
	Type        string                 `json:"type"` // NEARBY, ROUTE, INTERSECTION, LANE_CHANGE
	Position    Point                  `json:"position"`
	Radius      float64                `json:"radius_m"`
	Filters     map[string]interface{} `json:"filters"`
	Limit       int                    `json:"limit"`
	Offset      int                    `json:"offset"`
}

// MapUpdate represents a map update
type MapUpdate struct {
	ID          string                 `json:"id"`
	Type        string                 `json:"type"` // TILE, LANE, ROAD, INTERSECTION, TRAFFIC_SIGN
	Data        map[string]interface{} `json:"data"`
	Version     string                 `json:"version"`
	Source      string                 `json:"source"` // VEHICLE, SURVEY, CROWD, ADMIN
	Priority    int                    `json:"priority"`
	Timestamp   time.Time              `json:"timestamp"`
	Status      string                 `json:"status"` // PENDING, PROCESSING, COMPLETED, FAILED
}

func main() {
	// Load configuration
	cfg, err := config.Load()
	if err != nil {
		log.Fatalf("Failed to load configuration: %v", err)
	}

	// Initialize logger
	logger, err := logger.New(cfg.LogLevel)
	if err != nil {
		log.Fatalf("Failed to initialize logger: %v", err)
	}
	defer logger.Sync()

	// Initialize database
	db, err := database.New(cfg.Database)
	if err != nil {
		logger.Fatal("Failed to initialize database", zap.Error(err))
	}
	defer db.Close()

	// Initialize storage
	storage, err := storage.New(cfg.Storage)
	if err != nil {
		logger.Fatal("Failed to initialize storage", zap.Error(err))
	}

	// Initialize metrics
	metrics := metrics.New(cfg.Metrics)

	// Create context
	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()

	// Initialize service
	service := &HDMapService{
		config:  cfg,
		logger:  logger,
		db:      db,
		storage: storage,
		metrics: metrics,
		ctx:     ctx,
		cancel:  cancel,
		wsUpgrader: websocket.Upgrader{
			CheckOrigin: func(r *http.Request) bool {
				return true // In production, implement proper origin checking
			},
		},
	}

	// Initialize modules
	if err := service.initializeModules(); err != nil {
		logger.Fatal("Failed to initialize modules", zap.Error(err))
	}

	// Setup routes
	service.setupRoutes()

	// Start service
	if err := service.start(); err != nil {
		logger.Fatal("Failed to start service", zap.Error(err))
	}

	// Wait for shutdown signal
	service.waitForShutdown()
}

func (s *HDMapService) initializeModules() error {
	s.logger.Info("Initializing HD map service modules")

	// Initialize map builder
	mapBuilder, err := mapbuilder.New(s.config.MapBuilder, s.logger, s.db, s.storage)
	if err != nil {
		return fmt.Errorf("failed to initialize map builder: %w", err)
	}
	s.mapBuilder = mapBuilder

	// Initialize map server
	mapServer, err := mapserver.New(s.config.MapServer, s.logger, s.db, s.storage)
	if err != nil {
		return fmt.Errorf("failed to initialize map server: %w", err)
	}
	s.mapServer = mapServer

	// Initialize validator
	validator, err := validation.New(s.config.Validation, s.logger)
	if err != nil {
		return fmt.Errorf("failed to initialize validator: %w", err)
	}
	s.validator = validator

	s.logger.Info("All modules initialized successfully")
	return nil
}

func (s *HDMapService) setupRoutes() {
	s.router = gin.New()
	s.router.Use(gin.Recovery())
	s.router.Use(s.corsMiddleware())
	s.router.Use(s.loggingMiddleware())

	// Health check
	s.router.GET("/health", s.healthCheck)

	// Map data endpoints
	mapData := s.router.Group("/api/v1/hd-map")
	{
		mapData.GET("/tiles/:z/:x/:y", s.getMapTile)
		mapData.GET("/region", s.getMapRegion)
		mapData.GET("/lanes", s.getLanes)
		mapData.GET("/traffic-signs", s.getTrafficSigns)
		mapData.GET("/road-network", s.getRoadNetwork)
	}

	// Map management endpoints
	mapMgmt := s.router.Group("/api/v1/hd-map")
	{
		mapMgmt.POST("/upload", s.uploadMapData)
		mapMgmt.PUT("/update", s.updateMapData)
		mapMgmt.DELETE("/delete", s.deleteMapData)
		mapMgmt.GET("/versions", s.getMapVersions)
		mapMgmt.POST("/validate", s.validateMapData)
	}

	// Geospatial query endpoints
	geoQuery := s.router.Group("/api/v1/hd-map/query")
	{
		geoQuery.POST("/nearby", s.queryNearby)
		geoQuery.POST("/route", s.queryRoute)
		geoQuery.POST("/intersection", s.queryIntersection)
		geoQuery.POST("/lane-change", s.queryLaneChange)
	}

	// WebSocket endpoints
	s.router.GET("/ws/map-updates", s.handleMapUpdatesWebSocket)
}

func (s *HDMapService) start() error {
	s.logger.Info("Starting HD map service")

	// Start map builder
	s.wg.Add(1)
	go s.startMapBuilder()

	// Start map server
	s.wg.Add(1)
	go s.startMapServer()

	// Start HTTP server
	s.wg.Add(1)
	go s.startHTTPServer()

	s.logger.Info("HD map service started successfully")
	return nil
}

func (s *HDMapService) startMapBuilder() {
	defer s.wg.Done()
	s.logger.Info("Starting map builder")
	
	if err := s.mapBuilder.Start(s.ctx); err != nil {
		s.logger.Error("Map builder failed", zap.Error(err))
	}
}

func (s *HDMapService) startMapServer() {
	defer s.wg.Done()
	s.logger.Info("Starting map server")
	
	if err := s.mapServer.Start(s.ctx); err != nil {
		s.logger.Error("Map server failed", zap.Error(err))
	}
}

func (s *HDMapService) startHTTPServer() {
	defer s.wg.Done()
	
	server := &http.Server{
		Addr:    fmt.Sprintf(":%d", s.config.Server.Port),
		Handler: s.router,
	}

	s.logger.Info("Starting HTTP server", zap.Int("port", s.config.Server.Port))

	if err := server.ListenAndServe(); err != nil && err != http.ErrServerClosed {
		s.logger.Error("HTTP server failed", zap.Error(err))
	}
}

func (s *HDMapService) waitForShutdown() {
	// Wait for interrupt signal
	quit := make(chan os.Signal, 1)
	signal.Notify(quit, syscall.SIGINT, syscall.SIGTERM)
	<-quit

	s.logger.Info("Shutting down HD map service")

	// Cancel context
	s.cancel()

	// Wait for all goroutines to finish
	s.wg.Wait()

	s.logger.Info("HD map service stopped")
}

// HTTP Handlers

func (s *HDMapService) healthCheck(c *gin.Context) {
	c.JSON(http.StatusOK, gin.H{
		"status":    "healthy",
		"timestamp": time.Now(),
		"service":   "hd-map-service",
	})
}

func (s *HDMapService) getMapTile(c *gin.Context) {
	z := c.Param("z")
	x := c.Param("x")
	y := c.Param("y")

	// Get map tile
	tile, err := s.mapServer.GetTile(z, x, y)
	if err != nil {
		s.logger.Error("Failed to get map tile", zap.Error(err))
		c.JSON(http.StatusInternalServerError, gin.H{"error": "Failed to get map tile"})
		return
	}

	if tile == nil {
		c.JSON(http.StatusNotFound, gin.H{"error": "Map tile not found"})
		return
	}

	c.Data(http.StatusOK, "application/octet-stream", tile.Data)
}

func (s *HDMapService) getMapRegion(c *gin.Context) {
	var bounds Bounds
	if err := c.ShouldBindQuery(&bounds); err != nil {
		c.JSON(http.StatusBadRequest, gin.H{"error": err.Error()})
		return
	}

	// Get map region
	region, err := s.mapServer.GetRegion(bounds)
	if err != nil {
		s.logger.Error("Failed to get map region", zap.Error(err))
		c.JSON(http.StatusInternalServerError, gin.H{"error": "Failed to get map region"})
		return
	}

	c.JSON(http.StatusOK, region)
}

func (s *HDMapService) getLanes(c *gin.Context) {
	var bounds Bounds
	if err := c.ShouldBindQuery(&bounds); err != nil {
		c.JSON(http.StatusBadRequest, gin.H{"error": err.Error()})
		return
	}

	// Get lanes
	lanes, err := s.mapServer.GetLanes(bounds)
	if err != nil {
		s.logger.Error("Failed to get lanes", zap.Error(err))
		c.JSON(http.StatusInternalServerError, gin.H{"error": "Failed to get lanes"})
		return
	}

	c.JSON(http.StatusOK, gin.H{
		"lanes":     lanes,
		"timestamp": time.Now(),
	})
}

func (s *HDMapService) getTrafficSigns(c *gin.Context) {
	var bounds Bounds
	if err := c.ShouldBindQuery(&bounds); err != nil {
		c.JSON(http.StatusBadRequest, gin.H{"error": err.Error()})
		return
	}

	// Get traffic signs
	signs, err := s.mapServer.GetTrafficSigns(bounds)
	if err != nil {
		s.logger.Error("Failed to get traffic signs", zap.Error(err))
		c.JSON(http.StatusInternalServerError, gin.H{"error": "Failed to get traffic signs"})
		return
	}

	c.JSON(http.StatusOK, gin.H{
		"traffic_signs": signs,
		"timestamp":     time.Now(),
	})
}

func (s *HDMapService) getRoadNetwork(c *gin.Context) {
	var bounds Bounds
	if err := c.ShouldBindQuery(&bounds); err != nil {
		c.JSON(http.StatusBadRequest, gin.H{"error": err.Error()})
		return
	}

	// Get road network
	roads, err := s.mapServer.GetRoadNetwork(bounds)
	if err != nil {
		s.logger.Error("Failed to get road network", zap.Error(err))
		c.JSON(http.StatusInternalServerError, gin.H{"error": "Failed to get road network"})
		return
	}

	c.JSON(http.StatusOK, gin.H{
		"roads":     roads,
		"timestamp": time.Now(),
	})
}

func (s *HDMapService) uploadMapData(c *gin.Context) {
	var update MapUpdate
	if err := c.ShouldBindJSON(&update); err != nil {
		c.JSON(http.StatusBadRequest, gin.H{"error": err.Error()})
		return
	}

	// Upload map data
	if err := s.mapBuilder.UploadMapData(update); err != nil {
		s.logger.Error("Failed to upload map data", zap.Error(err))
		c.JSON(http.StatusInternalServerError, gin.H{"error": "Failed to upload map data"})
		return
	}

	c.JSON(http.StatusOK, gin.H{
		"status":    "uploaded",
		"timestamp": time.Now(),
	})
}

func (s *HDMapService) updateMapData(c *gin.Context) {
	var update MapUpdate
	if err := c.ShouldBindJSON(&update); err != nil {
		c.JSON(http.StatusBadRequest, gin.H{"error": err.Error()})
		return
	}

	// Update map data
	if err := s.mapBuilder.UpdateMapData(update); err != nil {
		s.logger.Error("Failed to update map data", zap.Error(err))
		c.JSON(http.StatusInternalServerError, gin.H{"error": "Failed to update map data"})
		return
	}

	c.JSON(http.StatusOK, gin.H{
		"status":    "updated",
		"timestamp": time.Now(),
	})
}

func (s *HDMapService) deleteMapData(c *gin.Context) {
	var update MapUpdate
	if err := c.ShouldBindJSON(&update); err != nil {
		c.JSON(http.StatusBadRequest, gin.H{"error": err.Error()})
		return
	}

	// Delete map data
	if err := s.mapBuilder.DeleteMapData(update); err != nil {
		s.logger.Error("Failed to delete map data", zap.Error(err))
		c.JSON(http.StatusInternalServerError, gin.H{"error": "Failed to delete map data"})
		return
	}

	c.JSON(http.StatusOK, gin.H{
		"status":    "deleted",
		"timestamp": time.Now(),
	})
}

func (s *HDMapService) getMapVersions(c *gin.Context) {
	// Get map versions
	versions, err := s.mapBuilder.GetMapVersions()
	if err != nil {
		s.logger.Error("Failed to get map versions", zap.Error(err))
		c.JSON(http.StatusInternalServerError, gin.H{"error": "Failed to get map versions"})
		return
	}

	c.JSON(http.StatusOK, gin.H{
		"versions":  versions,
		"timestamp": time.Now(),
	})
}

func (s *HDMapService) validateMapData(c *gin.Context) {
	var update MapUpdate
	if err := c.ShouldBindJSON(&update); err != nil {
		c.JSON(http.StatusBadRequest, gin.H{"error": err.Error()})
		return
	}

	// Validate map data
	validation, err := s.validator.ValidateMapData(update)
	if err != nil {
		s.logger.Error("Failed to validate map data", zap.Error(err))
		c.JSON(http.StatusInternalServerError, gin.H{"error": "Failed to validate map data"})
		return
	}

	c.JSON(http.StatusOK, gin.H{
		"validation": validation,
		"timestamp":  time.Now(),
	})
}

func (s *HDMapService) queryNearby(c *gin.Context) {
	var query GeospatialQuery
	if err := c.ShouldBindJSON(&query); err != nil {
		c.JSON(http.StatusBadRequest, gin.H{"error": err.Error()})
		return
	}

	// Query nearby features
	features, err := s.mapServer.QueryNearby(query)
	if err != nil {
		s.logger.Error("Failed to query nearby features", zap.Error(err))
		c.JSON(http.StatusInternalServerError, gin.H{"error": "Failed to query nearby features"})
		return
	}

	c.JSON(http.StatusOK, gin.H{
		"features":  features,
		"timestamp": time.Now(),
	})
}

func (s *HDMapService) queryRoute(c *gin.Context) {
	var query GeospatialQuery
	if err := c.ShouldBindJSON(&query); err != nil {
		c.JSON(http.StatusBadRequest, gin.H{"error": err.Error()})
		return
	}

	// Query route geometry
	route, err := s.mapServer.QueryRoute(query)
	if err != nil {
		s.logger.Error("Failed to query route", zap.Error(err))
		c.JSON(http.StatusInternalServerError, gin.H{"error": "Failed to query route"})
		return
	}

	c.JSON(http.StatusOK, gin.H{
		"route":     route,
		"timestamp": time.Now(),
	})
}

func (s *HDMapService) queryIntersection(c *gin.Context) {
	var query GeospatialQuery
	if err := c.ShouldBindJSON(&query); err != nil {
		c.JSON(http.StatusBadRequest, gin.H{"error": err.Error()})
		return
	}

	// Query intersection data
	intersection, err := s.mapServer.QueryIntersection(query)
	if err != nil {
		s.logger.Error("Failed to query intersection", zap.Error(err))
		c.JSON(http.StatusInternalServerError, gin.H{"error": "Failed to query intersection"})
		return
	}

	c.JSON(http.StatusOK, gin.H{
		"intersection": intersection,
		"timestamp":    time.Now(),
	})
}

func (s *HDMapService) queryLaneChange(c *gin.Context) {
	var query GeospatialQuery
	if err := c.ShouldBindJSON(&query); err != nil {
		c.JSON(http.StatusBadRequest, gin.H{"error": err.Error()})
		return
	}

	// Query lane change options
	options, err := s.mapServer.QueryLaneChange(query)
	if err != nil {
		s.logger.Error("Failed to query lane change options", zap.Error(err))
		c.JSON(http.StatusInternalServerError, gin.H{"error": "Failed to query lane change options"})
		return
	}

	c.JSON(http.StatusOK, gin.H{
		"options":   options,
		"timestamp": time.Now(),
	})
}

// WebSocket Handlers

func (s *HDMapService) handleMapUpdatesWebSocket(c *gin.Context) {
	conn, err := s.wsUpgrader.Upgrade(c.Writer, c.Request, nil)
	if err != nil {
		s.logger.Error("WebSocket upgrade failed", zap.Error(err))
		return
	}
	defer conn.Close()

	s.logger.Info("Map updates WebSocket connected")

	// Send map updates
	ticker := time.NewTicker(1 * time.Second)
	defer ticker.Stop()

	for {
		select {
		case <-s.ctx.Done():
			return
		case <-ticker.C:
			// Get latest map updates
			updates, err := s.mapBuilder.GetLatestUpdates()
			if err != nil {
				s.logger.Error("Failed to get latest updates", zap.Error(err))
				continue
			}

			if len(updates) > 0 {
				if err := conn.WriteJSON(updates); err != nil {
					s.logger.Error("WebSocket write failed", zap.Error(err))
					return
				}
			}
		}
	}
}

// Middleware

func (s *HDMapService) corsMiddleware() gin.HandlerFunc {
	return func(c *gin.Context) {
		c.Header("Access-Control-Allow-Origin", "*")
		c.Header("Access-Control-Allow-Methods", "GET, POST, PUT, DELETE, OPTIONS")
		c.Header("Access-Control-Allow-Headers", "Origin, Content-Type, Accept, Authorization")

		if c.Request.Method == "OPTIONS" {
			c.AbortWithStatus(204)
			return
		}

		c.Next()
	}
}

func (s *HDMapService) loggingMiddleware() gin.HandlerFunc {
	return gin.LoggerWithFormatter(func(param gin.LogFormatterParams) string {
		s.logger.Info("HTTP Request",
			zap.String("method", param.Method),
			zap.String("path", param.Path),
			zap.Int("status", param.StatusCode),
			zap.Duration("latency", param.Latency),
			zap.String("client_ip", param.ClientIP),
		)
		return ""
	})
}
