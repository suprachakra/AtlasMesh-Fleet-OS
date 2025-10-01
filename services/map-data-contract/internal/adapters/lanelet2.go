package adapters

import (
	"context"
	"encoding/json"
	"fmt"
	"log"
	"time"

	"map-data-contract/internal/config"
	"map-data-contract/internal/models"
	"map-data-contract/internal/provenance"
)

// Lanelet2Adapter provides access to Lanelet2 HD map format
// Handles parsing, validation, and conversion of Lanelet2 maps
// Optimized for Abu Dhabi autonomous vehicle operations
type Lanelet2Adapter struct {
	config            *config.Lanelet2Config
	provenanceTracker *provenance.Tracker
	parser            *Lanelet2Parser
	validator         *Lanelet2Validator
	converter         *Lanelet2Converter
	cache             map[string]*models.Lanelet2Map
}

// Lanelet2Map represents a parsed Lanelet2 map
type Lanelet2Map struct {
	ID          string                 `json:"id"`
	Version     string                 `json:"version"`
	Metadata    *Lanelet2Metadata      `json:"metadata"`
	Nodes       map[int64]*Node        `json:"nodes"`
	Ways        map[int64]*Way         `json:"ways"`
	Relations   map[int64]*Relation    `json:"relations"`
	Lanelets    map[int64]*Lanelet     `json:"lanelets"`
	RegElements map[int64]*RegElement  `json:"regulatory_elements"`
	Areas       map[int64]*Area        `json:"areas"`
	Linestrings map[int64]*Linestring  `json:"linestrings"`
	Polygons    map[int64]*Polygon     `json:"polygons"`
	RoutingGraph *RoutingGraph         `json:"routing_graph,omitempty"`
	CreatedAt   time.Time              `json:"created_at"`
	UpdatedAt   time.Time              `json:"updated_at"`
}

// Lanelet2Metadata contains map metadata
type Lanelet2Metadata struct {
	Name        string            `json:"name"`
	Description string            `json:"description"`
	Region      string            `json:"region"`
	Country     string            `json:"country"`
	Projection  string            `json:"projection"`
	Origin      *GeographicPoint  `json:"origin"`
	Bounds      *BoundingBox      `json:"bounds"`
	Version     string            `json:"version"`
	Created     time.Time         `json:"created"`
	Modified    time.Time         `json:"modified"`
	Authors     []string          `json:"authors"`
	Tags        map[string]string `json:"tags"`
}

// Node represents a Lanelet2 node (point)
type Node struct {
	ID         int64             `json:"id"`
	X          float64           `json:"x"`
	Y          float64           `json:"y"`
	Z          float64           `json:"z,omitempty"`
	Attributes map[string]string `json:"attributes"`
}

// Way represents a Lanelet2 way (linestring)
type Way struct {
	ID         int64             `json:"id"`
	Nodes      []int64           `json:"nodes"`
	Attributes map[string]string `json:"attributes"`
}

// Relation represents a Lanelet2 relation
type Relation struct {
	ID         int64             `json:"id"`
	Members    []RelationMember  `json:"members"`
	Attributes map[string]string `json:"attributes"`
}

// RelationMember represents a member of a relation
type RelationMember struct {
	Type string `json:"type"` // node, way, relation
	Ref  int64  `json:"ref"`
	Role string `json:"role"`
}

// Lanelet represents a driving lane
type Lanelet struct {
	ID           int64             `json:"id"`
	LeftBound    int64             `json:"left_bound"`
	RightBound   int64             `json:"right_bound"`
	Centerline   int64             `json:"centerline,omitempty"`
	RegElements  []int64           `json:"regulatory_elements"`
	Attributes   map[string]string `json:"attributes"`
	SpeedLimit   float64           `json:"speed_limit,omitempty"`
	LaneType     string            `json:"lane_type"`
	VehicleClass string            `json:"vehicle_class,omitempty"`
}

// RegElement represents a regulatory element
type RegElement struct {
	ID         int64             `json:"id"`
	Type       string            `json:"type"`
	Subtype    string            `json:"subtype"`
	Refers     []int64           `json:"refers"`
	RefLine    []int64           `json:"ref_line"`
	Attributes map[string]string `json:"attributes"`
}

// Area represents an area in the map
type Area struct {
	ID         int64             `json:"id"`
	OuterBound []int64           `json:"outer_bound"`
	InnerBound [][]int64         `json:"inner_bound"`
	Attributes map[string]string `json:"attributes"`
}

// Linestring represents a linestring
type Linestring struct {
	ID         int64             `json:"id"`
	Points     []int64           `json:"points"`
	Attributes map[string]string `json:"attributes"`
}

// Polygon represents a polygon
type Polygon struct {
	ID         int64             `json:"id"`
	OuterBound []int64           `json:"outer_bound"`
	InnerBound [][]int64         `json:"inner_bound"`
	Attributes map[string]string `json:"attributes"`
}

// RoutingGraph represents routing information
type RoutingGraph struct {
	Nodes []RoutingNode `json:"nodes"`
	Edges []RoutingEdge `json:"edges"`
}

// RoutingNode represents a node in the routing graph
type RoutingNode struct {
	ID       int64   `json:"id"`
	LaneletID int64  `json:"lanelet_id"`
	Position GeographicPoint `json:"position"`
}

// RoutingEdge represents an edge in the routing graph
type RoutingEdge struct {
	From   int64   `json:"from"`
	To     int64   `json:"to"`
	Cost   float64 `json:"cost"`
	Type   string  `json:"type"` // lane_change, successor, etc.
}

// GeographicPoint represents a geographic coordinate
type GeographicPoint struct {
	Lat float64 `json:"lat"`
	Lng float64 `json:"lng"`
	Alt float64 `json:"alt,omitempty"`
}

// BoundingBox represents a geographic bounding box
type BoundingBox struct {
	MinLat float64 `json:"min_lat"`
	MaxLat float64 `json:"max_lat"`
	MinLng float64 `json:"min_lng"`
	MaxLng float64 `json:"max_lng"`
}

// NewLanelet2Adapter creates a new Lanelet2 adapter
func NewLanelet2Adapter(config *config.Lanelet2Config, provenanceTracker *provenance.Tracker) (*Lanelet2Adapter, error) {
	adapter := &Lanelet2Adapter{
		config:            config,
		provenanceTracker: provenanceTracker,
		cache:             make(map[string]*models.Lanelet2Map),
	}
	
	// Initialize parser
	parser, err := NewLanelet2Parser(config)
	if err != nil {
		return nil, fmt.Errorf("failed to initialize Lanelet2 parser: %w", err)
	}
	adapter.parser = parser
	
	// Initialize validator
	validator, err := NewLanelet2Validator(config)
	if err != nil {
		return nil, fmt.Errorf("failed to initialize Lanelet2 validator: %w", err)
	}
	adapter.validator = validator
	
	// Initialize converter
	converter, err := NewLanelet2Converter(config)
	if err != nil {
		return nil, fmt.Errorf("failed to initialize Lanelet2 converter: %w", err)
	}
	adapter.converter = converter
	
	log.Printf("✅ Lanelet2 adapter initialized for region: %s", config.DefaultRegion)
	return adapter, nil
}

// LoadMap loads a Lanelet2 map from file or URL
func (a *Lanelet2Adapter) LoadMap(ctx context.Context, source string) (*Lanelet2Map, error) {
	// Check cache first
	if cached, exists := a.cache[source]; exists {
		return cached, nil
	}
	
	// Track provenance
	provenanceID, err := a.provenanceTracker.StartOperation(ctx, "lanelet2_load", map[string]interface{}{
		"source": source,
		"format": "lanelet2",
	})
	if err != nil {
		log.Printf("❌ Failed to track provenance: %v", err)
	}
	
	// Parse map
	mapData, err := a.parser.ParseFromSource(ctx, source)
	if err != nil {
		if provenanceID != "" {
			a.provenanceTracker.RecordError(ctx, provenanceID, err)
		}
		return nil, fmt.Errorf("failed to parse Lanelet2 map: %w", err)
	}
	
	// Validate map
	if err := a.validator.Validate(ctx, mapData); err != nil {
		if provenanceID != "" {
			a.provenanceTracker.RecordError(ctx, provenanceID, err)
		}
		return nil, fmt.Errorf("Lanelet2 map validation failed: %w", err)
	}
	
	// Build routing graph if enabled
	if a.config.BuildRoutingGraph {
		routingGraph, err := a.buildRoutingGraph(ctx, mapData)
		if err != nil {
			log.Printf("⚠️ Failed to build routing graph: %v", err)
		} else {
			mapData.RoutingGraph = routingGraph
		}
	}
	
	// Cache the map
	a.cache[source] = mapData
	
	// Complete provenance tracking
	if provenanceID != "" {
		a.provenanceTracker.CompleteOperation(ctx, provenanceID, map[string]interface{}{
			"map_id":      mapData.ID,
			"nodes":       len(mapData.Nodes),
			"ways":        len(mapData.Ways),
			"lanelets":    len(mapData.Lanelets),
			"reg_elements": len(mapData.RegElements),
		})
	}
	
	log.Printf("✅ Loaded Lanelet2 map: %s (%d lanelets)", mapData.ID, len(mapData.Lanelets))
	return mapData, nil
}

// GetLanelets returns all lanelets in the map
func (a *Lanelet2Adapter) GetLanelets(ctx context.Context, mapID string) (map[int64]*Lanelet, error) {
	mapData, exists := a.cache[mapID]
	if !exists {
		return nil, fmt.Errorf("map not found: %s", mapID)
	}
	
	return mapData.Lanelets, nil
}

// GetLanelet returns a specific lanelet
func (a *Lanelet2Adapter) GetLanelet(ctx context.Context, mapID string, laneletID int64) (*Lanelet, error) {
	lanelets, err := a.GetLanelets(ctx, mapID)
	if err != nil {
		return nil, err
	}
	
	lanelet, exists := lanelets[laneletID]
	if !exists {
		return nil, fmt.Errorf("lanelet not found: %d", laneletID)
	}
	
	return lanelet, nil
}

// GetRegulatoryElements returns all regulatory elements
func (a *Lanelet2Adapter) GetRegulatoryElements(ctx context.Context, mapID string) (map[int64]*RegElement, error) {
	mapData, exists := a.cache[mapID]
	if !exists {
		return nil, fmt.Errorf("map not found: %s", mapID)
	}
	
	return mapData.RegElements, nil
}

// GetRoutingGraph returns the routing graph
func (a *Lanelet2Adapter) GetRoutingGraph(ctx context.Context, mapID string) (*RoutingGraph, error) {
	mapData, exists := a.cache[mapID]
	if !exists {
		return nil, fmt.Errorf("map not found: %s", mapID)
	}
	
	if mapData.RoutingGraph == nil {
		// Build routing graph on demand
		routingGraph, err := a.buildRoutingGraph(ctx, mapData)
		if err != nil {
			return nil, fmt.Errorf("failed to build routing graph: %w", err)
		}
		mapData.RoutingGraph = routingGraph
	}
	
	return mapData.RoutingGraph, nil
}

// ValidateMap validates a Lanelet2 map
func (a *Lanelet2Adapter) ValidateMap(ctx context.Context, mapData *Lanelet2Map) error {
	return a.validator.Validate(ctx, mapData)
}

// ConvertToGeoJSON converts Lanelet2 map to GeoJSON
func (a *Lanelet2Adapter) ConvertToGeoJSON(ctx context.Context, mapData *Lanelet2Map) (map[string]interface{}, error) {
	return a.converter.ToGeoJSON(ctx, mapData)
}

// ConvertToOpenDRIVE converts Lanelet2 map to OpenDRIVE format
func (a *Lanelet2Adapter) ConvertToOpenDRIVE(ctx context.Context, mapData *Lanelet2Map) ([]byte, error) {
	return a.converter.ToOpenDRIVE(ctx, mapData)
}

// buildRoutingGraph builds a routing graph from the map data
func (a *Lanelet2Adapter) buildRoutingGraph(ctx context.Context, mapData *Lanelet2Map) (*RoutingGraph, error) {
	log.Printf("🔗 Building routing graph for map: %s", mapData.ID)
	
	nodes := make([]RoutingNode, 0, len(mapData.Lanelets))
	edges := make([]RoutingEdge, 0)
	
	// Create routing nodes for each lanelet
	nodeID := int64(1)
	laneletToNode := make(map[int64]int64)
	
	for laneletID, lanelet := range mapData.Lanelets {
		// Calculate lanelet center position (simplified)
		centerPos := a.calculateLaneletCenter(mapData, lanelet)
		
		node := RoutingNode{
			ID:        nodeID,
			LaneletID: laneletID,
			Position:  centerPos,
		}
		
		nodes = append(nodes, node)
		laneletToNode[laneletID] = nodeID
		nodeID++
	}
	
	// Create routing edges based on lanelet connectivity
	for laneletID, lanelet := range mapData.Lanelets {
		fromNodeID := laneletToNode[laneletID]
		
		// Find successor lanelets
		successors := a.findSuccessorLanelets(mapData, laneletID)
		for _, successorID := range successors {
			toNodeID := laneletToNode[successorID]
			
			edge := RoutingEdge{
				From: fromNodeID,
				To:   toNodeID,
				Cost: a.calculateEdgeCost(mapData, laneletID, successorID),
				Type: "successor",
			}
			
			edges = append(edges, edge)
		}
		
		// Find lane change possibilities
		leftLanelet := a.findAdjacentLanelet(mapData, laneletID, "left")
		if leftLanelet != 0 {
			toNodeID := laneletToNode[leftLanelet]
			
			edge := RoutingEdge{
				From: fromNodeID,
				To:   toNodeID,
				Cost: a.calculateLaneChangeCost(mapData, laneletID, leftLanelet),
				Type: "lane_change_left",
			}
			
			edges = append(edges, edge)
		}
		
		rightLanelet := a.findAdjacentLanelet(mapData, laneletID, "right")
		if rightLanelet != 0 {
			toNodeID := laneletToNode[rightLanelet]
			
			edge := RoutingEdge{
				From: fromNodeID,
				To:   toNodeID,
				Cost: a.calculateLaneChangeCost(mapData, laneletID, rightLanelet),
				Type: "lane_change_right",
			}
			
			edges = append(edges, edge)
		}
	}
	
	routingGraph := &RoutingGraph{
		Nodes: nodes,
		Edges: edges,
	}
	
	log.Printf("✅ Built routing graph: %d nodes, %d edges", len(nodes), len(edges))
	return routingGraph, nil
}

// calculateLaneletCenter calculates the center position of a lanelet
func (a *Lanelet2Adapter) calculateLaneletCenter(mapData *Lanelet2Map, lanelet *Lanelet) GeographicPoint {
	// Simplified implementation - would need proper geometric calculation
	return GeographicPoint{
		Lat: 24.4539, // Abu Dhabi center
		Lng: 54.3773,
	}
}

// findSuccessorLanelets finds lanelets that follow the given lanelet
func (a *Lanelet2Adapter) findSuccessorLanelets(mapData *Lanelet2Map, laneletID int64) []int64 {
	// Simplified implementation - would analyze topology
	return []int64{}
}

// findAdjacentLanelet finds the adjacent lanelet (left or right)
func (a *Lanelet2Adapter) findAdjacentLanelet(mapData *Lanelet2Map, laneletID int64, direction string) int64 {
	// Simplified implementation - would analyze topology
	return 0
}

// calculateEdgeCost calculates the cost of traversing between two lanelets
func (a *Lanelet2Adapter) calculateEdgeCost(mapData *Lanelet2Map, fromLanelet, toLanelet int64) float64 {
	// Simplified implementation - would consider distance, speed limit, etc.
	return 1.0
}

// calculateLaneChangeCost calculates the cost of a lane change
func (a *Lanelet2Adapter) calculateLaneChangeCost(mapData *Lanelet2Map, fromLanelet, toLanelet int64) float64 {
	// Lane changes have higher cost than straight driving
	return 5.0
}

// Status returns the adapter status
func (a *Lanelet2Adapter) Status() map[string]interface{} {
	return map[string]interface{}{
		"status":      "healthy",
		"maps_loaded": len(a.cache),
		"parser":      a.parser.Status(),
		"validator":   a.validator.Status(),
		"converter":   a.converter.Status(),
	}
}

// Close closes the adapter and releases resources
func (a *Lanelet2Adapter) Close() {
	log.Println("🧹 Closing Lanelet2 adapter")
	
	// Clear cache
	a.cache = make(map[string]*models.Lanelet2Map)
	
	// Close components
	if a.parser != nil {
		a.parser.Close()
	}
	if a.validator != nil {
		a.validator.Close()
	}
	if a.converter != nil {
		a.converter.Close()
	}
}
