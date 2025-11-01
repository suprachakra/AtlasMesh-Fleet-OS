package oracle

import (
	"bytes"
	"context"
	"crypto/tls"
	"encoding/json"
	"fmt"
	"io"
	"net/http"
	"net/url"
	"time"

	"go.uber.org/zap"
)

// Client represents the Oracle WMS client
type Client struct {
	baseURL    string
	httpClient *http.Client
	auth       AuthProvider
	logger     *zap.Logger
	config     *OracleWMSConfig
}

// OracleWMSConfig represents Oracle WMS client configuration
type OracleWMSConfig struct {
	Host          string
	Port          int
	User          string
	Password      string
	AuthType      string
	SSLEnabled    bool
	SSLVerify     bool
	APIVersion    string
	Timeout       time.Duration
	RetryAttempts int
	RetryDelay    time.Duration
}

// AuthProvider interface for authentication
type AuthProvider interface {
	Authenticate(req *http.Request) error
}

// BasicAuthProvider implements basic authentication
type BasicAuthProvider struct {
	username string
	password string
}

// NewBasicAuthProvider creates a new basic auth provider
func NewBasicAuthProvider(username, password string) *BasicAuthProvider {
	return &BasicAuthProvider{
		username: username,
		password: password,
	}
}

// Authenticate adds basic authentication to the request
func (b *BasicAuthProvider) Authenticate(req *http.Request) error {
	req.SetBasicAuth(b.username, b.password)
	return nil
}

// NewClient creates a new Oracle WMS client
func NewClient(config *OracleWMSConfig, logger *zap.Logger) (*Client, error) {
	// Create base URL
	scheme := "http"
	if config.SSLEnabled {
		scheme = "https"
	}
	baseURL := fmt.Sprintf("%s://%s:%d/api/%s", 
		scheme, config.Host, config.Port, config.APIVersion)

	// Create HTTP client
	httpClient := &http.Client{
		Timeout: config.Timeout,
	}

	// Configure SSL if enabled
	if config.SSLEnabled {
		httpClient.Transport = &http.Transport{
			TLSClientConfig: &tls.Config{
				InsecureSkipVerify: !config.SSLVerify,
			},
		}
	}

	// Create auth provider
	var auth AuthProvider
	switch config.AuthType {
	case "basic":
		auth = NewBasicAuthProvider(config.User, config.Password)
	default:
		return nil, fmt.Errorf("unsupported auth type: %s", config.AuthType)
	}

	return &Client{
		baseURL:    baseURL,
		httpClient: httpClient,
		auth:       auth,
		logger:     logger,
		config:     config,
	}, nil
}

// Start starts the Oracle client
func (c *Client) Start(ctx context.Context) error {
	c.logger.Info("Oracle WMS client started")
	return nil
}

// Stop stops the Oracle client
func (c *Client) Stop() error {
	c.logger.Info("Oracle WMS client stopped")
	return nil
}

// GetTasks retrieves tasks from Oracle WMS
func (c *Client) GetTasks(ctx context.Context, warehouseID string, status string) ([]Task, error) {
	endpoint := fmt.Sprintf("%s/tasks", c.baseURL)
	
	// Build query parameters
	params := url.Values{}
	params.Add("warehouse_id", warehouseID)
	if status != "" {
		params.Add("status", status)
	}
	
	fullURL := fmt.Sprintf("%s?%s", endpoint, params.Encode())
	
	req, err := http.NewRequestWithContext(ctx, "GET", fullURL, nil)
	if err != nil {
		return nil, fmt.Errorf("failed to create request: %w", err)
	}

	// Add authentication
	if err := c.auth.Authenticate(req); err != nil {
		return nil, fmt.Errorf("failed to authenticate: %w", err)
	}

	// Add headers
	req.Header.Set("Content-Type", "application/json")
	req.Header.Set("Accept", "application/json")

	// Execute request with retry
	var tasks []Task
	err = c.executeWithRetry(req, &tasks)
	if err != nil {
		return nil, fmt.Errorf("failed to get tasks: %w", err)
	}

	return tasks, nil
}

// GetTask retrieves a specific task by ID
func (c *Client) GetTask(ctx context.Context, taskID string) (*Task, error) {
	endpoint := fmt.Sprintf("%s/tasks/%s", c.baseURL, taskID)
	
	req, err := http.NewRequestWithContext(ctx, "GET", endpoint, nil)
	if err != nil {
		return nil, fmt.Errorf("failed to create request: %w", err)
	}

	// Add authentication
	if err := c.auth.Authenticate(req); err != nil {
		return nil, fmt.Errorf("failed to authenticate: %w", err)
	}

	// Add headers
	req.Header.Set("Content-Type", "application/json")
	req.Header.Set("Accept", "application/json")

	// Execute request with retry
	var task Task
	err = c.executeWithRetry(req, &task)
	if err != nil {
		return nil, fmt.Errorf("failed to get task: %w", err)
	}

	return &task, nil
}

// UpdateTaskStatus updates the status of a task
func (c *Client) UpdateTaskStatus(ctx context.Context, taskID string, status string, message string) error {
	endpoint := fmt.Sprintf("%s/tasks/%s/status", c.baseURL, taskID)
	
	updateReq := TaskStatusUpdate{
		Status:  status,
		Message: message,
	}

	jsonData, err := json.Marshal(updateReq)
	if err != nil {
		return fmt.Errorf("failed to marshal request: %w", err)
	}

	req, err := http.NewRequestWithContext(ctx, "PUT", endpoint, bytes.NewBuffer(jsonData))
	if err != nil {
		return fmt.Errorf("failed to create request: %w", err)
	}

	// Add authentication
	if err := c.auth.Authenticate(req); err != nil {
		return fmt.Errorf("failed to authenticate: %w", err)
	}

	// Add headers
	req.Header.Set("Content-Type", "application/json")
	req.Header.Set("Accept", "application/json")

	// Execute request with retry
	var response map[string]interface{}
	err = c.executeWithRetry(req, &response)
	if err != nil {
		return fmt.Errorf("failed to update task status: %w", err)
	}

	return nil
}

// GetInventory retrieves inventory information
func (c *Client) GetInventory(ctx context.Context, warehouseID string, zoneID string) ([]InventoryItem, error) {
	endpoint := fmt.Sprintf("%s/inventory", c.baseURL)
	
	// Build query parameters
	params := url.Values{}
	params.Add("warehouse_id", warehouseID)
	if zoneID != "" {
		params.Add("zone_id", zoneID)
	}
	
	fullURL := fmt.Sprintf("%s?%s", endpoint, params.Encode())
	
	req, err := http.NewRequestWithContext(ctx, "GET", fullURL, nil)
	if err != nil {
		return nil, fmt.Errorf("failed to create request: %w", err)
	}

	// Add authentication
	if err := c.auth.Authenticate(req); err != nil {
		return nil, fmt.Errorf("failed to authenticate: %w", err)
	}

	// Add headers
	req.Header.Set("Content-Type", "application/json")
	req.Header.Set("Accept", "application/json")

	// Execute request with retry
	var inventory []InventoryItem
	err = c.executeWithRetry(req, &inventory)
	if err != nil {
		return nil, fmt.Errorf("failed to get inventory: %w", err)
	}

	return inventory, nil
}

// GetResources retrieves resource information
func (c *Client) GetResources(ctx context.Context, warehouseID string) ([]Resource, error) {
	endpoint := fmt.Sprintf("%s/resources", c.baseURL)
	
	// Build query parameters
	params := url.Values{}
	params.Add("warehouse_id", warehouseID)
	
	fullURL := fmt.Sprintf("%s?%s", endpoint, params.Encode())
	
	req, err := http.NewRequestWithContext(ctx, "GET", fullURL, nil)
	if err != nil {
		return nil, fmt.Errorf("failed to create request: %w", err)
	}

	// Add authentication
	if err := c.auth.Authenticate(req); err != nil {
		return nil, fmt.Errorf("failed to authenticate: %w", err)
	}

	// Add headers
	req.Header.Set("Content-Type", "application/json")
	req.Header.Set("Accept", "application/json")

	// Execute request with retry
	var resources []Resource
	err = c.executeWithRetry(req, &resources)
	if err != nil {
		return nil, fmt.Errorf("failed to get resources: %w", err)
	}

	return resources, nil
}

// executeWithRetry executes a request with retry logic
func (c *Client) executeWithRetry(req *http.Request, response interface{}) error {
	var lastErr error
	
	for attempt := 0; attempt <= c.config.RetryAttempts; attempt++ {
		if attempt > 0 {
			// Wait before retry
			time.Sleep(c.config.RetryDelay)
			c.logger.Info("Retrying request", 
				zap.String("url", req.URL.String()),
				zap.Int("attempt", attempt+1))
		}

		resp, err := c.httpClient.Do(req)
		if err != nil {
			lastErr = err
			c.logger.Warn("Request failed", 
				zap.String("url", req.URL.String()),
				zap.Int("attempt", attempt+1),
				zap.Error(err))
			continue
		}
		defer resp.Body.Close()

		// Check status code
		if resp.StatusCode >= 400 {
			body, _ := io.ReadAll(resp.Body)
			lastErr = fmt.Errorf("HTTP %d: %s", resp.StatusCode, string(body))
			c.logger.Warn("Request returned error status", 
				zap.String("url", req.URL.String()),
				zap.Int("status_code", resp.StatusCode),
				zap.String("response", string(body)))
			continue
		}

		// Parse response
		if response != nil {
			if err := json.NewDecoder(resp.Body).Decode(response); err != nil {
				lastErr = err
				c.logger.Warn("Failed to decode response", 
					zap.String("url", req.URL.String()),
					zap.Int("attempt", attempt+1),
					zap.Error(err))
				continue
			}
		}

		// Success
		c.logger.Info("Request successful", 
			zap.String("url", req.URL.String()),
			zap.Int("attempt", attempt+1))
		return nil
	}

	return fmt.Errorf("request failed after %d attempts: %w", c.config.RetryAttempts+1, lastErr)
}

// Data structures

// Task represents a task from Oracle WMS
type Task struct {
	ID           string    `json:"id"`
	Type         string    `json:"type"`
	Status       string    `json:"status"`
	Priority     int       `json:"priority"`
	WarehouseID  string    `json:"warehouse_id"`
	ZoneID       string    `json:"zone_id"`
	Source       Location  `json:"source"`
	Destination  Location  `json:"destination"`
	Item         Item      `json:"item"`
	Quantity     int       `json:"quantity"`
	CreatedAt    time.Time `json:"created_at"`
	UpdatedAt    time.Time `json:"updated_at"`
	AssignedTo   string    `json:"assigned_to,omitempty"`
	CompletedAt  *time.Time `json:"completed_at,omitempty"`
	ErrorMessage string    `json:"error_message,omitempty"`
}

// Location represents a location in the warehouse
type Location struct {
	ZoneID     string  `json:"zone_id"`
	Position   string  `json:"position"`
	X          float64 `json:"x"`
	Y          float64 `json:"y"`
	Z          float64 `json:"z"`
	Accessible bool    `json:"accessible"`
}

// Item represents an item in the warehouse
type Item struct {
	ID          string     `json:"id"`
	SKU         string     `json:"sku"`
	Description string     `json:"description"`
	Weight      float64    `json:"weight"`
	Dimensions  Dimensions `json:"dimensions"`
	Category    string     `json:"category"`
}

// Dimensions represents item dimensions
type Dimensions struct {
	Length float64 `json:"length"`
	Width  float64 `json:"width"`
	Height float64 `json:"height"`
}

// TaskStatusUpdate represents a task status update request
type TaskStatusUpdate struct {
	Status  string `json:"status"`
	Message string `json:"message"`
}

// InventoryItem represents an inventory item
type InventoryItem struct {
	ID        string    `json:"id"`
	SKU       string    `json:"sku"`
	Quantity  int       `json:"quantity"`
	Location  Location  `json:"location"`
	UpdatedAt time.Time `json:"updated_at"`
}

// Resource represents a warehouse resource
type Resource struct {
	ID          string    `json:"id"`
	Type        string    `json:"type"`
	Status      string    `json:"status"`
	Location    Location  `json:"location"`
	Capacity    int       `json:"capacity"`
	CurrentLoad int       `json:"current_load"`
	UpdatedAt   time.Time `json:"updated_at"`
}
