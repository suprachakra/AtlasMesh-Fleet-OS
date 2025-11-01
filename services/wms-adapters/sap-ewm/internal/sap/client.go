package sap

import (
	"context"
	"time"

	"go.uber.org/zap"
	"github.com/atlasmesh/wms-adapters/sap-ewm/internal/config"
	"github.com/atlasmesh/wms-adapters/sap-ewm/internal/client"
)

// Client represents the SAP EWM client wrapper
type Client struct {
	client *client.SAPEWMClient
	config *config.SAPEWMConfig
	logger *zap.Logger
}

// NewClient creates a new SAP client
func NewClient(cfg *config.SAPEWMConfig, logger *zap.Logger) (*Client, error) {
	sapClient, err := client.NewSAPEWMClient(&client.SAPEWMConfig{
		Host:          cfg.Host,
		Port:          cfg.Port,
		Client:        cfg.Client,
		User:          cfg.User,
		Password:      cfg.Password,
		AuthType:      cfg.AuthType,
		SSLEnabled:    cfg.SSLEnabled,
		SSLVerify:     cfg.SSLVerify,
		APIVersion:    cfg.APIVersion,
		Timeout:       cfg.Timeout,
		RetryAttempts: cfg.RetryAttempts,
		RetryDelay:    cfg.RetryDelay,
	}, logger)

	if err != nil {
		return nil, err
	}

	return &Client{
		client: sapClient,
		config: cfg,
		logger: logger,
	}, nil
}

// Start starts the SAP client
func (c *Client) Start(ctx context.Context) error {
	c.logger.Info("SAP EWM client started")
	return nil
}

// Stop stops the SAP client
func (c *Client) Stop() error {
	c.logger.Info("SAP EWM client stopped")
	return nil
}

// GetTasks retrieves tasks from SAP EWM
func (c *Client) GetTasks(ctx context.Context, warehouseID string, status string) ([]client.Task, error) {
	return c.client.GetTasks(ctx, warehouseID, status)
}

// GetTask retrieves a specific task by ID
func (c *Client) GetTask(ctx context.Context, taskID string) (*client.Task, error) {
	return c.client.GetTask(ctx, taskID)
}

// UpdateTaskStatus updates the status of a task
func (c *Client) UpdateTaskStatus(ctx context.Context, taskID string, status string, message string) error {
	return c.client.UpdateTaskStatus(ctx, taskID, status, message)
}

// GetInventory retrieves inventory information
func (c *Client) GetInventory(ctx context.Context, warehouseID string, zoneID string) ([]client.InventoryItem, error) {
	return c.client.GetInventory(ctx, warehouseID, zoneID)
}

// GetResources retrieves resource information
func (c *Client) GetResources(ctx context.Context, warehouseID string) ([]client.Resource, error) {
	return c.client.GetResources(ctx, warehouseID)
}
