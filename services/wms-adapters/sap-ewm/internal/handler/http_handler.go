package handler

import (
	"encoding/json"
	"net/http"
	"strconv"
	"time"

	"github.com/gin-gonic/gin"
	"go.uber.org/zap"
	"github.com/atlasmesh/wms-adapters/sap-ewm/internal/repository"
	"github.com/atlasmesh/wms-adapters/sap-ewm/internal/service"
)

// HTTPHandler handles HTTP requests
type HTTPHandler struct {
	taskService *service.TaskService
	logger      *zap.Logger
}

// NewHTTPHandler creates a new HTTP handler
func NewHTTPHandler(taskService *service.TaskService, logger *zap.Logger) *HTTPHandler {
	return &HTTPHandler{
		taskService: taskService,
		logger:      logger,
	}
}

// SetupRoutes sets up HTTP routes
func (h *HTTPHandler) SetupRoutes(r *gin.Engine) {
	// Health check
	r.GET("/health", h.HealthCheck)

	// Task routes
	tasks := r.Group("/api/v1/tasks")
	{
		tasks.GET("", h.GetTasks)
		tasks.GET("/:id", h.GetTask)
		tasks.PUT("/:id/status", h.UpdateTaskStatus)
	}

	// Inventory routes
	inventory := r.Group("/api/v1/inventory")
	{
		inventory.GET("", h.GetInventory)
	}

	// Resource routes
	resources := r.Group("/api/v1/resources")
	{
		resources.GET("", h.GetResources)
	}
}

// HealthCheck handles health check requests
func (h *HTTPHandler) HealthCheck(c *gin.Context) {
	c.JSON(http.StatusOK, gin.H{
		"status":    "healthy",
		"timestamp": time.Now().UTC(),
		"service":   "sap-ewm-adapter",
	})
}

// GetTasks handles GET /api/v1/tasks
func (h *HTTPHandler) GetTasks(c *gin.Context) {
	// Parse query parameters
	filters := repository.TaskFilters{}

	if status := c.Query("status"); status != "" {
		filters.Status = status
	}

	if taskType := c.Query("type"); taskType != "" {
		filters.Type = taskType
	}

	if warehouseID := c.Query("warehouse_id"); warehouseID != "" {
		filters.WarehouseID = warehouseID
	}

	if zoneID := c.Query("zone_id"); zoneID != "" {
		filters.ZoneID = zoneID
	}

	if assignedTo := c.Query("assigned_to"); assignedTo != "" {
		filters.AssignedTo = assignedTo
	}

	if limitStr := c.Query("limit"); limitStr != "" {
		if limit, err := strconv.Atoi(limitStr); err == nil {
			filters.Limit = limit
		}
	}

	if offsetStr := c.Query("offset"); offsetStr != "" {
		if offset, err := strconv.Atoi(offsetStr); err == nil {
			filters.Offset = offset
		}
	}

	// Get tasks
	tasks, err := h.taskService.GetTasks(c.Request.Context(), filters)
	if err != nil {
		h.logger.Error("Failed to get tasks", zap.Error(err))
		c.JSON(http.StatusInternalServerError, gin.H{
			"error": "Failed to get tasks",
		})
		return
	}

	c.JSON(http.StatusOK, gin.H{
		"tasks": tasks,
		"count": len(tasks),
	})
}

// GetTask handles GET /api/v1/tasks/:id
func (h *HTTPHandler) GetTask(c *gin.Context) {
	taskID := c.Param("id")
	if taskID == "" {
		c.JSON(http.StatusBadRequest, gin.H{
			"error": "Task ID is required",
		})
		return
	}

	task, err := h.taskService.GetTask(c.Request.Context(), taskID)
	if err != nil {
		h.logger.Error("Failed to get task", 
			zap.String("task_id", taskID),
			zap.Error(err))
		c.JSON(http.StatusNotFound, gin.H{
			"error": "Task not found",
		})
		return
	}

	c.JSON(http.StatusOK, task)
}

// UpdateTaskStatus handles PUT /api/v1/tasks/:id/status
func (h *HTTPHandler) UpdateTaskStatus(c *gin.Context) {
	taskID := c.Param("id")
	if taskID == "" {
		c.JSON(http.StatusBadRequest, gin.H{
			"error": "Task ID is required",
		})
		return
	}

	var req struct {
		Status  string `json:"status" binding:"required"`
		Message string `json:"message"`
	}

	if err := c.ShouldBindJSON(&req); err != nil {
		c.JSON(http.StatusBadRequest, gin.H{
			"error": "Invalid request body",
		})
		return
	}

	err := h.taskService.UpdateTaskStatus(c.Request.Context(), taskID, req.Status, req.Message)
	if err != nil {
		h.logger.Error("Failed to update task status", 
			zap.String("task_id", taskID),
			zap.String("status", req.Status),
			zap.Error(err))
		c.JSON(http.StatusInternalServerError, gin.H{
			"error": "Failed to update task status",
		})
		return
	}

	c.JSON(http.StatusOK, gin.H{
		"message": "Task status updated successfully",
	})
}

// GetInventory handles GET /api/v1/inventory
func (h *HTTPHandler) GetInventory(c *gin.Context) {
	// This would integrate with SAP EWM inventory API
	// For now, return a placeholder response
	c.JSON(http.StatusOK, gin.H{
		"message": "Inventory endpoint - integration pending",
		"items":   []interface{}{},
	})
}

// GetResources handles GET /api/v1/resources
func (h *HTTPHandler) GetResources(c *gin.Context) {
	// This would integrate with SAP EWM resource API
	// For now, return a placeholder response
	c.JSON(http.StatusOK, gin.H{
		"message":   "Resources endpoint - integration pending",
		"resources": []interface{}{},
	})
}

// ErrorResponse represents an error response
type ErrorResponse struct {
	Error     string    `json:"error"`
	Message   string    `json:"message,omitempty"`
	Timestamp time.Time `json:"timestamp"`
}

// SuccessResponse represents a success response
type SuccessResponse struct {
	Message   string      `json:"message"`
	Data      interface{} `json:"data,omitempty"`
	Timestamp time.Time   `json:"timestamp"`
}
