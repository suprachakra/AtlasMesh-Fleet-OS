package service

import (
	"context"
	"fmt"
	"sync"
	"time"

	"go.uber.org/zap"
	"github.com/atlasmesh/wms-adapters/sap-ewm/internal/client"
	"github.com/atlasmesh/wms-adapters/sap-ewm/internal/config"
	"github.com/atlasmesh/wms-adapters/sap-ewm/internal/repository"
)

// TaskService handles task management operations
type TaskService struct {
	client     *client.SAPEWMClient
	repo       repository.TaskRepository
	config     *config.TaskConfig
	logger     *zap.Logger
	ctx        context.Context
	cancel     context.CancelFunc
	wg         sync.WaitGroup
	stopChan   chan struct{}
}

// NewTaskService creates a new task service
func NewTaskService(
	client *client.SAPEWMClient,
	repo repository.TaskRepository,
	config *config.TaskConfig,
	logger *zap.Logger,
) *TaskService {
	ctx, cancel := context.WithCancel(context.Background())
	
	return &TaskService{
		client:   client,
		repo:     repo,
		config:   config,
		logger:   logger,
		ctx:      ctx,
		cancel:   cancel,
		stopChan: make(chan struct{}),
	}
}

// Start starts the task service
func (s *TaskService) Start() error {
	if !s.config.Enabled {
		s.logger.Info("Task service is disabled")
		return nil
	}

	s.logger.Info("Starting task service")
	
	// Start task polling
	s.wg.Add(1)
	go s.pollTasks()
	
	// Start task processing
	s.wg.Add(1)
	go s.processTasks()

	return nil
}

// Stop stops the task service
func (s *TaskService) Stop() error {
	s.logger.Info("Stopping task service")
	
	close(s.stopChan)
	s.cancel()
	s.wg.Wait()
	
	return nil
}

// pollTasks polls for new tasks from SAP EWM
func (s *TaskService) pollTasks() {
	defer s.wg.Done()
	
	ticker := time.NewTicker(s.config.PollInterval)
	defer ticker.Stop()

	for {
		select {
		case <-s.ctx.Done():
			return
		case <-s.stopChan:
			return
		case <-ticker.C:
			if err := s.fetchTasks(); err != nil {
				s.logger.Error("Failed to fetch tasks", zap.Error(err))
			}
		}
	}
}

// fetchTasks fetches tasks from SAP EWM
func (s *TaskService) fetchTasks() error {
	// Get pending tasks from SAP EWM
	tasks, err := s.client.GetTasks(s.ctx, "", "pending")
	if err != nil {
		return fmt.Errorf("failed to get tasks from SAP EWM: %w", err)
	}

	if len(tasks) == 0 {
		return nil
	}

	s.logger.Info("Fetched tasks from SAP EWM", zap.Int("count", len(tasks)))

	// Process tasks in batches
	for i := 0; i < len(tasks); i += s.config.BatchSize {
		end := i + s.config.BatchSize
		if end > len(tasks) {
			end = len(tasks)
		}

		batch := tasks[i:end]
		if err := s.processBatch(batch); err != nil {
			s.logger.Error("Failed to process task batch", 
				zap.Int("batch_start", i),
				zap.Int("batch_size", len(batch)),
				zap.Error(err))
		}
	}

	return nil
}

// processBatch processes a batch of tasks
func (s *TaskService) processBatch(tasks []client.Task) error {
	for _, task := range tasks {
		// Convert SAP EWM task to internal task
		internalTask := s.convertToInternalTask(task)
		
		// Store in database
		if err := s.repo.Create(s.ctx, internalTask); err != nil {
			s.logger.Error("Failed to store task", 
				zap.String("task_id", task.ID),
				zap.Error(err))
			continue
		}

		s.logger.Info("Stored task", 
			zap.String("task_id", task.ID),
			zap.String("type", task.Type))
	}

	return nil
}

// processTasks processes tasks from the database
func (s *TaskService) processTasks() {
	defer s.wg.Done()
	
	ticker := time.NewTicker(s.config.PollInterval)
	defer ticker.Stop()

	for {
		select {
		case <-s.ctx.Done():
			return
		case <-s.stopChan:
			return
		case <-ticker.C:
			if err := s.processPendingTasks(); err != nil {
				s.logger.Error("Failed to process pending tasks", zap.Error(err))
			}
		}
	}
}

// processPendingTasks processes pending tasks from the database
func (s *TaskService) processPendingTasks() error {
	// Get pending tasks from database
	tasks, err := s.repo.GetPending(s.ctx, s.config.BatchSize)
	if err != nil {
		return fmt.Errorf("failed to get pending tasks: %w", err)
	}

	if len(tasks) == 0 {
		return nil
	}

	s.logger.Info("Processing pending tasks", zap.Int("count", len(tasks)))

	// Process each task
	for _, task := range tasks {
		if err := s.processTask(task); err != nil {
			s.logger.Error("Failed to process task", 
				zap.String("task_id", task.ID),
				zap.Error(err))
		}
	}

	return nil
}

// processTask processes a single task
func (s *TaskService) processTask(task *repository.Task) error {
	// Update task status to processing
	if err := s.repo.UpdateStatus(s.ctx, task.ID, "processing", ""); err != nil {
		return fmt.Errorf("failed to update task status: %w", err)
	}

	// Simulate task processing (in real implementation, this would integrate with fleet manager)
	time.Sleep(100 * time.Millisecond)

	// Update task status to completed
	if err := s.repo.UpdateStatus(s.ctx, task.ID, "completed", "Task completed successfully"); err != nil {
		return fmt.Errorf("failed to update task status: %w", err)
	}

	// Update SAP EWM
	if err := s.client.UpdateTaskStatus(s.ctx, task.ExternalID, "completed", "Task completed successfully"); err != nil {
		s.logger.Error("Failed to update SAP EWM task status", 
			zap.String("task_id", task.ID),
			zap.String("external_id", task.ExternalID),
			zap.Error(err))
	}

	s.logger.Info("Task processed successfully", 
		zap.String("task_id", task.ID),
		zap.String("external_id", task.ExternalID))

	return nil
}

// convertToInternalTask converts SAP EWM task to internal task
func (s *TaskService) convertToInternalTask(task client.Task) *repository.Task {
	return &repository.Task{
		ExternalID:  task.ID,
		Type:        task.Type,
		Status:      task.Status,
		Priority:    task.Priority,
		WarehouseID: task.WarehouseID,
		ZoneID:      task.ZoneID,
		Source: repository.Location{
			ZoneID:     task.Source.ZoneID,
			Position:   task.Source.Position,
			X:          task.Source.X,
			Y:          task.Source.Y,
			Z:          task.Source.Z,
			Accessible: task.Source.Accessible,
		},
		Destination: repository.Location{
			ZoneID:     task.Destination.ZoneID,
			Position:   task.Destination.Position,
			X:          task.Destination.X,
			Y:          task.Destination.Y,
			Z:          task.Destination.Z,
			Accessible: task.Destination.Accessible,
		},
		Item: repository.Item{
			ID:          task.Item.ID,
			SKU:         task.Item.SKU,
			Description: task.Item.Description,
			Weight:      task.Item.Weight,
			Dimensions: repository.Dimensions{
				Length: task.Item.Dimensions.Length,
				Width:  task.Item.Dimensions.Width,
				Height: task.Item.Dimensions.Height,
			},
			Category: task.Item.Category,
		},
		Quantity:    task.Quantity,
		CreatedAt:   task.CreatedAt,
		UpdatedAt:   task.UpdatedAt,
		AssignedTo:  task.AssignedTo,
		CompletedAt: task.CompletedAt,
		ErrorMessage: task.ErrorMessage,
	}
}

// GetTask retrieves a task by ID
func (s *TaskService) GetTask(ctx context.Context, taskID string) (*repository.Task, error) {
	return s.repo.GetByID(ctx, taskID)
}

// GetTasks retrieves tasks with filters
func (s *TaskService) GetTasks(ctx context.Context, filters repository.TaskFilters) ([]*repository.Task, error) {
	return s.repo.GetByFilters(ctx, filters)
}

// UpdateTaskStatus updates a task status
func (s *TaskService) UpdateTaskStatus(ctx context.Context, taskID string, status string, message string) error {
	// Update in database
	if err := s.repo.UpdateStatus(ctx, taskID, status, message); err != nil {
		return fmt.Errorf("failed to update task status in database: %w", err)
	}

	// Get task to find external ID
	task, err := s.repo.GetByID(ctx, taskID)
	if err != nil {
		return fmt.Errorf("failed to get task: %w", err)
	}

	// Update in SAP EWM
	if err := s.client.UpdateTaskStatus(ctx, task.ExternalID, status, message); err != nil {
		s.logger.Error("Failed to update SAP EWM task status", 
			zap.String("task_id", taskID),
			zap.String("external_id", task.ExternalID),
			zap.Error(err))
		// Don't return error as database update succeeded
	}

	return nil
}
