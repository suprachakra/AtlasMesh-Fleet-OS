package repository

import (
	"context"
	"database/sql"
	"encoding/json"
	"fmt"
	"time"

	"go.uber.org/zap"
)

// TaskRepository defines the interface for task data operations
type TaskRepository interface {
	Create(ctx context.Context, task *Task) error
	GetByID(ctx context.Context, id string) (*Task, error)
	GetByFilters(ctx context.Context, filters TaskFilters) ([]*Task, error)
	GetPending(ctx context.Context, limit int) ([]*Task, error)
	UpdateStatus(ctx context.Context, id string, status string, message string) error
	Update(ctx context.Context, task *Task) error
	Delete(ctx context.Context, id string) error
}

// Task represents a task in the database
type Task struct {
	ID           string     `json:"id" db:"id"`
	ExternalID   string     `json:"external_id" db:"external_id"`
	Type         string     `json:"type" db:"type"`
	Status       string     `json:"status" db:"status"`
	Priority     int        `json:"priority" db:"priority"`
	WarehouseID  string     `json:"warehouse_id" db:"warehouse_id"`
	ZoneID       string     `json:"zone_id" db:"zone_id"`
	Source       Location   `json:"source" db:"source"`
	Destination  Location   `json:"destination" db:"destination"`
	Item         Item       `json:"item" db:"item"`
	Quantity     int        `json:"quantity" db:"quantity"`
	CreatedAt    time.Time  `json:"created_at" db:"created_at"`
	UpdatedAt    time.Time  `json:"updated_at" db:"updated_at"`
	AssignedTo   string     `json:"assigned_to" db:"assigned_to"`
	CompletedAt  *time.Time `json:"completed_at" db:"completed_at"`
	ErrorMessage string     `json:"error_message" db:"error_message"`
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

// TaskFilters represents filters for task queries
type TaskFilters struct {
	Status      string    `json:"status,omitempty"`
	Type        string    `json:"type,omitempty"`
	WarehouseID string    `json:"warehouse_id,omitempty"`
	ZoneID      string    `json:"zone_id,omitempty"`
	AssignedTo  string    `json:"assigned_to,omitempty"`
	CreatedFrom *time.Time `json:"created_from,omitempty"`
	CreatedTo   *time.Time `json:"created_to,omitempty"`
	Limit       int       `json:"limit,omitempty"`
	Offset      int       `json:"offset,omitempty"`
}

// taskRepository implements TaskRepository interface
type taskRepository struct {
	db     *sql.DB
	logger *zap.Logger
}

// NewTaskRepository creates a new task repository
func NewTaskRepository(db *sql.DB, logger *zap.Logger) TaskRepository {
	return &taskRepository{
		db:     db,
		logger: logger,
	}
}

// Create creates a new task
func (r *taskRepository) Create(ctx context.Context, task *Task) error {
	query := `
		INSERT INTO tasks (
			id, external_id, type, status, priority, warehouse_id, zone_id,
			source, destination, item, quantity, created_at, updated_at,
			assigned_to, completed_at, error_message
		) VALUES (
			$1, $2, $3, $4, $5, $6, $7, $8, $9, $10, $11, $12, $13, $14, $15, $16
		)
	`

	// Convert structs to JSON for storage
	sourceJSON, err := json.Marshal(task.Source)
	if err != nil {
		return fmt.Errorf("failed to marshal source: %w", err)
	}

	destinationJSON, err := json.Marshal(task.Destination)
	if err != nil {
		return fmt.Errorf("failed to marshal destination: %w", err)
	}

	itemJSON, err := json.Marshal(task.Item)
	if err != nil {
		return fmt.Errorf("failed to marshal item: %w", err)
	}

	_, err = r.db.ExecContext(ctx, query,
		task.ID, task.ExternalID, task.Type, task.Status, task.Priority,
		task.WarehouseID, task.ZoneID, sourceJSON, destinationJSON,
		itemJSON, task.Quantity, task.CreatedAt, task.UpdatedAt,
		task.AssignedTo, task.CompletedAt, task.ErrorMessage,
	)

	if err != nil {
		return fmt.Errorf("failed to create task: %w", err)
	}

	r.logger.Info("Task created", zap.String("task_id", task.ID))
	return nil
}

// GetByID retrieves a task by ID
func (r *taskRepository) GetByID(ctx context.Context, id string) (*Task, error) {
	query := `
		SELECT id, external_id, type, status, priority, warehouse_id, zone_id,
			   source, destination, item, quantity, created_at, updated_at,
			   assigned_to, completed_at, error_message
		FROM tasks
		WHERE id = $1
	`

	row := r.db.QueryRowContext(ctx, query, id)
	task, err := r.scanTask(row)
	if err != nil {
		if err == sql.ErrNoRows {
			return nil, fmt.Errorf("task not found: %s", id)
		}
		return nil, fmt.Errorf("failed to get task: %w", err)
	}

	return task, nil
}

// GetByFilters retrieves tasks with filters
func (r *taskRepository) GetByFilters(ctx context.Context, filters TaskFilters) ([]*Task, error) {
	query := `
		SELECT id, external_id, type, status, priority, warehouse_id, zone_id,
			   source, destination, item, quantity, created_at, updated_at,
			   assigned_to, completed_at, error_message
		FROM tasks
		WHERE 1=1
	`
	args := []interface{}{}
	argIndex := 1

	// Add filters
	if filters.Status != "" {
		query += fmt.Sprintf(" AND status = $%d", argIndex)
		args = append(args, filters.Status)
		argIndex++
	}

	if filters.Type != "" {
		query += fmt.Sprintf(" AND type = $%d", argIndex)
		args = append(args, filters.Type)
		argIndex++
	}

	if filters.WarehouseID != "" {
		query += fmt.Sprintf(" AND warehouse_id = $%d", argIndex)
		args = append(args, filters.WarehouseID)
		argIndex++
	}

	if filters.ZoneID != "" {
		query += fmt.Sprintf(" AND zone_id = $%d", argIndex)
		args = append(args, filters.ZoneID)
		argIndex++
	}

	if filters.AssignedTo != "" {
		query += fmt.Sprintf(" AND assigned_to = $%d", argIndex)
		args = append(args, filters.AssignedTo)
		argIndex++
	}

	if filters.CreatedFrom != nil {
		query += fmt.Sprintf(" AND created_at >= $%d", argIndex)
		args = append(args, *filters.CreatedFrom)
		argIndex++
	}

	if filters.CreatedTo != nil {
		query += fmt.Sprintf(" AND created_at <= $%d", argIndex)
		args = append(args, *filters.CreatedTo)
		argIndex++
	}

	// Add ordering
	query += " ORDER BY priority DESC, created_at ASC"

	// Add limit and offset
	if filters.Limit > 0 {
		query += fmt.Sprintf(" LIMIT $%d", argIndex)
		args = append(args, filters.Limit)
		argIndex++
	}

	if filters.Offset > 0 {
		query += fmt.Sprintf(" OFFSET $%d", argIndex)
		args = append(args, filters.Offset)
		argIndex++
	}

	rows, err := r.db.QueryContext(ctx, query, args...)
	if err != nil {
		return nil, fmt.Errorf("failed to query tasks: %w", err)
	}
	defer rows.Close()

	var tasks []*Task
	for rows.Next() {
		task, err := r.scanTask(rows)
		if err != nil {
			return nil, fmt.Errorf("failed to scan task: %w", err)
		}
		tasks = append(tasks, task)
	}

	if err := rows.Err(); err != nil {
		return nil, fmt.Errorf("failed to iterate rows: %w", err)
	}

	return tasks, nil
}

// GetPending retrieves pending tasks
func (r *taskRepository) GetPending(ctx context.Context, limit int) ([]*Task, error) {
	filters := TaskFilters{
		Status: "pending",
		Limit:  limit,
	}
	return r.GetByFilters(ctx, filters)
}

// UpdateStatus updates a task status
func (r *taskRepository) UpdateStatus(ctx context.Context, id string, status string, message string) error {
	query := `
		UPDATE tasks 
		SET status = $1, error_message = $2, updated_at = $3
		WHERE id = $4
	`

	now := time.Now()
	result, err := r.db.ExecContext(ctx, query, status, message, now, id)
	if err != nil {
		return fmt.Errorf("failed to update task status: %w", err)
	}

	rowsAffected, err := result.RowsAffected()
	if err != nil {
		return fmt.Errorf("failed to get rows affected: %w", err)
	}

	if rowsAffected == 0 {
		return fmt.Errorf("task not found: %s", id)
	}

	r.logger.Info("Task status updated", 
		zap.String("task_id", id),
		zap.String("status", status))

	return nil
}

// Update updates a task
func (r *taskRepository) Update(ctx context.Context, task *Task) error {
	query := `
		UPDATE tasks 
		SET external_id = $1, type = $2, status = $3, priority = $4, 
			warehouse_id = $5, zone_id = $6, source = $7, destination = $8,
			item = $9, quantity = $10, updated_at = $11, assigned_to = $12,
			completed_at = $13, error_message = $14
		WHERE id = $15
	`

	// Convert structs to JSON for storage
	sourceJSON, err := json.Marshal(task.Source)
	if err != nil {
		return fmt.Errorf("failed to marshal source: %w", err)
	}

	destinationJSON, err := json.Marshal(task.Destination)
	if err != nil {
		return fmt.Errorf("failed to marshal destination: %w", err)
	}

	itemJSON, err := json.Marshal(task.Item)
	if err != nil {
		return fmt.Errorf("failed to marshal item: %w", err)
	}

	result, err := r.db.ExecContext(ctx, query,
		task.ExternalID, task.Type, task.Status, task.Priority,
		task.WarehouseID, task.ZoneID, sourceJSON, destinationJSON,
		itemJSON, task.Quantity, task.UpdatedAt, task.AssignedTo,
		task.CompletedAt, task.ErrorMessage, task.ID,
	)

	if err != nil {
		return fmt.Errorf("failed to update task: %w", err)
	}

	rowsAffected, err := result.RowsAffected()
	if err != nil {
		return fmt.Errorf("failed to get rows affected: %w", err)
	}

	if rowsAffected == 0 {
		return fmt.Errorf("task not found: %s", task.ID)
	}

	r.logger.Info("Task updated", zap.String("task_id", task.ID))
	return nil
}

// Delete deletes a task
func (r *taskRepository) Delete(ctx context.Context, id string) error {
	query := `DELETE FROM tasks WHERE id = $1`

	result, err := r.db.ExecContext(ctx, query, id)
	if err != nil {
		return fmt.Errorf("failed to delete task: %w", err)
	}

	rowsAffected, err := result.RowsAffected()
	if err != nil {
		return fmt.Errorf("failed to get rows affected: %w", err)
	}

	if rowsAffected == 0 {
		return fmt.Errorf("task not found: %s", id)
	}

	r.logger.Info("Task deleted", zap.String("task_id", id))
	return nil
}

// scanTask scans a database row into a Task struct
func (r *taskRepository) scanTask(scanner interface {
	Scan(dest ...interface{}) error
}) (*Task, error) {
	var task Task
	var sourceJSON, destinationJSON, itemJSON []byte

	err := scanner.Scan(
		&task.ID, &task.ExternalID, &task.Type, &task.Status, &task.Priority,
		&task.WarehouseID, &task.ZoneID, &sourceJSON, &destinationJSON,
		&itemJSON, &task.Quantity, &task.CreatedAt, &task.UpdatedAt,
		&task.AssignedTo, &task.CompletedAt, &task.ErrorMessage,
	)

	if err != nil {
		return nil, err
	}

	// Unmarshal JSON fields
	if err := json.Unmarshal(sourceJSON, &task.Source); err != nil {
		return nil, fmt.Errorf("failed to unmarshal source: %w", err)
	}

	if err := json.Unmarshal(destinationJSON, &task.Destination); err != nil {
		return nil, fmt.Errorf("failed to unmarshal destination: %w", err)
	}

	if err := json.Unmarshal(itemJSON, &task.Item); err != nil {
		return nil, fmt.Errorf("failed to unmarshal item: %w", err)
	}

	return &task, nil
}
