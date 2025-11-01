-- Create tasks table for SAP EWM adapter
CREATE TABLE IF NOT EXISTS tasks (
    id VARCHAR(255) PRIMARY KEY,
    external_id VARCHAR(255) NOT NULL UNIQUE,
    type VARCHAR(100) NOT NULL,
    status VARCHAR(50) NOT NULL DEFAULT 'pending',
    priority INTEGER NOT NULL DEFAULT 0,
    warehouse_id VARCHAR(100) NOT NULL,
    zone_id VARCHAR(100),
    source JSONB NOT NULL,
    destination JSONB NOT NULL,
    item JSONB NOT NULL,
    quantity INTEGER NOT NULL DEFAULT 1,
    created_at TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW(),
    updated_at TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW(),
    assigned_to VARCHAR(255),
    completed_at TIMESTAMP WITH TIME ZONE,
    error_message TEXT
);

-- Create indexes for better performance
CREATE INDEX IF NOT EXISTS idx_tasks_status ON tasks(status);
CREATE INDEX IF NOT EXISTS idx_tasks_type ON tasks(type);
CREATE INDEX IF NOT EXISTS idx_tasks_warehouse_id ON tasks(warehouse_id);
CREATE INDEX IF NOT EXISTS idx_tasks_zone_id ON tasks(zone_id);
CREATE INDEX IF NOT EXISTS idx_tasks_assigned_to ON tasks(assigned_to);
CREATE INDEX IF NOT EXISTS idx_tasks_created_at ON tasks(created_at);
CREATE INDEX IF NOT EXISTS idx_tasks_external_id ON tasks(external_id);

-- Create composite indexes for common queries
CREATE INDEX IF NOT EXISTS idx_tasks_status_warehouse ON tasks(status, warehouse_id);
CREATE INDEX IF NOT EXISTS idx_tasks_status_type ON tasks(status, type);
CREATE INDEX IF NOT EXISTS idx_tasks_priority_created_at ON tasks(priority DESC, created_at ASC);

-- Add comments for documentation
COMMENT ON TABLE tasks IS 'Tasks synchronized from SAP EWM system';
COMMENT ON COLUMN tasks.id IS 'Internal task ID (UUID)';
COMMENT ON COLUMN tasks.external_id IS 'SAP EWM task ID';
COMMENT ON COLUMN tasks.type IS 'Task type (pick, put, move, etc.)';
COMMENT ON COLUMN tasks.status IS 'Task status (pending, processing, completed, failed)';
COMMENT ON COLUMN tasks.priority IS 'Task priority (higher number = higher priority)';
COMMENT ON COLUMN tasks.warehouse_id IS 'Warehouse identifier';
COMMENT ON COLUMN tasks.zone_id IS 'Zone identifier within warehouse';
COMMENT ON COLUMN tasks.source IS 'Source location (JSON)';
COMMENT ON COLUMN tasks.destination IS 'Destination location (JSON)';
COMMENT ON COLUMN tasks.item IS 'Item information (JSON)';
COMMENT ON COLUMN tasks.quantity IS 'Item quantity';
COMMENT ON COLUMN tasks.assigned_to IS 'Vehicle/agent assigned to task';
COMMENT ON COLUMN tasks.completed_at IS 'Task completion timestamp';
COMMENT ON COLUMN tasks.error_message IS 'Error message if task failed';
