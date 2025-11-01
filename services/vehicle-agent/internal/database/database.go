package database

import (
	"database/sql"
	"fmt"

	_ "github.com/lib/pq"
)

// Database represents the database connection
type Database struct {
	conn *sql.DB
}

// New creates a new database connection
func New(config interface{}) (*Database, error) {
	// In a real implementation, this would use the config parameter
	// For now, return a mock database
	return &Database{}, nil
}

// Close closes the database connection
func (db *Database) Close() error {
	if db.conn != nil {
		return db.conn.Close()
	}
	return nil
}

// Query executes a query and returns rows
func (db *Database) Query(query string, args ...interface{}) (*sql.Rows, error) {
	if db.conn == nil {
		return nil, fmt.Errorf("database not connected")
	}
	return db.conn.Query(query, args...)
}

// QueryRow executes a query and returns a single row
func (db *Database) QueryRow(query string, args ...interface{}) *sql.Row {
	if db.conn == nil {
		return nil
	}
	return db.conn.QueryRow(query, args...)
}

// Exec executes a query without returning rows
func (db *Database) Exec(query string, args ...interface{}) (sql.Result, error) {
	if db.conn == nil {
		return nil, fmt.Errorf("database not connected")
	}
	return db.conn.Exec(query, args...)
}
