package storage

import (
	"fmt"
	"io"
	"os"
	"path/filepath"
)

// Storage represents the storage interface
type Storage struct {
	config *Config
}

// Config represents storage configuration
type Config struct {
	Type     string `yaml:"type"`
	Endpoint string `yaml:"endpoint"`
	Bucket   string `yaml:"bucket"`
	AccessKey string `yaml:"access_key"`
	SecretKey string `yaml:"secret_key"`
	Region   string `yaml:"region"`
	Path     string `yaml:"path"`
}

// New creates a new storage instance
func New(config *Config) (*Storage, error) {
	if config == nil {
		return nil, fmt.Errorf("config cannot be nil")
	}

	storage := &Storage{
		config: config,
	}

	// Initialize storage based on type
	switch config.Type {
	case "local":
		if err := storage.initLocalStorage(); err != nil {
			return nil, fmt.Errorf("failed to initialize local storage: %w", err)
		}
	case "S3", "MinIO":
		if err := storage.initS3Storage(); err != nil {
			return nil, fmt.Errorf("failed to initialize S3 storage: %w", err)
		}
	default:
		return nil, fmt.Errorf("unsupported storage type: %s", config.Type)
	}

	return storage, nil
}

// Store stores data to storage
func (s *Storage) Store(key string, data []byte) error {
	switch s.config.Type {
	case "local":
		return s.storeLocal(key, data)
	case "S3", "MinIO":
		return s.storeS3(key, data)
	default:
		return fmt.Errorf("unsupported storage type: %s", s.config.Type)
	}
}

// Retrieve retrieves data from storage
func (s *Storage) Retrieve(key string) ([]byte, error) {
	switch s.config.Type {
	case "local":
		return s.retrieveLocal(key)
	case "S3", "MinIO":
		return s.retrieveS3(key)
	default:
		return nil, fmt.Errorf("unsupported storage type: %s", s.config.Type)
	}
}

// Delete deletes data from storage
func (s *Storage) Delete(key string) error {
	switch s.config.Type {
	case "local":
		return s.deleteLocal(key)
	case "S3", "MinIO":
		return s.deleteS3(key)
	default:
		return fmt.Errorf("unsupported storage type: %s", s.config.Type)
	}
}

// Exists checks if data exists in storage
func (s *Storage) Exists(key string) (bool, error) {
	switch s.config.Type {
	case "local":
		return s.existsLocal(key)
	case "S3", "MinIO":
		return s.existsS3(key)
	default:
		return false, fmt.Errorf("unsupported storage type: %s", s.config.Type)
	}
}

// List lists data in storage
func (s *Storage) List(prefix string) ([]string, error) {
	switch s.config.Type {
	case "local":
		return s.listLocal(prefix)
	case "S3", "MinIO":
		return s.listS3(prefix)
	default:
		return nil, fmt.Errorf("unsupported storage type: %s", s.config.Type)
	}
}

// Local storage operations

func (s *Storage) initLocalStorage() error {
	// Create storage directory if it doesn't exist
	if err := os.MkdirAll(s.config.Path, 0755); err != nil {
		return fmt.Errorf("failed to create storage directory: %w", err)
	}
	return nil
}

func (s *Storage) storeLocal(key string, data []byte) error {
	filePath := filepath.Join(s.config.Path, key)
	
	// Create directory if it doesn't exist
	dir := filepath.Dir(filePath)
	if err := os.MkdirAll(dir, 0755); err != nil {
		return fmt.Errorf("failed to create directory: %w", err)
	}

	// Write data to file
	if err := os.WriteFile(filePath, data, 0644); err != nil {
		return fmt.Errorf("failed to write file: %w", err)
	}

	return nil
}

func (s *Storage) retrieveLocal(key string) ([]byte, error) {
	filePath := filepath.Join(s.config.Path, key)
	
	// Check if file exists
	if _, err := os.Stat(filePath); os.IsNotExist(err) {
		return nil, fmt.Errorf("file not found: %s", key)
	}

	// Read data from file
	data, err := os.ReadFile(filePath)
	if err != nil {
		return nil, fmt.Errorf("failed to read file: %w", err)
	}

	return data, nil
}

func (s *Storage) deleteLocal(key string) error {
	filePath := filepath.Join(s.config.Path, key)
	
	// Check if file exists
	if _, err := os.Stat(filePath); os.IsNotExist(err) {
		return fmt.Errorf("file not found: %s", key)
	}

	// Delete file
	if err := os.Remove(filePath); err != nil {
		return fmt.Errorf("failed to delete file: %w", err)
	}

	return nil
}

func (s *Storage) existsLocal(key string) (bool, error) {
	filePath := filepath.Join(s.config.Path, key)
	_, err := os.Stat(filePath)
	if os.IsNotExist(err) {
		return false, nil
	}
	if err != nil {
		return false, fmt.Errorf("failed to check file: %w", err)
	}
	return true, nil
}

func (s *Storage) listLocal(prefix string) ([]string, error) {
	searchPath := filepath.Join(s.config.Path, prefix)
	
	var files []string
	err := filepath.Walk(searchPath, func(path string, info os.FileInfo, err error) error {
		if err != nil {
			return err
		}
		if !info.IsDir() {
			relPath, err := filepath.Rel(s.config.Path, path)
			if err != nil {
				return err
			}
			files = append(files, relPath)
		}
		return nil
	})
	
	if err != nil {
		return nil, fmt.Errorf("failed to walk directory: %w", err)
	}

	return files, nil
}

// S3 storage operations

func (s *Storage) initS3Storage() error {
	// In a real implementation, this would initialize S3/MinIO client
	// For now, just validate configuration
	if s.config.Endpoint == "" {
		return fmt.Errorf("S3 endpoint is required")
	}
	if s.config.AccessKey == "" {
		return fmt.Errorf("S3 access key is required")
	}
	if s.config.SecretKey == "" {
		return fmt.Errorf("S3 secret key is required")
	}
	return nil
}

func (s *Storage) storeS3(key string, data []byte) error {
	// In a real implementation, this would upload to S3/MinIO
	// For now, just log the operation
	fmt.Printf("Storing to S3: %s (size: %d bytes)\n", key, len(data))
	return nil
}

func (s *Storage) retrieveS3(key string) ([]byte, error) {
	// In a real implementation, this would download from S3/MinIO
	// For now, return mock data
	return []byte("mock S3 data"), nil
}

func (s *Storage) deleteS3(key string) error {
	// In a real implementation, this would delete from S3/MinIO
	// For now, just log the operation
	fmt.Printf("Deleting from S3: %s\n", key)
	return nil
}

func (s *Storage) existsS3(key string) (bool, error) {
	// In a real implementation, this would check if object exists in S3/MinIO
	// For now, return true
	return true, nil
}

func (s *Storage) listS3(prefix string) ([]string, error) {
	// In a real implementation, this would list objects in S3/MinIO
	// For now, return empty list
	return []string{}, nil
}

// Utility functions

// GetURL returns the URL for a given key
func (s *Storage) GetURL(key string) (string, error) {
	switch s.config.Type {
	case "local":
		return filepath.Join(s.config.Path, key), nil
	case "S3", "MinIO":
		return fmt.Sprintf("%s/%s/%s", s.config.Endpoint, s.config.Bucket, key), nil
	default:
		return "", fmt.Errorf("unsupported storage type: %s", s.config.Type)
	}
}

// Copy copies data from source to destination
func (s *Storage) Copy(srcKey, dstKey string) error {
	// Retrieve data from source
	data, err := s.Retrieve(srcKey)
	if err != nil {
		return fmt.Errorf("failed to retrieve source data: %w", err)
	}

	// Store data to destination
	if err := s.Store(dstKey, data); err != nil {
		return fmt.Errorf("failed to store destination data: %w", err)
	}

	return nil
}

// Move moves data from source to destination
func (s *Storage) Move(srcKey, dstKey string) error {
	// Copy data
	if err := s.Copy(srcKey, dstKey); err != nil {
		return fmt.Errorf("failed to copy data: %w", err)
	}

	// Delete source
	if err := s.Delete(srcKey); err != nil {
		return fmt.Errorf("failed to delete source data: %w", err)
	}

	return nil
}

// GetSize returns the size of data for a given key
func (s *Storage) GetSize(key string) (int64, error) {
	switch s.config.Type {
	case "local":
		filePath := filepath.Join(s.config.Path, key)
		info, err := os.Stat(filePath)
		if err != nil {
			return 0, fmt.Errorf("failed to get file info: %w", err)
		}
		return info.Size(), nil
	case "S3", "MinIO":
		// In a real implementation, this would get object size from S3/MinIO
		return 0, fmt.Errorf("not implemented for S3/MinIO")
	default:
		return 0, fmt.Errorf("unsupported storage type: %s", s.config.Type)
	}
}

// GetReader returns a reader for a given key
func (s *Storage) GetReader(key string) (io.Reader, error) {
	switch s.config.Type {
	case "local":
		filePath := filepath.Join(s.config.Path, key)
		return os.Open(filePath)
	case "S3", "MinIO":
		// In a real implementation, this would return S3/MinIO reader
		return nil, fmt.Errorf("not implemented for S3/MinIO")
	default:
		return nil, fmt.Errorf("unsupported storage type: %s", s.config.Type)
	}
}
