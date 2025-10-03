package storage

import (
	"context"
	"errors"
	"fmt"
)

// Provider identifies a storage backend implementation.
type Provider string

const (
	ProviderAWS   Provider = "aws"
	ProviderAzure Provider = "azure"
	ProviderGCP   Provider = "gcp"
)

var (
	// ErrUnsupportedProvider is returned when a provider is not recognised.
	ErrUnsupportedProvider = errors.New("storage: unsupported provider")
)

// Adapter defines the interface for object storage implementations (S3, Blob, GCS).
type Adapter interface {
	PutObject(ctx context.Context, bucket, key string, data []byte, metadata map[string]string) error
	GetObject(ctx context.Context, bucket, key string) ([]byte, Metadata, error)
	DeleteObject(ctx context.Context, bucket, key string) error
	ListObjects(ctx context.Context, bucket, prefix string) ([]ObjectInfo, error)
	HealthCheck(ctx context.Context) error
}

// Metadata represents object metadata for storage providers.
type Metadata map[string]string

// ObjectInfo describes a stored object.
type ObjectInfo struct {
	Key          string            `json:"key"`
	SizeBytes    int64             `json:"size_bytes"`
	ETag         string            `json:"etag"`
	LastModified int64             `json:"last_modified_unix"`
	Metadata     map[string]string `json:"metadata"`
}

// AWSConfig holds configuration for the AWS S3 adapter.
type AWSConfig struct {
	Region       string
	Profile      string
	AccessKey    string
	SecretKey    string
	SessionToken string
	Endpoint     string
	BucketPrefix string
}

// AzureConfig holds configuration for the Azure Blob adapter.
type AzureConfig struct {
	AccountName      string
	AccountKey       string
	ConnectionString string
	ContainerPrefix  string
}

// GCPConfig holds configuration for the Google Cloud Storage adapter.
type GCPConfig struct {
	ProjectID       string
	CredentialsJSON []byte
	BucketPrefix    string
}

// NewAdapter returns a provider-specific storage adapter based on configuration.
func NewAdapter(ctx context.Context, provider Provider, cfg any) (Adapter, error) {
	switch provider {
	case ProviderAWS:
		awsCfg, ok := cfg.(AWSConfig)
		if !ok {
			return nil, fmt.Errorf("storage: expected AWSConfig, got %T", cfg)
		}
		return newS3Adapter(ctx, awsCfg)
	case ProviderAzure:
		azureCfg, ok := cfg.(AzureConfig)
		if !ok {
			return nil, fmt.Errorf("storage: expected AzureConfig, got %T", cfg)
		}
		return newAzureBlobAdapter(ctx, azureCfg)
	case ProviderGCP:
		gcpCfg, ok := cfg.(GCPConfig)
		if !ok {
			return nil, fmt.Errorf("storage: expected GCPConfig, got %T", cfg)
		}
		return newGCSAdapter(ctx, gcpCfg)
	default:
		return nil, ErrUnsupportedProvider
	}
}
