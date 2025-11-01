package storage

import (
	"bytes"
	"context"
	"fmt"
	"io"

	"cloud.google.com/go/storage"
	"google.golang.org/api/iterator"
)

// gcsAdapter implements the Adapter interface for Google Cloud Storage.
type gcsAdapter struct {
	client *storage.Client
	config GCPConfig
}

// newGCSAdapter creates a new Google Cloud Storage adapter.
func newGCSAdapter(ctx context.Context, cfg GCPConfig) (Adapter, error) {
	client, err := storage.NewClient(ctx)
	if err != nil {
		return nil, fmt.Errorf("failed to create GCS client: %w", err)
	}

	return &gcsAdapter{
		client: client,
		config: cfg,
	}, nil
}

// PutObject stores an object in GCS.
func (a *gcsAdapter) PutObject(ctx context.Context, bucket, key string, data []byte, metadata map[string]string) error {
	obj := a.client.Bucket(bucket).Object(key)
	writer := obj.NewWriter(ctx)

	// Set metadata
	writer.Metadata = metadata

	if _, err := io.Copy(writer, bytes.NewReader(data)); err != nil {
		writer.Close()
		return fmt.Errorf("GCS write failed: %w", err)
	}

	if err := writer.Close(); err != nil {
		return fmt.Errorf("GCS PutObject failed: %w", err)
	}

	return nil
}

// GetObject retrieves an object from GCS.
func (a *gcsAdapter) GetObject(ctx context.Context, bucket, key string) ([]byte, Metadata, error) {
	obj := a.client.Bucket(bucket).Object(key)
	reader, err := obj.NewReader(ctx)
	if err != nil {
		return nil, nil, fmt.Errorf("GCS GetObject failed: %w", err)
	}
	defer reader.Close()

	data, err := io.ReadAll(reader)
	if err != nil {
		return nil, nil, fmt.Errorf("failed to read GCS object body: %w", err)
	}

	// Get object attributes for metadata
	attrs, err := obj.Attrs(ctx)
	if err != nil {
		return data, nil, nil // Return data even if metadata fetch fails
	}

	return data, Metadata(attrs.Metadata), nil
}

// DeleteObject deletes an object from GCS.
func (a *gcsAdapter) DeleteObject(ctx context.Context, bucket, key string) error {
	obj := a.client.Bucket(bucket).Object(key)
	
	if err := obj.Delete(ctx); err != nil {
		return fmt.Errorf("GCS DeleteObject failed: %w", err)
	}

	return nil
}

// ListObjects lists objects in a GCS bucket with optional prefix.
func (a *gcsAdapter) ListObjects(ctx context.Context, bucket, prefix string) ([]ObjectInfo, error) {
	query := &storage.Query{
		Prefix: prefix,
	}

	it := a.client.Bucket(bucket).Objects(ctx, query)

	objects := []ObjectInfo{}

	for {
		attrs, err := it.Next()
		if err == iterator.Done {
			break
		}
		if err != nil {
			return nil, fmt.Errorf("GCS ListObjects iteration failed: %w", err)
		}

		obj := ObjectInfo{
			Key:          attrs.Name,
			SizeBytes:    attrs.Size,
			ETag:         fmt.Sprintf("%x", attrs.MD5),
			LastModified: attrs.Updated.Unix(),
			Metadata:     attrs.Metadata,
		}

		objects = append(objects, obj)
	}

	return objects, nil
}

// HealthCheck verifies GCS connectivity.
func (a *gcsAdapter) HealthCheck(ctx context.Context) error {
	// List buckets to verify connectivity
	it := a.client.Buckets(ctx, a.config.ProjectID)
	_, err := it.Next()
	if err != nil && err != iterator.Done {
		return fmt.Errorf("GCS health check failed: %w", err)
	}
	return nil
}

