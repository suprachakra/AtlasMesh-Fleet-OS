package storage

import (
	"bytes"
	"context"
	"fmt"
	"io"

	"github.com/aws/aws-sdk-go-v2/aws"
	"github.com/aws/aws-sdk-go-v2/config"
	"github.com/aws/aws-sdk-go-v2/service/s3"
)

// s3Adapter implements the Adapter interface for AWS S3.
type s3Adapter struct {
	client *s3.Client
	config AWSConfig
}

// newS3Adapter creates a new AWS S3 adapter.
func newS3Adapter(ctx context.Context, cfg AWSConfig) (Adapter, error) {
	// Load AWS config
	awsCfg, err := config.LoadDefaultConfig(ctx,
		config.WithRegion(cfg.Region),
	)
	if err != nil {
		return nil, fmt.Errorf("failed to load AWS config: %w", err)
	}

	// Override endpoint if specified (for MinIO compatibility)
	var options []func(*s3.Options)
	if cfg.Endpoint != "" {
		options = append(options, func(o *s3.Options) {
			o.BaseEndpoint = aws.String(cfg.Endpoint)
			o.UsePathStyle = true
		})
	}

	client := s3.NewFromConfig(awsCfg, options...)

	return &s3Adapter{
		client: client,
		config: cfg,
	}, nil
}

// PutObject stores an object in S3.
func (a *s3Adapter) PutObject(ctx context.Context, bucket, key string, data []byte, metadata map[string]string) error {
	input := &s3.PutObjectInput{
		Bucket:   aws.String(bucket),
		Key:      aws.String(key),
		Body:     bytes.NewReader(data),
		Metadata: metadata,
	}

	_, err := a.client.PutObject(ctx, input)
	if err != nil {
		return fmt.Errorf("S3 PutObject failed: %w", err)
	}

	return nil
}

// GetObject retrieves an object from S3.
func (a *s3Adapter) GetObject(ctx context.Context, bucket, key string) ([]byte, Metadata, error) {
	input := &s3.GetObjectInput{
		Bucket: aws.String(bucket),
		Key:    aws.String(key),
	}

	result, err := a.client.GetObject(ctx, input)
	if err != nil {
		return nil, nil, fmt.Errorf("S3 GetObject failed: %w", err)
	}
	defer result.Body.Close()

	data, err := io.ReadAll(result.Body)
	if err != nil {
		return nil, nil, fmt.Errorf("failed to read S3 object body: %w", err)
	}

	return data, Metadata(result.Metadata), nil
}

// DeleteObject deletes an object from S3.
func (a *s3Adapter) DeleteObject(ctx context.Context, bucket, key string) error {
	input := &s3.DeleteObjectInput{
		Bucket: aws.String(bucket),
		Key:    aws.String(key),
	}

	_, err := a.client.DeleteObject(ctx, input)
	if err != nil {
		return fmt.Errorf("S3 DeleteObject failed: %w", err)
	}

	return nil
}

// ListObjects lists objects in an S3 bucket with optional prefix.
func (a *s3Adapter) ListObjects(ctx context.Context, bucket, prefix string) ([]ObjectInfo, error) {
	input := &s3.ListObjectsV2Input{
		Bucket: aws.String(bucket),
	}

	if prefix != "" {
		input.Prefix = aws.String(prefix)
	}

	result, err := a.client.ListObjectsV2(ctx, input)
	if err != nil {
		return nil, fmt.Errorf("S3 ListObjects failed: %w", err)
	}

	objects := make([]ObjectInfo, 0, len(result.Contents))
	for _, obj := range result.Contents {
		objects = append(objects, ObjectInfo{
			Key:          *obj.Key,
			SizeBytes:    *obj.Size,
			ETag:         *obj.ETag,
			LastModified: obj.LastModified.Unix(),
			Metadata:     map[string]string{},
		})
	}

	return objects, nil
}

// HealthCheck verifies S3 connectivity.
func (a *s3Adapter) HealthCheck(ctx context.Context) error {
	// List buckets to verify connectivity
	_, err := a.client.ListBuckets(ctx, &s3.ListBucketsInput{})
	if err != nil {
		return fmt.Errorf("S3 health check failed: %w", err)
	}
	return nil
}

