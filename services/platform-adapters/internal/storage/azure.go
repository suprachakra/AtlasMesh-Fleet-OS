package storage

import (
	"bytes"
	"context"
	"fmt"
	"io"

	"github.com/Azure/azure-sdk-for-go/sdk/storage/azblob"
)

// azureBlobAdapter implements the Adapter interface for Azure Blob Storage.
type azureBlobAdapter struct {
	client *azblob.Client
	config AzureConfig
}

// newAzureBlobAdapter creates a new Azure Blob Storage adapter.
func newAzureBlobAdapter(ctx context.Context, cfg AzureConfig) (Adapter, error) {
	var client *azblob.Client
	var err error

	// Use connection string if provided
	if cfg.ConnectionString != "" {
		client, err = azblob.NewClientFromConnectionString(cfg.ConnectionString, nil)
		if err != nil {
			return nil, fmt.Errorf("failed to create Azure Blob client from connection string: %w", err)
		}
	} else {
		// Use account name and key
		credential, err := azblob.NewSharedKeyCredential(cfg.AccountName, cfg.AccountKey)
		if err != nil {
			return nil, fmt.Errorf("failed to create Azure shared key credential: %w", err)
		}

		serviceURL := fmt.Sprintf("https://%s.blob.core.windows.net/", cfg.AccountName)
		client, err = azblob.NewClientWithSharedKeyCredential(serviceURL, credential, nil)
		if err != nil {
			return nil, fmt.Errorf("failed to create Azure Blob client: %w", err)
		}
	}

	return &azureBlobAdapter{
		client: client,
		config: cfg,
	}, nil
}

// PutObject stores an object in Azure Blob Storage.
func (a *azureBlobAdapter) PutObject(ctx context.Context, bucket, key string, data []byte, metadata map[string]string) error {
	containerClient := a.client.ServiceClient().NewContainerClient(bucket)
	blobClient := containerClient.NewBlockBlobClient(key)

	// Convert metadata to Azure format
	azureMetadata := make(map[string]*string)
	for k, v := range metadata {
		val := v
		azureMetadata[k] = &val
	}

	_, err := blobClient.Upload(ctx, bytes.NewReader(data), &azblob.UploadBlockBlobOptions{
		Metadata: azureMetadata,
	})
	if err != nil {
		return fmt.Errorf("Azure Blob Upload failed: %w", err)
	}

	return nil
}

// GetObject retrieves an object from Azure Blob Storage.
func (a *azureBlobAdapter) GetObject(ctx context.Context, bucket, key string) ([]byte, Metadata, error) {
	containerClient := a.client.ServiceClient().NewContainerClient(bucket)
	blobClient := containerClient.NewBlockBlobClient(key)

	downloadResponse, err := blobClient.DownloadStream(ctx, nil)
	if err != nil {
		return nil, nil, fmt.Errorf("Azure Blob Download failed: %w", err)
	}
	defer downloadResponse.Body.Close()

	data, err := io.ReadAll(downloadResponse.Body)
	if err != nil {
		return nil, nil, fmt.Errorf("failed to read Azure Blob body: %w", err)
	}

	// Convert Azure metadata to generic format
	metadata := make(Metadata)
	for k, v := range downloadResponse.Metadata {
		if v != nil {
			metadata[k] = *v
		}
	}

	return data, metadata, nil
}

// DeleteObject deletes an object from Azure Blob Storage.
func (a *azureBlobAdapter) DeleteObject(ctx context.Context, bucket, key string) error {
	containerClient := a.client.ServiceClient().NewContainerClient(bucket)
	blobClient := containerClient.NewBlockBlobClient(key)

	_, err := blobClient.Delete(ctx, nil)
	if err != nil {
		return fmt.Errorf("Azure Blob Delete failed: %w", err)
	}

	return nil
}

// ListObjects lists objects in an Azure Blob container with optional prefix.
func (a *azureBlobAdapter) ListObjects(ctx context.Context, bucket, prefix string) ([]ObjectInfo, error) {
	containerClient := a.client.ServiceClient().NewContainerClient(bucket)

	pager := containerClient.NewListBlobsFlatPager(&azblob.ListBlobsFlatOptions{
		Prefix: &prefix,
	})

	objects := []ObjectInfo{}

	for pager.More() {
		page, err := pager.NextPage(ctx)
		if err != nil {
			return nil, fmt.Errorf("Azure Blob ListObjects failed: %w", err)
		}

		for _, blob := range page.Segment.BlobItems {
			if blob.Name == nil || blob.Properties == nil {
				continue
			}

			obj := ObjectInfo{
				Key:       *blob.Name,
				SizeBytes: *blob.Properties.ContentLength,
				Metadata:  map[string]string{},
			}

			if blob.Properties.ETag != nil {
				obj.ETag = string(*blob.Properties.ETag)
			}

			if blob.Properties.LastModified != nil {
				obj.LastModified = blob.Properties.LastModified.Unix()
			}

			objects = append(objects, obj)
		}
	}

	return objects, nil
}

// HealthCheck verifies Azure Blob Storage connectivity.
func (a *azureBlobAdapter) HealthCheck(ctx context.Context) error {
	// List containers to verify connectivity
	pager := a.client.NewListContainersPager(nil)
	_, err := pager.NextPage(ctx)
	if err != nil {
		return fmt.Errorf("Azure Blob health check failed: %w", err)
	}
	return nil
}

