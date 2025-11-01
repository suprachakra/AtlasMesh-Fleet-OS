package messaging

import (
	"context"
	"fmt"

	"github.com/Azure/azure-sdk-for-go/sdk/messaging/azeventhubs"
)

// eventHubsAdapter implements the Adapter interface for Azure Event Hubs.
type eventHubsAdapter struct {
	producerClient *azeventhubs.ProducerClient
	config         EventHubsConfig
}

// newEventHubsAdapter creates a new Azure Event Hubs adapter.
func newEventHubsAdapter(ctx context.Context, cfg EventHubsConfig) (Adapter, error) {
	producerClient, err := azeventhubs.NewProducerClientFromConnectionString(
		cfg.ConnectionString,
		cfg.Namespace,
		nil,
	)
	if err != nil {
		return nil, fmt.Errorf("failed to create Event Hubs producer: %w", err)
	}

	return &eventHubsAdapter{
		producerClient: producerClient,
		config:         cfg,
	}, nil
}

// ProduceMessage sends a message to an Event Hub.
func (a *eventHubsAdapter) ProduceMessage(ctx context.Context, topic string, key string, value []byte, headers map[string]string) error {
	// Convert headers to Event Hubs properties
	properties := make(map[string]any)
	for k, v := range headers {
		properties[k] = v
	}

	eventData := &azeventhubs.EventData{
		Body:       value,
		Properties: properties,
	}

	batch, err := a.producerClient.NewEventDataBatch(ctx, nil)
	if err != nil {
		return fmt.Errorf("failed to create Event Hubs batch: %w", err)
	}

	err = batch.AddEventData(eventData, nil)
	if err != nil {
		return fmt.Errorf("failed to add event to batch: %w", err)
	}

	err = a.producerClient.SendEventDataBatch(ctx, batch, nil)
	if err != nil {
		return fmt.Errorf("Event Hubs produce failed: %w", err)
	}

	return nil
}

// ConsumeMessages consumes messages from an Event Hub.
func (a *eventHubsAdapter) ConsumeMessages(ctx context.Context, topic string, groupID string, handler MessageHandler) error {
	consumerClient, err := azeventhubs.NewConsumerClientFromConnectionString(
		a.config.ConnectionString,
		topic,
		groupID,
		nil,
	)
	if err != nil {
		return fmt.Errorf("failed to create Event Hubs consumer: %w", err)
	}
	defer consumerClient.Close(ctx)

	// Get partition IDs
	props, err := consumerClient.GetEventHubProperties(ctx, nil)
	if err != nil {
		return fmt.Errorf("failed to get Event Hub properties: %w", err)
	}

	// Consume from all partitions
	for _, partitionID := range props.PartitionIDs {
		go func(pID string) {
			partitionClient, err := consumerClient.NewPartitionClient(pID, nil)
			if err != nil {
				return
			}
			defer partitionClient.Close(ctx)

			for {
				select {
				case <-ctx.Done():
					return
				default:
					events, err := partitionClient.ReceiveEvents(ctx, 100, nil)
					if err != nil {
						continue
					}

					for _, event := range events {
						// Convert to generic Message
						headers := make(map[string]string)
						for k, v := range event.Properties {
							if str, ok := v.(string); ok {
								headers[k] = str
							}
						}

						msg := &Message{
							Topic:     topic,
							Key:       "", // Event Hubs doesn't have keys
							Value:     event.Body,
							Headers:   headers,
							Timestamp: event.EnqueuedTime.Unix(),
						}

						// Process message
						if err := handler(ctx, msg); err != nil {
							// Log error but continue
							continue
						}
					}
				}
			}
		}(partitionID)
	}

	<-ctx.Done()
	return ctx.Err()
}

// CreateTopic creates a new Event Hub (requires admin permissions).
func (a *eventHubsAdapter) CreateTopic(ctx context.Context, topic string, partitions int, replication int) error {
	// Event Hub creation typically done via Azure portal or management SDK
	return fmt.Errorf("Event Hub creation not supported via client SDK")
}

// DeleteTopic deletes an Event Hub (requires admin permissions).
func (a *eventHubsAdapter) DeleteTopic(ctx context.Context, topic string) error {
	// Event Hub deletion typically done via Azure portal or management SDK
	return fmt.Errorf("Event Hub deletion not supported via client SDK")
}

// HealthCheck verifies Event Hubs connectivity.
func (a *eventHubsAdapter) HealthCheck(ctx context.Context) error {
	// Check producer client connectivity
	if a.producerClient == nil {
		return fmt.Errorf("Event Hubs producer client not initialized")
	}
	return nil
}

// Close closes the Event Hubs adapter.
func (a *eventHubsAdapter) Close() error {
	if a.producerClient != nil {
		return a.producerClient.Close(context.Background())
	}
	return nil
}

