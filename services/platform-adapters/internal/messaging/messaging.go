package messaging

import (
	"context"
	"errors"
	"fmt"
)

// Provider identifies a messaging backend implementation.
type Provider string

const (
	ProviderKafka      Provider = "kafka"
	ProviderEventHubs  Provider = "azure_eventhubs"
	ProviderKinesis    Provider = "aws_kinesis"
	ProviderPubSub     Provider = "gcp_pubsub"
)

var (
	// ErrUnsupportedProvider is returned when a provider is not recognised.
	ErrUnsupportedProvider = errors.New("messaging: unsupported provider")
)

// Adapter defines the interface for messaging implementations (Kafka, EventHubs, etc.).
type Adapter interface {
	ProduceMessage(ctx context.Context, topic string, key string, value []byte, headers map[string]string) error
	ConsumeMessages(ctx context.Context, topic string, groupID string, handler MessageHandler) error
	CreateTopic(ctx context.Context, topic string, partitions int, replication int) error
	DeleteTopic(ctx context.Context, topic string) error
	HealthCheck(ctx context.Context) error
	Close() error
}

// MessageHandler processes consumed messages.
type MessageHandler func(ctx context.Context, msg *Message) error

// Message represents a message in the messaging system.
type Message struct {
	Topic     string
	Partition int
	Offset    int64
	Key       string
	Value     []byte
	Headers   map[string]string
	Timestamp int64
}

// KafkaConfig holds configuration for Kafka adapter.
type KafkaConfig struct {
	Brokers       []string
	SASL          *SASLConfig
	TLS           *TLSConfig
	ClientID      string
}

// SASLConfig holds SASL authentication configuration.
type SASLConfig struct {
	Mechanism string
	Username  string
	Password  string
}

// TLSConfig holds TLS configuration.
type TLSConfig struct {
	Enabled            bool
	InsecureSkipVerify bool
	CertFile           string
	KeyFile            string
	CAFile             string
}

// EventHubsConfig holds configuration for Azure Event Hubs.
type EventHubsConfig struct {
	Namespace         string
	ConnectionString  string
	ConsumerGroup     string
}

// KinesisConfig holds configuration for AWS Kinesis.
type KinesisConfig struct {
	Region    string
	StreamARN string
}

// PubSubConfig holds configuration for Google Cloud Pub/Sub.
type PubSubConfig struct {
	ProjectID string
}

// NewAdapter returns a provider-specific messaging adapter.
func NewAdapter(ctx context.Context, provider Provider, cfg any) (Adapter, error) {
	switch provider {
	case ProviderKafka:
		kafkaCfg, ok := cfg.(KafkaConfig)
		if !ok {
			return nil, fmt.Errorf("messaging: expected KafkaConfig, got %T", cfg)
		}
		return newKafkaAdapter(ctx, kafkaCfg)
	case ProviderEventHubs:
		ehCfg, ok := cfg.(EventHubsConfig)
		if !ok {
			return nil, fmt.Errorf("messaging: expected EventHubsConfig, got %T", cfg)
		}
		return newEventHubsAdapter(ctx, ehCfg)
	case ProviderKinesis:
		kinesisCfg, ok := cfg.(KinesisConfig)
		if !ok {
			return nil, fmt.Errorf("messaging: expected KinesisConfig, got %T", cfg)
		}
		return newKinesisAdapter(ctx, kinesisCfg)
	case ProviderPubSub:
		pubsubCfg, ok := cfg.(PubSubConfig)
		if !ok {
			return nil, fmt.Errorf("messaging: expected PubSubConfig, got %T", cfg)
		}
		return newPubSubAdapter(ctx, pubsubCfg)
	default:
		return nil, ErrUnsupportedProvider
	}
}

