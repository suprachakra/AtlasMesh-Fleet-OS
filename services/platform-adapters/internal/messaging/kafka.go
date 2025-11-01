package messaging

import (
	"context"
	"fmt"
	"time"

	"github.com/segmentio/kafka-go"
)

// kafkaAdapter implements the Adapter interface for Apache Kafka.
type kafkaAdapter struct {
	config KafkaConfig
	writer *kafka.Writer
}

// newKafkaAdapter creates a new Kafka adapter.
func newKafkaAdapter(ctx context.Context, cfg KafkaConfig) (Adapter, error) {
	if len(cfg.Brokers) == 0 {
		return nil, fmt.Errorf("kafka: no brokers configured")
	}

	writer := &kafka.Writer{
		Addr:         kafka.TCP(cfg.Brokers...),
		Balancer:     &kafka.LeastBytes{},
		BatchSize:    100,
		BatchTimeout: 10 * time.Millisecond,
	}

	return &kafkaAdapter{
		config: cfg,
		writer: writer,
	}, nil
}

// ProduceMessage sends a message to a Kafka topic.
func (a *kafkaAdapter) ProduceMessage(ctx context.Context, topic string, key string, value []byte, headers map[string]string) error {
	kafkaHeaders := make([]kafka.Header, 0, len(headers))
	for k, v := range headers {
		kafkaHeaders = append(kafkaHeaders, kafka.Header{
			Key:   k,
			Value: []byte(v),
		})
	}

	err := a.writer.WriteMessages(ctx, kafka.Message{
		Topic:   topic,
		Key:     []byte(key),
		Value:   value,
		Headers: kafkaHeaders,
		Time:    time.Now(),
	})

	if err != nil {
		return fmt.Errorf("kafka produce failed: %w", err)
	}

	return nil
}

// ConsumeMessages consumes messages from a Kafka topic.
func (a *kafkaAdapter) ConsumeMessages(ctx context.Context, topic string, groupID string, handler MessageHandler) error {
	reader := kafka.NewReader(kafka.ReaderConfig{
		Brokers:  a.config.Brokers,
		Topic:    topic,
		GroupID:  groupID,
		MinBytes: 10e3, // 10KB
		MaxBytes: 10e6, // 10MB
	})
	defer reader.Close()

	for {
		select {
		case <-ctx.Done():
			return ctx.Err()
		default:
			msg, err := reader.FetchMessage(ctx)
			if err != nil {
				if err == context.Canceled || err == context.DeadlineExceeded {
					return err
				}
				continue // Log error and continue
			}

			// Convert Kafka message to generic Message
			headers := make(map[string]string)
			for _, h := range msg.Headers {
				headers[h.Key] = string(h.Value)
			}

			genericMsg := &Message{
				Topic:     msg.Topic,
				Partition: msg.Partition,
				Offset:    msg.Offset,
				Key:       string(msg.Key),
				Value:     msg.Value,
				Headers:   headers,
				Timestamp: msg.Time.Unix(),
			}

			// Process message
			if err := handler(ctx, genericMsg); err != nil {
				// Log error but continue processing
				continue
			}

			// Commit message
			if err := reader.CommitMessages(ctx, msg); err != nil {
				// Log commit error
				continue
			}
		}
	}
}

// CreateTopic creates a new Kafka topic.
func (a *kafkaAdapter) CreateTopic(ctx context.Context, topic string, partitions int, replication int) error {
	conn, err := kafka.Dial("tcp", a.config.Brokers[0])
	if err != nil {
		return fmt.Errorf("kafka dial failed: %w", err)
	}
	defer conn.Close()

	topicConfigs := []kafka.TopicConfig{
		{
			Topic:             topic,
			NumPartitions:     partitions,
			ReplicationFactor: replication,
		},
	}

	err = conn.CreateTopics(topicConfigs...)
	if err != nil {
		return fmt.Errorf("kafka create topic failed: %w", err)
	}

	return nil
}

// DeleteTopic deletes a Kafka topic.
func (a *kafkaAdapter) DeleteTopic(ctx context.Context, topic string) error {
	conn, err := kafka.Dial("tcp", a.config.Brokers[0])
	if err != nil {
		return fmt.Errorf("kafka dial failed: %w", err)
	}
	defer conn.Close()

	err = conn.DeleteTopics(topic)
	if err != nil {
		return fmt.Errorf("kafka delete topic failed: %w", err)
	}

	return nil
}

// HealthCheck verifies Kafka connectivity.
func (a *kafkaAdapter) HealthCheck(ctx context.Context) error {
	conn, err := kafka.DialContext(ctx, "tcp", a.config.Brokers[0])
	if err != nil {
		return fmt.Errorf("kafka health check failed: %w", err)
	}
	defer conn.Close()

	_, err = conn.Brokers()
	if err != nil {
		return fmt.Errorf("kafka broker list failed: %w", err)
	}

	return nil
}

// Close closes the Kafka adapter.
func (a *kafkaAdapter) Close() error {
	if a.writer != nil {
		return a.writer.Close()
	}
	return nil
}

