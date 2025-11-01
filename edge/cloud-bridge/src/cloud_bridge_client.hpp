/**
 * @file cloud_bridge_client.hpp
 * @brief Cloud Bridge Client for AtlasMesh Fleet OS
 * 
 * Handles communication between vehicle edge and cloud services:
 * - MQTT/gRPC/WebSocket protocols
 * - Store-and-forward for offline operation
 * - Message queuing and retry logic
 * - Telemetry streaming and command reception
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <string>
#include <queue>
#include <mutex>
#include <atomic>
#include <thread>
#include <chrono>
#include <functional>

#include "atlasmesh_vehicle_agent/msg/vehicle_state.hpp"
#include "atlasmesh_vehicle_agent/msg/vehicle_command.hpp"
#include "atlasmesh_vehicle_agent/msg/telemetry_data.hpp"

// Forward declarations for protocol implementations
class MQTTClient;
class GRPCClient;
class WebSocketClient;

/**
 * @enum ConnectionProtocol
 * @brief Supported communication protocols
 */
enum class ConnectionProtocol {
    MQTT,
    GRPC,
    WEBSOCKET,
    AUTO  // Automatically select best available
};

/**
 * @enum ConnectionStatus
 * @brief Connection status states
 */
enum class ConnectionStatus {
    DISCONNECTED,
    CONNECTING,
    CONNECTED,
    RECONNECTING,
    FAILED
};

/**
 * @enum MessagePriority
 * @brief Message priority levels for queuing
 */
enum class MessagePriority {
    LOW = 0,
    NORMAL = 1,
    HIGH = 2,
    CRITICAL = 3,
    EMERGENCY = 4
};

/**
 * @struct QueuedMessage
 * @brief Message wrapper for store-and-forward queue
 */
struct QueuedMessage {
    std::string topic;
    std::string payload;
    MessagePriority priority;
    std::chrono::steady_clock::time_point timestamp;
    int retry_count;
    int max_retries;
    std::chrono::seconds ttl;  // Time to live
    
    QueuedMessage(const std::string& t, const std::string& p, MessagePriority prio = MessagePriority::NORMAL)
        : topic(t), payload(p), priority(prio), 
          timestamp(std::chrono::steady_clock::now()), 
          retry_count(0), max_retries(3), ttl(std::chrono::seconds(300)) {}
    
    bool is_expired() const {
        return std::chrono::steady_clock::now() - timestamp > ttl;
    }
    
    bool should_retry() const {
        return retry_count < max_retries && !is_expired();
    }
};

/**
 * @class CloudBridgeClient
 * @brief Main cloud communication client
 */
class CloudBridgeClient
{
public:
    /**
     * @brief Constructor
     * @param node ROS2 node handle
     * @param vehicle_id Unique vehicle identifier
     */
    CloudBridgeClient(std::shared_ptr<rclcpp::Node> node, const std::string& vehicle_id);
    
    /**
     * @brief Destructor
     */
    ~CloudBridgeClient();
    
    /**
     * @brief Initialize and connect to cloud services
     * @param protocol Preferred connection protocol
     * @return True if connection successful
     */
    bool connect(ConnectionProtocol protocol = ConnectionProtocol::AUTO);
    
    /**
     * @brief Disconnect from cloud services
     */
    void disconnect();
    
    /**
     * @brief Publish vehicle telemetry
     * @param state Vehicle state message
     */
    void publish_telemetry(const atlasmesh_vehicle_agent::msg::VehicleState& state);
    
    /**
     * @brief Publish telemetry data
     * @param data Telemetry data message
     */
    void publish_telemetry_data(const atlasmesh_vehicle_agent::msg::TelemetryData& data);
    
    /**
     * @brief Report emergency stop event
     */
    void report_emergency_stop();
    
    /**
     * @brief Report system health status
     * @param health_data Health status JSON
     */
    void report_health_status(const std::string& health_data);
    
    /**
     * @brief Send command acknowledgment
     * @param command_id Command ID being acknowledged
     * @param status Execution status
     * @param message Optional status message
     */
    void send_command_ack(const std::string& command_id, bool status, const std::string& message = "");
    
    /**
     * @brief Get connection status
     * @return Current connection status
     */
    ConnectionStatus get_connection_status() const { return connection_status_; }
    
    /**
     * @brief Check if connected to cloud
     * @return True if connected
     */
    bool is_connected() const { return connection_status_ == ConnectionStatus::CONNECTED; }
    
    /**
     * @brief Get queued message count
     * @return Number of messages in store-and-forward queue
     */
    size_t get_queue_size() const;
    
    /**
     * @brief Set command callback
     * @param callback Function to call when command received
     */
    void set_command_callback(std::function<void(const atlasmesh_vehicle_agent::msg::VehicleCommand&)> callback);
    
    /**
     * @brief Enable/disable store-and-forward mode
     * @param enabled True to enable offline queuing
     */
    void set_store_and_forward_enabled(bool enabled) { store_and_forward_enabled_ = enabled; }
    
    /**
     * @brief Set maximum queue size for store-and-forward
     * @param max_size Maximum number of queued messages
     */
    void set_max_queue_size(size_t max_size) { max_queue_size_ = max_size; }

private:
    // ROS2 node handle
    std::shared_ptr<rclcpp::Node> node_;
    
    // Vehicle identification
    std::string vehicle_id_;
    
    // Connection management
    std::atomic<ConnectionStatus> connection_status_{ConnectionStatus::DISCONNECTED};
    ConnectionProtocol current_protocol_;
    std::string cloud_endpoint_;
    std::string mqtt_broker_;
    int mqtt_port_;
    bool use_tls_;
    
    // Protocol clients
    std::unique_ptr<MQTTClient> mqtt_client_;
    std::unique_ptr<GRPCClient> grpc_client_;
    std::unique_ptr<WebSocketClient> websocket_client_;
    
    // Store-and-forward queue
    std::priority_queue<QueuedMessage> message_queue_;
    mutable std::mutex queue_mutex_;
    std::atomic<bool> store_and_forward_enabled_{true};
    std::atomic<size_t> max_queue_size_{10000};
    
    // Background processing
    std::unique_ptr<std::thread> connection_thread_;
    std::unique_ptr<std::thread> queue_processor_thread_;
    std::atomic<bool> running_{false};
    
    // Callbacks
    std::function<void(const atlasmesh_vehicle_agent::msg::VehicleCommand&)> command_callback_;
    
    // Statistics
    std::atomic<uint64_t> messages_sent_{0};
    std::atomic<uint64_t> messages_failed_{0};
    std::atomic<uint64_t> bytes_sent_{0};
    std::atomic<uint64_t> bytes_received_{0};
    
    // Timing
    std::chrono::steady_clock::time_point last_heartbeat_;
    std::chrono::steady_clock::time_point last_successful_send_;
    
    // Private methods
    void initialize_configuration();
    void start_background_threads();
    void stop_background_threads();
    
    void connection_manager_loop();
    void queue_processor_loop();
    
    bool attempt_connection(ConnectionProtocol protocol);
    void handle_connection_loss();
    void send_heartbeat();
    
    void queue_message(const std::string& topic, const std::string& payload, 
                      MessagePriority priority = MessagePriority::NORMAL);
    bool send_message_immediate(const std::string& topic, const std::string& payload);
    void process_queued_messages();
    void cleanup_expired_messages();
    
    std::string serialize_vehicle_state(const atlasmesh_vehicle_agent::msg::VehicleState& state);
    std::string serialize_telemetry_data(const atlasmesh_vehicle_agent::msg::TelemetryData& data);
    
    void handle_incoming_command(const std::string& command_json);
    atlasmesh_vehicle_agent::msg::VehicleCommand deserialize_command(const std::string& command_json);
    
    // Protocol-specific implementations
    bool connect_mqtt();
    bool connect_grpc();
    bool connect_websocket();
    
    void disconnect_mqtt();
    void disconnect_grpc();
    void disconnect_websocket();
    
    bool send_mqtt_message(const std::string& topic, const std::string& payload);
    bool send_grpc_message(const std::string& topic, const std::string& payload);
    bool send_websocket_message(const std::string& topic, const std::string& payload);
    
    // Compression and encryption
    std::string compress_payload(const std::string& payload);
    std::string decompress_payload(const std::string& compressed_payload);
    std::string encrypt_payload(const std::string& payload);
    std::string decrypt_payload(const std::string& encrypted_payload);
    
    // Network diagnostics
    void update_network_metrics();
    double get_network_latency();
    double get_network_bandwidth();
    int get_signal_strength();
};

/**
 * @class MQTTClient
 * @brief MQTT protocol implementation
 */
class MQTTClient
{
public:
    MQTTClient(const std::string& broker, int port, bool use_tls);
    ~MQTTClient();
    
    bool connect(const std::string& client_id);
    void disconnect();
    bool is_connected() const;
    
    bool publish(const std::string& topic, const std::string& payload, int qos = 1);
    bool subscribe(const std::string& topic, int qos = 1);
    
    void set_message_callback(std::function<void(const std::string&, const std::string&)> callback);
    void set_connection_callback(std::function<void(bool)> callback);

private:
    std::string broker_;
    int port_;
    bool use_tls_;
    std::atomic<bool> connected_{false};
    
    // MQTT client implementation (would use library like Paho MQTT)
    void* mqtt_client_handle_;  // Placeholder for actual MQTT client
    
    std::function<void(const std::string&, const std::string&)> message_callback_;
    std::function<void(bool)> connection_callback_;
    
    void initialize_mqtt_client();
    void cleanup_mqtt_client();
    
    static void on_message_received(const std::string& topic, const std::string& payload, void* user_data);
    static void on_connection_changed(bool connected, void* user_data);
};

/**
 * @class GRPCClient
 * @brief gRPC protocol implementation
 */
class GRPCClient
{
public:
    GRPCClient(const std::string& endpoint, bool use_tls);
    ~GRPCClient();
    
    bool connect();
    void disconnect();
    bool is_connected() const;
    
    bool send_telemetry(const std::string& vehicle_id, const std::string& telemetry_data);
    bool send_command_ack(const std::string& command_id, bool status, const std::string& message);
    
    void set_command_callback(std::function<void(const std::string&)> callback);

private:
    std::string endpoint_;
    bool use_tls_;
    std::atomic<bool> connected_{false};
    
    // gRPC client implementation (would use gRPC C++ library)
    void* grpc_channel_;     // Placeholder for gRPC channel
    void* grpc_stub_;        // Placeholder for gRPC service stub
    
    std::function<void(const std::string&)> command_callback_;
    std::unique_ptr<std::thread> command_stream_thread_;
    
    void initialize_grpc_client();
    void cleanup_grpc_client();
    void command_stream_loop();
};

/**
 * @class WebSocketClient
 * @brief WebSocket protocol implementation
 */
class WebSocketClient
{
public:
    WebSocketClient(const std::string& url, bool use_tls);
    ~WebSocketClient();
    
    bool connect();
    void disconnect();
    bool is_connected() const;
    
    bool send_message(const std::string& message);
    void set_message_callback(std::function<void(const std::string&)> callback);

private:
    std::string url_;
    bool use_tls_;
    std::atomic<bool> connected_{false};
    
    // WebSocket client implementation (would use library like websocketpp)
    void* websocket_client_;  // Placeholder for WebSocket client
    
    std::function<void(const std::string&)> message_callback_;
    std::unique_ptr<std::thread> receive_thread_;
    
    void initialize_websocket_client();
    void cleanup_websocket_client();
    void receive_loop();
};

/**
 * @class NetworkMonitor
 * @brief Network connectivity and performance monitoring
 */
class NetworkMonitor
{
public:
    NetworkMonitor();
    ~NetworkMonitor();
    
    struct NetworkMetrics {
        bool is_connected;
        std::string connection_type;  // "4G", "5G", "WiFi", "Ethernet", "Satellite"
        int signal_strength;          // dBm or percentage
        double latency_ms;
        double bandwidth_mbps;
        double packet_loss_percent;
        std::chrono::steady_clock::time_point last_update;
    };
    
    NetworkMetrics get_current_metrics();
    bool is_network_available();
    std::string get_best_available_connection();
    
    void start_monitoring();
    void stop_monitoring();

private:
    std::atomic<bool> monitoring_{false};
    std::unique_ptr<std::thread> monitor_thread_;
    NetworkMetrics current_metrics_;
    mutable std::mutex metrics_mutex_;
    
    void monitoring_loop();
    void update_metrics();
    
    // Platform-specific network monitoring
    void check_cellular_connection();
    void check_wifi_connection();
    void check_ethernet_connection();
    void check_satellite_connection();
    
    double measure_latency(const std::string& host);
    double measure_bandwidth();
};
