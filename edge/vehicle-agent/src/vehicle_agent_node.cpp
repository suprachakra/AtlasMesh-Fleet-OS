/**
 * @file vehicle_agent_node.cpp
 * @brief Main vehicle agent node for AtlasMesh Fleet OS
 * 
 * This is the primary ROS2 node running on each vehicle, responsible for:
 * - Vehicle control and coordination
 * - Safety monitoring and enforcement
 * - Cloud communication and telemetry
 * - Policy evaluation and compliance
 * - OTA update management
 */

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <chrono>
#include <thread>

#include "vehicle_controller.hpp"
#include "safety_monitor.hpp"
#include "health_monitor.hpp"
#include "policy_client.hpp"
#include "cloud_bridge_client.hpp"
#include "sensor_manager.hpp"
#include "navigation_manager.hpp"
#include "ota_manager.hpp"

#include "atlasmesh_vehicle_agent/msg/vehicle_state.hpp"
#include "atlasmesh_vehicle_agent/msg/vehicle_command.hpp"
#include "atlasmesh_vehicle_agent/msg/vehicle_profile.hpp"

using namespace std::chrono_literals;

class VehicleAgentNode : public rclcpp::Node
{
public:
    VehicleAgentNode() : Node("vehicle_agent_node")
    {
        RCLCPP_INFO(this->get_logger(), "Initializing AtlasMesh Vehicle Agent...");
        
        // Declare parameters
        declare_parameters();
        
        // Load vehicle profile
        load_vehicle_profile();
        
        // Initialize components
        initialize_components();
        
        // Setup publishers and subscribers
        setup_communication();
        
        // Start main control loop
        start_control_loop();
        
        RCLCPP_INFO(this->get_logger(), "Vehicle Agent initialized successfully");
    }

    ~VehicleAgentNode()
    {
        RCLCPP_INFO(this->get_logger(), "Shutting down Vehicle Agent...");
        shutdown_components();
    }

private:
    // Core components
    std::unique_ptr<VehicleController> vehicle_controller_;
    std::unique_ptr<SafetyMonitor> safety_monitor_;
    std::unique_ptr<HealthMonitor> health_monitor_;
    std::unique_ptr<PolicyClient> policy_client_;
    std::unique_ptr<CloudBridgeClient> cloud_bridge_;
    std::unique_ptr<SensorManager> sensor_manager_;
    std::unique_ptr<NavigationManager> navigation_manager_;
    std::unique_ptr<OTAManager> ota_manager_;

    // ROS2 communication
    rclcpp::Publisher<atlasmesh_vehicle_agent::msg::VehicleState>::SharedPtr state_publisher_;
    rclcpp::Subscription<atlasmesh_vehicle_agent::msg::VehicleCommand>::SharedPtr command_subscriber_;
    rclcpp::Publisher<atlasmesh_vehicle_agent::msg::VehicleProfile>::SharedPtr profile_publisher_;
    
    // Timers
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::TimerBase::SharedPtr telemetry_timer_;
    rclcpp::TimerBase::SharedPtr health_timer_;
    
    // Configuration
    std::string vehicle_id_;
    std::string vehicle_type_;
    std::string profile_path_;
    double control_frequency_;
    double telemetry_frequency_;
    double health_check_frequency_;
    
    // Vehicle profile
    atlasmesh_vehicle_agent::msg::VehicleProfile vehicle_profile_;
    
    // State management
    std::atomic<bool> emergency_stop_active_{false};
    std::atomic<bool> autonomous_mode_active_{false};
    std::atomic<bool> system_healthy_{true};
    
    void declare_parameters()
    {
        // Vehicle identification
        this->declare_parameter("vehicle_id", "vehicle_001");
        this->declare_parameter("vehicle_type", "ugv_themis");
        this->declare_parameter("profile_path", "/opt/atlasmesh/profiles/default.yaml");
        
        // Control parameters
        this->declare_parameter("control_frequency", 50.0);  // 50 Hz control loop
        this->declare_parameter("telemetry_frequency", 10.0); // 10 Hz telemetry
        this->declare_parameter("health_check_frequency", 1.0); // 1 Hz health checks
        
        // Safety parameters
        this->declare_parameter("emergency_stop_timeout", 5.0); // seconds
        this->declare_parameter("watchdog_timeout", 2.0); // seconds
        this->declare_parameter("max_control_latency", 0.1); // seconds
        
        // Communication parameters
        this->declare_parameter("cloud_endpoint", "https://api.atlasmesh.com");
        this->declare_parameter("mqtt_broker", "mqtt.atlasmesh.com");
        this->declare_parameter("mqtt_port", 8883);
        this->declare_parameter("use_tls", true);
        
        // Policy parameters
        this->declare_parameter("policy_cache_size", 1000);
        this->declare_parameter("policy_evaluation_timeout", 10); // milliseconds
        
        // OTA parameters
        this->declare_parameter("ota_check_interval", 3600.0); // 1 hour
        this->declare_parameter("ota_download_timeout", 1800.0); // 30 minutes
        
        // Get parameter values
        vehicle_id_ = this->get_parameter("vehicle_id").as_string();
        vehicle_type_ = this->get_parameter("vehicle_type").as_string();
        profile_path_ = this->get_parameter("profile_path").as_string();
        control_frequency_ = this->get_parameter("control_frequency").as_double();
        telemetry_frequency_ = this->get_parameter("telemetry_frequency").as_double();
        health_check_frequency_ = this->get_parameter("health_check_frequency").as_double();
        
        RCLCPP_INFO(this->get_logger(), "Vehicle ID: %s, Type: %s", 
                   vehicle_id_.c_str(), vehicle_type_.c_str());
    }
    
    void load_vehicle_profile()
    {
        RCLCPP_INFO(this->get_logger(), "Loading vehicle profile from: %s", profile_path_.c_str());
        
        // TODO: Load profile from YAML file
        // For now, create a basic profile
        vehicle_profile_.header.stamp = this->now();
        vehicle_profile_.profile_id = vehicle_id_ + "_profile";
        vehicle_profile_.profile_version = "1.0.0";
        vehicle_profile_.vehicle_id = vehicle_id_;
        vehicle_profile_.vehicle_type = vehicle_type_;
        
        // Basic specifications (these would come from the profile file)
        vehicle_profile_.max_speed = 15.0; // m/s (54 km/h)
        vehicle_profile_.max_acceleration = 2.0; // m/s²
        vehicle_profile_.max_deceleration = 4.0; // m/s²
        vehicle_profile_.max_lateral_acceleration = 3.0; // m/s²
        vehicle_profile_.min_turning_radius = 5.0; // meters
        vehicle_profile_.autonomy_level = vehicle_profile_.AUTONOMY_LEVEL_4;
        
        RCLCPP_INFO(this->get_logger(), "Vehicle profile loaded successfully");
    }
    
    void initialize_components()
    {
        RCLCPP_INFO(this->get_logger(), "Initializing vehicle components...");
        
        // Initialize vehicle controller
        vehicle_controller_ = std::make_unique<VehicleController>(
            shared_from_this(), vehicle_profile_);
        
        // Initialize safety monitor (highest priority)
        safety_monitor_ = std::make_unique<SafetyMonitor>(
            shared_from_this(), vehicle_profile_);
        
        // Initialize health monitor
        health_monitor_ = std::make_unique<HealthMonitor>(
            shared_from_this(), vehicle_profile_);
        
        // Initialize policy client
        policy_client_ = std::make_unique<PolicyClient>(
            shared_from_this(), vehicle_id_);
        
        // Initialize cloud bridge
        cloud_bridge_ = std::make_unique<CloudBridgeClient>(
            shared_from_this(), vehicle_id_);
        
        // Initialize sensor manager
        sensor_manager_ = std::make_unique<SensorManager>(
            shared_from_this(), vehicle_profile_);
        
        // Initialize navigation manager
        navigation_manager_ = std::make_unique<NavigationManager>(
            shared_from_this(), vehicle_profile_);
        
        // Initialize OTA manager
        ota_manager_ = std::make_unique<OTAManager>(
            shared_from_this(), vehicle_id_);
        
        RCLCPP_INFO(this->get_logger(), "All components initialized");
    }
    
    void setup_communication()
    {
        RCLCPP_INFO(this->get_logger(), "Setting up communication interfaces...");
        
        // Publishers
        state_publisher_ = this->create_publisher<atlasmesh_vehicle_agent::msg::VehicleState>(
            "vehicle_state", rclcpp::QoS(10).reliable());
        
        profile_publisher_ = this->create_publisher<atlasmesh_vehicle_agent::msg::VehicleProfile>(
            "vehicle_profile", rclcpp::QoS(1).reliable().transient_local());
        
        // Subscribers
        command_subscriber_ = this->create_subscription<atlasmesh_vehicle_agent::msg::VehicleCommand>(
            "vehicle_command", rclcpp::QoS(10).reliable(),
            std::bind(&VehicleAgentNode::command_callback, this, std::placeholders::_1));
        
        // Publish initial vehicle profile
        profile_publisher_->publish(vehicle_profile_);
        
        RCLCPP_INFO(this->get_logger(), "Communication interfaces ready");
    }
    
    void start_control_loop()
    {
        RCLCPP_INFO(this->get_logger(), "Starting control loops...");
        
        // Main control loop (high frequency)
        auto control_period = std::chrono::duration<double>(1.0 / control_frequency_);
        control_timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(control_period),
            std::bind(&VehicleAgentNode::control_loop_callback, this));
        
        // Telemetry publishing loop
        auto telemetry_period = std::chrono::duration<double>(1.0 / telemetry_frequency_);
        telemetry_timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(telemetry_period),
            std::bind(&VehicleAgentNode::telemetry_callback, this));
        
        // Health monitoring loop
        auto health_period = std::chrono::duration<double>(1.0 / health_check_frequency_);
        health_timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(health_period),
            std::bind(&VehicleAgentNode::health_callback, this));
        
        RCLCPP_INFO(this->get_logger(), "Control loops started - Control: %.1f Hz, Telemetry: %.1f Hz", 
                   control_frequency_, telemetry_frequency_);
    }
    
    void control_loop_callback()
    {
        // This is the main control loop running at 50Hz
        auto start_time = std::chrono::high_resolution_clock::now();
        
        try {
            // 1. Safety checks (highest priority)
            if (!safety_monitor_->is_safe()) {
                if (!emergency_stop_active_) {
                    RCLCPP_ERROR(this->get_logger(), "Safety violation detected - executing emergency stop");
                    execute_emergency_stop();
                }
                return; // Skip control updates during emergency stop
            }
            
            // 2. Update sensor data
            sensor_manager_->update();
            
            // 3. Update navigation if in autonomous mode
            if (autonomous_mode_active_) {
                navigation_manager_->update();
            }
            
            // 4. Execute vehicle control
            vehicle_controller_->update();
            
            // 5. Monitor control loop performance
            auto end_time = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
            
            // Log warning if control loop is too slow
            if (duration.count() > 15000) { // 15ms threshold for 50Hz loop
                RCLCPP_WARN(this->get_logger(), "Control loop slow: %ld μs", duration.count());
            }
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Control loop exception: %s", e.what());
            execute_emergency_stop();
        }
    }
    
    void telemetry_callback()
    {
        // Publish vehicle state telemetry
        auto state_msg = create_vehicle_state_message();
        state_publisher_->publish(state_msg);
        
        // Send telemetry to cloud
        cloud_bridge_->publish_telemetry(state_msg);
    }
    
    void health_callback()
    {
        // Update system health status
        system_healthy_ = health_monitor_->check_system_health();
        
        if (!system_healthy_) {
            RCLCPP_WARN(this->get_logger(), "System health degraded");
        }
        
        // Check for OTA updates periodically
        ota_manager_->check_for_updates();
    }
    
    void command_callback(const atlasmesh_vehicle_agent::msg::VehicleCommand::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received command: %s (type: %d, priority: %d)", 
                   msg->command_id.c_str(), msg->command_type, msg->priority);
        
        try {
            // Validate command authorization
            if (!validate_command_authorization(*msg)) {
                RCLCPP_ERROR(this->get_logger(), "Command authorization failed: %s", msg->command_id.c_str());
                return;
            }
            
            // Process command based on type
            switch (msg->command_type) {
                case msg->COMMAND_TYPE_MOTION:
                    handle_motion_command(*msg);
                    break;
                    
                case msg->COMMAND_TYPE_SAFETY:
                    handle_safety_command(*msg);
                    break;
                    
                case msg->COMMAND_TYPE_SYSTEM:
                    handle_system_command(*msg);
                    break;
                    
                case msg->COMMAND_TYPE_TRIP:
                    handle_trip_command(*msg);
                    break;
                    
                case msg->COMMAND_TYPE_OTA:
                    handle_ota_command(*msg);
                    break;
                    
                default:
                    RCLCPP_WARN(this->get_logger(), "Unknown command type: %d", msg->command_type);
                    break;
            }
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Command processing exception: %s", e.what());
        }
    }
    
    bool validate_command_authorization(const atlasmesh_vehicle_agent::msg::VehicleCommand& cmd)
    {
        // TODO: Implement proper command authorization
        // For now, basic validation
        return !cmd.command_id.empty() && !cmd.issuer_id.empty();
    }
    
    void handle_motion_command(const atlasmesh_vehicle_agent::msg::VehicleCommand& cmd)
    {
        if (emergency_stop_active_) {
            RCLCPP_WARN(this->get_logger(), "Motion command ignored - emergency stop active");
            return;
        }
        
        vehicle_controller_->execute_motion_command(cmd);
    }
    
    void handle_safety_command(const atlasmesh_vehicle_agent::msg::VehicleCommand& cmd)
    {
        if (cmd.execute_emergency_stop) {
            RCLCPP_INFO(this->get_logger(), "Emergency stop commanded");
            execute_emergency_stop();
        }
        
        if (cmd.enable_autonomous_mode && !autonomous_mode_active_) {
            RCLCPP_INFO(this->get_logger(), "Enabling autonomous mode");
            autonomous_mode_active_ = true;
        }
        
        if (cmd.disable_autonomous_mode && autonomous_mode_active_) {
            RCLCPP_INFO(this->get_logger(), "Disabling autonomous mode");
            autonomous_mode_active_ = false;
        }
    }
    
    void handle_system_command(const atlasmesh_vehicle_agent::msg::VehicleCommand& cmd)
    {
        switch (cmd.system_command) {
            case cmd.SYSTEM_CMD_SHUTDOWN:
                RCLCPP_INFO(this->get_logger(), "System shutdown commanded");
                rclcpp::shutdown();
                break;
                
            case cmd.SYSTEM_CMD_RESTART:
                RCLCPP_INFO(this->get_logger(), "System restart commanded");
                // TODO: Implement graceful restart
                break;
                
            case cmd.SYSTEM_CMD_CALIBRATE:
                RCLCPP_INFO(this->get_logger(), "Sensor calibration commanded");
                sensor_manager_->calibrate_sensors();
                break;
                
            case cmd.SYSTEM_CMD_DIAGNOSTICS:
                RCLCPP_INFO(this->get_logger(), "Diagnostics commanded");
                health_monitor_->run_full_diagnostics();
                break;
                
            default:
                RCLCPP_WARN(this->get_logger(), "Unknown system command: %d", cmd.system_command);
                break;
        }
    }
    
    void handle_trip_command(const atlasmesh_vehicle_agent::msg::VehicleCommand& cmd)
    {
        navigation_manager_->handle_trip_command(cmd);
    }
    
    void handle_ota_command(const atlasmesh_vehicle_agent::msg::VehicleCommand& cmd)
    {
        ota_manager_->handle_ota_command(cmd);
    }
    
    void execute_emergency_stop()
    {
        emergency_stop_active_ = true;
        autonomous_mode_active_ = false;
        
        // Execute immediate stop
        vehicle_controller_->execute_emergency_stop();
        
        // Notify safety monitor
        safety_monitor_->emergency_stop_activated();
        
        // Notify cloud
        cloud_bridge_->report_emergency_stop();
        
        RCLCPP_ERROR(this->get_logger(), "EMERGENCY STOP ACTIVATED");
    }
    
    atlasmesh_vehicle_agent::msg::VehicleState create_vehicle_state_message()
    {
        atlasmesh_vehicle_agent::msg::VehicleState state;
        
        state.header.stamp = this->now();
        state.header.frame_id = "base_link";
        state.vehicle_id = vehicle_id_;
        state.vehicle_type = vehicle_type_;
        state.profile_version = vehicle_profile_.profile_version;
        
        // Get current pose and motion from vehicle controller
        vehicle_controller_->get_current_state(state);
        
        // Get sensor health from sensor manager
        sensor_manager_->get_sensor_health(state.sensor_health);
        
        // Get system status
        state.emergency_stop_active = emergency_stop_active_;
        state.autonomous_mode_active = autonomous_mode_active_;
        state.safety_systems_ok = safety_monitor_->is_safe();
        
        // Set operational mode
        if (emergency_stop_active_) {
            state.operational_mode = state.OPERATIONAL_MODE_EMERGENCY;
        } else if (autonomous_mode_active_) {
            state.operational_mode = state.OPERATIONAL_MODE_AUTONOMOUS;
        } else {
            state.operational_mode = state.OPERATIONAL_MODE_MANUAL;
        }
        
        // Performance metrics
        state.control_loop_frequency = control_frequency_;
        
        return state;
    }
    
    void shutdown_components()
    {
        // Graceful shutdown of all components
        if (vehicle_controller_) {
            vehicle_controller_->shutdown();
        }
        
        if (cloud_bridge_) {
            cloud_bridge_->disconnect();
        }
        
        RCLCPP_INFO(this->get_logger(), "Vehicle Agent shutdown complete");
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<VehicleAgentNode>();
        
        // Use multi-threaded executor for better performance
        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(node);
        
        RCLCPP_INFO(node->get_logger(), "Vehicle Agent running...");
        executor.spin();
        
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("vehicle_agent"), "Fatal error: %s", e.what());
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}
