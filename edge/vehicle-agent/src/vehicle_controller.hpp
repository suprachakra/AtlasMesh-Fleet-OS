/**
 * @file vehicle_controller.hpp
 * @brief Vehicle Controller for AtlasMesh Fleet OS
 * 
 * Handles low-level vehicle control including:
 * - Drive-by-wire interface
 * - Motion control and trajectory following
 * - Emergency stop execution
 * - Vehicle state monitoring
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>

#include "atlasmesh_vehicle_agent/msg/vehicle_state.hpp"
#include "atlasmesh_vehicle_agent/msg/vehicle_command.hpp"
#include "atlasmesh_vehicle_agent/msg/vehicle_profile.hpp"

#include <memory>
#include <atomic>
#include <mutex>
#include <chrono>

/**
 * @class VehicleController
 * @brief Main vehicle control interface
 */
class VehicleController
{
public:
    /**
     * @brief Constructor
     * @param node ROS2 node handle
     * @param profile Vehicle profile with capabilities and limits
     */
    VehicleController(std::shared_ptr<rclcpp::Node> node, 
                     const atlasmesh_vehicle_agent::msg::VehicleProfile& profile);
    
    /**
     * @brief Destructor
     */
    ~VehicleController();
    
    /**
     * @brief Update control loop (called at 50Hz)
     */
    void update();
    
    /**
     * @brief Execute motion command
     * @param cmd Motion command to execute
     */
    void execute_motion_command(const atlasmesh_vehicle_agent::msg::VehicleCommand& cmd);
    
    /**
     * @brief Execute emergency stop
     */
    void execute_emergency_stop();
    
    /**
     * @brief Get current vehicle state
     * @param state Output vehicle state message
     */
    void get_current_state(atlasmesh_vehicle_agent::msg::VehicleState& state);
    
    /**
     * @brief Check if vehicle is in motion
     * @return True if vehicle is moving
     */
    bool is_in_motion() const;
    
    /**
     * @brief Get current speed
     * @return Current speed in m/s
     */
    double get_current_speed() const;
    
    /**
     * @brief Shutdown controller
     */
    void shutdown();

private:
    // ROS2 node handle
    std::shared_ptr<rclcpp::Node> node_;
    
    // Vehicle profile
    atlasmesh_vehicle_agent::msg::VehicleProfile profile_;
    
    // Publishers for vehicle control
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr steering_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr throttle_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr brake_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    
    // Subscribers for vehicle feedback
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    
    // Control state
    std::atomic<bool> emergency_stop_active_{false};
    std::atomic<bool> control_enabled_{false};
    std::atomic<double> current_speed_{0.0};
    std::atomic<double> target_speed_{0.0};
    std::atomic<double> target_steering_{0.0};
    
    // Thread safety
    mutable std::mutex state_mutex_;
    mutable std::mutex command_mutex_;
    
    // Current vehicle state
    geometry_msgs::msg::PoseWithCovariance current_pose_;
    geometry_msgs::msg::TwistWithCovariance current_twist_;
    sensor_msgs::msg::JointState current_joint_state_;
    
    // Control parameters
    double max_acceleration_;
    double max_deceleration_;
    double max_steering_rate_;
    double control_timeout_;
    
    // PID controllers
    struct PIDController {
        double kp, ki, kd;
        double integral, previous_error;
        double output_min, output_max;
        std::chrono::steady_clock::time_point last_time;
        
        PIDController(double p, double i, double d, double min_out, double max_out)
            : kp(p), ki(i), kd(d), integral(0.0), previous_error(0.0), 
              output_min(min_out), output_max(max_out) {}
    };
    
    std::unique_ptr<PIDController> speed_controller_;
    std::unique_ptr<PIDController> steering_controller_;
    
    // Timing
    std::chrono::steady_clock::time_point last_command_time_;
    std::chrono::steady_clock::time_point last_update_time_;
    
    // Private methods
    void initialize_publishers();
    void initialize_subscribers();
    void initialize_controllers();
    
    void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
    
    void update_speed_control();
    void update_steering_control();
    void update_safety_limits();
    
    double compute_pid(PIDController& controller, double setpoint, double measured_value);
    double clamp(double value, double min_val, double max_val);
    
    bool validate_motion_command(const atlasmesh_vehicle_agent::msg::VehicleCommand& cmd);
    void apply_safety_limits(double& speed, double& steering);
    
    void publish_control_commands();
    void stop_vehicle();
    
    // Emergency stop implementation
    void execute_immediate_stop();
    void engage_parking_brake();
    
    // Diagnostics
    void log_control_performance();
};

/**
 * @class DriveByWireInterface
 * @brief Low-level drive-by-wire interface
 */
class DriveByWireInterface
{
public:
    DriveByWireInterface(std::shared_ptr<rclcpp::Node> node);
    ~DriveByWireInterface();
    
    // Control outputs
    void set_throttle(double throttle_percent);  // 0.0 to 1.0
    void set_brake(double brake_percent);        // 0.0 to 1.0
    void set_steering(double steering_angle);    // radians
    void set_gear(int gear);                     // -1=reverse, 0=neutral, 1+=forward
    
    // Status inputs
    double get_wheel_speed(int wheel_index);     // m/s
    double get_steering_angle();                 // radians
    double get_brake_pressure();                 // bar
    double get_throttle_position();              // 0.0 to 1.0
    int get_gear_position();                     // current gear
    
    // Safety
    void engage_emergency_brake();
    void disengage_emergency_brake();
    bool is_emergency_brake_engaged();
    
    // System status
    bool is_drive_by_wire_enabled();
    bool has_steering_control();
    bool has_throttle_control();
    bool has_brake_control();
    
    // Diagnostics
    std::vector<std::string> get_fault_codes();
    bool run_self_test();

private:
    std::shared_ptr<rclcpp::Node> node_;
    
    // Hardware interface (would be implemented for specific vehicle)
    // This is a placeholder for the actual hardware interface
    void initialize_hardware();
    void shutdown_hardware();
    
    // Mock hardware state for simulation
    std::atomic<double> hw_throttle_{0.0};
    std::atomic<double> hw_brake_{0.0};
    std::atomic<double> hw_steering_{0.0};
    std::atomic<int> hw_gear_{0};
    std::atomic<bool> hw_emergency_brake_{false};
    std::atomic<bool> hw_dbw_enabled_{true};
    
    mutable std::mutex hw_mutex_;
};

/**
 * @class SafetyLimiter
 * @brief Enforces vehicle safety limits
 */
class SafetyLimiter
{
public:
    SafetyLimiter(const atlasmesh_vehicle_agent::msg::VehicleProfile& profile);
    
    /**
     * @brief Apply safety limits to control commands
     * @param speed Target speed (will be modified)
     * @param steering Target steering (will be modified)
     * @param current_speed Current vehicle speed
     */
    void apply_limits(double& speed, double& steering, double current_speed);
    
    /**
     * @brief Check if emergency stop is required
     * @param speed Current speed
     * @param acceleration Current acceleration
     * @return True if emergency stop should be triggered
     */
    bool requires_emergency_stop(double speed, double acceleration);
    
    /**
     * @brief Update environmental limits (e.g., for weather conditions)
     * @param max_speed_factor Speed reduction factor (0.0 to 1.0)
     * @param max_acceleration_factor Acceleration reduction factor (0.0 to 1.0)
     */
    void update_environmental_limits(double max_speed_factor, double max_acceleration_factor);

private:
    // Vehicle limits from profile
    double max_speed_;
    double max_acceleration_;
    double max_deceleration_;
    double max_lateral_acceleration_;
    double min_turning_radius_;
    
    // Dynamic limits (can be reduced by environmental conditions)
    double current_max_speed_;
    double current_max_acceleration_;
    
    // Safety thresholds
    double emergency_deceleration_threshold_;
    double rollover_prevention_threshold_;
    
    double calculate_max_safe_speed_for_curvature(double steering_angle);
    double calculate_lateral_acceleration(double speed, double steering_angle);
};
