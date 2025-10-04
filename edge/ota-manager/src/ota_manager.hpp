/**
 * @file ota_manager.hpp
 * @brief OTA (Over-The-Air) Update Manager for AtlasMesh Fleet OS
 * 
 * Handles secure, reliable over-the-air updates:
 * - Cryptographic signature verification
 * - A/B partition updates with rollback
 * - Staged deployment and health monitoring
 * - SBOM (Software Bill of Materials) tracking
 * - Delta updates for bandwidth efficiency
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <string>
#include <vector>
#include <atomic>
#include <mutex>
#include <thread>
#include <chrono>
#include <functional>

#include "atlasmesh_vehicle_agent/msg/ota_update.hpp"
#include "atlasmesh_vehicle_agent/msg/vehicle_command.hpp"

/**
 * @enum UpdateStatus
 * @brief OTA update status states
 */
enum class UpdateStatus {
    IDLE,
    CHECKING,
    AVAILABLE,
    DOWNLOADING,
    VERIFYING,
    INSTALLING,
    INSTALLED,
    REBOOTING,
    FAILED,
    ROLLED_BACK
};

/**
 * @enum UpdateType
 * @brief Types of OTA updates
 */
enum class UpdateType {
    FULL_SYSTEM,      // Complete system image
    APPLICATION,      // Application layer only
    CONFIGURATION,    // Configuration files
    SECURITY_PATCH,   // Security-only update
    DELTA            // Delta/incremental update
};

/**
 * @enum UpdatePriority
 * @brief Update priority levels
 */
enum class UpdatePriority {
    LOW,
    NORMAL,
    HIGH,
    CRITICAL,         // Security patches
    EMERGENCY         // Emergency fixes
};

/**
 * @struct UpdatePackage
 * @brief OTA update package information
 */
struct UpdatePackage {
    std::string package_id;
    std::string version;
    std::string previous_version;
    UpdateType type;
    UpdatePriority priority;
    size_t size_bytes;
    std::string download_url;
    std::string signature;
    std::string checksum_sha256;
    std::string release_notes;
    std::chrono::system_clock::time_point release_date;
    std::chrono::system_clock::time_point expiry_date;
    std::vector<std::string> dependencies;
    std::vector<std::string> conflicts;
    bool requires_reboot;
    bool is_rollback_safe;
    std::string metadata_json;
    
    UpdatePackage() : type(UpdateType::APPLICATION), priority(UpdatePriority::NORMAL),
                     size_bytes(0), requires_reboot(false), is_rollback_safe(true) {}
};

/**
 * @struct UpdateProgress
 * @brief Update progress information
 */
struct UpdateProgress {
    UpdateStatus status;
    double progress_percent;
    std::string current_step;
    std::string status_message;
    std::chrono::steady_clock::time_point start_time;
    std::chrono::steady_clock::time_point estimated_completion;
    size_t bytes_downloaded;
    size_t total_bytes;
    double download_speed_mbps;
    
    UpdateProgress() : status(UpdateStatus::IDLE), progress_percent(0.0),
                      bytes_downloaded(0), total_bytes(0), download_speed_mbps(0.0) {}
};

/**
 * @class OTAManager
 * @brief Main OTA update manager
 */
class OTAManager
{
public:
    /**
     * @brief Constructor
     * @param node ROS2 node handle
     * @param vehicle_id Unique vehicle identifier
     */
    OTAManager(std::shared_ptr<rclcpp::Node> node, const std::string& vehicle_id);
    
    /**
     * @brief Destructor
     */
    ~OTAManager();
    
    /**
     * @brief Initialize OTA manager
     * @return True if initialization successful
     */
    bool initialize();
    
    /**
     * @brief Check for available updates
     * @param force_check Force check even if recently checked
     * @return True if check initiated successfully
     */
    bool check_for_updates(bool force_check = false);
    
    /**
     * @brief Download available update
     * @param package_id Package ID to download
     * @return True if download initiated successfully
     */
    bool download_update(const std::string& package_id);
    
    /**
     * @brief Install downloaded update
     * @param package_id Package ID to install
     * @return True if installation initiated successfully
     */
    bool install_update(const std::string& package_id);
    
    /**
     * @brief Rollback to previous version
     * @return True if rollback initiated successfully
     */
    bool rollback_update();
    
    /**
     * @brief Handle OTA command from cloud
     * @param cmd OTA command
     */
    void handle_ota_command(const atlasmesh_vehicle_agent::msg::VehicleCommand& cmd);
    
    /**
     * @brief Get current update status
     * @return Current update progress
     */
    UpdateProgress get_update_progress() const;
    
    /**
     * @brief Get available updates
     * @return List of available update packages
     */
    std::vector<UpdatePackage> get_available_updates() const;
    
    /**
     * @brief Get current system version
     * @return Current version string
     */
    std::string get_current_version() const;
    
    /**
     * @brief Check if update is in progress
     * @return True if update is active
     */
    bool is_update_in_progress() const;
    
    /**
     * @brief Set update callback for progress notifications
     * @param callback Function to call on progress updates
     */
    void set_progress_callback(std::function<void(const UpdateProgress&)> callback);
    
    /**
     * @brief Enable/disable automatic updates
     * @param enabled True to enable automatic updates
     */
    void set_auto_update_enabled(bool enabled) { auto_update_enabled_ = enabled; }
    
    /**
     * @brief Set update window (time when updates are allowed)
     * @param start_hour Start hour (0-23)
     * @param end_hour End hour (0-23)
     */
    void set_update_window(int start_hour, int end_hour);

private:
    // ROS2 node handle
    std::shared_ptr<rclcpp::Node> node_;
    
    // Vehicle identification
    std::string vehicle_id_;
    
    // Configuration
    std::string update_server_url_;
    std::string download_directory_;
    std::string install_directory_;
    std::string backup_directory_;
    std::string public_key_path_;
    std::atomic<bool> auto_update_enabled_{false};
    int update_window_start_hour_{2};  // 2 AM
    int update_window_end_hour_{6};    // 6 AM
    
    // Current state
    std::atomic<UpdateStatus> current_status_{UpdateStatus::IDLE};
    UpdateProgress current_progress_;
    mutable std::mutex progress_mutex_;
    
    // Available updates
    std::vector<UpdatePackage> available_updates_;
    mutable std::mutex updates_mutex_;
    
    // Background processing
    std::unique_ptr<std::thread> update_thread_;
    std::atomic<bool> running_{false};
    
    // Callbacks
    std::function<void(const UpdateProgress&)> progress_callback_;
    
    // Timing
    std::chrono::steady_clock::time_point last_check_time_;
    std::chrono::seconds check_interval_{3600};  // 1 hour
    
    // Private methods
    void initialize_configuration();
    void start_background_thread();
    void stop_background_thread();
    
    void update_manager_loop();
    void periodic_update_check();
    bool is_in_update_window();
    
    // Update discovery
    bool fetch_available_updates();
    std::vector<UpdatePackage> parse_update_manifest(const std::string& manifest_json);
    bool is_update_applicable(const UpdatePackage& package);
    
    // Download management
    bool download_package(const UpdatePackage& package);
    bool verify_download(const UpdatePackage& package, const std::string& file_path);
    bool verify_signature(const std::string& file_path, const std::string& signature);
    bool verify_checksum(const std::string& file_path, const std::string& expected_checksum);
    
    // Installation
    bool install_package(const UpdatePackage& package);
    bool backup_current_system();
    bool apply_update(const UpdatePackage& package);
    bool verify_installation(const UpdatePackage& package);
    bool update_system_version(const std::string& new_version);
    
    // Rollback
    bool create_rollback_point();
    bool restore_from_backup();
    bool validate_rollback();
    
    // Health monitoring
    bool perform_post_update_health_check();
    bool monitor_system_stability();
    void schedule_health_monitoring();
    
    // Progress reporting
    void update_progress(UpdateStatus status, double percent, const std::string& message);
    void notify_progress_callback();
    
    // Utilities
    std::string generate_download_path(const UpdatePackage& package);
    std::string calculate_file_checksum(const std::string& file_path);
    size_t get_file_size(const std::string& file_path);
    bool create_directory(const std::string& path);
    bool remove_file(const std::string& file_path);
    bool move_file(const std::string& source, const std::string& destination);
    
    // Network operations
    bool download_file(const std::string& url, const std::string& destination,
                      std::function<void(size_t, size_t)> progress_callback = nullptr);
    std::string http_get(const std::string& url);
    
    // Cryptographic operations
    bool load_public_key();
    bool verify_rsa_signature(const std::string& data, const std::string& signature);
    
    // System operations
    bool is_system_idle();
    bool request_system_reboot();
    std::string get_system_version();
    bool set_boot_partition(const std::string& partition);
    
    // SBOM management
    bool update_sbom(const UpdatePackage& package);
    std::string generate_sbom_entry(const UpdatePackage& package);
    bool validate_dependencies(const UpdatePackage& package);
};

/**
 * @class UpdateDownloader
 * @brief Handles update package downloads
 */
class UpdateDownloader
{
public:
    UpdateDownloader();
    ~UpdateDownloader();
    
    struct DownloadProgress {
        size_t bytes_downloaded;
        size_t total_bytes;
        double speed_mbps;
        std::chrono::steady_clock::time_point start_time;
        std::chrono::steady_clock::time_point estimated_completion;
    };
    
    bool download(const std::string& url, const std::string& destination,
                 std::function<void(const DownloadProgress&)> progress_callback = nullptr);
    
    void cancel_download();
    bool is_downloading() const { return downloading_; }
    
    void set_bandwidth_limit(double mbps) { bandwidth_limit_mbps_ = mbps; }
    void set_retry_count(int retries) { max_retries_ = retries; }

private:
    std::atomic<bool> downloading_{false};
    std::atomic<bool> cancel_requested_{false};
    double bandwidth_limit_mbps_{0.0};  // 0 = unlimited
    int max_retries_{3};
    
    bool download_with_resume(const std::string& url, const std::string& destination,
                             std::function<void(const DownloadProgress&)> progress_callback);
    size_t get_remote_file_size(const std::string& url);
    bool supports_resume(const std::string& url);
};

/**
 * @class UpdateVerifier
 * @brief Handles cryptographic verification of updates
 */
class UpdateVerifier
{
public:
    UpdateVerifier(const std::string& public_key_path);
    ~UpdateVerifier();
    
    bool verify_package(const std::string& package_path, const std::string& signature);
    bool verify_checksum(const std::string& file_path, const std::string& expected_checksum);
    bool verify_certificate_chain(const std::string& certificate_path);
    
    std::string calculate_checksum(const std::string& file_path);

private:
    std::string public_key_path_;
    void* rsa_key_;  // Placeholder for RSA key structure
    
    bool load_public_key();
    void cleanup_crypto();
};

/**
 * @class RollbackManager
 * @brief Manages system rollback capabilities
 */
class RollbackManager
{
public:
    RollbackManager(const std::string& backup_directory);
    ~RollbackManager();
    
    bool create_snapshot(const std::string& snapshot_name);
    bool restore_snapshot(const std::string& snapshot_name);
    bool delete_snapshot(const std::string& snapshot_name);
    
    std::vector<std::string> list_snapshots();
    bool validate_snapshot(const std::string& snapshot_name);
    
    void set_max_snapshots(int max_count) { max_snapshots_ = max_count; }
    void cleanup_old_snapshots();

private:
    std::string backup_directory_;
    int max_snapshots_{5};
    
    bool create_filesystem_snapshot(const std::string& source, const std::string& destination);
    bool restore_filesystem_snapshot(const std::string& source, const std::string& destination);
    size_t get_snapshot_size(const std::string& snapshot_name);
    std::chrono::system_clock::time_point get_snapshot_timestamp(const std::string& snapshot_name);
};

/**
 * @class HealthMonitor
 * @brief Monitors system health after updates
 */
class HealthMonitor
{
public:
    HealthMonitor();
    ~HealthMonitor();
    
    struct HealthMetrics {
        bool system_stable;
        double cpu_usage_percent;
        double memory_usage_percent;
        double disk_usage_percent;
        int process_count;
        std::vector<std::string> failed_services;
        std::vector<std::string> warnings;
        std::chrono::steady_clock::time_point timestamp;
    };
    
    bool start_monitoring(std::chrono::seconds duration);
    void stop_monitoring();
    
    HealthMetrics get_current_metrics();
    bool is_system_stable();
    
    void set_stability_threshold(double cpu_threshold, double memory_threshold);

private:
    std::atomic<bool> monitoring_{false};
    std::unique_ptr<std::thread> monitor_thread_;
    HealthMetrics current_metrics_;
    mutable std::mutex metrics_mutex_;
    
    double cpu_threshold_{80.0};      // 80% CPU usage threshold
    double memory_threshold_{90.0};   // 90% memory usage threshold
    
    void monitoring_loop(std::chrono::seconds duration);
    void collect_metrics();
    bool check_critical_services();
    double get_cpu_usage();
    double get_memory_usage();
    double get_disk_usage();
    int get_process_count();
};
