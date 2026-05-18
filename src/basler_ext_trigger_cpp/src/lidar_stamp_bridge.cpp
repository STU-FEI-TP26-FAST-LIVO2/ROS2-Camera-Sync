#include <rclcpp/rclcpp.hpp>
#include <cstdint>
#include <chrono>
#include <vector>
#include <cmath>
#include <algorithm>
#include <iomanip>

#include "basler_ext_trigger_cpp/lidar_stamp_reader.hpp"

namespace basler_ext_trigger_cpp
{

class LidarStampBridge : public rclcpp::Node
{
public:
  LidarStampBridge()
  : Node("lidar_stamp_bridge_node")
  {
    declare_parameter<std::string>("lidar_mmap_path", "/dev/shm/lidar_stamp.bin");
    declare_parameter<int>("monitor_period_ms", 1000);
    declare_parameter<int>("log_level", 1);  // 0=ERROR, 1=WARN, 2=INFO, 3=DEBUG

    lidar_mmap_path_ = get_parameter("lidar_mmap_path").as_string();
    monitor_period_ms_ = get_parameter("monitor_period_ms").as_int();
    log_level_ = get_parameter("log_level").as_int();

    RCLCPP_INFO(
      get_logger(),
      "LiDAR Stamp Bridge initialized: path=%s poll_ms=%d log_level=%d",
      lidar_mmap_path_.c_str(),
      monitor_period_ms_,
      log_level_);

    // Initialize reader
    try_connect_reader();

    // Start monitoring timer
    monitor_timer_ = create_wall_timer(
      std::chrono::milliseconds(monitor_period_ms_),
      [this]() { monitor_lidar_activity(); });
  }

  ~LidarStampBridge() override
  {
    if (monitor_timer_) {
      monitor_timer_->cancel();
    }
  }

private:
  std::string lidar_mmap_path_;
  int monitor_period_ms_;
  int log_level_;

  std::unique_ptr<LidarStampReader> reader_;
  rclcpp::TimerBase::SharedPtr monitor_timer_;

  // Monitoring state
  uint64_t last_write_index_{0};
  uint64_t last_seq_{0};
  int64_t last_record_time_ns_{0};
  std::vector<int64_t> interval_history_;  // Last 10 intervals
  static constexpr size_t HISTORY_SIZE = 10;

  void try_connect_reader()
  {
    try {
      reader_ = std::make_unique<LidarStampReader>(lidar_mmap_path_);
      RCLCPP_INFO(get_logger(), "✓ LiDAR mmap reader connected: %s", lidar_mmap_path_.c_str());
    } catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(), "✗ Failed to connect LiDAR mmap reader: %s", e.what());
    }
  }

  void monitor_lidar_activity()
  {
    if (!reader_) {
      try_connect_reader();
      if (!reader_) {
        return;  // Still not connected
      }
    }

    const auto records = reader_->recent_records();

    if (records.empty()) {
      if (log_level_ >= 1) {
        RCLCPP_WARN(get_logger(), "[MONITOR] Buffer EMPTY");
      }
      return;
    }

    const auto & latest_record = records.back();

    // Check if there's new data
    static uint64_t last_logged_seq = 0;
    if (latest_record.lidar_seq == last_logged_seq) {
      // No new data since last check
      return;
    }

    // Calculate interval between records
    int64_t interval_ns = 0;
    if (last_record_time_ns_ > 0) {
      interval_ns = static_cast<int64_t>(latest_record.lidar_host_receive_ns) - last_record_time_ns_;
    }
    last_record_time_ns_ = static_cast<int64_t>(latest_record.lidar_host_receive_ns);

    // Track interval history
    if (interval_ns > 0) {
      interval_history_.push_back(interval_ns);
      if (interval_history_.size() > HISTORY_SIZE) {
        interval_history_.erase(interval_history_.begin());
      }
    }

    // Calculate statistics
    double avg_interval_ms = 0;
    double min_interval_ms = 0;
    double max_interval_ms = 0;

    if (!interval_history_.empty()) {
      double sum = 0;
      for (int64_t iv : interval_history_) {
        sum += iv / 1e6;  // Convert to ms
      }
      avg_interval_ms = sum / interval_history_.size();

      min_interval_ms = *std::min_element(interval_history_.begin(), interval_history_.end()) / 1e6;
      max_interval_ms = *std::max_element(interval_history_.begin(), interval_history_.end()) / 1e6;
    }

    // Check for sequence gaps
    int seq_gap = 0;
    if (last_seq_ > 0) {
      seq_gap = static_cast<int>(latest_record.lidar_seq - last_seq_ - 1);
    }
    last_seq_ = latest_record.lidar_seq;

    // Detailed logging
    if (log_level_ >= 2) {
      std::ostringstream oss;
      oss << std::fixed << std::setprecision(1);
      oss << "[MONITOR-LiDAR] seq=" << latest_record.lidar_seq
          << " interval=" << (interval_ns / 1e6) << "ms"
          << " avg=" << avg_interval_ms << "ms"
          << " min=" << min_interval_ms << "ms"
          << " max=" << max_interval_ms << "ms"
          << " total_records=" << records.size();

      if (seq_gap > 0) {
        oss << " ⚠ GAP=" << seq_gap << " missing sequences";
      }

      if (log_level_ >= 2) {
        oss << "\n  Latest: seq=" << latest_record.lidar_seq
            << " receive_ts=" << latest_record.lidar_host_receive_ns
            << " ros_ts=" << latest_record.lidar_ros_header_ns;
      }

      RCLCPP_INFO(get_logger(), oss.str().c_str());
    }

    // Warn about anomalies
    if (seq_gap > 0 && log_level_ >= 1) {
      RCLCPP_WARN(
        get_logger(),
        "⚠ [MONITOR] Skipped sequences detected: gap=%d before seq=%lu",
        seq_gap, latest_record.lidar_seq);
    }

    if (interval_ns > 120 * 1000000 && last_record_time_ns_ > 0) {  // > 120ms
      RCLCPP_WARN(
        get_logger(),
        "⚠ [MONITOR] Long interval detected: %.1f ms (expected ~100ms for 10Hz)",
        interval_ns / 1e6);
    }

    last_logged_seq = latest_record.lidar_seq;
  }
};

}  // namespace basler_ext_trigger_cpp

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<basler_ext_trigger_cpp::LidarStampBridge>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
