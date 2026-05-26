#pragma once

#include <pylon/BaslerUniversalInstantCamera.h>
#include <pylon/PylonIncludes.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <atomic>
#include <cstdint>
#include <memory>
#include <set>
#include <string>
#include <thread>

#include "basler_ext_trigger_cpp/lidar_stamp_reader.hpp"

namespace basler_ext_trigger_cpp
{

struct MatchDecision
{
  uint64_t lidar_ros_header_ns{0};
  uint64_t lidar_seq{0};
  int64_t match_dt_ns{0};
  int num_candidates{0};
  uint64_t best_candidate_seq{0};
  int64_t best_candidate_dt_ns{0};
  std::string match_quality{"UNMATCHED"};
  std::string match_reason{"NO_READER"};

  bool accepted() const;
};

class BaslerExtTriggerNode : public rclcpp::Node
{
public:
  BaslerExtTriggerNode();
  ~BaslerExtTriggerNode() override;

private:
  static rclcpp::QoS build_qos(
    bool sensor_data_qos, int queue_depth, const std::string & reliability);

  std::string classify_match_quality(int64_t dt_ns, uint64_t lidar_seq) const;
  void ensure_lidar_reader();
  MatchDecision find_best_lidar_match(uint64_t host_ts_ns);
  Pylon::CDeviceInfo select_device();
  void open_camera();

  template<typename ParamT>
  void safe_set(ParamT & param, const std::string & value, const char * name);

  void safe_set_bool(GenApi::IBoolean & param, bool value, const char * name);
  void safe_set_float(GenApi::IFloat & param, double value, const char * name);
  void safe_execute(GenApi::ICommand & cmd, const char * name);
  void configure_camera_runtime_only();
  void enable_chunks();
  void start_grabbing();
  void grab_loop();
  void cleanup_used_lidar_seqs();
  void process_frame(const Pylon::CBaslerUniversalGrabResultPtr & ptr);

  std::string serial_number_;
  std::string device_user_id_;
  std::string load_user_set_{"UserSet2"};
  bool publish_ros_{true};
  std::string image_topic_{"/basler/image_raw"};
  std::string frame_id_{"basler_camera"};
  int max_images_{0};
  int grab_timeout_ms_{1000};

  std::string acquisition_mode_{"Continuous"};
  std::string exposure_mode_{"Timed"};
  double exposure_time_us_{300.0};
  std::string trigger_selector_{"FrameStart"};
  std::string trigger_source_{"Line1"};
  std::string trigger_activation_{"RisingEdge"};
  std::string trigger_mode_{"On"};
  bool line_inverter_{false};
  double line_debouncer_us_{0.0};
  bool chunk_line_status_all_{true};

  Pylon::CBaslerUniversalInstantCamera camera_;
  Pylon::CImageFormatConverter converter_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  std::thread grab_thread_;
  std::atomic<bool> stop_requested_{false};
  uint64_t seq_{0};

  bool use_lidar_stamp_for_ros_header_{true};
  std::string lidar_mmap_path_{"/dev/shm/lidar_stamp.bin"};
  int lidar_mmap_timeout_ms_{500};
  std::unique_ptr<LidarStampReader> lidar_stamp_reader_;

  int64_t match_threshold_ns_{40000000};
  bool enforce_monotonic_lidar_seq_{true};
  std::set<uint64_t> used_lidar_seqs_;
  uint64_t last_matched_lidar_seq_{0};
  int64_t last_lidar_reader_retry_ns_{0};

  int resync_unmatched_count_{0};
  bool resync_mode_active_{false};
  int64_t last_match_dt_ns_{0};
  int64_t resync_trigger_dt_ns_{40000000};
  int resync_trigger_count_{3};
};

}  // namespace basler_ext_trigger_cpp