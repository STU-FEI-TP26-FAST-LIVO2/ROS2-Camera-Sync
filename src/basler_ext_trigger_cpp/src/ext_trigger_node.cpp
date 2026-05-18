#include <pylon/PylonIncludes.h>
#include <pylon/BaslerUniversalInstantCamera.h>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

#include <chrono>
#include <cstdlib>
#include <cctype>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <atomic>
#include <vector>
#include <limits>
#include <algorithm>

#include "basler_ext_trigger_cpp/liv_mmap.hpp"
#include "basler_ext_trigger_cpp/lidar_stamp_reader.hpp"

#if __has_include(<cv_bridge/cv_bridge.hpp>)
  #include <cv_bridge/cv_bridge.hpp>
#else
  #include <cv_bridge/cv_bridge.h>
#endif

namespace fs = std::filesystem;
using namespace std::chrono_literals;
using Pylon::CBaslerUniversalGrabResultPtr;
using Pylon::CImageFormatConverter;
using Pylon::CPylonImage;
using Pylon::CTlFactory;
using Pylon::CDeviceInfo;
using Pylon::CBaslerUniversalInstantCamera;
using Pylon::TimeoutHandling_Return;

namespace basler_ext_trigger_cpp
{

class BaslerExtTriggerNode : public rclcpp::Node
{
public:
  BaslerExtTriggerNode()
  : Node("basler_ext_trigger_cpp_node")
  {
    declare_parameter<std::string>("serial_number", "");
    declare_parameter<std::string>("device_user_id", "");
    declare_parameter<std::string>("load_user_set", "UserSet1");
    declare_parameter<bool>("publish_ros", true);
    declare_parameter<std::string>("image_topic", "/basler/image_raw");
    declare_parameter<std::string>("frame_id", "basler_camera");
    declare_parameter<int>("max_images", 0);
    declare_parameter<int>("grab_timeout_ms", 1000);
    declare_parameter<std::string>("acquisition_mode", "Continuous");
    declare_parameter<std::string>("exposure_mode", "Timed");
    declare_parameter<double>("exposure_time_us", 3000.0);
    declare_parameter<std::string>("trigger_selector", "FrameStart");
    declare_parameter<std::string>("trigger_source", "Line1");
    declare_parameter<std::string>("trigger_activation", "RisingEdge");
    declare_parameter<std::string>("trigger_mode", "On");
    declare_parameter<bool>("line_inverter", false);
    declare_parameter<double>("line_debouncer_us", 0.0);
    declare_parameter<bool>("chunk_line_status_all", true);
    declare_parameter<bool>("enable_mmap", true);
    declare_parameter<std::string>("mmap_path", "/dev/shm/liv_sync_ring.bin");
    declare_parameter<int>("mmap_slots", 256);
    declare_parameter<std::string>("compat_stamp_path", "/dev/shm/liv_sync_stamp");

    declare_parameter<bool>("use_lidar_stamp_for_ros_header", true);
    declare_parameter<std::string>("lidar_mmap_path", "/dev/shm/lidar_stamp.bin");

    // QoS publisher
    declare_parameter<bool>("sensor_data_qos", false);
    declare_parameter<int>("queue_depth", 10);
    declare_parameter<std::string>("reliability", "reliable");

    declare_parameter<std::string>("matches_csv_path", "~/Matching/Matches.csv");
    declare_parameter<int>("match_threshold_ms", 100);
    declare_parameter<int>("lidar_reader_retry_ms", 500);
    declare_parameter<bool>("enforce_monotonic_lidar_seq", true);

    serial_number_ = get_parameter("serial_number").as_string();
    device_user_id_ = get_parameter("device_user_id").as_string();
    load_user_set_ = get_parameter("load_user_set").as_string();
    publish_ros_ = get_parameter("publish_ros").as_bool();
    image_topic_ = get_parameter("image_topic").as_string();
    frame_id_ = get_parameter("frame_id").as_string();
    max_images_ = get_parameter("max_images").as_int();
    grab_timeout_ms_ = get_parameter("grab_timeout_ms").as_int();
    acquisition_mode_ = get_parameter("acquisition_mode").as_string();
    exposure_mode_ = get_parameter("exposure_mode").as_string();
    exposure_time_us_ = get_parameter("exposure_time_us").as_double();
    trigger_selector_ = get_parameter("trigger_selector").as_string();
    trigger_source_ = get_parameter("trigger_source").as_string();
    trigger_activation_ = get_parameter("trigger_activation").as_string();
    trigger_mode_ = get_parameter("trigger_mode").as_string();
    line_inverter_ = get_parameter("line_inverter").as_bool();
    line_debouncer_us_ = get_parameter("line_debouncer_us").as_double();
    chunk_line_status_all_ = get_parameter("chunk_line_status_all").as_bool();
    enable_mmap_ = get_parameter("enable_mmap").as_bool();
    mmap_path_ = get_parameter("mmap_path").as_string();
    mmap_slots_ = get_parameter("mmap_slots").as_int();
    compat_stamp_path_ = get_parameter("compat_stamp_path").as_string();

    use_lidar_stamp_for_ros_header_ = get_parameter("use_lidar_stamp_for_ros_header").as_bool();
    lidar_mmap_path_ = get_parameter("lidar_mmap_path").as_string();

    const bool pub_sensor_data_qos = get_parameter("sensor_data_qos").as_bool();
    const int pub_queue_depth = get_parameter("queue_depth").as_int();
    const std::string pub_reliability = get_parameter("reliability").as_string();

    matches_csv_path_ = expand_user(get_parameter("matches_csv_path").as_string());
    match_threshold_ms_ = get_parameter("match_threshold_ms").as_int();
    lidar_reader_retry_ms_ = get_parameter("lidar_reader_retry_ms").as_int();
    enforce_monotonic_lidar_seq_ = get_parameter("enforce_monotonic_lidar_seq").as_bool();

    if (publish_ros_) {
      rclcpp::QoS pub_qos = build_qos(pub_sensor_data_qos, pub_queue_depth, pub_reliability);
      image_pub_ = create_publisher<sensor_msgs::msg::Image>(image_topic_, pub_qos);
      RCLCPP_INFO(
        get_logger(),
        "Image publisher QoS: sensor_data_qos=%s  queue_depth=%d  reliability=%s",
        pub_sensor_data_qos ? "true" : "false",
        pub_sensor_data_qos ? 5 : pub_queue_depth,
        pub_sensor_data_qos ? "best_effort" : pub_reliability.c_str());
    }

    if (enable_mmap_) {
      mmap_writer_ = std::make_unique<LivSyncRingWriter>(
        mmap_path_, static_cast<uint32_t>(mmap_slots_));
      RCLCPP_INFO(
        get_logger(), "MMAP ring enabled: %s (%d slots)",
        mmap_path_.c_str(), mmap_slots_);
    }

    open_csv();
    open_matches_csv();

    try {
      lidar_stamp_reader_ = std::make_unique<LidarStampReader>(lidar_mmap_path_);
      RCLCPP_INFO(
        get_logger(), "LiDAR mmap reader enabled: %s",
        lidar_mmap_path_.c_str());
    } catch (const std::exception & e) {
      RCLCPP_WARN(
        get_logger(),
        "LiDAR mmap reader not available yet: %s",
        e.what());
    }

    open_camera();
    configure_camera_runtime_only();
    start_grabbing();

    grab_thread_ = std::thread([this]() { grab_loop(); });
  }

  ~BaslerExtTriggerNode() override
  {
    stop_requested_ = true;
    if (grab_thread_.joinable()) {
      grab_thread_.join();
    }

    try {
      if (camera_.IsGrabbing()) {
        camera_.StopGrabbing();
      }
      if (camera_.IsOpen()) {
        camera_.Close();
      }
    } catch (...) {
    }

    if (csv_.is_open()) {
      csv_.flush();
      csv_.close();
    }

    if (matches_csv_.is_open()) {
      matches_csv_.flush();
      matches_csv_.close();
    }
  }

private:
  std::string serial_number_;
  std::string device_user_id_;
  std::string load_user_set_;
  bool publish_ros_{true};
  std::string image_topic_;
  std::string frame_id_;
  int max_images_{0};
  int grab_timeout_ms_{1000};
  std::string acquisition_mode_{"Continuous"};
  std::string exposure_mode_{"Timed"};
  double exposure_time_us_{3000.0};
  std::string trigger_selector_{"FrameStart"};
  std::string trigger_source_{"Line1"};
  std::string trigger_activation_{"RisingEdge"};
  std::string trigger_mode_{"On"};
  bool line_inverter_{false};
  double line_debouncer_us_{0.0};
  bool chunk_line_status_all_{true};
  bool enable_mmap_{true};
  std::string mmap_path_;
  int mmap_slots_{256};
  std::string compat_stamp_path_;

  CBaslerUniversalInstantCamera camera_;
  CImageFormatConverter converter_;
  std::unique_ptr<LivSyncRingWriter> mmap_writer_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  std::ofstream csv_;
  std::ofstream matches_csv_;
  std::thread grab_thread_;
  std::atomic<bool> stop_requested_{false};
  std::mutex io_mutex_;
  uint64_t seq_{0};

  bool use_lidar_stamp_for_ros_header_{true};
  std::string lidar_mmap_path_{"/dev/shm/lidar_stamp.bin"};
  std::unique_ptr<LidarStampReader> lidar_stamp_reader_;

  std::string matches_csv_path_;
  int match_threshold_ms_{55};
  int lidar_reader_retry_ms_{500};
  bool enforce_monotonic_lidar_seq_{true};
  std::set<uint64_t> used_lidar_seqs_;
  uint64_t last_matched_lidar_seq_{0};
  int64_t last_lidar_reader_retry_ns_{0};

  // Resync mode for handling long gaps or series of unmatched frames
  int resync_unmatched_count_{0};  // Counter for consecutive unmatched frames
  bool resync_mode_active_{false};  // True when in resync recovery
  int64_t last_match_dt_ns_{0};     // Last successful match dt
  static constexpr int64_t RESYNC_TRIGGER_DT_NS = 40 * 1000000LL;  // 40ms
  static constexpr int RESYNC_TRIGGER_COUNT = 3;  // Trigger after 3 unmatched frames

  struct MatchDecision
  {
    uint64_t lidar_ros_header_ns{0};
    uint64_t lidar_host_receive_ns{0};
    uint64_t lidar_seq{0};
    int64_t match_dt_ns{0};  // Signed: positive = LiDAR after camera, negative = before
    int num_candidates{0};   // Total valid candidates examined
    uint64_t best_candidate_seq{0};  // Best candidate seq (may be rejected)
    int64_t best_candidate_dt_ns{0}; // Best candidate dt
    std::string match_quality{"UNMATCHED"};
    std::string match_reason{"NO_READER"};

    bool accepted() const
    {
      return match_quality == "MATCHED";
    }
  };

  static rclcpp::QoS build_qos(bool sensor_data_qos, int queue_depth,
                               const std::string & reliability)
  {
    if (sensor_data_qos) {
      return rclcpp::SensorDataQoS();
    }
    auto qos = rclcpp::QoS(rclcpp::KeepLast(queue_depth));
    if (reliability == "best_effort") {
      qos.best_effort();
    } else {
      qos.reliable();
    }
    return qos;
  }

  static std::string lowercase(std::string v)
  {
    for (char & c : v) {
      c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    }
    return v;
  }

  static std::string expand_user(const std::string & path)
  {
    if (!path.empty() && path[0] == '~') {
      const char * home = std::getenv("HOME");
      if (home) {
        return std::string(home) + path.substr(1);
      }
    }
    return path;
  }

  void open_csv()
  {
    const fs::path matches_path(matches_csv_path_);
    const fs::path csv_path = matches_path.parent_path() / "capture_log.csv";
    fs::create_directories(csv_path.parent_path());
    csv_.open(csv_path, std::ios::out | std::ios::trunc);
    if (!csv_.is_open()) {
      throw std::runtime_error("failed to open CSV log: " + csv_path.string());
    }

    csv_
      << "seq,cam_hw_ts_ns,host_ts_ns,"
      << "lidar_ros_header_ns,lidar_host_receive_ns,lidar_seq,match_dt_ns,"
      << "match_quality,match_reason,num_candidates,best_candidate_seq,"
      << "block_id,width,height,line_status_all\n";
    csv_.flush();
  }

  void open_matches_csv()
  {
    const fs::path matches_path(matches_csv_path_);
    fs::create_directories(matches_path.parent_path());

    matches_csv_.open(matches_path, std::ios::out | std::ios::trunc);
    if (!matches_csv_.is_open()) {
      throw std::runtime_error("failed to open Matches CSV log: " + matches_path.string());
    }

    matches_csv_
      << "seq,cam_hw_ts_ns,host_ts_ns,"
      << "lidar_ros_header_ns,lidar_host_receive_ns,lidar_seq,match_dt_ns,"
      << "match_quality,match_reason,num_candidates,best_candidate_seq,"
      << "block_id,width,height,line_status_all\n";
    matches_csv_.flush();
  }

  std::string classify_match_quality(int64_t dt_ns, uint64_t lidar_seq) const
  {
    if (lidar_seq == 0) {
      return "UNMATCHED";
    }

    const int64_t match_ns =
      static_cast<int64_t>(match_threshold_ms_) * 1000000LL;

    const int64_t abs_dt_ns = std::llabs(dt_ns);
    return (abs_dt_ns <= match_ns) ? "MATCHED" : "UNMATCHED";
  }

  void ensure_lidar_reader()
  {
    if (lidar_stamp_reader_) {
      return;
    }

    const int64_t now_ns = this->now().nanoseconds();
    const int64_t retry_period_ns =
      static_cast<int64_t>(std::max(1, lidar_reader_retry_ms_)) * 1000000LL;

    if (last_lidar_reader_retry_ns_ != 0 &&
        (now_ns - last_lidar_reader_retry_ns_) < retry_period_ns)
    {
      return;
    }

    last_lidar_reader_retry_ns_ = now_ns;

    try {
      lidar_stamp_reader_ = std::make_unique<LidarStampReader>(lidar_mmap_path_);
      RCLCPP_INFO(get_logger(), "LiDAR mmap reader reconnected: %s", lidar_mmap_path_.c_str());
    } catch (const std::exception &) {
      // Retry quietly later.
    }
  }

  MatchDecision find_best_lidar_match(uint64_t host_ts_ns)
  {
    MatchDecision decision{};
    ensure_lidar_reader();

    if (!lidar_stamp_reader_) {
      decision.match_reason = "NO_READER";
      return decision;
    }

    const auto records = lidar_stamp_reader_->recent_records();
    if (records.empty()) {
      decision.match_reason = "BUFFER_EMPTY";
      return decision;
    }

    const int64_t max_match_ns =
      static_cast<int64_t>(std::max(0, match_threshold_ms_)) * 1000000LL;

    // Track candidates in both directions
    const LidarRingRecord * best_past = nullptr;
    int64_t best_past_dt_ns = 0;
    int64_t best_past_abs_dt_ns = std::numeric_limits<int64_t>::max();

    const LidarRingRecord * best_future = nullptr;
    int64_t best_future_dt_ns = 0;
    int64_t best_future_abs_dt_ns = std::numeric_limits<int64_t>::max();

    int valid_candidates = 0;
    int used_candidates = 0;
    int monotonic_rejected = 0;

    for (const auto & rec : records) {
      // Skip invalid records
      if (rec.lidar_seq == 0) {
        continue;
      }

      valid_candidates++;

      const bool already_used = used_lidar_seqs_.count(rec.lidar_seq) > 0;
      if (already_used) {
        used_candidates++;
        continue;
      }

      const bool monotonic_ok =
        !enforce_monotonic_lidar_seq_ ||
        last_matched_lidar_seq_ == 0 ||
        rec.lidar_seq > last_matched_lidar_seq_;

      if (!monotonic_ok) {
        monotonic_rejected++;
        continue;
      }

      // Calculate signed delta: positive if LiDAR is after camera host time
      const int64_t dt_ns = static_cast<int64_t>(host_ts_ns) - 
                            static_cast<int64_t>(rec.lidar_host_receive_ns);
      const int64_t abs_dt_ns = std::llabs(dt_ns);

      // Track best in past direction (dt_ns < 0 means LiDAR is older)
      if (dt_ns <= 0) {
        if (best_past == nullptr || abs_dt_ns < best_past_abs_dt_ns) {
          best_past = &rec;
          best_past_dt_ns = dt_ns;
          best_past_abs_dt_ns = abs_dt_ns;
        }
      } else {
        // Track best in future direction (dt_ns > 0 means LiDAR is newer)
        if (best_future == nullptr || abs_dt_ns < best_future_abs_dt_ns) {
          best_future = &rec;
          best_future_dt_ns = dt_ns;
          best_future_abs_dt_ns = abs_dt_ns;
        }
      }
    }

    // Determine the overall best candidate
    const LidarRingRecord * best_candidate = nullptr;
    int64_t best_dt_ns = 0;
    int64_t best_abs_dt_ns = std::numeric_limits<int64_t>::max();

    if (best_past != nullptr && best_future != nullptr) {
      // Both directions have candidates - choose by absolute distance
      // But if past is very old (>40ms) and future is much closer, prefer future
      if (best_past_abs_dt_ns > RESYNC_TRIGGER_DT_NS && 
          best_future_abs_dt_ns < best_past_abs_dt_ns * 0.8)
      {
        best_candidate = best_future;
        best_dt_ns = best_future_dt_ns;
        best_abs_dt_ns = best_future_abs_dt_ns;
      } else if (best_past_abs_dt_ns <= best_future_abs_dt_ns) {
        best_candidate = best_past;
        best_dt_ns = best_past_dt_ns;
        best_abs_dt_ns = best_past_abs_dt_ns;
      } else {
        best_candidate = best_future;
        best_dt_ns = best_future_dt_ns;
        best_abs_dt_ns = best_future_abs_dt_ns;
      }
    } else if (best_past != nullptr) {
      best_candidate = best_past;
      best_dt_ns = best_past_dt_ns;
      best_abs_dt_ns = best_past_abs_dt_ns;
    } else if (best_future != nullptr) {
      best_candidate = best_future;
      best_dt_ns = best_future_dt_ns;
      best_abs_dt_ns = best_future_abs_dt_ns;
    }

    // If no candidate found
    if (best_candidate == nullptr) {
      decision.num_candidates = valid_candidates;
      if (used_candidates > 0 && valid_candidates > used_candidates) {
        decision.match_reason = "CANDIDATE_USED";
      } else if (monotonic_rejected > 0) {
        decision.match_reason = "MONOTONIC_REJECT";
      } else {
        decision.match_reason = "BUFFER_STALE";
      }
      return decision;
    }

    decision.num_candidates = valid_candidates;
    decision.best_candidate_seq = best_candidate->lidar_seq;
    decision.best_candidate_dt_ns = best_dt_ns;

    // Check if best candidate is within threshold
    if (best_abs_dt_ns > max_match_ns) {
      decision.match_dt_ns = best_dt_ns;
      decision.match_reason = "CANDIDATE_TOO_FAR";
      
      RCLCPP_DEBUG(
        get_logger(),
        "[unmatched] seq=%lu dt=%.1f ms abs=%.1f ms candidates=%d used=%d reason=CANDIDATE_TOO_FAR",
        best_candidate->lidar_seq, best_dt_ns / 1e6, best_abs_dt_ns / 1e6, valid_candidates, used_candidates);
      
      return decision;
    }

    // Accept the best candidate
    decision.lidar_ros_header_ns = best_candidate->lidar_ros_header_ns;
    decision.lidar_host_receive_ns = best_candidate->lidar_host_receive_ns;
    decision.lidar_seq = best_candidate->lidar_seq;
    decision.match_dt_ns = best_dt_ns;
    decision.match_quality = classify_match_quality(best_dt_ns, best_candidate->lidar_seq);
    decision.match_reason = "NEAREST_WITHIN_THRESHOLD";
    
    // Reset resync counter on successful match
    resync_unmatched_count_ = 0;
    resync_mode_active_ = false;
    last_match_dt_ns_ = best_dt_ns;
    
    return decision;
  }

  CDeviceInfo select_device()
  {
    Pylon::DeviceInfoList_t devices;
    CTlFactory::GetInstance().EnumerateDevices(devices);

    if (devices.empty()) {
      throw std::runtime_error("No Basler cameras found.");
    }

    if (serial_number_.empty() && device_user_id_.empty()) {
      return devices[0];
    }

    for (const auto & dev : devices) {
      if (!serial_number_.empty() && dev.GetSerialNumber() == serial_number_.c_str()) {
        return dev;
      }
      if (!device_user_id_.empty() && dev.GetUserDefinedName() == device_user_id_.c_str()) {
        return dev;
      }
    }

    std::ostringstream oss;
    oss << "Requested camera not found. Available: ";
    for (const auto & dev : devices) {
      oss << dev.GetModelName() << " SN=" << dev.GetSerialNumber() << " ";
    }
    throw std::runtime_error(oss.str());
  }

  void open_camera()
  {
    const auto device_info = select_device();
    camera_.Attach(CTlFactory::GetInstance().CreateDevice(device_info));
    camera_.Open();

    RCLCPP_INFO(
      get_logger(),
      "Opened camera: model=%s serial=%s",
      camera_.GetDeviceInfo().GetModelName().c_str(),
      camera_.GetDeviceInfo().GetSerialNumber().c_str());

    converter_.OutputPixelFormat = Pylon::PixelType_BGR8packed;
    converter_.OutputBitAlignment = Pylon::OutputBitAlignment_MsbAligned;
  }

  template<typename ParamT>
  void safe_set(ParamT & param, const std::string & value, const char * name)
  {
    try {
      if (GenApi::IsWritable(param)) {
        param.SetValue(value.c_str());
        RCLCPP_INFO(get_logger(), "%s=%s", name, value.c_str());
      } else {
        RCLCPP_WARN(get_logger(), "%s not writable, skipping", name);
      }
    } catch (const GenICam::GenericException & e) {
      RCLCPP_WARN(get_logger(), "Could not set %s: %s", name, e.GetDescription());
    }
  }

  void safe_set_bool(GenApi::IBoolean & param, bool value, const char * name)
  {
    try {
      if (GenApi::IsWritable(param)) {
        param.SetValue(value);
        RCLCPP_INFO(get_logger(), "%s=%s", name, value ? "true" : "false");
      } else {
        RCLCPP_WARN(get_logger(), "%s not writable, skipping", name);
      }
    } catch (const GenICam::GenericException & e) {
      RCLCPP_WARN(get_logger(), "Could not set %s: %s", name, e.GetDescription());
    }
  }

  void safe_set_float(GenApi::IFloat & param, double value, const char * name)
  {
    try {
      if (GenApi::IsWritable(param)) {
        const double clamped = std::max(param.GetMin(), std::min(param.GetMax(), value));
        param.SetValue(clamped);
        RCLCPP_INFO(get_logger(), "%s=%f", name, clamped);
      } else {
        RCLCPP_WARN(get_logger(), "%s not writable, skipping", name);
      }
    } catch (const GenICam::GenericException & e) {
      RCLCPP_WARN(get_logger(), "Could not set %s: %s", name, e.GetDescription());
    }
  }

  void safe_execute(GenApi::ICommand & cmd, const char * name)
  {
    try {
      if (GenApi::IsWritable(cmd)) {
        cmd.Execute();
        RCLCPP_INFO(get_logger(), "Executed %s", name);
      } else {
        RCLCPP_WARN(get_logger(), "%s not writable, skipping", name);
      }
    } catch (const GenICam::GenericException & e) {
      RCLCPP_WARN(get_logger(), "Could not execute %s: %s", name, e.GetDescription());
    }
  }

  void configure_camera_runtime_only()
  {
    auto & p = camera_;

    safe_set(p.UserSetSelector, load_user_set_, "UserSetSelector");
    safe_execute(p.UserSetLoad, "UserSetLoad");

    safe_set(p.AcquisitionMode, acquisition_mode_, "AcquisitionMode");
    safe_set(p.ExposureMode, exposure_mode_, "ExposureMode");
    safe_set_float(p.ExposureTime, exposure_time_us_, "ExposureTime");

    safe_set(p.TriggerSelector, trigger_selector_, "TriggerSelector");
    safe_set(p.TriggerMode, trigger_mode_, "TriggerMode");
    safe_set(p.TriggerSource, trigger_source_, "TriggerSource");
    safe_set(p.TriggerActivation, trigger_activation_, "TriggerActivation");

    try {
      if (GenApi::IsWritable(p.LineSelector)) {
        p.LineSelector.SetValue(trigger_source_.c_str());
        RCLCPP_INFO(get_logger(), "LineSelector=%s", trigger_source_.c_str());
      }
    } catch (const GenICam::GenericException & e) {
      RCLCPP_WARN(
        get_logger(), "Could not select line %s: %s",
        trigger_source_.c_str(), e.GetDescription());
    }

    try {
      if (GenApi::IsWritable(p.LineInverter)) {
        p.LineInverter.SetValue(line_inverter_);
        RCLCPP_INFO(get_logger(), "LineInverter=%s", line_inverter_ ? "true" : "false");
      }
    } catch (const GenICam::GenericException & e) {
      RCLCPP_WARN(get_logger(), "Could not set LineInverter: %s", e.GetDescription());
    }

    try {
      if (GenApi::IsWritable(p.LineDebouncerTime)) {
        const double clamped =
          std::max(p.LineDebouncerTime.GetMin(),
                   std::min(p.LineDebouncerTime.GetMax(), line_debouncer_us_));
        p.LineDebouncerTime.SetValue(clamped);
        RCLCPP_INFO(get_logger(), "LineDebouncerTime=%f", clamped);
      }
    } catch (const GenICam::GenericException & e) {
      RCLCPP_WARN(get_logger(), "Could not set LineDebouncerTime: %s", e.GetDescription());
    }

    enable_chunks();
  }

  void enable_chunks()
  {
    auto & p = camera_;
    safe_set_bool(p.ChunkModeActive, true, "ChunkModeActive");
    if (chunk_line_status_all_) {
      safe_set(p.ChunkSelector, "LineStatusAll", "ChunkSelector");
      safe_set_bool(p.ChunkEnable, true, "ChunkEnable(LineStatusAll)");
    }
  }

  void start_grabbing()
  {
    camera_.StartGrabbing(Pylon::GrabStrategy_OneByOne, Pylon::GrabLoop_ProvidedByUser);
    RCLCPP_INFO(
      get_logger(),
      "Camera armed. Waiting for external triggers on %s. The node will only retrieve completed frames.",
      trigger_source_.c_str());
  }

  void grab_loop()
  {
    while (rclcpp::ok() && !stop_requested_ && camera_.IsGrabbing()) {
      try {
        CBaslerUniversalGrabResultPtr ptr;
        const bool got_result = camera_.RetrieveResult(
          static_cast<uint32_t>(grab_timeout_ms_), ptr, TimeoutHandling_Return);

        if (!got_result || !ptr) {
          continue;
        }

        if (!ptr->GrabSucceeded()) {
          RCLCPP_WARN(
            get_logger(), "Grab failed: code=%d desc=%s",
            ptr->GetErrorCode(), ptr->GetErrorDescription().c_str());
          continue;
        }

        process_frame(ptr);

        if (max_images_ > 0 && static_cast<int>(seq_) >= max_images_) {
          RCLCPP_INFO(get_logger(), "Reached max_images=%d, stopping.", max_images_);
          camera_.StopGrabbing();
          break;
        }
      } catch (const GenICam::GenericException & e) {
        RCLCPP_ERROR(get_logger(), "Grab loop exception: %s", e.GetDescription());
        std::this_thread::sleep_for(100ms);
      }
    }
  }

  void cleanup_used_lidar_seqs()
  {
    if (last_matched_lidar_seq_ == 0) {
      return;
    }

    const uint64_t keep_after =
      (last_matched_lidar_seq_ > 512) ? (last_matched_lidar_seq_ - 512) : 0;

    auto it = used_lidar_seqs_.begin();
    while (it != used_lidar_seqs_.end() && *it < keep_after) {
      it = used_lidar_seqs_.erase(it);
    }
  }

  void process_frame(const CBaslerUniversalGrabResultPtr & ptr)
  {
    ++seq_;

    const uint64_t cam_hw_ts_ns = ptr->GetTimeStamp();
    const uint64_t block_id = static_cast<uint64_t>(ptr->GetBlockID());
    const uint32_t width = static_cast<uint32_t>(ptr->GetWidth());
    const uint32_t height = static_cast<uint32_t>(ptr->GetHeight());
    const uint64_t host_ts_ns = static_cast<uint64_t>(this->now().nanoseconds());

    const MatchDecision match = find_best_lidar_match(host_ts_ns);
    const uint64_t lidar_ros_header_ns = match.lidar_ros_header_ns;
    const uint64_t lidar_host_receive_ns = match.lidar_host_receive_ns;
    const uint64_t lidar_seq = match.lidar_seq;
    const int64_t match_dt_ns = match.match_dt_ns;
    const std::string match_quality = match.match_quality;
    const std::string match_reason = match.match_reason;

    // Track resync state
    if (match.accepted()) {
      used_lidar_seqs_.insert(lidar_seq);
      last_matched_lidar_seq_ = lidar_seq;
      cleanup_used_lidar_seqs();
      resync_unmatched_count_ = 0;
    } else {
      resync_unmatched_count_++;
      
      // Check if we should activate resync mode
      if (resync_unmatched_count_ >= RESYNC_TRIGGER_COUNT ||
          std::llabs(match_dt_ns) > RESYNC_TRIGGER_DT_NS)
      {
        if (!resync_mode_active_) {
          resync_mode_active_ = true;
          RCLCPP_WARN(
            get_logger(),
            "[RESYNC-ACTIVATED] unmatched_count=%d dt=%.1f ms reason=%s",
            resync_unmatched_count_, match_dt_ns / 1e6, match_reason.c_str());
        }
      }
      
      // Detailed logging for unmatched frames
      RCLCPP_DEBUG(
        get_logger(),
        "[unmatched-detailed] seq=%lu cam_hw=%lu dt=%.1f ms abs=%.1f ms reason=%s "
        "candidates=%d best_seq=%lu best_dt=%.1f ms resync=%s",
        seq_, cam_hw_ts_ns, match_dt_ns / 1e6, std::llabs(match_dt_ns) / 1e6, match_reason.c_str(),
        match.num_candidates, match.best_candidate_seq, match.best_candidate_dt_ns / 1e6,
        resync_mode_active_ ? "YES" : "NO");
    }

    uint64_t line_status_all = 0;
    try {
      if (chunk_line_status_all_) {
        line_status_all = static_cast<uint64_t>(ptr->ChunkLineStatusAll.GetValue());
      }
    } catch (const GenICam::GenericException &) {
      line_status_all = 0;
    }

    CPylonImage converted;
    converter_.Convert(converted, ptr);

    cv::Mat bgr(
      static_cast<int>(height),
      static_cast<int>(width),
      CV_8UC3,
      reinterpret_cast<uint8_t *>(converted.GetBuffer()));

    if (publish_ros_ && image_pub_) {
      auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", bgr).toImageMsg();

      if (use_lidar_stamp_for_ros_header_ && lidar_ros_header_ns > 0) {
        msg->header.stamp = rclcpp::Time(static_cast<int64_t>(lidar_ros_header_ns));
      } else {
        msg->header.stamp = this->now();
      }

      msg->header.frame_id = frame_id_;
      image_pub_->publish(*msg);
    }

    {
      std::lock_guard<std::mutex> lock(io_mutex_);

      csv_
        << seq_ << ','
        << cam_hw_ts_ns << ','
        << host_ts_ns << ','
        << lidar_ros_header_ns << ','
        << lidar_host_receive_ns << ','
        << lidar_seq << ','
        << match_dt_ns << ','
        << match_quality << ','
        << match_reason << ','
        << match.num_candidates << ','
        << match.best_candidate_seq << ','
        << block_id << ','
        << width << ','
        << height << ','
        << line_status_all << "\n";
      csv_.flush();

      matches_csv_
        << seq_ << ','
        << cam_hw_ts_ns << ','
        << host_ts_ns << ','
        << lidar_ros_header_ns << ','
        << lidar_host_receive_ns << ','
        << lidar_seq << ','
        << match_dt_ns << ','
        << match_quality << ','
        << match_reason << ','
        << match.num_candidates << ','
        << match.best_candidate_seq << ','
        << block_id << ','
        << width << ','
        << height << ','
        << line_status_all << "\n";
      matches_csv_.flush();
    }

    if (mmap_writer_) {
      RingRecord rec{};
      rec.flags = 0;
      rec.pixel_format = kPixelFormatBGR8;
      rec.seq = seq_;
      rec.cam_hw_ts_ns = cam_hw_ts_ns;
      rec.host_ts_ns = host_ts_ns;
      rec.block_id = block_id;
      rec.width = width;
      rec.height = height;
      rec.line_status_all = line_status_all;
      rec.image_size_bytes = static_cast<uint64_t>(bgr.total() * bgr.elemSize());
      copy_path(rec.image_path, "");
      mmap_writer_->write(rec);
    }

    if (!compat_stamp_path_.empty()) {
      std::ofstream compat(compat_stamp_path_, std::ios::out | std::ios::trunc);
      compat
        << seq_ << ','
        << cam_hw_ts_ns << ','
        << host_ts_ns << ','
        << lidar_ros_header_ns << ','
        << lidar_host_receive_ns << ','
        << lidar_seq << '\n';
    }

    RCLCPP_INFO(
      get_logger(),
      "frame seq=%lu block_id=%lu cam_hw_ts_ns=%lu lidar_ros_header_ns=%lu lidar_seq=%lu match_dt_ns=%lu quality=%s reason=%s",
      static_cast<unsigned long>(seq_),
      static_cast<unsigned long>(block_id),
      static_cast<unsigned long>(cam_hw_ts_ns),
      static_cast<unsigned long>(lidar_ros_header_ns),
      static_cast<unsigned long>(lidar_seq),
      static_cast<unsigned long>(match_dt_ns),
      match_quality.c_str(),
      match_reason.c_str());
  }
};

}  // namespace basler_ext_trigger_cpp

int main(int argc, char ** argv)
{
  Pylon::PylonInitialize();
  rclcpp::init(argc, argv);

  int exit_code = 0;
  try {
    auto node = std::make_shared<basler_ext_trigger_cpp::BaslerExtTriggerNode>();
    rclcpp::spin(node);
  } catch (const GenICam::GenericException & e) {
    std::fprintf(stderr, "Basler/GenICam exception: %s\n", e.GetDescription());
    exit_code = 1;
  } catch (const std::exception & e) {
    std::fprintf(stderr, "Exception: %s\n", e.what());
    exit_code = 1;
  }

  rclcpp::shutdown();
  Pylon::PylonTerminate();
  return exit_code;
}