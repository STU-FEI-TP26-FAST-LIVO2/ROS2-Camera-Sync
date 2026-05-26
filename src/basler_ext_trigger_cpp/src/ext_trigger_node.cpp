#include "basler_ext_trigger_cpp/basler_ext_trigger_node.hpp"

#include <opencv2/core.hpp>
#include <std_msgs/msg/header.hpp>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <thread>

#if __has_include(<cv_bridge/cv_bridge.hpp>)
  #include <cv_bridge/cv_bridge.hpp>
#else
  #include <cv_bridge/cv_bridge.h>
#endif

using namespace std::chrono_literals;
using Pylon::CBaslerUniversalGrabResultPtr;
using Pylon::CPylonImage;
using Pylon::CTlFactory;
using Pylon::CDeviceInfo;
using Pylon::TimeoutHandling_Return;

namespace basler_ext_trigger_cpp
{

bool MatchDecision::accepted() const
{
  return match_quality == "MATCHED";
}

BaslerExtTriggerNode::BaslerExtTriggerNode()
: Node("basler_ext_trigger_cpp_node")
{
  // Camera selection and UserSet.
  declare_parameter<std::string>("serial_number", "");
  declare_parameter<std::string>("device_user_id", "");
  declare_parameter<std::string>("load_user_set", "UserSet2");

  // ROS image output.
  declare_parameter<bool>("publish_ros", true);
  declare_parameter<std::string>("image_topic", "/basler/image_raw");
  declare_parameter<std::string>("frame_id", "basler_camera");
  declare_parameter<bool>("sensor_data_qos", false);
  declare_parameter<int>("queue_depth", 10);
  declare_parameter<std::string>("reliability", "reliable");

  // Grabbing and Basler runtime settings.
  declare_parameter<int>("max_images", 0);
  declare_parameter<int>("grab_timeout_ms", 1000);
  declare_parameter<std::string>("acquisition_mode", "Continuous");
  declare_parameter<std::string>("exposure_mode", "Timed");
  declare_parameter<double>("exposure_time_us", 300.0);
  declare_parameter<std::string>("trigger_selector", "FrameStart");
  declare_parameter<std::string>("trigger_source", "Line1");
  declare_parameter<std::string>("trigger_activation", "RisingEdge");
  declare_parameter<std::string>("trigger_mode", "On");
  declare_parameter<bool>("line_inverter", false);
  declare_parameter<double>("line_debouncer_us", 0.0);
  declare_parameter<bool>("chunk_line_status_all", true);

  // LiDAR timestamp matching. The camera image header is assigned from the
  // nearest LiDAR header timestamp when a valid match is found.
  declare_parameter<bool>("use_lidar_stamp_for_ros_header", true);
  declare_parameter<std::string>("lidar_mmap_path", "/dev/shm/lidar_stamp.bin");
  declare_parameter<int>("lidar_mmap_timeout_ms", 500);
  declare_parameter<bool>("enforce_monotonic_lidar_seq", true);
  declare_parameter<int64_t>("match_threshold_ns", 40000000);
  declare_parameter<int64_t>("resync_trigger_dt_ns", 40000000);
  declare_parameter<int>("resync_trigger_count", 3);

  serial_number_ = get_parameter("serial_number").as_string();
  device_user_id_ = get_parameter("device_user_id").as_string();
  load_user_set_ = get_parameter("load_user_set").as_string();

  publish_ros_ = get_parameter("publish_ros").as_bool();
  image_topic_ = get_parameter("image_topic").as_string();
  frame_id_ = get_parameter("frame_id").as_string();
  const bool pub_sensor_data_qos = get_parameter("sensor_data_qos").as_bool();
  const int pub_queue_depth = get_parameter("queue_depth").as_int();
  const std::string pub_reliability = get_parameter("reliability").as_string();

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

  use_lidar_stamp_for_ros_header_ =
    get_parameter("use_lidar_stamp_for_ros_header").as_bool();
  lidar_mmap_path_ = get_parameter("lidar_mmap_path").as_string();
  lidar_mmap_timeout_ms_ = get_parameter("lidar_mmap_timeout_ms").as_int();
  enforce_monotonic_lidar_seq_ = get_parameter("enforce_monotonic_lidar_seq").as_bool();
  match_threshold_ns_ = get_parameter("match_threshold_ns").as_int();
  resync_trigger_dt_ns_ = get_parameter("resync_trigger_dt_ns").as_int();
  resync_trigger_count_ = get_parameter("resync_trigger_count").as_int();

  if (match_threshold_ns_ < 80000000) {
    RCLCPP_WARN(
      get_logger(),
      "match_threshold_ns=%ld is lower than 1 ms.",
      static_cast<long>(match_threshold_ns_));
  }
  if (resync_trigger_dt_ns_ < 80000000) {
    RCLCPP_WARN(
      get_logger(),
      "resync_trigger_dt_ns=%ld is lower than 1 ms.",
      static_cast<long>(resync_trigger_dt_ns_));
  }

  if (publish_ros_) {
    rclcpp::QoS pub_qos = build_qos(pub_sensor_data_qos, pub_queue_depth, pub_reliability);
    image_pub_ = create_publisher<sensor_msgs::msg::Image>(image_topic_, pub_qos);
    RCLCPP_INFO(
      get_logger(),
      "Image publisher QoS: sensor_data_qos=%s queue_depth=%d reliability=%s",
      pub_sensor_data_qos ? "true" : "false",
      pub_sensor_data_qos ? 5 : pub_queue_depth,
      pub_sensor_data_qos ? "best_effort" : pub_reliability.c_str());
  }

  try {
    lidar_stamp_reader_ = std::make_unique<LidarStampReader>(lidar_mmap_path_);
    RCLCPP_INFO(get_logger(), "LiDAR mmap reader enabled: %s", lidar_mmap_path_.c_str());
  } catch (const std::exception & e) {
    RCLCPP_WARN(get_logger(), "LiDAR mmap reader not available yet: %s", e.what());
  }

  open_camera();
  configure_camera_runtime_only();
  start_grabbing();

  grab_thread_ = std::thread([this]() { grab_loop(); });
}

BaslerExtTriggerNode::~BaslerExtTriggerNode()
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
}

rclcpp::QoS BaslerExtTriggerNode::build_qos(
  bool sensor_data_qos, int queue_depth, const std::string & reliability)
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

std::string BaslerExtTriggerNode::classify_match_quality(
  int64_t dt_ns, uint64_t lidar_seq) const
{
  if (lidar_seq == 0) {
    return "UNMATCHED";
  }

  const int64_t abs_dt_ns = std::llabs(dt_ns);
  return (abs_dt_ns <= std::max<int64_t>(0, match_threshold_ns_)) ?
         "MATCHED" : "UNMATCHED";
}

void BaslerExtTriggerNode::ensure_lidar_reader()
{
  if (lidar_stamp_reader_) {
    return;
  }

  const int64_t now_ns = this->now().nanoseconds();
  const int64_t retry_period_ns =
    static_cast<int64_t>(std::max(1, lidar_mmap_timeout_ms_)) * 80000000LL;

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

MatchDecision BaslerExtTriggerNode::find_best_lidar_match(uint64_t host_ts_ns)
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

  const int64_t max_match_ns = std::max<int64_t>(0, match_threshold_ns_);

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

    const int64_t dt_ns =
      static_cast<int64_t>(host_ts_ns) - static_cast<int64_t>(rec.lidar_host_receive_ns);
    const int64_t abs_dt_ns = std::llabs(dt_ns);

    if (dt_ns <= 0) {
      if (best_past == nullptr || abs_dt_ns < best_past_abs_dt_ns) {
        best_past = &rec;
        best_past_dt_ns = dt_ns;
        best_past_abs_dt_ns = abs_dt_ns;
      }
    } else if (best_future == nullptr || abs_dt_ns < best_future_abs_dt_ns) {
      best_future = &rec;
      best_future_dt_ns = dt_ns;
      best_future_abs_dt_ns = abs_dt_ns;
    }
  }

  const LidarRingRecord * best_candidate = nullptr;
  int64_t best_dt_ns = 0;
  int64_t best_abs_dt_ns = std::numeric_limits<int64_t>::max();

  if (best_past != nullptr && best_future != nullptr) {
    if (best_past_abs_dt_ns > resync_trigger_dt_ns_ &&
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

  if (best_abs_dt_ns > max_match_ns) {
    decision.match_dt_ns = best_dt_ns;
    decision.match_reason = "CANDIDATE_TOO_FAR";

    RCLCPP_DEBUG(
      get_logger(),
      "[unmatched] lidar_seq=%lu dt=%.1f ms abs=%.1f ms candidates=%d used=%d reason=%s",
      static_cast<unsigned long>(best_candidate->lidar_seq),
      best_dt_ns / 1e6,
      best_abs_dt_ns / 1e6,
      valid_candidates,
      used_candidates,
      decision.match_reason.c_str());

    return decision;
  }

  decision.lidar_ros_header_ns = best_candidate->lidar_ros_header_ns;
  decision.lidar_seq = best_candidate->lidar_seq;
  decision.match_dt_ns = best_dt_ns;
  decision.match_quality = classify_match_quality(best_dt_ns, best_candidate->lidar_seq);
  decision.match_reason = "NEAREST_WITHIN_THRESHOLD";

  resync_unmatched_count_ = 0;
  resync_mode_active_ = false;
  last_match_dt_ns_ = best_dt_ns;

  return decision;
}

CDeviceInfo BaslerExtTriggerNode::select_device()
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

void BaslerExtTriggerNode::open_camera()
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
void BaslerExtTriggerNode::safe_set(
  ParamT & param, const std::string & value, const char * name)
{
  if (value.empty()) {
    RCLCPP_INFO(get_logger(), "%s empty, skipping", name);
    return;
  }

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

void BaslerExtTriggerNode::safe_set_bool(GenApi::IBoolean & param, bool value, const char * name)
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

void BaslerExtTriggerNode::safe_set_float(GenApi::IFloat & param, double value, const char * name)
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

void BaslerExtTriggerNode::safe_execute(GenApi::ICommand & cmd, const char * name)
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

void BaslerExtTriggerNode::configure_camera_runtime_only()
{
  auto & p = camera_;

  safe_set(p.UserSetSelector, load_user_set_, "UserSetSelector");
  if (!load_user_set_.empty()) {
    safe_execute(p.UserSetLoad, "UserSetLoad");
  }

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

  safe_set_bool(p.LineInverter, line_inverter_, "LineInverter");
  safe_set_float(p.LineDebouncerTime, line_debouncer_us_, "LineDebouncerTime");

  enable_chunks();
}

void BaslerExtTriggerNode::enable_chunks()
{
  auto & p = camera_;
  safe_set_bool(p.ChunkModeActive, true, "ChunkModeActive");
  if (chunk_line_status_all_) {
    safe_set(p.ChunkSelector, "LineStatusAll", "ChunkSelector");
    safe_set_bool(p.ChunkEnable, true, "ChunkEnable(LineStatusAll)");
  }
}

void BaslerExtTriggerNode::start_grabbing()
{
  camera_.StartGrabbing(Pylon::GrabStrategy_OneByOne, Pylon::GrabLoop_ProvidedByUser);
  RCLCPP_INFO(
    get_logger(),
    "Camera armed. Waiting for external triggers on %s.",
    trigger_source_.c_str());
}

void BaslerExtTriggerNode::grab_loop()
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

void BaslerExtTriggerNode::cleanup_used_lidar_seqs()
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

void BaslerExtTriggerNode::process_frame(const CBaslerUniversalGrabResultPtr & ptr)
{
  ++seq_;

  const uint64_t cam_hw_ts_ns = ptr->GetTimeStamp();
  const uint64_t block_id = static_cast<uint64_t>(ptr->GetBlockID());
  const uint32_t width = static_cast<uint32_t>(ptr->GetWidth());
  const uint32_t height = static_cast<uint32_t>(ptr->GetHeight());
  const uint64_t host_ts_ns = static_cast<uint64_t>(this->now().nanoseconds());

  const MatchDecision match = find_best_lidar_match(host_ts_ns);
  const uint64_t lidar_ros_header_ns = match.lidar_ros_header_ns;
  const uint64_t lidar_seq = match.lidar_seq;
  const int64_t match_dt_ns = match.match_dt_ns;
  const std::string match_quality = match.match_quality;
  const std::string match_reason = match.match_reason;

  if (match.accepted()) {
    used_lidar_seqs_.insert(lidar_seq);
    last_matched_lidar_seq_ = lidar_seq;
    cleanup_used_lidar_seqs();
    resync_unmatched_count_ = 0;
  } else {
    resync_unmatched_count_++;

    if (resync_unmatched_count_ >= resync_trigger_count_ ||
        std::llabs(match_dt_ns) > resync_trigger_dt_ns_)
    {
      if (!resync_mode_active_) {
        resync_mode_active_ = true;
        RCLCPP_WARN(
          get_logger(),
          "[RESYNC-ACTIVATED] unmatched_count=%d dt=%.1f ms reason=%s",
          resync_unmatched_count_, match_dt_ns / 1e6, match_reason.c_str());
      }
    }

    RCLCPP_DEBUG(
      get_logger(),
      "[unmatched-detailed] seq=%lu cam_hw=%lu dt=%.1f ms abs=%.1f ms reason=%s "
      "candidates=%d best_seq=%lu best_dt=%.1f ms resync=%s",
      static_cast<unsigned long>(seq_),
      static_cast<unsigned long>(cam_hw_ts_ns),
      match_dt_ns / 1e6,
      std::llabs(match_dt_ns) / 1e6,
      match_reason.c_str(),
      match.num_candidates,
      static_cast<unsigned long>(match.best_candidate_seq),
      match.best_candidate_dt_ns / 1e6,
      resync_mode_active_ ? "YES" : "NO");
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

  RCLCPP_INFO(
    get_logger(),
    "frame seq=%lu block_id=%lu cam_hw_ts_ns=%lu lidar_ros_header_ns=%lu "
    "lidar_seq=%lu match_dt_ns=%ld quality=%s reason=%s",
    static_cast<unsigned long>(seq_),
    static_cast<unsigned long>(block_id),
    static_cast<unsigned long>(cam_hw_ts_ns),
    static_cast<unsigned long>(lidar_ros_header_ns),
    static_cast<unsigned long>(lidar_seq),
    static_cast<long>(match_dt_ns),
    match_quality.c_str(),
    match_reason.c_str());
}

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