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
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <atomic>
#include <vector>

#include "basler_ext_trigger_cpp/liv_mmap.hpp"

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
using Pylon::CIntegerParameter;
using Pylon::CBooleanParameter;
using Pylon::CEnumParameter;
using Pylon::CFloatParameter;
using Pylon::TimeoutHandling_Return;
using Pylon::TimeoutHandling_ThrowException;

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
    declare_parameter<std::string>("load_user_set", "Default");
    declare_parameter<std::string>("output_dir", "~/basler_trigger_output_cpp");
    declare_parameter<bool>("save_images", true);
    declare_parameter<bool>("publish_ros", false);
    declare_parameter<std::string>("image_topic", "/basler/image_raw");
    declare_parameter<std::string>("frame_id", "basler_camera");
    declare_parameter<std::string>("image_format", "png");
    declare_parameter<int>("jpeg_quality", 95);
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
    declare_parameter<int>("mmap_slots", 64);
    declare_parameter<std::string>("compat_stamp_path", "/dev/shm/liv_sync_stamp");

    serial_number_ = get_parameter("serial_number").as_string();
    device_user_id_ = get_parameter("device_user_id").as_string();
    load_user_set_ = get_parameter("load_user_set").as_string();
    output_dir_ = expand_user(get_parameter("output_dir").as_string());
    save_images_ = get_parameter("save_images").as_bool();
    publish_ros_ = get_parameter("publish_ros").as_bool();
    image_topic_ = get_parameter("image_topic").as_string();
    frame_id_ = get_parameter("frame_id").as_string();
    image_format_ = lowercase(get_parameter("image_format").as_string());
    jpeg_quality_ = get_parameter("jpeg_quality").as_int();
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

    fs::create_directories(output_dir_);

    if (publish_ros_) {
      image_pub_ = create_publisher<sensor_msgs::msg::Image>(image_topic_, 10);
    }

    if (enable_mmap_) {
      mmap_writer_ = std::make_unique<LivSyncRingWriter>(mmap_path_, static_cast<uint32_t>(mmap_slots_));
      RCLCPP_INFO(get_logger(), "MMAP ring enabled: %s (%d slots)", mmap_path_.c_str(), mmap_slots_);
    }

    open_csv();
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
  }

private:
  std::string serial_number_;
  std::string device_user_id_;
  std::string load_user_set_;
  std::string output_dir_;
  bool save_images_{true};
  bool publish_ros_{false};
  std::string image_topic_;
  std::string frame_id_;
  std::string image_format_{"png"};
  int jpeg_quality_{95};
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
  int mmap_slots_{64};
  std::string compat_stamp_path_;

  CBaslerUniversalInstantCamera camera_;
  CImageFormatConverter converter_;
  std::unique_ptr<LivSyncRingWriter> mmap_writer_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  std::ofstream csv_;
  std::thread grab_thread_;
  std::atomic<bool> stop_requested_{false};
  std::mutex io_mutex_;
  uint64_t seq_{0};

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
    const fs::path csv_path = fs::path(output_dir_) / "capture_log.csv";
    csv_.open(csv_path, std::ios::out | std::ios::trunc);
    if (!csv_.is_open()) {
      throw std::runtime_error("failed to open CSV log: " + csv_path.string());
    }
    csv_ << "seq,cam_hw_ts_ns,host_ts_ns,block_id,width,height,line_status_all,image_path\n";
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
      get_logger(), "Opened camera: model=%s serial=%s",
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

    // Load a clean runtime state without persisting anything to the camera.
    safe_set(p.UserSetSelector, load_user_set_, "UserSetSelector");
    safe_execute(p.UserSetLoad, "UserSetLoad");

    safe_set(p.AcquisitionMode, acquisition_mode_, "AcquisitionMode");
    safe_set(p.ExposureMode, exposure_mode_, "ExposureMode");
    safe_set_float(p.ExposureTime, exposure_time_us_, "ExposureTime");

    // Trigger configuration matching Basler's basic hardware triggering example.
    safe_set(p.TriggerSelector, trigger_selector_, "TriggerSelector");
    safe_set(p.TriggerMode, trigger_mode_, "TriggerMode");
    safe_set(p.TriggerSource, trigger_source_, "TriggerSource");
    safe_set(p.TriggerActivation, trigger_activation_, "TriggerActivation");

    // For ace 2 Line1 is typically an opto-coupled input; do not force LineMode unless needed.
    try {
      if (GenApi::IsWritable(p.LineSelector)) {
        p.LineSelector.SetValue(trigger_source_.c_str());
        RCLCPP_INFO(get_logger(), "LineSelector=%s", trigger_source_.c_str());
      }
    } catch (const GenICam::GenericException & e) {
      RCLCPP_WARN(get_logger(), "Could not select line %s: %s", trigger_source_.c_str(), e.GetDescription());
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
        const double clamped = std::max(p.LineDebouncerTime.GetMin(), std::min(p.LineDebouncerTime.GetMax(), line_debouncer_us_));
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

  void process_frame(const CBaslerUniversalGrabResultPtr & ptr)
  {
    ++seq_;
    const uint64_t cam_hw_ts_ns = ptr->GetTimeStamp();
    const uint64_t block_id = static_cast<uint64_t>(ptr->GetBlockID());
    const uint32_t width = static_cast<uint32_t>(ptr->GetWidth());
    const uint32_t height = static_cast<uint32_t>(ptr->GetHeight());
    const uint64_t host_ts_ns = static_cast<uint64_t>(this->now().nanoseconds());

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
      static_cast<int>(height), static_cast<int>(width), CV_8UC3,
      reinterpret_cast<uint8_t *>(converted.GetBuffer()));

    std::string image_path;
    if (save_images_) {
      image_path = save_image(bgr, seq_);
    }

    if (publish_ros_ && image_pub_) {
      auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", bgr).toImageMsg();
      msg->header.stamp = this->now();
      msg->header.frame_id = frame_id_;
      image_pub_->publish(*msg);
    }

    {
      std::lock_guard<std::mutex> lock(io_mutex_);
      csv_ << seq_ << ',' << cam_hw_ts_ns << ',' << host_ts_ns << ',' << block_id << ','
           << width << ',' << height << ',' << line_status_all << ',' << image_path << "\n";
      csv_.flush();
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
      copy_path(rec.image_path, image_path);
      mmap_writer_->write(rec);
    }

    if (!compat_stamp_path_.empty()) {
      std::ofstream compat(compat_stamp_path_, std::ios::out | std::ios::trunc);
      compat << seq_ << ',' << cam_hw_ts_ns << ',' << host_ts_ns << ',' << image_path << '\n';
    }

    RCLCPP_INFO(
      get_logger(),
      "frame seq=%lu block_id=%lu cam_hw_ts_ns=%lu line_status_all=0x%lx path=%s",
      static_cast<unsigned long>(seq_),
      static_cast<unsigned long>(block_id),
      static_cast<unsigned long>(cam_hw_ts_ns),
      static_cast<unsigned long>(line_status_all),
      image_path.c_str());
  }

  std::string save_image(const cv::Mat & bgr, uint64_t seq)
  {
    std::ostringstream name;
    name << "img_" << std::setfill('0') << std::setw(6) << seq << '.' << image_format_;
    const fs::path path = fs::path(output_dir_) / name.str();

    std::vector<int> params;
    bool ok = false;
    if (image_format_ == "jpg" || image_format_ == "jpeg") {
      params = {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_};
      ok = cv::imwrite(path.string(), bgr, params);
    } else if (image_format_ == "png") {
      ok = cv::imwrite(path.string(), bgr);
    } else {
      ok = cv::imwrite(path.string(), bgr);
    }

    if (!ok) {
      throw std::runtime_error("failed to save image: " + path.string());
    }
    return path.string();
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
