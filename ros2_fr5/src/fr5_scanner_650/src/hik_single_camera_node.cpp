#include "HikCalibrationCore.h"
#include "HikCameraWorker.h"

#include <QCoreApplication>
#include <QMetaObject>
#include <QObject>
#include <QTimer>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <cstdint>
#include <deque>
#include <functional>
#include <iomanip>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>

namespace
{

class HikSingleCameraNode final : public rclcpp::Node
{
public:
  HikSingleCameraNode()
  : Node("hik_single_camera")
  {
    camera_ip_ = declare_parameter<std::string>("camera_ip", "192.168.7.45");
    expected_model_ = declare_parameter<std::string>("expected_model", "MV-CS016-10GM");
    expected_serial_ = declare_parameter<std::string>("expected_serial", "DA8784601");
    frame_id_ = declare_parameter<std::string>("frame_id", "hik_camera_optical_frame");
    intrinsics_path_ = declare_parameter<std::string>("intrinsics_path", "");
    exposure_us_ = declare_parameter<double>("exposure_us", 1825.0);
    gain_db_ = declare_parameter<double>("gain_db", 0.0);
    fps_ = declare_parameter<double>("frames_per_second", 60.0);
    pool_capacity_ = declare_parameter<int>("image_pool_capacity", 32);
    publish_queue_capacity_ = declare_parameter<int>("ros_publish_queue_capacity", 16);
    camera_clock_mapping_window_ = declare_parameter<int>(
      "camera_clock_mapping_window", 600);
    camera_mapping_max_residual_us_ = declare_parameter<double>(
      "camera_mapping_max_residual_us", 2000.0);
    camera_fixed_time_offset_us_ = declare_parameter<double>(
      "camera_fixed_time_offset_us", 0.0);
    camera_transport_delay_us_ = declare_parameter<double>(
      "camera_transport_delay_us", 0.0);
    require_device_timestamp_mapping_ = declare_parameter<bool>(
      "require_device_timestamp_mapping", true);
    const bool auto_connect = declare_parameter<bool>("auto_connect", true);
    connection_timeout_s_ = declare_parameter<double>("connection_timeout_s", 8.0);
    const std::string connection_service_name = declare_parameter<std::string>(
      "connection_service_name", "/scanner_650/set_camera_connected");
    const std::string timestamp_reference = declare_parameter<std::string>(
      "camera_timestamp_reference", "exposure_start");
    if (timestamp_reference == "exposure_start") {
      camera_timestamp_reference_ = hik_sync::CameraTimestampReference::ExposureStart;
    } else if (timestamp_reference == "exposure_end") {
      camera_timestamp_reference_ = hik_sync::CameraTimestampReference::ExposureEnd;
    } else {
      throw std::runtime_error(
              "camera_timestamp_reference must be exposure_start or exposure_end");
    }
    if (camera_ip_.empty() || expected_serial_.empty() || frame_id_.empty() ||
      intrinsics_path_.empty() || exposure_us_ <= 0.0 || fps_ <= 0.0 || pool_capacity_ < 2 ||
      publish_queue_capacity_ < 2 || publish_queue_capacity_ >= pool_capacity_ ||
      camera_clock_mapping_window_ < 30 ||
      !std::isfinite(camera_mapping_max_residual_us_) || camera_mapping_max_residual_us_ <= 0.0 ||
      !std::isfinite(camera_fixed_time_offset_us_) ||
      !std::isfinite(camera_transport_delay_us_) || camera_transport_delay_us_ < 0.0 ||
      !std::isfinite(connection_timeout_s_) || connection_timeout_s_ < 1.0 ||
      connection_timeout_s_ > 30.0 || connection_service_name.empty())
    {
      throw std::runtime_error("invalid scanner_650 camera parameters");
    }
    camera_clock_mapper_ = std::make_unique<hik_sync::CameraClockMapper>(
      static_cast<std::size_t>(camera_clock_mapping_window_),
      camera_mapping_max_residual_us_ * 1000.0);

    std::string error;
    if (!hik_calibration::loadIntrinsicsYaml(
        intrinsics_path_, &intrinsics_, &intrinsics_metadata_, &error) || !intrinsics_.ok)
    {
      throw std::runtime_error("cannot load scanner_650 intrinsics: " + error);
    }
    if (intrinsics_metadata_.frameId != frame_id_ ||
      intrinsics_metadata_.cameraSerial != expected_serial_ ||
      (!expected_model_.empty() && intrinsics_metadata_.cameraModel != expected_model_))
    {
      throw std::runtime_error("camera identity/frame parameters do not match hik_intrinsics.yaml");
    }
    camera_info_ = make_camera_info();

    image_publisher_ = create_publisher<sensor_msgs::msg::Image>(
      declare_parameter<std::string>("image_topic", "/scanner_650/image_raw"),
      rclcpp::QoS(static_cast<std::size_t>(pool_capacity_)).reliable());
    info_publisher_ = create_publisher<sensor_msgs::msg::CameraInfo>(
      declare_parameter<std::string>("camera_info_topic", "/scanner_650/camera_info"),
      rclcpp::SensorDataQoS());
    status_publisher_ = create_publisher<std_msgs::msg::String>(
      "/scanner_650/camera_status", rclcpp::QoS(10).reliable().transient_local());

    worker_ = std::make_unique<HikCameraWorker>();
    QObject::connect(
      worker_.get(), &HikCameraWorker::identityChanged,
      [this](const QString & model, const QString & serial, const QString & ip) {
        const bool valid = model.toStdString() == expected_model_ &&
          serial.toStdString() == expected_serial_ && ip.toStdString() == camera_ip_;
        identity_valid_.store(valid);
        if (!valid) {
          RCLCPP_FATAL(
            get_logger(), "scanner_650 identity mismatch: got model=%s serial=%s ip=%s",
            model.toUtf8().constData(), serial.toUtf8().constData(), ip.toUtf8().constData());
        }
      });
    QObject::connect(
      worker_.get(), &HikCameraWorker::connectionChanged,
      [this](bool connected, const QString & detail) {
        RCLCPP_INFO(get_logger(), "%s", detail.toUtf8().constData());
        const bool accepted = connected && identity_valid_.load();
        bool streaming = false;
        {
          std::lock_guard<std::mutex> lock(connection_mutex_);
          camera_connected_ = accepted;
          if (!accepted) {
            camera_streaming_ = false;
          }
          streaming = camera_streaming_;
          last_connection_detail_ = detail.toStdString();
          ++connection_generation_;
        }
        if (!connected) {
          identity_valid_.store(false);
          stream_start_requested_.store(false);
          reset_camera_timing();
        }
        connection_condition_.notify_all();
        publish_connection_status(accepted, streaming, detail.toStdString());
        if (connected && !accepted) {
          QTimer::singleShot(0, worker_.get(), [this]() {worker_->disconnectCamera();});
          return;
        }
        if (accepted && !stream_start_requested_.exchange(true)) {
          // connectionChanged is emitted before HikCameraWorker clears its
          // short-lived connect busy flag.  Queue startup for the next Qt
          // event-loop turn instead of re-entering startContinuous here.
          QTimer::singleShot(0, worker_.get(), [this]() {
            worker_->startContinuous(exposure_us_, gain_db_, fps_, pool_capacity_);
          });
        }
      });
    QObject::connect(
      worker_.get(), &HikCameraWorker::continuousStarted,
      [this](double exposure, double fps, quint64 frequency, const QString & description) {
        camera_frames_received_.store(0U);
        camera_frames_published_.store(0U);
        camera_publish_queue_drops_.store(0U);
        first_device_timestamp_ns_.store(0);
        last_device_timestamp_ns_.store(0);
        {
          std::lock_guard<std::mutex> lock(connection_mutex_);
          camera_streaming_ = true;
          last_connection_detail_ = description.toStdString();
        }
        connection_condition_.notify_all();
        std_msgs::msg::String status;
        status.data = "connected=1; streaming=1; exposure_us=" + std::to_string(exposure) +
          "; fps=" + std::to_string(fps) + "; device_tick_hz=" +
          std::to_string(frequency) + "; " + description.toStdString();
        status_publisher_->publish(status);
        RCLCPP_INFO(get_logger(), "%s", status.data.c_str());
      });
    QObject::connect(
      worker_.get(), &HikCameraWorker::continuousStopped,
      [this](bool, const QString & detail) {
        {
          std::lock_guard<std::mutex> lock(connection_mutex_);
          camera_streaming_ = false;
          last_connection_detail_ = detail.toStdString();
        }
        stream_start_requested_.store(false);
        connection_condition_.notify_all();
      });
    QObject::connect(
      worker_.get(), &HikCameraWorker::continuousFrameReady,
      [this](hik_sync::CameraFrame frame) {enqueue_frame(std::move(frame));});
    QObject::connect(
      worker_.get(), &HikCameraWorker::continuousFrameRejected,
      [this](quint64 frame, const QString & reason) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000, "camera rejected frame %llu: %s",
          static_cast<unsigned long long>(frame), reason.toUtf8().constData());
      });
    QObject::connect(
      worker_.get(), &HikCameraWorker::error,
      [this](int, const QString & message) {
        {
          std::lock_guard<std::mutex> lock(connection_mutex_);
          last_connection_detail_ = message.toStdString();
        }
        connection_condition_.notify_all();
        RCLCPP_ERROR(get_logger(), "%s", message.toUtf8().constData());
      });
    QObject::connect(
      worker_.get(), &HikCameraWorker::log,
      [this](const QString & message) {
        RCLCPP_INFO(get_logger(), "%s", message.toUtf8().constData());
      });

    connection_service_ = create_service<std_srvs::srv::SetBool>(
      connection_service_name,
      std::bind(
        &HikSingleCameraNode::set_camera_connected, this,
        std::placeholders::_1, std::placeholders::_2));
    publisher_thread_ = std::thread([this]() {publisher_worker();});
    publish_connection_status(false, false, "camera node ready; camera disconnected");
    if (auto_connect) {
      QTimer::singleShot(
        0, [this]() {worker_->connectCamera(QString::fromStdString(camera_ip_));});
    }
  }

  ~HikSingleCameraNode() override
  {
    if (worker_) {
      worker_->stopContinuous();
      worker_->disconnectCamera();
      worker_.reset();
    }
    {
      std::lock_guard<std::mutex> lock(publish_queue_mutex_);
      publisher_stopping_ = true;
      publish_queue_.clear();
    }
    publish_queue_condition_.notify_all();
    if (publisher_thread_.joinable()) {
      publisher_thread_.join();
    }
  }

private:
  void publish_connection_status(
    bool connected, bool streaming, const std::string & detail)
  {
    std_msgs::msg::String status;
    status.data = "connected=" + std::to_string(connected) +
      "; streaming=" + std::to_string(streaming) + "; " + detail;
    status_publisher_->publish(status);
  }

  void reset_camera_timing()
  {
    {
      std::lock_guard<std::mutex> queue_lock(publish_queue_mutex_);
      publish_queue_.clear();
    }
    std::lock_guard<std::mutex> lock(timing_mutex_);
    camera_clock_mapper_ = std::make_unique<hik_sync::CameraClockMapper>(
      static_cast<std::size_t>(camera_clock_mapping_window_),
      camera_mapping_max_residual_us_ * 1000.0);
    mapping_was_stable_ = false;
    mapping_warmup_drops_ = 0;
    non_monotonic_stamp_drops_ = 0;
    last_image_stamp_ns_ = 0;
  }

  void enqueue_frame(hik_sync::CameraFrame frame)
  {
    ++camera_frames_received_;
    if (frame.timestampValid && frame.cameraTimestampNs > 0) {
      std::int64_t expected = 0;
      first_device_timestamp_ns_.compare_exchange_strong(expected, frame.cameraTimestampNs);
      last_device_timestamp_ns_.store(frame.cameraTimestampNs);
    }
    {
      std::lock_guard<std::mutex> lock(publish_queue_mutex_);
      if (publisher_stopping_) {
        return;
      }
      if (publish_queue_.size() >= static_cast<std::size_t>(publish_queue_capacity_)) {
        // Keep latency bounded.  Discard the oldest frame that has not begun
        // publishing, then retain the newest exposure and its robot pose time.
        publish_queue_.pop_front();
        ++camera_publish_queue_drops_;
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "ROS image publish queue full; dropped oldest frame "
          "(capacity=%d, drops=%llu)", publish_queue_capacity_,
          static_cast<unsigned long long>(camera_publish_queue_drops_.load()));
      }
      publish_queue_.push_back(std::move(frame));
    }
    publish_queue_condition_.notify_one();
  }

  void publisher_worker()
  {
    while (true) {
      hik_sync::CameraFrame frame;
      {
        std::unique_lock<std::mutex> lock(publish_queue_mutex_);
        publish_queue_condition_.wait(
          lock, [this]() {return publisher_stopping_ || !publish_queue_.empty();});
        if (publisher_stopping_ && publish_queue_.empty()) {
          return;
        }
        frame = std::move(publish_queue_.front());
        publish_queue_.pop_front();
      }
      publish_frame(std::move(frame));
    }
  }

  double received_device_fps() const
  {
    const std::uint64_t count = camera_frames_received_.load();
    const std::int64_t first_ns = first_device_timestamp_ns_.load();
    const std::int64_t last_ns = last_device_timestamp_ns_.load();
    if (count < 2U || last_ns <= first_ns) {
      return 0.0;
    }
    return static_cast<double>(count - 1U) * 1.0e9 /
           static_cast<double>(last_ns - first_ns);
  }

  void set_camera_connected(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    std::unique_lock<std::mutex> lock(connection_mutex_);
    if (request->data && camera_connected_ && camera_streaming_) {
      response->success = true;
      response->message = "HIK camera is already connected and streaming";
      return;
    }
    if (!request->data && !camera_connected_) {
      response->success = true;
      response->message = "HIK camera is already disconnected";
      return;
    }

    const std::uint64_t initial_generation = connection_generation_;
    if (request->data) {
      const bool connected_without_stream = camera_connected_;
      lock.unlock();
      if (connected_without_stream) {
        stream_start_requested_.store(true);
        QMetaObject::invokeMethod(
          worker_.get(),
          [this]() {worker_->startContinuous(exposure_us_, gain_db_, fps_, pool_capacity_);},
          Qt::QueuedConnection);
      } else {
        QMetaObject::invokeMethod(
          worker_.get(),
          [this]() {worker_->connectCamera(QString::fromStdString(camera_ip_));},
          Qt::QueuedConnection);
      }
      lock.lock();
      const bool completed = connection_condition_.wait_for(
        lock, std::chrono::duration<double>(connection_timeout_s_),
        [this, initial_generation]() {
          return camera_streaming_ ||
                 (connection_generation_ > initial_generation && !camera_connected_);
        });
      response->success = completed && camera_connected_ && camera_streaming_;
      response->message = response->success ?
        "HIK camera connected and continuous acquisition confirmed" :
        "HIK camera connection/streaming was not confirmed: " + last_connection_detail_;
      return;
    }

    lock.unlock();
    QMetaObject::invokeMethod(
      worker_.get(),
      [this]() {
        worker_->stopContinuous();
        worker_->disconnectCamera();
      }, Qt::QueuedConnection);
    lock.lock();
    const bool completed = connection_condition_.wait_for(
      lock, std::chrono::duration<double>(connection_timeout_s_),
      [this, initial_generation]() {
        return connection_generation_ > initial_generation && !camera_connected_;
      });
    response->success = completed && !camera_connected_;
    response->message = response->success ?
      "HIK camera acquisition stopped and camera disconnected" :
      "HIK camera disconnect was not confirmed: " + last_connection_detail_;
  }

  std::optional<rclcpp::Time> synchronized_exposure_midpoint(
    const hik_sync::CameraFrame & frame)
  {
    std::lock_guard<std::mutex> timing_lock(timing_mutex_);
    const rclcpp::Time ros_now = now();
    const int64_t raw_now_ns = hik_sync::getMonotonicRawNs();
    if (raw_now_ns <= 0) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000, "CLOCK_MONOTONIC_RAW is unavailable");
      return std::nullopt;
    }

    bool mapped = false;
    int64_t reference_raw_ns = 0;
    if (frame.timestampValid && frame.cameraTimestampNs > 0 && camera_clock_mapper_) {
      camera_clock_mapper_->addSample(frame.cameraTimestampNs, frame.hostCallbackNs);
      mapped = camera_clock_mapper_->mapToHost(frame.cameraTimestampNs, &reference_raw_ns);
    }

    const hik_sync::ClockFitReport report = camera_clock_mapper_
      ? camera_clock_mapper_->report() : hik_sync::ClockFitReport{};
    if (mapped != mapping_was_stable_) {
      mapping_was_stable_ = mapped;
      std_msgs::msg::String status;
      std::ostringstream text;
      text << "connected=1; streaming=1; camera_time=" <<
        (mapped ? "DEVICE_TIMESTAMP_MAPPING" : "WARMING_UP")
           << "; samples=" << report.sampleCount
           << "; residual_us=" << std::fixed << std::setprecision(3)
           << report.residualRmsNs * 1.0e-3;
      status.data = text.str();
      status_publisher_->publish(status);
      if (mapped) {
        RCLCPP_INFO(get_logger(), "%s", status.data.c_str());
      } else {
        RCLCPP_WARN(get_logger(), "%s", status.data.c_str());
      }
    }

    hik_sync::CameraTimestampReference reference = camera_timestamp_reference_;
    if (!mapped) {
      if (require_device_timestamp_mapping_) {
        ++mapping_warmup_drops_;
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "camera frame withheld until device timestamp mapping is stable "
          "(samples=%zu, residual=%.3f us, dropped=%llu)",
          report.sampleCount, report.residualRmsNs * 1.0e-3,
          static_cast<unsigned long long>(mapping_warmup_drops_));
        return std::nullopt;
      }
      reference_raw_ns = frame.hostCallbackNs - static_cast<int64_t>(
        std::llround(camera_transport_delay_us_ * 1000.0));
      reference = hik_sync::CameraTimestampReference::Callback;
    }

    const double exposure = frame.exposureUs > 0.0 ? frame.exposureUs : exposure_us_;
    const hik_sync::ExposureTiming timing = hik_sync::computeExposureTiming(
      reference_raw_ns, exposure, camera_fixed_time_offset_us_, reference);
    if (!timing.valid) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "camera exposure timing is invalid");
      return std::nullopt;
    }

    // CameraClockMapper operates in CLOCK_MONOTONIC_RAW.  Anchor that clock
    // to this node's ROS clock at the callback, retaining the mapped exposure
    // midpoint instead of the network-arrival time.
    const int64_t aligned_ros_ns = ros_now.nanoseconds() +
      timing.alignedTimestampNs - raw_now_ns;
    if (aligned_ros_ns <= 0 || aligned_ros_ns > ros_now.nanoseconds() + 1000000LL) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "mapped exposure midpoint is invalid or more than 1 ms in the future");
      return std::nullopt;
    }
    if (last_image_stamp_ns_ > 0 && aligned_ros_ns <= last_image_stamp_ns_) {
      ++non_monotonic_stamp_drops_;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "non-monotonic mapped camera timestamp rejected (dropped=%llu)",
        static_cast<unsigned long long>(non_monotonic_stamp_drops_));
      return std::nullopt;
    }
    last_image_stamp_ns_ = aligned_ros_ns;
    return rclcpp::Time(aligned_ros_ns, get_clock()->get_clock_type());
  }

  sensor_msgs::msg::CameraInfo make_camera_info() const
  {
    sensor_msgs::msg::CameraInfo info;
    info.header.frame_id = frame_id_;
    info.width = static_cast<std::uint32_t>(intrinsics_.imageSize.width);
    info.height = static_cast<std::uint32_t>(intrinsics_.imageSize.height);
    info.distortion_model = "plumb_bob";
    info.d.reserve(intrinsics_.distCoeffs.total());
    const cv::Mat distortion = intrinsics_.distCoeffs.reshape(1, 1);
    for (int column = 0; column < distortion.cols; ++column) {
      info.d.push_back(distortion.at<double>(0, column));
    }
    for (int row = 0; row < 3; ++row) {
      for (int column = 0; column < 3; ++column) {
        info.k[static_cast<std::size_t>(row * 3 + column)] =
          intrinsics_.cameraMatrix.at<double>(row, column);
        info.r[static_cast<std::size_t>(row * 3 + column)] = row == column ? 1.0 : 0.0;
      }
    }
    info.p[0] = info.k[0];
    info.p[2] = info.k[2];
    info.p[5] = info.k[4];
    info.p[6] = info.k[5];
    info.p[10] = 1.0;
    return info;
  }

  void publish_frame(hik_sync::CameraFrame frame)
  {
    if (!identity_valid_.load() || !frame.image || frame.width <= 0 || frame.height <= 0) {
      return;
    }
    const std::size_t byte_count =
      static_cast<std::size_t>(frame.width) * static_cast<std::size_t>(frame.height);
    if (frame.image->bytes.size() < byte_count) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "camera frame buffer is short");
      return;
    }

    const std::optional<rclcpp::Time> synchronized_stamp =
      synchronized_exposure_midpoint(frame);
    if (!synchronized_stamp) {
      return;
    }
    const rclcpp::Time stamp = *synchronized_stamp;

    sensor_msgs::msg::Image image;
    image.header.stamp = stamp;
    image.header.frame_id = frame_id_;
    image.height = static_cast<std::uint32_t>(frame.height);
    image.width = static_cast<std::uint32_t>(frame.width);
    image.encoding = "mono8";
    image.is_bigendian = false;
    image.step = image.width;
    image.data.assign(frame.image->bytes.begin(), frame.image->bytes.begin() + byte_count);

    camera_info_.header = image.header;
    image_publisher_->publish(image);
    info_publisher_->publish(camera_info_);
    const std::uint64_t published = ++camera_frames_published_;
    if (published % 120U == 0U) {
      std_msgs::msg::String status;
      std::ostringstream text;
      text << "connected=1; streaming=1; camera_time=DEVICE_TIMESTAMP_MAPPING"
           << "; target_fps=" << std::fixed << std::setprecision(3) << fps_
           << "; device_fps=" << received_device_fps()
           << "; received=" << camera_frames_received_.load()
           << "; published=" << published
           << "; publish_queue_dropped=" << camera_publish_queue_drops_.load();
      status.data = text.str();
      status_publisher_->publish(status);
    }
  }

  std::string camera_ip_;
  std::string expected_model_;
  std::string expected_serial_;
  std::string frame_id_;
  std::string intrinsics_path_;
  double exposure_us_{1825.0};
  double gain_db_{0.0};
  double fps_{60.0};
  int pool_capacity_{32};
  int publish_queue_capacity_{16};
  int camera_clock_mapping_window_{600};
  double camera_mapping_max_residual_us_{2000.0};
  double camera_fixed_time_offset_us_{0.0};
  double camera_transport_delay_us_{0.0};
  double connection_timeout_s_{8.0};
  bool require_device_timestamp_mapping_{true};
  bool mapping_was_stable_{false};
  hik_sync::CameraTimestampReference camera_timestamp_reference_{
    hik_sync::CameraTimestampReference::ExposureStart};
  std::uint64_t mapping_warmup_drops_{0};
  std::uint64_t non_monotonic_stamp_drops_{0};
  int64_t last_image_stamp_ns_{0};
  std::atomic<std::uint64_t> camera_frames_received_{0U};
  std::atomic<std::uint64_t> camera_frames_published_{0U};
  std::atomic<std::uint64_t> camera_publish_queue_drops_{0U};
  std::atomic<std::int64_t> first_device_timestamp_ns_{0};
  std::atomic<std::int64_t> last_device_timestamp_ns_{0};
  std::atomic<bool> identity_valid_{false};
  std::atomic<bool> stream_start_requested_{false};
  std::mutex connection_mutex_;
  std::mutex timing_mutex_;
  std::condition_variable connection_condition_;
  std::mutex publish_queue_mutex_;
  std::condition_variable publish_queue_condition_;
  std::deque<hik_sync::CameraFrame> publish_queue_;
  std::thread publisher_thread_;
  bool publisher_stopping_{false};
  bool camera_connected_{false};
  bool camera_streaming_{false};
  std::uint64_t connection_generation_{0};
  std::string last_connection_detail_{"camera disconnected"};
  std::unique_ptr<hik_sync::CameraClockMapper> camera_clock_mapper_;
  hik_calibration::IntrinsicCalibrationResult intrinsics_;
  hik_calibration::IntrinsicsYamlMetadata intrinsics_metadata_;
  sensor_msgs::msg::CameraInfo camera_info_;
  std::unique_ptr<HikCameraWorker> worker_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr connection_service_;
};

}  // namespace

int main(int argc, char ** argv)
{
  QCoreApplication application(argc, argv);
  rclcpp::init(argc, argv);
  std::shared_ptr<HikSingleCameraNode> node;
  try {
    node = std::make_shared<HikSingleCameraNode>();
  } catch (const std::exception & exception) {
    std::fprintf(stderr, "hik_single_camera startup failed: %s\n", exception.what());
    rclcpp::shutdown();
    return 2;
  }
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread ros_thread([&executor]() {executor.spin();});
  QObject::connect(&application, &QCoreApplication::aboutToQuit, [&executor]() {executor.cancel();});
  QTimer shutdown_monitor;
  QObject::connect(&shutdown_monitor, &QTimer::timeout, [&application]() {
    if (!rclcpp::ok()) {application.quit();}
  });
  shutdown_monitor.start(100);
  const int result = application.exec();
  executor.cancel();
  ros_thread.join();
  node.reset();
  rclcpp::shutdown();
  return result;
}
