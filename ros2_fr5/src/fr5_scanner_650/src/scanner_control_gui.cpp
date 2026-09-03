#include <rclcpp/parameter_client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <QApplication>
#include <QCheckBox>
#include <QCloseEvent>
#include <QComboBox>
#include <QDateTime>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QFrame>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QMainWindow>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QPointer>
#include <QProgressBar>
#include <QPushButton>
#include <QScrollArea>
#include <QStyle>
#include <QTabWidget>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace
{

using Trigger = std_srvs::srv::Trigger;
using SetBool = std_srvs::srv::SetBool;

struct LinearScanInput
{
  std::string frame{"camera"};
  std::string axis{"+y"};
  double distance_m{0.05};
  double velocity_scaling{0.20};
  double acceleration_scaling{0.20};
  bool maintain_base_height{true};
};

struct WorkspaceScanInput
{
  double azimuth_min_deg{-60.0};
  double azimuth_max_deg{60.0};
  double radial_min_m{0.0};
  double radial_max_m{0.72};
  double scan_speed_m_s{0.01};
  double transition_speed_m_s{0.01};
  double acceleration_scaling{0.30};
  bool return_to_start{true};
  double return_velocity_scaling{0.10};
  double return_acceleration_scaling{0.10};
};

struct Availability
{
  bool robot_state{false};
  bool move_services{false};
  bool camera_connected{false};
  bool camera_streaming{false};
  bool camera_connection_service{false};
  bool laser_connected{false};
  bool laser_connection_service{false};
  bool reconstruction{false};
};

int64_t steadyNowNs()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

class ScannerControlBridge final : public rclcpp::Node
{
public:
  using EventCallback = std::function<void (const std::string &, const std::string &, bool)>;
  using TopicCallback = std::function<void (const std::string &, const std::string &)>;
  using AvailabilityCallback = std::function<void (const Availability &)>;
  using JointCallback = std::function<void (const std::array<double, 6> &)>;
  using CloudCallback = std::function<void (std::size_t)>;

  ScannerControlBridge()
  : Node("scanner_650_control_gui")
  {
    workspace_parameters_ = std::make_shared<rclcpp::AsyncParametersClient>(
      this, "/workspace_coarse_scan_planner");
    linear_parameters_ = std::make_shared<rclcpp::AsyncParametersClient>(
      this, "/scan_motion_commander");

    plan_workspace_client_ = create_client<Trigger>(
      "/scanner_650/plan_workspace_coarse_scan");
    approve_workspace_client_ = create_client<SetBool>(
      "/scanner_650/approve_workspace_coarse_scan");
    execute_workspace_client_ = create_client<Trigger>(
      "/scanner_650/execute_workspace_coarse_scan");
    stop_workspace_client_ = create_client<Trigger>(
      "/scanner_650/stop_workspace_coarse_scan");

    plan_linear_client_ = create_client<Trigger>("/scanner_650/plan_linear_scan");
    execute_linear_client_ = create_client<Trigger>("/scanner_650/execute_last_plan");
    stop_motion_client_ = create_client<Trigger>("/scanner_650/stop_motion");

    clear_cloud_client_ = create_client<Trigger>("/scanner_650/clear_cloud");
    save_cloud_client_ = create_client<Trigger>("/scanner_650/save_cloud");
    accumulation_client_ = create_client<SetBool>("/scanner_650/set_accumulation");
    laser_client_ = create_client<SetBool>("/scanner_650/set_laser");
    camera_connection_client_ = create_client<SetBool>(
      "/scanner_650/set_camera_connected");
    laser_connection_client_ = create_client<SetBool>(
      "/scanner_650/set_laser_connected");
    rviz_mode_client_ = create_client<SetBool>("/scanner_650/set_rviz_scan_mode");

    joint_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::JointState::ConstSharedPtr message) {
        std::array<double, 6> joints;
        joints.fill(std::numeric_limits<double>::quiet_NaN());
        for (std::size_t i = 0; i < message->name.size() && i < message->position.size(); ++i) {
          for (std::size_t joint = 0; joint < joints.size(); ++joint) {
            const std::string expected = "j" + std::to_string(joint + 1U);
            if (message->name[i] == expected) {
              joints[joint] = message->position[i] * 180.0 / 3.14159265358979323846;
            }
          }
        }
        last_joint_ns_.store(steadyNowNs());
        if (joint_callback_) {
          joint_callback_(joints);
        }
      });

    cloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/scanner_650/scan_cloud", rclcpp::QoS(1).reliable().transient_local(),
      [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {
        const std::size_t count = static_cast<std::size_t>(message->width) * message->height;
        if (cloud_callback_) {
          cloud_callback_(count);
        }
      });

    camera_info_subscription_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      "/scanner_650/camera_info", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::CameraInfo::ConstSharedPtr) {
        last_camera_ns_.store(steadyNowNs());
        camera_connected_.store(true);
        camera_streaming_.store(true);
      });

    subscribeStatus("/scanner_650/camera_status", "camera", true);
    subscribeStatus("/scanner_650/laser_status", "laser", true);
    subscribeStatus("/scanner_650/reconstruction_status", "reconstruction", false);
    subscribeStatus("/scanner_650/workspace_coarse_scan_status", "workspace", true);
  }

  void setEventCallback(EventCallback callback) {event_callback_ = std::move(callback);}
  void setTopicCallback(TopicCallback callback) {topic_callback_ = std::move(callback);}
  void setAvailabilityCallback(AvailabilityCallback callback)
  {
    availability_callback_ = std::move(callback);
  }
  void setJointCallback(JointCallback callback) {joint_callback_ = std::move(callback);}
  void setCloudCallback(CloudCallback callback) {cloud_callback_ = std::move(callback);}

  void planWorkspace(const WorkspaceScanInput & input)
  {
    std::vector<rclcpp::Parameter> parameters{
      rclcpp::Parameter("azimuth_min_deg", input.azimuth_min_deg),
      rclcpp::Parameter("azimuth_max_deg", input.azimuth_max_deg),
      rclcpp::Parameter("radial_min_m", input.radial_min_m),
      rclcpp::Parameter("radial_max_m", input.radial_max_m),
      rclcpp::Parameter("scan_speed_m_s", input.scan_speed_m_s),
      rclcpp::Parameter("transition_speed_m_s", input.transition_speed_m_s),
      rclcpp::Parameter("acceleration_scaling", input.acceleration_scaling),
      rclcpp::Parameter("return_to_start_after_scan", input.return_to_start),
      rclcpp::Parameter("return_velocity_scaling", input.return_velocity_scaling),
      rclcpp::Parameter("return_acceleration_scaling", input.return_acceleration_scaling)};
    setParametersThenTrigger(
      workspace_parameters_, parameters, plan_workspace_client_, "规划工作区粗扫");
  }

  void approveWorkspace(bool approved)
  {
    callSetBool(approve_workspace_client_, approved, "批准工作区粗扫");
  }

  void executeWorkspace()
  {
    callTrigger(execute_workspace_client_, "执行工作区粗扫");
  }

  void planLinear(const LinearScanInput & input)
  {
    std::vector<rclcpp::Parameter> parameters{
      rclcpp::Parameter("direction_frame", input.frame),
      rclcpp::Parameter("direction_axis", input.axis),
      rclcpp::Parameter("distance_m", input.distance_m),
      rclcpp::Parameter("velocity_scaling", input.velocity_scaling),
      rclcpp::Parameter("acceleration_scaling", input.acceleration_scaling),
      rclcpp::Parameter("maintain_base_height", input.maintain_base_height)};
    setParametersThenTrigger(
      linear_parameters_, parameters, plan_linear_client_, "规划固定轴直线扫描");
  }

  void executeLinear()
  {
    callTrigger(execute_linear_client_, "执行固定轴直线扫描");
  }

  void clearCloud() {callTrigger(clear_cloud_client_, "清空点云");}
  void saveCloud() {callTrigger(save_cloud_client_, "保存点云");}
  void setAccumulation(bool enabled)
  {
    callSetBool(accumulation_client_, enabled, enabled ? "开始点云累积" : "停止点云累积");
  }
  void setLaser(bool enabled)
  {
    callSetBool(laser_client_, enabled, enabled ? "打开 650 nm 激光" : "关闭全部激光");
  }
  void setCameraConnected(bool connected)
  {
    callSetBool(
      camera_connection_client_, connected,
      connected ? "连接海康相机" : "断开海康相机");
  }
  void setLaserConnected(bool connected)
  {
    callSetBool(
      laser_connection_client_, connected,
      connected ? "连接激光控制器" : "断开激光控制器");
  }
  void setRvizScanMode(bool enabled)
  {
    callSetBool(rviz_mode_client_, enabled, enabled ? "RViz 扫描模式" : "RViz 定位模式");
  }

  void requestSafeStop()
  {
    notifyEvent("安全停止", "已向运动、粗扫、点云累积和激光控制发送停止请求", true);
    callTriggerIfReady(stop_workspace_client_, "停止工作区粗扫");
    callTriggerIfReady(stop_motion_client_, "停止 MoveIt 运动");
    callSetBoolIfReady(accumulation_client_, false, "停止点云累积");
    callSetBoolIfReady(laser_client_, false, "关闭全部激光");
  }

  void refreshAvailability()
  {
    const int64_t now = steadyNowNs();
    constexpr int64_t recent_ns = 3000000000LL;
    Availability value;
    value.robot_state = now - last_joint_ns_.load() < recent_ns;
    value.camera_connected = camera_connected_.load();
    value.camera_streaming = value.camera_connected && camera_streaming_.load() &&
      now - last_camera_ns_.load() < 5000000000LL;
    value.camera_connection_service = camera_connection_client_->service_is_ready();
    value.move_services = workspace_parameters_->service_is_ready() &&
      linear_parameters_->service_is_ready() && plan_workspace_client_->service_is_ready() &&
      plan_linear_client_->service_is_ready();
    value.laser_connected = laser_connected_.load();
    value.laser_connection_service = laser_connection_client_->service_is_ready();
    value.reconstruction = clear_cloud_client_->service_is_ready() &&
      save_cloud_client_->service_is_ready();
    if (availability_callback_) {
      availability_callback_(value);
    }
  }

private:
  void subscribeStatus(
    const std::string & topic, const std::string & key, bool transient)
  {
    const rclcpp::QoS qos = transient ?
      rclcpp::QoS(10).reliable().transient_local() : rclcpp::QoS(10).reliable();
    status_subscriptions_.push_back(
      create_subscription<std_msgs::msg::String>(
        topic, qos,
        [this, key](
          const std_msgs::msg::String::ConstSharedPtr message) {
          if (key == "camera") {
            if (message->data.find("connected=1") != std::string::npos) {
              camera_connected_.store(true);
            } else if (message->data.find("connected=0") != std::string::npos) {
              camera_connected_.store(false);
              camera_streaming_.store(false);
            }
            if (message->data.find("streaming=1") != std::string::npos) {
              camera_streaming_.store(true);
            } else if (message->data.find("streaming=0") != std::string::npos) {
              camera_streaming_.store(false);
            }
          } else if (key == "laser") {
            if (message->data.find("connection=ready") != std::string::npos ||
              message->data.find("connection=command_pending") != std::string::npos)
            {
              laser_connected_.store(true);
            } else if (message->data.find("connection=disconnected") != std::string::npos ||
              message->data.find("connection=fault") != std::string::npos)
            {
              laser_connected_.store(false);
            }
          }
          if (topic_callback_) {
            topic_callback_(key, message->data);
          }
        }));
  }

  void setParametersThenTrigger(
    const rclcpp::AsyncParametersClient::SharedPtr & parameter_client,
    const std::vector<rclcpp::Parameter> & parameters,
    const rclcpp::Client<Trigger>::SharedPtr & service_client,
    const std::string & operation)
  {
    if (!parameter_client->service_is_ready()) {
      notifyEvent(operation, "参数服务不可用，请确认对应规划节点已经启动", false);
      return;
    }
    parameter_client->set_parameters_atomically(
      parameters,
      [this, service_client, operation](
        std::shared_future<rcl_interfaces::msg::SetParametersResult> future) {
        try {
          const auto result = future.get();
          if (!result.successful) {
            notifyEvent(operation, "参数被节点拒绝：" + result.reason, false);
            return;
          }
          callTrigger(service_client, operation);
        } catch (const std::exception & exception) {
          notifyEvent(operation, "设置参数失败：" + std::string(exception.what()), false);
        }
      });
  }

  void callTrigger(
    const rclcpp::Client<Trigger>::SharedPtr & client, const std::string & operation)
  {
    if (!client->service_is_ready()) {
      notifyEvent(operation, "服务不可用，请检查 ROS 2 启动状态", false);
      return;
    }
    auto request = std::make_shared<Trigger::Request>();
    client->async_send_request(
      request,
      [this, operation](rclcpp::Client<Trigger>::SharedFuture future) {
        try {
          const auto response = future.get();
          notifyEvent(operation, response->message, response->success);
        } catch (const std::exception & exception) {
          notifyEvent(operation, "服务调用失败：" + std::string(exception.what()), false);
        }
      });
  }

  void callSetBool(
    const rclcpp::Client<SetBool>::SharedPtr & client, bool enabled,
    const std::string & operation)
  {
    if (!client->service_is_ready()) {
      notifyEvent(operation, "服务不可用，请检查对应节点是否启动", false);
      return;
    }
    auto request = std::make_shared<SetBool::Request>();
    request->data = enabled;
    client->async_send_request(
      request,
      [this, operation](rclcpp::Client<SetBool>::SharedFuture future) {
        try {
          const auto response = future.get();
          notifyEvent(operation, response->message, response->success);
        } catch (const std::exception & exception) {
          notifyEvent(operation, "服务调用失败：" + std::string(exception.what()), false);
        }
      });
  }

  void callTriggerIfReady(
    const rclcpp::Client<Trigger>::SharedPtr & client, const std::string & operation)
  {
    if (client->service_is_ready()) {
      callTrigger(client, operation);
    } else {
      notifyEvent(operation, "服务未运行，已跳过", false);
    }
  }

  void callSetBoolIfReady(
    const rclcpp::Client<SetBool>::SharedPtr & client, bool enabled,
    const std::string & operation)
  {
    if (client->service_is_ready()) {
      callSetBool(client, enabled, operation);
    } else {
      notifyEvent(operation, "服务未运行，已跳过", false);
    }
  }

  void notifyEvent(
    const std::string & operation, const std::string & message, bool success)
  {
    if (event_callback_) {
      event_callback_(operation, message, success);
    }
  }

  EventCallback event_callback_;
  TopicCallback topic_callback_;
  AvailabilityCallback availability_callback_;
  JointCallback joint_callback_;
  CloudCallback cloud_callback_;

  rclcpp::AsyncParametersClient::SharedPtr workspace_parameters_;
  rclcpp::AsyncParametersClient::SharedPtr linear_parameters_;
  rclcpp::Client<Trigger>::SharedPtr plan_workspace_client_;
  rclcpp::Client<SetBool>::SharedPtr approve_workspace_client_;
  rclcpp::Client<Trigger>::SharedPtr execute_workspace_client_;
  rclcpp::Client<Trigger>::SharedPtr stop_workspace_client_;
  rclcpp::Client<Trigger>::SharedPtr plan_linear_client_;
  rclcpp::Client<Trigger>::SharedPtr execute_linear_client_;
  rclcpp::Client<Trigger>::SharedPtr stop_motion_client_;
  rclcpp::Client<Trigger>::SharedPtr clear_cloud_client_;
  rclcpp::Client<Trigger>::SharedPtr save_cloud_client_;
  rclcpp::Client<SetBool>::SharedPtr accumulation_client_;
  rclcpp::Client<SetBool>::SharedPtr laser_client_;
  rclcpp::Client<SetBool>::SharedPtr camera_connection_client_;
  rclcpp::Client<SetBool>::SharedPtr laser_connection_client_;
  rclcpp::Client<SetBool>::SharedPtr rviz_mode_client_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscription_;
  std::vector<rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> status_subscriptions_;
  std::atomic<int64_t> last_joint_ns_{0};
  std::atomic<int64_t> last_camera_ns_{0};
  std::atomic<bool> camera_connected_{false};
  std::atomic<bool> camera_streaming_{false};
  std::atomic<bool> laser_connected_{false};
};

class ScannerControlWindow final : public QMainWindow
{
public:
  explicit ScannerControlWindow(std::shared_ptr<ScannerControlBridge> bridge)
  : bridge_(std::move(bridge))
  {
    setWindowTitle(QStringLiteral("FR5 线激光扫描操作台"));
    setMinimumSize(1120, 760);
    resize(1260, 860);
    buildUi();
    applyStyle();
    bindBridge();

    availability_timer_ = new QTimer(this);
    availability_timer_->setInterval(1000);
    connect(
      availability_timer_, &QTimer::timeout, this, [this]() {
        bridge_->refreshAvailability();
      });
    availability_timer_->start();
    bridge_->refreshAvailability();
    appendLog(QStringLiteral("INFO"), QStringLiteral("控制台已启动，等待 ROS 2 接口就绪"));
  }

private:
  static QDoubleSpinBox * spin(
    double minimum, double maximum, double value, const QString & suffix,
    int decimals = 1, double step = 1.0)
  {
    auto * box = new QDoubleSpinBox();
    box->setRange(minimum, maximum);
    box->setValue(value);
    box->setDecimals(decimals);
    box->setSingleStep(step);
    box->setSuffix(suffix);
    box->setMinimumHeight(36);
    box->setKeyboardTracking(false);
    return box;
  }

  QLabel * makeStatusCard(QGridLayout * layout, int column, const QString & title)
  {
    auto * frame = new QFrame();
    frame->setObjectName(QStringLiteral("statusCard"));
    auto * card_layout = new QVBoxLayout(frame);
    card_layout->setContentsMargins(14, 10, 14, 10);
    card_layout->setSpacing(3);
    auto * title_label = new QLabel(title);
    title_label->setObjectName(QStringLiteral("statusTitle"));
    auto * value = new QLabel(QStringLiteral("● 等待连接"));
    value->setObjectName(QStringLiteral("statusValue"));
    value->setProperty("online", false);
    card_layout->addWidget(title_label);
    card_layout->addWidget(value);
    layout->addWidget(frame, 0, column);
    return value;
  }

  QPushButton * button(const QString & text, const char * kind)
  {
    auto * result = new QPushButton(text);
    result->setProperty("kind", kind);
    result->setMinimumHeight(40);
    result->setCursor(Qt::PointingHandCursor);
    return result;
  }

  void buildUi()
  {
    auto * central = new QWidget();
    auto * root = new QVBoxLayout(central);
    root->setContentsMargins(22, 18, 22, 20);
    root->setSpacing(14);

    auto * header = new QHBoxLayout();
    auto * header_text = new QVBoxLayout();
    auto * title = new QLabel(QStringLiteral("FR5 · 线激光扫描操作台"));
    title->setObjectName(QStringLiteral("pageTitle"));
    auto * subtitle = new QLabel(
      QStringLiteral("运动指令在此操作；RViz / MoveIt 仅用于轨迹、碰撞场景与点云确认"));
    subtitle->setObjectName(QStringLiteral("pageSubtitle"));
    header_text->addWidget(title);
    header_text->addWidget(subtitle);
    auto * safety_hint = new QLabel(QStringLiteral("真实急停请使用硬件 E-Stop"));
    safety_hint->setObjectName(QStringLiteral("safetyHint"));
    header->addLayout(header_text, 1);
    header->addWidget(safety_hint, 0, Qt::AlignTop);
    root->addLayout(header);

    auto * status_grid = new QGridLayout();
    status_grid->setHorizontalSpacing(10);
    robot_badge_ = makeStatusCard(status_grid, 0, QStringLiteral("机械臂状态"));
    moveit_badge_ = makeStatusCard(status_grid, 1, QStringLiteral("MoveIt 规划"));
    camera_badge_ = makeStatusCard(status_grid, 2, QStringLiteral("HIK 相机"));
    laser_badge_ = makeStatusCard(status_grid, 3, QStringLiteral("激光联锁"));
    cloud_badge_ = makeStatusCard(status_grid, 4, QStringLiteral("点云重建 · base_link"));
    for (int column = 0; column < 5; ++column) {
      status_grid->setColumnStretch(column, 1);
    }
    root->addLayout(status_grid);

    auto * content = new QHBoxLayout();
    content->setSpacing(14);
    tabs_ = new QTabWidget();
    tabs_->addTab(buildWorkspaceTab(), QStringLiteral("工作区扇形粗扫"));
    tabs_->addTab(buildLinearTab(), QStringLiteral("相机坐标系定高线扫"));
    content->addWidget(tabs_, 3);
    content->addWidget(buildMonitorPanel(), 2);
    root->addLayout(content, 1);

    setCentralWidget(central);
  }

  QWidget * buildWorkspaceTab()
  {
    auto * scroll = new QScrollArea();
    scroll->setWidgetResizable(true);
    scroll->setFrameShape(QFrame::NoFrame);
    auto * page = new QWidget();
    auto * layout = new QVBoxLayout(page);
    layout->setContentsMargins(16, 16, 16, 16);
    layout->setSpacing(14);

    auto * geometry = new QGroupBox(QStringLiteral("1  扫描区域（相对当前示教 TCP）"));
    auto * geometry_form = new QFormLayout(geometry);
    geometry_form->setRowWrapPolicy(QFormLayout::DontWrapRows);
    azimuth_min_ = spin(-90.0, 0.0, -60.0, QStringLiteral(" °"), 1, 5.0);
    azimuth_max_ = spin(0.0, 90.0, 60.0, QStringLiteral(" °"), 1, 5.0);
    radial_min_ = spin(0.0, 500.0, 0.0, QStringLiteral(" mm"), 0, 10.0);
    radial_max_ = spin(100.0, 720.0, 720.0, QStringLiteral(" mm"), 0, 10.0);
    geometry_form->addRow(QStringLiteral("左边界角"), azimuth_min_);
    geometry_form->addRow(QStringLiteral("右边界角"), azimuth_max_);
    geometry_form->addRow(QStringLiteral("起始行程"), radial_min_);
    geometry_form->addRow(QStringLiteral("最大行程"), radial_max_);
    auto * geometry_tip = new QLabel(
      QStringLiteral("相机 TCP +Z 必须朝下；TCP +X 定义扇区中心。系统最多生成 3 条径向扫描线。"));
    geometry_tip->setWordWrap(true);
    geometry_tip->setObjectName(QStringLiteral("helpText"));
    geometry_form->addRow(geometry_tip);

    auto * motion = new QGroupBox(QStringLiteral("2  运动参数"));
    auto * motion_form = new QFormLayout(motion);
    scan_speed_ = spin(1.0, 50.0, 10.0, QStringLiteral(" mm/s"), 1, 1.0);
    transition_speed_ = spin(1.0, 50.0, 10.0, QStringLiteral(" mm/s"), 1, 1.0);
    workspace_acceleration_ = spin(1.0, 50.0, 30.0, QStringLiteral(" %"), 0, 5.0);
    return_to_start_ = new QCheckBox(QStringLiteral("扫描完成后关光并返回规划起点"));
    return_to_start_->setChecked(true);
    return_speed_ = spin(1.0, 50.0, 10.0, QStringLiteral(" %"), 0, 5.0);
    return_acceleration_ = spin(1.0, 50.0, 10.0, QStringLiteral(" %"), 0, 5.0);
    motion_form->addRow(QStringLiteral("扫描速度"), scan_speed_);
    motion_form->addRow(QStringLiteral("转场速度"), transition_speed_);
    motion_form->addRow(QStringLiteral("LIN 加速度"), workspace_acceleration_);
    motion_form->addRow(return_to_start_);
    motion_form->addRow(QStringLiteral("返程 PTP 速度"), return_speed_);
    motion_form->addRow(QStringLiteral("返程 PTP 加速度"), return_acceleration_);

    auto * workflow = new QGroupBox(QStringLiteral("3  规划、确认与执行"));
    auto * workflow_layout = new QVBoxLayout(workflow);
    auto * actions = new QHBoxLayout();
    workspace_plan_button_ = button(QStringLiteral("生成轨迹并在 RViz 预览"), "primary");
    workspace_approve_button_ = button(QStringLiteral("批准当前轨迹"), "secondary");
    workspace_execute_button_ = button(QStringLiteral("执行扫描"), "danger");
    workspace_approve_button_->setEnabled(false);
    workspace_execute_button_->setEnabled(false);
    actions->addWidget(workspace_plan_button_, 2);
    actions->addWidget(workspace_approve_button_, 1);
    actions->addWidget(workspace_execute_button_, 1);
    workspace_review_ = new QCheckBox(
      QStringLiteral("我已在 RViz 和现场检查绿色轨迹、碰撞体、扫描头线缆、净空与急停"));
    workspace_review_->setEnabled(false);
    workspace_review_->setObjectName(QStringLiteral("reviewCheck"));
    workflow_layout->addLayout(actions);
    workflow_layout->addWidget(workspace_review_);

    layout->addWidget(geometry);
    layout->addWidget(motion);
    layout->addWidget(workflow);
    layout->addStretch();
    scroll->setWidget(page);

    const std::vector<QDoubleSpinBox *> controls{
      azimuth_min_, azimuth_max_, radial_min_, radial_max_, scan_speed_, transition_speed_,
      workspace_acceleration_, return_speed_, return_acceleration_};
    for (auto * control : controls) {
      connect(
        control, qOverload<double>(&QDoubleSpinBox::valueChanged), this,
        [this](double) {invalidateWorkspacePlan();});
    }
    connect(
      return_to_start_, &QCheckBox::toggled, this, [this](bool enabled) {
        return_speed_->setEnabled(enabled);
        return_acceleration_->setEnabled(enabled);
        invalidateWorkspacePlan();
      });
    connect(
      workspace_review_, &QCheckBox::toggled, this, [this](bool checked) {
        workspace_approve_button_->setEnabled(workspace_planned_ && checked && !busy_);
      });
    connect(workspace_plan_button_, &QPushButton::clicked, this, [this]() {planWorkspace();});
    connect(
      workspace_approve_button_, &QPushButton::clicked, this, [this]() {
        beginOperation(QStringLiteral("批准工作区粗扫"));
        bridge_->approveWorkspace(true);
      });
    connect(
      workspace_execute_button_, &QPushButton::clicked, this, [this]() {
        if (QMessageBox::warning(
          this, QStringLiteral("确认执行真实运动"),
          QStringLiteral("即将执行已批准的扫描轨迹。请确认人员已离开工作区，安全门、急停、碰撞场景、负载和 TCP 均已检查。\n\n继续执行？"),
          QMessageBox::Yes | QMessageBox::No, QMessageBox::No) == QMessageBox::Yes)
        {
          beginOperation(QStringLiteral("执行工作区粗扫"));
          bridge_->executeWorkspace();
        }
      });
    return scroll;
  }

  QWidget * buildLinearTab()
  {
    auto * page = new QWidget();
    auto * layout = new QVBoxLayout(page);
    layout->setContentsMargins(16, 16, 16, 16);
    layout->setSpacing(14);

    auto * parameters = new QGroupBox(QStringLiteral("相机坐标系定高 Pilz LIN 参数"));
    auto * form = new QFormLayout(parameters);
    linear_frame_ = new QComboBox();
    linear_frame_->addItem(
      QStringLiteral("HIK 相机坐标系（hik_camera_optical_frame）"), QStringLiteral("camera"));
    linear_axis_ = new QComboBox();
    linear_axis_->addItem(QStringLiteral("相机 +Y（名义扫描方向）"), QStringLiteral("+y"));
    linear_axis_->addItem(QStringLiteral("相机 -Y（反向扫描）"), QStringLiteral("-y"));
    linear_distance_ = spin(1.0, 500.0, 50.0, QStringLiteral(" mm"), 1, 5.0);
    linear_velocity_ = spin(1.0, 100.0, 20.0, QStringLiteral(" %"), 0, 5.0);
    linear_acceleration_ = spin(1.0, 100.0, 20.0, QStringLiteral(" %"), 0, 5.0);
    form->addRow(QStringLiteral("扫描参考系"), linear_frame_);
    form->addRow(QStringLiteral("水平扫描方向"), linear_axis_);
    form->addRow(QStringLiteral("有符号距离"), linear_distance_);
    form->addRow(QStringLiteral("速度比例"), linear_velocity_);
    form->addRow(QStringLiteral("加速度比例"), linear_acceleration_);
    auto * tip = new QLabel(
      QStringLiteral(
        "相机 ±Y 垂直于线激光条纹。规划时将该方向投影到 base_link 的 XY 平面，"
        "因此相机与激光扫描头在整条路径上保持同一 base_link 高度；点云按逐帧 TF 换算到 base_link。"
        "执行时仍按“开激光 → 开始累积 → 运动 → 停止累积 → 确认关光”的顺序联锁。"));
    tip->setWordWrap(true);
    tip->setObjectName(QStringLiteral("helpText"));
    form->addRow(tip);

    auto * actions = new QGroupBox(QStringLiteral("规划与执行"));
    auto * actions_layout = new QVBoxLayout(actions);
    auto * buttons = new QHBoxLayout();
    linear_plan_button_ = button(QStringLiteral("规划并在 RViz 检查"), "primary");
    linear_execute_button_ = button(QStringLiteral("执行直线扫描"), "danger");
    linear_execute_button_->setEnabled(false);
    buttons->addWidget(linear_plan_button_, 2);
    buttons->addWidget(linear_execute_button_, 1);
    linear_review_ = new QCheckBox(
      QStringLiteral("我已检查本次直线轨迹、方向、距离、碰撞和现场净空"));
    linear_review_->setEnabled(false);
    linear_review_->setObjectName(QStringLiteral("reviewCheck"));
    actions_layout->addLayout(buttons);
    actions_layout->addWidget(linear_review_);

    layout->addWidget(parameters);
    layout->addWidget(actions);
    layout->addStretch();

    const std::vector<QDoubleSpinBox *> controls{
      linear_distance_, linear_velocity_, linear_acceleration_};
    for (auto * control : controls) {
      connect(
        control, qOverload<double>(&QDoubleSpinBox::valueChanged), this,
        [this](double) {invalidateLinearPlan();});
    }
    connect(
      linear_frame_, qOverload<int>(&QComboBox::currentIndexChanged), this,
      [this](int) {invalidateLinearPlan();});
    connect(
      linear_axis_, qOverload<int>(&QComboBox::currentIndexChanged), this,
      [this](int) {invalidateLinearPlan();});
    connect(
      linear_review_, &QCheckBox::toggled, this, [this](bool checked) {
        linear_execute_button_->setEnabled(linear_planned_ && checked && !busy_);
      });
    connect(linear_plan_button_, &QPushButton::clicked, this, [this]() {planLinear();});
    connect(
      linear_execute_button_, &QPushButton::clicked, this, [this]() {
        if (QMessageBox::warning(
          this, QStringLiteral("确认执行直线扫描"),
          QStringLiteral("即将让机械臂沿已规划的直线运动并打开线激光。确认现场安全后继续。"),
          QMessageBox::Yes | QMessageBox::No, QMessageBox::No) == QMessageBox::Yes)
        {
          beginOperation(QStringLiteral("执行固定轴直线扫描"));
          bridge_->executeLinear();
        }
      });
    return page;
  }

  QWidget * buildMonitorPanel()
  {
    auto * panel = new QFrame();
    panel->setObjectName(QStringLiteral("sidePanel"));
    auto * layout = new QVBoxLayout(panel);
    layout->setContentsMargins(16, 16, 16, 16);
    layout->setSpacing(12);

    auto * device_title = new QLabel(QStringLiteral("设备连接"));
    device_title->setObjectName(QStringLiteral("sectionTitle"));
    auto * device_grid = new QGridLayout();
    device_grid->setColumnStretch(1, 1);
    device_grid->setColumnStretch(2, 1);
    device_grid->addWidget(new QLabel(QStringLiteral("HIK 相机")), 0, 0);
    camera_connect_button_ = button(QStringLiteral("连接相机"), "primary");
    camera_disconnect_button_ = button(QStringLiteral("断开相机"), "secondary");
    camera_connect_button_->setToolTip(
      QStringLiteral("需要 hik_single_camera 节点提供连接服务"));
    camera_disconnect_button_->setToolTip(
      QStringLiteral("停止连续取流并断开 HIK 相机"));
    device_grid->addWidget(camera_connect_button_, 0, 1);
    device_grid->addWidget(camera_disconnect_button_, 0, 2);
    device_grid->addWidget(new QLabel(QStringLiteral("激光控制")), 1, 0);
    laser_connect_button_ = button(QStringLiteral("连接控制器"), "primary");
    laser_disconnect_button_ = button(QStringLiteral("断开控制器"), "secondary");
    laser_connect_button_->setToolTip(
      QStringLiteral("需要 line_laser_control 节点提供连接服务"));
    laser_disconnect_button_->setToolTip(
      QStringLiteral("确认两路 TTL 关闭后断开控制通道"));
    device_grid->addWidget(laser_connect_button_, 1, 1);
    device_grid->addWidget(laser_disconnect_button_, 1, 2);
    connect(
      camera_connect_button_, &QPushButton::clicked, this, [this]() {
        beginOperation(QStringLiteral("连接海康相机"));
        bridge_->setCameraConnected(true);
      });
    connect(
      camera_disconnect_button_, &QPushButton::clicked, this, [this]() {
        if (QMessageBox::question(
          this, QStringLiteral("断开海康相机"),
          QStringLiteral("断开后图像流停止，后续帧不会进入点云。确认当前没有扫描任务后继续。"),
          QMessageBox::Yes | QMessageBox::No, QMessageBox::No) == QMessageBox::Yes)
        {
          beginOperation(QStringLiteral("断开海康相机"));
          bridge_->setCameraConnected(false);
        }
      });
    connect(
      laser_connect_button_, &QPushButton::clicked, this, [this]() {
        beginOperation(QStringLiteral("连接激光控制器"));
        bridge_->setLaserConnected(true);
      });
    connect(
      laser_disconnect_button_, &QPushButton::clicked, this, [this]() {
        if (QMessageBox::warning(
          this, QStringLiteral("断开线激光控制器"),
          QStringLiteral(
            "后端将先关闭两路 TTL，并且只有收到关光读回确认后才断开控制通道。是否继续？"),
          QMessageBox::Yes | QMessageBox::No, QMessageBox::No) == QMessageBox::Yes)
        {
          beginOperation(QStringLiteral("断开激光控制器"));
          bridge_->setLaserConnected(false);
        }
      });
    layout->addWidget(device_title);
    layout->addLayout(device_grid);

    auto * stop_title = new QLabel(QStringLiteral("安全操作"));
    stop_title->setObjectName(QStringLiteral("sectionTitle"));
    safe_stop_button_ = button(QStringLiteral("■  停止运动并关激光"), "stop");
    safe_stop_button_->setMinimumHeight(54);
    connect(
      safe_stop_button_, &QPushButton::clicked, this, [this]() {
        appendLog(QStringLiteral("WARN"), QStringLiteral("正在请求软件停止；紧急情况请同时按硬件急停"));
        bridge_->requestSafeStop();
      });
    layout->addWidget(stop_title);
    layout->addWidget(safe_stop_button_);

    auto * utility_grid = new QGridLayout();
    auto * laser_off = button(QStringLiteral("关闭激光"), "secondary");
    auto * accumulation_off = button(QStringLiteral("停止累积"), "secondary");
    auto * clear_cloud = button(QStringLiteral("清空点云"), "secondary");
    auto * save_cloud = button(QStringLiteral("保存 PLY"), "secondary");
    utility_grid->addWidget(laser_off, 0, 0);
    utility_grid->addWidget(accumulation_off, 0, 1);
    utility_grid->addWidget(clear_cloud, 1, 0);
    utility_grid->addWidget(save_cloud, 1, 1);
    connect(laser_off, &QPushButton::clicked, this, [this]() {bridge_->setLaser(false);});
    connect(
      accumulation_off, &QPushButton::clicked, this, [this]() {
        bridge_->setAccumulation(false);
      });
    connect(clear_cloud, &QPushButton::clicked, this, [this]() {bridge_->clearCloud();});
    connect(save_cloud, &QPushButton::clicked, this, [this]() {bridge_->saveCloud();});
    layout->addLayout(utility_grid);

    auto * rviz_title = new QLabel(QStringLiteral("RViz Execute 模式"));
    rviz_title->setObjectName(QStringLiteral("sectionTitle"));
    auto * rviz_modes = new QHBoxLayout();
    auto * position_mode = button(QStringLiteral("关光定位"), "secondary");
    auto * scan_mode = button(QStringLiteral("整轨迹扫描"), "secondary");
    rviz_modes->addWidget(position_mode);
    rviz_modes->addWidget(scan_mode);
    connect(
      position_mode, &QPushButton::clicked, this, [this]() {
        bridge_->setRvizScanMode(false);
      });
    connect(
      scan_mode, &QPushButton::clicked, this, [this]() {
        if (QMessageBox::question(
          this, QStringLiteral("切换为扫描模式"),
          QStringLiteral("此模式下，RViz 中 Execute 会对完整规划轨迹打开激光并累积点云。是否继续？"),
          QMessageBox::Yes | QMessageBox::No, QMessageBox::No) == QMessageBox::Yes)
        {
          bridge_->setRvizScanMode(true);
        }
      });
    layout->addWidget(rviz_title);
    layout->addLayout(rviz_modes);

    auto * joints_title = new QLabel(QStringLiteral("实时关节角 / 点云"));
    joints_title->setObjectName(QStringLiteral("sectionTitle"));
    auto * joints_grid = new QGridLayout();
    for (int i = 0; i < 6; ++i) {
      auto * name = new QLabel(QStringLiteral("J%1").arg(i + 1));
      name->setObjectName(QStringLiteral("jointName"));
      joint_values_[static_cast<std::size_t>(i)] = new QLabel(QStringLiteral("—"));
      joint_values_[static_cast<std::size_t>(i)]->setObjectName(QStringLiteral("jointValue"));
      joints_grid->addWidget(name, i / 3 * 2, i % 3);
      joints_grid->addWidget(joint_values_[static_cast<std::size_t>(i)], i / 3 * 2 + 1, i % 3);
    }
    cloud_count_ = new QLabel(QStringLiteral("base_link 累计点云：0 点"));
    cloud_count_->setObjectName(QStringLiteral("cloudCount"));
    layout->addWidget(joints_title);
    layout->addLayout(joints_grid);
    layout->addWidget(cloud_count_);

    auto * log_title = new QLabel(QStringLiteral("运行记录"));
    log_title->setObjectName(QStringLiteral("sectionTitle"));
    operation_progress_ = new QProgressBar();
    operation_progress_->setTextVisible(true);
    operation_progress_->setRange(0, 1);
    operation_progress_->setValue(1);
    operation_progress_->setFormat(QStringLiteral("空闲"));
    log_ = new QPlainTextEdit();
    log_->setReadOnly(true);
    log_->setMaximumBlockCount(800);
    log_->setMinimumHeight(150);
    layout->addWidget(log_title);
    layout->addWidget(operation_progress_);
    layout->addWidget(log_, 1);
    return panel;
  }

  void bindBridge()
  {
    QPointer<ScannerControlWindow> self(this);
    bridge_->setEventCallback(
      [self](const std::string & operation, const std::string & message, bool success) {
        if (!self) {return;}
        QMetaObject::invokeMethod(
          self,
          [self, operation, message, success]() {
            if (self) {
              self->finishOperation(
                QString::fromStdString(operation), QString::fromStdString(message), success);
            }
          }, Qt::QueuedConnection);
      });
    bridge_->setTopicCallback(
      [self](const std::string & key, const std::string & message) {
        if (!self) {return;}
        QMetaObject::invokeMethod(
          self,
          [self, key, message]() {
            if (self) {
              self->handleTopicStatus(QString::fromStdString(key), QString::fromStdString(message));
            }
          }, Qt::QueuedConnection);
      });
    bridge_->setAvailabilityCallback(
      [self](const Availability & value) {
        if (!self) {return;}
        QMetaObject::invokeMethod(
          self, [self, value]() {if (self) {self->showAvailability(value);}},
          Qt::QueuedConnection);
      });
    bridge_->setJointCallback(
      [self](const std::array<double, 6> & joints) {
        if (!self) {return;}
        QMetaObject::invokeMethod(
          self,
          [self, joints]() {
            if (!self) {return;}
            for (std::size_t i = 0; i < joints.size(); ++i) {
              self->joint_values_[i]->setText(
                std::isfinite(joints[i]) ?
                QStringLiteral("%1°").arg(joints[i], 0, 'f', 1) : QStringLiteral("—"));
            }
          }, Qt::QueuedConnection);
      });
    bridge_->setCloudCallback(
      [self](std::size_t count) {
        if (!self) {return;}
        QMetaObject::invokeMethod(
          self,
          [self, count]() {
            if (self) {
              self->cloud_count_->setText(
                QStringLiteral("base_link 累计点云：%L1 点").arg(
                  static_cast<qulonglong>(count)));
            }
          }, Qt::QueuedConnection);
      });
  }

  void planWorkspace()
  {
    if (azimuth_min_->value() >= azimuth_max_->value()) {
      QMessageBox::information(this, QStringLiteral("参数错误"), QStringLiteral("左边界角必须小于右边界角。"));
      return;
    }
    if (radial_max_->value() - radial_min_->value() < 100.0) {
      QMessageBox::information(
        this, QStringLiteral("参数错误"), QStringLiteral("扫描行程差至少需要 100 mm。"));
      return;
    }
    invalidateWorkspacePlan();
    WorkspaceScanInput input;
    input.azimuth_min_deg = azimuth_min_->value();
    input.azimuth_max_deg = azimuth_max_->value();
    input.radial_min_m = radial_min_->value() * 0.001;
    input.radial_max_m = radial_max_->value() * 0.001;
    input.scan_speed_m_s = scan_speed_->value() * 0.001;
    input.transition_speed_m_s = transition_speed_->value() * 0.001;
    input.acceleration_scaling = workspace_acceleration_->value() * 0.01;
    input.return_to_start = return_to_start_->isChecked();
    input.return_velocity_scaling = return_speed_->value() * 0.01;
    input.return_acceleration_scaling = return_acceleration_->value() * 0.01;
    beginOperation(QStringLiteral("规划工作区粗扫"));
    bridge_->planWorkspace(input);
  }

  void planLinear()
  {
    invalidateLinearPlan();
    LinearScanInput input;
    input.frame = linear_frame_->currentData().toString().toStdString();
    input.axis = linear_axis_->currentData().toString().toStdString();
    input.distance_m = linear_distance_->value() * 0.001;
    input.velocity_scaling = linear_velocity_->value() * 0.01;
    input.acceleration_scaling = linear_acceleration_->value() * 0.01;
    input.maintain_base_height = true;
    beginOperation(QStringLiteral("规划固定轴直线扫描"));
    bridge_->planLinear(input);
  }

  void beginOperation(const QString & operation)
  {
    busy_ = true;
    current_operation_ = operation;
    operation_progress_->setRange(0, 0);
    operation_progress_->setFormat(operation + QStringLiteral("…"));
    updateActionButtons();
    appendLog(QStringLiteral("RUN"), operation);
  }

  void finishOperation(const QString & operation, const QString & message, bool success)
  {
    const bool tracked_operation = busy_ && operation == current_operation_;
    appendLog(
      success ? QStringLiteral("OK") : QStringLiteral("ERR"), operation + QStringLiteral(
        "：") + message);
    if (!tracked_operation) {
      return;
    }
    busy_ = false;
    current_operation_.clear();
    operation_progress_->setRange(0, 1);
    operation_progress_->setValue(1);
    operation_progress_->setFormat(success ? QStringLiteral("操作成功") : QStringLiteral("操作失败"));
    if (operation == QStringLiteral("规划工作区粗扫")) {
      workspace_planned_ = success;
      workspace_approved_ = false;
      workspace_review_->setEnabled(success);
      workspace_review_->setChecked(false);
    } else if (operation == QStringLiteral("批准工作区粗扫")) {
      workspace_approved_ = success;
    } else if (operation == QStringLiteral("执行工作区粗扫")) {
      workspace_planned_ = false;
      workspace_approved_ = false;
      workspace_review_->setChecked(false);
      workspace_review_->setEnabled(false);
    } else if (operation == QStringLiteral("规划固定轴直线扫描")) {
      linear_planned_ = success;
      linear_review_->setEnabled(success);
      linear_review_->setChecked(false);
    } else if (operation == QStringLiteral("执行固定轴直线扫描")) {
      linear_planned_ = false;
      linear_review_->setChecked(false);
      linear_review_->setEnabled(false);
    }
    updateActionButtons();
  }

  void invalidateWorkspacePlan()
  {
    workspace_planned_ = false;
    workspace_approved_ = false;
    if (workspace_review_) {
      workspace_review_->setChecked(false);
      workspace_review_->setEnabled(false);
    }
    updateActionButtons();
  }

  void invalidateLinearPlan()
  {
    linear_planned_ = false;
    if (linear_review_) {
      linear_review_->setChecked(false);
      linear_review_->setEnabled(false);
    }
    updateActionButtons();
  }

  void updateActionButtons()
  {
    if (!workspace_plan_button_) {return;}
    workspace_plan_button_->setEnabled(!busy_);
    workspace_approve_button_->setEnabled(
      !busy_ && workspace_planned_ && workspace_review_->isChecked());
    workspace_execute_button_->setEnabled(!busy_ && workspace_approved_);
    linear_plan_button_->setEnabled(!busy_);
    linear_execute_button_->setEnabled(
      !busy_ && linear_planned_ && linear_review_->isChecked());
    updateDeviceButtons();
  }

  void updateDeviceButtons()
  {
    if (!camera_connect_button_) {return;}
    camera_connect_button_->setEnabled(
      !busy_ && availability_.camera_connection_service &&
      !availability_.camera_connected);
    camera_disconnect_button_->setEnabled(
      !busy_ && availability_.camera_connection_service &&
      availability_.camera_connected);
    laser_connect_button_->setEnabled(
      !busy_ && availability_.laser_connection_service &&
      !availability_.laser_connected);
    laser_disconnect_button_->setEnabled(
      !busy_ && availability_.laser_connection_service &&
      availability_.laser_connected);
  }

  void showAvailability(const Availability & value)
  {
    availability_ = value;
    setBadge(
      robot_badge_, value.robot_state, value.robot_state ? QStringLiteral(
        "状态正常") : QStringLiteral("无关节状态"));
    setBadge(
      moveit_badge_, value.move_services, value.move_services ? QStringLiteral(
        "规划服务就绪") : QStringLiteral("服务未就绪"));
    setBadge(
      camera_badge_, value.camera_streaming,
      !value.camera_connection_service ? QStringLiteral("连接服务未启动") :
      (value.camera_streaming ? QStringLiteral("图像流在线") :
      (value.camera_connected ? QStringLiteral("已连接 / 等待图像") :
      QStringLiteral("相机未连接"))));
    setBadge(
      laser_badge_, value.laser_connected,
      !value.laser_connection_service ? QStringLiteral("连接服务未启动") :
      (value.laser_connected ? QStringLiteral("控制器已连接") :
      QStringLiteral("激光控制未连接")));
    setBadge(
      cloud_badge_, value.reconstruction, value.reconstruction ? QStringLiteral(
        "重建服务就绪") : QStringLiteral("服务未就绪"));
    updateDeviceButtons();
  }

  static void setBadge(QLabel * label, bool online, const QString & text)
  {
    label->setText((online ? QStringLiteral("● ") : QStringLiteral("○ ")) + text);
    label->setProperty("online", online);
    label->style()->unpolish(label);
    label->style()->polish(label);
  }

  void handleTopicStatus(const QString & key, const QString & message)
  {
    QString prefix;
    if (key == QStringLiteral("camera")) {
      prefix = QStringLiteral("相机");
    } else if (key == QStringLiteral("laser")) {
      prefix = QStringLiteral("激光");
    } else if (key == QStringLiteral("reconstruction")) {prefix = QStringLiteral("重建");} else {
      prefix = QStringLiteral("粗扫");
    }
    appendLog(QStringLiteral("ROS"), prefix + QStringLiteral("：") + message);
  }

  void appendLog(const QString & level, const QString & text)
  {
    const QString timestamp = QDateTime::currentDateTime().toString(QStringLiteral("HH:mm:ss"));
    log_->appendPlainText(
      QStringLiteral("[%1] %2  %3").arg(timestamp).arg(level, -5).arg(text));
  }

  void applyStyle()
  {
    setStyleSheet(
      QStringLiteral(
        R"(
      QMainWindow, QWidget { background: #f4f7fb; color: #17233c; font-family: "Noto Sans CJK SC", "Microsoft YaHei", sans-serif; font-size: 14px; }
      QLabel#pageTitle { font-size: 27px; font-weight: 700; color: #10213f; }
      QLabel#pageSubtitle { color: #66758d; font-size: 13px; }
      QLabel#safetyHint { background: #fff2df; color: #9a5200; border: 1px solid #ffd49b; border-radius: 15px; padding: 7px 12px; font-weight: 600; }
      QFrame#statusCard, QFrame#sidePanel, QGroupBox { background: white; border: 1px solid #dce4ef; border-radius: 9px; }
      QLabel#statusTitle { color: #718096; font-size: 12px; }
      QLabel#statusValue { color: #a03b3b; font-weight: 650; }
      QLabel#statusValue[online="true"] { color: #16845b; }
      QTabWidget::pane { border: 1px solid #dce4ef; border-radius: 9px; background: white; top: -1px; }
      QTabBar::tab { background: #e9eef5; color: #526177; padding: 11px 20px; border: 0; min-width: 150px; }
      QTabBar::tab:selected { background: white; color: #1264d8; font-weight: 700; border-top: 3px solid #1264d8; }
      QGroupBox { margin-top: 12px; padding: 16px 12px 12px 12px; font-weight: 650; color: #263955; }
      QGroupBox::title { subcontrol-origin: margin; left: 12px; padding: 0 6px; }
      QDoubleSpinBox, QComboBox { background: #f8fafc; border: 1px solid #cbd6e4; border-radius: 6px; padding: 6px 9px; min-height: 24px; }
      QDoubleSpinBox:focus, QComboBox:focus { border: 1px solid #2d7be5; background: white; }
      QPushButton { border: none; border-radius: 7px; padding: 8px 13px; font-weight: 650; }
      QPushButton[kind="primary"] { background: #1769dc; color: white; }
      QPushButton[kind="primary"]:hover { background: #0f59c2; }
      QPushButton[kind="secondary"] { background: #e8eef7; color: #284361; border: 1px solid #d1dcea; }
      QPushButton[kind="secondary"]:hover { background: #dbe6f4; }
      QPushButton[kind="danger"] { background: #c74747; color: white; }
      QPushButton[kind="stop"] { background: #9e2323; color: white; font-size: 16px; }
      QPushButton[kind="stop"]:hover { background: #861b1b; }
      QPushButton:disabled { background: #dfe5ec; color: #98a3b1; border: none; }
      QLabel#helpText { color: #66758d; font-size: 12px; font-weight: 400; background: #f5f8fc; border-radius: 5px; padding: 8px; }
      QLabel#sectionTitle { font-size: 15px; font-weight: 700; color: #253b5b; margin-top: 3px; }
      QLabel#jointName { color: #7a899e; font-size: 11px; }
      QLabel#jointValue { font-family: "DejaVu Sans Mono", monospace; font-size: 15px; font-weight: 650; color: #183e6b; }
      QLabel#cloudCount { background: #edf6ff; color: #195b9d; border-radius: 6px; padding: 7px; }
      QCheckBox#reviewCheck { background: #fff8ea; border: 1px solid #f4d697; border-radius: 6px; padding: 10px; color: #6c4b0e; }
      QPlainTextEdit { background: #101b2c; color: #d9e5f5; border: none; border-radius: 7px; padding: 8px; font-family: "DejaVu Sans Mono", monospace; font-size: 11px; }
      QProgressBar { border: 1px solid #d5deea; border-radius: 5px; background: #eef2f7; text-align: center; color: #40516a; min-height: 20px; }
      QProgressBar::chunk { background: #2d7be5; border-radius: 4px; }
      QScrollArea { background: white; }
    )"));
  }

  std::shared_ptr<ScannerControlBridge> bridge_;
  QTimer * availability_timer_{nullptr};
  QTabWidget * tabs_{nullptr};
  QLabel * robot_badge_{nullptr};
  QLabel * moveit_badge_{nullptr};
  QLabel * camera_badge_{nullptr};
  QLabel * laser_badge_{nullptr};
  QLabel * cloud_badge_{nullptr};
  std::array<QLabel *, 6> joint_values_{};
  QLabel * cloud_count_{nullptr};
  QPlainTextEdit * log_{nullptr};
  QProgressBar * operation_progress_{nullptr};
  QPushButton * safe_stop_button_{nullptr};
  QPushButton * camera_connect_button_{nullptr};
  QPushButton * camera_disconnect_button_{nullptr};
  QPushButton * laser_connect_button_{nullptr};
  QPushButton * laser_disconnect_button_{nullptr};
  Availability availability_;

  QDoubleSpinBox * azimuth_min_{nullptr};
  QDoubleSpinBox * azimuth_max_{nullptr};
  QDoubleSpinBox * radial_min_{nullptr};
  QDoubleSpinBox * radial_max_{nullptr};
  QDoubleSpinBox * scan_speed_{nullptr};
  QDoubleSpinBox * transition_speed_{nullptr};
  QDoubleSpinBox * workspace_acceleration_{nullptr};
  QCheckBox * return_to_start_{nullptr};
  QDoubleSpinBox * return_speed_{nullptr};
  QDoubleSpinBox * return_acceleration_{nullptr};
  QPushButton * workspace_plan_button_{nullptr};
  QPushButton * workspace_approve_button_{nullptr};
  QPushButton * workspace_execute_button_{nullptr};
  QCheckBox * workspace_review_{nullptr};

  QComboBox * linear_frame_{nullptr};
  QComboBox * linear_axis_{nullptr};
  QDoubleSpinBox * linear_distance_{nullptr};
  QDoubleSpinBox * linear_velocity_{nullptr};
  QDoubleSpinBox * linear_acceleration_{nullptr};
  QPushButton * linear_plan_button_{nullptr};
  QPushButton * linear_execute_button_{nullptr};
  QCheckBox * linear_review_{nullptr};

  bool busy_{false};
  QString current_operation_;
  bool workspace_planned_{false};
  bool workspace_approved_{false};
  bool linear_planned_{false};
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  QApplication application(argc, argv);
  application.setApplicationName(QStringLiteral("FR5 Scanner Control"));
  application.setOrganizationName(QStringLiteral("Vizum"));

  auto bridge = std::make_shared<ScannerControlBridge>();
  ScannerControlWindow window(bridge);
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
  executor.add_node(bridge);
  std::thread ros_thread([&executor]() {executor.spin();});

  window.show();
  QTimer ros_shutdown_timer;
  QObject::connect(
    &ros_shutdown_timer, &QTimer::timeout, &application, [&application]() {
      if (!rclcpp::ok()) {
        application.quit();
      }
    });
  ros_shutdown_timer.start(200);
  const int result = application.exec();

  executor.cancel();
  if (ros_thread.joinable()) {
    ros_thread.join();
  }
  executor.remove_node(bridge);
  bridge.reset();
  rclcpp::shutdown();
  return result;
}
