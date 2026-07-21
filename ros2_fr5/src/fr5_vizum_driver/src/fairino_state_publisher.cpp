#include "fr5_vizum_driver/flange_pose_utils.hpp"
#include "fr5_vizum_driver/joint_state_utils.hpp"

#include "robot.h"

#include <fr5_vizum_msgs/msg/flange_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker.hpp>

#include <arpa/inet.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

std::chrono::nanoseconds timerPeriodFromRate(double hz) {
    if (!std::isfinite(hz) || hz < 0.1 || hz > 200.0) {
        throw std::invalid_argument("publish_rate_hz must be within [0.1, 200.0]");
    }

    const auto period = std::chrono::duration<double>(1.0 / hz);
    return std::max(
        std::chrono::nanoseconds(1),
        std::chrono::duration_cast<std::chrono::nanoseconds>(period));
}

} // namespace

namespace fr5_vizum_driver {

class FairinoStatePublisher : public rclcpp::Node {
public:
    FairinoStatePublisher()
        : Node("fairino_state_publisher") {
        robotIp_ = declare_parameter<std::string>("robot_ip", "192.168.1.200");
        publishRateHz_ = declare_parameter<double>("publish_rate_hz", 20.0);
        logPosePeriodMs_ = declare_parameter<int>("log_pose_period_ms", 1000);
        baseFrame_ = declare_parameter<std::string>("base_frame", "base_link");
        flangeFrame_ = declare_parameter<std::string>(
            "flange_frame", "fairino_flange_reported");
        jointStateTopic_ = declare_parameter<std::string>(
            "joint_state_topic", "joint_states");
        flangePoseTopic_ = declare_parameter<std::string>(
            "flange_pose_topic", "fairino/flange_pose");
        flangeRawTopic_ = declare_parameter<std::string>(
            "flange_raw_topic", "fairino/flange_pose_mm_deg");
        flangeMarkerTopic_ = declare_parameter<std::string>(
            "flange_marker_topic", "fairino/flange_marker");
        publishFlangeTf_ = declare_parameter<bool>("publish_flange_tf", true);
        publishFlangeMarker_ = declare_parameter<bool>("publish_flange_marker", true);
        markerTextHeight_ = declare_parameter<double>("marker_text_height", 0.035);
        markerZOffset_ = declare_parameter<double>("marker_z_offset", 0.08);
        jointNames_ = declare_parameter<std::vector<std::string>>(
            "joint_names", defaultJointNames());

        validateParameters();

        jointPublisher_ = create_publisher<sensor_msgs::msg::JointState>(
            jointStateTopic_, rclcpp::QoS(10));
        flangePosePublisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(
            flangePoseTopic_, rclcpp::QoS(10));
        flangeRawPublisher_ = create_publisher<fr5_vizum_msgs::msg::FlangePose>(
            flangeRawTopic_, rclcpp::QoS(10));

        if (publishFlangeTf_) {
            tfBroadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        }
        if (publishFlangeMarker_) {
            markerPublisher_ = create_publisher<visualization_msgs::msg::Marker>(
                flangeMarkerTopic_, rclcpp::QoS(10));
        }

        timer_ = create_wall_timer(
            timerPeriodFromRate(publishRateHz_),
            std::bind(&FairinoStatePublisher::publishRobotState, this));

        RCLCPP_INFO(
            get_logger(),
            "Read-only FR5 state publisher ready: ip=%s rate=%.1f Hz, topics=[/%s, /%s, /%s]",
            robotIp_.c_str(),
            publishRateHz_,
            jointStateTopic_.c_str(),
            flangePoseTopic_.c_str(),
            flangeRawTopic_.c_str());
        RCLCPP_INFO(
            get_logger(),
            "This node never enables or moves the robot. It only reads cached joint and flange state.");
    }

    ~FairinoStatePublisher() override {
        closeRobot();
        // Fairino 3.9.4 starts detached worker threads.  CloseRPC() asks them
        // to stop, but the SDK provides no join API and can still touch the
        // FRRobot object while normal C++ member destruction is in progress.
        // Keep this one SDK object alive until process teardown to avoid a
        // shutdown-only use-after-free; the operating system reclaims it.
        (void)robot_.release();
    }

private:
    void validateParameters() const {
        if (robotIp_.empty() || robotIp_.size() > 63) {
            throw std::invalid_argument("robot_ip must contain 1 to 63 characters");
        }
        in_addr parsedAddress{};
        if (inet_pton(AF_INET, robotIp_.c_str(), &parsedAddress) != 1) {
            throw std::invalid_argument("robot_ip must be a valid IPv4 address");
        }
        if (jointNames_.size() != kJointCount) {
            throw std::invalid_argument("joint_names must contain exactly 6 names");
        }
        if (baseFrame_.empty() || flangeFrame_.empty()) {
            throw std::invalid_argument("base_frame and flange_frame must not be empty");
        }
        if (logPosePeriodMs_ < 0) {
            throw std::invalid_argument("log_pose_period_ms must be non-negative");
        }
        if (!std::isfinite(markerTextHeight_) || markerTextHeight_ <= 0.0 ||
            !std::isfinite(markerZOffset_)) {
            throw std::invalid_argument("invalid flange marker size or offset");
        }
        (void)timerPeriodFromRate(publishRateHz_);
    }

    void connectRobot() {
        if (connected_) {
            return;
        }

        RCLCPP_INFO(get_logger(), "Connecting to Fairino FR5 at %s ...", robotIp_.c_str());
        const errno_t err = robot_->RPC(robotIp_.c_str());
        if (err != 0) {
            // RPC() can leave detached SDK threads behind even when it fails.
            // Stop that one attempt and terminate this process; retrying RPC()
            // on the same object would reset its exit flags and race old threads.
            (void)robot_->CloseRPC();
            std::ostringstream message;
            message << "FRRobot::RPC(" << robotIp_ << ") failed, err=" << err
                    << ". Check the network and make sure no other SDK client owns "
                       "the robot connection, then restart the launch.";
            throw std::runtime_error(message.str());
        }

        connected_ = true;
        publishedFirstSample_ = false;
        RCLCPP_INFO(get_logger(), "Connected to Fairino FR5 at %s", robotIp_.c_str());
    }

    void closeRobot() {
        if (!connected_) {
            return;
        }

        const errno_t err = robot_->CloseRPC();
        connected_ = false;
        if (err != 0) {
            RCLCPP_WARN(get_logger(), "CloseRPC failed, err=%d", err);
        } else {
            RCLCPP_INFO(get_logger(), "Fairino RPC connection closed");
        }
    }

    void publishRobotState() {
        connectRobot();

        JointPos jointPosition;
        DescPose flangePose;
        const errno_t jointErr = robot_->GetActualJointPosDegree(0, &jointPosition);
        const errno_t flangeErr = robot_->GetActualToolFlangePose(0, &flangePose);
        if (jointErr != 0 || flangeErr != 0) {
            std::ostringstream message;
            message << "Fairino state read failed: joint err=" << jointErr
                    << ", flange err=" << flangeErr
                    << ". The SDK cannot reconnect safely in-process; restart the launch.";
            closeRobot();
            throw std::runtime_error(message.str());
        }

        std::array<double, kJointCount> jointDegrees{};
        std::array<double, kPoseDimension> flangeRaw{};
        std::copy_n(jointPosition.jPos, kJointCount, jointDegrees.begin());
        flangeRaw = {
            flangePose.tran.x,
            flangePose.tran.y,
            flangePose.tran.z,
            flangePose.rpy.rx,
            flangePose.rpy.ry,
            flangePose.rpy.rz,
        };

        if (!allFinite(jointDegrees) || !allFinite(flangeRaw)) {
            RCLCPP_ERROR_THROTTLE(
                get_logger(), *get_clock(), 3000,
                "Fairino realtime state contains a non-finite joint or flange value; sample skipped");
            return;
        }

        const rclcpp::Time stamp = now();
        publishJoints(stamp, jointDegrees);
        publishFlange(stamp, flangeRaw);

        if (!publishedFirstSample_) {
            publishedFirstSample_ = true;
            RCLCPP_INFO(
                get_logger(),
                "First live joint and flange sample received from the Fairino SDK cache");
        }
    }

    void publishJoints(
        const rclcpp::Time& stamp,
        const std::array<double, kJointCount>& jointDegrees) {
        const std::array<double, kJointCount> jointRadians =
            degreesToRadians(jointDegrees);

        sensor_msgs::msg::JointState msg;
        msg.header.stamp = stamp;
        msg.name = jointNames_;
        msg.position.assign(jointRadians.begin(), jointRadians.end());
        jointPublisher_->publish(msg);
    }

    void publishFlange(
        const rclcpp::Time& stamp,
        const std::array<double, kPoseDimension>& flangeRaw) {
        const FlangePoseValues converted = fairinoFlangePoseToRos(flangeRaw);

        tf2::Quaternion quaternion;
        quaternion.setRPY(
            converted.rpyRadians[0],
            converted.rpyRadians[1],
            converted.rpyRadians[2]);
        quaternion.normalize();

        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = stamp;
        pose.header.frame_id = baseFrame_;
        pose.pose.position.x = converted.translationMeters[0];
        pose.pose.position.y = converted.translationMeters[1];
        pose.pose.position.z = converted.translationMeters[2];
        pose.pose.orientation.x = quaternion.x();
        pose.pose.orientation.y = quaternion.y();
        pose.pose.orientation.z = quaternion.z();
        pose.pose.orientation.w = quaternion.w();
        flangePosePublisher_->publish(pose);

        fr5_vizum_msgs::msg::FlangePose raw;
        raw.header = pose.header;
        raw.x_mm = flangeRaw[0];
        raw.y_mm = flangeRaw[1];
        raw.z_mm = flangeRaw[2];
        raw.rx_deg = flangeRaw[3];
        raw.ry_deg = flangeRaw[4];
        raw.rz_deg = flangeRaw[5];
        flangeRawPublisher_->publish(raw);

        if (tfBroadcaster_) {
            geometry_msgs::msg::TransformStamped transform;
            transform.header = pose.header;
            transform.child_frame_id = flangeFrame_;
            transform.transform.translation.x = pose.pose.position.x;
            transform.transform.translation.y = pose.pose.position.y;
            transform.transform.translation.z = pose.pose.position.z;
            transform.transform.rotation = pose.pose.orientation;
            tfBroadcaster_->sendTransform(transform);
        }

        if (markerPublisher_) {
            publishFlangeMarker(pose, flangeRaw);
        }

        if (logPosePeriodMs_ > 0) {
            RCLCPP_INFO_THROTTLE(
                get_logger(),
                *get_clock(),
                static_cast<std::int64_t>(logPosePeriodMs_),
                "Flange XYZ [mm] = [%.3f, %.3f, %.3f], RPY [deg] = [%.3f, %.3f, %.3f]",
                flangeRaw[0], flangeRaw[1], flangeRaw[2],
                flangeRaw[3], flangeRaw[4], flangeRaw[5]);
        }
    }

    void publishFlangeMarker(
        const geometry_msgs::msg::PoseStamped& pose,
        const std::array<double, kPoseDimension>& flangeRaw) {
        visualization_msgs::msg::Marker marker;
        marker.header = pose.header;
        marker.ns = "fairino_flange_pose";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position = pose.pose.position;
        marker.pose.position.z += markerZOffset_;
        marker.pose.orientation.w = 1.0;
        marker.scale.z = markerTextHeight_;
        marker.color.r = 1.0F;
        marker.color.g = 0.82F;
        marker.color.b = 0.16F;
        marker.color.a = 1.0F;

        std::ostringstream text;
        text << std::fixed << std::setprecision(2)
             << "FR5 flange\nXYZ mm: "
             << flangeRaw[0] << ", " << flangeRaw[1] << ", " << flangeRaw[2]
             << "\nRPY deg: "
             << flangeRaw[3] << ", " << flangeRaw[4] << ", " << flangeRaw[5];
        marker.text = text.str();

        const double lifetimeSeconds = std::max(0.25, 3.0 / publishRateHz_);
        marker.lifetime = rclcpp::Duration::from_seconds(lifetimeSeconds);
        markerPublisher_->publish(marker);
    }

    std::unique_ptr<FRRobot> robot_{std::make_unique<FRRobot>()};
    bool connected_{false};
    bool publishedFirstSample_{false};

    std::string robotIp_;
    double publishRateHz_{20.0};
    int logPosePeriodMs_{1000};
    std::string baseFrame_;
    std::string flangeFrame_;
    std::string jointStateTopic_;
    std::string flangePoseTopic_;
    std::string flangeRawTopic_;
    std::string flangeMarkerTopic_;
    bool publishFlangeTf_{true};
    bool publishFlangeMarker_{true};
    double markerTextHeight_{0.035};
    double markerZOffset_{0.08};
    std::vector<std::string> jointNames_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jointPublisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr flangePosePublisher_;
    rclcpp::Publisher<fr5_vizum_msgs::msg::FlangePose>::SharedPtr flangeRawPublisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr markerPublisher_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace fr5_vizum_driver

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<fr5_vizum_driver::FairinoStatePublisher>());
    } catch (const std::exception& ex) {
        std::cerr << "fairino_state_publisher failed: " << ex.what() << std::endl;
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
