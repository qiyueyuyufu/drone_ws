#include "position_controller.hpp"

#include <chrono>
#include <memory>
#include <string>

#include <Eigen/Dense>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "rclcpp/rclcpp.hpp"
#include "struct.hpp"

using namespace std::chrono_literals;

class HoverNode : public rclcpp::Node
{
public:
    HoverNode() : Node("hover_node")
    {
        last_request_ = this->now();

        hover_altitude_ = this->declare_parameter<double>("hover_altitude", 1.0);
        controller_.setGain(
            this->declare_parameter<double>("kp", 0.55),
            this->declare_parameter<double>("kd", 0.09));
        controller_.setMaxVelocity(this->declare_parameter<double>("max_velocity", 1.0));

        target_.velocity.setZero();
        vel_cmd_.setZero();

        auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
        qos.best_effort();

        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/mavros/local_position/pose",
            qos,
            std::bind(&HoverNode::PoseCallback, this, std::placeholders::_1));

        velocity_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "/mavros/local_position/velocity_local",
            qos,
            std::bind(&HoverNode::VelocityCallback, this, std::placeholders::_1));

        state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
            "/mavros/state",
            qos,
            std::bind(&HoverNode::StateCallback, this, std::placeholders::_1));

        velocity_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/mavros/setpoint_velocity/cmd_vel",
            10);

        arm_client_ = this->create_client<mavros_msgs::srv::CommandBool>(
            "/mavros/cmd/arming");

        mode_client_ = this->create_client<mavros_msgs::srv::SetMode>(
            "/mavros/set_mode");

        timer_ = this->create_wall_timer(50ms, std::bind(&HoverNode::Loop, this));

        RCLCPP_INFO(this->get_logger(), "Hover node ready, target altitude %.2f m", hover_altitude_);
    }

private:
    void StateCallback(const mavros_msgs::msg::State::SharedPtr msg)
    {
        state_ = *msg;
    }

    void PoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        current_.position = Eigen::Vector3d(
            msg->pose.position.x,
            msg->pose.position.y,
            msg->pose.position.z);

        if (!pose_received_) {
            target_.position = Eigen::Vector3d(
                current_.position.x(),
                current_.position.y(),
                hover_altitude_);
            pose_received_ = true;
            RCLCPP_INFO(
                this->get_logger(),
                "Hover target set to x %.2f, y %.2f, z %.2f",
                target_.position.x(),
                target_.position.y(),
                target_.position.z());
        }
    }

    void VelocityCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
    {
        current_.velocity = Eigen::Vector3d(
            msg->twist.linear.x,
            msg->twist.linear.y,
            msg->twist.linear.z);
        velocity_received_ = true;
    }

    void Arm(bool arm_cmd)
    {
        if (!arm_client_->service_is_ready()) {
            return;
        }

        auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
        request->value = arm_cmd;
        arm_client_->async_send_request(request);
    }

    void SetMode(const std::string &mode)
    {
        if (!mode_client_->service_is_ready()) {
            return;
        }

        auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
        request->custom_mode = mode;
        mode_client_->async_send_request(request);
    }

    void PublishVelocity(const Eigen::Vector3d &vel_cmd, double yaw)
    {
        geometry_msgs::msg::TwistStamped msg;
        msg.header.stamp = this->get_clock()->now();
        msg.twist.linear.x = vel_cmd.x();
        msg.twist.linear.y = vel_cmd.y();
        msg.twist.linear.z = vel_cmd.z();
        msg.twist.angular.x = 0.0;
        msg.twist.angular.y = 0.0;
        msg.twist.angular.z = yaw;
        velocity_pub_->publish(msg);
    }

    void Loop()
    {
        auto now = this->now();

        if (!pose_received_ || !velocity_received_) {
            vel_cmd_.setZero();
            PublishVelocity(vel_cmd_, 0.0);
            return;
        }

        if (state_.connected && state_.mode != "OFFBOARD" && (now - last_request_).seconds() > 1.0) {
            SetMode("OFFBOARD");
            last_request_ = now;
        } else if (state_.connected && state_.mode == "OFFBOARD" && !state_.armed &&
                   (now - last_request_).seconds() > 1.0) {
            Arm(true);
            last_request_ = now;
        }

        vel_cmd_ = controller_.computeVelocity(current_, target_);
        PublishVelocity(vel_cmd_, 0.0);
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_sub_;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_pub_;

    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_client_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr mode_client_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time last_request_;

    mavros_msgs::msg::State state_;
    TrajectoryPoint current_;
    TrajectoryPoint target_;
    PositionController controller_;
    Eigen::Vector3d vel_cmd_;

    double hover_altitude_{1.0};
    bool pose_received_{false};
    bool velocity_received_{false};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HoverNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
