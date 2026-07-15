#ifndef __TEST_HPP
#define __TEST_HPP

#include "rclcpp/rclcpp.hpp"
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "position_controller.hpp"

#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "mavros_msgs/srv/command_tol.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "struct.hpp"

class Test :public rclcpp::Node
{
    public:
        Test();
    private:
        void StateCallback(mavros_msgs::msg::State::SharedPtr msg);
        void SpeedCallback(geometry_msgs::msg::TwistStamped::SharedPtr msg);
        void PoseCallback(geometry_msgs::msg::PoseStamped::SharedPtr msg);
        void SetMode(const std::string &mode);
        void arm(bool arm_cmd);
        void Pubvel(Eigen::Vector3d vel_cmd,double yaw);
        void loop();
        bool isoffboardready();
        TrajectoryPoint current_;
        TrajectoryPoint target_;

        rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_client_;
        rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr mode_client_;

        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pos_sub_;
        rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
        rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr speed_sub_;

        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr speed_pub_;

        rclcpp::Time last_request_;
        rclcpp::TimerBase::SharedPtr timer_;

        int count_ = 0;
        mavros_msgs::msg::State state_;
        Eigen::Vector3d vel_cmd_;

        double kp_{0.55};
        double kd_{0.09};
        PositionController position_;

};





#endif