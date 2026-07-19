#include "test.hpp"

using namespace std::chrono_literals;

Test::Test():Node("Test")
{
    last_request_ = this->now();
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    qos.best_effort();
    pos_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/mavros/local_position/pose",
        qos,
        std::bind(&Test::PoseCallback,this,std::placeholders::_1)
    );
    speed_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        "/mavros/local_position/velocity_local",
        qos,
        std::bind(&Test::SpeedCallback,this,std::placeholders::_1)
    );
    state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
        "/mavros/state",
        qos,
        std::bind(&Test::StateCallback,this,std::placeholders::_1)
    );
     speed_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
        "/mavros/setpoint_velocity/cmd_vel",
        10
    );
     arm_client_ = this->create_client<mavros_msgs::srv::CommandBool>(
        "/mavros/cmd/arming"
    );

    mode_client_ = this->create_client<mavros_msgs::srv::SetMode>(
        "/mavros/set_mode"
    );

    triangle_points_[0] = Eigen::Vector3d(0.0, 0.0, 1.0);
    triangle_points_[1] = Eigen::Vector3d(1.0, 0.0, 1.0);
    triangle_points_[2] = Eigen::Vector3d(0.5, 0.866, 1.0);

    target_.position<<0.0,0.0,1.0;
    target_.velocity<<0,0,0;
    position_.setGain(kp_,kd_);
    position_.setMaxVelocity(2.0);
    timer_ = this->create_wall_timer(
        50ms,
        std::bind(&Test::loop,this));
}

void Test::StateCallback(mavros_msgs::msg::State::SharedPtr msg)
{
    state_ = *msg;
}

void Test::arm(bool arm_cmd)
{
    auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    request->value=arm_cmd;
    arm_client_->async_send_request(request);
}

void Test::SetMode(const std::string &mode)
{
    auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    request->custom_mode = mode;
    mode_client_->async_send_request(request);

}

void Test::PoseCallback(geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    current_.position = Eigen::Vector3d(msg->pose.position.x,msg->pose.position.y,msg->pose.position.z);
};

void Test::SpeedCallback(geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
    current_.velocity = Eigen::Vector3d(msg->twist.linear.x,msg->twist.linear.y,msg->twist.linear.z);

}

void Test::Pubvel(Eigen::Vector3d vel_cmd,double yaw)
{
    geometry_msgs::msg::TwistStamped msg;
    msg.header.stamp = this->get_clock()->now();

    msg.twist.linear.x = vel_cmd.x();
    msg.twist.linear.y = vel_cmd.y();
    msg.twist.linear.z = vel_cmd.z();
    
    msg.twist.angular.x = 0;
    msg.twist.angular.y = 0;
    msg.twist.angular.z = yaw;

    speed_pub_->publish(msg);
}

bool Test::isoffboardready()
{
    return state_.connected&&(state_.mode=="OFFBOARD");
}

bool Test::isReachedTarget(double tolerance)
{
    return (current_.position - target_.position).norm() < tolerance;
}

void Test::loop()
{


    if(count_==20)
    {
        RCLCPP_INFO(this->get_logger(),"State:%d X:%.2f,Y:%.2f,Z:%.2f",current_state_,current_.position.x(),current_.position.y(),current_.position.z());
        count_ = 0;
    }
    auto now = this->now();

    switch(current_state_)
    {
        case WAIT:
        {
            if(!isoffboardready())
            {
                if((now-last_request_).seconds()>1.0)
                {
                    SetMode("OFFBOARD");
                    last_request_ = now;
                }
                vel_cmd_.setZero();
            }
            else
            {
                if(!state_.armed)
                {
                    if((now-last_request_).seconds()>1.0)
                    {
                        arm(true);
                        last_request_ = now;
                    }
                    vel_cmd_.setZero();
                }
                else
                {
                    RCLCPP_INFO(this->get_logger(),"Connected and Armed, entering TAKEOFF");
                    current_state_ = TAKEOFF;
                    target_.position = Eigen::Vector3d(0.0, 0.0, 1.0);
                }
            }
            break;
        }
        case TAKEOFF:
        {
            vel_cmd_ = position_.computeVelocity(current_,target_);
            if(isReachedTarget(0.2))
            {
                RCLCPP_INFO(this->get_logger(),"Takeoff complete, entering TRIANGLE");
                current_state_ = TRIANGLE;
                triangle_index_ = 0;
                target_.position = triangle_points_[triangle_index_];
            }
            break;
        }
        case TRIANGLE:
        {
            vel_cmd_ = position_.computeVelocity(current_,target_);
            if(isReachedTarget(0.2))
            {
                triangle_index_++;
                if(triangle_index_ >= 3)
                {
                    RCLCPP_INFO(this->get_logger(),"Triangle complete, entering LAND");
                    current_state_ = LAND;
                    target_.position = Eigen::Vector3d(0.5, 0.433, 0.0);
                }
                else
                {
                    target_.position = triangle_points_[triangle_index_];
                    RCLCPP_INFO(this->get_logger(),"Moving to triangle point %d",triangle_index_);
                }
            }
            break;
        }
        case LAND:
        {
            vel_cmd_ = position_.computeVelocity(current_,target_);
            if(isReachedTarget(0.2))
            {
                RCLCPP_INFO(this->get_logger(),"Landing complete");
                vel_cmd_.setZero();
            }
            break;
        }
    }

    Pubvel(vel_cmd_,0);
    count_++;
}

int main(int argc,char** argv)
{

    rclcpp::init(argc,argv);

    auto node = std::make_shared<Test>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;

}