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

    target_.position<<0.0,0.0,0.25;
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

void Test::loop()
{
    
    
    if(count_%20==0)
    {
        RCLCPP_INFO(this->get_logger(),"X:%.2f,Y:%.2f,Z:%.2f",current_.position.x(),current_.position.y(),current_.position.z());
        count_ = 0;
    }
    auto now = this->now();
    if(!isoffboardready())
    {
        if((now-last_request_).seconds()>1.0)
        {
            SetMode("OFFBOARD");
            last_request_ = now;
        }
        vel_cmd_.Zero();
    }else
    {
         if((now-last_request_).seconds()>1.0)
        {
          arm(true);
           last_request_ = now;
        }
        vel_cmd_ = position_.computeVelocity(current_,target_);
    }
    Pubvel(vel_cmd_,0);
}

int main(int argc,char** argv)
{

    rclcpp::init(argc,argv);

    auto node = std::make_shared<Test>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;

}