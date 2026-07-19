#include "position_controller.hpp"
#include <algorithm>

PositionController::PositionController()
{
    kp_ = 2.0;
    max_vel_ = 2.0;
}

void PositionController::setGain(double kp,double kd)
{
    kp_ = kp;
    kd_ = kd;
}

void PositionController::setMaxVelocity(double max_vel)
{
    max_vel_ = max_vel;
}

Eigen::Vector3d PositionController::computeVelocity(
    const TrajectoryPoint &current,
    const TrajectoryPoint &target)
{
    // 计算位置误差
    Eigen::Vector3d error_pos = target.position - current.position;
    Eigen::Vector3d error_vel = target.velocity - current.velocity;
    // P控制
    Eigen::Vector3d velocity = target.velocity+kp_ * error_pos + kd_*error_vel;

    // 限幅
    velocity.x() = std::clamp(velocity.x(), -max_vel_, max_vel_);
    velocity.y() = std::clamp(velocity.y(), -max_vel_, max_vel_);
    velocity.z() = std::clamp(velocity.z(), -max_vel_, max_vel_);

    return velocity;
}