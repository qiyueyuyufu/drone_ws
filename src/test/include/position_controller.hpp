#ifndef __POSITION_CONTROLLER_HPP
#define __POSITION_CONTROLLER_HPP

#include "rclcpp/rclcpp.hpp"
#include <Eigen/Dense>
#include "struct.hpp"

class PositionController 
{
    public :

    PositionController();
    Eigen::Vector3d computeVelocity(
        const TrajectoryPoint &current,
        const TrajectoryPoint &target
    );

    void setGain(double kp,double kd);

    void setMaxVelocity(double max_vel);
    private :

    double kp_;
    double kd_;
    double max_vel_;

};






#endif