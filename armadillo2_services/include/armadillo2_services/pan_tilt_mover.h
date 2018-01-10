#ifndef ARMADILLO2_SERVICES_PAN_TILT_MOVER_H
#define ARMADILLO2_SERVICES_PAN_TILT_MOVER_H

#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float32.h>
#include <armadillo2_msgs/PanTilt.h>

class PanTiltMover
{
private:
    ros::Publisher traj_pub_,
                   grp_pos_pub_;
    ros::ServiceServer mover_srv_;
    ros::NodeHandle *nh_;

    double pan_goal = 0,
           tilt_goal = 0;

    bool moveHeadCB(armadillo2_msgs::PanTilt::Request &req,
                    armadillo2_msgs::PanTilt::Response &res);

public:
    PanTiltMover(ros::NodeHandle &nh);
    bool publishTrajectoryMsg(float pan, float tilt) const;
    bool publishGroupPosMsg(float pan, float tilt) const;
    bool centerHead() const;

};


#endif //ARMADILLO2_SERVICES_PAN_TILT_MOVER_H
