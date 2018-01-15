
#ifndef ARMADILLO2_SERVICES_DXL_TORQUE_H
#define ARMADILLO2_SERVICES_DXL_TORQUE_H

#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>
#include <armadillo2_services/lift_arm.h>
#include <armadillo2_services/pan_tilt_mover.h>
#include <armadillo2_services/joints_state_reader.h>

struct head_state
{
    double pan_pos_rad = 0;
    double tilt_pos_rad = 0;
    bool got_state = false;
};

class DxlTorque
{
private:
    ros::NodeHandle* nh_;
    ros::Publisher dxl_traj_pub_;
    ros::ServiceServer set_torque_srv_;
    ros::ServiceClient set_torque_client_;
    const JointStateReader* joint_states_;


    const PanTiltMover *head_mover_;

    bool setTorqueCB(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
    void commandCurrentDxlPosition();

public:
    DxlTorque(ros::NodeHandle& nh,
              const PanTiltMover &head_mover,
              const JointStateReader &joints_state);
    bool setTorque(bool onoff);

};


#endif //ARMADILLO2_SERVICES_DXL_TORQUE_H
