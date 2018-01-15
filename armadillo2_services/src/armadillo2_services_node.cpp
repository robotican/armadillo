#include <ros/ros.h>
#include <armadillo2_services/lift_arm.h>
#include <armadillo2_services/pan_tilt_mover.h>
#include <armadillo2_services/shutdown.h>
#include <armadillo2_services/joints_state_reader.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "armadillo2_services_node");
    ros::NodeHandle nh;

    JointStateReader joints_state(nh);
    /* try center robot head upon startup */
    ROS_INFO("[armadillo2_services]: waiting for joints_state publisher...");
    uint8_t tries = 0;
    while (tries < 5 && !joints_state.gotState())
    {
        tries++;
        ros::Duration(1).sleep();
    }

    LiftArm arm_lifter(nh, joints_state);
    PanTiltMover head_mover(nh);
    DxlTorque dxl_torque(nh, head_mover, joints_state);
    Shutdown shutdown(nh, head_mover, dxl_torque);

    bool center_head = false;
    ros::param::get("~center_head", center_head);

    if (center_head)
    {
        /* try center robot head upon startup */
        ROS_INFO("[armadillo2_services]: centering head...");
        tries = 0;
        while (tries < 5 && !head_mover.centerHead())
        {
            tries++;
            ros::Duration(1).sleep();
        }
    }

    ROS_INFO("[armadillo2_services]: ready");
    ros::spin();
    return 0;
}

