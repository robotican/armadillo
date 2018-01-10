#include <ros/ros.h>
#include <armadillo2_services/lift_arm.h>
#include <armadillo2_services/pan_tilt_mover.h>
#include <armadillo2_services/shutdown.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "armadillo2_services_node");
    ros::NodeHandle nh;

    LiftArm arm_lifter(nh);
    PanTiltMover head_mover(nh);
    DxlTorque dxl_torque(nh, head_mover);
    Shutdown shutdown(nh, head_mover, dxl_torque);

    bool center_head = false;
    ros::param::get("~center_head", center_head);

    if (center_head)
    {
        /* try center robot head upon startup */
        ROS_INFO("[armadillo2_services]: centering head...");
        uint8_t tries = 0;
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

