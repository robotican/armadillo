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

    ros::spin();
    return 0;
}

