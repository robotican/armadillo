#include <ros/ros.h>
#include <armadillo2_services/lift_arm.h>
#include <armadillo2_services/pan_tilt_mover.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "armadillo2_services_node");
    ros::NodeHandle nh;

    LiftArm lifter(nh);
    PanTiltMover mover(nh);

    ros::spin();
    return 0;
}

