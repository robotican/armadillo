#include <ros/ros.h>
#include <armadillo2_services/lift_arm.h>
#include <armadillo2_services/pan_tilt_mover.h>
#include <armadillo2_services/shutdown.h>
#include <armadillo2_services/joints_state_reader.h>
#include <armadillo2_services/torso.h>

#define MAX_TRIES 10

int main(int argc, char** argv)
{
    ros::init(argc, argv, "armadillo2_services_node");
    ros::NodeHandle nh;

    /* give controllers time to load before */
    /* trying to send commands              */
    ros::Duration(5).sleep();

    JointStateReader joints_state(nh);
    LiftArm arm_lifter(nh, joints_state);
    PanTiltMover head_mover(nh);
    DxlTorque dxl_torque(nh, head_mover, joints_state);
    Shutdown shutdown(nh, head_mover, dxl_torque);
    //Torso torso(nh, joints_state);

    uint8_t tries = 0;
    bool center_head = false;
    ros::param::get("~center_head", center_head);

    if (center_head)
    {
        /* try center robot head upon startup */
        ROS_INFO("[armadillo2_services]: centering head...");
        tries = 0;
        while (tries < MAX_TRIES && !head_mover.centerHead())
        {
            tries++;
            ros::Duration(1).sleep();
        }
        if (tries >= MAX_TRIES)
            ROS_WARN("can't center head");
    }

    /* we must have joint state msg to determine    */
    /* current state of the robot, as some services */
    /* depend on it                                 */
    /*ROS_INFO("[armadillo2_services]: waiting for joint state messages...");
    tries = 0;
    while (tries < MAX_TRIES && !joints_state.gotState())
    {
        ros::spinOnce(); //try to get joint state messages
        tries++;
        ros::Duration(0.5).sleep();
    }
    if (tries >= MAX_TRIES)
    {
        ROS_ERROR("[armadillo2_services]: no joint state messages detected. shutting down...");
        exit(EXIT_FAILURE);
    }*/

    /* command torso to stay in place */
    //torso.commandCurrentPos();

    ROS_INFO("[armadillo2_services]: ready");
    ros::spin();
    return 0;
}

