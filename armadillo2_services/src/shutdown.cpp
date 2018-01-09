
#include <armadillo2_services/shutdown.h>

Shutdown::Shutdown(ros::NodeHandle &nh,
                   const PanTiltMover &head_mover,
                   DxlTorque &dxl_torque)
{
    nh_ = &nh;
    dxl_torque_ = &dxl_torque;
    head_mover_ = &head_mover;

    shutdown_srv_ = nh_->advertiseService("services/shutdown", &Shutdown::shutdownCB, this);
    espeak_pub_ = nh.advertise<std_msgs::String>("/espeak_node/speak_line", 10);
}

bool Shutdown::shutdownCB(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    std_msgs::String speak_msg;
    speak_msg.data = "shutting down";
    espeak_pub_.publish(speak_msg);
    /* prepare head for shutdown - lower by 40 deg */
    head_mover_->publishTrajectoryMsg(0, 0.698131);
    ros::Duration(3).sleep(); //give head time to go down
    dxl_torque_->setTorque(false);
    return true;
}