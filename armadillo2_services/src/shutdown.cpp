
#include <armadillo2_services/shutdown.h>

Shutdown::Shutdown(const PanTiltMover &head_mover, const DxlTorque &dxl_torque)
{
    dxl_torque_ = &dxl_torque;
    head_mover_ = &head_mover;
}

bool Shutdown::shutdownCB(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    /* prepare head for shutdown */
    head_mover_->publishTrajectoryMsg(0, 40);
    ros::Duration(3).sleep(); //give head time to go down
    dxl_torque_->set(false);
    return true;
}