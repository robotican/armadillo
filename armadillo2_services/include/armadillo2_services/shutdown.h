//
// Created by armadillo2 on 08/01/18.
//

#ifndef ARMADILLO2_SERVICES_SHUTDOWN_H
#define ARMADILLO2_SERVICES_SHUTDOWN_H

#include <armadillo2_services/dxl_torque.h>
#include <armadillo2_services/pan_tilt_mover.h>
#include <std_srvs/Trigger.h>


class Shutdown
{
private:

    ros::ServiceServer shutdown_srv_;
    const PanTiltMover* head_mover_;
    const DxlTorque* dxl_torque_;

    bool shutdownCB(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

public:
    Shutdown(const PanTiltMover& head_mover,
             const DxlTorque& dxl_torque);
};


#endif //ARMADILLO2_SERVICES_SHUTDOWN_H
