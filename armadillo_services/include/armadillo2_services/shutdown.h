/*******************************************************************************
* Copyright (c) 2018, RoboTICan, LTD.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright notice, this
*   list of conditions and the following disclaimer.
*
* * Redistributions in binary form must reproduce the above copyright notice,
*   this list of conditions and the following disclaimer in the documentation
*   and/or other materials provided with the distribution.
*
* * Neither the name of RoboTICan nor the names of its
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/
/* Author: Elchay Rauper*/


#ifndef ARMADILLO2_SERVICES_SHUTDOWN_H
#define ARMADILLO2_SERVICES_SHUTDOWN_H

#include <armadillo2_services/dxl_torque.h>
#include <armadillo2_services/pan_tilt_mover.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/String.h>

class Shutdown
{
private:
    ros::NodeHandle *nh_;
    ros::ServiceServer shutdown_srv_;
    ros::Publisher espeak_pub_;
    const PanTiltMover* head_mover_;
    DxlTorque* dxl_torque_;

    bool shutdownCB(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

public:
    Shutdown(ros::NodeHandle &nh,
             const PanTiltMover &head_mover,
             DxlTorque &dxl_torque);
};


#endif //ARMADILLO2_SERVICES_SHUTDOWN_H
