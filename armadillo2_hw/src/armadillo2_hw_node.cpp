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

#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include "armadillo2_hw.h"

#define LOOP_HZ 100.0
#define THREADS_NUM 2

int main(int argc, char **argv)
{
    ros::init(argc, argv, "armadillo2_hw_node");
    ros::NodeHandle nh;

    armadillo2_hw::ArmadilloHW armadillo_hw(nh);
    controller_manager::ControllerManager controller_manager(&armadillo_hw);

    ros::AsyncSpinner asyncSpinner(THREADS_NUM);
    asyncSpinner.start();

    ros::Time last_time = ros::Time::now();

    while (ros::ok())
    {
        armadillo_hw.read();

        ros::Duration(1.0 / (2.0 * LOOP_HZ)).sleep();

        ros::Duration duration = ros::Time::now() - last_time;
        controller_manager.update(ros::Time::now(), duration);
        last_time = ros::Time::now();

        armadillo_hw.write();

        ros::Duration(1.0 / (2.0 * LOOP_HZ)).sleep();

        ros::spinOnce;
    }
}