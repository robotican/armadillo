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
#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>
#include <armadillo2_msgs/SwitchCamTopic.h>


int main(int argc, char **argv) {

    ros::init(argc, argv, "demo_pick_node");
    ros::NodeHandle n;

    ros::ServiceClient drive_client = n.serviceClient<std_srvs::Trigger>("drive2object_go");
    ros::ServiceClient pick_client = n.serviceClient<std_srvs::Trigger>("pick_go");
    ros::ServiceClient sw_client = n.serviceClient<armadillo2_msgs::SwitchCamTopic>("switch_pcl_topic");
   // ros::ServiceClient uc_client = n.serviceClient<std_srvs::SetBool>("update_collision_objects");

     ROS_INFO("Waiting for services...");
    drive_client.waitForExistence();
    pick_client.waitForExistence();
    sw_client.waitForExistence();
  //  uc_client.waitForExistence();

    armadillo2_msgs::SwitchCamTopic sw_srv;
    sw_srv.request.num=1;
    sw_client.call(sw_srv);

    ros::Duration(20).sleep();
     ROS_INFO("Ready!");
    std_srvs::Trigger drive_srv;
    if (drive_client.call(drive_srv))
    {
        ROS_INFO("drive2object response: %s", drive_srv.response.message.c_str());
        if (drive_srv.response.success) {


            sw_srv.request.num=2;
            sw_client.call(sw_srv);



            ros::Duration(5).sleep();

            std_srvs::Trigger pick_srv;
            if (pick_client.call(pick_srv)) {
                ROS_INFO("pick response: %s", pick_srv.response.message.c_str());
                if (pick_srv.response.success)  ROS_INFO("Done!");
            }
            else ROS_ERROR("Failed to call pick service");

        }
    }
    else ROS_ERROR("Failed to call drive2object service");

ros::shutdown();
    return 0;
}

