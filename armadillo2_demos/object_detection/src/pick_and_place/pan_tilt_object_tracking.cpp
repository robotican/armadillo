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
#include <std_msgs/Float64MultiArray.h>
#include <tf/transform_listener.h>
#include <actionlib/client/simple_action_client.h>
#include <armadillo2_msgs/PointHeadAction.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <std_srvs/SetBool.h>

// Our Action interface type, provided as a typedef for convenience
typedef actionlib::SimpleActionClient<armadillo2_msgs::PointHeadAction> PointHeadClient;


void callback(const geometry_msgs::PoseStamped::ConstPtr& pose);
PointHeadClient* point_head_client;

int main(int argc, char **argv) {
    ros::init(argc, argv, "pan_tilt_object_trackking");
    ros::NodeHandle n;
    ros::NodeHandle pn("~");

    std::string object_name;
    pn.param<std::string>("object_name", object_name, "can");


    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    //Initialize the client for the Action interface to the head controller
    point_head_client = new PointHeadClient("/pan_tilt_trajectory_controller/point_head_action", true);

    //wait for head controller action server to come up
    while(!point_head_client->waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the point_head_action server to come up");
    }

    ROS_INFO("Ready to track!");
    ros::Rate r(10);

    ros::ServiceClient uc_client = n.serviceClient<std_srvs::SetBool>("update_collision_objects");
    ROS_INFO("Waiting for update_collision service...");
    uc_client.waitForExistence();
    std_srvs::SetBool srv;
    srv.request.data=true;
    uc_client.call(srv);

    while (ros::ok()) {

        std::vector<std::string> ids;
        ids.push_back(object_name);
        std::map<std::string, moveit_msgs::CollisionObject> poses=planning_scene_interface.getObjects(ids);
        std::map<std::string, moveit_msgs::CollisionObject>::iterator it;

        it = poses.find(object_name);

        if (it != poses.end()) {
            moveit_msgs::CollisionObject obj=it->second;
            //the goal message we will be sending
            armadillo2_msgs::PointHeadGoal goal;

            //the target point, expressed in the requested frame

            geometry_msgs::PointStamped point;

            point.header.frame_id =  obj.header.frame_id;
            point.header.stamp=obj.header.stamp;
            point.point = obj.primitive_poses[0].position;

            goal.target = point;

            //we are pointing the high-def camera frame
            //(pointing_axis defaults to X-axis)
            goal.pointing_frame = "kinect2_depth_frame";
            goal.pointing_axis.x = 1;
            goal.pointing_axis.y = 0;
            goal.pointing_axis.z = 0;

            //take at least 0.5 seconds to get there
            goal.min_duration = ros::Duration(0.5);

            //and go no faster than 0.2 rad/s
            goal.max_velocity = 0.3;

            //send the goal
            point_head_client->sendGoal(goal);

            //wait for it to get there (abort after 2 secs to prevent getting stuck)
            point_head_client->waitForResult(goal.min_duration );
        }

        ros::spinOnce();
        r.sleep();
    }


    return 0;


}
