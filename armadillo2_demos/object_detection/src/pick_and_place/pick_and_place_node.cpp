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
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <geometry_msgs/PointStamped.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <std_msgs/Empty.h>
#include <tf/transform_listener.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/GripperCommandAction.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/kinematics_metrics/kinematics_metrics.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <std_srvs/SetBool.h>
#include <moveit_msgs/PickupAction.h>
#include <moveit_msgs/PlaceAction.h>
#include <std_msgs/Float64MultiArray.h>

#define MAX_BOARD_PLACE 0.05
typedef actionlib::SimpleActionClient<moveit_msgs::PickupAction> PickClient;
typedef actionlib::SimpleActionClient<moveit_msgs::PlaceAction> PlaceClient;

struct Point_t {
    float x;
    float y;

    Point_t() {
        x = y = 0.0;
    }

};

void look_down();

bool set_collision_update(bool state);

moveit_msgs::PickupGoal BuildPickGoal(const std::string &objectName);

moveit_msgs::PlaceGoal buildPlaceGoal(const std::string &objectName);

bool pickAndPlaceCallBack(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

double randBetweenTwoNum(int max, int min);

bool exec = false;
ros::ServiceClient *uc_client_ptr;
ros::Publisher pub_controller_command;
ros::Publisher grp_pos_pub;
Point_t point;
  std::string startPositionName ;
moveit::planning_interface::MoveGroupInterface *group_ptr;
  
  
int main(int argc, char **argv) {

    ros::init(argc, argv, "pick_and_plce_node");
    ros::AsyncSpinner spinner(2);
    spinner.start();
    srand((unsigned int) time(NULL));
    ros::NodeHandle n;
    ros::NodeHandle pn("~");
    std::string object_name,table_name;
  



    pn.param<std::string>("start_position_name", startPositionName, "pre_grasp2");
    pn.param<std::string>("object_name", object_name, "can");
    pn.param<std::string>("table_name", table_name, "table");

    ros::ServiceServer pickAndPlace = n.advertiseService("pick_go", &pickAndPlaceCallBack);
    ROS_INFO("Hello");
    moveit::planning_interface::MoveGroupInterface group("arm");
    group_ptr=&group;
    //Config move group
    //group.setMaxVelocityScalingFactor(0.1);
    //group.setMaxAccelerationScalingFactor(0.5);
    group.setPlanningTime(10.0);
    group.setNumPlanningAttempts(50);
    group.setPlannerId("RRTConnectkConfigDefault");
    group.setPoseReferenceFrame("base_footprint");

    group.setStartStateToCurrentState();
    group.setNamedTarget(startPositionName);
    moveit::planning_interface::MoveGroupInterface::Plan startPosPlan;
    if(group.plan(startPosPlan)) { //Check if plan is valid
        group.execute(startPosPlan);
        pub_controller_command = n.advertise<trajectory_msgs::JointTrajectory>("/pan_tilt_trajectory_controller/command", 2);
 //grp_pos_pub = n.advertise<std_msgs::Float64MultiArray>("pan_tilt_controller/command", 5);
        ROS_INFO("Waiting for the moveit action server to come up");
        ros::ServiceClient uc_client = n.serviceClient<std_srvs::SetBool>("update_collision_objects");
        ROS_INFO("Waiting for update_collision service...");
        uc_client.waitForExistence();
        uc_client_ptr = &uc_client;
        set_collision_update(true);
        ros::Duration(5.0).sleep();
        look_down();
        ROS_INFO("Looking down...");
        ROS_INFO("Ready!");
    }
    else {
        ROS_ERROR("Error");
    }
    ros::waitForShutdown();
    return 0;
}

moveit_msgs::PlaceGoal buildPlaceGoal(const std::string &objectName) {
    moveit_msgs::PlaceGoal placeGoal;
    placeGoal.group_name = "arm";
    placeGoal.attached_object_name = objectName;
    placeGoal.place_eef = false;
    placeGoal.support_surface_name = "table";
    placeGoal.planner_id = "RRTConnectkConfigDefault";
    placeGoal.allowed_planning_time = 15.0;
    placeGoal.planning_options.replan = true;
    placeGoal.planning_options.replan_attempts = 5;
    placeGoal.planning_options.replan_delay = 2.0;
    placeGoal.planning_options.planning_scene_diff.is_diff = true;
    placeGoal.planning_options.planning_scene_diff.robot_state.is_diff = true;
    std::vector<moveit_msgs::PlaceLocation> locations;
    moveit_msgs::PlaceLocation location;
    location.pre_place_approach.direction.header.frame_id = "/base_footprint";
    location.pre_place_approach.direction.vector.z = -1.0;
    location.pre_place_approach.min_distance = 0.1;
    location.pre_place_approach.desired_distance = 0.2;

    location.post_place_retreat.direction.header.frame_id = "/gripper_link";
    location.post_place_retreat.direction.vector.x = -1.0;
    location.post_place_retreat.min_distance = 0.0;
    location.post_place_retreat.desired_distance = 0.2;

    location.place_pose.header.frame_id = placeGoal.support_surface_name;
    bool inTheBoard;
    do {
        inTheBoard = true;
        location.place_pose.pose.position.x = randBetweenTwoNum(10, -10);
        if(fabs(point.x + location.place_pose.pose.position.x) >= MAX_BOARD_PLACE) {
            inTheBoard = false;
        }
    } while (!inTheBoard);

    do {
        inTheBoard = true;
        location.place_pose.pose.position.y = randBetweenTwoNum(5, -5);
        if(fabs(point.y + location.place_pose.pose.position.y) >= MAX_BOARD_PLACE) {
            inTheBoard = false;
        }
    } while (!inTheBoard);

    location.place_pose.pose.position.z = 0.13;
    location.place_pose.pose.orientation.w = 1.0;

    locations.push_back(location);
    placeGoal.place_locations = locations;
    return placeGoal;
}

moveit_msgs::PickupGoal BuildPickGoal(const std::string &objectName) {
    moveit_msgs::PickupGoal goal;
    goal.target_name = objectName;
    goal.group_name = "arm";
    goal.end_effector = "eef";
    goal.allowed_planning_time = 15.0;
    goal.planning_options.replan_delay = 2.0;
    goal.planning_options.planning_scene_diff.is_diff = true;
    goal.planning_options.planning_scene_diff.robot_state.is_diff = true;
    goal.planning_options.replan=true;
    goal.planning_options.replan_attempts=5;
    goal.planner_id = "RRTConnectkConfigDefault";

    goal.minimize_object_distance = true;
    moveit_msgs::Grasp g;
    g.max_contact_force = 1.0; //0.01
    g.grasp_pose.header.frame_id = goal.target_name;
    g.grasp_pose.pose.position.x = -0.02;
    g.grasp_pose.pose.position.y = 0.0;
    g.grasp_pose.pose.position.z = 0.0;
    g.grasp_pose.pose.orientation.x = 0.0;
    g.grasp_pose.pose.orientation.y = 0.0;
    g.grasp_pose.pose.orientation.z = 0.0;
    g.grasp_pose.pose.orientation.w = 1.0;

    g.pre_grasp_approach.direction.header.frame_id = "/base_footprint"; //gripper_link
    g.pre_grasp_approach.direction.vector.x = 1.0;
    g.pre_grasp_approach.min_distance = 0.01;
    g.pre_grasp_approach.desired_distance = 0.2;

    g.post_grasp_retreat.direction.header.frame_id = "/base_footprint"; //gripper_link
    g.post_grasp_retreat.direction.vector.z = 1.0;
    g.post_grasp_retreat.min_distance = 0.1;
    g.post_grasp_retreat.desired_distance = 0.2;

    g.pre_grasp_posture.joint_names.push_back("left_finger_joint");
    g.pre_grasp_posture.joint_names.push_back("right_finger_joint");
    g.pre_grasp_posture.points.resize(1);
    g.pre_grasp_posture.points[0].positions.resize(g.pre_grasp_posture.joint_names.size());
    g.pre_grasp_posture.points[0].positions[0] = 0.14;

    g.grasp_posture.joint_names = g.pre_grasp_posture.joint_names;
    g.grasp_posture.points.resize(1);
    g.grasp_posture.points[0].positions.resize(g.grasp_posture.joint_names.size());
    g.grasp_posture.points[0].positions[0] = 0.01;
    g.grasp_posture.points[0].effort.resize(g.grasp_posture.joint_names.size());
    g.grasp_posture.points[0].effort[0] = 0.6;
    goal.possible_grasps.push_back(g);
    return goal;
}

void look_down() {

    trajectory_msgs::JointTrajectory traj;
    traj.header.stamp = ros::Time::now();
    traj.joint_names.push_back("head_pan_joint");
    traj.joint_names.push_back("head_tilt_joint");
    traj.points.resize(1);
    traj.points[0].time_from_start = ros::Duration(1.0);
    std::vector<double> q_goal(2);
    q_goal[0]=0.0;
    q_goal[1]=0.7;
    traj.points[0].positions=q_goal;
    traj.points[0].velocities.push_back(0);
    traj.points[0].velocities.push_back(0);
    pub_controller_command.publish(traj);
/*
 std_msgs::Float64MultiArray grp_pos_msg;
    grp_pos_msg.data.push_back(0);
    grp_pos_msg.data.push_back(0.7);
    grp_pos_pub.publish(grp_pos_msg);
*/
}

bool set_collision_update(bool state){
    std_srvs::SetBool srv;
    srv.request.data=state;
    if (uc_client_ptr->call(srv))
    {
        ROS_INFO("update_colision response: %s", srv.response.message.c_str());
        return true;
    }
    else
    {
        ROS_ERROR("Failed to call service /find_objects_node/update_colision");
        return false;
    }

}

bool pickAndPlaceCallBack(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
    ros::NodeHandle pn("~");
    ros::NodeHandle n;
    std::string object_name;

    pn.param<std::string>("object_name", object_name, "can");

    ros::ServiceClient uc_client = n.serviceClient<std_srvs::SetBool>("update_collision_objects");
    std_srvs::SetBool disableColl;
    disableColl.request.data = false;

    if(uc_client.call(disableColl)) {
        ROS_INFO("update_colision response: OFF ");
    }

    PickClient pickClient("pickup", true);
    pickClient.waitForServer();

    moveit_msgs::PickupGoal pickGoal = BuildPickGoal(object_name);
    actionlib::SimpleClientGoalState pickStatus = pickClient.sendGoalAndWait(pickGoal);
    if(pickStatus != actionlib::SimpleClientGoalState::SUCCEEDED) {
        res.success = (unsigned char) false;
        res.message = pickStatus.getText();
        point.x = point.y = 0;
    }
    else {
        PlaceClient placeClient("place", true);
        placeClient.waitForServer();
        bool found = false;
        do {
            moveit_msgs::PlaceGoal placeGoal = buildPlaceGoal(object_name);
            actionlib::SimpleClientGoalState placeStatus = placeClient.sendGoalAndWait(placeGoal);
            if (placeStatus == actionlib::SimpleClientGoalState::SUCCEEDED) {
                found = true;
                res.success = (unsigned char) (found);
                res.message = placeStatus.getText();
                point.x += placeGoal.place_locations[0].place_pose.pose.position.x;
                point.y += placeGoal.place_locations[0].place_pose.pose.position.y;
            }
        } while(!found);
    }

     group_ptr->setStartStateToCurrentState();
    group_ptr->setNamedTarget(startPositionName);
    moveit::planning_interface::MoveGroupInterface::Plan startPosPlan;
    if(group_ptr->plan(startPosPlan)) { //Check if plan is valid
        group_ptr->execute(startPosPlan);
    }
    
    std_srvs::SetBool enableColl;
    enableColl.request.data = true;
    if(uc_client.call(enableColl)) {
        ROS_INFO("update_colision response: ON ");
    }

    return true;


}
double randBetweenTwoNum(int max, int min) {
    int randomNum = rand()%(max-min + 1) + min;
    return randomNum / 100.0;
}
