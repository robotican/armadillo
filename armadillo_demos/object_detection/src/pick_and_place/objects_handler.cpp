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
#include <std_srvs/SetBool.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <iostream>
#include <sstream>
#include <kdl/chainfksolver.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/frames.hpp>
#include "kdl/chainfksolverpos_recursive.hpp"
#include "kdl_parser/kdl_parser.hpp"

#include "tf_conversions/tf_kdl.h"
#include <message_filters/subscriber.h>
#include <tf/message_filter.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>


bool update_cb(std_srvs::SetBool::Request  &req,std_srvs::SetBool::Response &res);
void handle_objects(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& markers);
std::string get_obj_id(int id);
shape_msgs::SolidPrimitive get_obj_shape(int id);
geometry_msgs::Pose get_obj_position(int id,geometry_msgs::Pose  pose);

bool update=false;



moveit::planning_interface::PlanningSceneInterface *planning_scene_interface_ptr;

//listening to alvar msgs, that are sent througth find object node


int main(int argc, char **argv) {
    ros::init(argc, argv, "objects_handler");
    ros::NodeHandle n;
    ros::NodeHandle pn("~");

    //TODO: READ OBJECTS DB
    std::string db_path;
    pn.param<std::string>("db_path", db_path, "??");

    ros::Subscriber ar_sub = n.subscribe("ar_pose_marker", 10, handle_objects);
    ros::Subscriber color_sub = n.subscribe("detected_objects", 10, handle_objects);

   //
    ros::ServiceServer service = n.advertiseService("update_collision_objects", update_cb);


    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    planning_scene_interface_ptr=&planning_scene_interface;

while (ros::ok()){
    //TODO: RATE+TIMEOUT
    //if (update) planning_scene_interface.removeCollisionObjects();

    ros::spinOnce();
}
    return 0;
}

std::string get_obj_id(int id) { //TODO: FROM DB

   /* std::ostringstream os;
    os<<"object_";
      os << id;
    return os.str();
    */
    if (id==1) return "can";
    else return "table";

}

shape_msgs::SolidPrimitive get_obj_shape(int id) { //TODO: FROM DB

shape_msgs::SolidPrimitive object_primitive;
if (id==1) {
object_primitive.type = object_primitive.CYLINDER;
object_primitive.dimensions.resize(2);
object_primitive.dimensions[0] = 0.145;
object_primitive.dimensions[1] = 0.03;
}
else {
   object_primitive.type = object_primitive.BOX;
   object_primitive.dimensions.resize(3);
   object_primitive.dimensions[0] = 0.3;
   object_primitive.dimensions[1] = 0.3;
     object_primitive.dimensions[2] = 0.01;
}
    return object_primitive;
}

geometry_msgs::Pose get_obj_position(int id,geometry_msgs::Pose  pose){ //TODO: FROM DB

    //if (id==1) pose.position.x+=0.015;
    return pose;
 }

void handle_objects(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& markers)
{
	//build collisions for moveit
    std::vector<moveit_msgs::CollisionObject> col_objects;

    for (int i=0;i<markers->markers.size();i++) {

        ar_track_alvar_msgs::AlvarMarker m=markers->markers[i];

        moveit_msgs::CollisionObject obj;
        obj.header.frame_id = m.header.frame_id;
        obj.id = get_obj_id(m.id);
        obj.primitives.push_back(get_obj_shape(m.id));

        obj.primitive_poses.push_back(get_obj_position(m.id,m.pose.pose));
        obj.operation=obj.ADD;

        col_objects.push_back(obj);

        if (update) planning_scene_interface_ptr->addCollisionObjects(col_objects);

    }
}


bool update_cb(std_srvs::SetBool::Request  &req,
               std_srvs::SetBool::Response &res) {

    update=req.data;
    res.success=true;
    if (update)  res.message="update collision is ON";
    else res.message="update collision is OFF";
    return true;
}
