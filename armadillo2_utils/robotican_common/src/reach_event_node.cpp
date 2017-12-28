//
// Created by tom on 26/06/16.
//

#include <cmath>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "goal_node");
    ros::NodeHandle nodeHandle("~");
    tf::TransformListener listener;
    tf::StampedTransform transform;

    ros::Publisher marker_pub=nodeHandle.advertise<visualization_msgs::Marker>("goal_marker", 2, true);

    ros::Rate loopRate(10);

    double goal_x,goal_y,goal_z,goal_tol;
    std::string moving_frame,sys_cmd;
    nodeHandle.getParam("goal_x", goal_x);
    nodeHandle.getParam("goal_y", goal_y);
    nodeHandle.getParam("goal_z", goal_z);
    nodeHandle.getParam("goal_tol", goal_tol);
    nodeHandle.getParam("moving_frame", moving_frame);
    nodeHandle.getParam("sys_cmd", sys_cmd);

    tf::Vector3 goal(goal_x,goal_y,goal_z);
    bool inPose = false;
#ifdef DEBUG_REACH_EVENT
    ROS_INFO("[%s]:node is active", ros::this_node::getName().c_str());
#endif

    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.ns = "/";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = goal_x;
    marker.pose.position.y = goal_y;
    marker.pose.position.z = goal_z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker_pub.publish(marker);

    while (ros::ok()) {

        try {
            listener.lookupTransform("map", moving_frame.c_str(), ros::Time(0), transform);

            tf::Vector3 mf=transform.getOrigin();
            double dist=mf.distance(goal);
            // if (dist<1.0) ROS_INFO("dist: %f",dist);

            if ((dist <= goal_tol)&&(!inPose)) {
                inPose = true;
#ifdef DEBUG_REACH_EVENT
                ROS_INFO("GOAL REACHED");
#endif
                FILE *process=popen(sys_cmd.c_str(),"r");
                if (process!=NULL) ROS_INFO("System command done");
                else ROS_ERROR("System command fail");
                ros::shutdown();
            }
        }
        catch (tf::TransformException ex) {
            // ROS_ERROR("Goal node error: %s",ex.what());
        }


        ros::spinOnce();
        loopRate.sleep();
    }
}
