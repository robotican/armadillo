


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

