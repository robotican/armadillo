//
// Created by tom on 20/09/16.
//


#include <ros/ros.h>
#include <boost/asio.hpp>
#include <std_srvs/Trigger.h>
#include <boost/smart_ptr.hpp>


#define MAX_LEN 128
using boost::asio::ip::tcp;

typedef boost::shared_ptr<tcp::socket> socket_ptr;

std_srvs::Trigger::Response pickAndPlace(ros::ServiceClient &pickAndPlaceClient);
void recover(ros::ServiceClient &pickAndPlaceClient, tcp::socket &client);
void changeColor(std::string colorName);

int main(int argc, char** argv) {
    ros::init(argc, argv, "pick_and_place_client");
    ros::NodeHandle nodeHandle;
    ros::NodeHandle nodeHandlePrivate("~");
    ros::ServiceClient pickAndPlaceClient = nodeHandle.serviceClient<std_srvs::Trigger>("pick_go");
    pickAndPlaceClient.waitForExistence();

    std::string ip;
    nodeHandlePrivate.param<std::string>("server_ip", ip, "localhost");

    boost::asio::io_service io_service;
    tcp::resolver resolver(io_service);
    tcp::resolver::query query(tcp::v4(), ip.c_str() , "5001");
    tcp::resolver::iterator iterator = resolver.resolve(query);
    tcp::socket connection(io_service);
    connection.connect(*iterator);

    ROS_INFO("[%s]: Connected to server", ros::this_node::getName().c_str());
    std::string colorName = "red";
    while(ros::ok()) {
        char data[MAX_LEN] = {'\0'};
        boost::asio::read(connection, boost::asio::buffer(data, 3));
        if (std::strcmp(data, "go\n") == 0) {
            if (pickAndPlace(pickAndPlaceClient).success) {
                boost::asio::write(connection, boost::asio::buffer("go\n", 3));
            } else {
                recover(pickAndPlaceClient, connection);
            }
        } else {
            ROS_WARN("[%s]: Unknown syntax: %s", ros::this_node::getName().c_str(), data);
        }
    }



    return 0;
}

std_srvs::Trigger::Response pickAndPlace(ros::ServiceClient &pickAndPlaceClient) {
    std_srvs::Trigger::Request pickAndPlaceReq;
    std_srvs::Trigger::Response pickAndPlaceRes;

    if (pickAndPlaceClient.call(pickAndPlaceReq, pickAndPlaceRes)) {
        ROS_INFO_STREAM(pickAndPlaceRes);
    }
    return pickAndPlaceRes;
}

void recover(ros::ServiceClient &pickAndPlaceClient, tcp::socket &client) {
    ROS_WARN("[%s]: plan failed type 'r' to re-plan or 'q' to quit", ros::this_node::getName().c_str());
    char choice;
    do {
        std::cin >> choice;
        if(choice == 'r') {
            if (pickAndPlace(pickAndPlaceClient).success) {
                write(client, boost::asio::buffer("go\n", 3));
                break;
            }
        } else if(choice == 'q') {
            ros::shutdown();
            break;
        }
    } while(ros::ok());
}

void changeColor(std::string colorName) {
    std::string baseCmd = "rosrun dynamic_reconfigure dynparam load /find_object_node `rospack find robotican_demos`/config/";
    if(colorName == "red") {

        FILE *process = popen((baseCmd + "red_object.yaml").c_str(), "r");
        ros::Duration(1.5).sleep();
        if(process != 0) {
            ROS_INFO("[%s]: Change to red", ros::this_node::getName().c_str());
        }
    }
    else if(colorName == "green") {
        FILE *process = popen((baseCmd + "green_object.yaml").c_str(), "r");
        ros::Duration(1.5).sleep();
        if(process != 0) {
            ROS_INFO("[%s]: Change to green", ros::this_node::getName().c_str());
        }
    } else if(colorName == "blue") {
        FILE *process = popen((baseCmd + "blue_object.yaml").c_str(), "r");
        ros::Duration(1.5).sleep();
        if(process != 0) {
            ROS_INFO("[%s]: Change to blue", ros::this_node::getName().c_str());
        }
    }
}