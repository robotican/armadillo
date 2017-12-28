//
// Created by tom on 03/05/16.
//

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

class JoyTeleop {
private:
    ros::NodeHandle _nodeHandle;
    ros::Subscriber _joyListener;
    ros::Publisher  _cmd;

    float _linearScale;
    float _angularScale;
    int _deadManIndex;
    int _linearAxisIndex;
    int _angularAxisIndex;

    void joyCallback(const sensor_msgs::Joy::ConstPtr &msg) {
        bool isDeadManActive = msg->buttons[_deadManIndex] == 1;
        if(isDeadManActive) {
            geometry_msgs::Twist twist;
            twist.linear.x = msg->axes[_linearAxisIndex] * _linearScale;
            twist.angular.z = msg->axes[_angularAxisIndex] * _angularScale;
            _cmd.publish(twist);
        }
    }

public:
    JoyTeleop() {
        if(!_nodeHandle.getParam("drive_joy_teleop_linear_axis", _linearAxisIndex)
           || !_nodeHandle.getParam("drive_joy_teleop_angular_axis", _angularAxisIndex)
           || !_nodeHandle.getParam("drive_joy_teleop_deadman_button", _deadManIndex)
           || !_nodeHandle.getParam("drive_joy_teleop_linear_max_vel", _linearScale)
           || !_nodeHandle.getParam("drive_joy_teleop_angular_max_vel", _angularScale)) {
            ROS_ERROR("[%s]: Missing parameter, the requird parameters are: "
                              "drive_joy_teleop_linear_axis, drive_joy_teleop_angular_axis"
                              ", drive_joy_teleop_deadman_button , drive_joy_teleop_linear_max_vel"
                              ", drive_joy_teleop_angular_max_vel", ros::this_node::getName().c_str());
            ros::shutdown();
        }
        else {
            _joyListener = _nodeHandle.subscribe<sensor_msgs::Joy>("joy", 10, &JoyTeleop::joyCallback, this);
            _cmd = _nodeHandle.advertise<geometry_msgs::Twist>("joy_vel", 10);
        }
    }

    void run() {
        ros::spin();
    }

    

};

int main(int argc, char **argv) {
    ros::init(argc, argv, "joy_teleop_node");
    JoyTeleop joyTeleop;
    joyTeleop.run();
    return 0;
}