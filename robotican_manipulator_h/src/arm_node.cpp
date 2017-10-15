//
// Created by tom on 05/06/16.
//

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <dynamixel_msgs/JointState.h>
#include <controller_manager/controller_manager.h>
#include <robotican_hardware_interface/dynamixel_pro_controller.h>
#include <hardware_interface/robot_hw.h>

class Arm : public hardware_interface::RobotHW {
private:
    ros::Time _time;
    ros::NodeHandle _nodeHandle;
    hardware_interface::JointStateInterface _jointStateInterface;
    hardware_interface::PosVelJointInterface _posVelJointInterface;
    hardware_interface::PositionJointInterface _positionJointInterface;
    dynamixel_pro_controller::DynamixelProController _controller;
    ros::Publisher _leftFingerCmd;
    ros::Publisher _rightFingerCmd;
    ros::Subscriber _leftFingerState;
    ros::Subscriber _rightFingerState;
    std::pair<std::string, dynamixel_pro_controller::JointInfo_t> _leftFingerInfo;
    std::pair<std::string, dynamixel_pro_controller::JointInfo_t> _rightFingerInfo;
    bool _first[2];

    void leftFingerCallback(const dynamixel_msgs::JointState::ConstPtr &msg) {
        _leftFingerInfo.second.position = msg->current_pos;
        _leftFingerInfo.second.velocity = msg->velocity;
        _leftFingerInfo.second.effort = msg->load;
        if(!_first[1]) {
            _leftFingerInfo.second.cmd_pos = _leftFingerInfo.second.position;
            _first[1] = true;
        }
    }
    void rightFingerCallback(const dynamixel_msgs::JointState::ConstPtr &msg) {
        _rightFingerInfo.second.position = msg->current_pos;
        _rightFingerInfo.second.velocity = msg->velocity;
        _rightFingerInfo.second.effort = msg->load;
        if(!_first[0]) {
            _rightFingerInfo.second.cmd_pos = _rightFingerInfo.second.position;
            _first[0] = true;
        }
    }
public:

    Arm() : _jointStateInterface(), _posVelJointInterface(), _positionJointInterface(), _controller(&_jointStateInterface, &_posVelJointInterface) {
        _first[0] = _first[1] = false;
        _time = ros::Time::now();
        std::string  leftFingerPubTopic, leftFingerSubTopic, leftFingerJointName,
                rightFingerPubTopic, rightFingerSubTopic, rightFingerJointName;
        if(!_nodeHandle.getParam("left_finger_topic_pub", leftFingerPubTopic) ||
           !_nodeHandle.getParam("left_finger_topic_sub", leftFingerSubTopic) ||
           !_nodeHandle.getParam("left_finger_joint", leftFingerJointName) ||
           !_nodeHandle.getParam("right_finger_topic_pub", rightFingerPubTopic) ||
           !_nodeHandle.getParam("right_finger_topic_sub", rightFingerSubTopic) ||
           !_nodeHandle.getParam("right_finger_joint", rightFingerJointName)) {/* parameters that must be instalize for the robot to work*/
            ROS_ERROR("[%s]: Invalid parameters", ros::this_node::getName().c_str());
            ros::shutdown();
        }


        _leftFingerCmd = _nodeHandle.advertise<std_msgs::Float64>(leftFingerPubTopic, 10);
        _rightFingerCmd = _nodeHandle.advertise<std_msgs::Float64>(rightFingerPubTopic, 10);

        _leftFingerState = _nodeHandle.subscribe<dynamixel_msgs::JointState>(leftFingerSubTopic, 10, &Arm::leftFingerCallback, this);
        _rightFingerState = _nodeHandle.subscribe<dynamixel_msgs::JointState>(rightFingerSubTopic, 10, &Arm::rightFingerCallback, this);

        _leftFingerInfo = std::pair<std::string, dynamixel_pro_controller::JointInfo_t>(leftFingerJointName, dynamixel_pro_controller::JointInfo_t());
        _rightFingerInfo = std::pair<std::string, dynamixel_pro_controller::JointInfo_t>(rightFingerJointName, dynamixel_pro_controller::JointInfo_t());

        hardware_interface::JointStateHandle leftJointStateHandle(_leftFingerInfo.first,
                                                                  &_leftFingerInfo.second.position,
                                                                  &_leftFingerInfo.second.velocity,
                                                                  &_leftFingerInfo.second.effort );

        hardware_interface::JointStateHandle rightJointStateHandle(_rightFingerInfo.first,
                                                                   &_rightFingerInfo.second.position,
                                                                   &_rightFingerInfo.second.velocity,
                                                                   &_rightFingerInfo.second.effort );

        _jointStateInterface.registerHandle(leftJointStateHandle);
        _jointStateInterface.registerHandle(rightJointStateHandle);

        hardware_interface::JointHandle leftJointHandle(_jointStateInterface.getHandle(_leftFingerInfo.first), &_leftFingerInfo.second.cmd_pos);
        hardware_interface::JointHandle rightJointHandle(_jointStateInterface.getHandle(_rightFingerInfo.first), &_rightFingerInfo.second.cmd_pos);

        _positionJointInterface.registerHandle(leftJointHandle);
        _positionJointInterface.registerHandle(rightJointHandle);

        registerInterface(&_jointStateInterface);
        registerInterface(&_posVelJointInterface);
        registerInterface(&_positionJointInterface);
    }

    ros::Time getTime() {
        return ros::Time::now();
    }

    ros::Duration getPeriod() {
        ros::Time now = ros::Time::now();
        ros::Duration period = now - _time;
        _time = now;
        return period;
    }

    void read() {
        _controller.read();
    }

    void write() {
        _controller.write();
        std_msgs::Float64 leftMsg, rightMsg;

        if(_first[0] && _first[1]) {
            leftMsg.data = _leftFingerInfo.second.cmd_pos;
            rightMsg.data = _rightFingerInfo.second.cmd_pos;

            _leftFingerCmd.publish(leftMsg);
            _rightFingerCmd.publish(rightMsg);
        }

    }
};


int main(int argc, char **argv) {
    ros::init(argc, argv, "arm_node");
    Arm arm;
    controller_manager::ControllerManager controllerManager(&arm);
    ros::AsyncSpinner asyncSpinner(2);
    asyncSpinner.start();
    ros::Rate loopRate(100);

    while (ros::ok()) {
        arm.read();
        controllerManager.update(arm.getTime(), arm.getPeriod());
        arm.write();
        loopRate.sleep();
    }


    return 0;
}