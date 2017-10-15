//
// Created by tom on 02/05/16.
//

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JointState.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

typedef  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ActionClient;
//#define DEBUG_TORSO_INC
class JoyIncrement {
private:
    ros::NodeHandle _nodeHandle;
    ActionClient _actionClient;     //Action client which send the goal to the torso
    ros::Subscriber _joySub;        // Joystick Listener.
    ros::Subscriber _jointStates;   // Joint status Listener, for the position if the torso.

    bool _isStop;                   //True if the dead man button is release or if none of the torso button is press.
    float _incTorso;                 //Increment
    double _torsoPos;                //The torso current position.
    int _upButtonIndex;
    int _downButtonIndex;
    int _deadManButtonIndex;

    /*
     * This method handle the joystick reading.
     */
    void joyCallback(const sensor_msgs::Joy::ConstPtr &msg) {
        bool isUpActive = (bool)msg->buttons[_upButtonIndex],
             isDownActive = (bool)msg->buttons[_downButtonIndex],
             isDeadManButtonActive = (bool)msg->buttons[_deadManButtonIndex];
        if(isDeadManButtonActive) {
            if (isUpActive) {
                sendGoal(_torsoPos + _incTorso);
                _isStop = false;
            }
            else if (isDownActive) {
                sendGoal(_torsoPos - _incTorso);
                _isStop = false;
            }
            else if(!_isStop){
                sendGoal(_torsoPos);
                _isStop = true;
            }
        }
        else {
            if(!_isStop) {
                sendGoal(_torsoPos);
                _isStop = true;
            }
        }

    }
    /*
     * This method get the torso current position.
     */
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg){
        size_t size = msg->name.size();
        bool found = false;
        for(int i = 0; i < size && !found; ++i) {
            if(msg->name[i] == "torso_joint") {
                _torsoPos = msg->position[i];
                found = true;
            }
        }
        if(!found) {
            ROS_ERROR("[%s]: torso_joint not found", ros::this_node::getName().c_str());
        }
    }

    /*
     * This method will get call after the action server s finish.
     */
    void doneCallback(const actionlib::SimpleClientGoalState &state, const control_msgs::FollowJointTrajectoryResult::ConstPtr &result) {
#ifdef DEBUG_TORSO_INC
        ROS_INFO("Finished in state [%s]", state.toString().c_str());
#endif
    }

    /*
     * This method construct the goal and send it to the action server.
     */
    void sendGoal(double elevPos) {
//        ROS_INFO("[%s]: %f", ros::this_node::getName().c_str(), elevPos);
        control_msgs::FollowJointTrajectoryGoal goal;
        goal.goal_time_tolerance = ros::Duration(5.0);
        goal.trajectory.header.frame_id = "base_link";
        goal.trajectory.joint_names.push_back("torso_joint");
        trajectory_msgs::JointTrajectoryPoint point;
        point.time_from_start = ros::Duration(5.0);
        point.positions.push_back(elevPos);
        goal.trajectory.points.push_back(point);
        _actionClient.sendGoal(goal, boost::bind(&JoyIncrement::doneCallback, this, _1, _2),
                               ActionClient::SimpleActiveCallback(), ActionClient::SimpleFeedbackCallback());
    }


public:
    JoyIncrement() : _actionClient("torso_trajectory_controller/follow_joint_trajectory", true) {
        _isStop = true;
        if(_actionClient.waitForServer(ros::Duration(20.0))) {
            if(!_nodeHandle.getParam("torso_increment", _incTorso)
               ||!_nodeHandle.getParam("torso_up_button", _upButtonIndex)
               ||!_nodeHandle.getParam("torso_down_button", _downButtonIndex)
               ||!_nodeHandle.getParam("joy_deadman_button", _deadManButtonIndex)) {
                ROS_ERROR("[%s]: Missing parameters, the requird parameters are: torso_increment,  torso_up_button, torso_down_button, joy_deadman_button", ros::this_node::getName().c_str());
                ros::shutdown();
            }
            else {
                _joySub = _nodeHandle.subscribe<sensor_msgs::Joy>("joy", 10, &JoyIncrement::joyCallback, this);
                _jointStates = _nodeHandle.subscribe<sensor_msgs::JointState>("joint_states", 10, &JoyIncrement::jointStateCallback, this);
            }
        }
        else {
            ROS_ERROR("[%s]: Server not responding", ros::this_node::getName().c_str());
            ros::shutdown();
        }
    }
    void run() {
        ros::spin();
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "joy_increment_node");
    JoyIncrement joyIncrement;
    joyIncrement.run();
    return 0;
}
