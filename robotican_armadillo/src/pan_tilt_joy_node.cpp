//
// Created by tom on 02/05/16.
//

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float64MultiArray.h>
#include <urdf/model.h>
#include <trajectory_msgs/JointTrajectory.h>

//#define DEBUG_JOY

typedef boost::shared_ptr<const urdf::Joint> UrdfJointConstPtr;

class PanTiltJoy {
private:
    int _panAxisIndex;
    int _tiltAxisIndex;
    float _deadManButtonIndex;
    float _zeroButtonIndex;

    float _incTilt;                 //The increment value for tilt.
    float _incPan;                  //The increment value for pan.

    float _tiltPos;                 //The tilt current position.
    float _panPos;                  //The pan current position.
    bool _deadManButtonActive;      //True if the dead man button is press.
    bool _zeroButtonActive;         //True if the zero button is press.

    UrdfJointConstPtr tiltLim;
    UrdfJointConstPtr panLim;

    ros::NodeHandle _nodeHandle;
    ros::Publisher _panTiltCommand; //This object is for sending command to the pan and tilt.
    ros::Subscriber _joySub;        //Listener to the joystick state.

    /*
     * This method is for the joystick state.
     */
    void joyCallback(const sensor_msgs::Joy::ConstPtr &msg) {
        _deadManButtonActive = msg->buttons[_deadManButtonIndex] == 1;
        _zeroButtonActive = msg->buttons[_zeroButtonIndex] == 1;
        if (_deadManButtonActive) {
            if (msg->axes[_tiltAxisIndex] > 0) {
                if (tiltLim->limits->lower <= (_tiltPos - _incTilt)) _tiltPos -= _incTilt;
            }
            else if (msg->axes[_tiltAxisIndex] < 0) {
                if (tiltLim->limits->upper >= (_tiltPos + _incTilt)) _tiltPos += _incTilt;
            }
            if (msg->axes[_panAxisIndex] > 0) {
                if (panLim->limits->upper >= (_panPos + _incPan)) _panPos += _incPan;

            }
            else if (msg->axes[_panAxisIndex] < 0) {
                if (panLim->limits->lower <= (_panPos - _incPan)) _panPos -= _incPan;
            }
            if (_zeroButtonActive) {
                _panPos = _tiltPos = 0.0;
            }

        }
    }

public:
    PanTiltJoy() {
        std::string panTiltTopic, joySubTopic;
        _tiltPos = _panPos = 0;
        _deadManButtonActive = _zeroButtonActive = false;
        boost::shared_ptr<urdf::Model> urdf(new urdf::Model);
        if (!urdf->initParam("robot_description")) {
            ROS_ERROR("[%s]: Need to have urdf model.", ros::this_node::getName().c_str());
            ros::shutdown();
        }
        tiltLim = urdf->getJoint("head_tilt_joint");
        panLim = urdf->getJoint("head_pan_joint");

        if (!tiltLim) {
            ROS_ERROR("[%s]: Could not find head_tilt_joint.", ros::this_node::getName().c_str());
            ros::shutdown();
        }

        if (!panLim) {
            ROS_ERROR("[%s]: Could not find head_pan_joint.", ros::this_node::getName().c_str());
            ros::shutdown();
        }

        // Validation check.
        if (!_nodeHandle.getParam("pan_tilt_topic", panTiltTopic)
            || !_nodeHandle.getParam("joy_sub_topic", joySubTopic)
            || !_nodeHandle.getParam("joy_pan_axis", _panAxisIndex)
            || !_nodeHandle.getParam("joy_tilt_axis", _tiltAxisIndex)
            || !_nodeHandle.getParam("increament_tilt", _incTilt)
            || !_nodeHandle.getParam("increament_pan", _incPan)
            || !_nodeHandle.getParam("joy_deadman_button", _deadManButtonIndex)
            || !_nodeHandle.getParam("zero_button", _zeroButtonIndex)) {
            ROS_ERROR(
                    "[%s]: Requird: pan_tilt_topic,  joy_sub_topic, joy_pan_axis, joy_tilt_axis, increament_tilt, increament_pan , zero_button, joy_deadman_button parameters",
                    ros::this_node::getName().c_str());
            ros::shutdown();
        }
        else {
            _panTiltCommand = _nodeHandle.advertise<trajectory_msgs::JointTrajectory>(panTiltTopic, 10);
            _joySub = _nodeHandle.subscribe<sensor_msgs::Joy>(joySubTopic, 10, &PanTiltJoy::joyCallback, this);
        }
    }

    // Method which publish position to the pan_tilt.
    void run() {
        ros::Rate loopRate(50);
        trajectory_msgs::JointTrajectory positions;
        positions.joint_names.push_back("head_pan_joint");
        positions.joint_names.push_back("head_tilt_joint");
        positions.points.resize(1);
        positions.points[0].positions.resize(2);
        positions.points[0].velocities.push_back(0);
        positions.points[0].velocities.push_back(0);
        while (ros::ok()) {

            if (_deadManButtonActive) {
                positions.points[0].time_from_start = ros::Duration(0.1);
                positions.points[0].positions[0] = _panPos;
                positions.points[0].positions[1] = _tiltPos;
#ifdef DEBUG_JOY
                ROS_INFO_STREAM("[" << ros::this_node::getName() << "]: " << positions);
#endif
                _panTiltCommand.publish(positions);
            }
            ros::spinOnce();
            loopRate.sleep();
        }
    }

};

int main(int argc, char **argv) {
    ros::init(argc, argv, "pan_tilt_joy_node");
    PanTiltJoy panTiltJoy;
    panTiltJoy.run();
    return 0;
}
