//
// Created by sub on 07/02/18.
//

#ifndef TRACK_FACE_BASE_TRACKER_H
#define TRACK_FACE_BASE_TRACKER_H

#define INDX_JOINT_PAN 0
#define INDX_JOINT_TILT 1
#define INDX_JOINT_LEFT_FINGER 2
#define INDX_JOINT_LEFT_WHEEL 3
#define INDX_JOINT_RIGHT_FINGER 4
#define INDX_JOINT_RIGHT_WHEEL 5
#define INDX_JOINT_ROTATION1 6
#define INDX_JOINT_ROTATION2 7
#define INDX_JOINT_SHOULDER1 8
#define INDX_JOINT_SHOULDER2 9
#define INDX_JOINT_SHOULDER3 10
#define INDX_JOINT_TORSO 11
#define INDX_JOIN_WRIST 12

#define SAFTY_MIN_URF 1.1 //meters

#include <ros/ros.h>
#include <opencv-3.3.1/opencv2/objdetect.hpp>
#include <std_srvs/SetBool.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Range.h>
#include <face_detector.h>

enum class OpMode
{
    PAN                 = 1, /* base stationary (only head pan tracking face)                        */
    PAN_ROTATE          = 2, /* base rotating in place to follow head pan                            */
    PAN_ROTATE_DRIVE    = 3, /* base rotating to follow head pan and moving fw                       */
    PAN_FACE            = 4, /* if pan is stationary, this will move base to track face              */
    PAN_FACE_DRIVE      = 5  /* if pan is stationary, rotate and move base fw to track face (fw)     */
};

struct armadillo2_state
{
    double rotation1 = 0; //rad
    double rotation2 = 0; //rad
    double shoulder1 = 0; //rad
    double shoulder2 = 0; //rad
    double shoulder3 = 0; //rad
    double wrist = 0; //rad
    double finger_left = 0; //rad
    double finger_right = 0; //rad
    double pan = 0; //rad
    double tilt = 0; //rad
    double torso = 0; //m
    double wheel_left = 0; //rad
    double wheel_right = 0; //rad
    double urf = 0; //meters
};

class BaseTracker
{
private:
    ros::NodeHandle *nh_;
    ros::ServiceServer start_track_srv_;
    ros::Subscriber joints_state_sub_;
    ros::Subscriber urf_sub_;
    ros::Publisher twise_pub_;

    armadillo2_state armadillo_state_;
    bool got_state_ = false;

    void jointsUpdateCB(const sensor_msgs::JointState::ConstPtr &msg);
    void urfCB(const sensor_msgs::Range &msg);

public:
    BaseTracker(ros::NodeHandle &nh);
    void track(OpMode op_mode, const CvPoint& face, const cv::Rect& frame);

    static double panMin() { return -M_PI / 4; } //rad
    static double panMax() { return M_PI / 4; } //rad
    static double angularMax() { return 0.3; } //rad
    static double angularMin() { return -0.3; } //rad
    static double forwardVel() { return 0.07; } //rad / s
    void stop();
    void drive(double linear_vel, double angular_vel);
};


#endif //TRACK_FACE_BASE_TRACKER_H
