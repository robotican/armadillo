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

#include <ros/ros.h>
#include <opencv-3.3.1/opencv2/objdetect.hpp>
#include <std_srvs/SetBool.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>

#include <face_detector.h>

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

};

class BaseTracker
{
private:
    ros::NodeHandle *nh_;
    ros::ServiceServer start_track_srv_;
    ros::Subscriber joints_state_sub_;
    ros::Publisher twise_pub_;

    bool track_face_ = false;
    armadillo2_state armadillo_state_;
    bool got_state_ = false;



public:
    BaseTracker(ros::NodeHandle &nh);
    void jointsUpdateCB(const sensor_msgs::JointState::ConstPtr &msg);
    void trackFace(const CvPoint& face, const cv::Rect& frame);
    void trackPan();
    bool startTrackingCB(std_srvs::SetBool::Request &req,
                         std_srvs::SetBool::Response &res);

};


#endif //TRACK_FACE_BASE_TRACKER_H
