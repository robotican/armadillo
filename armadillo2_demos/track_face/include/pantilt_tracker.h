
#ifndef TRACK_FACE_PANTILT_TRACKER_H
#define TRACK_FACE_PANTILT_TRACKER_H

#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float32.h>
#include <face_detector.h>
#include <math.h>

class PanTiltTracker
{
private:
    ros::Publisher traj_pub_,
            grp_pos_pub_;
    ros::ServiceServer mover_srv_;
    ros::NodeHandle *nh_;

public:
    PanTiltTracker(ros::NodeHandle &nh);
    void trackFace(const CvPoint &face, const cv::Rect& frame);

};


#endif //TRACK_FACE_PANTILT_TRACKER_H
