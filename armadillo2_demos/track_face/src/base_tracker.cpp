
#include <base_tracker.h>

BaseTracker::BaseTracker(ros::NodeHandle &nh)
{
    nh_ = &nh;
    start_track_srv_ = nh_->advertiseService("start_base_tracking", &BaseTracker::startTrackingCB, this);
    joints_state_sub_ = nh_->subscribe("joint_states", 5, &BaseTracker::jointsUpdateCB, this);
    twise_pub_ = nh_->advertise<geometry_msgs::Twist>("/cmd_vel", 5);
}


bool BaseTracker::startTrackingCB(std_srvs::SetBool::Request &req,
                                  std_srvs::SetBool::Response &res)
{
    track_face_ = !track_face_;
}

void BaseTracker::jointsUpdateCB(const sensor_msgs::JointState::ConstPtr &msg)
{
    /* save joints updated state */
    got_state_ = true;
    armadillo_state_.pan = msg->position[INDX_JOINT_PAN];
    armadillo_state_.tilt = msg->position[INDX_JOINT_TILT];
    armadillo_state_.finger_left = msg->position[INDX_JOINT_LEFT_FINGER];
    armadillo_state_.wheel_left = msg->position[INDX_JOINT_LEFT_WHEEL];
    armadillo_state_.finger_right = msg->position[INDX_JOINT_RIGHT_FINGER];
    armadillo_state_.wheel_right = msg->position[INDX_JOINT_RIGHT_WHEEL];
    armadillo_state_.rotation1 = msg->position[INDX_JOINT_ROTATION1];
    armadillo_state_.rotation2 = msg->position[INDX_JOINT_ROTATION2];
    armadillo_state_.shoulder1 = msg->position[INDX_JOINT_SHOULDER1];
    armadillo_state_.shoulder2 = msg->position[INDX_JOINT_SHOULDER2];
    armadillo_state_.shoulder3 = msg->position[INDX_JOINT_SHOULDER3];
    armadillo_state_.torso = msg->position[INDX_JOINT_TORSO];
    armadillo_state_.wrist = msg->position[INDX_JOIN_WRIST];

}

/* if pan is stationary, this will move base to track face */
void BaseTracker::trackFace(const CvPoint &face, const cv::Rect& frame)
{
    if (!got_state_ || !track_face_)
        return;
    double const fw_speed = 0.3;
    double const max_angular = 0.3;
    double const min_angular = -0.3;

    double angular_speed = FaceDetector::map(0, frame.width, min_angular, max_angular, face.x);

    geometry_msgs::Twist twist_msg;
    twist_msg.linear.x = fw_speed;
    twist_msg.angular.x = angular_speed;
    twise_pub_.publish(twist_msg);
}

/* if pan tracking face, this will make base follow pan movement */
void BaseTracker::trackPan()
{
    if (!got_state_ || !track_face_)
        return;
    double pan_min = -M_PI / 4;//rad
    double pan_max =  M_PI / 4;//rad
    double const max_angular = 0.3;
    double const min_angular = -0.3;
    double const fw_speed = 0.3;

    double angular_speed = FaceDetector::map(pan_min, pan_max, min_angular, max_angular, armadillo_state_.pan);

    ROS_INFO("angular speed: %f", angular_speed);

    geometry_msgs::Twist twist_msg;
    //twist_msg.linear.x = fw_speed;
    twist_msg.angular.z = angular_speed;
    twise_pub_.publish(twist_msg);
}

