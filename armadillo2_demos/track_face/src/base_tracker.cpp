
#include <base_tracker.h>

BaseTracker::BaseTracker(ros::NodeHandle &nh)
{
    nh_ = &nh;
    joints_state_sub_ = nh_->subscribe("/joint_states", 10, &BaseTracker::jointsUpdateCB, this);
    urf_sub_ = nh_->subscribe("/URF/front", 10, &BaseTracker::urfCB, this);
    twise_pub_ = nh_->advertise<geometry_msgs::Twist>("/cmd_vel", 5);
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

void BaseTracker::urfCB(const sensor_msgs::Range &msg)
{
    armadillo_state_.urf = msg.range;
}



/* if pan tracking face, this will make base follow pan movement */
void BaseTracker::track(OpMode op_mode, const CvPoint& face, const cv::Rect& frame)
{
    if (!got_state_)
        return;

    //ROS_WARN("URF: %f",armadillo_state_.urf );

    if (armadillo_state_.urf <= SAFTY_MIN_URF)
    {
        stop();
        return;
    }

    switch (op_mode)
    {
        case OpMode::PAN:
        {
            // DO NOTHING
            return;
        }
        case OpMode::PAN_ROTATE:
        {
            double angular_speed = FaceDetector::map(panMin(), panMax(), angularMin(), angularMax(), armadillo_state_.pan);
            //ROS_INFO("angular speed: %f", angular_speed);
            drive(0, angular_speed);
            break;
        }
        case OpMode::PAN_ROTATE_DRIVE:
        {
            double angular_speed = FaceDetector::map(panMin(), panMax(), angularMin(), angularMax(), armadillo_state_.pan);
            //ROS_INFO("angular speed: %f", angular_speed);
            drive(forwardVel(), angular_speed);
            break;
        }
        case OpMode::PAN_FACE:
        {
            double angular_speed = FaceDetector::map(0, frame.width, angularMin(), angularMax(), face.x);
            drive(0, angular_speed);
            break;
        }
        case OpMode::PAN_FACE_DRIVE:
        {
            double angular_speed = FaceDetector::map(0, frame.width, angularMin(), angularMax(), face.x);
            drive(forwardVel(), angular_speed);
            break;
        }
        default:
        {
            ROS_WARN("[track_face]: invalid operating mode");
            break;
        }
    }
}

void BaseTracker::stop()
{
    drive(0, 0);
}

void BaseTracker::drive(double linear_vel, double angular_vel)
{
    geometry_msgs::Twist twist_msg;
    twist_msg.linear.x = linear_vel;
    twist_msg.angular.z = angular_vel;
    twise_pub_.publish(twist_msg);
}

