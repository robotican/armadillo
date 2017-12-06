
#ifndef ARMADILLO2_HW_RICBOARD_PUB_H
#define ARMADILLO2_HW_RICBOARD_PUB_H

#include <ric_interface/ric_interface.h>
#include <ric_interface/ric_exception.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/posvel_command_interface.h>


#define RIC_PORT_PARAM "ric_port"
#define TORSO_JOINT_PARAM "torso_joint"
#define RIC_PUB_INTERVAL 0.1 //secs
#define RIC_WRITE_INTERVAL 0.1 //secs
#define RIC_DEAD_TIMEOUT 3 //secs

struct torso_joint
{
    double pos = 0;
    double vel = 0;
    double prev_pos = 0;
    double effort = 0; /* effort stub - not implemented */
    double command_pos = 0;
    double command_vel = 0;
    std::string joint_name;
};


class RicboardPub
{
private:

    bool  load_ric_hw_ = true;
    std::string ric_port_;
    ros::Publisher ric_gps_pub_;
    ros::Publisher ric_ultrasonic_pub_;
    ros::Publisher ric_imu_pub_;
    ros::Timer ric_pub_timer_;
    ros::Timer ric_dead_timer_;
    ros::Time last_read_time_;
    torso_joint torso_;
    ric_interface::RicInterface ric_;
    ros::NodeHandle *nh_;

    /* handles */
    std::vector<hardware_interface::JointStateHandle> joint_state_handles_;
    std::vector<hardware_interface::JointHandle> pos_handles_;

    void pubTimerCB(const ros::TimerEvent& event);
    void ricDeadTimerCB(const ros::TimerEvent& event);

public:
    RicboardPub(ros::NodeHandle &nh);
    void loop();
    void read(const ros::Duration elapsed);
    void write();
    void registerHandles(hardware_interface::JointStateInterface &joint_state_interface,
                         hardware_interface::PositionJointInterface &position_interface);
};


#endif //ARMADILLO2_HW_RICBOARD_PUB_H
