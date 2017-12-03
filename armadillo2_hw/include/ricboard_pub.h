
#ifndef ARMADILLO2_HW_RICBOARD_PUB_H
#define ARMADILLO2_HW_RICBOARD_PUB_H

#include <ric_interface/ric_interface.h>
#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/posvel_command_interface.h>


#define RIC_PORT_PARAM "ric_port"
#define TORSO_JOINT_PARAM "torso_joint"
#define RIC_PUB_INTERVAL 0.1 //secs

struct armadillo2_torso
{
    double pos = 0;
    double vel = 0;
    double prev_pos = 0;
    double command_pos = 0;
    double command_vel = 0;
    std::string joint_name;
};

struct ric_state
{
    armadillo2_torso torso;
};

class RicboardPub
{
private:
    ros::Publisher ric_gps_pub_;
    ros::Publisher ric_ultrasonic_pub_;
    ros::Publisher ric_imu_pub_;
    ros::Timer ric_pub_timer_;

    ric_interface::RicInterface ric_;
    ric_state ric_state_;
    ros::NodeHandle *nh_;
    std::string ric_port_;
    bool load_ric_hw_ = true;

    /* handles */
    std::vector<hardware_interface::JointStateHandle> joint_state_handles_;
    std::vector<hardware_interface::PosVelJointHandle> posvel_handles_;
    std::vector<hardware_interface::JointHandle> pos_handles_;

    void pubTimerCB(const ros::TimerEvent& event);

public:
    RicboardPub(ros::NodeHandle &nh);
    void loop();
    void read();
    void write();
    void startPublish();
    void stopPublish();
    void registerHandles(hardware_interface::JointStateInterface &joint_state_interface,
                         hardware_interface::PositionJointInterface &position_interface,
                         hardware_interface::PosVelJointInterface &posvel_interface);
};


#endif //ARMADILLO2_HW_RICBOARD_PUB_H
