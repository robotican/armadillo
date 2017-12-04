//
// Created by armadillo2 on 04/12/17.
//

#ifndef ARMADILLO2_HW_ROBOTEQ_DIFF_DRIVE_H
#define ARMADILLO2_HW_ROBOTEQ_DIFF_DRIVE_H

#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <ros/callback_queue.h>
#include <signal.h>
#include <roboteq/roboteq.h>
#include <roboteq/serial_controller.h>

#include <boost/chrono.hpp>

#define ROBOTEQ_PORT_PARAM "roboteq_port"
#define ROBOTEQ_BAUD_PARAM "roboteq_baud"
#define RIGHT_WHEEL_JOINT_PARAM "right_wheel_joint"
#define LEFT_WHEEL_JOINT_PARAM "left_wheel_joint"

typedef boost::chrono::steady_clock time_source;

class RoboteqDiffDrive
{
private:

    ros::NodeHandle *nh_;
    roboteq::serial_controller *roboteq_serial_;
    roboteq::Roboteq *roboteq_;
    boost::chrono::duration<double> elapsed_duration_;
    time_source::time_point last_time_ = time_source::now();

    std::string roboteq_port_;
    std::string right_wheel_joint_, left_wheel_joint_;
    int roboteq_baud_;
    bool load_roboteq_hw_ = false;


public:
    ~RoboteqDiffDrive() { delete roboteq_; }
    RoboteqDiffDrive(ros::NodeHandle &nh);
    void loop();
    void read();
    void write();
    void registerHandles(hardware_interface::JointStateInterface &joint_state_interface,
                         hardware_interface::VelocityJointInterface &velocity_joint_interface);
};


#endif //ARMADILLO2_HW_ROBOTEQ_DIFF_DRIVE_H
