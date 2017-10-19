//
// Created by sub on 16/10/17.
//

#ifndef ARMADILLO_HW_DXL_MOTORS_BUILDER_H
#define ARMADILLO_HW_DXL_MOTORS_BUILDER_H

#include "dxl_interface.h"
#include <armadillo_hw/EnableTorque.h>
#include <yaml-cpp/yaml.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/posvel_command_interface.h>

#define MAX_PING_ERRORS 5
#define MAX_READ_ERRORS 16
#define MAX_WRITE_ERRORS 16
#define ARM_CONFIG_PARAM "arm_config"

/* The >> operator disappeared in yaml-cpp 0.5, so this function is     */
/* added to provide support for code written under the yaml-cpp 0.3 API.*/
template<typename T>
void operator >> (const YAML::Node& node, T& i)
{
    i = node.as<T>();
}

class DxlMotorsBuilder
{
public:

    DxlMotorsBuilder(ros::NodeHandle &nh);
    void registerHandles(hardware_interface::JointStateInterface &joint_state_interface,
                         hardware_interface::PositionJointInterface &position_interface,
                         hardware_interface::PosVelJointInterface &posvel_interface);
    void read();
    void write();

private:

    ros::NodeHandle *node_handle_;

    /* handles */
    std::vector<hardware_interface::JointStateHandle> joint_state_handles_;
    std::vector<hardware_interface::PosVelJointHandle> posvel_handles_;
    std::vector<hardware_interface::JointHandle> pos_handles_;

    bool first_read_;
    int arm_baudrate_;
    int failed_reads_, failed_writes_;
    std::map<uint16_t, dxl_spec> models_specs_; /* key - model number, value - dxl spec */
    std::string arm_port_;
    DxlInterface dxl_interface_;
    XmlRpc::XmlRpcValue arm_config_;
    std::vector<dxl_motor> motors_;

    bool torqueServiceCB(armadillo_hw::EnableTorque::Request  &req,
                         armadillo_hw::EnableTorque::Response &res);

    void fetchParams();
    void openPort();
    void buildMotors();
    void pingMotors();
    bool setTorque(bool flag);
    void loadSpecs();

};
#endif //ARMADILLO_HW_DXL_MOTORS_BUILDER_H
