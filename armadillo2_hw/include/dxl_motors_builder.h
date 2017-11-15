//
// Created by sub on 16/10/17.
//

#ifndef ARMADILLO2_HW_DXL_MOTORS_BUILDER_H
#define ARMADILLO2_HW_DXL_MOTORS_BUILDER_H

#include <dxl_interface/dxl_interface.h>
#include <std_srvs/SetBool.h>
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
#define SPEC_CONFIG_PARAM "dxl_spec_config"
#define DXL_PROTOCOL_PARAM "dxl_protocol"
#define ARM_PORT_PARAM "arm_port_name"
#define ARM_PORT_BAUD_PARAM "arm_port_baudrate"

/* The >> operator disappeared in yaml-cpp 0.5, so this function is     */
/* added to provide support for code written under the yaml-cpp 0.3 API.*/
template<typename T>
void operator >> (const YAML::Node& node, T& i)
{
    i = node.as<T>();
}

namespace armadillo2_hw
{

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

        int arm_baudrate_;
        int failed_reads_, failed_writes_;
        float protocol_;
        std::map<uint16_t, dxl::spec> specs_; /* key - model number, value - dxl spec */
        std::string arm_port_;
        dxl::DxlInterface dxl_interface_;
        XmlRpc::XmlRpcValue arm_config_, dxl_spec_config_;
        std::vector<dxl::motor> motors_;
        ros::ServiceServer torque_srv_;

        bool torqueServiceCB(std_srvs::SetBool::Request  &req,
                             std_srvs::SetBool::Response &res);

        void fetchParams();
        void openPort();
        void buildMotors();
        void pingMotors();
        bool setTorque(bool flag);
        void loadSpecs();

    };
}
#endif //ARMADILLO2_HW_DXL_MOTORS_BUILDER_H
