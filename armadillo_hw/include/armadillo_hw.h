
#ifndef ARMADILLO_HW_ARMADILLO_HW_H
#define ARMADILLO_HW_ARMADILLO_HW_H


#include "dxl_interface.h"
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/posvel_command_interface.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>
#include <armadillo_hw/EnableTorque.h>

#define MAX_PING_RETRIES 5
#define ARM_CONFIG_PARAM "arm_config"

/* The >> operator disappeared in yaml-cpp 0.5, so this function is     */
/* added to provide support for code written under the yaml-cpp 0.3 API.*/
template<typename T>
void operator >> (const YAML::Node& node, T& i)
{
    i = node.as<T>();
}

struct armadillo_arm
{
    uint8_t num_of_motors; //degree of freedom / num of motors

};

class ArmadilloHW : public hardware_interface::RobotHW
{
private:

    ros::Time prev_time_;
    ros::NodeHandle *node_handle_;
    /* interfaces */
    hardware_interface::JointStateInterface joint_state_interface_;
    hardware_interface::PositionJointInterface position_interface_;
    hardware_interface::PosVelJointInterface posvel_interface_;
    /* handles */
    std::vector<hardware_interface::JointStateHandle> joint_state_handles_;
    std::vector<hardware_interface::PosVelJointHandle> posvel_handles_;
    std::vector<hardware_interface::JointHandle> pos_handles_;
    std::map<uint16_t, dxl_spec> models_specs_; /* key - model number, value - dxl spec */


    void registerInterfaces();


    /* arm */
    std::string arm_port_;
    int arm_baudrate_;
    DxlInterface dxl_interface_;
    XmlRpc::XmlRpcValue arm_config_;
    std::vector<dxl_motor> motors_;
    void fetchArmParams();
    void registerArmInterfaces();
    void openArmPort();
    void buildArmMotors();
    void pingMotors();
    void setArmTorque(bool flag);
    void loadMotorsSpecs();

public:

    ArmadilloHW(ros::NodeHandle &nh);
    void read();
    void write();
    static ros::Time getTime() { return ros::Time::now(); }
    ros::Duration getPeriod();

};

#endif //ARMADILLO_HW_ARMADILLO_HW_H
