
#include "armadillo_hw.h"


ArmadilloHW::ArmadilloHW(ros::NodeHandle &nh)
{
    node_handle_ = &nh;
    fetchArmParams();
    buildArmMotors();
    registerArmInterfaces();

}

void ArmadilloHW::fetchArmParams()
{
    if (!node_handle_->hasParam("dxl_motors"))
    {
        ROS_ERROR_NAMED("armadillo_hw", "dxl_motors param is missing on param server. shutting down...");
        ros::shutdown();
        exit (EXIT_FAILURE);
    }
    node_handle_->getParam("dxl_motors", yaml_dxl_motors_);
    if (yaml_dxl_motors_.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
        ROS_ERROR_NAMED("armadillo_hw", "dxl_motors param is invalid (need to be an of type array). shutting down...");
        ros::shutdown();
        exit (EXIT_FAILURE);
    }
    node_handle_->param<std::string>("port_name", arm_port_, "/dev/USB2DYNAMIXEL");
    node_handle_->param<int>("port_baudrate", device_baudrate, 1000000);
}

void ArmadilloHW::buildArmMotors()
{

    for(int i = 0; i < yaml_dxl_motors_.size(); i++)
    {
        if(yaml_dxl_motors_[i].getType() != XmlRpc::XmlRpcValue::TypeStruct)
        {
            ROS_ERROR_NAMED("armadillo_hw", "arm motor %d param data type is invalid . shutting down...", i);
            ros::shutdown();
            exit (EXIT_FAILURE);
        }

        struct dxl_motor new_motor;

        if(yaml_dxl_motors_[i]["id"].getType() != XmlRpc::XmlRpcValue::TypeInt)
        {
            ROS_ERROR_NAMED("armadillo_hw", "arm motor id at index %d: invalid data type. shutting down...", i);
            ros::shutdown();
            exit (EXIT_FAILURE);
        }
        new_motor.id = static_cast<int>(yaml_dxl_motors_[i]["id"]);

        if (yaml_dxl_motors_[i]["joint_name"].getType() != XmlRpc::XmlRpcValue::TypeString)
        {
            ROS_ERROR_NAMED("armadillo_hw", "arm motor joint_name at index %d: invalid data type. shutting down...", i);
            ros::shutdown();
            exit (EXIT_FAILURE);
        }
        new_motor.joint_name = static_cast<std::string>(yaml_dxl_motors_[i]["joint_name"]);

        if (yaml_dxl_motors_[i]["interface_type"].getType() != XmlRpc::XmlRpcValue::TypeString)
        {
            ROS_ERROR_NAMED("armadillo_hw", "arm motor interface_type at index %d: invalid data type. shutting down...", i);
            ros::shutdown();
            exit (EXIT_FAILURE);
        }
        std::string string_interface_type = static_cast<std::string>(yaml_dxl_motors_[i]["interface_type"]);
        new_motor.interface_type = dxl_motor::stringToInterfaceType(string_interface_type);

        motors_.push_back(new_motor);
    }
}

void ArmadilloHW::registerArmInterfaces()
{
    /* joint state registration */
    for(dxl_motor motor : motors_)
    {
        /* joint state registration */
        hardware_interface::JointStateHandle joint_state_handle(motor.joint_name,
                                                                &motor.position,
                                                                &motor.velocity,
                                                                &motor.effort);
        joint_state_handles_.push_back(joint_state_handle);
        joint_state_interface_.registerHandle(joint_state_handles_.back());

        /* joint command registration */
        switch (motor.interface_type)
        {
            case dxl_motor::InterfaceType ::POS:
                hardware_interface::JointHandle joint_handle(joint_state_interface_.getHandle(motor.joint_name),
                                                             &motor.command_position);
                pos_handles.push_back(joint_handle);
                joint_state_interface_.registerHandle(pos_handles.back());
                break;
            case dxl_motor::InterfaceType ::POS_VEL:
                hardware_interface::PosVelJointHandle posvel_joint_handle_(joint_state_interface_.getHandle(motor.joint_name),
                                                                           &motor.command_position,
                                                                           &motor.command_velocity);
                posvel_handles.push_back(posvel_joint_handle_);
                posvel_interface_.registerHandle(posvel_handles.back());
                break;

        }
    }

    registerInterface(&joint_state_interface_);
    registerInterface(&posvel_interface_);
    registerInterface(&position_interface_);
}

void ArmadilloHW::openArmPort(std::string port_name, unsigned int baudrate)
{
    DxlInterface::PortState port_state = dxl_interface_.openPort(port_name, baudrate);
    switch (port_state)
    {
        case DxlInterface::PortState::PORT_FAIL:
            ROS_ERROR_NAMED("armadillo_hw", "open arm port %s failed. shutting down...", port_name);
            ros::shutdown();
            exit (EXIT_FAILURE);
        case DxlInterface::PortState::BAUDRATE_FAIL:
            ROS_ERROR_NAMED("armadillo_hw", "setting arm baudrate to %u failed. shutting down...", baudrate);
            ros::shutdown();
            exit (EXIT_FAILURE);
        case DxlInterface::PortState::SUCCESS:
            ROS_ERROR_NAMED("armadillo_hw", "arm port opened successfully\nport name: %s\nbaudrate: %u", port_name, baudrate);
    }
}

ros::Duration ArmadilloHW::getPeriod()
{
    ros::Time now = getTime();
    ros::Duration period = now - prev_time_;
    prev_time_ = now;
    return period;
}