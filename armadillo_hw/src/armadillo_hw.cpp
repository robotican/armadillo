
#include "armadillo_hw.h"


ArmadilloHW::ArmadilloHW(ros::NodeHandle &nh)
{
    node_handle_ = &nh;
    fetchArmParams();
    buildArmMotors();
    registerArmInterfaces();
    openArmPort();
    pingMotors();
}

void ArmadilloHW::pingMotors()
{
    for (dxl_motor motor : motors_)
    {
        int error_counter = 0;
        while (!dxl_interface_.ping(motor) && ros::ok())
        {
            error_counter++;
            ROS_WARN_NAMED("armadillo_hw", "pinging motor %d failed", motor.id);
            if (error_counter > MAX_PING_RETRIES)
            {
                ROS_ERROR_NAMED("armadillo_hw", "too many ping errors, motor %d is not responding", motor.id);
                ros::shutdown();
                exit(EXIT_FAILURE);
            }
            ros::Rate(5).sleep();
        }
    }
}

void ArmadilloHW::setArmTorque(bool flag)
{
    for (dxl_motor motor : motors_)
    {
        dxl_interface_.setTorque(motor, flag);
    }
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
    node_handle_->param<int>("port_baudrate", arm_baudrate_, 1000000);
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
    for(dxl_motor motor : motors_)
    {
        /* joint state registration */
        ;
        joint_state_handles_.push_back(hardware_interface::JointStateHandle (motor.joint_name,
                                                                             &motor.position,
                                                                             &motor.velocity,
                                                                             &motor.effort));
        joint_state_interface_.registerHandle(joint_state_handles_.back());

        /* joint command registration */
        switch (motor.interface_type)
        {
            case dxl_motor::POS:

                pos_handles_.push_back(hardware_interface::JointHandle (joint_state_interface_.getHandle(motor.joint_name),
                                                                       &motor.command_position));
                joint_state_interface_.registerHandle(pos_handles_.back());
                break;
            case dxl_motor::POS_VEL:
                posvel_handles_.push_back(hardware_interface::PosVelJointHandle (joint_state_interface_.getHandle(motor.joint_name),
                                                                                &motor.command_position,
                                                                                &motor.command_velocity));
                posvel_interface_.registerHandle(posvel_handles_.back());
                break;

        }
    }

    registerInterface(&joint_state_interface_);
    registerInterface(&posvel_interface_);
    registerInterface(&position_interface_);
}

void ArmadilloHW::openArmPort()
{
    DxlInterface::PortState port_state = dxl_interface_.openPort(arm_port_, arm_baudrate_);
    switch (port_state)
    {
        case DxlInterface::PORT_FAIL:
            ROS_ERROR_NAMED("armadillo_hw", "open arm port %s failed. shutting down...", arm_port_.c_str());
            ros::shutdown();
            exit (EXIT_FAILURE);
        case DxlInterface::BAUDRATE_FAIL:
            ROS_ERROR_NAMED("armadillo_hw", "setting arm baudrate to %d failed. shutting down...", arm_baudrate_);
            ros::shutdown();
            exit (EXIT_FAILURE);
        case DxlInterface::SUCCESS:
            ROS_INFO_NAMED("armadillo_hw", "arm port opened successfully\nport name: %s\nbaudrate: %d", arm_port_.c_str(), arm_baudrate_);
    }
}

ros::Duration ArmadilloHW::getPeriod()
{
    ros::Time now = getTime();
    ros::Duration period = now - prev_time_;
    prev_time_ = now;
    return period;
}

void ArmadilloHW::read()
{

}

void ArmadilloHW::write()
{

}