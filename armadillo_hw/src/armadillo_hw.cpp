
#include "armadillo_hw.h"


ArmadilloHW::ArmadilloHW(ros::NodeHandle &nh)
{
    node_handle_ = &nh;
    /* the order of calling the following methods is important  */
    /* because some calls load params / objects for other calls */
    fetchArmParams();
    buildArmMotors();
    registerArmInterfaces();
    openArmPort();
    pingMotors();
    loadMotorsSpecs();
    setArmTorque(true);

    prev_time_ = getTime();
    ROS_INFO_NAMED("armadillo_hw", "armadillo hardware interface loaded successfully");
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

void ArmadilloHW::loadMotorsSpecs()
{
    /* load dxl motors specs */
    std::string motors_data_path = ros::package::getPath("armadillo_hw");
    motors_data_path += "/config/dxl_motor_data.yaml";
    YAML::Node motors_doc;
    motors_doc = YAML::LoadFile(motors_data_path);

    for (int block_num = 0; block_num < motors_doc.size(); block_num++)
    {
        dxl_spec spec;

        motors_doc[block_num]["name"] >> spec.name;
        motors_doc[block_num]["model"] >> spec.model;
        motors_doc[block_num]["cpr"]  >> spec.cpr;
        motors_doc[block_num]["rpm_scale_factor"]  >> spec.rpm_scale_factor;
        motors_doc[block_num]["torque_constant_a"]  >> spec.torque_const_a;
        motors_doc[block_num]["torque_constant_b"]  >> spec.torque_const_b;

        models_specs_[spec.model] = spec;
    }

    /* feed motors with model specs */
    for (int i=0; i < motors_.size(); i++)
    {
        bool spec_found = models_specs_.find(motors_[i].spec.model) != models_specs_.end();
        if (spec_found)
        {
            motors_[i].spec.cpr = models_specs_[motors_[i].spec.model].cpr;
            motors_[i].spec.rpm_scale_factor = models_specs_[motors_[i].spec.model].rpm_scale_factor;
            motors_[i].spec.torque_const_a = models_specs_[motors_[i].spec.model].torque_const_a;
            motors_[i].spec.torque_const_b = models_specs_[motors_[i].spec.model].torque_const_b;
        }
    }
}

void ArmadilloHW::setArmTorque(bool flag)
{
    std::string str_flag = flag ? "on" : "off";
    bool success = true;
    for (dxl_motor motor : motors_)
    {
        if (!dxl_interface_.setTorque(motor, flag))
        {
            success = false;
            ROS_WARN_NAMED("armadillo_hw", "failed set torque of motor id %d to %s", motor.id, str_flag.c_str());
        }
        else
            motor.in_torque = flag;
    }

    if (success)
        ROS_INFO_NAMED("armadillo_hw", "arm torque is %s", str_flag.c_str());
}

void ArmadilloHW::fetchArmParams()
{
    if (!node_handle_->hasParam(ARM_CONFIG_PARAM))
    {
        ROS_ERROR_NAMED("armadillo_hw", "%s param is missing on param server. shutting down...", ARM_CONFIG_PARAM);
        ros::shutdown();
        exit (EXIT_FAILURE);
    }
    node_handle_->getParam(ARM_CONFIG_PARAM, arm_config_);
    if (arm_config_.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
        ROS_ERROR_NAMED("armadillo_hw", "%s param is invalid (need to be an of type array). shutting down...", ARM_CONFIG_PARAM);
        ros::shutdown();
        exit (EXIT_FAILURE);
    }
    node_handle_->param<std::string>("port_name", arm_port_, "/dev/USB2DYNAMIXEL");
    node_handle_->param<int>("port_baudrate", arm_baudrate_, 1000000);
}

void ArmadilloHW::buildArmMotors()
{
    /* build motors */
    for(int i = 0; i < arm_config_.size(); i++)
    {
        /* feed motor with user defined settings */
        if(arm_config_[i].getType() != XmlRpc::XmlRpcValue::TypeStruct)
        {
            ROS_ERROR_NAMED("armadillo_hw", "arm motor %d param data type is invalid . shutting down...", i);
            ros::shutdown();
            exit (EXIT_FAILURE);
        }

        struct dxl_motor new_motor;

        if(arm_config_[i]["id"].getType() != XmlRpc::XmlRpcValue::TypeInt)
        {
            ROS_ERROR_NAMED("armadillo_hw", "arm motor id at index %d: invalid data type. shutting down...", i);
            ros::shutdown();
            exit (EXIT_FAILURE);
        }
        new_motor.id = static_cast<int>(arm_config_[i]["id"]);

        if (arm_config_[i]["joint_name"].getType() != XmlRpc::XmlRpcValue::TypeString)
        {
            ROS_ERROR_NAMED("armadillo_hw", "arm motor joint_name at index %d: invalid data type. shutting down...", i);
            ros::shutdown();
            exit (EXIT_FAILURE);
        }
        new_motor.joint_name = static_cast<std::string>(arm_config_[i]["joint_name"]);

        if (arm_config_[i]["interface_type"].getType() != XmlRpc::XmlRpcValue::TypeString)
        {
            ROS_ERROR_NAMED("armadillo_hw", "arm motor interface_type at index %d: invalid data type. shutting down...", i);
            ros::shutdown();
            exit (EXIT_FAILURE);
        }
        std::string string_interface_type = static_cast<std::string>(arm_config_[i]["interface_type"]);
        new_motor.interface_type = dxl_motor::stringToInterfaceType(string_interface_type);

        /* feed motor with dxl defined settings (constant) */

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
    dxl_interface_.readMotorosLoad(motors_);
    dxl_interface_.readMotorsError(motors_);
    dxl_interface_.readMotorsVel(motors_);
    dxl_interface_.readMotorsPos(motors_);
}

void ArmadilloHW::write()
{
    dxl_interface_.bulkWriteVelocity(motors_);
    dxl_interface_.bulkWritePosition(motors_);
}