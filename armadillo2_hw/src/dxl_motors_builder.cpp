//
// Created by sub on 16/10/17.
//
#include "dxl_motors_builder.h"

namespace armadillo2_hw
{
    DxlMotorsBuilder::DxlMotorsBuilder(ros::NodeHandle &nh)
    {
        node_handle_ = &nh;

        /* the order of calling the following methods is important      */
        /* because some calls load params / objects for following calls */
        fetchParams();
        buildMotors();
        openPort();
        pingMotors();
        loadSpecs();
        setTorque(true);

        for (dxl::motor &motor :  motors_)
        {
            ROS_INFO ("[dxl_motors_builder]: done motor id: %d, model: %d, pos: %f, effort: %f, vel: %f",
                      motor.id, motor.spec.model, motor.position, motor.effort, motor.velocity);
        }

        failed_reads_ = 0;
        failed_writes_ = 0;
        first_read_ = true;
    }

    void DxlMotorsBuilder::read()
    {
        if (!dxl_interface_.readMotorosLoad(motors_))
        {
            ROS_ERROR("[dxl_motors_builder]: reading motors load failed");
            failed_reads_++;
        }
        if (!dxl_interface_.readMotorsError(motors_))
        {
            ROS_ERROR("[dxl_motors_builder]: reading motors errors failed");
            failed_reads_++;
        }
        if (dxl_interface_.readMotorsVel(motors_))
        {
            ROS_ERROR("[dxl_motors_builder]: reading motors errors failed");
            failed_reads_++;
        }
        if (dxl_interface_.readMotorsPos(motors_))
        {
            ROS_ERROR("[dxl_motors_builder]: reading motors errors failed");
            failed_reads_++;
        }

        /*if (failed_reads_ >= MAX_READ_ERRORS)
        {
            ROS_ERROR("[dxl_motors_builder]: too many read errors, shutting down...");
            ros::shutdown();
            exit(EXIT_FAILURE);
        }*/

        if (first_read_)
        {
            for (dxl::motor &motor : motors_)
                motor.command_position = motor.position;
        }
    }

    void DxlMotorsBuilder::write()
    {
        if (dxl_interface_.bulkWriteVelocity(motors_))
        {
            ROS_ERROR("[dxl_motors_builder]: writing velocity failed");
            failed_writes_++;
        }

        if (dxl_interface_.bulkWritePosition(motors_))
        {
            ROS_ERROR("[dxl_motors_builder]: writing postision failed");
            failed_writes_++;
        }

        /*if (failed_writes_ >= MAX_READ_ERRORS)
        {
            ROS_ERROR("[dxl_motors_builder]: too many write errors, shutting down...");
            ros::shutdown();
            exit(EXIT_FAILURE);
        }*/
    }

    void DxlMotorsBuilder::pingMotors()
    {
        for (dxl::motor &motor : motors_)
        {
            int error_counter = 0;
            while (!dxl_interface_.ping(motor))
            {
                error_counter++;
                ROS_WARN("[dxl_motors_builder]: pinging motor id: %d spec: %d failed", motor.id, motor.spec.model);
                if (error_counter > MAX_PING_ERRORS)
                {
                    ROS_ERROR("[dxl_motors_builder]: too many ping errors, motor %d is not responding. \n"
                                      "check if motor crashed (red blink) and try to restart. \n"
                                      "also make sure LATENCY_TIMER is set to 1 in dynamixel_sdk, and that the appropriate"
                                      "rule for dynamixel port is existed with LATENCY_TIMER=1.", motor.id);
                    ros::shutdown();
                    exit(EXIT_FAILURE);
                }
                ros::Rate(5).sleep();
            }
        }
    }

    void DxlMotorsBuilder::loadSpecs()
    {
        /* load dxl motors specs */
       /* std::string motors_data_path = ros::package::getPath("armadillo2_hw");
        motors_data_path += "/config/dxl_motor_data.yaml";
        YAML::Node motors_doc;
        motors_doc = YAML::LoadFile(motors_data_path);

        for (int block_num = 0; block_num < motors_doc.size(); block_num++)
        {
            dxl::spec spec;

            motors_doc[block_num]["name"] >> spec.name;
            motors_doc[block_num]["model"] >> spec.model;
            motors_doc[block_num]["cpr"]  >> spec.cpr;
            motors_doc[block_num]["rpm_scale_factor"]  >> spec.rpm_scale_factor;
            motors_doc[block_num]["torque_constant_a"]  >> spec.torque_const_a;
            motors_doc[block_num]["torque_constant_b"]  >> spec.torque_const_b;

            specs_[spec.model] = spec;
        }

        feed motors with model specs
        for (dxl::motor &motor : motors_)
        {
            bool spec_found = specs_.find(motor.spec.model) != specs_.end();
            if (spec_found)
            {
                motor.spec.cpr = specs_[motor.spec.model].cpr;
                motor.spec.rpm_scale_factor = specs_[motor.spec.model].rpm_scale_factor;
                motor.spec.torque_const_a = specs_[motor.spec.model].torque_const_a;
                motor.spec.torque_const_b = specs_[motor.spec.model].torque_const_b;
            }
            else
            {
                ROS_ERROR("[dxl_motors_builder]: couldn't locate model specification for motor %d, model %d. "
                                  "make sure dxl_motor_data.yaml contains all the necessary specs", motor.id, motor.spec.model);
            }

        }*/

        /* build motors */
        for(int i = 0; i < dxl_spec_config_.size(); i++)
        {
            /* feed motor with user defined settings */
            if(dxl_spec_config_[i].getType() != XmlRpc::XmlRpcValue::TypeStruct)
            {
                ROS_ERROR("[dxl_motors_builder]: motor spec at index %d param data type is invalid or missing. "
                                  "make sure that this param exist in dxl_spec_config.yaml and that your launch includes this param file. shutting down...", i);
                ros::shutdown();
                exit (EXIT_FAILURE);
            }

            struct dxl::spec spec;

            if(dxl_spec_config_[i]["name"].getType() != XmlRpc::XmlRpcValue::TypeString) //invalid name field
            {
                ROS_ERROR("[dxl_motors_builder]: spec name at index %d: invalid data type or missing. "
                                  "make sure that this param exist in arm_config.yaml and that your launch includes this param file. shutting down...", i);
                ros::shutdown();
                exit (EXIT_FAILURE);
            }
            spec.name = static_cast<std::string>(dxl_spec_config_[i]["name"]);

            if(dxl_spec_config_[i]["model"].getType() != XmlRpc::XmlRpcValue::TypeInt) //invalid name field
            {
                ROS_ERROR("[dxl_motors_builder]: spec model at index %d: invalid data type or missing. "
                                  "make sure that this param exist in arm_config.yaml and that your launch includes this param file. shutting down...", i);
                ros::shutdown();
                exit (EXIT_FAILURE);
            }
            spec.model = static_cast<int>(dxl_spec_config_[i]["model"]);

            if(dxl_spec_config_[i]["cpr"].getType() != XmlRpc::XmlRpcValue::TypeInt) //invalid name field
            {
                ROS_ERROR("[dxl_motors_builder]: spec cpr at index %d: invalid data type or missing. "
                                  "make sure that this param exist in arm_config.yaml and that your launch includes this param file. shutting down...", i);
                ros::shutdown();
                exit (EXIT_FAILURE);
            }
            spec.cpr = static_cast<int>(dxl_spec_config_[i]["cpr"]);

            if(dxl_spec_config_[i]["rpm_factor"].getType() != XmlRpc::XmlRpcValue::TypeDouble) //invalid name field
            {
                ROS_ERROR("[dxl_motors_builder]: spec rpm_factor at index %d: invalid data type or missing. "
                                  "make sure that this param exist in arm_config.yaml and that your launch includes this param file. shutting down...", i);
                ros::shutdown();
                exit (EXIT_FAILURE);
            }
            spec.rpm_scale_factor = static_cast<double>(dxl_spec_config_[i]["rpm_factor"]);

            if(dxl_spec_config_[i]["torque_const_a"].getType() != XmlRpc::XmlRpcValue::TypeDouble) //invalid name field
            {
                ROS_ERROR("[dxl_motors_builder]: spec torque_const_a at index %d: invalid data type or missing. "
                                  "make sure that this param exist in arm_config.yaml and that your launch includes this param file. shutting down...", i);
                ros::shutdown();
                exit (EXIT_FAILURE);
            }
            spec.torque_const_a = static_cast<double>(dxl_spec_config_[i]["torque_const_a"]);

            if(dxl_spec_config_[i]["torque_const_b"].getType() != XmlRpc::XmlRpcValue::TypeDouble) //invalid name field
            {
                ROS_ERROR("[dxl_motors_builder]: spec torque_const_b at index %d: invalid data type or missing. "
                                  "make sure that this param exist in arm_config.yaml and that your launch includes this param file. shutting down...", i);
                ros::shutdown();
                exit (EXIT_FAILURE);
            }
            spec.torque_const_b = static_cast<double>(dxl_spec_config_[i]["torque_const_b"]);

            specs_[spec.model] = spec;
        }

        /* feed motors with model specs */
        for (dxl::motor &motor : motors_)
        {
            bool spec_found = specs_.find(motor.spec.model) != specs_.end();
            if (spec_found)
            {
                motor.spec.cpr = specs_[motor.spec.model].cpr;
                motor.spec.rpm_scale_factor = specs_[motor.spec.model].rpm_scale_factor;
                motor.spec.torque_const_a = specs_[motor.spec.model].torque_const_a;
                motor.spec.torque_const_b = specs_[motor.spec.model].torque_const_b;
            }
            else
            {
                ROS_ERROR("[dxl_motors_builder]: couldn't locate model specification for motor %d, model %d. "
                                  "make sure dxl_motor_data.yaml contains all the necessary specs", motor.id, motor.spec.model);
            }

        }
    }

    bool DxlMotorsBuilder::setTorque(bool flag)
    {
        std::string str_flag = flag ? "on" : "off";
        bool success = true;
        for (dxl::motor &motor : motors_)
        {
            if (!dxl_interface_.setTorque(motor, flag))
            {
                success = false;
                ROS_WARN("[dxl_motors_builder]: failed set torque of motor id %d to %s", motor.id, str_flag.c_str());
            }
            else
                motor.in_torque = flag;
        }

        if (success)
        {
            ROS_INFO("[dxl_motors_builder]: arm torque is %s", str_flag.c_str());
            return true;
        }
        return false;
    }

    bool DxlMotorsBuilder::torqueServiceCB(std_srvs::SetBool::Request  &req,
                                           std_srvs::SetBool::Response &res)
    {
        res.success = false;
        if (DxlMotorsBuilder::setTorque(req.data))
            res.success = true;
        return true;
    }


    void DxlMotorsBuilder::fetchParams()
    {
        if (!node_handle_->hasParam(ARM_CONFIG_PARAM))
        {
            ROS_ERROR("[dxl_motors_builder]: %s param is missing on param server. make sure that this param exist in arm_config.yaml "
                              "and that your launch includes this param file. shutting down...", ARM_CONFIG_PARAM);
            ros::shutdown();
            exit (EXIT_FAILURE);
        }
        node_handle_->getParam(ARM_CONFIG_PARAM, arm_config_);
        if (arm_config_.getType() != XmlRpc::XmlRpcValue::TypeArray)
        {
            ROS_ERROR("[dxl_motors_builder]: %s param is invalid (need to be an of type array) or missing. "
                              "make sure that this param exist in arm_config.yaml and that your launch "
                              "includes this param file. shutting down...", ARM_CONFIG_PARAM);
            ros::shutdown();
            exit (EXIT_FAILURE);
        }
        node_handle_->getParam(SPEC_CONFIG_PARAM, dxl_spec_config_);
        if (dxl_spec_config_.getType() != XmlRpc::XmlRpcValue::TypeArray)
        {
            ROS_ERROR("[dxl_motors_builder]: %s param is invalid (need to be an of type array) or missing. "
                              "make sure that this param exist in arm_config.yaml and that your launch "
                              "includes this param file. shutting down...", SPEC_CONFIG_PARAM);
            ros::shutdown();
            exit (EXIT_FAILURE);
        }
        node_handle_->param<std::string>("port_name", arm_port_, "/dev/USB2DYNAMIXEL");
        node_handle_->param<int>("port_baudrate", arm_baudrate_, 1000000);
    }

    void DxlMotorsBuilder::buildMotors()
    {
        /* build motors */
        for(int i = 0; i < arm_config_.size(); i++)
        {
            /* feed motor with user defined settings */
            if(arm_config_[i].getType() != XmlRpc::XmlRpcValue::TypeStruct)
            {
                ROS_ERROR("[dxl_motors_builder]: arm motor id at index %d param data type is invalid or missing. "
                                  "make sure that this param exist in arm_config.yaml and that your launch includes this param file. shutting down...", i);
                ros::shutdown();
                exit (EXIT_FAILURE);
            }

            struct dxl::motor new_motor;

            /* defaults to prevent bad movement on startup */
            new_motor.spec.model = 0;
            new_motor.protocol_ver = 2.0;
            new_motor.command_position = 0.0;
            new_motor.command_velocity = 0.15;
            new_motor.pre_vel = 0.01;

            if(arm_config_[i]["id"].getType() != XmlRpc::XmlRpcValue::TypeInt) //invalid id field
            {
                ROS_ERROR("[dxl_motors_builder]: arm motor id at index %d: invalid data type or missing. "
                                  "make sure that this param exist in arm_config.yaml and that your launch includes this param file. shutting down...", i);
                ros::shutdown();
                exit (EXIT_FAILURE);
            }
            new_motor.id = static_cast<int>(arm_config_[i]["id"]);

            if (arm_config_[i]["joint_name"].getType() != XmlRpc::XmlRpcValue::TypeString) //invalid joint_name field
            {
                ROS_ERROR("[dxl_motors_builder]: arm motor joint_name at index %d: invalid data type or missing. "
                                  "make sure that this param exist in arm_config.yaml and that your launch includes this param file. shutting down...", i);
                ros::shutdown();
                exit (EXIT_FAILURE);
            }
            new_motor.joint_name = static_cast<std::string>(arm_config_[i]["joint_name"]);

            if (arm_config_[i]["interface"].getType() != XmlRpc::XmlRpcValue::TypeString) //invalid interface field
            {
                ROS_ERROR("[dxl_motors_builder]: arm motor interface_type at index %d: invalid data type or missing. "
                                  "make sure that this param exist in arm_config.yaml and that your launch includes this param file. shutting down...", i);
                ros::shutdown();
                exit (EXIT_FAILURE);
            }
            std::string string_interface_type = static_cast<std::string>(arm_config_[i]["interface"]);
            new_motor.interface_type = dxl::motor::stringToInterfaceType(string_interface_type);

            motors_.push_back(new_motor);
        }
    }

    void DxlMotorsBuilder::registerHandles(hardware_interface::JointStateInterface &joint_state_interface,
                                           hardware_interface::PositionJointInterface &position_interface,
                                           hardware_interface::PosVelJointInterface &posvel_interface)
    {
        for(dxl::motor &motor : motors_)
        {
            /* joint state registration */

            joint_state_handles_.push_back(hardware_interface::JointStateHandle (motor.joint_name,
                                                                                 &motor.position,
                                                                                 &motor.velocity,
                                                                                 &motor.effort));
            joint_state_interface.registerHandle(joint_state_handles_.back());

            /* joint command registration */
            switch (motor.interface_type)
            {
                case dxl::motor::POS:

                    pos_handles_.push_back(hardware_interface::JointHandle (joint_state_interface.getHandle(motor.joint_name),
                                                                            &motor.command_position));
                    joint_state_interface.registerHandle(pos_handles_.back());
                    break;
                case dxl::motor::POS_VEL:
                    posvel_handles_.push_back(hardware_interface::PosVelJointHandle (joint_state_interface.getHandle(motor.joint_name),
                                                                                     &motor.command_position,
                                                                                     &motor.command_velocity));
                    posvel_interface.registerHandle(posvel_handles_.back());
                    break;

            }
        }
    }

    void DxlMotorsBuilder::openPort()
    {
        dxl::DxlInterface::PortState port_state = dxl_interface_.openPort(arm_port_, arm_baudrate_);
        switch (port_state)
        {
            case dxl::DxlInterface::PORT_FAIL:
                ROS_ERROR("[dxl_motors_builder]: open arm port %s failed. "
                                  "make sure cable is connected and port has the right permissions. shutting down...", arm_port_.c_str());
                ros::shutdown();
                exit (EXIT_FAILURE);
            case dxl::DxlInterface::BAUDRATE_FAIL:
                ROS_ERROR("[dxl_motors_builder]: setting arm baudrate to %d failed. shutting down...", arm_baudrate_);
                ros::shutdown();
                exit (EXIT_FAILURE);
            case dxl::DxlInterface::SUCCESS:
                ROS_INFO("[dxl_motors_builder]: arm port opened successfully \nport name: %s \nbaudrate: %d", arm_port_.c_str(), arm_baudrate_);
        }
    }
}
