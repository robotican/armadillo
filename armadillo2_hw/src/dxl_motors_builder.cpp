//
// Created by sub on 16/10/17.
//
#include "dxl_motors_builder.h"

namespace armadillo2_hw
{
    DxlMotorsBuilder::DxlMotorsBuilder(ros::NodeHandle &nh)
    {
        nh_->getParam("load_dxl_hw", load_dxl_hw_);
        if (load_dxl_hw_)
        {
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
                ROS_INFO ("[dxl_motors_builder]: done building motor id: %d, model: %d",
                          motor.id, motor.spec.model);
            }

            failed_reads_ = 0;
            failed_writes_ = 0;

            torque_srv_ = nh_->advertiseService("arm_torque", &DxlMotorsBuilder::torqueServiceCB, this);
        }
    }

    void DxlMotorsBuilder::read()
    {
        if (!load_dxl_hw_)
            return;
        if (!dxl_interface_.readMotorsPos(motors_))
        {
            //ROS_ERROR("[dxl_motors_builder]: reading motors position failed");
            failed_reads_++;
        }
        if (!dxl_interface_.readMotorsVel(motors_))
        {
            //ROS_ERROR("[dxl_motors_builder]: reading motors velocity failed");
            failed_reads_++;
        }
        if (!dxl_interface_.readMotorsLoad(motors_))
        {
            //ROS_ERROR("[dxl_motors_builder]: reading motors load failed");
            failed_reads_++;
        }
        if (!dxl_interface_.readMotorsError(motors_))
        {
            //ROS_ERROR("[dxl_motors_builder]: reading motors errors failed");
            failed_reads_++;
        }
        if (failed_reads_ >= MAX_READ_ERRORS)
        {
            ROS_ERROR("[dxl_motors_builder]: too many read errors, shutting down...");
            ros::shutdown();
            exit(EXIT_FAILURE);
        }
    }

    void DxlMotorsBuilder::write()
    {
        if (!load_dxl_hw_)
            return;
        if (!dxl_interface_.bulkWriteVelocity(motors_))
        {
            //ROS_ERROR("[dxl_motors_builder]: writing velocity failed");
            failed_writes_++;
        }

        if (!dxl_interface_.bulkWritePosition(motors_))
        {
            //ROS_ERROR("[dxl_motors_builder]: writing postision failed");
            failed_writes_++;
        }

        if (failed_writes_ >= MAX_WRITE_ERRORS)
        {
            ROS_ERROR("[dxl_motors_builder]: too many write errors, shutting down...");
            ros::shutdown();
            exit(EXIT_FAILURE);
        }

    }

    void DxlMotorsBuilder::pingMotors()
    {
        if (!load_dxl_hw_)
            return;
        ros::Duration(1).sleep();
        for (dxl::motor &motor : motors_)
        {
            int error_counter = 0;
            while (!dxl_interface_.ping(motor))
            {
                error_counter++;
                ROS_WARN("[dxl_motors_builder]: pinging motor id: %d failed", motor.id);
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
        if (!load_dxl_hw_)
            return;
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

            /* name */
            if(dxl_spec_config_[i]["name"].getType() != XmlRpc::XmlRpcValue::TypeString)
            {
                ROS_ERROR("[dxl_motors_builder]: spec name at index %d: invalid data type or missing. "
                                  "make sure that this param exist in dxl_joints_config.yaml and that your launch includes this param file. shutting down...", i);
                ros::shutdown();
                exit (EXIT_FAILURE);
            }
            spec.name = static_cast<std::string>(dxl_spec_config_[i]["name"]);

            /* model */
            if(dxl_spec_config_[i]["model"].getType() != XmlRpc::XmlRpcValue::TypeInt)
            {
                ROS_ERROR("[dxl_motors_builder]: spec model at index %d: invalid data type or missing. "
                                  "make sure that this param exist in dxl_joints_config.yaml and that your launch includes this param file. shutting down...", i);
                ros::shutdown();
                exit (EXIT_FAILURE);
            }
            spec.model = static_cast<int>(dxl_spec_config_[i]["model"]);

            /* cpr */
            if(dxl_spec_config_[i]["cpr"].getType() != XmlRpc::XmlRpcValue::TypeInt)
            {
                ROS_ERROR("[dxl_motors_builder]: spec cpr at index %d: invalid data type or missing. "
                                  "make sure that this param exist in dxl_joints_config.yaml and that your launch includes this param file. shutting down...", i);
                ros::shutdown();
                exit (EXIT_FAILURE);
            }
            spec.cpr = static_cast<int>(dxl_spec_config_[i]["cpr"]);

            /* rpm_factor */
            if(dxl_spec_config_[i]["rpm_factor"].getType() != XmlRpc::XmlRpcValue::TypeDouble)
            {
                ROS_ERROR("[dxl_motors_builder]: spec rpm_factor at index %d: invalid data type or missing. "
                                  "make sure that this param exist in dxl_joints_config.yaml and that your launch includes this param file. shutting down...", i);
                ros::shutdown();
                exit (EXIT_FAILURE);
            }
            spec.rpm_scale_factor = static_cast<double>(dxl_spec_config_[i]["rpm_factor"]);

            /* torque_const_a */
            if(dxl_spec_config_[i]["torque_const_a"].getType() != XmlRpc::XmlRpcValue::TypeDouble)
            {
                ROS_ERROR("[dxl_motors_builder]: spec torque_const_a at index %d: invalid data type or missing. "
                                  "make sure that this param exist in dxl_joints_config.yaml and that your launch includes this param file. shutting down...", i);
                ros::shutdown();
                exit (EXIT_FAILURE);
            }
            spec.torque_const_a = static_cast<double>(dxl_spec_config_[i]["torque_const_a"]);

            /* torque_const_b */
            if(dxl_spec_config_[i]["torque_const_b"].getType() != XmlRpc::XmlRpcValue::TypeDouble)
            {
                ROS_ERROR("[dxl_motors_builder]: spec torque_const_b at index %d: invalid data type or missing. "
                                  "make sure that this param exist in dxl_joints_config.yaml and that your launch includes this param file. shutting down...", i);
                ros::shutdown();
                exit (EXIT_FAILURE);
            }
            spec.torque_const_b = static_cast<double>(dxl_spec_config_[i]["torque_const_b"]);



            /* pos_read_addr */
            if(dxl_spec_config_[i]["pos_read_addr"].getType() != XmlRpc::XmlRpcValue::TypeInt)
            {
                ROS_ERROR("[dxl_motors_builder]: spec pos_read_addr at index %d: invalid data type or missing. "
                                  "make sure that this param exist in dxl_joints_config.yaml and that your launch includes this param file. shutting down...", i);
                ros::shutdown();
                exit (EXIT_FAILURE);
            }
            spec.pos_read_addr = static_cast<int>(dxl_spec_config_[i]["pos_read_addr"]);

            /* vel_read_addr */
            if(dxl_spec_config_[i]["vel_read_addr"].getType() != XmlRpc::XmlRpcValue::TypeInt)
            {
                ROS_ERROR("[dxl_motors_builder]: spec vel_read_addr at index %d: invalid data type or missing. "
                                  "make sure that this param exist in dxl_joints_config.yaml and that your launch includes this param file. shutting down...", i);
                ros::shutdown();
                exit (EXIT_FAILURE);
            }
            spec.vel_read_addr = static_cast<int>(dxl_spec_config_[i]["vel_read_addr"]);


            /* current_read_addr */
            if(dxl_spec_config_[i]["current_read_addr"].getType() != XmlRpc::XmlRpcValue::TypeInt)
            {
                ROS_ERROR("[dxl_motors_builder]: spec current_read_addr at index %d: invalid data type or missing. "
                                  "make sure that this param exist in dxl_joints_config.yaml and that your launch includes this param file. shutting down...", i);
                ros::shutdown();
                exit (EXIT_FAILURE);
            }
            spec.current_read_addr = static_cast<int>(dxl_spec_config_[i]["current_read_addr"]);

            /* error_read_addr */
            if(dxl_spec_config_[i]["error_read_addr"].getType() != XmlRpc::XmlRpcValue::TypeInt)
            {
                ROS_ERROR("[dxl_motors_builder]: spec error_read_addr at index %d: invalid data type or missing. "
                                  "make sure that this param exist in dxl_joints_config.yaml and that your launch includes this param file. shutting down...", i);
                ros::shutdown();
                exit (EXIT_FAILURE);
            }
            spec.error_read_addr = static_cast<int>(dxl_spec_config_[i]["error_read_addr"]);

            /* torque_write_addr */
            if(dxl_spec_config_[i]["torque_write_addr"].getType() != XmlRpc::XmlRpcValue::TypeInt)
            {
                ROS_ERROR("[dxl_motors_builder]: spec torque_write_addr at index %d: invalid data type or missing. "
                                  "make sure that this param exist in dxl_joints_config.yaml and that your launch includes this param file. shutting down...", i);
                ros::shutdown();
                exit (EXIT_FAILURE);
            }
            spec.torque_write_addr = static_cast<int>(dxl_spec_config_[i]["torque_write_addr"]);

            /* vel_write_addr */
            if(dxl_spec_config_[i]["vel_write_addr"].getType() != XmlRpc::XmlRpcValue::TypeInt)
            {
                ROS_ERROR("[dxl_motors_builder]: spec vel_write_addr at index %d: invalid data type or missing. "
                                  "make sure that this param exist in dxl_joints_config.yaml and that your launch includes this param file. shutting down...", i);
                ros::shutdown();
                exit (EXIT_FAILURE);
            }
            spec.vel_write_addr = static_cast<int>(dxl_spec_config_[i]["vel_write_addr"]);


            /* pos_write_addr */
            if(dxl_spec_config_[i]["pos_write_addr"].getType() != XmlRpc::XmlRpcValue::TypeInt)
            {
                ROS_ERROR("[dxl_motors_builder]: spec pos_write_addr at index %d: invalid data type or missing. "
                                  "make sure that this param exist in dxl_joints_config.yaml and that your launch includes this param file. shutting down...", i);
                ros::shutdown();
                exit (EXIT_FAILURE);
            }
            spec.pos_write_addr = static_cast<int>(dxl_spec_config_[i]["pos_write_addr"]);

            /* current_ratio */
            if(dxl_spec_config_[i]["current_ratio"].getType() != XmlRpc::XmlRpcValue::TypeDouble)
            {
                ROS_ERROR("[dxl_motors_builder]: spec current_ratio at index %d: invalid data type or missing. "
                                  "make sure that this param exist in dxl_joints_config.yaml and that your launch includes this param file. shutting down...", i);
                ros::shutdown();
                exit (EXIT_FAILURE);
            }
            spec.current_ratio = static_cast<double>(dxl_spec_config_[i]["current_ratio"]);

            /* len_present_speed */
            if(dxl_spec_config_[i]["len_present_speed"].getType() != XmlRpc::XmlRpcValue::TypeInt)
            {
                ROS_ERROR("[dxl_motors_builder]: spec len_present_speed at index %d: invalid data type or missing. "
                                  "make sure that this param exist in dxl_joints_config.yaml and that your launch includes this param file. shutting down...", i);
                ros::shutdown();
                exit (EXIT_FAILURE);
            }
            spec.len_present_speed = static_cast<int>(dxl_spec_config_[i]["len_present_speed"]);

            /* len_present_pos */
            if(dxl_spec_config_[i]["len_present_pos"].getType() != XmlRpc::XmlRpcValue::TypeInt)
            {
                ROS_ERROR("[dxl_motors_builder]: spec len_present_pos at index %d: invalid data type or missing. "
                                  "make sure that this param exist in dxl_joints_config.yaml and that your launch includes this param file. shutting down...", i);
                ros::shutdown();
                exit (EXIT_FAILURE);
            }
            spec.len_present_pos = static_cast<int>(dxl_spec_config_[i]["len_present_pos"]);

            /* len_present_curr */
            if(dxl_spec_config_[i]["len_present_curr"].getType() != XmlRpc::XmlRpcValue::TypeInt)
            {
                ROS_ERROR("[dxl_motors_builder]: spec len_present_curr at index %d: invalid data type or missing. "
                                  "make sure that this param exist in dxl_joints_config.yaml and that your launch includes this param file. shutting down...", i);
                ros::shutdown();
                exit (EXIT_FAILURE);
            }
            spec.len_present_curr = static_cast<int>(dxl_spec_config_[i]["len_present_curr"]);

            /* len_goal_speed */
            if(dxl_spec_config_[i]["len_goal_speed"].getType() != XmlRpc::XmlRpcValue::TypeInt)
            {
                ROS_ERROR("[dxl_motors_builder]: spec len_goal_speed at index %d: invalid data type or missing. "
                                  "make sure that this param exist in dxl_joints_config.yaml and that your launch includes this param file. shutting down...", i);
                ros::shutdown();
                exit (EXIT_FAILURE);
            }
            spec.len_goal_speed = static_cast<int>(dxl_spec_config_[i]["len_goal_speed"]);

            /* len_goal_pos */
            if(dxl_spec_config_[i]["len_goal_pos"].getType() != XmlRpc::XmlRpcValue::TypeInt)
            {
                ROS_ERROR("[dxl_motors_builder]: spec len_goal_pos at index %d: invalid data type or missing. "
                                  "make sure that this param exist in dxl_joints_config.yaml and that your launch includes this param file. shutting down...", i);
                ros::shutdown();
                exit (EXIT_FAILURE);
            }
            spec.len_goal_pos = static_cast<int>(dxl_spec_config_[i]["len_goal_pos"]);

            specs_[spec.model] = spec;
        }

        /* feed motors with model specs */
        for (dxl::motor &motor : motors_)
        {
            bool spec_found = specs_.find(motor.spec.model) != specs_.end();
            if (spec_found)
            {
                motor.spec.name = specs_[motor.spec.model].name;
                motor.spec.cpr = specs_[motor.spec.model].cpr;
                motor.spec.rpm_scale_factor = specs_[motor.spec.model].rpm_scale_factor;
                motor.spec.torque_const_a = specs_[motor.spec.model].torque_const_a;
                motor.spec.torque_const_b = specs_[motor.spec.model].torque_const_b;

                /* read addrs */
                motor.spec.pos_read_addr = specs_[motor.spec.model].pos_read_addr;
                motor.spec.vel_read_addr = specs_[motor.spec.model].vel_read_addr;
                motor.spec.current_read_addr = specs_[motor.spec.model].current_read_addr;
                motor.spec.error_read_addr = specs_[motor.spec.model].error_read_addr;

                /* write addrs */
                motor.spec.torque_write_addr = specs_[motor.spec.model].torque_write_addr;
                motor.spec.vel_write_addr = specs_[motor.spec.model].vel_write_addr;
                motor.spec.pos_write_addr = specs_[motor.spec.model].pos_write_addr;
                motor.spec.len_present_speed = specs_[motor.spec.model].len_present_speed;
                motor.spec.len_present_pos = specs_[motor.spec.model].len_present_pos;
                motor.spec.len_present_curr = specs_[motor.spec.model].len_present_curr;
                motor.spec.len_goal_speed = specs_[motor.spec.model].len_goal_speed;
                motor.spec.len_goal_pos = specs_[motor.spec.model].len_goal_pos;



                motor.spec.current_ratio = specs_[motor.spec.model].current_ratio;

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
        if (!load_dxl_hw_)
            return false;
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
        if (!load_dxl_hw_)
            return false;
        res.success = false;
        if (DxlMotorsBuilder::setTorque(req.data))
            res.success = true;
        return true;
    }


    void DxlMotorsBuilder::fetchParams()
    {
        if (!load_dxl_hw_)
            return;
        /* DXL_JOINTS_CONFIG_PARAM */
        if (!nh_->hasParam(DXL_JOINTS_CONFIG_PARAM))
        {
            ROS_ERROR("[dxl_motors_builder]: %s param is missing on param server. make sure that this param exist in dxl_joints_config.yaml "
                              "and that your launch includes this param file. shutting down...", DXL_JOINTS_CONFIG_PARAM);
            ros::shutdown();
            exit (EXIT_FAILURE);
        }
        nh_->getParam(DXL_JOINTS_CONFIG_PARAM, dxl_joints_config_);
        if (dxl_joints_config_.getType() != XmlRpc::XmlRpcValue::TypeArray)
        {
            ROS_ERROR("[dxl_motors_builder]: %s param is invalid (need to be an of type array) or missing. "
                              "make sure that this param exist in dxl_joints_config.yaml and that your launch "
                              "includes this param file. shutting down...", DXL_JOINTS_CONFIG_PARAM);
            ros::shutdown();
            exit (EXIT_FAILURE);
        }

        /* SPEC_CONFIG_PARAM */
        if (!nh_->hasParam(SPEC_CONFIG_PARAM))
        {
            ROS_ERROR("[dxl_motors_builder]: %s param is missing on param server. make sure that this param exist in dxl_joints_config.yaml "
                              "and that your launch includes this param file. shutting down...", SPEC_CONFIG_PARAM);
            ros::shutdown();
            exit (EXIT_FAILURE);
        }
        nh_->getParam(SPEC_CONFIG_PARAM, dxl_spec_config_);
        if (dxl_spec_config_.getType() != XmlRpc::XmlRpcValue::TypeArray)
        {
            ROS_ERROR("[dxl_motors_builder]: %s param is invalid (need to be an of type array) or missing. "
                              "make sure that this param exist in dxl_joints_config.yaml and that your launch "
                              "includes this param file. shutting down...", SPEC_CONFIG_PARAM);
            ros::shutdown();
            exit (EXIT_FAILURE);
        }

        /* DXL_PROTOCOL_PARAM */
        if (!nh_->hasParam(DXL_PROTOCOL_PARAM))
        {
            ROS_ERROR("[dxl_motors_builder]: %s param is missing on param server. make sure that this param exist in dxl_joints_config.yaml "
                              "and that your launch includes this param file. shutting down...", DXL_PROTOCOL_PARAM);
            ros::shutdown();
            exit (EXIT_FAILURE);
        }
        nh_->getParam(DXL_PROTOCOL_PARAM, protocol_);


        /* DXL_PORT_PARAM */
        if (!nh_->hasParam(DXL_PORT_PARAM))
        {
            ROS_ERROR("[dxl_motors_builder]: %s param is missing on param server. make sure that this param exist in dxl_joints_config.yaml "
                              "and that your launch includes this param file. shutting down...", DXL_PORT_PARAM);
            ros::shutdown();
            exit (EXIT_FAILURE);
        }
        nh_->getParam(DXL_PORT_PARAM, arm_port_);

        /* DXL_PORT_BAUD_PARAM */
        if (!nh_->hasParam(DXL_PORT_BAUD_PARAM))
        {
            ROS_ERROR("[dxl_motors_builder]: %s param is missing on param server. make sure that this param exist in dxl_joints_config.yaml "
                              "and that your launch includes this param file. shutting down...", DXL_PORT_BAUD_PARAM);
            ros::shutdown();
            exit (EXIT_FAILURE);
        }
        nh_->getParam(DXL_PORT_BAUD_PARAM, dxl_baudrate_);
    }

    void DxlMotorsBuilder::buildMotors()
    {
        if (!load_dxl_hw_)
            return;
        /* build motors */
        for(int i = 0; i < dxl_joints_config_.size(); i++)
        {
            /* feed motor with user defined settings */
            if(dxl_joints_config_[i].getType() != XmlRpc::XmlRpcValue::TypeStruct)
            {
                ROS_ERROR("[dxl_motors_builder]: arm motor id at index %d param data type is invalid or missing. "
                                  "make sure that this param exist in dxl_joints_config.yaml and that your launch includes this param file. shutting down...", i);
                ros::shutdown();
                exit (EXIT_FAILURE);
            }

            struct dxl::motor new_motor;

            /* defaults to prevent bad movement on startup */
            new_motor.spec.model = 0;
            new_motor.command_position = 0.0;
            new_motor.command_velocity = 0.15;
            new_motor.pre_vel = 0.01;

            if(dxl_joints_config_[i]["id"].getType() != XmlRpc::XmlRpcValue::TypeInt) //invalid id field
            {
                ROS_ERROR("[dxl_motors_builder]: arm motor id at index %d: invalid data type or missing. "
                                  "make sure that this param exist in dxl_joints_config.yaml and that your launch includes this param file. shutting down...", i);
                ros::shutdown();
                exit (EXIT_FAILURE);
            }
            new_motor.id = static_cast<int>(dxl_joints_config_[i]["id"]);

            if (dxl_joints_config_[i]["joint_name"].getType() != XmlRpc::XmlRpcValue::TypeString) //invalid joint_name field
            {
                ROS_ERROR("[dxl_motors_builder]: arm motor joint_name at index %d: invalid data type or missing. "
                                  "make sure that this param exist in dxl_joints_config.yaml and that your launch includes this param file. shutting down...", i);
                ros::shutdown();
                exit (EXIT_FAILURE);
            }
            new_motor.joint_name = static_cast<std::string>(dxl_joints_config_[i]["joint_name"]);

            if (dxl_joints_config_[i]["interface"].getType() != XmlRpc::XmlRpcValue::TypeString) //invalid interface field
            {
                ROS_ERROR("[dxl_motors_builder]: arm motor interface_type at index %d: invalid data type or missing. "
                                  "make sure that this param exist in dxl_joints_config.yaml and that your launch includes this param file. shutting down...", i);
                ros::shutdown();
                exit (EXIT_FAILURE);
            }
            std::string string_interface_type = static_cast<std::string>(dxl_joints_config_[i]["interface"]);
            new_motor.interface_type = dxl::motor::stringToInterfaceType(string_interface_type);

            motors_.push_back(new_motor);
        }
    }

    void DxlMotorsBuilder::registerHandles(hardware_interface::JointStateInterface &joint_state_interface,
                                           hardware_interface::PositionJointInterface &position_interface,
                                           hardware_interface::PosVelJointInterface &posvel_interface)
    {
        if (!load_dxl_hw_)
            return;
        for(dxl::motor &motor : motors_)
        {
            /* joint state registration */

            joint_state_handles_.push_back(hardware_interface::JointStateHandle (motor.joint_name,
                                                                                 &motor.position,
                                                                                 &motor.velocity,
                                                                                 &motor.current));
            joint_state_interface.registerHandle(joint_state_handles_.back());

            /* joint command registration */
            switch (motor.interface_type)
            {
                case dxl::motor::POS:

                    pos_handles_.push_back(hardware_interface::JointHandle (joint_state_interface.getHandle(motor.joint_name),
                                                                            &motor.command_position));
                    position_interface.registerHandle(pos_handles_.back());
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
        if (!load_dxl_hw_)
            return;
        dxl::DxlInterface::PortState port_state = dxl_interface_.openPort(arm_port_,
                                                                          dxl_baudrate_,
                                                                          protocol_);
        switch (port_state)
        {
            case dxl::DxlInterface::PORT_FAIL:
                ROS_ERROR("[dxl_motors_builder]: open arm port %s failed. "
                                  "make sure cable is connected and port has the right permissions. shutting down...", arm_port_.c_str());
                ros::shutdown();
                exit (EXIT_FAILURE);
            case dxl::DxlInterface::BAUDRATE_FAIL:
                ROS_ERROR("[dxl_motors_builder]: setting arm baudrate to %d failed. shutting down...", dxl_baudrate_);
                ros::shutdown();
                exit (EXIT_FAILURE);
            case dxl::DxlInterface::INVALID_PROTOCOL:
                ROS_ERROR("[dxl_motors_builder]: protocol version %f is invalid. shutting down...", protocol_);
                ros::shutdown();
                exit (EXIT_FAILURE);
            case dxl::DxlInterface::SUCCESS:
                ROS_INFO("[dxl_motors_builder]: arm port opened successfully \nport name: %s \nbaudrate: %d", arm_port_.c_str(), dxl_baudrate_);
        }
    }
}
