
#include "ricboard_pub.h"

RicboardPub::RicboardPub(ros::NodeHandle &nh)
{
    nh_ = &nh;

    /* get ric params */
    nh_->getParam("load_ric_hw", load_ric_hw_);

    if (load_ric_hw_)
    {
        if (!nh_->hasParam(RIC_PORT_PARAM))
        {
            ROS_ERROR("[armadillo2_hw/ricboard_pub]: %s param is missing on param server. make sure that this param exist in ricboard_config.yaml "
                              "and that your launch includes this param file. shutting down...", RIC_PORT_PARAM);
            ros::shutdown();
            exit (EXIT_FAILURE);
        }
        nh_->getParam(RIC_PORT_PARAM, ric_port_);

        if (!nh_->hasParam(TORSO_JOINT_PARAM))
        {
            ROS_ERROR("[armadillo2_hw/ricboard_pub]: %s param is missing on param server. make sure that this param exist in ricboard_config.yaml "
                              "and that your launch includes this param file. shutting down...", TORSO_JOINT_PARAM);
            ros::shutdown();
            exit (EXIT_FAILURE);
        }
        nh_->getParam(RIC_PORT_PARAM, ric_state_.torso.joint_name);

        /* if connection fails, exception will be thrown */
        ric_.connect(ric_port_);
        ROS_INFO("[armadillo2_hw/ricboard_pub]: ricboard port opened successfully \nport name: %s \nbaudrate: 115200", ric_port_.c_str());

        /* ric publisher */
        ric_gps_pub_ = nh.advertise<sensor_msgs::NavSatFix>("gps", 10);
        ric_ultrasonic_pub_ = nh.advertise<sensor_msgs::Range>("ultrasonic", 10);
        ric_imu_pub_ = nh.advertise<sensor_msgs::Imu>("imu", 10);

        ric_pub_timer_ = nh.createTimer(ros::Duration(RIC_PUB_INTERVAL), &RicboardPub::pubTimerCB, this);
        ROS_INFO("[armadillo2_hw/ricboard_pub]: ricboard is up");
    }
}

void RicboardPub::loop()
{
    if (!load_ric_hw_)
        return;
    if (ric_.isBoardAlive())

    ric_.loop();
}

void RicboardPub::pubTimerCB(const ros::TimerEvent &event)
{
    /* update robot state according to ric sensor for joints_states */
    ric_state_.torso.pos =  ric_.getSensorsState().ultrasonic.distance_mm / 1000.0;
    ric_state_.torso.vel = (ric_state_.torso.pos - ric_state_.torso.prev_pos) / RIC_PUB_INTERVAL;
    ric_state_.torso.prev_pos = ric_state_.torso.pos;

    /* publish ultrasonic */
    sensor_msgs::Range range_msg;
    range_msg.min_range = 0.3;
    range_msg.max_range = 3.0;
    range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
    range_msg.range = ric_.getSensorsState().ultrasonic.distance_mm / 1000.0;
    ric_ultrasonic_pub_.publish(range_msg);

    /* publish gps */
    sensor_msgs::NavSatFix gps_msg;
}

void RicboardPub::startPublish()
{
    ric_pub_timer_.start();
}

void RicboardPub::stopPublish()
{
    ric_pub_timer_.stop();
}

void RicboardPub::registerHandles(hardware_interface::JointStateInterface &joint_state_interface,
                                       hardware_interface::PositionJointInterface &position_interface,
                                       hardware_interface::PosVelJointInterface &posvel_interface)
{
    /* joint state registration */

    /*joint_state_handles_.push_back(hardware_interface::JointStateHandle (ric_state_.torso.joint_name,
                                                                         &ric_state_.torso.pos,
                                                                         &ric_state_.torso.vel,
                                                                         &motor.current));*/
    joint_state_interface.registerHandle(joint_state_handles_.back());

    /* joint command registration */

    posvel_handles_.push_back(hardware_interface::PosVelJointHandle (joint_state_interface.getHandle(ric_state_.torso.joint_name),
                                                                     &ric_state_.torso.command_pos,
                                                                     &ric_state_.torso.command_vel));
    posvel_interface.registerHandle(posvel_handles_.back());

}


