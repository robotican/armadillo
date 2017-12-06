
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
            ROS_ERROR("[armadillo2_hw/ricboard_pub]: %s param is missing on param server. make sure that you load this param exist in ricboard_config.yaml "
                              "and that your launch includes this param file. shutting down...", RIC_PORT_PARAM);
            ros::shutdown();
            exit (EXIT_FAILURE);
        }
        nh_->getParam(RIC_PORT_PARAM, ric_port_);

        if (!nh_->hasParam(TORSO_JOINT_PARAM))
        {
            ROS_ERROR("[armadillo2_hw/ricboard_pub]: %s param is missing on param server. make sure that this param exist in controllers.yaml "
                              "and that your launch includes this param file. shutting down...", TORSO_JOINT_PARAM);
            ros::shutdown();
            exit (EXIT_FAILURE);
        }
        nh_->getParam(TORSO_JOINT_PARAM, torso_.joint_name);

        try{
            ric_.connect(ric_port_);
            ROS_INFO("[armadillo2_hw/ricboard_pub]: ricboard port opened successfully \nport name: %s \nbaudrate: 115200", ric_port_.c_str());
        }catch (ric_interface::ConnectionExeption e) {
            ROS_ERROR("[armadillo2_hw/ricboard_pub]: can't open ricboard port. make sure that ricboard is connected. shutting down...");
            ros::shutdown();
            exit(1);
        }

        /* ric publishers */
        ric_gps_pub_ = nh.advertise<sensor_msgs::NavSatFix>("gps", 10);
        ric_ultrasonic_pub_ = nh.advertise<sensor_msgs::Range>("ultrasonic", 10);
        ric_imu_pub_ = nh.advertise<sensor_msgs::Imu>("imu", 10);

        ric_pub_timer_ = nh.createTimer(ros::Duration(RIC_PUB_INTERVAL), &RicboardPub::pubTimerCB, this);
        ric_dead_timer_ = nh.createTimer(ros::Duration(RIC_DEAD_TIMEOUT), &RicboardPub::ricDeadTimerCB, this);
        ROS_INFO("[armadillo2_hw/ricboard_pub]: ricboard is up");
    }
}

void RicboardPub::loop()
{
    if (!load_ric_hw_)
        return;

    ric_.loop();
    if (ric_.isBoardAlive())
    {
        ric_dead_timer_.stop();
        ric_pub_timer_.start();
    }
    else
    {
        ric_dead_timer_.start();
        ric_pub_timer_.stop();
    }
}

void RicboardPub::ricDeadTimerCB(const ros::TimerEvent &event)
{
    if (!load_ric_hw_)
        return;
    throw ric_interface::ConnectionExeption("[armadillo2_hw/ricboard_pub]: ricboard disconnected");
}

void RicboardPub::pubTimerCB(const ros::TimerEvent &event)
{
    if (!load_ric_hw_)
        return;

    ric_interface::sensors_state sensors = ric_.getSensorsState();

    /* update robot state according to ric sensor for joints_states */
    torso_.pos =  sensors.laser.distance_mm / 1000.0;
    torso_.vel = (torso_.pos - torso_.prev_pos) / RIC_PUB_INTERVAL;
    torso_.prev_pos = torso_.pos;

    /* publish ultrasonic */
    sensor_msgs::Range range_msg;
    range_msg.min_range = 0.3;
    range_msg.max_range = 3.0;
    range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
    range_msg.range = sensors.ultrasonic.distance_mm / 1000.0;
    ric_ultrasonic_pub_.publish(range_msg);

    /* publish imu */
    sensor_msgs::Imu imu_msg;

    tf::Quaternion orientation_q = tf::createQuaternionFromRPY(sensors.imu.roll_rad,
                                                               sensors.imu.pitch_rad,
                                                               sensors.imu.yaw_rad);
    geometry_msgs::Quaternion q_msg;
    q_msg.x = orientation_q.x();
    q_msg.y = orientation_q.y();
    q_msg.z = orientation_q.z();
    q_msg.w = orientation_q.w();

    imu_msg.orientation = q_msg;
    ric_imu_pub_.publish(imu_msg);

    /* publish gps */
    sensor_msgs::NavSatFix gps_msg;
}

void RicboardPub::registerHandles(hardware_interface::JointStateInterface &joint_state_interface,
                                       hardware_interface::PositionJointInterface &position_interface)
{
    if (!load_ric_hw_)
        return;
    /* joint state registration */

    joint_state_handles_.push_back(hardware_interface::JointStateHandle (torso_.joint_name,
                                                                         &torso_.pos,
                                                                         &torso_.vel,
                                                                         &torso_.effort));
    joint_state_interface.registerHandle(joint_state_handles_.back());

    /* joint command registration */
    pos_handles_.push_back(hardware_interface::JointHandle (joint_state_interface.getHandle(torso_.joint_name),
                                                            &torso_.command_pos));
    position_interface.registerHandle(pos_handles_.back());
}

void RicboardPub::write()
{

}

void RicboardPub::read()
{

}


