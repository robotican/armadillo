
#include "ricboard_pub.h"

RicboardPub::RicboardPub(ros::NodeHandle &nh)
{
    nh_ = &nh;

    /* get ric params */
    nh_->getParam("load_ric_hw", load_ric_hw_);

    if (!load_ric_hw_)
        return;

    if (!nh_->hasParam(RIC_PORT_PARAM))
    {
        ROS_ERROR("[armadillo2_hw/ricboard_pub]: %s param is missing on param server. make sure that this param exist in ricboard_config.yaml "
                          "and that your launch includes this param file. shutting down...", RIC_PORT_PARAM);
        ros::shutdown();
        exit (EXIT_FAILURE);
    }
    nh_->getParam(RIC_PORT_PARAM, ric_port_);

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

void RicboardPub::loop()
{
    if (!load_ric_hw_)
        return;
    if (ric_.isBoardAlive())

    ric_.loop();
}

void RicboardPub::pubTimerCB(const ros::TimerEvent &event)
{
    ric_interface::sensors_state sensors =  ric_.getSensorsState();

    /* publish ultrasonic */
    sensor_msgs::Range range_msg;
    range_msg.min_range = 0.3;
    range_msg.max_range = 3.0;
    range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
    range_msg.range = sensors.ultrasonic.distance_mm / 1000.0;
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


