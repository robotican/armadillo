
#ifndef ARMADILLO2_HW_RICBOARD_PUB_H
#define ARMADILLO2_HW_RICBOARD_PUB_H

#include <ric_interface/ric_interface.h>
#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>

#define RIC_PORT_PARAM "ric_port"
#define RIC_PUB_INTERVAL 0.1 //secs

class RicboardPub
{
private:
    ros::Publisher ric_gps_pub_;
    ros::Publisher ric_ultrasonic_pub_;
    ros::Publisher ric_imu_pub_;
    ros::Timer ric_pub_timer_;

    ric_interface::RicInterface ric_;
    ros::NodeHandle *nh_;
    std::string ric_port_;
    bool load_ric_hw_ = true;

    void pubTimerCB(const ros::TimerEvent& event);

public:
    RicboardPub(ros::NodeHandle &nh);
    void loop();
    void read();
    void write();
    void startPublish();
    void stopPublish();
};


#endif //ARMADILLO2_HW_RICBOARD_PUB_H
