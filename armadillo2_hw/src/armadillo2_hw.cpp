
#include "armadillo2_hw.h"

namespace armadillo2_hw
{

    ArmadilloHW::ArmadilloHW(ros::NodeHandle &nh) :
            dxl_motors_(nh), battery_(nh), ric_(nh), roboteq_(nh)
    {
        node_handle_ = &nh;

        /* register handles */
        dxl_motors_.registerHandles(joint_state_interface_,
                                    position_interface_,
                                    posvel_interface_);
        ric_.registerHandles(joint_state_interface_,
                             position_interface_);
        roboteq_.registerHandles(joint_state_interface_,
                                 velocity_interface_);

        /* register interfaces */
        registerInterface(&joint_state_interface_);
        registerInterface(&posvel_interface_);
        registerInterface(&position_interface_);
        registerInterface(&velocity_interface_);

        prev_time_ = getTime();
        ROS_INFO("[armadillo2_hw]: armadillo hardware interface loaded successfully");
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
        dxl_motors_.read();
        roboteq_.read(getPeriod());
        ric_.read();
    }

    void ArmadilloHW::write()
    {
        dxl_motors_.write();
        roboteq_.write(getPeriod());
        ric_.write();
    }

    void ArmadilloHW::loop()
    {
        ric_.loop();
    }
}
