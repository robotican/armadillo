
#ifndef ARMADILLO_HW_ARMADILLO_HW_H
#define ARMADILLO_HW_ARMADILLO_HW_H


#include "dxl_interface.h"
#include "dxl_motors_builder.h"
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/posvel_command_interface.h>

namespace armadillo2_hw
{
    class ArmadilloHW : public hardware_interface::RobotHW
    {
    private:

        ros::Time prev_time_;
        ros::NodeHandle *node_handle_;
        /* interfaces */
        hardware_interface::JointStateInterface joint_state_interface_;
        hardware_interface::PositionJointInterface position_interface_;
        hardware_interface::PosVelJointInterface posvel_interface_;

        /* robot close loop components */
        DxlMotorsBuilder dxl_motors_;



        void registerInterfaces();

    public:

        ArmadilloHW(ros::NodeHandle &nh);
        void read();
        void write();
        static ros::Time getTime() { return ros::Time::now(); }
        ros::Duration getPeriod();

    };
}

#endif //ARMADILLO_HW_ARMADILLO_HW_H
