//
// Created by tom on 07/04/16.
//
#include <pluginlib/class_list_macros.h>
#include <robotican_controllers/two_finger_controller.h>

namespace position_controllers
{
    /**
     * \brief Gripper action controller that sends
     * commands to a \b position interface.
     */
    typedef gripper_action_controller::GripperActionControllerTwo<hardware_interface::PositionJointInterface>
            GripperActionControllerTwo;
}

namespace effort_controllers
{
    /**
     * \brief Gripper action controller that sends
     * commands to a \b effort interface.
     */
    typedef gripper_action_controller::GripperActionControllerTwo<hardware_interface::EffortJointInterface>
            GripperActionControllerTwo;
}

PLUGINLIB_EXPORT_CLASS(position_controllers::GripperActionControllerTwo, controller_interface::ControllerBase);
