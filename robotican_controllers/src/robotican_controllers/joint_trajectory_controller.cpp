//
// Created by tom on 10/04/16.
//
#include <pluginlib/class_list_macros.h>
#include <robotican_controllers/joint_trajectory_controller.h>
#include <trajectory_interface/quintic_spline_segment.h>

namespace pos_vel_controllers {
    typedef joint_trajectory_controller::JointTrajectoryController<trajectory_interface::QuinticSplineSegment<double>, hardware_interface::PosVelJointInterface> JointTrajectoryController;
}

PLUGINLIB_EXPORT_CLASS(pos_vel_controllers::JointTrajectoryController, controller_interface::ControllerBase)