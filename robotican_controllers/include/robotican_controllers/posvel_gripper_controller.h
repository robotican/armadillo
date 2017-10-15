#ifndef ROBOTICAN_CONTROLLERS_POSVEL_GRIPPER_CONTROLLER_H
#define ROBOTICAN_CONTROLLERS_POSVEL_GRIPPER_CONTROLLER_H

#include <pluginlib/class_list_macros.h>
#include <control_msgs/GripperCommandAction.h>
#include <controller_interface/controller.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_server_goal_handle.h>
#include <hardware_interface/posvel_command_interface.h>
#include <ros/node_handle.h>

#include <actionlib/server/action_server.h>


namespace gripper_controllers{
struct Commands
{
    double gap; // Last commanded position
    double max_effort; // Max allowed effort
};
class PosVelGripperController: public controller_interface::Controller<hardware_interface::PosVelJointInterface>
{
private:

    typedef actionlib::ActionServer<control_msgs::GripperCommandAction>                         ActionServer;
    typedef boost::shared_ptr<ActionServer>                                                     ActionServerPtr;
typedef ActionServer::GoalHandle                                                            GoalHandle;
    typedef realtime_tools::RealtimeServerGoalHandle<control_msgs::GripperCommandAction>        RealtimeGoalHandle;
typedef boost::shared_ptr<RealtimeGoalHandle>                                               RealtimeGoalHandlePtr;

  hardware_interface::PosVelJointHandle leftjoint;
  hardware_interface::PosVelJointHandle rightjoint;

  ActionServerPtr    action_server_;
  RealtimeGoalHandlePtr                        rt_active_goal_;     ///< Currently active action goal, if any.
  Commands command_struct_,command_struct_rt_; // pre-allocated memory that is re-used to set the realtime buffer
 realtime_tools::RealtimeBuffer<Commands> command_;
control_msgs::GripperCommandResultPtr        pre_alloc_result_;
 ros::Time last_movement_time_;
 ros::Timer         goal_handle_timer_;
 ros::Duration action_monitor_period_;
ros::NodeHandle    controller_nh_;
double stall_timeout_, stall_velocity_threshold_;                 ///< Stall related parameters
double default_max_effort_;                                       ///< Max allowed effort
double goal_tolerance_;
double _lastGap;

double joints_vel_;

public:

   bool init(hardware_interface::PosVelJointInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);
   void starting(const ros::Time& time);
   void update(const ros::Time& time, const ros::Duration& period);
   void stopping();

 void goalCB(GoalHandle gh);
 void cancelCB(GoalHandle gh);
 void preemptActiveGoal();
 void setHoldPosition();
 double gap2Pos(double gap);
 double pos2Gap(double pos);
 void checkForSuccess(const ros::Time &time, double error_position, double current_position, double current_velocity,
                      double current_effort,double max_effort);

};

PLUGINLIB_EXPORT_CLASS(gripper_controllers::PosVelGripperController, controller_interface::ControllerBase);

}
#endif
