#include <pluginlib/class_list_macros.h>
#include "robotican_controllers/posvel_gripper_controller.h"
namespace gripper_controllers {


/// Controller initialization in non-realtime
bool PosVelGripperController::init(hardware_interface::PosVelJointInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
{

    // Default tolerances
    controller_nh_.param<double>("/gripper_controller/goal_tolerance", goal_tolerance_, 0.005);
    goal_tolerance_ = fabs(goal_tolerance_);
    // Max allowable effort
    controller_nh_.param<double>("/gripper_controller/max_effort", default_max_effort_, 0.0);
    default_max_effort_ = fabs(default_max_effort_);
    // Stall - stall velocity threshold, stall timeout
    controller_nh_.param<double>("/gripper_controller/stall_velocity_threshold", stall_velocity_threshold_, 0.001);
    controller_nh_.param<double>("/gripper_controller/stall_timeout", stall_timeout_, 1.0);

    controller_nh_.param<double>("/gripper_controller/joints_vel", joints_vel_, 0.05);


    std::string left_finger_joint_name="left_finger_joint";
  std::string right_finger_joint_name="right_finger_joint";
  controller_nh_.getParam("/gripper_controller/left_joint", left_finger_joint_name);
  controller_nh_.getParam("/gripper_controller/right_joint", right_finger_joint_name);
 leftjoint = hw->getHandle(left_finger_joint_name);
rightjoint = hw->getHandle(right_finger_joint_name);

// Cache controller node handle
controller_nh_ = controller_nh;

// Action status checking update rate
double action_monitor_rate = 20.0;
controller_nh_.getParam("/gripper_controller/action_monitor_rate", action_monitor_rate);
action_monitor_period_ = ros::Duration(1.0 / action_monitor_rate);
ROS_DEBUG("Action status changes will be monitored at %f Hz",action_monitor_rate);

// Result
pre_alloc_result_.reset(new control_msgs::GripperCommandResult());
pre_alloc_result_->position = command_struct_.gap;
pre_alloc_result_->reached_goal = false;
pre_alloc_result_->stalled = false;

action_server_.reset(new ActionServer(controller_nh, "gripper_cmd",
                                      boost::bind(&PosVelGripperController::goalCB, this, _1),
                                      boost::bind(&PosVelGripperController::cancelCB, this, _1),
                                      false));

action_server_->start();

  return true;
}


/// Controller startup in realtime
void PosVelGripperController::starting(const ros::Time& time)
{
    command_struct_rt_.gap = pos2Gap(rightjoint.getPosition());
    command_struct_rt_.max_effort = default_max_effort_;
    command_.initRT(command_struct_rt_);

    _lastGap = command_struct_rt_.gap;


    // Hardware interface adapter
    leftjoint.setCommand(leftjoint.getPosition(),joints_vel_);
    rightjoint.setCommand(rightjoint.getPosition(),joints_vel_);

    last_movement_time_ = time;
}


/// Controller update loop in realtime
void PosVelGripperController::update(const ros::Time& time, const ros::Duration& period)
{
    command_struct_rt_ = *(command_.readFromRT());

    double current_gap = pos2Gap(rightjoint.getPosition());
//ROS_INFO("current_gap: %f",current_position);
    double current_gap_velocity =  current_gap - _lastGap / period.toSec();
    double current_effort = (fabs(leftjoint.getEffort())>fabs(rightjoint.getEffort())) ? fabs(leftjoint.getEffort()):fabs(rightjoint.getEffort());
    double error_gap = command_struct_rt_.gap - current_gap;
    double error_velocity = - current_gap_velocity;
    _lastGap = current_gap;
    if (command_struct_rt_.max_effort==0) current_effort=-1;
    checkForSuccess(time, error_gap, current_gap, current_gap_velocity, current_effort,command_struct_rt_.max_effort);

    // Hardware interface adapter: Generate and send commands
    double jointsPos = gap2Pos(command_struct_rt_.gap);
    //ROS_WARN("jointsPos: %f",jointsPos);

    leftjoint.setCommand(-jointsPos,joints_vel_);
    rightjoint.setCommand(jointsPos,joints_vel_);


}


/// Controller stopping in realtime
void PosVelGripperController::stopping()
{

}

void PosVelGripperController::preemptActiveGoal()
{
    RealtimeGoalHandlePtr current_active_goal(rt_active_goal_);

    // Cancels the currently active goal
    if (current_active_goal)
    {
        // Marks the current goal as canceled
        rt_active_goal_.reset();
        if(current_active_goal->gh_.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)
            current_active_goal->gh_.setCanceled();
    }
}


void PosVelGripperController::goalCB(GoalHandle gh)
{
    ROS_INFO("Recieved new action goal");

    // Precondition: Running controller
    if (!this->isRunning())
    {
        ROS_ERROR("Can't accept new action goals. Controller is not running.");
        control_msgs::GripperCommandResult result;
        gh.setRejected(result);
        return;
    }

    // Try to update goal
    RealtimeGoalHandlePtr rt_goal(new RealtimeGoalHandle(gh));

    // Accept new goal
    preemptActiveGoal();
    gh.setAccepted();

    // This is the non-realtime command_struct
    // We use command_ for sharing

    command_struct_.gap = gh.getGoal()->command.position;
    command_struct_.max_effort = gh.getGoal()->command.max_effort;
    command_.writeFromNonRT(command_struct_);
    pre_alloc_result_->reached_goal = false;
    pre_alloc_result_->stalled = false;
    last_movement_time_ = ros::Time::now();
    // Setup goal status checking timer
    goal_handle_timer_ = controller_nh_.createTimer(action_monitor_period_,
                                                    &RealtimeGoalHandle::runNonRealtime,
                                                    rt_goal);
            goal_handle_timer_.start();
    rt_active_goal_ = rt_goal;
    ROS_INFO("GRIPPER: Got new goal");
}


void PosVelGripperController::cancelCB(GoalHandle gh)
{
    RealtimeGoalHandlePtr current_active_goal(rt_active_goal_);

    // Check that cancel request refers to currently active goal (if any)
    if (current_active_goal && current_active_goal->gh_ == gh)
    {
        // Reset current goal
        rt_active_goal_.reset();

        // Enter hold current position mode
        setHoldPosition();
        ROS_DEBUG( "Canceling active action goal because cancel callback recieved from actionlib.");

        // Mark the current goal as canceled
        current_active_goal->gh_.setCanceled();
    }
}

double PosVelGripperController::gap2Pos(double gap) {
    double max_gap=0.15;
    double rad_when_stright=0.144;
    double half_gap_on_zero_rad=0.021;
    double tip_r=0.09;

    if (gap  <0) gap=0;
    else if (gap>max_gap) gap=max_gap;


    double computeGap = ((gap / 2.0f) - half_gap_on_zero_rad) / tip_r;
    const double gap2Pos = asin(computeGap) - rad_when_stright;
    return gap2Pos;

}

double PosVelGripperController::pos2Gap(double pos) {
    double rad_when_stright=0.144;
    double half_gap_on_zero_rad=0.021;
    double tip_r=0.09;
    return 2 * (half_gap_on_zero_rad + tip_r * sin(pos + rad_when_stright));
}

void PosVelGripperController::setHoldPosition()
{

    command_struct_.gap = pos2Gap(rightjoint.getPosition());

    command_.writeFromNonRT(command_struct_);

}


void PosVelGripperController::checkForSuccess(const ros::Time &time, double error_gap, double current_gap, double current_gap_velocity,
                double current_effort,double max_effort)
{

//ROS_ERROR("error_gap: %f",error_gap);
    if(!rt_active_goal_) {
        return;
}

    if(rt_active_goal_->gh_.getGoalStatus().status != actionlib_msgs::GoalStatus::ACTIVE){
        return;
}
    if(fabs(error_gap) < goal_tolerance_)
    {
        pre_alloc_result_->effort = current_effort;
        pre_alloc_result_->position = current_gap;
        pre_alloc_result_->reached_goal = true;
        pre_alloc_result_->stalled = false;
        rt_active_goal_->setSucceeded(pre_alloc_result_);
        ROS_INFO("GRIPPER: Reached Goal");
    }
    else
    {
        if (current_effort >= fabs(max_effort)) {
            ROS_WARN("GRIPPER: MAX EFFORT");
            pre_alloc_result_->effort = current_effort;
            pre_alloc_result_->position = current_gap;
            pre_alloc_result_->reached_goal = false;
            pre_alloc_result_->stalled = true;

            ROS_WARN("pre_alloc_result_->effort: %f", pre_alloc_result_->effort);
            ROS_WARN("pre_alloc_result_->position: %f", pre_alloc_result_->position);

            rt_active_goal_->setSucceeded(pre_alloc_result_);
            setHoldPosition();
            return;
        }

        if(fabs(current_gap_velocity) > stall_velocity_threshold_) {

            last_movement_time_ = time;
        }
        else if( (time - last_movement_time_).toSec() > stall_timeout_)
        {
            ROS_WARN("GRIPPER: STALLED");
            pre_alloc_result_->effort = current_effort;
            pre_alloc_result_->position = current_gap;
            pre_alloc_result_->reached_goal = false;
            pre_alloc_result_->stalled = true;
            rt_active_goal_->setAborted(pre_alloc_result_);
            setHoldPosition();

        }
    }
}

} // namespace
