//
// Created by tom on 07/04/16.
//

#ifndef ROBOTICAN_CONTROLLERS_GRIPPER_ACTION_CONTROLLER_IMPL_H
#define ROBOTICAN_CONTROLLERS_GRIPPER_ACTION_CONTROLLER_IMPL_H



namespace gripper_action_controller
{
    namespace internal
    {
        std::string getLeafNamespace(const ros::NodeHandle& nh)
        {
            const std::string complete_ns = nh.getNamespace();
            std::size_t id   = complete_ns.find_last_of("/");
            return complete_ns.substr(id + 1);
        }

        boost::shared_ptr<urdf::Model> getUrdf(const ros::NodeHandle& nh, const std::string& param_name)
        {
            boost::shared_ptr<urdf::Model> urdf(new urdf::Model);

            std::string urdf_str;
            // Check for robot_description in proper namespace
            if (nh.getParam(param_name, urdf_str))
            {
                if (!urdf->initString(urdf_str))
                {
                    ROS_ERROR_STREAM("Failed to parse URDF contained in '" << param_name << "' parameter (namespace: " <<
                                                                           nh.getNamespace() << ").");
                    return boost::shared_ptr<urdf::Model>();
                }
            }
                // Check for robot_description in root
            else if (!urdf->initParam("robot_description"))
            {
                ROS_ERROR_STREAM("Failed to parse URDF contained in '" << param_name << "' parameter");
                return boost::shared_ptr<urdf::Model>();
            }
            return urdf;
        }

        typedef boost::shared_ptr<const urdf::Joint> UrdfJointConstPtr;
        std::vector<UrdfJointConstPtr> getUrdfJoints(const urdf::Model& urdf, const std::vector<std::string>& joint_names)
        {
            std::vector<UrdfJointConstPtr> out;
            for (unsigned int i = 0; i < joint_names.size(); ++i)
            {
                UrdfJointConstPtr urdf_joint = urdf.getJoint(joint_names[i]);
                if (urdf_joint)
                {
                    out.push_back(urdf_joint);
                }
                else
                {
                    ROS_ERROR_STREAM("Could not find joint '" << joint_names[i] << "' in URDF model.");
                    return std::vector<UrdfJointConstPtr>();
                }
            }
            return out;
        }

    } // namespace

    template <class HardwareInterface>
    inline void GripperActionControllerTwo<HardwareInterface>::
    starting(const ros::Time& time)
    {
        command_struct_rt_.position_ = pos2Gap(rightjoint_.getPosition());
        command_struct_rt_.max_effort_ = default_max_effort_;
        command_.initRT(command_struct_rt_);

        _lastPosition = command_struct_rt_.position_;


        // Hardware interface adapter
        left_hw_iface_adapter_.starting(ros::Time(0.0));
        right_hw_iface_adapter_.starting(ros::Time(0.0));
        last_movement_time_ = time;
    }

    template <class HardwareInterface>
    inline void GripperActionControllerTwo<HardwareInterface>::
    stopping(const ros::Time& time)
    {
        preemptActiveGoal();
    }

    template <class HardwareInterface>
    inline void GripperActionControllerTwo<HardwareInterface>::
    preemptActiveGoal()
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

    template <class HardwareInterface>
    GripperActionControllerTwo<HardwareInterface>::
    GripperActionControllerTwo()
            : verbose_(false) // Set to true during debugging
    {}

    template <class HardwareInterface>
    bool GripperActionControllerTwo<HardwareInterface>::init(HardwareInterface* hw,
                                                             ros::NodeHandle&   root_nh,
                                                             ros::NodeHandle&   controller_nh)
    {
        using namespace internal;

        // Cache controller node handle
        controller_nh_ = controller_nh;

        // Controller name
        name_ = getLeafNamespace(controller_nh_);

        // Action status checking update rate
        double action_monitor_rate = 20.0;
        controller_nh_.getParam("action_monitor_rate", action_monitor_rate);
        action_monitor_period_ = ros::Duration(1.0 / action_monitor_rate);
        ROS_DEBUG_STREAM_NAMED(name_, "Action status changes will be monitored at " << action_monitor_rate << "Hz.");

        // Controlled joint
        controller_nh_.getParam("leftJoint", leftJoint_name_);
        controller_nh_.getParam("rightJoint", rightJoint_name_);
        _gapPub = controller_nh_.advertise<control_msgs::GripperCommandResult>("current_gap", 10);
        if (leftJoint_name_.empty() || rightJoint_name_.empty())
        {
            ROS_ERROR_STREAM_NAMED(name_, "Could not find joint name on param server");
            return false;
        }

        // URDF joints
        boost::shared_ptr<urdf::Model> urdf = getUrdf(root_nh, "robot_description");
        if (!urdf)
        {
            return false;
        }

        std::vector<std::string> joint_names;
        joint_names.push_back(leftJoint_name_);
        joint_names.push_back(rightJoint_name_);
        std::vector<UrdfJointConstPtr> urdf_joints = getUrdfJoints(*urdf, joint_names);
        if (urdf_joints.empty())
        {
            return false;
        }

        // Initialize members
        // Joint handle
        try
        {
            leftjoint_ = hw->getHandle(leftJoint_name_);
            rightjoint_ = hw->getHandle(rightJoint_name_);
        }
        catch (...)
        {
            ROS_ERROR_STREAM_NAMED(name_, "Could not find joint '" << leftJoint_name_ << "' in '" <<
                                                                   this->getHardwareInterfaceType() << "'.");
            return false;
        }

        ROS_DEBUG_STREAM_NAMED(name_, "Initialized controller '" << name_ << "' with:" <<
                                                                 "\n- Hardware interface type: '" << this->getHardwareInterfaceType() << "'" <<
                                                                 "\n");

        // Default tolerances
        controller_nh_.param<double>("goal_tolerance", goal_tolerance_, 0.01);
        goal_tolerance_ = fabs(goal_tolerance_);
        // Max allowable effort
        controller_nh_.param<double>("max_effort", default_max_effort_, 0.0);
        default_max_effort_ = fabs(default_max_effort_);
        // Stall - stall velocity threshold, stall timeout
        controller_nh_.param<double>("stall_velocity_threshold", stall_velocity_threshold_, 0.001);
        controller_nh_.param<double>("stall_timeout", stall_timeout_, 1.0);

        // Hardware interface adapter
        left_hw_iface_adapter_.init(leftjoint_, controller_nh_);
        right_hw_iface_adapter_.init(rightjoint_, controller_nh_);

        // Command - non RT version
        command_struct_.position_ = pos2Gap(rightjoint_.getPosition());
        command_struct_.max_effort_ = default_max_effort_;

        // Result
        pre_alloc_result_.reset(new control_msgs::GripperCommandResult());
        pre_alloc_result_->position = command_struct_.position_;
        pre_alloc_result_->reached_goal = false;
        pre_alloc_result_->stalled = false;

        // ROS API: Action interface
        action_server_.reset(new ActionServer(controller_nh_, "gripper_cmd",
                                              boost::bind(&GripperActionControllerTwo::goalCB, this, _1),
                                              boost::bind(&GripperActionControllerTwo::cancelCB, this, _1),
                                              false));

        action_server_->start();
        return true;
    }

    template <class HardwareInterface>
    void GripperActionControllerTwo<HardwareInterface>::
    update(const ros::Time& time, const ros::Duration& period)
    {
        command_struct_rt_ = *(command_.readFromRT());

        double current_position = pos2Gap(rightjoint_.getPosition());
//ROS_INFO("current_gap: %f",current_position);
        double current_velocity =  current_position - _lastPosition / period.toSec();
        double current_effort = (fabs(leftjoint_.getEffort())>fabs(rightjoint_.getEffort())) ? fabs(leftjoint_.getEffort()):fabs(rightjoint_.getEffort());
        double error_position = command_struct_rt_.position_ - current_position;
        double error_velocity = - current_velocity;
        _lastPosition = current_position;
        if (command_struct_rt_.max_effort_==0) current_effort=-1;
        checkForSuccess(time, error_position, current_position, current_velocity, current_effort,command_struct_rt_.max_effort_);

        // Hardware interface adapter: Generate and send commands
        double jointsPos = gap2Pos(command_struct_rt_.position_);
//ROS_WARN("jointsPos: %f",jointsPos);
        computed_command_ = left_hw_iface_adapter_.updateCommand(time, period,
                                                                 -jointsPos, 0.0, error_position, error_velocity, command_struct_rt_.max_effort_);
        computed_command_ = right_hw_iface_adapter_.updateCommand(time, period,
                                                                  jointsPos, 0.0, error_position, error_velocity, command_struct_rt_.max_effort_);
    }

    template <class HardwareInterface>
    void GripperActionControllerTwo<HardwareInterface>::
    goalCB(GoalHandle gh)
    {
        ROS_INFO("Recieved new action goal");

        // Precondition: Running controller
        if (!this->isRunning())
        {
            ROS_ERROR_NAMED(name_, "Can't accept new action goals. Controller is not running.");
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
        command_struct_.position_ = gh.getGoal()->command.position;
        command_struct_.max_effort_ = gh.getGoal()->command.max_effort;
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

    template <class HardwareInterface>
    void GripperActionControllerTwo<HardwareInterface>::
    cancelCB(GoalHandle gh)
    {
        RealtimeGoalHandlePtr current_active_goal(rt_active_goal_);

        // Check that cancel request refers to currently active goal (if any)
        if (current_active_goal && current_active_goal->gh_ == gh)
        {
            // Reset current goal
            rt_active_goal_.reset();

            // Enter hold current position mode
            setHoldPosition(ros::Time(0.0));
            ROS_DEBUG_NAMED(name_, "Canceling active action goal because cancel callback recieved from actionlib.");

            // Mark the current goal as canceled
            current_active_goal->gh_.setCanceled();
        }
    }

    template <class HardwareInterface>
    void GripperActionControllerTwo<HardwareInterface>::
    setHoldPosition(const ros::Time& time)
    {
	ROS_WARN("INSIDE setHoldPosition");
	ROS_WARN("rightjoint_.getPosition(): %f", rightjoint_.getPosition());
	
        command_struct_.position_ = pos2Gap(rightjoint_.getPosition()) - 0.03;
	
	ROS_WARN("command_struct_.position_ = pos2Gap(rightjoint_.getPosition()): %f", pos2Gap(rightjoint_.getPosition()));

        command_.writeFromNonRT(command_struct_);

    }

    template <class HardwareInterface>
    void GripperActionControllerTwo<HardwareInterface>::
    checkForSuccess(const ros::Time &time, double error_position, double current_position, double current_velocity,
                    double current_effort,double max_effort)
    {

//ROS_ERROR("error_position: %f",error_position);
        if(!rt_active_goal_) {
            return;
}

        if(rt_active_goal_->gh_.getGoalStatus().status != actionlib_msgs::GoalStatus::ACTIVE){
            return;
}
        if(fabs(error_position) < goal_tolerance_)
        {
            pre_alloc_result_->effort = computed_command_;
            pre_alloc_result_->position = current_position;
            pre_alloc_result_->reached_goal = true;
            pre_alloc_result_->stalled = false;
            rt_active_goal_->setSucceeded(pre_alloc_result_);
            ROS_INFO("GRIPPER: Reached Goal");
        }
        else
        {
            if (current_effort >= fabs(max_effort)) {
                ROS_WARN("GRIPPER: MAX EFFORT");

		
		
                pre_alloc_result_->effort = computed_command_;
                pre_alloc_result_->position = current_position;
                pre_alloc_result_->reached_goal = false;
                pre_alloc_result_->stalled = true;


		        ROS_WARN("pre_alloc_result_->effort: %f", pre_alloc_result_->effort);
		        ROS_WARN("pre_alloc_result_->position: %f", pre_alloc_result_->position);

                rt_active_goal_->setSucceeded(pre_alloc_result_);
                setHoldPosition(ros::Time(0.0));
                return;
            }

            if(fabs(current_velocity) > stall_velocity_threshold_) {

                last_movement_time_ = time;
            }
            else if( (time - last_movement_time_).toSec() > stall_timeout_)
            {
                ROS_WARN("GRIPPER: STALLED");
                pre_alloc_result_->effort = computed_command_;
                pre_alloc_result_->position = current_position;
                pre_alloc_result_->reached_goal = false;
                pre_alloc_result_->stalled = true;
                rt_active_goal_->setAborted(pre_alloc_result_);
                setHoldPosition(ros::Time(0.0));

            }
        }
    }

}

#endif //ROBOTICAN_CONTROLLERS_GRIPPER_ACTION_CONTROLLER_IMPL_H
