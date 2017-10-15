//
// Created by tom on 07/04/16.
//

#ifndef ROBOTICAN_CONTROLLERS_TWO_FINGER_CONTROLLER_H
#define ROBOTICAN_CONTROLLERS_TWO_FINGER_CONTROLLER_H
#include <cassert>
#include <iterator>
#include <stdexcept>
#include <string>

// Boost
#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>

// ROS
#include <ros/node_handle.h>

// URDF
#include <urdf/model.h>

// ROS messages
#include <control_msgs/GripperCommandAction.h>

// actionlib
#include <actionlib/server/action_server.h>

// ros_controls
#include <realtime_tools/realtime_server_goal_handle.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/internal/demangle_symbol.h>
#include <realtime_tools/realtime_buffer.h>
#include <hardware_interface/posvel_command_interface.h>

// Project
#include <gripper_action_controller/hardware_interface_adapter.h>
#include <cassert>
#include <iterator>
#include <stdexcept>
#include <string>

// Boost
#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>

// ROS
#include <ros/node_handle.h>

// URDF
#include <urdf/model.h>

// ROS messages
#include <control_msgs/GripperCommandAction.h>

// actionlib
#include <actionlib/server/action_server.h>

// ros_controls
#include <realtime_tools/realtime_server_goal_handle.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/internal/demangle_symbol.h>
#include <realtime_tools/realtime_buffer.h>

// Project
#include <gripper_action_controller/hardware_interface_adapter.h>

namespace gripper_action_controller
{

/**
 * \brief Controller for executing a gripper command action for simple single-dof grippers.
 *
 * \tparam HardwareInterface Controller hardware interface. Currently \p hardware_interface::PositionJointInterface and
 * \p hardware_interface::EffortJointInterface are supported out-of-the-box.
 */
    template <class HardwareInterface>
    class GripperActionControllerTwo : public controller_interface::Controller<HardwareInterface>
    {
    public:

        /**
         * \brief Store position and max effort in struct to allow easier realtime buffer usage
         */
        struct Commands
        {
            double position_; // Last commanded position
            double max_effort_; // Max allowed effort
        };

        GripperActionControllerTwo();

        /** \name Non Real-Time Safe Functions
         *\{*/
        bool init(HardwareInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);
        /*\}*/

        /** \name Real-Time Safe Functions
         *\{*/
        /** \brief Holds the current position. */
        void starting(const ros::Time& time);

        /** \brief Cancels the active action goal, if any. */
        void stopping(const ros::Time& time);

        void update(const ros::Time& time, const ros::Duration& period);
        /*\}*/

        realtime_tools::RealtimeBuffer<Commands> command_;
        Commands command_struct_, command_struct_rt_; // pre-allocated memory that is re-used to set the realtime buffer

        double gap2Pos(double gap) {
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

        double pos2Gap(double pos) {
            double rad_when_stright=0.144;
            double half_gap_on_zero_rad=0.021;
            double tip_r=0.09;
            return 2 * (half_gap_on_zero_rad + tip_r * sin(pos + rad_when_stright));
        }

    private:
        double _lastPosition;
        typedef actionlib::ActionServer<control_msgs::GripperCommandAction>                         ActionServer;
        typedef boost::shared_ptr<ActionServer>                                                     ActionServerPtr;
        typedef ActionServer::GoalHandle                                                            GoalHandle;
        typedef realtime_tools::RealtimeServerGoalHandle<control_msgs::GripperCommandAction>        RealtimeGoalHandle;
        typedef boost::shared_ptr<RealtimeGoalHandle>                                               RealtimeGoalHandlePtr;

        typedef HardwareInterfaceAdapter<HardwareInterface> HwIfaceAdapter;

        bool                                          update_hold_position_;

        bool                                         verbose_;            ///< Hard coded verbose flag to help in debugging
        std::string                                  name_;               ///< Controller name.
        hardware_interface::JointHandle leftjoint_;                          ///< Handles to controlled joints.
        hardware_interface::JointHandle rightjoint_;                          ///< Handles to controlled joints.
        std::string                     leftJoint_name_;                      ///< Controlled joint names.
        std::string                     rightJoint_name_;                      ///< Controlled joint names.

        HwIfaceAdapter                  left_hw_iface_adapter_;   ///< Adapts desired goal state to HW interface.
        HwIfaceAdapter                  right_hw_iface_adapter_;   ///< Adapts desired goal state to HW interface.

        RealtimeGoalHandlePtr                        rt_active_goal_;     ///< Currently active action goal, if any.
        control_msgs::GripperCommandResultPtr        pre_alloc_result_;

        ros::Duration action_monitor_period_;

        // ROS API
        ros::NodeHandle    controller_nh_;
        ros::Publisher     _gapPub;
        ActionServerPtr    action_server_;


        ros::Timer         goal_handle_timer_;

        void goalCB(GoalHandle gh);
        void cancelCB(GoalHandle gh);
        void preemptActiveGoal();
        void setHoldPosition(const ros::Time& time);

        ros::Time last_movement_time_;                                    ///< Store stall time
        double computed_command_;                                         ///< Computed command

        double stall_timeout_, stall_velocity_threshold_;                 ///< Stall related parameters
        double default_max_effort_;                                       ///< Max allowed effort
        double goal_tolerance_;
        /**
         * \brief Check for success and publish appropriate result and feedback.
         **/
        void checkForSuccess(const ros::Time &time, double error_position, double current_position, double current_velocity,
                             double current_effort,double max_effort);

    };

} // namespace

#include <robotican_controllers/gripper_action_controller_impl.h>

#endif //ROBOTICAN_CONTROLLERS_TWO_FINGER_CONTROLLER_H
