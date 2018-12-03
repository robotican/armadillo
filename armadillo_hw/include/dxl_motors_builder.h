/*******************************************************************************
* Copyright (c) 2018, RoboTICan, LTD.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright notice, this
*   list of conditions and the following disclaimer.
*
* * Redistributions in binary form must reproduce the above copyright notice,
*   this list of conditions and the following disclaimer in the documentation
*   and/or other materials provided with the distribution.
*
* * Neither the name of RoboTICan nor the names of its
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/
/* Author: Elchay Rauper*/


#ifndef ARMADILLO2_HW_DXL_MOTORS_BUILDER_H
#define ARMADILLO2_HW_DXL_MOTORS_BUILDER_H

#include <dxl_interface/dxl_interface.h>
#include <std_srvs/SetBool.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/posvel_command_interface.h>
#include <boost/thread.hpp>

#define MAX_PING_ERRORS 5
#define MAX_READ_ERRORS 100
#define MAX_WRITE_ERRORS 100
#define DXL_RECOVER_TIMEOUT 1 //secs
#define DXL_JOINTS_CONFIG_PARAM "dxl_joints_config"
#define SPEC_CONFIG_PARAM "dxl_spec_config"
#define DXL_PROTOCOL_PARAM "~dxl_protocol"
#define DXL_PORT_PARAM "~dxl_port_name"
#define DXL_PORT_BAUD_PARAM "~dxl_port_baudrate"


namespace armadillo2_hw
{

    struct read_write_errs
    {
        bool read_err_pos = false;
        bool read_err_vel = false;
        bool read_err_load = false;
        bool read_err_report = false;
        int failed_reads_ = 0;

        bool write_err_pos = false;
        bool write_err_vel = false;
        int failed_writes_ = 0;
    };

    class DxlMotorsBuilder
    {
    public:

        DxlMotorsBuilder(ros::NodeHandle &nh);
        void registerHandles(hardware_interface::JointStateInterface &joint_state_interface,
                             hardware_interface::PositionJointInterface &position_interface,
                             hardware_interface::PosVelJointInterface &posvel_interface);

        /* read/write for controller use */
        void read();
        void write();

        /* writing to single motor, if this motor was build by dxl_builder */
        void writeToMotor(int motor_id, double position, double velocity);

    private:

        ros::NodeHandle *nh_;
        ros::Timer dxl_dead_timer_;

        /* handles */
        std::vector<hardware_interface::JointStateHandle> joint_state_handles_;
        std::vector<hardware_interface::PosVelJointHandle> posvel_handles_;
        std::vector<hardware_interface::JointHandle> pos_handles_;
        read_write_errs comm_errs_;
        int dxl_baudrate_ = 0;
        std::string dxl_port_;
        float protocol_ = 0;
        bool load_dxl_hw_ = true;
        std::map<uint16_t, dxl::spec> specs_; /* key - model number, value - dxl spec */
        dxl::DxlInterface dxl_interface_;
        XmlRpc::XmlRpcValue dxl_joints_config_, dxl_spec_config_;
        std::vector<dxl::motor> motors_;
        ros::ServiceServer torque_srv_;
        ros::Publisher espeak_pub_;
        /* prevent parallel access to dxl motors */
        boost::mutex comm_mutex_;

        bool torqueServiceCB(std_srvs::SetBool::Request  &req,
                             std_srvs::SetBool::Response &res);

        void fetchParams();
        void openPort();
        void buildMotors();
        void pingMotors();
        bool setTorque(bool flag);
        void loadSpecs();
        /* writing directly to motor hardware */
        void write(std::vector<dxl::motor> &motors);
        void dxlDeadTimerCB(const ros::TimerEvent& event);
        void speakMsg(std::string msg, int sleep_time)
        {
            std_msgs::String speak_msg;
            speak_msg.data = msg;
            espeak_pub_.publish(speak_msg);
            if (sleep_time > 0)
                sleep(sleep_time);
        }
    };
}
#endif //ARMADILLO2_HW_DXL_MOTORS_BUILDER_H
