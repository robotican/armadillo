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

#ifndef ARMADILLO2_TELEOP_JOY_PROFILE_H
#define ARMADILLO2_TELEOP_JOY_PROFILE_H

struct joy_puppet
{
    bool init_state_recorded = false;
};
struct joy_twist : joy_puppet
{
    float axis_val_linear = 0;
    float axis_val_angular = 0;

    float scale_angular = 0;
    float scale_linear = 0;

    int joy_axis_linear = 0;
    int joy_axis_angular = 0;
};

struct joy_torso : joy_puppet
{
    float axis_val_updown = 0;
    float increment = 0; //meters

    int joy_axis_updown = 0;

    float limit_upper = 0;
    float limit_lower = 0;
};

struct joy_arm : joy_puppet
{
    std::vector<double> axes_vals;
    std::vector<double> axes_vals_prev;

    static const uint8_t INDX_ROTATION1 = 0;
    static const uint8_t INDX_SHOULDER1 = 1;
    static const uint8_t INDX_SHOULDER2 = 2;
    static const uint8_t INDX_ROTATION2 = 3;
    static const uint8_t INDX_SHOULDER3 = 4;
    static const uint8_t INDX_WRIST = 5;
    static const uint8_t DOF = 6;

    int joy_axis_rotation1 = 0;
    int joy_axis_shoulder1 = 0;
    int joy_axis_shoulder2 = 0;
    int joy_axis_rotation2 = 0;
    int joy_btn_shoulder3_up = 0;
    int joy_btn_shoulder3_down = 0;
    int joy_btn_wrist_cw = 0;
    int joy_btn_wrist_ccw = 0;
    int joy_btn_reset = 0;
    float increment = 0;

    std::string start_pos = "ninety_deg";

    joy_arm()
    {
        axes_vals.reserve(6);
        axes_vals_prev.reserve(6);
    }
};

struct joy_gripper : joy_puppet
{
    int joy_axis = 0;
    control_msgs::GripperCommandGoal goal;
    float increment = 0;
    float limit_upper = 0;
    float limit_lower = 0;
};

struct joy_pan_tilt : joy_puppet
{
    float axis_val_pan = 0;
    float axis_val_tilt = 0;

    float inc_pan = 0;
    float inc_tilt = 0;

    int joy_btn_pan_right = 0;
    int joy_btn_pan_left = 0;
    int joy_btn_tilt_up = 0;
    int joy_btn_tilt_down = 0;

    float limit_upper_pan = 0;
    float limit_lower_pan = 0;
    float limit_upper_tilt = 0;
    float limit_lower_tilt = 0;
};

struct joy_utils
{
    int joy_btn_arm_mode = 0;
    int joy_btn_safety = 0;
};

struct joy_profile
{
    joy_torso torso;
    joy_twist twist;
    joy_arm arm;
    joy_gripper gripper;
    joy_pan_tilt head;
    joy_utils utils;
};


#endif //ARMADILLO2_TELEOP_JOY_PROFILE_H
