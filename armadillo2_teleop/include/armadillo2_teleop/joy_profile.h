
#ifndef ARMADILLO2_TELEOP_JOY_PROFILE_H
#define ARMADILLO2_TELEOP_JOY_PROFILE_H

struct twist_joy
{
    float axis_val_linear = 0;
    float axis_val_angular = 0;

    float scale_angular = 0;
    float scale_linear = 0;

    int joy_axis_linear = 0;
    int joy_axis_angular = 0;

    float scale_limit_angular = 0;
    float scale_limit_linear = 0;
};

struct torso_joy
{
    float axis_val_updown = 0;
    float inc_updown = 0; //meters

    int joy_axis_updown = 0;

    float limit_upper = 0;
    float limit_lower = 0;
};

struct arm_joy
{
    //std::vector<double> axes_vals;

    float axis_val_rotation1 = 0;
    float axis_val_rotation2 = 0;
    float axis_val_shoulder1 = 0;
    float axis_val_shoulder2 = 0;
    float axis_val_shoulder3 = 0;
    float axis_val_wrist = 0;

    float inc_rotation1 = 0;
    float inc_rotation2 = 0;
    float inc_shoulder1 = 0;
    float inc_shoulder2 = 0;
    float inc_shoulder3 = 0;
    float inc_wrist = 0;
};

struct gripper_joy
{
    float axis_val_right_finger = 0;
    float axis_val_left_finger = 0;

    float inc_right_finger = 0;
    float inc_left_finger = 0;
};

struct pan_tilt_joy
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

struct joy_profile
{
    torso_joy torso;
    twist_joy twist;
    arm_joy arm;
    gripper_joy gripper;
    pan_tilt_joy head;
};


#endif //ARMADILLO2_TELEOP_JOY_PROFILE_H
