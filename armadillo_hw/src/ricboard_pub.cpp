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

#include "ricboard_pub.h"

RicboardPub::RicboardPub(ros::NodeHandle &nh)
{
    nh_ = &nh;

    /* get ric params */
    ros::param::get("~load_ric_hw", load_ric_hw_);

    if (load_ric_hw_)
    {
        if (!ros::param::get(RIC_PORT_PARAM, ric_port_))
        {
            ROS_ERROR("[armadillo_hw/ricboard_pub]: %s param is missing on param server. make sure that you load this param exist in ricboard_config.yaml "
                              "and that your launch includes this param file. shutting down...", RIC_PORT_PARAM);
            ros::shutdown();
            exit (EXIT_FAILURE);
        }

        if (!ros::param::get(TORSO_JOINT_PARAM, torso_.joint_name))
        {
            ROS_ERROR("[armadillo_hw/ricboard_pub]: %s param is missing on param server. make sure that this param exist in controllers.yaml "
                              "and that your launch includes this param file. shutting down...", TORSO_JOINT_PARAM);
            ros::shutdown();
            exit (EXIT_FAILURE);
        }

        try{
            ric_.connect(ric_port_);
            ROS_INFO("[armadillo_hw/ricboard_pub]: ricboard port opened successfully \nport name: %s \nbaudrate: 115200", ric_port_.c_str());
        }catch (ric::ConnectionExeption e) {
            ROS_ERROR("[armadillo_hw/ricboard_pub]: can't open ricboard port. make sure that ricboard is connected. shutting down...");
            ros::shutdown();
            exit(1);
        }

        /* torso reading low pass filter */
        // torso_lpf_.setCutOffFrequency(0.3);
        // torso_lpf_.setDeltaTime(0.1);

        /* ric publishers */
        ric_gps_pub_ = nh.advertise<sensor_msgs::NavSatFix>("gps/fix", 10);
        ric_ultrasonic_pub_ = nh.advertise<sensor_msgs::Range>("urf/front", 10);
        ric_imu_pub_ = nh.advertise<sensor_msgs::Imu>("imu/data", 10);
        ric_mag_pub_ = nh.advertise<sensor_msgs::MagneticField>("imu/magnetic", 10);

        ric_pub_timer_ = nh.createTimer(ros::Duration(RIC_PUB_INTERVAL), &RicboardPub::pubTimerCB, this);
        ric_dead_timer_ = nh.createTimer(ros::Duration(RIC_DEAD_TIMEOUT), &RicboardPub::ricDeadTimerCB, this);
        ROS_INFO("[armadillo_hw/ricboard_pub]: ricboard is up");
        espeak_pub_ = nh.advertise<std_msgs::String>("/espeak_node/speak_line", 10);
        /*speakMsg("rik board is up", 1); */
    }
    else
        ROS_WARN("[armadillo_hw/ricboard_pub]: ric hardware is disabled");
}

void RicboardPub::startLoop()
{
    if (!load_ric_hw_)
        return;
    t = new boost::thread(boost::bind(&RicboardPub::loop, this));
}

void RicboardPub::stopLoop()
{
    if (!load_ric_hw_)
        return;
    t->interrupt();
}

void RicboardPub::loop()
{
    if (!load_ric_hw_)
        return;
    while (ros::ok() && !t->interruption_requested())
    {
        ric_.loop();
        if (ric_.isBoardAlive())
        {
            ric::protocol::error err_msg;
            std::string logger_msg;
            int32_t logger_val;
            ric_disconnections_counter_ = 0;
            ric_dead_timer_.stop();
            ric_pub_timer_.start();

            /* if emergecy pin disconnected, shutdown. ric will also kill torso */
            if (ric_.getSensorsState().emrgcy_alarm.is_on)
            {
                speakMsg("emergency, shutting down", 1);
                ROS_ERROR("[armadillo_hw/ricboard_pub]: EMERGENCY PIN DISCONNECTED, shutting down...");
                ros::shutdown();
                exit(EXIT_FAILURE);
            }
            if (ric_.readLoggerMsg(logger_msg, logger_val))
                ROS_INFO("[armadillo_hw/ricboard_pub]: ric logger is saying: '%s', value: %i", logger_msg.c_str(), logger_val);
            if (ric_.readErrorMsg(err_msg))
            {
                std::string comp_name = ric::RicInterface::compType2String((ric::protocol::Type)err_msg.comp_type);
                std::string err_desc = ric::RicInterface::errCode2String((ric::protocol::ErrCode)err_msg.code);
                if (err_msg.code != (uint8_t)ric::protocol::ErrCode::CALIB)
                {
                    ROS_ERROR("[armadillo_hw/ricboard_pub]: ric detected critical '%s' error in %s. shutting down...",
                              err_desc.c_str(), comp_name.c_str());
                    ros::shutdown();
                    exit(EXIT_FAILURE);
                }
                ROS_WARN("[armadillo_hw/ricboard_pub]: ric detected '%s' warning in %s",
                         err_desc.c_str(), comp_name.c_str());
            }
        }
        else
        {
            ric_dead_timer_.start();
            ric_pub_timer_.stop();
        }
    }
}

void RicboardPub::ricDeadTimerCB(const ros::TimerEvent &event)
{
    if (!load_ric_hw_)
        return;
    ric_disconnections_counter_++;
    if (ric_disconnections_counter_ >= MAX_RIC_DISCONNECTIONS)
    {
        speakMsg("rik board disconnected, shutting down", 1);
        ROS_ERROR("[armadillo_hw/ricboard_pub]: ricboard disconnected. shutting down...");
        ros::shutdown();
        exit(EXIT_FAILURE);
    }
}

void RicboardPub::pubTimerCB(const ros::TimerEvent &event)
{
    if (!load_ric_hw_ || !ric_.isBoardAlive())
        return;

    ric::sensors_state sensors = ric_.getSensorsState();

    /* publish ultrasonic */
    sensor_msgs::Range range_msg;
    range_msg.header.stamp = ros::Time::now();
    range_msg.header.frame_id = "front_urf_link";
    range_msg.min_range = 0.3;
    range_msg.max_range = 3.0;
    range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
    range_msg.range = sensors.ultrasonic.distance_mm / 1000.0;
    range_msg.field_of_view = 0.7f;
    ric_ultrasonic_pub_.publish(range_msg);

    /* publish imu */
    sensor_msgs::Imu imu_msg;
    imu_msg.header.stamp = ros::Time::now();
    imu_msg.header.frame_id = "base_link";

    double roll, pitch, yaw;
    pitch = -sensors.imu.roll_rad;
    roll = -sensors.imu.pitch_rad;
    yaw = sensors.imu.yaw_rad - M_PI / 2;

    //wrap to PI
    if (yaw > M_PI )
        yaw -= 2 * M_PI;
    else if (yaw < -M_PI)
        yaw += 2 * M_PI;

    /*ROS_INFO("ROLL %f, PITCH %f, YAW %f", roll * 180 / M_PI,
             pitch * 180 / M_PI,
             yaw * 180 / M_PI);*/

    tf::Quaternion orientation_q = tf::createQuaternionFromRPY(roll,
                                                               pitch,
                                                               yaw);

    imu_msg.orientation.x = orientation_q.x();
    imu_msg.orientation.y = orientation_q.y();
    imu_msg.orientation.z = orientation_q.z();
    imu_msg.orientation.w = orientation_q.w();
    imu_msg.angular_velocity.x = -1 * sensors.imu.gyro_y_rad;
    imu_msg.angular_velocity.y = -1 * sensors.imu.gyro_x_rad;
    imu_msg.angular_velocity.z = -1 * sensors.imu.gyro_z_rad;
    imu_msg.linear_acceleration.x = sensors.imu.accl_y_rad * G_FORCE;
    imu_msg.linear_acceleration.y = sensors.imu.accl_x_rad * G_FORCE;
    imu_msg.linear_acceleration.z = sensors.imu.accl_z_rad * G_FORCE;
    ric_imu_pub_.publish(imu_msg);

    sensor_msgs::MagneticField mag_msg;
    mag_msg.header.stamp = ros::Time::now();
    mag_msg.header.frame_id = "base_link";
    mag_msg.magnetic_field.x = sensors.imu.mag_x_rad;
    mag_msg.magnetic_field.y = sensors.imu.mag_y_rad;
    mag_msg.magnetic_field.z = sensors.imu.mag_z_rad;
    ric_mag_pub_.publish(mag_msg);

    /* publish gps if data is available */
    if (sensors.gps.satellites > 0)
    {
        sensor_msgs::NavSatFix gps_msg;
        sensor_msgs::NavSatStatus gps_status;

        gps_status.status = sensor_msgs::NavSatStatus::STATUS_SBAS_FIX;
        gps_status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;

        gps_msg.header.stamp = ros::Time::now();
        gps_msg.header.frame_id = "base_link";
        gps_msg.latitude = sensors.gps.lat;
        gps_msg.longitude = sensors.gps.lon;
        gps_msg.altitude = sensors.gps.alt;
        gps_msg.status = gps_status;

        ric_gps_pub_.publish(gps_msg);
    }
}

void RicboardPub::registerHandles(hardware_interface::JointStateInterface &joint_state_interface,
                                  hardware_interface::EffortJointInterface &effort_interface_)
{
    if (!load_ric_hw_)
        return;

    /* joint state registration */
    joint_state_handles_.push_back(hardware_interface::JointStateHandle (torso_.joint_name,
                                                                         &torso_.pos,
                                                                         &torso_.vel,
                                                                         &torso_.effort));
    joint_state_interface.registerHandle(joint_state_handles_.back());

    /* joint command registration */
    pos_handles_.push_back(hardware_interface::JointHandle (joint_state_interface.getHandle(torso_.joint_name),
                                                            &torso_.command_effort));
    effort_interface_.registerHandle(pos_handles_.back());
}

void RicboardPub::write(const ros::Duration elapsed)
{
    if (!load_ric_hw_ || !ric_.isBoardAlive())
        return;

    if (elapsed >= ros::Duration(RIC_WRITE_INTERVAL))
    {
        ric::protocol::servo torso_pkg;
        /* add 1500 offset because torso limits in */
        /* armadillo xacro are b/w -500 - 500,    */
        /* and ric servo get value b/w 1000-2000   */
        torso_pkg.cmd = torso_.command_effort + SERVO_NEUTRAL;
        //ROS_WARN("torso_.command_effort: %f, RIC CMD: %d ", torso_.command_effort, torso_pkg.cmd);
        ric_.writeCmd(torso_pkg, sizeof(torso_pkg), ric::protocol::Type::SERVO);
    }
}

void RicboardPub::read(const ros::Duration elapsed)
{
    if (!load_ric_hw_ || !ric_.isBoardAlive())
        return;

    /* update robot state according to ric sensor for controller use */
    ric::sensors_state sensors = ric_.getSensorsState();
    double torso_pos = sensors.laser.distance_mm / 1000.0;
    //double lpf_pos = torso_lpf_.update(torso_pos); //apply low pass filter
    //ROS_INFO("real: %f, lpf: %f", torso_pos, lpf_pos);
    torso_.pos = torso_pos;
    torso_.vel = (torso_.pos - torso_.prev_pos) / elapsed.sec;
    torso_.effort = torso_.command_effort;
    torso_.prev_pos = torso_.pos;
}


