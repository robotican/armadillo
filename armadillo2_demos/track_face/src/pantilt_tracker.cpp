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

#include <opencv-3.3.1/opencv2/core.hpp>

#include <pantilt_tracker.h>

PanTiltTracker::PanTiltTracker(ros::NodeHandle &nh)
{
    nh_ = &nh;
    traj_pub_ = nh_->advertise<trajectory_msgs::JointTrajectory>("/pan_tilt_trajectory_controller/command", 5);
}

void PanTiltTracker::trackFace(const CvPoint &face, const cv::Rect& frame)
{
    double pan_min = M_PI / 4;//rad
    double pan_max = -M_PI / 4;//rad
    double tilt_min = -M_PI / 6;//rad
    double tilt_max = (50 * M_PI) / 180;//rad

    double pan_goal = FaceDetector::map(0, frame.width, pan_min, pan_max, face.x);
    double tilt_goal = FaceDetector::map(0, frame.height, tilt_min, tilt_max, face.y);

    move(pan_goal, tilt_goal * P_GAIN);
}

void PanTiltTracker::move(double pan, double tilt)
{
    trajectory_msgs::JointTrajectory traj;
    traj.header.stamp = ros::Time::now();
    traj.joint_names.push_back("head_pan_joint");
    traj.joint_names.push_back("head_tilt_joint");
    traj.points.resize(1);
    traj.points[0].time_from_start = ros::Duration(1.0);
    std::vector<double> q_goal(2);

    //ROS_INFO("pan: %f, tilt: %f", pan_goal*180/M_PI, tilt_goal*180/M_PI);

    q_goal[0] = pan;
    q_goal[1] = tilt;
    traj.points[0].positions=q_goal;
    traj.points[0].velocities.push_back(0.1);
    traj.points[0].velocities.push_back(0.1);
    traj_pub_.publish(traj);
}

