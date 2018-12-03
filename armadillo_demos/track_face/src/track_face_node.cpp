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

#include <face_detector.h>
#include <pantilt_tracker.h>
#include <base_tracker.h>
#include <ros/ros.h>

#define MIN_DETECTIONS 2
#define NO_DETECTIONS 30

int main(int argc, char** argv)
{
    ros::init(argc, argv, "face_detector_node");
    ros::NodeHandle nh("~");

    OpMode operation_mode;
    int op_mode = 1;

    std::string img_topic;
    nh.getParam("image", img_topic);
    nh.getParam("mode", op_mode);

    operation_mode = (OpMode)op_mode;

    FaceDetector face_detector(nh, img_topic);
    BaseTracker base_tracker(nh);
    PanTiltTracker pantilt_tracker(nh);


    pantilt_tracker.move(0, 0);

    int following_detections = 0;
    int following_no_detecions = 0;

    while(ros::ok())
    {
        CvPoint face_xy;
        cv::Rect frame;
        bool detected = face_detector.detectAndDisplay(face_xy, frame);

        //ROS_INFO("x -> %i,\ty -> %i,\tw -> %i,\th -> %i", face_xy.x, face_xy.y, frame.width, frame.height);

        if (detected)
        {
            following_detections++;
            following_no_detecions = 0;
            if (following_detections >= MIN_DETECTIONS)
            {
                following_detections = 0;
                pantilt_tracker.trackFace(face_xy, frame);
                base_tracker.track(operation_mode, face_xy, frame);
            }
        }
        else
        {
            base_tracker.stop();
            following_detections = 0;
            following_no_detecions++;
            if (following_no_detecions >= NO_DETECTIONS)
            {
                following_no_detecions = 0;
                pantilt_tracker.move(0, 0);
            }
        }

        int c = cv::waitKey(10);
        if( (char)c == 'c' )
            break;

        ros::spinOnce();
    }

    return 0;
}