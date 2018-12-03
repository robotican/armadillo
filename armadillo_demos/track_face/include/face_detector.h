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


#ifndef TRACK_FACE_FACE_DETECTOR_H
#define TRACK_FACE_FACE_DETECTOR_H

#include <opencv-3.3.1/opencv2/core.hpp>
#include <opencv-3.3.1/opencv2/highgui.hpp>
#include <opencv-3.3.1/opencv2/imgproc.hpp>
#include <opencv-3.3.1/opencv2/objdetect.hpp>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#define FACE_DATA_PATH "/home/armadillo2/catkin_ws/src/armadillo2/armadillo2_demos/track_face/data/haarcascade_frontalface_alt.xml"
#define EYES_DATA_PATH "/home/armadillo2/catkin_ws/src/armadillo2/armadillo2_demos/track_face/data/haarcascade_eye_tree_eyeglasses.xml"
#define DETECTION_WINDOW_NAME "Face Detection"

struct FaceLocation
{
    int x = 0;
    int y = 0;
};
class FaceDetector
{

private:
    CvCapture* capture_;
    cv::CascadeClassifier face_cascade;
    cv::CascadeClassifier eyes_cascade;
    cv::VideoCapture cap_;
    cv::Mat frame_;

    ros::NodeHandle *nh_;
    ros::Subscriber img_sub_;

    void onIncomingImage(const sensor_msgs::Image &msg);

public:

    FaceDetector(ros::NodeHandle& nh, std::string image_topic);
    bool detectAndDisplay(CvPoint&, cv::Rect&);
    static double map(double input_start,
                      double input_end,
                      double output_start,
                      double output_end,
                      double input);

};


#endif //TRACK_FACE_FACE_DETECTOR_H
