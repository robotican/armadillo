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

FaceDetector::FaceDetector(ros::NodeHandle& nh, std::string image_topic)
{
    nh_ = &nh;
    if( !face_cascade.load( FACE_DATA_PATH ) || !eyes_cascade.load( EYES_DATA_PATH ))
    {
        ROS_ERROR("[face_detector]: error loading data file");
        exit(EXIT_FAILURE);
    }

    img_sub_ = nh_->subscribe(image_topic, 10, &FaceDetector::onIncomingImage, this);
}

void FaceDetector::onIncomingImage(const sensor_msgs::Image &msg)
{
    cv_bridge::CvImagePtr img = cv_bridge::toCvCopy(msg);
    img->image.copyTo(frame_);
}


bool FaceDetector::detectAndDisplay(CvPoint& face_point, cv::Rect& frame_data)
{
    //cv::Mat frame;
    //cap_ >> frame;
    //cv::Mat frame_/* = cv_img_->image*/;


    if (frame_.empty())
        return false;

    std::vector<cv::Rect> faces, eyes;
    cv::Mat frame_gray;

    cvtColor( frame_, frame_gray, CV_BGR2GRAY );
    equalizeHist( frame_gray, frame_gray );

    /* detect faces */
    face_cascade.detectMultiScale( frame_gray, faces, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, cv::Size(30, 30) );

    float biggest_face_size = 0;
    int biggest_face_indx = 0;

    for (int face_indx = 0; face_indx < faces.size(); face_indx++)
    {
        if (faces[face_indx].empty())
        {
            imshow(DETECTION_WINDOW_NAME, frame_ );
            return false;
        }


        cv::Mat faceROI = frame_gray( faces[face_indx] );

        //-- In each face, detect eyes
        eyes_cascade.detectMultiScale( faceROI, eyes, 1.1, 2, 0 |CV_HAAR_SCALE_IMAGE, cv::Size(30, 30) );

        // draw faces with eyes (reduce false positives)
        if (eyes.size() > 0)
        {
            face_point.x = faces[face_indx].x +  faces[face_indx].width / 2;
            face_point.y = faces[face_indx].y + faces[face_indx].height / 2;
            frame_data.width = frame_.cols;
            frame_data.height = frame_.rows;

            rectangle(frame_, CvPoint(faces[face_indx].x, faces[face_indx].y),
                      CvPoint(faces[face_indx].x + faces[face_indx].width,
                              faces[face_indx].y + faces[face_indx].height), 2, 8, 0 );

            imshow(DETECTION_WINDOW_NAME, frame_ );

            return true;
        }
        // if no eyes were found in faces, maby face is far away, so draw biggest face
        if (faces[face_indx].area() > biggest_face_size)
        {
            biggest_face_size = faces[face_indx].area();
            biggest_face_indx = face_indx;

            if (face_indx == faces.size() - 1)
            {
                face_point.x = faces[biggest_face_indx].x;
                face_point.y = faces[biggest_face_indx].y;
                frame_data.width = frame_.cols;
                frame_data.height = frame_.rows;

                rectangle(frame_, CvPoint(faces[biggest_face_indx].x, faces[biggest_face_indx].y),
                          CvPoint(faces[biggest_face_indx].x + faces[biggest_face_indx].width,
                                  faces[biggest_face_indx].y + faces[biggest_face_indx].height), 2, 8, 0 );

                imshow(DETECTION_WINDOW_NAME, frame_ );
                return true;
            }
        }

        face_indx++;
    }

    imshow(DETECTION_WINDOW_NAME, frame_ );
    return false;

}

double FaceDetector::map(double input_start,
                           double input_end,
                           double output_start,
                           double output_end,
                           double input)
{
    return output_start + ((output_end - output_start) / (input_end - input_start)) * (input - input_start);
}