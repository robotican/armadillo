
#ifndef TRACK_FACE_FACE_DETECTOR_H
#define TRACK_FACE_FACE_DETECTOR_H

#include <opencv-3.3.1/opencv2/core.hpp>
#include <opencv-3.3.1/opencv2/highgui.hpp>
#include <opencv-3.3.1/opencv2/imgproc.hpp>
#include <opencv-3.3.1/opencv2/objdetect.hpp>
#include <ros/ros.h>

#define FACE_DATA_PATH "/home/sub/Desktop/drive2person/data/haarcascade_frontalface_alt.xml"
#define EYES_DATA_PATH "/home/sub/Desktop/drive2person/data/haarcascade_eye_tree_eyeglasses.xml"
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
    cv::Mat frame_;
    cv::CascadeClassifier face_cascade;
    cv::CascadeClassifier eyes_cascade;
    cv::VideoCapture cap_;

public:

    FaceDetector();
    bool detectAndDisplay(CvPoint&, cv::Rect&);

};


#endif //TRACK_FACE_FACE_DETECTOR_H
