
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
