
#include <face_detector.h>
#include <pantilt_tracker.h>
#include <base_tracker.h>
#include <ros/ros.h>

FaceDetector face_detector;
CvPoint face_xy;
cv::Rect frame;

void onIncomingImage(const sensor_msgs::Image &msg)
{
    cv_bridge::CvImagePtr img = cv_bridge::toCvCopy(msg);

    face_detector.detectAndDisplay(face_xy, frame, img->image);

    int c = cv::waitKey(1);
    if( (char)c == 'c' )
        exit(EXIT_SUCCESS);

    ROS_WARN("IMAGE");
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "face_detector_node");
    ros::NodeHandle nh;
    ros::Subscriber image_sub_ = nh.subscribe("/kinect2/qhd/image_color_rect", 10, onIncomingImage);

    FaceDetector face_detector();
    BaseTracker base_tracker(nh);
    PanTiltTracker pantilt_tracker(nh);

    //TODO: GET OP MODE AS PARAM FROM LAUNCH FILE
    OpMode operation_mode = OpMode::PAN_ROTATE_DRIVE;

    /*while(ros::ok())
    {
        CvPoint face_xy;
        cv::Rect frame;
        bool detected = face_detector.detectAndDisplay(face_xy, frame);

        //ROS_INFO("x -> %i,\ty -> %i,\tw -> %i,\th -> %i", face_xy.x, face_xy.y, frame.width, frame.height);

        if (detected)
        {
            pantilt_tracker.trackFace(face_xy, frame);
            base_tracker.track(operation_mode, face_xy, frame);
        }

        int c = cv::waitKey(10);
        if( (char)c == 'c' )
            break;

        ros::spinOnce();
    }*/
    ros::spin();

    return 0;
}