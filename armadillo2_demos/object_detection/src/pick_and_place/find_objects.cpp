


#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PointStamped.h>

#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_listener.h>
#include <object_detection/FindObjectDynParamConfig.h>
#include <dynamic_reconfigure/server.h>

#include <tf/transform_broadcaster.h>
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <armadillo2_msgs/SwitchCamTopic.h>

using namespace cv;
using namespace std;
bool debug_vision=false;


bool find_object(Mat input, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,Point3d *obj,std::string frame);
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input);
void dynamicParamCallback(robotican_common::FindObjectDynParamConfig &config, uint32_t level);
void arm_msgCallback(const boost::shared_ptr<const geometry_msgs::PoseStamped>& point_ptr);




tf::TransformListener *listener_ptr;

//bool timeout=true;

int object_id;

//ros::Time detect_t;

std::string depth_topic1,depth_topic2,depth_topic;
bool have_object=false;

ros::Publisher object_pub;
image_transport::Publisher result_image_pub;
image_transport::Publisher object_image_pub;
image_transport::Publisher bw_image_pub;
ros::Publisher pose_pub;
//red
int minH=3,maxH=160;
int minS=70,maxS=255;

int minV=10,maxV=255;
int minA=200,maxA=50000;
int gaussian_ksize=0;
int gaussian_sigma=0;
int morph_size=0;

int inv_H=1;




void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input) {

	//pcl - point clound library with lot of algorithms

    pcl::PointCloud<pcl::PointXYZRGBA> cloud;
    //convert ros point cloud msg to pcl point cloud 
    pcl::fromROSMsg (*input, cloud); 
    //create projection image from p.c.
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudp (new pcl::PointCloud<pcl::PointXYZRGBA> (cloud));

    if (cloudp->empty()) {

        ROS_WARN("empty cloud");
        return;
    }
	//creating new ros sensor msg - picture is relative to depth camera tf
    sensor_msgs::ImagePtr image_msg(new sensor_msgs::Image);
    pcl::toROSMsg (*input, *image_msg);
    image_msg->header.stamp = input->header.stamp;
    image_msg->header.frame_id = input->header.frame_id;
	
	
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
    Mat result=cv_ptr->image;

    Point3d obj;
    //find object
    have_object= find_object(result,cloudp,&obj,input->header.frame_id);

    waitKey(1);



    if (have_object) {

	//publish geometry msg containing object's location
        geometry_msgs::PoseStamped target_pose;
        target_pose.header.frame_id=input->header.frame_id;
        target_pose.header.stamp=ros::Time::now();
        target_pose.pose.position.x =obj.x;
        target_pose.pose.position.y = obj.y;
        target_pose.pose.position.z = obj.z;
        target_pose.pose.orientation.w=1;
        pose_pub.publish(target_pose);
    }


}

void obj_msgCallback(const boost::shared_ptr<const geometry_msgs::PoseStamped>& point_ptr)
{
	//get object location msg
    try
    {
        geometry_msgs::PoseStamped base_object_pose;
        listener_ptr->transformPose("base_footprint", *point_ptr, base_object_pose);
        base_object_pose.pose.orientation= tf::createQuaternionMsgFromRollPitchYaw(0.0,0.0,0.0);

        //ROS_WARN("Z: %f", base_object_pose.pose.position.z);

		//simulate alvar msgs, to get tf
        ar_track_alvar_msgs::AlvarMarkers msg;
        msg.header.stamp=base_object_pose.header.stamp;
        msg.header.frame_id="base_footprint";

        ar_track_alvar_msgs::AlvarMarker m;
        m.id=object_id;
        m.header.stamp=base_object_pose.header.stamp;
        m.header.frame_id="base_footprint";
      m.pose=base_object_pose;
      m.pose.pose.position.z+=0.02;
        msg.markers.push_back(m);

        m.pose.pose.position.z-=0.08;
        m.id=2;
         msg.markers.push_back(m);

        object_pub.publish(msg);


    }
    catch (tf::TransformException &ex)
    {
        printf ("Failure %s\n", ex.what()); //Print exception which was caught
    }
}

bool find_object(Mat input, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudp,Point3d *pr,std::string frame) {

    Mat hsv,filtered,bw,mask;

    cv_bridge::CvImage out_msg;
    out_msg.header.stamp=ros::Time::now();
    out_msg.header.frame_id=  frame;
	//use hsv colores - no light sensitivity
    cvtColor(input,hsv,CV_BGR2HSV);


    if (inv_H) {
		//edges of spectrom - red colore
        Mat lower_hue_range;
        Mat upper_hue_range;
        inRange(hsv, cv::Scalar(0, minS, minV), cv::Scalar(minH, maxS, maxV), lower_hue_range);
        inRange(hsv, cv::Scalar(maxH, minS, minV), cv::Scalar(179, maxS, maxV), upper_hue_range);
        // Combine the above two images

        addWeighted(lower_hue_range, 1.0, upper_hue_range, 1.0, 0.0, mask);
    }
    else{
        //if not red use (middle of spectrom):
        inRange(hsv,Scalar(minH,minS,minV),Scalar(maxH,maxS,maxV),mask);
    }
    hsv.copyTo(filtered,mask);
    cvtColor(filtered,filtered,CV_HSV2BGR);
	
    out_msg.image    = filtered;
    out_msg.encoding = "bgr8";
    object_image_pub.publish(out_msg.toImageMsg());
    //convert to bw image, gaussian - blur, morphologic actions
    mask.copyTo(bw);
    if (gaussian_ksize>0) {
        if (gaussian_ksize % 2 == 0) gaussian_ksize++;
        GaussianBlur( bw, bw, Size(gaussian_ksize,gaussian_ksize), gaussian_sigma , 0);
    }


    if (morph_size>0) {
        Mat element = getStructuringElement( MORPH_ELLIPSE, Size( 2*morph_size + 1, 2*morph_size+1 ), Point( morph_size, morph_size ) );
        morphologyEx( bw, bw, MORPH_CLOSE, element, Point(-1,-1), 1 );
    }

    out_msg.image    = bw;
    out_msg.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
    bw_image_pub.publish(out_msg.toImageMsg());

	//get image contours
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;

    findContours(bw, contours,hierarchy,CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
	
	//get largest contour
    double largest_area=0;
    int largest_contour_index=0;
    for( int i = 0; i< contours.size(); i++ )
    {
        double area0 = abs(contourArea(contours[i]));
        if(area0>largest_area){
            largest_area=area0;
            largest_contour_index=i;
        }
    }
    bool ok=false;
    if ((largest_area>minA)&&(largest_area<maxA)) {

        //draw contours and details about object
        drawContours(input, contours, (int)largest_contour_index,  Scalar(255,0,0), 3, 8, hierarchy, 0);
        Moments mu=moments( contours[largest_contour_index], true );
        Point2f mc = Point2f( mu.m10/mu.m00 , mu.m01/mu.m00 );
        circle( input, mc, 4, Scalar(0,0,255), -1, 8, 0 );
        int pcl_index = ((int)(mc.y)*input.cols) + (int)(mc.x);
        circle( input, mc, 8, Scalar(0,255,0), -1, 8, 0 );

        pr->x=cloudp->points[pcl_index].x;
        pr->y=cloudp->points[pcl_index].y;
        pr->z=cloudp->points[pcl_index].z;
        char str[100];
        if (isnan (pr->x) || isnan (pr->y) || isnan (pr->z) ) {
            sprintf(str,"NaN");
            ok=false;
        }
        else {
            sprintf(str,"[%.3f,%.3f,%.3f] A=%lf",pr->x,pr->y,pr->z,largest_area);
            ok=true;
        }
        putText( input, str, mc, CV_FONT_HERSHEY_COMPLEX, 0.4, Scalar(255,255,255), 1, 8);

    }


    out_msg.image    = input;
    out_msg.encoding = "bgr8";
    result_image_pub.publish(out_msg.toImageMsg());

    return ok;
}

void dynamicParamCallback(robotican_common::FindObjectDynParamConfig &config, uint32_t level) {
    minH = config.H_min;
    maxH = config.H_max;

    minS = config.S_min;
    maxS = config.S_max;

    minV = config.V_min;
    maxV = config.V_max;

    minA = config.A_min;
    maxA = config.A_max;

    gaussian_ksize = config.gaussian_ksize;
    gaussian_sigma = config.gaussian_sigma;

    morph_size = config.morph_size;

    inv_H = config.invert_Hue;
}


void on_trackbar( int, void* ){}

bool switch_pcl_topic(armadillo2_msgs::SwitchCamTopic::Request &req, armadillo2_msgs::SwitchCamTopic::Response &res) {

    if (req.num==1) depth_topic=depth_topic1;
    else if (req.num==2) depth_topic=depth_topic2;
    res.success=true;
return true;
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "find_objects_node");
    ros::NodeHandle n;
    ros::NodeHandle pn("~");
	
    pn.param<int>("object_id", object_id, 1);
    pn.param<std::string>("depth_topic1", depth_topic1, "/kinect2/qhd/points");
    pn.param<std::string>("depth_topic2", depth_topic2, "/kinect2/qhd/points");
    depth_topic=depth_topic1;

    dynamic_reconfigure::Server<robotican_common::FindObjectDynParamConfig> dynamicServer;
    dynamic_reconfigure::Server<robotican_common::FindObjectDynParamConfig>::CallbackType callbackFunction;

    callbackFunction = boost::bind(&dynamicParamCallback, _1, _2);
    dynamicServer.setCallback(callbackFunction);

    image_transport::ImageTransport it_(pn);

    result_image_pub = it_.advertise("result", 1);
    object_image_pub = it_.advertise("hsv_filterd", 1);
    bw_image_pub = it_.advertise("bw", 1);
    ros::Subscriber pcl_sub = n.subscribe(depth_topic, 1, cloud_cb);
    ROS_INFO_STREAM(depth_topic);

    object_pub=n.advertise<ar_track_alvar_msgs::AlvarMarkers>("detected_objects", 2, true);

    pose_pub=pn.advertise<geometry_msgs::PoseStamped>("object_pose",10);

	//convert depth cam tf to base foot print tf (moveit work with base foot print tf)
    tf::TransformListener listener;
    listener_ptr=&listener;

    message_filters::Subscriber<geometry_msgs::PoseStamped> point_sub_;
    point_sub_.subscribe(pn, "object_pose", 10);

    tf::MessageFilter<geometry_msgs::PoseStamped> tf_filter(point_sub_, listener, "base_footprint", 10);
    tf_filter.registerCallback( boost::bind(obj_msgCallback, _1) );




ros::ServiceServer switch_sub = n.advertiseService("switch_pcl_topic", &switch_pcl_topic);

ros::Rate r(10);
    ROS_INFO("Ready to find objects!");
    while (ros::ok()) {
    if (pcl_sub.getTopic()!=depth_topic) {
        pcl_sub = n.subscribe(depth_topic, 1, cloud_cb);
         ROS_INFO("switching pcl topic");
    }
     ros::spinOnce();
     r.sleep();
    }

    return 0;
}

