
/*
 * blobdetection.cpp
 *
 *  Created on: 23/03/2017
 *      Author: zubair khan
 */
 

#include <ros/ros.h>
#include <stdlib.h> 
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <librealsense/rs.hpp>
#include <librealsense/rscore.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl/conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <boost/foreach.hpp>


namespace enc = sensor_msgs::image_encodings;
 using namespace std;

static const char WINDOW[] = "Image Processed";
static const char RESULT[] = "Tracking";
 
sensor_msgs::PointCloud2 my_pcl;

//Use method of ImageTransport to create image publisher
image_transport::Publisher pub;
bool hasNewPcl = false;

int LowerH = 170;
int LowerS = 150;
int LowerV = 60;
int UpperH = 179;
int UpperS = 255;
int UpperV = 255;

int posX;
int posY;



    ros::NodeHandle *n;
    
   sensor_msgs::PointCloud2 depth;
    pcl::PointCloud < pcl::PointXYZ > pcl_cloud;

   //  typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
     


void depthcallback (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
	my_pcl = *cloud_msg;
	hasNewPcl = true;
	cout<<"here"<<endl;
}

void getXYZ(int x, int y)
{
	int arrayPosition = y*my_pcl.row_step + x*my_pcl.point_step;
	int arrayPosX = arrayPosition + my_pcl.fields[0].offset; // X has an offset of 0
    int arrayPosY = arrayPosition + my_pcl.fields[1].offset; // Y has an offset of 4
    int arrayPosZ = arrayPosition + my_pcl.fields[2].offset; // Z has an offset of 8

    float X ;
   float Y ;
    float Z ;

    memcpy(&X, &my_pcl.data[arrayPosX], sizeof(float));
    memcpy(&Y, &my_pcl.data[arrayPosY], sizeof(float));
    memcpy(&Z, &my_pcl.data[arrayPosZ], sizeof(float));

geometry_msgs::Point p;
     // put data into the point p
    p.x = X;
    p.y = Y;
    p.z = Z;
    cout<<"Pt "<<p.z<<endl;
}

void blobDetectionCallback(const sensor_msgs::ImageConstPtr& original_image)
{
    //Convert from the ROS image message to a CvImage suitable for working with OpenCV for processing
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        //Always copy, returning a mutable CvImage
        //OpenCV expects color images to use BGR channel order.
        cv_ptr = cv_bridge::toCvCopy(original_image, enc::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        //if there is an error during conversion, display it
        ROS_ERROR("tutorialROSOpenCV::main.cpp::cv_bridge exception: %s", e.what());
        return;
    }
	cv::Mat img_mask,img_hsv; 
	cv::cvtColor(cv_ptr->image,img_hsv,CV_BGR2HSV);
	cv::inRange(img_hsv,cv::Scalar(LowerH,LowerS,LowerV),cv::Scalar(UpperH,UpperS,UpperV),img_mask); 
	
	
	cv::erode(img_mask, img_mask, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
    cv::erode(img_mask,img_mask, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
    cv::dilate(img_mask, img_mask, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) ); 
    
    //Calculate the moments of the thresholded image
 	cv:: Moments oMoments = moments(img_mask);
 			 
 	double dM01 = oMoments.m01;
 	double dM10 = oMoments.m10;
 	double dArea = oMoments.m00;
 			 
 	posX = dM10 / dArea;
 	posY = dM01 / dArea;
 				 
 	std::cout<<"posx:"<<posX<<"posy:"<<posY<<std::endl;   

	cv::circle(img_mask, cv::Point(posX, posY), 10, cv::Scalar(0,255,255), 2);
  
    cv::Mat Points;
	cv::findNonZero(img_mask,Points);
	cv::Rect Min_Rect=boundingRect(Points);

	cv::rectangle(img_mask,Min_Rect.tl(),Min_Rect.br(),cv::Scalar(255,0,0),1);
	
    //Display the image using OpenCV
    cv::imshow(RESULT, img_mask);
    
    
  
    //Convert the CvImage to a ROS image message and publish it on the "camera/image_processed" topic.
    pub.publish(cv_ptr->toImageMsg());
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "image_processor");
  
    ros::NodeHandle nh;
    //Create an ImageTransport instance, initializing it with our NodeHandle.
    image_transport::ImageTransport it(nh);

	
	cv::namedWindow("blob");
	cv::createTrackbar("LowerH","blob",&LowerH,179,NULL);
	cv::createTrackbar("UpperH","blob",&UpperH,179,NULL);
	cv::createTrackbar("LowerS","blob",&LowerS,255,NULL);
	cv::createTrackbar("UpperS","blob",&UpperS,255,NULL);
	cv::createTrackbar("LowerV","blob",&LowerV,255,NULL);
	cv::createTrackbar("UpperV","blob",&UpperV,255,NULL);
	
  //OpenCV HighGUI call to create a display window on start-up.
    cv::namedWindow(WINDOW, CV_WINDOW_AUTOSIZE);
    cv::namedWindow(RESULT, CV_WINDOW_AUTOSIZE);
    
    ros::Subscriber dep;
     dep = nh.subscribe ("/camera/depth_registered/points", 1, depthcallback);
   
   //image_transport::Subscriber depth = it.subscribe("/camera/depth/points",1,depthcallback);
    image_transport::Subscriber sub = it.subscribe("/camera/rgb/image_rect_color", 1, blobDetectionCallback);
    
    
    //OpenCV HighGUI call to destroy a display window on shut-down.
    //cv::destroyWindow(WINDOW);
    //cv::destroyWindow(RESULT);
   
    pub = it.advertise("camera/image_processed", 1);
	ros::Rate rate(10.0);
	while(nh.ok())
	{
		if(hasNewPcl)
		{
			getXYZ(posX, posY);
			hasNewPcl = false;
		}
		ros::spinOnce();
		rate.sleep();
	}
	
}