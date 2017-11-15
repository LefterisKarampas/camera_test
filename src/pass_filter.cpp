#include <ros/ros.h>
#include <iostream>
// PCL specific includes
#include <tf/transform_broadcaster.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <tf/transform_listener.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <camera_test/coke.h>
#include <sensor_msgs/PointCloud2.h>
#include <vector>

using namespace std;
using namespace sensor_msgs;
using namespace message_filters;
ros::Publisher pub;

bool flag = false;
string * tf_frames;
int size = 0;


void cloud_cb (const pcl::PCLPointCloud2ConstPtr& input){
	pcl::PCLPointCloud2::Ptr cloud_pass(new pcl::PCLPointCloud2);
	ros::NodeHandle nh;
	ros::Publisher publ;
	publ = nh.advertise<sensor_msgs::PointCloud2> ("object_cloud", 1);
	if(flag){
		pcl::PassThrough<pcl::PCLPointCloud2> pass_through_filter;
		tf::TransformListener listener;
		ros::Rate rate(10.0);
		tf::StampedTransform transform;
		std::vector<std::string> msg;
		int color_size = 0;
		for(int i =0;i<size;i++){
			try{
				listener.waitForTransform(input->header.frame_id,tf_frames[i], ros::Time(0),ros::Duration(3.0));
				listener.lookupTransform(input->header.frame_id,tf_frames[i],ros::Time(), transform);
				// Y 
			  	/*pass_through_filter.setInputCloud (input);
			  	pass_through_filter.setFilterFieldName ("y");
			  	pass_through_filter.setFilterLimits (transform.getOrigin().y()-0.15,transform.getOrigin().y()+0.2);
				pass_through_filter.filter (*cloud_pass);*/
				// X 
			  	pass_through_filter.setInputCloud (input);
			  	pass_through_filter.setFilterFieldName ("x");
			  	pass_through_filter.setFilterLimits (transform.getOrigin().x(),transform.getOrigin().x()+0.05);
			  	pass_through_filter.filter (*cloud_pass);
				// Z
				pass_through_filter.setInputCloud (cloud_pass);
				pass_through_filter.setFilterFieldName ("z");
			  	pass_through_filter.setFilterLimits (transform.getOrigin().z()-0.03,transform.getOrigin().z());
			  	pass_through_filter.filter (*cloud_pass);
				pub.publish(*cloud_pass);
			}
			catch (tf::TransformException ex){
				ROS_ERROR("%s",ex.what());
				ros::Duration(1.0).sleep();
			}
		}
		publ.publish(*cloud_pass);
	}
}

void frames(const camera_test::coke & frame){
	flag = true;
	ros::NodeHandle nh;
	tf_frames = new string [frame.size];
	size = frame.size;
	for(int i=0;i<frame.size;i++){
		tf_frames[i] = frame.tf_names[i];
	}
	ros::Subscriber sub = nh.subscribe ("camera/depth_registered/points", 1, cloud_cb);
	ros::spin();
}


int main (int argc, char** argv)
{
	// Initialize ROS
	ros::init (argc, argv, "my_pcl_tutorial");
	ros::NodeHandle nh;
	// Create a ROS publisher for the output point cloud
	pub = nh.advertise<sensor_msgs::PointCloud2> ("object_cloud", 1);
	// Create a ROS subscriber for the input point cloud
	
	ros::Subscriber sub = nh.subscribe ("coke_tf", 1, frames);
	// Spin
	ros::spin ();
} 
