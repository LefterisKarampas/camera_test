#include <ros/ros.h>
#include <iostream>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <tf/transform_listener.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

using namespace std;
ros::Publisher pub;

void cloud_cb (const pcl::PCLPointCloud2ConstPtr& input){

	pcl::PCLPointCloud2::Ptr cloud_pass(new pcl::PCLPointCloud2);

	pcl::PassThrough<pcl::PCLPointCloud2> pass_through_filter;
	tf::TransformListener listener;
	ros::Rate rate(10.0);
	tf::StampedTransform transform;
	std::string temp = "/"+input->header.frame_id;
	try{
		listener.waitForTransform(input->header.frame_id,"/coke1", ros::Time(0),ros::Duration(3.0));
		listener.lookupTransform(input->header.frame_id,"/coke1",ros::Time(), transform);
		// Z
		pass_through_filter.setInputCloud (input);
		pass_through_filter.setFilterFieldName ("z");
	  	pass_through_filter.setFilterLimits (transform.getOrigin().z()-0.05,transform.getOrigin().z()+0.05);
	  	pass_through_filter.filter (*cloud_pass);
	  	// X 
	  	pass_through_filter.setInputCloud (cloud_pass);
	  	pass_through_filter.setFilterFieldName ("x");
	  	pass_through_filter.setFilterLimits (transform.getOrigin().x()-0.05,transform.getOrigin().x()+0.05);
	  	pass_through_filter.filter (*cloud_pass);
	  	// Y 
	  	pass_through_filter.setInputCloud (cloud_pass);
	  	pass_through_filter.setFilterFieldName ("y");
	  	pass_through_filter.setFilterLimits (transform.getOrigin().y()-0.05,transform.getOrigin().y()+0.05);
		pass_through_filter.filter (*cloud_pass);
		pub.publish(*cloud_pass);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudXYZRGB(new pcl::PointCloud<pcl::PointXYZRGB>);
   		pcl::fromPCLPointCloud2(*cloud_pass,*PointCloudXYZRGB);
		int count = 0;
		int r1 = 0,g1 =0,b1 = 0;
		int cloudsize = (PointCloudXYZRGB->width) * (PointCloudXYZRGB->height);
		for (int i=0; i< cloudsize; i++){
			//std::cout << "(x,y,z,rgb) = " << PointCloudXYZRGB->points[i] << std::endl;
			uint32_t rgb = *reinterpret_cast<int*>(&PointCloudXYZRGB->points[i].rgb);
			uint8_t r = (rgb >> 16) & 0x0000ff;
			uint8_t g = (rgb >> 8)  & 0x0000ff;
			uint8_t b = (rgb)       & 0x0000ff;
			r1+= r;
			g1+= g;
			b1+= b;
			count++;
		}
		ROS_INFO("R G B:%d %d %d",r1/count,g1/count,b1/count);
		if(r1/count > 180){
			ROS_INFO("RED");
		}
		else if(b1/count > 160){
			ROS_INFO("BLUE");
		}
	}
	catch (tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
		ros::Duration(1.0).sleep();
	}
	//pass through filter
	printf("%d - %d\n",cloud_pass->data.size(),input->data.size());
}



int main (int argc, char** argv)
{
	// Initialize ROS
	ros::init (argc, argv, "my_pcl_tutorial");
	ros::NodeHandle nh;
	// Create a ROS publisher for the output point cloud
	pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
	// Create a ROS subscriber for the input point cloud
	ros::Subscriber sub = nh.subscribe ("camera/depth_registered/points", 1, cloud_cb);

	// Create a ROS publisher for the output point cloud

	// Spin
	ros::spin ();
} 
