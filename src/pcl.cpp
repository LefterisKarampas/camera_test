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

typedef struct{
	int r;
	int g;
	int b;
}colors;

bool flag = false;
string * tf_frames;
int size = 0;


void cloud_cb (const pcl::PCLPointCloud2ConstPtr& input){
	pcl::PCLPointCloud2::Ptr cloud_pass(new pcl::PCLPointCloud2);
	ros::NodeHandle nh;
	ros::Publisher publ;
	publ = nh.advertise<camera_test::coke> ("colors", 1);
	if(flag){
		pcl::PassThrough<pcl::PCLPointCloud2> pass_through_filter;
		tf::TransformListener listener;
		//ros::Rate rate(30.0);
		std::vector<std::string> msg;
		int color_size = 0;
		int max_red = 0,max_brown = 0;
		colors array[size];
		std::vector<tf::StampedTransform> v;
		for(int i =0;i<size;i++){
			tf::StampedTransform transform;
			try{
				listener.waitForTransform(input->header.frame_id,tf_frames[i], ros::Time(0),ros::Duration(3.0));
				listener.lookupTransform(input->header.frame_id,tf_frames[i],ros::Time(), transform);
				v.push_back(transform);
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
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudXYZRGB(new pcl::PointCloud<pcl::PointXYZRGB>);
		   		pcl::fromPCLPointCloud2(*cloud_pass,*PointCloudXYZRGB);
				int count = 0;
				int r1 = 0,g1 =0,b1 = 0;
				int cloudsize = (PointCloudXYZRGB->width) * (PointCloudXYZRGB->height);
				for (int j=0; j< cloudsize; j++){
					//std::cout << "(x,y,z,rgb) = " << PointCloudXYZRGB->points[i] << std::endl;
					uint32_t rgb = *reinterpret_cast<int*>(&PointCloudXYZRGB->points[j].rgb);
					uint8_t r = (rgb >> 16) & 0x0000ff;
					uint8_t g = (rgb >> 8)  & 0x0000ff;
					uint8_t b = (rgb)       & 0x0000ff;
					r1+= r;
					g1+= g;
					b1+= b;
					count++;
				}
				if(count > 0){
					r1 = r1/count;
					g1 = g1/count;
					b1 = b1/count;
					ROS_INFO("Frame: %s -> R G B:%d %d %d",tf_frames[i].c_str(),r1,g1,b1);
					array[i].r = r1;
					array[i].g = g1;
					array[i].b = b1;
					if(i == 0){
						max_red = i;
						max_brown = i;
					}
					else if(array[max_red].r < r1){
						max_red = i;
					}
					else{
						max_brown = i;
					}
				}
			}
			catch (tf::TransformException ex){
				ROS_ERROR("%s",ex.what());
				ros::Duration(1.0).sleep();
			}
		}
		tf::TransformBroadcaster br;
		br.sendTransform(tf::StampedTransform( v[max_red], ros::Time::now(), input->header.frame_id, "red"));
		msg.push_back("red");
		color_size++;
		tf::TransformBroadcaster br1;
		br1.sendTransform(tf::StampedTransform(v[max_brown],ros::Time::now(), input->header.frame_id, "brown"));
		msg.push_back("brown");
		color_size++;
	
		camera_test::coke color;
		color.tf_names = msg;
		color.size = color_size;
		publ.publish(color);
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
	ros::Subscriber sub = nh.subscribe ("camera/depth_registered/points", 100, cloud_cb);
	ros::spin();
}


int main (int argc, char** argv)
{
	// Initialize ROS
	ros::init (argc, argv, "my_pcl_tutorial");
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe ("coke_tf", 1, frames);
	// Spin
	ros::spin ();
} 
