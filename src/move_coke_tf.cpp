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
#include <cmath>

using namespace std;
using namespace sensor_msgs;
using namespace message_filters;
ros::Publisher pub;



void frames(const camera_test::coke & frame){

    tf::TransformListener listener;
    ros::Rate rate(10.0);
    tf::StampedTransform transform;
    for(int i =0;i<frame.size;i++){
        try{
            listener.waitForTransform("base_link",frame.tf_names[i],ros::Time(0),ros::Duration(3.0));
            listener.lookupTransform("base_link",frame.tf_names[i],ros::Time(), transform);
            double xp = transform.getOrigin().x();
            double yp = transform.getOrigin().y();
            double r = 3.3;
            double d = sqrt((double)(xp*xp+yp*yp));
            cout << "r/d " << (double)r/d << endl; 
            double theta = asin((double)(r/d));
            double k = cos((double)theta) * d;
            double w = atan((double)yp/xp);
            double g = w - theta;
            double yc = sin((double)g) * k;
            double xc = cos((double)g) * k;
            cout << "d: " << d << endl;
            cout << "theta: " << theta << endl;
            cout << "k: " << k << endl;
            cout << "w: " << w << endl;
            cout << "g: " << g << endl;
            cout << "xp: " << xp << " - xc: " << xc << endl;
            cout << "yp: " << yp << " - yc: " << yc << endl;
            exit(1);
            transform.setOrigin(tf::Vector3(xc,yc,transform.getOrigin().z()));
            tf::TransformBroadcaster br;
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "reverse"));
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }
    }
}


int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "my_pcl_tutorial");
    ros::NodeHandle nh;
    // Create a ROS publisher for the output point cloud
    ros::Subscriber sub = nh.subscribe ("coke_tf", 1, frames);
    // Spin
    ros::spin ();
} 
