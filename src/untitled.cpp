#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <math.h>
#include <iostream>

using namespace std;



int main(int argc, char** argv){
  ros::init(argc, argv, "tf_astra_camera1");
  ros::NodeHandle node;
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(0, 0, 0.0) );
  tf::Quaternion q;
  q.setRPY(-(M_PI/2), 0,-(M_PI/2));
  transform.setRotation(q);
  cout << transform.inverse().getOrigin().x() << " " << transform.inverse().getOrigin().y() << " " << transform.inverse().getOrigin().z() << endl;
  tf::Matrix3x3 m(transform.inverse().getRotation());
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  cout << roll << " " << pitch << " " << yaw << endl;
  
  return 0;
};
