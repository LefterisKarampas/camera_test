#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <math.h>



int main(int argc, char** argv){
  ros::init(argc, argv, "tf_astra_camera2");
  ros::NodeHandle node;

  static tf::TransformBroadcaster br1;
  tf::Transform transform1;
  transform1.setOrigin( tf::Vector3(0,-0.045,0));
  tf::Quaternion q1;
  q1.setRPY(0,0,0);
  transform1.setRotation(q1);
  while(1){
    br1.sendTransform(tf::StampedTransform(transform1.inverse(), ros::Time::now(), "camera_rgb_frame1", "camera_link"));
  }
  return 0;
};
