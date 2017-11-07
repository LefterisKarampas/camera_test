#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <math.h>



int main(int argc, char** argv){
  ros::init(argc, argv, "tf_astra_camera1");
  ros::NodeHandle node;
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(0, 0, 0.0) );
  tf::Quaternion q;
  q.setRPY(-(M_PI/2), 0,-(M_PI/2));
  transform.setRotation(q);

  while(1){
    br.sendTransform(tf::StampedTransform(transform.inverse(), ros::Time::now(), "camera_rgb_optical_frame1", "camera_rgb_frame1"));
  }
  return 0;
};
