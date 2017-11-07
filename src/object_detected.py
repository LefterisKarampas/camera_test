#!/usr/bin/env python
import rospy
import roslib
from object_recognition_msgs.msg import RecognizedObjectArray
from std_msgs.msg import String
import tf as tf


def callback(data):
    i = 0
    while(i < len(data.objects)):
        print(data.objects[i].pose.pose.pose)
        br = tf.TransformBroadcaster()
        br.sendTransform((data.objects[i].pose.pose.pose.position.x,
                            data.objects[i].pose.pose.pose.position.y,
                            data.objects[i].pose.pose.pose.position.z),
                        (data.objects[i].pose.pose.pose.orientation.x,
                        data.objects[i].pose.pose.pose.orientation.y,
                        data.objects[i].pose.pose.pose.orientation.z,
                        data.objects[i].pose.pose.pose.orientation.w),
                        rospy.Time.now(),
                        "/coke"+str(i+1),"/camera_rgb_optical_frame")
        i = i+1

def listener():
    rospy.init_node('object_detected', anonymous=True)
    rospy.Subscriber("recognized_object_array", RecognizedObjectArray, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()