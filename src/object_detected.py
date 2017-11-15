#!/usr/bin/env python
import rospy
import roslib
from object_recognition_msgs.msg import RecognizedObjectArray
from std_msgs.msg import String
from camera_test.msg import coke
import tf as tf
import math

pub = None

def callback(data):
    i = 0
    tf_cokes = coke()
    x = []
    pub = rospy.Publisher('coke_tf', coke, queue_size=10)
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
        x = x+["coke"+str(i+1)]
        i = i+1
    tf_cokes.tf_names = x
    tf_cokes.size = i;
    pub.publish(tf_cokes)

def listener():
    rospy.init_node('object_detected', anonymous=True)
    rospy.Subscriber("recognized_object_array", RecognizedObjectArray, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()