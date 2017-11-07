 
import roslib
import rospy
import math
import tf
import geometry_msgs.msg

if __name__ == '__main__':
    rospy.init_node('coke_tf')
    listener = tf.TransformListener()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/ar_marker_5', '/camera_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        print(trans)
        print(rot)
        t = tf.Transformer(rot,trans)
        br = tf.TransformBroadcaster()
        br.sendTransform((trans,rot).inverse(),rospy.Time.now(),"w","ar_marker_5")