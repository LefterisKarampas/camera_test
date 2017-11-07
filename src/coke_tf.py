 
import roslib
import rospy
import math
import tf
import geometry_msgs.msg

if __name__ == '__main__':
    rospy.init_node('coke_tf')
    listener = tf.TransformListener()
    coke_tf = rospy.Publisher('coke/pose', geometry_msgs.msg.Pose,queue_size=1)
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/coke1', '/camera_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        cmd = geometry_msgs.msg.Pose()
        cmd.position.x = trans[0]
        cmd.position.y = trans[1]
        cmd.position.z = trans[2]
        cmd.orientation.x = rot[0]
        cmd.orientation.y = rot[1]
        cmd.orientation.z = rot[2]
        cmd.orientation.w = rot[3]
        quaternion = (rot[0],
                    rot[1],
                    rot[2],
                    rot[3])
        euler = tf.transformations.euler_from_quaternion(quaternion)
        roll = euler[0]
        pitch = euler[1]
        yaw = euler[2]
        print(roll, pitch, yaw);
        coke_tf.publish(cmd)
        rate.sleep()