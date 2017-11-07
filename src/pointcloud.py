import rospy
from std_msgs.msg import String
import numpy as np
from sensor_msgs.msg import PointCloud2
from object_recognition_msgs.msg import RecognizedObjectArray

pub = rospy.Publisher('mine', PointCloud2, queue_size=10)
rate = None
temp = []
	

def callback(data):
	k = len(data.objects)
	#temp = data.objects[0].point_clouds
	#rospy.Subscriber("camera/depth_registered/points", PointCloud2, callback2)
	#rospy.spin()
	for i in range(0,k):
		for j in range(0,len(data.objects[i].point_clouds)):
			data.objects[i].point_clouds[j].header.stamp = rospy.Time()
			data.objects[i].point_clouds[j].header.frame_id = data.header.frame_id
			print(data.objects[i].point_clouds[j].fields)
			print("-------------------------")
			print(int(data.objects[i].point_clouds[j].data) + data.objects[i].point_clouds[j].fields[0].offset)
			print(int(data.objects[i].point_clouds[j].data) + data.objects[i].point_clouds[j].fields[1].offset)
			print(int(data.objects[i].point_clouds[j].data) + data.objects[i].point_clouds[j].fields[2].offset)
			pub.publish(data.objects[i].point_clouds[j])
			rate.sleep()

def listener():
    rospy.Subscriber("recognized_object_array", RecognizedObjectArray, callback)
    rospy.spin()

if __name__ == '__main__':
	rospy.init_node('pointcloud_converter', anonymous=True)
	rate = rospy.Rate(10)
	listener()