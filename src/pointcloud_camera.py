import rospy
from std_msgs.msg import String
import numpy as np
import struct
import ctypes
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from object_recognition_msgs.msg import RecognizedObjectArray

pub = rospy.Publisher('mine', PointCloud2, queue_size=10)
rate = None
temp = []
	

def callback(data):
	data_out = pc2.read_points(data, skip_nans=True)
	loop = True
	while loop:
		try:
			int_data = next(data_out)
			s = struct.pack('>f' ,int_data[3])
			i = struct.unpack('>l',s)[0]
			pack = ctypes.c_uint32(i).value

			r = (pack & 0x00FF0000)>> 16
			g = (pack & 0x0000FF00)>> 8
			b = (pack & 0x000000FF)

			print("%d %d %d" % r,g,b)

		except Exception as e:
			rospy.loginfo(e.message)
			loop = False

	data.header.stamp = rospy.Time()
	data.header.frame_id = data.header.frame_id
	#print(data.fields)
	#print("-------------------------")
	#pub.publish(data)
	#rate.sleep()

def listener():
    rospy.Subscriber("camera/depth_registered/points", PointCloud2, callback)
    rospy.spin()

if __name__ == '__main__':
	rospy.init_node('pointcloud_converter', anonymous=True)
	rate = rospy.Rate(10)
	listener()