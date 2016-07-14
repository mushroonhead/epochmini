#!/usr/bin/env python
import serial
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import tf
import math


def talker():
	pub = rospy.Publisher('poseGPS', PoseStamped, queue_size=1000)
	rospy.init_node('poseGPSBoat', anonymous=True)
 	rate = rospy.Rate(10) # 10hz
	while not rospy.is_shutdown():
		data = ser.readline()
		print data
		if data[0] == 'A' and len(data) >= 30:
			data = data.replace("A","").replace("\n","").replace("\r","")
			data_list = data.split(',')
			##data_list structure: long, lat, heading, vel
			float_list = [float(i) for i in data_list]
			poseGPS = PoseStamped()
			poseGPS.header.frame_id = "odom"
			poseGPS.header.stamp = rospy.Time.now()
			poseGPS.pose.position.x = float_list[0]
			poseGPS.pose.position.y = float_list[1]
			poseGPS.pose.position.z = 0.0
			quat = tf.transformations.quaternion_from_euler(0, 0, float_list[2])
			poseGPS.pose.orientation.x = quat[0]
			poseGPS.pose.orientation.y = quat[1]
			poseGPS.pose.orientation.z = quat[2]
			poseGPS.pose.orientation.w = quat[3]
			pub.publish(poseGPS)
			log = "Data: %f %f %f %f Publish at Time: %s" % (float_list[0], float_list[1], float_list[2], float_list[3], rospy.get_time())
			rospy.loginfo(log)
		else:
			log = "Error in Data! Message: %s" % data
			rospy.loginfo(log)
		rate.sleep()

if __name__ == '__main__':
	print 'Initiating: Trying to Reach Arduino'
	try:
		try:
			ser = serial.Serial('/dev/ttyACM0', 115200);
			print "serial ttyACM0"
		except:	
			ser = serial.Serial('/dev/ttyACM1', 115200);
			print "serial ttyACM1"
		print 'Test'
		talker()
	except rospy.ROSInterupptException:
		print 'Ros talker failed'
		pass


