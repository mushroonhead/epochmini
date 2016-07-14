#!/usr/bin/env python
import serial
import rospy
from geometry_msgs.msg import PoseStamped
import tf
import math

accel_scale = 9.81*4.0/1024.0
gyro_scale = 1.0/14.375

def talker():
	pub = rospy.Publisher('imu_orientation', PoseStamped, queue_size=1000)
	rospy.init_node('callibIMU', anonymous=True)
 	rate = rospy.Rate(100) # 100hz
	while not rospy.is_shutdown():
		data = ser.readline()
		# print data
		# print len(data)
		if data[0] == 'Y' and len(data) >= 70 and len(data) <= 85:
			data = data.replace("Y","").replace("\n","").replace("\r","")
			data_list = data.split(',')
			if len(data_list) == 9:
				try:
					##data_list structure: accel, magni, gyro
					float_list = [float(i) for i in data_list]
					imuData = PoseStamped()
					imuData.header.frame_id = "map"
					imuData.header.stamp = rospy.Time.now()
					quat = tf.transformations.quaternion_from_euler(float_list[3], float_list[4], float_list[5], 'rzyx')
					imuData.pose.orientation.x = quat[0]
					imuData.pose.orientation.y = quat[1]
					imuData.pose.orientation.z = quat[2]
					imuData.pose.orientation.w = quat[3]
					imuData.pose.position.x = 0
					imuData.pose.position.y = 0
					imuData.pose.position.z = 0
					pub.publish(imuData)
					log = "Data Published: %f %f %f" % (float_list[3], float_list[4], float_list[5])
				except: log = "Data Error! Data :  %s" % data
			else: log = "Data Error! Data :  %s" % data
			rospy.loginfo(log)
		else:
			log = "Non Data Serial Message: %s" % data
			rospy.loginfo(log)
		rate.sleep()

if __name__ == '__main__':
	print 'Initiating: Trying to Reach Arduino'
	try:
		try:
			ser = serial.Serial('/dev/ttyACM0', 57600);
			print "serial ttyACM0"
		except:	
			ser = serial.Serial('/dev/ttyACM1', 57600);
			print "serial ttyACM1"
		print 'Test'
		talker()
	except rospy.ROSInterupptException:
		print 'Ros talker failed'
		pass


