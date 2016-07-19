#!/usr/bin/env python
import serial
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import TwistWithCovarianceStamped
import math

knots = 0.514444
pubTwist = rospy.Publisher('twistGPS', TwistWithCovarianceStamped, queue_size=1000)
latest_yaw = 0.0

def callback(sensorstr):
	global latest_yaw
	global knots
	data = sensorstr.data
	if data[0] == 'Z':
		data = data.replace("Z","").replace("\n","").replace("\r","")
		data_list = data.split(',')
		if len(data_list) == 4:
			try:
				float_list = [float(i) for i in data_list]
				twistGPS = TwistWithCovarianceStamped()
				twistGPS.header.frame_id = "world"
				twistGPS.header.stamp = rospy.Time.now()
				twistGPS.twist.twist.linear.x = float_list[3]*knots*math.cos(latest_yaw)
				twistGPS.twist.twist.linear.y = float_list[3]*knots*math.sin(latest_yaw)
				twistGPS.twist.twist.linear.z = 0.0
				##angular data not used here
				twistGPS.twist.twist.angular.x = 0.0
				twistGPS.twist.twist.angular.y = 0.0
				twistGPS.twist.twist.angular.z = 0.0
				twistGPS.twist.covariance = [0.01, 0.01, 0, 0, 0, 0]
				try:
					pubTwist.publish(twistGPS)
					log = "GPS Twist Data: %f %f Publish at Time: %s" % (twistGPS.twist.twist.linear.x, twistGPS.twist.twist.linear.y, rospy.get_time())
				except: log = "twistGPS Publisher Error"

			except: 
					log = "GPS Twist Data Error! Data :  %s" % data
		else:
			log = "GPS Data List Wrong! Data :  %s" % data
			rospy.loginfo(log)
		rospy.loginfo(log)

	if data[0] == 'Y':
		data = data.replace("Y","").replace("\n","").replace("\r","")
		data_list = data.split(',')
		if len(data_list) == 9:
			try:
				##data_list structure: accel, magni, gyro
				float_list = [float(i) for i in data_list]
				latest_yaw = float_list[3]
			except:
				rospy.loginfo("IMU Data Type Error")
		else: rospy.loginfo("IMU Data structure Error")

def talker():
	rospy.init_node('gpstwistPublisher', anonymous=True)
	rospy.Subscriber('sensor/string', String, callback)
	rospy.spin()

if __name__ == '__main__':
	try: talker()
	except rospy.ROSInterupptException:
		print 'Ros talker failed'
		pass
