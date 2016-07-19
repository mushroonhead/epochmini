#!/usr/bin/env python
import serial
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
import tf
import math

accel_scale = 9.81*4.0/1024.0
gyro_scale = 1.0/14.375
knots = 0.514444
pubGPS = rospy.Publisher('poseGPS', NavSatFix, queue_size=1000)

def callback(sensorstr):
	data = sensorstr.data
	if data[0] == 'Z':
		data = data.replace("Z","").replace("\n","").replace("\r","")
		data_list = data.split(',')
		if len(data_list) == 4:
			try:
				##data_list structure: lat, lon, heading, vel 
				float_list = [float(i) for i in data_list]
				poseGPS = NavSatFix()
				poseGPS.header.frame_id = "world"
				poseGPS.header.stamp = rospy.Time.now()
				poseGPS.status.status = 0
				poseGPS.status.service = 1
				poseGPS.latitude = float_list[0]
				poseGPS.longitude = float_list[1]
				poseGPS.altitude = 0.0
				poseGPS.position_covariance = [3.24, 0, 0, 0, 3,24, 0, 0, 0, 0]
				poseGPS.position_covariance_type = 2
				try: 
					pubGPS.publish(poseGPS)
					log = "GPS Pose Data: %f %f Publish at Time: %s" % (float_list[0], float_list[1], rospy.get_time())
				except: 
					log = "poseGPS Publisher Error"

			except: 
				log = "GPS Data Error! Data :  %s" % data
				rospy.loginfo(log)
		rospy.loginfo(log)
	
def talker():
	rospy.init_node('gpsNavStatPublisher', anonymous=True)
	rospy.Subscriber('sensor/string', String, callback)
	rospy.spin()

if __name__ == '__main__':
	try: talker()
	except rospy.ROSInterupptException:
		print 'Ros talker failed'
		pass


