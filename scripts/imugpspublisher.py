#!/usr/bin/env python
import serial
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
import tf
import math

accel_scale = 9.81*4.0/1024.0
gyro_scale = 1.0/14.375

def talker():
	pubGPS = rospy.Publisher('poseGPS', NavSatFix, queue_size=1000)
	pubIMU = rospy.Publisher('imuBoat', Imu, queue_size=1000)
	rospy.init_node('imugpspublisher', anonymous=True)
	rate = rospy.Rate(100) # 100hz
	while not rospy.is_shutdown():
		data = ser.readline()
		##print data
		##print len(data)
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
					pubGPS.publish(poseGPS)
					log = "GPS Data: %f %f %f %f Publish at Time: %s" % (float_list[0], float_list[1], float_list[2], float_list[3], rospy.get_time())
				except: log = "GPS Data Error! Data :  %s" % data
			else:
				log = "GPS Data Error! Data :  %s" % data
			rospy.loginfo(log)
		elif data[0] == 'Y':
			data = data.replace("Y","").replace("\n","").replace("\r","")
			data_list = data.split(',')
			if len(data_list) == 9:
				try:
					##data_list structure: accel, magni, gyro
					float_list = [float(i) for i in data_list]
					imuData = Imu()
					imuData.header.frame_id = "base_link"
					imuData.header.stamp = rospy.Time.now()
					quat = tf.transformations.quaternion_from_euler(float_list[3], float_list[4], float_list[5], 'rzyx')
					imuData.orientation.x = quat[0]
					imuData.orientation.y = quat[1]
					imuData.orientation.z = quat[2]
					imuData.orientation.w = quat[3]
					imuData.angular_velocity.x = math.radians(float_list[6]*gyro_scale)
					imuData.angular_velocity.y = math.radians(-float_list[7]*gyro_scale)
					imuData.angular_velocity.z = math.radians(-float_list[8]*gyro_scale)
					imuData.linear_acceleration.x = float_list[0]*accel_scale
					imuData.linear_acceleration.y = -float_list[1]*accel_scale
					imuData.linear_acceleration.z = -float_list[2]*accel_scale
					imuData.orientation_covariance = [1.5838e-6, 0, 0, 0, 1.49402e-6, 0, 0, 0, 1.88934e-6]
					imuData.angular_velocity_covariance = [7.84113e-7, 0, 0, 0, 5.89609e-7, 0, 0, 0, 6.20293e-7]
					imuData.linear_acceleration_covariance = [9.8492e-4, 0, 0, 0, 7.10809e-4, 0, 0, 0, 1.42516e-3]
					pubIMU.publish(imuData)
					log = "IMU Data: %f %f %f %f %f %f %f %f %f Publish at Time: %s" \
					% (imuData.linear_acceleration.x, imuData.linear_acceleration.y, imuData.linear_acceleration.z,
						float_list[3], float_list[4], float_list[5],
						imuData.angular_velocity.x, imuData.angular_velocity.y, imuData.angular_velocity.z, rospy.get_time())
				except: log = "IMU Data Error! Data :  %s" % data
			else: log = "Data Error! Data :  %s" % data
			rospy.loginfo(log)
		else:
			log = "Data Error or Message: %s" % data
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


