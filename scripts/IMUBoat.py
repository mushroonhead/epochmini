#!/usr/bin/env python
import serial
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Imu
import tf
import math

accel_scale = 9.81*4.0/1024.0
gyro_scale = 1.0/14.375

def talker():
	pub = rospy.Publisher('IMUBoat', Imu, queue_size=1000)
	rospy.init_node('callibIMU', anonymous=True)
 	rate = rospy.Rate(10) # 10hz
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
					imuData = Imu()
					imuData.header.frame_id = "imu"
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
					imuData.orientation_covariance = []
					imuData.angular_velocity_covariance = []
					imuData.linear_acceleration_covariance = []
					pub.publish(imuData)
					# log = "Data: %f %f %f %f %f %f %f %f %f Publish at Time: %s" \
					# % (float_list[0], float_list[1], float_list[2], float_list[3], 
			 	# 	float_list[4], float_list[5], float_list[6], float_list[7],
					# float_list[8], rospy.get_time())
					print "%f %f %f %f %f %f %f %f %f" % (
						imuData.linear_acceleration.x, imuData.linear_acceleration.y, imuData.linear_acceleration.z,
						float_list[3], float_list[4], float_list[5],
						imuData.angular_velocity.x, imuData.angular_velocity.y, imuData.angular_velocity.z)
				except: continue #log = "Data Error! Data :  %s" % data
			else: continue #log = "Data Error! Data :  %s" % data
			#rospy.loginfo(log)
		else:
			continue
			#log = "Non Data Serial Message: %s" % data
			#rospy.loginfo(log)
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


