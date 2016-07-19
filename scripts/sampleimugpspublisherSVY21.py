#!/usr/bin/env python
import serial
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import TwistWithCovarianceStamped
from sensor_msgs.msg import Imu
import tf
import math
import traceback

accel_scale = 9.81*4.0/1024.0
gyro_scale = 1.0/14.375

##Conversion to SVY21 from GPS Provided by cgcai @https://github.com/cgcai/SVY21
class SVY21:
	# Ref: http://www.linz.govt.nz/geodetic/conversion-coordinates/projection-conversions/transverse-mercator-preliminary-computations/index.aspx
	
	# WGS84 Datum
	a = 6378137
	f = 1 / 298.257223563

	# SVY21 Projection
	# Fundamental point: Base 7 at Pierce Resevoir.
	# Latitude: 1 22 02.9154 N, longitude: 103 49 31.9752 E (of Greenwich).

	# Known Issue: Setting (oLat, oLon) to the exact coordinates specified above
	# results in computation being slightly off. The values below give the most 
	# accurate represenation of test data.
	oLat = 1.366666     # origin's lat in degrees
	oLon = 103.833333   # origin's lon in degrees
	oN = 38744.572      # false Northing
	oE = 28001.642      # false Easting
	k = 1               # scale factor

	#
	def __init__(self):
		self.b = self.a * (1 - self.f)
		self.e2 = (2 * self.f) - (self.f * self.f)
		self.e4 = self.e2 * self.e2
		self.e6 = self.e4 * self.e2
		self.A0 = 1 - (self.e2 / 4) - (3 * self.e4 / 64) - (5 * self.e6 / 256);
		self.A2 = (3. / 8.) * (self.e2 + (self.e4 / 4) + (15 * self.e6 / 128));
		self.A4 = (15. / 256.) * (self.e4 + (3 * self.e6 / 4));
		self.A6 = 35 * self.e6 / 3072;

	def computeSVY21(self, lat, lon):
		"""
		Returns a pair (N, E) representing Northings and Eastings in SVY21.
		"""

		latR = lat * math.pi / 180
		sinLat = math.sin(latR)
		sin2Lat = sinLat * sinLat
		cosLat = math.cos(latR)
		cos2Lat = cosLat * cosLat
		cos3Lat = cos2Lat * cosLat
		cos4Lat = cos3Lat * cosLat
		cos5Lat = cos4Lat * cosLat
		cos6Lat = cos5Lat * cosLat
		cos7Lat = cos6Lat * cosLat

		rho = self.calcRho(sin2Lat)
		v = self.calcV(sin2Lat)
		psi = v / rho
		t = math.tan(latR)
		w = (lon - self.oLon) * math.pi / 180

		M = self.calcM(lat)
		Mo = self.calcM(self.oLat)

		w2 = w * w
		w4 = w2 * w2
		w6 = w4 * w2
		w8 = w6 * w2

		psi2 = psi * psi
		psi3 = psi2 * psi
		psi4 = psi3 * psi

		t2 = t * t
		t4 = t2 * t2
		t6 = t4 * t2

		# Compute Northing
		nTerm1 = w2 / 2 * v * sinLat * cosLat
		nTerm2 = w4 / 24 * v * sinLat * cos3Lat * (4 * psi2 + psi - t2)
		nTerm3 = w6 / 720 * v * sinLat * cos5Lat * ((8 * psi4) * (11 - 24 * t2) - (28 * psi3) * (1 - 6 * t2) + psi2 * (1 - 32 * t2) - psi * 2 * t2 + t4)
		nTerm4 = w8 / 40320 * v * sinLat * cos7Lat * (1385 - 3111 * t2 + 543 * t4 - t6)
		N = self.oN + self.k * (M - Mo + nTerm1 + nTerm2 + nTerm3 + nTerm4)

		# Compute Easting
		eTerm1 = w2 / 6 * cos2Lat * (psi - t2)
		eTerm2 = w4 / 120 * cos4Lat * ((4 * psi3) * (1 - 6 * t2) + psi2 * (1 + 8 * t2) - psi * 2 * t2 + t4)
		eTerm3 = w6 / 5040 * cos6Lat * (61 - 479 * t2 + 179 * t4 - t6)
		E = self.oE + self.k * v * w * cosLat * (1 + eTerm1 + eTerm2 + eTerm3)

		return (N, E)

	def calcM(self, lat):
		latR = lat * math.pi / 180
		return self.a * ((self.A0 * latR) - (self.A2 * math.sin(2 * latR)) + (self.A4 * math.sin(4 * latR)) - (self.A6 * math.sin(6 * latR)))

	def calcRho(self, sin2Lat):
		num = self.a * (1 - self.e2)
		denom = math.pow(1 - self.e2 * sin2Lat, 3. / 2.)
		return num / denom

	def calcV(self, sin2Lat):
		poly = 1 - self.e2 * sin2Lat
		return self.a / math.sqrt(poly)

	def computeLatLon(self, N, E):
		"""
		Returns a pair (lat, lon) representing Latitude and Longitude.
		"""

		Nprime = N - self.oN
		Mo = self.calcM(self.oLat)
		Mprime = Mo + (Nprime / self.k)
		n = (self.a - self.b) / (self.a + self.b)
		n2 = n * n
		n3 = n2 * n
		n4 = n2 * n2
		G = self.a * (1 - n) * (1 - n2) * (1 + (9 * n2 / 4) + (225 * n4 / 64)) * (math.pi / 180)
		sigma = (Mprime * math.pi) / (180. * G)
		
		latPrimeT1 = ((3 * n / 2) - (27 * n3 / 32)) * math.sin(2 * sigma)
		latPrimeT2 = ((21 * n2 / 16) - (55 * n4 / 32)) * math.sin(4 * sigma)
		latPrimeT3 = (151 * n3 / 96) * math.sin(6 * sigma)
		latPrimeT4 = (1097 * n4 / 512) * math.sin(8 * sigma)
		latPrime = sigma + latPrimeT1 + latPrimeT2 + latPrimeT3 + latPrimeT4

		sinLatPrime = math.sin(latPrime)
		sin2LatPrime = sinLatPrime * sinLatPrime

		rhoPrime = self.calcRho(sin2LatPrime)
		vPrime = self.calcV(sin2LatPrime)
		psiPrime = vPrime / rhoPrime
		psiPrime2 = psiPrime * psiPrime
		psiPrime3 = psiPrime2 * psiPrime
		psiPrime4 = psiPrime3 * psiPrime
		tPrime = math.tan(latPrime)
		tPrime2 = tPrime * tPrime
		tPrime4 = tPrime2 * tPrime2
		tPrime6 = tPrime4 * tPrime2
		Eprime = E - self.oE
		x = Eprime / (self.k * vPrime)
		x2 = x * x
		x3 = x2 * x
		x5 = x3 * x2
		x7 = x5 * x2

		# Compute Latitude
		latFactor = tPrime / (self.k * rhoPrime)
		latTerm1 = latFactor * ((Eprime * x) / 2)
		latTerm2 = latFactor * ((Eprime * x3) / 24) * ((-4 * psiPrime2) + (9 * psiPrime) * (1 - tPrime2) + (12 * tPrime2))
		latTerm3 = latFactor * ((Eprime * x5) / 720) * ((8 * psiPrime4) * (11 - 24 * tPrime2) - (12 * psiPrime3) * (21 - 71 * tPrime2) + (15 * psiPrime2) * (15 - 98 * tPrime2 + 15 * tPrime4) + (180 * psiPrime) * (5 * tPrime2 - 3 * tPrime4) + 360 * tPrime4)
		latTerm4 = latFactor * ((Eprime * x7) / 40320) * (1385 - 3633 * tPrime2 + 4095 * tPrime4 + 1575 * tPrime6)
		lat = latPrime - latTerm1 + latTerm2 - latTerm3 + latTerm4

		# Compute Longitude
		secLatPrime = 1. / math.cos(lat)
		lonTerm1 = x * secLatPrime
		lonTerm2 = ((x3 * secLatPrime) / 6) * (psiPrime + 2 * tPrime2)
		lonTerm3 = ((x5 * secLatPrime) / 120) * ((-4 * psiPrime3) * (1 - 6 * tPrime2) + psiPrime2 * (9 - 68 * tPrime2) + 72 * psiPrime * tPrime2 + 24 * tPrime4)
		lonTerm4 = ((x7 * secLatPrime) / 5040) * (61 + 662 * tPrime2 + 1320 * tPrime4 + 720 * tPrime6)
		lon = (self.oLon * math.pi / 180) + lonTerm1 - lonTerm2 + lonTerm3 - lonTerm4

		return (lat / (math.pi / 180), lon / (math.pi / 180))

svy = SVY21()
oldPose = PoseWithCovarianceStamped()
oldPose.header.stamp = rospy.Time()
oldTime = rospy.Time()
latest_yaw = 0.0

def talker():
	global oldPose
	global oldTime
	print oldTime.to_sec()
	# currentTime = rospy.get_rostime()
	pubGPS = rospy.Publisher('poseGPS', PoseWithCovarianceStamped, queue_size=1000)
	pubIMU = rospy.Publisher('imuBoat', Imu, queue_size=1000)
	pubtwistGPS = rospy.Publisher('twistGPS', TwistWithCovarianceStamped, queue_size = 1000)
	rospy.init_node('imugpspublisher', anonymous=True)
	rate = rospy.Rate(100) # 100hz
	while not rospy.is_shutdown():
		currentTime = rospy.Time.now()
		timeCheck = currentTime.to_sec() - oldTime.to_sec()
		if (timeCheck) > 2.0:
			data = 'Z1.341725,103.965008,1.5100,0.0000'
			oldTime = currentTime;
		else: data = 'Y0.0000,0.0730,255.4516,1.5100,0.0000,0.0000,0.000,0.000,0.000'
		# data = ser.readline()
		if data[0] == 'Z':
			data = data.replace("Z","").replace("\n","").replace("\r","")
			data_list = data.split(',')
			if len(data_list) == 4:
				try:
					##data_list structure: lat, lon, heading, vel 
					float_list = [float(i) for i in data_list]
					ne = svy.computeSVY21(float_list[0],float_list[1])
					##print ne
					poseGPS = PoseWithCovarianceStamped()
					poseGPS.header.frame_id = "odom"
					poseGPS.header.stamp = rospy.Time.now()
					poseGPS.pose.pose.position.x = ne[1]
					poseGPS.pose.pose.position.y = ne[0]
					poseGPS.pose.pose.position.z = 0.0
					quat = tf.transformations.quaternion_from_euler(0, 0, float_list[2])
					poseGPS.pose.pose.orientation.x = quat[0]
					poseGPS.pose.pose.orientation.y = quat[1]
					poseGPS.pose.pose.orientation.z = quat[2]
					poseGPS.pose.pose.orientation.w = quat[3]
					poseGPS.pose.covariance = [3.24, 0, 0, 0, 0, 0, 0, 3.24, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 10487.47]
					pubGPS.publish(poseGPS)
					log = "GPS Data: east=%f north=%f ang=%f vel=%f Publish at Time: %s" % (ne[1], ne[0], float_list[2], float_list[3], rospy.get_time())
					rospy.loginfo(log)
					timeDiff = poseGPS.header.stamp.to_sec() - oldPose.header.stamp.to_sec()
					if timeDiff < 3.0:
						twistGPS = TwistWithCovarianceStamped()
						twistGPS.header.frame_id = "base_link"
						twistGPS.header.stamp = poseGPS.header.stamp
						twistGPS.twist.twist.linear.x = (poseGPS.pose.pose.position.x - oldPose.pose.pose.position.x)/timeDiff
						twistGPS.twist.twist.linear.y = (poseGPS.pose.pose.position.y - oldPose.pose.pose.position.y)/timeDiff
						twistGPS.twist.covariance = [6.48/pow(timeDiff,2), 6.48/pow(timeDiff,2), 0, 0, 0, 0]
						# angular velocity would not be used
						pubtwistGPS.publish(twistGPS)
						log = "GPS Twist: x_vel=%f y_vel=%f" % (twistGPS.twist.twist.linear.x, twistGPS.twist.twist.linear.y)
					oldPose = poseGPS
				except: 
					log = "GPS Data Error! Data :  %s" % data
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
					latest_yaw = float_list[3]
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
	try: talker()
	except rospy.ROSInterupptException:
		print 'Ros talker failed'
		pass


