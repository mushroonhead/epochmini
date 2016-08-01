#!/usr/bin/env python
import roslib
# roslib.load_manifest('my_package')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

#from __future__ import print_function


class image_converter:

	def __init__(self):
		self.image_pub = rospy.Publisher("processedComOutTwo", Image)

		self.bridge = CvBridge()
		self.image_subl = rospy.Subscriber("unprocessedCamOutput", Image, self.callback)

	def callback(self,data):
		try:
			img = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print (e)

		list_obj = []

		#converting frame to BGR to HSV

		hsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)

		#defining the range of red color
		red_lower=np.array([0,50,50],np.uint8) #136, 87, 111
		red_upper=np.array([10,255,255],np.uint8) #180, 255, 255

		#defining the range of Blue color
		blue_lower=np.array([70,80,60],np.uint8) #99, 115, 150
		blue_upper=np.array([160,255,255],np.uint8)
		#defining the Range of yellow color
		# yellow_lower=np.array([20,100,100],np.uint8)
		# yellow_upper=np.array([30,255,255],np.uint8)

		#defining range for green
		# green_lower=np.array([29,86,6],np.uint8)
		# green_upper=np.array([64,255,82],np.uint8)

		#finding the range of red, blue, yellow color int the image
		red=cv2.inRange(hsv, red_lower, red_upper)
		blue=cv2.inRange(hsv,blue_lower,blue_upper)
		# yellow=cv2.inRange(hsv,yellow_lower,yellow_upper)
		# green=cv2.inRange(hsv,green_lower,green_upper)


		kernal = np.ones((5 ,5), "uint8")
		red=cv2.dilate(red, kernal)
		res=cv2.bitwise_and(img, img, mask = red)

		blue=cv2.dilate(blue,kernal)
		res1=cv2.bitwise_and(img, img, mask = blue)

		# yellow=cv2.dilate(yellow,kernal)
		# res2=cv2.bitwise_and(img, img, mask = yellow) 

		# green=cv2.dilate(green,kernal)
		# res2=cv2.bitwise_and(img, img, mask = green)

		#tracking Red
		(contours,hierarchy)=cv2.findContours(red,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

		for pic, contour in enumerate(contours):
			area = cv2.contourArea(contour)
			if(area>5000):
				x,y,w,h = cv2.boundingRect(contour) 
				rect = cv2.minAreaRect(contour)
				if  rect[1][0]/rect[1][1] < 2.0 and rect[1][0]/rect[1][1] > 0.5:
					#box = cv2.boxPoints(rect)
					#box = np.int0(box)
					#cv2.drawContours(img,[box],(0,0,255),2)
					cv2.rectangle(img,(x,y),(x+w,y+h),(0,0,255),2)
					cv2.putText(img,"Red Obstacle",(x,y),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255))
					obj = [float(1), float(x+w/2), float(y+h/2), area] #identity 1 for red
					list_obj = list_obj+obj
					print ("Detected RedObstacle at (%d,%d)" %(x+w/2, y+h/2))

		#tracking Blue
		(contours,hierarchy)=cv2.findContours(blue,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

		for pic, contour in enumerate(contours):
			area = cv2.contourArea(contour)
			if(area>5000):
				x,y,w,h = cv2.boundingRect(contour) 
				rect = cv2.minAreaRect(contour)
				print rect[1][0]
				print rect[1][1]
				if rect[1][0]/rect[1][1] < 2.0 and rect[1][0]/rect[1][1] > 0.5: 
					#box = cv2.cv.boxPoints(rect)
					#box = np.int0(box)
					#cv2.drawContours(img,[box],(0,0,255),2)
					cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)
					cv2.putText(img,"Blue Obstacle",(x,y),cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255,0,0))
					obj = [float(2), float(x+w/2), float(y+h/2), area] #identity 2 for blue
					list_obj = list_obj+obj
					print ("Detected BlueObstacle at (%d,%d)" %(x+w/2, y+h/2))

		# #tracking Yellow
		# (_,contours,hierarchy)=cv2.findContours(yellow,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

		# for pic, contour in enumerate(contours):
			# 	area = cv2.contourArea(contour)
			# 	if(area>50000):
				# 		x,y,w,h = cv2.boundingRect(contour)
				# 		img = cv2.rectangle(img,(x,y),(x+w,y+h),(0,255,255),2)
				# 		cv2.putText(img,"Yellow color",(x,y),cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0,255,255))
				# 		print "Yellow in frame"
			
		# #tracking Green
		# (_,contours,hierarchy)=cv2.findContours(green,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

		# for pic, contour in enumerate(contours):
		# 	area = cv2.contourArea(contour)
		# 	if(area>50000):
		# 		x,y,w,h = cv2.boundingRect(contour)
		# 		img = cv2.rectangle(img,(x,y),(x+w,y+h),(0,255,0),2)
		# 		cv2.putText(img,"Green color",(x,y),cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0,255,0))
		# 		print "Green in frame"

		# out.write(img)
			# cv2.imshow("Redcolour",red)
		# cv2.imshow("Test",img)
		# cv2.imshow("red",res)
		# cv2.imshow("blue",res1)
		# if cv2.waitKey(10) & 0xFF == ord('q'):
		# 	break

		data = Float32MultiArray()
		data.layout.dim = [MultiArrayDimension(), MultiArrayDimension()]
		data.layout.dim[0].label = "objnumber"
		data.layout.dim[0].size = len(list_obj)/4
		data.layout.dim[0].stride = len(list_obj)
		data.layout.dim[1].label = "attributes"
		data.layout.dim[1].size = 4
		data.layout.dim[1].stride = 4
		data.layout.data_offset = 0
		data.data = list_obj
		for i in range(len(data.data)):
			print data.data[i]
			print type(data.data[i])

		list_obj = []

		cv2.imshow("Image window", img)
		cv2.waitKey(3)

		try:
			self.image_pub.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))
		except CvBridgeError as e:
			print (e)

def main(args):
	ic = image_converter()
	rospy.init_node('colour_tracker_test', anonymous=True)
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting Down")
	cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)