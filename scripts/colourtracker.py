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

def main(args):
	# fourcc = cv2.cv.CV_FOURCC(*'XVID')
	# out = cv2.VideoWriter("output.avi", -1, 20, (640, 480))

	#capturing video through webcam
	cap=cv2.VideoCapture(1)
	cap.set(3,640)
	cap.set(4,480)

	bridge = CvBridge()

	pubimg = rospy.Publisher("processedCamOutput",Image, queue_size = 10)
	pubcap = rospy.Publisher("unprocessedCamOutput",Image, queue_size = 10)
	pubobj = rospy.Publisher("detectedObject",Float32MultiArray, queue_size = 10)
	rospy.init_node('colourtracker', anonymous=True)
	rate = rospy.Rate(10) # 10hz

	list_obj = []

	while not rospy.is_shutdown():
		
		_, img = cap.read()
		timeImg = rospy.Time()

		image_unprocessed = bridge.cv2_to_imgmsg(img, "bgr8")
		pubcap.publish(image_unprocessed)

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
				cv2.rectangle(img,(x,y),(x+w,y+h),(0,0,255),2)
				cv2.putText(img,"Red color",(x,y),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255))
				obj = [float(1), float(x+w/2), float(y+h/2), area] #identity 1 for red
				list_obj = list_obj+obj
				print ("Red in frame")

		#tracking Blue
		(contours,hierarchy)=cv2.findContours(blue,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

		for pic, contour in enumerate(contours):
			area = cv2.contourArea(contour)
			if(area>5000):
				x,y,w,h = cv2.boundingRect(contour)
				cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)
				cv2.putText(img,"Blue color",(x,y),cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255,0,0))
				obj = [float(2), float(x+w/2), float(y+h/2), area] #identity 2 for blue
				list_obj = list_obj+obj
				print ("Blue in frame")

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
	
		image_output = bridge.cv2_to_imgmsg(img, "bgr8")
		pubimg.publish(image_output)
		pubobj.publish(data)

		list_obj = []
	
		rate.sleep()

if __name__ == '__main__':
	try:
		main(sys.argv)
	except rospy.ROSInterruptException:
		print("Shutting down")
		cv2.destroyAllWindows()