#!/usr/bin/env python

import rospy
import tf
import std_msgs.msg
import geometry_msgs.msg
from numpy import arange
from math import cos, sin, acos, sqrt
from sensor_msgs.msg import PointCloud
from visualization_msgs.msg import Marker

class Contamination:
	def __init__(self):
		self.step = 0.2
		self.is_contaminated=False
		#Point32 list to hold points that mark contam
		self.contaminated = []
		self.listener=''
		self.publisher=''

	#add initial contamination (rectangles)
	def base_contam(self, lower_left, upper_right):
		for x in arange(lower_left[0], upper_right[0], self.step):
			for y in arange(lower_left[1], upper_right[1], self.step):
				self.contaminated.append(geometry_msgs.msg.Point32(x, y, 0))

	#turn ellipse into points
	def get_ellipse_points(self,ellipse):
		#transform map to laser to use this
		try:
			(trans,rot) = self.listener.lookupTransform('/map', '/laser', rospy.Time(0))
			center = (ellipse.pose.position.x+trans[0], ellipse.pose.position.y+trans[1])
			a = ellipse.scale.x
			b = ellipse.scale.y
			theta = acos(ellipse.pose.orientation.w)*2
			ellipse_points=[]
			#outline square that fits ellipse and then find points within ellipse
			for x in arange(round(center[0]-a/2, 2), round(center[0]+a/2, 2), self.step):
				for y in arange(round(center[1]-a/2, 2), round(center[1]+a/2, 2), self.step):
					distance = (pow(((cos(theta)*(x-center[0])+sin(theta)*(y-center[1]))/a), 2) +
								pow(((sin(theta)*(x-center[0])+cos(theta)*(y-center[1]))/b), 2))
					if distance < 1:
						ellipse_points.append((x, y))
			return ellipse_points
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			pass

	def check_contam(self, ellipse):
		#if person isn't yet contaminated check to see if they have become contaminated
		ellipse_points=self.get_ellipse_points(ellipse) #convert marker to points
		if not self.is_contaminated:
			#check if person overlaps with contaminated stuff
			for point in ellipse_points:
				for c in self.contaminated:
					if abs(point[0]-c.x)<0.1 and abs(point[1]-c.y)<0.1:
						self.is_contaminated=True
						break
				if self.is_contaminated==True:
					break
			# if person is in area flip is_contaminated to true
		#if person is contaminated after that check, append all points to the contaminated list
		if self.is_contaminated:
			for point in ellipse_points:
				self.contaminated.append(geometry_msgs.msg.Point32(point[0], point[1], 0))

	#initialize node
	def contamination_node(self):
		rospy.init_node('contam', anonymous=True)

		self.publisher=rospy.Publisher("contamination", PointCloud, queue_size=10)
		rospy.Subscriber("ellipse_fit", Marker, self.check_contam)
		self.listener = tf.TransformListener()
		rate = rospy.Rate(10.0)
		while True:
			cloud=PointCloud(header=std_msgs.msg.Header(stamp=rospy.Time.now(),frame_id = "map"), points=self.contaminated)
			self.publisher.publish(cloud)
			if rospy.is_shutdown():
				break
			rate.sleep()
		

if __name__ == '__main__':
	node = Contamination()
	node.base_contam([-2.3, -0.2], [-1.4, 2,0])
	node.base_contam([-0.2, -0.2], [0.7, 2,0]) 
	try:
		node.contamination_node()
	except rospy.ROSInterruptException:
		pass