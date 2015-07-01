#!/usr/bin/env python

import struct
from numpy import arange
from math import cos, sin, acos, sqrt

import rospy
import tf
import std_msgs.msg
import geometry_msgs.msg
from sensor_msgs.msg import PointCloud, ChannelFloat32
from visualization_msgs.msg import Marker

class Contamination:
	def __init__(self):
		self.lgreen = struct.unpack('f', struct.pack('i', 0x00ff00))[0]
		#self.dgreen = struct.unpack('f', struct.pack('i', 0x007755))[0]
		self.step = 0.2
		self.is_contaminated=False
		#Point32 list to hold points that mark contam
		self.contaminated = []
		self.color = ChannelFloat32(name="rgb")
		self.listener=''
		self.publisher=''

	#add initial contamination (rectangles)
	def base_contam(self, lower_left, upper_right):
		for x in arange(lower_left[0], upper_right[0], self.step):
			for y in arange(lower_left[1], upper_right[1], self.step):
				self.contaminated.append(geometry_msgs.msg.Point32(x, y, 0))
				self.color.values.append(self.lgreen)

	#turn ellipse into points
	def get_ellipse_data(self,ellipse):
		#transform map to laser to use this
		try:
			(trans,rot) = self.listener.lookupTransform('/map', '/laser', rospy.Time(0))
			center = (ellipse.pose.position.x+trans[0], ellipse.pose.position.y+trans[1])
			a = ellipse.scale.x
			b = ellipse.scale.y
			theta = acos(ellipse.pose.orientation.w)*2
			return center, a, b, theta
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			pass

	def check_contam(self, ellipse):
		#if person isn't yet contaminated check to see if they have become contaminated
		(center, a, b, theta)=self.get_ellipse_data(ellipse) #convert marker to points
		if not self.is_contaminated:
			#check if person overlaps with contaminated stuff
			for c in self.contaminated:
				distance = (pow(((cos(theta)*(c.x-center[0])+sin(theta)*(c.y-center[1]))/a), 2) +
							pow(((sin(theta)*(c.x-center[0])+cos(theta)*(c.y-center[1]))/b), 2))
			# if person is in area flip is_contaminated to true and break
				if distance < 1:
					self.is_contaminated = True
					break
		#if person is contaminated after that check, append points to the contaminated list
		if self.is_contaminated:
			#outline square that fits ellipse and then find points within ellipse
			for x in arange(round(center[0]-a, 2), round(center[0]+a, 2), self.step):
				for y in arange(round(center[1]-a, 2), round(center[1]+a, 2), self.step):
					distance = (pow(((cos(theta)*(x-center[0])+sin(theta)*(y-center[1]))/a), 2) +
								pow(((sin(theta)*(x-center[0])+cos(theta)*(y-center[1]))/b), 2))
					if distance < 1: #within ellipse
						self.contaminated.append(geometry_msgs.msg.Point32(x, y, 0))
						self.color.values.append(self.lgreen)

	#initialize node
	def contamination_node(self):
		rospy.init_node('contam', anonymous=True)

		self.publisher=rospy.Publisher("contamination", PointCloud, queue_size=10)
		rospy.Subscriber("ellipse_fit", Marker, self.check_contam)
		self.listener = tf.TransformListener()
		rate = rospy.Rate(10.0)
		while True:
			cloud=PointCloud(header=std_msgs.msg.Header(stamp=rospy.Time.now(),frame_id = "map"),
							 points=self.contaminated, channels=[self.color])
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