#!/usr/bin/env python

import rospy
import std_msgs.msg
import geometry_msgs.msg

from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker

from math import sin,cos,atan2,pi,sqrt
import matplotlib.pyplot as plt

from ellipse2d import Ellipse2d

#turn laser range into xy, remove walls
def fit_ellipse(data, publisher):
    ellipse_xy = []
    #points = [] #array to hold all points - FOR DEBUGGING
    angle = data.angle_min
    incr = data.angle_increment
    max_range = data.range_max
    ranges = data.ranges
    #polar >> cartesian
    for r in ranges:
    	if r < max_range:
    	    ellipse_xy.append([cos(angle)*r, sin(angle)*r]) #make xy
    	angle+=incr
    #eliminate outlying points
    #x_avg = sum([xy[0] for xy in ellipse_xy])/len(ellipse_xy)
    #y_avg = sum([xy[1] for xy in ellipse_xy])/len(ellipse_xy)
    #for xy in ellipse_xy:
    #    if (abs(xy[0]-x_avg)<0.5 or abs(xy[1]-y_avg)<0.5):
    #        ellipse_xy.remove(xy)
    #fit ellipse to  points
    if len(ellipse_xy) > 1:
    	e2 = Ellipse2d()
    	e2.fit(ellipse_xy)
    	#Publish ellipse data as Marker message
    	h = std_msgs.msg.Header()
    	h.frame_id = "laser" #tie marker visualization to laser it comes from
    	h.stamp = rospy.Time.now() # Note you need to call rospy.init_node() before this will work
    	#publish marker:person_marker, modify a red cylinder, last indefinitely
    	mark = Marker()
    	mark.header=h
    	mark.ns="person_marker"
    	mark.id=0
    	mark.type=3
    	mark.action=0
    	mark.pose=geometry_msgs.msg.Pose(geometry_msgs.msg.Point(e2.center[0], e2.center[1], 0.5),
                                         geometry_msgs.msg.Quaternion(0.0,0.0,1.0,cos(e2.theta/2)))
    	mark.scale=geometry_msgs.msg.Vector3(e2.a,e2.b,1) #scale, in meters
    	mark.color=std_msgs.msg.ColorRGBA(1, 0, 0, 1) #marker is set to be opaque red
    	publisher.publish(mark)
    else:
        print "data not received"

def tracker():
    rospy.init_node("tracker", anonymous=True)
    #listen to filtered scan topic
    pub=rospy.Publisher("ellipse_fit", Marker, queue_size=10)
    rospy.Subscriber("filtered_scan", LaserScan, fit_ellipse, pub)
    #spin until node is closed
    rospy.spin()

if __name__ == '__main__':
    try:
        tracker()
    except rospy.ROSInterruptException:
        pass