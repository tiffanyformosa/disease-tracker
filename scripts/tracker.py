#!/usr/bin/env python

import rospy
import std_msgs.msg
import geometry_msgs.msg
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker

from math import sin,cos,atan2,pi,sqrt
import matplotlib.pyplot as plt

from ellipse2d import Ellipse2d

class Tracker:
    def __init__(self, axis_a=0.9, center_a=0.1):
        self.axis_alpha = axis_a
        self.center_alpha = center_a
        self.last_a = None
        self.last_b = None
        self.last_center = None
        self.red = ColorRGBA(1, 0, 0, 1)
        self.green = ColorRGBA(0, 1, 0, 1)
        self.color = self.red

    def reset (self, run):
        self.last_a = None
        self.last_b = None
        self.last_center = None
        self.color = self.red

    def get_colors(self, data):
        if data < 0.5:
            self.color = self.red
        else:
            self.color = self.green

    #turn filtered laser data into markers
    def _fit_ellipse(self, data, publisher):
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
        #fit ellipse to points - constrain size
        if len(ellipse_xy) > 1:
            e2 = Ellipse2d()
            e2.fit(ellipse_xy)
            #apply alpha to smooth changes over time, if old data exists
            if self.last_a != None and self.last_b != None and self.last_center != None:
                e2.center = [self.last_center[i]*self.center_alpha + e2.center[i]*(1-self.center_alpha) for i in [0, 1]]
                e2.a = self.last_a*self.axis_alpha + e2.a*(1-self.axis_alpha)
                e2.b = self.last_b*self.axis_alpha + e2.b*(1-self.axis_alpha)
            self.last_center = e2.center
            self.last_b = e2.b
            self.last_a = e2.a
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
            mark.scale=geometry_msgs.msg.Vector3(e2.a*2,e2.b*2,1) #scale, in meters
            mark.color=self.color #marker is set to be opaque red
            publisher.publish(mark)
        else:
            print "data not received"

    def setup(self):
        rospy.init_node("tracker", anonymous=True)
        #listen to filtered scan topic
        pub=rospy.Publisher("tracker", Marker, queue_size=10)
        rospy.Subscriber("filtered_scan", LaserScan, self._fit_ellipse, pub)
        rospy.Subscriber("update_filter_cmd", std_msgs.msg.Bool, self.reset)
        rospy.Subscriber("contam", Float32, self.get_colors)
        #spin until node is closed
        rospy.spin()

if __name__ == '__main__':
    node = Tracker()
    try:
        node.setup()
    except rospy.ROSInterruptException:
        pass