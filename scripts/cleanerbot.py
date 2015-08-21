#!/usr/bin/env python

#allows a robot to clean up modeled contamination

import numpy as np
import rospy
import tf
from std_msgs.msg import *
from geometry_msgs.msg import *
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker

class CleanerBot:
    def __init__(self):
        self.robot_diam=0.354 #value in meters for turtlebot 
        self.robot_height=0.420 #value in meters for turtlebot

    def clean(self, odometry, publisher):
        """Convert robot odometry to Marker msg"""
        (x, y) = (odometry.pose.pose.position.x, odometry.pose.pose.position.y)
        h = std_msgs.msg.Header()
        #tie marker visualization to source
        h.frame_id = odometry.header.frame_id
        # Note you need to call rospy.init_node() before this will work
        h.stamp = rospy.Time.now() 
        #publish marker:person_marker, modify a red cylinder, last indefinitely
        mark = Marker()
        mark.header=h
        mark.ns="cleaner_bot"
        mark.id=0
        mark.type=3
        mark.action=0
        mark.pose=geometry_msgs.msg.Pose(geometry_msgs.msg.Point(x, y, self.robot_height/2),
                                         odometry.pose.pose.orientation)
        mark.scale=geometry_msgs.msg.Vector3(self.robot_diam, self.robot_diam,
                                             self.robot_height)
        mark.color=ColorRGBA(0, 0, 1, 1) #marker is blue
        publisher.publish(mark)

    def setup(self):
        rospy.init_node('cleaner_bot', anonymous=True)
        publisher=rospy.Publisher('cleaner_bot', Marker, queue_size=10)
        rospy.Subscriber('odom', Odometry, self.clean, publisher)
        rospy.spin()
        #subscribe to tf: odom base_link to map?

if __name__ == '__main__':
    node = CleanerBot()
    try:
        node.setup()
    except rospy.ROSInterruptException:
        pass
