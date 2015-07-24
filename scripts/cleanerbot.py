#!/usr/bin/env python

#allows a robot to clean up modeled contamination

import numpy as np
import rospy
import tf
from std_msgs.msg import *
from nav_msgs.msg import *
from geometry_msgs.msg import *
from visualization_msgs.msg import 

class CleanerBot:
    def __init__(self):
        self.robot_radius=0.17575 #value in meters for turtlebot - can it be parameterized?
        
        self.ogrid = OccupancyGrid()
        self.step = 0

    def _set_map_metadata(self, metadata):
        self.ogrid.info = metadata
        self.ogrid.data = [0 for i in xrange(metadata.width*metadata.height)]
        self.step = metadata.resolution

    def _set_grid(self, data):
        self.ogrid.data = data

    def _xy_to_cell(self, x, y):
        #fit XY to nearest cell - each cell is <resolution> meters wide
        x=int(round(x/self.ogrid.info.resolution))
        y=int(round(y/self.ogrid.info.resolution))
        return y*self.ogrid.info.width+x

    def _get_circle(self, x, y):
        circle_pts = []
        for i in np.arange(x-self.robot_radius, y+self.robot_radius, self.step):
            for j in np.arange(x-self.robot_radius, y+self.robot_radius, self.step):
                if (i-x)**2 + (j-y)**2 <= self.robot_radius**2:
                    circle_pts.append(self._xy_to_cell(i, j))
        return circle_pts

    def clean(self, odometry, publisher):
        #turn robot into cleaning
        (x, y, z) = odometry.pose.pose.position.x, odometry.pose.pose.position.y, odometry.pose.pose.position.z
        circle = self._get_circle(x, y)
        for c in circle:
            self.ogrid.data[c] -= int(self.ogrid.data[c]*self.power)
            if self.ogrid.data[c] < 0:
                self.ogrid.data[c] = 0
        publisher.publish(self.ogrid)

    def setup(self):
        rospy.init_node('cleaner_bot', anonymous=True)
        metadata = rospy.wait_for_message("map_metadata", MapMetaData, 120)
        self._set_map_metadata(metadata)
        publisher=rospy.Publisher('cleaner_grid', OccupancyGrid, queue_size=10, latch=True)
        rospy.Subscriber('contamination_grid', OccupancyGrid, self._set_grid)
        rospy.Subscriber('odom', Odometry, self.clean, publisher)
        rospy.spin()
        #subscribe to tf: odom base_link to map

if __name__ == '__main__':
    node = CleanerBot()
    try:
        node.setup()
    except rospy.ROSInterruptException:
        pass
