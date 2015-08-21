#!/usr/bin/env python

import numpy as np
import sys
import re
from math import cos, sin, acos

import yaml

import rospy
import tf
import geometry_msgs.msg
from std_msgs.msg import *
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
from visualization_msgs.msg import Marker, MarkerArray

from contamination_points import ContaminationPC2

class ContaminationPC2_Multi(ContaminationPC2):
    def __init__(self):
        ContaminationPC2.__init__(self)
        self.contam_level=[]
        self.layout = MultiArrayLayout([MultiArrayDimension(label="contam", stride=1)], 0)

    def reset(self, run=True):
        """Empty contamination cloud"""
        ContaminationPC2.reset(self, run)
        self.contam_level=[]

    def _check_contam2(self, ellipse_array):
        """Compare ellipse location to map and modify contamination

        This function takes a ROS Marker as input and publishes
        the contamination cloud as output.
        If the marker represents a cleaning robot, remove contamination
        under the robot at a rate corresponding to power
        If points in contact with the ellipse are infected, the ellipse
        becomes infected as well.
        If the ellipse is currently infected, it spreads infection to
        all the points it touches.
        Keyword arguments:
        ellipse -- ellipse-shaped ROS Marker
        is_cleaner -- True if the marker represents a cleaning robot,
                      False if the marker represents a human
        """
        #count_empties = 0
        while len(self.contam_level) < len(ellipse_array.markers):
            self.contam_level.append(0)
        for ellipse in ellipse_array.markers:
            (center, a, b, theta)=self._get_ellipse_data(ellipse) #convert marker to points
            for k in self.contam:
                distance = self._e_dist(k, center, a, b, theta)
                if distance < 1.1:
                    try:
                        z, contam = zip(*self.contam[k])
                        max_c = max(contam)
                        # if person is in area increase relative contamination
                        #print center
                        if self.contam_level[ellipse.id]<max_c:
                            self.contam_level[ellipse.id]=max_c*self.infectivity
                            #print str(ellipse.id)+' is infected now!'
                        elif self.contam_level[ellipse.id] > 0:
                            x = self.contam_level[ellipse.id]*self.transfer
                            contam=[c+x if c+x < 100 else 100 for c in contam]
                            #contam=[c+x for c in contam]
                            self.contam[k] = zip(z, contam)
                            #print str(ellipse.id)+' is making a mess now!'
                    except ValueError:
                        pass
                    #count_empties += 1
            if self.contam_level[ellipse.id] > 0: self.contam_level[ellipse.id]*=(1-self.transfer) #reduce relative contamination
            #if count_empties > 0: print count_empties
        header = Header(stamp=rospy.Time.now(),frame_id = "/map")
        points_list = []
        #count = 0
        for k in self.contam:
            for v in self.contam[k]:
                if v[1] >= self.min_val:
                    #count+=1
                    points_list.append([k[0], k[1], v[0], v[1]])
        cloud = point_cloud2.create_cloud(header, self.fields, points_list)
        self.publisher.publish(cloud)
        #publish contamination to show correct colors
        self.layout.dim[0].size = len(self.contam_level)
        self.contam_pub.publish(Float32MultiArray(self.layout, self.contam_level))

    def _clean_contam(self, ellipse):
        """Remove contamination under robot at rate defined by power"""
        #call parent method
        ContaminationPC2._check_contam(self, ellipse, True)

    def setup(self):
        """Initialize node, publishers, and subscribers"""
        rospy.init_node('contamination2', anonymous=True)
        point_cloud=rospy.wait_for_message("octomap_point_cloud_centers", PointCloud2, 120)
        self._set_pc(point_cloud)
        self.publisher=rospy.Publisher("contamination_points", PointCloud2, queue_size=10, latch=True)
        self.contam_pub=rospy.Publisher("contam_array", Float32MultiArray, queue_size=10)
        rospy.Subscriber("cleaner_bot", Marker, self._clean_contam)
        rospy.Subscriber("rviz_selected_points", PointCloud2, self._init_contam)
        rospy.Subscriber("tracker_array", MarkerArray, self._check_contam2)
        rospy.Subscriber("update_filter_cmd", Bool, self.reset)
        self.listener = tf.TransformListener()
        self.reset(True)
        rospy.spin()
        # with open("/home/tiffany/contamination_pointcloud2.xyz", "w") as f:
        #     for k, v in self.contam.iteritems():
        #         for val in v:
        #             if val[1] > self.min_val: f.write("{: 6.3f} {: 6.3f} {: 6.3f} {: 6.3f}\n".format(k[0], k[1], val[0], val[1]))

if __name__ == '__main__':
    node = ContaminationPC2_Multi()
    try:
        node.setup()
    except rospy.ROSInterruptException: 
        pass