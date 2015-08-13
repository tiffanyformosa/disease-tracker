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
from visualization_msgs.msg import Marker

class ContaminationPC2:
    def __init__(self):
        self.contam_level=0
        self.listener=None
        self.publisher=None
        self.contam_pub=None
        #Efficiency of cleaning robot
        #self.power = 0.0
        self.min_val = 0.01
        #Contaminant picked up from environment
        self.infectivity = 1
        #Contaminant transfered upon moving
        self.transfer = 0.1
        #list of coordinates with contamination: coordinates are Float32, contam is UInt8
        self.fields = [PointField('x',0,7,1), PointField('y',4,7,1), PointField('z',8,7,1),
                       PointField('contam',12,2,1)]
        self.contam={}

    def _set_pc(self, pc):
        point_gen = point_cloud2.read_points(cloud=pc, skip_nans=True)
        while True:
            try:
                point = point_gen.next()
                key = (point[0], point[1])
                value = point[2]
                if key in self.contam: self.contam[key].append([value, 0])
                else: self.contam[key] = [[value, 0]]
            except StopIteration:
                break
        # count_empties = 0
        # for k in self.contam:
        #     if self.contam[k] == []: count_empties += 1
        # print count_empties

    def _init_contam(self, pc):
        point_gen = point_cloud2.read_points(cloud=pc, skip_nans=True)
        points_list = []
        while True:
            try:
                point = point_gen.next()
                key = (point[0], point[1])
                if key in self.contam:
                    z, contam = zip(*self.contam[key])
                    index = z.index(point[2])
                    self.contam[key][index][1]=100.0
            except StopIteration:
                break
        for k, v in self.contam.iteritems():
            for val in v:
                if val[1] > self.min_val: points_list.append([k[0], k[1], val[0], val[1]])
        header = Header(stamp=rospy.Time.now(),frame_id = "/map")
        cloud = point_cloud2.create_cloud(header, self.fields, points_list)
        self.publisher.publish(cloud)

    def reset(self, run):
        self.contam_level = 0
        self.contam={}
        point_cloud=rospy.wait_for_message("octomap_point_cloud_centers", PointCloud2, 120)
        self._set_pc(point_cloud)
        header = Header(stamp=rospy.Time.now(),frame_id = "/map")
        cloud = point_cloud2.create_cloud(header, self.fields, [])
        self.publisher.publish(cloud)
        with open(sys.argv[1]) as f:
            for k, v in yaml.load(f.read()).iteritems():
                if k == "transfer":
                    self.transfer = v
                    print "transfer = {0}".format(v)
                elif k == "infectivity":
                    self.infectivity = v
                    print "infectivity = {0}".format(v)
                elif k == "cleaning_power":
                    self.power = v
                    print "cleaning power = {0}".format(v)

    #turn ellipse into points
    def _get_ellipse_data(self,ellipse):
        #transform from ellipse frame to map frame and get ellipse data
        #FORMULA ASSUMES LASER AND MAP ARE ROTATED ONLY AROUND Z AXIS
        try:
            (trans,rot) = self.listener.lookupTransform('/map', '/'+ellipse.header.frame_id, rospy.Time(0))
            x = ellipse.pose.position.x
            y = ellipse.pose.position.y
            angle = acos(rot[3])*2
            center = (x*cos(angle)-y*sin(angle)+trans[0], x*sin(angle)+y*cos(angle)+trans[1])
            (a, b) = (ellipse.scale.x/2, ellipse.scale.y/2)
            q = ellipse.pose.orientation
            w =(q.w*rot[3]-q.x*rot[0]-q.y*rot[1]-q.z*rot[2])
            theta = acos(w)*2
            return center, a, b, theta
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

    #return distance from point to ellipse
    def _e_dist(self, point, center, a, b, theta):
        return (pow((abs(cos(theta)*(point[0]-center[0]))+abs(sin(theta)*(point[1]-center[1])))/a, 2) +
                pow((abs(sin(theta)*(point[0]-center[0]))+abs(cos(theta)*(point[1]-center[1])))/b, 2))

    def _check_contam(self, ellipse):
        #check to see if they have become contaminated
        (center, a, b, theta)=self._get_ellipse_data(ellipse) #convert marker to points
        points_list = []
        #print center
        count_empties = 0
        for k in self.contam:
            distance = self._e_dist(k, center, a, b, theta)
            try:
                z, contam = zip(*self.contam[k])
                max_c = max(contam)
                # if person is in area increase relative contamination
                if distance < 1.1:
                    if self.contam_level<max_c:
                        self.contam_level=max_c*self.infectivity
                        #print 'I\'m infected now!'
                    elif self.contam_level > 0:
                        contam=[c+(self.contam_level*self.transfer) if c+(self.contam_level*self.transfer) < 100 else 100 for c in contam]
                        self.contam[k] = zip(z, contam)
            except ValueError:
                count_empties += 1
            for v in self.contam[k]:
                if v[1] >= self.min_val: points_list.append([k[0], k[1], v[0], v[1]])
        if self.contam_level > 0: self.contam_level*=(1-self.transfer) #reduce relative contamination
        if count_empties > 0: print count_empties
        header = Header(stamp=rospy.Time.now(),frame_id = "/map")
        cloud = point_cloud2.create_cloud(header, self.fields, points_list)
        self.publisher.publish(cloud)
        self.contam_pub.publish(self.contam_level)

    def setup(self):
        rospy.init_node('contamination', anonymous=True)
        point_cloud=rospy.wait_for_message("octomap_point_cloud_centers", PointCloud2, 120)
        self._set_pc(point_cloud)
        self.publisher=rospy.Publisher("contamination_points", PointCloud2, queue_size=10, latch=True)
        self.contam_pub=rospy.Publisher("contam", Float32, queue_size=10)
        #rospy.Subscriber("cleaner_bot", Marker, self._clean_contam)
        rospy.Subscriber("rviz_selected_points", PointCloud2, self._init_contam)
        rospy.Subscriber("tracker", Marker, self._check_contam)
        rospy.Subscriber("update_filter_cmd", Bool, self.reset)
        self.listener = tf.TransformListener()
        rate = rospy.Rate(10.0)
        self.reset(True)
        rospy.spin()
        with open("/home/tiffany/contamination_pointcloud2.xyz", "w") as f:
            for k, v in self.contam.iteritems():
                for val in v:
                    if val[1] > self.min_val: f.write("{: 6.3f} {: 6.3f} {: 6.3f} {: 6.3f}\n".format(k[0], k[1], val[0], val[1]))

if __name__ == '__main__':
    node = ContaminationPC2()
    try:
        node.setup()
    except rospy.ROSInterruptException: 
        pass