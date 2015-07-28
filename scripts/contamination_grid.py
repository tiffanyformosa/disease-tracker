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
from nav_msgs.msg import OccupancyGrid, MapMetaData
from visualization_msgs.msg import Marker

#This class converts ellipse data into an OccupancyGrid message

class Contamination:
    def __init__(self):
        self.contam_level=-1
        self.listener=None
        self.publisher=None
        self.contam_pub=None
        self.ogrid = OccupancyGrid()
        self.step = None
        #Efficiency of cleaning robot
        self.power = 0.0
        #Contaminant picked up from environment
        self.infectivity = 0.0
        #Contaminant transfered upon moving
        self.transfer = 0.0
        #list of coordinates with contamination
        self.contam={}

    def _xy_to_cell(self, xy):
        #fit XY to nearest cell - each cell is <resolution> meters wide
        x=int(round(xy[0]/self.ogrid.info.resolution))
        y=int(round(xy[1]/self.ogrid.info.resolution))
        return y*self.ogrid.info.width+x
        #return x, y

    def _snap_to_cell(self, xy):
        return (round(xy[0]/self.ogrid.info.resolution) * self.ogrid.info.resolution,
                round(xy[1]/self.ogrid.info.resolution) * self.ogrid.info.resolution)

    def _set_map_metadata(self, metadata):
        self.ogrid.info = metadata
        self.ogrid.data = [0 for i in xrange(metadata.width*metadata.height)]
        self.step = metadata.resolution

    #add initial contamination (rectangles)
    def _base_contam(self, lower_left, upper_right, intensity):
        lower_left = self._snap_to_cell(lower_left)
        upper_right = self._snap_to_cell(upper_right)
        for x in np.arange(lower_left[0], upper_right[0], self.step):
            for y in np.arange(lower_left[1], upper_right[1], self.step):
                index = self._xy_to_cell((x, y))
                self.ogrid.data[index]=intensity
                self.contam[index]=(x, y)
        self.ogrid.header=Header(stamp=rospy.Time.now(),frame_id = "map")
        self.publisher.publish(self.ogrid)
        #print "loaded map"

    #turn ellipse into points
    def _get_ellipse_data(self,ellipse):
        #transform from ellipse frame to map frame and get ellipse data
        try:
            (trans,rot) = self.listener.lookupTransform('/map', '/'+ellipse.header.frame_id, rospy.Time(0))
            center = self._snap_to_cell((ellipse.pose.position.x+trans[0], ellipse.pose.position.y+trans[1]))
            (a, b) = self._snap_to_cell((ellipse.scale.x/2, ellipse.scale.y/2))
            theta = acos(ellipse.pose.orientation.w)*2
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
        for k, v in self.contam.iteritems():
            distance = self._e_dist(v, center, a, b, theta)
            # if person is in area increase relative contamination
            if distance < 1 and self.contam_level<self.ogrid.data[k]:
                self.contam_level = self.ogrid.data[k]*self.infectivity
                #print ellipse.id, self.contam_level[ellipse.id]
                self.ogrid.data[k] = int(self.ogrid.data[k]*(1-0.5*self.infectivity))
        #if person is contaminated after that check, amend contamination levels in points
        if self.contam_level > -1:
            #contamination levels decrease as person distributes sickness around
            self.contam_level = self.contam_level*self.rate
            #outline square that fits ellipse and then find points within ellipse
            for x in np.arange(center[0]-a, center[0]+a, self.step):
                for y in np.arange(center[1]-a, center[1]+a, self.step):
                    distance = self._e_dist((x, y), center, a, b, theta)
                    index = self._xy_to_cell((x, y))
                    if distance < 1:
                        self.ogrid.data[index]+=int(ceil(self.contam_level[ellipse.id]*self.transfer))
                        if self.ogrid.data[index] > 100:
                            self.ogrid.data[index] = 100
                        if index not in self.contam:
                            self.contam[index]=(x, y)
        self.ogrid.header=Header(stamp=rospy.Time.now(),frame_id = "map")
        self.contam_pub.publish(self.contam_level)
        self.publisher.publish(self.ogrid)

    def reset(self, run):
        self.contam_level = -1
        self.contam={}
        self.ogrid.data = [0 for i in xrange(self.ogrid.info.width*self.ogrid.info.height)]
        print sys.argv
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
                elif re.match("c[0-9]+", k):
                    self._base_contam(v["lower_left"], v["upper_right"], v["intensity"])

    def _clean_contam(self, ellipse):
        (center, a, b, theta)=self._get_ellipse_data(ellipse)
        #print center, a, b, theta
        for x in np.arange(center[0]-a, center[0]+a, self.step):
            for y in np.arange(center[1]-a, center[1]+a, self.step):
                distance = self._e_dist((x, y), center, a, b, theta)
                index = self._xy_to_cell((x, y))
                #if area within ellipse, remove disease
                if distance < 1:
                    self.ogrid.data[index]*=(1-self.power)
                    if self.ogrid.data[index] > 100: self.ogrid.data[index] = 100
                    elif self.ogrid.data[index] <= 0:
                        self.ogrid.data[index]=0
                        if index in self.contam: del self.contam[index]
                    #print self.ogrid.data[index]
        self.ogrid.header=Header(stamp=rospy.Time.now(),frame_id = "map")
        self.publisher.publish(self.ogrid)

    #initialize node
    def setup(self):
        rospy.init_node('contamination', anonymous=True)
        metadata = rospy.wait_for_message("map_metadata", MapMetaData, 120)
        self._set_map_metadata(metadata)
        self.publisher=rospy.Publisher("contamination_grid", OccupancyGrid, queue_size=10, latch=True)
        self.contam_pub=rospy.Publisher("contam", Float32, queue_size=10)
        rospy.Subscriber("cleaner_bot", Marker, self._clean_contam)
        rospy.Subscriber("tracker", Marker, self._check_contam)
        rospy.Subscriber("update_filter_cmd", Bool, self.reset)
        self.listener = tf.TransformListener()
        rate = rospy.Rate(10.0)
        self.reset(True)
        rospy.spin()

if __name__ == '__main__':
    node = Contamination()
    try:
        node.setup()
    except rospy.ROSInterruptException:
        pass