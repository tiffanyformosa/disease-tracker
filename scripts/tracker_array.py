#!/usr/bin/env python

import rospy
import numpy
from std_msgs.msg import *
from geometry_msgs.msg import Point, Quaternion, Pose, Vector3
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import LaserScan
from sklearn.cluster import DBSCAN
from sklearn.neighbors import NearestNeighbors
from math import sin, cos, sqrt

from ellipse2d import Ellipse2d

class Ellipse:
    def __init__(self, e=None, axis_a=0.9, center_a=0.1):
        if e is None:
            self.a = self.center = self.b = self.theta = None
        else:
            self.a = e.a
            self.b = e.b
            self.center = e.center
            self.theta = e.theta
        self.last_pos = None
        self.axis_alpha = axis_a
        self.center_alpha = center_a
    def update(self, e):
        if self.a != None and self.b != None and self.center != None:
            self.center = [self.center[i]*self.center_alpha + e.center[i]*(1-self.center_alpha) for i in [0, 1]]
            self.a = self.a*self.axis_alpha + e.a*(1-self.axis_alpha)
            self.b = self.b*self.axis_alpha + e.b*(1-self.axis_alpha)
        else:
            self.a = e.a
            self.b = e.b
            self.center = e.center
        self.theta = e.theta

class Tracker2:
    def __init__ (self):
        self.ellipses = []
        self.e_colors = []
        self.red = ColorRGBA(1, 0, 0, 1)
        self.green = ColorRGBA(0, 1, 0, 1)

    def reset (self, run, pub):
        m=Marker(header=Header(stamp=rospy.Time.now(), frame_id="laser"), ns="person", id=0, type=3, action=3)
        pub.publish(MarkerArray([m]))
        self.ellipses = []
        self.e_colors = []

    def _min_dist(self, dist, ind, sort, ellipse):
        for i in xrange(len(ind[ellipse])):
            k = ind[ellipse, i]
            if k not in sort: #if the ideal position is available
                sort[k] = (ellipse, dist[ellipse, i])
                break
            elif sort[k][1] > dist[ellipse, i]: #if it's taken, but this ellipse is closer
                temp = sort[k][0]
                sort[k] = (ellipse, dist[ellipse, i])
                sort = self._min_dist(dist, ind, sort, temp)
                break
        return sort

    def sort_ellipses(self, new_ellipses):
        if not self.ellipses: #empty list
            self.ellipses=[Ellipse(n) for n in new_ellipses]
            self.e_colors=[self.red for n in new_ellipses]
            return
        oldc = numpy.asarray([e.center for e in self.ellipses])
        #print oldc
        newc = numpy.asarray([e.center for e in new_ellipses])
        #print newc
        neighbors=NearestNeighbors(n_neighbors=3)
        neighbors.fit(oldc)
        dist, ind = neighbors.kneighbors(newc) #ind matches index of oldc to match with
        #print dist, ind #test
        # sort into dict: key=old index, value = (new index, distance)
        sort = {}
        ellipses = range(len(newc))
        for e in ellipses:
            sort = self._min_dist(dist, ind, sort, e)
        for old, new in sort.iteritems():
            self.ellipses[old].update(new_ellipses[new[0]])
            ellipses.remove(new[0])
        #add any remaining ellipses to the list
        if ellipses:
            for e in ellipses:
                self.ellipses.append(Ellipse(new_ellipses[e]))
                self.e_colors.append(self.red)

    def get_colors(self, contamination):
        data = contamination.data
        for c in xrange(len(data)):
            if data[c] < 0.5:
                self.e_colors[c] = self.red
            else:
                self.e_colors[c] = self.green

    #data to markers - does not link markers to past marker
    def pub_markers(self, data, publisher):
        angle = data.angle_min
        incr = data.angle_increment
        max_range = data.range_max
        ranges = data.ranges
        points = []
        for r in ranges:
            #add all valid ranges to some xy range
            if r < max_range:
                points.append([cos(angle)*r, sin(angle)*r])
            angle+=incr
        #eps = range, min_samples = min# of points in cluster.
        points = numpy.asarray(points)
        if len(points) > 3:
            db = DBSCAN(eps=0.5, min_samples=3).fit(points)
        else:
            return
        #core_samples_mask = numpy.zeros_like(db.labels_, dtype=bool)
        #core_samples_mask[db.core_sample_indices_] = True
        labels = db.labels_
        #return points, labels
        # Number of clusters in labels, ignoring noise if present.
        n_clusters = len(set(labels)) - (1 if -1 in db.labels_ else 0)
        new_ellipses = []
        for n in xrange(n_clusters):
            xy = points[labels==n]
            e = Ellipse2d()
            e.fit(xy)
            new_ellipses.append(e)
        #
        # match new ellipses to old ones
        #
        self.sort_ellipses(new_ellipses)
        markers = MarkerArray()
        for e in xrange(len(self.ellipses)):
            m=Marker(ns="person", id=e, type=3, action=0)
            m.header=Header(stamp=rospy.Time.now(), frame_id="laser")
            m.pose=Pose(Point(self.ellipses[e].center[0], self.ellipses[e].center[1], .5),
                        Quaternion(0.0,0.0,1.0,cos(self.ellipses[e].theta/2)))
            m.scale=Vector3(self.ellipses[e].a*2,self.ellipses[e].b*2,1) #scale, in meters
            m.color=self.e_colors[e] #clean = red, infected = green
            markers.markers.append(m)
        #print len(new_ellipses)
        publisher.publish(markers)

    def setup(self):
        rospy.init_node("array_test", anonymous=True)
        #listen to filtered scan topic
        pub = rospy.Publisher("tracker_array", MarkerArray, queue_size=10)
        rospy.Subscriber("filtered_scan", LaserScan, self.pub_markers, pub)
        rospy.Subscriber("contam_array", Float32MultiArray, self.get_colors)
        rospy.Subscriber("update_filter_cmd", Bool, self.reset, pub)
        #spin until node is closed
        rospy.spin()

if __name__ == '__main__':
    node = Tracker2()
    try:
        node.setup()
    except rospy.ROSInterruptException:
        pass