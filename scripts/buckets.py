#!/usr/bin/env python

import rospy
import numpy
from std_msgs.msg import Header
from geometry_msgs.msg import Point32
from sensor_msgs.msg import LaserScan, PointCloud
from sklearn.cluster import DBSCAN
from math import sin, cos

class Buckets:
    def __init__ (self):
        pass

    def fill_buckets(self, data, publisher):
        threshold = 2
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
        points = numpy.array(points)
        #print points
        db = DBSCAN(eps=0.5, min_samples=3).fit(points)
        #core_samples_mask = numpy.zeros_like(db.labels_, dtype=bool)
        #core_samples_mask[db.core_sample_indices_] = True
        labels = db.labels_
        #return points, labels
        # Number of clusters in labels, ignoring noise if present.
        n_clusters = len(set(labels)) - (1 if -1 in db.labels_ else 0)
        print n_clusters
        for n in xrange(n_clusters):
            xy = points[labels==n]
            cloud=PointCloud()
            cloud.header=Header(stamp=rospy.Time.now(), frame_id="laser")
            cloud.points = [Point32(p[0], p[1], 0) for p in xy]
            publisher.publish(cloud)

    def setup(self):
        rospy.init_node("bucket_test", anonymous=True)
        #listen to filtered scan topic
        pub = rospy.Publisher("buckets", PointCloud, queue_size=10)
        rospy.Subscriber("filtered_scan", LaserScan, self.fill_buckets, pub)
        #spin until node is closed
        rospy.spin()

if __name__ == '__main__':
    node = Buckets()
    try:
        node.setup()
    except rospy.ROSInterruptException:
        pass