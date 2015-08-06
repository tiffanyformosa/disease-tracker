#!/usr/bin/env python

from collections import deque

import rospy
import numpy
import tf
import actionlib
import yaml
from math import sqrt, cos
from collections import *
from sklearn.cluster import DBSCAN

from std_msgs.msg import *
from geometry_msgs.msg import *
from move_base_msgs.msg import *
from nav_msgs.msg import OccupancyGrid, Odometry, MapMetaData


class PathPlanner:
    def __init__(self):
        self.goals = deque()
        self.walls = None
        self.offset = (0, 0)
        self.listener = None
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.seq = 0
        self.step = 0.2
        self.up = Quaternion(0, 0, 0.7071, 0.7071)
        self.down = Quaternion(0, 0, -0.7071, 0.7071)
        self.left = Quaternion(0, 0, 1, 0)
        self.right = Quaternion(0, 0, 0, 1)

    def extract_points(self, ogrid):
        points = []
        #print numpy.asarray(ogrid.data)
        for i in xrange(len(ogrid.data)):
            if ogrid.data[i] > 0:
                newp = [j*ogrid.info.resolution for j in divmod(i, ogrid.info.width)]
                newp.reverse()
                points.append(newp)
        return numpy.asarray(points)

    def bounding_box(self, points):
        min_x, min_y = numpy.min(points, axis=0)
        max_x, max_y = numpy.max(points, axis=0)
        min_x += self.offset[0]
        max_x += self.offset[0]
        min_y += self.offset[1]
        max_y += self.offset[1]
        #print min_x, min_y, max_x, max_y
        #avoid collisions
        if self.walls:
            if min_x < self.walls[0]:
                min_x = self.walls[0]+self.step
            if min_y < self.walls[1]:
                min_y = self.walls[1]+self.step
            if max_x > self.walls[2]:
                max_x = self.walls[2]-self.step
            if max_y > self.walls[3]:
                max_y = self.walls[3]-self.step  
        center = ((min_x+max_x)/2, (min_y+max_y)/2)
        #print min_x, min_y, max_x, max_y, center
        return [min_x, min_y, max_x, max_y, center]
    
    def _set_map(self, new_map, metadata):
        #0.177 = robot radius - parameterize so it can source from same place as cleanerbot?
        self.offset = (metadata.origin.position.x, metadata.origin.position.y)
        points = self.bounding_box(self.extract_points(new_map))
        self.walls = [p+0.1 for p in points[:4]]
        
    def _set_cluster_goals(self, data, corner):
        #data format: (min_x, min_y, max_x, max_y, center)
        #print data
        vert, horiz = corner.split()
        g = []
        if (data[2] - data[0]) < (data[3]-data[1]):
            if vert == 'top': #start from top
                sweep = [data[3], data[1]]
                dirs = [self.down, self.up]
            elif vert == 'bottom': #start from bottom
                sweep = [data[1], data[3]]
                dirs = [self.up, self.down]
            if horiz == 'left': #start from left
                goal_range = numpy.arange(data[0], data[2], self.step)
            elif horiz == 'right': #start from right
                goal_range = numpy.arange(data[2], data[0], 0-self.step)
            #print goal_range, sweep, dirs
            for x in goal_range:
                header = Header(self.seq, rospy.Time.now(), '/map')
                self.seq += 1
                pose = PoseStamped(header, Pose(Point(x, sweep[0], 0), dirs[0]))
                goal = MoveBaseGoal()
                goal.target_pose = pose
                g.append(goal)
                pose = PoseStamped(header, Pose(Point(x, sweep[1], 0), dirs[1]))
                goal = MoveBaseGoal()
                goal.target_pose = pose
                g.append(goal)
        else:
            if vert == 'top': #start from top
                goal_range = numpy.arange(data[3], data[1], 0-self.step)
            elif vert == 'bottom': #start from bottom
                goal_range = numpy.arange(data[1], data[3], self.step)
            if horiz == 'left': #start from left
                sweep = [data[0], data[2]]
                dirs = [self.right, self.left]
            elif horiz == 'right': #start from right
                sweep = [data[2], data[0]]
                dirs = [self.left, self.right]
            #print goal_range, sweep, dirs
            for y in goal_range:
                header = Header(self.seq, rospy.Time.now(), '/map')
                self.seq += 1
                pose = PoseStamped(header, Pose(Point(sweep[0], y, 0), dirs[0]))
                goal = MoveBaseGoal()
                goal.target_pose = pose
                g.append(goal)
                pose = PoseStamped(header, Pose(Point(sweep[1], y, 0), dirs[1]))
                goal = MoveBaseGoal()
                goal.target_pose = pose
                g.append(goal)
        #print g
        return g

    def _dist(self, a, b):
        #print a, b
        return sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)

    #pass it points in order: brute force (recursive?) nearest-neighbor
    #point format: [(point_id, (x, y))]
    def _nearest_neighbor(self, base_point, points):
        if len(points) == 0:
            return []
        else:
            temp = numpy.inf
            index = -1
            for p in xrange(len(points)):
                dist = self._dist(base_point[1], points[p][1])
                #print 'robot_loc: {0} >> cluster{1} loc: {2}: {3}'.format(base_point[1], p, points[p][1], dist)
                if dist < temp:
                    temp = dist
                    index = p
            next_point = points.pop(p)
            direction = None
            if base_point[1][0] < next_point[1][0]:
                if base_point[1][1] < next_point[1][1]:
                    direction = "bottom left"
                else: direction = "top left"
            elif base_point[1][1] < next_point[1][1]: direction = "bottom right"
            else: direction = "top right"
            x = [(next_point[0], direction)]
            x.extend(self._nearest_neighbor(next_point, points))
            return x

    def _get_order(self, centers):
        #determine order of clusters to publish points for
        #centers should be a list of (x, y) tuples
        robot = rospy.wait_for_message("amcl_pose", PoseWithCovarianceStamped)
        robot_loc = robot.pose.pose.position
        points = zip(range(len(centers)), centers)
        #print points
        path = self._nearest_neighbor((None, (robot_loc.x, robot_loc.y)), points)
        #print path
        #point_order, directions = zip(*path)
        return path

    def set_path(self, ogrid):
        points = self.extract_points(ogrid)
        if points.size == 0:
            print "Cleaning complete. Program will now exit."
            return
        #print points
        #group contam_points into boxes
        db = DBSCAN(eps=1.0, min_samples=10).fit(points)
        # Number of clusters in labels, ignoring noise if present.
        n_clusters = len(set(db.labels_)) - (1 if -1 in db.labels_ else 0)
        #print n_clusters
        cluster_data = []
        if n_clusters >= 1:
            for n in xrange(n_clusters):
                xy = points[db.labels_==n]
                #print "point {0}".format(n)
                cluster_data.append(self.bounding_box(xy))
            order = self._get_order([x[4] for x in cluster_data])
            self.goals.extend(self._set_cluster_goals(cluster_data[order[0][0]], order[0][1]))
        # elif n_clusters == 0 and -1 in db.labels_:
        #     xy = points[db.labels_== -1]
        #     for point in xy:
        #         header = Header(self.seq, rospy.Time.now(), '/map')
        #         self.seq += 1
        #         pose = PoseStamped(header, Pose(Point(point[0], point[1], 0), self.right))
        #         goal = MoveBaseGoal()
        #         goal.target_pose = pose
        #         self.goals.append(goal)
        else:
            print "Cleaning complete. Program will now exit."

#send next goal if previous one has completed/terminated
    def send_goal(self):
        self.client.wait_for_server()
        try:
            print "Sending Goal"
            self.client.send_goal(self.goals.popleft())
        except IndexError:
            print "Oops! No more goals!"
            return
        self.client.wait_for_result()
        #return self.client.get_result() 

    def setup(self):
        rospy.init_node('path_planner', anonymous=True)
        base_map = rospy.wait_for_message("map", OccupancyGrid)
        metadata = rospy.wait_for_message("map_metadata", MapMetaData)
        self._set_map(base_map, metadata)
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            if self.goals: self.send_goal() #if there is a goal, send something
            else:
                msg = rospy.wait_for_message("contamination_grid", OccupancyGrid)
                self.set_path(msg)
            #print "Looping"
            r.sleep()


if __name__ == '__main__':
    node = PathPlanner()
    try:
        node.setup()
    except rospy.ROSInterruptException:
        pass