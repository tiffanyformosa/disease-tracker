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
    UP = Quaternion(0, 0, 0.7071, 0.7071)
    DOWN = Quaternion(0, 0, -0.7071, 0.7071)
    LEFT = Quaternion(0, 0, 1, 0)
    RIGHT = Quaternion(0, 0, 0, 1)

    def __init__(self):
        self.goals = deque()
        self.offset = (0, 0)
        self.map = None
        self.listener = None
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.seq = -1
        self.step = 0.2

    def extract_points(self, ogrid):
        points = []
        #print numpy.asarray(ogrid.data)
        for i in xrange(len(ogrid.data)):
            #if there is contamination in an empty space, add point to list
            if ogrid.data[i] > 0 and self.map[i] == 0:
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
        center = ((min_x+max_x)/2, (min_y+max_y)/2)
        #print min_x, min_y, max_x, max_y, center
        return [min_x, min_y, max_x, max_y, center]
    
    def _set_map(self, new_map):
        #0.177 = robot radius - parameterize so it can source from same place as cleanerbot?
        self.offset = (new_map.info.origin.position.x, new_map.info.origin.position.y)
        self.map = new_map.data

    def _set_goal(self, header, x, y, orientation):
        pose = PoseStamped(header, Pose(Point(x, y, 0), orientation))
        goal = MoveBaseGoal()
        goal.target_pose = pose
        return goal

    def _set_cluster_goals(self, points, data, corner):
        #data format: (min_x, min_y, max_x, max_y, center)
        #print data
        vert, horiz = corner.split()
        g = []
        if vert == 'top': #start from top
            goal_range = numpy.arange(data[3], data[1], 0-self.step)
        elif vert == 'bottom': #start from bottom
            goal_range = numpy.arange(data[1], data[3], self.step)
        for y in goal_range:
            point_slice = []
            for p in points:
                if abs(y-p[1]) <= self.step: point_slice.append(p)
            min_x, min_y = numpy.min(point_slice, axis=0)
            max_x, max_y = numpy.max(point_slice, axis=0)
            header = Header(self.seq, rospy.Time.now(), '/map')
            if horiz == 'left':
                g.append(self._set_goal(header, min_x, y, self.RIGHT))
                g.append(self._set_goal(header, max_x, y, self.LEFT))
            elif horiz == 'right': #start from right
                g.append(self._set_goal(header, max_x, y, self.LEFT))
                g.append(self._set_goal(header, min_x, y, self.RIGHT))
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
            next_point = points.pop(index)
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
        #group contam_points into boxes, prioritizing less dirty areas first
        points = self.extract_points(ogrid)
        if points.size == 0:
            rospy.signal_shutdown("Cleaning complete. Program will now exit.")
        else:
            db = DBSCAN(eps=0.5, min_samples=10).fit(points)
        # Number of clusters in labels, ignoring noise if present.
        n_clusters = len(set(db.labels_)) - (1 if -1 in db.labels_ else 0)
        print n_clusters
        cluster_data = []
        if n_clusters >= 1:
            for n in xrange(n_clusters):
                xy = points[db.labels_==n]
                #print "point {0}".format(n)
                cluster_data.append(self.bounding_box(xy))
            order = self._get_order([x[4] for x in cluster_data])
            self.seq += 1
            self.goals.extend(self._set_cluster_goals(points[db.labels_==order[0][0]].tolist(), cluster_data[order[0][0]], order[0][1]))
            # for o in order:
            #     self.goals.extend(self._set_cluster_goals(cluster_data[o[0]], o[1]))
            #     self.seq += 1
        # elif n_clusters == 0 and -1 in db.labels_:
        #     xy = points[db.labels_== -1]
        #     for point in xy:
        #         header = Header(self.seq, rospy.Time.now(), '/map')
        #         self.seq += 1
        #         pose = PoseStamped(header, Pose(Point(point[0], point[1], 0), RIGHT))
        #         goal = MoveBaseGoal()
        #         goal.target_pose = pose
        #         self.goals.append(goal)
        else:
            rospy.signal_shutdown("Cleaning complete. Program will now exit.")

#send next goal if previous one has completed/terminated
    def send_goal(self):
        self.client.wait_for_server()
        goal = self.goals.popleft()
        try:
            print "Sending Goal"
            self.client.send_goal(goal)
        except IndexError:
            print "Oops! No more goals!"
            return
        self.client.wait_for_result()
        state = self.client.get_state()
        count = 0
        while state == 4 and count < 3: #if server rejects goal, try another goal a little closer
            print "Goal failed. Trying again."
            pose = goal.target_pose.pose
            if pose.orientation == self.LEFT: pose.position.x -= self.step
            elif pose.orientation == self.RIGHT: pose.position.x += self.step
            goal.target_pose.pose = pose
            self.client.send_goal(goal)
            self.client.wait_for_result()
            state = self.client.get_state()
            count++

    def setup(self):
        rospy.init_node('path_planner', anonymous=True)
        base_map = rospy.wait_for_message("map", OccupancyGrid)
        self._set_map(base_map)
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