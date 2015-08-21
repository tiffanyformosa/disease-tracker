#!/usr/bin/env python

from collections import *
from math import sqrt, cos

import numpy
import yaml
from sklearn.cluster import KMeans

import rospy
import tf
import actionlib
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
        self.resolution = 0
        self.map = None
        self.listener = None
        self.client = actionlib.SimpleActionClient('move_base',
                                                   MoveBaseAction)
        self.seq = -1
        self.step = 0.2

    def extract_points(self, ogrid):
        """Convert OccupancyGrid to numpy array of occupied points"""
        points = []
        #print numpy.asarray(ogrid.data)
        for i in xrange(len(ogrid.data)):
            #if there is contamination in an empty space, add point to list
            if ogrid.data[i] > 0 and self.map[i] == 0:
                newp = [j*ogrid.info.resolution for j
                        in divmod(i, ogrid.info.width)]
                newp.reverse()
                points.append(newp)
        return numpy.asarray(points)

    def bounding_box(self, points):
        """Draw a bounding box around points and find center

        Returns list of integers in form [min_x, min_y, max_x, max_y, center]
        Keyword arguments:
        points -- numpy list of x, y, points
        """
        min_x, min_y = numpy.min(points, axis=0)
        max_x, max_y = numpy.max(points, axis=0)
        min_x += self.offset[0]
        max_x += self.offset[0]
        min_y += self.offset[1]
        max_y += self.offset[1]
        center = ((min_x+max_x)/2, (min_y+max_y)/2)
        #print min_x, min_y, max_x, max_y, center
        return [min_x, min_y, max_x, max_y, center]
    
    def _set_map(self, new_map, costmap):
        """Set a map to exclude points that are too close to obstacles

        Keyword arguments:
        new_map -- ROS OccupancyGrid message for map of room
        costmap -- ROS OccupancyGrid message for global costmap of room
        """
        self.offset = (new_map.info.origin.position.x,
                       new_map.info.origin.position.y)
        self.resolution = new_map.info.resolution
        m = []
        for i in xrange(len(new_map.data)):
            if (new_map.data[i] > -1 and new_map.data[i] < costmap.data[i]
                                     and costmap.data[i] >= 50): 
                m.append(costmap.data[i])
            else: m.append(new_map.data[i])
        self.map = m

    def _set_goal(self, header, x, y, orientation):
        """Create a MoveBaseGoal from input and return the message

        Keyword arguments:
        header -- ROS Header message
        x -- x coordinate, in meters, of desired location
        y -- y coordinate, in meters, of desired location
        orientation -- direction for the robot to face at desired goal
        """
        pose = PoseStamped(header, Pose(Point(x, y, 0), orientation))
        goal = MoveBaseGoal()
        goal.target_pose = pose
        return goal

    def _set_cluster_goals(self, points, bbox, corner):
        #bbox format: (min_x, min_y, max_x, max_y, center)
        """Create a list of MoveBaseGoals for a cluster and return

        This function creates a list of MoveBaseGoals to send a robot
        in horizontal zigzags across a contamination cluster to clean it.
        Keyword arguments:
        points -- a numpy array of all the points in a given cluster
        bbox -- the bounding box that surrounds those points.
        corner -- which direction to approach the cluster from:
                  a string with format "top/bottom left/right"
        """
        vert, horiz = corner.split()
        g = []
        if vert == 'top': #start from top
            goal_range = numpy.arange(bbox[3], bbox[1], 0-self.step)
        elif vert == 'bottom': #start from bottom
            goal_range = numpy.arange(bbox[1], bbox[3], self.step)
        for y in goal_range:
            point_slice = []
            for p in points:
                if abs(y-p[1]) <= self.step: point_slice.append(p)
            try:
                min_x, min_y = numpy.min(point_slice, axis=0)
                max_x, max_y = numpy.max(point_slice, axis=0)
            except ValueError:
                continue
            header = Header(self.seq, rospy.Time.now(), '/map')
            if horiz == 'left':
                g.append(self._set_goal(header, min_x, y, self.RIGHT))
                g.append(self._set_goal(header, max_x, y, self.LEFT))
            elif horiz == 'right': #start from right
                g.append(self._set_goal(header, max_x, y, self.LEFT))
                g.append(self._set_goal(header, min_x, y, self.RIGHT))
        return g

    def _dist(self, a, b):
        """Take the distance between points a and b and return"""
        #print a, b
        return sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)

    def _nearest_neighbor(self, base_point, points):
        """Find the nearest neighbor to the base_point and return

        This function is used to get the closest cluster to the robot,
        determine cluster ID and the best corner to approach from,
        and return index and direction values. Previous recursive functionality
        has been removed.
        Keyword arguments:
        base_point -- the location of the robot
        points -- a list of (x, y) tuples representing the centers of clusters
        """
        if len(points) == 0:
            return []
        else:
            temp = numpy.inf
            index = -1
            for p in xrange(len(points)):
                dist = self._dist(base_point, points[p])
                if dist < temp:
                    temp = dist
                    index = p
            next_point = points.pop(index)
            direction = None
            if base_point[0] < next_point[0]:
                if base_point[1] < next_point[1]:
                    direction = "bottom left"
                else: direction = "top left"
            elif base_point[1] < next_point[1]:
                direction = "bottom right"
            else: direction = "top right"
            return index, direction

    def set_path(self, ogrid):
        """Use the contaminated points in an OccupancyGrid to construct a path

        This function calls other functions to create a path to drive the 
        robot across areas of contamination to clean it. It clusters the
        contaminated points, finds the best cluster to send the robot to,
        and adds goals to the queue to make the robot drive across the cluster.

        Keyword arguments:
        ogrid -- a ROS OccupancyGrid message depicting contaminated areas
        """
        points = self.extract_points(ogrid)
        if points.size==0:
            rospy.signal_shutdown("Cleaning complete. Program will now exit.")
            return
        else:
            #1 cluster ~= 1 sq. meter
            n_clusters = int(round(points.size/2*(self.resolution**2))) 
        print n_clusters
        cluster_data = []
        if n_clusters >= 1:
            kmeans = KMeans(n_clusters).fit(points)
            for n in xrange(n_clusters):
                xy = points[kmeans.labels_==n]
                #print "point {0}".format(n)
                cluster_data.append(self.bounding_box(xy))
            robot = rospy.wait_for_message("amcl_pose", 
                                            PoseWithCovarianceStamped)
            loc = robot.pose.pose.position
            centers = [c[4] for c in cluster_data]
            #print points
            pid, direction = self._nearest_neighbor((loc.x, loc.y), centers)
            self.seq += 1
            new_goals = self._set_cluster_goals(points[kmeans.labels_==pid].tolist(),
                                                cluster_data[pid], direction)
            self.goals.extend(new_goals)
        else:
            rospy.signal_shutdown("Cleaning complete. Program will now exit.")

    def send_goal(self):
        """send a goal if previous one has completed/terminated"""
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
        #if server rejects goal, try another goal a little closer
        while state == 4 and count < 3: 
            print "Goal failed. Trying again."
            pose = goal.target_pose.pose
            if pose.orientation == self.LEFT: pose.position.x -= self.step
            elif pose.orientation == self.RIGHT: pose.position.x += self.step
            goal.target_pose.pose = pose
            self.client.send_goal(goal)
            self.client.wait_for_result()
            state = self.client.get_state()
            count+=1

    def setup(self):
        """Initialize node and contained client; receive map messages"""
        rospy.init_node('path_planner', anonymous=True)
        base_map = rospy.wait_for_message("map", OccupancyGrid)
        costmap = rospy.wait_for_message("/move_base/global_costmap/costmap",
                                         OccupancyGrid)
        self._set_map(base_map,costmap)
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            #if there is a goal, send something, else try to get new goals
            if self.goals: self.send_goal()
            else:
                msg = rospy.wait_for_message("contamination_grid",
                                             OccupancyGrid)
                self.set_path(msg)
            #print "Looping"
            r.sleep()


if __name__ == '__main__':
    node = PathPlanner()
    try:
        node.setup()
    except rospy.ROSInterruptException:
        pass