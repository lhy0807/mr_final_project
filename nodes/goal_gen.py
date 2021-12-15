#!/usr/bin/env python

import numpy as np
import rospy
import actionlib

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import MarkerArray

class GoalGen:

    def __init__(self):
        self.frontier_sub = rospy.Subscriber("/explore/frontiers", \
            MarkerArray, self.frontier_cb)
        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.map_cb)
        self.move_base_client = actionlib.SimpleActionClient('move_base', \
            MoveBaseAction)
        self.move_base_client.wait_for_server()
        self.frontier_empty = False
        self.map_resolution = None
        self.map_width = None
        self.map_height = None
        self.map_origin = None
        self.map = None

    def frontier_cb(self, data):
        if not self.frontier_empty:
            if len(data.markers.points) == 0:
                self.frontier_empty = True
        else:
            if not self.map:  # make sure we have a map
                # generate goals
                pass
    
    def map_cb(self, occ_grid):
        if self.frontier_empty:
            self.map_resolution = occ_grid.info.resolution
            self.map_width = occ_grid.info.width
            self.map_height = occ_grid.info.height
            self.map_origin = occ_grid.info.origin
            self.map = np.array(occ_grid.data).reshape((self.map_height, \
                                                        self.map_width))

    def send_random_goal(self):
        pass

if __name__ == "__main__":
    rospy.init_node("goal_gen")
    GoalGen()
    rospy.spin()