#!/usr/bin/env python

import numpy as np
import actionlib
import rospy
import tf2_ros
import tf2_geometry_msgs
import message_filters

from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import MarkerArray, Marker

from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker

class SimpleGoalState:
    PENDING = 0
    ACTIVE = 1
    DONE = 2

class GoalsTags:

    def __init__(self):
        rospy.sleep(1.)

        # map data
        self.map_resolution = None
        self.map_width = None
        self.map_height = None
        self.map_origin = None
        self.map = None
        self.map_tag_tf = tf2_geometry_msgs.PoseStamped()

        # frontier data (used to decide when to send random commands)
        self.last_frontier_msg_time = rospy.get_time()
        self.time_since_last_frontier_msg = -1
        self.time_diff = 5.0

        num_tags = 20
        self.tag_estimates = [None for _ in range(num_tags)]
        self.tags_sub = message_filters.Subscriber("tag_detections", \
            AprilTagDetectionArray)
        self.vis_pub = rospy.Publisher("visualization_marker", Marker, \
            queue_size=1)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # pubs and subs
        self.vis_pub = rospy.Publisher("visualization_marker", Marker, \
            queue_size=1)
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.frontier_sub = rospy.Subscriber("/explore/frontiers", \
            MarkerArray, self.frontier_cb)
        self.map_sub = message_filters.Subscriber("/map", OccupancyGrid)
        self.move_base_client = actionlib.SimpleActionClient('move_base', \
            MoveBaseAction)
        self.move_base_client.wait_for_server()

        # register callbacks as synchronized
        ts = message_filters.ApproximateTimeSynchronizer([self.map_sub, self.tags_sub], 1000, 1)
        ts.registerCallback(self.synced_cb)

        # message to send when we want to spin
        self.spin_msg = Twist()
        self.spin_msg.angular.z = 1.0

        while True:
            try:
                _ = self.tf_buffer.lookup_transform('map', 'base_link', \
                    rospy.Time(0))
                break
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, \
                    tf2_ros.ExtrapolationException):
                continue
    
    def frontier_cb(self, data):
        if data:
            self.last_frontier_msg_time = rospy.get_time()

    def synced_cb(self, occ_grid, tag_detection_array):
        
        # map stuff
        self.map_resolution = occ_grid.info.resolution
        self.map_width = occ_grid.info.width
        self.map_height = occ_grid.info.height
        self.map_origin = occ_grid.info.origin
        self.map = np.array(occ_grid.data).reshape((self.map_height, \
                                                    self.map_width))

        self.map = np.transpose(self.map)
        
        self.time_since_last_frontier_msg = rospy.get_time() - \
            self.last_frontier_msg_time

        # iterate through tags
        for tag in tag_detection_array.detections:
            # Get tag info
            tag_id = tag.id[0]
            tag_name = 'map_tag_' + str(tag_id)
            frame_tag_tf = tf2_geometry_msgs.PoseStamped()
            frame_tag_tf.pose = tag.pose.pose.pose
            frame_tag_tf.header = tag.pose.header
            for _ in range(100):
                try:
                    self.map_tag_tf = self.tf_buffer.transform(frame_tag_tf, "map")
                    break
                except:
                    pass

            rospy.loginfo("Publishing marker!")
            marker = Marker()
            marker.header.frame_id = "map"
            marker.ns = tag_name
            marker.header.stamp = rospy.Time.now()
            marker.id = 0
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.pose = self.map_tag_tf.pose
            marker.scale.x = 0.25
            marker.scale.y = 0.25
            marker.scale.z = 0.25
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.b = 1.0
            marker.color.g = 0.0
            self.vis_pub.publish(marker)
        
        if self.time_since_last_frontier_msg > self.time_diff:
            print("Going to random location!")
            if self.go_to_random_goal():
                rospy.loginfo("Sent goal!")
            else: 
                rospy.loginfo("Didn't send goal or action didn't work!")

    def go_to_random_goal(self):
        foo = True
        while foo:
            # get random index for map
            rand_h = np.random.randint(0, self.map_height)
            rand_w = np.random.randint(0, self.map_width)

            # ignore positions too close to obstacles
            n = round(.3 / self.map_resolution) # how many cells per 0.3 meter
            min_h = max(0, rand_h - n)
            max_h = min(self.map_height, rand_h + n)
            min_w = max(0, rand_w - n)
            max_w = min(self.map_width, rand_w + n)
            resample_flag = False
            for i in range(min_h, max_h):
                for j in range(min_w, max_w):
                    if self.map[i, j] == -1 or self.map[i, j] >= 80:
                        resample_flag = True
                    if resample_flag:
                        break
                if resample_flag:
                    break
            if resample_flag:
                continue


            # make sure location in map is likely unoccupied
            if self.map[rand_h, rand_w] >= 0 and self.map[rand_h, rand_w] <= 10:
                foo = False
                new_x = self.map_origin.position.x + \
                    rand_h * self.map_resolution
                new_y = self.map_origin.position.y + \
                    rand_w * self.map_resolution
                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = "map"
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.pose.position.x = new_x
                goal.target_pose.pose.position.y = new_y
                goal.target_pose.pose.orientation.w = 1.0

                # visualize marker
                marker = Marker()
                marker.header.frame_id = "map"
                marker.ns = "goal"
                marker.header.stamp = rospy.Time.now()
                marker.id = 0
                marker.type = marker.SPHERE
                marker.action = marker.ADD
                marker.pose = goal.target_pose.pose
                marker.scale.x = 0.25
                marker.scale.y = 0.25
                marker.scale.z = 0.25
                marker.color.a = 1.0
                marker.color.r = 1.0
                self.vis_pub.publish(marker)

                self.move_base_client.send_goal(goal)

                wait = self.move_base_client.wait_for_result()
                return wait
                
                # if not wait:
                #     self.move_base_client.cancel_all_goals()
                #     rospy.logerr("Goal not achieved!")
                #     return False
                # success = self.move_base_client.get_result()

                self.move_base_client.cancel_all_goals()
                # spin in circle for a brief period of time
                start_time = rospy.get_time()
                while rospy.get_time() - start_time < 7.0:
                    self.cmd_vel_pub.publish(self.spin_msg)
                return success


if __name__ == "__main__":
    rospy.init_node("synced_goal_tags_node")
    GoalsTags()
    rospy.spin()