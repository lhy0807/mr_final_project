#!/usr/bin/env python

import rospy
import tf2_ros
import tf2_geometry_msgs

from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker

class TagsUpdater:

    def __init__(self):
        rospy.sleep(1.)
        num_tags = 20
        self.tag_estimates = [None for _ in range(num_tags)]
        self.tags_sub = rospy.Subscriber("tag_detections", \
            AprilTagDetectionArray, self.tags_cb)
        self.vis_pub = rospy.Publisher("visualization_marker", Marker, \
            queue_size=1)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        while True:
            try:
                _ = self.tf_buffer.lookup_transform('map', 'base_link', \
                    rospy.Time(0))
                break
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, \
                    tf2_ros.ExtrapolationException):
                continue

    def tags_cb(self, tags):
        # iterate through tags
        for tag in tags.detections:
            # Get tag info
            tag_id = tag.id[0]
            tag_name = 'map_tag_' + str(tag_id)
            frame_tag_tf = tf2_geometry_msgs.PoseStamped()
            frame_tag_tf.pose = tag.pose.pose.pose
            frame_tag_tf.header = tag.pose.header
            map_tag_tf = None
            try:
                map_tag_tf = self.tf_buffer.transform(frame_tag_tf, "map")
            except:
                pass
            
            if map_tag_tf is None:
                continue

            marker = Marker()
            marker.header.frame_id = "map"
            marker.ns = tag_name
            marker.header.stamp = rospy.Time.now()
            marker.id = 0
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.pose.position.x = map_tag_tf.pose.position.z
            marker.pose.position.y = map_tag_tf.pose.position.x
            marker.pose.position.z = map_tag_tf.pose.position.y
            marker.scale.x = 0.25
            marker.scale.y = 0.25
            marker.scale.z = 0.25
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.b = 1.0
            marker.color.g = 0.0
            self.vis_pub.publish(marker)

if __name__ == "__main__":
    rospy.init_node('tag_updater')
    TagsUpdater()
    rospy.spin()