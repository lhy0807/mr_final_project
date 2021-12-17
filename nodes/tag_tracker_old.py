#!/usr/bin/env python

import rospy
import tf2_ros
import tf2_geometry_msgs

from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import MarkerArray, Marker

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
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, queue_size=1)
        self.vis_pub = rospy.Publisher("visualization_marker", Marker, \
            queue_size=1)
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

            # do not update the tag pose if detected before
            # if self.tag_estimates[tag_id]:
            #     continue

            tag_name = 'map_tag_' + str(tag_id)
            frame_tag_pose = tag.pose.pose.pose
            frame = tag.pose.header.frame_id

            # Get transform from map to tag
            map_tag_pose = self.transform_pose(frame_tag_pose, frame, 'map')
            print(map_tag_pose)
            self.tag_estimates[tag_id] = map_tag_pose

        for i, est in enumerate(self.tag_estimates):
            if est:
                t = TransformStamped()
                t.header.stamp = rospy.Time.now()
                t.header.frame_id = 'map'

                t.child_frame_id = 'map_tag_' + str(i)
                t.transform.translation = self.tag_estimates[i].position
                t.transform.rotation = self.tag_estimates[i].orientation
                self.tf_broadcaster.sendTransform(t)

                marker = Marker()
                marker.header.frame_id = "map"
                marker.ns = "our_tag"
                marker.header.stamp = rospy.Time.now()
                marker.id = i
                marker.type = marker.SPHERE
                marker.action = marker.ADD
                marker.pose = self.tag_estimates[i]
                marker.scale.x = 0.20
                marker.scale.y = 0.20
                marker.scale.z = 0.20
                marker.color.a = 1.0
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 1.0
                self.vis_pub.publish(marker)


    def transform_pose(self, input_pose, init_frame, final_frame):
        # **Assuming /tf2 topic is being broadcasted
        pose_stamped = tf2_geometry_msgs.PoseStamped()
        pose_stamped.pose = input_pose
        pose_stamped.header.frame_id = init_frame
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.header.frame_id = "camera_rgb_optical_frame"
        try:
            # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
            output_pose_stamped = self.tf_buffer.transform(pose_stamped, \
                final_frame, rospy.Duration(1))
            return output_pose_stamped.pose
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, \
            tf2_ros.ExtrapolationException):
            pass

if __name__ == "__main__":
    rospy.init_node('tag_updater')
    TagsUpdater()
    rospy.spin()