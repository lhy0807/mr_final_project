#!/usr/bin/env python

import rospy
import tf
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import Pose


class TagsUpdater:

    def __init__(self):
        num_tags = 20
        self.tag_estimates = [None for _ in range(num_tags)]
        self.tags_sub = rospy.Subscriber("tag_detections", \
            AprilTagDetectionArray, self.tags_cb)
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.tf_listener = tf.TransformListener()
        

    def tags_cb(tags):
        # iterate through tags
        for tag in tags:
            tag_id = tag.id[0]
            tag_name = 'global_tag_' + str(tag_id)
            local_pose = tag.pose.pose.pose
            frame = tag.pose.frame_id
            (trans, rot) = 


if __name__ == "__main__":
    rospy.init_node('tag_updater')
    TagsUpdater()
    rospy.spin()