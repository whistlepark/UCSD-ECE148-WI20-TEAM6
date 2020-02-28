#!/usr/bin/env python3

from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from rospy_wrapper import ros_wrapper

class SLAMWrapper():
    RGB_RAW='rgb'
    DEPTH_RAW='depth'
    INFRA1_RAW='infra1'
    INFRA2_RAW='infra2'
    ODOMETRY='odom'

    def __init__(self):
        self.topics={self.RGB_RAW: "/camera/color/image_raw",
                     self.DEPTH_RAW: "/camera/depth/image_rect_raw",
                     self.INFRA1_RAW: "/camera/infra1/image_rect_raw",
                     self.INFRA2_RAW: "/camera/infra2/image_rect_raw",
                     self.ODOMETRY: "/rtabmap/odom"}
        self.types={self.RGB_RAW: Image,
                    self.DEPTH_RAW: Image,
                    self.INFRA1_RAW: Image,
                    self.INFRA2_RAW: Image,
                    self.ODOMETRY: Odometry}

        self.ros_wrapper=ros_wrapper()

    def subscribe_batch(self, frames, callback):
        topics=[self.topics[frame] for frame in frames]
        types=[self.types[frame] for frame in frames]
        self.ros_wrapper.add_synced_topics(topics, types, callback)

    def subscribe_single(self, frame, callback):
        self.ros_wrapper.add_topic(self.topics[frame], self.types[frame], callback)
    def spin(self):
        self.ros_wrapper.spin()
    def start(self):
        self.ros_wrapper.start()
