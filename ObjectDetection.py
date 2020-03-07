import cv2 as cv
import pyrealsense2 as rs
import numpy as np


# input options in order (right->left): 'camera/rgb', 'camera/depth', 'slam/odom'

class ObjectDetection:
    # Look for area of depth image that correlates to closest object
    # Look for same location in rgb/infra
    # Detect most "present" object (i.e. the closest object that might be run into)
    # Return object location, height and width and pixel box location

    def __init__(self, rgb=None, depth=None):
        self.rgb = rgb
        self.depth = depth
        self.height, self.width = rgb.shape[:2]
        self.depth_scale = 0.0010000000474974513
        self.closest = None

    def run(self, rgb, depth):
        if not self.rgb or not self.depth:
            pass
        self.rgb = rgb
        self.depth = depth

    class Closest:
        def __init__(self):
            self.cx = None
            self.cy = None
            self.distance = None
            self.width = None
            self.height = None
