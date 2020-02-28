import cv2 as cv
import pyrealsense2 as rs
import numpy as np
import pandas as pd


# input options in order (right->left): 'camera/rgb', 'camera/depth', 'camera/infra1', 'camera/infra2', 'slam/odom'

class Object_Detection:
    # Look for area of depth image that correlates to closest object
    # Look for same location in rgb/infra
    # Filter and detect most "present" object (i.e. the closest object that might be run into)
    # Return object location, height and width and pixel box location

    def __init__(self, rgb=None, depth=None, infra1=None, infra2=None):
        self.rgb = cv.imread(rgb)
        self.depth = cv.imread(depth)
        self.infra1 = cv.imread(infra1)
        self.infra2 = cv.imread(infra2)

    def camera(self):
        pass

    def run(self):
        pass
