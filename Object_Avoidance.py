import cv2 as cv
import pyrealsense2 as rs
import numpy as np
import pandas as pd


# input options in order (right->left): 'camera/rgb', 'camera/depth', 'camera/infra1', 'camera/infra2', 'slam/odom'

class Object_Avoidance:

    def __init__(self, object_data):
        self.object = object_data
        self.height_threshold = 10 #cm??

    def can_run_over(self):
        # Just an example of implementation
        if(self.object.height() > self.height_threshold):
            return True
        else:
            return False

    def run(self):
        pass
