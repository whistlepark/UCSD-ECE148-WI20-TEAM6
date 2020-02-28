import cv2 as cv
import pyrealsense2 as rs
import numpy as np
import pandas as pd


# input options in order (right->left): 'camera/rgb', 'camera/depth', 'camera/infra1', 'camera/infra2', 'slam/odom'

class Object_Avoidance:
    # Possibly save alterations of steering to a buffer in order to reverse those (i.e. go back onto original path)
    # Knowing distance to object and location in image: move in such a way to avoid the object knowing the odometry info

    # For example: going at certain speed then on current trajectory will run into object, with width of object move
    # angle of steering so that the change in x (let's do right as default) is such that object is avoided

    def __init__(self, object_data, odom):
        self.object = object_data
        self.odom = odom
        self.height_threshold = 10 #cm??

    def can_run_over(self):
        # Just an example of implementation
        if(self.object.height() > self.height_threshold):
            return True
        else:
            return False

    def run(self):
        pass
