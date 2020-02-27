import cv2 as cv
import pyrealsense2 as rs
import numpy as np
import pandas as pd


# input options in order (right->left): 'camera/rgb', 'camera/depth', 'camera/infra1', 'camera/infra2', 'slam/odom'

class Object_Detection:

    def __init__(self, rgb=None, depth=None, infra1=None, infra2=None):
        self.rgb = rgb
        self.depth = depth
        self.infra1 = infra1
        self.infra2 = infra2

    def camera(self):
        pass