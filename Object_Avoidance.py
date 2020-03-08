import cv2 as cv
import pyrealsense2 as rs
import numpy as np
import pandas as pd


# input options in order (right->left): 'camera/rgb', 'camera/depth', 'camera/infra1', 'camera/infra2', 'slam/odom'
    # Possibly save alterations of steering to a buffer in order to reverse those (i.e. go back onto original path)
    # Knowing distance to object and location in image: move in such a way to avoid the object knowing the odometry info

    # For example: going at certain speed then on current trajectory will run into object, with width of object move
    # angle of steering so that the change in x (let's do right as default) is such that object is avoided

class Turn_direction_calc:
    def __init__(self, turn_deg, turn, x_min, x_max, object_height, width, distance, left_boundary, right_boundary, car_height):
        self.turn_deg = 0
        self.turn = ""
        self.x_min = 0
        self.x_max = 0
        self.object_height = 0
        self.width = 0
        self.distance = 0
        self.left_boundary = []
        self.right_boundary = []
        self.car_height = 0.01 # In meters, 10cm is the height of the wheel, this value can be changed after testing


    def can_run_over(self, object_height):
        if(self.object_height < self.car_height):
            return True
        else:
            return False
    
    def turn_direction(self, x_min, x_max, left_boundary, right_boundary):
        if(x_min >= left_boundary[0]):
            self.turn = "Left"
            self.turn_angle = -1
        elif(x_max <= right_boundary[0]):
            self.turn = "Right"
            self.turn_angle = 1
        else:
            self.turn = "Right"
            self.turn_angle = abs(np.arctan( (self.width/2) / self.distance ))

    

    def run(self, x_min, x_max, left_boundary, right_boundary):
        if (self.can_run_over == True):
            self.turn = "Over"
            self.turn_angle == 0
            return self.turn_angle, self.turn
        else:
            self.turn_direction(self.x_min, self.x_max, self.left_boundary, self.right_boundary)
            return self.turn_angle, self.turn


