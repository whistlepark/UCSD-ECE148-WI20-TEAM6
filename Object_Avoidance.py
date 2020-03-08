import cv2 as cv
import pyrealsense2 as rs
import numpy as np
import pandas as pd


# input options in order (right->left): 'camera/rgb', 'camera/depth', 'camera/infra1', 'camera/infra2', 'slam/odom'
    # Possibly save alterations of steering to a buffer in order to reverse those (i.e. go back onto original path)
    # Knowing distance to object and location in image: move in such a way to avoid the object knowing the odometry info

    # For example: going at certain speed then on current trajectory will run into object, with width of object move
    # angle of steering so that the change in x (let's do right as default) is such that object is avoided

class ObjectAvoidance:
    def __init__(self, left_boundary, right_boundary, throttle=0.05, car_height=0.01):
        self.turn_angle = 0
        self.turn = "Straight"
        self.throttle=throttle
        self.left_boundary = left_boundary
        self.right_boundary = right_boundary
        self.midpoint=(right_boundary-left_boundary)/2
        self.can_run_over == False
        self.car_height = car_height # In meters, 10cm is the height of the wheel, this value can be changed after testing


    def can_run_over(self):
        if(self.object_height < self.car_height):
            return True
        else:
            return False
    
    def turn_direction(self, cx, width, distance):
        if (cx+width/2 < self.left_boundary or cx-width/2 > self.right_boundary):
            self.turn = "Straight"
            self.turn_angle = 0
        elif(cx >= self.midpoint):
            self.turn = "Left"
            self.turn_angle = -1
        elif(cx < self.midpoint):
            self.turn = "Right"
            self.turn_angle = 1
        else:
            self.turn = "Right (angle)"
            self.turn_angle = abs(np.arctan( (width/2) / distance ))

    

    def run(self, objects):
        if (objects is None or len(objects)==0):
            print("Turning {}".format(self.turn))
            return 0, self.throttle

        obj=min(objects)
        if (self.can_run_over == True):
            self.turn = "Over"
            self.turn_angle == 0
        else:
            self.turn_direction(obj.cx, obj.width, obj.distance)
        print("Turning {}".format(self.turn))
        return self.turn_angle, self.throttle


