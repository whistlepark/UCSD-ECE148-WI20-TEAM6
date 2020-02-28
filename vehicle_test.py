import donkeycar as dk
from donkeycar.parts.realsense2 import RS_D435

V=dk.vehicle.Vehicle()
cam=RS_D435()

V.add(cam, outputs=['camera/rgb','camera/depth','camera/infra1','camera/infra2','slam/odom'])
