# UCSD-ECE148-WI20-TEAM6
OpenSource Code for Donkey/Realsense Object Detection Interface

Contributions: 

Brian Rush, Aaron Hanna, Stephen Lien

## ROS/SLAM
> Run the command: roslaunch ~/slam.launch
> > Note: To view this in vnc use 'rviz = true' argument after command
> > Make sure vncserver.sh is ran before using rviz!
>
>Make sure this is from home directory as the file is located there
>
## Object Detection
>Inputs: Camera(rgb, depth, infra)
>Outputs: Objects data

Ideally, will take in camera data (rgb, depth,infra) as input
and return objects and their location along with height and distance
information.

## Object Avoidance
>Inputs: Objects data from Object_Detection class
>
>Outputs: Steering, Throttle
>
>
### Uploading to Git: Commands
>git add -u (adds updated files)
>
>git commit -a -m "insert message here" (adds added files with message)
>
>git push (pushes to master branch)