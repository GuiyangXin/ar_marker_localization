# ar_marker_localization
-----------------------------------------------------------------

This library provides an interface to launch ar_track_alvar and realsense-ros to get the pose of ar_marker relative to the camera_link frame.

Contact  : Guiyang Xin [guiyang.xin ( at ) ed.ac.uk]

Author(s): Guiyang Xin

Date     : 25/11/2020

## Build status

DEPENDENCIES
-----------------------------------------------------------------
* realsense-ros - the driver for realsense camera and ROS interface (https://github.com/IntelRealSense/realsense-ros)
* ar_track_alvar - ar marker track package (sudo apt install ros-melodic-ar-track-alvar)

BUILDING and USAGE
--------------------
```
cd catkin_ws/src
git clone git@github.com:GuiyangXin/ar_marker_localization.git
cd ..
catkin_make
roslaunch ar_marker_localization ar_maker_localization.launch
```



