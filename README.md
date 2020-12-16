# ar_marker_localization
-----------------------------------------------------------------

This library provides an interface to launch ar_track_alvar and realsense-ros to get the pose of ar_marker relative to the camera_link frame.

Contact  : Guiyang Xin [guiyang.xin ( at ) ed.ac.uk]

Author(s): Guiyang Xin

Date     : 25/11/2020
![Camera in Rviz](doc/coverPhoto.png?raw=true "Camera in Rviz")
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
source devel/setup.bash
roslaunch ar_marker_localization ar_maker_localization.launch
```
* The upper object's pose with respect to the end-effector frame is published on ros topic `/uperPartPoseInRightEndEffectorFrame` with ``geometry_msgs/Pose`` message type.
* The lower object's pose with respect to the upper object's frame (locatated at the geometric centre) is published on ros topic `/lowerPartPoseInUpperPartFrame` with ``geometry_msgs/Pose`` message type.


