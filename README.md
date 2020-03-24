# Frames
Autonomous passing through frames 

---
# Overview
This mini project has the goal to make a quad pass through a frame autonomously using OpenCV and a cv_bridge for integration with ros.
The future goals include skewed frames and a race track.

---
# Dependencies

This package has the following dependencies:

1. [OpenCV](https://opencv.org) 
2. [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page)
3. ROS Kinetic with the following packages:

    1. catkin
    2. roscpp
    3. mavros_msgs
    4. geometry_msgs
    5. image_transport
    6. cv_bridge
    7. opencv2
    8. std_msgs
    9. eigen_conversations
    10. message_generation
    11. message_runtime
    12. tf
    13. tf_conversions

---
# Installation

Create a catkin workspace in home directory (ignore if already done).
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init
```
Clone the repository into the src folder
```
cd ~/catkin_ws/src
git clone https://github.com/isro01/Frames.git
```
Build using ``` catkin build Frames``` after making sure all the dependencies are met.

---
# Nodes
-> **controller** : sets the quad to offboard and can be used to make the quad go forward in a line by uncommenting line 31-41 and line 69-76.

-> **img_con** : has the cv_bridge and image processing algorithms.

-> **transform** : publishing pose to the quad using camera matrix.
