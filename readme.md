# Ride Cell Assignment Writeup

The Assigment is the 2 first tasks of [this](https://docs.google.com/document/d/1nPu88GcZtNbK_Ymds0xDD0di-BQol65H-c6o5yjIIq0/edit?usp=sharing).

## TimeLine:
* Installing ROS on my new Ubuntu 18.04 machine.
* `rqt_bag` GUI has some issues on ROS melodic and Ubuntu bionic (check [here](https://github.com/ros-visualization/rqt_bag/issues/27)). Only `rosbag` in terminal could be used.
* Sample some of the [images](https://github.com/yosoufe/Assignment/tree/master/scripts/cal_imgs) for calibration using rqt_image_viewer.
* Camera Calibration in this [Notebook](https://github.com/yosoufe/Assignment/blob/master/scripts/Camera%20Calibration.ipynb). ([hint](https://docs.opencv.org/3.4/dc/dbb/tutorial_py_calibration.html))
* Croping the Pointcloud to the ROI which has the checkerboard in [this ROS Node](https://github.com/yosoufe/Assignment/blob/master/catkin_ws/src/camera_calibration/src/preProcessPC.cpp).
* SubSample the cropped pintcloud to only the checkerboard surface in [this ROS Node](https://github.com/yosoufe/Assignment/blob/master/catkin_ws/src/camera_calibration/src/preProcessPC.cpp).
* Pose Estimation with camera and checkerboard ([hint](https://docs.opencv.org/3.4/d7/d53/tutorial_py_pose.html)) in [this node].

## Installations:

```
conda create -n assignment python=3
source activate assignment
# conda install -c menpo opencv3
conda install -c conda-forge opencv 

```

## Usage:
Put the bag file in a folder `data` in the same directory. 
Then it can be played by the following command.
```
source devel/setup.bash
roslaunch camera_calibration playBag.launch
```


