# Ride Cell Assignment Writeup

The Assigment is the 2 first tasks of [this](https://docs.google.com/document/d/1nPu88GcZtNbK_Ymds0xDD0di-BQol65H-c6o5yjIIq0/edit?usp=sharing).

## TimeLine:
1. Installing ROS on my new Ubuntu 18.04 machine.
2. `rqt_bag` GUI has some issues on ROS melodic and Ubuntu bionic (check [here](https://github.com/ros-visualization/rqt_bag/issues/27)). Only `rosbag` in terminal could be used.
3. Sample some of the [images](https://github.com/yosoufe/Assignment/tree/master/scripts/cal_imgs) for calibration using rqt_image_viewer.
4. Camera Calibration in this [Notebook](https://github.com/yosoufe/Assignment/blob/master/scripts/Camera%20Calibration.ipynb). ([hint](https://docs.opencv.org/3.4/dc/dbb/tutorial_py_calibration.html))
5. Croping the Pointcloud to the ROI which has the checkerboard in [this ROS Node](https://github.com/yosoufe/Assignment/blob/master/catkin_ws/src/camera_calibration/src/findPlaneInImages.cpp).
6. Detecting the checkerboard surface in the Point Cloud and finding the equation of the surface in [findPlaneInPC_node](https://github.com/yosoufe/Assignment/blob/master/catkin_ws/src/camera_calibration/src/findPlaneInPC.cpp) as shown [here](https://youtu.be/wLyaJD1hT4E).
7. Finding the Checkerboard Surface from the camera and calculating the Surface equation in Camera Frame ([hint](https://docs.opencv.org/3.4/d7/d53/tutorial_py_pose.html)) in [findPlaneInImages_node](https://github.com/yosoufe/Assignment/blob/master/catkin_ws/src/camera_calibration/src/findPlaneInImages.cpp).
8. Saving the normal vector to the checkerboard surface in two different LIDAR and Camera Frame into a csv file in [saver_node](https://github.com/yosoufe/Assignment/blob/master/catkin_ws/src/camera_calibration/src/saver.cpp). This node is offering a service to save. The [keyBoard_node](https://github.com/yosoufe/Assignment/blob/master/catkin_ws/src/camera_calibration/src/keyBoard.cpp) is reading the keyboard and when it receives charachter `q`, it calls the `save` service from `Saver_node`. In this way the user can monitor the LIDAR and Camera data and decide when the saving should be done.

The steps from 5. to 8. are done in [palyBag.launch](https://github.com/yosoufe/Assignment/blob/master/catkin_ws/src/camera_calibration/launch/playBag.launch). The following image is showing the topics and nodes.

![alt text](https://raw.githubusercontent.com/yosoufe/Assignment/master/Docs/ToProcess.png)

9. Match the normal vectors to find the rotation matrix between the camera and LIDAR ([tutorial](https://www.coursera.org/learn/robotics-perception/lecture/X22IH/pose-from-3d-point-correspondences-the-procrustes-problem)) at [this jupytern notebook](https://github.com/yosoufe/Assignment/blob/master/scripts/RotationMatrixCal.ipynb). The vectors are read from the csv file which was saved in the previous step.
10. Overlap the pointcloud into the image in [overlapScansOnImages_node](https://github.com/yosoufe/Assignment/blob/master/catkin_ws/src/camera_calibration/src/overlapScansOnImages.cpp) using the calculated Rotation Matrix in the previous step. The translation part of the transformation Matrix has been gained by trial and error. [final.launch](https://github.com/yosoufe/Assignment/blob/master/catkin_ws/src/camera_calibration/launch/final.launch) runs the final result which can be seen in [this video](https://www.youtube.com/watch?v=ZNkftIjOVeE&feature=youtu.be)

<a href="https://www.youtube.com/watch?v=ZNkftIjOVeE&feature=youtu.be" target="_blank"><img src="https://raw.githubusercontent.com/yosoufe/Assignment/master/Docs/video.png" alt="Final Video"/></a>

![alt text](https://raw.githubusercontent.com/yosoufe/Assignment/master/Docs/ToShowOff.png)

## Installations:

```
conda create -n assignment python=3
source activate assignment
# conda install -c menpo opencv3
conda install -c conda-forge opencv 
pip install scipy
```

## Usage:
Put the bag file in a folder `data` in the same directory. 
Then it can be played by the following command.
```
catkin_make
source devel/setup.bash
roslaunch camera_calibration playBag.launch
roslaunch camera_calibration final.launch
```


