# Ride Cell Assignment Writeup

The Assigment is the 2 first tasks of [this](https://docs.google.com/document/d/1nPu88GcZtNbK_Ymds0xDD0di-BQol65H-c6o5yjIIq0/edit?usp=sharing).

## TimeLine:
* Installing ROS on my new Ubuntu 18.04 machine.
* `rqt_bag` GUI has some issues on ROS melodic and Ubuntu bionic (check [here](https://github.com/ros-visualization/rqt_bag/issues/27)). Only `rosbag` in terminal could be used.
* Sample some of the images for calibration


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
rosbag play -q data/2016-11-22-14-32-13_test.bag
```


