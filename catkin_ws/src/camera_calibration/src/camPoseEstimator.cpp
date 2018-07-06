#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"

#include "opencv/cv.h"
#include "opencv2/calib3d.hpp"
#include <cv_bridge/cv_bridge.h>
#include <vector>

ros::Publisher pubImg;
bool isCamInfoAvailable = false;
sensor_msgs::CameraInfo camInfo;
cv::Size patternSize = {7,5};

void camInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& info)
{
  camInfo = *info;
  isCamInfoAvailable = true;
}

void imgCallback(const sensor_msgs::Image::ConstPtr& img)
{
  if(!isCamInfoAvailable) return;
  // From ROS image to OpenCV Image
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);

  // find checkerboard corners
  cv_bridge::CvImagePtr img_gry_ptr(new cv_bridge::CvImage);
  cv::cvtColor(cv_ptr->image,img_gry_ptr->image,cv::COLOR_RGB2GRAY);
  std::vector<cv::Point2f> corners;
  bool found = cv::findChessboardCorners(img_gry_ptr->image,
                                       patternSize,
                                       corners);
  if (!found) return;

  cv::drawChessboardCorners(cv_ptr->image,patternSize,corners,found);

  // find the rotation of the camera relative to the checkerboard

  // add axis to the checkerboard

  // Publish the image
  pubImg.publish(cv_ptr->toImageMsg());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "camPoseEstimator");
  ros::NodeHandle nh;

  ros::Subscriber subImg = nh.subscribe("/sensors/camera/image_color", 1, imgCallback);
  ros::Subscriber subCamInfo = nh.subscribe("/sensors/camera/camera_info", 1, camInfoCallback);
  pubImg = nh.advertise<sensor_msgs::Image>("imageWithAxis", 100);

  ros::spin();

  return 0;
}
