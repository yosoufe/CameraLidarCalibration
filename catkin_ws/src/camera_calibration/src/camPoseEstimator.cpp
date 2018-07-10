#include "ros/ros.h"
#include "ros/console.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"
#include "camera_calibration/NormalVec.h"

#include "opencv/cv.h"
#include "opencv2/calib3d.hpp"
#include <cv_bridge/cv_bridge.h>
#include <vector>


ros::Publisher pubImg;
ros::Publisher pubVect;

bool isCamInfoAvailable = false;
sensor_msgs::CameraInfo camInfo;
cv::Mat camMtx(3,3,CV_32F);
cv::Mat camDst(1,5,CV_32F);
cv::Size patternSize = {7,5};
std::vector<cv::Point3f> objPnts;

void camInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& info)
{
//  if(isCamInfoAvailable) return;
//  isCamInfoAvailable = true;
//  float tempK[9];
//  for (int i = 0; i < 9; i++) {
//     tempK[i] = info->K[i];
//  }
//  std::memcpy(camMtx.data, info->K.data(),3*3*sizeof(float));
//  std::memcpy(camDst.data, info->D.data(),1*5*sizeof(float));
  //ROS_INFO_STREAM( camDst.at<float>(0) << " " << camMtx.at<float>(0) );
}

void imgCallback(const sensor_msgs::Image::ConstPtr& img)
{
  if(!isCamInfoAvailable) return;
  // From ROS image to OpenCV Image
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);

  // find checkerboard corners
  cv_bridge::CvImagePtr img_gry_ptr(new cv_bridge::CvImage); //grayscale image
  cv::cvtColor(cv_ptr->image,img_gry_ptr->image,cv::COLOR_BGR2GRAY);
  std::vector<cv::Point2f> corners;
  bool found = cv::findChessboardCorners(img_gry_ptr->image,
                                       patternSize,
                                       corners);
  if (!found) return;

  // draw the corners on the image
  cv::drawChessboardCorners(cv_ptr->image,patternSize,corners,found);

  // find the rotation of the camera relative to the checkerboard
  cv::Mat rot;
  cv::Mat tra;
  cv::Mat rot33;

  try
  {
    cv::solvePnP(objPnts,corners,camMtx,camDst,rot,tra);
    cv::Rodrigues(rot,rot33);
    ROS_INFO_STREAM( "from Cam: " << rot33.row(2));

    // publish the normal axis
    camera_calibration::NormalVec coeff;
    coeff.vec.resize(3);
//    coeff.vec[0] = rot33.at<double>(2,0);
//    coeff.vec[1] = rot33.at<double>(2,1);
//    coeff.vec[2] = rot33.at<double>(2,2);
    coeff.vec[0] = rot33.at<double>(0,2);
    coeff.vec[1] = rot33.at<double>(1,2);
    coeff.vec[2] = rot33.at<double>(2,2);
    pubVect.publish(coeff);

    // add axis to the checkerboard
    std::vector<cv::Point3f> pts;
    std::vector<cv::Point2f> imgPts;
    pts.push_back(cv::Point3f(0,0,0));
    pts.push_back(cv::Point3f(0,0,5));
    cv::projectPoints(pts,rot,tra,camMtx,camDst,imgPts);
    cv::line(cv_ptr->image,imgPts[0],imgPts[1],cv::Scalar(0,250,0),3);

  }
  catch (cv::Exception e)
  {
    ROS_WARN_STREAM(e.msg);
  }

  // Publish the image
  pubImg.publish(cv_ptr->toImageMsg());
}

int main(int argc, char **argv)
{
  for(float height = 0; height<patternSize.height; height++)
  {
    for (float width = 0; width <patternSize.width; width++ )
    {
      cv::Point3f pnt(height,width,0);
      objPnts.push_back(pnt);
    }
  }
  camMtx = (cv::Mat_<float>(3,3) << 487.46738529 , 0.0 , 458.57846055 ,
  0 ,  485.48303131 , 372.73628683 ,
  0 , 0 , 1.0);
  camDst = (cv::Mat_<float>(1,5) << -0.2247311 , 0.11839985 , 0.00156723 , 0.00077842 , -0.02886818);
  isCamInfoAvailable = true;

  ros::init(argc, argv, "camPoseEstimator");
  ros::NodeHandle nh;

  ros::Subscriber subImg = nh.subscribe("/sensors/camera/image_color", 1, imgCallback);
  //ros::Subscriber subCamInfo = nh.subscribe("/sensors/camera/camera_info", 1, camInfoCallback);
  pubImg = nh.advertise<sensor_msgs::Image>("imageWithAxis", 100);
  pubVect = nh.advertise<camera_calibration::NormalVec>("plane/fromCam",10);

  ros::spin();

  return 0;
}
