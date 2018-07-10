#include "ros/ros.h"

#include "sensor_msgs/Image.h"
#include "opencv/cv.h"
#include "opencv2/calib3d.hpp"
#include <cv_bridge/cv_bridge.h>

cv::Mat camMtx(3,3,CV_32F);
cv::Mat camDst(1,5,CV_32F);
ros::Publisher pubImg;

#include "sensor_msgs/PointCloud2.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>

// Lidar to camera
Eigen::Matrix4f transform;
sensor_msgs::PointCloud2::ConstPtr lastPC;
Eigen::Vector4f p1,p2; // to crop ROI PC
bool isPCavailble = false;

void imageCB(const sensor_msgs::Image::ConstPtr& img)
{
  if (!isPCavailble) return;
  pcl::PointCloud<pcl::PointXYZ>::Ptr PCLXYZcloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCLPointCloud2* PCLcloud2 = new pcl::PCLPointCloud2;
  pcl_conversions::toPCL(*lastPC, *PCLcloud2);
  pcl::fromPCLPointCloud2(*PCLcloud2,*PCLXYZcloud);

  // crop ROI: remove the points that are back side of the LIDAR
  pcl::CropBox<pcl::PointXYZ> cropFilter;
  cropFilter.setInputCloud(PCLXYZcloud);
  cropFilter.setMin(p1);
  cropFilter.setMax(p2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pclCropped(new pcl::PointCloud<pcl::PointXYZ>);
  cropFilter.filter(*pclCropped);

  // transform the pointcloud to camer frame
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::transformPointCloud(*pclCropped, *transformed_cloud, transform);


  // Convert PCL points to OpenCV points
  std::vector<cv::Point3f> pts;
  std::vector<cv::Point2f> imgPts;
  for(pcl::PointCloud<pcl::PointXYZ>::const_iterator it = transformed_cloud->begin();
      it< transformed_cloud->end();
      it++)
  {
    cv::Point3f CVpt;
    CVpt.x = it->x;
    CVpt.y = it->y;
    CVpt.z = it->z;
    pts.push_back(CVpt);
  }

  // Project points to the image plane
  cv::Mat rot = (cv::Mat_<double>(1,3) << 0 , 0 , 0); // no rotattion and translation
  cv::Mat tra= (cv::Mat_<double>(1,3) << 0 , 0 , 0);
  cv::projectPoints(pts,rot,tra,camMtx,camDst,imgPts);

  // convert the ros image to opencv image
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);

  // Draw points on the image
  int radius = 1;
  int width = cv_ptr->image.size[1];
  int height = cv_ptr->image.size[0];
  for (auto it = imgPts.begin(); it < imgPts.end(); it++)
  {
    // projected point is not inside the image, continue
    if(it->x < radius
       && it->x > width - radius
       && it->y < radius
       && it->y > height - radius) continue;
    cv::circle(cv_ptr->image, *it, radius, cv::Scalar(0,0,200),2);
  }

  pubImg.publish(cv_ptr->toImageMsg());
}

void PCCB(const sensor_msgs::PointCloud2::ConstPtr& pc)
{
  lastPC = pc;
  isPCavailble = true;

}

int main(int argc, char **argv)
{
  transform = Eigen::Matrix4f::Identity();
  transform <<  -0.08604091,  -0.9962046,  0.01316683,   -0.15,
                0.07531615, -0.01968187,  -0.99696535,   -0.36,
               0.9934407,  -0.08478814,   0.07672376,  -0.31,
               0,  0,  0, 1;

//  transform <<  0,  -1,  0,   0,
//                0, 0,  -1,    0,
//               1,  0,   0,  0,
//               0,  0,  0, 1;

  p1 << 0.0, -200.0, -200.0 ,1;
  p2 << 200.0, 200.0, 200.0 ,1;

  camMtx = (cv::Mat_<float>(3,3) << 487.46738529 , 0.0 , 458.57846055 ,
            0 ,  485.48303131 , 372.73628683 ,
            0 , 0 , 1.0);
  camDst = (cv::Mat_<float>(1,5) << -0.2247311 , 0.11839985 , 0.00156723 , 0.00077842 , -0.02886818);

  ros::init(argc, argv, "overlapScansOnImages");
  ros::NodeHandle nh;

  ros::Subscriber subImage = nh.subscribe("/sensors/camera/image_color", 10, imageCB);
  ros::Subscriber subPC = nh.subscribe("/sensors/velodyne_points", 10, PCCB);
//  ros::Subscriber subPC = nh.subscribe("CheckerBoardSurface", 10, PCCB);

  pubImg = nh.advertise<sensor_msgs::Image>("imageCloud", 100);

  ros::spin();

  return 0;
}
