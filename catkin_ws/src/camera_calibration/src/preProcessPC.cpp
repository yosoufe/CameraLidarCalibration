#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"
#include <ros/console.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/crop_box.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <Eigen/Eigen>

ros::Publisher pub;
//pcl::PointCloud< pcl::PointXYZ > PointCloudXYZ;

Eigen::Vector4f p1,p2;


void pc2Callback(const sensor_msgs::PointCloud2ConstPtr& input)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr PCLXYZcloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCLPointCloud2* PCLcloud2 = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2ConstPtr cloudPtr(PCLcloud2);
  pcl::PCLPointCloud2 cloud_filtered;
  pcl_conversions::toPCL(*input, *PCLcloud2);
  pcl::fromPCLPointCloud2(*PCLcloud2,*PCLXYZcloud);

  // crop ROI
  pcl::CropBox<pcl::PointXYZ> cropFilter;
  cropFilter.setInputCloud(PCLXYZcloud);
  cropFilter.setMin(p1);
  cropFilter.setMax(p2);

  pcl::PointCloud<pcl::PointXYZ>::Ptr pclCropped(new pcl::PointCloud<pcl::PointXYZ>);
  cropFilter.filter(*pclCropped);

  // Create a container for the data.
  sensor_msgs::PointCloud2Ptr ROSoutput(new sensor_msgs::PointCloud2);
  //pcl::toROSMsg(*pclCropped,*ROSoutput);

  // Planar Segmentation
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);
  seg.setInputCloud (pclCropped);
  seg.segment (*inliers, *coefficients);
  if (inliers->indices.size () == 0)
  {
    ROS_DEBUG ("Could not estimate a planar model for the given dataset.");
    return;
  }
  ROS_INFO_STREAM(inliers->indices.size ()<<"Model coefficients: " << coefficients->values[0] << " "
                                  << coefficients->values[1] << " "
                                  << coefficients->values[2] << " "
                                  << coefficients->values[3]);
  // Just Inliers
  pcl::CropBox<pcl::PointXYZ> indFilter;
  cropFilter.setInputCloud(pclCropped);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pclPointsOnSurface(new pcl::PointCloud<pcl::PointXYZ>);
  cropFilter.setIndices(inliers);
  cropFilter.filter(*pclPointsOnSurface);

  pcl::toROSMsg(*pclPointsOnSurface,*ROSoutput);

  // Publish the Output.
  pub.publish (ROSoutput);
}

int main(int argc, char **argv)
{
  p1[0]=0.5; p1[1]=-1.5;  p1[2]=-1.5;
  p2[0]=2;   p2[1]=1.5;   p2[2]=2;

  ros::init(argc, argv, "preProcessPC");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("/sensors/velodyne_points", 1000, pc2Callback);
  pub = nh.advertise<sensor_msgs::PointCloud2>("CheckerBoardSurface", 100);


  ros::spin();

  return 0;
}
