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
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCLPointCloud2* cloud2 = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud2);
  pcl::PCLPointCloud2 cloud_filtered;
  pcl_conversions::toPCL(*input, *cloud2);
  pcl::fromPCLPointCloud2(*cloud2,*cloud);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::SACSegmentation<pcl::PointXYZ> seg;

  seg.setOptimizeCoefficients (true);

  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);

  seg.setInputCloud (cloud);
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


  pcl::CropBox<pcl::PointXYZ> cropFilter;
  cropFilter.setInputCloud(cloud);
  cropFilter.setMin(p1);
  cropFilter.setMax(p2);

  pcl::PointCloud<pcl::PointXYZ>::Ptr pclOtput(new pcl::PointCloud<pcl::PointXYZ>);
  cropFilter.filter(*pclOtput);

  // Create a container for the data.
  sensor_msgs::PointCloud2Ptr output(new sensor_msgs::PointCloud2);

  pcl::toROSMsg(*pclOtput,*output);

  // Do data processing here...
  //output = *input;

  // Publish the data.
  pub.publish (output);
}

int main(int argc, char **argv)
{
  p1[0]=0.5; p1[1]=-1.5;  p1[2]=-1.5;
  p2[0]=2;   p2[1]=1.5;   p2[2]=2;

  ros::init(argc, argv, "preProcessPC");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("/sensors/velodyne_points", 1000, pc2Callback);
  pub = nh.advertise<sensor_msgs::PointCloud2>("croped_PC", 100);


  ros::spin();

  return 0;
}
