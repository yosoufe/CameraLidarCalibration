#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "camera_calibration/NormalVec.h"
#include <ros/console.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/crop_box.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <boost/shared_ptr.hpp>

#include <Eigen/Eigen>

ros::Publisher pub;
ros::Publisher pubVect;

Eigen::Vector4f p1,p2;

using namespace pcl;
using namespace std;


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
  SampleConsensusModelPlane<PointXYZ>::Ptr model(
    new SampleConsensusModelPlane<PointXYZ> (pclCropped));
  RandomSampleConsensus<PointXYZ> sac (model, 0.01);
  bool result = sac.computeModel ();

  boost::shared_ptr< vector<int> > inliers (new vector<int>);
  sac.getInliers (*inliers);
  Eigen::VectorXf coefficients;
  sac.getModelCoefficients (coefficients);

  if (inliers->size () == 0)
  {
    ROS_DEBUG ("Could not estimate a planar model for the given dataset.");
    return;
  }

  camera_calibration::NormalVec coeff;
  for (int i = 0; i< 4; i++){
    coeff.vec.push_back(coefficients[i]);
  }
  pubVect.publish(coeff);


  // Planar Segmentation
  /*
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
  ROS_INFO_STREAM(inliers->indices.size ()<<" from PC: " << coefficients->values[0] << " "
                                  << coefficients->values[1] << " "
                                  << coefficients->values[2] << " "
                                  << coefficients->values[3]); 


  camera_calibration::NormalVec coeff;
  coeff.vec = coefficients->values;
  pubVect.publish(coeff);

  */


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
  p1 << 0.5, -1.5, -1.5 ,1; // x,y,y,1
  p2 << 2, 1.5, 2 ,1;

  ros::init(argc, argv, "findPlaneInPC");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("/sensors/velodyne_points", 1000, pc2Callback);
  pub = nh.advertise<sensor_msgs::PointCloud2>("/processed/CheckerBoardSurface", 100);
  pubVect = nh.advertise<camera_calibration::NormalVec>("/processed/planeEquation/fromPC",10);

  ros::spin();

  return 0;
}
