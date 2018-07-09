#include "ros/ros.h"
#include "camera_calibration/NormalVec.h"
#include "ros/console.h"
#include "camera_calibration/Save.h"

#include <fstream>

std::vector<float> fromCamLastSample;
std::vector<float> fromPCLastSample;
std::string filePath;

int n_CamLength = 3;
int n_PCLength = 4;

void init(void)
{
  fromCamLastSample.resize(n_CamLength);
  fromPCLastSample.resize(n_PCLength);
}

void fromCamCB(const camera_calibration::NormalVec::ConstPtr& in)
{
  std::memcpy(&(fromCamLastSample[0]),&(in->vec[0]), n_CamLength*sizeof(float));
//  ROS_INFO_STREAM(fromCamLastSample[0] << " "
//                                       << fromCamLastSample[1] << " "
//                                       << fromCamLastSample[2]);
  ROS_DEBUG("Norm Vector read 1");
}

void fromPCCB(const camera_calibration::NormalVec::ConstPtr& in)
{
  std::memcpy(&(fromPCLastSample[0]),&(in->vec[0]), n_PCLength*sizeof(float));
//  ROS_INFO_STREAM(fromPCLastSample[0] << " "
//                                      << fromPCLastSample[1] << " "
//                                      << fromPCLastSample[2] << " "
//                                      << fromPCLastSample[3]);
  ROS_DEBUG("Norm Vector read 1");
}

bool save(camera_calibration::Save::Request &req,
          camera_calibration::Save::Response &res)
{
  std::ofstream file;
  file.open(filePath, std::ofstream::in | std::ofstream::app);
  if (!file.is_open()) {
    res.result = false;
    return false;
  }
  for (int i = 0; i< fromCamLastSample.size(); i++)
  {
    file << fromCamLastSample[i] << ", ";
  }

  for (int i = 0; i< fromPCLastSample.size()-1; i++)
  {
    file << fromPCLastSample[i] << ", ";
  }
  file << fromPCLastSample[fromPCLastSample.size()-1] << std::endl;
  res.result = true;

  file.close();
  return true;
}


int main(int argc, char **argv)
{
  init();
  ros::init(argc, argv, "Saver");
  ros::NodeHandle nh;

  if(!nh.getParam("filePath",filePath)) filePath = "~/vectors.txt";
  ROS_INFO_STREAM(filePath);

  ros::Subscriber subCamVec = nh.subscribe("/plane/fromCam", 10, fromCamCB);
  ros::Subscriber subPCVec = nh.subscribe("/plane/fromPC", 10, fromPCCB);

  ros::ServiceServer service = nh.advertiseService("save", save);

  ros::spin();

  return 0;
}
