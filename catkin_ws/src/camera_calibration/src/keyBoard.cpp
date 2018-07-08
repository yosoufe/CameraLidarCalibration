#include "ros/ros.h"
#include "camera_calibration/Save.h"
#include <stdio.h>
#include "ros/console.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "keyBoard");

  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<camera_calibration::Save>("save");
  camera_calibration::Save srv;

  while(ros::ok())
  {
    int ch;
    ch = std::getchar();
    switch (ch) {
    case 'q':
      ROS_INFO_STREAM ("Typed word: " << (unsigned char)(ch));
      client.call(srv);
      break;
    }
    ros::spinOnce();
  }
}
