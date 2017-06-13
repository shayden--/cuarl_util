#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"

int main(int argc, char **argv)
{

  ros::init(argc, argv, "publish_fromfile");

  ros::NodeHandle pubFromFileHandle;

  ros::Publisher pubFromFileObj = pubFromFileHandle.advertise<geometry_msgs::Twist>("publish_fromfile",100);

  /*!
   * TODO: make this configurable by command line arg
   */
  ros::Rate sendRate(10);

  while(ros::ok())
  {
    geometry_msgs::Twist currentOutputMsg;
    currentOutputMsg.linear.x=0.0;
    currentOutputMsg.angular.z=0.0;

    pubFromFileObj.publish(currentOutputMsg);

    ros::spinOnce();
    sendRate.sleep();
  }

  return 0;
}
