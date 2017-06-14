// ROS includes
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ros/console.h"
#include "std_msgs/String.h"

// misc includes
#include "yaml-cpp/yaml.h"

int main(int argc, char **argv)
{

  // local variables for configurable params
  double rate_param;
  int msg_buffer_len=100;
  std::string topic_name("publish_fromfile");
  std::string input_file; // TODO: implement
  // local variables for local scope use
  const std::string node_name("publish_fromfile");

  // initialize ROS 
  ros::init(argc, argv, node_name);
  // make sure to create the node handle with the node_name used to initialize
  // (so that param relative scope works)
  ros::NodeHandle pubFromFileHandle(node_name);
  pubFromFileHandle.param("rate",rate_param,1.0);
  ROS_INFO_STREAM("/" << node_name << "/rate value set " << rate_param);
  pubFromFileHandle.param<std::string>("topic_name",topic_name,"publish_fromfile");
  ROS_INFO_STREAM("/" << node_name << "/topic_name value set " << topic_name);
  pubFromFileHandle.param("msg_buffer_len",msg_buffer_len,msg_buffer_len);
  ROS_INFO_STREAM("/" << node_name << "/msg_buffer_len value set " << msg_buffer_len);

  ros::Publisher pubFromFileObj = pubFromFileHandle.advertise<geometry_msgs::Twist>(topic_name,msg_buffer_len);

  ros::Rate sendRate(rate_param);

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
