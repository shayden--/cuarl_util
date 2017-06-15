// ROS includes
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ros/console.h"
#include "std_msgs/String.h"

// yaml includes
#include "yaml-cpp/yaml.h"

// std includes
#include <fstream>

// yaml input validation function
// TODO: write unit tests and use them to develope this function
bool inputIsValid(const YAML::Node &n_u_t)
{
  return true;
}

bool fileExists(const char* file_path)
{
  // explanation of this function:
  // http://www.cplusplus.com/forum/general/1796/
  std::ifstream ifile(file_path);
  return ifile;
}

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
  pubFromFileHandle.getParam("input_file",input_file);
  ROS_INFO_STREAM("/" << node_name << "/input_file value set " << input_file);

  // load YAML and validate
  if (input_file.empty()||(fileExists(input_file.c_str())==false))
  {
    ROS_ERROR_STREAM("/"<<node_name<<"/input_file unset or not found. exit(EXIT_FAILURE)");
    exit(EXIT_FAILURE);
  }
  YAML::Node twist_node = YAML::LoadFile(input_file);
  if (inputIsValid(twist_node)!=true)
  {
    ROS_ERROR_STREAM("/"<<node_name<<"/input_file corrupt or missing expected values. exit(EXIT_FAILURE)");
    exit(EXIT_FAILURE);
  }

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
