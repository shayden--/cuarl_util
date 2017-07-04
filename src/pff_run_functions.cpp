// ROS includes
/*#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ros/console.h"
#include "std_msgs/String.h"

// yaml includes
#include "yaml-cpp/yaml.h"

// std includes
#include <fstream>
*/

#include "pff_run_functions.hpp"

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

// it is possible for this function to silently fail, which is/results in a bug
// TODO: fix silent failure cases (when yaml_node.IsMap() evaluates false)
void setPublishVals(double &x, double &z, double& d,const YAML::Node &yaml_node)
{
  if (yaml_node.IsMap())
  {
    d=yaml_node["duration"].as<double>();
    x=yaml_node["twist"]["linear"]["x"].as<double>();
    z=yaml_node["twist"]["angular"]["z"].as<double>();
  }
}

int runner(int argc, char **argv)
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
  pubFromFileHandle.param("rate",rate_param,5.0);
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
    ROS_ERROR_STREAM("/"<<node_name<<"/input_file unset or not found. exiting");
    return(EXIT_FAILURE);
  }
  YAML::Node twist_node = YAML::LoadFile(input_file);
  if (inputIsValid(twist_node)!=true)
  {
    ROS_ERROR_STREAM("/"<<node_name<<"/input_file corrupt or missing expected values. exiting");
    return(EXIT_FAILURE);
  }

  ros::Publisher pubFromFileObj = pubFromFileHandle.advertise<geometry_msgs::Twist>(topic_name,msg_buffer_len);

  ros::Rate sendRate(rate_param);
  // sleep once before publishing so that existing subscribers have time to connect
  sendRate.sleep();

  std::size_t yaml_seq_index=0;
  double curr_duration=0.0,max_duration=0.0,curr_lin_x=0.0,curr_ang_z=0.0;
  // set the initial message properties
  setPublishVals(curr_lin_x, curr_ang_z, max_duration,twist_node[yaml_seq_index]); 
  ROS_INFO_STREAM("vals set: "<<curr_lin_x<<","<<curr_ang_z<<","<<max_duration);

  while(ros::ok()&&(yaml_seq_index<twist_node.size()))
  {
    /*
    float curr_duration,curr_lin_x,curr_ang_z;
    max_duration=twist_node[yaml_seq_index]["duration"].as<double>();
    curr_lin_x=twist_node[yaml_seq_index]["twist"]["linear"]["x"].as<double>();
    curr_ang_z=twist_node[yaml_seq_index]["twist"]["angular"]["z"].as<double>();
    ROS_INFO_STREAM("Object type: " << twist_node[yaml_seq_index].Type());
    */

    // TODO: do some thinking about the logic placement in regards to the duration and timing

    if (curr_duration<=max_duration){
      // rate_param given in hz, so increment current duration 
      // by rate_period each step (1/rate_param)
      curr_duration+=(1.0/rate_param);
      ROS_INFO_STREAM("curr_duration now "<<curr_duration);
    }
    else
    {
      // reset current duration, increment yaml sequence index and retrieve the data
      curr_duration=0.0;
      yaml_seq_index++;
      setPublishVals(curr_lin_x, curr_ang_z, max_duration,twist_node[yaml_seq_index]); 
      // log it
      ROS_INFO_STREAM("Parsed sequence["<<yaml_seq_index<<"] duration: "<<max_duration<<", curr_lin_x: "<<curr_lin_x<<", curr_ang_z: "<<curr_ang_z);
    }

    geometry_msgs::Twist currentOutputMsg;
    currentOutputMsg.linear.x=curr_lin_x;
    currentOutputMsg.angular.z=curr_ang_z;

    //if (pubFromFileObj.publish(currentOutputMsg);
    pubFromFileObj.publish(currentOutputMsg);

    ros::spinOnce();
 
    sendRate.sleep();

  }

  // once the file processing loop finishes, send some(3) stop commands
  // TODO: make this disablable (default: true) via passed option 
  std::size_t stop_seq_index=0;
  while(ros::ok()&&(stop_seq_index<3))
  {

    // explicitly set all values '0' for safety reasons
    geometry_msgs::Twist currentOutputMsg;
    currentOutputMsg.linear.x=0;
    currentOutputMsg.linear.y=0;
    currentOutputMsg.linear.z=0;
    currentOutputMsg.angular.x=0;
    currentOutputMsg.angular.y=0;
    currentOutputMsg.angular.z=0;

    //if (pubFromFileObj.publish(currentOutputMsg);
    ROS_INFO_STREAM("Sending '[0,0,0][0,0,0]' twist...");

    pubFromFileObj.publish(currentOutputMsg);
    stop_seq_index++;

    ros::spinOnce();
 
    sendRate.sleep();

  }

  // return success
  return 0;
}
