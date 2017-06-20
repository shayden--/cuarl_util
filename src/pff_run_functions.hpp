#ifndef __publish_fromfile_h_included
#define __publish_fromfile_h_included

// ros includes
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ros/console.h"
#include "std_msgs/String.h"

// project includes

// yaml includes
#include "yaml-cpp/yaml.h"

// std includes
#include <fstream>

// function declarations
bool inputIsValid(const YAML::Node &n_u_t);
bool fileExists(const char* file_path);
int runner(int argc, char **argv); // main by any other name

#endif
