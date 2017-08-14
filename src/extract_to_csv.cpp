// std includes
#include <iostream>
#include <fstream>

// boost
#include <boost/exception/all.hpp>
#include <boost/program_options.hpp>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

// ros includes
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosbag/query.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/Imu.h>
#include "aris_msgs/arisDrivelineMsg.h"
#include "aris_msgs/arisStatusMsg.h"

enum int_compare_enum
{
  disabled=0,
  lower_bound,
  upper_bound,
  range_bound
};

// function declarations
int_compare_enum set_test_type(uint32_t, uint32_t);
bool check_int_bounds(int_compare_enum, uint32_t, uint32_t, uint32_t);

int main(int argc, char **argv)
{

  bool driveline_parse=false;
  bool status_parse=false;
  bool imu_parse=false;
  bool imu_kf_parse=false;
  bool help_arg=false;
  uint32_t lower_epoch_ts=0,upper_epoch_ts=0,floor_seq_num=0,ceiling_seq_num=0;
  int_compare_enum ts_compare=disabled,seq_compare=disabled;

  std::string bag_file;
  std::string output_csv;
  std::string src_topic;
  std::string csv_header;

  // evaluate cli options for input and output file paths
  namespace po = boost::program_options;
  po::options_description args_desc("Rosbag data extraction options");
  args_desc.add_options()
    ("help,h",po::bool_switch(&help_arg),"display help")
    ("bag-file,b",po::value<std::string>(&bag_file)->required(),"path to input bagfile (required)")
    ("output,o",po::value<std::string>(&output_csv)->required(),"path to output csv (required)")
    ("driveline-parse,d",po::bool_switch(&driveline_parse),"generate output from aris_driveline topic")
    ("status-parse,s",po::bool_switch(&status_parse),"generate output from aris_status topic")
    ("imu-parse,i",po::bool_switch(&imu_parse),"generate output from the imu/pose topic")
    ("imu-kalman-parse,k",po::bool_switch(&imu_kf_parse),"generate output from the imu/pose/kalman_filter topic")
    ("lower-bound-epoch,l",po::value<uint32_t>(&lower_epoch_ts),"exclude messages with a epoch timestamp less than the passed arguement")
    ("upper-bound-epoch,u",po::value<uint32_t>(&upper_epoch_ts),"exclude messages with a epoch timestamp greater than the passed arguement")
    ("floor-seq-num,f",po::value<uint32_t>(&floor_seq_num),"exclude messages with a sequence number less than the passed arguement")
    ("ceiling-seq-num,c",po::value<uint32_t>(&ceiling_seq_num),"exclude messages with a sequence number greater than the passed arguement")
    ;

  try
  {
    po::variables_map avm;
    po::store(po::parse_command_line(argc,argv,args_desc),avm);
    po::notify(avm);
  }
  catch (boost::exception const& e)
  {
    std::cerr << "Error: missing required arguments." << std::endl;
    std::cout << args_desc << std::endl;
    exit(1);
  }

  // set the test types based on passed arguements
  ts_compare=set_test_type(lower_epoch_ts,upper_epoch_ts);
  seq_compare=set_test_type(floor_seq_num,ceiling_seq_num);

  // select the topic and header line
  if (driveline_parse)
  {
    src_topic="aris_driveline";
    csv_header="timestamp,seq,leftMotorData.rpm,leftMotorData.current,rightMotorData.rpm,rightMotorData.current,contactors,brakes";
  }
  else if (status_parse) 
  {
    src_topic="aris_status";
    csv_header="timestamp,seq,busVoltage";
  }
  else if (imu_parse)
  {
    src_topic="imu/pose";
    csv_header="timestamp,seq,orientation.x,orientation.y,orientation.z,orientation.w,angular_velocity.x,angular_velocity.y,angular_velocity.z,linear_acceleration.x,linear_acceleration.y,linear_acceleration.z";
  } 
  else if (imu_kf_parse)
  {
    src_topic="imu/pose/kalman_filter";
    csv_header="timestamp,seq,orientation.x,orientation.y,orientation.z,orientation.w,orientation_covar.0,orientation_covar.1,orientation_covar.2,orientation_covar.3,orientation_covar.4,orientation_covar.5,orientation_covar.6,orientation_covar.7,orientation_covar.8";
  }
  else if (help_arg)
  {
    // print out help and exit successfully
    std::cout << args_desc << std::endl;
    exit(0);
  }
  else
  {
    // no options given, can't determine action
    std::cout << args_desc << std::endl;
    std::cerr << "Error: No topics selected to extract. Specify one of -d -i -s" << std::endl;
    exit(1);
  }

  std::vector<std::string> topics;
  topics.push_back(src_topic);

  rosbag::Bag ip_bag;
  ip_bag.open(bag_file, rosbag::bagmode::Read);
  std::ofstream csv_fstream;
  csv_fstream.open(output_csv.c_str(),std::ios::out);

  rosbag::View ip_view(ip_bag, rosbag::TopicQuery(topics));

  // write out csv header 
  csv_fstream << csv_header << std::endl;

  std::string last_msg_def;
  foreach(rosbag::MessageInstance const m, ip_view)
  {
    if (driveline_parse)
    {
      aris_msgs::arisDrivelineMsg::ConstPtr m_dr = m.instantiate<aris_msgs::arisDrivelineMsg>();
      if (m_dr==NULL)
      {
        std::cerr << "arisDrivelineMsg null" << std::endl;
      }
      else if (check_int_bounds(ts_compare,lower_epoch_ts,upper_epoch_ts,m_dr->hdr.stamp.sec)==false)
      {
        std::cerr << "arisDrivelineMsg timestamp out of bounds" << std::endl;
      }
      else if (check_int_bounds(seq_compare,floor_seq_num,ceiling_seq_num,m_dr->hdr.seq)==false)
      {
        std::cerr << "arisDrivelineMsg sequence number out of bounds." << std::endl;
      }
      else
      {
        csv_fstream << m_dr->hdr.stamp << "," << m_dr->hdr.seq << ","; // << std::endl;
        csv_fstream << m_dr->leftMotorData.rpm << "," << m_dr->leftMotorData.current << ",";
        csv_fstream << m_dr->rightMotorData.rpm << "," << m_dr->rightMotorData.current << ",";
        csv_fstream << std::boolalpha << (bool)m_dr->contactors << "," << (bool)m_dr->brakes << std::endl; 
      }
    }
    else if (status_parse) 
    {
      aris_msgs::arisStatusMsg::ConstPtr m_as = m.instantiate<aris_msgs::arisStatusMsg>();
      if ((m_as==NULL)&&check_int_bounds(ts_compare,lower_epoch_ts,upper_epoch_ts,m_as->hdr.stamp.sec))
      {
        std::cerr << "arisStatusMsg null." << std::endl;
      }
      else if (check_int_bounds(ts_compare,lower_epoch_ts,upper_epoch_ts,m_as->hdr.stamp.sec)==false)
      {
        std::cerr << "arisStatusMsg timestamp out of bounds." << std::endl;
      } 
      else if (check_int_bounds(seq_compare,floor_seq_num,ceiling_seq_num,m_as->hdr.seq)==false)
      {
        std::cerr << "arisStatusMsg sequence number out of bounds." << std::endl;
      } 
      else
      {
        csv_fstream << m_as->hdr.stamp << "," << m_as->hdr.seq << ","; // << std::endl;
        csv_fstream << m_as->busVoltage << std::endl;
      }
    } 
    else if (imu_parse)
    {
      sensor_msgs::Imu::ConstPtr m_ip = m.instantiate<sensor_msgs::Imu>();
      //std_msgs::Header::ConstPtr m_hdr = m.instantiate<arisDrivelineMsg>();
      if (m_ip==NULL)
      {
        std::cerr << "sensor::Imu msg null." << std::endl;
      } 
      else if (check_int_bounds(ts_compare,lower_epoch_ts,upper_epoch_ts,m_ip->header.stamp.sec)==false)
      {
        std::cerr << "sensor::Imu msg timestamp out of bounds." << std::endl;
      } 
      else if (check_int_bounds(seq_compare,floor_seq_num,ceiling_seq_num,m_ip->header.seq)==false)
      {
        std::cerr << "sensor::Imu msg sequence number out of bounds." << std::endl;
      } 
      else
      {
        // write to the files stream if no expected errors
        csv_fstream << m_ip->header.stamp << "," << m_ip->header.seq << ","; // << std::endl;
        csv_fstream << m_ip->orientation.x << "," << m_ip->orientation.y << "," << m_ip->orientation.z << "," << m_ip->orientation.w << ",";
        csv_fstream << m_ip->angular_velocity.x << "," << m_ip->angular_velocity.y << "," << m_ip->angular_velocity.z << ",";
        csv_fstream << m_ip->linear_acceleration.x << "," << m_ip->linear_acceleration.y << "," << m_ip->linear_acceleration.z << std::endl;
      }
    } 
    else if (imu_kf_parse)
    {
      sensor_msgs::Imu::ConstPtr m_ip = m.instantiate<sensor_msgs::Imu>();
      //std_msgs::Header::ConstPtr m_hdr = m.instantiate<arisDrivelineMsg>();
      if (m_ip==NULL)
      {
        std::cerr << "sensor::Imu msg null." << std::endl;
      } 
      else if (check_int_bounds(ts_compare,lower_epoch_ts,upper_epoch_ts,m_ip->header.stamp.sec)==false)
      {
        std::cerr << "sensor::Imu msg timestamp out of bounds." << std::endl;
      } 
      else if (check_int_bounds(seq_compare,floor_seq_num,ceiling_seq_num,m_ip->header.seq)==false)
      {
        std::cerr << "sensor::Imu msg sequence number out of bounds." << std::endl;
      } 
      else
      {
        csv_fstream << m_ip->header.stamp << "," << m_ip->header.seq << ","; 
        csv_fstream << m_ip->orientation.x << "," << m_ip->orientation.y << "," << m_ip->orientation.z << "," << m_ip->orientation.w << ",";
        csv_fstream << m_ip->orientation_covariance[0] << "," << m_ip->orientation_covariance[1] << "," << m_ip->orientation_covariance[2] << ",";
        csv_fstream << m_ip->orientation_covariance[3] << "," << m_ip->orientation_covariance[4] << "," << m_ip->orientation_covariance[5] << ",";
        csv_fstream << m_ip->orientation_covariance[6] << "," << m_ip->orientation_covariance[7] << "," << m_ip->orientation_covariance[8] << std::endl;
      }
    }
  }

  ip_bag.close();
  csv_fstream.close();

  return 0;
}


int_compare_enum set_test_type(uint32_t lower_int, uint32_t upper_int)
{
  int_compare_enum return_type=disabled;

  if ((lower_int!=0)&&(upper_int!=0))
  {
    return_type=range_bound;
    //std::cout << "Bounds set [" << lower_int << "," << upper_int << "]" << std::endl;
  }
  else if ((lower_int!=0)&&(upper_int==0))
  {
    return_type=lower_bound;
    //std::cout << "Bounds set [" << lower_int << ",>" << std::endl;
  }
  else if ((lower_int==0)&&(upper_int!=0))
  {
    return_type=upper_bound;
    //std::cout << "Bounds set <," << lower_int << "]" << std::endl;
  }
  else
  {
    //std::cout << "Bounds set [" << lower_int << "," << upper_int << "]" << std::endl;
  }
  return return_type;
}

//bool check_epoch_ts(ts_compare_enum test_type, uint32_t lower_ts, uint32_t upper_ts, uint32_t test_ts)
bool check_int_bounds(int_compare_enum test_type, uint32_t lower_int, uint32_t upper_int, uint32_t test_ts)
{
  bool test_passed=false;

  if ((test_type==range_bound)&&(lower_int<test_ts)&&(test_ts<upper_int))
  {
    test_passed=true;
  }
  else if ((test_type==lower_bound)&&(lower_int<test_ts))
  {
    test_passed=true;
  } 
  else if ((test_type==upper_bound)&&(test_ts<upper_int))
  {
    test_passed=true;
  } 
  else if (test_type==disabled)
  {
    test_passed=true;
  }

  return test_passed;
}
