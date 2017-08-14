# CUARL ROS utilities

Concordia University - Aerospace Robotics Lab ROS utilities package currently consists of the following nodes:

* publish_fromfile - connects to roscore and sends geometry twist messages to configured topic
* extract_to_csv - stand-alone cli binary to extract data from sso bag recordings to csv

## Node details

### publish_fromfile

  Params:

  * rate - Desired frequency in Hz (type: double)
  * topic_name - ROS topic name to publish to (type: string)
  * msg_buffer_len - (type: int)
  * input_file - (type: string)

  Usage:

`
$ rosrun cuarl_utils publish_fromfile _rate:=2.0 _topic_name:=/aris_msg _msg_buffer_len:=200 _input_file:=~/twist2publish.yaml
`

  > note: the '_' character is required to pass a param via rosrun cli

### extract_to_csv

  Params:

  * -h [ --help ]                  display help
  * -b [ --bag-file ] arg          path to input bagfile (required)
  * -o [ --output ] arg            path to output csv (required)
  * -d [ --driveline-parse ]       generate output from aris_driveline topic
  * -s [ --status-parse ]          generate output from aris_status topic
  * -i [ --imu-parse ]             generate output from the imu/pose topic
  * -k [ --imu-kalman-parse ]      generate output from the 
                                   imu/pose/kalman_filter topic
  * -l [ --lower-bound-epoch ] arg exclude messages with a epoch timestamp less 
                                   than the passed arguement
  * -u [ --upper-bound-epoch ] arg exclude messages with a epoch timestamp 
                                   greater than the passed arguement
  * -f [ --floor-seq-num ] arg     exclude messages with a sequence number less 
                                   than the passed arguement
  * -c [ --ceiling-seq-num ] arg   exclude messages with a sequence number 
                                   greater than the passed arguement

  Usage:
`
rosuser@XenialKineticRos:~$ extract_to_csv -b sso-ft-bags/2017-07-04-15-12-03.test11b.orig.bag -o 2017-07-04-15-12-03.test11b.driveline_msgs.csv -d -l 1499195521                               
arisDrivelineMsg timestamp out of bounds
arisDrivelineMsg timestamp out of bounds
arisDrivelineMsg timestamp out of bounds
.
.
rosuser@XenialKineticRos:~$ wc -l 2017-07-04-15-12-03.test11b.driveline_msgs.csv 
435 2017-07-04-15-12-03.test11b.driveline_msgs.csv
`
