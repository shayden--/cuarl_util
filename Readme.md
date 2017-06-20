# CUARL ROS utilities

Concordia University - Aerospace Robotics Lab ROS utilities package currently consists of the following nodes:

* publish_fromfile

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
