#!/bin/bash

# This is the bash batch script used to process the SSO test results of July 2017, and it can be
# modified in the future for upcoming test results

# NOTE: this batch script has hard-coded paths to the extraction binary, please check or update the following paths:
#
# /home/rosuser/bin/extract_to_csv
#

# USAGE:
# place all the bag file recordings in subdirectories named YYYYMMDD.results/rosbags then update line 22 & 23 with the paths.
# 

# the following section is a the general purpose batch extraction loop. If there are no special cases comment out
# line 22 and uncomment line 23

ts_start="`/bin/date`"

special_test_cases="test10b|test11b|test12b|test13b|test14b-bad|test14c|test15b|test16b|test17b|test01e|test04c"

for src_bag in `ls -A1 2017070[456].results/rosbags/*.bag | egrep -v "$special_test_cases"`
#for src_bag in `ls -A1 2017070[456].results/rosbags/*.bag`
do
  test_date="`echo $src_bag | awk -F '.' '{print $1}'`"
  test_name="`echo $src_bag | awk -F '.' '{print $3}'`"
  test_datetime="`echo $src_bag | awk -F '/' '{print $3}' | sed 's/\..*$//g'`"
  echo "Processing '$src_bag'..."
  #echo $test_date $test_name
  mkdir -p "$test_date.csvs/$test_name"
  /usr/bin/time /home/rosuser/bin/extract_to_csv -b "$src_bag" -d -o "$test_date.csvs/$test_name/$test_datetime.aris_driveline.csv"
  /usr/bin/time /home/rosuser/bin/extract_to_csv -b "$src_bag" -s -o "$test_date.csvs/$test_name/$test_datetime.aris_status.csv"
  /usr/bin/time /home/rosuser/bin/extract_to_csv -b "$src_bag" -i -o "$test_date.csvs/$test_name/$test_datetime.imu.csv"
  /usr/bin/time /home/rosuser/bin/extract_to_csv -b "$src_bag" -k -o "$test_date.csvs/$test_name/$test_datetime.imu_kalman.csv"
done

# The following section processes special cases. These test recordings were found to have accidentally captured unrelated data
# The timestamp and sequence range limits were determined manually. Ideally this section will not be needed in the future and
# lines 41 through 64 can be commented out or deleted. 

declare -a spec_test_names
declare -a spec_test_lower_ts
declare -a spec_test_lower_seq
spec_test_names=(test10b test11b test12b test13b test14b-bad test14c test15b test16b test17b test01e test04c)
spec_test_lower_ts=(1499195269 1499195521 1499195678 1499195869 1499196026 1499196170 1499196362 1499196663 1499196982 1499197270 1499197449)
spec_test_lower_seq=(68470 69726 70507 71457 72238 72956 73916 75414 77003 78438 79328)

for spec_test_index in `seq 0 10`
do 
  #echo "${spec_test_names[spec_test_index]},${spec_test_lower_ts[spec_test_index]},${spec_test_lower_seq[spec_test_index]}"
  src_bag="`find ./ -iname "*${spec_test_names[spec_test_index]}*bag"`"
  src_bag="`echo "${src_bag:2}"`" # cut out the first two characters './'

  test_date="`echo $src_bag | awk -F '.' '{print $1}'`"
  test_name="`echo $src_bag | awk -F '.' '{print $3}'`"
  test_datetime="`echo $src_bag | awk -F '/' '{print $3}' | sed 's/\..*$//g'`"
  echo "Processing '$src_bag' with constraint arguements..."
  #echo $test_date $test_name
  mkdir -p "$test_date.csvs/$test_name"
  /usr/bin/time /home/rosuser/bin/extract_to_csv -b "$src_bag" -d -o "$test_date.csvs/$test_name/$test_datetime.aris_driveline.csv" -l ${spec_test_lower_ts[spec_test_index]}
  /usr/bin/time /home/rosuser/bin/extract_to_csv -b "$src_bag" -s -o "$test_date.csvs/$test_name/$test_datetime.aris_status.csv" -l ${spec_test_lower_ts[spec_test_index]}
  /usr/bin/time /home/rosuser/bin/extract_to_csv -b "$src_bag" -i -o "$test_date.csvs/$test_name/$test_datetime.imu.csv" -l ${spec_test_lower_ts[spec_test_index]}
  /usr/bin/time /home/rosuser/bin/extract_to_csv -b "$src_bag" -k -o "$test_date.csvs/$test_name/$test_datetime.imu_kalman.csv" -f ${spec_test_lower_seq[spec_test_index]}
done

echo "Start $ts_start, Finish `/bin/date`"
