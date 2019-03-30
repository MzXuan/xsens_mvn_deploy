#!/bin/bash
# My first script


trap finish SIGINT

function finish() {
        echo "bye bye!"
        exit
}

echo "start bash script"

#rosrun xsens_data_tf repub_trans.py

filepath="/home/xuan/Documents/Dataset_12_20/object1/pose4"
# let count=1
# let count2=2

for entry in "$filepath"/*".bag"
do
  echo "$entry"
  # extract filename
  filepath=${entry%/*}
  filebase=${entry##*/}
  filefext=${filebase##*.}
  filepref=${filebase%.*}

  echo path=${filepath}
  echo pref=${filepref} 
  echo ext=${filefext}

  human_filename="$filepath/human_$filepref.csv"
  robot_filename="$filepath/robot_$filepref.csv"

  echo human_path=${human_filename}
  echo robot_path=${robot_filename}

  sleep 3

  # start listen to the topics  
  rostopic echo -p /human_states  > ${human_filename} &
  rostopic echo -p /robot_states  > ${robot_filename} &

  #start to play certain rosbag
  rosbag play "$entry" 

  # jobs

  # var=$(jobs -p)
  # echo $var
  # echo $var | cut -d' ' -f1

  kill $(jobs -p)

  # sleep 1

  # jobs
  
done 

sleep 20