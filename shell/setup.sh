#! /bin/sh
#get and set path
SRC_PATH=$(cd `dirname $0`;cd ..; pwd)
DST_PATH=~/.nav

#update userpath
REP=s/#USER#/${USER}/
cp ${SRC_PATH}/shell/.laser_on.launch ${SRC_PATH}/pkg_msgs/launch/laser_on.launch
sed -i ${REP}  ${SRC_PATH}/pkg_msgs/launch/laser_on.launch
#setup files
mkdir -p ${DST_PATH}
cp ${SRC_PATH}/shell/kill_process.sh ${DST_PATH}/kill_process.sh 



