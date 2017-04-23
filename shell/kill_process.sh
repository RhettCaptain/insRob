#!/bin/sh

NAME=$1
echo $NAME
CORE="node_server|node_mapping_listener|node_odometry_listener|node_laser_listener"
ID=`ps -ef | grep "$NAME" | grep -v "$0" | egrep -v $CORE| grep -v "grep" | awk '{print $2}'`
echo $ID
echo "---------------"
for id in $ID
do
kill -9 $id
echo "killed $id"
done
echo "---------------"
