#!/usr/bin/env bash

# Get the path to the directory containing this script (and the python one):
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
# 
source /opt/ros/indigo/setup.bash
source $HOME/catkin_ws/devel/setup.bash
 
status=`python $DIR/estop_status.py` 

#status=`sudo -u ami -E bash -c 'source /opt/ros/indigo/setup.bash; python /home/ami/catkin_ws/src/ENRICHME/codes/ais/openhab/scripts/ros_status.py'`
echo $status

