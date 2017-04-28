#!/bin/bash

# based on idea proposed in https://community.openhab.org/t/display-log-files-in-ui/9168


HOME="/home/ami"
openhabDIR="$HOME/catkin_ws/src/ENRICHME/codes/ais/openhab"

#..........................................................
#params origFileURI length destURI
         #1             #2      #3 
function parser {

if [ -f $1 ];
then
   #File FILE exists
   echo -e "<html>\n<body>\n<pre>\n" > $3
   #tac $1  | head -n $2 >> $3
   tail -n $2 $1 | tac >> $3 
   echo -e "\n<pre>\n<body>\n<html>" >> $3
else
   echo -e "<html>\n<body>\n<pre>\n" > $3
   echo -e "File $1 does not exist..." >> $3
   echo -e "\n<pre>\n<body>\n<html>" >> $3

fi	
	
}
#..........................................................

parser /var/log/mongodb/mongodb.log      100  "${openhabDIR}"/webapps/weblogs/mongodb.log
parser "${HOME}"/.ros/latest/rosout.log  100 "${openhabDIR}"/webapps/weblogs/rosout.log
parser "${openhabDIR}"/logs/events.log   100 "${openhabDIR}"/webapps/weblogs/events.log
parser "${openhabDIR}"/logs/openhab.log  100 "${openhabDIR}"/webapps/weblogs/openhab.log

date +"%d.%m.%Y %T"


