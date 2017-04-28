#!/bin/bash

standard_mongo_file="/var/lib/mongodb/mongod.lock"
uol_mongo_file="/home/mongodb/database/mongod.lock"

if [ -f $uol_mongo_file ];
then
   #File FILE exists
   echo "ON"
else
	if [ -f $standard_mongo_file ];
	then
		#File FILE exists
		echo "ON"
	else
		echo "OFF"   
	fi	  
fi	



