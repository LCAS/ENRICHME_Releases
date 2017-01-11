#!/usr/bin/env python

import rospy
import rosgraph

status="ON"
try:
    rosgraph.Master('/rostopic').getPid()
except:
    status="OFF"

print status
