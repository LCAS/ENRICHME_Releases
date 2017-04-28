#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool

class EStopState:
    def __init__(self):
        self.sub_state = rospy.Subscriber("/force_stop_state", Bool, self.state_cb)
        self.timer_cb  = rospy.Timer(rospy.Duration(1), self.timer_cb)

    def state_cb(self, msg):
        if (msg.data):
            print "ON"
        else:
            print "OFF"
        rospy.signal_shutdown("state ok")

    def timer_cb(self, evt):
        print "UNKNOWN"
        rospy.signal_shutdown("timeout")

if __name__ == "__main__":
    rospy.init_node("get_estop_status")
    node = EStopState()

    rospy.spin()

