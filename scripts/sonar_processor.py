#!/usr/bin/env python

import rospy
from multibeam.sonar import Sonar

from mb_msgs.msg import Ping

def callback(data):
    # do stuff here
    rospy.loginfo(rospy.get_caller_id() + "I got a ping" )

def listener():

    rospy.init_node('processor', anonymous=True)

    rospy.Subscriber("/reader/ping", Ping, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
