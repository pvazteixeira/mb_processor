#!/usr/bin/env python

# run with:
# rosrun mb_processor sonar_processor.py ping:=/reader/ping  

import rospy

import sys

from multibeam.sonar import Sonar

from sensor_msgs.msg import Image

from mb_msgs.msg import Ping, SonarScan

class Processor(object):
    """
    A class to process multibeam pings
    """

    def __init__(self):
        # doot doot
        self.ping_sub = rospy.Subscriber('ping', Ping, self.ping_handler)
        self.sonar = Sonar()
        self.scan_pub = rospy.Publisher('scan',SonarScan,queue_size=10)
        self.ping_pub = rospy.Publisher('ping_cart',Image,queue_size=10)

    def load_config(self, cfg_file):
        """
        Load sonar configuration (see: Sonar.load_config).
        """
        rospy.loginfo("load_config")
        self.sonar.load_config(cfg_file)

    def ping_handler(self, ping):
        """
        Callback function for new pings
        """
        rospy.loginfo("ping_handler")
        # TODO: pre-process image
        # TODO: convert to cartesian and publish
        # TODO: segment image
        # TODO: publish scans (may require new message spec?)

if __name__ == '__main__':
    rospy.myargv(argv=sys.argv)

    rospy.init_node("processor", anonymous=True)

    # create processor object
    mbp = Processor()

    if rospy.has_param('sonar_config_file'):
        config_file = rospy.get_param('sonar_config_file')
        mbp.load_config(config_file)
    else:
        rospy.logerr('Sonar configuration not found; must specify the \'sonar_config_file\' parameter')
        sys.exit(1)


    rospy.spin()

