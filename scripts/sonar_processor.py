#!/usr/bin/env python

"""
Multibeam ping processing and range extraction

This script subscribes to mb_msgs/Ping messages and publishes both mb_msgs/SonarScan and sensor_msgs/Image. The first is the sonar equivalent of a sensor_msgs/LaserScan, containing the estimated range measurement for each beam; the second is the pre-processed sonar image  converted to Cartesian coordinates for visualization purposes. Thanks to the multibeam library, this script can also save ping information as a PNG (image only) or as protobuf- or JSON-encoded files.

Run with 
  rosrun mb_processor sonar_processor.py ping:=/reader/ping _sonar_config_file="path-to-config-file"
"""

__author__     = "Pedro Vaz Teixeira"
__version__    = "1.0.0"
__email__      = "pvt@mit.edu"
__status__     = "Development"

# run with:


import sys
import numpy as np

import rospy

# ros - messages
from multibeam.sonar import Sonar
from sensor_msgs.msg import Image
# mb - messages
from mb_msgs.msg import Ping, SonarScan

class Processor(object):
    """
    A class to process multibeam pings
    """

    def __init__(self):
        rospy.logdebug("initializing sonar processor")

        rospy.logdebug("setting up subscribers")
        self.ping_sub = rospy.Subscriber('ping', Ping, self.ping_handler)
        rospy.logdebug("setting up publishers")
        self.scan_pub = rospy.Publisher('scan', SonarScan, queue_size=10)
        self.ping_pub = rospy.Publisher('ping_cart', Image, queue_size=10)

        self.sonar = Sonar()

    def load_config(self, cfg_file):
        """
        Load sonar configuration (see: Sonar.load_config).
        """
        rospy.logdebug("loading sonar configuration from %s", cfg_file)
        self.sonar.load_config(cfg_file)

    def ping_handler(self, ping_msg):
        """
        Callback function for new pings
        """
        rospy.logdebug("received ping")

        ping = np.copy(np.asarray(ping_msg.data))
        ping.shape = ()

        # TODO: pre-process image
        # TODO: convert to cartesian and publish
        # TODO: segment image
        # TODO: publish scans (may require new message spec?)

if __name__ == '__main__':
    rospy.myargv(argv=sys.argv)

    rospy.init_node("processor", anonymous=True)

    # create processor object
    mbp = Processor()

    if rospy.has_param('~sonar_config_file'):
        config_file = rospy.get_param('~sonar_config_file')
        mbp.load_config(config_file)
    else:
        rospy.logerr('Sonar configuration not found; must specify the \'sonar_config_file\' parameter')
        sys.exit(1)


    rospy.spin()

