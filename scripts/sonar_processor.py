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

import sys
import numpy as np

import cv2

import rospy

# multibeam package
from multibeam.sonar import Sonar
from multibeam.utils import *

# ros - messages
from sensor_msgs.msg import Image
# mb - messages
from mb_msgs.msg import Ping, SonarScan
# ros - image marshalling (cv2<->image)
from cv_bridge import CvBridge, CvBridgeError

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

        self.bridge = CvBridge() # used to deserialize sensor_msgs/Image
        self.sonar = Sonar()

    def load_config(self, cfg_file):
        """
        Load sonar configuration (see: Sonar.load_config).
        """
        rospy.logdebug("loading sonar configuration from %s", cfg_file)
        self.sonar.load_config(cfg_file)

    def update_config(self, msg):
        """
        update sonar object if sonar window has changed.
        """
        min_range = msg.ranges[0]
        max_range = msg.ranges[-1]

        if (self.sonar.min_range != min_range) or (self.sonar.max_range != max_range):
            # window parameters have changed
            self.sonar.reset_window(min_range, max_range, 0.02)

        if self.sonar.rx_gain != msg.rx_gain:
            # receiver gain has changed
            self.sonar.rx_gain = msg.rx_gain

    def ping_handler(self, ping_msg):
        """
        Callback function for new pings
        """
        rospy.logdebug("received ping")

        # check if we need to update the sonar object

        # get image
        pingb = self.bridge.imgmsg_to_cv2(ping_msg.image)
        ping = pingb/255.0

        # cv2.imwrite('ping_polar.png', ping.astype(np.uint8))

        self.update_config(ping_msg)

        # pre-process image
        ping2 = self.sonar.preprocess(ping, False)
        # cv2.imwrite('ping_polar_pp.png', ping_deconv.astype(np.uint8))
        ping2c = self.sonar.to_cart(ping)
        ping2c*=255.0

        # convert to cartesian and publish
        cart_ping_msg = self.bridge.cv2_to_imgmsg(ping2c.astype(np.uint8), encoding='8UC1')
        cart_ping_msg.header = ping_msg.image.header
        self.ping_pub.publish(cart_ping_msg)

        # segment image
        bin_length = (ping_msg.ranges[-1] - ping_msg.ranges[0])/(ping_msg.num_bins + 0.0)
        pulse = get_template(dr=bin_length)
        idx = segment_smap(ping2, pulse, 1.0)
        idx[idx < 1] = -1

        ranges = np.copy(idx).astype(float)
        ranges[ranges > 0] *= bin_length
        ranges[ranges > 0] += ping_msg.ranges[0]*np.ones_like(ranges[ranges > 0])
        intensities = ping2[idx, range(0, ping_msg.num_beams)]
        intensities[idx < 1] = 0

        # publish scans
        scan_msg = SonarScan()
        scan_msg.header = cart_ping_msg.header
        scan_msg.hfov = 5.236e-3
        scan_msg.vfov = 17.453e-3
        scan_msg.range_min = ping_msg.ranges[0]
        scan_msg.range_max = ping_msg.ranges[-1]
        scan_msg.ranges = ranges
        scan_msg.azimuths = np.copy(ping_msg.azimuths)
        scan_msg.intensities = intensities
        self.scan_pub.publish(scan_msg)

        # TODO: publish LaserScan message just for visualization purposes.

if __name__ == '__main__':
    rospy.myargv(argv=sys.argv)

    rospy.init_node("processor", anonymous=True)

    # create processor object
    mb_handler = Processor()

    if rospy.has_param('~sonar_config_file'):
        mb_handler.load_config(rospy.get_param('~sonar_config_file'))
    else:
        rospy.logerr('Sonar configuration not found; must set the \'sonar_config_file\' parameter')
        sys.exit(1)


    rospy.spin()
