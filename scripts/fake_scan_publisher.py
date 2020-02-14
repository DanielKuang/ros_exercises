#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

import rospy
from sensor_msgs.msg import LaserScan
import math
import numpy as np
import random

pi = math.pi

def fake_scan_publisher():
    pub = rospy.Publisher('fake_scan', LaserScan, queue_size=10)
    rospy.init_node('fake_scan_publisher', anonymous=True)
    rate = rospy.Rate(20)
    prev_time = 0.
    while not rospy.is_shutdown():
        angle_max = (2./3.)*pi
        rospy.loginfo("an" + str(angle_max))
        angle_min = (-2/3.)*pi
        angle_increment = pi/300.
        range_min = 1.0
        range_max = 10.0
        ranges = [random.uniform(range_min, range_max) for i in range(int(abs(angle_max-angle_min)/angle_increment + 1))]
        capture_time = rospy.get_time()
        scan_time = capture_time - prev_time
        laserScan = LaserScan(angle_min=angle_min, angle_max=angle_max, angle_increment=angle_increment, range_min=range_min, range_max=range_max, ranges=ranges, scan_time=scan_time)
        laserScan.header.frame_id = "base_link"
        laserScan.header.stamp.secs = capture_time
        laserScan.header.stamp.nsecs = capture_time*10**-9
        rospy.loginfo(laserScan)
        pub.publish(laserScan)
        rate.sleep()
        prev_time = capture_time

if __name__ == '__main__':
    try:
        fake_scan_publisher()
    except rospy.ROSInterruptException:
        pass
