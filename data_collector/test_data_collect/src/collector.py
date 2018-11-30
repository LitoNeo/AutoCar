#!/usr/bin/python
# -*- coding:utf-8 -*-

import os
import time
import rospy
import rosbag
from sensor_msgs.msg import PointCloud2
from common_msgs.msg import AttrMsg


class LidarRecorder(object):
    def __init__(self, dirpath):
        self._bag_file = os.path.join(dirpath, "lidar.bag")
        self._bag_writer = rosbag.Bag(self._bag_file, 'w')

    def record_lidar(self, topic, data):
        self._bag_writer.write(topic, data)

    def close(self):
        self._bag_writer.close()

if __name__ == '__main__':
    rospy.init_node("collector")
    print("hello")
    rospy.spin()