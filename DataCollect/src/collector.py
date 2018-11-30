# !/usr/bin/python
# -*- coding:utf-8 -*-

import os
import time
import rospy
import rosbag
from sensor_msgs.msg import PointCloud2


class LidarRecorder(object):
    def __init__(self, dirpath):
        self._bag_file = os.path.join(dirpath, "lidar.bag")
        self._bag_writer = rosbag.Bag(self._bag_file, 'w')

    def record_lidar(self, topic, data):
        self._bag_writer.write(topic, data)

    def close(self):
        self._bag_writer.close()




class App(object):
    rospy.init_node("Data_bag_collector")
