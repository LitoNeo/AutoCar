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


class MyTest(object):
    def __init__(self, dirpath):
        self._bag_file = os.path.join(dirpath, "myTest.bag")
        self._bag_writer = rosbag.Bag(self._bag_file, 'w')

    def record_myTest(self, topic, data):
        self._bag_writer.write(topic, data)

    def close(self):
        self._bag_writer.close()


class APP(object):
    def __init__(self):
        rospy.init_node("collector",log_level=rospy.INFO)
        self.top_dir = rospy.get_param("~file_path",default="/home/yunle/Documents/data")
        if not os.path.exists(self.top_dir):
            os.mkdir(self.top_dir)

        self._attr_record = MyTest(self.top_dir)

    def _add_sub(self):
        rospy.Subscriber("NDT_attr",AttrMsg,self._attr_cb)

    def _attr_cb(self,msg):
        self._attr_record.record_myTest("NDT_attr",msg)

    def run(self):
        rospy.spin()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self._attr_record.close()


if __name__ == '__main__':
    with APP() as app:
        app.run()