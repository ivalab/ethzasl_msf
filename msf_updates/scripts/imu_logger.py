#!/usr/bin/env python
# -*-coding:utf-8 -*-
"""
@file imu_logger.py
@author Yanwei Du (yanwei.du@gatech.edu)
@date 07-26-2025
@version 1.0
@license Copyright (c) 2025
@desc None
"""
#!/usr/bin/env python
import rospy
import csv
from sensor_msgs.msg import Imu
from pathlib import Path


class IMULogger:
    def __init__(self):
        rospy.init_node("imu_orientation_logger")

        self.output_dir = rospy.get_param("~output_dir", "/mnt/DATA/experiments/msf/tsrb/imu_test")
        self.dataname = rospy.get_param("~dataname", "")

        imu_num = rospy.get_param("~imu_num", "0")
        topics = []
        for i in range(imu_num):
            topics.append(rospy.get_param(f"~imu{i}_topic", f"/imu{i}"))

        # Initialize CSV writers for each topic
        self.writers = {}
        for topic in topics:
            # Extract filename from topic (e.g., "/imu/filtered" -> "imu_filtered.csv")
            filepath = self.output_dir + "/" + topic + f"/{self.dataname}_filtered_orientation.csv"
            Path(filepath).parent.mkdir(exist_ok=True, parents=True)
            f = open(filepath, "w")
            writer = csv.writer(f)
            writer.writerow(["timestamp", "x", "y", "z", "w"])  # Header
            self.writers[topic] = (writer, f)

            # Subscribe to the topic
            rospy.Subscriber(topic, Imu, self.callback, callback_args=topic)
        rospy.on_shutdown(self.shutdown)

    def callback(self, msg, topic):
        writer, _ = self.writers[topic]
        writer.writerow(
            [msg.header.stamp.to_sec(), msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        )

    def shutdown(self):
        for _, (_, f) in self.writers.items():
            f.close()
        rospy.loginfo(f"Data saved to {self.output_dir}")


if __name__ == "__main__":
    try:
        logger = IMULogger()
        rospy.spin()
        # logger.save()
    except rospy.ROSInterruptException:
        pass
