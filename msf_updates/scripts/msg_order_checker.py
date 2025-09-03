#!/usr/bin/env python
# -*-coding:utf-8 -*-
"""
@file imu_order_checker.py
@author Yanwei Du (yanwei.du@gatech.edu)
@date 08-20-2025
@version 1.0
@license Copyright (c) 2025
@desc None
"""

#!/usr/bin/env python
import rospy
import csv
from sensor_msgs.msg import Imu, Image
from pathlib import Path


class MsgOrderChecker:
    def __init__(self):
        rospy.init_node("msg_order_checker")

        self.output_dir = rospy.get_param("~output_dir", "/mnt/DATA/experiments/msf/tsrb/msg_checker")
        self.dataname = rospy.get_param("~dataname", "")

        # imu_num = rospy.get_param("~imu_num", "0")
        topics = ["/imu", "/msf_in_imu", "/camera/infra1/image_rect_raw"]
        # for i in range(imu_num):
        #     topics.append(rospy.get_param(f"~imu{i}_topic", f"/imu{i}"))

        # # Initialize CSV writers for each topic
        self.writers = {}
        self.last_timestamps = {}
        for topic in topics:
            # Extract filename from topic (e.g., "/imu/filtered" -> "imu_filtered.csv")
            filepath = self.output_dir + f"/{self.dataname}_{topic[1:4]}.csv"
            Path(filepath).parent.mkdir(exist_ok=True, parents=True)
            f = open(filepath, "w")
            writer = csv.writer(f)
            writer.writerow(["timestamp"])  # Header
            self.writers[topic] = (writer, f)

            # Subscribe to the topic
            if "imu" in topic:
                rospy.Subscriber(topic, Imu, self.callback, callback_args=topic)
            elif "image" in topic:
                rospy.Subscriber(topic, Image, self.callback, callback_args=topic)
            self.last_timestamps[topic] = -1.0
        rospy.on_shutdown(self.shutdown)

    def callback(self, msg, topic):
        writer, _ = self.writers[topic]
        writer.writerow([msg.header.stamp.to_sec()])

        cur_timestamp = msg.header.stamp.to_sec()
        if self.last_timestamps[topic] < 0:
            self.last_timestamps[topic] = cur_timestamp
            return
        last_timestamp = self.last_timestamps[topic]
        dt = cur_timestamp - last_timestamp
        if dt <= 0:
            print(f"{topic} Msg msg dis-ordered !!! Cur {cur_timestamp} v.s. last {last_timestamp}")
        self.last_timestamps[topic] = cur_timestamp

    def shutdown(self):
        for _, (_, f) in self.writers.items():
            f.close()
        rospy.loginfo(f"Data saved to {self.output_dir}")


if __name__ == "__main__":
    try:
        logger = MsgOrderChecker()
        rospy.spin()
        # logger.save()
    except rospy.ROSInterruptException:
        pass
