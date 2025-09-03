#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import (
    PoseWithCovarianceStamped,
    TransformStamped,
)
import numpy as np
from pathlib import Path
import tf2_ros


class OdomToPoseConverter:
    def __init__(self):
        rospy.init_node("odom_to_pose")

        # Setup publisher and subscriber
        self.pose_pub = rospy.Publisher("/pose", PoseWithCovarianceStamped, queue_size=10)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.fused_odom_sub = rospy.Subscriber("/fused_odom", Odometry, self.fused_odom_callback)

        self.output_dir = rospy.get_param("~output_dir", "/mnt/DATA/experiments/msf/tsrb/odom_test")
        self.dataname = rospy.get_param("~dataname", "")
        self.enable_tf = rospy.get_param("~enable_tf", False)
        self.odom_frame = rospy.get_param("~odom_frame", "odom")
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.br = tf2_ros.TransformBroadcaster()
        self.odom = []
        self.fused_odom = []

    def odom_callback(self, odom_msg):
        self.odom.append(self.to_pose(odom_msg))
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header = odom_msg.header
        pose_msg.pose.pose = odom_msg.pose.pose
        pose_msg.pose.covariance = odom_msg.pose.covariance
        self.pose_pub.publish(pose_msg)

    def fused_odom_callback(self, msg):
        self.fused_odom.append(self.to_pose(msg))

        if self.enable_tf:
            transform = TransformStamped()
            transform.header.stamp = msg.header.stamp
            transform.header.frame_id = self.odom_frame
            transform.child_frame_id = self.base_frame
            transform.transform.translation = msg.pose.pose.position
            transform.transform.rotation = msg.pose.pose.orientation

            self.br.sendTransform(transform)

    def to_pose(self, msg):
        return [
            msg.header.stamp.to_sec(),
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        ]

    def save(self):
        self.output_dir = Path(self.output_dir)
        self.output_dir.mkdir(exist_ok=True, parents=True)
        prefix = str(self.output_dir) + f"/{self.dataname}"
        np.savetxt(prefix + "_odom_pose.txt", self.odom, fmt="%.6f", header="timestamp tx ty tz qx qy qz qw")
        np.savetxt(
            prefix + "_AllFrameTrajectory.txt",
            self.fused_odom,
            fmt="%.6f",
            header="timestamp tx ty tz qx qy qz qw",
        )
        print(f"Saved odom to {self.output_dir}")


if __name__ == "__main__":
    try:
        converter = OdomToPoseConverter()
        rospy.spin()
        converter.save()
    except rospy.ROSInterruptException:
        pass
