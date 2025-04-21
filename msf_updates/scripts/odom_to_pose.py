#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np
from pathlib import Path

class OdomToPoseConverter:
    def __init__(self):
        rospy.init_node('odom_to_pose')
        
        # Setup publisher and subscriber
        self.pose_pub = rospy.Publisher("/pose", PoseWithCovarianceStamped, queue_size=10)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.fused_odom_sub = rospy.Subscriber("/fused_odom", Odometry, self.fused_odom_callback)

        self.output_dir = "/mnt/DATA/experiments/msf/tsrb/odom_test"
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

    def to_pose(self, msg):
        return [msg.header.stamp.to_sec(),
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z,
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w]

    def save(self):
        self.output_dir = Path(self.output_dir)
        self.output_dir.mkdir(exist_ok=True, parents=True)
        np.savetxt(self.output_dir / "odom_pose.txt", self.odom, fmt="%.6f", header="timestamp tx ty tz qx qy qz qw")
        np.savetxt(self.output_dir / "fused_odom_pose.txt", self.fused_odom, fmt="%.6f", header="timestamp tx ty tz qx qy qz qw")
        print(f"Saved odom to {self.output_dir}")

if __name__ == '__main__':
    try:
        converter = OdomToPoseConverter()
        rospy.spin()
        converter.save()
    except rospy.ROSInterruptException:
        pass