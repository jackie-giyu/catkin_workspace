#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np


LiDAR_raw_array = []


def LiDAR_callback(data):
    global LiDAR_raw_array

    LiDAR_raw_array = data.ranges
    LiDAR_raw_array = np.asarray(LiDAR_raw_array)
    LiDAR_raw_array = np.reshape(LiDAR_raw_array, (1,-1))
    # rospy.loginfo(LiDAR_raw_array)

def your_algorithm(lidar_array):
   return [0.0, -2.5]

if __name__ == '__main__':
    rospy.init_node("testNode", anonymous=True)

    scan_topic_red = rospy.get_param("~scan_topic_red")
    rospy.Subscriber(scan_topic_red, LaserScan, LiDAR_callback)

    drive_topic = rospy.get_param("testNode_drive_topic")
    drive_pub_red = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=10)
    rospy.wait_for_message(scan_topic_red, LaserScan)
    while not rospy.is_shutdown():
        command = your_algorithm(LiDAR_raw_array)
        ack_msg = AckermannDriveStamped()
        ack_msg.header.stamp = rospy.Time.now()
        ack_msg.drive.steering_angle = command[0]
        ack_msg.drive.speed = command[1]
        drive_pub_red.publish(ack_msg)
