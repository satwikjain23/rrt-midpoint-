#!/usr/bin/env python3
import rospy
import math
import numpy as np
from race.msg import perception
from race.msg import slam
from std_msgs.msg import Float32MultiArray
from race.msg import final_coordinates
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion, quaternion_from_euler


def callback(data):
	global roll, pitch, yaw
	orientation_q = data.pose.orientation
	orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
	(roll, pitch, yaw) = euler_from_quaternion (orientation_list)
	print(math.degrees(yaw))
if __name__ == '__main__':
	c=0
	print("Hokuyo LIDAR node started")
	rospy.init_node('dist_finder',anonymous = True)
	rospy.Subscriber("/gt_pose",PoseStamped, callback)
	
	rospy.spin()