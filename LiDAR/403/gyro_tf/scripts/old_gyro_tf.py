#!/usr/bin/env python
## Simple talker demo that published std_msgs/Strings messages to the 'chatter' topic
import roslib
import rospy
import tf
from std_msgs.msg import String

from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *

from mpu6050 import mpu6050
import math
import time
#https://github.com/m-rtijn/mpu6050

#source ./devel/setup.bash
#sudo ifconfig eth0 192.168.2.100
#roslaunch sick_scan sick_tim_7xx.launch

#source ./devel/setup.bash
#rosrun rviz rviz

#source ./devel/setup.bash
#rosrun beginner_tutorials gyro_tf.py



if __name__=="__main__":
	rospy.init_node('dynamic_tf_broadcaster')
	br = tf.TransformBroadcaster()
	rate = rospy.Rate(10.0)
	sensor = mpu6050(0x68)
	sensor.set_gyro_range(0x08)
	sensor.read_gyro_range(True)
	while not rospy.is_shutdown():
		gyro_data = sensor.get_accel_data()
		quaternion = tf.transformations.quaternion_from_euler(gyro_data['x']/5, 0.0, 0.0)
		br.sendTransform((0.0, 0.0, 0.0), quaternion, rospy.Time.now(),"point_cloud", "map")
		
		#print("Acel: " + str(acel_data['x']) + ", " + str(acel_data['y']) + ", " + str(acel_data['z']))
		#print("Gyro: " + str(gyro_data['x']) + ", " + str(gyro_data['y']) + ", " + str(gyro_data['z']))
		rate.sleep()
