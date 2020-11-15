#!/usr/bin/env python

import roslib
import rospy
import tf

import smbus
import math
import time

#import rosbag
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan

#https://github.com/ControlEverythingCommunity/L3GD20 ALSO HAS C++ LIB

#source ./devel/setup.bash
#sudo ifconfig eth0 192.168.2.100
#roslaunch sick_scan sick_tim_7xx.launch

#source ./devel/setup.bash
#rosrun rviz rviz

#source ./devel/setup.bash
#rosrun beginner_tutorials gyro_tf.py

#https://www.theconstructsim.com/read-laserscan-data/


bus = smbus.SMBus(1)
address = 0x6B

def xGyro():
	data0 = bus.read_byte_data(address, 0x28)
	data1 = bus.read_byte_data(address, 0x29)
	xGyro = data1 * 256.0 + data0
	if xGyro > 32767.0 :
		xGyro -= 65536.0
	return xGyro

def yGyro():
	data0 = bus.read_byte_data(address, 0x2A)
	data1 = bus.read_byte_data(address, 0x2B)
	yGyro = data1 * 256.0 + data0
	if yGyro > 32767.0 :
		yGyro -= 65536.0
	return yGyro

def zGyro():
	data0 = bus.read_byte_data(address, 0x2C)
	data1 = bus.read_byte_data(address, 0x2D)
	zGyro = data1 * 256.0 + data0
	if zGyro > 32767.0 :
		zGyro -= 65536.0
	return zGyro




def callback(msg):
    #print "Center: %f" % (msg.ranges[180])
    print "inc: %f" % (msg.angle_increment)

#bag = rosbag.Bag('test.bag', 'w')

if __name__=="__main__":
	rospy.init_node('dynamic_tf_broadcaster')
	br = tf.TransformBroadcaster()
	fqz = 20.0
	rate = rospy.Rate(fqz) #10Hz
	bus.write_byte_data(address, 0x20, 0x0F)
	bus.write_byte_data(address, 0x23, 0x30)
	time.sleep(0.5)
	zAngle = 0.0
	
	
	while not rospy.is_shutdown():
		rate.sleep()
		gyro = (((zGyro())/fqz)/720)
		if (zGyro()/fqz) >= 1.0 or (zGyro()/fqz) <= -1.0:
			zAngle = zAngle + gyro #/(fqz/1.0)
		quaternion = tf.transformations.quaternion_from_euler(0.0, 1.5, zAngle)
		br.sendTransform((0.0, 0.0, 0.0), quaternion, rospy.Time.now(),"point_cloud", "map")
		
		rospy.Subscriber("/scan", LaserScan, callback) #https://www.theconstructsim.com/read-laserscan-data/
		#rospy.spin()
    		
		#bag.write('dynamic_tf_broadcaster')
		#print "Test: %d" % ()
		#print "%d : %d" % (gyro, zAngle)
		#print "(%d, %d, %d)" % (xGyro(),yGyro(),zGyro())
		
