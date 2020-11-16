#!/usr/bin/env python

import roslib
import rospy
import tf

import math
import time
import numpy as np

#import rosbag
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2
#from sensor_msgs.msg import PointCloud


#https://github.com/ControlEverythingCommunity/L3GD20 ALSO HAS C++ LIB

#source ./devel/setup.bash
#sudo ifconfig eth0 192.168.2.100
#roslaunch sick_scan sick_tim_7xx.launch

#source ./devel/setup.bash
#rosrun rviz rviz

#source ./devel/setup.bash
#rosrun beginner_tutorials gyro_tf.py

#https://www.theconstructsim.com/read-laserscan-data/


'''
---
header: 
  seq: 93241
  stamp: 
    secs: 1605486512
    nsecs: 850167751
  frame_id: "/point_cloud"
height: 1
width: 361
fields: 
  - 
    name: "x"
    offset: 0
    datatype: 7
    count: 1
  - 
    name: "y"
    offset: 4
    datatype: 7
    count: 1
  - 
    name: "z"
    offset: 8
    datatype: 7
    count: 1
  - 
    name: "intensity"
    offset: 12
    datatype: 7
    count: 1
is_bigendian: False
point_step: 16
row_step: 5776
data: [250, 126, 122, 63, 152, 239, 216, 191, 0, 0, 0, 0, 0, 40, 56, 70, 208, 193, 124, 63, 146, 251, 215, 191, 0,
'''

	
def from_bits(y):
  y -= 0x3e800000
  return math.ldexp(float(0x800000 + y & 0x7fffff) / 0x1000000, (y - 0x800000) >> 23)

def get_point(data, index):
	point = [0.0, 0.0, 0.0, 0.0] #[X,Y,Z,Intensity]
	offset = 4

	bit3 = ((ord(msg.data[3+offset]) % 2**8) << 24)
	bit2 = ((ord(msg.data[2+offset]) % 2**8) << 16)
	bit1 = ((ord(msg.data[1+offset]) % 2**8) << 8 )
	bit0 = ((ord(msg.data[0+offset]) % 2**8) << 0 )
	value = from_bits(bit3+bit2+bit1+bit0)

def callback(msg):
	#print (msg.data)
	#data = bytearray(str(msg.data[1]), 'utf-8')
	

	

	print data

	'''
	data30 = (msg.data[3])
	data20 = (msg.data[2])
	data10 = (msg.data[1])
	data00 = (msg.data[0])
	
	data3 = bytes(data30, 'utf-8')
	data2 = bytes(data20, 'utf-8')
	data1 = bytes(data10, 'utf-8')
	data0 = bytes(data00, 'utf-8')'''

	#print data3,data2,data1,data0

	
	#data1 = numpy.uint32(msg.data[1])
	#data2 = numpy.uint32(msg.data[2])
	#data3 = numpy.uint32(msg.data[3])
	#data = ((data3)<<24)+((data2)<<16)+((data1)<<8)+((data0)<<0)
	
	#print "Point_Size:  %d [Bytes]  |  Row_Size:  %d [Bytes]  |  P1: %f" % (msg.point_step, msg.row_step, data)
		

	#for value in 2D_Cloud.ranges:
	#tf_shift = current_far
    	
    	#print "inc: %f" % (msg.angle_increment)

#bag = rosbag.Bag('test.bag', 'w')

if __name__=="__main__":
	rospy.init_node('point_cloud_conversion')

	#pub = rospy.Publisher("/jeffrey_cloud", PointCloud, queue_size=10)
	
	rospy.Subscriber("/point_cloud", PointCloud2, callback) #https://www.theconstructsim.com/read-laserscan-data/
	
	rospy.spin()
	
	#print "%d : %d" % (gyro, zAngle)
	#print "(%d, %d, %d)" % (xGyro(),yGyro(),zGyro())
		