#!/usr/bin/env python

#NOTES: 
#	- Currently Not working the way it needs to
#	- This is less important right now

import roslib
import rospys
import tf

import math
import time
import numpy as np
import csv
from csv import writer

from datetime import datetime

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

fields = ['X','Y','Z']

now = datetime.now()
print("now =", now)

# dd/mm/YY H:M:S
dt_string = now.strftime("%Y_%m_%d_%H_%M_%S")
print("date and time =", dt_string)	

filename = dt_string+"_lidar_scan_data.csv"
#lidar_bag_filename = dt_string+"_lidar_bag.bag"	
#final_bag_filename = dt_string+"_final_bag.bag"	

#lidar_bag = rosbag.Bag(lidar_bag_filename, 'w')
#final_bag = rosbag.Bag(final_bag_filename, 'w')

with open(filename, 'a+') as csvfile: 
	csvwriter = csv.writer(csvfile) 
	csvwriter.writerow(fields) 

def append_row(list_of_elem):
    with open(filename, 'a+') as write_obj:
        csv_writer = writer(write_obj)
        csv_writer.writerow(list_of_elem)

def from_bits(y):
  	y -= 0x3e800000
  	return math.ldexp(float(0x800000 + y & 0x7fffff) / 0x1000000, (y - 0x800000) >> 23)

def get_point(msg, index):
	point = [0.0, 0.0, 0.0, 0.0] #[X,Y,Z,Intensity]
	for offset in range(0, 4, 1):
		bit0 = ((ord(msg.data[0+(offset*4)+(index*16)]) % 2**8) << 0 )
		bit1 = ((ord(msg.data[1+(offset*4)+(index*16)]) % 2**8) << 8 )
		bit2 = ((ord(msg.data[2+(offset*4)+(index*16)]) % 2**8) << 16)
		bit3 = ((ord(msg.data[3+(offset*4)+(index*16)]) % 2**8) << 24)
		point[offset] = from_bits(bit3+bit2+bit1+bit0)
	#point[1] = point[1]/(10**71)
	return point


def callback(msg):
	for i in range(90,270):
		point = get_point(msg, i)
		row = [point[0], point[1], point[3]]
		append_row(row)
		#print row


if __name__=="__main__":
	rospy.init_node('point_cloud_conversion')
	#pub = rospy.Publisher("/jeffrey_cloud", PointCloud, queue_size=10)
	rospy.Subscriber("/point_cloud", PointCloud2, callback) #https://www.theconstructsim.com/read-laserscan-data/
	#rospy.Subscriber("/point_cloud", PointCloud2, callback)
	#print "%d : %d" % (gyro, zAngle)
	#print "(%d, %d, %d)" % (xGyro(),yGyro(),zGyro())
	rospy.spin()


	