#!/usr/bin/env python

#NOTES: 
#	- Example I found online that I got working on the Simulator, but not with the SICK Lidar
#	- It should create a collective point cloud with a max of 400 samples at a time

import rospy
from laser_assembler.srv import AssembleScans2
from sensor_msgs.msg import PointCloud2

rospy.init_node("assemble_scans_to_cloud")
rospy.wait_for_service("assemble_scans2")
assemble_scans = rospy.ServiceProxy('assemble_scans2', AssembleScans2)
pub = rospy.Publisher("/laser_point_cloud", PointCloud2, queue_size=1)

r = rospy.Rate(1)

while(True):
    try: 
	resp = assemble_scans(rospy.Time(0,0), rospy.get_rostime())
	print "Got cloud with %u points" % len(resp.cloud.data)
	pub.publish(resp.cloud)
    except rospy.ServiceException, e:
	print "Service call failed: %s"%e
    r.sleep()