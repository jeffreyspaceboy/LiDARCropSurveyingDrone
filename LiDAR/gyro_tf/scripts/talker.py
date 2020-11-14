#!/usr/bin/env python
## Simple talker demo that published std_msgs/Strings messages to the 'chatter' topic
import rospy
from std_msgs.msg import String

from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *

from mpu6050 import mpu6050
import math
import time
#https://github.com/m-rtijn/mpu6050

#source ./devel/setup.bash
#rosrun beginner_tutorials talker.py

def talker():
    rospy.init_node("simple_marker", anonymous=True)

    rate = rospy.Rate(10) # 10hz

    sensor = mpu6050(0x68)

    while not rospy.is_shutdown():
	gyro_data = sensor.get_gyro_data()
	hello_str = "Y: %d" % gyro_data['y']
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

def processFeedback(feedback):
    p = feedback.pose.position
    print feedback.marker_name + " is now at " + str(p.x) + ", " + str(p.y) + ", " + str(p.z)

if __name__=="__main__":
    rospy.init_node("simple_marker", anonymous=True)
    pub = rospy.Publisher("/gyro", Marker, queue_size=10)
    
    rate = rospy.Rate(10)
    # create an interactive marker server on the topic namespace simple_marker
    #server = InteractiveMarkerServer("simple_marker")
    
    # create an interactive marker for our server
    #int_marker = InteractiveMarker()
    #int_marker.header.frame_id = "base_link"
    #int_marker.name = "my_marker"
    #int_marker.description = "Simple 1-DOF Control"
    while not rospy.is_shutdown():
	    # create a grey box marker
	    box_marker = Marker()
	    box_marker.type = Marker.ARROW
	    box_marker.header.frame_id = "/point_cloud"
	    box_marker.scale.x = 0.50
	    box_marker.scale.y = 0.10
	    box_marker.scale.z = 0.10
	    box_marker.color.r = 0.0
	    box_marker.color.g = 0.5
	    box_marker.color.b = 0.5
	    box_marker.color.a = 1.0

	    sensor = mpu6050(0x68)
	    #gyro_data = sensor.get_gyro_data()
            gyro_data = sensor.get_accel_data()
	    box_marker.pose.orientation.z = gyro_data['x']/10;
	    box_marker.pose.orientation.y = gyro_data['y']/10;
	    box_marker.pose.orientation.x = 0.0 #gyro_data['z']/1;
	    box_marker.pose.orientation.w = 0.9;


	    # create a non-interactive control which contains the box
	    #box_control = InteractiveMarkerControl()
	    #box_control.always_visible = True
	    #box_control.markers.append( box_marker )

	    # add the control to the interactive marker
	    #int_marker.controls.append( box_control )
	    #int_marker.controls.append( box_marker )

	    # create a control which will move the box
	    # this control does not contain any markers,
	    # which will cause RViz to insert two arrows
	    #rotate_control = InteractiveMarkerControl()
	    #rotate_control.name = "move_x"
	    #rotate_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS

	    # add the control to the interactive marker
	    #int_marker.controls.append(rotate_control);

	    # add the interactive marker to our collection &
	    # tell the server to call processFeedback() when feedback arrives for it
	    #server.insert(int_marker, processFeedback)

	    # 'commit' changes and send to all clients
	    #server.applyChanges()
	    rospy.loginfo(box_marker);
	    pub.publish(box_marker);
	    rate.sleep()

    #rospy.spin()


