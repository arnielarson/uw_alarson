#!/usr/bin/env python
# Student: Arnold Larson
# Class:  UW EEP 545

import rospy
import rosbag
import os
from ackermann_msgs.msg import AckermannDriveStamped

BAG_TOPIC = "/car/mux/ackermann_cmd_mux/output"
PUB_TOPIC = "/car/mux/ackermann_cmd_mux/input/teleop"
PUB_RATE = 20 # The rate [Hz] at which messages should be published


def follow_bag(bagfile, follow_backwards=False):

        rospy.loginfo("Loading bag: %s"%bagfile)
        rospy.loginfo("Following bag backwards: %s"%follow_backwards)
        rospy.loginfo("Publishing to: %s"%PUB_TOPIC)

	bag =  rosbag.Bag(bagfile)
        rate = rospy.Rate(PUB_RATE)
        pub = rospy.Publisher(PUB_TOPIC, AckermannDriveStamped, queue_size=100)
        
        # single iteration over bag..
        for topic, msg, t in bag.read_messages(topics=[BAG_TOPIC]):
           ## check reverse
           if follow_backwards:
               msg.drive.speed*=-1
           pub.publish(msg)
           rate.sleep()
            

if __name__ == '__main__':
	
	rospy.init_node('bag_follower', anonymous=True)

	bag_path = rospy.get_param("~bag_path", "")
	bag_filename = rospy.get_param("~bag_filename", "")
	follow_backwards = bool(rospy.get_param("~follow_backwards", "0")) 
	
	bagfile=bag_path + "/" + bag_filename
	
	follow_bag(bagfile, follow_backwards)
