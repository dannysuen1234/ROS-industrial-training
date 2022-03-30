#!/usr/bin/env python

import rospy 
from fake_ar_publisher.msg import ARMarker
from myworkcell_core.srv import LocalizePart

#import the following libraries:
from geometry_msgs.msg import PoseStamped

#include these libraries 
import copy
import tf2_ros
import tf2_geometry_msgs


def callback(msg):
	global last_msg
	last_msg = msg

#create the function for the service
def server():
	s = rospy.Service("localize_part", LocalizePart, server_callback)
	rospy.loginfo("service ready")
	rospy.spin()

def server_callback(req):
	# save the last message we subscribed to the variable p
	p =last_msg

	#return None if last message is None
	if p == None:
		return None
	print("before transform, pose is ", p)

	# define the object to be transformed
	target_pose_from_cam = PoseStamped()
	
	#the header of the object to be transformed is the same as the last message
	target_pose_from_cam.header = copy.deepcopy(p.header)
	

	#Define the pose same as the pose of last message
	target_pose_from_cam.pose = copy.deepcopy(p.pose.pose)

	#do the transformation 
	tf_buffer = tf2_ros.Buffer(rospy.Duration(100.0))  # tf buffer length
	tf_listener = tf2_ros.TransformListener(tf_buffer)

	transform = tf_buffer.lookup_transform(p.header.frame_id,
                                       # source frame:
                                       target_pose_from_cam.header.frame_id,
                                       # get the tf at the time the pose was valid
                                       target_pose_from_cam.header.stamp,
                                       # wait for at most 1 second for transform, otherwise throw
                                       rospy.Duration(1.0))

	pose_transformed = tf2_geometry_msgs.do_transform_pose(target_pose_from_cam, transform)

	#return this message to the client
	return pose_transformed.pose


if __name__ == '__main__':
	last_msg = None	
	rospy.init_node("vision_node")

	rospy.Subscriber("ar_pose_marker", ARMarker, callback)
	

	server()
	rospy.loginfo("service node starting")
	rospy.spin()





