#!/usr/bin/env python

import rospy
from myworkcell_core.srv import LocalizePart

#include new libraries 
import sys
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg



def client():
	rospy.wait_for_service("localize_part")

	call_service = rospy.ServiceProxy("localize_part", LocalizePart)

	result = call_service(rospy.get_param("/myworkcell_node/base_frame"))
	move_target = result.pose

	#call the go_to_pose function 
	go_to_pose(move_target, rospy.get_param("/myworkcell_node/base_frame"))

	print("after transform, result is ", result)




#create a function for motion planning, this function have 2 parameters
#pose is the  target pose you want the UR5 to move, base_frame is the frame of the pose
def go_to_pose(pose, base_frame):

	#initialize the moveit commander
	moveit_commander.roscpp_initialize(sys.argv)

	#create an object of moveit_commander.RobotCommander 
	#and moveit_commander.PlanningSceneInterface
	robot = moveit_commander.RobotCommander()
	scene = moveit_commander.PlanningSceneInterface()

	#our move group commander is "manipulator"
	group_name = "manipulator"
	group = moveit_commander.MoveGroupCommander(group_name)

	#initialie a ros publish to publish the target information
	display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)

	#set the  reference frame to base_frame
	group.set_pose_reference_frame(base_frame)

	#set the pose target to the variable pose
	group.set_pose_target(pose)
	#go to the target pose
	group.go(wait=True)
	#stop the move group commander 
	group.stop()

if __name__ == "__main__":

	base_frame = rospy.get_param("/myworkcell_node/base_frame", "world")

	rospy.set_param("/myworkcell_node/base_frame", base_frame)
	
	#initialize a node 
	rospy.init_node('move_group_python_interface_tutorial',anonymous=True)


	client()
	

	
	


