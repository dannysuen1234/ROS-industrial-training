#!/usr/bin/env python
import rospy
from myworkcell_core.srv import LocalizePart
from myworkcell_core.srv import PlanCartesianPath
import copy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryGoal



#include new libraries 
import sys
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

move_target = 0

def client():
	global move_target
	rospy.wait_for_service("localize_part")

	call_service = rospy.ServiceProxy("localize_part", LocalizePart)

	result = call_service(rospy.get_param("/myworkcell_node/base_frame"))
	move_target = result.pose

	#call the go_to_pose function 
	go_to_pose(move_target, rospy.get_param("/myworkcell_node/base_frame"))

	print("after transform, result is ", result)


def testing():
	global move_target
	robot = moveit_commander.RobotCommander()
	scale = 1
	group_name = "manipulator"
	group = moveit_commander.MoveGroupCommander(group_name)

	#initialie a ros publish to publish the target information
	display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)
	waypoints = []

	wpose = move_target
	wpose.position.z -= scale * 0.1  # First move up (z)
	wpose.position.y += scale * 0.2  # and sideways (y)
	waypoints.append(copy.deepcopy(wpose))

	wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
	waypoints.append(copy.deepcopy(wpose))

	wpose.position.y -= scale * 0.1  # Third move sideways (y)
	waypoints.append(copy.deepcopy(wpose))

	print("waypoints", waypoints)

	# We want the Cartesian path to be interpolated at a resolution of 1 cm
	# which is why we will specify 0.01 as the eef_step in Cartesian
	# translation.  We will disable the jump threshold by setting it to 0.0 disabling:
	(plan, fraction) = group.compute_cartesian_path(
		                           waypoints,   # waypoints to follow
		                           0.01,        # eef_step
		                           0.0)         # jump_threshold

	print("plan is", plan)
	#display_trajectory = moveit_msgs.msg.DisplayTrajectory()rosrun myworkcell_core adv_myworkcell_node
	#display_trajectory.trajectory_start = robot.get_current_state()
	#display_trajectory.trajectory.append(plan)
	# Publish
	#display_trajectory_publisher.publish(display_trajectory)
	group.execute(plan, wait =True)
	print("executed")

def client_2():
	global move_target

	#call the service /adv_plan_path
	rospy.wait_for_service("/adv_plan_path")
	call_service = rospy.ServiceProxy("/adv_plan_path", PlanCartesianPath)
	result = call_service(move_target)

	# initialize a vaiable goal and set the trajectory as the trajectory of the result from the service call
	goal = FollowJointTrajectoryGoal()
	goal.trajectory = result.trajectory

	#send the goal to the actionlib and wait for the result
	ac.send_goal(goal)
	ac.wait_for_result()
	print("advanced path planning done ")
	

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
	
	ac = actionlib.SimpleActionClient("joint_trajectory_action", FollowJointTrajectoryAction)
	


	client()
	
	print("1 ends here")

	client_2()

	print("end")
	
	


