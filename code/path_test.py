#!/usr/bin/env python
"""
Path Planning Script for Lab 8
Author: Valmik Prabhu
"""

import sys
import rospy
import numpy as np
import math

from moveit_msgs.msg import OrientationConstraint
from geometry_msgs.msg import PoseStamped

from path_planner import PathPlanner
from baxter_interface import Limb
from controller import Controller
from baxter_interface import gripper as robot_gripper
# from intera_interface import Limb

def main():
	"""
	Main Script
	"""

	# Make sure that you've looked at and understand path_planner.py before starting

	plannerR = PathPlanner("right_arm")
	plannerL = PathPlanner("left_arm")

	right_gripper = robot_gripper.Gripper('right')
	left_gripper = robot_gripper.Gripper('left')

	# Kp = 0.1 * np.array([0.3, 2, 1, 1.5, 2, 2, 3]) # Stolen from 106B Students
	# Kd = 0.01 * np.array([2, 1, 2, 0.5, 0.5, 0.5, 0.5]) # Stolen from 106B Students
	# Ki = 0.01 * np.array([1, 1, 1, 1, 1, 1, 1]) # Untuned
	# Kw = np.array([0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9]) # Untuned

	# right_arm = Limb("right")
	# controller = Controller(Kp, Kd, Ki, Kw, right_arm)

	##
	## Add the obstacle to the planning scene here
	##

	
	obstacle1 = PoseStamped()
	obstacle1.header.frame_id = "base"

	#x, y, and z position
	obstacle1.pose.position.x = 0.4
	obstacle1.pose.position.y = 0
	obstacle1.pose.position.z = -0.29

	#Orientation as a quaternion
	obstacle1.pose.orientation.x = 0.0
	obstacle1.pose.orientation.y = 0.0
	obstacle1.pose.orientation.z = 0.0
	obstacle1.pose.orientation.w = 1
	plannerR.add_box_obstacle(np.array([1.2, 1.2, .005]), "tableBox", obstacle1)

	# obstacle2 = PoseStamped()
	# obstacle2.header.frame_id = "wall"

	# #x, y, and z position
	# obstacle2.pose.position.x = 0.466
	# obstacle2.pose.position.y = -0.670
	# obstacle2.pose.position.z = -0.005

	# #Orientation as a quaternion
	# obstacle2.pose.orientation.x = 0.694
	# obstacle2.pose.orientation.y = -0.669
	# obstacle2.pose.orientation.z = 0.251
	# obstacle2.pose.orientation.w = -0.092
	# planner.add_box_obstacle(np.array([0.01, 2, 2]), "wall", obstacle2)
	
	# #Create a path constraint for the arm
	# #UNCOMMENT FOR THE ORIENTATION CONSTRAINTS PART
	# orien_const = OrientationConstraint()
	# orien_const.link_name = "right_gripper";
	# orien_const.header.frame_id = "base";
	# orien_const.orientation.y = -1.0;
	# orien_const.absolute_x_axis_tolerance = 0.1;
	# orien_const.absolute_y_axis_tolerance = 0.1;
	# orien_const.absolute_z_axis_tolerance = 0.1;
	# orien_const.weight = 1.0;

	while not rospy.is_shutdown():
		while not rospy.is_shutdown():
			try:
				goal_3 = PoseStamped()
				goal_3.header.frame_id = "base"

				#x, y, and z position
				goal_3.pose.position.x = 0.440
				goal_3.pose.position.y = -0.012
				goal_3.pose.position.z = 0.549

				#Orientation as a quaternion
				goal_3.pose.orientation.x = -0.314
				goal_3.pose.orientation.y = -0.389
				goal_3.pose.orientation.z = 0.432
				goal_3.pose.orientation.w = 0.749

				#plan = plannerR.plan_to_pose(goal_3, list())

				raw_input("Press <Enter> to move the right arm to goal pose 3: ")
				if not plannerR.execute_plan(plannerR.plan_to_pose(goal_3, list())):
					raise Exception("Execution failed")
			except Exception as e:
				print e
			else:
				break
				
		while not rospy.is_shutdown():
			try:
				goal_2 = PoseStamped()
				goal_2.header.frame_id = "base"

				#x, y, and z position
				goal_2.pose.position.x = 0.448
				goal_2.pose.position.y = -0.047
				goal_2.pose.position.z = -0.245

				#Orientation as a quaternion
				goal_2.pose.orientation.x = 0
				goal_2.pose.orientation.y = 1
				goal_2.pose.orientation.z = 0
				goal_2.pose.orientation.w = 0

				#plan = plannerR.plan_to_pose(goal_2, list())

				raw_input("Press <Enter> to move the right arm to goal pose 2: ")
				if not plannerR.execute_plan(plannerR.plan_to_pose(goal_2, list())):
					raise Exception("Execution failed")
			except Exception as e:
				print e
			else:
				break

		while not rospy.is_shutdown():
			try:
				goal_1 = PoseStamped()
				goal_1.header.frame_id = "base"

				#x, y, and z position
				goal_1.pose.position.x = 0.448
				goal_1.pose.position.y = -0.047
				goal_1.pose.position.z = -0.245

				#Orientation as a quaternion
				goal_1.pose.orientation.x = 1
				goal_1.pose.orientation.y = 0
				goal_1.pose.orientation.z = 0
				goal_1.pose.orientation.w = 0

				# Might have to edit this . . . 

				#plan = plannerR.plan_to_pose(goal_1, list()) # put orien_const here

				raw_input("Press <Enter> to move the right arm to goal pose 1: ")
				if not plannerR.execute_plan(plannerR.plan_to_pose(goal_1, list())):
					raise Exception("Execution failed")
			except Exception as e:
				print e
			else:
				break




if __name__ == '__main__':
	rospy.init_node('moveit_node')
	main()
