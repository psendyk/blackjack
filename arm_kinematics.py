#!/usr/bin/env python

import sys
import rospy
import moveit_commander
from moveit_msgs.msg import OrientationConstraint, Constraints, CollisionObject
from geometry_msgs.msg import PoseStamped
import baxter_interface

#globals
class ArmPlanner(object):
	def __init__(self):
		"""
		Constructor.

		Inputs:
		group_name: the name of the move_group.
			For Baxter, this would be 'left_arm' or 'right_arm'
			For Sawyer, this would be 'right_arm'
		"""

		# If the node is shutdown, call this function    
		rospy.on_shutdown(self.shutdown)

		# Initialize moveit_commander
		moveit_commander.roscpp_initialize(sys.argv)

		# Initialize the robot
		self._robot = moveit_commander.RobotCommander()

		# Initialize the planning scene
		self._scene = moveit_commander.PlanningSceneInterface()

		# This publishes updates to the planning scene
		self._planning_scene_publisher = rospy.Publisher('/collision_object', CollisionObject, queue_size=10)

		# Instantiate a move group
		self._groupR = moveit_commander.MoveGroupCommander("right_arm")

		# Set the maximum time MoveIt will try to plan before giving up
		self._groupR.set_planning_time(5)


		self._groupR.allow_replanning(1)

		# Instantiate a move group
		self._groupL = moveit_commander.MoveGroupCommander("left_arm")

		# Set the maximum time MoveIt will try to plan before giving up
		self._groupL.set_planning_time(5)


		self._groupL.allow_replanning(1)

		self._gripperR = baxter_interface.gripper.Gripper("right")
		self._gripperL = baxter_interface.gripper.Gripper("left")

		# Sleep for a bit to ensure that all inititialization has finished
		rospy.sleep(0.5)

	def shutdown(self):
		"""
		Code to run on shutdown. This is good practice for safety

		Currently deletes the object's MoveGroup, so that further commands will do nothing
		"""
		self._groupR = None
		self._groupL = None
		rospy.loginfo("Stopping Path Planner")

	#combines all right arm 
	def pick_look(self, depthOffset):
		#pick up card
		self.right_over_deck()
		self.right_draw_card(depthOffset)
		self.right_card_look()

	def handoff_deal(self, target_position):
		self.right_handoff()
		self.left_pre_handoff()
		self.eft_handoff()

		self.right_over_deck()
		self.left_deal(target_position)
		self.left_reset()

	def plan_and_executeIK(self, targetPose, right):
		if right > 0:
			self._groupR.set_pose_target(targetPose)
			self._groupR.set_start_state_to_current_state()

			plan = self._groupR.plan()
			self._groupR.execute(plan)

		else:
			self._groupL.set_pose_target(targetPose)
			self._groupL.set_start_state_to_current_state()

			plan = self._groupL.plan()
			self._groupL.execute(plan)


		rospy.sleep(0.1)

	def plan_and_executeFK(self, targetJoints, right):
		joints = targetJoints
		if right > 0:
			jointDict = {}
			j = self._groupR.get_joints()
			print(joints)
			for index in range(1,8):
				jointDict[j[index]] = joints[index - 1]

			print("before exe")
			self._groupR.set_joint_value_target(jointDict)
			self._groupR.set_start_state_to_current_state()

			plan = self._groupR.plan()
			self._groupR.execute(plan)

		else:
			jointDict = {}
			j = self._groupR.get_joints()
			for index in range(1,8):
				jointDict[j[index]] = joints[index - 1]

			self._groupL.set_joint_value_target(jointDict)
			self._groupL.set_start_state_to_current_state()

			plan = self._groupL.plan()
			self._groupL.execute(plan)


		rospy.sleep(0.1)

	def setConstr(self, orien_const, right):
		constraints = Constraints()
		constraints.orientation_constraints = orien_const

		if right > 0:
			self._groupR.set_path_constraints(constraints)
		else:
			self._groupL.set_path_constraints(constraints)

	def formatJoints(self, joints):
		j = []
		j.extend(joints[11:13])
		j.extend(joints[9:11])
		j.extend(joints[13:-1])
		return j

	#======================RIGHT ARM==============================

	#get right arm out of the way
	def right_reset(self):
		return

	#move to slightly above deck, so we dont knock it over
	def right_over_deck(self):
		joints = [0.0, -0.06941263065181497, -1.1428156869746333, 1.9370342399023062, -0.01802427425765361, -0.9921020745648913, 0.6764855274574675, 1.035053536625683, -0.49777676566881673, 1.2532623037023831, 1.848830344598895, -0.68568941218478, 0.5629709491539469, -2.244980883070303, 1.440024464627432, -2.700573177072271, -12.565987119160338]
		joints = self.formatJoints(joints)


		self.setConstr([], 1)
		self.plan_and_executeFK(joints, 1)
		print("hover done")


	#move down and pick up card from the right_over_deck position
	def right_draw_card(self, depthOffset):
		currPose = self._groupR.get_current_pose()

		print("Pose retrieved")
		print(currPose)

		goal = currPose
		goal.pose.position.z = currPose.pose.position.z - depthOffset*0.01

		orien_const = OrientationConstraint()
		orien_const.link_name = "right_gripper";
		orien_const.header.frame_id = "base";
		orien_const.orientation.y = -1.0;
		orien_const.absolute_x_axis_tolerance = 0.1;
		orien_const.absolute_y_axis_tolerance = 0.1;
		orien_const.absolute_z_axis_tolerance = 0.1;
		orien_const.weight = 1.0;

		self.setConstr([orien_const], 1)
		self.plan_and_executeIK(goal, 1)
		self._gripperR.command_suction(1)


		self.right_over_deck()

	#move right arm to look at card
	def right_card_look(self):
		joints = [0.0, -0.06902913545484361, -1.1424321917776619, 1.936650744705335, -0.018791264651596317, -0.99171857936792, 0.6768690226544388, 1.035053536625683, -0.49854375606275947, 1.7153740160528639, 1.4530633013244583, 0.3643204371227858, -0.18522818013716372, 0.6649806715483269, 1.8752915131899184, -1.3970730025666405, -12.565987119160338]
		joints = self.formatJoints(joints)

		self.setConstr([], 1)
		self.plan_and_executeFK(joints, 1)

	#touch tips to handoff card - require extra function to determine what order to succ/unsucc in
	#right to left: succ left, then unsucc right; vice versa
	#for handoff orient grippers so they are parallel to the ground but facing each other
	def right_handoff(self):
		joints = [0.0, -0.06902913545484361, -1.8392429646746111, 1.904437148159741, 0.4494563708504262, 1.0473253829287663, 1.2221991927477034, 1.4005244593393829, -1.379048728308987, 1.5746312787643773, 1.8365584982958116, -0.48473792897179074, 0.6722670802907825, -0.7524175764577954, 1.0944952921562427, -1.403975916112125, -12.565987119160338]
		joints = joints[10:len(joints)-1]

		self.setConstr([], 1)
		self.plan_and_executeFK(joints, 1)

	def right_deal(self, target_position):
		joints = [TODO]

		self.setConstr([], 1)
		self.plan_and_executeFK(joints, 1)


	#======================LEFT ARM==============================
	#move left arm out of the way
	def left_reset(self):
		joints = [TODO]

		self.setConstr([], 0)
		self.plan_and_executeFK(joints, 0)

	#move into handoff position but slightly farther away so we dont break things
	def left_pre_handoff(self):
		joints = [TODO]

		self.setConstr([], 0)
		self.plan_and_executeFK(joints, 0)

	#move left arm to touch tips and actually handoff the card
	def left_handoff(self):
		joints = [TODO]

		self.setConstr([], 0)
		self.plan_and_executeFK(joints, 0)
		self._gripperL.command_suction(1)
		self._gripperR.stop()

	#should be able to handle placing down the flipped dealer's card as well as dealing cards to players
	def left_deal(self, target_position):
		return