#!/usr/bin/env python

import sys
import rospy
import moveit_commander
from moveit_msgs.msg import OrientationConstraint, Constraints
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

	    # Set the bounds of the workspace
	    self._groupR.set_workspace([-2, -2, -2, 2, 2, 2])

	    self._groupR.allow_replanning(1)

	    # Instantiate a move group
	    self._groupL = moveit_commander.MoveGroupCommander("left_arm")

	    # Set the maximum time MoveIt will try to plan before giving up
	    self._groupL.set_planning_time(5)

	    # Set the bounds of the workspace
	    self._groupL.set_workspace([-2, -2, -2, 2, 2, 2])

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
		right_over_deck()
		right_draw_card(depthOffset)
		right_card_look()

	def handoff_deal(self, target_position):
		right_handoff()
		left_pre_handoff()
		left_handoff()
		right_reset()
		left_deal(target_position)
		left_reset()

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
			for index, name in enumerate(self._groupR.get_joints()):
				jointDict[name] = joints[index]

			self._groupR.set_joint_value_target(jointDict)
			self._groupR.set_start_state_to_current_state()

			plan = self._groupR.plan()
			self._groupR.execute(plan)

		else:
			jointDict = {}
			for index, name in enumerate(self._groupR.get_joints()):
				jointDict[name] = joints[index]

			self._groupL.set_joint_value_target(jointDict)
			self._groupL.set_start_state_to_current_state()

			plan = self._groupL.plan()
			self._groupL.execute(plan)


		rospy.sleep(0.1)


	#======================RIGHT ARM==============================

	#get right arm out of the way
	def right_reset(self):
		return

	#move to slightly above deck, so we dont knock it over
	def right_over_deck(self):
		joints = [TODO]

		self.plan_and_executeFK(joints, 1)


	#move down and pick up card from the right_over_deck position
	def right_draw_card(self, depthOffset):
		currPose = self._groupR.get_current_pose()

		goal = currPose
		goal.pose.position.z = currPose.pose.position.z - depthOffset*CONST

		self.plan_and_executeIK(goal, 1)

		self.right_over_deck()

	#move right arm to look at card
	def right_card_look(self):
		joints = [TODO]

		self.plan_and_executeFK(joints, 1)

	#touch tips to handoff card - require extra function to determine what order to succ/unsucc in
	#right to left: succ left, then unsucc right; vice versa
	#for handoff orient grippers so they are parallel to the ground but facing each other
	def right_handoff(self):
		return

	def right_deal(self, target_position):
		
	#======================LEFT ARM==============================
	#move left arm out of the way
	def left_reset(self):
		return

	#move into handoff position but slightly farther away so we dont break things
	def left_pre_handoff(self):
		return

	#move left arm to touch tips and actually handoff the card
	def left_handoff(self):
		return

	#deal card, can we make 1 function to do it all?
	#should be able to handle placing down the flipped dealer's card as well as dealing cards to players
	def left_deal(self, target_position):
		return