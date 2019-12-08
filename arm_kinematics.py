#!/usr/bin/env python

import sys
import rospy
import moveit_commander
from moveit_msgs.msg import OrientationConstraint, Constraints
from geometry_msgs.msg import PoseStamped
import baxter_interface

#globals

#======================RIGHT ARM==============================
#get right arm out of the way
def right_away(right_arm):
	return

#move to slightly above deck, so we dont knock it over
def right_over_deck(right_arm):
	return

#move down and pick up card from the right_over_deck position
def right_draw_card(right_arm):
	return

#move right arm to look at card
def right_card_look(right_arm):
	return

#touch tips to handoff card - require extra function to determine what order to succ/unsucc in
#right to left: succ left, then unsucc right; vice versa
#for handoff orient grippers so they are parallel to the ground but facing each other
def right_handoff(right_arm):
	return

#for flipping dealer's face down card
#start with hover over it
def right_over_fd(right_arm):
	return

#go down and pick it up
def right_pickup_fd(right_arm):
	return

#then touch tips to handoff, already exists - let left hand put the card down

#======================LEFT ARM==============================
#move left arm out of the way
def left_away(left_arm):
	return

#move into handoff position but slightly farther away so we dont break things
def left_pre_handoff(left_arm):
	return

#move left arm to touch tips and actually handoff the card
def left_handoff(left_arm):
	return

#deal card, can we make 1 function to do it all?
#should be able to handle placing down the flipped dealer's card as well as dealing cards to players
def left_deal(left_arm, target_position):
	return