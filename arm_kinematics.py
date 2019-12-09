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
def right_away():
	return

#move to slightly above deck, so we dont knock it over
def right_over_deck():
	return

#move down and pick up card from the right_over_deck position
def right_draw_card(depthOffset):
	return

#move right arm to look at card
def right_card_look():
	return

#touch tips to handoff card - require extra function to determine what order to succ/unsucc in
#right to left: succ left, then unsucc right; vice versa
#for handoff orient grippers so they are parallel to the ground but facing each other
def right_handoff():
	return

def right_deal(target_position):
	
#======================LEFT ARM==============================
#move left arm out of the way
def left_away():
	return

#move into handoff position but slightly farther away so we dont break things
def left_pre_handoff():
	return

#move left arm to touch tips and actually handoff the card
def left_handoff():
	return

#deal card, can we make 1 function to do it all?
#should be able to handle placing down the flipped dealer's card as well as dealing cards to players
def left_deal(target_position):
	return