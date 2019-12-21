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

		self._gripperR.calibrate()
		self._gripperL.calibrate()

		self._group = moveit_commander.MoveGroupCommander("both_arms")

		# self._groupR.set_goal_position_tolerance(0.001)
		# self._groupR.set_goal_orientation_tolerance(0.01)
		# self._groupR.set_goal_joint_tolerance(0.001)
		# self._groupL.set_goal_position_tolerance(0.001)

		# Sleep for a bit to ensure that all inititialization has finished
		rospy.sleep(0.5)

	def shutdown(self):
		"""
		Code to run on shutdown. This is good practice for safety

		Currently deletes the object's MoveGroup, so that further commands will do nothing
		"""
		self._groupR = None
		self._groupL = None
		self._group = None
		rospy.loginfo("Stopping Path Planner")

	#combines all right arm 
	def pick_look(self, depthOffset):
		#pick up card
		self.right_over_deck()
		self.right_draw_card(depthOffset)
		self.right_card_look()

	def handoff_deal(self, target_position):
		# self.right_handoff()
		# self.left_pre_handoff()
		# self.left_handoff()
		self.handoff()

		# self.right_over_deck()
		self.left_deal(target_position)
		# self.left_reset()

	def pick_table(self):
		self.right_pick()
		self.handoff_deal((-1,0))

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
			# print(joints)
			for index in range(1,8):
				jointDict[j[index]] = joints[index - 1]
			self._groupR.set_joint_value_target(jointDict)
			self._groupR.set_start_state_to_current_state()

			plan = self._groupR.plan()
			reach = self._groupR.execute(plan)
			while not reach:
				plan = self._groupR.plan()
				reach = self._groupR.execute(plan)

		else:
			jointDict = {}
			j = self._groupL.get_joints()
			# print(j)
			for index in range(1,8):
				jointDict[j[index]] = joints[index - 1]

			self._groupL.set_joint_value_target(jointDict)
			self._groupL.set_start_state_to_current_state()

			plan = self._groupL.plan()
			reach = self._groupL.execute(plan)
			while not reach:
				plan = self._groupL.plan()
				reach = self._groupL.execute(plan)


		rospy.sleep(0.1)

	def setConstr(self, orien_const, right):
		constraints = Constraints()
		constraints.orientation_constraints = orien_const

		if right > 0:
			self._groupR.set_path_constraints(constraints)
		else:
			self._groupL.set_path_constraints(constraints)

	def formatJoints(self, joints, right=1):
		if right > 0:
			j = []
			j.extend(joints[11:13])
			j.extend(joints[9:11])
			j.extend(joints[13:-1])
			return j
		else:
			j = []
			j.extend(joints[4:6])
			j.extend(joints[2:4])
			j.extend(joints[6:9])
			return j

	#======================RIGHT ARM==============================

	def right_pick(self):
		jointsL = [0.0, -0.08091748656095557, -1.1604564660353156, 1.2294856014901592, 1.410495334460638, 1.0465583925348236, -2.4677915925106593, -1.5650438988400934, -0.9809807138527221, 2.611602291374917, 0.934194299822217, 0.9901845985800346, 0.7777282594579048, -1.7452866414166295, 1.690446828249726, -1.870689570826262, -12.565987119160338]
		jointsL = self.formatJoints(jointsL, 0)
		jointsR = [0.0, -0.08053399136398422, -0.41647578391088985, 0.5407282277296084, 1.4243011615516066, 0.9345777950191884, -1.8008934449774758, -0.5499321124569209, -0.9828981898375788, 1.3081021168692866, 1.7199759584165202, 0.088970885697354, 0.7320923310183137, -2.31784497049486, 1.4254516471425207, -2.014883764887491, -12.565987119160338]
		jointsR = self.formatJoints(jointsR)
		# concat to set joints for simutaneous motion
		# TODO
		jointDict = {}
		j = self._group.get_joints()
		# print(joints)
		# for index, jo in enumerate(j):
		# 	print(str(index) + ", " + str(jo))
		for index in range(1,8):
			jointDict[j[index]] = jointsL[index - 1]
		for index in range(12,19):
			jointDict[j[index]] = jointsR[index - 12]

		self._group.set_joint_value_target(jointDict)
		self._group.set_start_state_to_current_state()

		plan = self._group.plan()
		self._group.execute(plan)


		#joints = [0.0, -0.08015049616701286, -1.1792477306869118, 1.1558545236716593, 1.1520195717019457, 1.0595972292318496, -2.661840162178164, -1.543951663006669, -1.1136700520048104, 1.4891118498397653, 1.488728354642794, 0.1449611844551716, 1.0400389741863105, -2.628092584844685, 1.4392574742334894, -2.290616811509894, -12.565987119160338]
		joints = [0.0, -0.1349903093339164, -2.2576362245703576, 1.0327525654438547, 0.03413107253045045, 0.7405292253516835, -0.8195292359277823, -1.5715633171886063, -0.22127672865247094, 1.323825419945112, 1.5090536000822758, 0.2067039111675595, 0.8555777844430895, -2.41985469288924, 1.4058933920969816, -2.133383780751639, -12.565987119160338]
		joints = self.formatJoints(joints)


		self.setConstr([], 1)
		self.plan_and_executeFK(joints, 1)

		self._gripperR.set_vacuum_threshold(30)
		# make sure card attached
		attached = self._gripperR.command_suction(1, 2)
		print(attached)

		while not attached:
			self.setConstr([], 1)
			self.plan_and_executeFK(joints, 1)
			attached = self._gripperR.command_suction(1, 2)

		print("Right_Pick finished")

	#move to slightly above deck, so we dont knock it over
	def right_over_deck(self):
		joints = [0.0, -0.08015049616701286, -0.41800976469877527, 0.5407282277296084, 1.4239176663546353, 0.934194299822217, -1.8024274257653612, -0.5483981316690354, -0.9828981898375788, 1.193437052974852, 1.9055876337506552, -0.8505923468824619, 0.8379370053824072, -2.455519746207576, 1.584218658688661, -2.6948207491177008, -12.565987119160338]
		joints = self.formatJoints(joints)


		self.setConstr([], 1)
		self.plan_and_executeFK(joints, 1)
		print("Right_Over_Deck finished")


	#move down and pick up card from the right_over_deck position
	def right_draw_card(self, depthOffset):
		#fix so abort during execution retries until card attached
		currPose = self._groupR.get_current_pose()

		# print("Pose retrieved")
		# print(currPose)

		goal = currPose
		goal.pose.position.z = currPose.pose.position.z - 0.048 - depthOffset*0.001

		orien_const = OrientationConstraint()
		orien_const.link_name = "right_gripper";
		orien_const.header.frame_id = "base";
		orien_const.orientation.x = 1.0;
		orien_const.absolute_x_axis_tolerance = 0.1;
		orien_const.absolute_y_axis_tolerance = 0.1;
		orien_const.absolute_z_axis_tolerance = 0.1;
		orien_const.weight = 1.0;

		self.setConstr([orien_const], 1)
		self.plan_and_executeIK(goal, 1)

		self._gripperR.set_vacuum_threshold(10)
		# make sure card attached
		attached = self._gripperR.command_suction(1, 0.8)
		print(attached)

		while not attached:
			self.setConstr([orien_const], 1)
			self.plan_and_executeIK(goal, 1)
			attached = self._gripperR.command_suction(1, 0.8)

		# rospy.sleep(0.5)

		self.right_over_deck()

		print("Right_Draw_Card finished")

	#move right arm to look at card
	def right_card_look(self):
		# joints = [0.0, -0.06902913545484361, -1.1424321917776619, 1.936650744705335, -0.018791264651596317, -0.99171857936792, 0.6768690226544388, 1.035053536625683, -0.49854375606275947, 1.7153740160528639, 1.4530633013244583, 0.3643204371227858, -0.18522818013716372, 0.6649806715483269, 1.8752915131899184, -1.3970730025666405, -12.565987119160338]
		# joints = self.formatJoints(joints)


		# self.setConstr([], 1)
		# self.plan_and_executeFK(joints, 1)

		jointsL = [0.0, -0.08091748656095557, -1.1604564660353156, 1.2294856014901592, 1.410495334460638, 1.0465583925348236, -2.4677915925106593, -1.5650438988400934, -0.9809807138527221, 2.611602291374917, 0.934194299822217, 0.9901845985800346, 0.7777282594579048, -1.7452866414166295, 1.690446828249726, -1.870689570826262, -12.565987119160338]
		jointsL = self.formatJoints(jointsL, 0)
		jointsR = [0.0, -0.06902913545484361, -1.1424321917776619, 1.936650744705335, -0.018791264651596317, -0.99171857936792, 0.6768690226544388, 1.035053536625683, -0.49854375606275947, 1.7153740160528639, 1.4530633013244583, 0.3643204371227858, -0.18522818013716372, 0.6649806715483269, 1.8752915131899184, -1.3970730025666405, -12.565987119160338]
		jointsR = self.formatJoints(jointsR, 1)
		# concat to set joints for simutaneous motion
		# TODO
		jointDict = {}
		j = self._group.get_joints()
		# print(joints)
		# for index, jo in enumerate(j):
		# 	print(str(index) + ", " + str(jo))
		for index in range(1,8):
			jointDict[j[index]] = jointsL[index - 1]
		for index in range(12,19):
			jointDict[j[index]] = jointsR[index - 12]

		self._group.set_joint_value_target(jointDict)
		self._group.set_start_state_to_current_state()

		plan = self._group.plan()
		self._group.execute(plan)
		print("Right_Card_Look finished")

		
	def right_handoff(self):
		joints = [0.0, -0.08015049616701286, -1.153553552489831, 1.1961215193536514, 1.1919030721869666, 1.0465583925348236, -2.6775634652539897, -1.545102148597583, -0.9046651696554228, 2.6123692817688595, 0.9334273094282742, 0.9909515889739773, 0.7773447642609335, -1.744903146219658, 1.6900633330527546, -1.8710730660232333, -12.565987119160338]
		joints = self.formatJoints(joints)

		self.setConstr([], 1)
		self.plan_and_executeFK(joints, 1)

	#only to dealer
	def right_deal(self):
		joints = [0.0, -0.08053399136398422, -0.41647578391088985, 0.5407282277296084, 1.4243011615516066, 0.9345777950191884, -1.8008934449774758, -0.5499321124569209, -0.9828981898375788, 1.3081021168692866, 1.7199759584165202, 0.088970885697354, 0.7320923310183137, -2.31784497049486, 1.4254516471425207, -2.014883764887491, -12.565987119160338]
		joints = self.formatJoints(joints)

		self.setConstr([], 1)
		self.plan_and_executeFK(joints, 1)

		self._gripperR.set_blow_off(0.1)
		self._gripperR.stop()
		print("Dealer Dealt")


	#======================LEFT ARM==============================
	#move left arm out of the way
	def left_reset(self):
		joints = [TODO]

		self.setConstr([], 0)
		self.plan_and_executeFK(joints, 0)

	#move into handoff position but slightly farther away so we dont break things
	def left_pre_handoff(self):
		joints = [0.0, -0.08091748656095557, -1.1604564660353156, 1.2294856014901592, 1.410495334460638, 1.0465583925348236, -2.4677915925106593, -1.5650438988400934, -0.9809807138527221, 2.611602291374917, 0.934194299822217, 0.9901845985800346, 0.7777282594579048, -1.7452866414166295, 1.690446828249726, -1.870689570826262, -12.565987119160338]
		joints = self.formatJoints(joints, 0)

		self.setConstr([], 0)
		self.plan_and_executeFK(joints, 0)
		print("Left_Pre_Handoff finished")

	#move left arm to touch tips and actually handoff the card
	def left_handoff(self):
		joints = [0.0, -0.08015049616701286, -1.1788642354899406, 1.1570050092625732, 1.1520195717019457, 1.0469418877317949, -2.661840162178164, -1.5435681678096975, -1.1136700520048104, 2.8083353274212213, 0.9338108046252456, 1.0726360659288756, 0.7884661249731026, -1.9017526817809416, 1.7763497523713092, -1.8668546188565485, -12.565987119160338]
		#joints = [0.0, -0.1349903093339164, -1.169660350762628, 1.2095438512476488, 1.151252581308003, 1.0473253829287663, -2.6614566669811928, -1.4894953450367368, -1.114437042398753, 2.6127527769658307, 0.9318933286403889, 0.9909515889739773, 0.776961269063962, -1.7452866414166295, 1.690446828249726, -1.8714565612202048, -12.565987119160338]
		joints = self.formatJoints(joints, 0)


		self.setConstr([], 0)
		self.plan_and_executeFK(joints, 0)
		rospy.sleep(0.3)
		self._gripperL.set_vacuum_threshold(20)
		self._gripperR.set_blow_off(0.01)
		self._gripperR.stop()

		# make sure card attached
		attached = self._gripperL.command_suction(0, 1)
		print(attached)

		while not attached:
			self.setConstr([], 0)
			self.plan_and_executeFK(joints, 0)
			attached = self._gripperL.command_suction(0, 1)

		print("Left_Handoff finished")


	def left_deal(self, target_position):
		jointsR = [0.0, -0.08015049616701286, -0.41800976469877527, 0.5407282277296084, 1.4239176663546353, 0.934194299822217, -1.8024274257653612, -0.5483981316690354, -0.9828981898375788, 1.193437052974852, 1.9055876337506552, -0.8505923468824619, 0.8379370053824072, -2.455519746207576, 1.584218658688661, -2.6948207491177008, -12.565987119160338]
		jointsR = self.formatJoints(jointsR)

		constraints = Constraints()
		constraints.orientation_constraints = []
		self._group.set_path_constraints(constraints)

		playerNum = target_position[0]
		playerOffset = target_position[1]
		k1 = 0.05 #fill this in later constant for playerNum
		k2 = 0.006 #constant for playerOffset
		#keep a static pose that is the default position and then we can edit that position everytime
		#first case default pos

		if (playerNum == -1):
			jointsL = [0.0, -0.13460681413694506, -1.4365730078546899, 1.6701215828102443, 0.08705340971249723, 1.0469418877317949, -0.7125340759727747, -1.4595827196729712, -1.212995308020391, 1.193437052974852, 1.9055876337506552, -0.8505923468824619, 0.8383205005793786, -2.45667023179849, 1.5849856490826038, -2.694053758723758, -12.565987119160338]
			jointsL = self.formatJoints(jointsL,0)
			# self.setConstr([], 0)
			# self.plan_and_executeFK(jointsL, 0)

			jointDict = {}
			j = self._group.get_joints()
			# print(joints)
			# for index, jo in enumerate(j):
			# 	print(str(index) + ", " + str(jo))
			for index in range(1,8):
				jointDict[j[index]] = jointsL[index - 1]
			for index in range(12,19):
				jointDict[j[index]] = jointsR[index - 12]

			self._group.set_joint_value_target(jointDict)
			self._group.set_start_state_to_current_state()

			plan = self._group.plan()
			self._group.execute(plan)
	
			print("Left dealing to dealer")
		else:
			print("Left dealing to player " + str(playerNum))
			if playerNum == 0:
				jointsL = [0.0, -0.1292378813793461, -2.3201459416766883, 0.4490728756534549, -1.0653496571864198, 0.6929758209272356, -0.5817622138055432, -1.2014904521112504, -0.5844466801843426, 1.193437052974852, 1.905204138553684, -0.8505923468824619, 0.8383205005793786, -2.455519746207576, 1.5846021538856323, -2.694437253920729, -12.565987119160338]
			elif playerNum == 1:
				jointsL = [0.0, -0.1349903093339164, -2.3082575905705762, 1.1470341341413182, -0.4670971499111085, 1.0469418877317949, -0.38234471138043935, -1.4124128104454947, -0.12616991980357528, 1.1949710337627373, 1.905204138553684, -0.8502088516854905, 0.83870399577635, -2.456286736601519, 1.5849856490826038, -2.695204244314672, -12.565987119160338]
			else:
				jointsL = [0.0, -0.13460681413694506, -2.338553711131313, 1.076471017898589, -0.18484468494019235, 1.010126348822545, -0.3869466537440956, -1.5056021433095337, 0.06596117387907278, 1.193437052974852, 1.9059711289476267, -0.8505923468824619, 0.8375535101854359, -2.455519746207576, 1.5846021538856323, -2.694437253920729, -12.565987119160338]

			jointsL = self.formatJoints(jointsL,0)
			# TODO: FIX THIS
			# self.setConstr([], 0)
			# self.plan_and_executeFK(jointsL, 0)

			jointDict = {}
			j = self._group.get_joints()
			# print(joints)
			# for index, jo in enumerate(j):
			# 	print(str(index) + ", " + str(jo))
			for index in range(1,8):
				jointDict[j[index]] = jointsL[index - 1]
			for index in range(12,19):
				jointDict[j[index]] = jointsR[index - 12]

			self._group.set_joint_value_target(jointDict)
			self._group.set_start_state_to_current_state()

			plan = self._group.plan()
			self._group.execute(plan)
			#else
			# goal = self._groupL.get_current_pose()
			# # print(self._group.right_gripper.get_current_pose())
			# #set this new pose to be equal to the default position
			# goal.pose.position.y = goal.pose.position.y + 0.001 - k1*playerNum + k2*playerOffset 
			# #not sure if y axis 
			# self.setConstr([], 0)
			# self.plan_and_executeIK(goal, 0)

		self._gripperL.stop()
		return

	#======================BOTH ARMS==============================

	def handoff(self):
		jointsL = [0.0, -0.08091748656095557, -1.1604564660353156, 1.2294856014901592, 1.410495334460638, 1.0465583925348236, -2.4677915925106593, -1.5650438988400934, -0.9809807138527221, 2.611602291374917, 0.934194299822217, 0.9901845985800346, 0.7777282594579048, -1.7452866414166295, 1.690446828249726, -1.870689570826262, -12.565987119160338]
		jointsL = self.formatJoints(jointsL, 0)
		jointsR = [0.0, -0.08015049616701286, -1.153553552489831, 1.1961215193536514, 1.1919030721869666, 1.0465583925348236, -2.6775634652539897, -1.545102148597583, -0.9046651696554228, 2.6123692817688595, 0.9334273094282742, 0.9909515889739773, 0.7773447642609335, -1.744903146219658, 1.6900633330527546, -1.8710730660232333, -12.565987119160338]
		jointsR = self.formatJoints(jointsR)
		# concat to set joints for simutaneous motion
		# TODO
		jointDict = {}
		j = self._group.get_joints()
		# print(joints)
		# for index, jo in enumerate(j):
		# 	print(str(index) + ", " + str(jo))
		for index in range(1,8):
			jointDict[j[index]] = jointsL[index - 1]
		for index in range(12,19):
			jointDict[j[index]] = jointsR[index - 12]

		self._group.set_joint_value_target(jointDict)
		self._group.set_start_state_to_current_state()


		plan = self._group.plan()
		reach = self._group.execute(plan)
		while not reach:
			plan = self._group.plan()
			reach = self._group.execute(plan)

		self.left_handoff()
