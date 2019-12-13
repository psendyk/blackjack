#!/usr/bin/env python


import sys
import rospy
from arm_kinematics import *
from gesture_classifier import GestureClassifier
from card_classifier import CardClassifier
import cv2
import cv_bridge

from sensor_msgs.msg import Image

#dictionary of card names to values
cardValues = {'1':1, '2':2, '3':3, '4':4, '5':5, '6':6, '7':7, '8':8, '9':9,'10':10, 'jack':10, 'queen':10, 'king':10, 'ace':[1, 11]} 
tenCards = ['10', 'jack', 'queen', 'king']
#keeps track of total cards drawn
cards_drawn = 0
numBusted = 0
imageS = "/home/cc/ee106a/fa19/class/ee106a-aav/ros_workspace/project/src/dealplan/src/display_images/"
#HARDCODED DECK change

tempDeck = ['jack', 'queen', 'ace', '6', '9', 'queen', '8','king', '7', '10', '7', '3', 'jack', '5', '3', '3', '5', '4', 'ace',
'queen', '10', '5', '8', '9', 'king', 'jack','2', '2', '6','queen', '8','2','6','jack','king','10','2','10','ace','king','6',
'5','4','4','7','9','3','8','ace','9','4','7']


robot = None

class Player:
	#position, hand, offset, gameOver?
	def __init__(self, isDealer, position):
		self.hand = [] #keeps track of the player's hand
		self.isDealer = isDealer #if the player is a dealer
		self.gameOver = False #if the player has lost
		self.isBusted = False #if the player has busted
		self.blackjack = False #if the player has blackjack initially
		self.offset = 0 #measures how far over the cards need to be dealt each time
		self.position = position #position of where the player's are physically

def send_image(path, sleepTime = 0):
    """
    Send the image located at the specified path to the head
    display on Baxter.
    @param path: path to the image file to load and send
    """
    img = cv2.imread(path)
    msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
    pub = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=1)
    pub.publish(msg)
    # Sleep to allow for image to be published.
    if sleepTime > 0:
    	rospy.sleep(sleepTime) #TODO not sure if we need this
    print("BLACKJACK successfully sent image")

#this will deal a card to player. it appends the card to the player's hand and then checks if they have busted
#this involves picking up a card, reading it, and placing it by the player
def deal(player, orientation = 'up'):
	global cards_drawn, tempDeck

	#DO KINEMATICS HERE <-------------
	target_position = (player.position, player.offset)
	robot.pick_look(cards_drawn)
	#get cardValue from computer vision
	#cardValue = card_cls.recognize()
	#HARDCODED CHANGE LATER
	cardValue = tempDeck.pop(0)
	#if we are dealing facedown, right hand deals
	if (orientation == 'down'):
		robot.right_deal()
	else:
		robot.handoff_deal(target_position)
	player.hand.append(cardValue)
	print("BLACKJACK Successfully appended to player " + str(player.position) + "'s hand")
	checkBust(player)
	print("BLACKJACK Sucessfully checked bust of player " + str(player.position))
	player.offset += 1
	cards_drawn += 1

#flips over the card on the table that is face down
def flip():
	robot.pick_table()

def dealHand(player): #deals 
	if player.isDealer:
		deal(player, 'down')
	else:
		deal(player)
	deal(player)

def total(player): #add up all cards in player's hand and return value
	total = 0
	aceCount = 0    
	for card in player.hand:
		if card != 'ace':
			total += cardValues[card]
		else:
			aceCount += 1
	#run through twice
	totalLow = total
	totalHigh = total
	if aceCount >= 1:
		totalHigh += 11 + (aceCount - 1)
		totalLow += aceCount
	if totalHigh > 21:
		return totalLow
	else:
		return totalHigh

def checkBust(player):
	global numBusted
	if total(player) > 21:
		player.isBusted = True
		player.gameOver = True
		if (player.position == -1):
			send_image(imageS + "bust-1.png", 2)
		else: 
			numBusted += 1
			print("Player " + str(player.position) + " busted", 1)
			send_image(imageS + "bust" + str(player.position) +".png") 

def blackJack(player):
	if 'ace' in player.hand:
		for card in tenCards:
			if card in player.hand:
				if player.position == -1:
					send_image(imageS + "bjd.png", 2) 
				else:
					send_image(imageS + "bj" + str(player.position) +".png") 
				return True
	return False

def game():
	send_image(imageS + "welcome.png", 3)
	send_image(imageS + "players.png")
	numPlayers = int(raw_input("Enter how many players are playing: "))
	dealer = Player(True, -1)
	players = []
	print("BLACKJACK Creating players")
	for i in range(numPlayers):
		players.append(Player(False, i)) #add a new player to players array
	print("BLACKJACK Finished creating players")

	#setup board
	send_image(imageS + "dealingdealer.png") 
	dealHand(dealer)
	print("BLACKJACK Finished dealing dealer hand")
	
	print("Dealer " + str(dealer.position) + " has a hand of " + str(dealer.hand))
	for player in players:
		send_image(imageS + "dealing" + str(player.position) +".png") 
		dealHand(player) 
	print("BLACKJACK Finished dealing hands for all players")	
	for player in players:
		print("Player " + str(player.position) + " has a hand of " + str(player.hand))

	if blackJack(dealer): #CORNER CASE: if dealer has blackjack immediately
		print("Dealer has blackjack")
		for player in players:
			if blackJack(player):
				win(player)
			else:
				lose(player)
	else:
		for player in players:
			print("Player " + str(player.position) + " has a hand of " + str(player.hand) + " for a total of " + str(total(player)))
			if blackJack(player):
				player.blackJack = True
				player.gameOver = True
			while not player.gameOver: #while the player hasnt busted
				send_image(imageS + "turn" + str(player.position) +".png", 3) 
				#choice = raw_input("Do you want to 'hit' or 'stay'").lower() #convert to computer vision
				choice = gesture_cls.recognize()
				print(choice)
				if choice == "hit":
					send_image(imageS + "hit" +".png") 
					deal(player)
					print("Dealt " + str(player.hand[-1]))
					print("Player " + str(player.position) + " has a hand of " + str(player.hand) + " for a total of " + str(total(player)))
				elif choice == "stay":
					send_image(imageS + "stand" +".png") 
					player.gameOver = True

		print("BLACKJACK Finished playing for all players")
		if numBusted != numPlayers:
			flip()
			print("BLACKJACK Flipped dealer card")
			while total(dealer) < 17 and not dealer.isBusted:
				deal(dealer)
				print("Dealt " + str(dealer.hand[-1]))
			print("Dealer " + str(dealer.position) + " has a hand of " + str(dealer.hand) + " for a total of " + str(total(dealer)))		

		print("BLACKJACK Dealer finished playing")
		#now dealer has been dealed we check win conditions on each player
		if dealer.isBusted:
			#everyone that has not busted wins
			for player in players:
				if not player.isBusted:
					#player wins
					win(player)
				else:
					#player loses
					lose(player)
		else:
			#everyone that has not busted and has a value greater than the dealer's wins or if they have blackjack
			for player in players:
				if ((not player.isBusted) and (total(player) > total(dealer))) or player.blackjack:
					win(player)
					#player wins
				else:
					lose(player)
					#player loses
	print("BLACKJACK Game over")
	choice = raw_input("Do you want to play again?'").lower()
	if choice == 'yes':
		actualGame()	

def win(player):
	#introduce delay
	send_image(imageS + "win" + str(player.position) +".png", 3) 	
def lose(player):
	send_image(imageS + "lose" + str(player.position) +".png", 3) 
def actualGame():
	global robot, cards_drawn, gesture_cls, card_cls, numBusted
	rospy.init_node('moveit_node')
	robot = ArmPlanner()
	numBusted = 0
	gesture_cls = GestureClassifier()
	card_cls = CardClassifier()
	while not rospy.is_shutdown():
		try:
			game()
			robot.shutdown()
		except Exception as e:
			print e
		else:
			break
	
if __name__ == "__main__":
	actualGame()
