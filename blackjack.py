#!/usr/bin/env python


import sys
import rospy
from arm_kinematics import *

#dictionary of card names to values
cardValues = {'one':1, 'two':2, 'three':3, 'four':4, 'five':5, 'six':6, 'seven':7, 'eight':8, 'nine':9,'ten':10, 'jack':10, 'queen':10, 'king':10, 'ace':[1, 11]} 
tenCards = ['ten', 'jack', 'queen', 'king']
#keeps track of total cards drawn
cards_drawn = 0

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

#this will deal a card to player. it appends the card to the player's hand and then checks if they have busted
#this involves picking up a card, reading it, and placing it by the player
def deal(player, orientation = 'up'):
	global cards_drawn

	#DO KINEMATICS HERE <-------------
	target_position = (player.position, player.offset)
	robot.pick_look(cards_drawn)
	#get cardValue from computer vision
	#cardValue = COMPUTER VISION STUFF0
	cardValue = 'one'
	#if we are dealing facedown, right hand deals
	if (orientation == 'down'):
		robot.right_deal()
	else:
		robot.handoff_deal(target_position)
	print("deal")
	player.hand.append(cardValue)
	print("appended to hand")
	checkBust(player)
	print("checked bust")
	player.offset += 1
	cards_drawn += 1
	print('finish deal')

#flips over the card on the table that is face down
def flip():
	robot.pick_table()

def dealHand(player): #deals 
	if player.isDealer:
		deal(player, 'down')
	else:
		deal(player)
	print("dealt first card")
	deal(player)
	print("dealt second card")        

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
	print("total works")

def checkBust(player):
	if total(player) > 21:
		player.isBusted = True
		player.gameOver = True
		print("game over")
	print("Check bust worked")

def blackJack(player):
	if 'ace' in player.hand:
		for card in tenCards:
			if card in player.hand:
				return True
	return False

def game():
	numPlayers = int(raw_input("Enter how many players are playing: "))
	dealer = Player(True, 0)
	players = []
	for i in range(numPlayers):
		players.append(Player(False, i)) #add a new player to players array
	print("Finished creating players")

	#setup board
	dealHand(dealer)
	print("Dealt dealer hand")
	for player in players:
		dealHand(player) 
	print("Finished dealing hands")

	#if blackJack(dealer): #CORNER CASE: if dealer has blackjack immediately
		#check if players have blackjack
	
	for player in players:
		if blackJack(player):
			player.blackJack = True
			player.gameOver = True
		while not player.gameOver: #while the player hasnt either busted or quit
			choice = raw_input("Do you want to 'hit' or 'stand'").lower() #convert to computer vision
			if choice == "hit":
				deal(player)
			elif choice == "stand":
				player.gameOver = True
	print("Finished playing")

	flip()
	print("Flipped dealer card")
	while total(dealer) < 17 and not dealer.isBusted:
		deal(dealer)

	print("Dealer finished playing")
	#now dealer has been dealed we check win conditions on each player
	if dealer.isBusted:
		#everyone that has not busted wins
		for player in players:
			if not player.isBusted:
				#player wins
				print("Congratulations player " + str(player.position))
			else:
				#player loses
				print("You lost " + str(player.position))
	else:
		#everyone that has not busted and has a value greater than the dealer's wins 
		for player in players:
			if (not player.isBusted and total(player) > total(dealer)) or player.blackjack:
				print("Player won!")
				#player wins
			else:
				print("Player loses!")
				#player loses
	print("Game over")

def actualGame():
	global robot
	rospy.init_node('moveit_node')
	robot = ArmPlanner()
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