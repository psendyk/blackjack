#!/usr/bin/env python


import sys
import random
#dictionary of card names to values
cardValues = {'one':1, 'two':2, 'three':3, 'four':4, 'five':5, 'six':6, 'seven':7, 'eight':8, 'nine':9,'ten':10, 'jack':10, 'queen':10, 'king':10, 'ace':[1, 11]} 
tenCards = ['ten', 'jack', 'queen', 'king']
#keeps track of total cards drawn
cards_drawn = 0
tempDeck = ['ace', 'ten', 'queen', 'ace', 'jack', 'ace', 'eight', 'nine', 'two', 'ace', 'king', 'eight']


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
	global cards_drawn, tempDeck

	#DO KINEMATICS HERE <-------------
	target_position = (player.position, player.offset)
	print("ROBOT")
	#get cardValue from computer vision
	#cardValue = COMPUTER VISION STUFF
	cardValue = tempDeck.pop(0)
	#if we are dealing facedown, right hand deals
	if (orientation == 'down'):
		print("ROBOT")
	else:
		print("ROBOT")
	player.hand.append(cardValue)
	print("BLACKJACK appended to hand")
	checkBust(player)
	print("BLACKJACK checked bust")
	player.offset += 1
	cards_drawn += 1

#flips over the card on the table that is face down
def flip():
	print("ROBOT")

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
	if total(player) > 21:
		player.isBusted = True
		player.gameOver = True
		print("Player " + str(player.position) + " busted")

def blackJack(player):
	if 'ace' in player.hand:
		for card in tenCards:
			if card in player.hand:
				return True
	return False

def game():
	numPlayers = int(input("Enter how many players are playing: "))
	dealer = Player(True, -1)
	players = []
	print("BLACKJACK Creating players")
	for i in range(numPlayers):
		players.append(Player(False, i)) #add a new player to players array
	print("BLACKJACK Finished creating players")

	#setup board
	dealHand(dealer)
	print("BLACKJACK Finished dealing dealer hand")
	print("Dealer " + str(dealer.position) + " has a hand of " + str(dealer.hand))	
	for player in players:
		dealHand(player) 
		print("Player " + str(player.position) + " has a hand of " + str(player.hand))
	print("BLACKJACK Finished dealing hands for all players")

	if blackJack(dealer): #CORNER CASE: if dealer has blackjack immediately
		print("Dealer has blackjack")
		for player in players:
			if blackJack(player):
				print("Congratulations player " + str(player.position))
			else:
				print("You lost player " + str(player.position))
		#check if players have blackjack
	else:
		for player in players:
			if blackJack(player):
				player.blackJack = True
				player.gameOver = True
			while not player.gameOver: #while the player hasnt busted
				choice = input("Do you want to 'hit' or 'stand' player " + str(player.position)).lower() #convert to computer vision
				#choice = cls.recognize()
				print(choice)
				if choice == "hit":
					deal(player)
					print("Dealt " + str(player.hand[-1]))
					print("Player " + str(player.position) + " has a hand of " + str(player.hand) + " for a total of " + str(total(player)))
				elif choice == "stand":
					player.gameOver = True
		print("BLACKJACK Finished playing for all players")

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
					print("Congratulations player " + str(player.position))
				else:
					#player loses
					print("You lost " + str(player.position))
		else:
			#everyone that has not busted and has a value greater than the dealer's wins or if they have blackjack
			for player in players:
				if ((not player.isBusted) and (total(player) > total(dealer))) or player.blackjack:
					print("Congratulations player " + str(player.position))
					#player wins
				else:
					print("You lost " + str(player.position))
					#player loses
	print("BLACKJACK Game over")
	choice = input("Do you want to play again?'").lower()
	if choice == 'yes':
		actualGame()

def actualGame():
	global cards_drawn
	cards_drawn = 0
	game()
	
if __name__ == "__main__":
	actualGame()
