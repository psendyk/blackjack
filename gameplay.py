"""
*    Title: python blackjack
*    Author: mjhea0
*    Date: 11/25/2019
*    Availability: https://gist.github.com/mjhea0/5680216
*
"""
import os
import random

#dictionary of card names to values
card_values = {'one':1, 'two':2, 'three':3, 'four':4, 'five':5, 'six':6, 'seven':7, 'eight':8, 'nine':9,
'ten':10, 'jack':10, 'queen':10, 'king':10}

class Player:
	#position, hand, offset, gameOver?
	def __init__(self, isDealer):
		self.hand = []
		self.isDealer = isDealer
		self.gameOver = False
		self.isBusted = False

#this will deal a card to player. it returns the value of the card
#this involves picking up a card, reading it, and placing it by the player
def deal(player, orientation):
	offsets[player] += 1
	#pick up card from physical deck
	#get cardValue from computer vision
	cardValue = 0 #replace later
	#place down at player + offset
	if (orientation == 'down'):
		#face down
	else:
		#face up
	player.hand.append(cardValue)


def deal_hand(player): #deals 
    if player.isDealer:
    	deal(player, 'down')
	else:
		deal(player, 'up')
	deal(player, 'up')
	    

def play_again():
    again = raw_input("Do you want to play again? (Y/N) : ").lower() #replace to gesture 
    if again == "y":
	    dealer_hand = []
	    player_hand = []
	    game()
    else:
	    exit()

def total(player): #add up all cards in player's hand and return value
    total = 0	
    for card in player.hand:
    	#add up total
    return total

def checkBust(player):
	if total(player) > 21:
		player.isBusted = True
		player.gameOver = True

def blackjack(player):
	#differentiate between actual blackjack and other 21
	if ace in player_hand:
		if face_card or 10 in player_hand:
			print(str(player) + " got blackjack")
			player.gameOver = True

def score(dealer_hand, player_hand):
	if total(player_hand) == 21:
		print_results(dealer_hand, player_hand)
		print "Congratulations! You got a Blackjack!\n"
	elif total(dealer_hand) == 21:
		print_results(dealer_hand, player_hand)		
		print "Sorry, you lose. The dealer got a blackjack.\n"
	elif total(player_hand) > 21:
		print_results(dealer_hand, player_hand)
		print "Sorry. You busted. You lose.\n"
	elif total(dealer_hand) > 21:
		print_results(dealer_hand, player_hand)			   
		print "Dealer busts. You win!\n"
	elif total(player_hand) < total(dealer_hand):
		print_results(dealer_hand, player_hand)
   		print "Sorry. Your score isn't higher than the dealer. You lose.\n"
	elif total(player_hand) > total(dealer_hand):
		print_results(dealer_hand, player_hand)			   
		print "Congratulations. Your score is higher than the dealer. You win\n"		

def game():
	choice = 0 #gesture
	numPlayers = raw_input("Enter how many players are playing: ")
	dealer = Player()
	players = []
	for i in range(numPlayers):
		players.append(Player()) #add a new player to players array

	#setup board
	deal_hand(dealer, True)
	blackjack(dealer) #not really sure when to check blackjack
	for player in players: #deal 2 to each once or 1 to each twice?
		deal_hand(player, False) 

	blackjack(dealer)
	for player in players:
		while !player.gameOver #while the player hasnt either busted or quit
			choice = raw_input("Do you want to [H]it, [S]tand, or [Q]uit: ").lower() #convert to computer vision
			if choice == "hit":
				deal(player)
				blackjack(player) #combine blackjack() and checkBust()
				checkBust(player)
			elif choice == "stay":
				player.gameOver = True
	#now all the player turns are over so we can deal for dealer
	while total(dealer) < 17 and !dealer.isBusted:
		deal(dealer)
		checkBust(dealer)
	#now dealer has been dealed we check win conditions on each player
	if dealer.isBusted:
		#everyone that has not busted wins
		for player in players:
			if !player.isBusted:
				#player wins
			else:
				#player loses
	else:
		#everyone that has not busted and has a value greater than the dealer's wins 
		for player in players:
			if !player.isBusted and total(player) > total(dealer):
				#player wins
			else:
				#player loses

	
if __name__ == "__main__":
   game()