# EECS C106a Fall 2019 Final Project

## Automated BlackJack Dealer [Baxter]

##### Group Number: 41 <br>
##### Group Members: Warren Deng, Pawel Sendyk, Henry Leou, Michael Wang, Bryan Yang <br>


## Introduction
   Blackjack is an extremely popular card game that is played in most casinos. We noticed that throughout a game of Blackjack, the dealer goes through a standard procedure: setting up the board, dealing cards to the players, dealing cards to itself, and then checking who won. We thought that this process could be completely automated by replacing the dealer with a Baxter robot. In doing so we wanted to make the Baxter behave as humanlike as possible by minimizing the amount a player had to manually interact with a computer interface.
  
   To replace the dealer, we needed to accomplish three tasks. First, we needed to create the game of Blackjack in code so that the robot could deal and play in accordance to the rules. Second, we needed the robot to be able to mimic the motions of the dealer. These included drawing a card, dealing a card to its correct location, and flipping a card if it was originally face down. Lastly, we needed to the robot to be able to interpret both the player’s actions (hit or stand) and card values from an image of the card. We determined that computer vision using a real sense camera and the Baxter head camera was the best way to do this. Together, these three tasks represented the Gameplay, Kinematics, and Computer Vision aspects of our project. 
  
   The most obvious real-world application for our project is replacing Blackjack dealers in casinos. Casinos would save a lot of money and would still satisfy players who would rather play Blackjack with physical cards instead of digitally. Our implementation of drawing, interpreting, and dealing cards could also be applied to different casino card games such as poker.


## Design
  The desired functionality of our project was for multiple players to be able to play a game of Blackjack with the Baxter robot. The design criteria is that the players must not be able to directly interact with the terminal aside from starting the game. They should be able to play the game just with the Baxter. 


## Implementation
#### Gameplay 
Our implementation of gameplay follows the standard rules of Blackjack and supports multiple players. It is located in blackjack.py.  
To keep track of everyone playing, we created a Player class that has the following attributes: hand (keeps a count of what cards the player has), isDealer (if the player is a dealer), gameOver (if the player’s turn is over), isBusted (if the player busted while hitting), blackjack (if the player had initially gotten blackjack), position (used by kinematics to determine where the player is), offset (keeps track of how many cards the player has, used by the kinematics to determine where to place the card relative to position). 
When the game starts, a user-inputted amount of Players are created, along with the dealer. They are each dealt two cards. The dealer’s hand is then checked to see if it has blackjack. If it does, the game is over and each player who doesn’t have blackjack loses. If it doesn’t, the game continues. 
Now each player is allowed to play their turn. A player’s turn consists of either hitting or standing (in which case their turn is over). Everytime they hit a card is appended to their hand. This hand is then checked to make sure they haven’t busted. Aces are handled automatically by choosing the highest possible value without having the hand bust.
If all players have not busted after their turns, the dealer gets its turn. It is forced to hit until it busts or passes a hand value of 17. If the dealer busts, everyone who hasn’t busted wins. If the dealer doesn’t bust, every player who hasn’t busted that has a higher hand value than the dealer wins. The game is now over and all variables are reset.



## Results
Our robot was able to follow all of the functionalities that we needed it to do for kinematics. This includes drawing a card, holding a card up to the front camera to read, handing a card off to the other gripper, flipping a card from the table, and dealing to any player. Additionally, it was able to successfully recognize hand gestures of players. However, card recognition did not work very well with the Baxter head camera despite working with other cameras including images taken by an iPhone. We deduced that the resolution and brightness on the ASIMOV camera was too low for our data set. 
##### Video of drawing and dealing to dealer:
##### Video of dealing to player:
##### Video of handoff:
##### Video of flipping card from table:
##### Video of gesture recognization: 


## Conclusion


## Team


## Additional Materials
