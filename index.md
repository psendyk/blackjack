# EECS C106a Fall 2019 Final Project

## Automated BlackJack Dealer [Baxter]

##### Group Number: 41 <br>
##### Group Members: Warren Deng, Pawel Sendyk, Henry Leou, Michael Wang, Bryan Yang <br>


## Introduction
   Blackjack is an extremely popular card game that is played in most casinos. We noticed that throughout a game of Blackjack, the dealer goes through a standard procedure: setting up the board, dealing cards to the players, dealing cards to itself, and then checking who won. We thought that this process could be completely automated by replacing the dealer with a robot. Our goal was for multiple players to be able to play a game of Blackjack with the Baxter robot. In doing so we wanted to make the Baxter behave as humanlike as possible by minimizing the amount a player had to manually interact with a computer interface.
    To replace the dealer, we needed to accomplish three tasks. First, we needed to create the game of Blackjack in code so that the robot could deal and play in accordance to the rules. Second, we needed the robot to be able to mimic the motions of the dealer. These included drawing a card, dealing a card to its correct location, and flipping a card if it was originally face down. Lastly, we needed to the robot to be able to interpret both the player’s actions (hit or stand) and card values from an image of the card. Together, these three tasks represented the Gameplay, Kinematics, and Computer Vision aspects of our project. 
   The most obvious real-world application for our project is replacing Blackjack dealers in casinos. Casinos would save a lot of money and would still satisfy players who would rather play Blackjack with physical cards instead of digitally. Our implementation of drawing, interpreting, and dealing cards could also be applied to different casino card games such as poker.


## Design
![File Hierarchy](/blkjk_filestruct.jpg)
<!--img src="https://github.com/psendyk/blackjack/blob/master/blkjk_filestruct.jpg" alt="Project file hierarchy" class="inline"/-->

#### Design Criteria
   We had three design rules that we wanted to folow:
   1. Players should only need to communicate with the Baxter via hand gestures in order to play
   2. Baxter should be relatively fast and precise when dealing cards
   3. All major rules of Blackjack should be followed (such as not revealing the dealer's second card until after everyone's turn)
#### Our Design
   As described previously, we decided to split our project into three parts: Gameplay, Kinematics, and Computer Vision. After finishing every part we would combine them in the blackjack.py file so that only one script would need to be run.  
#### Design Choices
###### Gameplay
   We wanted Baxter to play blackjack properly, which in terms of gameplay included multiple player support and the ability to hit, stay, split, or double. Due to time constraints and practicality, we ultimately went with multiple player support as well as the ability to hit and stay. Splitting may be simple for humans to perform but would require significant game logic modifications, and doubling involves betting which we chose to disregard.
###### Kinematics
We decided to attach a gripper on each of Baxter's arms to allow it to efficiently pick up, flip, and deal the cards. For Baxter's movement we planned on primarily using inverse kinematics with the MoveIt package to perform motion planning for card manipulation. We also added constraints and objects within MoveIt, similar to how Lab 8 worked, to avoid hitting the table or the deck. Four different motions are required to deal the blackjack game. Baxter needs to pick up the card, look at it, and place it down. Placing the card face up requires the card to be flipped, which is achieved by handing the card off to the other arm. Additionally, movements of both arms are parallelized for efficiency, meaning that Baxter can deal a card while pick up the next.
###### Computer Vision
#### Design Choices Impact (how did the design choices affect things)


## Implementation
#### Gameplay 
Our implementation of gameplay follows the standard rules of Blackjack and supports multiple players. It is located in blackjack.py.  


To keep track of everyone playing, we created a Player class that has the following attributes: hand (keeps a count of what cards the player has), isDealer (if the player is a dealer), gameOver (if the player’s turn is over), isBusted (if the player busted while hitting), blackjack (if the player had initially gotten blackjack), position (used by kinematics to determine where the player is), offset (keeps track of how many cards the player has, used by the kinematics to determine where to place the card relative to position).


When the game starts, a user-inputted amount of Players are created, along with the dealer. They are each dealt two cards. The dealer’s hand is then checked to see if it has blackjack. If it does, the game is over and each player who doesn’t have blackjack loses. If it doesn’t, the game continues. 


Now each player is allowed to play their turn. A player’s turn consists of either hitting or standing (in which case their turn is over). Everytime they hit a card is appended to their hand. This hand is then checked to make sure they haven’t busted. Aces are handled automatically by choosing the highest possible value without having the hand bust.
If all players have not busted after their turns, the dealer gets its turn. It is forced to hit until it busts or passes a hand value of 17. If the dealer busts, everyone who hasn’t busted wins. If the dealer doesn’t bust, every player who hasn’t busted that has a higher hand value than the dealer wins. The game is now over and all variables are reset.
#### Kinematics
The robot kinematics and the various actions are broken down into several smaller poses, located in `arm_kinematics.py`.

When testing to find out what positions we wanted Baxter in for its various actions, we found that MoveIt did not produce consistent or desirable results. For example, when Baxter draws and looks at a card the card should be tilted up at an angle to face the head camera, but MoveIt frequently held the card perfectly straight and not at a good camera angle. Therefore, we ended up using a combination of both forward and inverse kinematics to consistently achieve very specific desired postures.
#### Computer Vision

#### Card Holder
We created a custom card holder for our robot to use. This gave us more precision when picking up the cards because they wouldn’t slide around when being picked up. We decided to laser cut the card holder because it was relatively cheap and much faster than 3D printing. We measured the cards, designed the base and side parts in SolidWorks, and assembled them virtually them to make sure they fit. We then laser cut the box using ¼ inch acrylic.    
![Test](/display_images/index.png)
![CardBox](/display_images/cardbox.png)




## Results
Our robot was able to follow all of the functionalities that we needed it to do for kinematics. This includes drawing a card, holding a card up to the front camera to read, handing a card off to the other gripper, flipping a card from the table, and dealing to any player. Additionally, it was able to successfully recognize hand gestures of players. However, card recognition did not work very well with the Baxter head camera despite working with other cameras including images taken by an iPhone. We deduced that the resolution and brightness on the ASIMOV camera was too low for our data set. 
##### Video of drawing and dealing to dealer:

<iframe width="560" height="315" src="https://www.youtube.com/embed/pnGARduixqI" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

##### Video of dealing to player:

<iframe width="560" height="315" src="https://www.youtube.com/embed/nDfTyWIa99I" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

##### Video of handoff:
<iframe width="560" height="315" src="https://www.youtube.com/embed/kDGdsuQSVFA" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

##### Video of flipping card from table:

<iframe width="560" height="315" src="https://www.youtube.com/embed/q_Hr4I73LuA" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

## Conclusion

###### Demo Video
<iframe width="560" height="315" src="https://www.youtube.com/embed/RZcsRvl0Fs4" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

## Team


## Additional Materials
