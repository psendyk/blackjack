# EECS C106a Fall 2019 Final Project

## Automated BlackJack Dealer [Baxter]

##### Group Number: 41 <br>
##### Group Members: Warren Deng, Henry Leou, Pawel Sendyk , Michael Wang, Bryan Yang <br>


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
We decided to use a neural network for our implementation of the gesture classifier, which allowed for more generalization without adding more complexity to the code. It's also easily extensible to more gestures, such as split or double down, which we collected in our dataset but decided not to include in the gameplay as mentioned above.
We built the card classifier for a specific deck of cards, i.e. it can only work with one type of cards at a time. We made sure to make it easy to load and use different decks, provided images of it. The size of a deck of cards creates a 52-dimensional classification problem, which we decided would be best solved with a minimum difference algorithm.


#### Design Choices Impact
##### Robustness
By using forward kinematics, Baxter's motion was very consistent and robust, able to regularly achieve satisfactory results over several trials. The drawing motion was quite consistent; apart from unexpected suction gripper failures Baxter was able to draw cards easily. However, due to the use of dual suction grippers and the way we had Baxter hand off the card between grippers, the card flipping action had less consistent results. Most of the time the card handoff went smoothly, but on occasion the receiving gripper would not activate in time or would end up slightly out of position, resulting in Baxter dropping the card.

Our computer vision for recognizing hand gestures was quite robust, with high accuracy given a blank background. The way we collected and augmented the dataset allowed our to recognize gestures of multiple players from different angles and distances. For card recognition on the other hand, while functional in a testing environment our computer vision did not work well with the Baxter's head camera since the background of the captured image was too noisy.
##### Durability
For Baxter's kinematics, by mostly using forward kinematics our motion system was quite durable, able to achieve the same position over and over again. Combined with MoveIt constraints and inverse kinematics for other actions, Baxter's movements are easily repeatable between different games of blackjack.
##### Efficiency
When designing the kinematics, we initially started with moving a single arm at a time. To make Baxter waste less time moving around and spend more time playing the game we combined left and right arm movements into a single action, therefore improving efficiency. Additionally, using two suction grippers versus one gripper plus one claw saves time by simply handing off a card instead of handing off, rotating, and again handing off a card. Possible improvements would be to find other actions where the arm motions can be parallellized or otherwise further refined.

### Computer Vision
##### Card Recognition  
We used computer vision to allow the robot to recognize the cards it's dealing.
In our final implementation, we used template matching which compares the captured card against every card in our deck.
The most important component of this part is the accuracy and robustness to image rotation and translation which arises with variance in Baxter's movements.
We had to adjust our algorithms for Baxter's low quality camera, and make some trade-offs between speed and accuracy. Our algorithm had problems with lower quality images and at the end we mounted an external camera to Baxter's head to improve the accuracy.
##### Hand-gesture Recognition
Furthermore, to make the human-robot interaction more natural, we implemented hand gestures for the actions players can take.
In order to accomplish this, we trained a neural net classifier using manually collected dataset.  

## Implementation
#### Gameplay 
Our implementation of gameplay follows the standard rules of Blackjack and supports multiple players. It is located in blackjack.py.  


To keep track of everyone playing, we created a Player class that has the following attributes: hand (keeps a count of what cards the player has), isDealer (if the player is a dealer), gameOver (if the player’s turn is over), isBusted (if the player busted while hitting), blackjack (if the player had initially gotten blackjack), position (used by kinematics to determine where the player is), offset (keeps track of how many cards the player has, used by the kinematics to determine where to place the card relative to position).


When the game starts, a user-inputted amount of Players are created, along with the dealer. They are each dealt two cards. The dealer’s hand is then checked to see if it has blackjack. If it does, the game is over and each player who doesn’t have blackjack loses. If it doesn’t, the game continues. 


Now each player is allowed to play their turn. A player’s turn consists of either hitting or standing (in which case their turn is over). Everytime they hit a card is appended to their hand. This hand is then checked to make sure they haven’t busted. Aces are handled automatically by choosing the highest possible value without having the hand bust.
If all players have not busted after their turns, the dealer gets its turn. It is forced to hit until it busts or passes a hand value of 17. If the dealer busts, everyone who hasn’t busted wins. If the dealer doesn’t bust, every player who hasn’t busted that has a higher hand value than the dealer wins. The game is now over and all variables are reset.
#### Kinematics
The robot kinematics and the various actions are broken down into several smaller poses, located in `arm_kinematics.py`.

When testing to find out what positions we wanted Baxter in for its various actions, we found that MoveIt did not produce consistent or desirable results. For example, when Baxter draws and looks at a card the card should be tilted up at an angle to face the head camera, but MoveIt frequently held the card perfectly straight and not at a good camera angle. Therefore, we ended up using a combination of both forward and inverse kinematics to consistently achieve very specific desired postures. We manipulated Baxter's arms into our desired positions, used `tf_echo` to record the angles, then used forward kinematics to achieve that position in our code.


#### Computer Vision
##### Card Recognition  
We applied multi-step preprocessing to the captured image to extract the card from it. We greyscale, blur, and threshold to get rid of the noise, and find contours of the card.
Once we've found the contours, we use an affine transform to get the card in a rectangular form.
In the recognition step we used [template matching](https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_template_matching/py_template_matching.html) to compare the captured card against every card in our deck, and output the one with minimum squared difference.
##### Hand-gesture Recognition
We started by collecting a dataset containing ~100 frames for each gesture.
We built and trained a 2-layer convolutional neural network ([CNN](https://medium.com/@RaghavPrabhu/understanding-of-convolutional-neural-network-cnn-deep-learning-99760835f148)) which gave us 100% test accuracy.
The CNN creates filters in training, which correspond to different elements of a hand.
Thanks to the static background, we were able to threshold the images and create a mask, which we then feed into our neural net. This additional preprocessing allowed for a simplified network architecture, which turns out to be the biggest expense in terms of time and memory. Below is an example of the original frame and input into our network.   
![hand_original](/display_images/hand_original.jpg) ![arrow](/display_images/arrow.jpg) ![hand_input](/display_images/hand_thres)
The main component of our network were two sets of a convolutional layer with [ReLU](https://en.wikipedia.org/wiki/Rectifier_(neural_networks)) activation function, followed by a [max pooling](https://www.quora.com/What-is-max-pooling-in-convolutional-neural-networks) layer.
Since our implementation supported only two choices (hit or stay), we used a sigmoid activation function in the final layer, which approximates the probability of the corresponding gesture.
We also augmented the dataset with rotations and translations, which made the network more robust and was necessary for multi-player support.  
After the robot prompted the player to either hit or stay, we collected multiple frames over ~2s window, classified each, and chose the gesture with the most votes.

#### Card Holder
We created a custom card holder for our robot to use. This gave us more precision when picking up the cards because they wouldn’t slide around when being picked up. We decided to laser cut the card holder because it was relatively cheap and much faster than 3D printing. We measured the cards, designed the base and side parts in SolidWorks, and assembled them virtually them to make sure they fit. We then laser cut the box using ¼ inch acrylic.    
![Test](/display_images/index.png =250x) ![CardBox](/display_images/cardbox.png =250x)
<img src="/display_images/index.png" alt="test" width="250"/> <img src="/display_images/cardbox.png" alt="cardbox" width="250"/>

## Results
Our robot was able to follow all of the functionalities that we needed it to do for kinematics. This includes drawing a card, holding a card up to the front camera to read, handing a card off to the other gripper, flipping a card from the table, and dealing to any player.   

Additionally, it was able to successfully recognize hand gestures of players. Our neural net was very confident of its choices and hand gestures recognition never failed us. An example can be seen in the full demo below.

Due to limited access to the robots, we worked on the computer vision parts remotely. We underestimated how much time integration takes and ran into some issues while implementing the algorithms on the robot's hardware.  
As illustrated, our algorithm had trouble extracting the cards from Baxter camera.
[insert pic]
For comparison, here's how it worked with RealSense camera, which we were just a little bit short of integrating into our project.
[insert pic]
This might not seem like a big difference but sometimes it was enough to misclassify the cards, which wasn't acceptable for our blackjack dealer.


##### Video of drawing and dealing to dealer:

<iframe width="560" height="315" src="https://www.youtube.com/embed/pnGARduixqI" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

##### Video of dealing to player:

<iframe width="560" height="315" src="https://www.youtube.com/embed/nDfTyWIa99I" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

##### Video of handoff:
<iframe width="560" height="315" src="https://www.youtube.com/embed/kDGdsuQSVFA" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

##### Video of flipping card from table:
<iframe width="560" height="315" src="https://www.youtube.com/embed/q_Hr4I73LuA" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>


## Conclusion

Overall, we were satisfied with how our project turned out. All of the components support multiple players and the human-robot interaction felt pretty natural.
 Given more time, we would have integrated another RealSense camera into our project to assure that the robot can read the cards correctly with high enough accuracy.

###### Demo Video
<iframe width="560" height="315" src="https://www.youtube.com/embed/RZcsRvl0Fs4" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

## Team
#### Warren Deng
Warren is a third year double major in EECS and Mechanical Engineering. He has taken EE16B and EE120 and hopes to expand his knowledge in the field of Mechatronics. He worked at an defense contractor doing power engineering his freshman year. For the project, Warren worked on gameplay and kinematics. He wrote most of the gameplay and helped integrate the kinematics so that they worked when neccessary. He also designed the card holder in SolidWorks. 
#### Henry Leou
#### Pawel Sendyk
#### Michael Wang
#### Bryan Yang
Bryan is a third year EECS major, with an interest in both hardware and software. He has taken CS162, EE120, and simultaneously took this class (EE106A) with EE149A. For the project, Bryan designed the basic flow of the gameplay and kinematics, laying out the game's states and Baxter actions, and helped integrate them. He also had to laser cut the card holder since he was the only one with direct access to a laser cutter.


## Additional Materials
Github Project Repository: https://github.com/psendyk/blackjack
