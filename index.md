# EECS C106 Fall 2019 Final Project

## BlackJack Dealer [Baxter]

##### Group Number: 41 <br>
##### Group Members: Warren Deng, Pawel Sendyk, Henry Leou, Michael Wang, Bryan Yang <br>


## Introduction
The end goal of our project was to have Baxter act as a blackjack dealer, while acting as similarly to a real blackjack dealer as possible. This meant that we wanted Baxter to be able to behave in certain ways and perform specific actions:
* Deal and flip playing cards with two suction grippers
* Recognize playing cards with computer vision
* Recognize hand gestures as blackjack-specific player actions, using computer vision
* Play through a game of blackjack properly, which involves: who has which cards, what the values of players' hands are, and when the game terminates

## Design


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


### Computer Vision
##### Card Recognition  
We applied multi-step preprocessing to the captured image to extract the card from it. We greyscale, blur, and threshold to get rid of the noise, and find contours of the card.
Once we've found the contours, we use an affine transform to get the card in a rectangular form.
In the recognition step we used [template matching](https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_template_matching/py_template_matching.html) to compare the captured card against every card in our deck, and output the one with minimum squared difference.
##### Hand-gesture Recognition
We started by collecting a dataset containing ~100 frames for each gesture.
We built and trained a 2-layer convolutional neural network (CNN) which gave us 100% test accuracy.
The CNN creates filters in training, which correspond to different elements of a hand.
Thanks to the static background, we were able to threshold the images and create a mask, which we then feed into our neural net. This additional preprocessing allowed for a simplified network architecture, which turns out to be the biggest [...] in terms of time and memory. Below is an example input into our network.   
[insert pic of preprocessing]   
The main component of our network were two sets of a convolutional layer with [ReLU] (link) activation function, followed by [max pooling] (link) layer.
Since our implementation supported only two choices (hit or stay), we used a sigmoid activation function in the final layer, which approximates the probability of the corresponding gesture.
We also augmented the dataset with rotations and translations, which made the network more robust and was necessary for multi-player support.  
After the robot prompted the player to either hit or stay, we collected multiple frames over ~2s window, classified each, and chose the gesture with the most votes.

## Results

Our neural net was very confident of its choices and hand gestures recognition never failed us.   


## Conclusion

Due to limited access to the robots, we worked on the computer vision parts remotely. We underestimated how much time integration takes and ran into some issues while implementing the algorithms on the robot's hardware.  
As illustrated, our algorithm had trouble extracting the cards from Baxter camera.
[insert pic]
For comparison, here's how it worked with RealSense camera, which we were just a little bit short of integrating into our project.
[insert pic]
This might not seem like a big difference but sometimes it was enough to misclassify the cards, which wasn't acceptable for our blackjack dealer.
Regarding the card recognition, we underestimated how much tim integration takes. Given more time, we would have integrated RealSense camera into our project to assure that the robot can read the cards correctly.

## Team


## Additional Materials
