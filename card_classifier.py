import cv2

import rospy
import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import numpy as np
import os
import re
import collections

CAMERA_TOPIC = "/card_camera/color/image_raw"

class CardClassifier():
    def __init__(self):
        self.deck = collections.defaultdict(list)
        self.load_deck(os.path.join(os.getcwd(), "cards/train"))
	rospy.init_node('gesture')
        self.bridge = CvBridge()

    def load_deck(self, path):
        for filename in os.listdir(path):
            match = re.match("([0-9AJQK]+).JPG", filename)
            if not match:
                continue
            cardname = match.groups()[0]
	    filepath = os.path.join(path, filename)
            img = cv2.imread(filepath)
            for card in self.getCards(img, 4):
                self.deck[cardname].append(card)

    def classify(self, card):
        min_diff = np.inf
        label = None
        for cardname, cards in self.deck.items():
	    for card1 in cards:
            	diff = cv2.matchTemplate(card, card1, cv2.TM_SQDIFF_NORMED) 
	    	print(diff)
            	if diff < min_diff:
               	    min_diff = diff
                    label = cardname
        return label

    def recognize(self):
        print("Collecting image...")
        msg = rospy.wait_for_message(CAMERA_TOPIC, Image)
        try:
            img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
            return
	print(img.shape)
	img = self.crop(img.reshape((-1,img.shape[0],img.shape[1])),size=(400,400))[0]
        print("Extracting the card...")
        card = next(self.getCards(img))
	print(card)
        print("Classifying the card...")
        cardname = self.classify(card)
        return cardname

    ### Helpers for image processing ###

    def rectify(self, h):
        h = h.reshape((4,2))
        hnew = np.zeros((4,2),dtype = np.float32)

        add = h.sum(1)
        hnew[0] = h[np.argmin(add)]
        hnew[2] = h[np.argmax(add)]

        diff = np.diff(h,axis = 1)
        hnew[1] = h[np.argmin(diff)]
        hnew[3] = h[np.argmax(diff)]

        return hnew

    def preprocess(self, img):
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray,(5,5),2 )
        thresh = cv2.adaptiveThreshold(blur,255,1,1,11,1)
        return thresh
    
    def getCards(self, im, numcards=1):
        gray = cv2.cvtColor(im,cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray,(1,1),1000)
        flag, thresh = cv2.threshold(blur, 120, 255, cv2.THRESH_BINARY)

        _, contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

        contours = sorted(contours, key=cv2.contourArea,reverse=True)[:numcards]

        for card in contours:
            peri = cv2.arcLength(card,True)
            approx = self.rectify(cv2.approxPolyDP(card,0.03*peri,True))

            h = np.array([ [0,0],[449,0],[449,449],[0,449] ],np.float32)

            transform = cv2.getPerspectiveTransform(approx,h)
            warp = cv2.warpPerspective(im,transform,(450,450))

            yield warp

    def crop(self, imgs, size=None, shift=(0,0)):
        if size is None:
            size = int(imgs.shape[1]), int(imgs.shape[2])
        h, w = size
        imgs_crop = np.zeros((len(imgs), h, w))
        center = int(0.5 * imgs.shape[1]), int(0.5 * imgs.shape[2])
        x0, y0 = center[1] - w//2, center[0] - h//2
        x0, y0 = x0 + shift[0], y0 + shift[1]
        for i, img in enumerate(imgs):
	    imgs_crop[i] = img[y0:y0+h, x0:x0+w]
        return imgs_crop

