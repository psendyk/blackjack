import cv2

import rospy
import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import numpy as np
import os
import collections

CAMERA_TOPIC = "left_hand_camera"

class CardClassifier():
    def __init__(self):
        self.deck = collections.defaultdict(list)
        self.load_deck(os.path.join(os.getcwd(), "cards/train"))
        self.bridge = CvBridge()

    def load_deck(self, path):
        for filename in os.listdir(path):
            match = re.match("([0-9AJQK]+).JPG", filename)
            if not match:
                continue
            cardname = match.groups()[0]
            img = cv2.imread(path)
            for card in self.getCards(img, 4):
                self.deck[cardname].append(card)

    def classify(self, card):
        match = lambda template: cv2.matchTemplate(card, template, cv2.TM_SQDIFF_NORMED)
        min_diff = np.inf
        label = None
        for cardname in self.deck.keys():
            diff = min(map(match, deck[card]))[0][0]
            if diff < min_diff:
                min_diff = diff
                label = cardname
        return label

    def recognize():
        print("Collecting image...")
        msg = rospy.wait_for_message(CAMERA_TOPIC, Image)
        try:
            img = self.bridge.imgmsg_to_cv2(msg, "mono8")
        except CvBridgeError as e:
            print(e)
            return

        print("Extracting the card...")
        card = self.getCards(img)
        print("Classifying the card...")
        cardname = self.classify(card)
        return cardname

    ### Helpers for image processing ###

    @staticmethod
    def rectify(h):
        h = h.reshape((4,2))
        hnew = np.zeros((4,2),dtype = np.float32)

        add = h.sum(1)
        hnew[0] = h[np.argmin(add)]
        hnew[2] = h[np.argmax(add)]

        diff = np.diff(h,axis = 1)
        hnew[1] = h[np.argmin(diff)]
        hnew[3] = h[np.argmax(diff)]

        return hnew

    @staticmethod
    def preprocess(img):
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray,(5,5),2 )
        thresh = cv2.adaptiveThreshold(blur,255,1,1,11,1)
        return thresh
    
    @staticmethod
    def getCards(im, numcards=1):
        gray = cv2.cvtColor(im,cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray,(1,1),1000)
        flag, thresh = cv2.threshold(blur, 120, 255, cv2.THRESH_BINARY)

        _, contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

        contours = sorted(contours, key=cv2.contourArea,reverse=True)[:numcards]

        for card in contours:
            peri = cv2.arcLength(card,True)
            approx = rectify(cv2.approxPolyDP(card,0.02*peri,True))

            h = np.array([ [0,0],[449,0],[449,449],[0,449] ],np.float32)

            transform = cv2.getPerspectiveTransform(approx,h)
            warp = cv2.warpPerspective(im,transform,(450,450))

            yield warp
