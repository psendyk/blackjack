import cv2

import rospy
import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from keras.models import Sequential
from keras.layers import Dense
from keras.layers.convolutional import Conv2D, MaxPooling2D
from keras.models import load_model

import dill
import threading

import numpy as np

REALSENSE_TOPIC = "/camera/raw_image"
BUFF_SIZE = 100 # We should set this to rate * time (~2s)
SAMPLE_SIZE = 10 # How many images to look at

class GestureClassifier():
    def init(self, model):
        self.model = load_model(model)
        self.buffer = []
        self.bridge = CvBridge()
        # Set up topic subscriber and run it in background
        self.listener = threading.Thread(target=listener)
        self.listener.start()

    def recognize(self):
        # get data the past ~2s
        # select 10 random images
        # preprocess -> classify -> select gesture with most votes
        imgs = map(self.preprocess, np.random.choice(self.buffer, SAMPLE_SIZE))
        pred = [self.classify(img) for img in imgs]
        return sum(pred) >= SAMPLE_SIZE / 2
    
    def listener(self):
        rospy.init_node('gesture_listener', anonymous=True)
        rospy.Subscriber(REALSENSE_TOPIC, Image, self.enqueue)
        rospy.spin()
    
    def classify(self, X):
        if self.model is None:
            print("Set up the model first.")
            return
        return model.predict(X)

    def enqueue(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, "mono8")
        except CvBridgeError as e:
            print(e)
            return
        self.buffer.append(img)
        if len(self.buffer) > BUFF_SIZE:
            self.buffer.pop(0)

    def crop(self, imgs, size=None, shift=(0,0)):
        if size is None:
            size = int(imgs.shape[1]), int(imgs.shape[2])
        h, w = size
        imgs_crop = np.zeros((len(imgs), h, w))
        center = int(1/2 * imgs.shape[1]), int(1/2 * imgs.shape[2])
        x0, y0 = center[1] - w//2, center[0] - h//2
        x0, y0 = x0 + shift[0], y0 + shift[1]
        for i, img in enumerate(imgs):
            imgs_crop[i] = img[y0:y0+h, x0:x0+w]
        return imgs_crop

    def threshold(self, thres=160):
        imgs_thres = np.zeros(imgs.shape)
        for i, img in enumerate(imgs):
        idx = np.where(img > thres)
        imgs_thres[i][idx] = 255
        return imgs_thres

    def preprocess(self, size=(240, 360), shift=(20,0), thres=160):
        imgs = self.crop(imgs, size=size, shift=shift)
        imgs = self.threshold(imgs, thres=thres)
        return imgs

