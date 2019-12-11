import cv2

import rospy
import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from keras.models import Sequential
from keras.layers import Dense
from keras.layers.convolutional import Conv2D, MaxPooling2D
from keras.models import load_model

import numpy as np
import time

CAMERA_TOPIC = "/camera/color/image_raw"
MODEL_PATH = "model.h5"
NUM_SAMPLES = 10 # How many images to look at
WINDOW_SIZE = 2 # 2s

class GestureClassifier():
    def __init__(self):
        print("Loading tensorflow model...")
        self.model = load_model(MODEL_PATH)
	print("Model loaded!")
        self.bridge = CvBridge()
        rospy.init_node('gesture_classifier', anonymous=True)
	print("Ready to classify gestures!")

    def recognize(self):
	# collect images over a fixed window time
	# preprocess -> classify -> select gesture with most votes
	imgs = []
	print("Collecting data...")
	for _ in range(NUM_SAMPLES):
	    msg = rospy.wait_for_message(CAMERA_TOPIC, Image)
	    try:
                img = self.bridge.imgmsg_to_cv2(msg, "mono8")
            except CvBridgeError as e:
                print(e)
                return
	    imgs.append(img)
	    time.sleep(WINDOW_SIZE / NUM_SAMPLES)
	
	print("Classifying data...")
	imgs = np.array(imgs)
       	imgs = self.preprocess(imgs)
	imgs = imgs.reshape((-1, imgs.shape[1], imgs.shape[2], 1))
	pred = self.classify(imgs).reshape((-1,))
	print(pred)
	pred = map(int, pred > 0.5) 
	print(list(pred))
	label = np.argmax(np.bincount(pred))
	gesture = "hit" if label == 1 else "stay"
	
	print("Gesture recognized as {}.".format(gesture))
	return np.argmax(np.bincount(pred))   
 
    def classify(self, X):
        if self.model is None:
            print("Set up the model first.")
            return
        return self.model.predict(X)

    ### Helpers for image processing ###    

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

    def threshold(self, imgs, thres=160):
        imgs_thres = np.zeros(imgs.shape)
        for i, img in enumerate(imgs):
            idx = np.where(img > thres)
	    print(idx)
            imgs_thres[i][idx] = 1
        return imgs_thres

    def preprocess(self, imgs, size=(240, 360), shift=(20,0), thres=160):
        imgs = self.crop(imgs, size=size, shift=shift)
        imgs = self.threshold(imgs, thres=thres)
        return imgs

