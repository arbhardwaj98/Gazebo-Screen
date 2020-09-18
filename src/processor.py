#!/usr/bin/env python
from __future__ import print_function

import sys
import rospy
import cv2 as cv
from std_msgs.msg import String
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from PIL import Image as ImageModule
import numpy as np
import face_recognition
import os
import pickle
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  def __init__(self):
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("image_topic",Image,self.callback,queue_size=None)
    self.image_pub = rospy.Publisher("/robot/processed",Bool,queue_size=None)

  def callback(self,data):
    try:
      print(1)
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    (rows,cols,channels) = cv_image.shape
    state = face_detection(cv_image)
    self.image_pub.publish(state)

def face_detection(frame):

  small_frame = cv.resize(frame, (0, 0), fx=0.5, fy=0.5)
  rgb_small_frame = small_frame[:, :, ::-1]
  face_landmarks_list = face_recognition.face_landmarks(rgb_small_frame)
  state=False
  if len(face_landmarks_list)>0:
      face_landmarks = face_landmarks_list[0]
      upperlip = np.array(face_landmarks['top_lip'])
      lowerlip = np.array(face_landmarks['bottom_lip'])
      ulpos = np.mean(upperlip[[8,9],1])
      llpos = np.mean(lowerlip[[8,9],1])
      ulw = ulpos- np.mean(upperlip[[2,3],1])
      llw = np.mean(lowerlip[[2,3],1])-llpos
      dist = llpos-ulpos
      threshold = 0.4*(ulw+llw)
      print("-------")
      if dist>=threshold:
          state=True
      print(dist)
      print(threshold)
      print(state)
  return state

def main(args):
  rospy.init_node('image_converter', anonymous=True)
  ic = image_converter()

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv.destroyAllWindows()

if __name__ == '__main__':
  main(sys.argv)
