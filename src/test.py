#!/usr/bin/env python
from __future__ import print_function

import sys
import rospy
import cv2 as cv
from std_msgs.msg import String
from sensor_msgs.msg import Image
from PIL import Image as ImageModule
import numpy as np
import face_recognition
import os
import pickle
from cv_bridge import CvBridge, CvBridgeError


def face_detection(frame):

  small_frame = cv.resize(frame, (0, 0), fx=0.5, fy=0.5)
  rgb_small_frame = small_frame[:, :, ::-1]
  face_landmarks_list = face_recognition.face_landmarks(rgb_small_frame)
  state="closed"
  if len(face_landmarks_list)>0:
      face_landmarks = face_landmarks_list[0]
      upperlip = np.array(face_landmarks['top_lip'])
      lowerlip = np.array(face_landmarks['bottom_lip'])
      ulpos = np.mean(upperlip[[8,9],1])
      llpos = np.mean(lowerlip[[8,9],1])
      ulw = ulpos- np.mean(upperlip[[2,3],1])
      llw = np.mean(lowerlip[[2,3],1])-llpos
      dist = llpos-ulpos
      threshold = 0.5*(ulw+llw)
      print("-------")
      if dist>=threshold:
          state="open"
      print(state)
      upperlip.reshape(-1,1,2)
      upperlip = 2*upperlip.astype(np.int32)
      lowerlip.reshape(-1,1,2)
      lowerlip = 2*lowerlip.astype(np.int32)
      frame = cv.polylines(frame,[upperlip],True,(255,0,0),thickness=2)
      frame = cv.polylines(frame,[lowerlip],True,(0,255,255),thickness=2)
      font = cv.FONT_HERSHEY_SIMPLEX
      cv.putText(frame,state,(20,50), font, 2,(0,0,255),2,cv.LINE_AA)
  return frame

def mainloop(cap):
  ret, frame = cap.read()
  if ret:
    frame = face_detection(frame)
    cv.imshow("frame", frame)
    cv.waitKey(1)

if __name__ == '__main__':
   cap = cv.VideoCapture(0)
   try:
     while True:
        mainloop(cap)
   except KeyboardInterrupt:
     print('interrupted!')
   cap.release()
