# import the opencv library
import cv2
import time
from random import randint

from numpy.lib.function_base import median
# import mp_test_pose
import numpy as np
from scipy.stats import t
import matplotlib.pyplot as plt
#from sklearn.svm import SVC
import short_dist
#import aura
multiTracker = cv2.MultiTracker_create()


bboxes=[]
colors=[]
flag =0

def re_id_initiate(tracker_bbox,frame):
        print("******************************Initiated", )
        bboxes.append(tracker_bbox)
        colors.append((randint(64, 255), randint(64, 255), randint(64, 255)))
        for bbox in bboxes:
              multiTracker.add(( cv2.TrackerCSRT_create()), frame, bbox)
        return flag==1
       

def re_id_update(frame,count):
             
        success, boxes = multiTracker.update(frame)
        for i, newbox in enumerate(boxes):
                  print("**********************************Updating")
                  p1 = (int(newbox[0]), int(newbox[1]))
                  p2 = (int(newbox[0] + newbox[2]), int(newbox[1] + newbox[3]))
                  cv2.rectangle(frame, p1, p2, colors[i], -1, 1)
                  cv2.putText(frame, "Tag:" + count,(int(newbox[0]), int(newbox[1]-10)),0, 0.75, (255,255,255),2)

          

'''
  closet_point = short_dist.closest_point(point1,point2)
  v1,v2,v3 = closet_point
  if v3 <radius :
      flag_collision = True
      cv2.putText(frame_copy,"collision True",(100,100),cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 0), 2)
'''
       
        
        
