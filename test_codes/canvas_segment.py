import cv2
import numpy as np
import os

# bboxes = [(bbox),(bbox)] is list of ROIs


def frame_copy(frame, bboxes):
    h, w, _ = frame.shape #video size as height and width
    canvas = np.ones((h,w,3), np.uint8)*255 #initializing white background of same resolution
    for i in bboxes: 
        cropped = frame[i[0]:i[0]+i[2],i[1]:i[1]+i[3]]
        canvas[i[0]:i[0]+i[2],i[1]:i[1]+i[3]] = cropped
    cv2.imshow("try", canvas)
    cv2.waitKey(0)
    return canvas


frame = cv2.imread('screen.png') #just for reference
bboxes = [[100,100,200,200],[300,300,500,500]]
frame_copy(frame,bboxes)