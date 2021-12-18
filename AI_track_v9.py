from __future__ import print_function
import sys
import cv2
from random import randint
from KalmanFilter import KalmanFilter
import numpy as np
import matplotlib.pyplot as plt
def automate_tracker():


        bboxes.append(bbox)
        colors.append((randint(64, 255), randint(64, 255), randint(64, 255)))

        k = cv2.waitKey(0) & 0xFF
        if (k == 113):  # q is pressed
            break

    print('Selected bounding boxes {}'.format(bboxes))

    multiTracker = cv2.MultiTracker_create()

    # Initialize MultiTracker
    for bbox in bboxes:
        multiTracker.add(createTrackerByName(trackerType), frame, bbox)

        # get updated location of objects in subsequent frames
        success, boxes = multiTracker.update(frame)

        # draw tracked objects
        for i, newbox in enumerate(boxes):
            p1 = (int(newbox[0]), int(newbox[1]))
            p2 = (int(newbox[0] + newbox[2]), int(newbox[1] + newbox[3]))
            cv2.rectangle(frame, p1, p2, colors[i], -1, 1)
            cv2.putText(frame, "Tag:" + str(i),(int(newbox[0]), int(newbox[1]-10)),0, 0.75, (255,255,255),2)	
        out.write(frame)
        # show frame
        cv2.imshow('MultiTracker', frame)

        # quit on ESC button
        if cv2.waitKey(1) & 0xFF == 27:  # Esc pressed
            break







#
# tracker = cv2.TrackerCSRT_create()
#
#
# videoPath = "run.mp4"
# video = cv2.VideoCapture(videoPath)
#
# while True:
#     k,frame = video.read()
#     cv2.imshow("Tracking",frame)
#     k = cv2.waitKey(30) & 0xff
#     if k == 27:
#         break
# bbox = cv2.selectROI(frame, False)
#
# ok = tracker.init(frame, bbox)
# cv2.destroyWindow("ROI selector")
#
# while True:
#     ok, frame = video.read()
#     ok, bbox = tracker.update(frame)
#
#     if ok:
#         p1 = (int(bbox[0]), int(bbox[1]))
#         p2 = (int(bbox[0] + bbox[2]),
#               int(bbox[1] + bbox[3]))
#         cv2.rectangle(frame, p1, p2, (0,0,255), 2, 2)
#
#     cv2.imshow("Tracking", frame)
#     k = cv2.waitKey(1) & 0xff
#     if k == 27 : break
