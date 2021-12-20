from __future__ import print_function
import sys
import cv2
from random import randint
# from KalmanFilter import KalmanFilter
import numpy as np
import matplotlib.pyplot as plt

trackerTypes = ['BOOSTING', 'MIL', 'KCF', 'TLD', 'MEDIANFLOW', 'GOTURN', 'MOSSE', 'CSRT']



def createTrackerByName(trackerType):
    # Create a tracker based on tracker name
    if trackerType == trackerTypes[0]:
        tracker = cv2.TrackerBoosting_create()
    elif trackerType == trackerTypes[1]:
        tracker = cv2.TrackerMIL_create()
    elif trackerType == trackerTypes[2]:
        tracker = cv2.TrackerKCF_create()
    elif trackerType == trackerTypes[3]:
        tracker = cv2.TrackerTLD_create()
    elif trackerType == trackerTypes[4]:
        tracker = cv2.TrackerMedianFlow_create()
    elif trackerType == trackerTypes[5]:
        tracker = cv2.TrackerGOTURN_create()
    elif trackerType == trackerTypes[6]:
        tracker = cv2.TrackerMOSSE_create()
    elif trackerType == trackerTypes[7]:
        tracker = cv2.TrackerCSRT_create()
    else:
        tracker = None
        print('Incorrect tracker name')
        print('Available trackers are:')
        for t in trackerTypes:
            print(t)

    return tracker


if __name__ == '__main__':

    print("Default tracking algoritm is CSRT \n"
          "Available tracking algorithms are:\n")
    for t in trackerTypes:
        print(t)

    trackerType = "CSRT"

    # Set video to load
    videoPath = "/home/aiotel/sambhav/yolov4-deepsort/data/video/test.mp4"
    # Create a video capture object to read videos
    cap = cv2.VideoCapture(videoPath)
    fourcc = cv2.VideoWriter_fourcc(*'MP4V')
    out = cv2.VideoWriter('output.mp4', fourcc, 20.0, (1280, 720))

    # Read first frame
    success, frame = cap.read()
    # quit if unable to read the video file
    if not success:
        print('Failed to read video')
        sys.exit(1)

    ## Select boxes
    bboxes = []
    colors = []

    while True:
        # draw bounding boxes over objects
        # selectROI's default behaviour is to draw box starting from the center
        # when fromCenter is set to false, you can draw box starting from top left corner
        bbox = cv2.selectROI('MultiTracker', frame)
        bboxes.append(bbox)
        colors.append((randint(64, 255), randint(64, 255), randint(64, 255)))
        print(bboxes)
        print("Press q to quit selecting boxes and start tracking")
        print("Press any other key to select next object")
        k = cv2.waitKey(0) & 0xFF
        if (k == 113):  # q is pressed
            break

    print('Selected bounding boxes {}'.format(bboxes))

    multiTracker = cv2.MultiTracker_create()

    # Initialize MultiTracker
    for bbox in bboxes:
        multiTracker.add(createTrackerByName(trackerType), frame, bbox)

    # Process video and track objects
    while cap.isOpened():
        success, frame = cap.read()
        if not success:
            break

        centers = bboxes






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
'''
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
'''







# tracker = cv2.TrackerCSRT_create()


# videoPath = "run.mp4"
# video = cv2.VideoCapture(videoPath)

# while True:
#     k,frame = video.read()
#     cv2.imshow("Tracking",frame)
#     k = cv2.waitKey(30) & 0xff
#     if k == 27:
#         break
# bbox = cv2.selectROI(frame, False)

# ok = tracker.init(frame, bbox)
# cv2.destroyWindow("ROI selector")

# while True:
#     ok, frame = video.read()
#     ok, bbox = tracker.update(frame)

#     if ok:
#         p1 = (int(bbox[0]), int(bbox[1]))
#         p2 = (int(bbox[0] + bbox[2]),
#               int(bbox[1] + bbox[3]))
#         cv2.rectangle(frame, p1, p2, (0,0,255), 2, 2)

#     cv2.imshow("Tracking", frame)
#     k = cv2.waitKey(1) & 0xff
#     if k == 27 : break
