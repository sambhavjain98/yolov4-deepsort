import os 
import cv2
import mp_test
import numpy as np
from draw_utils import draw_utils
from random import randint
# prev_person_count = 0
# pres_person_count = 0
from mp_test import mp_value
class person_tracking():
    # prev_person_count = 
    def __init__(self):
        self.pres_person_count = 0
        self.prev_person_count = 0
        self.pres_bboxes = np.array([])
        self.prev_bboxes = np.array([])
        self.orig_frame  = None
        self.multiTracker = cv2.MultiTracker_create()
        self.colors=[]
        self.draw = draw_utils()
        # FLAGS 
        self.multi_tracker_init_flag = False

    def segment_initiate(self,frame,bboxes,count):
        # print("Person tracking ")
        # print(bboxes,count)
        # Bboxes updating
        # if bboxes is not None:
    
        bboxes_temp = []
        for i in bboxes:
            # print([int(j) for j in i])
            bboxes_temp.append([int(j) for j in i])

        self.prev_bboxes = self.pres_bboxes
        self.pres_bboxes = bboxes_temp
        
        print("count"+str(self.prev_bboxes)+"pres count"+ str(self.pres_bboxes))
        
        # Count updation 
        self.prev_person_count = self.pres_person_count
        self.pres_person_count = count
        # Frame is resized and colored 
        self.orig_frame = frame 

        # Condition 1 - Check if there are any person in room
        if(self.pres_person_count >= 1):
            # seg_frame - superimposed white frame with multiple bboxes
            # sepereate frames - list of segmented bboxes 
            seg_frame,seperate_frames = self.segment_canvas(self.orig_frame,self.pres_bboxes)

            # Condition 2 - Check if the new person enters the room or not
            if(self.pres_person_count > self.prev_person_count):
                # TODO : Looping for multiple entries together 
                # Adding a person ID 
                for i in seperate_frames:

                    x_cor,y_cor,z_cor,_ = mp_test.mp_value(i)
                    if(x_cor is not None and y_cor is not None):
                        self.colors.append((randint(64, 255), randint(64, 255), randint(64, 255)))
                        cv2.circle(seg_frame, (int(x_cor),int(y_cor)),40,self.colors[0],1)
               
                self.multiTracker.add(( cv2.TrackerCSRT_create()), seg_frame, [100,100,200,200])
                self.multi_tracker_init_flag = True


            elif( self.pres_person_count == self.prev_person_count):
                for i in seperate_frames:
                    x_cor,y_cor,z_cor,_ = mp_test.mp_value(i)
                    if(x_cor is not None and y_cor is not None):
                        self.colors.append((randint(64, 255), randint(64, 255), randint(64, 255)))
                        cv2.circle(seg_frame, (int(x_cor),int(y_cor)),40,self.colors[0],1)
                print()
                
                
            if( self.multi_tracker_init_flag):
                success, boxes = self.multiTracker.update(seg_frame)

                for i, newbox in enumerate(boxes):
                    # print("**********************************Updating")
                    p1 = (int(newbox[0]), int(newbox[1]))
                    p2 = (int(newbox[0] + newbox[2]), int(newbox[1] + newbox[3]))
                    cv2.rectangle(seg_frame, p1, p2, self.draw.color_list[i], -1, 1)
                    cv2.putText(seg_frame, "Tag:" + str(i),(int(newbox[0]), int(newbox[1]-10)),0, 0.75, (255,255,255),2)


            # for bbox in mp_bboxes:
            #         self.multiTracker.add(( cv2.TrackerCSRT_create()), frame, bbox)

        # Post segmentation
        # self.update_mp_coord()
            cv2.imshow("Segmented frame",seg_frame)
    # def update_mp_coord(self):
    # def update(self,frame):
    #     #Tracker update for every frame with latest mp initalised bbox value
    #     success, boxes = self.multiTracker.update(frame)
    #     for i, newbox in enumerate(boxes):
    #               print("**********************************Updating")
    #               p1 = (int(newbox[0]), int(newbox[1]))
    #               p2 = (int(newbox[0] + newbox[2]), int(newbox[1] + newbox[3]))
    #               cv2.rectangle(frame, p1, p2, self.colors[i], -1, 1)
    #               cv2.putText(frame, "Tag:" + i,(int(newbox[0]), int(newbox[1]-10)),0, 0.75, (255,255,255),2)

    # def mp_calculations(self,x,y,z,frame):
    #     frame_width,frame_height = frame.shape()
    #     X = x*frame_width
    #     Y = y*frame_height
        
    #     return mp_bboxes

    def segment_canvas(self,frame,pres_bboxes):
        # print("segmenting canvas")
        # print(pres_bboxes)
        h, w, _ = frame.shape #video size as height and width
        seperate_canvas = np.ones((h,w,3), np.uint8)*255 #initializing white background of same resolution
        canvas = np.ones((h,w,3), np.uint8)*255 #initializing white background of same resolution
        segregated_frames = []
        for i in pres_bboxes: 
            seperate_canvas = np.ones((h,w,3), np.uint8)*255 #initializing white background of same resolution
            
            cropped = frame[i[1]:i[1]+i[3],i[0]:i[0]+i[2]]
            
            # seperated canvas - will hve a fresh canvas every iteration
            seperate_canvas[i[1]:i[1]+i[3],i[0]:i[0]+i[2]] = cropped
            # The segmented images of boundary are overlayed
            canvas[i[1]:i[1]+i[3],i[0]:i[0]+i[2]] = cropped

            segregated_frames.append(seperate_canvas)

        cv2.imshow("Segmented frame", canvas)
        return canvas,segregated_frames



        

        
# Person Class
class person_id():
    def __init__(self):
        self.id = 0 
        self.frame_cropped = None 
        self.tracker_obj = None

    def update(self,id,frame_cropped,tracker_obj):
        self.id = id 

