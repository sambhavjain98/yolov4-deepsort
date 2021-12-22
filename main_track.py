import os 
import cv2
import mp_test
import numpy as np
import short_dist
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
        self.tracker_bboxes=[]
        self.draw = draw_utils()
        # Variables 
        self.tracking_radius = 50
        # Counters 
        self.counter_master = 0
        # FLAGS 
        self.multi_tracker_init_flag = False
        self.flag_yolo =False
        self.flag_mp = False

    def aura(self,bbox,frame):
    
        overlays = []
        counter = 0
        final_image = []
        
        first_frame = frame.copy()
        first_frame_v1 = frame.copy()
        # for i in range(len(bbox)):
        print("aura---------------------------------------------------------",bbox)
        cv2.circle(first_frame, bbox,150,(0, 0, 255),-1,1)
        
        alpha = 0.5
        image_new = cv2.addWeighted(first_frame_v1, alpha, first_frame, 1 - alpha, 0)
        cv2.imshow('aura', image_new)
        #cv2.waitKey(0)
        return image_new


    def segment_initiate(self,frame,bboxes,count):
        bboxes_temp = []
        for i in bboxes:
            # print([int(j) for j in i])
            bboxes_temp.append([int(j) for j in i])
        self.colors.append((randint(64, 255), randint(64, 255), randint(64, 255)))
        # print("Person tracking ")
        # print(bboxes,count)
        # Bboxes updating
        # if bboxes is not None:

        
        self.tracker_bboxes = []
        bboxes_temp = []
        for i in bboxes:
            # print([int(j) for j in i])
            bboxes_temp.append([int(j) for j in i])

        self.prev_bboxes = self.pres_bboxes
        self.pres_bboxes = bboxes_temp
        # Frame is resized and colored 
        self.orig_frame = frame 
        try:
            #check for first person
            self.pres_person_count = count
            if self.pres_person_count>=1:
                self.flag_yolo = True
                #segmentation of frames to pass to mediapipe
                seg_frame,seperated_frame = self.segment_canvas(self.orig_frame,self.pres_bboxes)
                tracking_frame = seg_frame.copy()
                #condition to add bbox to multitracker everytime a new person enters
                if(self.pres_person_count > self.prev_person_count): 
                    print("Condition 1:New one entered, MC-"+str(self.counter_master)+"Pres-"+str(self.pres_person_count)+"Prev"+str(self.prev_person_count))
                    #creating a index number to get latest appended value
                    N = self.pres_person_count -self.prev_person_count
                    # print("----------------------------------------------------------------",N)
                    # print("count_pres_prev",self.pres_person_count,self.prev_person_count)
                    for i in seperated_frame:
                   
                        x_cor,y_cor,z_cor,_ = mp_test.mp_value(i)                 
                        if (x_cor is not None and y_cor is not None):
                            self.flag_mp = True      
                            cv2.circle(seg_frame,(x_cor,y_cor),self.tracking_radius,self.colors[count],-1,1)
                            #creating a bbox with dummy width and height
                            bbox = [x_cor-50,y_cor-50,100,100]
                            #TODO : Resizing and offsetting the bboxes
                            self.tracker_bboxes.append(bbox)
                            # print("trackerbox_with index*****************************************************************",self.tracker_bboxes[-N:])
                        elif (x_cor is None and y_cor is None):
                            #Mediapipe failed 
                            # print("EXCEPTION MP FAILED")
                            raise exception_mp_failed
                    # Adding new tracker instance to the last 'N' new tracked BBoxes        
                    for j in self.tracker_bboxes[-N:]:
                        #self.aura((x_cor,y_cor),frame)
                        print("j-Add--------------------------------------------",j, N)
                        self.multiTracker.add(( cv2.TrackerCSRT_create()), seg_frame, j)
                        self.counter_master +=1
                        #TODO Add a person object to the tracked ID 
                        

                    # Updating the remaining tracker instances 
                    # for j in self.tracker_bboxes[:(len(self.tracker_bboxes)-N)]:
                    #     #self.aura((x_cor,y_cor),frame)
                    #     print("j-Update-------------------------------------------",j, N)
                    # success, boxes = self.multiTracker.update(seg_frame)

                    # Count updation 
                    self.prev_person_count = self.pres_person_count
                    # cv2.imshow("Tracker_init",seg_frame)
                #condition to update tracker all the time
                if self.pres_person_count == self.prev_person_count:
                        print("Condition 2: TE, MC-"+str(self.counter_master)+"Pres-"+str(self.pres_person_count)+"Prev"+str(self.prev_person_count))
                        
                        
                        for i in seperated_frame:
                            
                            x_cor,y_cor,z_cor,_ = mp_test.mp_value(i)                 
                            if (x_cor is not None and y_cor is not None):
                                #TODO : if x & y of MP is None- Raise exceptions 
                                self.flag_mp = True      
                                cv2.circle(seg_frame,(x_cor,y_cor),self.tracking_radius,self.colors[count],-1,1)
                                #creating a bbox with dummy width and height

                                # print("trackerbox_with index*****************************************************************",self.tracker_bboxes[-N:])
                            elif (x_cor is None and y_cor is None):
                                #Mediapipe failed 
                                # print("EXCEPTION MP FAILED")
                                raise exception_mp_failed


                        success, boxes = self.multiTracker.update(seg_frame)

                        for i, newbox in enumerate(boxes):
                            
                            p1 = (int(newbox[0]), int(newbox[1]))
                            p2 = (int(newbox[0]+newbox[2]), int(newbox[1]+newbox[3]))
                            
                            cv2.rectangle(tracking_frame,p1,p2,self.colors[i],-1,1)
                            cv2.putText(tracking_frame, "Tag:" + str(i),(int(newbox[0]), int(newbox[1]-10)),0, 0.75, self.draw.RED,2)
                
                            

                if self.pres_person_count < self.prev_person_count : 
                    #YOLO might or might not have failed 
                    print("Condition 3: Missing roi, MC-"+str(self.counter_master)+"Pres-"+str(self.pres_person_count)+"Prev"+str(self.prev_person_count))

                    for i in seperated_frame:
                            x_cor,y_cor,z_cor,_ = mp_test.mp_value(i)                 
                            if (x_cor is not None and y_cor is not None):
                                #TODO : if x & y of MP is None- Raise exceptions 
                                self.flag_mp = True      
                                cv2.circle(seg_frame,(x_cor,y_cor),20,self.colors[count],-1,1)
                                #creating a bbox with dummy width and height

                                # print("trackerbox_with index*****************************************************************",self.tracker_bboxes[-N:])
                            elif (x_cor is None and y_cor is None):
                                #Mediapipe failed 
                                # print("EXCEPTION MP FAILED")                            
                                raise exception_mp_failed
                    
                    subtracted_framed = self.subtract_bbox(self.orig_frame,self.pres_bboxes)                       
                    raise exception_yolo_failed

                cv2.imshow("Seg Frame",seg_frame)
                cv2.imshow("Tracker Frame",tracking_frame)

                        # closet_point = short_dist.closest_point(point1,point2)
                        # radius = p1_r,p2_r
                        # print("close",closet_point)
                        # v1,v2,v3 = closet_point
                        # #print("v3,radius",v3,radius)
                        # if v3 <radius :
                        #     flag_collision = True
                        #     cv2.putText(frame_copy,"collision True",(100,100),cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 0), 2)
                                            
        # except:
        #     print("yolo_status::"+str(self.flag_yolo)+", mp_status::"+str(self.flag_mp))

        except exception_yolo_failed:
            print("YOLO Failed, Run mediapipe")
        
        except exception_mp_failed:
            if self.pres_person_count>=1:
                self.multiTracker.update(seg_frame)    
                # "Updating Multitracker"
            print("MP Failed")


    def segment_canvas(self,frame,pres_bboxes):
        # print("segmenting canvas")
        # print(pres_bboxes)
        h, w, _ = frame.shape #video size as height and width
        seperate_canvas = np.ones((h,w,3), np.uint8)*255 #initializing white background of same resolution
        canvas = np.ones((h,w,3), np.uint8)*255 #initializing white background of same resolution
        segregated_frames = []
        #for j in range(len(pres_bboxes)):
        for i in pres_bboxes: 
            seperate_canvas = np.ones((h,w,3), np.uint8)*255 #initializing white background of same resolution
            
            cropped = frame[i[1]:i[1]+i[3],i[0]:i[0]+i[2]]
            
            # seperated canvas - will hve a fresh canvas every iteration
            seperate_canvas[i[1]:i[1]+i[3],i[0]:i[0]+i[2]] = cropped
            # The segmented images of boundary are overlayed
            canvas[i[1]:i[1]+i[3],i[0]:i[0]+i[2]] = cropped
            segregated_frames.append(seperate_canvas)


        #cv2.imshow("canvas", segregated_frames)
        
       
        return canvas,segregated_frames

    def subtract_bbox(frame, pres_yolo_bboxes):
        print("sub")

# define Python user-defined exceptions
class Error(Exception):
    """Base class for other exceptions"""
    pass

class exception_mp_failed(Error):
    pass

class exception_yolo_failed(Error):
    pass


# Every track id created will have a object of person associated with it 
class person():
    def __init__(self,id, color,track_bbox = None, yolo_bbox= None, isolated_frame = None):
        self.id = id
        self.color = color
        # TODO Present and Past bboxes to be stored 
        self.sep_frame = isolated_frame 
        self.track_bbox = track_bbox
        self.yolo_bbox = yolo_bbox
    
    def update(self,track_bbox, yolo_bbox):
        self.track_bbox = track_bbox
        self.yolo_bbox = yolo_bbox