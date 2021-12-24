import os 
import cv2
import time
import mp_test
import numpy as np
import short_dist
from kalmanfilter import KalmanFilter
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
        self.unique_id=[]
        self.tracker_bboxes=[]
        self.draw = draw_utils()
        # Variables 
        self.tracking_radius = 50
        # self.pres_tag =0
        self.kf_point = (0,0)
        # Counters 
        self.counter_master = 0
        #kalmanfilter properties(dt,acceleration_x,acceleration_y,std_acc,x_std_meas,y_std_meas)
        #self.KF = KalmanFilter(2, 1, 1, 1, 0.1, 0.1)
        self.KF = KalmanFilter(0.1, 1,1, 1, 1, 1)
        
        # FLAGS 
        self.multi_tracker_init_flag = False
        self.flag_yolo =False
        self.flag_mp = False
        self.flag_kalman = False
        self.flag_condition = -1
        #Tag variables 
        self.tag_id_index = 0
        self.tag_dict = {}

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
    def collision_detection(self,frame,point1,point2):
        closet_point = short_dist.closest_point(point1,point2)
        radius =self.tracking_radius+self.tracking_radius
        print("close",closet_point)
        v1,v2,v3 = closet_point
        #print("v3,radius",v3,radius)
        if v3 <radius :
            flag_collision = True
            cv2.putText(frame,"collision True",(100,100),cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 0), 2)
                


    def segment_initiate(self,frame,bboxes,count):

        # Clearing the flags 
        self.flag_yolo = False 
        self.flag_mp = False 
        self.flag_kalman = False
        self.flag_condition = -1
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
                self.flag_condition = 1
                #segmentation of frames to pass to mediapipe
                seg_frame,seperated_frame = self.segment_canvas(self.orig_frame,self.pres_bboxes)
                tracking_frame = seg_frame.copy()
                ###########################################################################################
                #condition to add bbox to multitracker everytime a new person enters
                if(self.pres_person_count > self.counter_master): 
                    print("Condition 1:New one entered, MC-"+str(self.counter_master)+"Pres-"+str(self.pres_person_count)+"Prev"+str(self.prev_person_count))
                    #creating a index number to get latest appended value
                    N = self.pres_person_count - self.counter_master 
                    # print("----------------------------------------------------------------",N)
                    # print("count_pres_prev",self.pres_person_count,self.prev_person_count)
                    for frame_index in seperated_frame:
                        #TODO Try block for each mp value and handle it accordingly in except and finally block 
                        try:
                            x_cor,y_cor,z_cor,_ = mp_test.mp_value(frame_index)                 
                            if (x_cor is not None and y_cor is not None):
                                # MP succesfully detected a person
                                # TODO : Check who is the new person 
                                self.flag_mp = True      
                                # cv2.circle(seg_frame,(x_cor,y_cor),self.tracking_radius,self.colors[self.tag_id_index],-1,1)
                                #creating a bbox with dummy width and height
                                bbox = [x_cor-50,y_cor-50,100,100]
                                #TODO : Resizing and offsetting the bboxes
                                self.tracker_bboxes.append([bbox,x_cor,y_cor])
                                # print("trackerbox_with index*****************************************************************",self.tracker_bboxes[-N:])
                            elif (x_cor is None and y_cor is None):
                                #Mediapipe failed 
                                print("Exception - MP Failed")
                                #TODO Add try locally and handle it with a finally block and else block 
                                self.flag_mp = True
                                raise exception_mp_failed
                        except exception_mp_failed: 
                            print("Exception MP Failed")

                        else: 
                            # Adding new tracker instance to the last 'N' new tracked BBoxes        
                            for bbox_index in self.tracker_bboxes[-N:]:
                                #self.aura((x_cor,y_cor),frame)
                                # Bbox_index - [bbox_index,x_cor,y_cor]
                                print("Multi Tracker - Add--------------------------------------------",bbox_index[0], N)
                                cv2.circle(seg_frame,(bbox_index[1],bbox_index[2]),self.tracking_radius,self.colors[0],-1,1)

                                self.multiTracker.add(( cv2.TrackerCSRT_create()), seg_frame, bbox_index[0])
                                #TODO add isolated frame to new object
                                self.tag_dict[self.tag_id_index]=person(self.tag_id_index,self.draw.color_list[self.tag_id_index],bbox_index[0])
                                print("Added a new object",self.tag_id_index)
                                #cv2.imshow("init_frame",seg_frame)

                                self.counter_master +=1
                                self.tag_id_index +=1
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
                ######################################################################################
                if self.pres_person_count == self.counter_master:
                        self.flag_condition = 2
                        print("Condition 2: TE, MC-"+str(self.counter_master)+"Pres-"+str(self.pres_person_count)+"Prev"+str(self.prev_person_count))
                        
                        
                        for i in seperated_frame:
                            
                            x_cor,y_cor,z_cor,_ = mp_test.mp_value(i)                 
                            if (x_cor is not None and y_cor is not None):
                                #TODO : if x & y of MP is None- Raise exceptions 
                                self.flag_mp = True      
                                cv2.circle(seg_frame,(x_cor,y_cor),self.tracking_radius,self.draw.color_list[0],-1,1)
                                #creating a bbox with dummy width and height
                                # print("trackerbox_with index*****************************************************************",self.tracker_bboxes[-N:])
                            elif (x_cor is None and y_cor is None):
                                #Mediapipe failed 
                                print("Exception - MP Failed")
                                #TODO Add try locally and handle it with a finally block and else block 
                                self.flag_mp = True
                                raise exception_mp_failed

                        # Append your coordinate data - circle 
                        success, boxes = self.multiTracker.update(seg_frame)
                        if( success == 0 ): 
                            print("Tracker: Updation not succesfull")
                        else:
                            for i, newbox in enumerate(boxes):
                                
                                p1 = (int(newbox[0]), int(newbox[1]))
                                p2 = (int(newbox[0]+newbox[2]), int(newbox[1]+newbox[3]))
                                # TODO Moving average 
                                # self.kf_point=p1
                                
                                cv2.rectangle(tracking_frame,p1,p2,self.colors[i],-1,1)
                                cv2.putText(tracking_frame, "Tag:" + str(i),(int(newbox[0]), int(newbox[1]-10)),0, 0.75, self.draw.RED,2)
                                #saving unique identity of tracking person
                                # self.unique_id.append(self.multiTracker.getObjects())
                                self.tag_dict[i].update(newbox)
                                #print(self.unique_id)
                                
                        

                if self.pres_person_count < self.counter_master : 
                    #YOLO might or might not have failed 
                    self.flag_condition = 4
                    print("Condition 3: Missing roi, MC-"+str(self.counter_master)+"Pres-"+str(self.pres_person_count)+"Prev"+str(self.prev_person_count))
                    # if self.pres_bboxes>1:
                    try:
                        if False:  # TODO Add Collision condition check 
                            self.collision(seg_frame,self.pres_bboxes[-1],self.pres_bboxes[-2])
                        else:
                            for i in seperated_frame:
                                x_cor,y_cor,z_cor,_ = mp_test.mp_value(i)
                                try:                  
                                    if (x_cor is not None and y_cor is not None):
                                        #TODO : if x & y of MP is None- Raise exceptions 
                                        self.flag_mp = True      
                                        cv2.circle(seg_frame,(x_cor,y_cor),self.tracking_radius,self.colors[0],-1,1)
                                    
                                    elif (x_cor is None and y_cor is None):
                                        #Mediapipe failed 
                                        print("Exception -MP") 
                                        self.flag_mp = True

                                        # raise exception_mp_failed

                                except exception_mp_failed:

                                    print("HANDLING MP FAILED EXCEPTION")    
                                    # TODO : Use kalman filter 
                                # finally: 
                                #     print("Handled--")
                        
                            self.flag_yolo =True            
                            raise exception_yolo_failed
                        
                        # success, boxes = self.multiTracker.update(seg_frame)
                        # print("bboxes",boxes)

                            # for i, newbox in enumerate(boxes):
                                
                            #     p1 = (int(newbox[0]), int(newbox[1]))
                            #     p2 = (int(newbox[0]+newbox[2]), int(newbox[1]+newbox[3]))
                                
                            #     cv2.rectangle(tracking_frame,p1,p2,self.colors[i],-1,1)
                            #     cv2.putText(tracking_frame, "Tag:" + str(i),(int(newbox[0]), int(newbox[1]-10)),0, 0.75, self.draw.RED,2)

                    except exception_yolo_failed:
                        print("Exception - Yolo Failed,Run MP")
                        subtracted_framed = self.subtract_bbox(self.orig_frame,self.pres_bboxes)
                        # TODO : For multiple bboxes missed in yolo 
                        x_cor,y_cor,z_cor,_ = mp_test.mp_value(subtracted_framed,True)                 
                        if (x_cor is not None and y_cor is not None):
                            #TODO : if x & y of MP is None- Raise exceptions 
                            self.flag_mp = True      
                            cv2.circle(seg_frame,(x_cor,y_cor),self.tracking_radius,self.colors[0],-1,1)
                            
                        if (x_cor is None and y_cor is None):
                            #Mediapipe failed 
                            pre_x,pre_y = self.KF.predict()
                            pre_x = pre_x.tolist()
                            print("predicted_kalman-----------------------",pre_x,pre_y)
                            # #for i in range(3):
                            upd_x = self.KF.update(self.kf_point)
                            upd_x = upd_x.tolist()
                            try:
                                cv2.circle(seg_frame,(int(pre_x[0][0]),int(pre_x[0][1])),10,self.draw.YELLOW,-1,1)
                          
                                cv2.circle(seg_frame,(int(upd_x[0][0]),int(upd_x[0][1])),10,self.draw.RED,-1,1)
                                cv2.circle(seg_frame,(int(upd_x[0][0]),int(upd_x[0][1])),self.tracking_radius,self.draw.color_list[0],-1,1)
                                
                            except:
                                print("Kalman not predicting")
                                self.flag_kalman= True
                            print("EXCEPTION MP FAILED as well as YOLO failed")                            
                            # raise exception_mp_failed 

                    success, boxes = self.multiTracker.update(seg_frame)
                    if(success == 0 ): 
                        print("Tracker - Update - Not successfull")
                    else: 
                        for i, newbox in enumerate(boxes):        
                            p1 = (int(newbox[0]), int(newbox[1]))
                            p2 = (int(newbox[0]+newbox[2]), int(newbox[1]+newbox[3]))
                            self.tag_dict[i].update(newbox)
                            cv2.rectangle(tracking_frame,p1,p2,self.colors[i],-1,1)
                            cv2.putText(tracking_frame, "Tag:" + str(i),(int(newbox[0]), int(newbox[1]-10)),0, 0.75, self.draw.RED,2) 




                cv2.moveWindow("Seg Frame 3",960,0)
                cv2.moveWindow("Tracker Frame 4",0,540)

                cv2.imshow("Seg Frame 3",cv2.resize(seg_frame,(960,540)))
                cv2.imshow("Tracker Frame 4 ",cv2.resize(tracking_frame,(960,540)))
            print("Logs--Cond "+str(self.flag_condition)+"Yolo-"+str(self.flag_yolo)+"MP-"+str(self.flag_mp)+"Kalman-"+str(self.flag_kalman))
            self.prev_bboxes = self.pres_bboxes
                        
                                            
        # except:
        #     print("yolo_status::"+str(self.flag_yolo)+", mp_status::"+str(self.flag_mp))
 
        

        except exception_mp_failed:
            if self.pres_person_count>=1:
                self.multiTracker.update(seg_frame)    
                # "Updating Multitracker"
            print("MP Failed exception handled")


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

    def subtract_bbox(self,frame, pres_yolo_bboxes):
        print("sub")
        h, w, _ = frame.shape #video size as height and width
        for i in pres_yolo_bboxes: 
            white_box = np.ones((i[3],i[2],3),np.uint8)*255
            frame[i[1]:i[1]+i[3],i[0]:i[0]+i[2]]= white_box
        # cv2.imshow("sub_frame",frame)
        return frame

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
    
    def update(self,track_bbox, yolo_bbox=None):
        # self.track_bbox = track_bbox
        self.track_bbox.append(track_bbox)
        # self.yolo_bbox = yolo_bbox