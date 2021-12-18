# import the opencv library
import cv2
import time
from draw_utils import draw_utils
from numpy.lib.function_base import median
import mp_test_pose
import numpy as np
from scipy.stats import t
import matplotlib.pyplot as plt
#from sklearn.svm import SVC
import short_dist
import mp_test
#import aura
draw = draw_utils()
flag =0
flag_collision = False
flag_copy =0
cnf_x=[]
aura_radius = 150
door_x =1200

center_buffer =[]
center_buffer_y =[]
first_x_buffer =[]
diff_buffer =[]

# define a video capture object
vid = cv2.VideoCapture('data/video/cropped.mp4')
tracker = cv2.TrackerCSRT_create()
tracker_copy = cv2.TrackerCSRT_create()
tracker_copy_2 = cv2.TrackerCSRT_create()
fig = plt.figure()
frame_debug = []
#ret, frame = vid.read()

def confidence_chart(x_plot,y_plot):
            ax = fig.add_subplot(111)
            #fig.show()
            x = np.array(x_plot)
            y = np.array(y_plot)
            x_sum = sum(x)
            y_sum = sum(y)
            cnf_score_x = (x_sum/(x_sum+y_sum))
            print("x_sum",cnf_score_x)
           
            #ax.plot(int(cnf_score),color='b',label="center")
            #fig.canvas.draw()
            #time.sleep(0.1)
def aura(bbox,frame):
    
    overlays = []
    counter = 0
    final_image = []
    

    first_frame = frame.copy()
    first_frame_v1 = frame.copy()
    # for i in range(len(bbox)):
   
    cv2.circle(first_frame, bbox,150,(0, 0, 255),-1,1)
    
    alpha = 0.5
    image_new = cv2.addWeighted(first_frame_v1, alpha, first_frame, 1 - alpha, 0)
    # cv2.imshow('aura', image_new)
    #cv2.waitKey(0)
    return frame
            
            

while(True):

    ret, frame = vid.read()
    frame_copy = frame.copy()
    frame_debug = frame
    #results = facepoints.mp_face_values(frame_copy)
    #print(results)
    try:
    #frame_copy = frame.copy()
        if flag ==0 :

        #cv2.waitKey(10)
        
            x,y,z,frame_debug= mp_test.mp_value(frame)
            print(x,y,z)
            
            X = x*1920
            Y = y*1080
             
            p1 = (int(X),int(Y))
            p2 = (int(X)+100,int(Y)+100)

            tracker_bbox = (int(X)-70,int(Y)-70,140,140) 
            door_x,door_y = (1200,500)

            cv2.circle(frame,p1,60,draw.RED,-1,1)
            #SJ 
            # cv2.rectangle(frame_debug,  (int(X)-70,int(Y)-70), (int(X)+70,int(Y)+70), (255,0,0), 2)
            #aura.aura(p1,frame)
            #cv2.rectangle(frame,p1,p2,(0,0,255),-1,1)
            ok = tracker.init(frame, tracker_bbox)
            # print("tracker_bbox",ok)          

            #           
            flag =1
    except:
        print("no features_detected")

    if flag ==1:
        ok, bbox = tracker.update(frame)
        # print(bbox)
        x_1,y,w,h = bbox
        z_1 = z+w
        
        t_p1 = (int(x_1),int(y))
        t_p2 = (int(x_1)+int(w),int(y)+int(h))
        cv2.rectangle(frame_debug,t_p1,t_p2,(0,0,255),-1,1)
        
        first_x_buffer.append(int(x_1))
        #confidence_chart(first_x_buffer[-20:])
        
        center_x1 = (x_1+(x_1+w))/2 
        center_y1 = (y+(y+h))/2 
        center = (int(center_x1)-20,int(center_y1)-20)
        frame = aura(center,frame)
        cv2.circle(frame,(center),60,draw.YELLOW,-1,1)

        cv2.putText(frame, "Tag:" + '0',(center),0, 0.75, (255,255,255),2)
        print("door_x",center_x1,door_x)
        '''
        if center_x1 > door_x:
            cv2.putText(frame, "Tag:0" + 'Exited',(150,100),0, 0.75, (255,255,255),2)
            flag =0
            reid_flag =0
        if center_x1 < door_x and flag == 1:
            flag =1 
            cv2.circle(frame,(center),60,(0,0,255),-1,1)
            cv2.putText(frame, "Tag:" + '0',(center),0, 0.75, (255,255,255),2)
         '''   
            

        frame_copy = frame
    if flag_copy ==0:
        try:
            x_c,y_c,z_2 = mp_test.mp_value(frame_copy)
            x_c = x_c*1920
            diff = x-x_c
            diff_buffer.append(diff)
            
            print(diff)          
            if -(int(diff)) > 1000:
                    diff_check = diff_buffer[-2]-diff
                    print("diff_check",diff_check)    
                    if diff_check <20:             
                        y_c = y_c*1080
                        tracker_box = (int(x_c),int(y_c),100,100)
                        # print(tracker_copy)
                        #frame_copy2 =frame_copy.copy() 
                        ok = tracker_copy.init(frame_copy, tracker_box)
                        
                        flag_copy =1
                        
                        print("flag_copy")
        except:
                print("no data")
    try:

        print(flag_copy)
        if flag_copy ==1:
                ok, bbox_copy = tracker_copy.update(frame_copy)
                #print(bbox_copy)
                x_2,y_2,w_2,h_2 = bbox_copy
                z_c = z_2 +w_2

                t2_p1 = (int(x_2),int(y_2))
                t2_p2 = (int(x_2)+int(w_2),int(y_2)+int(w_2))
                center_x1_copy = (x_2+(x_2+w_2))/2 
                center_y1_copy = (y_2+(y_2+h_2))/2 
                center_copy = (int(center_x1_copy)-20,int(center_y1_copy)-20)
                #print("center_copy",center_copy)
                #cv2.rectangle(frame_copy,t2_p1,t2_p2,(255,0,0),-1,1)
                cv2.circle(frame,(center_copy),60,draw.WHITE,-1,1)
                frame_copy = aura(center_copy,frame)
                cv2.putText(frame_copy, "Tag:" + '1',center_copy,0, 0.75, (0,0,255),2)
                center_diff = (center_x1_copy-center_x1)
                center_diff_y = (center_y1_copy -center_y1)
                #print("center_diff",center_diff)              
                center_buffer.append(int(center_diff))
                center_buffer_y.append(int(center_diff_y))
                p1_w = w/2
                p2_w = w_2/2
                min_value = p1_w - p2_w
                print("min_value",min_value)
                point2 = [center_x1_copy,center_y1_copy,"id2"]
                point1 = [center_x1,center_y1,"id1"]
                radius = 100+100
                closet_point = short_dist.closest_point(point1,point2)
                print("close",closet_point)
                v1,v2,v3 = closet_point
                #print("v3,radius",v3,radius)
                if v3 <radius :
                    flag_collision = True
                    cv2.putText(frame_copy,"collision True",(100,100),cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 0), 2)
                    flag_copy =2
        if flag_copy ==2 :
            xc_2,yc_2,zc_2 = mp_test.mp_value(frame_copy)
            Xc = xc_2*1920
            Yc = yc_2*1080
           

            print("latest tag values",Xc,Yc)
            cv2.putText(frame_copy,"Tag1",(int(Xc),int(Yc)),cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 0), 2)
            #
            if Xc-x_1 >aura_radius:
                ok = tracker_copy_2.init(frame_copy, tracker_box)
                flag_copy =3
        if flag_copy ==3:
            ok, bbox_copy = tracker_copy.update(frame_copy)
            #print(bbox_copy)
            x_c_2,y_c_2,w_c_2,h_c_2 = bbox_copy
            cv2.circle(frame_copy,(int(x_c_2),int(y_c_2)),60,(0,255,0),-1,1)
            cv2.putText(frame_copy,"Tag1",(int(x_c_2),int(y_c_2)),cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 0), 2)




                #confidence_chart(center_buffer[-20:],center_buffer_y[-20:])
    
                
    except:
        print("copy data not found")
    

	# Display the resulting frame
   
    cv2.imshow('frame', frame)
    cv2.imshow('Aura ',frame_copy)
    # if(frame_debug.all()):
    cv2.imshow('MediaPipe Holistic', frame_debug)

	
	# the 'q' button is set as the
	# quitting button you may use any
	# desired button of your choice
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# After the loop release the cap object
vid.release()
# Destroy all the windows
cv2.destroyAllWindows()
