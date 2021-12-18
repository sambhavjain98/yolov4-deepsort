import cv2
from statistics import mean
import statistics
import mediapipe as mp
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_holistic = mp.solutions.holistic

# For static images:
# For webcam input:
def mp_value(image):
    face_x =[]
    face_y =[]  
    face_z=[]
    with mp_holistic.Holistic(
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5) as holistic:

        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        results = holistic.process(image)

        # Draw landmark annotation on the image.
        image.flags.writeable = True
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        mp_drawing.draw_landmarks(
            image,
            results.face_landmarks,
            mp_holistic.FACEMESH_CONTOURS,
            landmark_drawing_spec=None,
            connection_drawing_spec=mp_drawing_styles
            .get_default_face_mesh_contours_style())
        mp_drawing.draw_landmarks(
            image,
            results.pose_landmarks,
            mp_holistic.POSE_CONNECTIONS,
            landmark_drawing_spec=mp_drawing_styles
            .get_default_pose_landmarks_style())
        # Flip the image horizontally for a selfie-view display.
        # cv2.imshow('MediaPipe Holistic', cv2.flip(image, 1))
        
        
                        
        try:
            for data_point in results.face_landmarks.landmark:
                # print(data_point.x,data_point.y,data_point.z)
                face_x.append(data_point.x)
                face_y.append(data_point.y) 
                face_z.append(data_point.z)     
            x_face = mean(face_x)
            y_face = mean(face_y)
            z_face = mean(face_z)
            
           
                    
        except:
            pass
        
        return x_face,y_face,z_face,image
     