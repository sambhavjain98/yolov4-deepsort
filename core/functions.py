import os
import cv2
import random
import numpy as np
import tensorflow as tf
# import pytesseract
from core.utils import read_class_names
from core.config import cfg

# function to count objects, can return total classes or count per class
def count_objects(data, by_class = False, allowed_classes = list(read_class_names(cfg.YOLO.CLASSES).values())):
    boxes, scores, classes, num_objects = data

    #create dictionary to hold count of objects
    counts = dict()

    # if by_class = True then count objects per class
    if by_class:
        class_names = read_class_names(cfg.YOLO.CLASSES)

        # loop through total number of objects found
        for i in range(num_objects):
            # grab class index and convert into corresponding class name
            class_index = int(classes[i])
            class_name = class_names[class_index]
            if class_name in allowed_classes:
                counts[class_name] = counts.get(class_name, 0) + 1
            else:
                continue

    # else count total objects found
    else:
        counts['total object'] = num_objects
    
    return counts

# function for cropping each detection and saving as new image
def crop_objects(img, data, path, allowed_classes,boxes_dim):
    boxes, scores, classes, num_objects = data
    class_names = read_class_names(cfg.YOLO.CLASSES)
    #create dictionary to hold count of objects for image name
    counts = dict()
    for i in range(num_objects):
        # get count of class for part of image name
        class_index = int(classes[i])
        class_name = class_names[classes[i]]
        print("Crop objets fun -")
        print(class_name)
        if class_name in allowed_classes:
            counts[class_name] = counts.get(class_name, 0) + 1
            bbox_t = boxes_dim[i]
            xmin, ymin, xmax, ymax = boxes_dim[i]
            # crop detection from image (take an additional 5 pixels around all edges)
            # cropped_img = img[int(ymin)-5:int(ymax)+5, int(xmin)-5:int(xmax)+5]
            # construct image name and join it to path for saving crop properly
            img_name = class_name + '_' + str(counts[class_name]) + '.jpg'
            img_path = os.path.join(path, img_name )
            # cropped_img = img
            # cv2.rectangle(cropped_img, (int(bbox_t[0]), int(bbox_t[1]) ), (int(bbox_t[2]), int(bbox_t[3])),(255,255,0), 2)
            # cv2.circle(cropped_img, (int(bbox_t[0]),int(bbox_t[1])), radius=5, color=(255, 0, 255), thickness=-1)
            # cropped_img = img[( int(bbox_t[1]) : int(bbox_t[3]) ),( int(bbox_t[0]):int(bbox_t[2]))]
            cropped_img = img[int(bbox_t[1])-5:int(bbox_t[3])+5, int(bbox_t[0])-5:int(bbox_t[2])+5]

            # save image
            print("Image path" + img_path)
            retval = cv2.imwrite(img_path, cropped_img)
            cv2.imshow('frame2',cropped_img)
            # print("THe image saved is"+ str(retval))
            cv2.imwrite(img_path, cropped_img)
            
        else:
            continue
