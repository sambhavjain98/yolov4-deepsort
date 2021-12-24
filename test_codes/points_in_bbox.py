

class person():
    def __init__(self,id, color,track_bbox = None, yolo_bbox= None, isolated_frame = None):
        self.id = id
        self.color = color
        # TODO Present and Past bboxes to be stored 
        self.sep_frame = isolated_frame 
        self.track_bbox = []
        self.yolo_bbox = []
        self.track_bbox.append(track_bbox)
        self.yolo_bbox.append(yolo_bbox)
    
    def update(self,track_bbox, yolo_bbox=None):
        # self.track_bbox = track_bbox
        self.track_bbox.append(track_bbox)
        # self.yolo_bbox = yolo_bbox

def missed_multi_tracker_tag(bboxes,tag_dict): 
        tag_dict_copy = tag_dict.copy()
        for tag_index in tag_dict:
            bbox_flag = False

            print("Loop Number-------------",tag_index)
            track_bbox = tag_dict_copy[tag_index].track_bbox[-1]
            # print(track_bbox[-1])
            tag_point = (track_bbox[0]+50,track_bbox[1]+50)

            for bbox_index,bbox in enumerate(bboxes):
                success = pointInRect(tag_point,bbox)
                if (success): 
                    bbox_flag = True
                    print("Bounding box",bbox_index)
                    print("tag_index",tag_index)
                    bboxes.remove(bbox)
                    break
                else : 
                    print("No")

            if(bbox_flag):
                print("Removed",tag_dict_copy.pop(tag_index),tag_index,bboxes)
                print("Not removed",tag_dict)

        return tag_dict_copy

def pointInRect(point,rect):
    x1, y1, w, h = rect
    x2, y2 = x1+w, y1+h
    x, y = point
    if (x1 < x and x < x2):
        if (y1 < y and y < y2):
            print("true",point,rect)
            
            return True
    print("False",point,rect)
    return False


tag_dict = {}
tag_dict[0]= person(0,0,[100,100,100,100])
tag_dict[1]= person(0,0,[400,400,100,100])
tag_dict[2]= person(0,0,[700,700,100,100])


tag_dict[0].update([100,100,100,100])
tag_dict[1].update([400,400,100,100])
tag_dict[2].update([700,700,100,100])

bboxes = [[680,680,130,130],[80,80,130,130]]

print(missed_multi_tracker_tag(bboxes,tag_dict))
# pointInR((150,150),[80,80,130,130])



