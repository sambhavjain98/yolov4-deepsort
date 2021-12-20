import cv2 
class draw_utils():
    def __init__(self):
        self.draw_enabled = False
        self.win_name = ""
        self.BLACK      = (0,0,0)
        self.WHITE      = (255,255,255)
        self.EGGSHELL   = (16,78,139)
        self.PINK       = (255,192,203)
        self.BLUE       = (255,0,0)
        self.GREEN      = (0,255,0)
        self.RED        = (0,0,255)
        self.YELLOW     = (0, 255, 255)
        self.color_list = [(0,0,0),(255,255,255),(16,78,139),(255,192,203),(255,0,0)]

    def pointer(self,image,x,y,color = (0,0,255)):
        # x, y = self.pointer
        cv2.circle(image, (x, y), 1, color, -1)
    
    # def line(self,image,p1,p2,color = (0,0,255)):
    #     cv2.line(image,pt1,p2,color)
    