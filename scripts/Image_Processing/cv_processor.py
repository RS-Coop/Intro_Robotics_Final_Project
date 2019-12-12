import rospy
import numpy as np
import cv2 as cv
import pyzbar.pyzbar as pyz
import time
#This class deals with processing images from the Bebop drones
class CVProcessor:
    #List of line color ranges
    line_colors = {'orange':[[5,100,150], [15,255,255]],
                        'purple':[[275,100,150], [285,255,255]]}

    def __init__(self):
        pass

################################################################################
    # Take in an image and return a dictionary with the information found in the image
    # Should return:
    '''
    {
        "qr" : 
            {
                "hasQR" : bool, 
                "centroid" : (int, int) # (x, y)
            },
        "edges" :
            [
                {
                    "color" : String, # (orange, purple)
                    "angle" : float,
                    "centroid" : (int, int) # (x, y)
                },
                ...
            ]
    }
    '''
    def process_image(self, img):
        # Call process image
        # Publish results to topics
        output_data = {"qr" : { "hasQR" : None, "centroid" : None}, "edges" : []}
        qrResult = self.detect_QR_code(img)

        if(qrResult != None):
             output_data["qr"]["centroid"] = qrResult
             output_data["qr"]["hasQR"] = True
        else:
            output_data["qr"]["hasQR"] = False
      
        lineResult = []

        return output_data
    #Takes an image and color filters for all
    #potential line colors to get line blobs
    #DONE: Creates mask for specfied color or a mask for each color if non-specific
    #TODO: Add more colors to line_colors
    def color_filter(self, image, color=None):
        masks = []
        hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)
        #Right now this is just orange, I think it will need to be a loop
        if color == None:
            for i in range(0, len(self.line_colors)):
                mask = np.zeros((image.shape[0], image.shape[1],3),np.uint8)

                color = self.line_colors[i]
                color_mask = cv.inRange(hsv, color[0], color[1])
                mask = cv.bitwise_and(mask, color_mask)

                masks.append(mask)
        else:
            masks.append(cv.inRange(hsv, self.line_colors[color][0], self.line_colors[color][1]))

        return masks

    #Takes an image that is the isolated line blob
    #and returns the angle to vertical
    #DONE: Calculates avg x and y components and then computes angle
    #TODO: Test and tweak parameters
    def line_angle(self, line_mask):
        edges = cv.Canny(line_mask,50,150,apertureSize = 3)
        lines = cv.HoughLinesP(edges,1,np.pi/180,10)

        #Now to calculate angle from lines
        sum_opp = 0
        sum_adj = 0
        num_lines = 0
        for line in lines:
            for x1,y1,x2,y2 in line:
                len = np.sqrt((x1-x2)**2+(y1-y2)**2)
                if len > 5:
                    adj = np.abs(y1-y2)
                    opp = np.abs(x1-x2)
                    if adj > 0:
                        num_lines += 1
                        sum_opp += opp
                        sum_adj += adj

        num = (float(sum_opp/num_lines))/float((sum_adj/num_lines))
        angle = np.arctan(num)

        return np.degrees(angle)

    #Detect a QR code and determine centroid
    #DONE: Detect and calculate centroid if it exists
    def detect_QR_code(self, image):
        code = pyz.decode(image)
        # print(code)
        # print(code[0])
        # print(code[0][2])
        # print(code[0][2][0])
        if len(code) != 0:
            bb = code[0][2]
            x = (bb[0] + bb[2])/2
            y = (bb[1] + bb[3])/2

            return (x,y)

        return None

    #Takes a QR code image and proccess it.
    #DONE: Returns string of QR code data
    def process_QR_code(self, image):
        code = pyz.decode(image)

        return code[0].data.decode('utf-8')