import rospy
import numpy as np
import cv2 as cv
import pyzbar.pyzbar as pyz
import time
from Intro_Robotics_Final_Project.msg import QR, EdgeList

#This class deals with processing images from the Bebop drones
class CVProcessor:
    #List of line color ranges
    #NOTE: Will need to add to this
    # line_colors = {'orange':[np.array([5,100,150]), np.array([15,255,255])],'purple':[np.array([275,100,150]), np.array([285,255,255])]}

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
        output_data = {"qr" : { "hasQR" : None, "centroid" : None, "value" : None}, "edges" : []}
        value, bb = self.detect_QR_code(img)

        if(value != None):
             output_data["qr"]["centroid"] = bb
             output_data["qr"]["hasQR"] = True
             output_data["qr"]["value"] = value
        else:
            output_data["qr"]["hasQR"] = False
            output_data["qr"]["centroid"] = (0,0)
            output_data["qr"]["value"] = 0

        colors, masks = self.color_filter(img)
        for i in range(len(masks)):
            angle = self.line_angle(masks[i])
            if angle != None:
                edge = {"color":colors[i], "angle":angle, "centroid":None}
                output_data["edges"].append(edge)

        return output_data

    #Takes an image and color filters for all
    #potential line colors to get line blobs
    #DONE: Creates mask for specfied color or a mask for each color if non-specific
    #TODO: Add more colors to line_colors
    def color_filter(self, image, color_type=None):
        masks = []
        colors = []
        hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)

        if color_type == None:
            for color, range in G.LINE_COLORS.items():
                mask = cv.inRange(hsv, range[0], range[1])
                masks.append(mask)
                colors.append(color)
        else:
            masks.append(cv.inRange(hsv, G.LINE_COLORS[color][0], G.LINE_COLORS[color][1]))
            colors.append(color_type)

        return colors, masks

    #Takes an image that is the isolated line blob
    #and returns the angle to vertical
    #DONE: Calculates avg x and y components and then computes angle
    #TODO: Test and tweak parameters
    def line_angle(self, line_mask):
        edges = cv.Canny(line_mask,50,150,apertureSize = 3)
        lines = cv.HoughLinesP(edges,1,np.pi/180,10)

        #Now to calculate angle from lines
        if lines is not None:
            if len(lines) > 0:
                sum_opp = 0
                sum_adj = 0
                num_lines = 0
                for line in lines:
                    for x1,y1,x2,y2 in line:
                        length = np.sqrt((x1-x2)**2+(y1-y2)**2)
                        if length > 5:
                            adj = np.abs(y1-y2)
                            opp = np.abs(x1-x2)
                            if adj > 0:
                                num_lines += 1
                                sum_opp += opp
                                sum_adj += adj

                num = (float(sum_opp/num_lines))/float((sum_adj/num_lines))
                angle = np.arctan(num)

                return np.degrees(angle)

        return None

    #Detect a QR code and determine centroid
    #DONE: Detect and calculate centroid if it exists
    def detect_QR_code(self, image):
        print('In QR detection: ', type(image))
        code = pyz.decode(image)
        if len(code) != 0:
            bb = code[0][2]
            x = (bb[0] + bb[2])/2
            y = (bb[1] + bb[3])/2
            value = code[0].data.decode('utf-8')

            return value, (x,y)

        return None, None
