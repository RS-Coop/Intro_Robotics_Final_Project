import rospy
import numpy as np
import cv2 as cv
import pyzbar.pyzbar as pyz
import time
import math
from Intro_Robotics_Final_Project.msg import QR, EdgeList
from Globals import Globals as G
from cv_bridge import CvBridge

#This class deals with processing images from the Bebop drones
class CVProcessor:
    #List of line color ranges
    #NOTE: Will need to add to this
    # line_colors = {'orange':[np.array([5,100,150]), np.array([15,255,255])],'purple':[np.array([275,100,150]), np.array([285,255,255])]}
    bridge = CvBridge()
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
                "centroid" : (int, int) # (x, y),
                "value: : int
            },
        "edges" :
            [
                {
                    "color" : String, # (orange, purple)
                    "pos_avgs" : {
                                    O_TOP :
                                    O_BOTTOM:
                                    O_LEFT :
                                    O_RIGHT:
                                    I_TOP :
                                    I_BOTTOM:
                                    I_LEFT :
                                    I_RIGHT :
                                  },
                },
                ...
            ]
    }
    '''
    def process_image(self, image):
        # Call process image
        # Publish results to topics
        output_data = {"qr" : { "hasQR" : None, "centroid" : None, "value" : None}, "edges" : []}
        value, bb = self.detect_QR_code(image)

        if(value != None):
            output_data["qr"]["hasQR"] = True
            output_data["qr"]["centroid"] = bb
            output_data["qr"]["value"] = value
        else:
            output_data["qr"]["hasQR"] = False
            output_data["qr"]["centroid"] = (0,0)
            output_data["qr"]["value"] = 0

        colors, masks = self.color_filter(image)

        for i in range(len(masks)):
            band_avgs = self.get_band_averages(masks[i])

            if band_avgs != None:
                edge = {"color":colors[i], "pos_avgs":band_avgs}
                output_data["edges"].append(edge)

        return output_data

    def process_ros_image(self, ros_img):
        img = self.bridge.imgmsg_to_cv2(ros_img, desired_encoding="bgr8")
        output_data = self.process_image(img)
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

    #Takes a mask and gets the band averages
    #TODO:
    #NOTE: Returns a dictionary of avgs
    def get_band_averages(self, mask):
        band_avgs = {}
        height = mask.shape[0]
        width = mask.shape[1]

        sep_h = (height/3)/3
        sep_w = (width/3)/3

        sum_y = 0
        sum_x = 0
        num_pixels = 0

        for row in range(G.BAND_SIZE):
            for col in range(width):
                if mask[row, col] == 255:
                    sum_x += col
                    sum_y += row
                    num_pixels += 1

        if num_pixels > G.BAND_THRESHOLD:
            band_avgs.update({G.O_TOP:(int(sum_x/num_pixels), int(sum_y/num_pixels))})
        else:
            band_avgs.update({G.O_TOP:(None,None)})
    ###############################################################################
        sum_y = 0
        sum_x = 0
        num_pixels = 0

        for row in range(height-G.BAND_SIZE, height):
            for col in range(width):
                if mask[row, col] == 255:
                    sum_x += col
                    sum_y += row
                    num_pixels += 1

        if num_pixels > G.BAND_THRESHOLD:
            band_avgs.update({G.O_BOTTOM:(int(sum_x/num_pixels), int(sum_y/num_pixels))})
        else:
            band_avgs.update({G.O_BOTTOM:(None,None)})
    ###############################################################################
        sum_y = 0
        sum_x = 0
        num_pixels = 0

        for row in range(height):
            for col in range(G.BAND_SIZE):
                if mask[row, col] == 255:
                    sum_x += col
                    sum_y += row
                    num_pixels += 1

        if num_pixels > G.BAND_THRESHOLD:
            band_avgs.update({G.O_LEFT:(int(sum_x/num_pixels), int(sum_y/num_pixels))})
        else:
            band_avgs.update({G.O_LEFT:(None,None)})
    ###############################################################################
        sum_y = 0
        sum_x = 0
        num_pixels = 0

        for row in range(height):
            for col in range(width-G.BAND_SIZE, width):
                if mask[row, col] == 255:
                    sum_x += col
                    sum_y += row
                    num_pixels += 1

        if num_pixels > G.BAND_THRESHOLD:
            band_avgs.update({G.O_RIGHT:(int(sum_x/num_pixels), int(sum_y/num_pixels))})
        else:
            band_avgs.update({G.O_RIGHT:(None,None)})
    ###############################################################################
        sum_y = 0
        sum_x = 0
        num_pixels = 0

        for row in range(sep_h+G.BAND_SIZE, sep_h+2*G.BAND_SIZE):
            for col in range(width):
                if mask[row, col] == 255:
                    sum_x += col
                    sum_y += row
                    num_pixels += 1

        if num_pixels > G.BAND_THRESHOLD:
            band_avgs.update({G.I_TOP:(int(sum_x/num_pixels), int(sum_y/num_pixels))})
        else:
            band_avgs.update({G.I_TOP:(None,None)})
    ###############################################################################
        sum_y = 0
        sum_x = 0
        num_pixels = 0

        for row in range(height-2*G.BAND_SIZE-sep_h, height-G.BAND_SIZE-sep_h):
            for col in range(width):
                if mask[row, col] == 255:
                    sum_x += col
                    sum_y += row
                    num_pixels += 1

        if num_pixels > G.BAND_THRESHOLD:
            band_avgs.update({G.I_BOTTOM:(int(sum_x/num_pixels), int(sum_y/num_pixels))})
        else:
            band_avgs.update({G.I_BOTTOM:(None,None)})
    ###############################################################################
        sum_y = 0
        sum_x = 0
        num_pixels = 0

        for row in range(height):
            for col in range(sep_w+G.BAND_SIZE, sep_w+2*G.BAND_SIZE):
                if mask[row, col] == 255:
                    sum_x += col
                    sum_y += row
                    num_pixels += 1

        if num_pixels > G.BAND_THRESHOLD:
            band_avgs.update({G.I_LEFT:(int(sum_x/num_pixels), int(sum_y/num_pixels))})
        else:
            band_avgs.update({G.I_LEFT:(None,None)})
    ###############################################################################
        sum_y = 0
        sum_x = 0
        num_pixels = 0

        for row in range(height):
            for col in range(width-2*G.BAND_SIZE-sep_w, width-G.BAND_SIZE-sep_w):
                if mask[row, col] == 255:
                    sum_x += col
                    sum_y += row
                    num_pixels += 1

        if num_pixels > G.BAND_THRESHOLD:
            band_avgs.update({G.I_RIGHT:(int(sum_x/num_pixels), int(sum_y/num_pixels))})
        else:
            band_avgs.update({G.I_RIGHT:(None,None)})
    ###############################################################################
        for band in band_avgs.values():
            if band != (None,None):
                return band_avgs

        return None

    #Detect a QR code and determine centroid
    #DONE: Detect and calculate centroid if it exists
    def detect_QR_code(self, image):
        code = pyz.decode(image)
        if len(code) != 0:
            bb = code[0].rect
            value = int(code[0].data.decode('utf-8'))
            x = bb[0] + int(bb[2]/2)
            y = bb[1] + int(bb[3]/2)

            return value, (x, y)
        else:
            hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)
            mask = cv.inRange(hsv, G.LINE_COLORS["white"][0], G.LINE_COLORS["white"][1])
            x_sum = 0
            y_sum = 0
            counter = 0

            for i in range(0, len(mask)):
                for j in range(0, len(mask[i])):
                    if(mask[i][j] == 255):
                        x_sum = x_sum + j
                        y_sum = y_sum + i
                        counter = counter + 1

            if(counter > 4000):
                return None, (x_sum / counter , y_sum / counter)
                
        return None, None

    '''
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

                        if length > 2:
                            adj = np.abs(y1-y2)
                            opp = np.abs(x1-x2)

                            num_lines += 1
                            sum_opp += opp
                            sum_adj += adj
                try:
                    hyp = math.sqrt(float(sum_opp/num_lines)**2 + float(sum_adj/num_lines)**2)
                    #num = (float(sum_opp/num_lines))/float((sum_adj/num_lines))
                    num = float(sum_adj/num_lines) / hyp
                    angle = np.arccos(num)

                    return np.degrees(angle)

                except:
                    return None

        return None
        '''
