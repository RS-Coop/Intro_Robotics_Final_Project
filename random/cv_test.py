import numpy as np
import cv2 as cv
import pyzbar.pyzbar as pyz
import time
import math

#This class deals with processing images from the Bebop drones
class CVProcessor:
    blank = None

    def __init__(self):
        self.line_colors = {
            'orange':[np.array([5,100,150]), np.array([15,255,255])],
            'purple':[np.array([275,50,50]), np.array([285,255,255])],
            'blue':[np.array([100,50,50]), np.array([115,255,255])]
        }


    #Takes an image and color filters for all
    #potential line colors to get line blobs
    #DONE: Creates mask for specfied color or a mask for each color if non-specific
    #TODO: Add more colors to line_colors
    def color_filter(self, image, color_type=None):
        masks = []
        colors = []
        hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)

        if color_type == None:
            for color, range in self.line_colors.items():
                mask = cv.inRange(hsv, range[0], range[1])
                masks.append(mask)
                colors.append(color)
        else:
            masks.append(cv.inRange(hsv, self.line_colors[color_type][0], self.line_colors[color_type][1]))
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

                        if length > 2:
                            cv.line(self.blank,(x1,y1),(x2,y2),(0,255,0),2)
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

    #Detect a QR code and determine centroid
    #DONE: Detect and calculate centroid if it exists
    def detect_QR_code(self, image):
        try :
            code = pyz.decode(image)
            if len(code) != 0:
                bb = code[0].rect
                value = code[0].data.decode('utf-8')

                return value, (bb[0], bb[1]), bb
            return None, None, None
        except:
            print("An exception occurred")
            return None, None, None

if __name__=="__main__":
    cvP = CVProcessor()

    # for i in range(1,8):
    #     image = cv.imread('../test_images/one_line_following_'+str(i)+'.jpg')
    #     dims = image.shape
    #
    #     cv.imshow('Image',image)
    #     cv.waitKey(0)
    #     cv.destroyAllWindows()
    #
    #     colors, masks = cvP.color_filter(image, 'blue')
    #
    #     for mask in masks:
    #         cvP.blank = np.zeros((dims[0],dims[1],3),np.uint8)
    #         value, cent, bb = cvP.detect_QR_code(image)
    #
    #         if value != None:
    #             cv.rectangle(mask, (bb[0],bb[1]), (bb[0]+bb[2], bb[1]+bb[3]), (0, 0, 255), 2)
    #
    #         cv.imshow('Mask', mask)
    #         cv.waitKey(0)
    #         cv.destroyAllWindows()
    #
    #         angle = cvP.line_angle(mask)
    #         #
    #         # cv.imshow('Lines',cvP.blank)
    #         # cv.waitKey(0)
    #         # cv.destroyAllWindows()
    #
    #         print('Angle: ', angle)
    #         print('QR: ', value)

    image = cv.imread('../test_images/one_line_following_7.jpg')
    dims = image.shape
    cv.namedWindow('Image', cv.WINDOW_NORMAL)
    cv.resizeWindow('Image', 600,600)

    cv.imshow('Image',image)
    cv.waitKey(0)
    # cv.destroyAllWindows()

    value, cent, bb = cvP.detect_QR_code(image)

    if value != None:
        cv.rectangle(image, (bb[0],bb[1]), (bb[0]+bb[2], bb[1]+bb[3]), (0, 0, 255), 2)
        print('QR: '+value)

    cv.imshow('Image',image)
    cv.waitKey(0)
    cv.destroyAllWindows()
