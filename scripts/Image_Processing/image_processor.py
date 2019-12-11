import rospy
import numpy as np
import cv2 as cv
import pyzbar.pyzbar as pyz
#This class deals with processing images from the Bebop drones
class ImageProcessor:
    #List of line color ranges
    self.line_colors = [[5,100,150], [15,255,255]] #Oragne right now

    def __init__(self):
        #Initialize pubs and subs
        #Publishers

        #Subscribers
        self.image_sub = rospy.Subscriber('/swarm/drone_image', TaggedImage, self.image_callback())

        sleep(1.0)

################################################################################
#Main methods and publisher methods
    #Loop to be run in main script
    #DONE: I think this works
    def run_node(self):
        while not rospy.is_shutdown():
            continue

    #Takes an image and color filters for all
    #potential line colors to get line blobs
    #TODO: Do this
    def line_color_filter(self, image):
        hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)
        #Right now this is just orange, I think it will need to be a loop
        mask = cv.inRange(hsv, self.line_colors[0], self.line_colors[1])

    #Takes an image that is the isolated line blob
    #and returns the angle to vertical
    #TODO: Figure out how to do this
    def line_angle(self, line_mask):
        edges = cv.Canny(line_mask,50,150,apertureSize = 3)
        lines = cv.HoughLinesP(edges,1,np.pi/180,10)

        #Now to calculate angle from lines

    #Takes a QR code image and proccess it.
    #DONE: Returns string of QR code data
    def process_QR_code(self,image):
        code = pyz.decode(img)

        return code[0].data.decode('utf-8')

    '''
    #Takes in a
    #TODO: What does this do
    def image_evaluate_features_from_file(self, img_loc):
        img = cv.imread(img_loc)
        self.image_evaluate_features_from_matrix(img)

        return True

    #What does this do?
    #TODO:
    def image_evaluate_features_from_matrix(self, img):
        # Call mask_land_mark and post the image to the topic for consumption by the swarm controller
        self.mask_land_mark(img)
        # Call mask_path and post the image to the topic for consumption by the swarm controller

        return True

    #Returns an image where a mask is applied only to pixels in the image
    #that are determined to constitute a landmark
    #DONE
    def mask_land_mark(self, img):
        print(img)
        return True

    #Returns an image where a mask is applied only to pixels in the image
    #that are determined to constitute the path
    def mask_lines(self, img):
        img = cv.imread(img,0)
        edges = cv.Canny(img,700,750)
        return edges
    '''

################################################################################
#Callbacks
    #Proccesses image from drone
    #TODO: What will this call or do?
    def image_callback(self,data):
        pass

################################################################################
#Tests
    #Not sure what this does
    #TODO:
    def test_method(self):
        return True
