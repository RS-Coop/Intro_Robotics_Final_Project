import rospy
import numpy as np
import cv2 as cv
import pyzbar.pyzbar as pyz
#This class deals with processing images from the Bebop drones
class ImageProcessor:
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

    #Takes a QR code image and proccess it.
    #DONE: Returns string of QR code data
    def process_QR_code(self,image):
        code = pyz.decode(img)

        return code[0].data.decode('utf-8')

    #Takes an line image, or maybe a sub feature extracted
    #from an image and determines its direction
    #i.e. left, right, straight, horizontal
    #TODO: Figure out how to do this
    def line_direction(self,image):
        

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
