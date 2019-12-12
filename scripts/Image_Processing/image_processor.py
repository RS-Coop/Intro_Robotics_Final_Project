import rospy
import numpy as np
import cv2 as cv
import pyzbar.pyzbar as pyz
from Image_Processing import cv_processor as cvp
from Intro_Robotics_Final_Project.msg import EdgeList, TaggedImage, QR

#This class deals with processing images from the Bebop drones
class ImageProcessor:

    def __init__(self):
        #Initialize pubs and subs
        #Publishers
        self.qr_pub = rospy.Publisher('/swarm/qr_code', QR, queue_size=1)
        self.edges_pub = rospy.Publisher('/swarm/edges', EdgeList, queue_size=1)
        #Subscribers
        self.image_sub = rospy.Subscriber('/swarm/drone_image', TaggedImage, self.image_callback())

        rospy.sleep(1.0)

################################################################################
#Callbacks
    #Proccesses image from drone
    #TODO: What will this call or do?
    def image_callback(self,data):
        # Process the image
        process_image(data)

        # Publish results to topic

        pass
