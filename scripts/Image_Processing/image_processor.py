import rospy
import numpy as np
import cv2 as cv
import pyzbar.pyzbar as pyz
from Image_Processing import cv_processor as cvp
from Intro_Robotics_Final_Project.msg import EdgeList, TaggedImage, QR

#This class deals with processing images from the Bebop drones
class ImageProcessor:
    cvP = cvp.CVProcessor()
    last_call = 0.0

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
    #DONE: Seems kinda empty, but thats fine
    #NOTE: Stuff still needs to be finished in cvp
    def image_callback(self,data):
        #Not sure we need this but if process_image is too slow then yes
        # if rospy.get_time() - last_call < 0.5

        qrMsg = QR()
        edgeMsg = EdgeList()

        # Process the image
        img_data = self.cvP.process_image(data)
        qrMsg.existing = img_data["qr"]["hasQR"]
        qrMsg.centroid[0] = img_data["qr"]["centroid"][0]
        qrMsg.centroid[1] = img_data["qr"]["centroid"][1]

        currentIndex = 0
        currentAngleInex = 0
        for i in img_data["edges"]:
            edgeMsg.colors[currentIndex] = i["color"]
            edgeMsg.angles[currentIndex] = i["angle"]
            edgeMsg.angles[currentAngleInex] = i["centroid"][0]
            edgeMsg.angles[currentAngleInex + 1] = i["centroid"][1]

            currentIndex+=1
            currentAngleInex+=2

        # Publish results to topic
        qr_pub.publish(qrMsg)
        edges_pub.publish(edgeMsg)
