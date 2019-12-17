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
        #Publishers
        self.qr_pub = rospy.Publisher('/swarm/qr_code', QR, queue_size=1)
        self.edges_pub = rospy.Publisher('/swarm/edges', EdgeList, queue_size=1)
        #Subscribers
        self.image_sub = rospy.Subscriber('/swarm/drone_image', TaggedImage, self.image_callback)

        rospy.sleep(1.0)

################################################################################
#Main loop
    def run_node(self):
        while not rospy.is_shutdown():
            continue

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
        img_data = self.cvP.process_ros_image(data.image)

        qrMsg.existing = img_data["qr"]["hasQR"]
        qrMsg.centroid.append(img_data["qr"]["centroid"][0])
        qrMsg.centroid.append(img_data["qr"]["centroid"][1])
        qrMsg.value = img_data["qr"]["value"]

        for i in img_data["edges"]:
            if i!=None:
                edgeMsg.colors.append(i["color"])
                for value in i["pos_avgs"].values():
                    edgeMsg.pos_avgs.append(value[0])
                    edgeMsg.pos_avgs.append(value[1])
                    
        # Publish results to topic
        self.qr_pub.publish(qrMsg)
        self.edges_pub.publish(edgeMsg)
