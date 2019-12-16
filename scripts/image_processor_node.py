#!/usr/bin/env python
import rospy
import Image_Processing as ip

#Initializes a node for a image processor
rospy.init_node('Image_Processor')

#Initializes a drone
proc = ImageProcessor()

print("Image processor ready")

#Runs loop
proc.run_node()

print("Image processor finished")
