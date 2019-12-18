#!/usr/bin/env python
import rospy
import os
import sys
from Globals import Globals as G
from Image_Processing import cv_processor as cvP
import cv2 as cv

class ImageTester():
    parent_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    cv_proc = cvP.CVProcessor()

    def __init__(self, image_name):
        self.file = self.parent_dir+"/test_images/"+image_name
        print(self.file)

    def run(self):
        img = cv.imread('../test_images/two_line_following_10.jpg')
        output_data = self.cv_proc.process_image(img)
        print("Output:", output_data)

if __name__=='__main__':
    file_name = sys.argv[1]

    IT = ImageTester(file_name)

    IT.run()
