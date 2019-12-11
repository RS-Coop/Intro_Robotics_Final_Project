#!/usr/bin/env python
import rospy
import unittest
import Drone_Control as dc
import Image_Processing as ip
import cv2 as cv

class TestImageProcessing(unittest.TestCase):
    def setUp(self):
        self.imageProcessor = ip.ImageProcessor()

    # Identify a landmark in an image
    # def test_identify_landmark(self):
    #     img = cv.imread("has_landmark.jpeg")
    #     self.assertTrue(self.imageProcessor.image_evaluate_features_from_file(fileName))

    #### QR Code Identification
    # Test detect_qr code with no QR code
    def test_no_detect_qr_code(self):
        img = cv.imread("test_images/not_has_qr_code.jpg")
        self.assertEqual(self.imageProcessor.detect_QR_code(img), None)

    # Test detect_qr code with QR code
    def test_detect_qr_code(self):
        img = cv.imread("test_images/has_qr_code.jpg")
        self.assertNotEqual(self.imageProcessor.detect_QR_code(img), None)

    # Test detect_qr code with QR code partially showing in image
    def test_detect_partial_qr_code(self):
        img = cv.imread("test_images/has_partial_qr_upper_left.jpg")
        self.assertEqual(self.imageProcessor.detect_QR_code(img), None)
    
    #### Vertex Identification
    # Identify that a QR code is NOT in the image (no QR code showing)
    # Requires an image with a QR code in it
    # def test_identify_vertex(self):
    #     fileName = "has_vertex.png"

    #     # expectedOutput = {"qr" : {"hasQR" : True, ""}}

    #     self.assertEqual()

    # Identify that a QR code is NOT in the image (QR code partially showing)
    # Requires an image with a QR code in it
    
    # Identify that a QR code is in the image (entire QR code showing)
    # Requires an image with a QR code in it

    # Identify the location of a QR code in the image
    # Requires an image with a QR code in it

    # Identify a QR code is not centered in the image (only location)
    # Requires an image with a QR code in it not centered

    # Identify a QR code is centered in the image (only location)
    # Requires an image with a QR code in it centered centered

    #### Edge Identification 
    # Identify a single edge coming out of a vertex
    # Requires an image of a centered QR code with one color tape coming out of it

    # Identify the angle a single edge is coming out of a vertex
    # Requires an image of a centered QR code with one color tape coming out of it
    
    # Identify 2 edges coming out of a vertex
    # Requires an image of a centered QR code with two colors of tape coming out of it

    # Identify the angles of 2 edges coming out of a vertex
    # Requires an image of a centered QR code with two colors of tape coming out of it

class TestDroneController():
    def __init__():
        self.drone = dc.DroneController()

    def test_test1(self):
        self.drone.test_takeoffandland()

class TestSwarmController():
    def __init__():
        self.swarm = dc.SwarmController()

    def test_test1(self):
        pass

if __name__ == '__main__':
    # rospy.init_node('Test_Node')

    unittest.main()
