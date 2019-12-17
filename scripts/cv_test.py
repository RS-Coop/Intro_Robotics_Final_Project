#!/usr/bin/env python
import rospy
import unittest
import os
import Drone_Control as dc
from Globals import Globals as G
from Image_Processing import cv_processor as cvp

import cv2 as cv

class TestImageProcessing(unittest.TestCase):
    parent_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

    def setUp(self):
        self.cv_processor = cvp.CVProcessor()

    # Identify a landmark in an image
    # def test_identify_landmark(self):
    #     img = cv.imread("has_landmark.jpeg")
    #     self.assertTrue(self.imageProcessor.image_evaluate_features_from_file(fileName))

    #### QR Code Identification
    # Test detect_qr code with no QR code
    def test_no_detect_qr_code(self):
        img = cv.imread(self.parent_dir+"/test_images/not_has_qr_code.jpg")
        self.assertEqual(self.cv_processor.detect_QR_code(img)[0], None)

    # Test detect_qr code with QR code
    def test_detect_qr_code(self):
        img = cv.imread(self.parent_dir+"/test_images/has_qr_code.jpg")
        self.assertNotEqual(self.cv_processor.detect_QR_code(img)[0], None)

    # Test detect_qr code with QR code partially showing in image
    def test_detect_partial_qr_code(self):
        img = cv.imread(self.parent_dir+"/test_images/has_partial_qr_upper_left.jpg")
        self.assertEqual(self.cv_processor.detect_QR_code(img)[0], None)

       #### Vertex Identification
    # Identify that a QR code is NOT in the image (no QR code showing)
    # Requires an image with a QR code in it
    def test_dont_identify_no_qr(self):
        img = cv.imread(self.parent_dir+"/test_images/not_has_qr_code.jpg")
        self.assertEqual(self.cv_processor.process_image(img)["qr"]["hasQR"], False)

    # Identify that a QR code is NOT in the image (QR code partially showing)
    # Requires an image with a QR code in it
    def test_dont_identify_partial_qr(self):
        img = cv.imread(self.parent_dir+"/test_images/has_partial_qr_upper_left.jpg")
        self.assertEqual(self.cv_processor.process_image(img)["qr"]["hasQR"], False)

    # Identify that a QR code is in the image (entire QR code showing)
    # Requires an image with a QR code in it
    def test_identify_qr(self):
        img = cv.imread(self.parent_dir+"/test_images/has_qr_code.jpg")
        self.assertEqual(self.cv_processor.process_image(img)["qr"]["hasQR"], True)

    # Identify the location of a QR code in the image
    # Requires an image with a QR code in it
    # Upper 3rd
    def test_identify_upper_third_qr(self):
        pass

    # Identify a QR code is not centered in the image (only location)
    # Requires an image with a QR code in it not centered
    def test_identify_right_third_qr(self):
        pass

    # Identify a QR code is centered in the image (only location)
    # Requires an image with a QR code in it centered centered
    def test_identify_centered_qr(self):
        pass

    #### Edge Identification
    # Identify a single edge coming out of a vertex
    # Requires an image of a centered QR code with one color tape coming out of it
    def test_identify_one_blue_edge(self):
        img = cv.imread(self.parent_dir+"/test_images/blue_edge.jpg")
        # img = cv.resize(img, (640, 368))
        found = False

        for i in self.cv_processor.process_image(img)["edges"]:
            if i["color"] == "blue":
                found = True

        self.assertTrue(found)

    def test_identify_one_orange_edge(self):
        img = cv.imread(self.parent_dir+"/test_images/orange_edge.jpg")
        # img = cv.resize(img, (640, 368))
        found = False

        for i in self.cv_processor.process_image(img)["edges"]:
            if i["color"] == "orange":
                found = True

        self.assertTrue(found)

    # def test_identify_one_purple_edge(self):
    #     img = cv.imread(self.parent_dir+"/test_images/purple_edge.jpg")
    #     # img = cv.resize(img, (640, 368))
    #     found = False

    #     for i in self.cv_processor.process_image(img)["edges"]:
    #         if i["color"] == "purple":
    #             found = True

    #     self.assertTrue(found)

    def test_identify_one_edge(self):
        img = cv.imread(self.parent_dir+"/test_images/1_edge.jpg")
        # img = cv.resize(img, (640, 368))
        self.assertEqual(len(self.cv_processor.process_image(img)["edges"]), 1)

    # Identify 2 edges coming out of a vertex
    # Requires an image of a centered QR code with two colors of tape coming out of it
    def test_identify_two_edges(self):
        img = cv.imread(self.parent_dir+"/test_images/2_edge.jpg")
        # img = cv.resize(img, (640, 368))
        self.assertEqual(len(self.cv_processor.process_image(img)["edges"]), 2)

    '''
    # ###
    # ### Test decode
    # ### 
    # def test_decode_none_1(self):
    #     img = None
    #     self.assertEqual(self.cv_processor.detect_QR_code(img), (None, None))

    def test_one_line_line_following(self):
        # Starts on QR code
        img = cv.imread(self.parent_dir+"/test_images/one_line_following_1.jpg")
        output_data = self.cv_processor.process_image(img)
        # Asserts
        # Check has QR code
        self.assertTrue(output_data["qr"]["hasQR"])
        # Check QR centroid
        self.assertEqual(output_data["qr"]["centroid"], (0,0))
        # Check QR value
        self.assertEqual(output_data["qr"]["value"], 1)
        # Check centroid
        self.assertEqual(output_data["edges"][0]["centroid"], (0, 0))

        # Only line visible
        img = cv.imread(self.parent_dir+"/test_images/one_line_following_2.jpg")
        output_data = self.cv_processor.process_image(img)
        # Asserts
        # Check has QR code
        self.assertTrue(output_data["qr"]["hasQR"])
        # Check QR centroid
        self.assertEqual(output_data["qr"]["centroid"], (0,0))
        # Check QR value
        self.assertEqual(output_data["qr"]["value"], 1)
        # Check centroid
        self.assertEqual(output_data["edges"][0]["centroid"], (0, 0))

        # QR code partially visible ariving
        img = cv.imread(self.parent_dir+"/test_images/one_line_following_3.jpg")
        output_data = self.cv_processor.process_image(img)
        # Asserts
        self.assertTrue(-5 <= self.cv_processor.process_image(img)["edges"][0]["angle"] <= 5, True)
        # Check has QR code
        self.assertTrue(output_data["qr"]["hasQR"])
        # Check QR centroid
        self.assertEqual(output_data["qr"]["centroid"], (0,0))
        # Check QR value
        self.assertEqual(output_data["qr"]["value"], 1)
        # Check angle
        self.assertEqual(output_data["edges"][0]["angle"], 0)
        # Check centroid
        self.assertEqual(output_data["edges"][0]["centroid"], (0, 0))

        # QR code entirley visible ariving
        img = cv.imread(self.parent_dir+"/test_images/one_line_following_4.jpg")
        output_data = self.cv_processor.process_image(img)
        # Asserts
        self.assertTrue(-5 <= self.cv_processor.process_image(img)["edges"][0]["angle"] <= 5, True)
        # Check has QR code
        self.assertTrue(output_data["qr"]["hasQR"])
        # Check QR centroid
        self.assertEqual(output_data["qr"]["centroid"], (0,0))
        # Check QR value
        self.assertEqual(output_data["qr"]["value"], 1)
        # Check centroid
        self.assertEqual(output_data["edges"][0]["centroid"], (0, 0))
    '''

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
