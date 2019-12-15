#!/usr/bin/env python
import rospy
import unittest
import os
import Drone_Control as dc
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
        self.assertEqual(self.cv_processor.detect_QR_code(img), None)

    # Test detect_qr code with QR code
    def test_detect_qr_code(self):
        img = cv.imread(self.parent_dir+"/test_images/has_qr_code.jpg")
        self.assertNotEqual(self.cv_processor.detect_QR_code(img), None)

    # Test detect_qr code with QR code partially showing in image
    def test_detect_partial_qr_code(self):
        img = cv.imread(self.parent_dir+"/test_images/has_partial_qr_upper_left.jpg")
        self.assertEqual(self.cv_processor.detect_QR_code(img), None)

       #### Vertex Identification
    # Identify that a QR code is NOT in the image (no QR code showing)
    # Requires an image with a QR code in it
    def test_dont_identify_no_qr(self):
        img = cv.imread(self.parent_dir+"/test_images/not_has_qr_code.jpg")
<<<<<<< HEAD
        self.assertEqual(self.cv_processor.process_image(img)["qr"]["hasQR"], False)
=======
        output = self.cv_processor.process_image(img)
        print(output)
        self.assertEqual(output["QR"]["hasQR"], False)
>>>>>>> 6d20798ad92d8e97a1921cb60c58df1c42e10bb2

    # Identify that a QR code is NOT in the image (QR code partially showing)
    # Requires an image with a QR code in it
    def test_dont_identify_partial_qr(self):
        img = cv.imread(self.parent_dir+"/test_images/has_partial_qr_upper_left.jpg")
<<<<<<< HEAD
        self.assertEqual(self.cv_processor.process_image(img)["qr"]["hasQR"], False)

=======
        output = self.cv_processor.process_image(img)
        print(output)
        self.assertEqual(output["QR"]["hasQR"], False)
    
>>>>>>> 6d20798ad92d8e97a1921cb60c58df1c42e10bb2
    # Identify that a QR code is in the image (entire QR code showing)
    # Requires an image with a QR code in it
    def test_identify_qr(self):
        img = cv.imread(self.parent_dir+"/test_images/has_qr_code.jpg")
<<<<<<< HEAD
        self.assertEqual(self.cv_processor.process_image(img)["qr"]["hasQR"], True)
=======
        output = self.cv_processor.process_image(img)
        print(output)
        self.assertEqual(output["QR"]["hasQR"], True)
        self.assertEqual(output["QR"]["centroid"], (0, 0))
>>>>>>> 6d20798ad92d8e97a1921cb60c58df1c42e10bb2

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
    def test_identify_one_edge(self):
        pass

    # Identify the angle a single edge is coming out of a vertex
    # Requires an image of a centered QR code with one color tape coming out of it
<<<<<<< HEAD

=======
    # Test edge at 90 degrees
    def test_identify_one_edge_135_degrees(self):
        pass

    def test_identify_one_edge_90_degrees(self):
        pass

    def test_identify_one_edge_45_degrees(self):
        pass

    def test_identify_one_edge_0_degrees(self):
        pass

    def test_identify_one_edge_neg_45_degrees(self):
        pass

    def test_identify_one_edge_neg_90_degrees(self):
        pass

    def test_identify_one_edge_neg_135_degrees(self):
        pass
    
>>>>>>> 6d20798ad92d8e97a1921cb60c58df1c42e10bb2
    # Identify 2 edges coming out of a vertex
    # Requires an image of a centered QR code with two colors of tape coming out of it
    def test_identify_two_edges(self):
        pass

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
