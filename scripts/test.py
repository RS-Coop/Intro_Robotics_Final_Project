#!/usr/bin/env python
import rospy
import unittest
import Drone_Control as dc
import Image_Processing as ip

class TestImageProcessing(unittest.TestCase):
    def setUp(self):
        self.imageProcessor = ip.ImageProcessor()

    def test_test1(self):
        self.assertTrue(True)

    def test_test2(self):
        self.assertTrue(self.imageProcessor.test_method())

    # Identify a landmark in an image
    def test_identify_landmark(self):
        fileName = "has_landmark.jpeg"
        self.assertTrue(self.imageProcessor.image_evaluate_features_from_file(fileName))

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
    rospy.init_node('Test_Node')

    unittest.main()
