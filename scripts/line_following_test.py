#!/usr/bin/env python
import rospy
import unittest
import os
# import Drone_Control as dc
# from Image_Processing import cv_processor as cvp
from Globals import Globals as G
from Drone_Control import swarm_control as sc
from Image_Processing import cv_processor as cvp

import cv2 as cv

class TestLineFollowing(unittest.TestCase):
    parent_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

    def setUp(self):
        self.swarm_controller = sc.SwarmController()
        self.cv_processor = cvp.CVProcessor()

    ###
    ### Test follow one line
    ###
    def test_follow_one_line(self):
        #### SETUP
        # Instantiate swarm controller
        thisSwarmC = sc.SwarmController()
        # Check starting values are correct
        
        #### START
        self.assertEqual(thisSwarmC.current_state, G.TAKEOFF)
        self.assertEqual(thisSwarmC.current_graph_edge, None)

        #### TAKEOFF
        thisSwarmC.run_state()
        self.assertEqual(thisSwarmC.current_state, G.CENTER_QR)
        self.assertEqual(thisSwarmC.current_graph_edge, None)

        #### CENTER
        self.simulate_swarm_callback(thisSwarmC, self.parent_dir+"/test_images/one_line_following_1.jpg")
        thisSwarmC.run_state()
        self.assertEqual(thisSwarmC.current_state, G.DETERMINE_NEXT_LINE)
        self.assertEqual(thisSwarmC.current_graph_edge, None)

        #### DETERMINE_NEXT_LINE

        #### MOVE_ONTO_LINE
        #### MOVE_ONTO_LINE
        #### FOLLOW_LINE
        #### FOLLOW_LINE
        #### CENTER
        #### CENTER
        #### LAND
        
    # Simulate callbacks with image
    def simulate_swarm_callback(self, swarm, img_name):
        img = cv.imread(img_name)
        output_data = self.cv_processor.process_image(img)

        thisSwarmC.qr_data = output_data["qr"]
        thisSwarmC.edge_data = output_data["edges"]

if __name__ == '__main__':
    rospy.init_node('Test_Node')

    unittest.main()
