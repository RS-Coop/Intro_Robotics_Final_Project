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
        self.swarmC = sc.SwarmController()
        self.cv_processor = cvp.CVProcessor()

    ###
    ### Test follow one line
    ###

    #### START
    def test_follow_one_line_1(self):        
        self.assertEqual(self.swarmC.current_state, G.TAKEOFF)
        self.assertEqual(self.swarmC.current_graph_edge, None)

    #### TAKEOFF
    # def test_follow_one_line_2(self):
        self.swarmC.run_state()
        self.assertEqual(self.swarmC.current_state, G.CENTER_QR)
        self.assertEqual(self.swarmC.current_graph_edge, None)

    #### CENTER
    # def test_follow_one_line_3(self):
        self.simulate_swarm_callback(self.swarmC, self.parent_dir+"/test_images/one_line_following_1.jpg")
        self.swarmC.run_state()
        self.assertEqual(self.swarmC.current_state, G.DETERMINE_NEXT_LINE)
        self.assertEqual(self.swarmC.current_graph_edge, None)

    #### DETERMINE_NEXT_LINE
    # def test_follow_one_line_4(self):
        self.simulate_swarm_callback(self.swarmC, self.parent_dir+"/test_images/one_line_following_1.jpg")
        self.swarmC.run_state()
        self.assertEqual(self.swarmC.current_state, G.MOVE_ONTO_LINE)
        self.assertEqual(self.swarmC.current_graph_edge, {"color": G.BLUE, "v1": 1, "v2": None})

    #### MOVE_ONTO_LINE (qr code is still readable, but at bottom of image)
    # def test_follow_one_line_5(self):
        self.simulate_swarm_callback(self.swarmC, self.parent_dir+"/test_images/one_line_following_2.jpg")
        self.swarmC.run_state()
        self.assertEqual(self.swarmC.current_state, G.MOVE_ONTO_LINE)
        self.assertEqual(self.swarmC.current_graph_edge, {"color": G.BLUE, "v1": 1, "v2": None})


    #### MOVE_ONTO_LINE (qr code is visible but not readabledeparting)
    # def test_follow_one_line_6(self):
        self.simulate_swarm_callback(self.swarmC, self.parent_dir+"/test_images/one_line_following_3.jpg")
        self.swarmC.run_state()
        self.assertEqual(self.swarmC.current_state, G.FOLLOW_LINE)
        self.assertEqual(self.swarmC.current_graph_edge, {"color": G.BLUE, "v1": 1, "v2": None})

    #### FOLLOW_LINE (just line)
    # def test_follow_one_line_7(self):
        self.simulate_swarm_callback(self.swarmC, self.parent_dir+"/test_images/one_line_following_4.jpg")
        self.swarmC.run_state()
        self.assertEqual(self.swarmC.current_state, G.FOLLOW_LINE)
        self.assertEqual(self.swarmC.current_graph_edge, {"color": G.BLUE, "v1": 1, "v2": None})
    
    #### FOLLOW_LINE (qr code is visible but not readable ariving)
    # def test_follow_one_line_8(self):
        self.simulate_swarm_callback(self.swarmC, self.parent_dir+"/test_images/one_line_following_5.jpg")
        self.swarmC.run_state()
        self.assertEqual(self.swarmC.current_state, G.FOLLOW_LINE)
        self.assertEqual(self.swarmC.current_graph_edge, {"color": G.BLUE, "v1": 1, "v2": None})
        self.assertEqual(self.swarmC.graph_edges, [{"color": G.BLUE, "v1": 1, "v2": None}])

    #### FOLLOW_LINE (qr code is visible and readable)
    # def test_follow_one_line_9(self):
        self.simulate_swarm_callback(self.swarmC, self.parent_dir+"/test_images/one_line_following_6.jpg")
        self.swarmC.run_state()
        self.assertEqual(self.swarmC.current_state, G.CENTER_QR)
        self.assertEqual(self.swarmC.current_graph_edge, None)
        self.assertEqual(self.swarmC.graph_edges, [{"color": G.BLUE, "v1": 1, "v2": 2}])

    #### CENTER_QR (qr code is readable, but at top of image)
    # def test_follow_one_line_10(self):
        self.simulate_swarm_callback(self.swarmC, self.parent_dir+"/test_images/one_line_following_6.jpg")
        self.swarmC.run_state()
        self.assertEqual(self.swarmC.current_state, G.CENTER_QR)
        self.assertEqual(self.swarmC.current_graph_edge, None)

    #### CENTER_QR (qr code is readable and centered)
    # def test_follow_one_line_11(self):
        self.simulate_swarm_callback(self.swarmC, self.parent_dir+"/test_images/one_line_following_7.jpg")
        self.swarmC.run_state()
        self.assertEqual(self.swarmC.current_state, G.DETERMINE_NEXT_LINE)
        self.assertEqual(self.swarmC.current_graph_edge, None)

    #### DETERMINE_NEXT_LINE
    # def test_follow_one_line_12(self):
        self.simulate_swarm_callback(self.swarmC, self.parent_dir+"/test_images/one_line_following_7.jpg")
        self.swarmC.run_state()
        self.assertEqual(self.swarmC.current_state, G.LAND)
        self.assertEqual(self.swarmC.current_graph_edge, None)
        
        
    # Simulate callbacks with image
    def simulate_swarm_callback(self, swarm, img_name):
        img = cv.imread(img_name)
        output_data = self.cv_processor.process_image(img)

        swarm.qr_data = output_data["qr"]
        swarm.edge_data = output_data["edges"]

if __name__ == '__main__':
    rospy.init_node('Test_Node')

    unittest.main()
