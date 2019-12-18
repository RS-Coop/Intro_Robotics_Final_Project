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
        self.swarmC.CENTER = G.MAX_CENTER
        self.swarmC.CENTER_X_ERROR = G.MAX_X_ERROR
        self.swarmC.CENTER_Y_ERROR = G.MAX_Y_ERROR
        self.cv_processor = cvp.CVProcessor()

    ###
    ###
    ### Test follow one line1
    ###
    ###

    #### START
    def test_follow_one_line_1(self):
        self.assertEqual(self.swarmC.current_state, G.TAKEOFF)
        self.assertEqual(self.swarmC.current_edge_color, None)

        print("HERE")
        self.simulate_swarm_callback(self.swarmC, self.parent_dir+"/test_images/two_line_following_1.jpg")
        self.swarmC.run_state()
        print("DONE")

    #### TAKEOFF
    # def test_follow_one_line_2(self):
        self.swarmC.run_state()
        self.assertEqual(self.swarmC.current_state, G.CENTER_QR)
        self.assertEqual(self.swarmC.current_edge_color, None)

    #### CENTER
    # def test_follow_one_line_3(self):
        self.simulate_swarm_callback(self.swarmC, self.parent_dir+"/test_images/one_line_following_1.jpg")
        self.swarmC.run_state()
        self.assertEqual(self.swarmC.current_state, G.DETERMINE_NEXT_LINE)
        self.assertEqual(self.swarmC.current_edge_color, None)

    #### DETERMINE_NEXT_LINE
    # def test_follow_one_line_4(self):
        self.simulate_swarm_callback(self.swarmC, self.parent_dir+"/test_images/one_line_following_1.jpg")
        self.swarmC.run_state()
        self.assertEqual(self.swarmC.current_state, G.MOVE_ONTO_LINE)
        self.assertEqual(self.swarmC.current_edge_color, G.BLUE)

        '''
    #### MOVE_ONTO_LINE (qr code is still readable, but at bottom of image)
    # def test_follow_one_line_5(self):
        self.simulate_swarm_callback(self.swarmC, self.parent_dir+"/test_images/one_line_following_2.jpg")
        self.swarmC.run_state()
        self.assertEqual(self.swarmC.current_state, G.MOVE_ONTO_LINE)
        self.assertEqual(self.swarmC.current_edge_color, G.BLUE)
        '''

    #### MOVE_ONTO_LINE (qr code is visible but not readabledeparting)
    # def test_follow_one_line_6(self):
        self.simulate_swarm_callback(self.swarmC, self.parent_dir+"/test_images/one_line_following_3.jpg")
        self.swarmC.run_state()
        self.assertEqual(self.swarmC.current_state, G.FOLLOW_LINE)
        self.assertEqual(self.swarmC.current_edge_color, G.BLUE)

    #### FOLLOW_LINE (just line)
    # def test_follow_one_line_7(self):
        self.simulate_swarm_callback(self.swarmC, self.parent_dir+"/test_images/one_line_following_4.jpg")
        self.swarmC.run_state()
        self.assertEqual(self.swarmC.current_state, G.FOLLOW_LINE)
        self.assertEqual(self.swarmC.current_edge_color, G.BLUE)

    #### FOLLOW_LINE (qr code is visible but not readable ariving)
    # def test_follow_one_line_8(self):
        self.simulate_swarm_callback(self.swarmC, self.parent_dir+"/test_images/one_line_following_5.jpg")
        self.swarmC.run_state()
        self.assertEqual(self.swarmC.current_state, G.FOLLOW_LINE)
        self.assertEqual(self.swarmC.current_edge_color, G.BLUE)
        self.assertEqual(self.swarmC.graph_edges, [{"color": G.BLUE, "v1": 2, "v2": None}])

    ###
    ###
    ### Test follow one line2
    ###
    ###

    #### START
    def test_follow_one_line_2(self):
        del self.swarmC
        # self.swarmC = sc.SwarmController()
        # self.swarmC.CENTER = G.MAX_CENTER
        # self.swarmC.CENTER_X_ERROR = G.MAX_X_ERROR
        # self.swarmC.CENTER_Y_ERROR = G.MAX_Y_ERROR
        self.setUp()
        self.swarmC.graph_edges = []

        self.assertEqual(self.swarmC.current_state, G.TAKEOFF)
        self.assertEqual(self.swarmC.current_edge_color, None)

    #### TAKEOFF
    # def test_follow_one_line_2(self):
        self.swarmC.run_state()
        self.assertEqual(self.swarmC.current_state, G.CENTER_QR)
        self.assertEqual(self.swarmC.current_edge_color, None)

    #### CENTER
    # def test_follow_one_line_3(self):
        self.simulate_swarm_callback(self.swarmC, self.parent_dir+"/test_images/two_line_following_7.jpg")
        self.swarmC.run_state()
        self.assertEqual(self.swarmC.current_state, G.DETERMINE_NEXT_LINE)
        self.assertEqual(self.swarmC.current_edge_color, None)

    #### DETERMINE_NEXT_LINE
    # def test_follow_one_line_4(self):
        self.simulate_swarm_callback(self.swarmC, self.parent_dir+"/test_images/two_line_following_7.jpg")
        self.swarmC.run_state()
        self.assertEqual(self.swarmC.current_state, G.MOVE_ONTO_LINE)
        self.assertEqual(self.swarmC.current_edge_color, G.ORANGE)

    #### MOVE_ONTO_LINE (just line)
    # def test_follow_one_line_7(self):
        self.simulate_swarm_callback(self.swarmC, self.parent_dir+"/test_images/two_line_following_8.jpg")
        self.swarmC.run_state()
        self.assertEqual(self.swarmC.current_state, G.FOLLOW_LINE)
        self.assertEqual(self.swarmC.current_edge_color, G.ORANGE)

    #### FOLLOW_LINE (qr code is readable,and centered)
    # def test_follow_one_line_10(self):
        self.simulate_swarm_callback(self.swarmC, self.parent_dir+"/test_images/two_line_following_9.jpg")
        self.swarmC.run_state()
        self.assertEqual(self.swarmC.current_state, G.CENTER_QR)
        self.assertEqual(self.swarmC.current_edge_color, None)

    #### CENTER_QR (qr code is readable,and centered)
    # def test_follow_one_line_10(self):
        self.simulate_swarm_callback(self.swarmC, self.parent_dir+"/test_images/two_line_following_9.jpg")
        self.swarmC.run_state()
        self.assertEqual(self.swarmC.current_state, G.DETERMINE_NEXT_LINE)
        self.assertEqual(self.swarmC.current_edge_color, None)

    #### DETERMINE_NEXT_LINE
    # def test_follow_one_line_3(self):
        self.simulate_swarm_callback(self.swarmC, self.parent_dir+"/test_images/two_line_following_10.jpg")
        print("#"*40)
        print(self.swarmC.graph_edges)
        self.swarmC.run_state()
        self.assertEqual(self.swarmC.current_state, G.MOVE_ONTO_LINE)
        print("#"*40)
        print(self.swarmC.graph_edges)
        self.assertEqual(self.swarmC.current_edge_color, G.BLUE)

    #### DETERMINE_NEXT_LINE
    # def test_follow_one_line_4(self):
        self.simulate_swarm_callback(self.swarmC, self.parent_dir+"/test_images/two_line_following_10.jpg")
        self.swarmC.run_state()
        self.assertEqual(self.swarmC.current_state, G.MOVE_ONTO_LINE)
        self.assertEqual(self.swarmC.current_edge_color, G.BLUE)

    #### MOVE_ONTO_LINE (just line)
    # def test_follow_one_line_7(self):
        self.simulate_swarm_callback(self.swarmC, self.parent_dir+"/test_images/two_line_following_11.jpg")
        self.swarmC.run_state()
        self.assertEqual(self.swarmC.current_state, G.MOVE_ONTO_LINE)
        self.assertEqual(self.swarmC.current_edge_color, G.BLUE)

    #### MOVE_ONTO_LINE
    # def test_follow_one_line_4(self):
        self.simulate_swarm_callback(self.swarmC, self.parent_dir+"/test_images/two_line_following_12.jpg")
        self.swarmC.run_state()
        self.assertEqual(self.swarmC.current_state, G.FOLLOW_LINE)
        self.assertEqual(self.swarmC.current_edge_color, G.BLUE)

    #### FOLLOW_LINE (qr code is readable,and centered)
    # def test_follow_one_line_10(self):
        self.simulate_swarm_callback(self.swarmC, self.parent_dir+"/test_images/two_line_following_13.jpg")
        self.swarmC.run_state()
        self.assertEqual(self.swarmC.current_state, G.CENTER_QR)
        self.assertEqual(self.swarmC.current_edge_color, None)

    #### CENTER_QR (qr code is readable,and centered)
    # def test_follow_one_line_10(self):
        self.simulate_swarm_callback(self.swarmC, self.parent_dir+"/test_images/two_line_following_13.jpg")
        self.swarmC.run_state()
        self.assertEqual(self.swarmC.current_state, G.LAND)
        self.assertEqual(self.swarmC.current_edge_color, None)

    ###
    ###
    ### Test follow one line 3
    ###
    ###
    '''
    #### START
    def test_follow_one_line_3(self):
        # Setup
        self.swarmC = sc.SwarmController()
        self.swarmC.CENTER = G.MAX_CENTER
        self.swarmC.CENTER_X_ERROR = G.MAX_X_ERROR
        self.swarmC.CENTER_Y_ERROR = G.MAX_Y_ERROR
        self.cv_processor = cvp.CVProcessor()

        self.assertEqual(self.swarmC.current_state, G.TAKEOFF)
        self.assertEqual(self.swarmC.current_edge_color, None)

    #### TAKEOFF
    # def test_follow_one_line_2(self):
        self.swarmC.run_state()
        self.assertEqual(self.swarmC.current_state, G.CENTER_QR)
        self.assertEqual(self.swarmC.current_edge_color, None)

    #### CENTER
    # def test_follow_one_line_3(self):
        self.simulate_swarm_callback(self.swarmC, self.parent_dir+"/test_images/two_line_following_0.jpg")
        self.swarmC.run_state()
        self.assertEqual(self.swarmC.current_state, G.DETERMINE_NEXT_LINE)
        self.assertEqual(self.swarmC.current_edge_color, None)

    #### DETERMINE_NEXT_LINE
    # def test_follow_one_line_4(self):
        self.simulate_swarm_callback(self.swarmC, self.parent_dir+"/test_images/two_line_following_0.jpg")
        self.swarmC.run_state()
        self.assertEqual(self.swarmC.current_state, G.MOVE_ONTO_LINE)
        self.assertEqual(self.swarmC.current_edge_color, G.BLUE)

    #### MOVE_ONTO_LINE (qr code is still readable, but at bottom of image)
    # def test_follow_one_line_5(self):
        self.simulate_swarm_callback(self.swarmC, self.parent_dir+"/test_images/two_line_following_1.jpg")
        self.swarmC.run_state()
        self.assertEqual(self.swarmC.current_state, G.MOVE_ONTO_LINE)
        self.assertEqual(self.swarmC.current_edge_color, G.BLUE)


    #### MOVE_ONTO_LINE (qr code is visible but not readabledeparting)
    # def test_follow_one_line_6(self):
        self.simulate_swarm_callback(self.swarmC, self.parent_dir+"/test_images/two_line_following_2.jpg")
        self.swarmC.run_state()
        self.assertEqual(self.swarmC.current_state, G.FOLLOW_LINE)
        self.assertEqual(self.swarmC.current_edge_color, G.BLUE)

    #### FOLLOW_LINE (just line)
    # def test_follow_one_line_7(self):
        self.simulate_swarm_callback(self.swarmC, self.parent_dir+"/test_images/two_line_following_3.jpg")
        self.swarmC.run_state()
        self.assertEqual(self.swarmC.current_state, G.FOLLOW_LINE)
        self.assertEqual(self.swarmC.current_edge_color, G.BLUE)

    #### FOLLOW_LINE (qr code is visible but not readable ariving)
    # def test_follow_one_line_8(self):
        self.simulate_swarm_callback(self.swarmC, self.parent_dir+"/test_images/two_line_following_4.jpg")
        self.swarmC.run_state()
        self.assertEqual(self.swarmC.current_state, G.FOLLOW_LINE)
        self.assertEqual(self.swarmC.current_edge_color, G.BLUE)
        self.assertEqual(self.swarmC.graph_edges, [{"color": G.BLUE, "v1": 2, "v2": None}])

    #### FOLLOW_LINE (qr code is visible and readable, ariving)
    # def test_follow_one_line_9(self):
        self.simulate_swarm_callback(self.swarmC, self.parent_dir+"/test_images/two_line_following_4_2.jpg")
        self.swarmC.run_state()
        self.assertEqual(self.swarmC.current_state, G.CENTER_QR)
        self.assertEqual(self.swarmC.current_edge_color, None)
        self.assertEqual(self.swarmC.graph_edges, [{"color": G.BLUE, "v1": 2, "v2": 1}])

    #### CENTER_QR (qr code is readable,and centered)
    # def test_follow_one_line_10(self):
        self.simulate_swarm_callback(self.swarmC, self.parent_dir+"/test_images/two_line_following_5.jpg")
        self.swarmC.run_state()
        self.assertEqual(self.swarmC.current_state, G.DETERMINE_NEXT_LINE)
        self.assertEqual(self.swarmC.current_edge_color, None)

    # #### CENTER_QR (qr code is readable and centered)
    # # def test_follow_one_line_11(self):
    #     self.simulate_swarm_callback(self.swarmC, self.parent_dir+"/test_images/two_line_following_7.jpg")
    #     self.swarmC.run_state()
    #     self.assertEqual(self.swarmC.current_state, G.DETERMINE_NEXT_LINE)
    #     self.assertEqual(self.swarmC.current_edge_color, None)

    # #### DETERMINE_NEXT_LINE
    # # def test_follow_one_line_12(self):
    #     self.simulate_swarm_callback(self.swarmC, self.parent_dir+"/test_images/two_line_following_7.jpg")
    #     self.swarmC.run_state()
    #     self.assertEqual(self.swarmC.current_state, G.LAND)
    #     self.assertEqual(self.swarmC.current_edge_color, None)
    '''
    ##
    ##
    ##
    ##
    ##
    # Simulate callbacks with image
    def simulate_swarm_callback(self, swarm, img_name):
        img = cv.imread(img_name)
        output_data = self.cv_processor.process_image(img)
        print("Output:", output_data)
        swarm.qr_data = output_data["qr"]
        swarm.edge_data = output_data["edges"]

if __name__ == '__main__':
    rospy.init_node('Test_Node')

    unittest.main()
