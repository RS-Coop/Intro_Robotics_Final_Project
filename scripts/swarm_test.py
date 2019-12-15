#!/usr/bin/env python
import rospy
import unittest
import os
# import Drone_Control as dc
# from Image_Processing import cv_processor as cvp
from Drone_Control import swarm_control as sc

import cv2 as cv

class TestSwarm(unittest.TestCase):
    parent_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

    def setUp(self):
        self.swarm_controller = sc.SwarmController()

    # def test_line_follow(self):
    #     # Instantiate swarm controller
    #     thisSwarmC = new SwarmController()
    #     # Set curr_state
    #     thisSwarmC.current_state = LINE_FOLLOW
    #     thisSwarmC.qr

    #     thisSwarmC.follow_line()

    #     self.assertEqual(thisSwarmC.current_state)

    def test_is_edge_in_graph_true(self):
        # Vertex and color already in graph
        graph = [{"color": "orange", "v1": 1, "v2": None}]
        edge = {"color" : "orange", "angle" : 90, "centroid" : (1, 1)}
        qrValue = 1
        self.assertTrue(self.swarm_controller.is_edge_in_graph(edge, graph, qrValue))

        graph = [{"color": "orange", "v1": None, "v2": 1}]
        edge = {"color" : "orange", "angle" : 90, "centroid" : (1, 1)}
        qrValue = 1
        self.assertTrue(self.swarm_controller.is_edge_in_graph(edge, graph, qrValue))

        # Two edges in graph
        graph = [{"color": "purple", "v1": 1, "v2": 2}, {"color": "orange", "v1": 2, "v2": None}]
        edge = {"color" : "orange", "angle" : 90, "centroid" : (1, 1)}
        qrValue = 2
        self.assertTrue(self.swarm_controller.is_edge_in_graph(edge, graph, qrValue))

    def test_is_edge_in_graph_false(self):
        # Empty graph
        graph = []
        edge = {"color" : "orange", "angle" : 90, "centroid" : (1, 1)}
        qrValue = 1
        self.assertFalse(self.swarm_controller.is_edge_in_graph(edge, graph, qrValue))

        # Graph has different color
        graph = [{"color": "purple", "v1": 1, "v2": None}]
        edge = {"color" : "orange", "angle" : 90, "centroid" : (1, 1)}
        qrValue = 1
        self.assertFalse(self.swarm_controller.is_edge_in_graph(edge, graph, qrValue))

        graph = [{"color": "purple", "v1": None, "v2": 1}]
        edge = {"color" : "orange", "angle" : 90, "centroid" : (1, 1)}
        qrValue = 1
        self.assertFalse(self.swarm_controller.is_edge_in_graph(edge, graph, qrValue))

        # Graph has different vertex id
        graph = [{"color": "orange", "v1": 2, "v2": None}]
        edge = {"color" : "orange", "angle" : 90, "centroid" : (1, 1)}
        qrValue = 1
        self.assertFalse(self.swarm_controller.is_edge_in_graph(edge, graph, qrValue))

        graph = [{"color": "orange", "v1": None, "v2": 2}]
        edge = {"color" : "orange", "angle" : 90, "centroid" : (1, 1)}
        qrValue = 1
        self.assertFalse(self.swarm_controller.is_edge_in_graph(edge, graph, qrValue))
        
        
if __name__ == '__main__':
    # rospy.init_node('Test_Node')

    unittest.main()
