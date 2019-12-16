#!/usr/bin/env python
import rospy
import unittest
import os
# import Drone_Control as dc
# from Image_Processing import cv_processor as cvp
from Globals import Globals as G
from Drone_Control import swarm_control as sc

import cv2 as cv

class TestSwarm(unittest.TestCase):
    parent_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

    def setUp(self):
        self.swarm_controller = sc.SwarmController()

    ###
    ### tests for determine_next_line
    ###
    def test_determine_next_line_1(self):
        # Instantiate swarm controller
        thisSwarmC = sc.SwarmController()
        # Set curr_state
        thisSwarmC.qr_data = {"hasQR" : True, "centroid" : (0, 0), "value" : 1}
        thisSwarmC.edge_data = [{"color" : "orange", "angle" : 90, "centroid" : (0, 0)}]
        thisSwarmC.graph_edges = []
        thisSwarmC.current_edge = None
        thisSwarmC.current_state = G.DETERMINE_NEXT_LINE
        # Determine next line
        thisSwarmC.determine_next_line()
        # Check resulting state and values
        self.assertEqual(thisSwarmC.graph_edges, [{"color": "orange", "v1": 1, "v2": None}])
        self.assertEqual(thisSwarmC.current_edge, {"color": "orange", "v1": 1, "v2": None})
        self.assertEqual(thisSwarmC.current_state, G.MOVE_ONTO_LINE)

    def test_determine_no_line_1(self):
        # Instantiate swarm controller
        thisSwarmC = sc.SwarmController()
        # Set curr_state
        thisSwarmC.qr_data = {"hasQR" : True, "centroid" : (0, 0), "value" : 2}
        thisSwarmC.edge_data = [{"color" : "orange", "angle" : 90, "centroid" : (0, 0)}]
        thisSwarmC.graph_edges = [{"color": "orange", "v1": 1, "v2": None}]
        thisSwarmC.current_edge = {"color": "orange", "v1": 1, "v2": None}
        thisSwarmC.current_state = G.DETERMINE_NEXT_LINE
        # Determine next line
        thisSwarmC.determine_next_line()
        # Check resulting state and values
        self.assertEqual(thisSwarmC.graph_edges, [{"color": "orange", "v1": 1, "v2": 2}])
        self.assertEqual(thisSwarmC.current_edge, None)
        self.assertEqual(thisSwarmC.current_state, G.LAND)

    ###
    ### tests for center_qr
    ###
    def test_center_qr_uncentered_1(self):
        # Instantiate swarm controller
        thisSwarmC = sc.SwarmController()
        # Set curr_state
        thisSwarmC.qr_data = {"hasQR" : True, "centroid" : (280, 184), "value" : 1}
        thisSwarmC.edge_data = []
        thisSwarmC.graph_edges = []
        thisSwarmC.current_edge = None
        thisSwarmC.current_state = G.CENTER_QR
        # Determine next line
        thisSwarmC.center_qr()
        # Check resulting state and values
        self.assertEqual(thisSwarmC.current_state, G.CENTER_QR)

        # Instantiate swarm controller
        thisSwarmC = sc.SwarmController()
        # Set curr_state
        thisSwarmC.qr_data = {"hasQR" : True, "centroid" : (360, 184), "value" : 1}
        thisSwarmC.edge_data = []
        thisSwarmC.graph_edges = []
        thisSwarmC.current_edge = None
        thisSwarmC.current_state = G.CENTER_QR
        # Determine next line
        thisSwarmC.center_qr()
        # Check resulting state and values
        self.assertEqual(thisSwarmC.current_state, G.CENTER_QR)

        # Instantiate swarm controller
        thisSwarmC = sc.SwarmController()
        # Set curr_state
        thisSwarmC.qr_data = {"hasQR" : True, "centroid" : (320, 144), "value" : 1}
        thisSwarmC.edge_data = []
        thisSwarmC.graph_edges = []
        thisSwarmC.current_edge = None
        thisSwarmC.current_state = G.CENTER_QR
        # Determine next line
        thisSwarmC.center_qr()
        # Check resulting state and values
        self.assertEqual(thisSwarmC.current_state, G.CENTER_QR)

        # Instantiate swarm controller
        thisSwarmC = sc.SwarmController()
        # Set curr_state
        thisSwarmC.qr_data = {"hasQR" : True, "centroid" : (320, 224), "value" : 1}
        thisSwarmC.edge_data = []
        thisSwarmC.graph_edges = []
        thisSwarmC.current_edge = None
        thisSwarmC.current_state = G.CENTER_QR
        # Determine next line
        thisSwarmC.center_qr()
        # Check resulting state and values
        self.assertEqual(thisSwarmC.current_state, G.CENTER_QR)

        # Instantiate swarm controller
        thisSwarmC = sc.SwarmController()
        # Set curr_state
        thisSwarmC.qr_data = {"hasQR" : True, "centroid" : (100, 224), "value" : 1}
        thisSwarmC.edge_data = []
        thisSwarmC.graph_edges = []
        thisSwarmC.current_edge = None
        thisSwarmC.current_state = G.CENTER_QR
        # Determine next line
        thisSwarmC.center_qr()
        # Check resulting state and values
        self.assertEqual(thisSwarmC.current_state, G.CENTER_QR)

    def test_center_qr_uncentered_2(self):
        # Instantiate swarm controller
        thisSwarmC = sc.SwarmController()
        # Set curr_state
        thisSwarmC.qr_data = {"hasQR" : True, "centroid" : (100, 224), "value" : 1}
        thisSwarmC.edge_data = [{"color" : "orange", "angle" : 90, "centroid" : (0, 0)}]
        thisSwarmC.graph_edges = [{"color": "orange", "v1": 1, "v2": None}]
        thisSwarmC.current_edge = None
        thisSwarmC.current_state = G.CENTER_QR
        # Determine next line
        thisSwarmC.center_qr()
        # Check resulting state and values
        self.assertEqual(thisSwarmC.current_state, G.CENTER_QR)

    def test_center_qr_centered_1(self):
        # Instantiate swarm controller
        thisSwarmC = sc.SwarmController()
        # Set curr_state
        thisSwarmC.qr_data = {"hasQR" : True, "centroid" : (320, 184), "value" : 1}
        thisSwarmC.edge_data = []
        thisSwarmC.graph_edges = []
        thisSwarmC.current_edge = None
        thisSwarmC.current_state = G.CENTER_QR
        # Determine next line
        thisSwarmC.center_qr()
        # Check resulting state and values
        self.assertEqual(thisSwarmC.current_state, G.DETERMINE_NEXT_LINE)

    def test_center_qr_centered_2(self):
        # Instantiate swarm controller
        thisSwarmC = sc.SwarmController()
        # Set curr_state
        thisSwarmC.qr_data = {"hasQR" : True, "centroid" : (320, 184), "value" : 1}
        thisSwarmC.edge_data = [{"color" : "orange", "angle" : 90, "centroid" : (0, 0)}]
        thisSwarmC.graph_edges = [{"color": "orange", "v1": 1, "v2": None}]
        thisSwarmC.current_edge = None
        thisSwarmC.current_state = G.CENTER_QR
        # Determine next line
        thisSwarmC.center_qr()
        # Check resulting state and values
        self.assertEqual(thisSwarmC.current_state, G.DETERMINE_NEXT_LINE)



    ###
    ### tests for get_edge_in_graph helper function
    ###
    def test_is_edge_in_graph_true(self):
        # Vertex and color already in graph
        graph = [{"color": "orange", "v1": 1, "v2": None}]
        edge = {"color" : "orange", "angle" : 90, "centroid" : (1, 1)}
        qrValue = 1
        self.assertNotEqual(self.swarm_controller.get_edge_in_graph(edge, graph, qrValue), None)

        graph = [{"color": "orange", "v1": None, "v2": 1}]
        edge = {"color" : "orange", "angle" : 90, "centroid" : (1, 1)}
        qrValue = 1
        self.assertNotEqual(self.swarm_controller.get_edge_in_graph(edge, graph, qrValue), None)

        # Two edges in graph
        graph = [{"color": "purple", "v1": 1, "v2": 2}, {"color": "orange", "v1": 2, "v2": None}]
        edge = {"color" : "orange", "angle" : 90, "centroid" : (1, 1)}
        qrValue = 2
        self.assertNotEqual(self.swarm_controller.get_edge_in_graph(edge, graph, qrValue), None)

    def test_is_edge_in_graph_false(self):
        # Empty graph
        graph = []
        edge = {"color" : "orange", "angle" : 90, "centroid" : (1, 1)}
        qrValue = 1
        self.assertEqual(self.swarm_controller.get_edge_in_graph(edge, graph, qrValue), None)

        # Graph has different color
        graph = [{"color": "purple", "v1": 1, "v2": None}]
        edge = {"color" : "orange", "angle" : 90, "centroid" : (1, 1)}
        qrValue = 1
        self.assertEqual(self.swarm_controller.get_edge_in_graph(edge, graph, qrValue), None)

        graph = [{"color": "purple", "v1": None, "v2": 1}]
        edge = {"color" : "orange", "angle" : 90, "centroid" : (1, 1)}
        qrValue = 1
        self.assertEqual(self.swarm_controller.get_edge_in_graph(edge, graph, qrValue), None)

        # Graph has different vertex id
        graph = [{"color": "orange", "v1": 2, "v2": None}]
        edge = {"color" : "orange", "angle" : 90, "centroid" : (1, 1)}
        qrValue = 1
        self.assertEqual(self.swarm_controller.get_edge_in_graph(edge, graph, qrValue), None)

        graph = [{"color": "orange", "v1": None, "v2": 2}]
        edge = {"color" : "orange", "angle" : 90, "centroid" : (1, 1)}
        qrValue = 1
        self.assertEqual(self.swarm_controller.get_edge_in_graph(edge, graph, qrValue), None)


if __name__ == '__main__':
    rospy.init_node('Test_Node')

    unittest.main()
