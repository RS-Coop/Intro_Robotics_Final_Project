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
        thisSwarmC.edge_data = [{"color" : G.ORANGE, "angle" : 90, "centroid" : (0, 0)}]
        thisSwarmC.graph_edges = []
        thisSwarmC.current_edge_color = None
        thisSwarmC.current_state = G.DETERMINE_NEXT_LINE
        # Determine next line
        thisSwarmC.determine_next_line()
        # Check resulting state and values
        self.assertEqual(thisSwarmC.graph_edges, [{"color": G.ORANGE, "v1": 1, "v2": None}])
        self.assertEqual(thisSwarmC.current_edge_color, G.ORANGE)
        self.assertEqual(thisSwarmC.current_state, G.MOVE_ONTO_LINE)

    def test_determine_no_line_1(self):
        # Instantiate swarm controller
        thisSwarmC = sc.SwarmController()
        # Set curr_state
        thisSwarmC.qr_data = {"hasQR" : True, "centroid" : (0, 0), "value" : 2}
        thisSwarmC.edge_data = [{"color" : G.ORANGE, "angle" : 90, "centroid" : (0, 0)}]
        thisSwarmC.graph_edges = [{"color": G.ORANGE, "v1": 1, "v2": 2}]
        thisSwarmC.current_edge_color = None
        thisSwarmC.current_state = G.DETERMINE_NEXT_LINE
        # Determine next line
        thisSwarmC.determine_next_line()
        # Check resulting state and values
        self.assertEqual(thisSwarmC.graph_edges, [{"color": G.ORANGE, "v1": 1, "v2": 2}])
        self.assertEqual(thisSwarmC.current_edge_color, None)
        self.assertEqual(thisSwarmC.current_state, G.LAND)

    def test_determine_no_line_2(self):
        # Instantiate swarm controller
        thisSwarmC = sc.SwarmController()
        # Set curr_state
        thisSwarmC.qr_data = {"hasQR" : True, "centroid" : (0, 0), "value" : 2}
        thisSwarmC.edge_data = [{"color" : G.ORANGE, "angle" : 90, "centroid" : (0, 0)}]
        thisSwarmC.graph_edges = [{"color": G.ORANGE, "v1": 1, "v2": 2}]
        thisSwarmC.current_edge_color = None
        thisSwarmC.current_state = G.DETERMINE_NEXT_LINE
        # Determine next line
        thisSwarmC.determine_next_line()
        # Check resulting state and values
        self.assertEqual(thisSwarmC.graph_edges, [{"color": G.ORANGE, "v1": 1, "v2": 2}])
        self.assertEqual(thisSwarmC.current_edge_color, None)
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
        thisSwarmC.current_edge_color = None
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
        thisSwarmC.current_edge_color = None
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
        thisSwarmC.current_edge_color = None
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
        thisSwarmC.current_edge_color = None
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
        thisSwarmC.current_edge_color = None
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
        thisSwarmC.edge_data = [{"color" : G.ORANGE, "angle" : 90, "centroid" : (0, 0)}]
        thisSwarmC.graph_edges = [{"color": G.ORANGE, "v1": 1, "v2": None}]
        thisSwarmC.current_edge_color = None
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
        thisSwarmC.current_edge_color = None
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
        thisSwarmC.edge_data = [{"color" : G.ORANGE, "angle" : 90, "centroid" : (0, 0)}]
        thisSwarmC.graph_edges = [{"color": G.ORANGE, "v1": 1, "v2": None}]
        thisSwarmC.current_edge_color = None
        thisSwarmC.current_state = G.CENTER_QR
        # Determine next line
        thisSwarmC.center_qr()
        # Check resulting state and values
        self.assertEqual(thisSwarmC.current_state, G.DETERMINE_NEXT_LINE)

    ###
    ### tests for follow_line
    ###
    def test_follow_line_no_qr_detected_1(self):
        # Instantiate swarm controller
        thisSwarmC = sc.SwarmController()
        # Set curr_state
        thisSwarmC.qr_data = {"hasQR" : False, "centroid" : None, "value" : None}
        thisSwarmC.edge_data = [{"color" : G.ORANGE, "angle" : 90, "centroid" : (0, 0)}]
        thisSwarmC.graph_edges = [{"color": G.ORANGE, "v1": 1, "v2": None}]
        thisSwarmC.current_edge_color = {"color": G.ORANGE, "v1": 1, "v2": None}
        thisSwarmC.current_state = G.FOLLOW_LINE
        # Determine next line
        thisSwarmC.follow_line()
        # Check resulting state and values
        self.assertEqual(thisSwarmC.current_state, G.FOLLOW_LINE)

    def test_follow_line_qr_detected_1(self):
        # Instantiate swarm controller
        thisSwarmC = sc.SwarmController()
        # Set curr_state
        thisSwarmC.qr_data = {"hasQR" : True, "centroid" : (0, 0), "value" : 2}
        thisSwarmC.edge_data = [{"color" : G.ORANGE, "angle" : 90, "centroid" : (0, 0)}]
        thisSwarmC.graph_edges = [{"color": G.ORANGE, "v1": 1, "v2": None}]
        thisSwarmC.current_edge_color = G.ORANGE
        thisSwarmC.current_state = G.FOLLOW_LINE
        # Determine next line
        thisSwarmC.follow_line()
        # Check resulting state and values
        self.assertEqual(thisSwarmC.current_state, G.CENTER_QR)

    # ###
    # ### tests for move_onto_line
    # ###
    # def test_follow_line_no_qr_detected_1(self):
    #     # Instantiate swarm controller
    #     thisSwarmC = sc.SwarmController()
    #     # Set curr_state
    #     thisSwarmC.qr_data = {"hasQR" : False, "centroid" : None, "value" : None}
    #     thisSwarmC.edge_data = [{"color" : G.ORANGE, "angle" : 90, "centroid" : (0, 0)}]
    #     thisSwarmC.graph_edges = [{"color": G.ORANGE, "v1": 1, "v2": None}]
    #     thisSwarmC.current_edge_color = G.ORANGE
    #     thisSwarmC.current_state = G.FOLLOW_LINE
    #     # Determine next line
    #     thisSwarmC.move_onto_line()
    #     # Check resulting state and values
    #     self.assertEqual(thisSwarmC.current_state, G.FOLLOW_LINE)

    # def test_follow_line_qr_detected_1(self):
    #     # Instantiate swarm controller
    #     thisSwarmC = sc.SwarmController()
    #     # Set curr_state
    #     thisSwarmC.qr_data = {"hasQR" : True, "centroid" : (0, 0), "value" : 1}
    #     thisSwarmC.edge_data = [{"color" : G.ORANGE, "angle" : 90, "centroid" : (0, 0)}]
    #     thisSwarmC.graph_edges = [{"color": G.ORANGE, "v1": 1, "v2": None}]
    #     thisSwarmC.current_edge_color = G.ORANGE
    #     thisSwarmC.current_state = G.MOVE_ONTO_LINE
    #     # Determine next line
    #     thisSwarmC.move_onto_line()
    #     # Check resulting state and values
    #     self.assertEqual(thisSwarmC.current_state, G.MOVE_ONTO_LINE)

    ###
    ### tests for get_edge_in_graph helper function
    ###
    def test_is_edge_in_graph_true(self):
        # Vertex and color already in graph
        graph = [{"color": G.ORANGE, "v1": 1, "v2": None}]
        edge_color = G.ORANGE
        qrValue = 1
        self.assertNotEqual(self.swarm_controller.get_edge_in_graph(edge_color, graph, qrValue), None)

        graph = [{"color": G.ORANGE, "v1": None, "v2": 1}]
        edge = G.ORANGE
        qrValue = 1
        self.assertNotEqual(self.swarm_controller.get_edge_in_graph(edge, graph, qrValue), None)

        # Two edges in graph
        graph = [{"color": G.PURPLE, "v1": 1, "v2": 2}, {"color": G.ORANGE, "v1": 2, "v2": None}]
        edge = G.ORANGE
        qrValue = 2
        self.assertNotEqual(self.swarm_controller.get_edge_in_graph(edge, graph, qrValue), None)

    def test_is_edge_in_graph_false(self):
        # Empty graph
        graph = []
        edge = {"color" : G.ORANGE, "angle" : 90, "centroid" : (1, 1)}
        qrValue = 1
        self.assertEqual(self.swarm_controller.get_edge_in_graph(edge, graph, qrValue), None)

        # Graph has different color
        graph = [{"color": G.PURPLE, "v1": 1, "v2": None}]
        edge = {"color" : G.ORANGE, "angle" : 90, "centroid" : (1, 1)}
        qrValue = 1
        self.assertEqual(self.swarm_controller.get_edge_in_graph(edge, graph, qrValue), None)

        graph = [{"color": G.PURPLE, "v1": None, "v2": 1}]
        edge = {"color" : G.ORANGE, "angle" : 90, "centroid" : (1, 1)}
        qrValue = 1
        self.assertEqual(self.swarm_controller.get_edge_in_graph(edge, graph, qrValue), None)

        # Graph has different vertex id
        graph = [{"color": G.ORANGE, "v1": 2, "v2": None}]
        edge = {"color" : G.ORANGE, "angle" : 90, "centroid" : (1, 1)}
        qrValue = 1
        self.assertEqual(self.swarm_controller.get_edge_in_graph(edge, graph, qrValue), None)

        graph = [{"color": G.ORANGE, "v1": None, "v2": 2}]
        edge = {"color" : G.ORANGE, "angle" : 90, "centroid" : (1, 1)}
        qrValue = 1
        self.assertEqual(self.swarm_controller.get_edge_in_graph(edge, graph, qrValue), None)

    ###
    ### tests for update_v2 helper function
    ###
    def test_update_v2_1(self):
        # Instantiate swarm controller
        thisSwarmC = sc.SwarmController()
        # Set curr_state
        thisSwarmC.qr_data = {"hasQR" : True, "centroid" : (0, 0), "value" : 2}
        thisSwarmC.edge_data = [{"color" : G.ORANGE, "angle" : 90, "centroid" : (0, 0)}]
        thisSwarmC.graph_edges = [{"color": G.ORANGE, "v1": 1, "v2": None}]
        thisSwarmC.current_edge_color = G.ORANGE
        thisSwarmC.current_state = G.MOVE_ONTO_LINE
        # Determine next line
        thisSwarmC.update_v2(thisSwarmC.current_edge_color, thisSwarmC.graph_edges, thisSwarmC.get_edge_pose(thisSwarmC.current_edge_color)["v1"], thisSwarmC.qr_data["value"])
        # Check resulting state and values
        
        self.assertEqual(thisSwarmC.graph_edges, [{"color": G.ORANGE, "v1": 1, "v2": 2}])

    def test_update_v2_2(self):
        # Instantiate swarm controller
        thisSwarmC = sc.SwarmController()
        # Set curr_state
        thisSwarmC.qr_data = {"hasQR" : True, "centroid" : (0, 0), "value" : 2}
        thisSwarmC.edge_data = [{"color" : G.ORANGE, "angle" : 90, "centroid" : (0, 0)}]
        thisSwarmC.graph_edges = [{"color": G.ORANGE, "v1": 1, "v2": None}, {"color": G.PURPLE, "v1": 1, "v2": None}]
        thisSwarmC.current_edge_color = G.ORANGE
        thisSwarmC.current_state = G.MOVE_ONTO_LINE
        # Determine next line
        thisSwarmC.update_v2(thisSwarmC.current_edge_color, thisSwarmC.graph_edges, thisSwarmC.get_edge_pose(thisSwarmC.current_edge_color)["v1"], thisSwarmC.qr_data["value"])
        # Check resulting state and values
        
        self.assertEqual(thisSwarmC.graph_edges, [{"color": G.ORANGE, "v1": 1, "v2": 2}, {"color": G.PURPLE, "v1": 1, "v2": None}])

    ###
    ### tests for get_edge_in_graph helper function
    ###
    def get_line_pose(self):
        thisSwarmC = sc.SwarmController()
        # Set curr_state
        thisSwarmC.qr_data = {"hasQR" : True, "centroid" : (0, 0), "value" : 2}
        thisSwarmC.edge_data = [{"color" : G.ORANGE, "angle" : 90, "centroid" : (0, 0)}]
        thisSwarmC.graph_edges = [{"color": G.ORANGE, "v1": 1, "v2": None}, {"color": G.PURPLE, "v1": 1, "v2": None}]
        thisSwarmC.current_edge_color = G.ORANGE
        thisSwarmC.current_state = G.MOVE_ONTO_LINE

if __name__ == '__main__':
    rospy.init_node('Test_Node')

    unittest.main()
