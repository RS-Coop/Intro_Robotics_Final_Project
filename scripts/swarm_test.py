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

<<<<<<< HEAD
    # def test_qr_center

    # def test_line_follow(self):
    #     # Instantiate swarm controller
    #     thisSwarmC = new SwarmController()
    #     # Set curr_state
    #     thisSwarmC.current_state = LINE_FOLLOW
    #     thisSwarmC.qr
=======
    def test_determine_next_line_1(self):
        # Instantiate swarm controller
        thisSwarmC = sc.SwarmController()
        # Set curr_state
        thisSwarmC.current_state = sc.SwarmController.DETERMINE_NEXT_LINE
        thisSwarmC.qr_data = {"hasQR" : True, "centroid" : (0, 0), "value" : 1}
        thisSwarmC.graph_edges = []
        thisSwarmC.edge_data = [{"color" : "orange", "angle" : 90, "centroid" : (0, 0)}]
        thisSwarmC.current_edge = None
        # Determine next line
        thisSwarmC.determine_next_line()
        # Check resulting state and values
        self.assertEqual(thisSwarmC.graph_edges, [{"color": "orange", "v1": 1, "v2": None}])
        self.assertEqual(thisSwarmC.current_edge, {"color": "orange", "v1": 1, "v2": None})
        self.assertEqual(thisSwarmC.current_state, sc.SwarmController.MOVE_ONTO_LINE)
>>>>>>> 7cb16a83fea106fc21b06231b9aedd446c58f50b

    def test_determine_no_line_1(self):
        # Instantiate swarm controller
        thisSwarmC = sc.SwarmController()
        # Set curr_state
        thisSwarmC.current_state = sc.SwarmController.DETERMINE_NEXT_LINE
        thisSwarmC.qr_data = {"hasQR" : True, "centroid" : (0, 0), "value" : 2}
        thisSwarmC.graph_edges = [{"color": "orange", "v1": 1, "v2": None}]
        thisSwarmC.edge_data = [{"color" : "orange", "angle" : 90, "centroid" : (0, 0)}]
        thisSwarmC.current_edge = {"color": "orange", "v1": 1, "v2": None}
        # Determine next line
        thisSwarmC.determine_next_line()
        # Check resulting state and values
        self.assertEqual(thisSwarmC.graph_edges, [{"color": "orange", "v1": 1, "v2": 2}])
        self.assertEqual(thisSwarmC.current_edge, None)
        self.assertEqual(thisSwarmC.current_state, sc.SwarmController.LAND)

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
<<<<<<< HEAD
        self.assertEqual(self.swarm_controller.is_edge_in_graph(edge, graph, qrValue), None)


=======
        self.assertEqual(self.swarm_controller.get_edge_in_graph(edge, graph, qrValue), None)
        
        
>>>>>>> 7cb16a83fea106fc21b06231b9aedd446c58f50b
if __name__ == '__main__':
    # rospy.init_node('Test_Node')

    unittest.main()
