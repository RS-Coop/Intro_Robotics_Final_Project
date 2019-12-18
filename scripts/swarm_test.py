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
        thisSwarmC.current_qr_code = 1
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
        thisSwarmC.current_qr_code = 2
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
        thisSwarmC.current_qr_code = 2
        # Determine next line
        thisSwarmC.determine_next_line()
        # Check resulting state and values
        self.assertEqual(thisSwarmC.graph_edges, [{"color": G.ORANGE, "v1": 1, "v2": 2}])
        self.assertEqual(thisSwarmC.current_edge_color, None)
        self.assertEqual(thisSwarmC.current_state, G.LAND)

    def test_determine_line_no_qr_code(self):
        # Instantiate swarm controller
        thisSwarmC = sc.SwarmController()
        # Set curr_state
        thisSwarmC.qr_data = {"hasQR" : False, "centroid" : (0, 0), "value" : 0}
        thisSwarmC.edge_data = [{"color" : G.ORANGE, "angle" : 90, "centroid" : (0, 0)}, {"color" : G.BLUE, "angle" : 90, "centroid" : (0, 0)}]
        thisSwarmC.graph_edges = [{"color": G.ORANGE, "v1": 1, "v2": 2}]
        thisSwarmC.current_qr_code = 2
        thisSwarmC.current_edge_color = None
        thisSwarmC.current_state = G.DETERMINE_NEXT_LINE
        # Determine next line
        thisSwarmC.determine_next_line()
        # Check resulting state and values
        self.assertEqual(thisSwarmC.graph_edges, [{"color": G.ORANGE, "v1": 1, "v2": 2}, {"color": G.BLUE, "v1": 2, "v2": None}])
        self.assertEqual(thisSwarmC.current_edge_color, G.BLUE)
        self.assertEqual(thisSwarmC.current_state, G.MOVE_ONTO_LINE)

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
    ### tests for add_edges_to_graph helper function
    ###
    def test_add_edges_to_graph_1(self):
        # Instantiate swarm controller
        thisSwarmC = sc.SwarmController()
        # Set curr_state
        thisSwarmC.qr_data = {"hasQR" : False, "centroid" : (0, 0), "value" : 0}
        thisSwarmC.edge_data = [{"color" : G.ORANGE, "angle" : 90, "centroid" : (0, 0)}, {"color" : G.BLUE, "angle" : 90, "centroid" : (0, 0)}]
        thisSwarmC.graph_edges = [{"color": G.ORANGE, "v1": 1, "v2": 2}]
        thisSwarmC.current_qr_code = 2
        thisSwarmC.current_edge_color = None
        thisSwarmC.current_state = G.DETERMINE_NEXT_LINE
        # Determine next line
        thisSwarmC.add_edges_to_graph(thisSwarmC.edge_data)
        # Check resulting state and values
        self.assertEqual(thisSwarmC.graph_edges, [{"color": G.ORANGE, "v1": 1, "v2": 2}, {"color": G.BLUE, "v1": 2, "v2": None}])
        # self.assertEqual(thisSwarmC.current_edge_color, G.BLUE)
        # self.assertEqual(thisSwarmC.current_state, G.MOVE_ONTO_LINE)
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
    def test_get_line_pose(self):
        thisSwarmC = sc.SwarmController()
        # Set curr_state
        thisSwarmC.qr_data = {"hasQR" : False, "centroid" : (0, 0), "value" : 2}
        #thisSwarmC.edge_data = [{'color': 'orange', 'pos_avgs': {'outer right': (None, None), 'outer top': (None, None), 'outer left': (None, None), 'inner left': (None, None), 'inner right': (None, None), 'inner top': (None, None), 'outer bottom': (1594, 4029), 'inner bottom': (1607, 3575)}}, {'color': 'blue', 'pos_avgs': {'outer right': (None, None), 'outer top': (None, None), 'outer left': (1, 2116), 'inner left': (343, 2111), 'inner right': (None, None), 'inner top': (None, None), 'outer bottom': (None, None), 'inner bottom': (None, None)}}]
        #thisSwarmC.edge_data = [{'color': 'orange', 'pos_avgs': {'outer right': (None, None), 'outer top': (1336, 1), 'outer left': (None, None), 'inner left': (None, None), 'inner right': (None, None), 'inner top': (1323, 455), 'outer bottom': (None, None), 'inner bottom': (None, None)}}, {'color': 'blue', 'pos_avgs': {'outer right': (3020, 3183), 'outer top': (None, None), 'outer left': (None, None), 'inner left': (None, None), 'inner right': (2680, 3190), 'inner top': (None, None), 'outer bottom': (None, None), 'inner bottom': (None, None)}}]
        #thisSwarmC.edge_data = [{'color': 'blue', 'pos_avgs': {'outer right': (3020, 775), 'outer top': (None, None), 'outer left': (2, 3359), 'inner left': (342, 3077), 'inner right': (2680, 1075), 'inner top': (None, None), 'outer bottom': (None, None), 'inner bottom': (None, None)}}]
        thisSwarmC.edge_data = [{'color': 'blue', 'pos_avgs': {'outer right': (None, None), 'outer top': (671, 1), 'outer left': (None, None), 'inner left': (None, None), 'inner right': (None, None), 'inner top': (954, 342), 'outer bottom': (3255, 3020), 'inner bottom': (2955, 2680)}}]
        thisSwarmC.graph_edges = [{"color": G.ORANGE, "v1": 1, "v2": None}, {"color": G.PURPLE, "v1": 1, "v2": None}]
        thisSwarmC.current_edge_color = G.BLUE
        thisSwarmC.current_state = G.FOLLOW_LINE
        print("Blue Pose:", thisSwarmC.get_line_pose('blue'))
        print("Orange Pose:", thisSwarmC.get_line_pose('orange'))
        pass

if __name__ == '__main__':
    rospy.init_node('Test_Node')

    unittest.main()
