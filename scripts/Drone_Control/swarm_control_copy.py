import rospy
import numpy as np
from Globals import Globals as G
from Intro_Robotics_Final_Project.msg import QR, EdgeList, DroneCommand, DroneMovementCommand

#This class deals with controlling all drones
class SwarmController:
    # Region definitions for image
    CENTER = G.BEBOP_CENTER
    ANGLE_ERROR = G.ANGLE_BOUND
    CENTER_X_ERROR = G.QR_ERROR_X
    CENTER_Y_ERROR = G.QR_ERROR_Y
    callback_count = 0

    # Has the drone already launched?
    has_launched = False

    #Drones in swarm (NOTE: Right now just 1)
    drones = ['/bebop']
    #QR Code data from img processor
    qr_data = {"hasQR" : None, "centroid" : None, "value" : None}
    #Line data from img processor
    '''
    [
        {
            "color" : String, # (orange, purple)
            "angle" : float,
            "centroid" : (int, int) # (x, y)
        }
    ]
    '''
    edge_data = []

    #Edges in graph
    '''
    [
        {
            color: string,
            v1: int,
            v2: int
        }
    ]
    '''
    graph_edges = []

    #Current state
    current_state = G.TAKEOFF
    #Color of current edge being followed
    current_edge_color = None
    current_qr_code = None

    def __init__(self, namespace='/swarm'):
        #Publishers
        self.drone_command_pub = rospy.Publisher(namespace+'/drone_command', DroneMovementCommand, queue_size=1)
        #Subscribers
        self.qr_sub = rospy.Subscriber('/swarm/qr_code', QR, self.qr_callback)
        self.edges_sub = rospy.Subscriber('/swarm/edges', EdgeList, self.edge_callback)

        rospy.sleep(2.0)

################################################################################
#Main methods and publishing methods

    #Loop to be run inside main script
    #DONE: Loops while core is running
    #NOTE: Navigate to line not implemented
    def run_node(self):
        while not rospy.is_shutdown():
            # if self.run_state():
            #     break
            pass

    # Run based off state, if state is land, return true
    def run_state(self):
        print("State: ", self.current_state)
        print("QR: ", self.qr_data)
        print("Callbacks", self.callback_count)
        print("Current Line: ", self.current_edge_color)
        if self.current_state == G.KILL:
            self.kill() #Center qr code
            return False
        elif self.current_state == G.TAKEOFF:
            self.take_off() #Center qr code
            return False
        elif self.current_state == G.SEARCH_QR:
            self.search_qr() #Center qr code
            return False
        elif self.current_state == G.CENTER_QR:
            self.center_qr() #Center qr code
            return False
        elif self.current_state == G.DETERMINE_NEXT_LINE:
            self.determine_next_line() #Choose a new line to follow
            return False
        #NOTE: Not yet being used
        elif self.current_state == G.NAVIGATE_TO_LINE:
            self.navigate_to_line()
            return False
        elif self.current_state == G.FOLLOW_LINE:
            self.follow_line() #Follow current edge
            return False
        elif self.current_state == G.MOVE_ONTO_LINE:
            self.move_onto_line() #Move into position over new line
            return False
        elif self.current_state == G.LAND:
            self.land_drone() #Land the drones
            return True

    # Kill the drones: Land immediately
    def kill(self):
        self.land_drone()

    def take_off(self):
        if not self.has_launched:
            self.takeoff_drone()
            self.has_launched = True
        if self.qr_data["centroid"] != (0, 0):
            self.current_state = G.SEARCH_QR

    def search_qr(self):
        if self.qr_data["hasQR"] == True:
            
            current_edge = self.get_edge_by_color(self.current_edge_color)
            self.update_v2(self.current_edge_color, self.graph_edges, current_edge["v1"], self.qr_data["value"])
            self.current_edge_color = None
            
            #Change state
            self.current_qr_code = self.qr_data["value"]
            self.current_state = G.CENTER_QR
        else:
            # Try to get a good view of the QR code to eliminate glare
            centroid = self.qr_data["centroid"]

            y_err = self.CENTER[0]-centroid[1] #pos means forward
            x_err = self.CENTER[1]-centroid[0]  #pos means left

            # print("XERR:", x_err, "YERR:", y_err)
            # print("", abs(x_err), ">", self.CENTER_X_ERROR, "or", abs(y_err), ">", self.CENTER_Y_ERROR)

            if abs(x_err) > self.CENTER_X_ERROR or abs(y_err) > self.CENTER_Y_ERROR:
                # If the centroid is in the top of the image, move forward
                if y_err > 0 and abs(y_err > self.CENTER_Y_ERROR):
                    self.move_drone_forward()
                # If the centroid is in the bottom of the image, move backward
                elif y_err < 0 and abs(y_err > self.CENTER_Y_ERROR):
                    self.move_drone_backwards()
                # If the qr code is to the left, move left
                elif x_err > 0 and abs(x_err > self.CENTER_X_ERROR):
                    self.move_drone_left()
                # If the qr code is to the right, move right
                elif x_err < 0 and abs(x_err > self.CENTER_X_ERROR):
                    self.move_drone_right()

    #Centers the drone over the QR code
    #DONE: Based on the centroid move the drone
    #TODO: Test this
    #NOTE: This will be within some range of error, right now 10 pixels
    def center_qr(self):
        if self.qr_data["hasQR"] == True:
            current_qr_code = self.qr_data["value"]
            centroid = self.qr_data["centroid"]

            y_err = self.CENTER[0]-centroid[1] #pos means forward
            x_err = self.CENTER[1]-centroid[0]  #pos means left

            # print("XERR:", x_err, "YERR:", y_err)
            # print("", abs(x_err), ">", self.CENTER_X_ERROR, "or", abs(y_err), ">", self.CENTER_Y_ERROR)

            if abs(x_err) > self.CENTER_X_ERROR or abs(y_err) > self.CENTER_Y_ERROR:
                # If the centroid is in the top of the image, move forward
                if y_err > 0 and abs(y_err > self.CENTER_Y_ERROR):
                    self.move_drone_forward()
                # If the centroid is in the bottom of the image, move backward
                elif y_err < 0 and abs(y_err > self.CENTER_Y_ERROR):
                    self.move_drone_backwards()
                # If the qr code is to the left, move left
                elif x_err > 0 and abs(x_err > self.CENTER_X_ERROR):
                    self.move_drone_left()
                # If the qr code is to the right, move right
                elif x_err < 0 and abs(x_err > self.CENTER_X_ERROR):
                    self.move_drone_right()
            else:
                self.current_state = G.DETERMINE_NEXT_LINE

    #Determines next line to follow out of the vertex
    #TODO:
    def determine_next_line(self):
        # Add the currently visible lines to the graph
        self.add_edges_to_graph(self.edge_data)

        # If there is an unexplored edge out of the current vertex, switch to line following state
        for edge in self.edge_data:
            # Get the current edge in graph_edges
            existingEdge = self.get_edge_in_graph(edge["color"], self.graph_edges, self.current_qr_code)
            # If the current edge started at the current QR (has it for v1 instaed of v2) then explore it
            # print(existingEdge)
            if existingEdge != None and existingEdge["v2"] == None:
                print("Graph:", self.graph_edges)
                print("Edges:", self.edge_data)
                self.current_edge_color = existingEdge["color"]
                print("Current edge color: ", self.current_edge_color)
                self.current_state = G.MOVE_ONTO_LINE
                return
        # If there are no unexplored edges out of the current vertex, and
        self.current_edge_color = None
        self.current_state = G.LAND

    #Establishes the drone on a new line to follow
    #TODO: Move the drone onto
    #NOTE:
    def move_onto_line(self):
        if self.qr_data["centroid"] != (0, 0):
            centroid, angle = self.get_line_pose(self.current_edge_color)
            angle -= 90
            print("Centroid: ", centroid, ", Angle: ", angle)

            # If the line to follow is not detected, kill
            if (centroid == None and angle == None):
                self.current_state = G.KILL

            #If the line is not vertical
            elif self.is_line_vertical(self.current_edge_color) == False:
                print("line not vertical")
                #Rotate to get the line vertical
                if angle > 0 and abs(angle) > self.ANGLE_ERROR:
                    self.turn_drone_left()
                elif angle < 0 and abs(angle) > self.ANGLE_ERROR::
                    self.turn_drone_right()

            #If the line is not centered
            elif self.is_line_centered(self.current_edge_color) == False:
                print("line not centered")
                #Shift left or right to center line
                #We should just care about x error
                y_err = self.CENTER[0]-centroid[1] #pos means forward
                x_err = self.CENTER[1]-centroid[0]  #pos means left

                # If the centroid is in the top of the image, move forward
                if y_err > 0 and abs(y_err > self.CENTER_Y_ERROR):
                    self.move_drone_forward()
                # If the centroid is in the bottom of the image, move backward
                elif y_err < 0 and abs(y_err > self.CENTER_Y_ERROR):
                    self.move_drone_backwards()
                # If the qr code is to the left, move left
                elif x_err > 0 and abs(x_err > self.CENTER_X_ERROR):
                    self.move_drone_left()
                # If the qr code is to the right, move right
                elif x_err < 0 and abs(x_err > self.CENTER_X_ERROR):
                    self.move_drone_right()

            #If the line is vertical and centered
            else:
                print("line vertical and centered")
                #Move forward
                self.move_drone_forward()
        else:
            self.current_state = G.FOLLOW_LINE

    #Dispatches drone from a vertex to a edge
    #TODO: Select an edge leaving the vertex and go there
    def navigate_to_line(self):
        pass

    #Follows line untill it reaches vertex, adjusts as neccesary
    #TODO: What data do I have for moving, then fix movement commands
    #NOTE: Not yet fully implemented
    def follow_line(self):
        if self.qr_data["centroid"] == (0, 0) or self.qr_data["value"] == self.current_qr_code:
            centroid, angle = self.get_line_pose(self.current_edge_color)

            # If the line to follow is not detected, kill
            if (centroid == None and angle == None):
                self.current_state = G.KILL

            #If the line is not centered
            elif self.is_line_centered(self.current_edge_color) == False:
                print("line not centered")
                #Shift left or right to center line
                #We should just care about x error
                # If the centroid is in the top of the image, move forward
                if y_err > 0 and abs(y_err > self.CENTER_Y_ERROR):
                    self.move_drone_forward()
                # If the centroid is in the bottom of the image, move backward
                elif y_err < 0 and abs(y_err > self.CENTER_Y_ERROR):
                    self.move_drone_backwards()
                # If the qr code is to the left, move left
                elif x_err > 0 and abs(x_err > self.CENTER_X_ERROR):
                    self.move_drone_left()
                # If the qr code is to the right, move right
                elif x_err < 0 and abs(x_err > self.CENTER_X_ERROR):
                    self.move_drone_right()


            #If the line is not vertical
            elif self.is_line_vertical(self.current_edge_color) == False:
                print("line not vertical")
                #Rotate to get the line vertical
                # Angle greater than zero, turn left
                if angle > 0 and abs(angle) > self.ANGLE_ERROR:
                    self.turn_drone_left()
                elif angle < 0 and abs(angle) > self.ANGLE_ERROR::
                    self.turn_drone_right()

            #If the line is vertical and centered
            else:
                print("line vertical and centered")
                #Move forward
                self.move_drone_forward()
        else:
            #Change state
            self.current_state = G.SEARCH_QR

    def takeoff_drone(self):
        cmd = DroneMovementCommand()
        cmd.movement_command =  G.DO_TAKEOFF
        self.drone_command_pub.publish(cmd)
        rospy.sleep(1.0)

    def land_drone(self):
        cmd = DroneMovementCommand()
        cmd.movement_command =  G.DO_LAND
        self.drone_command_pub.publish(cmd)
        rospy.sleep(1.0)

    def move_drone_forward(self):
        cmd = DroneMovementCommand()
        cmd.movement_command =  G.GO_FORWARD
        self.drone_command_pub.publish(cmd)
        rospy.sleep(1.0)

    def move_drone_backwards(self):
        cmd = DroneMovementCommand()
        cmd.movement_command =  G.GO_BACKWARD
        self.drone_command_pub.publish(cmd)
        rospy.sleep(1.0)

    def move_drone_right(self):
        cmd = DroneMovementCommand()
        cmd.movement_command =  G.GO_RIGHT
        self.drone_command_pub.publish(cmd)
        rospy.sleep(1.0)

    def move_drone_left(self):
        cmd = DroneMovementCommand()
        cmd.movement_command =  G.GO_LEFT
        self.drone_command_pub.publish(cmd)
        rospy.sleep(1.0)

    def turn_drone_right(self):
        cmd = DroneMovementCommand()
        cmd.movement_command =  G.TURN_RIGHT
        self.drone_command_pub.publish(cmd)
        rospy.sleep(1.0)

    def turn_drone_left(self):
        cmd = DroneMovementCommand()
        cmd.movement_command =  G.TURN_LEFT
        self.drone_command_pub.publish(cmd)
        rospy.sleep(1.0)

    #Failsafe
    def failsafe(self):
        cmd = DroneMovementCommand()
        cmd.movement_command =  G.DO_EMERGENCY
        self.drone_command_pub.publish(cmd)

        print('Failed Safely')

    # Return true if the line is veritical in the image with a certain error
    def is_line_vertical(self, line_color):
        centroid, angle = self.get_line_pose(line_color)

        if abs(angle) > self.ANGLE_ERROR:
            return True

        return False

    # Return true if the line is centered in the image with a certain error
    def is_line_centered(self, line_color):
        centroid, angle = self.get_line_pose(line_color)

        x_err = np.abs(self.CENTER[1]-centroid[1])

        if x_err > self.CENTER_X_ERROR:
            return True

        return False

    # Returns centroid, angle
    def get_line_pose(self, line_color):
        currentLine = self.get_line_by_color(line_color)
        print("Line pose: ", currentLine)

        # print(currentLine)
        zone_names_outer = [G.O_TOP, G.O_BOTTOM, G.O_LEFT, G.O_RIGHT]
        zone_names_inner = [G.I_TOP, G.I_BOTTOM, G.I_LEFT, G.I_RIGHT]

        if currentLine == None:
            return None, None
        else:
            firstPoint = (None, None)
            secondPoint = (None, None)

            for i in zone_names_outer:
                # print(currentLine["pos_avgs"][i])
                # print(currentLine["pos_avgs"][i] != (None, None))

                if((currentLine["pos_avgs"][i] != (None, None)) and (firstPoint == (None,None))):
                    firstPoint = currentLine["pos_avgs"][i]
                    # print("firstPoint", i)
                elif((currentLine["pos_avgs"][i] != (None,None)) and (secondPoint == (None,None))):
                    secondPoint = currentLine["pos_avgs"][i]
                    # print("secondPoint", i)
                    break

            #A second point was not found in the outer ring, checking inner ring
            if(firstPoint != (None,None) and secondPoint == (None,None)):
                for i in zone_names_inner:
                    if(currentLine["pos_avgs"][i] != (None,None)):
                        secondPoint = currentLine["pos_avgs"][i]
                        # print("secondPoint", i)
                        break

            #If either point is not found, not enough data to calculate centroid and angle, return None
            if(firstPoint == (None,None) or secondPoint == (None,None)):
                return None, None
            else:
                centroid = ((firstPoint[0] + secondPoint[0]) / 2 , (firstPoint[1] + secondPoint[1]) / 2)
                x = secondPoint[0] - firstPoint[0]
                y = secondPoint[1] - firstPoint[1]
                theta = np.degrees(np.arctan2(x , y))
                return centroid, theta

        return None, None

    # This helper funciton will add all of the edge colors in the edges array to the graph
    def add_edges_to_graph(self, edges):
        try:
            for edge in edges:
                edgeFromGraph = self.get_edge_in_graph(edge["color"], self.graph_edges, self.current_qr_code)
                # If there is no matching edge in the graph:
                if(edgeFromGraph == None):
                    # if self.current_qr_code != 0:
                    #     new_edge = {"color": edge["color"], "v1": self.current_qr_code, "v2": None}
                    # else:
                    #     new_edge = {"color": edge["color"], "v1": self.current_qr_code, "v2": None}
                    new_edge = {"color": edge["color"], "v1": self.current_qr_code, "v2": None}
                    self.graph_edges.append(new_edge)
                    pass
        except:
            print("WARN: UNEXPECTED ISSUE IN add_edges_to_graph")

    # Returns None if not found, return the graph edge otherwise
    def get_edge_in_graph(self, edge_color, graph, qrValue):
        for g_edge in graph:
            if (g_edge["color"] == edge_color and (g_edge["v1"] == qrValue or g_edge["v2"] == qrValue)):
                return g_edge
        return None

    # Returns None if not found, return the graph edge otherwise
    def update_v2(self, edge_color, graph, v1, qrValue):
        for g_edge in graph:
            if (g_edge["color"] == edge_color and g_edge["v1"] == v1):
                g_edge["v2"] = qrValue
        return None

    #
    def get_edge_by_color(self, edge_color):
        for g_edge in self.graph_edges:
            if (g_edge["color"] == edge_color):
                return g_edge
        return None

    #
    def get_line_by_color(self, line_color):
        for i in range(0, len(self.edge_data)):
            if(self.edge_data[i]['color'] == line_color):
                currentLine = self.edge_data[i]
                return(currentLine)
        return None



################################################################################
#Callbacks

    def qr_callback(self, data):
        self.qr_data["hasQR"] = data.existing
        self.qr_data["centroid"] = data.centroid
        self.qr_data["value"] = data.value
        self.callback_count += 1

        self.run_state()

    def edge_callback(self, data):
        currentIndex = 0
        self.edge_data = []
        for i in range(0, len(data.colors), 16):

            newDict = {
                        "color" : data.colors[i],
                        "pos_avgs" : {
                                    G.O_TOP : (None,None),
                                    G.O_BOTTOM : (None,None),
                                    G.O_LEFT : (None,None),
                                    G.O_RIGHT : (None,None),
                                    G.I_TOP : (None,None),
                                    G.I_BOTTOM : (None,None),
                                    G.I_LEFT : (None,None),
                                    G.I_RIGHT : (None,None)
                                    }
                      }

            if (data.pos_avgs[currentIndex], data.pos_avgs[currentIndex+1]) != (0,0):
                newDict["pos_avgs"][G.O_TOP] = (data.pos_avgs[currentIndex], data.pos_avgs[currentIndex+1])
            if (data.pos_avgs[currentIndex+2], data.pos_avgs[currentIndex+3]) != (0,0):
                newDict["pos_avgs"][G.O_BOTTOM] = (data.pos_avgs[currentIndex+2], data.pos_avgs[currentIndex+3])
            if (data.pos_avgs[currentIndex+4], data.pos_avgs[currentIndex+5]) != (0,0):
                newDict["pos_avgs"][G.O_LEFT] = (data.pos_avgs[currentIndex+4], data.pos_avgs[currentIndex+5])
            if (data.pos_avgs[currentIndex+6], data.pos_avgs[currentIndex+7]) != (0,0):
                newDict["pos_avgs"][G.O_RIGHT] = (data.pos_avgs[currentIndex+6], data.pos_avgs[currentIndex+7])
            if (data.pos_avgs[currentIndex+8], data.pos_avgs[currentIndex+9]) != (0,0):
                newDict["pos_avgs"][G.I_TOP] = (data.pos_avgs[currentIndex+8], data.pos_avgs[currentIndex+9])
            if (data.pos_avgs[currentIndex+10], data.pos_avgs[currentIndex+11]) != (0,0):
                newDict["pos_avgs"][G.I_BOTTOM] = (data.pos_avgs[currentIndex+10], data.pos_avgs[currentIndex+11])
            if (data.pos_avgs[currentIndex+12], data.pos_avgs[currentIndex+13]) != (0,0):
                newDict["pos_avgs"][G.I_LEFT] = (data.pos_avgs[currentIndex+12], data.pos_avgs[currentIndex+13])
            if (data.pos_avgs[currentIndex+14], data.pos_avgs[currentIndex+15]) != (0,0):
                newDict["pos_avgs"][G.I_RIGHT] = (data.pos_avgs[currentIndex+14], data.pos_avgs[currentIndex+15])

            self.edge_data.append(newDict)
            currentIndex+=16
        self.run_state()

################################################################################
#Tests
